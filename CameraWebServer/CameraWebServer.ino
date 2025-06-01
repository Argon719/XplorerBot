/*
 * File: ESP32CamServer.cpp
 * Description: ESP32-CAM WebServer zum Streamen von MJPEG-Bildern und Empfangen von
 *              Befehlen über die serielle Schnittstelle. Verwendet Preferences für
 *              das Speichern von WLAN- und Qualitäts-Einstellungen, ArduinoJson für
 *              JSON-Verarbeitung und WebServer für HTTP-Handling.
 * Author: Yassine Louchi
 * Date: 2025-4-25
 */

/* === Bibliotheken und Header === */
#include <ArduinoJson.h>    /* JSON-Verarbeitung */
#include <WiFi.h>           /* WLAN-Funktionalität für ESP32 */
#include <Preferences.h>    /* Persistent Storage für Einstellungen */
#include <esp_wifi.h>       /* esp_wifi_stop(), esp_wifi_start() */
#include "esp_camera.h"     /* Kamera-Funktionalität für ESP32-CAM */
#include <WebServer.h>      /* Einfacher Webserver für HTTP-Anfragen */

#define CAMERA_MODEL_AI_THINKER /* Definition des Kameramodells (AI-Thinker) */
#include "camera_pins.h"     /* Pinbelegung für AI-Thinker Kamera */

/* === Globale Objekte und Variablen === */
WebServer server(80);       /* HTTP-Server auf Port 80 */
Preferences prefs;          /* Preferences-Objekt zum Speichern von Einstellungen */

String inputBuffer      = "";    /* Puffer für serielle Eingaben (Kommandos) */
String ssid             = "";    /* WLAN-SSID aus Preferences oder benutzerdefiniert */
String pass             = "";    /* WLAN-Passwort aus Preferences oder benutzerdefiniert */
bool wifiStarted        = false; /* Flag, ob WLAN-Verbindung initialisiert wurde */
bool debug              = false; /* Flag zur Aktivierung von Debug-Ausgaben */
String currentQuality   = "mid"; /* Aktuelle Kamerakonfigurationsqualität ("low", "mid", "high") */

/*
 * Funktion: applyCameraQuality
 * ----------------------------
 * Passt die Bildgröße und JPEG-Qualität des Kamerasensors an, basierend auf dem
 * übergebenen Qualitätslevel-String ("low", "mid", "high"). Aktualisiert außerdem
 * die globale Variable currentQuality. Gibt bei unbekanntem Level optional einen
 * Fehler im Serial-Monitor aus.
 *
 * level: Gewünschtes Qualitätslevel als String
 */
void applyCameraQuality(const String& level) {
    sensor_t* s = esp_camera_sensor_get(); /* Sensor-Handle holen */

    /* Qualität "low" setzen: VGA-Auflösung, JPEG-Qualität 20 */
    if (level == "low") {
        s->set_framesize(s, FRAMESIZE_VGA);
        s->set_quality(s, 20);
        currentQuality = "low";
    }
    /* Qualität "mid" setzen: SVGA-Auflösung, JPEG-Qualität 12 */
    else if (level == "mid") {
        s->set_framesize(s, FRAMESIZE_SVGA);
        s->set_quality(s, 12);
        currentQuality = "mid";
    }
    /* Qualität "high" setzen: HD-Auflösung, JPEG-Qualität 10 */
    else if (level == "high") {
        s->set_framesize(s, FRAMESIZE_HD);
        s->set_quality(s, 10);
        currentQuality = "high";
    }
    /* Unbekanntes Qualitätslevel */
    else {
        if (debug) {
            Serial.print("{\"error\":\"Unknown quality\"}\n");
        }
        return;
    }

    /* Debug-Ausgabe des neuen Qualitätslevels */
    if (debug) {
        Serial.print("{\"quality\":\"");
        Serial.print(currentQuality);
        Serial.print("\"}\n");
    }
}

/*
 * Funktion: initCamera
 * --------------------
 * Initialisiert die ESP32-CAM-Hardware mit den in camera_pins.h definierten Pins
 * und der gewünschten JPEG-Qualität. Konfiguriert LEDC-Kanal, XCLK, Pixel-Format
 * und Framebuffer-Einstellungen. Führt anschließend esp_camera_init() aus und
 * blockiert endlos bei Fehler.
 *
 * quality: Gewünschtes Qualitätslevel als String ("low", "mid", "high")
 */
void initCamera(const String& quality) {
    camera_config_t config; /* Struktur für Kamerakonfiguration */

    /* Grundeinstellungen: LEDC-Kanal und Timer */
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;

    /* Pins für Kameradatenleitungen (D0–D7) */
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;

    /* Pins für Takt- (XCLK), Pixel- (PCLK), VSYNC und HREF */
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;

    /* Pins für SCCB (I²C) zur Konfiguration des Sensors */
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    /* Pins für Kamerastromversorgung und Reset */
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;

    /* Taktfrequenz für Sensor (20 MHz) und JPEG-Format */
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    /* Qualität spezifische Einstellungen */
    if (quality == "low") {
        config.frame_size   = FRAMESIZE_VGA;  /* VGA-Auflösung */
        config.jpeg_quality = 20;             /* JPEG-Qualität (0-63, niedrig = besser) */
    }
    else if (quality == "high") {
        config.frame_size   = FRAMESIZE_HD;   /* HD-Auflösung */
        config.jpeg_quality = 10;             /* Höhere Bildqualität */
    }
    /* Standard (mid) */
    else {
        config.frame_size   = FRAMESIZE_SVGA; /* SVGA-Auflösung */
        config.jpeg_quality = 12;             /* Mittlere Bildqualität */
    }

    /* Framebuffer-Anzahl: 2 (für doppelte Pufferung) */
    config.fb_count    = 2;
    /* Falls PSRAM vorhanden, Bilddaten in PSRAM speichern */
    config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    /* Lese-Modus: Neueste oder wenn leer */
    config.grab_mode   = psramFound() ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;

    /* Kamera initialisieren und im Fehlerfall hängen bleiben */
    if (esp_camera_init(&config) != ESP_OK) {
        if (debug) {
            Serial.print("Camera init failed\n");
        }
        while (true) { /* Endlosschleife bei Initialisierungsfehler */ ; }
    }
}

/*
 * Funktion: setupWiFiEvents
 * -------------------------
 * Registriert WLAN-Event-Handler für die Ereignisse:
 *   - ARDUINO_EVENT_WIFI_STA_CONNECTED: Debug-Ausgabe "WiFi connected"
 *   - ARDUINO_EVENT_WIFI_STA_GOT_IP: Debug-Ausgabe der erhaltenen IP-Adresse
 *   - ARDUINO_EVENT_WIFI_STA_DISCONNECTED: Debug-Ausgabe und erneuter
 *     Verbindungsaufbau mit gespeicherten SSID/Passwort.
 */
void setupWiFiEvents() {
    /* Event: WLAN-Station verbunden */
    WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t) {
        if (debug) {
            Serial.print("WiFi connected\n");
        }
    }, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);

    /* Event: WLAN-Station hat IP erhalten */
    WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t) {
        if (debug) {
            Serial.print("IP address: ");
            Serial.print(WiFi.localIP());
            Serial.print("\n");
        }
    }, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);

    /* Event: WLAN-Station getrennt */
    WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t) {
        if (debug) {
            Serial.print("WiFi disconnected, reconnecting...\n");
        }
        /* Versuch, die Verbindung mit den gespeicherten Credentials wiederherzustellen */
        WiFi.begin(ssid.c_str(), pass.c_str());
    }, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
}

/*
 * Funktion: connectWiFi
 * ---------------------
 * Verbindet das ESP32-Modul mit einem WLAN-Netzwerk. Liest SSID und Passwort
 * aus den globalen Variablen ssid und pass. Führt einen sauberen Neustart
 * der WLAN-Hardware durch (disconnect, esp_wifi_stop/esp_wifi_start), 
 * registriert Event-Handler, schaltet in den STA-Modus und wartet bis zu
 * 10 Sekunden auf die Verbindung. Gibt über Serial-JSON-Status (connected/failed)
 * aus, wenn debug aktiviert ist.
 */
void connectWiFi() {
    /* Falls SSID oder Passwort leer, nichts tun */
    if (ssid == "" || pass == "") return;
    wifiStarted = true; /* WLAN-Start-Flag setzen */

    /* 1) Saubere Trennung und Neustart der WLAN-Hardware */
    WiFi.disconnect();      /* Trennt vorhandene Verbindungen */
    delay(200);
    esp_wifi_stop();        /* Deaktiviert den WLAN-Chip */
    delay(200);
    esp_wifi_start();       /* Reaktiviert den WLAN-Chip */
    delay(200);

    /* 2) Event-Handler für WLAN registrieren */
    setupWiFiEvents();

    /* 3) Wechsel in den Station-Modus und Verbindungsaufbau */
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    WiFi.setSleep(false);   /* Deaktiviert WLAN-Schlafmodus */

    /* Debug-Ausgabe des Verbindungsversuchs */
    if (debug) {
        Serial.print("Connecting to \"");
        Serial.print(ssid);
        Serial.print("\"...\n");
    }

    /* Warte maximal 10 Sekunden auf die Verbindung */
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        delay(500);
        if (debug) {
            Serial.print(".");
        }
    }
    if (debug) {
        Serial.print("\n");
    }

    /* 4) Status ausgeben: Erfolg oder Fehlgeschlagen */
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("{\"wifi\":\"connected\",\"ip\":\"");
        Serial.print(WiFi.localIP());
        Serial.print("\"}\n");
    } else {
        Serial.print("{\"wifi\":\"failed\"}\n");
    }
}

/*
 * Funktion: streamTask
 * --------------------
 * FreeRTOS-Task, der auf einem eigenen Core als Task ausgeführt wird. 
 * Diese Funktion sendet kontinuierlich JPEG-Bilder im MJPEG-Format über
 * den TCP-Client (WiFiClient), der vom WebServer als Parameter übergeben wird.
 *
 * param: Zeiger auf WiFiClient-Objekt, das für den MJPEG-Stream verwendet wird.
 */
void streamTask(void* param) {
    WiFiClient client = *(WiFiClient*)param; /* Kopiere Client aus Parameter */
    delete (WiFiClient*)param;               /* Löse ursprünglichen Zeiger auf */

    /* 1) Sende HTTP-Header für MJPEG-Stream */
    client.print(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n"
    );

    /* 2) Endlosschleife, bis der Client die Verbindung schließt */
    while (client.connected()) {
        /* Kamera-Frame abrufen */
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            /* Falls kein Frame verfügbar, kurze Pause und erneut versuchen */
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* Sende Frame-Header mit Längenangabe */
        client.printf(
            "--frame\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n\r\n",
            (unsigned)fb->len
        );
        /* Sende JPEG-Daten */
        client.write(fb->buf, fb->len);
        client.print("\r\n");

        /* Gebe den Frame-Buffer zurück an den Treiber */
        esp_camera_fb_return(fb);

        /* Kurze Pause zwischen den einzelnen Frames (10 ms) */
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Verbindung beenden, wenn der Client nicht mehr verbunden ist */
    client.stop();
    vTaskDelete(nullptr); /* Task löschen */
}

/*
 * Funktion: handleCommand
 * -----------------------
 * Verarbeitet eine komplette Eingabezeile (String) aus der seriellen Schnittstelle.
 * Mögliche Kommandos (als JSON oder einfache Zeichenfolge):
 *   - "h" oder "help": Hilfetext über verfügbare Kommandos ausgeben
 *   - {"reset":true}        : Neustart (ESP.restart())
 *   - {"led":"on"} / "off" : schaltet die Onboard-LED (GPIO4) an oder aus
 *   - {"wifi": {...}}      : speichert SSID/Passwort in Preferences, verbindet
 *                            und startet neu
 *   - {"wifi":"get_stream_ip"}: speichert keine Daten, wird im loop() behandelt 
 *   - {"quality":"low|mid|high"}: speichert neues Qualitätslevel in Preferences
 *                                und startet neu
 * JSON-Parsing erfolgt mit ArduinoJson. Bei Fehler im JSON wird ein Fehler-JSON
 * ausgegeben.
 *
 * line: Eingabezeile als String (kompletter JSON- oder Hilfebefehl)
 */
void handleCommand(const String& line) {
    /* 1) Wenn "h" oder "help" → Hilfetext ausgeben */
    if (line == "h" || line == "help") {
        Serial.print("\n=== ESP32-CAM Commands ===\n");
        Serial.print("  {\"reset\":true}             restart\n");
        Serial.print("  {\"led\":\"on/off\"}         toggle LED\n");
        Serial.print("  {\"wifi\":{\"ssid\":\"…\",\"pass\":\"…\"}}  save & restart\n");
        Serial.print("  {\"wifi\":\"get_stream_ip\"}    get IP+quality\n");
        Serial.print("  {\"quality\":\"low|mid|high\"} change quality (restart)\n");
        Serial.print("  h or help                    show help\n\n");
        return;
    }

    /* 2) Versuch, JSON zu parsen */
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, line);
    if (err) {
        /* Ungültiges JSON → Fehler zurückgeben */
        Serial.print("{\"error\":\"Invalid JSON\"}\n");
        return;
    }

    /* 3) Kommando "reset": true → Neustart */
    if (doc["reset"] == true) {
        Serial.print("{\"resetting\":true}\n");
        delay(200);
        ESP.restart();
        return;
    }

    /* 4) Kommando "led": "on"/"off" → schalte GPIO4 (Onboard-LED) */
    if (doc.containsKey("led")) {
        digitalWrite(4, doc["led"] == String("on") ? HIGH : LOW);
        Serial.print(doc["led"] == String("on") ? "{\"led\":true}\n" : "{\"led\":false}\n");
        return;
    }

    /* 5) Kommando "wifi" */
    if (doc.containsKey("wifi")) {
        /* 5a) Wenn "wifi" ist ein Objekt: speichere SSID/Passwort und starte neu */
        if (doc["wifi"].is<JsonObject>()) {
            ssid = doc["wifi"]["ssid"].as<String>();
            pass = doc["wifi"]["pass"].as<String>();
            if (ssid != "" && pass != "") {
                /* Preferences öffnen im RW-Modus (Namespace "cam_cfg") */
                prefs.begin("cam_cfg", false);
                prefs.putString("ssid", ssid);
                prefs.putString("pass", pass);
                prefs.end(); /* Preferences schließen */

                Serial.print("{\"wifi\":\"saved\",\"restart\":true}\n");
                delay(500);
                connectWiFi(); /* Verbinde sofort mit neuem WLAN */
                delay(500);
                ESP.restart();
            } else {
                /* Fehlende SSID oder Passwort */
                Serial.print("{\"error\":\"Missing SSID/pass\"}\n");
            }
        }
        /* 5b) Wenn "wifi" ist kein Objekt: wird im loop() speziell behandelt */
        return;
    }

    /* 6) Kommando "quality": speichere Qualitätslevel und starte neu */
    if (doc.containsKey("quality")) {
        String level = doc["quality"].as<String>();
        /* Preferences öffnen im RW-Modus (Namespace "cam") */
        prefs.begin("cam", false);
        prefs.putString("quality", level);
        prefs.end();

        Serial.print("{\"quality_saved\":\"");
        Serial.print(level);
        Serial.print("\",\"restart\":true}\n");
        delay(200);
        ESP.restart();
        return;
    }

    /* 7) Unbekanntes Kommando → Fehler zurückgeben */
    Serial.print("{\"error\":\"Unknown command\"}\n");
}

/*
 * Funktion: handleStream
 * ----------------------
 * HTTP-Request-Handler für den Pfad "/stream". Wird aufgerufen, wenn der Client
 * per Browser oder anderer MJPEG-Player auf http://<ip>/stream zugreift.
 * Legt für jeden neuen Client einen FreeRTOS-Task an, der den MJPEG-Stream
 * auf dem Hintergrundcore ausführt (Task mit streamTask).
 */
void handleStream() {
    WiFiClient client = server.client(); /* Hole aktuellen Client aus server */
    /* Erstelle neuen Task für MJPEG-Stream auf Core 0 mit Stack 8 KB */
    xTaskCreatePinnedToCore(
        streamTask,           /* Task-Funktion */
        "stream",             /* Name für Debug-Zwecke */
        8192,                 /* Stackgröße in Bytes */
        new WiFiClient(client), /* Übergabe des Clients als Parameter (Heap allozieren) */
        1,                    /* Priorität (niedrig) */
        NULL,                 /* Task-Handle (nicht benötigt) */
        0                     /* Pinning auf Core 0 */
    );
}

/*
 * Funktion: setup
 * ---------------
 * Arduino-Setup: Wird einmal beim Start aufgerufen. Führt folgende Schritte durch:
 *   1. Kurze Startverzögerung und Serial-Begin
 *   2. Setze GPIO4 (LED) als Ausgang und ausschalten
 *   3. Unterdrücke Kameralogs
 *   4. Lese gespeicherte Qualitäts-Einstellung und initialisiere Kamera
 *   5. Lese gespeicherte WLAN-Credentials und verbinde ggf.
 *   6. Falls erfolgreich verbunden, starte HTTP-Server und konfiguriere "/stream"
 */
void setup() {
    delay(2000);             /* Warte 2 Sekunden (Stabilisierung nach Reset) */
    Serial.begin(115200);    /* Serielle Schnittstelle für Debug und JSON-Antworten */
    
    /* Setze Onboard-LED (GPIO4) als Ausgang und schalte sie aus */
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);

    /* Unterdrücke lästige Kameralog-Meldungen */
    esp_log_level_set("*", ESP_LOG_NONE);

    /* --- Kamera initialisieren anhand gespeicherter Qualität --- */
    prefs.begin("cam", true);                  /* Preferences-Namespace "cam" lesen */
    String quality = prefs.getString("quality", "mid"); /* Default "mid" */
    prefs.end();
    initCamera(quality);                       /* Kamera mit dieser Qualität initialisieren */
    currentQuality = quality;                  /* Globale Variable setzen */

    /* Debug/Initialausgabe: Qualität an die MCU senden */
    Serial.print("{\"quality\":\"");
    Serial.print(quality);
    Serial.print("\"}\n");

    /* --- Lese gespeicherte WLAN-Credentials aus Preferences --- */
    prefs.begin("cam_cfg", true);              /* Preferences-Namespace "cam_cfg" lesen */
    ssid = prefs.getString("ssid", "");         /* SSID, falls vorhanden */
    pass = prefs.getString("pass", "");         /* Passwort, falls vorhanden */
    prefs.end();

    delay(1000); /* Kurze Pause, bevor WLAN-Verbindung aufgebaut wird */

    /* Falls SSID und Passwort vorhanden, versuche, WLAN zu verbinden */
    if (ssid != "" && pass != "") {
        connectWiFi();
    }

    /* Falls WLAN verbunden ist, starte HTTP-Server und setze "/stream" */
    if (WiFi.status() == WL_CONNECTED) {
        server.on("/stream", HTTP_GET, handleStream);
        server.begin(); /* HTTP-Server starten */
        if (debug) {
            Serial.print("HTTP stream server started\n");
        }
    }
}

/*
 * Funktion: loop
 * --------------
 * Arduino-Loop: Wird fortlaufend aufgerufen. Hier verarbeiten wir serielle
 * Eingaben (Kommandos) und lassen den WebServer seine Clients bedienen.
 *
 * 1. Lese serielle Bytes und baue inputBuffer auf, bis '\n' oder '\r' erkannt
 * 2. Schneide den Puffer und rufe handleCommand() auf, wenn eine komplette Zeile
 *    empfangen wurde
 * 3. Wenn inputBuffer exakt "{\"wifi\":\"get_stream_ip\"}", beantworte direkt
 *    mit JSON {"stream_ip":"http://<ip>/stream","quality":"<currentQuality>"}
 *    und überspringe server.handleClient() für diese Iteration
 * 4. Rufe server.handleClient(), damit der HTTP-Server neue Anfragen bedient
 */
void loop() {
    /* --- Serielle Befehlserkennung --- */
    while (Serial.available()) {
        char c = Serial.read();
        /* Ende der Zeile (LF oder CR) */
        if (c == '\n' || c == '\r') {
            inputBuffer.trim(); /* Führende/trailing Leerzeichen entfernen */
            if (inputBuffer.length()) {
                handleCommand(inputBuffer); /* Vollständige Zeile verarbeiten */
            }
            inputBuffer = ""; /* Puffer zurücksetzen */
        }
        /* Falls druckbares Zeichen, zum Puffer hinzufügen */
        else if (isPrintable(c)) {
            inputBuffer += c;
        }

        /* --- Direkte Antwort auf {"wifi":"get_stream_ip"} --- */
        if (inputBuffer == "{\"wifi\":\"get_stream_ip\"}") {
            String ip = WiFi.localIP().toString();
            StaticJsonDocument<128> doc;
            /* Stream-URL (MJPEG-Stream) und aktuelle Qualität einfügen */
            doc["stream_ip"] = "http://" + ip + "/stream";
            doc["quality"]   = currentQuality;

            char outBuf[128];
            serializeJson(doc, outBuf);
            Serial.println(outBuf); /* JSON-Antwort an UART senden */

            inputBuffer = "";  /* Puffer leeren */
            return;            /* Überspringe server.handleClient() in dieser Iteration */
        }
    }

    /* --- HTTP-Server-Clients bedienen --- */
    server.handleClient();
}
