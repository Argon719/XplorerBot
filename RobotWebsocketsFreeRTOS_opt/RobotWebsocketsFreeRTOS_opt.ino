/* 
 * File: XplorerBot.cpp
 * Description: Hauptprogramm für XplorerBot, das diverse Sensoren, Aktuatoren und
 *              Kommunikationsschnittstellen initialisiert und steuert. 
 *              Verwendet FreeRTOS für Task-Management und AsyncWebServer für WebSocket- /
 *              HTTP-Kommunikation, sowie SPIFFS/LittleFS für Dateisystem-Integration.
 * Author: Yassine Louchi
 * Date: 2025-04-25
 */

/* === System- und Bibliotheks-Header === */
#include <Arduino.h>             /* Grundlegende Arduino-Funktionen und -Typen */
#include <Wire.h>                /* I2C-Kommunikation über Wire-Library */
#include <ArduinoJson.h>         /* JSON-Verarbeitung */
#include <freertos/FreeRTOS.h>   /* FreeRTOS-Grundlagen */
#include <freertos/task.h>       /* FreeRTOS-Task-API */
#include <freertos/semphr.h>     /* FreeRTOS-Semaphore-API */
#include <string.h>              /* Standard C string-Funktionen */

#include <WiFi.h>                /* WiFi-Funktionalität für ESP32 */
#include <ESPmDNS.h>             /* mDNS (Bonjour) für Hostnamelookup */
#include "esp_wifi.h"            /* ESP-IDF WiFi-API */
#include <LittleFS.h>            /* LittleFS-Dateisystem (SPIFFS-Ersatz) */
#include <AsyncTCP.h>            /* Async TCP-Library für ESPAsyncWebServer */
#include <ESPAsyncWebServer.h>   /* Asynchroner Webserver */
#include <ElegantOTA.h>          /* OTA-Update-Funktionalität */

#include <XplorerTCA9548A.h>     /* Multiplexer für I2C (TCA9548A) */
#include <XplorerIBUS.h>         /* IBUS (RC-Empfänger) Bibliothek */
#include <Adafruit_PWMServoDriver.h> /* PWM-Servotreiber (PCA9685) */
#include <Xplorer74HC4052.h>     /* UART-Multiplexer (74HC4052) */
#include <Xplorer_WT901.h>       /* WT901 IMU-Sensor-Bibliothek */
#include <XplorerTOFvl53l1x.h>   /* Time-of-Flight Sensor (VL53L1X) Bibliothek */
#include <XplorerLED.h>          /* LED-Steuerung (NeoPixel) */
#include <XplorerAutoLights.h>   /* Automatische Beleuchtungssteuerung */

/* === PIN- und ADRESSKONFIGURATION === */
#define CH_RX            0     /* Kanalnummer für RX-Stick am RC-Empfänger */
#define CH_RY            1     /* Kanalnummer für RY-Stick am RC-Empfänger */
#define CH_LX            3     /* Kanalnummer für LX-Stick am RC-Empfänger */
#define CH_LY            2     /* Kanalnummer für LY-Stick am RC-Empfänger */
#define CH_SWC           6     /* Kanalnummer für Three-Position-Switch SWC */
#define CH_SWD           7     /* Kanalnummer für Two-Position-Switch SWD */
#define CH_KNOB          8     /* Kanalnummer für Drehknopf */

#define TCA_ADDR         0x70  /* I2C-Adresse des TCA9548A-Multiplexers */
#define PCA9685_ADDR     0x40  /* I2C-Adresse des PCA9685-Servotreibers */
#define I2C_ADDR         0x34  /* Basisadresse für Motor- und Encoder-I2C-Geräte */
#define TCA_MOTORS       3     /* Multiplexer-Kanal für Motorsteuerung */
#define TCA_SERVOS       7     /* Multiplexer-Kanal für Servosteuerung */

#define MOTOR_SPEED      0x33  /* Registeradresse für Motor-Geschwindigkeit */
#define MOTOR_ENCODER    0x3C  /* Registeradresse für Motor-Encoder-Werte */
#define ADC_BAT_ADDR     0x00  /* Adressregister für Batterie-ADC (Spannungsmessung) */

#define S0_MIN           160   /* Minimale PWM für Servo 0 */
#define S0_MAX           630   /* Maximale PWM für Servo 0 */
#define S1_MIN           254   /* Minimale PWM für Servo 1 */
#define S1_MAX           577   /* Maximale PWM für Servo 1 */

#define LED_PIN          10    /* Pin für NeoPixel-LEDs */
#define PIN_CAM_ENABLE   7     /* Pin zum An- bzw. Abschalten der Kamera */
#define CAM_CH           2     /* UART-Multiplexer-Kanal für Kamera-Kommunikation */

#define PIN             5      /* NeoPixel-Datenpin */
#define NUMPIXELS       12     /* Anzahl der NeoPixel-LEDs */

#define RX_PIN          20     /* RX-Pin für die WT901-IMU */
#define TX_PIN          21     /* TX-Pin für die WT901-IMU */
#define MUX_S0          0      /* Multiplexer-Select-Bit S0 für UART */
#define MUX_S1          1      /* Multiplexer-Select-Bit S1 für UART */
#define IMU_CH          0      /* UART-Multiplexer-Kanal für IMU */

#define US_TRIG_PIN     3      /* Trigger-Pin für Ultraschallsensor */
#define US_ECHO_PIN     4      /* Echo-Pin für Ultraschallsensor */

#define TOF_I2C_CH      0      /* I2C-Multiplexer-Kanal für Time-of-Flight-Sensor */

static const TickType_t IBUS_MS    = pdMS_TO_TICKS(10);   /* Intervall für IBUS-Task in ms */
static const TickType_t DRIVE_MS   = pdMS_TO_TICKS(10);   /* Intervall für Drive-Task in ms */
static const TickType_t REP_MS     = pdMS_TO_TICKS(200);  /* Intervall für Report-Task in ms */
static const TickType_t SENSOR_MS  = pdMS_TO_TICKS(100);  /* Intervall für Sensor-Task in ms */
static const TickType_t IMU_MS     = pdMS_TO_TICKS(100);  /* Intervall für IMU-Task in ms */

constexpr size_t FLOAT_COUNT = 24; /* Anzahl der Float-Werte, die in Binär über WebSocket gesendet werden */

/* === GLOBALE VARIABLEN & Puffer === */

/* Ringpuffer für Time-of-Flight (TOF) Messungen (5 Proben) */
static int16_t tofBuf[5] = { 0, 0, 0, 0, 0 };
static uint8_t tofPos = 0; /* Aktuelle Position im TOF-Ringpuffer */

/* Ringpuffer für Ultraschall (US) Messungen (5 Proben) */
static float usBuf[5] = { 0, 0, 0, 0, 0 };
static uint8_t usPos = 0;  /* Aktuelle Position im US-Ringpuffer */

static volatile bool imuResetRequested = false; /* Flag, um IMU-Reset anzufordern */

/* Struktur für Controller-Eingaben vom RC-Empfänger */
struct Ctrl {
    int8_t rx, ry;    /* Positionswert des rechten Joysticks (X/Y) */
    int8_t lx, ly;    /* Positionswert des linken Joysticks (X/Y) */
    int8_t swc;       /* Status des Three-Position-Switch SWC */
    int8_t swd;       /* Status des Two-Position-Switch SWD */
    int8_t knob;      /* Wert des Drehknopfes */
} ctrl;

/* Array für Motor-Geschwindigkeiten */
int8_t speeds[4] = { 0, 0, 0, 0 };

/* Array für Polaritäts-Flags der Motoren (true = invertierte Richtung) */
bool motorPol[4] = { false, true, false, false };

/* Array für die zuletzt gelesenen Encoder-Werte (4 Motoren) */
int32_t latestEnc[4] = { 0 };

/* Arrays für zuletzt berechnete Umdrehungen und Strecke der Motoren */
float latestRev[4], latestDist[4];

/* Konstanten für Umrechnung: Impulse pro Umdrehung und Raddurchmesser */
const float pulsesPerRev = 555.0f;            /* Impulse pro Radumdrehung */
const float wheelDiameterMM = 40.0f;           /* Raddurchmesser in Millimetern */
const float wheelCircumference = wheelDiameterMM * 3.14159f; /* Radumfang */

/* Flags für Blinkerstatus */
static bool leftBlinking  = false;  /* Linker Blinker aktiv */
static bool rightBlinking = false;  /* Rechter Blinker aktiv */

/* Struktur für Sensordaten (IMU + US + TOF) */
struct {
    float roll, pitch, yaw;  /* IMU-Winkel */
    float usRaw, usFilt;     /* Ultraschall Roh- und gefilterter Wert */
    int16_t tof;             /* TOF-Messwert (gefiltert) */
} sensorData;

/* Puffer für Kamera-Stream-URL und Qualitätseinstellung */
char camStreamURL[32] = { 0 };
char camQuality[8]    = "mid";

/* Status der LED (an/aus) */
bool ledState = false;

/* WLAN-Zugangsdaten */
const char* WIFI_SSID = "SSID";
const char* WIFI_PASS = "PASSWORD";


/* Instanzen von Netzwerk-/Peripherieobjekten */
AsyncWebServer server(80);                    /* HTTP-Server auf Port 80 */
AsyncWebSocket ws("/ws");                     /* WebSocket unter /ws */

XplorerTCA9548A tca;                          /* I2C-Multiplexer TCA9548A */
XplorerIBUS rc;                               /* IBUS-Empfänger (RC) */
Adafruit_PWMServoDriver pwm(PCA9685_ADDR);    /* PCA9685-Servotreiber */
Xplorer74HC4052 uartMux(1, 0);                /* UART-Multiplexer 74HC4052 (Pins S1=1, S0=0) */
Xplorer_WT901 imu(Serial0, RX_PIN, TX_PIN, MUX_S0, MUX_S1, IMU_CH); /* WT901-IMU auf UART-Channel */
XplorerTOFvl53l1x tof(0x29);                  /* VL53L1X Time-of-Flight Sensor */
XplorerLED led(LED_PIN);                      /* NeoPixel-LED-Steuerung */
XplorerAutoLights autoLights(PIN, NUMPIXELS); /* Automatische Beleuchtungssteuerung (NeoPixel) */

/* FreeRTOS-Semaphoren für Bus-Zugriff */
SemaphoreHandle_t i2cMux;       /* I2C-Multiplexer-Mutex */
SemaphoreHandle_t uartMuxMux;   /* UART-Multiplexer-Mutex */

/* =============================================================================
 * Hilfsfunktionen
 * ============================================================================= */

/**
 * @brief Berechnet den getrimmten Mittelwert (Trimmed Mean) aus 5 int16-Werten.
 *        Entfernt den kleinsten und größten Wert, summiert die restlichen 3 und
 *        teilt durch 3.
 * @param b Zeiger auf ein Array mit 5 int16-Werten.
 * @return Durchschnitt der mittleren drei Werte (int16).
 */
static int16_t trimmedMean5(const int16_t* b) {
    int imin = 0, imax = 0;

    /* Indizes für Minimum und Maximum ermitteln */
    for (int i = 1; i < 5; i++) {
        if (b[i] < b[imin]) imin = i;
        if (b[i] > b[imax]) imax = i;
    }

    /* Summe der drei mittleren Werte berechnen */
    int sum = 0, cnt = 0;
    for (int i = 0; i < 5; i++) {
        if (i == imin || i == imax) continue; /* Skip min/max */
        sum += b[i];
        cnt++;
    }
    return (cnt > 0) ? (sum / cnt) : 0;
}

/**
 * @brief Berechnet den getrimmten Mittelwert (Trimmed Mean) aus 5 float-Werten.
 *        Entfernt den kleinsten und größten Wert, summiert die restlichen 3 und
 *        teilt durch 3.0f.
 * @param b Zeiger auf ein Array mit 5 float-Werten.
 * @return Durchschnitt der mittleren drei Werte (float).
 */
static float trimmedMean5f(const float* b) {
    uint8_t imin = 0, imax = 0;

    /* Indizes für Minimum und Maximum ermitteln */
    for (uint8_t i = 1; i < 5; i++) {
        if (b[i] < b[imin]) imin = i;
        if (b[i] > b[imax]) imax = i;
    }

    /* Summe der drei mittleren Werte berechnen */
    float sum = 0.0f;
    for (uint8_t i = 0; i < 5; i++) {
        if (i == imin || i == imax) continue; /* Skip min/max */
        sum += b[i];
    }
    return (sum / 3.0f);
}

/**
 * @brief Wählt den I2C-Kanal auf dem TCA9548A-Multiplexer aus.
 * @param ch Kanalnummer (0 bis 7), auf den umgeschaltet werden soll.
 */
void selectI2C(uint8_t ch) {
    Wire.beginTransmission(TCA_ADDR); /* I2C-Startbedingung für Multiplexer */
    Wire.write(1 << ch);             /* Bitmaske für Kanalwahl schreiben */
    Wire.endTransmission();           /* Übertragung beenden */
}

/**
 * @brief Setzt das Flag zum Zurücksetzen der IMU.
 *        In der IMU-Task wird dieses Flag abgefragt und die IMU zurückgesetzt.
 */
void requestIMUReset() {
    imuResetRequested = true;
}

/**
 * @brief Setzt die Encoder aller 4 Motoren auf Null zurück.
 *        Dazu wird über I2C eine Null-Array-Übertragung an das Encoder-Register gesendet.
 *        Schützt den Zugriff über einen I2C-Mutex.
 */
void resetEncoders() {
    uint8_t zeros[16] = { 0 }; /* 16-Byte-Array, alle 0 */

    /* I2C-Multiplexer-Mutex anfordern */
    if (xSemaphoreTake(i2cMux, pdMS_TO_TICKS(50))) {
        /* 1) Multiplexer auf Kanal für Motorsteuerung schalten */
        selectI2C(TCA_MOTORS);
        vTaskDelay(pdMS_TO_TICKS(2)); /* Kurze Verzögerung für Umschaltung */

        /* 2) 16 Null-Bytes an Encoder-Register senden */
        Wire.beginTransmission(I2C_ADDR);
        Wire.write(MOTOR_ENCODER);    /* 0x3C */
        Wire.write(zeros, 16);        /* 16 Byte Nulldaten */
        Wire.endTransmission();

        xSemaphoreGive(i2cMux);       /* I2C-Mutex freigeben */
        Serial.println(F("Encoders reset via I2C."));
    } else {
        Serial.println(F("ERROR: Cannot take i2cMux to reset encoders."));
    }
}

/**
 * @brief Sendet WLAN-Zugangsdaten an die Kamera über UART.
 *        Verwendet festen JSON-String mit SSID und Passwort.
 *        Schützt den Zugriff über UART-Multiplexer-Mutex.
 */
void sendCamWiFiCredentials() {
    /* Fester JSON-Befehl mit WLAN-Zugangsdaten */
    static const char cmd[] =
        "{\"wifi\":{\"ssid\":\"o2-HS5GSA-1121-UT\",\"pass\":\"927CZKK7D8MNMM8Z\"}}";

    /* 1) UART-Multiplexer auf Kamerakanal umschalten */
    uartMux.setChannel(CAM_CH);
    vTaskDelay(pdMS_TO_TICKS(5)); /* Kurze Verzögerung für Umschaltung */

    /* 2) JSON-Befehl senden und mit CRLF terminieren */
    Serial0.println(cmd);
    vTaskDelay(pdMS_TO_TICKS(50)); /* Zeit, damit die Kamera es parsen kann */

    /* 3) Log-Echo über USB-Serial */
    Serial.print(F("Sent WiFi credentials to CAM: "));
    Serial.println(cmd);
}

/**
 * @brief Verarbeitet eingehende Befehle (String) von WebSocket oder anderen Quellen.
 *        Unterstützte Befehle:
 *          - "ping": nichts tun (Heartbeats)
 *          - "ledon"/"ledoff": LED einschalten oder ausschalten
 *          - "low"/"mid"/"high": Qualitätsstufe der Kamera ändern
 *          - "reset_imu": IMU-Reset anfordern
 *          - "reset_encoders": Encoder-Reset durchführen
 *          - "reset_camera": WLAN-Credentials erneut an Kamera senden und Kamera neu starten
 *          - "reset_robot": ESP32 neu starten
 *        Unbekannte Befehle führen zu einer Fehlermeldung.
 *
 * @param cmd Referenz auf String mit Kommando
 */
void processCommand(const String& cmd) {
    if (cmd == "ping") {
        return; /* Ping: kein Inhalt, nur Antwortzeitpunkt aktualisieren */
    }

    StaticJsonDocument<64> resp; /* JSON-Dokument für Antwort */

    if (cmd == "ledon" || cmd == "ledoff") {
        /* Umschalten des UART-Multiplexers auf Kamerakanal */
        uartMux.setChannel(CAM_CH);
        vTaskDelay(pdMS_TO_TICKS(2));

        if (cmd == "ledon") {
            Serial0.println(F("{\"led\":\"on\"}")); /* JSON an Kamera senden */
            ledState = true;                        /* LED-Status updaten */
            resp["led"] = true;                     /* JSON-Antwort vorbereiten */
        } else {
            Serial0.println(F("{\"led\":\"off\"}"));
            ledState = false;
            resp["led"] = false;
        }

        String out;
        serializeJson(resp, out);  /* JSON-String generieren */
        ws.textAll(out);           /* Antwort an alle WebSocket-Clients senden */
    }
    else if (cmd == "low" || cmd == "mid" || cmd == "high") {
        /* Qualitätsstufe der Kamera ändern */
        uartMux.setChannel(CAM_CH);
        vTaskDelay(pdMS_TO_TICKS(2));

        /* Kommando in char-Array kopieren */
        char q[6];
        strlcpy(q, cmd.c_str(), sizeof(q));
        resp["quality"] = q;
        strlcpy(camQuality, q, sizeof(camQuality)); /* Globale Qualitätsvariable updaten */

        String out;
        serializeJson(resp, out);
        Serial0.println(out);      /* JSON an Kamera senden */
    }
    else if (cmd == "reset_imu") {
        requestIMUReset();          /* IMU-Reset-Flag setzen */
        Serial.print(F("IMU Reset\n"));
        return;                     /* Keine weitere Verarbeitung nötig */
    }
    else if (cmd == "reset_encoders") {
        Serial.print(F("Received reset_encoders command\n"));
        resetEncoders();            /* Encoder-Reset durchführen */
    }
    else if (cmd == "reset_camera") {
        uartMux.setChannel(CAM_CH);
        vTaskDelay(pdMS_TO_TICKS(5));
        sendCamWiFiCredentials();   /* WLAN-Credentials erneut senden */
        Serial0.println(F("{\"reset\":true}")); /* Kamera anweisen, sich zurückzusetzen */
    }
    else if (cmd == "reset_robot") {
        Serial.println(F("Robot reset in 1 Second"));
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();              /* ESP32-Neustart */
    }
    else {
        Serial.println(F("Unknown command")); /* Unbekanntes Kommando */
    }
}

/* =============================================================================
 * Task-Prototypen (FreeRTOS-Aufgaben)
 * ============================================================================= */
void Task_ReadIBUS(void*);       /* Liest RC-Empfänger-Daten (IBUS) */
void Task_IMU(void*);            /* Liest IMU-Daten und verarbeitet Reset-Anfrage */
void Task_Sensors(void*);        /* Liest Ultraschall- und TOF-Daten */
void Task_Drive(void*);          /* Steuert Motoren, Servos, LEDs, Blinker */
void Task_Report(void*);         /* Meldet Sensordaten und Telemetrie über WebSocket */
void Task_RequestCamIP(void*);   /* (Optional) Fragt Kamera-IP über UART ab */
void Task_ReadCamData(void*);    /* Liest Bilddaten (FPS, Qualität, Stream-IP) von Kamera */

/* =============================================================================
 * Arduino-Setup: Wird einmal beim Start aufgerufen
 * ============================================================================= */
void setup() {
    /* --- Serielle Schnittstellen initialisieren --- */
    Serial.begin(115200);                   /* USB-Serial für Debug-Ausgaben */
    Serial1.begin(115200, SERIAL_8N1, 6, -1); /* UART1: RX=6, TX=nicht genutzt (für IBUS) */
    uartMux.begin();                        /* Initialisiere UART-Multiplexer */

    /* --- Erstelle Mutex für UART-Multiplexer --- */
    uartMuxMux = xSemaphoreCreateMutex();
    if (!uartMuxMux) {
        Serial.println(F("ERROR: failed to create UART-Mux mutex"));
        while (1); /* Hänge bei Fehler */
    }

    /* --- Erstelle Mutex für I2C-Multiplexer --- */
    i2cMux = xSemaphoreCreateMutex();
    if (!i2cMux) {
        Serial.println(F("ERROR: failed to create I2C mutex"));
        while (1); /* Hänge bei Fehler */
    }

    /* --- I2C- und Servokonfiguration --- */
    Wire.begin(8, 9, 400000);   /* I2C starten: SDA=8, SCL=9, Frequenz=400kHz */
    selectI2C(TCA_SERVOS);      /* Multiplexer auf Servokanäle umschalten */
    pwm.begin();                /* PCA9685 starten */
    pwm.setPWMFreq(60);         /* PWM-Frequenz = 60 Hz */
    /* Servomittelstellung initialisieren */
    pwm.setPWM(0, 0, (S0_MIN + S0_MAX) / 2);
    pwm.setPWM(1, 0, (S1_MIN + S1_MAX) / 2);

    /* --- Time-of-Flight (TOF) Sensor initialisieren --- */
    selectI2C(TOF_I2C_CH);      /* Multiplexer auf TOF-Kanal umschalten */
    if (!tof.begin()) {
        Serial.print(F("FATAL: ToF init failed\n"));
        while (1) delay(10);    /* Hänge bei TOF-Initialisierungsfehler */
    }

    /* --- Motoren ausschalten --- */
    selectI2C(TCA_MOTORS);      /* Multiplexer auf Motorkanal umschalten */
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(MOTOR_SPEED);    /* Registeradresse für Motor-Geschwindigkeit */
    for (int i = 0; i < 4; i++) {
        Wire.write((uint8_t)0); /* Setze alle 4 Motoren auf Geschwindigkeit 0 */
    }
    Wire.endTransmission();

    /* --- LED- und Kamera-Konfiguration --- */
    led.begin();                /* NeoPixel-LEDs initialisieren */
    led.turnOff();              /* LEDs aus */
    pinMode(PIN_CAM_ENABLE, OUTPUT); /* Pin für Kamerasteuerung als Ausgang */
    digitalWrite(PIN_CAM_ENABLE, LOW); /* Kamera deaktiviert (Low) */
    autoLights.begin();         /* Automatische Beleuchtungssteuerung starten */
    autoLights.setBrakeLights(false);
    autoLights.setReverseLights(false);

    /* --- Ultraschall-Sensor konfigurieren --- */
    pinMode(US_TRIG_PIN, OUTPUT); /* Triggerpin als Ausgang */
    pinMode(US_ECHO_PIN, INPUT);  /* Echopin als Eingang */

    /* --- WiFi-Event-Handler registrieren --- */
    /* Warte kurz, damit WiFi-Modul initialisiert ist */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Event: WLAN-Station verbunden */
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        Serial.println(F("STA connected"));
        if (xSemaphoreTake(uartMuxMux, pdMS_TO_TICKS(50))) {
            /* WLAN-Credentials zur Kamera senden, wenn UART verfügbar */
            sendCamWiFiCredentials();
            xSemaphoreGive(uartMuxMux);
        }
    }, ARDUINO_EVENT_WIFI_STA_CONNECTED);

    /* Event: IP-Adresse erhalten */
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        Serial.println(F("Got IP"));
        Serial.println(WiFi.localIP());
    }, ARDUINO_EVENT_WIFI_STA_GOT_IP);

    /* Event: WLAN-Verbindung verloren */
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        if (xSemaphoreTake(i2cMux, pdMS_TO_TICKS(50))) {
            /* Motoren auf 0 setzen */
            selectI2C(TCA_MOTORS);
            Wire.beginTransmission(I2C_ADDR);
            Wire.write(MOTOR_SPEED);
            for (int i = 0; i < 4; i++) {
                Wire.write((uint8_t)0);
            }
            Wire.endTransmission();

            /* Servos auf Mittelstellung zurücksetzen */
            selectI2C(TCA_SERVOS);
            pwm.setPWM(0, 0, (S0_MIN + S0_MAX) / 2);
            pwm.setPWM(1, 0, (S1_MIN + S1_MAX) / 2);

            xSemaphoreGive(i2cMux);
        }
        /* LEDs ausschalten */
        led.turnOff();
        autoLights.setBrakeLights(false);
        autoLights.setReverseLights(false);
        Serial.println(F("WiFi lost, reconnecting"));
        WiFi.begin(); /* Verbindung erneut starten */
    }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

    /* --- WiFi-Verbindung starten --- */
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print(F("Connecting to WiFi"));
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(F("."));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    Serial.println();
    Serial.print(F("Connected! IP address: "));
    Serial.println(WiFi.localIP());

    /* --- mDNS (Bonjour) starten --- */
    if (!MDNS.begin("XplorerBot01")) {
        Serial.println(F("mDNS failed"));
        while (1);
    }
    Serial.println(F("mDNS responder started"));

    /* Nach dem Start: WLAN-Credentials an Kamera senden */
    if (xSemaphoreTake(uartMuxMux, pdMS_TO_TICKS(50))) {
        sendCamWiFiCredentials();
        xSemaphoreGive(uartMuxMux);
    }

    /* --- LittleFS mounten (Dateisystem auf Flash) --- */
    if (!LittleFS.begin(true)) {
        Serial.println(F("LittleFS mount failed"));
        while (1);
    }
    /* Statische Dateien unter "/" bereitstellen, Default-Datei "index.html" */
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    /* --- WebSocket-Event-Handler konfigurieren --- */
    ws.onEvent([](AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* a, uint8_t* d, size_t l) {
        if (t == WS_EVT_CONNECT) {
            /* Neuer WebSocket-Client verbunden */
            Serial.printf("WS Connect %u\n", c->id());
            if (camStreamURL[0]) {
                /* Wenn Kamerastream-URL bekannt, direkt senden */
                StaticJsonDocument<64> doc;
                doc["cam_ip"] = camStreamURL;
                String msg;
                serializeJson(doc, msg);
                c->text(msg);
            }
            /* LED-Zustand senden */
            StaticJsonDocument<64> doc2;
            doc2["led"] = ledState;
            String m2;
            serializeJson(doc2, m2);
            c->text(m2);
        }
        if (t == WS_EVT_DISCONNECT || t == WS_EVT_ERROR) {
            /* WebSocket getrennt oder Fehler */
            Serial.printf("WS %s %u\n", (t == WS_EVT_DISCONNECT) ? "Disc." : "Err.", c->id());
            if (xSemaphoreTake(i2cMux, pdMS_TO_TICKS(50))) {
                /* Motoren & Servos in sichere Position bringen */
                selectI2C(TCA_MOTORS);
                Wire.beginTransmission(I2C_ADDR);
                Wire.write(MOTOR_SPEED);
                for (int i = 0; i < 4; i++) {
                    Wire.write((uint8_t)0);
                }
                Wire.endTransmission();
                selectI2C(TCA_SERVOS);
                pwm.setPWM(0, 0, (S0_MIN + S0_MAX) / 2);
                pwm.setPWM(1, 0, (S1_MIN + S1_MAX) / 2);
                xSemaphoreGive(i2cMux);
            }
            /* LEDs ausschalten */
            led.turnOff();
            autoLights.setBrakeLights(false);
            autoLights.setReverseLights(false);
        }
        if (t == WS_EVT_DATA) {
            /* Daten über WebSocket empfangen */
            AwsFrameInfo* info = (AwsFrameInfo*)a;
            if (info->opcode == WS_TEXT) {
                d[l] = 0; /* Null-Terminierung des Strings */
                String cmd = (char*)d;
                if (xSemaphoreTake(uartMuxMux, pdMS_TO_TICKS(50))) {
                    processCommand(cmd); /* Kommando verarbeiten */
                    xSemaphoreGive(uartMuxMux);
                }
            }
        }
    });
    server.addHandler(&ws); /* WebSocket-Handler dem Server hinzufügen */
    server.begin();         /* HTTP- und WebSocket-Server starten */
    Serial.printf("HTTP+WS @ %s\n", WiFi.localIP().toString().c_str());

    /* --- OTA (Over-The-Air) Updates aktivieren --- */
    ElegantOTA.begin(&server, "admin", "password");
    Serial.printf("OTA ready @ %s\n", WiFi.localIP().toString().c_str());

    /* --- FreeRTOS-Tasks erstellen --- */
    xTaskCreate(Task_ReadIBUS, "IBUS",    1024, NULL, 3, NULL);
    xTaskCreate(Task_IMU,    "IMU",     1024, NULL, 4, NULL);
    xTaskCreate(Task_Sensors,"SNSR",    2048, NULL, 3, NULL);
    xTaskCreate(Task_Drive,  "DRIV",    2048, NULL, 2, NULL);
    xTaskCreate(Task_Report, "REPT",    3072, NULL, 1, NULL);
    xTaskCreate(
        Task_ReadCamData,      /* Task-Funktion */
        "ReadCamData",         /* Name (Debug) */
        2048,                  /* Stack-Size in Bytes */
        NULL,                  /* Parameter (keine) */
        4,                     /* Priorität (höher, damit genug CPU erhält) */
        NULL                   /* Task-Handle (nicht benötigt) */
    );
}

/* =============================================================================
 * Arduino-Loop: Wird kontinuierlich aufgerufen (hinter den Kulissen von FreeRTOS)
 * ============================================================================= */
void loop() {
    /* ElegantOTA.loop() im Loop aufrufen, damit OTA-Updates verarbeitet werden */
    ElegantOTA.loop();
}

/* =============================================================================
 * Task-Implementierungen
 * ============================================================================= */

/**
 * @brief Task: Liest kontinuierlich RC-Empfänger-Daten (IBUS) ein und aktualisiert
 *        die globale Struktur 'ctrl'.
 * @param pvParameters Unbenutzte Parameter (FreeRTOS-Prototyp).
 */
void Task_ReadIBUS(void*) {
    rc.begin(Serial1); /* IBUS-Objekt mit UART1 initialisieren */
    TickType_t next = xTaskGetTickCount(); /* Nächster Wakeup-Zeitpunkt */
    for (;;) {
        /* Werten des Contollers updaten */
        rc.loop();
        /* Joystick-Positionen und Schalterzustände lesen */
        ctrl.rx   = rc.readJoystick(CH_RX, -100, 100, 0, false);
        ctrl.ry   = rc.readJoystick(CH_RY, -100, 100, 0, false);
        ctrl.lx   = rc.readJoystick(CH_LX, -100, 100, 0, false);
        ctrl.ly   = rc.readJoystick(CH_LY, -100, 100, 0, false);
        ctrl.swc  = rc.readThreePositionSwitch(CH_SWC, true);
        ctrl.swd  = rc.readTwoPositionSwitch(CH_SWD, true);
        ctrl.knob = rc.readKnob(CH_KNOB, 0, 100, true);
        vTaskDelayUntil(&next, IBUS_MS); /* Bis zum nächsten Zyklus warten */
    }
}

/**
 * @brief Task: Liest IMU-Daten ein, verarbeitet Reset-Anfrage und aktualisiert
 *        die globale Struktur 'sensorData' (roll/pitch/yaw).
 * @param pvParameters Unbenutzte Parameter (FreeRTOS-Prototyp).
 */
void Task_IMU(void*) {
    /* --- Einmalige IMU-Initialisierung beim Task-Start --- */
    if (xSemaphoreTake(uartMuxMux, pdMS_TO_TICKS(50))) {
        /* UART-Multiplexer auf IMU-Kanal umschalten */
        uartMux.setChannel(IMU_CH);
        vTaskDelay(pdMS_TO_TICKS(2));

        imu.begin();  /* IMU initialisieren */
        xSemaphoreGive(uartMuxMux);
    }

    TickType_t next = xTaskGetTickCount();
    for (;;) {
        /* --- IMU-Reset verarbeiten, falls angefordert --- */
        if (imuResetRequested) {
            if (xSemaphoreTake(uartMuxMux, pdMS_TO_TICKS(50))) {
                uartMux.setChannel(IMU_CH);
                vTaskDelay(pdMS_TO_TICKS(2));
                imu.resetReference(); /* IMU-Referenz neu setzen */
                xSemaphoreGive(uartMuxMux);
            }
            imuResetRequested = false; /* Flag zurücksetzen */
        }

        /* --- IMU-Daten auslesen und speichern --- */
        if (xSemaphoreTake(uartMuxMux, pdMS_TO_TICKS(50))) {
            uartMux.setChannel(IMU_CH);
            vTaskDelay(pdMS_TO_TICKS(2));

            imu.update();                /* Neue IMU-Messung lesen */
            sensorData.roll  = imu.getRoll();
            sensorData.pitch = imu.getPitch();
            sensorData.yaw   = imu.getYaw();

            xSemaphoreGive(uartMuxMux);
        }

        vTaskDelayUntil(&next, IMU_MS); /* Bis zum nächsten Zyklus warten */
    }
}

/**
 * @brief Task: Liest Ultraschall- und TOF-Sensoren in festen Intervallen ein,
 *        filtert die Daten (Trimmed Mean) und speichert sie in 'sensorData'.
 * @param pvParameters Unbenutzte Parameter (FreeRTOS-Prototyp).
 */
void Task_Sensors(void*) {
    const float alpha = 0.6f, beta = 0.4f; /* (Nicht verwendet, evtl. für Filter) */
    float lastCm = 0.0f; /* Letzter gültiger Ultraschall-Wert */
    TickType_t next = xTaskGetTickCount();

    for (;;) {
        /* --- Ultraschall-Sensor (pulseIn) auslesen --- */
        digitalWrite(US_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(US_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(US_TRIG_PIN, LOW);
        unsigned long dur = pulseIn(US_ECHO_PIN, HIGH, 20000); /* Max. Wartezeit 20ms */
        float cm = dur ? (dur * 0.0343f / 2.0f) : lastCm;       /* Distanz in cm */
        if (dur) lastCm = cm;
        usBuf[usPos] = cm;  /* Rohwert in Ringpuffer speichern */
        usPos = (usPos + 1) % 5;

        sensorData.usRaw = cm;
        /* Getrimmter Mittelwert aus Ringpuffer berechnen */
        sensorData.usFilt = trimmedMean5f(usBuf);

        /* --- TOF-Sensor auslesen (nur, wenn I2C-Mutex verfügbar) --- */
        if (xSemaphoreTake(i2cMux, pdMS_TO_TICKS(50))) {
            selectI2C(TOF_I2C_CH);       /* Multiplexer auf TOF-Kanal umschalten */
            vTaskDelay(pdMS_TO_TICKS(2));/* Umschaltverzögerung */
            int16_t raw = tof.readDistance(false); /* Rohdistanz in mm */
            xSemaphoreGive(i2cMux);

            /* Puffer für TOF-Messungen füllen */
            tofBuf[tofPos] = raw;
            tofPos = (tofPos + 1) % 5;

            /* Getrimmten Mittelwert berechnen */
            sensorData.tof = trimmedMean5(tofBuf);
        }

        vTaskDelayUntil(&next, SENSOR_MS); /* Bis zum nächsten Zyklus warten */
    }
}

/**
 * @brief Task: Steuert die Motoren, Servos, LEDs und Blinker basierend auf RC-Eingaben
 *        in der Struktur 'ctrl'. Führt folgende Schritte aus:
 *          1. Throttle-/Turn-Werte berechnen
 *          2. Motorbefehle über I2C senden
 *          3. Servopositionen berechnen und setzen
 *          4. Kamera-Pin skripten (Ein/Aus)
 *          5. LED-Helligkeit basierend auf Knopf
 *          6. Blinker ein-/ausschalten
 * @param pvParameters Unbenutzte Parameter (FreeRTOS-Prototyp).
 */
void Task_Drive(void*) {
    TickType_t next = xTaskGetTickCount();

    for (;;) {
        /* --- Geschwindigkeits- und Lenkwinkel berechnen --- */
        int throttle = (ctrl.swc == 0) ? 0 : map(ctrl.ly, -95, 100, 0, 20);
        int turn     = (ctrl.swc == 0) ? 0 : map(ctrl.lx, -100, 100, -20, 20);
        int base = (ctrl.swc == 1) ? throttle : ((ctrl.swc == -1) ? -throttle : 0);
        int mL = constrain(base + turn, -20, 20); /* Linkes Rad */
        int mR = constrain(base - turn, -20, 20); /* Rechtes Rad */

        /* Geschwindigkeiten in Array schreiben (0 und 2 bleiben 0) */
        speeds[0] = 0;
        speeds[1] = mR;
        speeds[2] = 0;
        speeds[3] = mL;

        /* --- Motorbefehle senden (I2C) --- */
        if (xSemaphoreTake(i2cMux, pdMS_TO_TICKS(50))) {
            selectI2C(TCA_MOTORS);    /* Umschalten auf Motorkanal */
            Wire.beginTransmission(I2C_ADDR);
            Wire.write(MOTOR_SPEED);  /* Registeradresse */
            uint8_t out[4];
            for (int i = 0; i < 4; i++) {
                /* Invertiere Geschwindigkeit, falls Polarität gesetzt */
                out[i] = motorPol[i] ? -speeds[i] : speeds[i];
            }
            Wire.write(out, 4);       /* 4 Motor-Bytes senden */
            Wire.endTransmission();
            xSemaphoreGive(i2cMux);
        }

        /* --- Servos steuern (Lenkung und Kamera-Neigung) --- */
        int ang0 = map(ctrl.rx, -100, 100, 90, -90); /* Winkel für Servo 0 */
        int ang1 = constrain(map(ctrl.ry, -100, 100, 90, -90), -50, 70); /* Servo 1 */

        int p0 = map(ang0, 90, -90, S0_MAX, S0_MIN);
        int p1 = map(ang1, 70, -50, S1_MAX, S1_MIN);
        if (xSemaphoreTake(i2cMux, pdMS_TO_TICKS(50))) {
            selectI2C(TCA_SERVOS);
            pwm.setPWM(0, 0, p0); /* Servo 0 positionieren */
            pwm.setPWM(1, 0, p1); /* Servo 1 positionieren */
            xSemaphoreGive(i2cMux);
        }

        /* --- Kamera ein-/ausschalten basierend auf SWD --- */
        digitalWrite(PIN_CAM_ENABLE, ctrl.swd ? HIGH : LOW);

        /* --- LED-Helligkeit abhängig vom Knopf regeln --- */
        if (ctrl.knob > 5) {
            led.setBrightness(ctrl.knob);
            led.turnOn();
        } else {
            led.turnOff();
        }

        /* --- Brems- und Rücklichter steuern --- */
        autoLights.setBrakeLights(ctrl.swc == 0);
        autoLights.setReverseLights(ctrl.swc == -1);

        /* --- Blinkersteuerung (LX > Schwellwert) --- */
        const int TH = 20;
        if (ctrl.lx > TH && !rightBlinking) {
            autoLights.toggleRightBlinker();
            rightBlinking = true;
        } else if (ctrl.lx <= TH && rightBlinking) {
            autoLights.toggleRightBlinker();
            rightBlinking = false;
        }
        if (ctrl.lx < -TH && !leftBlinking) {
            autoLights.toggleLeftBlinker();
            leftBlinking = true;
        } else if (ctrl.lx >= -TH && leftBlinking) {
            autoLights.toggleLeftBlinker();
            leftBlinking = false;
        }
        autoLights.updateBlinkers();

        vTaskDelayUntil(&next, DRIVE_MS); /* Bis zum nächsten Zyklus warten */
    }
}

/**
 * @brief Task: Liest Encoder- und Batteriewerte aus, wendet Tiefpassfilter auf die
 *        Batteriespannung an, berechnet Telemetrie-Daten (Roll, Pitch, Yaw, US, TOF,
 *        Motor-Daten, Batterieladung, etc.) und sendet sie über WebSocket.
 * @param pvParameters Unbenutzte Parameter (FreeRTOS-Prototyp).
 */
void Task_Report(void*) {
    TickType_t next = xTaskGetTickCount();
    unsigned long lastCamSent = 0;      /* Zeitpunkt, wann zuletzt Kamera-IP gesendet wurde */
    const float alpha = 0.2f;           /* Tiefpassfilterfaktor für Batterie */
    static float battFilt = 0.0f;       /* Gefilterter Batteriewert (float) */

    for (;;) {
        /* --- Motordaten (Encoder) einlesen --- */
        if (xSemaphoreTake(i2cMux, pdMS_TO_TICKS(50))) {
            selectI2C(TCA_MOTORS);
            Wire.beginTransmission(I2C_ADDR);
            Wire.write(MOTOR_ENCODER);
            Wire.endTransmission();
            Wire.requestFrom(I2C_ADDR, (size_t)16); /* 4 Encoder x 4 Byte */

            for (int i = 0; i < 4; i++) {
                int32_t v = Wire.read() | (Wire.read() << 8) | (Wire.read() << 16) | (Wire.read() << 24);
                latestEnc[i] = motorPol[i] ? -v : v; /* Richtung berücksichtigen */
                latestRev[i]  = latestEnc[i] / pulsesPerRev; /* Umdrehungen berechnen */
                latestDist[i] = latestRev[i] * wheelCircumference; /* Distanz in mm */
            }
            xSemaphoreGive(i2cMux);
        }

        /* --- Batteriespannung über ADC einlesen --- */
        uint16_t battMV = 0;
        if (xSemaphoreTake(i2cMux, pdMS_TO_TICKS(50))) {
            selectI2C(TCA_MOTORS);
            Wire.beginTransmission(I2C_ADDR);
            Wire.write(ADC_BAT_ADDR);
            Wire.endTransmission();
            Wire.requestFrom(I2C_ADDR, (size_t)2); /* 2 Byte ADC-Wert */
            battMV = Wire.read() | (Wire.read() << 8);
            battMV += 300; /* Offset anpassen (Kalibrierung) */
            xSemaphoreGive(i2cMux);
        }

        /* --- Exponentieller Tiefpass für Batteriewert --- */
        if (battFilt == 0.0f) {
            battFilt = battMV; /* Initialisierung beim ersten Lauf */
        }
        battFilt = alpha * battMV + (1 - alpha) * battFilt;

        /* --- Batterieladung in Prozent berechnen --- */
        float battPC = map(constrain((int)battFilt, 10000, 12800), 10000, 12500, 0, 100) / 1.0f;

        /* --- Kamera-LED-Status für Telemetrie setzen --- */
        uint8_t cam_led = ledState ? 1 : 0;

        /* --- Variablen für Throttle und Turn erneut berechnen (float) --- */
        float thr = map(ctrl.ly, -95, 100, 0, 20);
        float turn = map(ctrl.lx, -100, 100, -20, 20);
        if (turn >= 19) turn = 20;

        /* --- Servopositionen für Telemetrie berechnen --- */
        int8_t serv0 = map(ctrl.rx, -100, 100, 90, -90);
        if (serv0 <= -88) serv0 = -90; /* Begrenzung auf -90 */

        /* --- Binärpuffer für WebSocket-Telemetrie aufbauen --- */
        float buf[FLOAT_COUNT] = {
            sensorData.roll,        /* 0: Roll-Winkel */
            sensorData.pitch,       /* 1: Pitch-Winkel */
            sensorData.yaw,         /* 2: Yaw-Winkel */
            sensorData.usRaw,       /* 3: Ultraschall-Roh */
            sensorData.usFilt,      /* 4: Ultraschall-gefiltert */
            (float)sensorData.tof,  /* 5: TOF (mm) */
            (float)ctrl.swc,        /* 6: SWC-Status */
            (float)ctrl.swd,        /* 7: SWD-Status */
            thr,                    /* 8: Throttle (float) */
            turn,                   /* 9: Turn (float) */
            (float)speeds[1],       /* 10: Geschwindigkeit rechtes Rad */
            (float)speeds[3],       /* 11: Geschwindigkeit linkes Rad */
            (float)latestEnc[1],    /* 12: Encoder rechtes Rad (raw) */
            (float)latestEnc[3],    /* 13: Encoder linkes Rad (raw) */
            latestRev[1],           /* 14: Umdrehungen rechtes Rad */
            latestRev[3],           /* 15: Umdrehungen linkes Rad */
            latestDist[1],          /* 16: Distanz rechtes Rad */
            latestDist[3],          /* 17: Distanz linkes Rad */
            (float)serv0,           /* 18: Servo0-Winkel */
            (float)constrain(map(ctrl.ry, -100, 100, 90, -90), -50, 70), /* 19: Servo1-Winkel */
            (float)WiFi.RSSI(),     /* 20: WLAN-Signalstärke */
            battFilt,               /* 21: Batteriewert (gefiltert) */
            (float)ctrl.knob,       /* 22: Knopfwert */
            (float)cam_led          /* 23: Kamera-LED-Status */
        };
        ws.binaryAll((uint8_t*)buf, sizeof(buf)); /* Telemetrie binär senden */

        /* --- Kamera-Stream-IP und Qualität periodisch über WebSocket senden --- */
        unsigned long now = millis();
        if (camStreamURL[0] && (now - lastCamSent > 1000)) {
            StaticJsonDocument<64> j;
            j["cam_ip"]  = camStreamURL;
            j["quality"] = camQuality;
            String out;
            serializeJson(j, out);
            ws.textAll(out);
            lastCamSent = now;
        }

        vTaskDelayUntil(&next, REP_MS); /* Bis zum nächsten Zyklus warten */
    }
}

/**
 * @brief Task: Abgeänderter Kameraprotokoll-Task, der einmal pro Sekunde die Kamera
 *        nach Stream-IP, Qualität und FPS abfragt und die empfangenen JSON-Daten
 *        verarbeitet, um globale Variablen camStreamURL und camQuality zu aktualisieren.
 * @param pvParameters Unbenutzte Parameter (FreeRTOS-Prototyp).
 */
void Task_ReadCamData(void*) {
    const TickType_t period = pdMS_TO_TICKS(1000); /* Zyklusdauer = 1 Sekunde */
    TickType_t nextWake = xTaskGetTickCount();
    static char buf[128]; /* Puffer für eingehende JSON-Zeilen */
    size_t idx = 0;       /* Leseindex in Puffer */

    for (;;) {
        /* --- 1) Pro Zyklus: Kamerakanal auswählen und Stream-IP-Abfrage senden --- */
        if (xSemaphoreTake(uartMuxMux, pdMS_TO_TICKS(50))) {
            uartMux.setChannel(CAM_CH);     /* Multiplexer auf Kamera umschalten */
            vTaskDelay(pdMS_TO_TICKS(5));   /* Kurze Verzögerung */

            /* JSON-Befehl senden, um die Kamera nach Stream-IP & Qualität zu fragen */
            Serial0.print("{\"wifi\":\"get_stream_ip\"}\n");
            
            xSemaphoreGive(uartMuxMux);
        }

        /* --- 2) Bis zu 200 ms: Eingehende JSON-Zeile von Serial0 einlesen --- */
        unsigned long start = millis();
        idx = 0;
        while ((millis() - start < 200) && (idx < (sizeof(buf) - 1))) {
            if (xSemaphoreTake(uartMuxMux, pdMS_TO_TICKS(50))) {
                uartMux.setChannel(CAM_CH); /* Sicherstellen, dass MUX auf Kamera steht */
                vTaskDelay(pdMS_TO_TICKS(5)); /* Umschaltverzögerung */

                /* Alle verfügbaren Bytes lesen */
                while (Serial0.available() && (idx < (sizeof(buf) - 1))) {
                    char c = Serial0.read();
                    /* Gebe bei Zeilenumbruch Null-Terminator ein */
                    if (c == '\n' || c == '\r') {
                        buf[idx] = '\0';
                        break;
                    }
                    /* Nur druckbare Zeichen übernehmen */
                    if (isPrintable(c)) {
                        buf[idx++] = c;
                    }
                }
                xSemaphoreGive(uartMuxMux);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            /* Wenn wir bereits Null-Terminator haben, abbrechen */
            if ((idx > 0) && (buf[idx] == '\0')) break;
        }

        /* --- 3) Wenn vollständige JSON-Zeile vorhanden, parsen und verarbeiten --- */
        if ((idx > 0) && (buf[0] != '\0')) {
            StaticJsonDocument<128> doc;
            DeserializationError err = deserializeJson(doc, buf);
            if (!err) {
                /* Nur überschreiben, wenn Schlüssel existiert */
                if (doc.containsKey("stream_ip")) {
                    strlcpy(camStreamURL, doc["stream_ip"], sizeof(camStreamURL));
                }
                if (doc.containsKey("quality")) {
                    strlcpy(camQuality, doc["quality"], sizeof(camQuality));
                }
            }
            idx = 0; /* Puffer-Index zurücksetzen */
        }

        /* --- 4) Bis zum nächsten Zyklus (1 Sekunde) blockieren --- */
        vTaskDelayUntil(&nextWake, period);
    }
}
