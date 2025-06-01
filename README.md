# XplorerBot

**ESP32C3 (als Steuerung) & AI-Thinker ESP32-CAM (für Video) basierter Roboter**

Dieses Repository enthält  
- Den FreeRTOS-basierten Haupt-Sketch für den ESP32-C3 (`XplorerBot.cpp`)  
- Den ESP32-CAM-Sketch (AI-Thinker) für Live-Video-Streaming (`CameraWebServer/`)  
- Ein einfaches Web-Dashboard (HTML, CSS, JS + WebSocket)  
- Alle dazugehörigen Bibliotheken im Ordner `libraries/`  

## Features

1. IBUS-Fernsteuerung (RC-Empfänger) über UART1 (GPIO 16 als RX)  
2. Motoren & Servos über I2C-Multiplexer (TCA9548A) + PCA9685  
3. Sensoren  
   - WT901-IMU über UART (74HC4052-Multiplexer)  
   - HC-SR04 Ultraschall  
   - VL53L1X ToF  
4. Telemetrie & Web-Dashboard  
   - WebSocket sendet live Floats (Roll, Pitch, Yaw, US, ToF, Encoder, Battery, RSSI usw.)  
   - Dashboard-Dateien (`dashboard.html`, `dashboard.css`, `dashboard.js`) zeigen Sensordaten und Video-Stream  
5. OTA-Updates via ElegantOTA (ESP32C3)  
6. LittleFS zum Serven von HTML, CSS und JS auf dem ESP32C3  

## Pin-Belegung (ESP32C3)

| Funktion                         | Pin (ESP32C3)   | Beschreibung                                   |
|----------------------------------|-----------------|------------------------------------------------|
| IBUS-RX (RC-Empfänger)           | GPIO 16         | UART1_RX, 115200 Bd                            |
| UART-MUX S0 / S1                 | GPIO 0 / GPIO 1 | Steuerleitungen für 74HC4052 (IMU & Kamera)     |
| IMU (WT901)                      | UART0 (GPIO 20/21) über MUX                       |
| Kamera (ESP32-CAM)               | UART0 über MUX   | AI-Thinker ESP32-CAM, PIN_CAM_ENABLE = 7        |
| TCA9548A SDA / SCL               | GPIO 8 / GPIO 9  | I2C (400 kHz)                                   |
| TCA-Motor-Kanal                  | I2C-Kanal 3      | Adresse 0x70 (TCA9548A)                        |
| TCA-Servo-Kanal                  | I2C-Kanal 7      | Adresse 0x70 (TCA9548A)                        |
| PCA9685 (Servos)                 | via TCA          | Adresse 0x40                                    |
| NeoPixel-Daten                   | GPIO 5           | 12 LEDs                                         |
| US-Trig / US-Echo (HC-SR04)      | GPIO 3 / GPIO 4  |                                                 |
| TOF (VL53L1X)                    | I2C-Kanal 0      | Adresse 0x29                                    |
| Motor-I2C (Speed / Encoder)      | I2C-Adresse 0x34 | Register 0x33 / 0x3C                            |
| LED_PIN (NeoPixel)               | GPIO 10          |                                                 |

## Software-Abhängigkeiten

Installiere in der Arduino IDE unter **Sketch → Bibliothek einbinden → Bibliotheken verwalten**:  
- ESP32-C3 Board Support (Boardverwalter → esp32 by Espressif Systems)  
- ArduinoJson  
- Adafruit BusIO  
- Adafruit PWM Servo Driver Library (PCA9685)  
- AsyncTCP  
- ESPAsyncWebServer  
- ElegantOTA  
- LittleFS by lorol  

Zusätzlich Ordner aus `libraries/` ins Arduino-Libraries-Verzeichnis kopieren (z. B. `Dokumente/Arduino/libraries/`):  
- XplorerTCA9548A  
- XplorerIBUS  
- Xplorer74HC4052  
- Xplorer_WT901  
- XplorerUltrasonicSensor  
- XplorerTOFvl53l1x  
- XplorerLED  
- XplorerAutoLights  

## Ordnerstruktur

```
XplorerBot/
├── CameraWebServer/         ← ESP32-CAM-Code für Video-Stream
├── RobotWebsocketsFreeRTOS_opt/
│   └── XplorerBot.cpp       ← Haupt-Sketch für ESP32-C3
├── dashboard.html           ← Web-Dashboard (LittleFS)
├── dashboard.css
├── dashboard.js
├── libraries/               ← Alle Xplorer-Bibliotheken hier
├── camera_offline.jpg       ← Platzhalter, falls kein Stream
├── .gitignore
└── README.md                ← Diese Datei
```

## Installation & Hochladen

1. **ESP32-C3 in der Arduino IDE auswählen**  
   - Tools → Board: „ESP32C3 Dev Module“  
   - Tools → Port: Wähle den COM-Port des ESP32-C3 aus  

2. **Bibliotheken installieren** (siehe oben unter „Software-Abhängigkeiten“).

3. **Projekt öffnen**  
   - Öffne in der Arduino IDE `RobotWebsocketsFreeRTOS_opt/XplorerBot.cpp`.

4. **Kompilieren & Hochladen**  
   - Klicke auf 🗸 (Verify) zum Kompilieren  
   - Klicke auf → (Upload), um den Sketch aufs ESP32-C3 zu flashen

5. **ESP32-CAM einrichten**  
   - Öffne in der Arduino IDE den Ordner `CameraWebServer/`  
   - Tools → Board: „AI-Thinker ESP32-CAM“  
   - Klicke auf → (Upload), um das Kameraskript zu flashen

## Betrieb

1. **Serieller Monitor** (115200 Baud)  
   - Zeigt Debug-Ausgaben, IP-Adresse, OTA-Status etc.

2. **IBUS**  
   - Stelle sicher, dass der IBUS-Ausgang des RC-Empfängers an GPIO 16 (UART1_RX) liegt  
   - `Task_ReadIBUS` liest alle 10 ms Steuerbefehle ein

3. **Web-Dashboard**  
   - ESP32-C3 verbindet sich per WLAN (SSID & Passwort in `XplorerBot.cpp`)  
   - Im Browser öffnen:  
     ```
     http://<ESP32_C3_IP>/
     ```  
   - Dashboard verbindet sich via WebSocket:  
     ```
     ws://<ESP32_C3_IP>/ws
     ```  
   - Anzeige von Sensordaten und Video-Stream  
   - Video-Stream (ESP32-CAM):  
     ```
     http://<ESP32CAM_IP>/stream
     ```

## OTA-Updates

1. Im Seriellen Monitor nach folgender Zeile suchen:  
   ```
   OTA ready @ <ESP32_C3_IP>
   ```
2. Im Browser öffnen:  
   ```
   http://<ESP32_C3_IP>/update
   ```
3. Neue `.bin`-Datei auswählen und OTA flashen lassen

## Lizenz

Dieses Projekt steht unter der MIT License:

```
MIT License

Copyright (c) 2025 Yassine Louchi - yassinelouchi.sf@gmail

Permission is hereby granted, free of charge, to any person obtaining a copy  
of this software and associated documentation files (the “Software”), to deal  
in the Software without restriction, including without limitation the rights  
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell  
copies of the Software, and to permit persons to whom the Software is  
furnished to do so, subject to the following conditions:

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,  
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE  
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER  
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN  
THE SOFTWARE.
```
