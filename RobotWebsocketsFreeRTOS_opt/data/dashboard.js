/*
 * File: dashboard.js
 * Description: JavaScript für das Robot Dashboard. Stellt eine WebSocket-Verbindung
 *              zum Backend her, empfängt Binärdaten (Float32-Array) und Textnachrichten
 *              (JSON), um das HUD (Heads-Up-Display) und das seitliche Panel live zu aktualisieren.
 *              Enthält Funktionen zum Rendern von Telemetrie, Steuerelemente (Buttons, Slider),
 *              Fullscreen-Unterstützung für den Kamerastream sowie Datum/Uhrzeit-Updates.
 * Author: Dein Name
 * Date: 2025-06-01
 */

/* === Konstanten und globale Variablen === */

/* Zeitintervall (ms), bevor nach einer geschlossenen WS-Verbindung erneut verbunden wird */
const RECONNECT_DELAY = 500;

/* WebSocket-Instanz */
let socket = null;

/* Timer-ID für erneutes Verbinden (setTimeout) */
let reconnectTimer = null;

/* Timer-ID für regelmäßiges Senden von 'ping' über WebSocket (setInterval) */
let wsPingTimer = null;

/* Letzt empfangenes Frame (Float32Array) aus Binärnachricht */
let latestFrame = null;

/* Aktuelle IP-Adresse des Kamerastreams (String, z.B. "http://192.168.1.2/stream") */
let cameraIP = "";

/* Aktuelle Qualität als String ("HIGH", "MEDIUM", "LOW" oder "–" initial) */
let latestQuality = '–';

/*
 * Funktion: connectWS
 * -------------------
 * Stellt eine WebSocket-Verbindung zum Server unter ws://<hostname>/ws her.
 * Setzt event handlers:
 *   - onopen:  WebSocket geöffnet → Statustext aktualisieren, Ping-Intervall starten
 *   - onerror: Fehler → Fehlerprotokoll, Statustext "WS error", WS schließen
 *   - onclose: Verbindung geschlossen → Warnung, Statustext "WS closed", Ping-Intervall stoppen,
 *              nach RECONNECT_DELAY erneut versuchen
 *   - onmessage: Bei Empfang:
 *                  - Binärdaten (ArrayBuffer) → In Float32Array umwandeln → latestFrame aktualisieren
 *                  - Textnachrichten → handleTextMessage aufrufen
 */
function connectWS() {
  console.log(`▶️ Connecting to ws://${location.hostname}/ws`);
  socket = new WebSocket(`ws://${location.hostname}/ws`);
  /* Stelle sicher, dass empfangene Binärnachrichten als ArrayBuffer interpretiert werden */
  socket.binaryType = "arraybuffer";

  /* --- WebSocket-Event: Verbindung hergestellt --- */
  socket.onopen = () => {
    console.log('✅ WS connected');
    /* Statusanzeige im HUD aktualisieren */
    document.getElementById('statusText').textContent = 'WS connected';
    /* Alle 5 Sekunden ein 'ping' senden, um Verbindung aktiv zu halten */
    wsPingTimer = setInterval(() => {
      if (socket.readyState === WebSocket.OPEN) {
        socket.send('ping');
      }
    }, 5000);
  };

  /* --- WebSocket-Event: Fehler aufgetreten --- */
  socket.onerror = err => {
    console.error('❌ WS error', err);
    document.getElementById('statusText').textContent = 'WS error';
    /* WebSocket schließen, onclose-Handler übernimmt Reconnect-Logik */
    socket.close();
  };

  /* --- WebSocket-Event: Verbindung geschlossen --- */
  socket.onclose = () => {
    console.warn(`🔌 WS closed — reconnecting in ${RECONNECT_DELAY}ms`);
    document.getElementById('statusText').textContent = 'WS closed';
    /* Ping-Intervall stoppen */
    clearInterval(wsPingTimer);
    /* Nach RECONNECT_DELAY ms erneut connectWS aufrufen */
    reconnectTimer = setTimeout(connectWS, RECONNECT_DELAY);
  };

  /* --- WebSocket-Event: Nachricht empfangen --- */
  socket.onmessage = evt => {
    /* Unterscheide zwischen Binärdaten und Textnachrichten */
    if (evt.data instanceof ArrayBuffer) {
      /* Binärdaten → Float32Array */
      latestFrame = new Float32Array(evt.data);
    } else {
      /* Textnachricht (JSON oder einfacher String) → verarbeiten */
      handleTextMessage(evt.data);
    }
  };
}

/*
 * Funktion: handleTextMessage
 * ---------------------------
 * Verarbeitet eingehende Textnachrichten vom WebSocket (erwartet JSON-Strings).
 *   - Parsen des JSON-Strings; falls fehlerhaft → Warnung und Abbruch
 *   - Wenn "cam_ip" im JSON → updateStreamURL aufrufen
 *   - Wenn "quality" im JSON → Anzeige aktualisieren (Großbuchstaben-Label, Buttons)
 *   - Wenn "led" im JSON → updateLedToggle aufrufen
 *
 * data: Eingehender Text (evt.data)
 */
function handleTextMessage(data) {
  console.log('📩 WS Text:', data);
  try {
    const msg = JSON.parse(data);

    /* --- Kamerastream-URL empfangen --- */
    if (msg.cam_ip) {
      updateStreamURL(msg.cam_ip);
    }

    /* --- Qualitätsänderung empfangen --- */
    if (msg.quality) {
      let displayText;
      /* Mapping von internen Strings ('high', 'mid', 'low') auf Anzeigenamen */
      switch (msg.quality) {
        case 'high':
          displayText = 'HIGH';
          break;
        case 'mid':
          displayText = 'MEDIUM';
          break;
        case 'low':
          displayText = 'LOW';
          break;
        default:
          /* Fallback: gesamte Zeichenkette in Großbuchstaben */
          displayText = msg.quality.toUpperCase();
      }

      /* Buttons hervorheben (active Class) */
      updateActiveQualityButton(msg.quality);

      /* Qualität im HUD anzeigen (<span id="camQualityDisplay">) */
      const disp = document.getElementById('camQualityDisplay');
      if (disp) {
        disp.textContent = displayText;
      }
      /* Globale Variable aktualisieren */
      latestQuality = displayText;
    }

    /* --- LED-Zustand empfangen (true/false) --- */
    if (msg.led !== undefined) {
      updateLedToggle(msg.led);
    }
  } catch (e) {
    console.warn('Invalid JSON message:', data);
  }
}

/* =============================================================================
 * HUD-Update-Routinen
 * ============================================================================= */

/*
 * Funktion: setTable
 * ------------------
 * Füllt eine Tabelle in einer HUD-Box mit beschrifteten Werten.
 *
 * id:   ID des HUD-Box-Containers (z.B. 'orientation', 'distance', etc.)
 * rows: Array von Paare [Label, Wert], z.B. [['Yaw', '10°'], ['Pitch', '5°']]
 */
function setTable(id, rows) {
  const tbl = document.getElementById(id).querySelector('table');
  /* Setze innerHTML mit einer Reihe von <tr><td>…</td></tr> */
  tbl.innerHTML = rows.map(([lbl, val]) =>
    `<tr><td>${lbl}</td><td>:</td><td>${val}</td></tr>`
  ).join('');
}

/*
 * Funktion: updateHUD
 * -------------------
 * Aktualisiert alle HUD-Boxen (die sich innerhalb des Kamerastream-Containers befinden):
 *   - Datum/Uhrzeit
 *   - Orientierung (Yaw, Pitch, Roll)
 *   - Entfernung (Ultraschall, TOF)
 *   - Drive-Daten (Drive Mode, Throttle, Turn, Speed R/L, Main LED)
 *   - Servo-Status (Tilt, Pan, Cam LED, FPV Camera, IP-Camera Qualität)
 *   - Systeminformationen (Battery, RSSI, Status)
 *   - Position und Rotation der Crosshair-Linien basierend auf Pitch, Roll, Turn
 *   - Optional Text-Overlay (wenn <div id="telemetry"> existiert)
 *
 * Verwendet das globale latestFrame (Float32Array), Abbruch, falls kein Frame vorhanden.
 */
function updateHUD() {
  if (!latestFrame) return;
  const f = latestFrame;

  /* --- Datum/Uhrzeit anzeigen --- */
  const dt = getFormattedDateTime();
  setTable("datetime", [
    ["Date/Time", dt]
  ]);

  /* --- Orientierung: Yaw (f[2]), Pitch (f[1]), Roll (f[0]) --- */
  setTable('orientation', [
    ['Yaw', `${f[2].toFixed(1)}°`],
    ['Pitch', `${f[1].toFixed(1)}°`],
    ['Roll', `${f[0].toFixed(1)}°`]
  ]);

  /* --- Entfernung: Ultraschall (f[3]), TOF (f[5]) --- */
  setTable('distance', [
    ['Ultrasonic', `${f[3].toFixed(1)} cm`],
    ['TOF', `${f[5]} mm`]
  ]);

  /* --- Drive-Daten: Drive Mode (f[6]), Throttle (f[8]), Turn (f[9]),
   *     Speed R (f[10]), Speed L (f[11]), Main LED (f[22]) --- */
  /* Drive Mode: 1→'F', 0→'P', –1→'R', sonst '!'. */
  const driveMode = f[6] === 1 ? 'F'
    : f[6] === 0 ? 'P'
      : f[6] === -1 ? 'R'
        : '!';
  setTable('drive', [
    ['Drive Mode', driveMode],
    ['Throttle', f[8]],
    ['Turn', f[9]],
    ['Speed R', f[10]],
    ['Speed L', f[11]],
    ['Main LED', `${f[22]} %`]
  ]);

  /* --- Servo-Status: Tilt (f[18]), Pan (f[19]), Cam LED (f[23]), FPV Camera (f[7]), IP-Camera Qualität --- */
  const fpvState = f[7] ? 'ON' : 'OFF';
  const camLedState = f[23] ? 'ON' : 'OFF';
  setTable('servo', [
    ['Servo Tilt', `${f[18]}°`],
    ['Servo Pan', `${f[19]}°`],
    ['Cam LED', camLedState],
    ['FPV Camera', fpvState],
    ['IP-Camera', latestQuality]
  ]);

  /* --- Systeminformationen: Battery (f[21]), RSSI (f[20]), Status (from statusText element) --- */
  setTable('system', [
    ['Battery', `${f[21].toFixed(0)} mV`],
    ['RSSI', `${f[20]} dBm`],
    ['Status', document.getElementById('statusText').textContent]
  ]);

  /* =============================================================================
   * Crosshair-Positionierung und Rotation
   * ============================================================================= */

  /* --- Horizontales Kreuz: bewegt sich vertikal anhand Pitch und rotiert anhand Roll --- */
  const horizontalLine = document.querySelector('.crosshair .line.horizontal');
  if (horizontalLine) {
    /* 1) Vertikale Position anhand Pitch (–25…+25 → 0…200px) */
    const pitch = Math.max(-25, Math.min(25, f[1]));
    const crosshairHeight = 200;
    const yOffset = ((pitch + 25) / 50) * crosshairHeight;
    horizontalLine.style.top = `${yOffset}px`;

    /* 2) Rotation anhand Roll (–45…+45 → –45°…+45°) */
    const roll = Math.max(-45, Math.min(45, f[0]));
    horizontalLine.style.transform = `translateY(-50%) rotate(${roll}deg)`;
  }

  /* --- Vertikales Kreuz: bewegt sich horizontal anhand Turn, keine Rotation --- */
  const verticalLine = document.querySelector('.crosshair .line.vertical');
  if (verticalLine) {
    /* Turn (–20…+20 → 0…200px) */
    const turn = Math.max(-20, Math.min(20, f[9]));
    const crosshairWidth = 200;
    const xOffset = ((turn + 20) / 40) * crosshairWidth;
    verticalLine.style.left = `${xOffset}px`;
    /* Behalte perfekte Vertikalität (translateX(-50%)) */
    verticalLine.style.transform = 'translateX(-50%)';
  }

  /* =============================================================================
   * Optional: Text Overlay Telemetry (wenn <div id="telemetry"> existiert)
   * ============================================================================= */
  const telem = document.getElementById('telemetry');
  if (telem) {
    telem.textContent =
      `Servo Tilt: ${f[18]}°\n` +
      `Servo Pan : ${f[19]}°\n` +
      `TOF       : ${f[4]} mm\n` +
      `Ultrasonic: ${f[5].toFixed(1)} cm\n` +
      `Yaw       : ${f[2].toFixed(1)}°\n` +
      `Pitch     : ${f[1].toFixed(1)}°\n` +
      `Roll      : ${f[0].toFixed(1)}°\n` +
      `Throttle  : ${f[8]}\n` +
      `Turn      : ${f[9]}\n` +
      `Speed R   : ${f[10]}\n` +
      `Speed L   : ${f[11]}\n` +
      `RSSI      : ${f[20].toFixed(1)}\n` +
      `Battery   : ${f[21].toFixed(0)}%\n`;
  }
}

/*
 * Funktion: renderTelemetry
 * --------------------------
 * Startet ein Intervall (100 ms), das sowohl updateHUD als auch updateSidePanel
 * aufruft, um die HUD-Boxen und das Seitenpanel regelmäßig zu aktualisieren.
 */
function renderTelemetry() {
  setInterval(() => {
    updateHUD();
    updateSidePanel();
  }, 100);
}

/* =============================================================================
 * Control Panel (Buttons, Dropdown, Slider) – Event Listener Setup
 * ============================================================================= */

/*
 * Funktion: setupControls
 * -----------------------
 * Bindet Event-Listener an verschiedene Steuerelemente im Footer:
 *   - LED-Toggle (Checkbox) → sendet 'ledon' bzw. 'ledoff'
 *   - Qualitätsbuttons (btnLow, btnMid, btnHigh) → sendet 'low', 'mid', 'high'
 *   - Aktion-Button (Go für Dropdown) → sendet 'reset_robot' oder 'reset_camera'
 *   - HUD-Hintergrund-Slider → passt Alpha-Wert von .hud-box-Hintergrund an
 *   - Reset-Buttons (IMU, Encoders) → sendet 'reset_imu' bzw. 'reset_encoders', inklusive visueller Feedback-Klasse
 */
function setupControls() {
  /* Hilfsfunktion zum Senden von Befehlen, falls WS geöffnet ist */
  const send = cmd => {
    if (socket?.readyState === WebSocket.OPEN) {
      socket.send(cmd);
      console.log('Sent command:', cmd);
    }
  };

  /* --- LED Toggle Checkbox --- */
  document.getElementById('ledToggle').addEventListener('change', e => {
    send(e.target.checked ? 'ledon' : 'ledoff');
  });

  /* --- Qualitätsbuttons: btnLow, btnMid, btnHigh --- */
  ['Low', 'Mid', 'High'].forEach(level => {
    document.getElementById(`btn${level}`)
      .addEventListener('click', () => send(level.toLowerCase()));
  });

  /* --- Dropdown-Aktion 'Go' (btnActionSend) --- */
  const btnActionSend = document.getElementById('btnActionSend');
  if (btnActionSend) {
    btnActionSend.addEventListener('click', () => {
      const sel = document.getElementById('actionSelect');
      const val = sel.value;        // z.B. 'reset_robot' oder 'reset_camera'
      if (val) {
        send(val);
        sel.selectedIndex = 0;      // Auswahl zurücksetzen
      }
    });
  }

  /* --- Slider für HUD-Hintergrundtransparenz (lightSlider) --- */
  const slider = document.getElementById('lightSlider');
  if (slider) {
    /* Maximale Alpha-Transparenz, die ursprünglich gewünscht war */
    const maxAlpha = 0.6;

    /*
     * Funktion: applyHudBackground
     * ----------------------------
     * Wendet eine Hintergrundfarbe mit Alpha auf alle .hud-box-Elemente an.
     *
     * value: Wert des Sliders (0…100), wird auf 0.0…maxAlpha gemappt
     */
    const applyHudBackground = (value) => {
      const alpha = (value / 100) * maxAlpha; /* 0.0 … maxAlpha */
      /* Feste RGB-Komponente (125,123,123), nur Alpha variiert */
      const bg = `rgba(125, 123, 123, ${alpha.toFixed(3)})`;
      /* Setze für jedes HUD-Box-Element den Hintergrundstil */
      document.querySelectorAll('.hud-box').forEach(box => {
        box.style.background = bg;
      });
    };

    /* Event: Slider bewegt → HUD-Hintergrund aktualisieren */
    slider.addEventListener('input', e => {
      applyHudBackground(parseInt(e.target.value, 10));
    });

    /* Initiale Anwendung, falls Slider nicht auf 0 steht */
    applyHudBackground(parseInt(slider.value, 10));
  }

  /* --- Button Reset IMU (btnResetIMU) --- */
  const btnResetIMU = document.getElementById('btnResetIMU');
  if (btnResetIMU) {
    btnResetIMU.addEventListener('click', () => {
      send('reset_imu');
      /* Visuelles Feedback: 'active'-Klasse kurz hinzufügen */
      btnResetIMU.classList.add('active');
      setTimeout(() => btnResetIMU.classList.remove('active'), 150);
    });
  }

  /* --- Button Reset Encoders (btnResetEnc) --- */
  const btnResetEnc = document.getElementById('btnResetEnc');
  if (btnResetEnc) {
    btnResetEnc.addEventListener('click', () => {
      send('reset_encoders');
      btnResetEnc.classList.add('active');
      setTimeout(() => btnResetEnc.classList.remove('active'), 150);
    });
  }
}

/*
 * Funktion: updateActiveQualityButton
 * -----------------------------------
 * Hebt den aktuell ausgewählten Qualitätsbutton (btnLow, btnMid oder btnHigh) hervor,
 * indem die Klasse 'btn-primary' gesetzt wird, alle anderen werden zu 'btn-secondary'.
 *
 * quality: String in Kleinbuchstaben (z.B. 'low', 'mid', 'high')
 */
function updateActiveQualityButton(quality) {
  ['Low', 'Mid', 'High'].forEach(level => {
    const btn = document.getElementById(`btn${level}`);
    /* Wenn der Button zum aktuellen quality-String passt, mache ihn primär */
    btn.classList.toggle('btn-primary', level.toLowerCase() === quality);
    btn.classList.toggle('btn-secondary', level.toLowerCase() !== quality);
  });
}

/*
 * Funktion: updateLedToggle
 * -------------------------
 * Aktualisiert den Zustand der LED-Toggle-Checkbox basierend auf dem empfangenen Zustand.
 *
 * state: Boolean (true → eingeschaltet, false → ausgeschaltet)
 */
function updateLedToggle(state) {
  document.getElementById('ledToggle').checked = state;
}

/*
 * Funktion: updateStreamURL
 * -------------------------
 * Aktualisiert die URL des MJPEG-Kamerastreams und setzt onload/onerror-Handler,
 * um den Status anzuzeigen. Ein optionaler Timeout (1s) setzt bei Nichterreichen
 * der Quelle den Stream auf 'camera_offline.jpg' und zeigt 'Stream offline'.
 *
 * ip: String, der mit "http" beginnt (z.B. "http://192.168.1.2/stream")
 */
function updateStreamURL(ip) {
  if (typeof ip !== 'string' || !ip.startsWith('http')) {
    return;
  }
  cameraIP = ip;
  const img = document.getElementById('stream');

  /* Timeout: Wenn innerhalb von 1s kein onload/OError, gilt Stream als offline */
  const failTimer = setTimeout(() => {
    document.getElementById('statusText').textContent = 'Stream offline';
    img.src = 'camera_offline.jpg';
  }, 1000);

  /* Wenn Bild geladen wird, Status "Streaming" setzen, Timeout abbrechen */
  img.onload = () => {
    clearTimeout(failTimer);
    document.getElementById('statusText').textContent = `Streaming`;
  };

  /* Bei Fehler, Timeout abbrechen und auf camera_offline.jpg zurücksetzen */
  img.onerror = () => {
    clearTimeout(failTimer);
    document.getElementById('statusText').textContent = 'Stream error';
    img.src = 'camera_offline.jpg';
  };

  /* Setze Bildquelle neu, füge Timestamp hinzu, um Caching zu vermeiden */
  img.src = `${cameraIP}?t=${Date.now()}`;
  /* IP im HUD anzeigen (<span id="camIP">) */
  document.getElementById('camIP').textContent = `URL: ${cameraIP}`;
}

/*
 * Funktion: setupStream
 * ---------------------
 * Fügt dem Container mit dem Kamerabild ('.stream-container') einen Doppelklick-Listener
 * hinzu, um Fullscreen umzuschalten. Außerdem lädt jede Sekunde das Bild neu, falls
 * cameraIP gesetzt ist und nicht 'camera_offline.jpg'.
 */
function setupStream() {
  const container = document.querySelector('.stream-container');
  const img = document.getElementById('stream');

  /* Fullscreen Toggle bei Doppelklick */
  container.addEventListener('dblclick', () => {
    if (document.fullscreenElement) {
      document.exitFullscreen();
    } else {
      container.requestFullscreen().catch(err => console.error('Fullscreen:', err));
    }
  });

  /* Periodisches Neuladen des JPG-Frames (MJPEG) alle 1s */
  setInterval(() => {
    if (cameraIP && img.src !== 'camera_offline.jpg') {
      img.src = `${cameraIP}?t=${Date.now()}`;
    }
  }, 1000);
}

/*
 * Funktion: updateSidePanel
 * -------------------------
 * Aktualisiert das seitliche Panel (linke und rechte Spalte) mit den Telemetriedaten
 * aus latestFrame (Float32Array). Füllt folgende Felder:
 *   - IMU: roll (f[0]), pitch (f[1]), yaw (f[2])
 *   - Ultraschall: raw (f[3]), filtered (f[4])
 *   - TOF (f[5])
 *   - Network (rssi: f[20]), Battery (f[21])
 *   - Drive: swc (f[6]), swd (f[7]), thr (f[8]), turn (f[9]), speed1 (f[10]), speed3 (f[11])
 *   - Encoders: enc1 (f[12]), enc3 (f[13]), rev1 (f[14]), rev3 (f[15]), dist1 (f[16]), dist3 (f[17])
 *   - Servos & LEDs: servo0 (f[18]), servo1 (f[19]), led (f[22]), camLed (f[23])
 */
function updateSidePanel() {
  if (!latestFrame) return;
  const f = latestFrame;

  /* --- IMU --- */
  document.getElementById('roll').textContent = f[0].toFixed(1);
  document.getElementById('pitch').textContent = f[1].toFixed(1);
  document.getElementById('yaw').textContent = f[2].toFixed(1);

  /* --- Ultraschall --- */
  document.getElementById('usRaw').textContent = f[3].toFixed(1);
  document.getElementById('usFilt').textContent = f[4].toFixed(1);

  /* --- Laser TOF --- */
  document.getElementById('tof').textContent = f[5];

  /* --- Netzwerk & Batterie --- */
  document.getElementById('rssi').textContent = f[20];
  document.getElementById('battery').textContent = f[21].toFixed(0);

  /* --- Drive & Encoders --- */
  document.getElementById('swc').textContent = f[6];    /* Drive Mode */
  document.getElementById('swd').textContent = f[7];    /* FPV Camera */
  document.getElementById('thr').textContent = f[8];
  document.getElementById('turn').textContent = f[9];
  document.getElementById('speed1').textContent = f[10];
  document.getElementById('speed3').textContent = f[11];

  document.getElementById('enc1').textContent = f[12];
  document.getElementById('enc3').textContent = f[13];
  document.getElementById('rev1').textContent = f[14].toFixed(2);
  document.getElementById('rev3').textContent = f[15].toFixed(2);
  document.getElementById('dist1').textContent = f[16].toFixed(2);
  document.getElementById('dist3').textContent = f[17].toFixed(2);

  /* --- Servos & LED --- */
  document.getElementById('servo0').textContent = f[18];
  document.getElementById('servo1').textContent = f[19];
  document.getElementById('led').textContent = f[22];
  document.getElementById('cameraled').textContent = f[23];
}

/*
 * Funktion: updateDateTime
 * ------------------------
 * Aktualisiert das <span> mit ID 'dateTimeDisplay' mit der aktuellen lokalen
 * Datum/Uhrzeit in Format "DD.MM.YYYY, HH:MM:SS". Wird initial aufgerufen und
 * anschließend jede Sekunde via setInterval.
 */
function updateDateTime() {
  const disp = document.getElementById('dateTimeDisplay');
  if (!disp) return;

  const now = new Date();
  const day = String(now.getDate()).padStart(2, '0');
  const month = String(now.getMonth() + 1).padStart(2, '0'); // Monate sind 0-indexiert
  const year = now.getFullYear();
  const hours = String(now.getHours()).padStart(2, '0');
  const mins = String(now.getMinutes()).padStart(2, '0');
  const secs = String(now.getSeconds()).padStart(2, '0');

  const formatted = `${day}.${month}.${year}, ${hours}:${mins}:${secs}`;
  disp.textContent = formatted;
}

/*
 * Funktion: getFormattedDateTime
 * ------------------------------
 * Gibt einen formatierten String der aktuellen Datum/Uhrzeit zurück
 * im Format "DD.MM.YYYY - HH:MM:SS". Wird für die HUD-Box 'datetime' verwendet.
 *
 * return: formatierter Datum/Uhrzeit-String
 */
function getFormattedDateTime() {
  const now = new Date();
  const day = String(now.getDate()).padStart(2, '0');
  const month = String(now.getMonth() + 1).padStart(2, '0'); // Monate 0-basiert
  const year = now.getFullYear();
  const hours = String(now.getHours()).padStart(2, '0');
  const mins = String(now.getMinutes()).padStart(2, '0');
  const secs = String(now.getSeconds()).padStart(2, '0');
  return `${day}.${month}.${year} - ${hours}:${mins}:${secs}`;
}

/* =============================================================================
 * Initialisierung bei Seitenaufruf
 * ============================================================================= */

/*
 * Event-Listener: DOMContentLoaded
 * --------------------------------
 * Sobald das DOM vollständig geladen ist, werden initialisiert:
 *   - connectWS()         → WebSocket-Verbindung starten
 *   - renderTelemetry()   → HUD- und Side-Panel-Updates starten
 *   - setupControls()     → Event-Listener für Steuerungselemente binden
 *   - setupStream()       → Fullscreen-Logik und Periodisches Neuladen des Streams
 *   - updateDateTime()    → Aktuelles Datum/Uhrzeit setzen
 *   - setInterval(updateDateTime, 1000) → Datum/Uhrzeit jede Sekunde aktualisieren
 */
window.addEventListener('DOMContentLoaded', () => {
  connectWS();
  renderTelemetry();
  setupControls();
  setupStream();
  updateDateTime();
  setInterval(updateDateTime, 1000);
});
