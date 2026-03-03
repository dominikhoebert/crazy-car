# crazy-car (Arduino/PlatformIO)

Firmware für ein RC-Car auf **Arduino Mega 2560** (PlatformIO), mit

- Lenkservo (Grad über `Servo::write(...)` um Neutral bei 90°)
- ESC/Throttle (PWM in Mikrosekunden über `Servo::writeMicroseconds(...)`)
- 3 IR-Distanzsensoren (Analog)
- Start/Stop-Taster (negative Logik: gedrückt == `LOW`)

## Quickstart

Voraussetzungen:

- VS Code + PlatformIO Extension **oder** PlatformIO CLI

Sicher (ohne Hardwarezugriff):

- Build: `pio run -e megaatmega2560`
- Clean: `pio run -e megaatmega2560 -t clean`
- Geräte anzeigen: `pio device list`

Hardware/USB (nur wenn du es wirklich ausführen willst):

- Upload/Flash: `pio run -e megaatmega2560 -t upload`
- Serieller Monitor (9600): `pio device monitor -b 9600`

## Sicherheit (ESC / Fahrzeug)

- In `src/main.cpp` existiert eine ESC-Kalibrier-/Setup-Routine `setupESCPWM()`.
- Diese Routine sendet potenziell **gefährliche** PWM-Werte (inkl. starkem Vorwärts/Rückwärts).
- Sie wird nur ausgeführt, wenn `#define MOTOR_SETUP 1` gesetzt ist.

Wenn du `MOTOR_SETUP` aktivierst:

- Fahrzeug **physisch sichern** (Räder frei / Auto aufbocken), Abstand halten.
- Erst dann flashen und beobachten.

## Konfiguration (Tuning via `#define`)

Alle zentralen Tuning-Parameter sind oben in `src/main.cpp` als `#define` gesetzt:

- `TARGET_DISTANCE` – Zielabstand (Regelung basiert aktuell auf linkem Sensor)
- `SPEED` – ESC-PWM in µs (z.B. 1500…2000 vorwärts, 1480 Bremse)
- `MOTOR_SETUP` – 0/1, führt optional ESC-Setup aus (siehe Sicherheit)

Lenk-Regelung:

- Neutral/Limit/Deadband: `STEERING_NEUTRAL_DEG`, `STEERING_MIN_DEG`, `STEERING_MAX_DEG`, `STEERING_DEADBAND`
- Proportionalfaktor: `STEERING_KP_DIV`

Hindernis-Override:

- Wenn der mittlere Sensor „nah“ meldet, wird ein fixer Ausweichwinkel gesetzt:
  `MIDDLE_OBSTACLE_THRESHOLD`, `MIDDLE_OBSTACLE_STEER_DEG`

## Hardware / Pins

Hinweis: Kommentare können vom tatsächlichen `#define` abweichen – **Quelle der Wahrheit sind die Defines** in `src/main.cpp`.

### Servos

- Lenkservo Signal: `PIN_STEERING_SERVO` = D5
- ESC/Speed Signal: `PIN_SPEED_SERVO` = D2

### Analoge Eingänge

- Batterie-Messung: `VBAT` = A0
- Distanz links: `LEFTSENSOR` = A1
- Distanz rechts: `RIGTHSENSOR` = A2
- Distanz mitte: `MIDDLESENSOR` = A3

### Taster (negative Logik)

- Start: `STARTBUTTON` = D12 (schwarze Taste)
- Stop: `STOPBUTTON` = D13 (rote Taste)

## Laufzeitverhalten (kurz)

- `setup()` hängt beide Servos an die Pins, setzt Lenkung auf 90° und startet Serial mit 9600 Baud.
- `loop()` liest Sensoren über `analogRead(...)`.
- Stop-Taste (`LOW`) setzt `runMode = 0`, bremst (1480 µs) und zentriert die Lenkung.
- Start-Taste (`LOW`) setzt `runMode = 1`.
- Wenn `runMode == 1`: setzt Geschwindigkeit auf `SPEED` und berechnet den Lenkwinkel.

## Abhängigkeiten

- Servo Library: `arduino-libraries/Servo` (siehe `platformio.ini`)

## Troubleshooting

- Wenn Servos/ESC nicht reagieren: Prüfe GND-Gemeinschaft, Pinbelegung, und ob der ESC ein neutrales Signal (typ. ~1500 µs) erwartet.
- Bei unerwarteter Bewegung: `MOTOR_SETUP` muss **0** sein, und `SPEED` sollte konservativ gewählt werden.

## TODO

- [x] Mittig Fahren
- [x] Batteriespannung messen und ausgeben
- [x] PID Speed
- [x] reverse
- [ ] State Machine (stop/schnellstart/regel/kurve-links/kurve-rechts)
