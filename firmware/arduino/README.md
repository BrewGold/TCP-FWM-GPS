# Firmware Arduino

Este directorio contiene la implementación funcional del firmware para el Arduino UNO R4 WiFi utilizado como MCU del sistema GNSS FWD.

## Versiones de firmware

- `RS232-FWS-GPS_V2-7` — **Rev.2.7 BLE**
- `RS232-FMW-GPS_V-2_1.ino` — **Rev.2.1 WiFi/TCP** (documentada abajo)

La Rev.2.7 añade parser `$GNGST`, precisión BLE, validación FIX_IN/HAS para `LOCKED`,
calibración magnética de fábrica de solo lectura y lógica para dos LEDs.

La siguiente descripción corresponde a la Rev.2.1:

- GNSS UM980 por Serial1 @ 115200 bps
- Salida FWD por D2 @ 38400 bps (soft-serial TX-only)
- BNO085 por I2C para corrección de offset antena-pistón
- Detección HAS mediante `PUBX,00`
- LEDs D5/D6 con estados GNSS + IMU + HAS + movimiento
- Ethernet W5500 a 192.168.1.122:15919
- WiFi AP para diagnóstico / control por TCP

---

## 1) Hardware

### UART entrada GNSS (UM980)
- **Puerto**: `Serial1`
- **Pins**: D0 = RX, D1 = TX
- **Baud**: `115200`
- **Formato**: GGA + RMC + PUBX,00

### UART salida FWD (Dynatest)
- **Puerto**: software TX-only en D2
- **Baud**: `38400`
- **Formato**: GGA únicamente
- **Periodo**: `100 ms` (10 Hz)

### IMU / Magnetómetro
- **Sensor**: BNO085 / BNO086
- **Bus**: I2C
- **Pins**: SDA/SCL (pines del Arduino UNO R4 WiFi)
- **Frecuencia**: `100 kHz`
- **Uso**: yaw para corrección de offset antena-pistón

### LEDs
- **LED1**: D5 — estado GNSS + IMU + HAS
- **LED2**: D6 — estado movimiento / bloqueado

### Ethernet
- **Shield**: W5500
- **Destino**: `192.168.1.122:15919`

### WiFi diagnóstico / control
- **AP SSID**: `FWD-GPS-Diag`
- **Contraseña**: `12345678`
- **TCP**: `192.168.4.1:15920`

---

## 2) Máquina de estados

### Movimiento

- `MOVING → AVERAGING`
  - se entra cuando velocidad < umbral durante 2 s
- `AVERAGING → LOCKED`
  - cuando se completa la ventana de promedio
- `LOCKED → MOVING`
  - si la velocidad aumenta o se aleja +1.0 m del punto bloqueado

### HAS

- `SIN_HAS`
- `HAS_ACTIVO`
- `HAS_CONVERGIENDO` (estado visual por LED)

### Corrección antena → pistón

- Offset por defecto: `0.55 m`
- Dirección: `yaw + 270°`
- Declinación magnética por defecto: `1.0°`
- Solo se aplica si el magnetómetro está operativo (`BNO085` disponible)

---

## 3) Parámetros de navegación

Los valores por defecto son:

- `OUTPUT_PERIOD_MS = 100` → 10 Hz
- `OFFSET_M = 0.55`
- `DECLINATION_DEG = 1.0`
- `STOP_CONFIRMATION_MS = 2000`
- `AVERAGING_WINDOW_MS = 15000` → 15 s
- `SPEED_ENTER_STOP = 0.20 m/s`
- `SPEED_EXIT_STOP = 0.30 m/s`
- `RELOCK_DISTANCE_M = 1.0`
- `MAX_SAMPLES = 160`

En Rev.2.1 estos parámetros pueden modificarse en tiempo real por TCP mediante comandos del terminal.

---

## 4) Formato de salida NMEA

El firmware genera y transmite siempre una trama tipo:

- `$GPGGA,...*CS`

Con fix quality según el estado:

1. `LOCKED` + posición válida → `fixQ = 4`
2. HAS activo → `fixQ = 2`
3. GNSS válido y no bloqueado → `fixQ = 1`

---

## 5) Flujo de funcionamiento

1. Lee líneas NMEA del UM980 por `Serial1` (`$GPGGA`, `$GPRMC`, `$PUBX,00`).
2. Valida checksum y parsea posición, velocidad, altitud y tipo de solución.
3. Revisa HAS y su estado activo mediante `PUBX,00`.
4. Lee yaw del BNO085 vía I2C.
5. Actualiza la máquina de estados (`MOVING`, `AVERAGING`, `LOCKED`).
6. En `AVERAGING` acumula lat/lon/alt/yaw y calcula promedio recortado.
7. En `LOCKED`, transmite la posición corregida; si no hay bloqueo, transmite instantánea con offset si hay yaw.
8. Emite GGA por D2 hacia el Dynatest a 10 Hz.
9. Envía la misma trama por Ethernet.
10. Publica diagnóstico por WiFi TCP y permite comandos interactivos.

---

## 6) Diagnóstico TCP / WiFi

La Rev.2.1 incorpora una interfaz TCP por WiFi para diagnosticar y ajustar parámetros en campo sin depender del USB.

### Comandos disponibles

```
freq <1-10>        Cambiar frecuencia GNSS (Hz)
speed_stop <0-1>   Umbral parada (m/s)
speed_move <0-1>   Umbral movimiento (m/s)
offset <0-2>       Offset antena-pistón (m)
decl <-180-180>    Declinación magnética (°)
avg <5-60>         Ventana promedio (segundos)
status             Mostrar estado actual
help               Lista de comandos
```

### Ejemplo

```
> help
=== COMANDOS DISPONIBLES ===
freq <1-10>        - Frecuencia en Hz
speed_stop <0-1>   - Umbral parada (m/s)
speed_move <0-1>   - Umbral movimiento (m/s)
offset <0-2>       - Offset antena (m)
decl <-180-180>    - Declinación magnética (°)
avg <5-60>         - Ventana promedio (s)
status             - Mostrar estado actual
help               - Este mensaje
===========================

> freq 5
OK: Frecuencia cambiada a 5 Hz (200 ms)

> status
=== ESTADO ACTUAL ===
Frecuencia: 5 Hz (200 ms)
Offset antena: 0.55 m
Estado GNSS: OK
HAS activo: SI
Movimiento: LOCKED
...
```

---

## 7) Checklist rápido de validación

1. Conectar Arduino por alimentación externa adecuada.
2. Confirmar que el WiFi AP `FWD-GPS-Diag` aparece.
3. Conectar desde Android al TCP `192.168.4.1:15920`.
4. Verificar que llegan líneas tipo:
   - `[time] GNSS:OK LAT:... LON:... ALT:...`
5. Confirmar `PUBX,00` y `HAS activo` en logs.
6. Confirmar `GGA` en salida FWD por D2 a 10 Hz.
7. Validar LEDs en estados reales del sistema.
8. Comprobar `status` y `help` funcionales.

---

## 8) Notas técnicas

- `SoftwareSerial.h` no es compatible con Arduino UNO R4 WiFi; la salida FWD se implementa por bit-banging TX-only en D2.
- El UM980 debe configurarse con:

```
UNLOG COM3
CONFIG COM3 115200
GNGGA COM3 0.1
GNRMC COM3 0.1
CONFIG NMEA PUBX ENABLE
ENABLE HAS
SAVECONFIG
```

- La corrección de antena se aplica solo si el BNO085 está disponible.
- El diagnostic TCP se usa para pruebas de campo y ajuste sin reprogramar.

---

## 9) Archivos relevantes

- `RS232-FMW-GPS_V-2_1.ino` — firmware actual de referencia
- `README.md` — documentación general del proyecto
- `CHANGELOG.md` — historial de versiones y novedades

---

## 10) Versiones

- **Rev.2.1**: WiFi AP + control TCP + diagnóstico interactivo
- **Rev.2**: HAS, LEDs, 10 Hz, BNO085, Ethernet
- **Rev.1**: base funcional con promedio de coordenadas y salida FWD
