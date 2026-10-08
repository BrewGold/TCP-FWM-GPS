# Firmware Arduino

Este directorio contiene la implementación funcional del firmware para el Arduino UNO R4 WiFi utilizado como MCU del sistema GNSS FWD.

## Versión actual

- `RS232-RWM-GPS_V2-5.ino` — **Rev.2.5**

La versión actual incluye:

- GNSS UM980 por Serial1 @ 115200 bps
- Salida FWD (COM2) por D2 @ 38400 bps (UART software TX-only)
- ICM-20948 por I2C para corrección de offset antena-pistón
- Detección HAS por fix 5 en el GGA de entrada
- Precisión horizontal estimada según HDOP y tipo de fix
- LEDs D5/D6 con estados GNSS + IMU + HAS + movimiento
- Ethernet W5500 a 192.168.1.122:15919
- Diagnóstico y control por Bluetooth Low Energy (Nordic UART Service)
- Calibración del magnetómetro por BLE con guardado en EEPROM

### Rev.2.6 BLE (`RS232-FWS-GPS_V2-6`)

Basada en `RS232-FWS-GPS_V2-5`, con estos cambios:

- El HDOP del último GGA se guarda en `inputHdop`.
- `FIX_IN=5` (PPP Float / RTK Float, E6-HAS en UM980) se interpreta como `HAS=ON` y `SOL=HAS`; cualquier otro valor → `HAS=OFF`. Si llega `PUBX,00` reciente, su navStat tiene prioridad.
- Lectura de `$GPGST` / `$GNGST`: error horizontal `H_ERR = sqrt(σlat² + σlon²)` en metros (`N/A` si no llega GST).
- Diagnóstico BLE: `GNSS=OK FIX_IN=5 HAS=ON SOL=HAS SAT=34 HDOP=0.7 H_ERR=0.05`

---

## 1) Hardware

### UART entrada GNSS (UM980)
- **Puerto**: `Serial1`
- **Pins**: D0 = RX, D1 = TX
- **Baud**: `115200`
- **Formato**: GGA + RMC

### UART salida FWD (Dynatest)
- **Puerto**: UART software TX-only en D2 (bit-banging con `noInterrupts()`)
- **Baud**: `38400`
- **Formato**: `$GCGGA` únicamente
- **Periodo**: `100 ms` (10 Hz)

### IMU / Magnetómetro
- **Sensor**: ICM-20948 (SparkFun)
- **Bus**: I2C @ 100 kHz
- **Direcciones**: 0x69 (primaria) / 0x68 (secundaria)
- **Uso**: yaw fusionado (giróscopo + magnetómetro con compensación de inclinación) para corrección de offset antena-pistón
- **Calibración**: offsets hard-iron guardados en EEPROM, gestionados por BLE

### LEDs
- **LED1**: D5 — estado GNSS + IMU + HAS
- **LED2**: D6 — estado movimiento / bloqueado

### Ethernet
- **Shield**: W5500 (CS en D10, SD CS en D4 deshabilitado)
- **Destino**: `192.168.1.122:15919` (DHCP)

### Bluetooth Low Energy
- **Nombre**: `FWD-GPS-Diag`
- **Servicio**: Nordic UART Service
  - Servicio: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
  - RX (comandos): `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
  - TX (notificaciones): `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
- Chunks de 20 bytes; acepta comandos con o sin CR/LF (compatible con MIT App Inventor)

---

## 2) Máquina de estados

### Movimiento

- `MOVING → AVERAGING`
  - velocidad < 0.20 m/s mantenida 2 s
- `AVERAGING → LOCKED`
  - al completar la ventana de promedio (15 s), media recortada 5% + media circular del yaw
- `LOCKED → MOVING`
  - velocidad > 0.30 m/s, o alejamiento > 1.0 m del punto bloqueado

### Detección de solución (Rev.2.5)

El tipo de solución se determina por el campo de calidad del GGA de entrada:

| Fix GGA | Solución | HAS activo |
|---|---|---|
| 1 | GPS autónomo | NO |
| 2 | DGPS | NO |
| 4 | RTK | NO |
| 5 | HAS (PPP/Float) | **SÍ** |

### Corrección antena → pistón

- Offset por defecto: `0.55 m`
- Dirección: `yaw + 270°`
- Declinación magnética por defecto: `1.0°`
- Solo se aplica si el ICM-20948 está operativo y hay yaw válido

---

## 3) Parámetros de navegación

Valores por defecto:

- `OUTPUT_PERIOD_MS = 100` → 10 Hz
- `OFFSET_M = 0.55`
- `DECLINATION_DEG = 1.0`
- `STOP_CONFIRMATION_MS = 2000`
- `AVERAGING_WINDOW_MS = 15000` → 15 s
- `SPEED_ENTER_STOP = 0.20 m/s`
- `SPEED_EXIT_STOP = 0.30 m/s`
- `RELOCK_DISTANCE_M = 1.0`
- `MAX_SAMPLES = 160`
- `YAW_GYRO_WEIGHT = 0.98` (filtro complementario)

---

## 4) Formato de salida NMEA

El firmware genera y transmite siempre una trama tipo:

- `$GCGGA,...*CS`

Con fix quality de salida según el estado:

1. `LOCKED` + posición válida → `fixQ = 4`
2. HAS activo → `fixQ = 2`
3. GNSS válido y no bloqueado → `fixQ = 1`

---

## 5) Flujo de funcionamiento

1. Lee líneas NMEA del UM980 por `Serial1` (`GGA`, `RMC`; prefijos GP/GN/GC).
2. Valida checksum y parsea posición, velocidad, altitud, HDOP y fix de entrada.
3. Determina la solución (GPS/DGPS/RTK/HAS) desde el fix del GGA.
4. Lee el ICM-20948 y calcula yaw fusionado.
5. Actualiza la máquina de estados (`MOVING`, `AVERAGING`, `LOCKED`).
6. En `AVERAGING` acumula lat/lon/alt/yaw y calcula promedio recortado.
7. En `LOCKED` transmite la posición corregida; si no hay bloqueo, transmite instantánea con offset si hay yaw.
8. Emite `$GCGGA` por D2 hacia el Dynatest a 10 Hz.
9. Envía la misma trama por Ethernet.
10. Publica diagnóstico por BLE cada 5 s y atiende comandos interactivos.

---

## 6) Diagnóstico y comandos BLE

### Comandos disponibles

```
status             Estado general completo (incluye precisión estimada)
imu                Datos del ICM-20948 y calibración
com2               Estado y contadores COM2
magcal start       Inicia calibración del magnetómetro
magcal stop        Finaliza, valida y guarda en EEPROM
magcal reset       Borra la calibración
yawoff <grados>    Ajuste montaje yaw (-180..180)
help               Lista de comandos
```

### Calibración del magnetómetro

1. Enviar `magcal start`.
2. Girar el equipo lentamente en todos los ejes (mínimo 200 muestras, span ≥ 10 µT en X e Y).
3. Enviar `magcal stop` → guarda offsets en EEPROM.
4. Ajustar `yawoff` comparando el yaw mostrado con el rumbo real.

### Precisión estimada

Se reporta en `status` (`Precision est`) y en `[DIAG]` (`ACC`), calculada como `UERE × HDOP`:

- GPS: 3.00 m · DGPS: 1.00 m · RTK: 0.02 m · HAS: 0.20 m

### Ejemplo de diagnóstico periódico

```text
[DIAG]
GNSS=OK FIX_IN=5 HAS=ON SOL=HAS SAT=22
HDOP=0.6 ACC=0.12 m
STATE=LOCKED LOCK=YES SPD=0.030 m/s
IMU=OK YAW=181.2 MAGCAL=YES
RAW=40.12345678,-3.45678901
OUT=40.12345950,-3.45679120 FIX_OUT=4
COM2 frames=1250 bytes=103750
```

---

## 7) Checklist rápido de validación

1. Conectar Arduino con alimentación externa adecuada.
2. Confirmar que el dispositivo BLE `FWD-GPS-Diag` aparece al escanear.
3. Conectar desde Android (Serial Bluetooth Terminal / nRF Connect / app propia) y activar notificaciones en TX.
4. Enviar `status` y verificar GNSS, fix de entrada, HAS y precisión estimada.
5. Confirmar LED1 fijo solo con fix 5 (HAS) y parpadeo con fix 1.
6. Confirmar `$GCGGA` en salida FWD por D2 a 10 Hz.
7. Validar calibración con `magcal start/stop` e `imu`.
8. Comprobar la conexión Ethernet al servidor externo.

---

## 8) Notas técnicas

- `SoftwareSerial.h` no es compatible con Arduino UNO R4 WiFi; la salida FWD se implementa por bit-banging TX-only en D2 con interrupciones desactivadas por byte.
- El UM980 debe configurarse con:

```
UNLOG COM3
CONFIG COM3 115200
GNGGA COM3 0.1
GNRMC COM3 0.1
ENABLE HAS
SAVECONFIG
```

- La corrección de antena se aplica solo si el ICM-20948 está disponible y el yaw es válido.
- La calibración del magnetómetro y el `yawoff` se guardan en EEPROM y sobreviven reinicios.
- El WiFi Server de revisiones anteriores fue eliminado en Rev.2.4.

---

## 9) Archivos relevantes

- `RS232-RWM-GPS_V2-5.ino` — firmware actual de referencia
- `RS232-RWM-GPS_V2-4.ino` — revisión anterior (BLE, ICM-20948)
- `RS232-FMW-GPS_V-2_2.ino` / `V-2_1` / `V-2_0` / `V-1_0` — revisiones históricas
- `../../README.md` — documentación general del proyecto
- `../../CHANGELOG.md` — historial de versiones

---

## 10) Versiones

- **Rev.2.5**: HAS por fix 5 en GGA, LED1 fijo solo con HAS, precisión estimada por HDOP en BLE
- **Rev.2.4**: BLE (Nordic UART), ICM-20948, calibración magnetómetro, COM2 UART software, sin WiFi
- **Rev.2.2**: separación `freq` / `diag` en terminal TCP
- **Rev.2.1**: WiFi AP + control TCP interactivo
- **Rev.2**: HAS (`PUBX,00`), LEDs, 10 Hz, BNO085, Ethernet
- **Rev.1**: base funcional con promedio de coordenadas y salida FWD
