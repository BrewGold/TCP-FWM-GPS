# Firmware Arduino

Este directorio contiene la implementación funcional del firmware para el Arduino UNO R4 WiFi utilizado como MCU del sistema GNSS FWD.

## Versión actual

- `RS232-RWM-GPS_V2-6.ino` — **Rev.2.6**
- `RS232-RWM-GPS_V2-5.ino` — Rev.2.5 histórica

La versión actual incluye:

- GNSS UM980 por Serial1 @ 115200 bps
- Salida FWD (COM2) por D2 @ 38400 bps (UART software TX-only)
- ICM-20948 por I2C para corrección de offset antena-pistón
- Detección HAS por fix 5 en el GGA de entrada
- Precisión horizontal estimada según HDOP y tipo de fix
- LEDs D5/D6 con estados GNSS + IMU + HAS + movimiento
- Servidores TCP Ethernet W5500: NMEA en puerto 15919 y diagnóstico en puerto 15920
- Sin ArduinoBLE ni cliente TCP saliente
- Calibración del magnetómetro por TCP con guardado en EEPROM

### Dependencias (Rev.2.6)

- `Ethernet` (W5500)
- `SparkFun ICM-20948 Arduino Library`
- `EEPROM` (nativa)

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
- **Calibración**: offsets hard-iron guardados en EEPROM, gestionados por TCP

### LEDs
- **LED1**: D5 — estado GNSS + IMU + HAS
- **LED2**: D6 — estado movimiento / bloqueado

### Ethernet
- **Shield**: W5500 (CS en D10, SD CS en D4 deshabilitado)
- **IP**: DHCP; los clientes se conectan a la IP local del W5500

| Puerto TCP | Servicio |
|---:|---|
| 15919 | Servidor NMEA `$GCGGA` (hasta cuatro clientes entrantes) |
| 15920 | Servidor interactivo de diagnóstico (una conexión activa) |

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
9. Envía la misma trama a los clientes TCP conectados al puerto 15919.
10. Publica diagnóstico TCP cada 5 s (si está activado) y atiende comandos en el puerto 15920.

---

## 6) Diagnóstico y comandos TCP

### Comandos disponibles

| Comando | Función |
|---|---|
| `status` | Estado general completo, incluye precisión estimada |
| `imu` | Datos del ICM-20948 y calibración |
| `com2` | Estado y contadores COM2 |
| `help` | Lista de comandos |
| `magcal start` | Inicia calibración del magnetómetro |
| `magcal stop` | Finaliza, valida y guarda en EEPROM |
| `magcal reset` | Borra la calibración |
| `yawoff <-180..180>` | Ajuste de montaje yaw |
| `freq <1..10>` | Frecuencia GGA/COM2 (Hz) |
| `diag on/off` | Activa o desactiva el bloque periódico `[DIAG]` |

Conectar un cliente TCP a `<IP-del-W5500>:15920` y enviar cada comando terminado en CR/LF. El puerto 15919 distribuye GGA a los clientes entrantes; no existe conexión saliente a `192.168.1.122:15919`.

### Calibración del magnetómetro

1. Enviar `magcal start`.
2. Girar el equipo lentamente en todos los ejes (mínimo 200 muestras, span ≥ 10 µT en X e Y).
3. Enviar `magcal stop` → guarda offsets en EEPROM.
4. Ajustar `yawoff` comparando el yaw mostrado con el rumbo real.

### Precisión estimada

Se reporta en `status` (`Precision est`) y en `[DIAG]` (`ACC`), calculada como `UERE × HDOP`:

- GPS: 3.00 m · DGPS: 1.00 m · RTK: 0.02 m · HAS: 0.20 m

### Ejemplo de sesión TCP en puerto 15920

```text
> status
========== STATUS ==========
GNSS: OK
GGA entrada fix: 5
HAS: ACTIVO
HDOP: 0.6
Precision est: 0.12 m
Ethernet TCP: NMEA 15919 / diagnostico 15920 (OK)

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
2. Confirmar que el W5500 obtiene una IP DHCP.
3. Conectar un cliente TCP a `<IP-del-W5500>:15920` y ejecutar `status`.
4. Conectar un cliente TCP a `<IP-del-W5500>:15919` y verificar `$GCGGA`.
5. Confirmar LED1 fijo solo con fix 5 (HAS) y parpadeo con fix 1.
6. Confirmar `$GCGGA` en salida FWD por D2 a 10 Hz.
7. Validar calibración con `magcal start/stop` e `imu`.

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
- La app Android `FWD_GPS_Diag/` se conserva como referencia histórica de Rev.2.4/2.5; Rev.2.6 la sustituye por diagnóstico TCP en 15920.

---

## 9) Archivos relevantes

- `RS232-RWM-GPS_V2-6.ino` — firmware actual
- `RS232-RWM-GPS_V2-5.ino` — revisión anterior de referencia
- `RS232-RWM-GPS_V2-4.ino` — revisión anterior (BLE, ICM-20948)
- `RS232-FMW-GPS_V-2_2.ino` / `V-2_1` / `V-2_0` / `V-1_0` — revisiones históricas
- `../../README.md` — documentación general del proyecto
- `../../CHANGELOG.md` — historial de versiones

---

## 10) Versiones

- **Rev.2.6**: TCP limpia sin BLE; servidores NMEA 15919 y diagnóstico 15920
- **Rev.2.5**: HAS por fix 5 en GGA, LED1 fijo solo con HAS, precisión estimada por HDOP
- **Rev.2.4**: BLE (Nordic UART), ICM-20948, calibración magnetómetro, COM2 UART software, sin WiFi
- **Rev.2.2**: separación `freq` / `diag` en terminal TCP
- **Rev.2.1**: WiFi AP + control TCP interactivo
- **Rev.2**: HAS (`PUBX,00`), LEDs, 10 Hz, BNO085, Ethernet
- **Rev.1**: base funcional con promedio de coordenadas y salida FWD
