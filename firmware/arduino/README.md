# Firmware Arduino

Este directorio contiene la implementación funcional del firmware para el Arduino UNO R4 WiFi utilizado como MCU del sistema GNSS FWD.

## Versión actual

- `RS232-FMW-GPS_V-3_0.ino` — **Rev.3.0**

La versión actual incluye:

- GNSS UM980 por Serial1 @ 115200 bps
- Salida FWD por D2 @ 38400 bps (soft-serial TX-only)
- BNO085 por I2C para corrección de offset antena-pistón
- Detección HAS mediante `PUBX,00`
- LEDs D5/D6 con estados GNSS + IMU + HAS + movimiento
- Ethernet W5500 a 192.168.1.122:15919
- BLE para diagnóstico / control

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

### BLE diagnóstico / control
- **Nombre anunciado/local**: `FWD-GPS`
- **Servicio**: `19B10000-E8F2-537E-4F6C-D104768A1214`
- **Comando (write)**: `19B10001-E8F2-537E-4F6C-D104768A1214`
- **Respuesta (read/notify)**: `19B10002-E8F2-537E-4F6C-D104768A1214`
- **Diagnóstico (read/notify)**: `19B10003-E8F2-537E-4F6C-D104768A1214`

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

En Rev.3.0 estos parámetros pueden modificarse en tiempo real mediante comandos BLE.

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
10. Publica diagnóstico por BLE y permite comandos interactivos.

---

## 6) Diagnóstico BLE

Rev.3.0 incorpora una interfaz BLE para diagnosticar y ajustar parámetros en campo sin depender del USB.

### Comandos disponibles

```text
freq <1-10>        Cambiar frecuencia GNSS (Hz)
speed_stop <0-1>   Umbral parada (m/s)
speed_move <0-1>   Umbral movimiento (m/s)
offset <0-2>       Offset antena-pistón (m)
decl <-180-180>    Declinación magnética (°)
avg <5-60>         Ventana promedio (segundos)
diag <1-60>        Intervalo diagnóstico (segundos)
status             Mostrar estado actual
help               Lista de comandos
```

Cada escritura contiene un comando ASCII/UTF-8. El firmware elimina espacios y CR/LF de los extremos. Las respuestas se notifican en la característica de respuesta y se imprimen también por USB Serial.

### Diagnóstico y LEDs virtuales

La característica de diagnóstico notifica cada 5 s por defecto con un texto compacto de hasta 240 bytes:

```text
LED1=BLINK_GREEN;LED2=OFF;GNSS=OK;FIX=1;SATS=25;HDOP=0.8;LAT=37.462140;LON=-6.058661;ALT=20.2;SPEED=0.02;COURSE=146.8;SOLUTION=HAS;HAS=ON;IMU=OK;YAW=1.5;STATE=MOVING;LOCK=NO;N=0;HZ=10;CFG=15/0.55/1.0
```

- `LED1`: `OFF` sin GNSS, `BLINK_RED` sin IMU, `BLINK_YELLOW` sin HAS, `ON/GREEN` con HAS y bloqueo, `BLINK_GREEN` en los demás casos.
- `LED2`: `OFF` en `MOVING`, `BLINK_YELLOW` en `AVERAGING`, `ON/GREEN` en `LOCKED`.

### Prueba con nRF Connect

1. Buscar y conectar a `FWD-GPS`.
2. Activar notificaciones en las características de respuesta y diagnóstico.
3. Escribir `status` o `help` como texto UTF-8 en la característica de comando.
4. Verificar la respuesta inmediata y las notificaciones de diagnóstico periódicas.

Para recibir una notificación completa, solicitar MTU 247 (nRF Connect lo negocia normalmente). Si un cliente conserva el MTU BLE mínimo, debe leer la característica completa mediante `read`; los campos LED se priorizan al principio del valor.

---

## 7) Checklist rápido de validación

1. Conectar Arduino por alimentación externa adecuada.
2. Confirmar que nRF Connect detecta `FWD-GPS`.
3. Conectar y suscribirse a respuesta y diagnóstico.
4. Verificar que llegan campos `GNSS`, `LAT`, `LON`, `LED1` y `LED2`.
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
- BNO085 sigue siendo provisional; Rev.3.0 no integra todavía ICM-20948.

---

## 9) Archivos relevantes

- `RS232-FMW-GPS_V-3_0.ino` — firmware actual de referencia
- `README.md` — documentación general del proyecto
- `CHANGELOG.md` — historial de versiones y novedades

---

## 10) Versiones

- **Rev.3.0**: BLE + Ethernet primario + COM2 emergencia/supervisión
- **Rev.2.1**: WiFi AP + control TCP + diagnóstico interactivo
- **Rev.2**: HAS, LEDs, 10 Hz, BNO085, Ethernet
- **Rev.1**: base funcional con promedio de coordenadas y salida FWD
