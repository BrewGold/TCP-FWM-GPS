# Firmware Arduino

## Versión final (compilada)

- `rs232_fwd_gps_final_v_0_99.ino` — **v0.9.9 MADRID FINAL**

Esta es la versión final que se ha compilado y usado en campo. Sustituye a la versión unificada anterior (`rs232_fwd_gps_unificado.ino`) y al borrador `rs232_fwd_gps_draft_v0_9.ino`, que se mantienen solo como referencia histórica.

### Hardware

- GNSS (UM980): RX=44, TX=43, BAUD=115200 (`Serial1`)
- Salida Dynatest: RX=18, TX=17, BAUD=38400 (`Serial2` → MAX3232)
- IMU (BNO085): bus `Wire` (SDA/SCL principal), I2C 100 kHz, direcciones `0x4A/0x4B` con auto-detección, reset por `A3`
- LED_RED: pin 4 (estado GNSS/HAS)
- LED_GREEN: pin 5 (estado de movimiento/lock)

### Máquina de estados

**Movimiento:**
`MOVING → AVERAGING` (velocidad < 0.20 m/s durante 2 s) `→ LOCKED` (promedio de 15 s) `→ MOVING` (si se desplaza > 1.0 m o velocidad > 0.30 m/s)

**PPP/HAS:**
`SIN_PPP → PPP_CONVERGING → PPP_ESTABLE`

### Corrección antena → pistón

- Offset: 0.55 m
- Bearing: `yaw + 270°` (antena a la derecha, pistón a la izquierda)
- Declinación magnética (Madrid): 1.0° (`yaw_geografico = yaw_magnetico - declinacion`)

---

## 1) Mapa de pines

### UART entrada GNSS (UM980)
- `GNSS_RX = 44` → ESP32 recibe desde TX del UM980
- `GNSS_TX = 43` → ESP32 transmite hacia RX del UM980
- `GNSS_BAUD = 115200`

### UART salida a Dynatest (vía MAX3232)
- `DYNATEST_TX = 17`
- `DYNATEST_RX = 18`
- `DYNATEST_BAUD = 38400`

### I2C IMU (BNO085)
- Bus recomendado: `Wire` (SDA/SCL principal del UNO R4)
- `I2C_FREQ = 100000`
- Direcciones probadas: `0x4A` y `0x4B`
- `BNO085_RST_PIN = A3` (reinicio hardware antes de `begin_I2C`)

### LEDs
- `LED_RED = 4`: estado GNSS/HAS (parpadeo en `PPP_CONVERGING`, fijo en fix/`PPP_ESTABLE`)
- `LED_GREEN = 5`: estado de movimiento (parpadeo en `AVERAGING`, fijo en `LOCKED`)

---

## 2) Parámetros de navegación

- `OFFSET_M = 0.55`: distancia antena→pistón en metros.
- `DECLINATION_OFFSET = 1.0`: declinación magnética local (Madrid), en grados.
- `STOP_THRESHOLD_MS = 2000`: tiempo bajo velocidad de entrada para pasar a `AVERAGING`.
- `AVERAGING_WINDOW_MS = 15000`: ventana de promedio en parada (15 s).
- `SPEED_ENTER_STOP = 0.20` / `SPEED_EXIT_STOP = 0.30` (m/s): histéresis de detección de parada.
- `NEW_LOCATION_DIST = 1.0`: distancia (m) para salir de `LOCKED` por desplazamiento.
- `MAX_SAMPLES = 160`: tamaño de buffer de muestras para promedio (lat/lon/alt/yaw).
- `OUTPUT_PERIOD_MS = 100`: periodo de salida (10 Hz).

---

## 3) Formato de salida NMEA

El firmware genera y envía siempre:
- **`$GPGGA,...*CS`**

El `fixQ` de salida se calcula según prioridad:
1. `LOCKED` con posición válida → `fixQ = 4`
2. `PPP_ESTABLE` (HAS) → `fixQ = 4`
3. `PPP_CONVERGING` → `fixQ = 2`
4. En otro caso, con GNSS válido → `fixQ = 1`

---

## 4) Flujo de funcionamiento

1. Lee líneas NMEA/propietarias del UM980 por `Serial1` (`$GPGGA`, `$GPRMC`, `#PPPNAVA`/HAS).
2. Valida checksum y parsea GGA (posición, altitud, satélites, fix) y RMC (velocidad, rumbo).
3. Actualiza estado PPP/HAS (`SIN_PPP` / `PPP_CONVERGING` / `PPP_ESTABLE`).
4. Lee yaw del IMU BNO085 (geomagnetic rotation vector) vía `Wire` a 100 kHz, con reintentos y detección de reset.
5. Actualiza la máquina de estados de movimiento (`MOVING` / `AVERAGING` / `LOCKED`).
6. En `AVERAGING`, acumula muestras de lat/lon/alt/yaw; al completar 15 s calcula medias recortadas (trimmed mean) y media circular del yaw, y aplica el offset antena→pistón.
7. En `LOCKED`, transmite la posición corregida; en otro caso transmite la posición instantánea (con offset si hay yaw disponible).
8. Construye la trama `$GPGGA` y la envía por `Serial2` hacia el MAX3232, a 10 Hz.
9. Actualiza LEDs de estado (`LED_RED`/`LED_GNSS`, `LED_GREEN`/movimiento).

---

## 5) Checklist rápido de validación

1. Ver logs por USB a `115200`.
2. Confirmar recepción de tramas `[GNSS] GGA:` y `[GNSS] RMC:`.
3. Confirmar inicialización del IMU (`[IMU] BNO085 initialized`) o reintentos periódicos si falla.
4. Verificar transición de estados en logs (`[STATE] MOVING -> AVERAGING -> LOCKED`).
5. Confirmar salida `$GPGGA` hacia el Dynatest y que este la acepta sin errores.
6. Validar comportamiento de LEDs: `LED_RED` (GNSS/HAS) y `LED_GREEN` (movimiento/lock).

---

## 6) Notas

- Requiere la librería `Adafruit_BNO08x`.
- Si el escáner I2C ve `0x4A/0x4B` pero `begin_I2C()` falla, revisar `RST` en A3 y evitar Qwiic/Wire1 por posibles problemas de temporización del BNO08x en I2C.
- Si el receptor Dynatest requiere otro baudrate, ajustar `DYNATEST_BAUD`.
- La declinación magnética (`DECLINATION_OFFSET`) está calibrada para Madrid; ajustar si se despliega en otra ubicación.
- Los archivos `rs232_fwd_gps_unificado.ino` y `rs232_fwd_gps_draft_v0_9.ino` se conservan como versiones anteriores/experimentales, no representan el firmware final compilado.
