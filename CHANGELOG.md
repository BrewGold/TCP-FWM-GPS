# Changelog

Todos los cambios notables en este proyecto están documentados aquí.

El formato se basa en [Keep a Changelog](https://keepachangelog.com/es-ES/1.0.0/),
y este proyecto adhiere a [Semantic Versioning](https://semver.org/lang/es/).

## [Unreleased]

### ✨ Agregado

- **Comando BLE `freq`** (Rev.2.5)
  - `freq` muestra la frecuencia actual de salida GGA/COM2
  - `freq <1-10>` cambia la frecuencia de salida en Hz (reutiliza `OUTPUT_PERIOD_MS_VAR`)
  - Valor fuera de rango o no entero responde `ERROR: usa freq 1..10 (Hz)`
  - No se guarda en EEPROM: tras reiniciar vuelve a 10 Hz

## [2.5.0] - 2026-10-02

### ✨ Agregado

- **Precisión horizontal estimada según HDOP y tipo de fix**
  - Cálculo: `UERE típico × HDOP`
  - UERE: GPS ~3.00 m · DGPS ~1.00 m · RTK ~0.02 m · HAS ~0.20 m
  - Nueva línea `Precision est` en el comando BLE `status`
  - Nueva línea `HDOP=... ACC=... m` en el diagnóstico periódico BLE `[DIAG]`
  - Nueva función `estimatedAccuracyMeters()` y variable `currentHdop`

### 🔄 Modificado

- **Detección HAS por fix 5 en el GGA de entrada**
  - El tipo de solución (GPS/DGPS/RTK/HAS) se determina desde el campo
    de calidad del GGA: 1=GPS, 2=DGPS, 4=RTK, 5=HAS
  - `hasActive` solo es true con fix 5
- **LED1 (D5)**: solo queda fijo con HAS activo (fix 5);
  con fix 1/2/4 parpadea a 600 ms (antes quedaba fijo con fix 1 en LOCKED)

### 🗑️ Eliminado

- Parser `$PUBX,00` (sentencia u-blox que el UM980 no emite)

### 📝 Documentación

- README principal y README de firmware actualizados a Rev.2.5

## [2.4.0] - 2026-10-01

### ✨ Agregado

- **Diagnóstico Bluetooth Low Energy (BLE)**
  - Nombre: `FWD-GPS-Diag`, servicio compatible Nordic UART Service
  - Comandos: `status`, `imu`, `com2`, `help`, `magcal start/stop/reset`, `yawoff <-180..180>`
  - Diagnóstico periódico `[DIAG]` cada 5 s
  - Chunks TX de 20 bytes; acepta comandos con o sin CR/LF (compatible MIT App Inventor)
- **IMU ICM-20948** (reemplaza al BNO085)
  - Fusión de yaw: giróscopo + magnetómetro con compensación de inclinación
  - Filtro complementario (`YAW_GYRO_WEIGHT = 0.98`)
  - Reintento automático de inicialización y detección en 0x69/0x68
- **Calibración del magnetómetro por BLE**
  - Offsets hard-iron y `yawoff` guardados en EEPROM (persisten tras reinicio)
  - Validación: mínimo 200 muestras y span ≥ 10 µT en X e Y

### 🔄 Modificado

- COM2 reescrito como UART software directa sin buffer (bit-banging con `noInterrupts()` por byte)
- Salida NMEA renombrada a `$GCGGA`

### 🗑️ Eliminado

- WiFi AP y terminal TCP de diagnóstico (sustituidos por BLE)

## [2.2.0] - 2026-09-29

### ✨ Agregado

- Comando `diag <1-60>` — intervalo del diagnóstico TCP (s)

### 🔄 Modificado

- Diagnóstico TCP independiente de la frecuencia de salida GGA
  - `freq` controla solo la salida FWD/GGA (1–10 Hz)
  - `diag` controla el intervalo del diagnóstico TCP (por defecto 5 s)
- `status` y `help` responden de forma inmediata

## [2.1.0] - 2026-09-28

### ✨ Agregado

- **WiFi AP nativo** para diagnóstico sin USB
  - SSID: `FWD-GPS-Diag`, contraseña: `12345678`
  - Servidor TCP puerto 15920 (192.168.4.1:15920)
  
- **Terminal TCP interactivo** con 8 comandos
  - `freq <1-10>` — Cambiar frecuencia GNSS (Hz)
  - `speed_stop <0-1>` — Umbral parada (m/s)
  - `speed_move <0-1>` — Umbral movimiento (m/s)
  - `offset <0-2>` — Offset antena-pistón (m)
  - `decl <-180-180>` — Declinación magnética (°)
  - `avg <5-60>` — Ventana promedio (segundos)
  - `status` — Mostrar estado actual de todas las variables
  - `help` — Lista de comandos

- **Variables dinámicas configurables en tiempo real**
  - Sin necesidad de reprogramación
  - Validación de rangos en Arduino
  - Confirmación de cambios en TCP

- **Diagnóstico continuo por TCP**
  - Líneas cada 500ms (ajustable con `freq`)
  - Formato: `[time] GNSS:OK LAT:... LON:... ALT:... SAT:... HDOP:... SPEED:... STATE:... HAS:... IMU:... LOCK:...`

- **Compatibilidad con Android**
  - Apps TCP: TCP Socket Client, ConnectBot, nRF Connect
  - Sin dependencia de USB
  - Alimentación externa estable

### 🔄 Modificado

- **Actualización de README principal** para documentar Rev.2.1
  - Arquitectura WiFi AP
  - Comandos TCP
  - LEDs 4 estados
  - Tabla de versiones

- **Estructura de configuración**
  - Variables globales configurables: `*_VAR`
  - Valores por defecto en constantes: `*_DEFAULT`
  - Reset a valores por defecto al arranque

### 🐛 Corregido

- Mejor gestión de conexiones TCP (desconexiones limpias)
- Buffer de comandos redimensionado (128 bytes)
- Parsing de comandos mejorado (tolower + validación)

### 📝 Documentación

- Nuevo archivo `CHANGELOG.md`
- README actualizado con versiones, LEDs, comandos TCP
- Ejemplo de sesión TCP en README

### 🔧 Técnico

- Librerías: `WiFiS3.h` (nativa UNO R4)
- Parser TCP con `sscanf` y validación de rangos
- Variables dinámicas sin EEPROM (reset en reinicio)
- Mantiene 100% compatibilidad con Rev.2

## [2.0.0] - 2026-09-27

### ✨ Agregado

- **Parser PUBX,00** para detectar HAS activo
  - Lectura de tipo de solución (GPS, RTK, HAS)
  - Variable global `hasActive`

- **LEDs 4 estados** (Rev.2)
  - LED1 (D5): GNSS + IMU + HAS (ON/OFF/parpadeo 200ms/600ms/1000ms)
  - LED2 (D6): Movimiento (ON/OFF/parpadeo 400ms)

- **Garantías de corrección de antena**
  - Verificación BNO085 disponible antes de aplicar offset
  - Solo yaw si magnetómetro activo
  - Advertencias en logs si magnetómetro no disponible

- **Configuración UM980 documentada**
  - Comentarios con comandos u-center
  - Habilitación de HAS, PUBX,00, GGA/RMC a 10 Hz

### 🔄 Modificado

- Soft-serial COM2 con compensación absoluta (micros())
- Temporización de salida GGA ajustada a 100ms (10 Hz)
- Parsers GNSS mejorados (GGA, RMC, PUBX)

### 🐛 Corregido

- Evitar bloqueos en bit-banging COM2 (SoftwareSerial incompatible con RA4M1)
- Buffer Serial1 overflow en 115200 bps (solved con lectura más frecuente)

## [1.0.0] - 2026-09-20

### ✨ Agregado

- **Base funcional inicial**
  - 10 Hz GNSS entrada (UM980 COM3)
  - 10 Hz salida FWD (D2 soft-serial 38400 bps)
  - Máquina de estados: MOVING → AVERAGING → LOCKED
  - Promedio de coordenadas 15 segundos (media recortada)
  - BNO085 opcional para yaw

- **Salida múltiple**
  - FWD por soft-serial D2 (38400 bps)
  - Ethernet por W5500 a 192.168.1.122:15919
  - Diagnóstico por Serial USB (115200 bps)

- **LEDs básicos**
  - LED1 (D5): estado GNSS
  - LED2 (D6): estado movimiento

- **Offset antena-pistón**
  - Distancia: 0.55 m
  - Bearing: yaw + 270°
  - Declinación: 1.0° (Madrid)

---

## Notas de compatibilidad

- **Rev.2.5** es retrocompatible con Rev.2.4 (mismo hardware; solo cambia la detección HAS, el LED1 y el contenido BLE)
- **Rev.2.4** cambia el IMU (BNO085 → ICM-20948) y elimina el WiFi AP/TCP en favor de BLE
- La calibración del magnetómetro y `yawoff` se guardan en EEPROM desde Rev.2.4
- El WiFi AP (Rev.2.1–2.2) fue eliminado en Rev.2.4; Ethernet sigue funcionando en paralelo

## Dependencias por versión

| Versión | Arduino | IMU                  | Ethernet | WiFiS3 | ArduinoBLE | EEPROM |
|---------|---------|----------------------|----------|--------|------------|--------|
| 2.5     | UNO R4  | SparkFun ICM-20948   | ✅       | ❌     | ✅         | ✅     |
| 2.4     | UNO R4  | SparkFun ICM-20948   | ✅       | ❌     | ✅         | ✅     |
| 2.2     | UNO R4  | Adafruit_BNO08x      | ✅       | ✅     | ❌         | ❌     |
| 2.1     | UNO R4  | Adafruit_BNO08x      | ✅       | ✅     | ❌         | ❌     |
| 2.0     | UNO R4  | Adafruit_BNO08x      | ✅       | ❌     | ❌         | ❌     |
| 1.0     | UNO R4  | Adafruit_BNO08x      | ✅       | ❌     | ❌         | ❌     |
