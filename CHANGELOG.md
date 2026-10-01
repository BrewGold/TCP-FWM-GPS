# Changelog

Todos los cambios notables en este proyecto están documentados aquí.

El formato se basa en [Keep a Changelog](https://keepachangelog.com/es-ES/1.0.0/),
y este proyecto adhiere a [Semantic Versioning](https://semver.org/lang/es/).

## [2.7.0] - 2026-10-01

### ✨ Agregado

- Parser `$GNGST` y diagnóstico BLE de precisión horizontal y vertical.
- Validación de FIX_IN/HAS antes de entrar o permanecer en `LOCKED`.
- Consulta de calibración magnética de fábrica mediante `magcal status`.

### 🔄 Modificado

- Estados de dos LEDs y apagado de ambos ante fallos GNSS, IMU o calibración.
- Declinación fija de -3.5°, offset de montaje de 0° y bienvenida BLE Rev.2.7.

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

- **Rev.2.1** es **retrocompatible** con Rev.2 (sin cambios en hardware/protocolo de entrada)
- Las variables dinámicas no se guardan en EEPROM (reset al reinicio)
- El WiFi AP es complementario; Ethernet sigue funcionando en paralelo

## Dependencias por versión

| Versión | Arduino | Adafruit_BNO08x | Ethernet | WiFiS3 |
|---------|---------|-----------------|----------|--------|
| 2.1     | UNO R4  | ✅              | ✅       | ✅     |
| 2.0     | UNO R4  | ✅              | ✅       | ❌     |
| 1.0     | UNO R4  | ✅              | ✅       | ❌     |
