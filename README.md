# RS232-FWD-GPS

Sistema GNSS para Dynatest FWD con receptor ArduSimple simpleRTK3B Budget (UM980) y controlador Arduino UNO R4 WiFi.

## Objetivo

Proporcionar al Dynatest FWD una posición GNSS mejorada mediante:

- Galileo HAS (High Accuracy Service) cuando esté disponible.
- SBAS/EGNOS como respaldo.
- Promedio temporal de coordenadas durante la parada.
- Corrección de offset antena-pistón mediante IMU (BNO085).
- Presentación del estado GNSS mediante LEDs externos.
- Terminal TCP interactivo para diagnóstico y control en tiempo real.

## Arquitectura (alto nivel)

- **GNSS**: ArduSimple simpleRTK3B Budget (UM980) → Serial1 (D0/D1) @ 115200 bps
- **MCU**: Arduino UNO R4 WiFi (Renesas RA4M1)
- **IMU**: Adafruit BNO085/BNO086 → I2C (SDA/SCL) @ 100 kHz
- **Salida FWD**: Software-Serial D2 @ 38400 bps (GGA 10 Hz)
- **Ethernet**: Shield W5500 → Servidor TCP 192.168.1.122:15919
- **WiFi AP**: Diagnóstico interactivo → TCP 192.168.4.1:15920
- **LEDs**: D5 (GNSS+IMU+HAS), D6 (Movimiento)

Flujo principal:

1. Arduino UNO R4 WiFi recibe GGA/RMC/PUBX,00 del UM980 a 10 Hz por Serial1.
2. Detecta estado HAS activo mediante PUBX,00.
3. Lee yaw del BNO085 (corrección antena-pistón).
4. Detecta estado MOVING/STOPPED (máquina de estados).
5. En STOPPED promedia coordenadas (15 s a 10 Hz, media recortada + media circular).
6. Aplica offset antena-pistón solo si magnetómetro disponible.
7. Emite GGA corregida a FWD por D2 (soft-serial @ 38400 bps, 10 Hz).
8. Simultáneamente transmite por Ethernet a servidor externo.
9. WiFi AP proporciona terminal TCP para diagnóstico y cambio de variables en tiempo real.
10. LEDs indican estado GNSS, IMU, HAS y movimiento/bloqueo.

## Prioridad de solución GNSS

1. Galileo HAS (cuando disponible)
2. GPS autónomo con HAS mejorado
3. SBAS (EGNOS)
4. GPS estándar

## Estados de software

- **MOVING**: Lectura GNSS continua, sin promedio.
- **AVERAGING**: Acumulación de muestras durante 15 s (configurable por TCP).
- **LOCKED**: Posición fija, transmisión de coordenadas promediadas.

## Estructura del repositorio

- `firmware/arduino/RS232-FMW-GPS_V-2_1.ino`: **Versión actual Rev.2.1** (WiFi AP + Terminal TCP interactivo)
- `firmware/arduino/README.md`: Detalle técnico del firmware.
- `docs/functional-spec-v1.0.md`: Especificación funcional completa.
- `docs/system-architecture.md`: Detalles de arquitectura y comunicaciones.

## Versiones

### Rev.2.1 (Actual - **RECOMENDADA**)

**Cambios principales:**
- ✅ WiFi AP nativo `FWD-GPS-Diag` (sin USB, alimentación externa)
- ✅ Terminal TCP interactivo puerto 15920
- ✅ 8 comandos: `freq`, `offset`, `decl`, `speed_*`, `avg`, `status`, `help`
- ✅ Variables dinámicas configurables en tiempo real
- ✅ Diagnóstico cada 500ms por TCP
- ✅ Rev.2 completa preservada (GNSS, LEDs, BNO085, Ethernet, FWD)

**Conectividad:**
- SSID: `FWD-GPS-Diag`
- Contraseña: `12345678`
- Servidor: `192.168.4.1:15920`
- App Android: TCP Socket Client, ConnectBot, nRF Connect

### Rev.2

**Cambios principales:**
- ✅ Parser PUBX,00 para detectar HAS activo
- ✅ LEDs 4 estados (GNSS + IMU + HAS + Movimiento)
- ✅ 10 Hz GNSS entrada (UM980)
- ✅ 10 Hz salida FWD (D2 soft-serial 38400 bps)
- ✅ BNO085 corrección antena-pistón (offset 0.55m)
- ✅ Ethernet salida GCGGA simultánea

### Rev.1

**Base funcional:**
- 10 Hz GNSS entrada/salida
- Máquina de estados MOVING/AVERAGING/LOCKED
- Promedio de coordenadas 15s
- BNO085 opcional

## LEDs (Rev.2.1)

### LED1 (D5) - GNSS + IMU + HAS

| Estado | Significado |
|--------|-------------|
| **OFF** | Sin GNSS |
| **200ms parpadeo** | GNSS OK pero SIN magnetómetro (sin corrección antena) |
| **600ms parpadeo** | GNSS + IMU pero SIN HAS |
| **1000ms parpadeo** | GNSS + IMU + HAS, esperando bloqueo |
| **ON** | ✅ GNSS + IMU + HAS + Posición bloqueada |

### LED2 (D6) - Movimiento

| Estado | Significado |
|--------|-------------|
| **OFF** | En movimiento |
| **400ms parpadeo** | Promediando posición |
| **ON** | Posición bloqueada |

## Comandos TCP (Rev.2.1)

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

## Configuración UM980 (u-center)

```
UNLOG COM3
CONFIG COM3 115200
GNGGA COM3 0.1
GNRMC COM3 0.1
CONFIG NMEA PUBX ENABLE
ENABLE HAS
SAVECONFIG
```

## Roadmap

- ✅ Fase 1: Validar enlace UM980 → Arduino UNO R4 WiFi → FWD (38400, GGA 10 Hz)
- ✅ Fase 2: Implementar HAS detection (PUBX,00)
- ✅ Fase 3: Promedio de 15 s en parada + corrección antena
- ✅ Fase 4: BNO085, LEDs mejorados, diagnóstico
- ✅ Fase 5: WiFi AP + Terminal TCP interactivo (Rev.2.1)

## Próximas mejoras

- [ ] Guardado de configuración en EEPROM (persistencia tras reinicio)
- [ ] Sincronización de diagnóstico USB + WiFi TCP
- [ ] Autenticación WiFi mejorada
- [ ] Logging a microSD (opcional)
- [ ] Dashboard web (HTML embebido)

## Dependencias

- Arduino IDE 2.x+
- Librería: `Adafruit_BNO08x`
- Librería: `Ethernet` (W5500)
- Librería: `WiFiS3` (nativa en UNO R4 WiFi)

## Hardware requerido

- Arduino UNO R4 WiFi
- ArduSimple simpleRTK3B Budget (UM980)
- Adafruit BNO085 o BNO086
- Ethernet Shield 2 (W5500)
- MAX3232 o similar para RS232
- 2 LEDs + resistencias (5mm, ~220Ω)

## Autor

**BrewGold** - Desarrollo e integración

## Licencia

MIT (ver LICENSE)
