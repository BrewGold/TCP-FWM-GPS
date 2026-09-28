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
- **IMU**: Adafruit BNO085/BNO086 → I2C (SDA/SCL)
- **Salida FWD**: Software-Serial D2 @ 38400 bps (GGA 10 Hz)
- **Ethernet**: Shield W5500 → servidor TCP 192.168.1.122:15919
- **WiFi AP**: diagnóstico y control por TCP 192.168.4.1:15920
- **LEDs**: D5 (GNSS+IMU+HAS), D6 (Movimiento)

Flujo principal:

1. Arduino UNO R4 WiFi recibe GGA/RMC/PUBX,00 del UM980 por Serial1.
2. Detecta estado HAS activo mediante `PUBX,00`.
3. Lee yaw del BNO085 para corrección de offset antena-pistón.
4. Detecta estado MOVING/STOPPED mediante máquina de estados.
5. En STOPPED promedia coordenadas (15 s por defecto).
6. Aplica offset antena-pistón si el magnetómetro está operativo.
7. Emite GGA corregida a FWD por D2 @ 38400 bps.
8. Envía la misma GGA por Ethernet a un servidor externo.
9. WiFi AP ofrece terminal TCP para diagnóstico y control en tiempo real.
10. LEDs reflejan GNSS, IMU, HAS y movimiento/bloqueo.

## Prioridad de solución GNSS

1. Galileo HAS (cuando esté disponible)
2. GPS autónomo con HAS mejorado
3. SBAS (EGNOS)
4. GPS estándar

## Estados de software

- **MOVING**: lectura GNSS continua.
- **AVERAGING**: acumulación de muestras durante la ventana de promedio.
- **LOCKED**: posición fija y de salida estable.

## Estructura del repositorio

- `firmware/arduino/RS232-FMW-GPS_V-2_2.ino`: versión actual de firmware.
- `firmware/arduino/README.md`: detalle técnico del firmware.
- `docs/functional-spec-v1.0.md`: especificación funcional completa.
- `docs/system-architecture.md`: detalle de arquitectura y comunicaciones.

## Versiones

### Rev.2.2 (Actual)

**Cambios principales:**
- ✅ Diagnóstico TCP independiente de la frecuencia de salida GGA
- ✅ `freq` controla solo la salida FWD/GGA (1–10 Hz)
- ✅ `diag` controla el intervalo del diagnóstico TCP (1–60 s)
- ✅ Diagnóstico por TCP cada 5 s por defecto
- ✅ `status` y `help` responden de forma inmediata
- ✅ `OUTPUT` y `DIAG` están separadas conceptualmente

**Terminal TCP:**
- SSID: `FWD-GPS-Diag`
- Contraseña: `12345678`
- IP del AP: `192.168.4.1:15920`

**Comandos Rev.2.2:**
- `freq <1-10>` — salida GGA/FWD (Hz)
- `speed_stop <0-1>` — umbral parada (m/s)
- `speed_move <0-1>` — umbral movimiento (m/s)
- `offset <0-2>` — offset antena-pistón (m)
- `decl <-180-180>` — declinación magnética (°)
- `avg <5-60>` — ventana promedio (s)
- `diag <1-60>` — intervalo diagnóstico TCP (s)
- `status` — estado actual inmediato
- `help` — ayuda del terminal

### Rev.2.1

**Cambios principales:**
- WiFi AP nativo `FWD-GPS-Diag`
- Terminal TCP interactivo
- Variables dinámicas y validación de rangos
- Diagnóstico y control por TCP

### Rev.2

**Cambios principales:**
- Detección HAS mediante `PUBX,00`
- LEDs con semaforización GNSS + IMU + HAS + movimiento
- Salida 10 Hz a FWD
- Ethernet a 192.168.1.122:15919
- Corrección antena-pistón con BNO085

### Rev.1

**Base funcional:**
- 10 Hz GNSS entrada/salida
- Máquina de estados MOVING/AVERAGING/LOCKED
- Promedio de coordenadas 15 s
- BNO085 opcional

## LEDs (Rev.2.2)

### LED1 (D5) - GNSS + IMU + HAS

| Estado | Significado |
|--------|-------------|
| **OFF** | Sin GNSS |
| **200ms parpadeo** | GNSS OK pero sin magnetómetro |
| **600ms parpadeo** | GNSS + IMU pero sin HAS |
| **1000ms parpadeo** | GNSS + IMU + HAS, esperando bloqueo |
| **ON** | GNSS + IMU + HAS + posición bloqueada |

### LED2 (D6) - Movimiento

| Estado | Significado |
|--------|-------------|
| **OFF** | En movimiento |
| **400ms parpadeo** | Promediando posición |
| **ON** | Posición bloqueada |

## Diagnóstico TCP (Rev.2.2)

La salida GGA/FWD y el diagnóstico TCP están separados:

- `freq` controla la salida FWD/GGA
- `diag` controla la periodicidad del diagnóstico TCP
- por defecto, el diagnóstico se envía cada 5 s
- `status` siempre responde inmediatamente

Ejemplo de diagnóstico TCP:

```text
[DIAG] uptime=125s
  GNSS=OK fix=4 sats=18 hdop=0.8
  LAT=40.123456 LON=-3.456789 ALT=650.4
  SPEED=0.03 m/s COURSE=181.2 deg
  SOLUTION=HAS HAS=ON
  IMU=OK YAW=OK
  STATE=LOCKED LOCK=YES samples=150
  OUTPUT=10 Hz avg=15s offset=0.55 m decl=1.0 deg
```

## Configuración UM980 (u-center)

```text
UNLOG COM3
CONFIG COM3 115200
GNGGA COM3 0.1
GNRMC COM3 0.1
CONFIG NMEA PUBX ENABLE
ENABLE HAS
SAVECONFIG
```

## Roadmap

- ✅ Fase 1: validación UM980 → Arduino UNO R4 WiFi → FWD
- ✅ Fase 2: HAS detection (PUBX,00)
- ✅ Fase 3: promedio de 15 s en parada + corrección antena
- ✅ Fase 4: LEDs, Ethernet y diagnóstico WiFi
- ✅ Fase 5: terminal TCP con separación `freq` / `diag` (Rev.2.2)

## Dependencias

- Arduino IDE 2.x+
- Librería: `Adafruit_BNO08x`
- Librería: `Ethernet` (W5500)
- Librería: `WiFiS3` (nativa del UNO R4 WiFi)

## Hardware requerido

- Arduino UNO R4 WiFi
- ArduSimple simpleRTK3B Budget (UM980)
- Adafruit BNO085/BNO086
- W5500 Ethernet shield
- MAX3232 o equivalente para RS232
- 2 LEDs + resistencias (220 Ω)

## Autor

**BrewGold** — desarrollo e integración

## Licencia

MIT (ver LICENSE)
