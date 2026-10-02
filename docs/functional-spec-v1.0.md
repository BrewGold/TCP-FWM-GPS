# Especificación funcional

> **Rev.2.5** — actualizado 2026-10-02. La v1.0 original (ESP32-S3 + NTRIP) se conserva al final como apéndice histórico.

## Sistema GNSS para Dynatest FWD con simpleRTK3B Budget (UM980)

## Objetivo

Proporcionar al Dynatest FWD una posición GNSS mejorada mediante:

- Galileo HAS (High Accuracy Service) cuando esté disponible (fix 5 en GGA).
- Promedio temporal de coordenadas durante la parada (media recortada).
- Corrección de offset antena-pistón mediante yaw del IMU.
- Presentación del estado GNSS mediante LEDs externos.
- Diagnóstico y control por Bluetooth Low Energy (BLE).

## Arquitectura Hardware

### Receptor GNSS

- ArduSimple simpleRTK3B Budget (UM980)
- COM1 → UPrecise (configuración)
- COM3 → Comunicación principal con el Arduino (115200 bps, GGA+RMC a 10 Hz)
- HAS habilitado (`ENABLE HAS`)

### Controlador

- Arduino UNO R4 WiFi (Renesas RA4M1)

Funciones:

- Recepción y parsing GNSS (GGA/RMC, checksum NMEA)
- Detección del tipo de solución por el fix del GGA (1=GPS, 2=DGPS, 4=RTK, 5=HAS)
- Detección de parada y máquina de estados MOVING/AVERAGING/LOCKED
- Promedio de coordenadas (media recortada 5%, media circular del yaw)
- Corrección de offset antena-pistón (0.55 m, yaw+270°)
- Generación `$GCGGA` a 10 Hz
- Control de LEDs
- Diagnóstico BLE y envío por Ethernet

### Sensor de orientación

- SparkFun ICM-20948 (I²C, 0x69/0x68)
- Yaw fusionado: magnetómetro con compensación de inclinación + giróscopo (filtro complementario 0.98)
- Calibración hard-iron y offset de montaje guardados en EEPROM, gestionados por BLE
- Uso: dirección de la corrección antena-pistón

### Comunicación Dynatest

- UART software TX-only en D2 @ 38400 bps (bit-banging; SoftwareSerial no es compatible con RA4M1)
- Conversión RS232 con MAX3232

### Ethernet

- Shield W5500 (CS D10) → servidor TCP `192.168.1.122:15919` (misma trama GGA)

### Diagnóstico BLE

- Nombre: `FWD-GPS-Diag`, servicio Nordic UART Service
- Comandos: `status`, `imu`, `com2`, `magcal start/stop/reset`, `yawoff`, `help`
- Diagnóstico periódico cada 5 s con precisión estimada (UERE × HDOP)

## Comunicaciones

| Enlace | Puerto | Velocidad | Datos |
|---|---|---|---|
| UM980 → Arduino | Serial1 (D0/D1) | 115200 | GGA + RMC (10 Hz) |
| Arduino → Dynatest | D2 (UART SW) + MAX3232 | 38400 | `$GCGGA` 10 Hz |
| Arduino ↔ ICM-20948 | I²C | 100 kHz | ACC/MAG/GYR/TEMP |
| Arduino → Servidor | W5500 Ethernet | TCP | `$GCGGA` |
| Arduino ↔ Móvil | BLE (NUS) | — | diagnóstico/comandos |

## Posicionamiento

### Detección de solución

Según el campo de calidad del GGA de entrada:

| Fix | Solución | HAS activo | Precisión típica (UERE) |
|---|---|---|---|
| 1 | GPS autónomo | NO | ~3.00 m |
| 2 | DGPS | NO | ~1.00 m |
| 4 | RTK | NO | ~0.02 m |
| 5 | HAS (PPP/Float) | **SÍ** | ~0.20 m |

Precisión estimada reportada por BLE: `UERE × HDOP`.

### Detección de parada

- Velocidad < 0.20 m/s mantenida 2 s → entra en AVERAGING
- Velocidad > 0.30 m/s → vuelve a MOVING (histéresis)
- Alejamiento > 1.0 m del punto bloqueado → vuelve a MOVING

### Promedio GNSS

- Ventana: 15 s (hasta 160 muestras a 10 Hz)
- Media recortada 5% para lat/lon/alt
- Media circular para el yaw

### Coordenada enviada

- En LOCKED: posición promediada + offset antena-pistón con yaw promediado (fix salida = 4)
- En MOVING/AVERAGING: posición instantánea + offset con yaw actual (fix salida = 2 con HAS, 1 sin HAS)

## Indicadores externos

### LED1 (D5) — GNSS + IMU + HAS

- OFF → sin GNSS válido
- Parpadeo 200 ms → GNSS OK, IMU no disponible
- Parpadeo 600 ms → GNSS + IMU, sin HAS (fix 1/2/4)
- ON fijo → HAS activo (fix 5)

### LED2 (D6) — Movimiento

- OFF → en movimiento
- Parpadeo 400 ms → promediando
- ON fijo → posición bloqueada

## Cableado IMU (RJ45)

- Pin 1: SDA · Pin 2: GND · Pin 3: SCL · Pin 4: +3V3
- Pin 5: +3V3 · Pin 6: GND · Pin 7: LED1 · Pin 8: LED2

## Fases del proyecto

- ✅ Fase 1: validación UM980 → Arduino → FWD (38400, GGA 10 Hz)
- ✅ Fase 2: detección HAS
- ✅ Fase 3: promedio 15 s en parada + corrección antena
- ✅ Fase 4: LEDs, Ethernet y diagnóstico
- ✅ Fase 5: diagnóstico BLE con calibración IMU y precisión estimada (Rev.2.4–2.5)

---

## Apéndice: especificación original v1.0 (histórica, no vigente)

La concepción inicial del sistema difiere de la implementación actual:

- Controlador previsto: **ESP32-S3** (formato UNO) → implementado finalmente con **Arduino UNO R4 WiFi**
- Cliente **NTRIP/RTCM para RTK FIX** → no implementado; se usa **Galileo HAS** como solución de precisión
- IMU previsto: **BNO085/BNO086** (experimental) → sustituido por **ICM-20948** con calibración propia
- Prioridad original: RTK FIX > HAS > SBAS > autónomo
- LEDs originales: LED_POWER + LED_GNSS con códigos de destellos por tipo de solución
- Detección de parada original: velocidad < 0.2 km/h o desplazamiento < 10 cm durante 2 s
