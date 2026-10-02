# Arquitectura del sistema

> **Rev.2.5** — actualizado 2026-10-02. Sustituye a la arquitectura original basada en ESP32-S3 + NTRIP.

## 1. Resumen

El sistema desacopla adquisición GNSS, lógica de estado, corrección IMU y salida serial hacia Dynatest FWD.

Bloques principales:

1. Receptor GNSS (UM980 / simpleRTK3B Budget) con Galileo HAS
2. Controlador Arduino UNO R4 WiFi (Renesas RA4M1)
3. Interfaz RS232 (MAX3232) hacia Dynatest
4. IMU ICM-20948 (yaw para corrección antena-pistón)
5. Ethernet W5500 (reenvío de tramas a servidor)
6. Diagnóstico BLE (Nordic UART Service)
7. LEDs externos de estado

## 2. Diagrama lógico

```text
Galileo HAS (E6-B, satélite)
        │
        ▼
   UM980 (simpleRTK3B Budget)
        │
        └─ NMEA GGA/RMC 10 Hz ──► Serial1 (D0/D1 @ 115200)
                                        │
                              Arduino UNO R4 WiFi
                                        │
        ┌─ I2C (100 kHz) ──► ICM-20948 (yaw, calibración EEPROM)
        ├─ GPIO ──► LED1 (D5, GNSS+IMU+HAS) / LED2 (D6, movimiento)
        ├─ BLE (NUS "FWD-GPS-Diag") ──► móvil (diagnóstico/comandos)
        ├─ SPI ──► W5500 ──► TCP 192.168.1.122:15919 ($GCGGA)
        └─ D2 UART SW (38400) ─► MAX3232 ─► Dynatest FWD ($GCGGA 10 Hz)
```

## 3. Interfaz de datos

### 3.1 GNSS ↔ Arduino

- Enlace: Serial1 (D0/D1)
- Baudrate: 115200
- Entrada: NMEA GGA + RMC (prefijos GP/GN/GC), validación de checksum
- El tipo de solución se deriva del campo de calidad del GGA (1=GPS, 2=DGPS, 4=RTK, 5=HAS)

### 3.2 Arduino → Dynatest

- Enlace: UART software TX-only en D2 + MAX3232 (SoftwareSerial no compatible con RA4M1)
- Baudrate: 38400
- Trama: `$GCGGA` a 10 Hz
- Fix de salida: 4 (LOCKED), 2 (HAS activo), 1 (GNSS válido)

### 3.3 Arduino ↔ ICM-20948

- Enlace: I²C (100 kHz), direcciones 0x69/0x68
- Yaw fusionado: magnetómetro con compensación de inclinación + giróscopo Z (filtro complementario 0.98)
- Calibración hard-iron + offset de montaje en EEPROM
- Impacto en coordenada: dirección del offset antena-pistón (0.55 m, yaw+270°)

### 3.4 Arduino → Ethernet

- W5500 por SPI (CS D10), DHCP
- Reenvío de cada `$GCGGA` a `192.168.1.122:15919` (TCP)

### 3.5 Arduino ↔ BLE

- Nordic UART Service, nombre `FWD-GPS-Diag`
- RX: comandos (`status`, `imu`, `com2`, `magcal start/stop/reset`, `yawoff`, `help`)
- TX: respuestas + diagnóstico periódico `[DIAG]` cada 5 s (incluye HDOP y precisión estimada)
- Chunks de 20 bytes; comandos aceptados con o sin CR/LF (compatible MIT App Inventor)

## 4. Máquina de estados

### MOVING

- Ingesta GNSS continua; salida GGA 10 Hz con posición instantánea + offset (si hay yaw).
- Si velocidad < 0.20 m/s durante 2 s → AVERAGING.

### AVERAGING

- Acumula lat/lon/alt por cada GGA nuevo (hasta 160 muestras) y yaw válido.
- Ventana de 15 s → media recortada 5% (posición) y media circular (yaw) → LOCKED.
- Si velocidad > 0.30 m/s → vuelve a MOVING.

### LOCKED

- Publica la posición promediada + offset con fix de salida 4.
- Sale a MOVING si velocidad > 0.30 m/s o alejamiento > 1.0 m del punto bloqueado.

## 5. Selección de calidad GNSS

Derivada del fix del GGA de entrada:

| Fix entrada | Solución | HAS | UERE típico |
|---|---|---|---|
| 1 | GPS autónomo | NO | ~3.00 m |
| 2 | DGPS | NO | ~1.00 m |
| 4 | RTK | NO | ~0.02 m |
| 5 | HAS | **SÍ** | ~0.20 m |

Precisión estimada (BLE): `UERE × HDOP`.

## 6. Detección de parada

- Entrada: velocidad RMC < 0.20 m/s mantenida 2 s.
- Salida: velocidad > 0.30 m/s (histéresis) o distancia > 1.0 m al punto bloqueado.
- Frescura de datos: RMC y GGA caducan a los 2 s (invalida velocidad/posición).

## 7. LEDs de estado

### LED1 (D5) — GNSS + IMU + HAS

- OFF: sin GNSS válido
- Parpadeo 200 ms: IMU no disponible
- Parpadeo 600 ms: GNSS + IMU sin HAS (fix 1/2/4)
- ON fijo: HAS activo (fix 5)

### LED2 (D6) — Movimiento

- OFF: en movimiento
- Parpadeo 400 ms: promediando
- ON fijo: posición bloqueada

## 8. Cableado RJ45 (módulo remoto)

- Pin 1: SDA · Pin 2: GND · Pin 3: SCL · Pin 4: +3V3
- Pin 5: +3V3 · Pin 6: GND · Pin 7: LED1 · Pin 8: LED2

## 9. Configuración UM980

```text
UNLOG COM3
CONFIG COM3 115200
GNGGA COM3 0.1
GNRMC COM3 0.1
ENABLE HAS
SAVECONFIG
```

## 10. Historial de arquitectura

| Revisión | Controlador | IMU | Corrección | Diagnóstico |
|---|---|---|---|---|
| Diseño v1.0 | ESP32-S3 (previsto) | BNO085 (experimental) | NTRIP/RTK (previsto) | — |
| Rev.1–2.2 | Arduino UNO R4 WiFi | BNO085 | HAS (`PUBX,00`) | USB / WiFi AP + TCP |
| Rev.2.4 | Arduino UNO R4 WiFi | ICM-20948 | HAS (`PUBX,00`) | BLE (NUS) |
| **Rev.2.5** | Arduino UNO R4 WiFi | ICM-20948 | **HAS por fix 5 en GGA** | BLE + precisión HDOP |
