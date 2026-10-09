# RS232-FWD-GPS

Sistema GNSS para Dynatest FWD con receptor ArduSimple simpleRTK3B Budget (UM980) y controlador Arduino UNO R4 WiFi.

## Objetivo

Proporcionar al Dynatest FWD una posición GNSS mejorada mediante:

- Galileo HAS (High Accuracy Service) cuando esté disponible (fix 5 en GGA).
- Promedio temporal de coordenadas durante la parada.
- Corrección de offset antena-pistón mediante IMU (ICM-20948).
- Presentación del estado GNSS mediante LEDs externos.
- Diagnóstico y control por TCP.

## Arquitectura (alto nivel)

- **GNSS**: ArduSimple simpleRTK3B Budget (UM980) → Serial1 (D0/D1) @ 115200 bps
- **MCU**: Arduino UNO R4 WiFi (Renesas RA4M1)
- **IMU**: SparkFun ICM-20948 → I2C (0x69/0x68)
- **Salida FWD (COM2)**: UART software TX-only en D2 @ 38400 bps ($GCGGA a 10 Hz)
- **Ethernet**: Shield W5500 → servidor TCP NMEA en puerto 15919 y servidor de diagnóstico en puerto 15920
- **LEDs**: D5 (GNSS+IMU+HAS), D6 (Movimiento)

Flujo principal:

1. Arduino recibe GGA/RMC del UM980 por Serial1.
2. Detecta el tipo de solución desde el campo de calidad del GGA (fix 5 = HAS activo).
3. Lee yaw del ICM-20948 (fusión magnetómetro + giróscopo, compensación de inclinación).
4. Detecta estado MOVING/AVERAGING/LOCKED mediante máquina de estados.
5. En parada promedia coordenadas (15 s por defecto, media recortada 5%).
6. Aplica offset antena-pistón (0.55 m, dirección yaw+270°).
7. Emite $GCGGA corregida al FWD por D2 @ 38400 bps y a los clientes conectados al servidor TCP.
8. El servidor TCP de diagnóstico ofrece comandos y un bloque `[DIAG]` periódico.
9. LEDs reflejan GNSS, IMU, HAS y movimiento/bloqueo.

## Estados de software

- **MOVING**: lectura GNSS continua.
- **AVERAGING**: acumulación de muestras durante la ventana de promedio.
- **LOCKED**: posición fija y de salida estable (fix de salida = 4).

## Estructura del repositorio

- `firmware/arduino/RS232-RWM-GPS_V2-6.ino`: versión actual de firmware.
- `firmware/arduino/README.md`: detalle técnico del firmware.
- `docs/functional-spec-v1.0.md`: especificación funcional completa.
- `docs/system-architecture.md`: detalle de arquitectura y comunicaciones.
- `CHANGELOG.md`: historial de versiones.

## Versiones

### Rev.2.6 (Actual)

- Doble servidor TCP Ethernet: salida NMEA GCGGA en puerto 15919 y diagnóstico en puerto 15920
- Sin BLE ni cliente TCP saliente a un servidor externo
- Comandos TCP: `status`, `imu`, `com2`, `help`, `magcal start|stop|reset`, `yawoff`, `freq`, `diag on|off`
- `freq <1..10>` controla la salida GGA/COM2; `diag on/off` controla el bloque periódico cada 5 s

| Puerto | Servicio |
|---:|---|
| 15919 | GCGGA corregida para clientes TCP entrantes |
| 15920 | Diagnóstico interactivo TCP |

Conectarse a `<IP-del-W5500>:15920` con un cliente TCP y enviar comandos terminados en CR/LF.
`help` lista los comandos disponibles:

| Comando | Función |
|---|---|
| `status` | Estado general, incluye precisión estimada `UERE × HDOP` |
| `imu` | Datos del ICM-20948 y calibración |
| `com2` | Estado y contadores de COM2 |
| `help` | Lista de comandos |
| `magcal start/stop/reset` | Controla la calibración del magnetómetro en EEPROM |
| `yawoff <-180..180>` | Ajuste de montaje del yaw |
| `freq <1..10>` | Frecuencia de salida GGA/COM2 en Hz |
| `diag on/off` | Activa o desactiva el bloque `[DIAG]` periódico |

El diagnóstico periódico incluye la precisión estimada `UERE × HDOP`:

```text
TCP client: <IP-del-W5500>:15920
> status
[respuesta de estado completa]

[DIAG]
GNSS=OK FIX_IN=5 HAS=ON SOL=HAS SAT=22
HDOP=0.6 ACC=0.12 m
STATE=LOCKED LOCK=YES SPD=0.030 m/s
IMU=OK YAW=181.2 MAGCAL=YES
RAW=40.12345678,-3.45678901
OUT=40.12345950,-3.45679120 FIX_OUT=4
COM2 frames=1250 bytes=103750
```

### Rev.2.5 (Histórica)

**Cambios principales:**
- ✅ Detección HAS desde el campo de calidad del GGA de entrada (fix 5 = HAS activo)
- ✅ LED1 solo queda fijo con HAS activo; con fix 1 parpadea (nunca fijo)
- ✅ Precisión horizontal estimada según HDOP y tipo de fix (UERE × HDOP) en BLE `status` y diagnóstico periódico
- ✅ Eliminado el parser `$PUBX,00` (el UM980 no lo emite)

### Rev.2.4

- Diagnóstico BLE (Nordic UART Service, `FWD-GPS-Diag`)
- IMU ICM-20948 por I2C con fusión yaw (giróscopo + magnetómetro)
- Calibración del magnetómetro por BLE con guardado en EEPROM
- COM2 por UART software directa en D2 @ 38400
- WiFi Server eliminado

### Rev.2.2

- Diagnóstico TCP independiente de la frecuencia de salida GGA (`freq` / `diag`)

### Rev.2.1

- WiFi AP nativo + terminal TCP interactivo con variables dinámicas

### Rev.2

- Detección HAS mediante `PUBX,00`, LEDs, salida 10 Hz, Ethernet, BNO085

### Rev.1

- Base funcional: máquina de estados, promedio de coordenadas, salida FWD

## LEDs (Rev.2.6; comportamiento conservado de Rev.2.5)

### LED1 (D5) - GNSS + IMU + HAS

| Estado | Significado |
|--------|-------------|
| **OFF** | Sin GNSS válido |
| **200 ms parpadeo** | GNSS OK pero IMU no disponible |
| **600 ms parpadeo** | GNSS + IMU pero sin HAS (fix 1/2/4) |
| **ON** | HAS activo (fix 5 en GGA de entrada) |

### LED2 (D6) - Movimiento

| Estado | Significado |
|--------|-------------|
| **OFF** | En movimiento |
| **400 ms parpadeo** | Promediando posición |
| **ON** | Posición bloqueada |

## Diagnóstico BLE (Rev.2.4/2.5, histórico)

- **Nombre**: `FWD-GPS-Diag`
- **Servicio**: Nordic UART Service (`6E400001-B5A3-F393-E0A9-E50E24DCCA9E`)
- **RX (escribir comandos)**: `6E400002-...`
- **TX (notificaciones)**: `6E400003-...`
- Compatible con apps tipo Serial Bluetooth Terminal, nRF Connect y MIT App Inventor (extensión BluetoothLE).

**Comandos BLE históricos:**

| Comando | Función |
|---|---|
| `status` | Estado general completo (incluye precisión estimada) |
| `imu` | Datos del ICM-20948 y calibración |
| `com2` | Estado y contadores de la salida COM2 |
| `magcal start` | Inicia calibración del magnetómetro |
| `magcal stop` | Finaliza y guarda calibración en EEPROM |
| `magcal reset` | Borra la calibración |
| `yawoff <-180..180>` | Ajuste de montaje del yaw |
| `help` | Ayuda |

Además, cada 5 s se envía un bloque `[DIAG]` con GNSS, fix, HAS, HDOP, precisión estimada, estado, IMU y contadores COM2.

La aplicación Android `firmware/arduino/FWD_GPS_Diag/` se conserva como herramienta histórica para Rev.2.4/2.5; Rev.2.6 usa diagnóstico TCP por el puerto 15920.

## Precisión estimada

La precisión horizontal aproximada se calcula como `UERE típico × HDOP`:

| Tipo de solución | UERE |
|---|---|
| GPS autónomo (fix 1) | ~3.00 m |
| DGPS (fix 2) | ~1.00 m |
| RTK (fix 4) | ~0.02 m |
| HAS (fix 5) | ~0.20 m |

## Configuración UM980

```text
UNLOG COM3
CONFIG COM3 115200
GNGGA COM3 0.1
GNRMC COM3 0.1
ENABLE HAS
SAVECONFIG
```

## Dependencias

- Arduino IDE 2.x+
- Librería: `SparkFun ICM-20948 Arduino Library`
- Librería: `Ethernet` (W5500)
- Librería: `EEPROM` (nativa)

## Hardware requerido

- Arduino UNO R4 WiFi
- ArduSimple simpleRTK3B Budget (UM980)
- SparkFun ICM-20948
- W5500 Ethernet shield
- MAX3232 o equivalente para RS232
- 2 LEDs + resistencias (220 Ω)

## Autor

**BrewGold** — desarrollo e integración

## Licencia

MIT (ver LICENSE)
