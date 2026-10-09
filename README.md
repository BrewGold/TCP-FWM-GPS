# RS232-FWD-GPS

Sistema GNSS para Dynatest FWD con receptor ArduSimple simpleRTK3B Budget (UM980) y controlador Adafruit Metro M4.

## Objetivo

Proporcionar al Dynatest FWD una posición GNSS mejorada mediante:

- Galileo HAS (High Accuracy Service) cuando esté disponible (fix 5 en GGA).
- Promedio temporal de coordenadas durante la parada.
- Corrección de offset antena-pistón mediante IMU (ICM-20948).
- Presentación del estado GNSS mediante LEDs externos.
- Diagnóstico y control por TCP Ethernet (sin BLE en Rev.3.0).

## Arquitectura (alto nivel)

- **GNSS**: ArduSimple simpleRTK3B Budget (UM980) → Serial1 (D0/D1) @ 115200 bps
- **MCU**: Adafruit Metro M4 (SAMD51, Cortex-M4 @ 120 MHz)
- **IMU**: SparkFun ICM-20948 → I2C (0x69/0x68)
- **Salida FWD (COM2)**: UART hardware SERCOM4 TX-only en D7 @ 38400 bps, 8N1 ($GCGGA a 10 Hz)
- **Ethernet**: Shield W5500 → servidores TCP NMEA (15919) y diagnóstico (15920)
  con DHCP o IP fija de respaldo `192.168.1.22`
- **Diagnóstico**: comandos TCP por Ethernet (sin BLE en Rev.3.0)
- **LEDs**: D5 (GNSS+IMU+HAS), D6 (Movimiento)

Flujo principal:

1. Arduino recibe GGA/RMC del UM980 por Serial1.
2. Detecta el tipo de solución desde el campo de calidad del GGA (fix 5 = HAS activo).
3. Lee yaw del ICM-20948 (fusión magnetómetro + giróscopo, compensación de inclinación).
4. Detecta estado MOVING/AVERAGING/LOCKED mediante máquina de estados.
5. En parada promedia coordenadas (15 s por defecto, media recortada 5%).
6. Aplica offset antena-pistón (0.55 m, dirección yaw+270°).
7. Emite $GCGGA corregida al FWD por D7 @ 38400 bps.
8. Sirve la trama por Ethernet TCP a los clientes conectados.
9. El servidor TCP ofrece diagnóstico periódico y comandos interactivos.
10. LEDs reflejan GNSS, IMU, HAS y movimiento/bloqueo.

## Estados de software

- **MOVING**: lectura GNSS continua.
- **AVERAGING**: acumulación de muestras durante la ventana de promedio.
- **LOCKED**: posición fija y de salida estable (fix de salida = 4).

## Estructura del repositorio

- `firmware/arduino/RS232-RWM-GPS_V3-0.ino`: versión actual de firmware (Metro M4).
- `firmware/arduino/RS232-RWM-GPS_V2-8.ino`: última base UNO R4, conservada.
- `firmware/arduino/README.md`: detalle técnico del firmware.
- `docs/functional-spec-v1.0.md`: especificación funcional completa.
- `docs/system-architecture.md`: detalle de arquitectura y comunicaciones.
- `CHANGELOG.md`: historial de versiones.

## Versiones

### Rev.3.0 (Actual): migración a Metro M4

- Se sustituye UNO R4 WiFi por **Adafruit Metro M4 (SAMD51)** para evitar el
  defecto del core Renesas descrito en [issue #543](https://github.com/arduino/ArduinoCore-renesas/issues/543):
  SDA/SCL de I2C SCI configurados como CMOS en vez de open-drain. El issue
  sigue abierto; su informe original se refiere a XIAO RA4M1/SCI, no demuestra
  por sí solo un fallo determinista de todas las UNO R4. El cambio de orden
  de Rev.2.8 no resolvió el IMU; la detección en la nueva placa debe probarse.
- **Único cambio de señal: COM2 TX de D2 a D7**, hacia la entrada TTL del
  MAX3232/FWD. D2 = PB17/PAD1 no admite TX USART en SAMD51; D3 usa SERCOM5,
  reservado para Wire. D7 = PB12/SERCOM4/PAD0 sí permite TX y queda libre.
  `Serial2` es un `Uart` creado por
  este firmware, no un puerto predefinido. Salida 38400/8N1 por hardware,
  sin bit-banging ni GPT; no se conecta RX.
- GNSS `Serial1` D0/D1, LEDs D5/D6, IMU en los conectores **SDA/SCL**
  y W5500 por **SPI ICSP**, CS D10/SD CS D4, mantienen su cableado.
  En Metro M4, A4/A5 no son SDA/SCL y D11–D13 no son el SPI ICSP.
  Sus GPIO son **3.3 V y no toleran 5 V**: verificar niveles y pull-ups
  del GNSS, IMU, shield y transceptor antes de apilar/conectar.
- `Wire` permanece a 100 kHz, sin workaround de modo de pin Renesas;
  requiere pull-ups adecuados a 3.3 V. Ethernet conserva la API SPI genérica,
  DHCP y respaldo `192.168.1.22/24` (gateway/DNS `192.168.1.1`).
- Calibración y `yawoff` pasan de EEPROM a flash interna con **FlashStorage
  con soporte SAMD51**; persisten tras reiniciar, no tras cargar otro sketch.
  No se importan datos de la UNO R4. Recalibrar tras migrar.
- Sin cambios en GNSS/HAS, MOVING/AVERAGING/LOCKED, offset, `$GCGGA`, puertos
  TCP 15919/15920 ni comandos `status`, `imu`, `com2`, `help`,
  `magcal start/stop/reset`, `yawoff`, `freq`, `diag on/off`.
- Instalación, APIs verificadas y pruebas de banco pendientes:
  [README de firmware](firmware/arduino/README.md#rev30-metro-m4-y-com2-por-uart-hardware).

### Rev.2.8 (Histórica)

**Cambios principales:**
- I2C/ICM-20948 se inicializan antes de arrancar el timer GPT4 de COM2;
  el cambio de orden no resolvió la detección del IMU.
- ✅ Ethernet intenta DHCP primero y usa `192.168.1.22/24` (gateway
  `192.168.1.1`) si DHCP no está disponible.
- ✅ Clientes TCP: conectar a la IP obtenida por DHCP o, en modo de respaldo,
  a `192.168.1.22` en los puertos NMEA `15919` y diagnóstico `15920`.

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

## LEDs (Rev.2.5)

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

## Diagnóstico BLE (Rev.2.5 histórica)

- **Nombre**: `FWD-GPS-Diag`
- **Servicio**: Nordic UART Service (`6E400001-B5A3-F393-E0A9-E50E24DCCA9E`)
- **RX (escribir comandos)**: `6E400002-...`
- **TX (notificaciones)**: `6E400003-...`
- Compatible con apps tipo Serial Bluetooth Terminal, nRF Connect y MIT App Inventor (extensión BluetoothLE).

**Comandos BLE:**

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
- Plataforma: **Adafruit SAMD Boards**, placa **Adafruit Metro M4**
- Librería: `Ethernet` (W5500)
- Librería: `FlashStorage` con soporte SAMD51 (ver README de firmware)
- `Wire`, `SPI` y `Uart` incluidos en el core; no se necesita ArduinoBLE

## Hardware requerido

- Adafruit Metro M4 (lógica 3.3 V)
- ArduSimple simpleRTK3B Budget (UM980)
- SparkFun ICM-20948
- W5500 Ethernet shield
- MAX3232 o equivalente para RS232
- 2 LEDs + resistencias (220 Ω)

## Autor

**BrewGold** — desarrollo e integración

## Licencia

MIT (ver LICENSE)
