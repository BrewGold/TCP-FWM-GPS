# Firmware Arduino

Este directorio contiene la implementación funcional del firmware para el Arduino UNO R4 WiFi utilizado como MCU del sistema GNSS FWD.

## Versión actual

- `RS232-RWM-GPS_V2-8.ino` — **Rev.2.8 TCP sin BLE**

### Rev.2.8: inicialización I2C/IMU y respaldo Ethernet

- `Wire.begin()`, `Wire.setClock()` y la inicialización del ICM-20948 se
  ejecutan antes de `softSerialInit()`, que inicia el timer GPT4 de COM2.
  `Serial1` se inicia antes de I2C y `gnssStarted` permite atender la entrada
  GNSS mediante `yield()` durante las operaciones cooperativas de setup.
- Ethernet mantiene DHCP como primera opción. Si falla, configura la IP fija
  `192.168.1.22`, máscara `255.255.255.0` y gateway/DNS `192.168.1.1`.
  El log indica cuál modo se usó. Los clientes TCP pueden conectar a esa IP
  de respaldo en NMEA `15919` y diagnóstico `15920`; con DHCP, usan la IP
  asignada e impresa en serie.
- Se mantienen COM2 por timer GPT4, GNSS UM980/HAS, yaw/IMU, los estados
  MOVING/AVERAGING/LOCKED, offset antena-pistón, LEDs D5/D6 y los comandos
  TCP de diagnóstico descritos para Rev.2.7.

### Rev.2.7: COM2 sin bloqueo global de interrupciones

Versión anterior, conservada como referencia:

Basada en la Rev.2.6 de doble servidor TCP de la [PR #21](https://github.com/BrewGold/TCP-FWM-GPS/pull/21)
(commit `02573a8ab5e1353dddf0777ac1f01b5ef54b9b41`); esa revisión no estaba
presente en esta rama. Las revisiones históricas no se modifican.

- COM2 conserva **D2, 38400 bps, 8N1, `$GCGGA` y 10 Hz por defecto**.
  Un timer GPT libre del RA4M1, gestionado por `FspTimer` del core oficial,
  genera una interrupción por bit. Con PCLKD a 48 MHz y divisor 1 se usan
  exactamente 1250 cuentas: periodo nominal **26.0417 µs**, sin error de
  cuantización del baudrate. La ISR tiene prioridad 4 y solo actualiza D2
  mediante escritura atómica de PCNTR3 y el estado TX; no usa SPI/I2C,
  espera activa ni enmascaramiento global de interrupciones.
- Se copia cada trama a un buffer de 240 bytes antes de publicarla a la ISR.
  El `loop()` no espera a la transmisión. Cada bit de parada ocupa un periodo
  completo. Los contadores de bytes/tramas y la fecha avanzan al finalizar
  la transmisión; `com2` informa del timer, tamaño de trama en curso y descartes.
  Si el buffer está ocupado o falla la inicialización, se descarta la trama
  completa (nunca una trama parcial), sin recurrir al bit-banging bloqueante.
  Incluso el buffer completo tarda 62.5 ms, menos que el periodo mínimo de
  salida de 100 ms.
- **Decisión de hardware:** D2 es P104 y su mux SCI solo ofrece RX, no TX.
  Los TX expuestos D1, D7, D11, D13 y A4 comparten SCI2/SCI0 con Serial1
  o requieren cambiar el cableado/pines usados. Se mantiene D2 y no se
  ocupan timers PWM reservados ni AGT de `millis()`/`micros()`.
  Véanse las APIs y el mux del [core oficial](https://github.com/arduino/ArduinoCore-renesas/tree/424e86eff92d37f72123c2b641dd8bbf06a38b47)
  (`variants/UNOWIFIR4/pinmux.inc`, `cores/arduino/FspTimer.h`).
- `Serial1` empieza después de las esperas de arranque (LEDs, IMU y DHCP).
  Se drena GNSS antes y entre las tareas del `loop()`, y mediante `yield()`
  durante las esperas cooperativas de las librerías. El parser se protege
  contra reentrada y nunca se ejecuta desde una ISR. El core fija su buffer
  UART en **512 bytes** (unos 44 ms a 115200 bps) sin API pública de ampliación;
  no se modifica el core ni se añade un `#define` ineficaz.
- El `loop()` ya no espera a COM2. Ethernet aún puede esperar por ACK/reintentos
  TCP o cierre de clientes, e IMU puede usar `delay()` al reinicializar:
  esas esperas llaman a `yield()` y mantienen atendido GNSS. Se comprueba
  espacio TX antes de escribir y se desconectan clientes saturados, evitando
  la espera ilimitada por espacio del socket. No se promete un `loop()`
  totalmente no bloqueante ni se cambian los timeouts internos de Wire.
- Se conservan HAS/fix 5, yaw/IMU, MOVING/AVERAGING/LOCKED, offset, LEDs D5/D6,
  EEPROM y los servidores **NMEA TCP 15919 / diagnóstico TCP 15920**, con
  `status`, `imu`, `com2`, `help`, `magcal start/stop/reset`, `yawoff`,
  `freq <1..10>` y `diag on/off`. Rev.2.7 no usa BLE.

**Arduino IDE 2.x:** instalar la plataforma oficial **Arduino UNO R4 Boards**
y seleccionar **Arduino UNO R4 WiFi**. Abrir el `.ino` en una carpeta propia
llamada `RS232-RWM-GPS_V2-7` (no agrupar las revisiones históricas en un único
sketch). Instalar Ethernet (W5500) y SparkFun ICM-20948 Arduino Library;
Wire, EEPROM y FspTimer vienen con el core. No hace falta SoftwareSerial
ni una librería adicional de timers.

**Validación en placa pendiente:** la lógica puede simularse en host, pero
no reemplaza la compilación con el core instalado ni las mediciones de banco.
Con analizador lógico en D2, decodificar 38400/8N1 y medir los intervalos de
bit bajo carga simultánea GNSS, SPI, I2C y comandos TCP. Objetivo de aceptación:
error/jitter de transición inferior a ±2% de un bit (±0.52 µs), sin bits
omitidos, checksum `$GCGGA` correcto y decodificación continua en el Dynatest.
El divisor exacto no garantiza ese jitter ni la precisión del oscilador HOCO.
Revisar que no reaparezcan errores de checksum GGA/RMC ni falsos negativos
W5500/ICM-20948, que `com2` no muestre descartes a 1/10 Hz, y probar clientes
lentos, cable desconectado, reintento IMU y todos los comandos de diagnóstico.

## Referencia histórica Rev.2.5 (BLE)

- `RS232-RWM-GPS_V2-5.ino` — **Rev.2.5**

Esta revisión histórica incluye:

- GNSS UM980 por Serial1 @ 115200 bps
- Salida FWD (COM2) por D2 @ 38400 bps (UART software TX-only)
- ICM-20948 por I2C para corrección de offset antena-pistón
- Detección HAS por fix 5 en el GGA de entrada
- Precisión horizontal estimada según HDOP y tipo de fix
- LEDs D5/D6 con estados GNSS + IMU + HAS + movimiento
- Ethernet W5500 a 192.168.1.122:15919
- Diagnóstico y control por Bluetooth Low Energy (Nordic UART Service)
- Calibración del magnetómetro por BLE con guardado en EEPROM

---

## 1) Hardware

### UART entrada GNSS (UM980)
- **Puerto**: `Serial1`
- **Pins**: D0 = RX, D1 = TX
- **Baud**: `115200`
- **Formato**: GGA + RMC

### UART salida FWD (Dynatest)
- **Puerto**: UART TX-only por timer GPT en D2
- **Baud**: `38400`
- **Formato**: `$GCGGA` únicamente
- **Periodo**: `100 ms` (10 Hz)

### IMU / Magnetómetro
- **Sensor**: ICM-20948 (SparkFun)
- **Bus**: I2C @ 100 kHz
- **Direcciones**: 0x69 (primaria) / 0x68 (secundaria)
- **Uso**: yaw fusionado (giróscopo + magnetómetro con compensación de inclinación) para corrección de offset antena-pistón
- **Calibración**: offsets hard-iron guardados en EEPROM, gestionados por TCP

### LEDs
- **LED1**: D5 — estado GNSS + IMU + HAS
- **LED2**: D6 — estado movimiento / bloqueado

### Ethernet
- **Shield**: W5500 (CS en D10, SD CS en D4 deshabilitado)
- **IP del dispositivo**: por DHCP o `192.168.1.22` como respaldo
- **Servidores TCP**: NMEA `15919` y diagnóstico `15920`

### Bluetooth Low Energy
- **Nombre**: `FWD-GPS-Diag`
- **Servicio**: Nordic UART Service
  - Servicio: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
  - RX (comandos): `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
  - TX (notificaciones): `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
- Chunks de 20 bytes; acepta comandos con o sin CR/LF (compatible con MIT App Inventor)

---

## 2) Máquina de estados

### Movimiento

- `MOVING → AVERAGING`
  - velocidad < 0.20 m/s mantenida 2 s
- `AVERAGING → LOCKED`
  - al completar la ventana de promedio (15 s), media recortada 5% + media circular del yaw
- `LOCKED → MOVING`
  - velocidad > 0.30 m/s, o alejamiento > 1.0 m del punto bloqueado

### Detección de solución (Rev.2.5)

El tipo de solución se determina por el campo de calidad del GGA de entrada:

| Fix GGA | Solución | HAS activo |
|---|---|---|
| 1 | GPS autónomo | NO |
| 2 | DGPS | NO |
| 4 | RTK | NO |
| 5 | HAS (PPP/Float) | **SÍ** |

### Corrección antena → pistón

- Offset por defecto: `0.55 m`
- Dirección: `yaw + 270°`
- Declinación magnética por defecto: `1.0°`
- Solo se aplica si el ICM-20948 está operativo y hay yaw válido

---

## 3) Parámetros de navegación

Valores por defecto:

- `OUTPUT_PERIOD_MS = 100` → 10 Hz
- `OFFSET_M = 0.55`
- `DECLINATION_DEG = 1.0`
- `STOP_CONFIRMATION_MS = 2000`
- `AVERAGING_WINDOW_MS = 15000` → 15 s
- `SPEED_ENTER_STOP = 0.20 m/s`
- `SPEED_EXIT_STOP = 0.30 m/s`
- `RELOCK_DISTANCE_M = 1.0`
- `MAX_SAMPLES = 160`
- `YAW_GYRO_WEIGHT = 0.98` (filtro complementario)

---

## 4) Formato de salida NMEA

El firmware genera y transmite siempre una trama tipo:

- `$GCGGA,...*CS`

Con fix quality de salida según el estado:

1. `LOCKED` + posición válida → `fixQ = 4`
2. HAS activo → `fixQ = 2`
3. GNSS válido y no bloqueado → `fixQ = 1`

---

## 5) Flujo de funcionamiento

1. Lee líneas NMEA del UM980 por `Serial1` (`GGA`, `RMC`; prefijos GP/GN/GC).
2. Valida checksum y parsea posición, velocidad, altitud, HDOP y fix de entrada.
3. Determina la solución (GPS/DGPS/RTK/HAS) desde el fix del GGA.
4. Lee el ICM-20948 y calcula yaw fusionado.
5. Actualiza la máquina de estados (`MOVING`, `AVERAGING`, `LOCKED`).
6. En `AVERAGING` acumula lat/lon/alt/yaw y calcula promedio recortado.
7. En `LOCKED` transmite la posición corregida; si no hay bloqueo, transmite instantánea con offset si hay yaw.
8. Emite `$GCGGA` por D2 hacia el Dynatest a 10 Hz.
9. Envía la misma trama por Ethernet.
10. Publica diagnóstico por BLE cada 5 s y atiende comandos interactivos.

---

## 6) Diagnóstico y comandos BLE

### Comandos disponibles

```
status             Estado general completo (incluye precisión estimada)
imu                Datos del ICM-20948 y calibración
com2               Estado y contadores COM2
magcal start       Inicia calibración del magnetómetro
magcal stop        Finaliza, valida y guarda en EEPROM
magcal reset       Borra la calibración
yawoff <grados>    Ajuste montaje yaw (-180..180)
help               Lista de comandos
```

### Calibración del magnetómetro

1. Enviar `magcal start`.
2. Girar el equipo lentamente en todos los ejes (mínimo 200 muestras, span ≥ 10 µT en X e Y).
3. Enviar `magcal stop` → guarda offsets en EEPROM.
4. Ajustar `yawoff` comparando el yaw mostrado con el rumbo real.

### Precisión estimada

Se reporta en `status` (`Precision est`) y en `[DIAG]` (`ACC`), calculada como `UERE × HDOP`:

- GPS: 3.00 m · DGPS: 1.00 m · RTK: 0.02 m · HAS: 0.20 m

### Ejemplo de diagnóstico periódico

```text
[DIAG]
GNSS=OK FIX_IN=5 HAS=ON SOL=HAS SAT=22
HDOP=0.6 ACC=0.12 m
STATE=LOCKED LOCK=YES SPD=0.030 m/s
IMU=OK YAW=181.2 MAGCAL=YES
RAW=40.12345678,-3.45678901
OUT=40.12345950,-3.45679120 FIX_OUT=4
COM2 frames=1250 bytes=103750
```

---

## 7) Checklist rápido de validación

1. Conectar Arduino con alimentación externa adecuada.
2. Confirmar que el dispositivo BLE `FWD-GPS-Diag` aparece al escanear.
3. Conectar desde Android (Serial Bluetooth Terminal / nRF Connect / app propia) y activar notificaciones en TX.
4. Enviar `status` y verificar GNSS, fix de entrada, HAS y precisión estimada.
5. Confirmar LED1 fijo solo con fix 5 (HAS) y parpadeo con fix 1.
6. Confirmar `$GCGGA` en salida FWD por D2 a 10 Hz.
7. Validar calibración con `magcal start/stop` e `imu`.
8. Comprobar la conexión Ethernet al servidor externo.

---

## 8) Notas técnicas

- `SoftwareSerial.h` no es compatible con Arduino UNO R4 WiFi; la salida FWD se implementa por bit-banging TX-only en D2 con interrupciones desactivadas por byte.
- El UM980 debe configurarse con:

```
UNLOG COM3
CONFIG COM3 115200
GNGGA COM3 0.1
GNRMC COM3 0.1
ENABLE HAS
SAVECONFIG
```

- La corrección de antena se aplica solo si el ICM-20948 está disponible y el yaw es válido.
- La calibración del magnetómetro y el `yawoff` se guardan en EEPROM y sobreviven reinicios.
- El WiFi Server de revisiones anteriores fue eliminado en Rev.2.4.

---

## 9) Archivos relevantes

- `RS232-RWM-GPS_V2-5.ino` — firmware actual de referencia
- `RS232-RWM-GPS_V2-4.ino` — revisión anterior (BLE, ICM-20948)
- `RS232-FMW-GPS_V-2_2.ino` / `V-2_1` / `V-2_0` / `V-1_0` — revisiones históricas
- `../../README.md` — documentación general del proyecto
- `../../CHANGELOG.md` — historial de versiones

---

## 10) Versiones

- **Rev.2.5**: HAS por fix 5 en GGA, LED1 fijo solo con HAS, precisión estimada por HDOP en BLE
- **Rev.2.4**: BLE (Nordic UART), ICM-20948, calibración magnetómetro, COM2 UART software, sin WiFi
- **Rev.2.2**: separación `freq` / `diag` en terminal TCP
- **Rev.2.1**: WiFi AP + control TCP interactivo
- **Rev.2**: HAS (`PUBX,00`), LEDs, 10 Hz, BNO085, Ethernet
- **Rev.1**: base funcional con promedio de coordenadas y salida FWD
