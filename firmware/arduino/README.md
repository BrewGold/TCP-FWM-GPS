# Firmware Arduino

Este directorio contiene la implementación funcional del firmware para el Arduino UNO R4 WiFi utilizado como MCU del sistema GNSS FWD.

## Versión actual

- `RS232-RWM-GPS_V2-6.ino` — **Rev.2.6**, archivo completo independiente

Para compilar en Arduino IDE 2.x, abre este archivo en una carpeta llamada `RS232-RWM-GPS_V2-6` (acepta la propuesta del IDE de crearla), selecciona **Arduino UNO R4 WiFi** e instala `SparkFun ICM-20948 Arduino Library`, `ArduinoBLE` y `Ethernet` **2.0.0 o posterior** (`EthernetServer.accept()`). `Wire` y `EEPROM` pertenecen al núcleo de la placa. Mantén las revisiones históricas fuera de la carpeta del sketch.

La versión actual incluye:

- GNSS UM980 por Serial1 @ 115200 bps
- Salida FWD (COM2) por D2 @ 38400 bps (UART software TX-only)
- ICM-20948 por I2C para corrección de offset antena-pistón
- Detección HAS por fix 5 en el GGA de entrada
- Precisión horizontal estimada según HDOP y tipo de fix
- LEDs D5/D6 con estados GNSS + IMU + HAS + movimiento
- Ethernet W5500 como servidor TCP en puerto 15919, IP local por DHCP
- Diagnóstico y control por Bluetooth Low Energy (Nordic UART Service)
- Calibración del magnetómetro por BLE con guardado en EEPROM
- Frecuencia COM2/Ethernet configurable por BLE (`freq <1-10>`) y persistente en EEPROM

---

## 1) Hardware

### UART entrada GNSS (UM980)
- **Puerto**: `Serial1`
- **Pins**: D0 = RX, D1 = TX
- **Baud**: `115200`
- **Formato**: GGA + RMC

### UART salida FWD (Dynatest)
- **Puerto**: UART software TX-only en D2 (bit-banging con `noInterrupts()`)
- **Baud**: `38400`
- **Formato**: `$GPGGA` únicamente
- **Periodo**: `1000 / frecuencia` ms (división entera); 100 ms / 10 Hz por defecto

### IMU / Magnetómetro
- **Sensor**: ICM-20948 (SparkFun)
- **Bus**: I2C @ 100 kHz
- **Direcciones**: 0x69 (primaria) / 0x68 (secundaria)
- **Uso**: yaw fusionado (giróscopo + magnetómetro con compensación de inclinación) para corrección de offset antena-pistón
- **Calibración**: offsets hard-iron guardados en EEPROM, gestionados por BLE

### LEDs
- **LED1**: D5 — estado GNSS + IMU + HAS
- **LED2**: D6 — estado movimiento / bloqueado

### Ethernet
- **Shield**: W5500 (CS en D10, SD CS en D4 deshabilitado)
- **Servidor**: Arduino escucha en `15919`; su IP local se obtiene por DHCP y se muestra como `[ETH] IP local: ...`
- **PC**: conectar un lector TCP/Raw, por ejemplo PuTTY, a esa IP y puerto. No se necesita enviar comandos ni iniciar un servidor en el PC.
- **Conexiones**: un lector activo; al desconectar se acepta uno nuevo. Se mantiene la concesión DHCP y se reanuda el servidor después de recuperar el enlace. Si DHCP falla al arrancar, corrige la red y reinicia el Arduino.
- **Red**: se necesita DHCP; `192.168.1.22` solo es la dirección del Arduino si el router se la asigna. No existe una IP remota de destino.

### Bluetooth Low Energy
- **Nombre**: `FWD-GPS-Diag2`
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
- `freq <1-10>` modifica el periodo compartido COM2/Ethernet y guarda los Hz en EEPROM, después del bloque de calibración. Al arrancar, un byte fuera de 1–10 se sustituye por 10 Hz y se guarda. Cambiar la frecuencia no cambia el diagnóstico BLE de 5 s ni la tasa de entrada GNSS.
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

- `$GPGGA,...*CS`

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
8. Emite `$GPGGA` por D2 hacia el Dynatest a la frecuencia guardada.
9. Envía la misma trama al cliente Ethernet conectado, con el mismo periodo.
10. Publica diagnóstico por BLE cada 5 s y atiende comandos interactivos.

---

## 6) Diagnóstico y comandos BLE

### Comandos disponibles

```
status             Estado general completo (incluye precisión estimada)
imu                Datos del ICM-20948 y calibración
com2               Estado y contadores COM2
freq <1-10>        Guarda frecuencia COM2/Ethernet en EEPROM (Hz)
magcal start       Inicia calibración del magnetómetro
magcal stop        Finaliza, valida y guarda en EEPROM
magcal reset       Borra la calibración
yawoff <grados>    Ajuste montaje yaw (-180..180)
help               Lista de comandos
```

`status`, `com2` y `help` muestran la frecuencia guardada y el periodo activo en ms. Por ejemplo, `freq 3` confirma 3 Hz y 333 ms. Valores como `freq 0`, `freq 11`, `freq 2.5` o `freq abc` devuelven un error sin modificar el valor guardado.

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
2. Confirmar que el dispositivo BLE `FWD-GPS-Diag2` aparece al escanear.
3. Conectar desde Android (Serial Bluetooth Terminal / nRF Connect / app propia) y activar notificaciones en TX.
4. Enviar `status` y verificar GNSS, fix de entrada, HAS y precisión estimada.
5. Confirmar LED1 fijo solo con fix 5 (HAS) y parpadeo con fix 1.
6. Confirmar `$GPGGA` con checksum y CR/LF en salida FWD por D2 a 10 Hz inicialmente.
7. Validar calibración con `magcal start/stop` e `imu`.
8. Conectar PuTTY en modo **Raw** a la IP `[ETH] IP local` del Arduino, puerto 15919, sin enviar datos; confirmar las mismas tramas `$GPGGA` que en COM2. Desconectar y reconectar el lector.
9. Enviar `freq 1`, `freq 3` y `freq 10`: comprobar en `status`, `com2` y `help` los periodos 1000, 333 y 100 ms y medir ambas salidas. No deben cambiar los intervalos del diagnóstico BLE.
10. Enviar `freq`, `freq 0`, `freq 11`, `freq -1`, `freq 2.5`, `freq abc` y `freq 5 extra`: comprobar error y ausencia de cambios. Probar también ` FREQ 5 `.
11. Guardar `freq 3`, cortar alimentación y reiniciar: comprobar 3 Hz / 333 ms y que la calibración/yawoff siguen intactos. En EEPROM sin frecuencia válida, comprobar 10 Hz / 100 ms.
12. Desconectar/reconectar el cable Ethernet: comprobar recuperación del servidor y continuidad de COM2. Sin GNSS válido, el servidor debe admitir conexión aunque no emita tramas.

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

- `RS232-RWM-GPS_V2-6.ino` — firmware actual de referencia
- `RS232-RWM-GPS_V2-5.ino` — base Rev.2.5 conservada sin cambios
- `RS232-RWM-GPS_V2-4.ino` — revisión anterior (BLE, ICM-20948)
- `RS232-FMW-GPS_V-2_2.ino` / `V-2_1` / `V-2_0` / `V-1_0` — revisiones históricas
- `../../README.md` — documentación general del proyecto
- `../../CHANGELOG.md` — historial de versiones

---

## 10) Versiones

- **Rev.2.6**: servidor TCP :15919, `freq` persistente COM2/Ethernet, `$GPGGA`, BLE `FWD-GPS-Diag2`
- **Rev.2.5**: HAS por fix 5 en GGA, LED1 fijo solo con HAS, precisión estimada por HDOP en BLE
- **Rev.2.4**: BLE (Nordic UART), ICM-20948, calibración magnetómetro, COM2 UART software, sin WiFi
- **Rev.2.2**: separación `freq` / `diag` en terminal TCP
- **Rev.2.1**: WiFi AP + control TCP interactivo
- **Rev.2**: HAS (`PUBX,00`), LEDs, 10 Hz, BNO085, Ethernet
- **Rev.1**: base funcional con promedio de coordenadas y salida FWD
