# Firmware Arduino

Este directorio contiene el firmware del sistema GNSS FWD para Adafruit Metro RP2040
y las revisiones históricas para Metro M4 y Arduino UNO R4 WiFi.

## TCP-IRI-GPS: movimiento continuo

Abrir **`TCP-IRI-GPS/TCP-IRI-GPS.ino`**: carpeta Arduino autónoma con implementación
real de `setup`/`loop`, LEDs, COM2 PIO y W5500 en el sketch, y parser/estado GNSS y
generación NMEA en su archivo local `Gnss.h`. Se puede copiar **solo esa carpeta**.
No incluye `shared/`, otro `.ino` ni un selector `TCP_IRI_GPS`.
Se eliminó el antiguo wrapper `firmware/arduino/TCP-IRI-GPS.ino` para evitar dos
entradas distintas. FWM sigue usando `RS232-RWM-GPS_V4-0.ino` y
`shared/TcpGpsFirmware.h`, sin cambios en sus fuentes.

IRI usa la misma placa, cableado GNSS, COM2 PIO y W5500 de Rev.4.0:

- No inicializa I2C, IMU ni EEPROM, no lee magnetómetro ni calcula yaw.
  No requiere instalar SparkFun ICM-20948. `imu` responde `OFF`; los comandos
  `magcal`/`yawoff` se rechazan sin modificar calibraciones.
- Cada salida usa la última latitud/longitud/altitud GNSS válida, sin filtros,
  promedios, offset ni posición bloqueada, incluso a velocidad cero.
  El fix de salida es el fix de entrada (incluido 5 para HAS), sin remapeo.
  Mantiene `$GPGGA`, checksum/CRLF, frecuencia 1–10 Hz, TCP 15919 y COM2 D2.
- Velocidad directa desde RMC válido: nudos × 0.51444 = m/s, sin promedio.
  Se registra en `status` (`Velocidad`), `[DIAG]` (`SPD`, cada 5 s por defecto)
  y monitor USB junto a cada transmisión. `N/A` significa dato no disponible:
  antes del primer RMC, RMC inactivo/velocidad inválida, o más de 2 s sin RMC.
  No se presenta la falta de velocidad como cero. GGA caducado (>2 s) o fix cero
  detiene la salida y desactiva HAS. Un RMC inválido (incluido checksum incorrecto,
  truncado o modo N) invalida inmediatamente la velocidad, sin alterar la posición
  GGA. Un GGA con checksum incorrecto no actualiza posición ni frescura; uno con
  checksum válido pero campos inválidos invalida el fix. Se validan coordenadas,
  hemisferios, UTC, calidad, satélites, HDOP y altitud: no se inventan ceros.
- Conserva la prueba de ambos LEDs al arrancar (5 s). LED1/D5: apagado sin
  GNSS, parpadeo 600 ms con GNSS sin HAS, fijo con HAS. La ausencia intencional
  de IMU no activa el aviso de fallo de 200 ms. LED2/D6 conserva el significado
  de MOVING: apagado; no hay estados AVERAGING/LOCKED en IRI.
- Banner USB, bienvenida TCP, `status` y `[DIAG]` identifican `TCP-IRI-GPS`.
  El cliente DiagTelnetWin sigue siendo compatible y conserva la velocidad en
  el registro/exportación de la respuesta Telnet.

### Compilar y seleccionar IRI

Versiones verificadas: **Arduino CLI 1.3.1**, **Arduino-Pico 5.4.3**
(Earle Philhower), **Ethernet 2.0.2** de Arduino. `SerialPIO`, SPI y Pico SDK
pertenecen al core. No necesita SparkFun ICM-20948, Wire ni EEPROM.
Placa confirmada mediante `arduino-cli board details`:
**Adafruit Metro RP2040, `rp2040:rp2040:adafruit_metro`** (opciones predeterminadas).
No seleccionar el core Arduino Mbed ni compilar el directorio superior con sus
sketches históricos.

```sh
REPO=/home/runner/work/TCP-FWM-GPS/TCP-FWM-GPS # ajustar a la ruta absoluta local
INDEX=https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli core update-index --additional-urls "$INDEX"
arduino-cli core install rp2040:rp2040@5.4.3 --additional-urls "$INDEX"
arduino-cli lib install "Ethernet@2.0.2"
arduino-cli board details --fqbn rp2040:rp2040:adafruit_metro
arduino-cli compile --fqbn rp2040:rp2040:adafruit_metro --warnings all \
  "$REPO/firmware/arduino/TCP-IRI-GPS"
arduino-cli board list
# Sustituir /dev/ttyACM0 por el puerto real; en primera carga usar BOOTSEL si hace falta.
arduino-cli upload --fqbn rp2040:rp2040:adafruit_metro --port /dev/ttyACM0 \
  "$REPO/firmware/arduino/TCP-IRI-GPS"
```

Cableado inalterado: UM980 Serial1 D0 RX/D1 TX a 115200; COM2/RS232 D2 TX-only
PIO a 38400/8N1 hacia el transceptor del shield; W5500 por SPI ICSP, CS D10 y
SD deseleccionada en D4. TCP NMEA 15919 (hasta 4 clientes), diagnóstico 15920
(1 cliente), DHCP con respaldo `192.168.1.22/24`, gateway/DNS `192.168.1.1`.
Salida `$GPGGA` con checksum y CRLF, 10 Hz por defecto, `freq 1..10` configurable.
Conserva recuperación Ethernet cooperativa y vaciado incremental del FIFO PIO.
No cambia talker, precisión decimal ni formato heredado de salida.

En Arduino IDE abrir la carpeta autónoma IRI con `Gnss.h` presente.
Para volver a FWM compilar/cargar `RS232-RWM-GPS_V4-0.ino` con su copia de
`shared/`, según las instrucciones de Rev.4.0.

### Validación de IRI

Validado en este cambio (2026-10-19): compilación **real RP2040** con las versiones
anteriores; 107836 bytes de programa, 24512 bytes globales de RAM. El sketch se
compila directamente como carpeta independiente, sin concatenar otros `.ino`.
También se comprobó una copia aislada de esa carpeta.

Límites del entorno: `downloads.arduino.cc` no resolvía por DNS, de modo que
`core update-index` no pudo obtener los índices generales ni discovery.
CLI 1.3.1 se obtuvo de su release oficial GitHub, el core mediante `core install`
y Ethernet de su archivo oficial de tag, instalado mediante `lib install --zip-path`.
El primer build se detuvo por ausencia del `ctags` de Arduino; tras instalar su
release oficial 5.8-arduino11, el build terminó con código 0. Comandos adicionales
usados en este entorno (los errores de índices/discovery no impidieron compilar):

```sh
mkdir -p /tmp/iri-tools
curl -fL https://github.com/arduino/arduino-cli/releases/download/v1.3.1/arduino-cli_1.3.1_Linux_64bit.tar.gz \
  -o /tmp/iri-tools/cli.tar.gz
tar -xzf /tmp/iri-tools/cli.tar.gz -C /tmp/iri-tools
curl -fL https://github.com/arduino-libraries/Ethernet/archive/refs/tags/2.0.2.zip \
  -o /tmp/iri-tools/Ethernet-2.0.2.zip
/tmp/iri-tools/arduino-cli config set library.enable_unsafe_install true
/tmp/iri-tools/arduino-cli lib install --zip-path /tmp/iri-tools/Ethernet-2.0.2.zip
/tmp/iri-tools/arduino-cli config set library.enable_unsafe_install false
curl -fL https://github.com/arduino/ctags/releases/download/5.8-arduino11/ctags-5.8-arduino11-x86_64-pc-linux-gnu.tar.bz2 \
  -o /tmp/iri-tools/ctags.tar.bz2
tar -xjf /tmp/iri-tools/ctags.tar.bz2 -C /tmp/iri-tools
/tmp/iri-tools/arduino-cli compile --fqbn rp2040:rp2040:adafruit_metro --warnings all \
  --build-property runtime.tools.ctags.path=/tmp/iri-tools \
  "$REPO/firmware/arduino/TCP-IRI-GPS"
```

Pruebas host reproducibles, sin bibliotecas de test adicionales:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer \
  -DARDUINO_ADAFRUIT_METRO_RP2040 -DARDUINO_ARCH_RP2040 \
  -I"$REPO/firmware/tests/stubs" "$REPO/firmware/tests/TcpIriGpsTest.cpp" \
  -o /tmp/TcpIriGpsTest
/tmp/TcpIriGpsTest
```

El test incluye el sketch real con dobles de hardware: prueba LEDs de arranque,
25 s de coordenadas cambiantes a velocidad cero sin promedio/bloqueo, mismas tramas
COM2/TCP con fix 1/2/4/5 y checksum/CRLF, validación y pérdida de fix, RMC
válido/vacío/inactivo/corrupto/caducado, rollover de `millis`, status y `[DIAG]`,
comandos y rechazo de calibración. No hay stubs de IMU/EEPROM: incluirlos por
accidente haría fallar la compilación host. No sustituye el build RP2040.
DiagTelnetWin sigue recibiendo delimitadores `STATUS`, `[DIAG]` y claves
GNSS/fix/HAS/solución compatibles; la velocidad se conserva en el registro,
sin exigir cambios del cliente.

**Pendiente con hardware** (no se han probado físicamente LEDs, RS232, W5500,
DHCP, temporización real ni carga USB en esta sesión):

1. Sin IMU conectada, comprobar banner IRI y ausencia de reintentos I2C.
2. Inyectar GGA sucesivos con coordenadas/altitud distintas y RMC a 0 m/s
   durante más de 20 s: no debe entrar en promedio/bloqueo. Repetir en movimiento;
   comprobar que COM2 y TCP emiten las últimas coordenadas, no una media.
3. Probar fix 1/2/4/5 y verificar el mismo fix en `$GPGGA`, checksum y CRLF.
4. Probar RMC a 10 nudos (`5.144 m/s`), otro a 20 (`10.289 m/s`), velocidad
   cero, campo vacío, negativo, estado V y checksum incorrecto. No mezclar
   velocidades; sin RMC durante más de 2 s debe indicar `N/A`.
5. Consultar `status`, activar `diag on` y comprobar velocidad e identidad en
   TCP 15920 y USB; verificar `com2`, `freq`, `help`, `diag off`, `debug hex`.
6. Verificar LEDs en arranque, sin GNSS, con fix 1 y con HAS; D6 siempre MOVING.
7. Cargar FWM y verificar que conserva IMU, calibración, promedio y bloqueo.

## Versión actual

- `RS232-RWM-GPS_V4-0.ino` — **Rev.4.0 TCP sin BLE, Metro RP2040**

### Rev.4.0: Metro RP2040, shield sin cambios

Base completa: `RS232-RWM-GPS_V3-0.ino`, conservada sin modificaciones.
Se mantienen GGA/RMC, HAS/fix 5, MOVING/AVERAGING/LOCKED, corrección de antena,
calibración yaw/magnetómetro y todos los comandos TCP de Rev.3.0.

| Función | Pin del shield | Recurso Metro RP2040 |
|---|---|---|
| GNSS UM980 | D0 RX / D1 TX, 115200 | `Serial1`, UART0 (GPIO1 RX / GPIO0 TX) |
| FWD/COM2 TX | **D2**, 38400/8N1 | `SerialPIO`, GPIO2, RX=`NOPIN` |
| LEDs | D5/D6 | GPIO5/GPIO6 |
| IMU | SDA/SCL, 100 kHz, 0x69/0x68 | `Wire`, GPIO16/GPIO17 |
| W5500 | SPI ICSP, CS D10 | `SPI`, SCK GPIO18 / MOSI GPIO19 / MISO GPIO20 |
| SD del shield | CS D4 en HIGH | Deshabilitada |

**No mover FWD a D7.** D2 no es TX de UART hardware RP2040: PIO genera los
bits en ese mismo pin. La cola de una trama se vacía al FIFO PIO sin bloquear
ni exigir que el FIFO de ocho bytes contenga toda la trama. Si la anterior
sigue pendiente, se descarta la nueva completa, sin mezclar tramas.
Una implementación cooperativa de `delay()` mantiene GNSS y COM2 atendidos
durante las esperas de IMU/DHCP (el `delay()` del core no llama a `yield()`).
`com2BytesSent` cuenta bytes aceptados por PIO; `com2FramesSent` y
`com2LastFrameMs` avanzan al aceptar el último byte. No confirman recepción FWD
ni finalización física del último bit.

**Switch RX/TX de Metro RP2040:** seleccionar la posición que conecta
**D0 del shield a RX (GPIO1) y D1 a TX (GPIO0)**, no D0=TX/D1=RX.
El switch sí intercambia esos dos contactos; no afecta D2/FWD.
Se conservan los pines UART0 predeterminados del core, sin reasignarlos.
Ver [guía de Metro RP2040](https://learn.adafruit.com/adafruit-metro-rp2040/pinouts)
y [variante Arduino-Pico](https://github.com/earlephilhower/arduino-pico/blob/master/variants/adafruit_metro/pins_arduino.h).
Toda la lógica es **3.3 V, no tolerante a 5 V**; verificar señales y pull-ups.
No conectar el nivel RS232 directamente a D2: usar MAX3232.

Ethernet mantiene DHCP, respaldo `192.168.1.22/24`, NMEA **15919** y diagnóstico
**15920**. El diagnóstico utiliza una cola acotada de 4096 bytes, conserva
escrituras parciales y cierra el cliente si la cola se desborda. Los informes
formateados rechazan truncamiento; los comandos demasiado largos se descartan
hasta fin de línea, sin ejecutar su sufijo.

El arranque realiza hasta tres intentos, con backoff de 1 y 2 segundos.
DHCP tiene un timeout de 5 segundos y, tras configurar DHCP o la IP de respaldo,
se espera hasta 5 segundos a `LinkON` antes de abrir los servidores. USB muestra
`Servidor TCP: OK` solo cuando ambos puertos tienen un socket en escucha; en caso
contrario muestra el fallo. `status` informa `LISTEN`/`SIN ESCUCHA` por puerto.
D4 permanece alto y se deja 1 µs de margen antes de los accesos Ethernet.
Si se pierde el enlace, se suspenden los servicios TCP; al recuperarlo se
reinicializa Ethernet reutilizando la última configuración IP, sin esperar DHCP
ni enlace en runtime, con reintentos separados por backoff hasta 30 segundos.
Los clientes anteriores se cierran con timeout acotado y deben reconectarse.
Las esperas de detección de hardware mantienen la generación periódica COM2,
sin acceder a Ethernet desde la espera cooperativa.
Si cambia la configuración de red, reiniciar la placa para obtener DHCP de nuevo.

Comprobación en banco: conectar el PC a la misma subred que la IP indicada por
USB (para el respaldo, una IP libre `192.168.1.x/24`) y ejecutar
`telnet <IP> 15920`. Debe aparecer `FWD-GPS Rev.4.0 TCP diagnostic`; probar
`help` y `status`, y comprobar ambos puertos en escucha. Abrir también
`telnet <IP> 15919` para recibir NMEA cuando haya salida GNSS. Repetir tras
arrancar sin cable y tras desconectarlo/reconectarlo, reconectando los clientes.

**Captura HEX integrada de Serial1:** en el puerto **15920**, enviar
`debug hex on`. Responde `OK: captura iniciada`; tras 10 segundos entrega
`[HEX CAPTURE]`, duración, bytes guardados/descartados, líneas de hasta 16 bytes
en hexadecimal y `[FIN CAPTURA]`. Captura todos los bytes antes del filtrado
del parser, incluidos CR/LF, caracteres binarios y frases con checksum incorrecto.
El buffer circular de **8192 bytes** conserva los bytes más recientes en orden;
cada byte antiguo sobrescrito cuenta como descartado. La captura no se repite
automáticamente. `debug hex off` finaliza antes y entrega los datos parciales
con su duración real. Otro `debug hex on` durante la captura o su volcado responde
`ERROR: captura ya en curso`. Desconectar el cliente o perder Ethernet cancela
la captura pendiente; al reconectar hay que iniciarla de nuevo.

La recepción y el parseo GGA/RMC, COM2 en D2, TCP NMEA e IMU siguen operativos.
El volcado utiliza la cola TCP existente, una línea por vuelta y sin esperar
espacio; `[DIAG]` se pausa solo durante el volcado y se reanuda al finalizar.
Esto es diagnóstico, no una corrección de checksum ni una prueba de su causa.
En banco, comprobar capturas sin datos, con más de 8192 bytes, parada anticipada,
inicio duplicado y cliente lento/desconectado mientras COM2/NMEA/IMU continúan.

Calibración: `EEPROM.begin/get/put/commit` del core, en el sector de flash
reservado por Arduino-Pico; no usa FlashStorage SAMD ni SPI del shield.
Se mantienen el formato, magic y validación de valores finitos. Recalibrar
al migrar; no se importan datos de otras placas. Guardar solo mediante
`magcal stop/reset` o `yawoff`, nunca periódicamente, para evitar desgaste.
La calibración sobrevive reinicios; borrar toda la flash la elimina.

#### Instalación y compilación

En Arduino IDE 2.x, añadir al gestor de placas:
`https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`.
Instalar **Raspberry Pi Pico/RP2040/RP2350 de Earle F. Philhower** y seleccionar
**Adafruit Metro RP2040** (no el core Arduino Mbed, UNO R4 ni Metro M4).
Instalar **Ethernet 2.0.2** y **SparkFun ICM-20948 Arduino Library 1.3.2**.
`SerialPIO`, `EEPROM`, `Wire` y `SPI` están incluidos en el core.
Abrir Rev.4 en una carpeta propia `RS232-RWM-GPS_V4-0`, copiando también
`shared/` dentro de esa carpeta; no agrupar revisiones. El sketch selecciona
la aplicación FWM, cuya implementación reside en `shared/TcpGpsFirmware.h`.

Desde la raíz del repositorio, con Arduino CLI y las bibliotecas instaladas:

```sh
arduino-cli core install rp2040:rp2040 --additional-urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
mkdir -p /tmp/RS232-RWM-GPS_V4-0
cp "$PWD/firmware/arduino/RS232-RWM-GPS_V4-0.ino" /tmp/RS232-RWM-GPS_V4-0/
cp -R "$PWD/firmware/arduino/shared" /tmp/RS232-RWM-GPS_V4-0/
arduino-cli compile --fqbn rp2040:rp2040:adafruit_metro --warnings all /tmp/RS232-RWM-GPS_V4-0
```

Validación en banco pendiente: comprobar D2 con analizador a 38400/8N1,
checksum/CRLF y contadores a 1/10 Hz bajo carga GNSS/I2C/TCP; verificar
switch GNSS, ICM-20948 y W5500 simultáneamente; probar `status`, `imu`,
`com2`, `help`, `magcal start/stop/reset`, `yawoff`, `freq`, `diag on/off`,
`debug hex on/off`,
clientes lentos/desconectados, comandos largos y persistencia tras reinicio.

### Rev.3.0 (histórica): Metro M4 y COM2 por UART hardware

Base: `RS232-RWM-GPS_V2-8.ino`, conservada sin modificaciones. Se migra de
RA4M1 a **SAMD51 Cortex-M4 a 120 MHz** para evitar la dependencia del core
Renesas y su [issue #543](https://github.com/arduino/ArduinoCore-renesas/issues/543),
abierto, sobre SDA/SCL en modo CMOS en vez de open-drain en I2C SCI.
El informe original es de XIAO RA4M1/SCI y no demuestra por sí solo un fallo
determinista de todas las UNO R4. El orden de init de Rev.2.8 no resolvió
la detección del IMU; no se presupone un conflicto SPI/I2C/GPT.

#### Cableado y recursos

| Función | Metro M4 | Recurso |
|---|---|---|
| GNSS UM980 | D0 RX / D1 TX, 115200 | `Serial1`, SERCOM3, PAD1/PAD0 |
| COM2 TX hacia MAX3232/FWD | **D7**, 38400/8N1 | `Serial2` propio, SERCOM4/PAD0, mux `PIO_SERCOM` |
| IMU ICM-20948 | SDA/SCL, 100 kHz, 0x69/0x68 | `Wire`, SERCOM5 |
| W5500 | SPI **ICSP**, CS D10, SD CS D4 deshabilitado | `SPI`, SERCOM2 |
| LEDs | D5/D6 | GPIO |

**Mover únicamente COM2 TX de D2 a D7.** D2 = PB17 ofrece SERCOM5/PAD1,
no un TX USART SAMD51; D3 = PB16 ofrece SERCOM5/PAD0, pero SERCOM5 está
reservado para I2C y no se reutiliza. D7 = PB12 ofrece SERCOM4/PAD0 por mux C
y no lo usan el shield ni los otros periféricos. No hay
`Serial2`/`Serial3` predefinidos en esta variante: el sketch declara su `Uart`
y los cuatro handlers `SERCOM4_0/1/2/3_Handler`. No necesita Adafruit_ZeroSeries.
El core exige un índice RX válido en el constructor: se reutiliza el índice D7
sin mapear otro GPIO y se deshabilita el receptor USART. **No conectar RX COM2**.

El shield apilable debe tomar SPI del **ICSP**, no de D11/D12/D13.
Conectar el IMU a los conectores **SDA/SCL**, no a A4/A5 (en Metro M4 son
pines distintos). Para esos conectores se mantiene el cableado del shield.
**Metro M4 usa lógica 3.3 V, no tolera 5 V**: verificar las señales del UM980,
el MAX3232 y el shield, así como los pull-ups del IMU a 3.3 V antes de conectar.
`Wire.begin()`/`Wire.setClock(100000)` usan el I2C hardware open-drain;
no se aplica ningún workaround Renesas ni se omiten los pull-ups externos
(usar los del breakout si ya los incorpora).
`Ethernet.h` conserva su API SPI genérica y CS explícito D10. Se mantienen
DHCP y respaldo `192.168.1.22/24`, gateway/DNS `192.168.1.1`.

#### Transmisión y persistencia

- `Serial2.begin(COM2_BAUD, SERIAL_8N1)` configura el USART real;
  `Serial2.write()` transmite la misma trama `$GCGGA`, checksum y CR/LF.
  Se comprueba `availableForWrite()` antes de encolar una trama completa
  para no esperar por espacio ni introducir una trama parcial por saturación.
- Los bits los genera el USART: a 38400/8N1 cada bit dura nominalmente
  **26.0417 µs**, cada byte **260.417 µs** y 240 bytes **62.5 ms**, menor que
  el periodo mínimo de 100 ms. No hay ISR por bit, FspTimer ni bit-banging.
  Los handlers solo atienden el buffer UART del core, sin parsear GNSS.
- `com2BytesSent` suma el retorno de `write()`; `com2FramesSent` y
  `com2LastFrameMs` avanzan al aceptar la trama completa. Estos contadores
  indican bytes/tramas aceptados por el UART, no confirmación del FWD ni
  finalización física del último bit. No se usa `flush()` bloqueante.
- SAMD51 no tiene la EEPROM de RA4M1: `FlashStorage(magCalStorage,
  MagCalibration)` reserva flash **interna** (no QSPI ni SPI del shield).
  `read()`/`write()` sustituyen `EEPROM.get()`/`put()`, manteniendo magic y
  comprobaciones de valores finitos. Calibración y `yawoff` sobreviven
  reinicios, pero se pierden al cargar un sketch; no se importan los datos
  de la UNO R4. Recalibrar después de migrar o reprogramar.
  Guardar solo con `magcal stop/reset` o `yawoff`, nunca periódicamente:
  cada escritura borra flash y consume su vida útil; evitar comandos repetidos.
- `millis()`, `micros()`, `delay()` y la comprobación CMSIS `__get_IPSR()`
  existen en SAMD51; se mantienen las restas sin signo y la atención GNSS
  mediante `yield()`. No hay accesos a registros RA4M1.
- Sin cambios en HAS/fix 5, MOVING/AVERAGING/LOCKED, offset antena-pistón,
  LEDs o TCP NMEA **15919** / diagnóstico **15920** y sus comandos:
  `status`, `imu`, `com2`, `help`, `magcal start/stop/reset`, `yawoff`,
  `freq <1..10>`, `diag on/off`.

#### Instalación y validación

**Arduino IDE 2.x:** instalar **Adafruit SAMD Boards** y seleccionar
**Adafruit Metro M4** (no UNO R4 ni Metro M0). Abrir el archivo en una carpeta
propia `RS232-RWM-GPS_V3-0`; no agrupar las revisiones en un mismo sketch.
Instalar Ethernet (W5500), SparkFun ICM-20948 Arduino Library y
**FlashStorage de cmaglie, commit
[`634e7fd0c84120260d52ab75dd7687980941c3d9`](https://github.com/cmaglie/FlashStorage/tree/634e7fd0c84120260d52ab75dd7687980941c3d9)**:
descargar el ZIP de ese commit e instalar mediante «Añadir biblioteca .ZIP».
Su metadata indica 1.0.0, pero incluye correcciones SAMD51 de caché NVMCTRL/CMCC
posteriores al tag 1.0.0; no usar solo la versión del gestor sin esos fixes.
Wire, SPI, Uart, SERCOM y
`wiring_private.h` pertenecen al core Adafruit; no instalar EEPROM/FspTimer/BLE.

Fuentes para la validación conceptual:
[variante Metro M4 (core 1.7.16)](https://github.com/adafruit/ArduinoCore-samd/tree/1.7.16/variants/metro_m4),
[Uart.cpp](https://github.com/adafruit/ArduinoCore-samd/blob/1.7.16/cores/arduino/Uart.cpp),
[SERCOM.cpp](https://github.com/adafruit/ArduinoCore-samd/blob/1.7.16/cores/arduino/SERCOM.cpp),
[mux SAMD51J19A](https://github.com/adafruit/asf4/blob/d270f79aa16dd8fd4ae3b6c14544283dcb992e9c/samd51/include/pio/samd51j19a.h#L1218-L1225)
y [FlashStorage](https://github.com/cmaglie/FlashStorage/tree/634e7fd0c84120260d52ab75dd7687980941c3d9).

No hay Arduino CLI ni infraestructura de tests en este repositorio.
**Compilación con el core y pruebas físicas pendientes**; la revisión de APIs
no sustituye estas comprobaciones:

1. Compilar/subir solo Rev.3.0 y comprobar banner Metro M4/SERCOM4 en D7.
2. Detectar ICM-20948 en 0x69/0x68 y W5500 simultáneamente, verificar yaw.
3. Decodificar D7 a 38400/8N1 con analizador lógico bajo carga GNSS/I2C/TCP:
   checksum/CRLF correctos, sin descartes a 1/10 Hz ni errores GNSS.
4. Probar TCP 15919/15920, todos los comandos, clientes lentos, desconexión
   Ethernet, reintento IMU y respaldo fijo cuando DHCP no responde.
5. Guardar calibración/`yawoff`, reiniciar y comprobar lectura; repetir tras
   reprogramar para confirmar que se debe recalibrar. Verificar estados y LEDs.

### Rev.2.9: puente temporal UNO R4 WiFi, sin IMU/I2C

`RS232-RWM-GPS_V2-9.ino` parte de Rev.2.8 y permite validar TCP mientras
llegan las Metro M4. **No es la revisión final recomendada: usar Rev.3.0
para Metro M4**; ni esa revisión ni la base Rev.2.8 se modifican.

- IMU/I2C deshabilitado a propósito por el problema de detección y el bug
  del core Renesas [#543](https://github.com/arduino/ArduinoCore-renesas/issues/543).
  El código de sensor/calibración se conserva bajo `#if 0`: no hay
  `Wire.begin()`, sondeos, lecturas, reintentos ni acceso a EEPROM.
- `imuAvailable` constante `false`, yaw fijo ficticio `NAN`; ACC/MAG,
  giro, inclinación y temperatura son ceros ficticios, no mediciones.
  `status`/`imu` indican **DESHABILITADO** y `[DIAG]` muestra **IMU=OFF**.
  `magcal start/stop/reset` y `yawoff <-180..180>` permanecen en ayuda,
  pero devuelven un error de modo puente sin cambiar calibración.
- Salida COM2 **D2, GPT4, 38400/8N1** y NMEA TCP **15919** sin offset
  antena-pistón: posición GNSS cruda en MOVING/AVERAGING y promedio GNSS
  sin offset en LOCKED, conservando estados y fix quality.
- LED1/D5: apagado sin GNSS válido; parpadeo 200 ms con GNSS válido,
  incluso con HAS, como el fallback sin IMU. LED2/D6 sin cambios.
- Conserva DHCP y respaldo **192.168.1.22/24**, gateway/DNS `192.168.1.1`,
  cuatro clientes NMEA y un cliente de diagnóstico TCP **15920**.

**Arduino IDE 2.x:** seleccionar **Arduino UNO R4 WiFi** (core oficial
Arduino UNO R4 Boards) y abrir solo esta revisión en una carpeta propia
`RS232-RWM-GPS_V2-9`. Requiere Ethernet (W5500); Arduino/FspTimer vienen
con el core. No requiere instalar SparkFun ICM-20948 ni EEPROM para el
modo puente. No agrupar revisiones en un mismo sketch.

**Validación conceptual, no compilación en placa:** sin Arduino CLI ni
infraestructura de tests, se revisan sintaxis y exclusión del código IMU.
Compilación con el core y validación física pendientes:

1. Confirmar banner Rev.2.9, sin mensajes de reintento IMU ni fallo del sensor.
2. Con DHCP y sin DHCP (respaldo), conectar cuatro clientes a **15919**;
   comprobar `$GCGGA`, checksum/CRLF y frecuencia con `freq 1` / `freq 10`,
   junto con COM2 D2 a 38400/8N1.
3. Conectar a **15920**; probar `status`, `imu`, `com2`, `help`, `freq`
   y `diag on/off`. Verificar valores ficticios, `IMU=OFF`, rechazo de
   comandos IMU y reconexión del cliente de diagnóstico.
4. Confirmar ausencia de offset y conservar promedio/bloqueo/fix en
   MOVING/AVERAGING/LOCKED y LED1 de 200 ms con GNSS válido.
   La revisión no soluciona fallos de detección W5500; verificar el shield.

### Rev.2.8: inicialización I2C/IMU y respaldo Ethernet

Revisión histórica UNO R4: el cambio de orden no resolvió la detección del IMU.

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

## 1) Hardware (referencia Rev.2.x, UNO R4)

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
CONFIG SIGNALGROUP 2
CONFIG PPP ENABLE E6-HAS
SAVECONFIG
```

- La corrección de antena se aplica solo si el ICM-20948 está disponible y el yaw es válido.
- La calibración del magnetómetro y el `yawoff` se guardan en EEPROM y sobreviven reinicios.
- El WiFi Server de revisiones anteriores fue eliminado en Rev.2.4.

---

## 9) Archivos relevantes

- `RS232-RWM-GPS_V3-0.ino` — firmware actual, Metro M4
- `RS232-RWM-GPS_V2-9.ino` — puente temporal UNO R4, sin IMU/I2C
- `RS232-RWM-GPS_V2-8.ino` — base histórica UNO R4
- `RS232-RWM-GPS_V2-5.ino` — referencia histórica BLE
- `RS232-RWM-GPS_V2-4.ino` — revisión anterior (BLE, ICM-20948)
- `RS232-FMW-GPS_V-2_2.ino` / `V-2_1` / `V-2_0` / `V-1_0` — revisiones históricas
- `../../README.md` — documentación general del proyecto
- `../../CHANGELOG.md` — historial de versiones

---

## 10) Versiones

- **Rev.3.0**: Metro M4, COM2 UART hardware SERCOM4 en D7, calibración en flash
- **Rev.2.9**: puente temporal UNO R4 sin IMU/I2C para validar TCP 15919/15920
- **Rev.2.8**: base UNO R4 con cambio de orden I2C y respaldo IP fija
- **Rev.2.5**: HAS por fix 5 en GGA, LED1 fijo solo con HAS, precisión estimada por HDOP en BLE
- **Rev.2.4**: BLE (Nordic UART), ICM-20948, calibración magnetómetro, COM2 UART software, sin WiFi
- **Rev.2.2**: separación `freq` / `diag` en terminal TCP
- **Rev.2.1**: WiFi AP + control TCP interactivo
- **Rev.2**: HAS (`PUBX,00`), LEDs, 10 Hz, BNO085, Ethernet
- **Rev.1**: base funcional con promedio de coordenadas y salida FWD
