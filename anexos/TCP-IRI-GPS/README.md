# TCP-IRI-GPS: anexo de movimiento continuo

Este anexo reúne el firmware, las pruebas host y sus dobles de hardware.
Es una aplicación independiente de TCP-FWM-GPS, no una revisión del firmware
FWM. Su desarrollo y validación física siguen pendientes.

Abrir **`firmware/TCP-IRI-GPS/TCP-IRI-GPS.ino`**: carpeta Arduino autónoma con implementación
real de `setup`/`loop`, LEDs, COM2 PIO y W5500 en el sketch, y parser/estado GNSS y
generación NMEA en su archivo local `Gnss.h`. Se puede copiar **solo esa carpeta**.
No incluye `shared/`, otro `.ino` ni un selector `TCP_IRI_GPS`.
El antiguo wrapper `firmware/arduino/TCP-IRI-GPS.ino` se eliminó anteriormente
para evitar dos entradas distintas. El firmware FWM se documenta por separado en
su [README](../../firmware/arduino/README.md).

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
ANEXO="$(pwd)" # ejecutar desde la carpeta TCP-IRI-GPS del anexo
INDEX=https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli core update-index --additional-urls "$INDEX"
arduino-cli core install rp2040:rp2040@5.4.3 --additional-urls "$INDEX"
arduino-cli lib install "Ethernet@2.0.2"
arduino-cli board details --fqbn rp2040:rp2040:adafruit_metro
arduino-cli compile --fqbn rp2040:rp2040:adafruit_metro --warnings all \
  "$ANEXO/firmware/TCP-IRI-GPS"
arduino-cli board list
# Sustituir /dev/ttyACM0 por el puerto real; en primera carga usar BOOTSEL si hace falta.
arduino-cli upload --fqbn rp2040:rp2040:adafruit_metro --port /dev/ttyACM0 \
  "$ANEXO/firmware/TCP-IRI-GPS"
```

Cableado inalterado: UM980 Serial1 D0 RX/D1 TX a 115200; COM2/RS232 D2 TX-only
PIO a 38400/8N1 hacia el transceptor del shield; W5500 por SPI ICSP, CS D10 y
SD deseleccionada en D4. TCP NMEA 15919 (hasta 4 clientes), diagnóstico 15920
(1 cliente), DHCP con respaldo `192.168.1.22/24`, gateway/DNS `192.168.1.1`.
Salida `$GPGGA` con checksum y CRLF, 10 Hz por defecto, `freq 1..10` configurable.
Conserva recuperación Ethernet cooperativa y vaciado incremental del FIFO PIO.
No cambia talker, precisión decimal ni formato heredado de salida.

En Arduino IDE abrir la carpeta autónoma IRI con `Gnss.h` presente.
Para volver a FWM compilar/cargar la carpeta independiente de Rev.4.1,
o el sketch histórico monolítico según las instrucciones de Rev.4.0.

### Validación de IRI

Validado originalmente (2026-10-19): compilación **real RP2040** con las versiones
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
  "$ANEXO/firmware/TCP-IRI-GPS"
```

Pruebas host reproducibles desde la carpeta del anexo, sin bibliotecas de test
adicionales:

```sh
ANEXO="$(pwd)"
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer \
  -DARDUINO_ADAFRUIT_METRO_RP2040 -DARDUINO_ARCH_RP2040 \
  -I"$ANEXO/tests/stubs" "$ANEXO/tests/TcpIriGpsTest.cpp" \
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

También pasó la suite existente del cliente (.NET 8), incluidos sus casos IRI
y sesiones TCP fragmentadas. Esta prueba opcional requiere el repositorio
principal y se ejecuta desde la carpeta del anexo con:

```sh
dotnet run --project ../../tools/DiagTelnetWin/Tests/DiagTelnetWin.Tests.csproj
```

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
