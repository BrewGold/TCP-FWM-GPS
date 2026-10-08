MANUAL OPERATIVO
simpleRTK3B Budget + UM980 + Arduino UNO R4 WiFi + FWD GGA + Ethernet + WiFi TCP + Galileo HAS

Versión: 2.2
Fecha: 28/09/2026
Preparado para: operación técnica, pruebas de campo y validación
Firmware de referencia: firmware/arduino/legacy/RS232-FMW-GPS_V-2_2.ino

====================================================

1. OBJETIVO
-----------

Implementar y validar un sistema GNSS para Dynatest FWD donde:

- El receptor simpleRTK3B Budget / UM980 entrega GNSS al Arduino UNO R4 WiFi.
- El Arduino recibe GGA, RMC y PUBX,00 por Serial1 a 115200 baudios.
- El estado HAS se observa mediante PUBX,00 cuando el firmware del UM980 lo proporciona.
- El Arduino procesa posición, velocidad, altitud, satélites, HDOP y rumbo.
- El Arduino detecta movimiento, parada, promedio y posición bloqueada.
- El Arduino aplica la corrección antena-pistón mediante BNO085 cuando el IMU está disponible.
- El Arduino genera GGA para el FWD por D2 a 38400 baudios.
- La misma GGA se transmite simultáneamente por Ethernet.
- El WiFi AP permite consultar el estado y ajustar parámetros mediante TCP.
- El diagnóstico TCP es independiente de la frecuencia de salida GGA y se envía cada 5 s por defecto.

La salida de posicionamiento y el diagnóstico son funciones separadas:

- `freq` modifica la frecuencia de salida GGA/FWD.
- `diag` modifica la periodicidad del diagnóstico TCP.
- `status` devuelve el estado inmediatamente.
- `help` muestra los comandos disponibles inmediatamente.

Importante:

- El texto exacto de los mensajes HAS depende del firmware del UM980.
- No debe asumirse un estado final como `PPP_VALID` sin una captura real del receptor.
- Las líneas de diagnóstico HAS deben conservarse durante la prueba de campo.
- El firmware debe mantener la salida GGA aunque el estado HAS no sea reconocido.

====================================================

2. HARDWARE Y MAPA DE PUERTOS
-----------------------------

2.1 Receptor simpleRTK3B Budget / UM980

| Interfaz | Puerto UM980 | Uso |
|---|---|---|
| USB GPS | COM1 | Configuración, comandos y diagnóstico local |
| XBee / TX2-RX2 | COM2 | Monitor USB2 y retorno GGA del Arduino |
| Pixhawk / TX3-RX3 | COM3 | GNSS hacia el Arduino |

2.2 Arduino UNO R4 WiFi

| Función | Puerto/pin | Parámetros |
|---|---|---|
| Entrada GNSS | Serial1, D0/D1 | 115200 baudios |
| Salida FWD | D2, TX software | 38400 baudios, TX-only |
| LED1 | D5 | GNSS + IMU + HAS |
| LED2 | D6 | Movimiento / bloqueo |
| BNO085 | I2C SDA/SCL | 100 kHz, dirección 0x4B |
| Ethernet CS | D10 | W5500 |
| SD desactivada | D4 | Chip select en HIGH |
| WiFi diagnóstico | AP integrado | TCP 192.168.4.1:15920 |

2.3 Ethernet

- Shield: W5500.
- Destino configurado: `192.168.1.122:15919`.
- La GGA se transmite por Ethernet cuando el enlace está disponible.

2.4 Conexiones principales

- TX3 del UM980 -> RX de Serial1 del Arduino.
- GND del UM980 -> GND del Arduino.
- D2 del Arduino -> entrada del circuito de salida FWD / MAX3232.
- Salida del MAX3232 -> entrada RS232 del Dynatest.
- BNO085 -> SDA, SCL, 3V3 y GND según el módulo utilizado.

Regla UART:

- TX del origen -> RX del destino.
- Mantener masa común entre los equipos.
- No conectar directamente una salida TTL del Arduino a una entrada RS232 del Dynatest sin MAX3232.

====================================================

3. ARQUITECTURA DE SEÑAL
------------------------

Flujo principal:

1) UM980 COM3 TX3 -> Arduino Serial1.
2) El Arduino recibe GGA, RMC y PUBX,00.
3) El Arduino actualiza GNSS, velocidad, HAS, IMU y máquina de estados.
4) En movimiento transmite la posición instantánea corregida si hay yaw válido.
5) En parada acumula muestras durante la ventana de promedio.
6) Al completar el promedio cambia a `LOCKED`.
7) El Arduino genera GGA y la transmite por D2.
8) La misma GGA se transmite por Ethernet.
9) El WiFi AP entrega diagnóstico cada 5 s por defecto.
10) El terminal TCP acepta cambios de configuración en tiempo real.

El Dynatest recibe únicamente GGA. No se deben reenviar al Dynatest:

- RMC.
- PUBX,00.
- PPPNAVA.
- BESTNAVA.
- Mensajes de diagnóstico TCP.

====================================================

4. REQUISITOS PREVIOS
---------------------

- Antena GNSS conectada antes de energizar.
- Cielo abierto y buena visibilidad de satélites.
- Alimentación estable para UM980, Arduino, BNO085 y MAX3232.
- GND común entre los equipos.
- Firmware del UM980 compatible con la función HAS requerida.
- Arduino UNO R4 WiFi con firmware Rev.2.2.
- Librerías instaladas:
  - `Adafruit_BNO08x`
  - `Ethernet`
  - `WiFiS3`
- Monitor serie configurado a 115200, 8N1 y CR+LF para COM1.
- Cliente TCP en Android o PC.

Para comprobar HAS:

- No activar NTRIP/RTCM externo durante la prueba inicial.
- Mantener la antena fija.
- No apagar el receptor durante la convergencia.
- Guardar `VERSIONA`, `UNILOGLIST`, `PPPNAVA` y `BESTNAVA`.

====================================================

5. CONFIGURACIÓN DEL UM980
---------------------------

Conectarse por USB GPS / COM1, normalmente a 115200, 8N1 y CR+LF.
Enviar los comandos uno a uno y comprobar la respuesta del receptor.

5.1 Mensajes necesarios en COM3

```text
GPGGA COM3 1
GPRMC COM3 1
```

Si el equipo utiliza VTG para velocidad y rumbo, activar también:

```text
GPVTG COM3 1
```

5.2 Diagnóstico HAS en COM3

```text
PPPNAVA COM3 1
BESTNAVA COM3 1
```

Si la sintaxis con puerto no es aceptada, probar la forma indicada por el firmware:

```text
PPPNAVA 1
BESTNAVA 1
```

Confirmar después con:

```text
UNILOGLIST
```

El Arduino debe conservar las líneas HAS como diagnóstico. La salida FWD debe seguir siendo únicamente GGA.

5.3 Diagnóstico HAS en COM1

```text
GPGGA COM1 1
GPGSA COM1 1
GPGSV COM1 1
GPGST COM1 1
GPRMC COM1 1
PPPNAVA COM1 1
BESTNAVA COM1 1
```

5.4 Limpiar COM2

Para evitar que el monitor USB2 mezcle mensajes propios del UM980 con la GGA del Arduino:

```text
GPGGA COM2 0
GPGSA COM2 0
GPGSV COM2 0
GPGST COM2 0
GPRMC COM2 0
GPVTG COM2 0
PPPNAVA COM2 0
BESTNAVA COM2 0
```

Resultado esperado:

- COM2 sin NMEA propio del UM980.
- USB2 mostrando principalmente la GGA inyectada por el Arduino.

5.5 Activar HAS

Utilizar la secuencia compatible con el firmware instalado en el UM980. Como referencia de la configuración utilizada en este proyecto:

```text
CONFIG PPP ENABLE E6-HAS
CONFIG PPP DATUM WGS84
CONFIG PPP CONVERGE 50 50
CONFIG SIGNALGROUP 2
SAVECONFIG
```

Para desactivar posteriormente PPP/HAS:

```text
CONFIG PPP DISABLE
SAVECONFIG
```

La sintaxis exacta debe confirmarse con la documentación del firmware instalado y con `UNILOGLIST`.

====================================================

6. SECUENCIA DE CONFIGURACIÓN COMPLETA
---------------------------------------

Conectado a COM1 / USB GPS, enviar una línea cada vez:

```text
GPGGA COM3 1
GPRMC COM3 1
GPVTG COM3 1
PPPNAVA COM3 1
BESTNAVA COM3 1

GPGGA COM1 1
GPGSA COM1 1
GPGSV COM1 1
GPGST COM1 1
GPRMC COM1 1
PPPNAVA COM1 1
BESTNAVA COM1 1

GPGGA COM2 0
GPGSA COM2 0
GPGSV COM2 0
GPGST COM2 0
GPRMC COM2 0
GPVTG COM2 0
PPPNAVA COM2 0
BESTNAVA COM2 0

CONFIG PPP ENABLE E6-HAS
CONFIG PPP DATUM WGS84
CONFIG PPP CONVERGE 50 50
CONFIG SIGNALGROUP 2
SAVECONFIG
UNILOGLIST
```

Guardar la respuesta de `UNILOGLIST`. La respuesta puede variar según el firmware del UM980.

====================================================

7. CARGA Y ARRANQUE DEL FIRMWARE
--------------------------------

Firmware de referencia:

```text
firmware/arduino/legacy/RS232-FMW-GPS_V-2_2.ino
```

Antes de cargar:

- Seleccionar la placa Arduino UNO R4 WiFi.
- Confirmar la librería `Adafruit_BNO08x`.
- Confirmar la librería `Ethernet`.
- Confirmar la librería `WiFiS3`.
- Desconectar temporalmente cualquier equipo que pueda interferir con D0/D1 durante la carga.

Al arrancar:

- D5 y D6 parpadean durante aproximadamente 5 s.
- El monitor USB muestra la identificación Rev.2.2.
- El Arduino inicializa GNSS, BNO085, Ethernet y WiFi AP.
- El AP aparece como `FWD-GPS-Diag`.

Parámetros WiFi:

```text
SSID:       FWD-GPS-Diag
Contraseña: 12345678
IP AP:      192.168.4.1
TCP:        15920
```

====================================================

8. TERMINAL TCP REV.2.2
-----------------------

Conectar el teléfono o PC al AP:

```text
FWD-GPS-Diag
```

Abrir un cliente TCP y conectar a:

```text
192.168.4.1:15920
```

Al conectar se recibe:

```text
=== FWD-GPS Diagnostico Rev.2.2 ===
Escribe 'help' para ver comandos
```

8.1 Comandos

```text
freq <1-10>        Salida GGA/FWD en Hz
speed_stop <0-1>   Umbral de parada en m/s
speed_move <0-1>   Umbral de movimiento en m/s
offset <0-2>       Offset antena-pistón en m
decl <-180-180>    Declinación magnética en grados
avg <5-60>         Ventana de promedio en segundos
diag <1-60>        Intervalo del diagnóstico TCP en segundos
status             Estado actual inmediato
help               Lista de comandos
```

8.2 Separación entre salida y diagnóstico

`freq` solo modifica el periodo de salida GGA por D2 y Ethernet.

- `freq 10` -> GGA cada 100 ms.
- `freq 5` -> GGA cada 200 ms.
- `freq 1` -> GGA cada 1000 ms.

El diagnóstico TCP no cambia con `freq`.

`diag` modifica únicamente el intervalo del diagnóstico:

- Por defecto: 5 s.
- `diag 10` -> diagnóstico cada 10 s.
- `diag 1` -> diagnóstico cada 1 s.
- `diag 60` -> diagnóstico cada 60 s.

`status` y `help` responden inmediatamente.

8.3 Ejemplos

```text
freq 5
OK: salida GGA cambiada a 5 Hz (200 ms)

diag 10
OK: diagnóstico cada 10 segundos

offset 0.60
OK: Offset antena = 0.60 m

avg 20
OK: Ventana promedio = 20 segundos

status
```

8.4 Diagnóstico periódico

Por defecto se recibe cada 5 s una línea con información similar a:

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

El diagnóstico permite comprobar:

- que el equipo sigue ejecutándose (`uptime`);
- si el GNSS es válido;
- fix quality, satélites y HDOP;
- posición y altitud;
- velocidad y rumbo;
- solución detectada y estado HAS;
- disponibilidad del BNO085;
- estado `MOVING`, `AVERAGING` o `LOCKED`;
- existencia de una posición bloqueada;
- frecuencia GGA, ventana de promedio, offset y declinación.

====================================================

9. LEDS Y ESTADOS
-----------------

9.1 LED1 en D5: GNSS + IMU + HAS

| Estado | Indicación |
|---|---|
| GNSS no válido | Apagado |
| GNSS válido, IMU no disponible | Parpadeo rápido, 200 ms |
| GNSS + IMU, HAS no activo | Parpadeo medio, 600 ms |
| GNSS + IMU + HAS, esperando bloqueo | Parpadeo lento, 1000 ms |
| GNSS + IMU + HAS + posición bloqueada | Encendido fijo |

9.2 LED2 en D6: movimiento

| Estado | Indicación |
|---|---|
| `MOVING` | Apagado |
| `AVERAGING` | Parpadeo, 400 ms |
| `LOCKED` | Encendido fijo |

====================================================

10. MÁQUINA DE ESTADOS
----------------------

10.1 MOVING

El sistema transmite la posición instantánea. Si el BNO085 está disponible y proporciona yaw, se aplica la corrección antena-pistón.

10.2 AVERAGING

Se entra cuando la velocidad se mantiene por debajo del umbral de parada durante aproximadamente 2 s.

Durante esta fase:

- se acumulan muestras de latitud, longitud y altitud;
- se acumulan muestras de yaw si el IMU está disponible;
- se aplica una media recortada a las coordenadas;
- se calcula una media circular del yaw.

10.3 LOCKED

Al finalizar la ventana de promedio:

- se calcula la posición bloqueada;
- se aplica el offset antena-pistón;
- la salida GGA utiliza la posición bloqueada;
- el sistema vuelve a `MOVING` si supera el umbral de movimiento o la distancia de re-lock.

Valores por defecto:

- Umbral de entrada a parada: `0.20 m/s`.
- Umbral de salida por movimiento: `0.30 m/s`.
- Confirmación de parada: `2 s`.
- Ventana de promedio: `15 s`.
- Distancia de re-lock: `1.0 m`.
- Offset antena-pistón: `0.55 m`.
- Declinación magnética: `1.0°`.

====================================================

11. CAPTURA Y VALIDACIÓN DE HAS
--------------------------------

La captura real del UM980 es obligatoria antes de cerrar una clasificación definitiva del estado HAS.

11.1 Captura de convergencia

En COM1 o COM3 guardar 10-20 líneas completas consecutivas que incluyan, si aparecen:

```text
#PPPNAVA,...
#BESTNAVA,...
```

Conservar checksums y no modificar las líneas.

11.2 Captura estable

Tras dejar el receptor con cielo abierto el tiempo suficiente, guardar otras 10-20 líneas del estado posterior.

No asumir que el estado estable se denomina `PPP_VALID`. La clasificación debe basarse en el texto real del UM980.

11.3 Información que debe acompañar la captura

- `VERSIONA`.
- `UNILOGLIST`.
- versión/build del UM980.
- fecha y hora.
- ubicación aproximada.
- presencia o ausencia de NTRIP/RTCM.
- antena y condiciones de cielo.

Estados de trabajo recomendados:

- `SIN_PPP`: no hay confirmación de PPP/HAS.
- `PPP_CONVERGING`: el texto recibido indica convergencia.
- `PPP_ESTABLE`: confirmado por captura real.
- `PPP_DESCONOCIDO`: se recibe información HAS, pero el literal no coincide con los estados conocidos.

====================================================

12. SALIDAS Y MONITORES
-----------------------

| Monitor | Contenido esperado | Objetivo |
|---|---|---|
| COM1 / USB GPS | comandos, respuestas, NMEA y HAS | configurar y observar UM980 |
| COM3 / TX3 | GGA, RMC/VTG y diagnóstico HAS | alimentar Arduino |
| USB Serial Arduino | logs GNSS, IMU, estados y TX | depuración detallada |
| WiFi TCP | diagnóstico resumido cada 5 s | saber si funciona y estado actual |
| USB2 / COM2 | GGA del Arduino sin mezcla | validar retorno |
| Dynatest / RS232 | solo GGA a 38400 | validar entrada FWD |
| Ethernet | GGA hacia 192.168.1.122:15919 | salida adicional |

La salida WiFi TCP no sustituye a la salida GGA profesional del FWD. Es un canal de diagnóstico y configuración.

====================================================

13. PROCEDIMIENTO DE VALIDACIÓN
-------------------------------

1. Verificar antena, alimentación, masa común y cielo abierto.
2. Configurar COM1 a 115200, 8N1 y CR+LF.
3. Ejecutar `VERSIONA` y `UNILOGLIST`; guardar las respuestas.
4. Configurar COM3 con GGA y RMC/VTG.
5. Activar PPPNAVA y BESTNAVA en COM1 y COM3 si se va a probar HAS.
6. Limpiar COM2 para evitar mensajes propios del UM980.
7. Activar HAS según la configuración compatible con el firmware instalado.
8. Cargar `legacy/RS232-FMW-GPS_V-2_2.ino` en el Arduino UNO R4 WiFi.
9. Confirmar la prueba de LEDs de arranque.
10. Confirmar que aparece el AP `FWD-GPS-Diag`.
11. Conectar al TCP `192.168.4.1:15920`.
12. Ejecutar `help` y `status`.
13. Confirmar un diagnóstico periódico `[DIAG]` cada 5 s.
14. En movimiento, confirmar `STATE=MOVING`.
15. Detener el vehículo y confirmar `STATE=AVERAGING`.
16. Esperar la ventana de promedio y confirmar `STATE=LOCKED`.
17. Confirmar GGA por D2 al periodo configurado.
18. Confirmar GGA por Ethernet si el enlace está disponible.
19. Confirmar que Dynatest recibe solo GGA a 38400 baudios.
20. Confirmar que no se aplica dos veces el offset.
21. Guardar capturas de campo y logs.

Criterios de aceptación:

- GNSS válido recibido por Serial1.
- GGA válida emitida por D2.
- Diagnóstico TCP visible cada 5 s por defecto.
- `freq` no modifica la periodicidad del diagnóstico.
- `diag` no modifica la frecuencia GGA.
- `status` responde inmediatamente.
- La máquina MOVING/AVERAGING/LOCKED funciona.
- USB2 no presenta mezcla del NMEA propio del UM980.
- Dynatest recibe únicamente GGA.

====================================================

14. TROUBLESHOOTING
-------------------

Caso A: el Arduino no recibe GNSS

- Revisar TX3 -> RX Serial1.
- Confirmar 115200 baudios.
- Revisar GND común.
- Confirmar que COM3 emite GGA.

Caso B: no aparece HAS

- Ejecutar `VERSIONA` y `UNILOGLIST`.
- Confirmar que el firmware del UM980 soporta la configuración utilizada.
- Activar PPPNAVA/BESTNAVA en COM1 y COM3.
- Mantener la antena fija y con cielo abierto.
- No declarar `PPP_ESTABLE` sin captura real.

Caso C: USB2 muestra mensajes mezclados

- Desactivar GGA, RMC, VTG, PPPNAVA y BESTNAVA de COM2.
- Ejecutar `SAVECONFIG`.
- Confirmar que la GGA del Arduino entra por RX2.

Caso D: el diagnóstico TCP llega demasiado rápido

- Ejecutar `diag 5` o `diag 10`.
- Recordar que `freq` solo controla la GGA.

Caso E: la salida GGA llega a una frecuencia incorrecta

- Ejecutar `status`.
- Confirmar `Frecuencia salida GGA`.
- Ejecutar `freq 10` para volver a 10 Hz.

Caso F: no entra en AVERAGING

- Confirmar que llega RMC o VTG válido.
- Revisar `SPEED` en el diagnóstico TCP.
- Revisar `speed_stop` y `speed_move`.
- Confirmar que el vehículo permanece detenido durante 2 s.

Caso G: no se aplica corrección de antena

- Revisar `IMU` y `YAW` en el diagnóstico TCP.
- Confirmar conexión I2C del BNO085.
- Confirmar que `offset` tiene el valor esperado.

Caso H: Dynatest no recibe datos

- Confirmar MAX3232 y cruce TX/RX.
- Confirmar 38400, 8N1.
- Confirmar GGA válida y checksum.
- No conectar D2 directamente a una entrada RS232.

====================================================

15. BLOQUES RÁPIDOS
--------------------

15.1 Producción mínima sin HAS

```text
GPGGA COM3 1
GPRMC COM3 1
GPVTG COM3 1
GPGGA COM2 0
GPGSA COM2 0
GPGSV COM2 0
GPGST COM2 0
GPRMC COM2 0
GPVTG COM2 0
PPPNAVA COM2 0
BESTNAVA COM2 0
SAVECONFIG
```

15.2 Producción con HAS visible en COM1 y COM3

```text
GPGGA COM3 1
GPRMC COM3 1
GPVTG COM3 1
PPPNAVA COM3 1
BESTNAVA COM3 1
GPGGA COM1 1
GPGSA COM1 1
GPGSV COM1 1
GPGST COM1 1
GPRMC COM1 1
PPPNAVA COM1 1
BESTNAVA COM1 1
GPGGA COM2 0
GPGSA COM2 0
GPGSV COM2 0
GPGST COM2 0
GPRMC COM2 0
GPVTG COM2 0
PPPNAVA COM2 0
BESTNAVA COM2 0
CONFIG PPP ENABLE E6-HAS
CONFIG PPP DATUM WGS84
CONFIG PPP CONVERGE 50 50
CONFIG SIGNALGROUP 2
SAVECONFIG
UNILOGLIST
```

15.3 Configuración inicial del terminal TCP

```text
freq 10
diag 5
status
help
```

15.4 Ajuste de campo de ejemplo

```text
speed_stop 0.20
speed_move 0.30
offset 0.55
decl 1.0
avg 15
freq 10
diag 5
status
```

15.5 Desactivar diagnósticos HAS después de capturar

```text
PPPNAVA COM1 0
BESTNAVA COM1 0
PPPNAVA COM3 0
BESTNAVA COM3 0
SAVECONFIG
```

====================================================

16. CHECKLIST DE CAMPO
----------------------

Datos de ensayo:

- Fecha: __________________________
- Técnico: _______________________
- Ubicación: _____________________
- Firmware Arduino: ______________
- Firmware/build UM980: __________
- Baud COM1: _____________________
- Baud COM3: _____________________
- Baud COM2: _____________________

Pre-check:

[ ] Antena conectada antes de energizar
[ ] Cielo abierto suficiente
[ ] Alimentación estable
[ ] GND común confirmado
[ ] COM1 accesible por USB GPS
[ ] Firmware/build UM980 registrado
[ ] Sin NTRIP/RTCM externo durante la prueba HAS

Configuración UM980:

[ ] COM3 con GGA
[ ] COM3 con RMC o VTG
[ ] PPPNAVA activo en COM1, si aplica
[ ] BESTNAVA activo en COM1, si aplica
[ ] PPPNAVA activo en COM3, si aplica
[ ] BESTNAVA activo en COM3, si aplica
[ ] COM2 sin NMEA propio
[ ] COM2 sin PPPNAVA/BESTNAVA
[ ] HAS configurado
[ ] SAVECONFIG ejecutado
[ ] VERSIONA guardado
[ ] UNILOGLIST guardado

Arduino Rev.2.2:

[ ] Firmware cargado
[ ] Prueba de LEDs completada
[ ] AP `FWD-GPS-Diag` visible
[ ] TCP `192.168.4.1:15920` accesible
[ ] `help` responde
[ ] `status` responde
[ ] Diagnóstico `[DIAG]` llega cada 5 s
[ ] `freq` controla salida GGA
[ ] `diag` controla diagnóstico independientemente

Validación funcional:

[ ] GNSS válido recibido
[ ] Estado MOVING confirmado
[ ] Estado AVERAGING confirmado
[ ] Estado LOCKED confirmado
[ ] GGA presente en D2
[ ] GGA presente en Ethernet, si aplica
[ ] USB2 limpio
[ ] Dynatest recibe GGA a 38400
[ ] BNO085 operativo, si se requiere offset
[ ] No hay doble aplicación del offset
[ ] Capturas HAS guardadas, si aplica

Resultado:

[ ] APROBADO
[ ] OBSERVADO
[ ] PENDIENTE DE REPETICIÓN

Observaciones:

____________________________________________________
____________________________________________________
____________________________________________________

Firmas:

Técnico: __________________________ Fecha: ___/___/____

Supervisor: _______________________ Fecha: ___/___/____
