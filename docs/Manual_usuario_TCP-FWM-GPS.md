# Manual de usuario de TCP-FWM-GPS

**Versión del manual:** 1.0  
**Firmware descrito:** TCP-FWM-GPS Rev.4.1, Adafruit Metro RP2040  
**Fecha:** 2026-11-19

Este manual cubre el montaje, la configuración del receptor ArduSimple simpleRTK3B Budget (UM980), la instalación del firmware y el uso diario de TCP-FWM-GPS con un equipo Dynatest FWD. La guía corresponde a la aplicación **FWM**; no seguirla para TCP-IRI-GPS ni para placas/revisiones históricas UNO R4 o Metro M4. Para detalles técnicos de compilación y transporte, consultar el [README del firmware](../firmware/arduino/README.md).

## 1. Qué hace el sistema

TCP-FWM-GPS recibe sentencias NMEA GGA/RMC del UM980, utiliza el ICM-20948 para orientar la corrección del offset entre la antena y el pistón, promedia la posición cuando el equipo se detiene y proporciona salida al FWD. La salida NMEA es **`$GPGGA`**, con checksum y terminación CRLF, a 10 Hz de forma predeterminada.

La salida está disponible en:

- **COM2:** TX del Metro RP2040 por D2, 38400 baudios, 8N1, hacia el Dynatest mediante un adaptador de nivel RS232.
- **TCP NMEA:** puerto **15919**, hasta cuatro clientes.
- **TCP diagnóstico:** puerto **15920**, un cliente de diagnóstico a la vez.

La Ethernet usa DHCP y, si no obtiene una dirección, la IP de respaldo es `192.168.1.22/24` (gateway/DNS `192.168.1.1`). No se requiere HAS para usar la salida básica: un fix GNSS autónomo válido (fix 1) puede producir GGA. HAS mejora la solución cuando converge y está disponible.

## 2. Material y precauciones

Se requiere una Adafruit Metro RP2040, receptor simpleRTK3B Budget con UM980 y antena GNSS compatible, shield W5500, IMU SparkFun ICM-20948, transceptor RS232 (por ejemplo MAX3232), indicadores LED con resistencias y el cableado de integración con el FWD.

> **Importante:** la lógica del Metro RP2040 es de 3.3 V y no tolera 5 V. Verificar niveles eléctricos y pull-ups antes de conectar señales. No conectar una señal RS232 directamente al pin D2; debe pasar por el transceptor/adaptador apropiado. Apagar los equipos antes de cambiar el cableado y mantener una masa común en las conexiones UART que la requieran.

### Conexiones de la Rev.4.1

| Función | Conexión Metro/shield | Parámetros |
|---|---|---|
| GNSS hacia el Metro | `Serial1`: D0 = RX, D1 = TX. TX del UART UM980 conectado a D0; GND común. | 115200 baudios, 8N1 |
| Switch RX/TX del shield | Posición que conecta D0 del shield a RX y D1 a TX. | Confirmar la posición antes de diagnosticar ausencia de GNSS. |
| Salida COM2 al FWD | D2 = TX-only, a través del transceptor RS232. | 38400 baudios, 8N1 |
| IMU | ICM-20948 a SDA/SCL. | I2C; necesario para yaw y offset. |
| LEDs | LED1 a D5; LED2 a D6. | Con resistencias apropiadas. |
| Ethernet | Shield W5500 por SPI ICSP, CS en D10. SD CS D4 deseleccionado (HIGH). | Cable Ethernet a la red local. |

No cambiar el pin COM2, el switch RX/TX ni el cableado del shield por instrucciones de revisiones anteriores. El detalle de arquitectura se encuentra en [Arquitectura del sistema](system-architecture.md).

## 3. Configurar el UM980 y Galileo HAS

La configuración se realiza en la consola del receptor, **no** en el puerto de diagnóstico del Metro.

1. Conectar el puerto USB GPS/COM1 del simpleRTK3B Budget al ordenador. Abrir una terminal serie a **115200 baudios, 8N1**, con terminación CR/LF. Enviar `VERSIONA` y comprobar que el receptor responde.
2. Si se necesita borrar una configuración previa, enviar `FRESET` y esperar a que el receptor reinicie y vuelva a responder. **Es opcional y destructivo:** borra la configuración guardada, efemérides y posición.
3. Configurar y guardar NMEA a 10 Hz en COM1 y COM3. Enviar una línea por vez y verificar `response: OK`:

   ```text
   GNGGA COM1 0.1
   GPRMC COM1 0.1
   GNGGA COM3 0.1
   GPRMC COM3 0.1
   SAVECONFIG
   ```

   `0.1` equivale a 10 mensajes por segundo. Asegurarse de que el UART que entrega NMEA al Metro sea el que está conectado físicamente a `Serial1`.
4. Seleccionar el grupo de señales Galileo E6-HAS y esperar a que el receptor lo aplique (puede reiniciarse):

   ```text
   CONFIG SIGNALGROUP 2
   ```
5. Una vez que el receptor vuelva a responder, habilitar PPP por HAS y guardar:

   ```text
   CONFIG PPP ENABLE E6-HAS
   SAVECONFIG
   ```

6. Comprobar que salen GGA y RMC por el UART conectado al Metro. Conectar TX del receptor a D0/RX del Metro y compartir GND. La guía detallada de comandos y precauciones está en [Configuración del UM980](CONFIGURACION_UM980.md).

La convergencia de HAS depende de una antena compatible con Galileo E6, cielo despejado, firmware del receptor compatible y tiempo de convergencia. La salida GNSS autónoma sigue funcionando mientras HAS está inactivo; no esperar HAS para el uso básico.

## 4. Instalar/cargar el firmware del Metro

El repositorio contiene el código fuente, no un instalador binario del firmware. Para compilar desde fuente se requiere Arduino IDE 2.x o Arduino CLI, el core **Arduino-Pico de Earle Philhower**, la placa **Adafruit Metro RP2040**, y estas bibliotecas:

- Ethernet **2.0.2**.
- SparkFun 9DoF IMU Breakout - ICM 20948 Arduino Library **1.3.2**.

`SerialPIO`, `EEPROM`, `Wire` y `SPI` vienen con el core. No seleccionar Arduino Mbed, UNO R4 ni Metro M4.

### Arduino IDE

1. En Preferencias del IDE, añadir esta URL de índice de placas adicionales:
   `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
2. En el Gestor de placas, instalar Raspberry Pi Pico/RP2040/RP2350 de Earle F. Philhower (versión usada para Rev.4.1: **5.4.3**), y en el selector de placa elegir **Adafruit Metro RP2040**.
3. En el Gestor de bibliotecas instalar Ethernet **2.0.2** y SparkFun ICM-20948 **1.3.2**.
4. Abrir `firmware/arduino/RS232-RWM-GPS_V4-1/RS232-RWM-GPS_V4-1.ino`. Mantener junto al sketch los archivos `TcpTransport.h` y `W5500Transport.h` de esa misma carpeta; no mover ni combinar sketches de otras revisiones.
5. Compilar y, con el Metro conectado por USB, cargar el programa a la placa. Si la primera carga no encuentra el dispositivo, usar el procedimiento BOOTSEL de la placa y repetir la carga.

### Arduino CLI (alternativa)

Desde la raíz del repositorio, con Arduino CLI 1.3.1 instalado:

```sh
INDEX=https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli core update-index --additional-urls "$INDEX"
arduino-cli core install rp2040:rp2040@5.4.3 --additional-urls "$INDEX"
arduino-cli lib install "Ethernet@2.0.2"
arduino-cli lib install "SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library@1.3.2"
arduino-cli compile --fqbn rp2040:rp2040:adafruit_metro --warnings all \
  "$PWD/firmware/arduino/RS232-RWM-GPS_V4-1"
arduino-cli board list
# Reemplazar /dev/ttyACM0 por el puerto real de la placa.
arduino-cli upload --fqbn rp2040:rp2040:adafruit_metro --port /dev/ttyACM0 \
  "$PWD/firmware/arduino/RS232-RWM-GPS_V4-1"
```

La selección de versión y los pasos técnicos de compilación están en el [README del firmware](../firmware/arduino/README.md#compilación-rev41).

## 5. Instalar el software Dynatest

Los instaladores de Dynatest son externos y **no se distribuyen en este repositorio**. Si el ordenador del FWD los necesita, obtenerlos del proveedor o responsable del proyecto; no descargar ejecutables de fuentes no autorizadas. Cerrar Dynatest Data Collection antes de instalar y seguir este orden:

1. Ejecutar como administrador **Dynatest Data Collection Prerequisites Installer 2.1.0.246** y reiniciar si lo solicita.
2. Ejecutar como administrador **Dynatest Data Collection Installer 2.4.1**.
3. Reiniciar el ordenador y abrir Dynatest Data Collection.

La referencia mantenida está en [Instaladores Dynatest](../firmware/Programa_FWM/README.md).

## 6. Puesta en marcha y operación

1. Revisar cableado, adaptador RS232, switch RX/TX y alimentación antes de energizar.
2. Encender el UM980, el Metro y el shield. Durante el inicio ambos LEDs ejecutan una secuencia luminosa de aproximadamente cinco segundos.
3. En la consola USB del Metro (115200 baudios), consultar los mensajes de inicio para conocer el estado del GNSS, IMU, Ethernet y la dirección IP asignada por DHCP o la de respaldo.
4. El equipo puede operar con una solución GNSS válida; no es necesario esperar HAS para la salida básica.
5. Conectar el Dynatest a COM2 según la interfaz instalada. Para recibir NMEA o consultar el diagnóstico por Ethernet, conectar un PC a la misma red y usar la IP del equipo:
   - NMEA: cliente TCP a `<IP_DEL_METRO>:15919`.
   - Diagnóstico: cliente TCP a `<IP_DEL_METRO>:15920`.
6. El cliente TCP NMEA debe mantenerse conectado para recibir tramas.

Si se usa la IP de respaldo `192.168.1.22`, el PC debe tener conectividad en `192.168.1.0/24`; coordinar con el administrador de la red para evitar direcciones duplicadas. No exponer los puertos TCP a Internet: no ofrecen cifrado ni autenticación.

### Estados e indicadores

| Indicador | Señal | Significado |
|---|---|---|
| LED1 | D5 | Apagado: sin GNSS válido. Parpadeo rápido (~200 ms): GNSS válido, IMU no disponible. Parpadeo lento (~600 ms): GNSS e IMU disponibles, HAS inactivo. Fijo: HAS activo (fix de entrada 5). |
| LED2 | D6 | Apagado: `MOVING`; parpadeo (~400 ms): `AVERAGING`; fijo: `LOCKED`, posición estabilizada. |

Los estados de procesamiento son `MOVING` (posición GNSS actual), `AVERAGING` (acumulación durante la parada) y `LOCKED` (posición promediada y corregida). En la salida Rev.4.1, el fix es 4 en `LOCKED`; en movimiento, HAS de entrada se mapea a fix de salida 2 y los demás fixes válidos a 1. Por ello, consultar el fix de entrada en `status` para distinguir el servicio HAS recibido.

## 7. Diagnóstico y comandos

Conectar un solo cliente al puerto TCP **15920** y enviar comandos terminados en CR/LF. `help` lista la ayuda del firmware.

| Comando | Uso |
|---|---|
| `status` | Estado GNSS, fix/HAS, movimiento, IMU, Ethernet y contadores. |
| `imu` | Estado del ICM-20948, yaw y calibración. |
| `com2` | Estado y contadores de la salida COM2. |
| `freq 1..10` | Cambia la frecuencia de salida entre 1 y 10 Hz (10 Hz predeterminado). |
| `diag on` / `diag off` | Activa/desactiva el diagnóstico periódico. |
| `magcal start` / `magcal stop` | Inicia/finaliza la calibración del magnetómetro; `stop` valida y guarda si es válida. |
| `magcal reset` | Borra la calibración guardada. |
| `yawoff <grados>` | Ajusta el offset de montaje entre -180 y 180 grados. |
| `debug hex on` / `debug hex off` | Inicia o finaliza una captura diagnóstica temporal de bytes GNSS. |
| `help` | Muestra comandos disponibles. |

La calibración magnética solo debe hacerse con el montaje instalado y siguiendo el procedimiento de servicio del proyecto: orientar lentamente el equipo en distintas direcciones durante la captura, finalizar con `magcal stop` y consultar `imu`. Si la calibración no es válida, el firmware informa el error; no usar una calibración fallida para trabajo de campo. Guardar cambios solo cuando sea necesario.

`debug hex on` captura bytes de entrada durante un periodo corto y luego informa el volcado por el diagnóstico; interrumpe temporalmente parte del diagnóstico periódico, no es una salida NMEA. No iniciar otra captura mientras una está activa o se está enviando.

Como alternativa en Windows, [DiagTelnetWin](../tools/DiagTelnetWin/README.md) se conecta a `<IP_DEL_METRO>:15920` y consulta `status`; no consume COM2 ni el puerto NMEA. La aplicación es solo de lectura y no reemplaza los comandos manuales. Su modo portable requiere extraer el ZIP y ejecutar `DiagTelnetWin.exe`.

## 8. Solución de problemas

| Síntoma | Comprobaciones recomendadas |
|---|---|
| `GNSS` ausente o fix 0 | Verificar alimentación/antena y vista de cielo; confirmar GGA/RMC en el UART correcto a 115200, switch RX/TX D0=RX/D1=TX, TX UM980 a D0 y GND común. Confirmar `response: OK` en la configuración UM980. |
| HAS no aparece | Verificar que se ejecutaron y guardaron los comandos Galileo E6-HAS, que la antena admite E6 y que hay cielo despejado; esperar convergencia. El fix autónomo puede operar sin HAS. |
| IMU no disponible | Revisar alimentación y SDA/SCL del ICM-20948, niveles/pull-ups de 3.3 V y comando `imu`. LED1 rápido indica GNSS válido sin IMU. |
| No hay IP/servidor TCP | Revisar cable y enlace W5500, DHCP/subred y mensajes USB. Si se usa la IP de respaldo, conectar el PC a la misma subred y comprobar que no haya conflicto. Volver a conectar los clientes después de recuperar el enlace. |
| NMEA no llega por TCP | Verificar IP correcta, puerto **15919**, cliente conectado y GNSS válido; el puerto **15920** es diagnóstico, no NMEA. |
| FWD no recibe datos | Verificar COM2 por D2, 38400/8N1, adaptador RS232 entre Metro y Dynatest y masa/cableado. No conectar RS232 directamente al GPIO. Consultar `com2`. |
| No se puede cargar firmware | Confirmar placa Metro RP2040 con core Arduino-Pico de Earle Philhower, las versiones de bibliotecas y que los dos headers de Rev.4.1 permanecen junto al sketch. |

