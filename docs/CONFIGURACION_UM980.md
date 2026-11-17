# Configuración del ArduSimple simpleRTK3B Budget (UM980)

Procedimiento probado para el UM980 con firmware R4.10Build20739, usado con
TCP-FWM-GPS. Comandos en la consola del receptor, no en el diagnóstico TCP del
Metro. Enviar una línea cada vez y comprobar `response: OK`.

## Conexión a la consola

- Conectar el puerto **USB GPS / COM1** del simpleRTK3B Budget al ordenador.
- Terminal: **115200 baudios, 8N1**; habilitar terminación CR/LF si la terminal
  lo requiere.
- Para comprobar el modelo y firmware, enviar `VERSIONA`.

## Restauración opcional de fábrica

Solo si se desea borrar la configuración existente:

```text
FRESET
```

Esperar a que el receptor indique que se reinicia y vuelva a aparecer en la
terminal. `FRESET` borra configuración guardada, efemérides y posición, y
restablece la comunicación a 115200 baudios.

## Configurar NMEA a 10 Hz en COM1 y COM3

```text
GNGGA COM1 0.1
GPRMC COM1 0.1
GNGGA COM3 0.1
GPRMC COM3 0.1
SAVECONFIG
```

`0.1` equivale a 10 mensajes por segundo. Guardar esta configuración antes de
cambiar el grupo de señales, que puede reiniciar el receptor.

## Activar Galileo E6-HAS

```text
CONFIG SIGNALGROUP 2
```

Esperar unos segundos a que aplique el grupo y, si el receptor reinicia,
esperar a que vuelva a conectarse. Después habilitar PPP por Galileo HAS y
guardar de nuevo:

```text
CONFIG PPP ENABLE E6-HAS
SAVECONFIG
```

Esta configuración fue probada y Galileo HAS converge correctamente. La
convergencia depende de disponer de antena compatible con Galileo E6, cielo
despejado, firmware compatible y tiempo suficiente; la salida GNSS autónoma
con fix 1 funciona sin HAS.

## Verificación y conexión al Metro

1. Consultar `VERSIONA` y confirmar que el receptor responde.
2. Comprobar que salen `$GNGGA` y `$GNRMC` por el puerto UART utilizado.
3. El Metro RP2040 espera UART a **115200, 8N1**, con **D0 como RX** y D1 como
   TX. Conectar TX del puerto UM980 al RX del Metro y compartir GND.
4. Verificar el switch RX/TX de la placa/shield. El switch en posición
   incorrecta puede dejar GNSS sin recepción aunque el UM980 emita NMEA por USB.
5. En diagnóstico TCP del Metro (`:15920`), `status` debe mostrar `GNSS=OK`
   y el fix recibido. El puerto TCP NMEA es `:15919`; conectarse a él para
   recibir las tramas de salida.

No es necesario esperar a fix 2 para probar la salida del Metro: fix 1 es
válido para la conversión y salida básica por COM2/TCP.
