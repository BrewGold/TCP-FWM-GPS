# TCP-FWM-GPS · Manual de usuario

## Para qué sirve

TCP-FWM-GPS envía la posición al programa FWM por Ethernet. Mientras
el vehículo se mueve, actualiza la posición. Al detenerse, reúne varias
lecturas y mantiene una posición estable para ese punto de medida. Cuando
el vehículo vuelve a moverse, el equipo actualiza la posición de nuevo.

## Conectar y encender

1. **Orientación.** Fija el dispositivo de dirección con **los LED apuntando
   hacia delante, en el sentido de la marcha**. Debe quedar firme y conservar
   esa orientación durante el trabajo.
2. **Antena.** Con el equipo apagado, conecta el cable al conector de antena
   GNSS. Coloca la antena donde tenga vista despejada del cielo, sin cubrirla.
   Para usar Galileo HAS, la antena debe ser compatible con Galileo E6.
3. **Red.** Conecta el cable Ethernet a la red del ordenador que ejecuta FWM
   o directamente al ordenador si así está preparada la instalación.
4. **Alimentación.** Conecta el cable de alimentación al puerto **USB-C** del
   equipo. Los dos LED parpadean durante unos cinco segundos al arrancar.
   Interpreta sus indicaciones cuando termine esa secuencia.

## Usar con FWM

Abre el cliente TCP que utiliza FWM. Si ya está configurado, usa la conexión
guardada. Si tienes que configurarlo, introduce **la IP del TCP-FWM-GPS** y
el puerto **15919**. Usa la IP asignada por la red; si no se asigna ninguna,
el equipo usa `192.168.1.22`. En ese caso, el ordenador debe estar en la misma
subred con una dirección distinta.

Espera a que el LED verde indique que hay posición. El equipo puede entregar una
posición sin HAS; si el trabajo requiere esa corrección, espera a que el
**LED verde quede fijo**. HAS puede tardar en estar disponible, incluso con cielo
despejado.

Al llegar a un punto de medida, detén el vehículo. El **LED rojo parpadea**
mientras el equipo promedia la posición durante unos 15 segundos. Cuando
queda **fijo**, la posición de ese punto está estabilizada. Al reanudar la
marcha, el LED rojo se apaga y la posición vuelve a actualizarse.

## Qué indican los LED

| Indicador | Señal | Significado |
|---|---|---|
| **LED verde · Posición** | Apagado | Aún no hay una posición GNSS válida. |
| | Parpadeo rápido | Hay posición, pero el dispositivo no dispone de la lectura de dirección. |
| | Parpadeo lento | Hay posición y lectura de dirección, pero HAS aún no está activo. |
| | Encendido fijo | Hay posición y lectura de dirección, y HAS está activo. |
| **LED rojo · Punto de medida** | Apagado | La posición no está estabilizada; es el estado normal en marcha. |
| | Parpadeo | El equipo está promediando la posición tras detenerse. |
| | Encendido fijo | El equipo mantiene una posición estable para ese punto. |

## Si algo no funciona

| Qué ocurre | Qué hacer |
|---|---|
| Los LED no se encienden al conectar el equipo | Revisa que el cable USB-C esté bien conectado y que la fuente de alimentación esté encendida. |
| El LED verde sigue apagado | Revisa el cable de la antena, despeja su vista del cielo y deja tiempo para que el receptor obtenga posición. |
| El LED verde parpadea rápido de forma persistente | Revisa la conexión del dispositivo de dirección. Si no es accesible o el parpadeo continúa, contacta con el responsable del equipo. |
| El LED verde parpadea lento y necesitas HAS | Deja el vehículo con vista despejada del cielo y comprueba que la antena admite Galileo E6. Si se borró la configuración del receptor, sigue el apartado de recuperación. |
| El LED rojo no queda fijo en el punto de medida | Comprueba que el LED verde indique que hay posición y mantén el vehículo detenido hasta que termine el promedio. |
| Hay posición, pero FWM no la recibe | Revisa el cable Ethernet y que el cliente TCP esté conectado a la IP del equipo por el puerto **15919**. Si se desconectó el cable, vuelve a conectar el cliente. |

## Recuperar la configuración del UM980 y Galileo HAS

Hazlo **solo si se borró la configuración** del receptor ArduSimple
simpleRTK3B Budget (UM980) o si se sustituyó por uno sin configurar. No
restaures los valores de fábrica: no hace falta borrar nada para volver a
guardar los ajustes.

1. Conecta un cable USB de datos desde el ordenador al puerto **USB GPS**
   del receptor. Para este paso usa el puerto del receptor, no el USB-C
   de alimentación del TCP-FWM-GPS. Si no puedes acceder a USB GPS sin abrir
   el equipo, pide ayuda al responsable de la instalación.
2. Abre una terminal serie, selecciona el puerto que aparece al conectar el
   receptor y elige **115200 baudios, 8 bits, sin paridad, 1 bit de parada
   (8N1)** y fin de línea **CR/LF**. Envía `VERSIONA`: el receptor debe
   responder con su información de versión.
3. Envía estos comandos **uno por uno**. Espera `response: OK` antes de enviar
   el siguiente. El último guarda los ajustes de salida de posición:

   ```text
   GNGGA COM1 0.1
   GPRMC COM1 0.1
   GNGGA COM3 0.1
   GPRMC COM3 0.1
   SAVECONFIG
   ```

4. Envía `CONFIG SIGNALGROUP 2`. Si el receptor se reinicia, espera a que
   vuelva a responder antes de seguir.
5. Activa Galileo HAS y guarda los cambios:

   ```text
   CONFIG PPP ENABLE E6-HAS
   SAVECONFIG
   ```

6. Vuelve al uso normal con la antena conectada y el equipo al aire libre.
   El LED verde indica primero si hay posición y queda fijo cuando HAS está
   activo y la lectura de dirección está disponible.

Si un comando da error, detente y anota el comando y la respuesta para el
responsable del equipo.
