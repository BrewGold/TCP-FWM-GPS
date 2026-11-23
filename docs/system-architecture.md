# Arquitectura del sistema

Estado vigente a 2026-11-17. La selección de versiones, compilación y pruebas
se encuentra en el [README del firmware](../firmware/arduino/README.md);
la puesta en marcha del receptor, en [Configuración del UM980](CONFIGURACION_UM980.md).

## Plataformas y aplicaciones

El hardware actual utiliza el receptor ArduSimple simpleRTK3B Budget (UM980),
una placa Adafruit Metro RP2040 y un shield Ethernet W5500. El repositorio
contiene dos aplicaciones autónomas:

| Aplicación | Firmware | Función | Estado |
|---|---|---|---|
| TCP-FWM-GPS | `firmware/arduino/RS232-RWM-GPS_V4-1/` | Flujo FWM con IMU, offset, promedio/bloqueo y salida GNSS | Pruebas realizadas satisfactorias |
| TCP-IRI-GPS | [Anexo](../anexos/TCP-IRI-GPS/README.md) | Posición GNSS directa para movimiento continuo, sin IMU ni promedio/bloqueo | Trabajo y validación física pendientes |

No combinar las dos aplicaciones: cada una tiene su propio sketch de entrada,
parser y comportamiento.

## Conexiones principales

```text
simpleRTK3B Budget / UM980
  COM1 USB GPS: configuración y consola
  TX del puerto GNSS ───────────────► Metro RP2040 D0 / Serial1 RX
  GND ──────────────────────────────► GND común

Metro RP2040
  D2 / COM2 TX ──► interfaz RS232 ─► Dynatest FWD
  W5500 ─────────► TCP NMEA :15919 y diagnóstico :15920
  D5 ────────────► LED1
  D6 ────────────► LED2
  SDA/SCL ───────► ICM-20948 (FWM únicamente)
```

La entrada GNSS es `Serial1` a **115200 baudios, 8N1**. La Rev.4.1 espera
**D0 como RX y D1 como TX**: comprobar el switch RX/TX de la placa/shield.
Conectar TX del receptor a RX del Metro y mantener masa común. COM2 sale por
D2 a **38400 baudios, 8N1** mediante UART PIO y requiere adaptación de nivel
RS232 para el Dynatest.

## Red y sentencias

- TCP NMEA: puerto **15919**; requiere un cliente conectado para recibir tramas.
- TCP diagnóstico: puerto **15920**; comandos y estado del equipo.
- La salida NMEA del firmware actual es **`$GPGGA`**, con checksum y CRLF.
- El receptor entrega GGA y RMC por el UART seleccionado. Con fix autónomo 1
  ya se genera salida; no se requiere fix 2 ni HAS para la salida básica.
- El firmware descarta la posición si deja de recibir GGA válido dentro del
  plazo de frescura configurado.

## Diferencias de comportamiento

### TCP-FWM-GPS Rev.4.1

- Procesa GNSS, IMU ICM-20948, movimiento, promedio/bloqueo y offset
  antena-pistón.
- GGA con fix válido (incluido fix 1) puede alimentar COM2 y TCP NMEA.
- LED1/D5 indica GNSS/HAS; LED2/D6 indica los estados de movimiento y bloqueo.
- Galileo HAS con la configuración UM980 documentada ha sido probado y
  converge correctamente. La salida básica no depende de HAS.

### TCP-IRI-GPS

- Usa GNSS directamente y conserva el fix recibido en la GGA de salida.
- No usa IMU, EEPROM de calibración, yaw, offset, promedio ni bloqueo.
- Mantiene COM2, TCP NMEA/diagnóstico y la señalización LED descrita en su
  documentación de firmware.
- La implementación y pruebas host están descritas en el [anexo](../anexos/TCP-IRI-GPS/README.md);
  la validación física sigue pendiente.

## Referencias históricas

Los documentos de arquitectura Rev.2.x describen UNO R4 WiFi, BLE, BNO085 o
WiFi AP y no deben usarse como referencia de cableado para Metro RP2040 Rev.4.1.
Consulta el [índice de documentación](README.md) para localizar manuales y
material histórico.
