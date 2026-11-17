# Desarrollo y validación

Este documento resume el estado técnico actual y enlaza a las instrucciones
reproducibles. La documentación está organizada en el
[índice de docs](README.md).

## Estado de validación (2026-11-17)

- **TCP-FWM-GPS Rev.4.1:** todas las pruebas realizadas fueron satisfactorias.
  La recepción GNSS con fix 1, salida COM2 y NMEA por TCP fueron verificadas.
- **Galileo HAS:** probado con la configuración UM980 vigente y converge
  correctamente.
- **TCP-IRI-GPS:** el firmware y las pruebas host están documentados; el trabajo
  y la validación física siguen pendientes.

## Aplicaciones

FWM e IRI son aplicaciones independientes. Selección de sketch, dependencias,
compilación/carga y comandos de pruebas están en el
[README del firmware Arduino](../firmware/arduino/README.md).

| Aplicación | Ruta | Alcance |
|---|---|---|
| TCP-FWM-GPS Rev.4.1 | `firmware/arduino/RS232-RWM-GPS_V4-1/` | FWM con IMU, offset, promedio/bloqueo y transporte TCP cooperativo |
| TCP-IRI-GPS | `firmware/arduino/TCP-IRI-GPS/` | GNSS directo de movimiento continuo, sin IMU ni promedio/bloqueo |

No utilizar las notas de pruebas históricas Rev.2.x como evidencia de
validación física de IRI o como instrucción de carga para Rev.4.1.

## Reglas de integración GNSS

- El Metro recibe UART a 115200 baudios, 8N1, por `Serial1` (D0 RX/D1 TX).
- Confirmar switch RX/TX, TX del UM980 a RX del Metro y GND común.
- Fix 1 es válido para generar salida; no esperar fix 2 para probar COM2/TCP.
- La salida NMEA actual es `$GPGGA`, checksum y CRLF.
- El TCP NMEA requiere un cliente conectado al puerto 15919; el diagnóstico
  está en el puerto 15920.
- Mantener la validación del checksum. No desactivar validación para ocultar
  un problema de cableado o de configuración del receptor.

## Configuración del receptor

Usar la [guía vigente del UM980](CONFIGURACION_UM980.md), que cubre FRESET,
salida GGA/RMC en COM1 y COM3, grupo de señales Galileo E6 y PPP E6-HAS.
Las instrucciones de configuración contenidas en documentos Rev.2.x son
históricas y no sustituyen esa guía.

## Regresión y pruebas

Ejecutar los comandos y pruebas documentados en el README del firmware para la
aplicación que se modifica. Los cambios de transporte deben conservar la
integridad de las tramas NMEA y el aislamiento de clientes TCP. No marcar como
prueba física un resultado de compilación o de pruebas host.

## Documentación histórica

- [Changelog V2-5](CHANGELOG_V2-5.md): notas de una revisión anterior.
- [Especificación funcional Rev.2.5](functional-spec-v1.0.md): requisitos
  heredados de UNO R4/BLE.
- [Guía BLE](ble-app-guide.md): solo para firmware BLE antiguo.
