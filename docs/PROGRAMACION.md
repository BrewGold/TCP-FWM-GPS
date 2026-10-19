# Programación y validación técnica

## Alcance de esta fase

Este documento recoge decisiones y validaciones de la fase de estabilización de entrada GNSS y salida NMEA.

## TCP-IRI-GPS autónomo

La nueva aplicación está en `firmware/arduino/TCP-IRI-GPS/TCP-IRI-GPS.ino`
con su parser local `Gnss.h`; ya no utiliza el wrapper superior ni el header
compartido de FWM. Mantiene GNSS/HAS, LEDs D5/D6 (D6 apagado en modo directo),
COM2 D2 y W5500 TCP 15919/15920; no usa IMU, EEPROM, yaw, promedio ni bloqueo.
Velocidad RMC directa en m/s en diagnóstico, `N/A` cuando no está disponible.

Versiones, comandos reproducibles de compilación/carga, pruebas host ejecutadas
y límites de red se detallan en
[README del firmware](../firmware/arduino/README.md#tcp-iri-gps-movimiento-continuo).
Build RP2040 verificado con Arduino CLI 1.3.1, Arduino-Pico 5.4.3,
Ethernet 2.0.2 y FQBN `rp2040:rp2040:adafruit_metro`.
Los checks de operación física que siguen corresponden a validaciones anteriores:
**no acreditan pruebas físicas del nuevo IRI**, todavía pendientes.

## Decisiones clave

- Se mantiene checksum NMEA activo (no se deshabilita).
- Se prioriza robustez de parser frente a líneas truncadas/fragmentadas.
- Requisito de integración: salida de sentencia en **GPGGA** para Telnet/COM2.

## Reglas de parser recomendadas

1. Aceptar solo líneas:
   - no vacías,
   - que comiencen con `$`,
   - con longitud mínima razonable,
   - con `*HH` final (dos hexadecimales).
2. Descartar silenciosamente líneas malformadas.
3. Mantener validación checksum en GGA/RMC.
4. Limitar logs de error de checksum (rate-limit) para evitar flooding.

## Checklist de pruebas

- [x] Recepción GNSS continua sin corrupción visible en captura limpia.
- [x] Salida de trama en formato esperado por consumidor.
- [x] Confirmación de operación de LEDs.
- [x] Estado general funcional por parte de programación.
- [ ] Activación y convergencia HAS validada en UM980.

## Comandos útiles UM980 (referencia operativa)

```text
CONFIG SIGNALGROUP 2
CONFIG PPP ENABLE E6-HAS
SAVECONFIG
VERSIONA
```

## Criterio de cierre técnico

La parte de firmware se considera cerrada cuando:
- Se mantiene estabilidad de entrada/salida NMEA en operación continua.
- La integración remota consume correctamente GPGGA por COM2/TCP.
- Solo queda seguimiento de estado HAS dependiente de receptor/firmware/entorno GNSS.
