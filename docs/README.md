# Índice de documentación

Este índice separa la documentación vigente del material histórico. Para el
estado del proyecto y la selección del firmware, empieza en el
[README principal](../README.md).

## Guías vigentes

| Tema | Documento | Uso |
|---|---|---|
| Manual de usuario TCP-FWM-GPS | [Manual de instalación y uso](Manual_usuario_TCP-FWM-GPS.md) | Montaje, configuración UM980/HAS, carga del firmware y operación Rev.4.1 |
| Arquitectura | [Arquitectura del sistema](system-architecture.md) | Componentes, conexiones y diferencias entre FWM e IRI |
| UM980 / RTK3B Budget | [Configuración del UM980](CONFIGURACION_UM980.md) | Restauración, NMEA por COM1/COM3, Galileo E6-HAS y persistencia |
| Desarrollo y validación | [Programación](PROGRAMACION.md) | Pruebas, estado de validación y referencias para desarrolladores |
| Firmware | [README del firmware](../firmware/arduino/README.md) | Selección, compilación, carga y pruebas de FWM e IRI |
| Diagnóstico Windows | [DiagTelnetWin](../tools/DiagTelnetWin/README.md) | Cliente del puerto TCP de diagnóstico 15920 |
| Instalación Dynatest | [Programa FWM](../firmware/Programa_FWM/README.md) | Instaladores externos y orden de instalación |

## Materiales y referencias de hardware

Las hojas de datos y esquemas son referencias de componentes, no instrucciones
de montaje por sí solas.

| Material | Archivo |
|---|---|
| Adafruit Metro RP2040 (ABX00087) | [Hoja de datos](ABX00087-datasheet.pdf) |
| RP2040 | [Hoja de datos](RP-008371-DS-1-rp2040-datasheet.pdf) |
| Arduino Ethernet Shield 2 | [Esquema](arduino-Ethernet-Shield2-V2-sch.pdf) |
| SparkFun ICM-20948 | [Hoja de datos](icm-20948-9dof-imu-breakout-qwiic-datasheet.pdf) |
| Dynatest FWD | [Manual del equipo](Manual%20DynaTest%20FWD.pdf) |

## Manuales y especificaciones históricas

Estos documentos describen revisiones anteriores y se conservan como
referencia; no sustituyen las guías vigentes de arriba.

| Documento | Alcance / limitación |
|---|---|
| [Manual operativo Rev.2.2](Manual_operativo_RTK3B_Budget_ESP32_FWD_WordFriendly.md) | Arduino UNO R4 WiFi, BNO085 y WiFi; no es el procedimiento de la Rev.4.1 Metro RP2040 |
| [Manual técnico TCP-FW-GPS](Manual%20técnico%20de%20usuario%20TCP-FW-GPS_01.pdf) | Manual de una versión anterior; confirmar modelo y cableado con la arquitectura vigente |
| [Especificación funcional Rev.2.5](functional-spec-v1.0.md) | Conserva requisitos de BLE/UNO R4; incluye además el diseño original v1.0 como apéndice |
| [Guía BLE Rev.2.5](ble-app-guide.md) | Aplicable únicamente a firmware antiguo con BLE |
| [Configuración GNSS Rev.2.7](CONFIGURATION_RTK3B_GNSS.md) | Notas heredadas; sus estados, declinación y ejemplo BLE no representan la Rev.4.1 |
| [Changelog V2-5](CHANGELOG_V2-5.md) | Registro histórico; el estado actual está en el [changelog principal](../CHANGELOG.md) |
| [Registro de sesión 2026-10-13](../logs/2026-10-13-session.md) | Apunte de sesión, no especificación vigente |

## Software y validación

- **TCP-FWM-GPS Rev.4.1** (`firmware/arduino/RS232-RWM-GPS_V4-1/`) es el
  firmware FWM actual para Metro RP2040; las pruebas realizadas fueron
  satisfactorias y Galileo HAS converge con la configuración vigente.
- **TCP-IRI-GPS** (`firmware/arduino/TCP-IRI-GPS/`) es una aplicación
  independiente; continuar su desarrollo y validación física está pendiente.
- Las pruebas host y los comandos de compilación/carga se documentan en el
  [README del firmware](../firmware/arduino/README.md).
- Las pruebas host están en `firmware/tests/`; el código antiguo se conserva
  en `firmware/arduino/Obsoleto/` y los prototipos BLE en
  `firmware/arduino/FWD_GPS_Diag/` y `firmware/arduino/FWM_GPS_R0/`.
- La utilidad Windows actual es [DiagTelnetWin](../tools/DiagTelnetWin/README.md).

No usar una prueba host ni una revisión histórica como evidencia de validación
física de IRI o como instrucción de carga para Rev.4.1.
