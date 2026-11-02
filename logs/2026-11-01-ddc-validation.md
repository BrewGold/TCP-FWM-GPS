# Validación de integración DDC — 2026-11-01

## Configuración

- Firmware: TCP-FWM-GPS Rev.4.1, IP `192.168.1.22`.
- DDC en el equipo `192.168.1.100`, conectado al flujo GPS externo TCP 15919.
- Telnet de diagnóstico TCP 15920; mensajes observados cada 5 s.
- Captura Wireshark en la interfaz Wi-Fi correcta.
- Cadencia NMEA observada: aproximadamente 10 Hz, 71 bytes por muestra.

## Resultados

- DDC recibió el GPS externo. La GGA capturada
  (`$GPGGA,120847.70,3927.5629,N,00552.5288,W,4,24,0.8,528.0,M,0.0,M,,*47`)
  tenía checksum válido y coordenadas coincidentes con la pantalla de DDC.
- La pantalla de DDC cambió de valores durante la prueba, confirmando que no
  permaneció congelada en el estado mostrado inicialmente.
- Con el filtro de incidencias para 15919 se observaron 0 coincidencias de
  ZeroWindow/retransmisiones/FIN/RST durante el primer tramo de 30.000 paquetes.
- Más adelante apareció una retransmisión TCP rápida en el frame 45689, a
  `1917.726646800 s`. Los ACK duplicados indicaron un hueco; el emisor
  retransmitió 284 bytes y el receptor confirmó hasta `Ack=1333736`. Después
  continuó la cadencia aproximada de 100 ms. La captura no permite afirmar si
  el segmento original se perdió en la red o quedó fuera de la captura.
- No se reportaron cuelgues ni pérdida persistente de GPS durante los
  escenarios observados.

## Conclusión y alcance

**Pruebas funcionales ejecutadas: superadas.** La retransmisión aislada se
recuperó sin cierre de conexión ni interrupción persistente observada. Se
mantiene la Rev.4.1; con estos resultados no se justifica una Rev.4.1.1.

Este registro no certifica una sesión indefinida ni los escenarios ampliados
de múltiples clientes, lectores lentos, reinicios/desconexiones o medición de
COM2/loop. La prueba prolongada debe registrarse por separado cuando concluya.
