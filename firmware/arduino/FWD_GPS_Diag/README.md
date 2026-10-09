# FWD_GPS_Diag

> **Aplicación histórica/deprecada:** compatible con el firmware Rev.2.4/2.5. Desde Rev.2.6 el firmware no incluye BLE; usar el diagnóstico TCP en el puerto 15920. Esta app y su proyecto se conservan como referencia.

Aplicación Android desarrollada con MIT App Inventor para diagnóstico y control del sistema FWD-GPS por Bluetooth Low Energy (BLE).

## Función
La app permite:

- buscar el dispositivo BLE
- conectarse al equipo
- recibir mensajes de estado
- enviar comandos de control
- ajustar el valor `yawoff`
- consultar información de la IMU

## Dispositivo BLE
- Nombre: `FWD-GPS-Diag`
- Servicio: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- RX: `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
- TX: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

## Comandos disponibles
- `status`
- `imu`
- `com2`
- `magcal start`
- `magcal stop`
- `magcal reset`
- `yawoff <valor>`
- `help`

## Archivo principal
- `FWD_GPS_Diag.aia`

## Uso
1. Importar el proyecto en MIT App Inventor.
2. Abrir el archivo `.aia`.
3. Compilar o exportar APK.
4. Instalar en Android.
5. Conectarse al dispositivo BLE `FWD-GPS-Diag`.

## Notas
- El firmware puede enviar respuestas fragmentadas en bloques de 20 bytes.
- La aplicación debe concatenar los mensajes recibidos.
- `yawoff` es un ajuste fino de orientación, no una calibración magnética.
