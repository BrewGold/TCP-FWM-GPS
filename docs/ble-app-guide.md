# Guía BLE para MIT App Inventor

> Aplica a firmware **Rev.2.5** (`RS232-RWM-GPS_V2-5.ino`). Guía para construir una app Android de diagnóstico con MIT App Inventor que se comunica con el equipo por Bluetooth Low Energy.

## 1. Requisitos

- [MIT App Inventor](https://appinventor.mit.edu/) (cuenta gratuita)
- Extensión **BluetoothLE** (`edu.mit.appinventor.ble`): descargar desde [MIT App Inventor Extensions](http://appinventor.mit.edu/extensions/)
- Android 6+ (recomendado Android 10+)
- Equipo FWD-GPS encendido con firmware Rev.2.4 o superior

## 2. Datos BLE del equipo

| Parámetro | Valor |
|---|---|
| Nombre del dispositivo | `FWD-GPS-Diag` |
| Servicio (Nordic UART) | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| Característica RX (app → Arduino, escribir comandos) | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` |
| Característica TX (Arduino → app, notificaciones) | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` |
| Tamaño de chunk TX | 20 bytes |
| Terminador de comandos | Opcional (acepta con o sin CR/LF) |

El firmware envía los textos troceados en fragmentos de 20 bytes: la app debe **concatenar** los fragmentos recibidos.

## 3. Comandos disponibles

| Comando | Función |
|---|---|
| `status` | Estado general completo (GNSS, fix, HAS, HDOP, precisión estimada, IMU, COM2, Ethernet) |
| `imu` | Datos del ICM-20948 (ACC/MAG/GYRO, roll/pitch, rumbos, calibración) |
| `com2` | Estado y contadores de la salida COM2 |
| `freq <1-10>` | Ajusta la frecuencia GCGGA compartida por COM2 y Ethernet (Hz enteros, solo Rev.2.5 actualizada) |
| `magcal start` | Inicia calibración del magnetómetro |
| `magcal stop` | Finaliza, valida y guarda la calibración en EEPROM |
| `magcal reset` | Borra la calibración |
| `yawoff <-180..180>` | Ajuste del offset de montaje del yaw |
| `help` | Lista de comandos |

Además, cada 5 s el equipo envía automáticamente un bloque `[DIAG]` con el estado resumido (requiere notificaciones activas).

Por ejemplo, enviar `freq 5` devuelve `OK: freq=5 Hz (200 ms), COM2/Ethernet`.
Los valores inválidos devuelven `ERROR` y conservan el período anterior.
El ajuste vuelve a 10 Hz al reiniciar y no cambia la frecuencia del GNSS ni del diagnóstico BLE.

## 4. Diseño de pantalla sugerido (Designer)

Componentes:

- **BluetoothLE1** (extensión, componente no visible)
- **Clock1** (no visible, para timeout de escaneo)
- `ListView1` — dispositivos encontrados
- `btnScan` — "Buscar"
- `btnConnect` — "Conectar" / `btnDisconnect` — "Desconectar"
- `lblStatus` — estado de conexión
- Fila de botones de comandos: `btnStatus`, `btnIMU`, `btnCOM2`, `btnHelp`
- Fila de calibración: `btnCalStart`, `btnCalStop`, `btnCalReset`
- `txtYawoff` + `btnYawoff` — ajuste de yaw
- `txtTerminal` — TextBox multilínea (ReadOnly) o Label dentro de un `VerticalScrollArrangement` — consola de recepción
- `btnClear` — limpiar consola

## 5. Lógica de bloques (Blocks)

### 5.1 Escaneo y conexión

```text
Cuando btnScan.Click:
  → llamar BluetoothLE1.StartScanning
  → lblStatus.Text = "Escaneando..."

Cuando BluetoothLE1.DeviceFound:
  → ListView1.Elements = BluetoothLE1.DeviceList

Cuando btnConnect.Click:
  → llamar BluetoothLE1.StopScanning
  → llamar BluetoothLE1.ConnectWithAddress
      address: (dirección extraída del elemento seleccionado en ListView1)

Cuando BluetoothLE1.Connected:
  → lblStatus.Text = "Conectado"
  → llamar BluetoothLE1.RegisterForStrings
      serviceUuid:        6E400001-B5A3-F393-E0A9-E50E24DCCA9E
      characteristicUuid: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
      utf16: false

Cuando BluetoothLE1.Disconnected:
  → lblStatus.Text = "Desconectado"
```

> Nota: en `DeviceList` cada elemento tiene formato `dirección nombre rssi`. Puedes filtrar por nombre `FWD-GPS-Diag` o seleccionar manualmente de la lista.

### 5.2 Recepción (consola)

```text
Cuando BluetoothLE1.StringsReceived (serviceUuid, characteristicUuid, stringValues):
  → para cada valor en la lista stringValues:
      txtTerminal.Text = join(txtTerminal.Text, valor)
```

Los fragmentos llegan de 20 bytes e incluyen ya los saltos de línea `\r\n`, por lo que basta con concatenar. Cada mensaje largo termina con una línea de `====...====`, que puede usarse como delimitador si quieres separar respuestas.

### 5.3 Envío de comandos

Crear un procedimiento reutilizable:

```text
Procedimiento enviarComando(cmd):
  → llamar BluetoothLE1.WriteStrings
      serviceUuid:        6E400001-B5A3-F393-E0A9-E50E24DCCA9E
      characteristicUuid: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
      utf16: false
      values: crear lista(cmd)
```

Y conectarlo a cada botón:

```text
Cuando btnStatus.Click:    → enviarComando("status")
Cuando btnIMU.Click:       → enviarComando("imu")
Cuando btnCOM2.Click:      → enviarComando("com2")
Cuando btnHelp.Click:      → enviarComando("help")
Cuando btnCalStart.Click:  → enviarComando("magcal start")
Cuando btnCalStop.Click:   → enviarComando("magcal stop")
Cuando btnCalReset.Click:  → enviarComando("magcal reset")
Cuando btnYawoff.Click:    → enviarComando(join("yawoff ", txtYawoff.Text))
Cuando btnClear.Click:     → txtTerminal.Text = ""
```

No es necesario añadir `\n` al final: el firmware procesa el paquete completo como comando.

## 6. Permisos Android

En Android 12+ la extensión requiere permisos de ejecución:

- `BLUETOOTH_SCAN`
- `BLUETOOTH_CONNECT`
- En Android 6–11: también **Ubicación** (requisito del sistema para escaneo BLE)

App Inventor los solicita automáticamente al usar la extensión; el usuario debe aceptarlos. Si el escaneo no encuentra nada, verificar que la ubicación del teléfono esté activada (Android ≤ 11).

## 7. Flujo de uso típico

1. Abrir la app → **Buscar** → seleccionar `FWD-GPS-Diag` → **Conectar**.
2. Al conectar, el equipo envía el mensaje de bienvenida y cada 5 s el bloque `[DIAG]`.
3. Pulsar **Status** para ver el estado completo, incluida la precisión estimada (`Precision est`).
4. Calibración del magnetómetro:
   - Pulsar **MagCal Start**.
   - Girar el equipo lentamente en todos los ejes (mínimo 200 muestras).
   - Pulsar **MagCal Stop** → confirma offsets guardados.
5. Ajustar `yawoff` si el yaw mostrado difiere del rumbo real del vehículo.

## 8. Resolución de problemas

| Problema | Causa probable | Solución |
|---|---|---|
| No aparece el dispositivo al escanear | Ubicación desactivada (Android ≤ 11) o ya hay otro central conectado | Activar ubicación; cerrar otras apps BLE (el equipo solo admite una conexión) |
| Conecta pero no llega texto | Notificaciones no registradas | Verificar `RegisterForStrings` en el evento `Connected`, con los UUID de TX |
| Texto entrecortado o mezclado | Comportamiento normal (chunks de 20 bytes) | Concatenar siempre en el mismo orden de llegada |
| Comando no responde | Comando mal escrito | Enviar `help`; los comandos son en minúsculas |
| Se desconecta al salir de la app | Android suspende BLE en segundo plano | Mantener la app en primer plano durante la calibración |

## 9. Referencias

- Firmware: `firmware/arduino/RS232-RWM-GPS_V2-5.ino`
- Documentación BLE del firmware: `firmware/arduino/README.md` (sección 6)
- Extensión BluetoothLE: http://appinventor.mit.edu/extensions/
- Nordic UART Service (NUS): especificación de Nordic Semiconductor
