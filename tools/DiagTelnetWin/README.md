# Diagnóstico TCP en Windows por Telnet (puerto 15920, sin COM2)

Aplicación independiente WinForms, .NET 8, con interfaz en español. Consulta
únicamente el endpoint de diagnóstico Telnet/TCP mediante `status\r\n`.
No modifica el firmware, el IDE ni la configuración del receptor; no envía
`diag`, `freq`, `com2` ni comandos de configuración.
El servidor TCP NMEA usa el puerto `15919`; esta aplicación consulta el
servidor TCP de diagnóstico en el puerto `15920`.

**COM2 NO se consume por esta aplicación. COM2 permanece para diagnóstico
directo del usuario.** No se abre ningún puerto serie, no se conecta al servidor
TCP-NMEA y no se interpreta NMEA. El propio texto de `status` puede mencionar
contadores COM2: son texto recibido por Telnet, no una lectura de COM2.
No se incluye panel web ni instalador.

## Compilar y ejecutar

Requisitos: Windows 10/11 y SDK de .NET 8. Desde la raíz del repositorio:

```powershell
dotnet build tools/DiagTelnetWin/DiagTelnetWin.csproj -c Release
dotnet run --project tools/DiagTelnetWin/DiagTelnetWin.csproj -c Release
```

Para generar un ejecutable autónomo para Windows x64, sin requerir .NET instalado
en el ordenador destino:

```powershell
dotnet publish tools/DiagTelnetWin/DiagTelnetWin.csproj -c Release -r win-x64 --self-contained true -p:PublishSingleFile=true
```

Ejecutar `DiagTelnetWin.exe` en
`tools/DiagTelnetWin/bin/Release/net8.0-windows/win-x64/publish/`.
Para Windows ARM64, sustituir `win-x64` por `win-arm64`.
La compilación cruzada es posible; la interfaz solo se ejecuta en Windows.

## Empaquetado portable (ZIP)

Con el SDK de .NET 8 instalado, ejecutar con doble clic
`tools/DiagTelnetWin/package-win.bat` en Windows. Publica la versión `v1.0.0`
en modo Release, autónoma y single-file, sin símbolos de depuración.

Desde la raíz del repositorio, los resultados quedan en:

- Carpeta portable: `dist/DiagTelnetWin-win-x64/`.
- ZIP para distribución: `dist/DiagTelnetWin-win-x64.zip`.

Extraer todo el ZIP y ejecutar `DiagTelnetWin.exe`; no requiere instalación
ni .NET en el equipo destino. El paquete incluye instrucciones de conexión
y `VERSION.txt`. Si falla la restauración o publicación, el script aborta
sin empaquetar archivos antiguos. No añadir las salidas generadas al repositorio.

Para ARM64 o una versión distinta, ejecutar en PowerShell:

```powershell
.\tools\DiagTelnetWin\package-win.ps1 -Runtime win-arm64 -Configuration Release -Version v1.0.0
```

## Configuración de ejemplo

Introducir en la ventana (no se necesita archivo de configuración):

| Campo | Ejemplo / valor inicial |
| --- | --- |
| IP del dispositivo | `192.168.1.22` |
| Puerto Telnet de diagnóstico | `15920` |
| Intervalo de consulta | `1000` ms |

Usar la IP real asignada al dispositivo; `192.168.1.22` es la dirección de
respaldo Ethernet de Rev.4, no una IP garantizada. En las versiones con
punto de acceso WiFi, el ejemplo es `192.168.4.1:15920`.
**No seleccionar `15919`: es el servidor NMEA, no el de diagnóstico.**
Se aceptan direcciones IPv4/IPv6, puertos 1–65535 e intervalos 250–60000 ms.
La configuración se mantiene solo durante la sesión de la aplicación.

Pulsar **Conectar** para iniciar las consultas y **Desconectar** para detenerlas.
Los ajustes quedan bloqueados mientras se conecta o consulta. No hay reconexión
automática: después de una desconexión, revisar la red y pulsar Conectar.

## Interpretación y alertas

El panel muestra GNSS, FIX_IN, HAS, solución, conexión y edad de la última
**respuesta completa válida** recibida por la aplicación. Esta edad mide
frescura de la telemetría Telnet, **no** antigüedad de la última medida GNSS:
el firmware actual no publica esa edad en `status`.

### Dashboard

- Cinco tarjetas muestran **GNSS / FIX_IN / HAS / Tipo de solución / Telnet**.
  El semáforo combina color y texto: verde **Correcto**, amarillo **Atención**
  y rojo **Alarma**, sin depender exclusivamente del color.
- GNSS correcto es verde y fallo es rojo. FIX_IN `0` es rojo, `4` (RTK fijo)
  verde y los demás valores válidos amarillos (incluidos flotante, estimado,
  manual y simulado). HAS activo es verde e inactivo amarillo.
  Solución `HAS`/`RTK_FIX` es verde, `NOFIX` roja y las demás amarillas;
  `RTK` sin especificar no se interpreta como RTK fijo. Cada campo es independiente:
  no se deduce HAS ni solución a partir de FIX_IN.
- Un campo ausente muestra **No disponible** en amarillo. Sin conexión,
  con respuesta incompleta/inválida o con datos obsoletos, las tarjetas de
  telemetría conservan el texto pero muestran amarillo **Sin actualizar**.
  Telnet desconectado es rojo; conectando o conectado sin estado fresco es
  amarillo; conectado con respuesta completa vigente es verde.
- **Posición y tiempos** reserva latitud/longitud y altitud como **No disponible**:
  el modelo del parser actual no expone estos campos. No se analiza el registro
  ni se añade una lectura NMEA para completarlos. Muestra la última respuesta
  completa válida en UTC, su edad Telnet y el intervalo de consulta.
- **Alarmas activas** lista fallos GNSS, ausencia de fijación/solución, HAS
  inactivo y avisos de conexión, respuesta incompleta y datos obsoletos.
  La **consola inferior** mantiene las respuestas originales, registro acotado
  y exportación. Si falta espacio o se amplía la escala de Windows, el área
  superior permite desplazamiento sin ocultar la consola.

Solo cambia la presentación; se conservan el parser, la conexión TCP, los
comandos, plazos y criterio de respuesta válida. No se modifica el firmware.

Se reconocen tanto las etiquetas Rev.4 como pares de claves, sin distinguir
mayúsculas ni acentos:

```text
========== STATUS ==========
GNSS: OK
GGA entrada fix: 5
HAS: ACTIVO
Solucion: HAS
============================
```

```text
GNSS=OK FIX_IN=5 HAS=ON SOL=HAS
```

También se admiten `Estado GNSS`, `HAS activo`, `fix` y `SOLUTION`.
Las versiones antiguas que no publican FIX_IN o solución mostrarán
**No disponible** y la alerta de claves ausentes; no se infiere HAS ni solución
a partir del fix. Los valores desconocidos o malformados se tratan como ausentes.
Las respuestas parciales no heredan campos de consultas anteriores.
Los mensajes espontáneos `[DIAG]` no sustituyen la respuesta solicitada, incluso
si llegan fragmentados entre consultas. Después de `[DIAG]`, se espera una
nueva cabecera `STATUS`/`ESTADO` para reanudar el análisis, como emite el firmware;
un endpoint personalizado que mezcle mensajes debe usar esas cabeceras.

- Conexión: espera máxima de 5 s; errores de red y cierre remoto se notifican.
- Respuesta: espera máxima de 3 s, con delimitadores del firmware y una pausa
  de 250 ms para respuestas sin delimitadores. Solo hay una consulta pendiente.
  Si tarda más que el intervalo configurado, no se acumulan consultas.
- Datos obsoletos: más de `máximo(5000 ms, 3 × intervalo)` sin respuesta completa
  válida. El panel puede conservar la última lectura, siempre con su edad/alerta.
- Claves ausentes: lista de GNSS, FIX_IN, HAS o SOL ausentes/no válidas.
- Protección: máximo 64 KiB por consulta, negociación Telnet rechazada sin
  habilitar eco, recepción UTF-8 fragmentada y cancelación al cerrar.

Telnet no cifra ni autentica el tráfico. Utilizar únicamente en una red local
de confianza; no exponer este servicio directamente a Internet.

## Registro y exportación

El registro conserva hasta 300 entradas, con fecha y hora, respuestas Telnet
y alertas. Cada entrada se limita a 8192 caracteres; se indica el truncado.
**Exportar diagnóstico** permite elegir `.txt` o `.json`, guardar el estado,
configuración, edad, alertas y una instantánea del registro. La exportación no
contiene historial anterior al límite del registro ni captura de puertos serie.
Los informes pueden contener IP y coordenadas recibidas por Telnet: revisar su
contenido antes de compartirlos.

## Pruebas rápidas

Pruebas de protocolo sin dependencias externas, ejecutables también en Linux:

```powershell
dotnet run --project tools/DiagTelnetWin/Tests/DiagTelnetWin.Tests.csproj -c Release
```

Comprueban los formatos de firmware, campos ausentes/malformados, límites,
negociación Telnet fragmentada, UTF-8, consultas contra un servidor TCP local,
timeout, cancelación, desconexión y el mapeo visual de estados/frescura.
No requieren un dispositivo.

Comprobación manual en Windows:

1. Conectar al endpoint de diagnóstico real; comprobar GNSS/FIX_IN/HAS/solución.
2. Comparar con `status` en una sesión Telnet manual (cerrarla antes de conectar
   la aplicación: algunos firmwares admiten un solo cliente).
3. Desconectar el cable/red o cerrar el servidor: verificar alerta y que la
   ventana siga respondiendo. Volver a conectar mediante el botón.
4. Con un servidor de prueba que acepte `status` pero no responda, verificar
   timeout y datos obsoletos; con `GNSS=OK HAS=ON`, verificar claves ausentes.
5. Exportar ambos formatos y revisar estado y registro; probar una ruta sin
   permiso de escritura para comprobar el mensaje de error.
6. Pulsar Desconectar y cerrar la ventana durante una lectura: comprobar que
   termina sin bloquearse y que COM2 sigue disponible para el usuario.
7. Comprobar semáforos con `GNSS=OK FIX_IN=4 HAS=ON SOL=RTK_FIX`,
   `GNSS=OK FIX_IN=5 HAS=OFF SOL=RTK_FLOAT` y
   `GNSS=FAIL FIX_IN=0 HAS=OFF SOL=NOFIX`. Verificar también campos ausentes,
   transición a datos obsoletos y desconexión: ninguna lectura retenida debe
   parecer vigente en verde.
8. Redimensionar a tamaño mínimo y probar escala 125–150 %: tarjetas, tiempos
   y alarmas accesibles mediante desplazamiento y consola inferior visible.
   Las coordenadas/altitud deben seguir como No disponible. Capturar la ventana
   en Windows para revisión visual (WinForms no se ejecuta en Linux).
