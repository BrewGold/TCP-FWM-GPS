# Diagnóstico Windows por Telnet (sin COM2)

Aplicación independiente WinForms, .NET 8, con interfaz en español. Consulta
únicamente el endpoint de diagnóstico Telnet/TCP mediante `status\r\n`.
No modifica el firmware, el IDE ni la configuración del receptor; no envía
`diag`, `freq`, `com2` ni comandos de configuración.

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
La IP, el puerto y el intervalo se mantienen solo durante la sesión de la
aplicación; el modo de vista sí se guarda entre ejecuciones.

Pulsar **Conectar** para iniciar las consultas y **Desconectar** para detenerlas.
Los ajustes quedan bloqueados mientras se conecta o consulta. No hay reconexión
automática: después de una desconexión, revisar la red y pulsar Conectar.

## Modos de interfaz

El botón **Modo: Compacto (50%)** / **Modo: Completo (100%)**, siempre en la barra
superior, alterna la vista sin desconectar ni reiniciar las consultas.
Los porcentajes identifican la densidad de información, no un escalado del texto.

- **Compacto (50%)**, vista inicial: ventana de 760×430, reducible hasta
  640×360 (dimensiones aproximadas con escala de Windows al 100%). Solo muestra
  GNSS, FIX_IN, HAS, solución, salud Telnet, última actualización y alarmas
  activas. Los valores principales son grandes y usan semáforos de color con
  texto: verde = correcto/activo, naranja = atención/inactivo, rojo =
  fallo/sin solución, gris = lectura sin vigencia confirmada. HAS inactivo no
  significa por sí solo que Telnet esté averiado.
  El log, la exportación y los ajustes se ocultan por defecto. **Configuración**
  despliega u oculta IP, puerto e intervalo; Conectar/Desconectar permanecen
  accesibles. En ventanas pequeñas o con escala elevada se permite desplazamiento.
- **Completo (100%)**: ventana inicial de 980×720, con todos los estados,
  configuración, registro en vivo y exportación. El log sigue recogiéndose en
  compacto (hasta 300 entradas) y aparece al volver a completo; no se pierde
  el diagnóstico por cambiar la vista.

La preferencia se guarda por usuario en
`%LOCALAPPDATA%\DiagTelnetWin\vista.json` al pulsar el selector. Si el archivo
falta, está dañado o no se puede leer, se abre compacto. Si no se puede guardar,
se avisa y el cambio sigue funcionando durante esa ejecución.
Eliminar ese archivo restablece la vista inicial; no contiene IP ni telemetría.

### Ejemplo de uso y esquema de las vistas

1. Abrir la app en compacto, pulsar **Configuración**, introducir el endpoint
   Telnet y pulsar **Conectar**. Ocultar Configuración para dejar solo el estado.
2. Pulsar **Modo: Compacto (50%)** para pasar a completo y revisar el log o
   **Exportar diagnóstico**. Las consultas continúan sin interrupción.
3. Volver a compacto y cerrar. Al abrir de nuevo, se recupera compacto; si se
   cierra en completo, se recupera completo.

Esquemas orientativos (no son capturas reales; la disposición y tamaño del texto
dependen de la escala de pantalla):

```text
Compacto (50%)
┌────────────────────────────────────────────────────────────────┐
│ Modo: Compacto (50%)  Configuración  Conectar  Desconectar       │
│ Estado GNSS                    FIX_IN                          │
│ Correcto                       5                               │
│ Estado HAS                     Solución                        │
│ Activo                         HAS                             │
│ Salud Telnet                   Última actualización            │
│ Conectado · Salud correcta      10:42:30 · hace 0,8 s            │
│ Sin alarmas activas.                                           │
└────────────────────────────────────────────────────────────────┘

Completo (100%)
┌────────────────────────────────────────────────────────────────┐
│ Modo: Completo (100%)  Conectar  Desconectar                     │
│ IP del dispositivo […]  Puerto Telnet […]  Intervalo (ms) […]    │
│ Salud Telnet                   Conectado · Salud correcta      │
│ Estado GNSS / FIX_IN / HAS      Correcto / 5 / Activo            │
│ Solución / Última actualización HAS / 10:42:30 · hace 0,8 s     │
│ Sin alarmas activas.                                           │
│ [Registro Telnet en vivo con fecha y hora]                      │
│ Exportar diagnóstico · Solo Telnet                             │
└────────────────────────────────────────────────────────────────┘
```

## Interpretación y alertas

El panel muestra GNSS, FIX_IN, HAS, solución, conexión y edad de la última
**respuesta completa válida** recibida por la aplicación. Esta edad mide
frescura de la telemetría Telnet, **no** antigüedad de la última medida GNSS:
el firmware actual no publica esa edad en `status`.
La hora mostrada es la hora local de recepción de esa respuesta completa.
La salud Telnet distingue conexión con datos actuales de conexión sin datos
actuales; no garantiza que GNSS tenga solución. Fallo GNSS o FIX_IN=0/sin solución
también aparecen como alarmas.

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

Comprueban la persistencia de ambos modos y la recuperación ante preferencias
inválidas o errores de escritura, además de formatos de firmware, campos ausentes/malformados, límites,
negociación Telnet fragmentada, UTF-8, consultas contra un servidor TCP local,
timeout, cancelación y desconexión. No requieren un dispositivo.

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
7. Alternar modos conectado: comprobar que no se interrumpe la consulta y que el
   registro acumulado reaparece en completo. Cerrar y abrir en cada modo para
   comprobar la persistencia; probar archivo `vista.json` dañado y ruta sin permiso.
8. En compacto, probar 640×360, 760×430 y 800×450, con escala de Windows al 100%,
   150% y 200%: comprobar lectura de estados, alarmas largas y acceso al selector
   y Configuración (desplazamiento cuando sea necesario). Verificar colores y
   textos ante GNSS fallido, sin fix, respuesta parcial, timeout y desconexión.
