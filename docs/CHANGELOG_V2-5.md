# Changelog V2-5 — RS232-GPS

Fecha: 2026-10-14  
Repositorio: `BrewGold/TCP-FWM-GPS`

## Resumen
Esta actualización consolida mejoras de robustez para operación continua, correcciones de formato NMEA y lineamientos de configuración UM980/HAS.

## Cambios incluidos

### 1) Robustez de comunicaciones (COM2)
- Se mantiene criterio **no bloqueante** en salida COM2.
- Se refuerza envío por partes controlando disponibilidad de buffer (`availableForWrite()`).

**Impacto:** disminuye riesgo de congelamiento del loop principal bajo carga o backpressure.

---

### 2) Manejo de clientes TCP inactivos
- Se identifica como punto crítico de operación prolongada la presencia de clientes/sockets inactivos.
- Se deja recomendación explícita de endurecer timeout y limpieza de sesiones en canales NMEA/diagnóstico.

**Impacto:** mejora estabilidad de uptime y reduce degradación progresiva.

---

### 3) Corrección de formato NMEA GGA
- Ajuste de plantilla `snprintf` en `createGCGGA` para consistencia de campos de HDOP/altitud.
- Cadena de referencia:
  - `"$GPGGA,%s,%s,%c,%s,%c,%d,%d,%s,%s,M,0.0,M,,"`

**Impacto:** evita desalineación de campos y tramas potencialmente inválidas.

---

### 4) Documentación UM980 / Galileo HAS
Se agregan comandos operativos clave:

```text
CONFIG SIGNALGROUP 2
CONFIG PPP ENABLE E6-HAS
```

Recomendaciones asociadas:
- Guardar configuración con `SAVECONFIG`.
- Verificar versión/estado con `VERSIONA` y diagnóstico del receptor.

**Impacto:** reduce errores de configuración y acelera puesta en marcha HAS.

---

### 5) App Windows de diagnóstico (lineamiento UI)
- Se define estrategia de dos modos de visualización:
  - **Compacto (50%)**: estado crítico.
  - **Completo (100%)**: diagnóstico extendido + log en vivo.

**Estado:** lineamiento aprobado; implementación sujeta a PR de UI.

## Compatibilidad
- Arquitectura objetivo: RP2040 + PIO (COM2) + W5500.
- Integración RS232-FWD mantenida.
- Enfoque de operación: 24/7.

## Riesgos conocidos (pendientes)
1. Sesiones TCP inactivas prolongadas.
2. Degradación de enlace en ruido de red alto.
3. Regresiones silenciosas en formato NMEA sin pruebas automáticas.

## Acciones recomendadas post-release
- Ejecutar prueba de estabilidad >24h.
- Validar integridad de tramas NMEA (incluyendo GGA) en escenario de carga.
- Cerrar PR de app Windows para modo compacto/completo.
- Formalizar checklist de operación UM980 para sesiones PPK/HAS.
