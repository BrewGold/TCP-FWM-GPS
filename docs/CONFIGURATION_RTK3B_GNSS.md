# Configuración GNSS - ArduSimple RTK3B Budget
## Rev.2.7 FWD-GPS

---

## 📍 PARÁMETROS GNSS FINALES

### 1. Declinación Magnética
```
DECLINATION_DEG_DEFAULT = -3.5°
Ubicación: España
Tipo: Fija (no dinámica)
```

### 2. Validación FIX_IN para LOCKED
```
Condición de bloqueo (LOCKED):
├─ FIX_IN >= 4  (RTK / PPP)
└─ O hasActive = true (HAS activo)

Rechaza:
├─ FIX_IN = 1  (GPS puro, no RTK)
├─ FIX_IN = 2  (DGPS)
└─ FIX_IN = 3  (RTK Float - inestable)
```

### 3. Tipo de Solución (SOL)
```
Estado actual: SOL_UNKNOWN
Valores posibles:
├─ SOL_GPS    (1) = GPS estándar
├─ SOL_DGPS   (2) = Diferencial
├─ SOL_RTK    (3) = RTK Time (fijo)
└─ SOL_HAS    (4) = Augmentation Service

Detectado por: Campo GGA[6] + PUBX,00
```

### 4. Precisión GNGST
```
Mensaje NMEA: $GNGST
Campos extraídos:
├─ std_lat    (metros) = Desviación estándar latitud
├─ std_lon    (metros) = Desviación estándar longitud
└─ std_alt    (metros) = Desviación estándar altitud

Cálculo final:
├─ H_ERR = sqrt(std_lat² + std_lon²)  [metros]
└─ V_ERR = std_alt                     [metros]

Mostrado en BLE: [DIAG] H_ERR=0.050m V_ERR=0.100m
Validez: < 5 segundos sin actualización
```

### 5. HDOP (Dilución Horizontal de Precisión)
```
Rango recomendado: < 2.5
Extraído de: Mensaje GGA campo 8
Mostrado en BLE: HDOP=0.6
```

### 6. Satélites
```
Mínimo recomendado: >= 5 satélites para GPS
Mínimo RTK: >= 5 satélites + correcciones

Extraído de: Mensaje GGA campo 7
Variable: satelliteCount
```

---

## 🔧 CONFIGURACIÓN UM980 (GNSS Receptor)

### Baudios
```
Serial principal (GNSS): 115200 bps
COM2 (Arduino): 38400 bps
Modo: 8N1 (8 bits, sin paridad, 1 stop)
```

### Mensajes NMEA Configurados
```
Salida obligatoria:
├─ $GNGGA   (Posición + FIX_IN)    @ 10 Hz
├─ $GNRMC   (Velocidad + Curso)    @ 10 Hz
├─ $GNGST   (Precisión GNSS)       @ 1 Hz
└─ $PUBX,00 (Solución extendida)   @ 1 Hz

Frecuencia COM2:
└─ $GCGGA   (Salida corregida)     @ 10 Hz
```

### Variables GNSS en Arduino
```cpp
// Posición
double currentLat;     // Latitud [grados decimales]
double currentLon;     // Longitud [grados decimales]
double currentAlt;     // Altitud [metros]

// Movimiento
double currentSpeedMS; // Velocidad [m/s]
double currentCourse;  // Curso [grados 0-360]

// Calidad
int inputFixQuality;   // FIX_IN (1-5)
int satelliteCount;    // Número de satélites
char hdopText[16];     // HDOP formateado

// Precisión GNGST (NUEVO)
double gnsHorizErr;    // Error horizontal [metros]
double gnsVertErr;     // Error vertical [metros]

// Timestamp
uint32_t lastGgaMs;    // Último GGA recibido
uint32_t lastGnstMs;   // Último GNGST recibido
char utcTime[16];      // Hora UTC HHmmss.ss
```

---

## 📊 ESTADOS DE MOVIMIENTO Y FIX

### MOVING (Buscando)
```
Condición: velocidad > 0.30 m/s
FIX_IN: Cualquier valor >= 1
Acción: No promedia, transmite en tiempo real
LED1: Parpadea 300ms (rojo)
LED2: Apagado
```

### AVERAGING (Promediando)
```
Condición: velocidad < 0.20 m/s durante 2 segundos
FIX_IN: Cualquier valor >= 1
Acción: Acumula muestras GNSS + Yaw IMU
Duración: 15 segundos
LED1: Parpadea 150ms (rojo)
LED2: Parpadea 150ms (verde)
```

### LOCKED (Bloqueado)
```
Condición: Promedio completado Y (FIX_IN >= 4 O hasActive)
FIX_IN requerido: >= 4 (RTK fijo o HAS)
Rechazo: FIX_IN = 1,2,3 (GPS puro, DGPS, RTK Float)
Acción: Posición fija, no se actualiza
LED1: Apagado
LED2: Encendido continuo (verde)

Re-activación:
├─ Velocidad > 0.30 m/s, O
└─ Distancia > 1.0 metro
```

### FALLA
```
Condición: !gnssValid OR !imuAvailable OR !magCalValid
LED1: Apagado
LED2: Apagado
Causas:
├─ Sin datos GNSS (GGA no recibido)
├─ IMU no disponible
└─ Magnetómetro no calibrado (EEPROM vacía)
```

---

## 🎯 FLUJO DE VALIDACIÓN GNSS

```
Recibir $GNGGA
    ↓
Validar checksum NMEA ✓
    ↓
Parsear FIX_IN (campo 6)
    ↓
FIX_IN >= 1? ✓ (GPS puro, DGPS, RTK)
    ↓
Guardar: currentLat, currentLon, currentAlt
         satelliteCount, inputFixQuality
    ↓
gnssValid = true
    ↓
Actualizar timestamp (lastGgaMs)
    ↓
Validar FIX_IN >= 4 para LOCKED
    ├─ SI → Puede entrar LOCKED
    └─ NO → Solo MOVING/AVERAGING
```

---

## 📡 MENSAJE DIAGNÓSTICO BLE

```
[DIAG]
GNSS=OK FIX_IN=5 HAS=ON SOL=HAS SAT=26 HDOP=0.6
H_ERR=0.050m V_ERR=0.100m
STATE=LOCKED LOCK=YES SPD=0.002 m/s
IMU=OK YAW=45.3 MAGCAL=YES
RAW=39.45937784,-5.87548742
OUT=39.45936860,-5.87548799 FIX_OUT=5
COM2 frames=2769 bytes=39519
```

### Explicación:
```
GNSS=OK              Datos GNSS válidos
FIX_IN=5             RTK fijo (5 = mejor)
HAS=ON               Augmentation Service activo
SOL=HAS              Tipo solución: HAS
SAT=26               26 satélites conectados
HDOP=0.6             Precisión horizontal muy buena

H_ERR=0.050m         Error horizontal 5 cm
V_ERR=0.100m         Error vertical 10 cm

STATE=LOCKED         Estado actual: LOCKED
LOCK=YES             Posición bloqueada
SPD=0.002 m/s        Velocidad casi 0

IMU=OK               Sensor IMU operativo
YAW=45.3             Orientación N-E
MAGCAL=YES           Magnetómetro calibrado

RAW=LAT,LON          Posición cruda sin corrección
OUT=LAT,LON          Posición final enviada por COM2
FIX_OUT=5            Calidad de FIX en salida

COM2 frames=2769     2769 tramas enviadas
COM2 bytes=39519     39519 bytes totales
```

---

## ⚙️ CONFIGURACIÓN ARDUINO (Rev 2.7)

### Líneas a modificar para otras tarjetas:

```cpp
// Línea 81 - DECLINACIÓN (cambiar según ubicación)
#define DECLINATION_DEG_DEFAULT     -3.5  // España
// Ejemplos:
// Francia:        0.5
// Alemania:       2.5
// Portugal:      -4.0
// USA (este):     -8.0
// USA (oeste):   12.0

// Línea 1810 - VALIDACIÓN FIX_IN (NO CAMBIAR)
if (
  sampleCount > 0 &&
  elapsedMs >= AVERAGING_WINDOW_MS_VAR &&
  (inputFixQuality >= 4 || hasActive)  // ← Crítico para RTK
)

// Línea 2924 - Mostrar precisión GNGST (NO CAMBIAR)
"H_ERR=%sm V_ERR=%sm\r\n"
```

---

## 🔍 VALIDACIÓN EN CAMPO

### Checklist antes de usar:

- [ ] ArduSimple RTK3B conectado a Serial1 (D0/D1)
- [ ] Antena GNSS en ubicación despejada
- [ ] Señal RTK o HAS disponible (FIX_IN >= 4)
- [ ] BLE conectado y recibiendo [DIAG]
- [ ] H_ERR < 0.1 metro (10 cm) para precisión
- [ ] HDOP < 2.5 (ideal < 1.0)
- [ ] SAT >= 5 (ideal >= 15)
- [ ] LED1 rojo, LED2 verde durante LOCKED

### Troubleshooting:

```
Problema: FIX_IN=1, no sube a 4
Solución:
├─ Activar RTK (base)
├─ Activar HAS (Augmentation)
└─ Cambiar ubicación (más satélites)

Problema: H_ERR muy alto (> 1 metro)
Solución:
├─ Esperar convergencia RTK (5-10 min)
├─ Verificar conexión antena
└─ Reiniciar receptor

Problema: No entra LOCKED
Solución:
├─ Verificar FIX_IN >= 4 (BLE [DIAG])
├─ IMU calibrado (MAGCAL=YES)
└─ Parado durante 15 segundos
```

---

## 📚 Referencias

- **GNSS Receptor**: u-blox UM980 (ArduSimple RTK3B Budget)
- **Protocolo**: NMEA 0183
- **Precisión esperada**: 1-5 cm (RTK fijo)
- **Latencia**: < 1 segundo
- **Actualización**: 10 Hz (GNGGA, GNRMC)

**Última actualización**: Rev 2.7
**Fecha**: 2026-10-01
