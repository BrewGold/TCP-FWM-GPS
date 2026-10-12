// ============================================================================
// GNSS RECEPCIÓN - DEBUG HEXADECIMAL
// ============================================================================
// Este archivo contiene SOLO la función readGNSS() modificada para captura en HEX
// Reemplaza la función readGNSS() original en RS232-RWM-GPS_V4-0.ino (línea 1041)
// 
// MODO DE USO:
// 1. Copia esta función readGNSS() en tu sketch
// 2. Compila y sube a la Metro
// 3. Reinicia la Metro
// 4. En el monitor serie a 115200 bps, veras 10 segundos de captura hexadecimal
// 5. Copia TODA la salida (incluidos espacios en blanco)
// ============================================================================

// Variable de control para captura debug
static bool gnssDebugCaptureActive = false;
static uint32_t gnssDebugCaptureStartMs = 0;

void readGNSS() {
  if (!gnssStarted || gnssReading || __get_current_exception() != 0) {
    return;
  }

  gnssReading = true;

  // ========================================================================
  // CAPTURA EN HEXADECIMAL (PRIMEROS 10 SEGUNDOS DESPUÉS DE ENCENDER)
  // ========================================================================
  
  uint32_t now = millis();
  
  // Inicia captura en el primer ciclo
  if (!gnssDebugCaptureActive) {
    gnssDebugCaptureActive = true;
    gnssDebugCaptureStartMs = now;
    Serial.println("\n[DEBUG] Iniciando captura hexadecimal de Serial1...");
    Serial.println("[DEBUG] Esperando 10 segundos...\n");
  }

  // Captura activa: imprime TODO lo que llega en hexadecimal
  if (gnssDebugCaptureActive && (now - gnssDebugCaptureStartMs) < 10000) {
    while (Serial1.available() > 0) {
      uint8_t byte = Serial1.read();
      Serial.printf("%02X ", byte);
    }
  } 
  // Finaliza captura después de 10 segundos
  else if (gnssDebugCaptureActive && (now - gnssDebugCaptureStartMs) >= 10000) {
    gnssDebugCaptureActive = false;
    Serial.println("\n\n[DEBUG] Captura hexadecimal FINALIZADA");
    Serial.println("[DEBUG] Vuelve a subir el firmware SIN DEBUG para reanudar operación normal");
    
    // Vacía Serial1 para evitar basura
    while (Serial1.available() > 0) {
      Serial1.read();
    }
  }

  gnssReading = false;
}

// ============================================================================
// ALTERNATIVA: Si prefieres captura CON PARSING NORMAL (GGA/RMC válidas)
// ============================================================================
// Descomenta esta función y comenta la anterior para capturar SOLO frases válidas
// void readGNSS() {
//   if (!gnssStarted || gnssReading || __get_current_exception() != 0) {
//     return;
//   }
// 
//   gnssReading = true;
//   
//   // Debug: captura hexadecimal ANTES de procesar
//   static uint32_t debugStartMs = 0;
//   if (debugStartMs == 0) {
//     debugStartMs = millis();
//     Serial.println("\n[DEBUG] Modo: Captura líneas recibidas con HEX\n");
//   }
//   
//   uint32_t now = millis();
//   bool captureActive = (now - debugStartMs) < 10000;
//   
//   while (Serial1.available() > 0) {
//     char character = (char)Serial1.read();
//
//     // Imprime en HEX cada carácter (solo primeros 10 segundos)
//     if (captureActive) {
//       Serial.printf("[%02X] ", (uint8_t)character);
//     }
// 
//     if (
//       character == '$' ||
//       character == '#'
//     ) {
//       gnssLineIndex = 0;
//       gnssLine[gnssLineIndex++] = character;
//       if (captureActive) Serial.println();
//       continue;
//     }
// 
//     if (gnssLineIndex <= 0) {
//       continue;
//     }
// 
//     if (character == '\r') {
//       continue;
//     }
// 
//     if (character == '\n') {
//       gnssLine[gnssLineIndex] = '\0';
//
//       if (captureActive) {
//         Serial.print("[RX LINE] ");
//         Serial.println(gnssLine);
//       }
// 
//       processGnssLine(gnssLine);
// 
//       gnssLineIndex = 0;
//       continue;
//     }
// 
//     if (
//       gnssLineIndex <
//       (int)sizeof(gnssLine) - 1
//     ) {
//       gnssLine[gnssLineIndex++] = character;
//     } else {
//       Serial.println("[GNSS] Línea demasiado larga");
//       gnssLineIndex = 0;
//     }
//   }
//   
//   if (!captureActive && debugStartMs != 0) {
//     Serial.println("\n[DEBUG] Captura finalizada. Ahora en modo normal.");
//     debugStartMs = -1; // Marca como finalizado
//   }
//   
//   gnssReading = false;
// }
