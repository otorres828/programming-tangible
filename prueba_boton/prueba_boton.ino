// Prueba de botón: inicio/pausa/reanudar/reinicio
const int BOTON_INICIO = 2;

enum SystemState { ESTADO_LEER, ESTADO_CORRER, ESTADO_PAUSA, ESTADO_REINICIO };
SystemState estadoSistemaActual = ESTADO_LEER;

unsigned long lastButtonStateChangeTime = 0;
unsigned long buttonPressStartTime = 0;
bool lastButtonReading = HIGH;
bool currentButtonReading = HIGH;
bool longPressTriggered = false;

const unsigned long BUTTON_DEBOUNCE_DELAY = 50; // ms
const unsigned long LONG_PRESS_THRESHOLD = 3000; // ms (3s)

void setup() {
  Serial.begin(2400);
  pinMode(BOTON_INICIO, INPUT_PULLUP);
  Serial.println("Prueba boton: listo. Usa el boton conectado a pin 2 (INPUT_PULLUP).");
  Serial.println("Pulsacion corta: INICIO / PAUSA / REANUDAR. Pulsacion larga (3s): REINICIO.");

  // esperar liberación del botón al arranque
  while (digitalRead(BOTON_INICIO) == LOW) { delay(5); }
  lastButtonReading = HIGH;
  currentButtonReading = HIGH;
}

void loop() {
  bool reading = digitalRead(BOTON_INICIO);

  if (reading != lastButtonReading) {
    lastButtonStateChangeTime = millis();
  }

  if ((millis() - lastButtonStateChangeTime) > BUTTON_DEBOUNCE_DELAY) {
    // estado estable
    if (reading != currentButtonReading) {
      currentButtonReading = reading;

      if (currentButtonReading == LOW) {
        buttonPressStartTime = millis();
        longPressTriggered = false;
      } else {
        if (!longPressTriggered) {
          if (estadoSistemaActual == ESTADO_LEER) {
            Serial.println("Boton: INICIO de secuencia.");
            estadoSistemaActual = ESTADO_CORRER;
          } else if (estadoSistemaActual == ESTADO_CORRER) {
            Serial.println("Boton: PAUSA de secuencia.");
            estadoSistemaActual = ESTADO_PAUSA;
          } else if (estadoSistemaActual == ESTADO_PAUSA) {
            Serial.println("Boton: REANUDAR secuencia.");
            estadoSistemaActual = ESTADO_CORRER;
          } else if (estadoSistemaActual == ESTADO_REINICIO) {
            Serial.println("Estado: ya en REINICIO. Volviendo a LEER.");
            estadoSistemaActual = ESTADO_LEER;
          }
        }
      }
    }
  }

  if (currentButtonReading == LOW && !longPressTriggered) {
    if ((millis() - buttonPressStartTime) >= LONG_PRESS_THRESHOLD) {
      Serial.println("Boton: REINICIO COMPLETO (pulsacion larga).");
      longPressTriggered = true;
      estadoSistemaActual = ESTADO_REINICIO;
      delay(200);
      estadoSistemaActual = ESTADO_LEER;
      Serial.println("Estado: REINICIO completado. Volviendo a LEER.");
    }
  }

  lastButtonReading = reading;
  delay(10);
}