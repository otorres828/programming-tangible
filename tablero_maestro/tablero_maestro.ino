// ---------------------------------------------------------------------------------------
// Código para Arduino Nano Central (MAESTRO I2C) - Archivo Único
// Solicita datos de 2 Arduinos de columna (Esclavos I2C, 4 resistencias cada uno)
// y un tercer Arduino (Esclavo I2C, Bloque de Control de 3 resistencias).
// Consolida los valores de resistencia en un array de 11 posiciones.
// Nota: Utilizamos A4 y A5 para la comunicacion I2C con cada columna y con los LEDS de cada ficha.
// El movimiento del robot es en direcciones cardinales fijas (arriba, abajo, izquierda, derecha)
// ---------------------------------------------------------------------------------------

#include <Wire.h>                     // Librería para comunicación I2C
#include <SoftwareSerial.h>           // Libreria para comunicacion Bluetooth
#include <Adafruit_PWMServoDriver.h>  // Librería para el PCA9685 - Controlador de Leds de las fichas

#define BOTON_INICIO 2  // Pin del botón (con resistencia pull-up)

SoftwareSerial mySerial(10, 11); // RX, TX para Bluetooth

// Direcciones I2C de los Arduinos esclavos: Columna 1, Columna 2, Bloque de Control
const uint8_t SLAVE_ADDRESSES[] = { 0x01, 0x02, 0x03 };

// Arrays para almacenar los datos
float allResistances[11];         // Almacena todas las 11 resistencias leídas
float instruccionesColumnas[8];   // Las primeras 8 instrucciones (de las 2 columnas)
float bloqueControl[3];           // El bloque de control (las últimas 3 posiciones)

// Snapshot estable para ejecución (evita incoherencias al pausar/reanudar)
float instruccionesSnapshot[8];
float bloqueControlSnapshot[3];

// --- DEFINICIONES DE ACCIONES ---
enum ActionType {
  MOVER_ARRIBA = 1,     // Resistencia 800-1600 Ohms                   - 1K
  MOVER_ABAJO = 2,      // Resistencia 100-600 Ohms                    - 220omh
  MOVER_IZQUIERDA = 3,  // Resistencia 3k-6k Ohms                      - 4.7k
  MOVER_DERECHA = 4,    // Resistencia 1.7k-2.6k Ohms                  - 2k
  BLOQUE_CONTROL = 5,   // Resistencia 70k-110k Omhs                   - 100k
  MELODIA_1 = 6,        // Resistencia 7k-15k Ohms                     - 10k
  DO_HOMMING = 7,
};

// --- UNIÓN PARA CONVERTIR FLOAT A BYTES Y VICEVERSA ---
union FloatBytes {
  float f;    // El valor flotante
  byte b[4];  // Sus 4 bytes constituyentes
};

// --- ESTADO GLOBAL DEL ROBOT ---
int robotX = 2;  // Posición actual en X 
int robotY = 2;  // Posición actual en Y 

// Crea un objeto PCA9685 , con Direccion 0x40 para comunicacion I2C con el driver Pca9685 que controlara los LEDS de las instrucciones
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Define el número total de LEDs que vas a controlar
const uint8_t NUM_LEDS = 11; // 8 instrucciones de columnas + 3 de bloque de control

// Define los valores de brillo (PWM)
// El PCA9685 tiene una resolución de 12 bits, lo que significa un rango de 0 a 4095.
const int BRILLO_OFF = 0;            // LED apagado
const int BRILLO_20_PORCIENTO = 80; // 20% de 4095
const int BRILLO_100_PORCIENTO = 4000; // LED al máximo brillo

// Filtros para evitar parpadeo por ruido o ráfagas I2C.
const int UMBRAL_PRESENCIA_ON = 20;
const int UMBRAL_PRESENCIA_OFF = 5;
const unsigned long LED_UPDATE_MIN_MS = 25;

// ---  VARIABLES GLOBALES PARA LA MÁQUINA DE ESTADOS Y BOTÓN ---
enum SystemState { ESTADO_LEER, ESTADO_CORRER, ESTADO_PAUSA, ESTADO_REINICIO };
SystemState estadoSistemaActual = ESTADO_LEER;
int actualInstruccionIndex = 0; // Índice de la instrucción actual en la secuencia principal (0-7)

// Estado de ejecución reanudable del BLOQUE_CONTROL
bool blockControlActivo = false;
int blockControlOwnerIndex = -1;
int blockControlSubIndex = 0;
bool ledEstadoPrevio[NUM_LEDS] = {false};
int ledBrilloPrevio[NUM_LEDS] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
unsigned long ledUltimaActualizacion[NUM_LEDS] = {0};

// Variables para el manejo del botón (Debouncing y Pulsación Larga)
unsigned long lastButtonStateChangeTime = 0;
unsigned long buttonPressStartTime = 0;
bool lastButtonReading = HIGH; // INPUT_PULLUP, HIGH cuando no está presionado
bool currentButtonReading = HIGH;
bool longPressTriggered = false; // Flag para saber si una pulsación larga ya fue manejada

const unsigned long BUTTON_DEBOUNCE_DELAY = 50;  // ms de delay para debounce
const unsigned long LONG_PRESS_THRESHOLD = 1000; // 1 segundos para pulsación larga

// Variable para el retardo no bloqueante entre acciones
unsigned long lastActionExecutionTime = 0;
const unsigned long DELAY_POR_INSTRUCCION = 100; // 100 ms de retraso entre la ejecución de acciones
unsigned long lastReadColumnsTime = 0;
const unsigned long INTERVALO_LECTURA_COLUMNAS = 120; // Evita saturar el bus I2C con lecturas/escrituras continuas

// --- Opciones de detección de conexión Bluetooth ---
#define BT_STATE_PIN A2
const int ledVerde = A0; // LED de Conectado
const int ledRojo = A1;  // LED de Desconectado
bool conectado = false;  // LED state helper

// Variables de estado para detección
bool btConnected = false; // estado lógico (basado en STATE pin)

// Variables para debounce del pin STATE
int btRawLast = LOW;
int btStableState = LOW;
unsigned long btLastDebounceTime = 0;
const unsigned long BT_STATE_DEBOUNCE_MS = 100; // tiempo de debounce para pin STATE (ms)

// --- DECLARACIÓN DE FUNCIONES---
void leerTodasColumnas();                                  // Leer los datos de los 3 esclavos I2C
void copiarArrays();                                       // Copiar los datos leídos a las variables globales
void capturarSnapshotInstrucciones();                      // Congelar instrucciones para la corrida actual
void botonPulsaciones();                                   // Manejar el botón
void ejecutarSiguienteInstruccion();                       // Ejecutar una sola instrucción
void enviarBluetooth(ActionType action, int globalIndex);  // Ejecutar una acción específica
void ejecutarBlockControl(int globalIndexPrincipal);       // Ejecutar la lógica del bloque de control
void esperarNoBloqueante(unsigned long duracionMs);        // Espera no bloqueante procesando botón/estado
void resetEstadoBloqueControl();                            // Reinicia estado interno de subrutina BLOQUE_CONTROL
void setBrilloLeds(int ledIndex, int brightness);          // Establecer el brillo de un LED
bool validarPosicionXY(int x, int y);                      // Validar si una posición (x,y) es válida en la cuadrícula
const __FlashStringHelper* getAccionText(ActionType action); // Obtener el texto descriptivo de una acción
int manejoLedBT(int stableState);
void volverAlCentro();                                      // Mover al centro paso a paso con espera entre acciones
void printResistance(ActionType action);

void setup() {
  Serial.begin(9600);  // Para comunicación con el Monitor Serial del PC

  pinMode(BT_STATE_PIN, INPUT);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledRojo, OUTPUT);

  while (!Serial) {
    ;
  }

  Wire.begin();
  Wire.setClock(100000L); // 100kHz para mayor estabilidad en bus compartido/cableado largo.

  pwm.begin();
  pwm.setPWMFreq(1600);

  for (int q = 0; q < NUM_LEDS; q++) {
    pwm.setPWM(q, 0, BRILLO_20_PORCIENTO);
    ledBrilloPrevio[q] = BRILLO_20_PORCIENTO;
    ledEstadoPrevio[q] = true;
  }

  pinMode(BOTON_INICIO, INPUT_PULLUP);

  Serial.println(F("----------------------------------"));
  Serial.println(F(" Arduino Central (Maestro I2C) "));
  Serial.println(F("----------------------------------"));
  Serial.println(F("Esperando conexion por bluetooth (INICIAL)..."));

  mySerial.begin(9600);

  int s = digitalRead(BT_STATE_PIN);
  btConnected = (s == HIGH);
  btRawLast = s;
  btStableState = s;
  btLastDebounceTime = millis();
  manejoLedBT(s);

  while (digitalRead(BOTON_INICIO) == LOW) { delay(5); }
  lastButtonReading = HIGH;
  currentButtonReading = HIGH;
}

void loop() {

  // leerTodasColumnas();return;
  
  int raw = digitalRead(BT_STATE_PIN);
  if (raw != btRawLast) {
    btLastDebounceTime = millis();
    btRawLast = raw;
  }

  if ((millis() - btLastDebounceTime) > BT_STATE_DEBOUNCE_MS && raw != btStableState) {
    btStableState = raw;
    manejoLedBT(btStableState);

    if (btStableState == HIGH && !btConnected) {
      btConnected = true;
      Serial.println(F("Bluetooth MAESTRO: conectado (STATE pin)..."));

      // Al conectar, todos los LEDs deben quedar visibles al 20%.
      for (int i = 0; i < NUM_LEDS; i++) {
        setBrilloLeds(i, BRILLO_20_PORCIENTO);
        ledEstadoPrevio[i] = true;
      }

    } else if (btStableState == LOW && btConnected) {
      btConnected = false;
      Serial.println(F("Bluetooth MAESTRO: desconectado (STATE pin)."));
      while (mySerial.available()) { mySerial.read(); }
    }
  }

  if (!btConnected) {
    return;
  }

  botonPulsaciones();

  switch (estadoSistemaActual) {
    case ESTADO_LEER:
    case ESTADO_PAUSA:
      if (millis() - lastReadColumnsTime >= INTERVALO_LECTURA_COLUMNAS) {
        leerTodasColumnas();
        copiarArrays();
        lastReadColumnsTime = millis();
      }
      break;

    case ESTADO_REINICIO:
      Serial.println(F("Estado REINICIO: enviando '10' al CNC para reinicio/accion."));
      mySerial.println(10);
      delay(100);
      resetEstadoBloqueControl();
      estadoSistemaActual = ESTADO_LEER;
      actualInstruccionIndex = 0;
      lastActionExecutionTime = millis();
      break;

    case ESTADO_CORRER:
      if (millis() - lastActionExecutionTime >= DELAY_POR_INSTRUCCION) {
        if (actualInstruccionIndex < 8) {
          ejecutarSiguienteInstruccion();
          lastActionExecutionTime = millis();
        } else {
          Serial.println(F("Secuencia principal finalizada."));
          mySerial.println(11);
          resetEstadoBloqueControl();
          estadoSistemaActual = ESTADO_LEER;
          actualInstruccionIndex = 0;
          for (int i = 0; i < NUM_LEDS; i++) {
            setBrilloLeds(i, BRILLO_20_PORCIENTO);
          }
        }
      }
      break;
  }
}

// ------------------------------------------------------------------------
// FUNCION MANEJO DE LED BLUETOOTH
// ------------------------------------------------------------------------
int manejoLedBT(int stableState) {
  bool prev = conectado;

  if (stableState == HIGH) {
    digitalWrite(ledVerde, HIGH);
    digitalWrite(ledRojo, LOW);
    conectado = true;
  } else {
    digitalWrite(ledVerde, LOW);
    digitalWrite(ledRojo, HIGH);
    conectado = false;
  }

  if (conectado && !prev) {
    Serial.println(F(">>> ESTADO: CONECTADO <<<"));
    Serial.println(F("Estado actual: IDLE (esperando boton para iniciar)"));

  } else if (!conectado && prev) {
    Serial.println(F(">>> ESTADO: DESCONECTADO <<<"));
  }

  return conectado ? 1 : 0;
}

// Espera no bloqueante: permite pausar/reiniciar durante la espera
void esperarNoBloqueante(unsigned long duracionMs) {
  unsigned long inicio = millis();
  while (millis() - inicio < duracionMs) {
    botonPulsaciones();
    if (!btConnected || estadoSistemaActual != ESTADO_CORRER) {
      break;
    }
  }
}

void resetEstadoBloqueControl() {
  blockControlActivo = false;
  blockControlOwnerIndex = -1;
  blockControlSubIndex = 0;
}

// -------------------------------------------------------------------------
// FUNCIONES DE MANEJO DEL BOTÓN
// -------------------------------------------------------------------------
void botonPulsaciones() {
  bool reading = digitalRead(BOTON_INICIO);

  if (reading != lastButtonReading) {
    lastButtonStateChangeTime = millis();
  }

  if ((millis() - lastButtonStateChangeTime) > BUTTON_DEBOUNCE_DELAY) {
    if (reading != currentButtonReading) {
      currentButtonReading = reading;

      if (currentButtonReading == LOW) {
        buttonPressStartTime = millis();
        longPressTriggered = false;
      } else {
        if (!longPressTriggered) {
          if (estadoSistemaActual == ESTADO_LEER) {
            Serial.println(F("Boton: INICIO de secuencia."));
            capturarSnapshotInstrucciones();
            resetEstadoBloqueControl();
            estadoSistemaActual = ESTADO_CORRER;
            actualInstruccionIndex = 0;
            lastActionExecutionTime = millis();
          } else if (estadoSistemaActual == ESTADO_CORRER) {
            Serial.println(F("Boton: PAUSA de secuencia."));
            estadoSistemaActual = ESTADO_PAUSA;
          } else if (estadoSistemaActual == ESTADO_PAUSA) {
            Serial.println(F("Boton: REANUDAR secuencia."));
            estadoSistemaActual = ESTADO_CORRER;
            lastActionExecutionTime = millis();
          }
        }
      }
    }
  }

  if (currentButtonReading == LOW && !longPressTriggered) {
    if ((millis() - buttonPressStartTime) >= LONG_PRESS_THRESHOLD) {
      longPressTriggered = true;
      estadoSistemaActual = ESTADO_REINICIO;
      actualInstruccionIndex = 0;
      resetEstadoBloqueControl();
      for (int i = 0; i < NUM_LEDS; i++) {
        setBrilloLeds(i, BRILLO_20_PORCIENTO);
      }

      if (robotX == 2 && robotY == 2) {
        Serial.println(F("Boton REINICIO: robot ya esta en (2,2), comando ignorado."));
      } else if (robotX == 0 && robotY == 0) {
        Serial.println(F("Boton REINICIO: robot en (0,0), emitiendo codigo 10 (DO_HOMMING)."));
        mySerial.println(10);
        delay(6000);
        robotX = 0;
        robotY = 0;
      } else {
        Serial.println(F("Boton REINICIO: fuera de (0,0) y (2,2). Moviendo al centro con pasos de 9s."));
        volverAlCentro();
      }

      estadoSistemaActual = ESTADO_LEER;
    }
  }

  lastButtonReading = reading;
}

void volverAlCentro() {
  mySerial.println(13);
  delay(3000);
  Serial.print(F("Volviendo al centro desde ("));
  Serial.print(robotX);
  Serial.print(F(","));
  Serial.print(robotY);
  Serial.println(F(") con pasos de 9 segundos."));

  while (robotX < 2) {
    mySerial.println(MOVER_DERECHA);
    robotX++;
    delay(9000);
  }
  while (robotX > 2) {
    mySerial.println(MOVER_IZQUIERDA);
    robotX--;
    delay(9000);
  }
  while (robotY < 2) {
    mySerial.println(MOVER_ARRIBA);
    robotY++;
    delay(9000);
  }
  while (robotY > 2) {
    mySerial.println(MOVER_ABAJO);
    robotY--;
    delay(9000);
  }

  Serial.println(F("Centro alcanzado en (2,2)."));
}

// -------------------------------------------------------------------------
// FUNCIONES DE LECTURA Y PROCESAMIENTO
// -------------------------------------------------------------------------
void leerTodasColumnas() {

    int currentGlobalIndex = 0;

    for (int col = 0; col < 2; col++) {
        int slaveAddress = SLAVE_ADDRESSES[col];
        int floatsToRequest = 4;
        int bytesToRequest = floatsToRequest * sizeof(float);

        Wire.requestFrom(slaveAddress, bytesToRequest);

        FloatBytes fb;
        for (int i = 0; i < floatsToRequest; i++) {
        if (Wire.available() >= sizeof(float)) {
            for (int j = 0; j < sizeof(float); j++) {
            fb.b[j] = Wire.read();
            }
            allResistances[currentGlobalIndex++] = fb.f;
        } else {
            allResistances[currentGlobalIndex++] = -1.0;
        }
        }
        delay(10);
    }

    int blockControlSlaveAddress = SLAVE_ADDRESSES[2];
    int floatBloqueControl = 3;
    int bytesToRequest = floatBloqueControl * sizeof(float);

    Wire.requestFrom(blockControlSlaveAddress, bytesToRequest);

    FloatBytes fb;
    for (int i = 0; i < floatBloqueControl; i++) {
        if (Wire.available() >= sizeof(float)) {
        for (int j = 0; j < sizeof(float); j++) {
            fb.b[j] = Wire.read();
        }
        allResistances[currentGlobalIndex++] = fb.f;
        } else {
        allResistances[currentGlobalIndex++] = -1.0;
        }
    }
    delay(10);

    // En modo lectura/pausa mantenemos todos los LEDs al 20%.
    // Esto evita apagados al conectar o por valores transitorios de lectura.
    for (int i = 0; i < NUM_LEDS; i++) {
      if (!ledEstadoPrevio[i]) {
        setBrilloLeds(i, BRILLO_20_PORCIENTO);
        ledEstadoPrevio[i] = true;
      }
    }

    // Serial.println("----------------------------------- ");

    // for(int i=0;i<11;i++){
    //     Serial.print("Instruccion ");
    //     Serial.print(i+1);
    //     Serial.print(": ");
    //     printResistance((ActionType)(int(allResistances[i])));
    //     Serial.println("");
      
    // }
    // delay(1000);
}

void copiarArrays() {
  for (int i = 0; i < 8; i++) {
    instruccionesColumnas[i] = allResistances[i];
  }
  for (int i = 0; i < 3; i++) {
    bloqueControl[i] = allResistances[8 + i];
  }
}

void capturarSnapshotInstrucciones() {
  for (int i = 0; i < 8; i++) {
    instruccionesSnapshot[i] = instruccionesColumnas[i];
  }
  for (int i = 0; i < 3; i++) {
    bloqueControlSnapshot[i] = bloqueControl[i];
  }
}

// -------------------------------------------------------------------------
// FUNCIONES DE EJECUCIÓN DE SECUENCIA
// -------------------------------------------------------------------------
void ejecutarSiguienteInstruccion() {
  float instruccionActual = instruccionesSnapshot[actualInstruccionIndex];
  ActionType actualAction = (ActionType)instruccionActual;

  if (instruccionActual > 0) {
    if (actualInstruccionIndex == 0) {
      mySerial.println(8);
      esperarNoBloqueante(3000);
    }

    if (actualAction == BLOQUE_CONTROL) {
      if (!blockControlActivo) {
        Serial.print(F("Instruccion "));
        Serial.print(actualInstruccionIndex + 1);
        Serial.println(F(": Bloque de Control detectado "));
      } else {
        Serial.print(F("Instruccion "));
        Serial.print(actualInstruccionIndex + 1);
        Serial.print(F(": Reanudando Bloque de Control en subinstruccion "));
        Serial.println(blockControlSubIndex + 1);
      }
      ejecutarBlockControl(actualInstruccionIndex);

      // Si la subrutina quedó incompleta (por PAUSA/REINICIO/BT), no avanzar instrucción principal
      if (blockControlActivo) {
        return;
      }
    } else {
      enviarBluetooth(actualAction, actualInstruccionIndex);
    }
  } else {
    Serial.print(F("Instruccion "));
    Serial.print(actualInstruccionIndex + 1);
    Serial.println(F(": S/I "));
  }

  actualInstruccionIndex++;
}

void enviarBluetooth(ActionType action, int globalIndex) {
  Serial.print(F("Instruccion "));
  Serial.print(globalIndex + 1);
  Serial.print(F(": "));
  Serial.print(getAccionText(action));

  if (!btConnected) {
    Serial.println(F("No hay conexion a bluetooth"));
    return;
  }

  if (estadoSistemaActual != ESTADO_CORRER) {
    Serial.println(F("Ejecucion pausada/reiniciada. Accion omitida."));
    return;
  }

  int nextX = robotX;
  int nextY = robotY;

  int oldX = robotX;
  int oldY = robotY;

  if (globalIndex >= 0 && globalIndex < NUM_LEDS) {
    setBrilloLeds(globalIndex, BRILLO_100_PORCIENTO);
  }

  switch (action) {
    case MOVER_ARRIBA:
      nextY++;
      break;
    case MOVER_ABAJO:
      nextY--;
      break;
    case MOVER_IZQUIERDA:
      nextX--;
      break;
    case MOVER_DERECHA:
      nextX++;
      break;
    default:
      break;
  }

  bool accionValida = false;
  if (action == MOVER_ARRIBA || action == MOVER_ABAJO || action == MOVER_IZQUIERDA || action == MOVER_DERECHA) {
    if (validarPosicionXY(nextX, nextY)) {
      robotX = nextX;
      robotY = nextY;
      accionValida = true;
    }
  } else if (action == MELODIA_1) {
    accionValida = true;
  }

  if (accionValida) {
    Serial.print(F(": Accion valida"));
    Serial.print(F(": X - ")); Serial.print((robotX));
    Serial.print(F(" Y - ")); Serial.print((robotY));
    mySerial.println(action);
    Serial.println();
    esperarNoBloqueante(10000);
  }else{
    Serial.println(F(": Accion no valida (movimiento fuera de limites)"));
    mySerial.println(12);
    esperarNoBloqueante(3000);
    robotX = oldX;
    robotY = oldY;
  }

  if (globalIndex >= 0 && globalIndex < NUM_LEDS) {
    if (allResistances[globalIndex] > 0) {
      setBrilloLeds(globalIndex, BRILLO_20_PORCIENTO);
    } else {
      setBrilloLeds(globalIndex, BRILLO_OFF);
    }
  }
}

void ejecutarBlockControl(int globalIndexPrincipal) {
  if (!blockControlActivo || blockControlOwnerIndex != globalIndexPrincipal) {
    blockControlActivo = true;
    blockControlOwnerIndex = globalIndexPrincipal;
    blockControlSubIndex = 0;

    if (globalIndexPrincipal >= 0 && globalIndexPrincipal < NUM_LEDS) {
      setBrilloLeds(globalIndexPrincipal, BRILLO_100_PORCIENTO);
    }
  }

  for (int i = blockControlSubIndex; i < 3; i++) {
    if (!btConnected || estadoSistemaActual != ESTADO_CORRER) {
      Serial.println(F("Bloque de control interrumpido por PAUSA/REINICIO o desconexion BT."));
      blockControlSubIndex = i;
      break;
    }

    float controlRawInstruction = bloqueControlSnapshot[i];
    ActionType controlAction = (ActionType)controlRawInstruction;

    if (controlRawInstruction > 0 && controlAction != BLOQUE_CONTROL) {
      enviarBluetooth(controlAction, i + 8);
      if (!btConnected || estadoSistemaActual != ESTADO_CORRER) {
        Serial.println(F("Bloque de control detenido despues de subinstruccion."));
        blockControlSubIndex = i + 1;
        break;
      }
      blockControlSubIndex = i + 1;
    } else {
      Serial.print(F("Bloque de control "));
      Serial.print(i + 1);
      Serial.println(F(": S/I"));
      blockControlSubIndex = i + 1;
    }
  }

  // Si terminó las 3 subinstrucciones, cerrar subrutina y liberar avance de instrucción principal
  if (blockControlSubIndex >= 3) {
    if (globalIndexPrincipal >= 0 && globalIndexPrincipal < NUM_LEDS) {
      if (allResistances[globalIndexPrincipal] > 0) {
        setBrilloLeds(globalIndexPrincipal, BRILLO_20_PORCIENTO);
      } else {
        setBrilloLeds(globalIndexPrincipal, BRILLO_OFF);
      }
    }

    resetEstadoBloqueControl();
  }
}

bool validarPosicionXY(int x, int y) {
  const int GRID_MAX = 5;
  return x >= 0 && x < GRID_MAX && y >= 0 && y < GRID_MAX;
}

const __FlashStringHelper* getAccionText(ActionType action) {
  switch (action) {
    case MOVER_ARRIBA:    return F("Avanzar");
    case MOVER_ABAJO:     return F("Retroceder");
    case MOVER_IZQUIERDA: return F("Izquierda");
    case MOVER_DERECHA:   return F("Derecha");
    case BLOQUE_CONTROL:  return F("Bloque de Control");
    case MELODIA_1:       return F("Melodia");
    case DO_HOMMING:      return F("Reinicio");
    default:              return F("Ninguna instruccion / Error");
  }
}

void setBrilloLeds(int ledIndex, int brightness) {
  if (ledIndex < 0 || ledIndex >= NUM_LEDS) return;

  if (brightness < 0) brightness = 0;
  if (brightness > 4095) brightness = 4095;

  if (brightness == ledBrilloPrevio[ledIndex]) return;

  unsigned long now = millis();
  bool esCambioCritico = (brightness == BRILLO_OFF || brightness == BRILLO_100_PORCIENTO);
  if (!esCambioCritico && (now - ledUltimaActualizacion[ledIndex] < LED_UPDATE_MIN_MS)) return;

  pwm.setPWM(ledIndex, 0, brightness);
  ledBrilloPrevio[ledIndex] = brightness;
  ledUltimaActualizacion[ledIndex] = now;
}

void printResistance(ActionType action) {
  
  switch (action) {
    case MOVER_ARRIBA:
      Serial.print("ARRIBA...");
      break;
    case MOVER_ABAJO:
      Serial.print("ABAJO...");
      break;
    case MOVER_IZQUIERDA:
      Serial.print("IZQUIERDA...");
      break;
    case MOVER_DERECHA:
      Serial.print("DERECHA...");
      break;
    case MELODIA_1:
      Serial.print("Melodia 1...");
      break;
    default:
      Serial.print("Instruccion desconocida.");
      break;
  }
}
