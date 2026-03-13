// ---------------------------------------------------------------------------------------
// Código para Arduino Nano Esclavo (CNC Control de Motores) - cnc_esclavo.ino
// Recibe instrucciones del Arduino Maestro vía Bluetooth y controla 2 motores paso a paso.
// Adaptado para un sistema H-BOT.
// Incluye proceso de homing/calibración con finales de carrera (sensor de efecto hall) y reproducción de audio.
// ACTUALIZADO: Control no bloqueante con micros(), velocidad unificada y secuencia de DOBLE BOBINA para MÁXIMO TORQUE.
// ---------------------------------------------------------------------------------------

#include <SoftwareSerial.h> // Librería para comunicación Bluetooth
#include <DFRobotDFPlayerMini.h> // Librería para DFPlayer Mini (la oficial de DFRobot)

// --- Pines del módulo Bluetooth HC-05 ---
SoftwareSerial bluetoothSerial(10, 11); // RX, TX para Bluetooth (conectado a TX, RX del CNC)

// --- Pines para SoftSerial del DFPlayer Mini ---
SoftwareSerial myMp3Serial(12, 13); // RX, TX para DFPlayer (conectado a TX, RX del DFPlayer)

// Objeto del DFPlayer Mini (DFRobotDFPlayerMini)
DFRobotDFPlayerMini myDFPlayer;

// --- Opciones de detección de conexión Bluetooth ---
#define BT_STATE_PIN A2
const int ledVerde = A3;  // LED de Conectado
const int ledRojo = A4;   // LED de Desconectado
bool conectado = false;

// Variables de estado para detección
unsigned long lastBtReceiveMillis = 0;
bool btConnected = false; // estado lógico (basado en STATE pin o en actividad)
bool homingDone = false; // Bandera para evitar ejecutar homing más de una vez por conexión

// Variables para debounce del pin STATE
int btRawLast = LOW;
int btStableState = LOW;
unsigned long btLastDebounceTime = 0;
const unsigned long BT_STATE_DEBOUNCE_MS = 200; // tiempo de debounce para pin STATE (ms)

// --- Definiciones para los motores 28BYJ-48 con ULN2003 ---
// Motor 1 (izquierda)
#define IN1_M1 2
#define IN2_M1 3
#define IN3_M1 4
#define IN4_M1 5
// Motor 2 (derecha)
#define IN1_M2 6
#define IN2_M2 7
#define IN3_M2 8
#define IN4_M2 9

// Pasos por revolución para el 28BYJ-48 (con factor de reducción de 1/64)
const int STEPS_PER_REVOLUTION = 2048;

// --- Definición de pines para Finales de Carrera ---
#define ENDSTOP_X_PIN A0 // Pin para el final de carrera del Eje X
#define ENDSTOP_Y_PIN A1 // Pin para el final de carrera del Eje Y

// --- Velocidad de los motores (UNIFICADA) ---
// Reducida a 10 RPM para garantizar que el motor tenga la fuerza (torque) suficiente y no se atasque
const int MOTOR_SPEED_RPM = 10; 

// --- Variables para control de motores con micros() ---
long motorM1Remaining = 0;
long motorM2Remaining = 0;
int currentStepM1 = 0;
int currentStepM2 = 0;
int directionM1 = 1;
int directionM2 = 1;
unsigned long lastMotorStep = 0;
unsigned long motorIntervalMicros = 2000; // Se calculará dinámicamente

// --- Guarda el ultimo movimiento
int calibration = 0;

bool dfPlayerListo = false; // estado del MP3 Player (si se pudo inicializar correctamente)

// --- Definiciones de Acciones ---
enum ActionType {
  MOVER_ARRIBA = 1,
  MOVER_ABAJO = 2,
  MOVER_IZQUIERDA = 3,
  MOVER_DERECHA = 4,
  CONEXION_PERDIDA = 5,
  MELODIA_1 = 6,
  DO_HOMMING = 7,
  INICIO_CORERRIDO = 8,
  HOMMING_COMPLETO = 9,
  INIT_HOMMING = 10,
  RECORRIDO_TERMINADO = 11,
  NO_VALIDA = 12,
  CENTRO = 13,
  CENTRO_TERMINADO = 14
};

// --- Array de secuencias para el motor 28BYJ-48 (DOBLE BOBINA para MAXIMO TORQUE) ---
const int steps[][4] = {
  {1, 1, 0, 0}, // Paso 1 (IN1 e IN2 ON)
  {0, 1, 1, 0}, // Paso 2 (IN2 e IN3 ON)
  {0, 0, 1, 1}, // Paso 3 (IN3 e IN4 ON)
  {1, 0, 0, 1}  // Paso 4 (IN4 e IN1 ON)
};

const int NUM_STEPS_SEQUENCE = 4;

// --- CABECERAS DE FUNCIONES ---
void setMotorPins(int in1, int in2, int in3, int in4, int stepIndex);
void startMoveHbot(int stepsX, int stepsY, int motorSpeedRpm);
void updateMotors();
void moveHbot(int stepsX, int stepsY, int motorSpeedRpm);
void doHoming(bool playAudio = true);
void playInstructionAudio(ActionType action);
void executeAction(ActionType action);
void calibrationX(ActionType currentAction);
int manejoLedBT(int stableState);

// --- SETUP Y LOOP ---
void setup() {

  // iniciar ambos SoftSerial
  Serial.begin(115200); // Inicializar puerto Serial para Monitor Serial
  bluetoothSerial.begin(9600);  //Inicializar Bluetooth Serial
  myMp3Serial.begin(9600); // Inicializar  DFPlayer

  Serial.println("CNC Esclavo Iniciado.");

  // Configuracion DFPlayer Mini
  myMp3Serial.listen(); 
  delay(500);
  
  if (myDFPlayer.begin(myMp3Serial, false, false)) { 
    Serial.println(F("✅ DFPlayer: Conectado (Modo rápido)"));
    dfPlayerListo = true;
    myDFPlayer.volume(30);
    delay(100);
    myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  } else {
    Serial.println(F("❌ DFPlayer: Error de inicio."));
  }

  bluetoothSerial.listen();
  Serial.println(F("✅ Bluetooth: Escuchando..."));

  // Inicializar pin STATE
  pinMode(BT_STATE_PIN, INPUT);
  int s = digitalRead(BT_STATE_PIN);
  btConnected = (s == HIGH);

  //leds que indican conexion con bluetooth
  pinMode(ledVerde, OUTPUT);
  pinMode(ledRojo, OUTPUT);

  // Inicializar debounce/estados
  btRawLast = s;
  btStableState = s;
  btLastDebounceTime = millis();
  Serial.print("BT_STATE_PIN inicial: "); Serial.println(s);
  
  manejoLedBT(s);

  // Configurar pines de motores
  pinMode(IN1_M1, OUTPUT);
  pinMode(IN2_M1, OUTPUT);
  pinMode(IN3_M1, OUTPUT);
  pinMode(IN4_M1, OUTPUT);
  pinMode(IN1_M2, OUTPUT);
  pinMode(IN2_M2, OUTPUT);
  pinMode(IN3_M2, OUTPUT);
  pinMode(IN4_M2, OUTPUT);

  // Configurar pines de finales de carrera
  pinMode(ENDSTOP_X_PIN, INPUT_PULLUP);
  pinMode(ENDSTOP_Y_PIN, INPUT_PULLUP);

  Serial.println("✅ FINAL DE LA CONFIGURACION INICIAL...");
}

void loop() {

  // Leer pin STATE con debounce
  int raw = digitalRead(BT_STATE_PIN);
  if (raw != btRawLast) {
      btLastDebounceTime = millis();
      btRawLast = raw;
  }

  if (millis() - btLastDebounceTime > BT_STATE_DEBOUNCE_MS) {

      if (raw != btStableState) {
        Serial.println("Bluetooth Estable...");
        btStableState = raw;
        manejoLedBT(btStableState);
        
        if (btStableState == HIGH && !btConnected) {
          btConnected = true;
          Serial.println("Bluetooth: conectado (STATE pin). Ejecutando homing...");
          if (!homingDone) {
            doHoming(); 
            homingDone = true;
          }
        } else if (btStableState == LOW && btConnected) {
          btConnected = false;
          homingDone = false; 
          Serial.println("Bluetooth: desconectado (STATE pin).");
          playInstructionAudio(CONEXION_PERDIDA);
        }
      }
  }
  
  if (btConnected) {
      if (bluetoothSerial.isListening() && bluetoothSerial.available() > 0) {
          bluetoothSerial.setTimeout(50); 
          String receivedAction = bluetoothSerial.readStringUntil(':'); 
          receivedAction.trim();

          Serial.print("Comando recibido: ");
          Serial.println(receivedAction);

          if (receivedAction.length() > 0 ) { 
              ActionType action = (ActionType)receivedAction.toInt();
              executeAction(action);
          }
      }
  } else {
    while (bluetoothSerial.available()) {
      bluetoothSerial.read();
    }
  }
}

// ------------------------------------------------------------------------
// FUNCION MANEJO DE LED BLUETOOTH
// ------------------------------------------------------------------------
int manejoLedBT(int stableState){
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
    Serial.println(">>> ESTADO: CONECTADO <<<");
  } else if (!conectado && prev) {
    Serial.println(">>> ESTADO: DESCONECTADO <<<");
  }

  return conectado ? 1 : 0;
}

void setMotorPins(int in1, int in2, int in3, int in4, int stepIndex) {
  digitalWrite(in1, steps[stepIndex][0]);
  digitalWrite(in2, steps[stepIndex][1]);
  digitalWrite(in3, steps[stepIndex][2]);
  digitalWrite(in4, steps[stepIndex][3]);
}

// ------------------------------------------------------------------------
// NUEVAS FUNCIONES DE CONTROL DE MOTORES
// ------------------------------------------------------------------------
void startMoveHbot(int stepsX, int stepsY, int motorSpeedRpm) {
  if (motorSpeedRpm <= 0) motorSpeedRpm = 1;
  
  unsigned long denom = (unsigned long)STEPS_PER_REVOLUTION * (unsigned long)motorSpeedRpm;
  motorIntervalMicros = 60000000UL / denom; 

  long stepsM1 = stepsY - stepsX;
  long stepsM2 = stepsY + stepsX;

  directionM1 = (stepsM1 > 0) ? 1 : -1;
  directionM2 = (stepsM2 > 0) ? 1 : -1;

  motorM1Remaining = abs(stepsM1);
  motorM2Remaining = abs(stepsM2);
}

void updateMotors() {
  if (motorM1Remaining == 0 && motorM2Remaining == 0) return;

  if (micros() - lastMotorStep < motorIntervalMicros) return;
  lastMotorStep = micros();

  if (motorM1Remaining > 0) {
    setMotorPins(IN1_M1, IN2_M1, IN3_M1, IN4_M1, currentStepM1);
    currentStepM1 = (currentStepM1 + directionM1 + NUM_STEPS_SEQUENCE) % NUM_STEPS_SEQUENCE;
    motorM1Remaining--;
  }

  if (motorM2Remaining > 0) {
    setMotorPins(IN1_M2, IN2_M2, IN3_M2, IN4_M2, currentStepM2);
    currentStepM2 = (currentStepM2 + directionM2 + NUM_STEPS_SEQUENCE) % NUM_STEPS_SEQUENCE;
    motorM2Remaining--;
  }

  // Importante: al terminar, apaga las bobinas para que no se sobrecalienten
  if (motorM1Remaining == 0 && motorM2Remaining == 0) {
    digitalWrite(IN1_M1, LOW); digitalWrite(IN2_M1, LOW); digitalWrite(IN3_M1, LOW); digitalWrite(IN4_M1, LOW);
    digitalWrite(IN1_M2, LOW); digitalWrite(IN2_M2, LOW); digitalWrite(IN3_M2, LOW); digitalWrite(IN4_M2, LOW);
  }
}

void moveHbot(int stepsX, int stepsY, int motorSpeedRpm) {
  startMoveHbot(stepsX, stepsY, motorSpeedRpm);
  while (motorM1Remaining > 0 || motorM2Remaining > 0) {
    updateMotors();
  }
}

void doHoming(bool playAudio = true) {

  if(playAudio){
    playInstructionAudio(DO_HOMMING);
  }
  Serial.println("Iniciando Homing (H-Bot)...");
  const int steps = 50;

  Serial.println("Homing Eje Y (moviendo ABAJO)...");
  playInstructionAudio(MOVER_ABAJO);
  while (digitalRead(ENDSTOP_Y_PIN) == HIGH) {
    moveHbot(steps, 0, MOTOR_SPEED_RPM);
  }

  delay(1000);
  Serial.println("Final de carrera Y alcanzado.");
  moveHbot(steps - 20, 0, MOTOR_SPEED_RPM);
  delay(1500);

  Serial.println("Homing Eje X (moviendo izquierda)...");
  playInstructionAudio(MOVER_IZQUIERDA);
  while (digitalRead(ENDSTOP_X_PIN) == HIGH) {
    moveHbot(0, -steps, MOTOR_SPEED_RPM);
  }

  delay(1000);
  Serial.println("Final de carrera X alcanzado.");
  moveHbot(0, steps - 20, MOTOR_SPEED_RPM);

  doCenter();
  // playInstructionAudio(HOMMING_COMPLETO);
  Serial.println("Homing Completo.");
}

void doCenter(){

  const int steps = 3070;
  const int movementX = 60;

  const int movimientoDerecha = (steps + movementX)*2;
  const int movimientoArriba = steps*2;

  Serial.println("Reiniciando al Centro (3,3)...");
  
  Serial.println("Moviendo DERECHA...");
  moveHbot(0, (movimientoDerecha), MOTOR_SPEED_RPM); 

  Serial.println("Moviendo ARRIBA...");
  moveHbot(-movimientoArriba, 0, MOTOR_SPEED_RPM);

  Serial.println("Reinicio al Centro Completo.");
  playInstructionAudio(CENTRO_TERMINADO);
}

void playInstructionAudio(ActionType action) {
  int trackNumber = 0;
  switch (action) {
    case MOVER_ARRIBA:    trackNumber = 1; break;
    case MOVER_ABAJO:     trackNumber = 2; break;
    case MOVER_IZQUIERDA: trackNumber = 3; break;
    case MOVER_DERECHA:   trackNumber = 4; break;
    case CONEXION_PERDIDA:trackNumber = 5; break;
    case MELODIA_1:       trackNumber = 6; break; 
    case DO_HOMMING:      trackNumber = 7; break;
    case INICIO_CORERRIDO:trackNumber = 8; break;
    case HOMMING_COMPLETO:trackNumber = 9; break;
    case INIT_HOMMING:    trackNumber = 10; break;
    case RECORRIDO_TERMINADO: trackNumber = 11; break;
    case NO_VALIDA:       trackNumber = 12; break;
    case CENTRO:          trackNumber = 13; break;
    case CENTRO_TERMINADO:trackNumber = 11; break;
    default:              return;
  }

  if(dfPlayerListo) {
    Serial.print("Reproduciendo audio para acción: ");
    Serial.println((int)action);

    myMp3Serial.listen();
    delay(30);
    myDFPlayer.play(trackNumber);
    if(trackNumber == DO_HOMMING){
      delay(5500);
    }

    delay(30); 
    bluetoothSerial.listen();
    
    while(bluetoothSerial.available() > 0) { bluetoothSerial.read(); }
    
  } else {
    Serial.println("DFPlayer no listo.");
  }
}

void executeAction(ActionType action) {
  
  playInstructionAudio(action);
  calibrationX(action);
  
  const int steps = 3070;
  const int movementX = 60;

  switch (action) {
    case MOVER_ARRIBA:
      Serial.println("Moviendo ARRIBA...");
      moveHbot(-steps, 0, MOTOR_SPEED_RPM);
      break;
    case MOVER_ABAJO:
      Serial.println("Moviendo ABAJO...");
      moveHbot(steps, 0, MOTOR_SPEED_RPM);
      break;
    case MOVER_IZQUIERDA:
      Serial.println("Moviendo IZQUIERDA...");
      moveHbot(0, -(steps + movementX), MOTOR_SPEED_RPM); 
      break;
    case MOVER_DERECHA:
      Serial.println("Moviendo DERECHA...");
      moveHbot(0, (steps + movementX), MOTOR_SPEED_RPM); 
      break;
    case MELODIA_1:
      Serial.println("Reproduciendo Melodia 1...");
      break;
    case DO_HOMMING:
    case INIT_HOMMING:
      doHoming(false); 
      break;
    case CENTRO:
      doCenter(); 
      break;
    default:
      Serial.println("Instruccion desconocida.");
      break;
  }
}

void calibrationX(ActionType currentAction) {
  
    if (currentAction != MOVER_IZQUIERDA && currentAction != MOVER_DERECHA) {
        return; 
    }

    if (calibration != 0 && calibration != currentAction) {
        const int CALIBRATION_STEPS = 200; 
        Serial.println("Aplicando calibración por cambio de dirección...");

        if (currentAction == MOVER_IZQUIERDA) {
            moveHbot(0, -CALIBRATION_STEPS, MOTOR_SPEED_RPM); 
        } else if (currentAction == MOVER_DERECHA) {
            moveHbot(0, CALIBRATION_STEPS, MOTOR_SPEED_RPM); 
        }
    }

    calibration = currentAction;
}