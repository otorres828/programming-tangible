// ---------------------------------------------------------------------------------------
// Código para Arduino Nano Central (MAESTRO I2C) - Archivo Único
// Solicita datos de 2 Arduinos de columna (Esclavos I2C, 4 resistencias cada uno)
// y un tercer Arduino (Esclavo I2C, Bloque de Control de 3resistencias).
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
const int SLAVE_ADDRESSES[] = { 0x01, 0x02, 0x03 };

// Arrays para almacenar los datos
float allResistances[11];         // Almacena todas las 11 resistencias leídas
float instruccionesColumnas[8];  // Las primeras 8 instrucciones (de las 2 columnas)
float bloqueControl[3];           // El bloque de control (las últimas 4 posiciones)

// --- DEFINICIONES DE ACCIONES ---
enum ActionType {
  MOVER_ARRIBA = 1,     // Resistencia 800-1600 Ohms                   - 1K
  MOVER_ABAJO = 2,      // Resistencia 100-600 Ohms                    - 220omh 
  MOVER_IZQUIERDA = 3,  // Resistencia 3k-6k Ohms                      - 4.7k
  MOVER_DERECHA = 4,    // Resistencia 1.7k-2.6k Ohms                  - 2k
  BLOQUE_CONTROL = 5,   // Resistencia 70k-110k Omhs                   - 100k
  MELODIA_1 = 6,         // Resistencia 7k-15k Ohms                    - 10k
  DO_HOMMING = 7,         
};

// --- UNIÓN PARA CONVERTIR FLOAT A BYTES Y VICEVERSA ---
union FloatBytes {
  float f;    // El valor flotante
  byte b[4];  // Sus 4 bytes constituyentes
};

// --- ESTADO GLOBAL DEL ROBOT ---
int robotX = 0;  // Posición actual en X (columna de la cuadrícula 0-4)
int robotY = 0;  // Posición actual en Y (fila de la cuadrícula 0-4)

// Crea un objeto PCA9685 , con Direccion 0x40 para comunicacion I2C con el driver Pca9685 que controlara los LEDS de las instrucciones
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Define el número total de LEDs que vas a controlar
const int NUM_LEDS = 11; // 8 instrucciones de columnas + 3 de bloque de control

// Define los valores de brillo (PWM)
// El PCA9685 tiene una resolución de 12 bits, lo que significa un rango de 0 a 4095.
const int BRILLO_OFF           = 0;    // LED apagado
const int BRILLO_20_PORCIENTO  = 100;  // 20% de 4095
const int BRILLO_100_PORCIENTO = 4095; // LED al máximo brillo

// ---  VARIABLES GLOBALES PARA LA MÁQUINA DE ESTADOS Y BOTÓN ---
enum SystemState { ESTADO_LEER, ESTADO_CORRER, ESTADO_PAUSA, ESTADO_REINICIO };
SystemState estadoSistemaActual = ESTADO_LEER;
int actualInstruccionIndex = 0; // Índice de la instrucción actual en la secuencia principal (0-9)

// Variables para el manejo del botón (Debouncing y Pulsación Larga)
unsigned long lastButtonStateChangeTime = 0;
unsigned long buttonPressStartTime = 0;
bool lastButtonReading = HIGH; //  INPUT_PULLUP, HIGH cuando no está presionado
bool currentButtonReading = HIGH;
bool longPressTriggered = false; // Flag para saber si una pulsación larga ya fue manejada

const unsigned long BUTTON_DEBOUNCE_DELAY = 50; // ms de delay para debounce
const unsigned long LONG_PRESS_THRESHOLD = 1500; // 1.5 segundos para pulsación larga

// Variable para el retardo no bloqueante entre acciones
unsigned long lastActionExecutionTime = 0;
const unsigned long DELAY_POR_INSTRUCCION = 100; // 100 ms de retraso entre la ejecución de acciones

// --- Opciones de detección de conexión Bluetooth ---
#define BT_STATE_PIN A2
const int ledVerde = A0;  // LED de Conectado
const int ledRojo = A1;   // LED de Desconectado
bool conectado = false;   // LED state helper

// Variables de estado para detección
unsigned long lastBtReceiveMillis = 0;
bool btConnected = false; // estado lógico (basado en STATE pin o en actividad)
bool homingDone = false;  // Bandera para evitar ejecutar homing más de una vez por conexión

// Variables para debounce del pin STATE
int btRawLast = LOW;
int btStableState = LOW;
unsigned long btLastDebounceTime = 0;
const unsigned long BT_STATE_DEBOUNCE_MS = 100; // tiempo de debounce para pin STATE (ms)


// --- DECLARACIÓN DE FUNCIONES---
void leerTodasColumnas();                                  // Leer los datos de los 3 esclavos I2C (2 columnas y bloque de control)
void copiarArrays();                                       // Copiar los datos leídos a las variables globales
void botonPulsaciones();                                   // Manejar el botón
void ejecutarSiguienteInstruccion();                       // Ejecutar una sola instrucción
void enviarBluetooth(ActionType action, int globalIndex);  // Ejecutar una acción específica
void ejecutarBlockControl(int globalIndexPrincipal);       // Ejecutar la lógica del bloque de control
void setBrilloLeds(int ledIndex, int brightness);          // Establecer el brillo de un LED
bool validarPosicionXY(int x, int y);                      // Validar si una posición (x,y) es válida en la cuadrícula
String getAccionText(ActionType action);                   // Obtener el texto descriptivo de una acción - Lo usamos para debuguear

int manejoLedBT(int stableState);

void setup() {

  Serial.begin(9600);  // Para comunicación con el Monitor Serial del PC

  pinMode(BT_STATE_PIN, INPUT);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledRojo, OUTPUT);

  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  Wire.begin();  // Inicia la comunicación I2C como Maestro

  // Inicia el PCA9685 y la comunicación I2C / Establecemos la frecuencia.
  pwm.begin();
  pwm.setPWMFreq(1600);

  for (int q = 0; q < NUM_LEDS; q++) {
    pwm.setPWM(q, 0, BRILLO_20_PORCIENTO); // Canal i, ON_time = 0, OFF_time = BRILLO_OFF
  }

  pinMode(BOTON_INICIO, INPUT_PULLUP); // Configura el pin del botón con resistencia pull-up interna

  Serial.println("----------------------------------");
  Serial.println(" Arduino Central (Maestro I2C) ");
  Serial.println("----------------------------------");
  Serial.println("Esperando conexion por bluetooth...");

  mySerial.begin(9600); // Colocamos el módulo bluetooth en 9600

  // Inicializar estado BT (lectura inicial + debounce vars + LEDs)
  int s = digitalRead(BT_STATE_PIN);
  btConnected = (s == HIGH);
  btRawLast = s;
  btStableState = s;
  btLastDebounceTime = millis();
  manejoLedBT(s);

  // Evitar inicio falso al arrancar
  while (digitalRead(BOTON_INICIO) == LOW) { delay(5); }
  lastButtonReading = HIGH;
  currentButtonReading = HIGH;
}

void loop() {


  // Leer pin STATE con debounce
  int raw = digitalRead(BT_STATE_PIN);
  if (raw != btRawLast) {
    btLastDebounceTime = millis();
    btRawLast = raw;
  }

  // El estado se considera estable
  if (raw != btStableState) {
      btStableState = raw;
      // Actualizar LEDs al cambiar el estado estable
      manejoLedBT(btStableState);
      if (btStableState == HIGH && !btConnected) {
        btConnected = true;
        Serial.println("Bluetooth MAESTRO: conectado (STATE pin). Enviando homing...");
        if (!homingDone) {
          // Enviar comando de homing al esclavo al reconectar
          // mySerial.println(7); // DO_HOMMING
          homingDone = true;
        }
      } else if (btStableState == LOW && btConnected) {
        btConnected = false;
        homingDone = false; // permitir homing en la próxima conexión
        Serial.println("Bluetooth MAESTRO: desconectado (STATE pin).");
        // Limpiar buffer BT saliente/entrante
        while (mySerial.available()) { mySerial.read(); }
      }
  }

  // si se desconecta el bluetooth no deberia de mandar mas instrucciones
  if (!btConnected) {
    return;
  }

  botonPulsaciones();// Procesa las entradas del botón en cada ciclo del loop

  switch (estadoSistemaActual) {
    case ESTADO_LEER:
    case ESTADO_PAUSA:
      // Cuando está en LEER, sigue leyendo las fichas para actualizar los LEDs al 20%  y prepararse para el inicio.
      // En el estado PAUSA, no se ejecutan nuevas acciones de movimiento.
      leerTodasColumnas();
      copiarArrays();
      break;

    case ESTADO_REINICIO:
      // Estado temporal para realizar el reinicio lógico del CNC.
      // Enviar el dígito 7 (MELODIA_1 según el enum) al CNC por Bluetooth.
      Serial.println("Estado REINICIO: enviando '7' al CNC para reinicio/accion.");
      mySerial.println(7); // envía 7, el CNC ejecutará su caso DO_HOMMING
      // Dar un momento para que el mensaje salga y el esclavo lo procese
      delay(100);
      // Volver a estado de lectura (IDLE) y asegurar que se vuelva a leer fichas
      estadoSistemaActual = ESTADO_LEER;
      // Reiniciar índice y tiempos por si se desea iniciar después
      actualInstruccionIndex = 0;
      lastActionExecutionTime = millis();
      break;

    case ESTADO_CORRER:

      // Si la secuencia está corriendo y ha pasado suficiente tiempo desde la última acción
      if (millis() - lastActionExecutionTime >= DELAY_POR_INSTRUCCION) {

        if (actualInstruccionIndex < 8) { // Si aún quedan instrucciones en la secuencia principal (2 columnas × 4)

          ejecutarSiguienteInstruccion(); // Ejecuta la siguiente instrucción
          lastActionExecutionTime = millis(); // Actualiza el tiempo de la última acción ejecutada

        } else {

          // La secuencia principal ha terminado
          Serial.println("Secuencia principal finalizada.");
          mySerial.println(11); // Reproducir recorrido terminado
          estadoSistemaActual = ESTADO_LEER; // Vuelve al estado IDLE
          actualInstruccionIndex = 0; // Reinicia el índice para la próxima ejecución

          for (int i = 0; i < NUM_LEDS; i++) {
            setBrilloLeds(i, BRILLO_20_PORCIENTO);
          }
        }
      }
      break;
  }

  
}

// ------------------------------------------------------------------------
// FUNCION MANEJO DE LED BLUETOOTH (usa estado DEBOUNCEADO)
// ------------------------------------------------------------------------
int manejoLedBT(int stableState){
  bool prev = conectado;
  // Set LEDs deterministically based on debounced state
  if (stableState == HIGH) {
    digitalWrite(ledVerde, HIGH);
    digitalWrite(ledRojo, LOW);
    conectado = true;
  } else {
    digitalWrite(ledVerde, LOW);
    digitalWrite(ledRojo, HIGH);
    conectado = false;
  }

  // Log only on transitions
  if (conectado && !prev) {
    Serial.println(">>> ESTADO: CONECTADO <<<");
    Serial.println("Estado actual: IDLE (esperando boton para iniciar)");
  } else if (!conectado && prev) {
    Serial.println(">>> ESTADO: DESCONECTADO <<<");
  }

  return conectado ? 1 : 0;
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
            Serial.println("Boton: INICIO de secuencia.");
            estadoSistemaActual = ESTADO_CORRER;
            actualInstruccionIndex = 0; // Iniciar desde la primera instrucción
            lastActionExecutionTime = millis(); // Preparar el temporizador para la primera acción
          } else if (estadoSistemaActual == ESTADO_CORRER) {
            Serial.println("Boton: PAUSA de secuencia.");
            estadoSistemaActual = ESTADO_PAUSA;
          } else if (estadoSistemaActual == ESTADO_PAUSA) {
            Serial.println("Boton: REANUDAR secuencia.");
            estadoSistemaActual = ESTADO_CORRER;
            lastActionExecutionTime = millis(); // Re-preparar el temporizador para reanudar
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
      actualInstruccionIndex = 0;
      for (int i = 0; i < NUM_LEDS; i++) {
        setBrilloLeds(i, BRILLO_20_PORCIENTO);
      }
      mySerial.print(10); // Enviar comando de homing al CNC (INIT_HOMMING)
      delay(6000);
      estadoSistemaActual = ESTADO_LEER;
      robotX = 0;
      robotY = 0;
    }
  }

  lastButtonReading = reading;
}

// -------------------------------------------------------------------------
// FUNCIONES DE LECTURA Y PROCESAMIENTO
// -------------------------------------------------------------------------

// Lee los datos de los 3 esclavos I2C y llena el array 'allResistances'
void leerTodasColumnas() {
  int currentGlobalIndex = 0; // Para llevar el control de la posición en allResistances

  // 1. Leer de los dos primeros esclavos (columnas), cada uno con 4 resistencias
  for (int col = 0; col < 2; col++) { // Para las 2 columnas
    int slaveAddress = SLAVE_ADDRESSES[col];
    int floatsToRequest = 4; // Cada columna envía 4 floats
    int bytesToRequest = floatsToRequest * sizeof(float); // 4 floats * 4 bytes/float = 16 bytes

    Wire.requestFrom(slaveAddress, bytesToRequest);

    FloatBytes fb;
    for (int i = 0; i < floatsToRequest; i++) {
      if (Wire.available() >= sizeof(float)) {
        for (int j = 0; j < sizeof(float); j++) {
          fb.b[j] = Wire.read();
        }
        allResistances[currentGlobalIndex++] = fb.f;
      } else {
        allResistances[currentGlobalIndex++] = -1.0; // Valor por defecto si no hay datos
      }
    }
    delay(10); // Pequeña pausa para I2C
  }

  // 2. Leer del tercer esclavo (bloque de control), con 4 resistencias
  int blockControlSlaveAddress = SLAVE_ADDRESSES[2];    // El tercer esclavo
  int floatBloqueControl = 3;                           // El bloque de control tiene 4 resistencias
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
      allResistances[currentGlobalIndex++] = -1.0; // Valor por defecto si no hay datos
    }
  }
  delay(10); // Pequeña pausa para I2C

  // Actualizar LEDs al 20% para las fichas presentes cuando se lee en IDLE/PAUSED
  // Esta lógica ya está en el loop(), pero la ponemos aquí para ser explícitos
  // que los LEDs se actualizan cada vez que se leen las resistencias.
  for (int i = 0; i < NUM_LEDS; i++) {
    if (allResistances[i] > 0) {
      setBrilloLeds(i, BRILLO_20_PORCIENTO);
    } else {
      setBrilloLeds(i, BRILLO_OFF);
    }
  }

  // Imprimir todas las instrucciones leidas
  // Serial.println("-----------------------------------------------------------");
  // Serial.println("Instrucciones leidas:");
  // for (int i = 0; i < NUM_LEDS; i++) {
  //   Serial.print("IDX ");
  //   Serial.print(i + 1);
  //   Serial.print(": ");
  //   Serial.print(allResistances[i]);
  //   Serial.print(" -> ");
  //   if (allResistances[i] > 0) {
  //     ActionType action = (ActionType)((int)allResistances[i]);+      
  //     Serial.println(getAccionText(action));
  //   } else {
  //     Serial.println("Sin instruccion");
  //   }
  // }
  // delay(1000);

}

// Copiar los valores después de leer todas las columnas a sus arrays específicos
void copiarArrays() {
  
  // Copia las primeras 8 posiciones a instruccionesColumnas (las 2 columnas de 4 resistencias)
  for (int i = 0; i < 8; i++) {
    instruccionesColumnas[i] = allResistances[i];
  }
  // Copia las últimas 3 posiciones a bloqueControl (el bloque de control)
  for (int i = 0; i < 3; i++) {
    bloqueControl[i] = allResistances[8 + i]; // Empieza en el índice 8
  }
}

// -------------------------------------------------------------------------
// FUNCIONES DE EJECUCIÓN DE SECUENCIA
// -------------------------------------------------------------------------

// Esta función ejecuta UNA SOLA instrucción de la secuencia principal (la actual de actualInstruccionIndex)
void ejecutarSiguienteInstruccion() {

  float instruccionActual = instruccionesColumnas[actualInstruccionIndex];
  ActionType actualAction = (ActionType)instruccionActual; // Castear a enum

  // Solo procesar si la instrucción es válida (resistencia positiva)
  if (instruccionActual > 0) {

      if(actualInstruccionIndex == 0){
        //Si es la primera instruccion mandamos a reproducir el audio de inicio de recorrido
        mySerial.println(8); // Enviar comando de melodía de inicio al CNC
        delay(3000);

      }
      if (actualAction == BLOQUE_CONTROL) {

        Serial.print("Instruccion "); Serial.print(actualInstruccionIndex + 1); Serial.println("- Bloque de Control detectado "); 
        ejecutarBlockControl(actualInstruccionIndex); // Esta función es bloqueante.
        
      } else {
        
        // Para todas las demás acciones (movimiento, melodía)
        enviarBluetooth(actualAction, actualInstruccionIndex);

      }
    

  }else{

    Serial.print("Instruccion "); Serial.print(actualInstruccionIndex + 1);
    Serial.println(": S/I "); 

  }
  // El incremento de actualInstruccionIndex lo maneja el loop()
  // para que pueda pausar y reanudar.
  actualInstruccionIndex++; // Mueve al siguiente índice de instrucción
}

// Ejecuta una acción específica basada en el ActionType.
void enviarBluetooth(ActionType action, int globalIndex) {

  Serial.print("Instruccion ");
  Serial.print(globalIndex + 1);
  Serial.print(": ");
  Serial.println(getAccionText(action));

  if (!btConnected) {
    Serial.println("No hay conexion a bluetooth");
    return;
  }
  int nextX = robotX;
  int nextY = robotY;

  // Enciende el LED de la ficha al 100% durante la ejecución
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
  // Actualiza la posición del robot solo si fue una acción de movimiento válida
  if (action == MOVER_ARRIBA || action == MOVER_ABAJO || action == MOVER_IZQUIERDA || action == MOVER_DERECHA) {
    if (validarPosicionXY(nextX, nextY)) {
      robotX = nextX;
      robotY = nextY;
      accionValida = true;
    }else{
      accionValida = false;
    }
  }else if(action == MELODIA_1){
    accionValida = true; // Siempre válida para melodía
  }

  if(accionValida){
    Serial.print("Accion valida"); // Salto de línea para mejor legibilidad en el monitor serial
    // Envía la acción por Bluetooth al CNC solo si el movimiento es valido
    mySerial.println(action);
    Serial.println(); // Salto de línea para mejor legibilidad en el monitor serial
    delay(9000); //Esperar 9 segundos entre cada instrucción para que el CNC tenga tiempo de ejecutar y el usuario pueda ver la acción

  }

  // Baja el brillo del LED de la ficha de nuevo al 20% después de ejecutar la acción
  // (si la ficha sigue presente)
  if (globalIndex >= 0 && globalIndex < NUM_LEDS) {
    if (allResistances[globalIndex] > 0) { // Verifica si la ficha aún es considerada válida
      setBrilloLeds(globalIndex, BRILLO_20_PORCIENTO);
    } else {
      setBrilloLeds(globalIndex, BRILLO_OFF); // Si la ficha fue quitada, apagar completamente
    }
  }


}

// Ejecuta la lógica del bloque de control, procesando las 3 resistencias de bloqueControl.
// Esta función es BLOQUEANTE: termina toda la subrutina antes de volver.
void ejecutarBlockControl(int globalIndexPrincipal) {

  // Encender LED de la ficha principal BLOQUE_CONTROL mientras corre la subrutina completa
  if (globalIndexPrincipal >= 0 && globalIndexPrincipal < NUM_LEDS) {
    setBrilloLeds(globalIndexPrincipal, BRILLO_100_PORCIENTO);
  }

  for (int i = 0; i < 3; i++) {
    float controlRawInstruction = bloqueControl[i];
    ActionType controlAction = (ActionType)controlRawInstruction;

    if (controlRawInstruction > 0 && controlAction != BLOQUE_CONTROL) {
      // LED del bloque de control (índices 8,9,10) se enciende/apaga dentro de enviarBluetooth
      enviarBluetooth(controlAction, i + 8);
    } else {
      Serial.print("Bloque de control ");
      Serial.print(i + 1);
      Serial.println(": S/I");
    }
  }

  // Devolver brillo de la ficha principal BLOQUE_CONTROL a estado normal
  if (globalIndexPrincipal >= 0 && globalIndexPrincipal < NUM_LEDS) {
    if (allResistances[globalIndexPrincipal] > 0) {
      setBrilloLeds(globalIndexPrincipal, BRILLO_20_PORCIENTO);
    } else {
      setBrilloLeds(globalIndexPrincipal, BRILLO_OFF);
    }
  }
}

// Verifica si una posición (x, y) está dentro de los límites de la cuadrícula.
bool validarPosicionXY(int x, int y) {
  int GRID_SIZE = 5;
  return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
}

// Devuelve el tipo de acción en formato de texto para el monitor serial
String getAccionText(ActionType action) {
  switch (action) {
    case MOVER_ARRIBA:    return "Avanzar";
    case MOVER_ABAJO:     return "Retroceder";
    case MOVER_IZQUIERDA: return "Izquierda";
    case MOVER_DERECHA:   return "Derecha";
    case BLOQUE_CONTROL:  return "Bloque Control";
    case MELODIA_1:       return "Melodia";
    case 7:               return "Reinicio";
    default:              return "Ninguna instruccion / Error";
  }
}

//  controla el brillo de un LED en el PCA9685
void setBrilloLeds(int ledIndex, int brightness) {
  if (ledIndex >= 0 && ledIndex < NUM_LEDS) {
    pwm.setPWM(ledIndex, 0, brightness);
  }
}