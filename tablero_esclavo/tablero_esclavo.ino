// ---------------------------------------------------------------------------------------
// Código para Medidor de 5 Resistencias con Arduino Nano (ESCLAVO I2C)
// Utiliza A0 para medir Vin y A1, A2, A3, A6, A7 para medir Vout de cada divisor.
// Muestra los valores de las resistencias en el Monitor Serial.
// Nota: Utilizamos A4 y A5 para la comunicacion I2C con el Arduino Central.
// ---------------------------------------------------------------------------------------

#include <Wire.h>  // Librería para comunicación I2C

// Dirección I2C ÚNICA para cada Arduino de columna.
#define I2C_SLAVE_ADDRESS 0x01  // <--- COLUMNA 1 (Cambiar para otros esclavos)

// --- DEFINICIONES DE PINES Y VALORES DE REFERENCIA ---

// Pin para medir el voltaje de entrada (Vin) del divisor. Conectar a 3V3 del Arduino.
#define inputVoltaje A0

// Pines para medir el voltaje de salida (Vout) de cada divisor.
// Conectar cada uno al punto medio de su respectivo divisor (entre R1 y Rx).
#define dividerInput1 A1
#define dividerInput2 A2
#define dividerInput3 A3
#define dividerInput4 A6

// Valor de la resistencia de referencia conocida (R1) en Ohmios (Ω).
#define RC 1000  // Resistencia de Referencia = 1 kOhmio (1000 Ohms)

// Voltaje de referencia del ADC. Si tu Arduino Nano está alimentado a 5V y usas la referencia por defecto,
// esto debería ser 5.0. Si lo alimentas a 3.3V o usas una referencia externa de 3.3V, entonces 3.3 es correcto.
const float ADC_REFERENCE_VOLTAGE = 3.3;  // Voltaje de referencia del ADC del Arduino

float measuredResistances[4];  // Array para guardar Rx1, Rx2, Rx3, Rx4
float valueInstruction[4];   // Array el valor de las instrucciones

// --- UNIÓN PARA CONVERTIR FLOAT A BYTES Y VICEVERSA ---
// Esto es necesario porque Wire.write() y Wire.read() operan con bytes.
union FloatBytes {
  float f;
  byte b[4]; // Un float ocupa 4 bytes
};


// --- DEFINICIONES DE ACCIONES ---
enum ActionType {
  MOVER_ARRIBA = 1,     // Resistencia 800-1600 Ohms                   - 1K
  MOVER_ABAJO = 2,      // Resistencia 100-600 Ohms                    - 220omh 
  MOVER_IZQUIERDA = 3,  // Resistencia 3k-6k Ohms                      - 4.7k
  MOVER_DERECHA = 4,    // Resistencia 1.7k-2.6k Ohms                  - 2k
  BLOQUE_CONTROL = 5,   // Resistencia 40k-190k Omhs                   - 100k
  MELODIA_1 = 6,         // Resistencia 7k-15k Ohms                    - 10k
};


void setup() {
  Serial.begin(4800);  // Inicia la comunicación serial para depuración

  // Inicia la comunicación I2C como ESCLAVO con la dirección asignada.
  Wire.begin(I2C_SLAVE_ADDRESS);
  // Registra la función que se llamará cuando el Maestro solicite datos.
  Wire.onRequest(requestEvent);

  // Mensajes iniciales informativos en el Monitor Serial.
  Serial.println("----------------------------------");
  Serial.println("  Medidor de 4 Resistencias Activo ");
  Serial.println("----------------------------------");
  Serial.print("Voltaje de referencia ADC: ");
  Serial.print(ADC_REFERENCE_VOLTAGE, 1);
  Serial.println(" V");
  Serial.print("Resistencia de referencia (R1): ");
  Serial.print(RC);
  Serial.println(" Ohmios");
  Serial.print("Direccion I2C: 0x");
  Serial.println(I2C_SLAVE_ADDRESS, HEX);
}

// Convierte el valor analógico en voltaje
float getVoltage(int analogPin) {
  int bits = analogRead(analogPin);
  // delay(100); // Eliminado: Este delay es excesivo y ralentiza las lecturas
  float dividerVoltage = (bits * ADC_REFERENCE_VOLTAGE) / 1023.0;  // 0-1023 es el rango del ADC
  return dividerVoltage;
}

// Obtiene un promedio del voltaje obtenido
float getAverageVoltage(int analogPinToMeasure) {
  float sumVoltage = 0;
  for (int i = 0; i < 10; i++) {  // Promedia 10 lecturas
    sumVoltage += getVoltage(analogPinToMeasure);
  }
  return sumVoltage / 10.0;
}

//Obtiene el valor de la resustencia
float getResistanceValue(float v2, float v1) {

  // Manejo de casos especiales para evitar divisiones por cero o resultados erróneos.
  if (v1 < 0.01) { // Si Vin es muy bajo, la fuente no está conectada o hay un error.
    return -1.0; // Código de error: Vin inválido
  }

  if ((v1 - v2) < 0.001) { // Si (Vin - Vout) es casi cero, significa que Vout ~ Vin,
                           // lo que implica que Rx es muy pequeña o un cortocircuito.
    return -2.0; // Código de error: Corto Circuito / Valor Inválido
  }

  if (v2 < 0.001) { // Si Vout es casi cero, Rx es extremadamente grande (circuito abierto).
    return -999.0; // Código de error: Circuito Abierto
  }

  // Fórmula para calcular Rx: Rx = R1 * (Vout / (Vin - Vout))
  float resistance = (v2 * RC) / (v1 - v2);

  if(resistance > 190000.0) { // Si la resistencia calculada es mayor a 130kOhm, se considera un error.
    return -3.0; // Código de error: Corto Circuito / Valor Inválido
  }

  return resistance;
}

//Imprime el valor de las resistencias
void printResistance(float resistance, int rxNumber) {

  Serial.print("Rx");
  Serial.print(rxNumber);
  Serial.print(": ");


  if (resistance == -2.0 || resistance < -3.0 || resistance < 0) { // Incluye negativos por ruido
    Serial.println("CORTO / Valor Invalido");
  } else {

    // Formatear la resistencia para mostrar en Ohms, kOhms o MOhms
      if (resistance >= 1000.0) { // Mayor o igual a 1 kOhms
      Serial.print(resistance / 1000.0, 2);
      Serial.print(" kOhms - ");
      Serial.println(getAccionText(mapResistanceToAction(resistance)));
      } else { // Menor de 1 kOhms
      Serial.print(resistance, 2);
      Serial.print(" Ohms - ");
      Serial.println(getAccionText(mapResistanceToAction(resistance)));

    }
    
  }
}

//obtiene el valor de cada instruccion
void getValorInstruction(){

  for (int i = 0; i < 4; i++) {
   
    if(measuredResistances[i]>0){
       valueInstruction[i] = mapResistanceToAction(measuredResistances[i]);  // Mapea la resistencia a un tipo de acción
    }else{
      valueInstruction[i] = -1;
    }
    
  }
}

// Mapea un valor de resistencia a un tipo de acción (no ejecuta, solo clasifica).
ActionType mapResistanceToAction(float resistanceValue) {

  if (resistanceValue >= 800.0 && resistanceValue <= 1600.0) {
      return MOVER_ARRIBA;
  } 
  else if (resistanceValue >= 100.0 && resistanceValue <= 600.0) {
      return MOVER_ABAJO;
  } 
  else if (resistanceValue >= 3000.0 && resistanceValue <= 6000.0) {
      return MOVER_IZQUIERDA;
  } 
  else if (resistanceValue >= 1700.0 && resistanceValue <= 2600.0) {
      return MOVER_DERECHA;
  } 
  else if (resistanceValue >= 40000.0 && resistanceValue <= 190000.0) {
      return BLOQUE_CONTROL;    
  } 
  else if (resistanceValue >= 7000.0 && resistanceValue <= 15000.0) {
      return MELODIA_1;
  } 
  else if (resistanceValue <= 0) {
      return (ActionType)-1;  // Valor de error para indicar corto/valor inválido
  } 
  else {
      return (ActionType)0;   // Si es un valor positivo pero no cae en ningún rango definido
  }

}

// Esta función se llama automáticamente cuando el Arduino Maestro solicita datos.
void requestEvent() {
  // Convertir cada float en el array a 4 bytes y enviarlos.
  // El Maestro espera 4 floats (4 * 4 bytes/float = 16 bytes).
  for (int i = 0; i < 4; i++) {
    FloatBytes fb;

    //nos aseguramos que solo mandemos isntancias válidas
    // Si la resistencia medida es válida (mayor que 0), enviamos su valor
    if(valueInstruction[i]>0){
      fb.f = valueInstruction[i];
    }else{
      // Si es -999.0, -1.0, -2.0 o -3.0,  Enviamos -1 como valor de error si la resistencia no es válida  
      fb.f = -1; 
    }
    Wire.write(fb.b, 4); // Envía los 4 bytes de cada float
  }
}

String getAccionText(ActionType action) {
  switch (action) {
    case MOVER_ARRIBA:    return "Avanzar";
    case MOVER_ABAJO:     return "Retroceder";
    case MOVER_IZQUIERDA: return "Izquierda";
    case MOVER_DERECHA:   return "Derecha";
    case BLOQUE_CONTROL:  return "Bloque de Control";
    case MELODIA_1:       return "Melodia";
    default:              return "Ninguna instruccion / Error";
  }
}

void loop() {
  Serial.println("--------------------------------");
  float avgInputVoltage = getAverageVoltage(inputVoltaje);
  Serial.print("Vin Promedio: ");
  Serial.print(avgInputVoltage, 2);
  Serial.println(" V");

  // Medir y calcular cada resistencia Rx, y controlar su LED correspondiente.
  float avgDividerVoltage_Rx1 = getAverageVoltage(dividerInput1);
  measuredResistances[0] = getResistanceValue(avgDividerVoltage_Rx1, avgInputVoltage);  // v_out, v_in
  printResistance(measuredResistances[0], 1);

  float avgDividerVoltage_Rx2 = getAverageVoltage(dividerInput2);
  measuredResistances[1] = getResistanceValue(avgDividerVoltage_Rx2, avgInputVoltage);  // v_out, v_in
  printResistance(measuredResistances[1], 2);

  float avgDividerVoltage_Rx3 = getAverageVoltage(dividerInput3);
  measuredResistances[2] = getResistanceValue(avgDividerVoltage_Rx3, avgInputVoltage);  // v_out, v_in
  printResistance(measuredResistances[2], 3);

  float avgDividerVoltage_Rx4 = getAverageVoltage(dividerInput4);
  measuredResistances[3] = getResistanceValue(avgDividerVoltage_Rx4, avgInputVoltage);  // v_out, v_in
  printResistance(measuredResistances[3], 4);

  getValorInstruction();

  delay(1000);  // Espera 1 segundo antes de la siguiente ronda de mediciones
  
}