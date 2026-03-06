// ---------------------------------------------------------------------------------------
// Código para Medidor de 4 Resistencias con Arduino Nano (ESCLAVO I2C)
// Utiliza A0 para medir Vin y A1, A2, A3, A6 para medir Vout de cada divisor.
// Nota: A4 y A5 quedan para comunicación I2C con el Arduino maestro.
// ---------------------------------------------------------------------------------------

#include <Wire.h>
#include <math.h>

// Dirección I2C única para este Arduino de columna
#define I2C_SLAVE_ADDRESS 0x01

// Pines
#define inputVoltaje A0
#define dividerInput1 A1
#define dividerInput2 A2
#define dividerInput3 A3
#define dividerInput4 A6

// Parámetros eléctricos
#define RC 1000
const float ADC_REFERENCE_VOLTAGE = 3.3;

// Cantidad de canales de instrucción por esclavo
const int NUM_CHANNELS = 4;

// Rangos de clasificación (Ohms)
const float RANGO_ARRIBA_MIN = 800.0;
const float RANGO_ARRIBA_MAX = 1600.0;

const float RANGO_ABAJO_MIN = 100.0;
const float RANGO_ABAJO_MAX = 600.0;

const float RANGO_IZQ_MIN = 3000.0;
const float RANGO_IZQ_MAX = 6000.0;

const float RANGO_DER_MIN = 1700.0;
const float RANGO_DER_MAX = 2600.0;

// Ajustado para evitar falsos positivos de fichas vacías/ruido
const float RANGO_BLOQUE_MIN = 70000.0;
const float RANGO_BLOQUE_MAX = 130000.0;

const float RANGO_MELODIA_MIN = 7000.0;
const float RANGO_MELODIA_MAX = 15000.0;

// Umbrales de validación
const float VIN_MIN_VALIDO = 0.05;
const float VOUT_MIN_CORTO = 0.005;
const float DEN_MIN = 0.01;
const float RATIO_OPEN_CIRCUIT = 1.3;
const float RES_MAX_VALIDA = 200000.0;

float measuredResistances[NUM_CHANNELS];
float valueInstruction[NUM_CHANNELS];

union FloatBytes {
  float f;
  byte b[4];
};

enum ActionType {
  MOVER_ARRIBA = 1,
  MOVER_ABAJO = 2,
  MOVER_IZQUIERDA = 3,
  MOVER_DERECHA = 4,
  BLOQUE_CONTROL = 5,
  MELODIA_1 = 6,
};

float getVoltage(int analogPin);
float getAverageVoltage(int analogPinToMeasure);
float getResistanceValue(float v2, float v1);
ActionType mapResistanceToAction(float resistanceValue);
String getAccionText(ActionType action);
void printResistance(float resistance, int rxNumber);
void getValorInstruction();
void requestEvent();

void setup() {
  Serial.begin(4800);

  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);

  Serial.println("----------------------------------");
  Serial.println(" Medidor de 4 Resistencias Activo ");
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

float getVoltage(int analogPin) {
  analogRead(analogPin);
  delayMicroseconds(200);

  int bits = analogRead(analogPin);
  float dividerVoltage = (bits * ADC_REFERENCE_VOLTAGE) / 1023.0;
  return dividerVoltage;
}

float getAverageVoltage(int analogPinToMeasure) {
  const int samples = 12;
  float sumVoltage = 0;

  for (int i = 0; i < samples; i++) {
    sumVoltage += getVoltage(analogPinToMeasure);
    delayMicroseconds(150);
  }

  return sumVoltage / samples;
}

float getResistanceValue(float v2, float v1) {
  if (v1 < VIN_MIN_VALIDO) {
    return -1.0;
  }

  if (v2 <= VOUT_MIN_CORTO) {
    return -2.0;
  }

  if ((v2 / v1) >= 11) {
    return -999.0;
  }

  float denominator = (v1 - v2);
  if (denominator <= DEN_MIN) {
    return -999.0;
  }

  float resistance = (v2 * RC) / denominator;

  if (isnan(resistance) || isinf(resistance) || resistance <= 0.0 || resistance > RES_MAX_VALIDA) {
    return -3.0;
  }

  return resistance;
}

void printResistance(float resistance, int rxNumber) {
  Serial.print("Rx");
  Serial.print(rxNumber);
  Serial.print(": ");

  if (resistance == -999.0) {
    Serial.println("Sin ficha / circuito abierto");
    return;
  }

  if (resistance < 0) {
    Serial.println("CORTO / Valor Invalido");
    return;
  }

  if (resistance >= 1000.0) {
    Serial.print(resistance / 1000.0, 2);
    Serial.print(" kOhms - ");
  } else {
    Serial.print(resistance, 2);
    Serial.print(" Ohms - ");
  }

  Serial.println(getAccionText(mapResistanceToAction(resistance)));
}

void getValorInstruction() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (measuredResistances[i] > 0) {
      valueInstruction[i] = mapResistanceToAction(measuredResistances[i]);
    } else {
      valueInstruction[i] = 0;
    }
  }
}

ActionType mapResistanceToAction(float resistanceValue) {
  if (resistanceValue <= 0) {
    return (ActionType)-1;
  }

  if (resistanceValue >= RANGO_ARRIBA_MIN && resistanceValue <= RANGO_ARRIBA_MAX) {
    return MOVER_ARRIBA;
  }

  if (resistanceValue >= RANGO_ABAJO_MIN && resistanceValue <= RANGO_ABAJO_MAX) {
    return MOVER_ABAJO;
  }

  if (resistanceValue >= RANGO_IZQ_MIN && resistanceValue <= RANGO_IZQ_MAX) {
    return MOVER_IZQUIERDA;
  }

  if (resistanceValue >= RANGO_DER_MIN && resistanceValue <= RANGO_DER_MAX) {
    return MOVER_DERECHA;
  }

  if (resistanceValue >= RANGO_MELODIA_MIN && resistanceValue <= RANGO_MELODIA_MAX) {
    return MELODIA_1;
  }

  if (resistanceValue >= RANGO_BLOQUE_MIN && resistanceValue <= RANGO_BLOQUE_MAX) {
    return BLOQUE_CONTROL;
  }

  return (ActionType)0;
}

void requestEvent() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    FloatBytes fb;

    if (valueInstruction[i] > 0) {
      fb.f = valueInstruction[i];
    } else {
      fb.f = -1;
    }

    Wire.write(fb.b, 4);
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

  float avgDividerVoltage_Rx1 = getAverageVoltage(dividerInput1);
  measuredResistances[0] = getResistanceValue(avgDividerVoltage_Rx1, avgInputVoltage);
  printResistance(measuredResistances[0], 1);

  float avgDividerVoltage_Rx2 = getAverageVoltage(dividerInput2);
  measuredResistances[1] = getResistanceValue(avgDividerVoltage_Rx2, avgInputVoltage);
  printResistance(measuredResistances[1], 2);

  float avgDividerVoltage_Rx3 = getAverageVoltage(dividerInput3);
  measuredResistances[2] = getResistanceValue(avgDividerVoltage_Rx3, avgInputVoltage);
  printResistance(measuredResistances[2], 3);

  float avgDividerVoltage_Rx4 = getAverageVoltage(dividerInput4);
  measuredResistances[3] = getResistanceValue(avgDividerVoltage_Rx4, avgInputVoltage);
  printResistance(measuredResistances[3], 4);

  getValorInstruction();

  delay(1000);
}