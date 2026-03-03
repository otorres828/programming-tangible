#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

SoftwareSerial mySoftwareSerial(12,13); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

void setup() {
  Serial.begin(9600);
  mySoftwareSerial.begin(9600);

  Serial.println("Iniciando DFPlayer...");

  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println("Error: Revisa conexiones o tarjeta SD.");
    while(true);
  }

  Serial.println("DFPlayer Online.");
  myDFPlayer.volume(20);
  myDFPlayer.play(1);
}

void loop() {
}
