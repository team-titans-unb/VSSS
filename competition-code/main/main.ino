#include "Robot.h"

Robot neymar("0",0,11);
Robot messi("1", 1, 10);
Robot cristiano("2", 2, 7);

void setup() {
      Serial.begin(9600);
      Serial.print(neymar.getIp());
}

void loop() {
    // Debug
    Serial.print("Neymar ip: ");
    Serial.println(neymar.getIp());

    Serial.print("Messi ip: ");
    Serial.println(messi.getIp());

    Serial.print("Cristiano ip: ");
    Serial.println(cristiano.getIp());
    delay(2000);
}