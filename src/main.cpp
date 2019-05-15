#include <Arduino.h>
#include <GPS_Handler.h>


GPS_Handler gps;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  gps.init(&Serial2);
  //long startMillis = millis();
  Serial.println("Waiting for connection: ");
  while(!gps.isConnected()){
    Serial.println(millis());
    delay(1000);
  }
  Serial.println("Connected!");
}

bool previousState = true;

void loop() {
  gps.update();
  if(gps.isConnected() != previousState){
    if(gps.isConnected()) Serial.println("Connection reestablished!");
    else Serial.println("Connection Lost!");
    previousState = !previousState;
  }
}
