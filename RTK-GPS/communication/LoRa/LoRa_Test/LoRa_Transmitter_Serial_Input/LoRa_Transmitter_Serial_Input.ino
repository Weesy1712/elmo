#include <SPI.h>
#include <LoRa.h>

#define ss 18
#define rst 14
#define dio0 26


void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Sender");
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  
 

  // send packet
   while (Serial.available()) {
  LoRa.beginPacket();
  
  
  LoRa.print(char(Serial.read()));
  LoRa.endPacket();
 }
  

  delay(1000);
}
