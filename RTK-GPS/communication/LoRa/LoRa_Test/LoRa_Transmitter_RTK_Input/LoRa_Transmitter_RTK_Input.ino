#include <SPI.h>
#include <LoRa.h>

#define ss 18
#define rst 14
#define dio0 26

#define RXD2 16
#define TXD2 17
void setup() {
Serial.begin(9600);
 while (!Serial);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
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
   while (Serial2.available()) {
  LoRa.beginPacket();
  
  
  LoRa.print(char(Serial2.read()));
  LoRa.endPacket();
 }
  

  delay(1000);
}
