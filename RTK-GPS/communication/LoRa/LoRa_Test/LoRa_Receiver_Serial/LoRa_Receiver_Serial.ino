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
  Serial.println("LoRa Receiver");
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial2.print((char)LoRa.read());
       Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
