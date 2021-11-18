// #include <Arduino.h>

// #define Light 2

// void setup() {
//   Serial.begin(9600);
//   delay(1000);
// }

// void loop() {
//   int val = analogRead(Light);
//   Serial.println(val);
//   delay(100);
// }



#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// const int SensorPin = 15;
// int soilMoistureValue = 0;
// int soilmoisturepercent = 0;

// void setup()
// {
//   Serial.begin(9600);                      // open serial port, set the baud rate to 9600 bps
 
// }

// void loop()
// {
//   soilMoistureValue = analogRead(SensorPin); //put Sensor insert into soil
//   Serial.println(soilMoistureValue);
//   delay(500);
// }

const int AirValue = 520;
const int WaterValue = 260;
const int SensorPin = 15;
int intervals = (AirValue - WaterValue) / 3;
int soilMoistureValue = 0;
void setup()
{
  Serial.begin(9600); // open serial port, set the baud rate to 9600 bps
}
void loop()
{
  soilMoistureValue = analogRead(SensorPin); //put Sensor insert into soil
  Serial.println(soilMoistureValue);
  if (soilMoistureValue > WaterValue && soilMoistureValue < (WaterValue + intervals))
  {
    Serial.println("Very Wet");
  }
  else if (soilMoistureValue > (WaterValue + intervals) && soilMoistureValue < (AirValue - intervals))
  {
    Serial.println("Wet");
  }
  else if (soilMoistureValue < AirValue && soilMoistureValue > (AirValue - intervals))
  {
    Serial.println("Dry");
  }
  delay(500);
}