#include <Arduino.h> 
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Servo mwerk;

Adafruit_BME280 bme;

const char *ssid = "ELMO";        
const char *password = "elmo123";

/* Netzwerk Details */
IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);
boolean mwstatus = false;

#define Light 39
#define gasPin 36

boolean wetter = true;
boolean swi=true;
boolean daytime = true;

const int AirValue = 3500;
const int WaterValue = 1000;
const int SensorPin = 34;
int intervals = (AirValue - WaterValue) / 3;
int soilMoistureValue = 0;

float temperature, humidity, pressure, altitude;

String SendHTML(boolean mwstat, boolean wetter, float temperature, float humidity, float pressure);
void handle_OnConnect();
void handle_mwon();
void handle_mwoff();
void handle_NotFound();

void setup()
{
  Serial.begin(9600);
  bme.begin(0x76);
  mwerk.attach(35);
  mwerk.writeMicroseconds(1500);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  server.on("/", handle_OnConnect);
  server.on("/mwon", handle_mwon);
  server.on("/mwoff", handle_mwoff);
  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop()
{
  server.handleClient();
  if (mwstatus)
  {
    mwerk.writeMicroseconds(1650);
  }
  else
  {
    mwerk.writeMicroseconds(1500);
  }

  int val = analogRead(Light);
  Serial.println("Lichtwert:");
  Serial.println(val);

  soilMoistureValue = analogRead(SensorPin);

  Serial.println("Feuchtigkeit:");
  Serial.println(soilMoistureValue);

  int gasval = analogRead(gasPin);
  Serial.println("Gaswert:");
  Serial.println(gasval);

  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;

  if(swi)
  {
    if ((soilMoistureValue > (WaterValue + intervals) && soilMoistureValue < (AirValue - intervals)))
    {
      wetter = false;
      server.send(200, "text/html", SendHTML(mwstatus, wetter, temperature, humidity, pressure));
      swi = false;
    }else if(val <= 3000){
      daytime = false;
      swi = false;
    }
  }else if((soilMoistureValue < AirValue && soilMoistureValue > (AirValue - intervals)) && val >=3000)
  {
    wetter = true;
    daytime = true;
    server.send(200, "text/html", SendHTML(mwstatus, wetter, temperature, humidity, pressure));
    swi = true;
  }
  delay(500);
}

void handle_OnConnect()
{
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  server.send(200, "text/html", SendHTML(mwstatus, wetter, temperature, humidity, pressure));
}

void handle_mwon()
{
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  mwstatus = true;
  Serial.println("Mähwerk Status: ON");
  server.send(200, "text/html", SendHTML(mwstatus, wetter, temperature, humidity, pressure));
}

void handle_mwoff()
{
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  mwstatus = false;
  Serial.println("Mähwerk Status: OFF");
  server.send(200, "text/html", SendHTML(mwstatus, wetter, temperature, humidity, pressure));
}

void handle_NotFound()
{
  server.send(404, "text/plain", "Not found");
}

String SendHTML(boolean mwstat,boolean wetter, float temperature, float humidity, float pressure)
{
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr += "<meta charset=\"UTF-8\">";
  ptr += "<meta http-equiv=\"refresh\" content=\"10\">\n";
  ptr += "<title>ELMO Control</title>\n";
  ptr += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr += "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr += ".button {display: block;width: 170px;background-color: #3498db;border: none;color: white;padding: 13px 2px;text-decoration: none;font-size: 21px;margin: 0px auto 35px;cursor: pointer;text-align: center;border-radius: 4px;}\n";
  ptr += ".button-on {background-color: #3498db;}\n";
  ptr += ".button-on:active {background-color: #2980b9;}\n";
  ptr += ".button-off {background-color: #F72020;}\n";
  ptr += ".button-off:active {background-color: #B20F0F;}\n";
  ptr += "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr += "</style>\n";
  ptr += "</head>\n";
  ptr += "<body>\n";
  ptr += "<h1>ELMO Control</h1>\n";
  ptr += "<h2>Status</h2>\n";
  if(wetter && daytime){

    ptr += "<p style=\"color:white;font-size:15px;background-color:#42D017; margin-right:30%;margin-left:30%\">Optimale Bedingungen</p>";
  }else if(!wetter){
    ptr += "<p style=\"color:white;font-size:15px;background-color:#F91111; margin-right:30%;margin-left:30%\">Achtung: Regen</p>";
  }else{
    ptr += "<p style=\"color:white;font-size:15px;background-color:#F91111; margin-right:30%;margin-left:30%\">Achtung: Dunkel</p>";
  }
  ptr += "<h2>Mähwerk</p>";

      if (mwstatus)
  {
    ptr += "<p>Mähwerk Status: ON</p><a class=\"button button-off\" href=\"/mwoff\">Ausschalten</a>\n";
  }
  else
  {
    ptr += "<p>Mähwerk Status: OFF</p><a class=\"button button-on\" href=\"/mwon\">Einschalten</a>\n";
  }
  ptr += "<h2>Sensor</h2>\n";
  ptr += "<p>Temperatur: ";
  ptr += temperature;
  ptr += "&deg;C</p>";
  ptr += "<p>Luftfeuchtigkeit: ";
  ptr += humidity;
  ptr += "%</p>";
  ptr += "<p>Pressure: ";
  ptr += pressure;
  ptr += "hPa</p>";
  ptr += "</body>\n";
  ptr += "</html>\n";
  Serial.println(ptr);
  return ptr;
}
