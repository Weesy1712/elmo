#include <Arduino.h> 
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

#define RXD2 16
#define TXD2 17 

    Servo ESCL, ESCR;



//static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 57600;

double latitude;
double longitude;

static double waypoint[] = {
    46.610754, 14.304656,
46.610789, 14.304984,
46.610873, 14.304972,
46.610789, 14.304975,
46.610754, 14.304653,

};
double finalX = waypoint[0];
double finalY = waypoint[1];

double finalX2 = waypoint[2];
double finalY2 = waypoint[3];

double finalX3 = waypoint[4];
double finalY3 = waypoint[5];

double finalX4 = waypoint[6];
double finalY4 = waypoint[7];

double finalX5 = waypoint[8];
double finalY5 = waypoint[9];

int cnt = 0;
int x, y, z;
long xx, yy, zz;

int count = 2;
double pointing;
double shouldPoint;

/* Assign a unique ID to the sensors */
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

void initSensors()
{
  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1)
      ;
  }
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }
}

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the NEO-6m GPS module
//SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(9600);
  Serial2.begin(GPSBaud, SERIAL_8N1, RXD2, TXD2);
 // ss.begin(GPSBaud);

  initSensors();

  ESCL.attach(2);
  ESCR.attach(4);
  ESCL.writeMicroseconds(1500);
  ESCR.writeMicroseconds(1500);

  Serial.println(F("Simple Test with TinyGPS++ and attached NEO-6M GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();

}
void multiplePoint(double a1, double b1, double a2, double b2, double a3, double b3, double a4, double b4, double a5, double b5);
void fromPointToPoint(double x, double y, double a, double b);
void turnDegrees(double x, double y, double a, double b);
void turnToWhichSide(double x, double a);
void displayGpsInfo();
void calcDist(double x, double y, double a, double b);
void displayCompassInfo();
void backward();
void forward();
void stopTheEngine();
void left();
void right();

void loop()
{

    //    while (ss.available() > 0)
      //if (gps.encode(ss.read()))
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
        displayGpsInfo();
        multiplePoint(finalX, finalY, finalX2, finalY2, finalX3, finalY3, finalX4, finalY4, finalX5, finalY5);
}

void multiplePoint(double a1, double b1, double a2, double b2, double a3, double b3, double a4, double b4, double a5, double b5)
{
  if (count == 1)
  {
    fromPointToPoint(latitude, longitude, a1, b1);
  }
  else if (count == 2)
  {
    fromPointToPoint(latitude, longitude, a2, b2);
  }
  else if (count == 3)
  {
    fromPointToPoint(latitude, longitude, a3, b3);
  }
  else if (count == 4)
  {
    fromPointToPoint(latitude, longitude, a4, b4);
  }
  else if (count == 5)
  {
    fromPointToPoint(latitude, longitude, a5, b5);
  }
  else
    stopTheEngine();
}

void fromPointToPoint(double x, double y, double a, double b)
{
  if (x < a + 0.0000096 && x > a - 0.0000096 && y < b + 0.0000096 && y > b - 0.0000096)
  { //is the car at the fin.dest?
    count++;
  }
  else
  {
    turnDegrees(x, y, a, b); //calculates/thinks where the car should point
    Serial.print("Should Point:");
    Serial.print(shouldPoint);
    Serial.print(count);
    Serial.println("");

    if (pointing > shouldPoint - 20 && pointing < shouldPoint + 20)
    { //is it already pointing at that point?
      forward();
      Serial.println("Matching/Just go fwd");
    }
    else
    {
      turnToWhichSide(pointing, shouldPoint); //look the fin.dest way
      Serial.println("Need to turn");
    }
  }
}

void turnDegrees(double MyLat, double MyLong, double TargetLat, double TargetLong){

  //Calculate the posititon from degrees to rad 
  MyLat = MyLat *PI/180;
  MyLong = MyLong*PI/180;
  TargetLat = TargetLat*PI/180;
  TargetLong = TargetLong*PI/180;

  double dLon = (TargetLong - MyLong);

  double y = sin(dLon) * cos(TargetLat);
  double x = cos(MyLat) * sin(TargetLat) - sin(MyLat) * cos(TargetLat) * cos(dLon);

  double brng = atan2(y, x);


  //calculate from rad to degree
  brng = brng * 180 / PI;

  if (brng < 0)
  {
    brng += 360;
  }

  shouldPoint=  brng;
}

void turnToWhichSide(double x, double a)
{ //calculates turning to which side is the shortest
  if (180 < x)
  {
    if (x - 180 < a && a < x)
    {
      right();
      Serial.println("Testing: Turn 1RIGHT");
    }
    else
    {
      left();
      Serial.println("Testing: Turn 1LEFT");
    }
  }
  else
  { //right
    if (x + 180 > a && a > x)
    {
      left();
      Serial.println("Testing: Turn 2LEFT");
    }
    else
    {
      right();
      Serial.println("Testing: Turn 2RIGHT");
    }
  }
}

/*
void turnToWhichSide(double x, double a)
{ //calculates turning to which side
  double delta = x-a;
  if(delta > 180) delta -= 360;
  if(delta < -180) delta += 360;

  if (delta < -2)
  {
    left();
    Serial.println("Turn RIGHT");
  }
  else if(delta > 2)
  {
    right();
    Serial.println("Turn LEFT");    
  }
}
*/
void calcDist(double x, double y, double a, double b)
{
  double num1 = double(fabs(x));
  double num2 = double(fabs(y));
  double num3 = double(fabs(a));
  double num4 = double(fabs(b));

  //Each degree of  latitudeX is approximately 69 miles=111.045 kilometers=4,371,840 Inches=4,371,850.393701 Inches=364320ft
  //Each degree of longitudeY is approximately 69.172 miles=111.321543 kilometers=4382737.913386inches=365228.16ft
  double xDiff = double(fabs(num1 - num3)); //long
  double yDiff = double(fabs(num2 - num4)); //lat
  //now to inches
  xDiff = xDiff * 364320;
  yDiff = yDiff * 365228.16;
  double dist = sqrt(xDiff * xDiff + yDiff * yDiff);
  Serial.print("Distance: "); 
  Serial.println(dist); 
}

void displayGpsInfo()
{
/*
latitude = 46.6106835670165; 
longitude = 14.304572032092713;


  Serial.print(F("Location: "));

    Serial.print(latitude, 12);
    latitude = latitude;
    Serial.print(F(","));
    Serial.print(longitude, 12);
    longitude = longitude;
    Serial.println("");
*/
  Serial.print(F("Location: "));

 if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 12);
    latitude = gps.location.lat();
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 12);
    longitude = gps.location.lng();
    Serial.println("");

    
  }
  else
  {
    Serial.println("NO GPS SIGNAL");
  }
  displayCompassInfo();
}



void displayCompassInfo()
{
  /* Get a new sensor event */
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t orientation;

  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; "));
  }
  Serial.println(F(""));
  pointing = orientation.heading;
}

void stopTheEngine()
{
  
  ESCL.writeMicroseconds(1500);
  ESCR.writeMicroseconds(1500);
  
}

void forward()
{
  ESCL.writeMicroseconds(1630);
  ESCR.writeMicroseconds(1630);
  
}

void left()
{
  
  stopTheEngine();
  delay(1000);
  ESCL.writeMicroseconds(1630);
  ESCR.writeMicroseconds(1500);
  delay(500);
  stopTheEngine();
  
}

void right()
{
  
  stopTheEngine();
  delay(1000);
  ESCL.writeMicroseconds(1500);
  ESCR.writeMicroseconds(1650);
  delay(500);
 stopTheEngine();
  
}

void evadeRicht(){

}

void evadeLeft(){
  
}