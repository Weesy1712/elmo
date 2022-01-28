#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#define address 0x1E //0011110b, I2C 7bit address of HMC5883
#include <LiquidCrystal.h> //Load Liquid Crystal Library
LiquidCrystal LCD(6, 7, 5, 4, 3, 2);  //Create Liquid Crystal Object called LCD


static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 57600;

double latitude;
double longitude;

static double waypoint[] = {46.609201, 14.305195,
                            46.608943, 14.305204,
                            46.609015, 14.306148,
                            46.609305, 14.306128,
                            46.609201, 14.305195,
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

int count = 1;
double pointing;
double shouldPoint;

int in4 = 10;
int in3 = 11;
int in2 = 12;
int in1 = 13;

// Assign a Uniquej ID to the HMC5883 Compass Sensor
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the NEO-6m GPS module
SoftwareSerial ss(RXPin, TXPin);

void displaySensorDetails(void) {
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(100);
}

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  LCD.begin(16, 2); //Tell Arduino to start your 16 column 2 row LCD
  LCD.setCursor(0, 0); //Set LCD cursor to upper left corner, column 0, row 0
  LCD.print("Starting :D");  //Print Message on First Row
  LCD.setCursor(0, 1); //Set LCD cursor to upper left corner, column 0, row 0
  LCD.print("Starting :D");  //Print Message on First Row

  Serial.println(F("Simple Test with TinyGPS++ and attached NEO-6M GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
  displaySensorDetails();

  Wire.begin();
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}
void multiplePoint(double a1, double b1, double a2, double b2, double a3, double b3, double a4, double b4, double a5, double b5) ;
void fromPointToPoint(double x, double y, double a, double b);
void turnDegrees(double x, double y, double a, double b);
void turnToWhichSide(double x, double a);
void displayGpsInfo();
void lcdclear();
void calcDist(double x, double y, double a, double b);
void displayCompassInfo();
void backward();
void forward();
void stopTheEngine();
void left();
void right();


void loop() {
  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      Serial.println("");
      displayGpsInfo();
      lcdclear();
      //multiplePoint(finalX, finalY, finalX2, finalY2, finalX3, finalY3);
      multiplePoint(finalX, finalY, finalX2, finalY2, finalX3, finalY3, finalX4, finalY4, finalX5, finalY5);
    }
  }
}

void multiplePoint(double a1, double b1, double a2, double b2, double a3, double b3, double a4, double b4, double a5, double b5) {
  if (count == 1) {
    fromPointToPoint(latitude, longitude, a1, b1);
    LCD.setCursor(0, 0); //Set LCD cursor to upper left corner, column 0, row 0
    LCD.print("->1st");
  } else if (count == 2) {
    fromPointToPoint(latitude, longitude, a2, b2);
    LCD.setCursor(0, 0); //Set LCD cursor to upper left corner, column 0, row 0
    LCD.print("->2nd");
  } else if (count == 3) {
    fromPointToPoint(latitude, longitude, a3, b3);
    LCD.setCursor(0, 0); //Set LCD cursor to upper left corner, column 0, row 0
    LCD.print("->3rd");
  } else if (count == 4) {
    fromPointToPoint(latitude, longitude, a4, b4);
    LCD.setCursor(0, 0); //Set LCD cursor to upper left corner, column 0, row 0
    LCD.print("->4rd");
  } else if (count == 5) {
    fromPointToPoint(latitude, longitude, a5, b5);
    LCD.setCursor(0, 0); //Set LCD cursor to upper left corner, column 0, row 0
    LCD.print("->5rd");
  } else
    stopTheEngine();
}

void fromPointToPoint(double x, double y, double a, double b) {
  if (x < a + 0.0000096 && x > a - 0.0000096 && y < b + 0.0000096 && y > b - 0.0000096) { //is the car at the fin.dest?
    count++;
  } else {
    turnDegrees(x, y, a, b);//calculates/thinks where the car should point
    Serial.print("Should Point:");
    Serial.print(shouldPoint);
    Serial.println("");

    LCD.setCursor(0, 1); //Set LCD cursor to upper left corner, column 0, row 0
    LCD.print("tH:");
    LCD.print(shouldPoint);

    LCD.setCursor(8, 1); //Set LCD cursor to upper left corner, column 0, row 0
    LCD.print("cH:");
    LCD.print(pointing);

    calcDist(x, y, a, b);

    if (pointing > shouldPoint - 20 && pointing < shouldPoint + 20 ) { //is it already pointing at that point?
      forward();
      Serial.println("Matching/Just go fwd");
    } else {
      turnToWhichSide(pointing, shouldPoint);//look the fin.dest way
      Serial.println("Need to turn");
    }
  }
}

void turnDegrees(double x, double y, double a, double b) { //calculated where the car should point
  double num1 = double (fabs(x));
  double num2 = double (fabs(y));
  double num3 = double (fabs(a));
  double num4 = double (fabs(b));

  Serial.println("");
  if (x < a && y < b) { // top right
    Serial.println("Option4");
    shouldPoint = 270 + ( atan2 (fabs(num4 - num2), fabs(num3 - num1)) * 180.0 / 3.14159265 );
  } else if (x < a && b < y) { // top left
    Serial.println("Option1");
    shouldPoint = 90.0 - ( atan2 (fabs(num4 - num2), fabs(num3 - num1)) * 180.0 / 3.14159265 );
  } else if (a < x && b < y) { // bottom left
    Serial.println("Option2");
    shouldPoint = 180.0 - ( atan2 ( fabs( num3 - num1 ) , fabs(num4 - num2) )  * 180.0 / 3.14159265 );
  } else if (a < x && y < b) { //bottom right
    Serial.println("Option3");
    shouldPoint = 180.0 + ( atan2 ( fabs( num3 - num1 ) , fabs(num4 - num2) )  * 180.0 / 3.14159265 );
  } else {
    Serial.println("No Option");
  }
}

void turnToWhichSide(double x, double a) { //calculates turning to which side is the shortest
  if (180 < x) {
    if (x - 180 < a && a < x) {
      right();
      forward();
      Serial.println("Testing: Turn 1RIGHT") ;
    } else {
      left();
      forward();
      Serial.println("Testing: Turn 1LEFT") ;
    }
  } else { //right
    if (x + 180 > a && a > x) {
      left();
      forward();
      Serial.println("Testing: Turn 2LEFT") ;
    } else {
      right();
      forward();
      Serial.println("Testing: Turn 2RIGHT") ;
    }
  }
}

void calcDist(double x, double y, double a, double b) {
  double num1 = double (fabs(x));
  double num2 = double (fabs(y));
  double num3 = double (fabs(a));
  double num4 = double (fabs(b));

  //Each degree of  latitudeX is approximately 69 miles=111.045 kilometers=4,371,840 Inches=4,371,850.393701 Inches=364320ft
  //Each degree of longitudeY is approximately 69.172 miles=111.321543 kilometers=4382737.913386inches=365228.16ft
  double xDiff = double (fabs(num1 - num3));//long
  double yDiff = double (fabs(num2 - num4)); //lat
  //now to inches
  xDiff = xDiff * 364320;
  yDiff = yDiff * 365228.16;
  double dist = sqrt(xDiff * xDiff + yDiff * yDiff);
  LCD.setCursor(8, 0); //Set LCD cursor to upper left corner, column 0, row 0
  LCD.print(dist);
  LCD.print("ft");
}

void lcdclear() {
  LCD.setCursor(0, 0); //Set LCD cursor to upper left corner, column 0, row 0
  LCD.print("                 ");
  LCD.setCursor(0, 1); //Set LCD cursor to upper left corner, column 0, row 0
  LCD.print("                 ");
}

void displayGpsInfo() {
  ss.begin(GPSBaud);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  //Prints the location if lat-lng information was recieved
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 12);
    latitude = gps.location.lat();
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 12);
    longitude = gps.location.lng();
  }
  // prints invalid if no information was recieved in regards to location.
  else
  {
    Serial.print(F("INVALID"));
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // prints the recieved GPS module date if it was decoded in a valid response.
  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    // prints invalid otherwise.
    Serial.print(F("INVALID"));
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // prints the recieved GPS module time if it was decoded in a valid response.
  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    // Print invalid otherwise.
    Serial.print(F("INVALID"));
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  displayCompassInfo();

}

void displayCompassInfo() {
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  //    Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  //    Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  //    Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  "); Serial.println("uT");
  //
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  float declinationAngle = 0.22;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;

  double a = 214.2;
  double b = 264.0;
  double c = 321.0;
  double d = 360.0;

  if (headingDegrees < a )
    headingDegrees *= (90.0 / a);
  else if (headingDegrees >= a && headingDegrees < b)
    headingDegrees = 90.0 + (headingDegrees - a) * (90.0 / (b - a));
  else if (headingDegrees >= b && headingDegrees < c)
    headingDegrees = 180.0 + (headingDegrees - b) * (90.0 / (c - b));
  else if (headingDegrees >= c && headingDegrees < d)
    headingDegrees = 270.0 + (headingDegrees - c) * (90.0 / (d - c));

  Serial.println("");
  Serial.print("Heading (degrees): ");
  Serial.print(headingDegrees);
  pointing = headingDegrees;
}

void backward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void forward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void stopTheEngine() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void left() {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void right() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
