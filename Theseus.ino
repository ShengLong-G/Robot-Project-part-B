
/*
 * Potentially usful websites
 * https://www.digikey.jp/htmldatasheets/production/1766819/0/0/1/robotics-with-the-boe-bot-student-guide.html
 */


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL6180X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// Setup the lidar -------------------------------------------------------
Adafruit_VL6180X vl = Adafruit_VL6180X();

Adafruit_SSD1306 display = Adafruit_SSD1306();

#if (SSD1306_LCDHEIGHT != 32)
 #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
//-------------------------------------------------------------------------

// Setup the robot Servo---------------------------------------------------
#include <Servo.h>  
int servoPin1 = 12; //Right
int servoPin2 = 13; //Left
Servo servoLeft;       // Declare left and right servos
Servo servoRight;
//-------------------------------------------------------------------------


// Setup the Ultrasonic Sensor---------------------------------------------
// defines pins numbers where sensor is attached
const int trigPinRIGHT = 5;
const int echoPinRIGHT = 6;
const int trigPinLEFT = 9;
const int echoPinLEFT = 10;
//-------------------------------------------------------------------------

// defines variables-------------------------------------------------------
long duration, distance, distanceLEFT, distanceRIGHT;
int j, k;
int trial = 0;  //Comment out if causing issues
int t_time[3];  // ''
float heading;
int fwdL = 1600;
int fwdR = 1400;
int rtrnL = 1535;
int rtrnR = 1500;
int ltrnL = 1500;
int ltrnR = 1465;
//-------------------------------------------------------------------------

// Assign a unique ID to this sensor at the same time 
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

void setup()
{
  pinMode(trigPinLEFT, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinLEFT, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinRIGHT, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinRIGHT, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done
  display.display();
  delay(1000);
  
  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");

  // text display big!
  display.setTextSize(4);
  display.setTextColor(WHITE);
  
  #ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println("Magnetometer Test"); Serial.println("");

  /* Enable auto-gain */
  mag.enableAutoRange(true);

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

void loop()
{
  // Initialising the readings from the lidar
  uint8_t distanceFront = vl.readRange(); // Taking measurements for the front distance
  uint8_t status = vl.readRangeStatus();  // Taking the status of the lidar, incase of an error
  Serial.print("Range: "); Serial.println(distanceFront);
  // Checking for errors from the lidar sensor.
  ErrorCheck(status);

  // Initialising the readings from Ultrasonic sensors and attaching servos
  SonarSensor(trigPinLEFT, echoPinLEFT);
  distanceLEFT = distance;
  
  servoRight.attach(servoPin2);  // Attach right signal to pin 12
  servoLeft.attach(servoPin1);   // Attach left signal to pin 13
  
  SonarSensor(trigPinRIGHT, echoPinRIGHT);
  distanceRIGHT = distance;

// Begin main program-------------------------------------------------------
  RLog(distanceFront, trial);
  
  
}

// -----------------------------Functions-----------------------------------

// Ultrasonic sensor reading function---------------------------------------
void SonarSensor(int trigPin,int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration*0.034/2;
}


// Run Logger function (NEW FUNCTION, COMMENT OUT IF ERROR)
void RLog(uint8_t distanceFront,int trial)
{
  bool Start = false;
  bool End = false;
  int runtime = 0;
  servoLeft.writeMicroseconds(fwdL);      // Move forwards to try enter maze
  servoRight.writeMicroseconds(fwdR);     
  Serial.println("RLog");
  delay(100);
  if ((distanceFront > 10) &&  (distanceLEFT < 20) && (distanceRIGHT < 20)){
    Start = true;
  }
  while ((Start = true) && (End = false)){
    runtime += 1;
    AriadnesThread(distanceFront);
    if ((distanceFront > 110) &&  (distanceLEFT > 20) && (distanceRIGHT > 20) && (runtime > 2)){
      End = true;
    }
  }
  if ((End = true) && (trial < 2)){
    t_time[trial] = runtime;
    trial += 1;
    delay (10000);
    Start = false;
    RLog(distanceFront, trial);
  }
  else if ((End = true) && (trial = 2)){
    t_time[trial] = runtime;
  }
}


// Gets current bearing------------------------------------------------------
float GetHeading(void){
   /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);
  delay(10);

  /* Get a new sensor event */
  float Pi = 3.14159;

  // Calculate the angle of the vector y,x
  heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;

  // Normalize to 0-360
  if (heading < 0) {
    heading = 360 + heading;
  }
  //Serial.println(heading);
  delay(10);
  return heading;
}


// Positions Theseus in the middle of the path-------------------------------
void Center(int R, int L){
  int scale;
  if (distanceLEFT < 4) { // getting too close to the right wall, adjust left
    scale = abs(distanceLEFT-4)*1.5;
    if (distanceLEFT < 3){
      scale = 4;
    }
    servoLeft.writeMicroseconds(L + 100*scale);
    servoRight.writeMicroseconds(R);
  } 
  
  if (distanceRIGHT < 4) { // getting too close to the left wall, adjust right
    scale = abs(distanceRIGHT-4);
    if (distanceRIGHT < 3){
      scale = 4;
    }
    servoLeft.writeMicroseconds(L);
    servoRight.writeMicroseconds(R - 100*scale);
  }
}


// Main sensor logic---------------------------------------------------------
void AriadnesThread(uint8_t distanceFront)
{
  float inithead;
  float head;
  SonarSensor(trigPinLEFT, echoPinLEFT);
  distanceLEFT = distance;
  SonarSensor(trigPinRIGHT, echoPinRIGHT);
  distanceRIGHT = distance;
  distanceFront = vl.readRange();
  //TR(distanceFront);
  GetHeading();
  if ((distanceFront < 110) && (distanceLEFT > 10) && (distanceRIGHT < 10)){     // Turns Left in a corner
    inithead = GetHeading();
    head = GetHeading();
    while (((abs(head - inithead))*2) < 60){
      servoLeft.writeMicroseconds(ltrnL);      // L-cw
      servoRight.writeMicroseconds(ltrnR);     // R-cw
      head = GetHeading();
      Center(ltrnR, ltrnL);
      Serial.println((abs(head - inithead))*2);
      delay(10);
    }
  }
  else if ((distanceFront < 110) && (distanceLEFT < 10) && (distanceRIGHT > 10)){      // Turns right in a corner
    inithead = GetHeading();
    head = GetHeading();
    while (((abs(inithead - head))*2) < 60){
      servoLeft.writeMicroseconds(rtrnL);      // L-ccw
      servoRight.writeMicroseconds(rtrnR);     // R-ccw
      head = GetHeading();
      Center(rtrnR, rtrnL);
      Serial.println(abs(inithead - head)*2);
      delay(10);
    }
  }
  else if ((distanceFront > 110)){     // Continue straight on
    FWD();
  }
  
}

void FWD(){
  servoLeft.writeMicroseconds(fwdL);      // L-ccw
    servoRight.writeMicroseconds(fwdR);     // R-cw
    Center(fwdR, fwdL);
    delay(10);

}

// Turn left at T junction---------------------------------------------------
void TL(uint8_t distanceFront) {
  if ((distanceFront < 80) && (distanceLEFT > 10) && (distanceRIGHT > 10)) {
    
      servoLeft.writeMicroseconds(1420);         // Left wheel clockwise
      servoRight.writeMicroseconds(1420);        // Right wheel anticlockwise
      delay(580);
  }
}


// Turn right at T junction--------------------------------------------------
void TR(uint8_t distanceFront) {
  if ((distanceFront < 80) && (distanceLEFT > 10) && (distanceRIGHT > 10)) {
    
      servoRight.writeMicroseconds(1580);         // Right wheel clockwise
      servoLeft.writeMicroseconds(1580);          // Left wheel anticlockwise
      delay(580);
  }
}


// Lidar error check function------------------------------------------------
void ErrorCheck(uint8_t status){
  // Some error occurred, print it out!
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range reading overflow");
  }
  delay(10);
}
