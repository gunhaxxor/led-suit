#include "debug.h"
#include "helpers.h"
// #include "printHelpers.h"

#include <Adafruit_NeoPixel.h>

#include <Adafruit_Sensor.h>

#include <i2c_t3.h>
#include <Adafruit_BNO055_t3.h>

// #include <Wire.h>
// #include <Adafruit_BNO055.h>

#include "RF24.h"
#include "nRF24L01.h"
#include <SPI.h>

// #include <Bounce.h>

// TODO to try get pose estimation from IMU
// 1. clip acc values to zero below a certain threshold
// 2. orient the linear acc vector with absolute orientation quaternion
// 3. Special algorithm:
//  - converge speed vector towards zero correlated to how much the acc vector have rotated the last few moments. Much turning gives quicker convergence.
//  - maaaaybe "delay" the zeroing to when the acc vector reaches zero. So speed vector converges towards zero when both acc vector is zero and acc vector turning has been high.


#include "button.h"

void button1interrupt();
void button2interrupt();
Button_Class button[] = {
    Button_Class(3, true, button1interrupt),
    Button_Class(4, true, button2interrupt),
};
void button1interrupt() { button[0].interrupt(); }
void button2interrupt() { button[1].interrupt(); }
const int nrOfButtons = sizeof(button) / sizeof(button[0]);

const int armStripLeftPin = 5;
const int armStripRightPin = 6;
const int legStripLeftPin = 7;
const int legStripRightPin = 8;
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
Adafruit_NeoPixel armStripLeft = Adafruit_NeoPixel(200, armStripLeftPin, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel armStripRight = Adafruit_NeoPixel(200, armStripRightPin, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel legStripLeft = Adafruit_NeoPixel(200, legStripLeftPin, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel legStripRight = Adafruit_NeoPixel(200, legStripRightPin, NEO_GRB + NEO_KHZ800);

// #define PIN 6
// Adafruit_NeoPixel strip = Adafruit_NeoPixel(30, PIN, NEO_GRB + NEO_KHZ800);


Adafruit_NeoPixel* allStrips[] = {&armStripLeft, &armStripRight, &legStripLeft, &legStripRight};
const int nrOfStrips = sizeof(allStrips) / sizeof(allStrips[0]);

elapsedMicros sinceLastLoop = 0;
unsigned long dt = 0;

elapsedMillis now = 0;

elapsedMillis sincePrint = 0;
unsigned long printInterval = 5;

elapsedMillis sinceHeartBeat = 0;
unsigned long heartBeatInterval = 100;

// Orientation sensor stuff
// Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
const int bnoAddressPin = 20;
const int bnoPowerPin = 22;
Adafruit_BNO055 bno =
    Adafruit_BNO055(WIRE_BUS, -1, BNO055_ADDRESS_B, I2C_MASTER, I2C_PINS_18_19,
                    I2C_PULLUP_EXT, I2C_RATE_400);
                    
// Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Quaternion quat;
imu::Vector<3> eul;
vec3 euler;
imu::Vector<3> acc;
vec3 rawAcc;

quaternion absoluteOrientation = {1.0f, 0, 0, 0};
quaternion referenceQuaternion = {1.0f, 0, 0, 0};
bool referenceQuaternionSet = false;
quaternion currentQuaternion = {1.0f, 0, 0, 0};

void setup()
{
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  delay(1000);
  printf("Starting LED-Suit firmware \n");

  for (int i = 0; i < nrOfStrips; i++)
  {
    printf("Setting up strip nr %i \n", i);
    allStrips[i]->begin();
    allStrips[i]->clear();
    allStrips[i]->show();
  }

  // Setup pins for BNO
  pinMode(bnoAddressPin, OUTPUT);
  digitalWrite(bnoAddressPin, HIGH);
  pinMode(bnoPowerPin, OUTPUT);
  digitalWrite(bnoPowerPin, HIGH);
  delay(10);
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  bno.setExtCrystalUse(true);
  
  /* Display the current temperature */
  // int8_t temp = bno.getTemp();
  // Serial.print("Current Temperature: ");
  // Serial.print(temp);
  // Serial.println(" C");
  // Serial.println("");

  for (size_t i = 0; i < nrOfButtons; i++)
  {
    button[i].init();
  }
  
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  dt = sinceLastLoop;
  sinceLastLoop = 0;
  double freq = 1000000.0/dt;
  // printf("dt: %ul \n", dt);

  if(sinceHeartBeat >= heartBeatInterval){
    sinceHeartBeat -= heartBeatInterval;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  
  quat = bno.getQuat();
  absoluteOrientation = {quat.w(), quat.x(), quat.y(), quat.z()};

  acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  rawAcc = {acc.x(), acc.y(), acc.z()};

  
  double xPos = estimatePose(rawAcc);
  
  


  // double accelToPosition = 0.5 * dtSeconds * dtSeconds;
  // xPos += euler.x * accelToPosition;
  
  // quat_conj(absoluteOrientation);
  // vec3 linearAccWorldSpace = quat_rotate(absoluteOrientation, euler);
  // // printVector(linearAccWorldSpace);

  // float accFilter = 0.001;
  // float xAccOffset = accFilter * euler.x + (1.f - accFilter) * xAccOffset;
  

  // float xAcc = euler.x * 0.1; // - xAccOffset;

  // float speedFilter = 0.2f;
  // xSpeed = speedFilter * 0.0f + (1.0f - speedFilter) * (xSpeed - xAcc);
  // xPos += xSpeed;
  

  // printf("xPos: %f\n", xPos);

  const int posWrap = 6;
  unsigned long xPosInt = ((int)(xPos*0.0007) + posWrap) % posWrap;

  unsigned long now_ul = now;
  
  int movingAngle = now_ul * 0.05f;
  movingAngle %= 360;


  const int stepSize = 6;
  int j = millis() / 10;
  for (int stripIdx = 0; stripIdx < nrOfStrips; stripIdx++)
  {
    Adafruit_NeoPixel* currentStrip = allStrips[stripIdx];
    // int stripPosition = xPosInt * 0.01;
    // stripPosition %= currentStrip->numPixels();
    currentStrip->clear();
    for (int i = 0; i < currentStrip->numPixels() / stepSize; i++)
    {
      currentStrip->setPixelColor(i * stepSize - xPosInt, colorWheel(movingAngle));
    }
    currentStrip->show();
  }

  // if(button[0].isActive()){
  //   accelIntensityRainbowAnimation(rawAcc);
  // }else{
  //   clearAllStrips();
  // }

  if(sincePrint >= printInterval){
    sincePrint -= printInterval;
    
    // printf("freq: %f\n", freq);
    debugPose();
    

  //  printf("xAccel: %f \n", rawAcc.x);
    // printQuaternion(absoluteOrientation);

    // for (int i = 0; i < nrOfButtons; i++)
    // {
    //     printf("Button %i value: %i \n", i, button[i].value);
    // }
  }
  for (int i = 0; i < nrOfButtons; i++)
  {
    button[i].clearUpdateFlag();
  }
}

// float xSpeed = 0.f;
// float xPos = 0.f;
// unsigned long xPosInt = 0;

double accelThreshold = .2;

double xAccel;
double xAccelBias;
double xAccelCorrected;
double prevXAccelCorrected;
double xSpeed;
double xSpeedBias;
double xSpeedCorrected;
double prevXSpeedCorrected;
double xPos;
bool isMoving = false;

double estimatePose(vec3 acc){
  double dtMillis =  (double)(dt) / 1000.0;
  // double dtSeconds =  dtMillis / 1000.0;

  xAccel = acc.y;

  xAccelBias = lowPass(0.1, xAccelBias, xAccel);

  // static double xAccelCorrected;
  prevXAccelCorrected = xAccelCorrected;
  xAccelCorrected = xAccel;// - xAccelBias;

  isMoving = false;
  if(abs(xAccel) > accelThreshold){
    isMoving = true;
  }

  if(isMoving){

    xSpeed = trapedzoidalIntegration(dtMillis, xSpeed, xAccelCorrected, prevXAccelCorrected);

    xSpeed = constrain(xSpeed, -200, 200);



  }else{
    xSpeed = lowPass(0.3, xSpeed, 0.f);
  }

  xSpeedBias = lowPass(0.001, xSpeedBias, xSpeed);

  // xSpeedCorrected;
  prevXSpeedCorrected = xSpeedCorrected;
  xSpeedCorrected = xSpeed; // - xSpeedBias;
  xPos = trapedzoidalIntegration(dtMillis, xPos, xSpeedCorrected, prevXSpeedCorrected);
  return xPos;
}

void debugPose(){
  printf("xAccel: %f\n", xAccel);
  printf("xAccelBias: %f\n", xAccelBias);
  printf("xAccelCorrected: %f\n", xAccelCorrected);
  printf("xSpeed: %f\n", xSpeed);
  printf("xSpeedBias: %f\n", xSpeedBias);
  printf("xSpeedCorrected: %f\n", xSpeedCorrected);
  printf("xPos: %f\n", xPos);
}

uint32_t colorWheel(int wheelPosition)
{
  // WheelPos = 255 - WheelPos;
  byte wheelPos = map(wheelPosition, 0, 256, 0, 360);
  if (wheelPos < 85)
  {
    return armStripLeft.Color(255 - wheelPos * 3, 0, wheelPos * 3);
  }
  else if (wheelPos < 170)
  {
    wheelPos -= 85;
    return armStripLeft.Color(0, wheelPos * 3, 255 - wheelPos * 3);
  }
  else
  {
    wheelPos -= 170;
    return armStripLeft.Color(wheelPos * 3, 255 - wheelPos * 3, 0);
  }
}

double trapedzoidalIntegration(double dt, double prevOut, double in, double prevIn){
  return prevOut + 0.5f * dt * (prevIn + in);
}

double lowPass(double filterQ, double prevOut, double in){
  return filterQ * in + (1.f - filterQ) * prevOut;
}

elapsedMillis sinceRainbowUpdate = 0;
void accelIntensityRainbowAnimation(vec3 accelVector){
  unsigned long dt = sinceRainbowUpdate;
  sinceRainbowUpdate = 0;
  float animationSpeed = vec_lengthSquared(accelVector);

  static unsigned int hue;
  hue += animationSpeed * dt * 0.1;
  hue %= 360;

  for (int stripIdx = 0; stripIdx < nrOfStrips; stripIdx++)
  {
    Adafruit_NeoPixel* currentStrip = allStrips[stripIdx];
    currentStrip->clear();
    for (int i = 0; i < currentStrip->numPixels(); i++)
    {
      currentStrip->setPixelColor(i, colorWheel(hue));
    }
    currentStrip->show();
  }
  
}

void clearAllStrips(){
  for (int stripIdx = 0; stripIdx < nrOfStrips; stripIdx++)
  {
    Adafruit_NeoPixel* currentStrip = allStrips[stripIdx];
    currentStrip->clear();
    currentStrip->show();
  }
}