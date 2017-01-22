
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int rearFallSensorPin  = 49; //rear fall sensor pin number
boolean rearFallSensorState = true;
int frontFallSensorPin = 46;
boolean frontFallSensorState = true;
int frontLeftSensorPin = 44;
boolean frontLeftSensorState = true;
int sideRightSensorPin = 45;
boolean sideRightSensorState = true;
int frontRightSensorPin = 43;
boolean frontRightSensorState = true;
int sideLeftSensorPin = 41;
boolean sideLeftSensorState = true;
int frontCenterSensorPin = 42;
boolean frontCenterSensorState = true;
int rearLeftSensorPin = 40;
boolean rearLeftSensorState = true;
int rearCenterSensorPin = 38;
boolean rearCenterSensorState = true;
int rearRightSensorPin = 39;
boolean rearRightSensorState = true;
#include "BluefruitConfig.h"
//diffine sector avalibility
boolean forward = true;
boolean forwardRight = true;
bool debuging = true;
int RRWL = 35;
int LRWL = 37;
int RFWL = 33;
int LFWL = 36;
int fullSpeed = 512;
int halfSpeed = 256;
int threeQuarterSpeed = 384;
int quarterSpeed = 128;
float origonalHeading = 0.0;
float finalHeading = 0.0;
int RRRL = 33;
int LRRL = 34;
int seirnRed[5] = {22, 24, 26 };
int seirnBlue[5] = {23, 25, 27, 29, 31};
int horn = 23;
/*  MonsterMoto Shield Example Sketch
  date: 5/24/11
  code by: Jim Lindblom
  hardware by: Nate Bernstein
  SparkFun Electronics

  License: CC-SA 3.0, feel free to use this code however you'd like.
  Please improve upon it! Let me know how you've made it better.

  This is really simple example code to get you some basic
  functionality with the MonsterMoto Shield. The MonsterMote uses
  two VNH2SP30 high-current full-bridge motor drivers.

  Use the motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
  function to get motors going in either CW, CCW, BRAKEVCC, or
  BRAKEGND. Use motorOff(int motor) to turn a specific motor off.

  The motor variable in each function should be either a 0 or a 1.
  pwm in the motorGo function should be a value between 0 and 255.
*/
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

/*  VNH2SP30 pin definitions
  xxx[0] controls '1' outputs
  xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)

int statpin = 13;


#define FACTORYRESET_ENABLE         0
//#define BUFSIZE                        48   // Size of the read buffer for incoming data
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

SoftwareSerial bluefruitSS = SoftwareSerial(51, 50);

Adafruit_BluefruitLE_UART ble(bluefruitSS, 53,
                              52, 48);


int mode = 0;
String str = "";
// A small helper
void error(const __FlashStringHelper*err) {
  if (mode > 0) {
    sendDebugMesg(err);
  }
  while (1);
}

void setup(void)
{
  pinMode(RRWL, OUTPUT);
  pinMode(RFWL, OUTPUT);
  pinMode(LRWL, OUTPUT);
  pinMode(LFWL, OUTPUT);
  pinMode(RRRL, OUTPUT);
  pinMode(LRRL, OUTPUT);
  pinMode(statpin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  pinMode(rearFallSensorPin, INPUT);
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  /* Initialise the sensor */
 if(!bno.begin())
 {
   /* There was a problem detecting the BNO055 ... check your connections */
   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
   while(1);
 }

 /* Display some basic information on this sensor */
 displaySensorDetails();

 /* Optional: Display current status */
 displaySensorStatus();

 bno.setExtCrystalUse(true);
  if (mode > 0) {
    sendDebugMesg(F("Adafruit Bluefruit Command <-> Data Mode Example"));
    sendDebugMesg(F("------------------------------------------------"));

    /* Initialise the module */
    sendDebugMesg(F("Initialising the Bluefruit LE module: "));
  }
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  if (mode > 0) {
    sendDebugMesg( F("OK!") );
  }
  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    if (mode > 0) {
      sendDebugMesg(F("Performing a factory reset: "));
    }
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  if (mode > 1) {
    sendDebugMesg("Requesting Bluefruit info:");
  }
  /* Print Bluefruit information */
  ble.info();
  if (mode > 0) {
    sendDebugMesg(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
    sendDebugMesg(F("Then Enter characters to send to Bluefruit"));
    //sendDebugMesg();
  }
  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection
    while (! c) {
    delay(500);
    }
  */
  if (mode > 0) {
    sendDebugMesg(F("******************************"));
  }
  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    if (mode > 0) {
      sendDebugMesg(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    }
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    ble.sendCommandCheckOK("AT+GAPDEVNAME=Robot By Conner");
    //ble.sendCommandCheckOK("AT+BLEPOWERLEVEL=4");
  }

  // Set module to DATA mode
  sendDebugMesg( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
  if (mode > 0) {
    sendDebugMesg(F("******************************"));
  }
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  //delay(2500);
  //Serial.println("Entering Check Sensors Function");
  if (millis() % 100 == 0) {
    checkSensors();
  }
  if (Serial.available()) {

    // Echo receive data
    while ( Serial.available() )
    {
      byte c = Serial.read();
      str = str + char(c);
      // Hex output too, helps w/debugging!
      //    Serial.print(" [0x");
      //    if (c <= 0xF) Serial.print(F("0"));
      //    Serial.print(c, HEX);
      //    Serial.print("] ");
      if (c == '/') {
      ///  if (mode > 1) {
          sendDebugMesg(str);
          sendDebugMesg(F("Running Command Checker..."));
        //}
        if (str[0] == 'r') {
          float motorValue = str.substring(1, str.length() - 1).toFloat() / 5;
          sendDebugMesg(str.substring(1, str.length() - 1));
          sendDebugMesg("Right Motor run at: " + String(motorValue) + "%");
          if ((motorValue * 1023) < 0.000) {
            motorgo(0, 1, motorValue * -1023);
          } else if ((motorValue * 1023) > 0.000) {
            motorgo(0, 2, motorValue * 1023);
          } else {
            motorgo(0, 1, 0);
          }
        } else if (str[0] == 'l') {
          float motorValue = str.substring(1, str.length() - 1).toFloat() / 5;
          sendDebugMesg(str.substring(1, str.length() - 1));
          sendDebugMesg("Right Motor run at: " + String(motorValue) + "%");
          if ((motorValue * 1023) < 0.000) {
            motorgo(1, 1, motorValue * -1023);
          } else if ((motorValue * 1023) > 0.000) {
            motorgo(1, 2, motorValue * 1023);
          } else {
            motorgo(1, 1, 0);
          }
        } else {
          sendDebugMesg("command");
          checkCommand(str);
        }
        str = "";
        break;
      }
    }
  }

  if (ble.available()) {

    // Echo receive data
    while ( ble.available() )
    {
      byte c = ble.read();
      str = str + char(c);
      // Hex output too, helps w/debugging!
      //    Serial.print(" [0x");
      //    if (c <= 0xF) Serial.print(F("0"));
      //    Serial.print(c, HEX);
      //    Serial.print("] ");
      if (c == '/') {
      ///  if (mode > 1) {
          sendDebugMesg(str);
          sendDebugMesg(F("Running Command Checker..."));
        //}
        if (str[0] == 'r') {
          float motorValue = str.substring(1, str.length() - 1).toFloat() / 5;
          sendDebugMesg(str.substring(1, str.length() - 1));
          sendDebugMesg("Right Motor run at: " + String(motorValue) + "%");
          if ((motorValue * 1023) < 0.000) {
            motorgo(0, 1, motorValue * -1023);
          } else if ((motorValue * 1023) > 0.000) {
            motorgo(0, 2, motorValue * 1023);
          } else {
            motorgo(0, 1, 0);
          }
        } else if (str[0] == 'l') {
          float motorValue = str.substring(1, str.length() - 1).toFloat() / 5;
          sendDebugMesg(str.substring(1, str.length() - 1));
          sendDebugMesg("Right Motor run at: " + String(motorValue) + "%");
          if ((motorValue * 1023) < 0.000) {
            motorgo(1, 1, motorValue * -1023);
          } else if ((motorValue * 1023) > 0.000) {
            motorgo(1, 2, motorValue * 1023);
          } else {
            motorgo(1, 1, 0);
          }
        } else {
          sendDebugMesg("command");
          checkCommand(str);
        }
        str = "";
        break;
      }
    }
  }
  //delay(100);
}
void checkSensors() {
  //Serial.println("Entering Check Sensors Function");
  Serial.print("s");
  if (irSensorCheck(sideLeftSensorPin)) {
    Serial.print("1");
    sideLeftSensorState = false;
    stop();
  } else {
    Serial.print("0");
    sideLeftSensorState = true;
  }
  if (irSensorCheck(frontRightSensorPin)) {
    Serial.print("1");
    frontRightSensorState = false;
    stop();
  } else {
    frontRightSensorState = true;
    Serial.print("0");
  }
  if (irSensorCheck(rearLeftSensorPin)) { //4
    Serial.print("1");
    rearLeftSensorState = false;
    stop();
  } else {
    rearLeftSensorState = true;
    Serial.print("0");
  }

  if (irSensorCheck(sideRightSensorPin)) { //1
    Serial.print("1");
    sideRightSensorState = false;
    stop();
  } else {
    Serial.print("0");
    sideRightSensorState = true;
  }
  if (irSensorCheck(frontLeftSensorPin)) { //5
    Serial.print("1");
    frontLeftSensorState = false;
    stop();
  } else {
    Serial.print("0");
    frontLeftSensorState = true;
  }
  if (irSensorCheck(rearRightSensorPin)) { //2
    Serial.print("1");
    rearRightSensorState = false;
    stop();
  } else {
    Serial.print("0");
    rearRightSensorState = true;
  }
  if (irSensorCheck(frontCenterSensorPin)) {
    Serial.print("1");
    frontCenterSensorState = false;
    stop();
  } else {
    Serial.print("0");
    frontCenterSensorState = true;
  }
  if (irSensorCheck(rearCenterSensorPin)) {
    Serial.print("1");
    rearCenterSensorState = false;
    stop();
  } else {
    Serial.print("0");
    rearCenterSensorState = true;
  }



  if (irSensorCheck(rearFallSensorPin)) { //3
    Serial.print("1");
    rearFallSensorState = false;
    stop();
  } else {
    Serial.print("0");
    rearFallSensorState = true;
  }



  if (irSensorCheck(frontFallSensorPin)) { //4
    Serial.print("1");
    frontFallSensorState = false;
    stop();
  } else {
    Serial.print("0");
    frontFallSensorState = true;
  }



  //

  // if (irSensorCheck(frontFallSensorPin)){
  //   Serial.print("1");
  //   stop();
  // }else{
  //   Serial.print("0");
  // }
  //Serial.println();
  //delay(2000);
}
void checkCommand(String str) {
  sendDebugMesg(str);
  if (str.indexOf("emergancyMode/") > -1) {

  } else if (str.indexOf("debugoff/") > -1) {
    mode = 0;
  }
  else if (str.indexOf("hLightOn/") > -1) {
    headLightOn();
    sendDebugMesg("headLightOn");
  }
  else if (str.indexOf("hLightOff/") > -1) {
    sendDebugMesg("head lights off");
    headLightOff();
  }
}
void headLightOn() {
  digitalWrite(LRWL, true);
  digitalWrite(RRWL, true);
  digitalWrite(LFWL, true);
  digitalWrite(RFWL, true);
}
void headLightOff() {
  digitalWrite(LRWL, false);
  digitalWrite(RRWL, false);
  digitalWrite(LFWL, false);
  digitalWrite(RFWL, false);
}
boolean irSensorCheck(int pin) {
  //Serial.println(digitalRead(pin));
  int sensorResult = digitalRead(pin);
  //Serial.print("pin: "+String(pin)+" reads: ");
  //Serial.println(sensorResult);
  if (sensorResult == 1) {
    if (mode > 0) {
      sendDebugMesg("NO OBJECT PRESENT on pin: " + String(pin));
    }
    return true;
  } else {
    if (mode > 0) {
      sendDebugMesg("OBJECT PRESENT pin: " + String(pin));
    }
    return false;
  }
  //delay(2500);
}


// motor controller Functions

void stop() {

}
int findAvaliblePath() {
  if (frontCenterSensorState) {
    return 1;//forward
  }else if (frontRightSensorState) {
    if (sideRightSensorState) {
      return 3;//turn Right
    }
    return 2;//right diagonal
  }else if (frontLeftSensorState) {
    if (sideLeftSensorState) {
      return 5;//turn left
    }
    return 4;//left diagonal
  } else if (rearLeftSensorState){
    return 6;//turn around and then turn left
  } else if (rearRightSensorState){
    return 7;//turn around and then turn Right
  }else if (rearCenterSensorState){
    return 8;//turn right and if a side is becomes open turn that way
  }
return 0;
}


void sendDebugMesg(String msg){
  if (debuging){
    Serial.println(msg);
  }
}
/* motorGo() will set a motor going in a specific direction
  the motor will continue going in that direction, at that speed
  until told to do otherwise.

  motor: this should be either 0 or 1, will selet which of the two
  motors to be controlled

  direct: Should be between 0 and 3, with the following result
  0: Brake to VCC
  1: Clockwise
  2: CounterClockwise
  3: Brake to GND

  pwm: should be a value between ? and 1023, higher the number, the faster
  it'll go
*/
void motorgo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <= 4)
    {
      // Set inA[motor]
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}
