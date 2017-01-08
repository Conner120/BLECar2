
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
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
if (mode>0){
  Serial.println(err);
}
  while (1);
}

void setup(void)
{
  pinMode(rearFallSensorPin, INPUT);
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
if (mode>0){
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));
}
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  if (mode>0){
  Serial.println( F("OK!") );
}
  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    if (mode>0){
    Serial.println(F("Performing a factory reset: "));
    }
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);
if (mode>0){
  Serial.println("Requesting Bluefruit info:");
}
  /* Print Bluefruit information */
  ble.info();
if (mode>0){
  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();
}
  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection
  while (! ble.isConnected()) {
    delay(500);
  }
*/
if (mode>0){
  Serial.println(F("******************************"));
}
  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    if (mode>0){
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    }
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    //ble.sendCommandCheckOK("AT+BLEPOWERLEVEL=4");
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
if (mode>0){
  Serial.println(F("******************************"));
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
  checkSensors();
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
        if (mode>1){
          Serial.println(str);
          Serial.println(F("Running Command Checker..."));
        }
        if (str[0]=='r'){
          float motorValue = str.substring(1,str.length()-1).toFloat()/5;
          Serial.println(str.substring(1,str.length()-1));
          Serial.println("Right Motor run at: "+String(motorValue)+"%");
        }else if(str[0]=='l'){
          float motorValue = str.substring(1,str.length()-1).toFloat()/5;
          Serial.println(str.substring(1,str.length()-1));
          Serial.println("Right Motor run at: "+String(motorValue)+"%");
        }else{
          checkCommand(str);
        }
        str="";
        break;
    }
  }
}
delay(100);
}
void checkSensors(){
  //Serial.println("Entering Check Sensors Function");
  Serial.print("s");
  if (irSensorCheck(sideLeftSensorPin)){
    Serial.print("1");
    sideLeftSensorState = false;
    stop();
  }else{
    Serial.print("0");
    sideLeftSensorState = true;
  }
  if (irSensorCheck(frontRightSensorPin)){
    Serial.print("1");
    frontRightSensorState = false;
    stop();
  }else{
    frontRightSensorState = true;
    Serial.print("0");
  }
  if (irSensorCheck(rearLeftSensorPin)){//4
      Serial.print("1");
      rearLeftSensorState = false;
      stop();
    }else{
      rearLeftSensorState = true;
      Serial.print("0");
    }

  if (irSensorCheck(sideRightSensorPin)){//1
    Serial.print("1");
    sideRightSensorState = false;
    stop();
  }else{
    Serial.print("0");
    sideRightSensorState = true;
  }
    if (irSensorCheck(frontLeftSensorPin)){//5
      Serial.print("1");
      frontLeftSensorState = false;
      stop();
    }else{
      Serial.print("0");
      frontLeftSensorState = true;
    }
    if (irSensorCheck(rearRightSensorPin)){//2
      Serial.print("1");
      rearRightSensorState = false;
      stop();
    }else{
      Serial.print("0");
      rearRightSensorState = true;
    }
        if (irSensorCheck(frontCenterSensorPin)){
          Serial.print("1");
          frontCenterSensorState = false;
          stop();
        }else{
          Serial.print("0");
          frontCenterSensorState = true;
        }
  if (irSensorCheck(rearCenterSensorPin)){
    Serial.print("1");
    rearCenterSensorState = false;
    stop();
  }else{
    Serial.print("0");
    rearCenterSensorState = true;
  }



  if (irSensorCheck(rearFallSensorPin)){//3
    Serial.print("1");
    rearFallSensorState = false;
    stop();
  }else{
    Serial.print("0");
    rearFallSensorState = true;
  }



    if (irSensorCheck(frontFallSensorPin)){//4
        Serial.print("1");
        frontFallSensorState = false;
        stop();
      }else{
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
    Serial.println();
  //delay(2000);
}
void checkCommand(String str){
  if (str.indexOf("emergancyMode")){
    
  }
}
boolean irSensorCheck(int pin){
  //Serial.println(digitalRead(pin));
  int sensorResult = digitalRead(pin);
  //Serial.print("pin: "+String(pin)+" reads: ");
  //Serial.println(sensorResult);
  if (sensorResult==1){
    if (mode>0){
    Serial.println("NO OBJECT PRESENT on pin: "+String(pin));
    }
    return true;
  }else{
    if(mode>0){
    Serial.println("OBJECT PRESENT pin: "+String(pin));
    }
    return false;
  }
  //delay(2500);
}


// motor controller Functions

void stop() {

}
void findAvaliblePath(){
    if (frontCenterSensorState){
      return 1;
    }
    if (frontRightSensorState) {
      if (sideRightSensorState){
        return 3;
      }
      return 2;
    }
    if (frontLeftSensorState){
      if (sideLeftSensorState){
        return 5;
      }
      return 4;
    }
}
