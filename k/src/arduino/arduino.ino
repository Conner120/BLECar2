
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

#include <Adafruit_GPS.h>

// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
//SoftwareSerial mySerial(53,52 );
Adafruit_GPS GPS(&Serial1);
// If using hardware serial (e.g. Arduino Mega), comment out the
// above SoftwareSerial line, and enable this line instead
// (you can change the Serial number to match your wiring):
HardwareSerial mySerial = Serial1;



// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
// COMMON SETTINGS
// ----------------------------------------------------------------------------------------------
// These settings are used in both SW UART, HW UART and SPI mode
// ----------------------------------------------------------------------------------------------
#define BUFSIZE                        1024   // Size of the read buffer for incoming data
#define VERBOSE_MODE                   true  // If set to 'true' enables debug output


// SOFTWARE UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins that will be used for 'SW' serial.
// You should use this option if you are connecting the UART Friend to an UNO
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SWUART_RXD_PIN       9    // Required for software serial!
#define BLUEFRUIT_SWUART_TXD_PIN       10   // Required for software serial!
#define BLUEFRUIT_UART_CTS_PIN         11   // Required for software serial!
#define BLUEFRUIT_UART_RTS_PIN         -1   // Optional, set to -1 if unused


// HARDWARE UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the HW serial port you are using. Uncomment
// this line if you are connecting the BLE to Leonardo/Micro or Flora
// ----------------------------------------------------------------------------------------------
#ifdef Serial1    // this makes it not complain on compilation if there's no Serial1
  #define BLUEFRUIT_HWSERIAL_NAME      Serial1
#endif


// SHARED UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following sets the optional Mode pin, its recommended but not required
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_UART_MODE_PIN        12    // Set to -1 if unused


// SHARED SPI SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins to use for HW and SW SPI communication.
// SCK, MISO and MOSI should be connected to the HW SPI pins on the Uno when
// using HW SPI.  This should be used with nRF51822 based Bluefruit LE modules
// that use SPI (Bluefruit LE SPI Friend).
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SPI_CS               8
#define BLUEFRUIT_SPI_IRQ              7
#define BLUEFRUIT_SPI_RST              4    // Optional but recommended, set to -1 if unused

// SOFTWARE SPI SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins to use for SW SPI communication.
// This should be used with nRF51822 based Bluefruit LE modules that use SPI
// (Bluefruit LE SPI Friend).
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SPI_SCK              13
#define BLUEFRUIT_SPI_MISO             12
#define BLUEFRUIT_SPI_MOSI             11
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
//#include "BluefruitConfig.h"
//diffine sector avalibility
boolean forward = true;
boolean forwardRight = true;
bool debuging = true;
int RRWL = 35;
int LRWL = 37;
int RFWL = 33;
int LFWL = 36;
int fullSpeed = 256;
int halfSpeed = 128;
int threeQuarterSpeed = 192;
int quarterSpeed = 64;
float orgHeading = 0.0;
float finalHeading = 0.0;
float heading = 0.0;

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
  Serial2.begin(57600);
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    sendDebugMesg(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    //while (1);
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
    sendDebugMesg(F("Requesting Bluefruit info:"));
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
    ble.sendCommandCheckOK(F("AT+HWModeLED=" MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Robot By Conner"));
    //ble.sendCommandCheckOK("AT+BLEPOWERLEVEL=4");
  }

  // Set module to DATA mode
  sendDebugMesg( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
  if (mode > 0) {
    sendDebugMesg(F("******************************"));
  }
  Serial.println(F("Adafruit GPS library basic test!"));

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  //mySerial.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!

#ifdef __arm__
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
  useInterrupt(true);
#endif

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}

#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__


/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/

uint32_t timer = millis();

void loop(void)
{
  while(Serial2.available()){
    Serial.print(Serial2.read());
  }
  //Serial.println();
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c)
        sendDebugMesg(String(c));
  }


  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    sendDebugMesg("\nTime: " + String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + ":" + String(GPS.milliseconds));
    sendDebugMesg("Date: " + String(GPS.day) + "/" + String(GPS.month) + "/20" + String(GPS.year));
    sendDebugMesg("Fix: " + String(GPS.fix));
    sendDebugMesg(" quality: " + String(GPS.fixquality));
    if (GPS.fix) {
      sendDebugMesg("Location: " + String(GPS.latitude) + String(GPS.lat) + ", " + String(GPS.longitude) + String(GPS.lon));

      sendDebugMesg("Speed (knots): " + String(GPS.speed));
      sendDebugMesg("Angle: " + String(GPS.angle));
      sendDebugMesg("Altitude: " + String(GPS.altitude));
      sendDebugMesg("Satellites: " + String(GPS.satellites));
   }
  }
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
          float motorValue = str.substring(1, str.length() - 1).toFloat()*20.0;
          sendDebugMesg(str.substring(1, str.length() - 1));
          sendDebugMesg("Right Motor run at: " + String(motorValue) + "%");
          if ((motorValue * 330) < 0.000) {
            motorgo(0, 1, motorValue * 330);
          } else if ((motorValue * 330) > 0.000) {
            motorgo(0, 2, motorValue * 330);
          } else {
            motorgo(0, 0, 0);
          }
        } else if (str[0] == 'l') {
          float motorValue = str.substring(1, str.length() - 1).toFloat()*20.0;
          sendDebugMesg(str.substring(1, str.length() - 1));
          sendDebugMesg("Right Motor run at: " + String(motorValue) + "%");
          if ((motorValue * 330) < 0.000) {
            motorgo(1, 1, motorValue * 330);
          } else if ((motorValue * 330) > 0.000) {
            motorgo(1, 2, motorValue * 330);
          } else {
            motorgo(1, 0, 0);
          }
        } else {
          sendDebugMesg(F("command"));
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
        ///  if (mode > 1) {b
        //sendDebugMesg(str);
        //sendDebugMesg(F("Running Command Checker..."));
        //}
        if (str[0] == 'r') {
          float motorValue = str.substring(1, str.length() - 1).toFloat()*8.0;
          sendDebugMesg(str.substring(1, str.length() - 1));
          sendDebugMesg("Right Motor run at: " + String(motorValue) + "%");
         if ((motorValue/100 * 1023   ) < 0.000) {
            motorgo(0, 1, motorValue/100 * -1023);
          } else if ((motorValue/100 * 1023) > 0.000) {
            motorgo(0, 2, motorValue/100 * 1023);
          } else {
            motorgo(0, 0, 0);
          }
        } else if (str[0] == 'l') {
          float motorValue = str.substring(1, str.length() - 1).toFloat()*8.0;
          sendDebugMesg(str.substring(1, str.length() - 1));
          sendDebugMesg("Right Motor run at: " + String(motorValue) + "%");
          if ((motorValue/100 * 1023) < 0.000) {
            motorgo(1, 1, motorValue/100 *-1023);
          } else if ((motorValue/100 * 1023) > 0.000) {
            sendDebugMesg(String(motorValue/100 * 1023));
            motorgo(1, 2, motorValue/100 * 1023);
          } else {
            motorgo(1, 0, 0);
          }
        } else {
          sendDebugMesg(F("command"));
          checkCommand(str);
        }
        str = "";
        ble.flush();
        break;
      }
    }
  }
  //delay(100);
}



void checkSensors() {
  String statStr = "s";
  //Serial.println("Entering Check Sensors Function");
  if (irSensorCheck(sideLeftSensorPin)) {
    statStr += "1";
    sideLeftSensorState = false;
    stop();
  } else {
    statStr += "0";
    sideLeftSensorState = true;
  }
  if (irSensorCheck(frontRightSensorPin)) {
    statStr += "1";
    frontRightSensorState = false;
    stop();
  } else {
    frontRightSensorState = true;
    statStr += "0";
  }
  if (irSensorCheck(rearLeftSensorPin)) { //4
    statStr += "1";
    rearLeftSensorState = false;
    stop();
  } else {
    rearLeftSensorState = true;
    statStr += "0";
  }

  if (irSensorCheck(sideRightSensorPin)) { //1
    statStr += "1";
    sideRightSensorState = false;
    stop();
  } else {
    statStr += "0";
    sideRightSensorState = true;
  }
  if (irSensorCheck(frontLeftSensorPin)) { //5
    statStr += "1";
    frontLeftSensorState = false;
    stop();
  } else {
    statStr += "0";
    frontLeftSensorState = true;
  }
  if (irSensorCheck(rearRightSensorPin)) { //2
    statStr += "1";
    rearRightSensorState = false;
    stop();
  } else {
    statStr += "0";
    rearRightSensorState = true;
  }
  if (irSensorCheck(frontCenterSensorPin)) {
    statStr += "1";
    frontCenterSensorState = false;
    stop();
  } else {
    statStr += "0";
    frontCenterSensorState = true;
  }
  if (irSensorCheck(rearCenterSensorPin)) {
    statStr += "1";
    rearCenterSensorState = false;
    stop();
  } else {
    statStr += "0";
    rearCenterSensorState = true;
  }



  if (irSensorCheck(rearFallSensorPin)) { //3
    statStr += "1";
    rearFallSensorState = false;
    stop();
  } else {
    statStr += "0";
    rearFallSensorState = true;
  }



  if (irSensorCheck(frontFallSensorPin)) { //4
    statStr += "1";
    frontFallSensorState = false;
    stop();
  } else {
    statStr += "0";
    frontFallSensorState = true;
  }


  //Serial.println(statStr);


  // if (irSensorCheck(frontFallSensorPin)){
  //   statStr += "1";
  //   stop();
  // }else{
  //   statStr += "0";
  // }
  //Serial.println();
  //delay(2000);
}


void checkCommand(String str) {
  sendDebugMesg(str);
  if (str.indexOf(F("emergancyMode/")) > -1) {

  } else if (str.indexOf(F("debugoff/")) > -1) {
    mode = 0;
  }
  else if (str.indexOf(F("hLightOn/")) > -1) {
    headLightOn();
    sendDebugMesg(F("headLightOn"));
  }
  else if (str.indexOf(F("hLightOff/")) > -1) {
    sendDebugMesg(F("head lights off"));
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
  } else if (frontRightSensorState) {
    if (sideRightSensorState) {
      return 3;//turn Right
    }
    return 2;//right diagonal
  } else if (frontLeftSensorState) {
    if (sideLeftSensorState) {
      return 5;//turn left
    }
    return 4;//left diagonal
  } else if (rearLeftSensorState) {
    return 6;//turn around and then turn left
  } else if (rearRightSensorState) {
    return 7;//turn around and then turn Right
  } else if (rearCenterSensorState) {
    return 8;//turn right and if a side is becomes open turn that way
  }
  return 0;
}


void sendDebugMesg(String msg) {
  if (debuging) {
    Serial.println(msg);
    ble.println(msg);

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
  sendDebugMesg(F("\t"));
  if (!system)
  {
    sendDebugMesg(F("! "));
  }

  /* Display the individual values */
  sendDebugMesg(F("Sys:"));
  sendDebugMesg(String(system));
  sendDebugMesg(F(" G:"));
  sendDebugMesg(String(gyro));
  sendDebugMesg(F(" A:"));
  sendDebugMesg(String(accel));
  sendDebugMesg(F(" M:"));
  sendDebugMesg(String(mag));
}



void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  sendDebugMesg("------------------------------------");
  sendDebugMesg  (F("Sensor:       ")); sendDebugMesg(String(sensor.name));
  sendDebugMesg  (F("Driver Ver:   ")); sendDebugMesg(String(sensor.version));
  sendDebugMesg  (F("Unique ID:    ")); sendDebugMesg(String(sensor.sensor_id));
  sendDebugMesg  (F("Max Value:    ")); sendDebugMesg(String(sensor.max_value)); sendDebugMesg(" xxx");
  sendDebugMesg  (F("Min Value:    ")); sendDebugMesg(String(sensor.min_value)); sendDebugMesg(" xxx");
  sendDebugMesg  (F("Resolution:   ")); sendDebugMesg(String(sensor.resolution)); sendDebugMesg(" xxx");
  sendDebugMesg(F("------------------------------------"));
  sendDebugMesg(F(""));
  //delay(500);
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
  sendDebugMesg(F(""));
  sendDebugMesg(F("System Status: 0x"));
  sendDebugMesg(String(system_status));
  sendDebugMesg(F("Self Test:     0x"));
  sendDebugMesg(String(self_test_results));
  sendDebugMesg(F("System Error:  0x"));
  sendDebugMesg(String(system_error));
  sendDebugMesg(F(""));
  //delay(500);
}

void exPath() {
  int pathIndex = findAvaliblePath();
  switch (pathIndex) {
    case 1:
      forwardAuto(fullSpeed);
      break;
  }
}



void forwardAuto(int speed) {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  if (euler.x() > orgHeading) {
    float mag = euler.x() / (orgHeading);
    forwardLeft(mag);
  }

}



void getSensor() {
  sensors_event_t event;
  bno.getEvent(&event);
}



void forwardLeft(float mag) {
  if (debuging){
  sendDebugMesg("magnatude: " + String(mag));
  }
}

