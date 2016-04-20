/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* File has been modified by cheeto836 for use with a 'wiiBot'.
 */

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
// A small helper
void error(const __FlashStringHelper*err) 
{
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
enum class Buttons
{
  One =   1,
  Two =   2,
  Three = 3,
  Four =  4,
  Up =    5,
  Down =  6,
  Left =  7,
  Right = 8
};

Servo leftWheel;
Servo rightWheel;
const int forward = 0;
const int backward = 0;
const int neutral = 180;
const int leftWheelPin = 10;
const int rightWheelPin = 11;
const int waitPeriodMs = 500;

void setup(void)
{
  delay(waitPeriodMs);

  Serial.begin(115200);
  /* Initialise the module */
  if ( !ble.begin(false) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in Command mode & check wiring?"));
  }
  
  ble.echo(false);
  while (! ble.isConnected()) 
  {
      delay(waitPeriodMs);
  }
  ble.setMode(BLUEFRUIT_MODE_DATA);
  leftWheel.attach(leftWheelPin);
  rightWheel.attach(rightWheelPin);
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  float throttle = 0;
  float steer = 0;
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;
  // Buttons
  if (packetbuffer[1] == 'B') 
  {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Buttons buttonPressed = static_cast<Buttons>(buttnum);
    //set throttle and steer values based on button pressed
    switch (buttonPressed)
    {
      //reverse
      case Buttons::Down:
      if (pressed)
      {
        throttle = -1;
      }     
      else
      {
        throttle = 0;
      }
      break;
      case Buttons::Up:
      //forward
      if(pressed)
      {
        throttle = 1;
      }
      else
      {
        throttle = 0;
      }
      break;
      case Buttons::One:
      case Buttons::Three:
      //go left
      if (pressed)
      {
        steer = -1;
      }
      else
      {
        steer = 0;
      }
      break;
      case Buttons::Two:
      case Buttons::Four:
      //go right
      if (pressed)
      {
        steer = 1;
      }
      else
      {
        steer = 0;
      }
      break;
    }
    //generate motor values
    float left, right;
    diffSteer(throttle, steer, left, right);
  }

  // Accelerometer
  if (packetbuffer[1] == 'A') 
  {
    float x, y;
    x = parsefloat(packetbuffer+2); //long edge of phone
    y = parsefloat(packetbuffer+6); //short edge of phone
    z = parsefloat(packetbuffer+10);
    //Serial.print("Accel\t");
    //Serial.print(x); Serial.print('\t');
    //Serial.print(y); Serial.print('\t');
    //Serial.print(z); Serial.println();
  }
}

void diffSteer(float throttle, float steer, float& left, float& right)
{
  /* Function will be used to take in a throttle and steer value for driving
   * a robot. Input values (throttle and steer) will be bounded to the range
   * [1, -1].
   *
   * OPERATOIN
   * The equation will calculate a difference that will be applied to the left
   * and right drive motors. The difference will be larger for steer values
   * closer to 1 and-1. The equation used to calculate the difference is:
   *   diff = steer * turnSense (constant)
   *
   * INPUTS
   * throttle-Value equating to the forward movement of the robot. 1 is full
   *          speed forward. -1 is full speed backward.
   * steer-Value equating to the turning rate of the robot. 1 is full turn in
   *       one direction, -1 is full turn in the other direction. Direction
   *       is not garunteed as direction is dependent on things such as
   *       where the front of the robot is defined to be. 
   *
   * OUTPUTS
   * left-motor value bounded to the range [-1, 1] for robot's left motors
   * right-motor value bounded to the range [-1,1] for robot's right motors
   */
  //declare constants.
  const float TURN_SENSE = 1.0; //need to update value (see VI)
  const float MIN = -1.0;
  const float MAX = 1.0;

  //input bounding
  throttle = coerceInput(throttle, MIN, MAX);
  steer = coerceInput(steer, MIN, MAX);

  //calculate difference
  double diff = steer * TURN_SENSE;

  //apply difference to motor values
  left = throttle + diff;
  right = throttle - diff;

  //output bounding
  left = coerceInput(left, MIN, MAX);
  right = coerceInput(right, MIN, MAX);
}

float coerceInput(float in, float low, float high)
{
  /* Function takes in a numeric value and will coerce it to a value in the
   * range [low, high].
   * 
   * INPUTS:
   * in-floating point value to be coerced into range
   *
   * OUTPUTS:
   * low-floating point number that represents the inclusive lower bound of
   *     the coercion range.
   * high-floating point number that represents the inclusive upper bound of
   *      the coercion range.
   */
  if (in < low)
    return low;
  if (in > high)
    return high;
  return in;
}
