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
//#include "Motor.h"

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
void error(const __FlashStringHelper*err) {
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
  One = 1,
  Two = 2,
  Three = 3,
  Four = 4,
  Up = 5,
  Down = 6,
  Left = 7,
  Right = 8
};

//motor a pins
/*int aOne = 5;
int aTwo = 9;
//motor b pins
int bOne = 10;
int bTwo = 22;

class Motor
{
  private:
  int m_pinOne;
  int m_pinTwo;
  const double maxVal = 255;
  public:
  Motor(int pinOne, int pinTwo)
  {
    m_pinOne = pinOne;
    m_pinTwo = pinTwo;
    pinMode(m_pinOne, OUTPUT);
    pinMode(m_pinTwo, OUTPUT);
  }
  void setMotor(double val)
  {
    int motorVal = 0;
    //ensure value is in range
    if (val > 1)
      val = 1;
    if(val < -1)
      val = -1;

    //write to pins correct direction
    if (val == 0)
    {
      digitalWrite(m_pinOne, LOW);
      digitalWrite(m_pinTwo, LOW);
    }
    else if (val > 0)
    {
      //forward
      digitalWrite(m_pinTwo, LOW);
      digitalWrite(m_pinOne, HIGH);
    }
    else
    {
      //backward
      digitalWrite(m_pinOne, LOW);
      digitalWrite(m_pinTwo, HIGH);
    }
  }
  void stopMotor()
  {
    this->setMotor(0);
  }
};*/

Servo leftWheel;
Servo rightWheel;
const int forward = 180;
const int backward = 0;
const int neutral = 90;
void setup(void)
{
  pinMode(13, OUTPUT);
  delay(500);

  Serial.begin(115200);
  /* Initialise the module */
  if ( !ble.begin(false) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  
  ble.echo(false);
  while (! ble.isConnected()) 
  {
      delay(500);
  }
  digitalWrite(13, HIGH);
  ble.setMode(BLUEFRUIT_MODE_DATA);
  leftWheel.attach(10);
  rightWheel.attach(11);
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  float throttle = 0, steer = 0;
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  /*if (!ble.isConnected())
  {
    rightWheel.write(neutral);
    leftWheel.write(neutral);
  }*/
  if (len == 0) return;
  // Buttons
  if (packetbuffer[1] == 'B') 
  {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Buttons buttonPressed = static_cast<Buttons>(buttnum);
    switch (buttonPressed)
    {
      case Buttons::Down:
      if (pressed)
      {
        //throttle = -1;
        rightWheel.write(180);
        leftWheel.write(180);
      }     
      else
      {
        //throttle = 0;
        rightWheel.write(90);
        leftWheel.write(90);
      }
      //reverse
      break;
      case Buttons::Up:
      //forward
      if(pressed)
      {
        //throttle = 1;
        rightWheel.write(0);
        leftWheel.write(0);
      }
      else
      {
        //throttle = 0;
        rightWheel.write(90);
        leftWheel.write(90);
      }
      break;
      case Buttons::One:
      case Buttons::Three:
      //go left
      if (pressed)
      {
        //steer = -1;
        rightWheel.write(0);
       leftWheel.write(180);
      }
      else
      {
        //steer = 0;
        leftWheel.write(90);
        rightWheel.write(90);
      }
      break;
      case Buttons::Two:
      case Buttons::Four:
      //go right
      if (pressed)
      {
        //steer = 1;
        leftWheel.write(0);
        rightWheel.write(180);
      }
      else
      {
        //steer = 0;
        leftWheel.write(90);
        rightWheel.write(90);
      }
      break;
    }
    /*if (throttle == 1 && steer == 1)
    {
      //forward and right
      leftWheel.write(forward);
      rightWheel.write(forward - (neutral / 2));
    }
    else if (throttle == 1 && steer == -1)
    {
      //forward and left
      leftWheel.write(forward - (neutral / 2));
      rightWheel.write(forward);
    }
    else if (throttle == 1 && steer == 0)
    {
      //forward and straight
      leftWheel.write(forward);
      rightWheel.write(forward);
    }
    else if (throttle == -1 && steer == 1)
    {
      //backward and right
      leftWheel.write(backward);
      rightWheel.write(backward + (neutral / 2));
    }
    else if (throttle == -1 && steer == -1)
    {
      //backward and left
      leftWheel.write(backward + (neutral / 2));
      rightWheel.write(backward);
    }
    else if (throttle == -1 && steer == 0)
    {
      //backward and straight
      leftWheel.write(backward);
      rightWheel.write(backward);
    }
    else if (throttle == 0 && steer == 1)
    {
      //right only
      leftWheel.write(forward);
      rightWheel.write(backward);
    }
    else if (throttle == 0 && steer == -1)
    {
      //left only
      leftWheel.write(backward);
      rightWheel.write(forward);
    }
    else
    {
      //stop
      leftWheel.write(neutral);
      rightWheel.write(neutral);
    }*/
  }

  // Accelerometer
  if (packetbuffer[1] == 'A') 
  {
    float x, y;
    x = parsefloat(packetbuffer+2); //long edge of phone
    y = parsefloat(packetbuffer+6); //short edge of phone
    //z = parsefloat(packetbuffer+10);

    if (x > 2)
    {
      digitalWrite(13, HIGH);
    }
    else
    {
      digitalWrite(13, LOW);
    }
    //Serial.print("Accel\t");
    //Serial.print(x); Serial.print('\t');
    //Serial.print(y); Serial.print('\t');
    //Serial.print(z); Serial.println();
  }
}

void diffSteer(double steer, double throttle, double& left, double& right)
{
  //input coercion
  if (steer < -1)
    steer = -1;
  if (steer > 1)
    steer = 1;
  if (throttle < -1)
    throttle = -1;
  if (throttle > 1)
    throttle = 1;

  //check for point spin
  if (throttle == 0 && throttle != 0)
  {
  }
}
