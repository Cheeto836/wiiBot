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
int aOne = 2;
int aTwo = 3;
int aPwm = 10;

//motor b pins
int bOne = 5;
int bTwo = 6;
int bPwm = 9;

class Motor
{
  private:
  int m_pinOne;
  int m_pinTwo;
  int m_pinPwm;
  const double maxVal = 255;
  public:
  Motor(int pinOne, int pinTwo, int pinThree)
  {
    m_pinOne = pinOne;
    m_pinTwo = pinTwo;
    m_pinPwm = pinThree;
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
      analogWrite(m_pinPwm, 0);
    }
    else if (val > 0)
    {
      //forward
      digitalWrite(m_pinTwo, LOW);
      digitalWrite(m_pinOne, HIGH);
      analogWrite(m_pinPwm, maxVal * val);
    }
    else
    {
      //backward
      digitalWrite(m_pinOne, LOW);
      digitalWrite(m_pinTwo, HIGH);
      analogWrite(m_pinPwm, maxVal * (-1 *val));
    }
  }
  void stopMotor()
  {
    this->setMotor(0);
  }
};

Motor motorA(aOne, aTwo, aPwm);
Motor motorB(bOne, bTwo, bPwm);

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
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
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
        motorA.setMotor(-1);
        motorB.setMotor(-1);
      }
      else
      {
        motorA.stopMotor();
        motorB.stopMotor();
      }
      //reverse
      break;
      case Buttons::Up:
      //forward
      if(pressed)
      {
        motorA.setMotor(1);
        motorB.setMotor(1);
      }
      else
      {
        motorA.stopMotor();
        motorB.stopMotor();
      }
      break;
      case Buttons::One:
      case Buttons::Three:
      //go left
      if (pressed)
      {
        motorA.setMotor(1);
        motorB.setMotor(-1);
      }
      else
      {
        motorA.stopMotor();
        motorB.stopMotor();
      }
      break;
      case Buttons::Two:
      case Buttons::Four:
      //go right
      if (pressed)
      {
        motorA.setMotor(-1);
        motorB.setMotor(1);
      }
      else
      {
        motorA.stopMotor();
        motorB.stopMotor();
      }
      break;
    }
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
