#include "Motor.h"

//private functions
void Victor884::writeMotorSpeed()
{
}

double coerceInput(double in, double low, double high)
{
  /* Function takes in a numeric value and will coerce it to a value in the
   * range [low, high].
   * 
   * INPUTS:
   * in-double value to be coerced into range
   *
   * OUTPUTS:
   * low-double number that represents the inclusive lower bound of
   *     the coercion range.
   * high-double number that represents the inclusive upper bound of
   *      the coercion range.
   */
  if (in < low)
    return low;
  if (in > high)
    return high;
  return in;
}

//constructors
Victor884::Victor884(int pin, bool reverse)
{
}

//public functions
void Victor884::stop()
{
}

//getters
double Victor884::getSpeed()
{
  return m_speed;
}
double Victor884::isReverse()
{
  return m_reverse;
}

//setters
void Victor884::setSpeed(double speed)
{
  /* Function sets the speed and direction of the motor
   * 
   * INPUTS:
   * speed: double value representing the speed and direction of the motor.
   *        Valid range is [-1, 1]. 
   *
   */
  speed = (m_reverse) ? speed : (speed * -1);
  speed = coerceInput(speed, -1.0, 1.0);
  m_speed = speed;
  writeMotorSpeed();
}
void Victor884::setReverse(bool reverse)
{
  m_reverse = reverse;
  setSpeed(m_speed);
}
