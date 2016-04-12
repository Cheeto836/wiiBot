Motor::Motor(int pinOne, int pinTwo, int pinThree)
{
  m_pinOne = pinOne;
  m_pinTwo = pinTwo;
  m_pinPwm = pinThree;
}
void Motor::setMotor(double val)
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
void Motor::stopMotor()
{
  this->setMotor(0);
}
