#ifndef MOTOR_H
#define MOTOR_H
class Motor
{
  private:
  int m_pinOne;
  int m_pinTwo;
  int m_pinPwm;
  const double maxVal = 255;
  public:
  Motor(int pinOne, int pinTwo, int pinThree);
  void setMotor(double val);
  void stopMotor();
};
#endif
