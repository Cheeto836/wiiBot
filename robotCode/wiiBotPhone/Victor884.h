#ifndef VICTOR884_H
#define VICTOR884_H
class Victor884
{
  private:
    //member fields
    static const int FORWARD = 180;
    static const int NEUTRAL = 90;
    static const int BACKWARD = 0;
    const int m_pin;
    bool m_reverse;
    double m_speed;

    //private functions
    void writeMotorSpeed();
    double coerceInput(double in, double low, double high);
  public:
    //constructors
    Victor884(int pin, bool reverse = false);
    
    //public functions
    void stop();

    //getters
    double getSpeed();
    double isReverse();

    //setters
    void setSpeed(double speed);
    void setReverse(bool reverse);
};
#endif
