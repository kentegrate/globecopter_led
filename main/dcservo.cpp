#include "dcservo.hpp"
DCServo::DCServo(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t FB, uint8_t nD2, uint8_t nSF,
                 uint8_t PhaseA, uint8_t PhaseB, double coeff_step2rad)
{
  _motor = MC33926Driver(IN1, IN2, PWM, FB, nD2, nSF);
  _enc = Encoder(PhaseA, PhaseB, coeff_step2rad);

  _posKp = 0;
  _posKi = 0;
  _posKd = 0;
  _velKp = 0;
  _velKi = 0;
  _velKd = 0;
  _curKp = 0;
  _curKi = 0;
  _curKd = 0;
  _speed = 0;
  _targetspeed = 0;
  _currentMilliamp = 0;
  _targetcurrentMilliamp = 0;
  _position = 0;
  _targetposition = 0;
  _errorpos = 0;
  _errorpos_pre = 0;
  _errorpos_prepre = 0;
  _errorspeed = 0;
  _errorspeed_pre = 0;
  _errorspeed_prepre = 0;
  _errorcurrent = 0;
  _errorcurrent_pre = 0;
  _errorcurrent_prepre = 0;
  _dt = 0;
  _uPWM = 0;
  _maxMilliamps = 0;
}

void DCServo::SetKCurrent(double Kp, double Ki, double Kd)
{
  _curKp = Kp;
  _curKi = Ki;
  _curKd = Kd;
}

void DCServo::SetKSpeed(double Kp, double Ki, double Kd)
{
  _velKp = Kp;
  _velKi = Ki;
  _velKd = Kd;
}

void DCServo::SetKPosition(double Kp, double Ki, double Kd)
{
  _posKp = Kp;
  _posKi = Ki;
  _posKd = Kd;
}

void DCServo::SetTargetSpeed(double tSpeed)
{
  _targetspeed = tSpeed;
}

void DCServo::SetTargetCurrent(double tCurrent)
{
  _targetcurrentMilliamp = tCurrent;
}

void DCServo::init()
{
  _motor.init();
  _enc.init();
}

void DCServo::startControl()
{
  /*
    timer_config_t tcfg;
    tcfg.alarm_en = 1;
    tcfg.counter_en = 1;
    tcfg.intr_type = TIMER_INTR_SEL;
    tcfg.counter_dir = TIMER_COUNT_UP;
    tcfg.auto_reload = 1;
    tcfg.divider = TIMER_DIVIDER;

    timer_init(TIMER_GROUP_0, TIMER_0, &tcfg);
    timer_pause(TIMER_GROUP_0, TIMER_0);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, );

    tiemr_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, )
  */

  _HandleCurrentControl = xTimerCreate("CurrentControl", (1 / portTICK_RATE_MS), pdTRUE, (void*)this, _CurrentControlfunc);
  _HandleSpeedControl = xTimerCreate("SpeedControl", (5 / portTICK_RATE_MS), pdTRUE, (void*)this, _SpeedControlfunc);
  if(_HandleCurrentControl !=NULL) {
    xTimerStart(_HandleCurrentControl,0);
  }
  if(_HandleSpeedControl !=NULL) {
    xTimerStart(_HandleSpeedControl,0);
  }
}

void DCServo::_CurrentControlfunc(TimerHandle_t xTimer)
{
  DCServo* myServo = (DCServo*)pvTimerGetTimerID(xTimer);
  myServo->_currentMilliamp = myServo->_motor.getCurrentMilliamps();

  myServo->_errorcurrent_prepre = myServo->_errorcurrent_pre;
  myServo->_errorcurrent_pre = myServo->_errorcurrent;
  myServo->_errorcurrent = myServo->_currentMilliamp - myServo->_targetcurrentMilliamp;

  myServo->_uPWM += myServo->_curKp * (myServo->_errorcurrent - myServo->_errorcurrent_pre)
    + myServo->_curKi * myServo->_errorcurrent
    + myServo->_curKd * ((myServo->_errorcurrent - myServo->_errorcurrent_pre) - (myServo->_errorcurrent_pre - myServo->_errorcurrent_prepre));

  myServo->_motor.SetPWM(myServo->_uPWM);
}

void DCServo::_SpeedControlfunc(TimerHandle_t xTimer)
  {
    DCServo* myServo = (DCServo*)pvTimerGetTimerID(xTimer);
  }
