#include "dcservo.hpp"
#include "esp_log.h"

DCServo::DCServo(uint8_t IN1, uint8_t IN2,
                 uint8_t PhaseA, uint8_t PhaseB, double coeff_step2rad)
{
  _motor = MC33926Driver(IN1, IN2);
  _enc = Encoder(PhaseA, PhaseB, coeff_step2rad);

  _targetspeed = 0;
  _currentMilliamp = 0;
  _targetcurrentMilliamp = 0;
  _position = 0;
  _targetposition = 0;
  _dt = 0;
  _uPWM = 0;
  _maxMilliamps = 0;
  _currentFiltered = 0;
}

void DCServo::SetKCurrent(double Kp, double Ki, double Kd)
{
  currentPID.setGain(Kp, Ki, Kd);
}

void DCServo::SetKSpeed(double Kp, double Ki, double Kd)
{
  speedPID.setGain(Kp, Ki, Kd);
}

void DCServo::SetKPosition(double Kp, double Ki, double Kd)
{

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
  gpio_initialize_output(INDICATOR_GPIO, 1);
  is_protected = false;
  
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

  _HandleCurrentControl = xTimerCreate("CurrentControl", (20.0/ portTICK_RATE_MS), pdTRUE, (void*)this, _CurrentControlfunc);
  _HandleSpeedControl = xTimerCreate("SpeedControl", (20.0 / portTICK_RATE_MS), pdTRUE, (void*)this, _SpeedControlfunc);
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
  myServo->_motor.enable = (myServo->_enc.pwm_decoder_0.width > 1500);
    
  double r = 0.9;
  static const char *tag = "DCservo";
  myServo->_currentMilliamp = myServo->_motor.getCurrentMilliamps();
  myServo->_currentFiltered = myServo->_currentFiltered * r + (1-r) * myServo->_currentMilliamp;
  if(myServo->_currentFiltered > 1200){
    myServo->protectMotor();
  }
  //  ESP_LOGI(tag, "current filtered %f", myServo->_currentFiltered);
  if(!myServo->motorProtected()){
    myServo->_uPWM = myServo->currentPID.update(myServo->_targetcurrentMilliamp, myServo->_currentMilliamp);
    myServo->_motor.SetPWM(myServo->_uPWM);
  }
  else{
    myServo->_motor.SetPWM(0);
  }
  
  //  ESP_LOGI(tag, "current I : %f, target I: %f pwm: %f" , myServo->_currentMilliamp, myServo->_targetcurrentMilliamp, myServo->_uPWM);
}

void DCServo::_SpeedControlfunc(TimerHandle_t xTimer)
  {

    DCServo* myServo = (DCServo*)pvTimerGetTimerID(xTimer);
    myServo->_speed = myServo->_enc.filter->w;    
    myServo->_targetcurrentMilliamp = myServo->speedPID.update(myServo->_targetspeed,
							       myServo->_speed);
    static const char *tag = "DCservo";
    ESP_LOGI(tag, "speed target : %f speed current: %f", myServo->_targetspeed,
	     myServo->_speed);
  }


bool DCServo::motorProtected(){

  if(is_protected){
    struct timeval current_time, time_delta;
    gettimeofday(&current_time, NULL);
    timersub(&current_time, &overcurrent_time, &time_delta);
    
    if(time_delta.tv_sec>5){
      is_protected = false;
    }
  }
  if(is_protected){
    gpio_set_level(static_cast<gpio_num_t>(INDICATOR_GPIO), 0);
  }
  else{
    gpio_set_level(static_cast<gpio_num_t>(INDICATOR_GPIO), 1);
  }
  
  return is_protected;
}

void DCServo::protectMotor(){
  gettimeofday(&overcurrent_time, NULL);
  is_protected = true;
}
