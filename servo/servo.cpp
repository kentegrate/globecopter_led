#include <cstdio>
#include <cmath>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/portmacro.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"

#define TIMER_INTR_SEL TIMER_INTR_LEVEL  /*!< Timer level interrupt */
#define TIMER_GROUP    TIMER_GROUP_0     /*!< Test on timer group 0 */
#define TIMER_DIVIDER   16               /*!< Hardware timer clock divider */
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (1.4*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */
#define TIMER_INTERVAL0_SEC   (3.4179)   /*!< test interval for timer 0 */
#define TIMER_INTERVAL1_SEC   (5.78)   /*!< test interval for timer 1 */
#define TEST_WITHOUT_RELOAD   0   /*!< example of auto-reload mode */
#define TEST_WITH_RELOAD   1      /*!< example without auto-reload mode */

// Global variables
int channel_num = 0;

const int8_t _encRef[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
int8_t _encnum = 0;

/*
 * MC33926Driver
 */
class MC33926Driver
{
public:
  // constructors
  MC33926Driver();
  MC33926Driver( unsigned char IN1,
                 unsigned char IN2,
                 unsigned char PWM,
                 unsigned char FB,
                 unsigned char nD2,
                 unsigned char nSF);

  // Public method
  void init();
  void SetPWM(int32_t PWM);
  double getCurrentMilliamps();
  uint8_t getFault();

private:
  // Pin Configuration
  uint8_t _nD2;
  uint8_t _IN1;
  uint8_t _IN2;
  uint8_t _PWM;
  uint8_t _nSF;
  uint8_t _FB;

  // ledc Configuration
  ledc_channel_config_t _chcfg;
  ledc_timer_config_t _tmcfg;

  //
  uint32_t _maxPWM;
};

MC33926Driver::MC33926Driver()
{

}

MC33926Driver::MC33926Driver( unsigned char IN1,
                              unsigned char IN2,
                              unsigned char PWM,
                              unsigned char FB,
                              unsigned char nD2,
                              unsigned char nSF)
{
  _IN1 = IN1;
  _IN2 = IN2;
  _PWM = PWM;
  _FB = FB;
  _nD2 = nD2;
  _nSF = nSF;
}

void MC33926Driver::init()
{
  /*
   * ledc configuration
   */
  ledc_channel_config_t chcfg;
  chcfg.gpio_num = _PWM;
  chcfg.speed_mode = LEDC_HIGH_SPEED_MODE; // what's this?
  chcfg.channel = static_cast<ledc_channel_t>(channel_num++);
  chcfg.intr_type = LEDC_INTR_DISABLE; // configure interrupt, Fade interrupt enable or Fade interrupt disable
  chcfg.timer_sel = LEDC_TIMER_2;
  chcfg.duty = 0;

  ledc_timer_config_t tmcfg;
  tmcfg.speed_mode = LEDC_HIGH_SPEED_MODE; // what's this?
  tmcfg.bit_num = LEDC_TIMER_10_BIT;
  tmcfg.timer_num = LEDC_TIMER_2;
  tmcfg.freq_hz = 20000;

  _chcfg = chcfg;
  _tmcfg = tmcfg;

  ledc_channel_config(&chcfg);
  ledc_timer_config(&tmcfg);

  /*
   * gpio configuration
   */
  // nD2
  gpio_intr_disable(static_cast<gpio_num_t>(_nD2));
  gpio_set_direction(static_cast<gpio_num_t>(_nD2),GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(_nD2),1); // default HIGH

  // FB
  /*
  gpio_set_intr_disable(_FB);
  gpio_set_direction(_FB,GPIO_MODE_INPUT);
  gpio_set_pull_mode(_FB,GPIO_PULLUP_ONLY);
  */
  // gpio_get_level(_FB);

  // IN1, 2
  gpio_intr_disable(static_cast<gpio_num_t>(_IN1));
  gpio_set_direction(static_cast<gpio_num_t>(_IN1),GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(_IN1),1);
  gpio_intr_disable(static_cast<gpio_num_t>(_IN2));
  gpio_set_direction(static_cast<gpio_num_t>(_IN2),GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(_IN2),0);

  // SF
  gpio_intr_disable(static_cast<gpio_num_t>(_nSF));
  gpio_set_direction(static_cast<gpio_num_t>(_nSF),GPIO_MODE_INPUT);

  /*
   * ADC1
   */
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db); // GPIO 36
  /*
  ADC1_CHANNEL_0 = 0
  ADC1 channel 0 is GPIO36

  ADC1_CHANNEL_1
  ADC1 channel 1 is GPIO37

  ADC1_CHANNEL_2
  ADC1 channel 2 is GPIO38

  ADC1_CHANNEL_3
  ADC1 channel 3 is GPIO39

  ADC1_CHANNEL_4
  ADC1 channel 4 is GPIO32

  ADC1_CHANNEL_5
  ADC1 channel 5 is GPIO33

  ADC1_CHANNEL_6
  ADC1 channel 6 is GPIO34

  ADC1_CHANNEL_7
  ADC1 channel 7 is GPIO35

  ADC1_CHANNEL_MAX
  */
}

void MC33926Driver::SetPWM(int32_t speed)
{
  uint8_t reverse = 0;

  if (speed < 0)
  {
    speed = -speed;
    reverse = -1;
  }
  if (speed > _maxPWM)
    speed = _maxPWM;

  ledc_set_duty(LEDC_HIGH_SPEED_MODE, _chcfg.channel, speed);

  if (reverse) {
    gpio_set_level(static_cast<gpio_num_t>(_IN1),0);
    gpio_set_level(static_cast<gpio_num_t>(_IN2),1);
    } else {
        gpio_set_level(static_cast<gpio_num_t>(_IN1),1);
        gpio_set_level(static_cast<gpio_num_t>(_IN2),2);
    }
}

double MC33926Driver::getCurrentMilliamps()
{
  // 3.6 V / 4096 ADC counts / 525mV per A = 1.67410 mA per count
  return adc1_get_voltage(ADC1_CHANNEL_0) * ((3.6 / 4096)/0.000525);
}

uint8_t MC33926Driver::getFault()
{
  return !gpio_get_level(static_cast<gpio_num_t>(_nSF));
}




/*
 * Encoder
 */
 class Encoder
 {
 public:
   // variable
   long absCount;
   double theta_raw;
   double theta_processed;
   int id;
   int8_t _oldA;
   int8_t _oldB;

   // constructors
   Encoder();
   Encoder(uint8_t PhaseA, uint8_t PhaseB, double coeff_step2rad);

   // Public method
   void init();
   long getCount();
   double getTheta();

   void _calctheta();

   uint8_t _PhaseA;
   uint8_t _PhaseB;
   double _coeff_step2rad;

   // Private Method

 };

 static void IRAM_ATTR _updateEnc(void*);

 // Constructors
Encoder::Encoder()
{

}

 Encoder::Encoder(uint8_t PhaseA, uint8_t PhaseB, double coeff_step2rad)
 {
   _PhaseA = PhaseA;
   _PhaseB = PhaseB;
   _coeff_step2rad = coeff_step2rad;
   id = _encnum++;
 }

// Public method
void Encoder::init()
{
    // PhaseA
    gpio_set_intr_type(static_cast<gpio_num_t>(_PhaseA),GPIO_INTR_ANYEDGE);
    gpio_set_direction(static_cast<gpio_num_t>(_PhaseA),GPIO_MODE_INPUT);
    gpio_set_pull_mode(static_cast<gpio_num_t>(_PhaseA),GPIO_PULLUP_ONLY);

    if(id==0) {
     /*
     gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));h
     xTaskCreate(gpio_task_example, "")
     */

        gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    }

    gpio_isr_handler_add(static_cast<gpio_num_t>(_PhaseA),_updateEnc,this);

    absCount=0;
}

void Encoder::_calctheta()
{
    theta_processed = theta_raw - 2*M_PI*(int)(theta_raw/(2*M_PI));
}

static void IRAM_ATTR _updateEnc(void* arg)
{
    Encoder *enc = (Encoder*)arg;
    int8_t newA = gpio_get_level(static_cast<gpio_num_t>(enc->_PhaseA));
    int8_t newB = gpio_get_level(static_cast<gpio_num_t>(enc->_PhaseB));

    enc->absCount += _encRef[( enc->_oldA << 3 )
                       | ( enc->_oldB << 2 )
                       | ( newA << 1 )
                       | ( newB )];

    enc->theta_raw = enc->_coeff_step2rad * enc->absCount;
    enc->_calctheta();

    enc->_oldA = newA;
    enc->_oldB = newB;
}




/*
 * Servo
 */
 class DCServo
 {
 public:
     // variables
    Encoder _enc;
    MC33926Driver _motor;

     // constructor
    DCServo(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t FB, uint8_t nD2, uint8_t nSF,
             uint8_t PhaseA, uint8_t PhaseB, double coeff_step2rad);

     // Public Member Function
    void SetKCurrent(double Kp, double Ki, double Kd);
    void SetKSpeed(double Kp, double Ki, double Kd);
    void SetKPosition(double Kp, double Ki, double Kd);
    void init();
    void startControl();
    void SetTargetSpeed(double tSpeed);
    void SetTargetCurrent(double tCurrent);
    uint8_t getState();
    double getTheta();
    double getSpeed();
    double getCurrent();

    void IRAM_ATTR control();

 //private:
     // variables
    double _posKp;
    double _posKi;
    double _posKd;
    double _velKp;
    double _velKi;
    double _velKd;
    double _curKp;
    double _curKi;
    double _curKd;

    int8_t _mode;
     /*
      *-1 : Fault
      * 0 : do nothing
      * 1 : current control
      * 2 : speed control
      * 3 : speed control without current limit
      * 4 : position control
      * 5 : position control without current limit
      */
    double _speed, _targetspeed;
    double _currentMilliamp, _targetcurrentMilliamp;
    double _position, _targetposition;
    double _errorpos, _errorpos_pre, _errorpos_prepre;
    double _errorspeed, _errorspeed_pre, _errorspeed_prepre;
    double _errorcurrent, _errorcurrent_pre, _errorcurrent_prepre;
    double _dt;
    double _uPWM;

    double _maxMilliamps;

    TimerHandle_t _HandleCurrentControl;
    TimerHandle_t _HandleSpeedControl;
};

void _CurrentControlfunc(void*);
void _SpeedControlfunc(void*);

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

    _HandleCurrentControl = xTimerCreate("CurrentControl", (1 / portTICK_RATE_MS), pdTRUE, (void*)0, _CurrentControlfunc);
    _HandleSpeedControl = xTimerCreate("SpeedControl", (5 / portTICK_RATE_MS), pdTRUE, (void*)0, _SpeedControlfunc);
    if(_HandleCurrentControl !=NULL) {
        xTimerStart(_HandleCurrentControl,0);
    }
    if(_HandleSpeedControl !=NULL) {
        xTimerStart(_HandleSpeedControl,0);
    }
}

 DCServo myServo(25,26,27,34,14,12,32,33,1.0);

 void _CurrentControlfunc(void *arg)
 {
     myServo._currentMilliamp = myServo._motor.getCurrentMilliamps();

     myServo._errorcurrent_prepre = myServo._errorcurrent_pre;
     myServo._errorcurrent_pre = myServo._errorcurrent;
     myServo._errorcurrent = myServo._currentMilliamp - myServo._targetcurrentMilliamp;

     myServo._uPWM += myServo._curKp * (myServo._errorcurrent - myServo._errorcurrent_pre)
                   + myServo._curKi * myServo._errorcurrent
                   + myServo._curKd * ((myServo._errorcurrent - myServo._errorcurrent_pre) - (myServo._errorcurrent_pre - myServo._errorcurrent_prepre));

     myServo._motor.SetPWM(myServo._uPWM);
 }

 void _SpeedControlfunc(void *arg)
 {

 }

extern "C" void app_main(void)
{
    DCServo hoge(25,26,27,34,14,12,32,33,1.0);
    myServo = hoge;
    myServo.init();
    myServo.SetKCurrent(0.1,0.1,0.1);
    myServo.SetTargetCurrent(500);
    myServo.startControl();
}
