#ifndef GLOBECOPTER_DCSERVO_HPP
#define GLOBECOPTER_DCSERVO_HPP
#include <cstdio>
#include <cmath>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "encoder.hpp"
#include "mc33926.hpp"
#include "pid.hpp"
#define TIMER_INTR_SEL TIMER_INTR_LEVEL  /*!< Timer level interrupt */
#define TIMER_GROUP    TIMER_GROUP_0     /*!< Test on timer group 0 */
#define TIMER_DIVIDER   16               /*!< Hardware timer clock divider */
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (1.4*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */
#define TIMER_INTERVAL0_SEC   (3.4179)   /*!< test interval for timer 0 */
#define TIMER_INTERVAL1_SEC   (5.78)   /*!< test interval for timer 1 */
#define TEST_WITHOUT_RELOAD   0   /*!< example of auto-reload mode */
#define TEST_WITH_RELOAD   1      /*!< example without auto-reload mode */


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
    DCServo(uint8_t IN1, uint8_t IN2,
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
   static void _CurrentControlfunc(void*);
   static void _SpeedControlfunc(void*);   
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
   double _currentMilliamp, _targetcurrentMilliamp, _currentFiltered;
    double _position, _targetposition;
    double _errorpos, _errorpos_pre, _errorpos_prepre;
    double _errorspeed, _errorspeed_pre, _errorspeed_prepre;
    double _errorcurrent, _errorcurrent_pre, _errorcurrent_prepre;
    double _dt;
    double _uPWM;
   
    double _maxMilliamps;

    TimerHandle_t _HandleCurrentControl;
    TimerHandle_t _HandleSpeedControl;

   PID speedPID;
   PID currentPID;
};
#endif //GLOBECOPTER_DCSERVO_HPP
