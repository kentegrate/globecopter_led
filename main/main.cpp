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
#include "mc33926.hpp"
#include "encoder.hpp"
#include "dcservo.hpp"


// Global variables
#define MC33926_IN1 25
#define MC33926_IN2 26
#define MC33926_PWM 27
#define MC33926_FB 36
#define MC33926_nD2 14
#define MC33926_nSF 12
#define ENCODER_PHASE_A 32
#define ENCODER_PHASE_B 33
#define COEFF_STEP2RAD 1.0


extern "C" void app_main(void)
{
  DCServo myServo(MC33926_IN1, MC33926_IN2,
		  MC33926_PWM, MC33926_FB,
		  MC33926_nD2, MC33926_nSF,
		  ENCODER_PHASE_A, ENCODER_PHASE_B,
		  COEFF_STEP2RAD);
  myServo.init();
  myServo.SetKCurrent(0.1,0.1,0.1);
  myServo.SetTargetCurrent(500);
  myServo.startControl();
}
