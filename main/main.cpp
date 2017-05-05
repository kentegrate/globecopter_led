#include <cstdio>
#include <cmath>
#include "esp_types.h"
#include "esp_log.h"

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
#include "utils.hpp"

// Global variables
 #define MC33926_IN1 17
#define MC33926_IN2 16
#define ENCODER_PHASE_A 32
#define ENCODER_PHASE_B 33
#define COEFF_STEP2RAD 1.0

static const char *tag = "myservo";
extern "C" void app_main(void)
{
  DCServo myServo(MC33926_IN1, MC33926_IN2,
		  ENCODER_PHASE_A, ENCODER_PHASE_B,
		  COEFF_STEP2RAD);
  myServo.init();
  myServo.SetKCurrent(0.5,0.08,0);
  myServo.SetTargetCurrent(30);
  myServo.startControl();
  while(1){
    ESP_LOGI(tag, "ok");
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}
