#ifndef GLOBECOPTER_MC33926_HPP
#define GLOBECOPTER_MC33926_HPP

#include <cstdio>
#include <cmath>
#include "esp_types.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"

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

#endif //GLOBECOPTER_MC33926_HPP
