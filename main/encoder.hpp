#ifndef GLOBECOPTER_ENCODER_HPP
#define GLOBECOPTER_ENCODER_HPP
#include <cstdio>
#include <cmath>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "freertos/timers.h"
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
   static void _EncoderUpdatefunc(void*);
   //   static void IRAM_ATTR _updateEnc(void*);
   uint8_t _PhaseA;
   uint8_t _PhaseB;
   double _coeff_step2rad;
   static int8_t _encnum;
   TimerHandle_t _HandleEncoderUpdate;
   

 };

#endif //GLOBECOPTER_ENCODER_HPP
