#ifndef GLOBECOPTER_ENCODER_HPP
#define GLOBECOPTER_ENCODER_HPP
#include <cstdio>
#include <cmath>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "freertos/timers.h"
#include <sys/time.h>
class KF{
public:
  double omega_raw;
  double theta;
  double theta_encoder;   
  double P;
  double w;
  struct timeval last_update;
  void _measurementUpdate();
  void _timeUpdate();  
  KF();
  
};

/*
 * Encoder
 */
 class Encoder
 {
 public:
   // variable
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


   static void _EncoderUpdatefunc(void*);
   //   static void IRAM_ATTR _updateEnc(void*);
   uint8_t _PhaseA;
   uint8_t _PhaseB;
   double _coeff_step2rad;
   //   static int8_t _encnum;
   TimerHandle_t _HandleEncoderUpdate;
   KF* filter;

 };

#ifndef timersub
#define timersub(a,b,result)                                                \
do {                                                                        \
  (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;                             \
  (result)->tv_usec = (a)->tv_usec - (b)->tv_usec;                          \
  if ((result)->tv_usec < 0) {                                              \
    --(result)->tv_sec;                                                     \
    (result)->tv_usec += 1000000;                                           \
  }                                                                         \
 } while (0)
#endif
#endif //GLOBECOPTER_ENCODER_HPP
