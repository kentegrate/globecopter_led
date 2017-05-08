#ifndef GLOBECOPTER_PWMDECODER_HPP
#define GLOBECOPTER_PWMDECODER_HPP
#include <sys/time.h>
class PWMDecoder{
public:
  static PWMDecoder* pwm0;
  static PWMDecoder* pwm1;
  static PWMDecoder* getPWMDecoder(int pin);
  static int id;
  int pin;
  void init(int _pin);
  int width;
  struct timeval posedge_time;
};

#endif //GLOBECOPTER_PWMDECODER_HPP
