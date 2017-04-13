
#include "ring_led_data.hpp"


ring_led_data::ring_led_data(int led_n, int steps_per_revolution){//number of leds on one side
  this->led_n = led_n;
  this->steps_per_revolution = steps_per_revolution;
}
void ring_led_data::set_data(const uint32_t* raw_data){
  data = raw_data;
}

const uint32_t* ring_led_data::get_led_pattern(int idx){
  return &data[led_n*idx];
}


