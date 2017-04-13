#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
struct ring_led_data{
  ring_led_data(int led_n, int steps_per_revolution);

  void set_data(const uint32_t* raw_data);

  const uint32_t* get_led_pattern(int idx);

  const uint32_t *data;
  int led_n;
  int steps_per_revolution;
};


