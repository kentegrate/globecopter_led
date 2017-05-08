#ifndef GLOBECOPTER_UTILS_HPP
#define GLOBECOPTER_UTILS_HPP
#include <cstdio>
#include <cmath>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include <sys/time.h>
void gpio_initialize_input(int gpio_num);
void gpio_initialize_output(int gpio_num, int initial_state);
void gpio_initialize_interrupt(int gpio_num, gpio_int_type_t intr_type, gpio_pull_mode_t pullup_type);

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

#endif
