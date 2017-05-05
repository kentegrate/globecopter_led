#ifndef GLOBECOPTER_UTILS_HPP
#define GLOBECOPTER_UTILS_HPP
#include <cstdio>
#include <cmath>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
void gpio_initialize_input(int gpio_num);
void gpio_initialize_output(int gpio_num, int initial_state);
void gpio_initialize_interrupt(int gpio_num, gpio_int_type_t intr_type, gpio_pull_mode_t pullup_type);

#endif
