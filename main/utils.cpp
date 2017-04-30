#include "utils.hpp"

void gpio_initiailze_input(int gpio_num){
  gpio_intr_disable(static_cast<gpio_num_t>(gpio_num));
  gpio_set_direction(static_cast<gpio_num_t>(gpio_num),GPIO_MODE_INPUT);
}

void gpio_initiailze_output(int gpio_num, int initial_state){
  gpio_intr_disable(static_cast<gpio_num_t>(gpio_num));
  gpio_set_direction(static_cast<gpio_num_t>(gpio_num),GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(gpio_num), initial_state); // default HIGH

}

void gpio_initialize_interrupt(int gpio_num, gpio_int_type_t intr_type, gpio_pull_mode_t pullup_type){
  gpio_set_intr_type(static_cast<gpio_num_t>(gpio_num),intr_type);
  gpio_set_direction(static_cast<gpio_num_t>(gpio_num),GPIO_MODE_INPUT);
  gpio_set_pull_mode(static_cast<gpio_num_t>(gpio_num),pullup_type);
}
