#include "utils.hpp"
void gpio_initialize_input(int gpio_num){
  gpio_intr_disable(static_cast<gpio_num_t>(gpio_num));
  gpio_set_direction(static_cast<gpio_num_t>(gpio_num),GPIO_MODE_INPUT);
}
void gpio_initialize_output(int gpio_num, int initial_state){
  gpio_pad_select_gpio(static_cast<gpio_num_t>(gpio_num));

  gpio_set_direction(static_cast<gpio_num_t>(gpio_num),GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(gpio_num), initial_state); // default HIGH
}
void gpio_initialize_interrupt(int gpio_num, gpio_int_type_t intr_type, gpio_pull_mode_t pullup_type){
  gpio_config_t io_conf;
  io_conf.intr_type = intr_type;
  io_conf.pin_bit_mask = (GPIO_SEL_33 | GPIO_SEL_32);
  io_conf.pull_up_en = static_cast<gpio_pullup_t>(0);
  io_conf.pull_down_en = static_cast<gpio_pulldown_t>(0);
  io_conf.mode = GPIO_MODE_INPUT;
  gpio_config(&io_conf);
  gpio_set_intr_type(static_cast<gpio_num_t>(32), intr_type);
  gpio_set_intr_type(static_cast<gpio_num_t>(33), intr_type);    
  //  
  /*  gpio_intr_enable(static_cast<gpio_num_t>(gpio_num));
  gpio_set_intr_type(static_cast<gpio_num_t>(gpio_num),intr_type);
  gpio_set_direction(static_cast<gpio_num_t>(gpio_num),GPIO_MODE_INPUT);
  gpio_set_pull_mode(static_cast<gpio_num_t>(gpio_num),pullup_type);*/
}
