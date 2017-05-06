#include "mc33926.hpp"
#include "utils.hpp"
#include "esp_log.h"
MC33926Driver::MC33926Driver()
{
}

MC33926Driver::MC33926Driver( unsigned char IN1,
                              unsigned char IN2)

{
  _IN1 = IN1;
  _IN2 = IN2;
  _maxPWM = 2047;
}

void MC33926Driver::init()
{
  /*
   * ledc configuration
   */
  ledc_channel_config_t chcfg_1;
  chcfg_1.gpio_num = _IN1;
  chcfg_1.speed_mode = LEDC_HIGH_SPEED_MODE; // what's this?
  chcfg_1.channel = static_cast<ledc_channel_t>(0);
  chcfg_1.intr_type = LEDC_INTR_FADE_END;
  chcfg_1.timer_sel = LEDC_TIMER_0;
  chcfg_1.duty = 0;

  ledc_channel_config_t chcfg_2;
  chcfg_2.gpio_num = _IN2;
  chcfg_2.speed_mode = LEDC_HIGH_SPEED_MODE; // what's this?
  chcfg_2.channel = static_cast<ledc_channel_t>(1);
  chcfg_2.intr_type = LEDC_INTR_FADE_END;
  chcfg_2.timer_sel = LEDC_TIMER_0;
  chcfg_2.duty = 0;
  

  ledc_timer_config_t tmcfg;
  tmcfg.speed_mode = LEDC_HIGH_SPEED_MODE; // what's this?
  tmcfg.bit_num = LEDC_TIMER_11_BIT;
  tmcfg.timer_num = LEDC_TIMER_0;
  tmcfg.freq_hz =20000;

  _chcfg_1 = chcfg_1;
  _chcfg_2 = chcfg_2;  
  _tmcfg = tmcfg;
  ledc_timer_config(&tmcfg);
  ledc_channel_config(&chcfg_1);
  ledc_channel_config(&chcfg_2);  


  /*
   * gpio configuration
   */
  // nD2
  //  gpio_initialize_output(_nD2, 1);

  // IN1, 2
  gpio_initialize_output(_IN1, 0);
  gpio_initialize_output(_IN2, 0);

  // SF

  /*
   * ADC1
   */
  adc1_config_width(ADC_WIDTH_12Bit);
  //  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_11db); // GPIO 36
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_0db); // GPIO 36
}

void MC33926Driver::SetPWM(int32_t speed)
{
  uint8_t reverse = 0;

  if (speed < 0)
  {
    speed = -speed;
    reverse = -1;
  }
  if (speed > _maxPWM)
    speed = _maxPWM;

  
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, _chcfg_1.channel, 0);//speed);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, _chcfg_2.channel, 0);//speed);
  if(reverse){
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, _chcfg_1.channel, speed);//speed);        

  }
  else{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, _chcfg_2.channel, speed);//speed);

  }
  
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, _chcfg_1.channel);  
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, _chcfg_2.channel);
  
  /*  if (reverse) {
    gpio_set_level(static_cast<gpio_num_t>(_IN1),0);
    gpio_set_level(static_cast<gpio_num_t>(_IN2),1);
  } else {
    gpio_set_level(static_cast<gpio_num_t>(_IN1),1);
    gpio_set_level(static_cast<gpio_num_t>(_IN2),0);
    }*/
}

double MC33926Driver::getCurrentMilliamps()
{
  // 1.0 V / 4096 ADC counts / 525mV per A = 1.67410 mA per count
  int raw = adc1_get_voltage(ADC1_CHANNEL_3);
  double vout = raw * ((1.0 / 4096));
  double vsense = vout * 100.0/4700.0;
  double isense = vsense / 20e-3;
  static const char *tag = "m33926";  
  //  ESP_LOGI(tag, "current %f", isense*1000);
  //  ESP_LOGI(tag, "raw %d", raw);
  return isense * 1000;
}



