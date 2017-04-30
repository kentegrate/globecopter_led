#include "mc33926.hpp"
#include "utils.hpp"

MC33926Driver::MC33926Driver()
{
}

MC33926Driver::MC33926Driver( unsigned char IN1,
                              unsigned char IN2,
                              unsigned char PWM,
                              unsigned char FB,
                              unsigned char nD2,
                              unsigned char nSF)
{
  _IN1 = IN1;
  _IN2 = IN2;
  _PWM = PWM;
  _FB = FB;
  _nD2 = nD2;
  _nSF = nSF;
}

void MC33926Driver::init()
{
  /*
   * ledc configuration
   */
  ledc_channel_config_t chcfg;
  chcfg.gpio_num = _PWM;
  chcfg.speed_mode = LEDC_HIGH_SPEED_MODE; // what's this?
  chcfg.channel = static_cast<ledc_channel_t>(0);
  chcfg.intr_type = LEDC_INTR_DISABLE; // configure interrupt, Fade interrupt enable or Fade interrupt disable
  chcfg.timer_sel = LEDC_TIMER_2;
  chcfg.duty = 0;

  ledc_timer_config_t tmcfg;
  tmcfg.speed_mode = LEDC_HIGH_SPEED_MODE; // what's this?
  tmcfg.bit_num = LEDC_TIMER_10_BIT;
  tmcfg.timer_num = LEDC_TIMER_2;
  tmcfg.freq_hz = 20000;

  _chcfg = chcfg;
  _tmcfg = tmcfg;

  ledc_channel_config(&chcfg);
  ledc_timer_config(&tmcfg);

  /*
   * gpio configuration
   */
  // nD2
  gpio_initialize_output(_nD2, 1);

  // IN1, 2
  gpio_initialize_output(_IN1, 1);
  gpio_initialize_output(_IN2, 0);

  // SF
  gpio_initialize_input(_nSF);

  /*
   * ADC1
   */
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db); // GPIO 36

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

  ledc_set_duty(LEDC_HIGH_SPEED_MODE, _chcfg.channel, speed);

  if (reverse) {
    gpio_set_level(static_cast<gpio_num_t>(_IN1),0);
    gpio_set_level(static_cast<gpio_num_t>(_IN2),1);
  } else {
    gpio_set_level(static_cast<gpio_num_t>(_IN1),1);
    gpio_set_level(static_cast<gpio_num_t>(_IN2),0);
  }
}

double MC33926Driver::getCurrentMilliamps()
{
  // 3.6 V / 4096 ADC counts / 525mV per A = 1.67410 mA per count
  return adc1_get_voltage(ADC1_CHANNEL_0) * ((3.6 / 4096)/0.000525);
}

uint8_t MC33926Driver::getFault()
{
  return !gpio_get_level(static_cast<gpio_num_t>(_nSF));
}


