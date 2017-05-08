#include "encoder.hpp"
#include "utils.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "driver/pcnt.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "soc/gpio_sig_map.h"
#include "Adafruit_DotStar.hpp"
#include "globe_pattern.h"
#define MAX_COUNT 6170
//#define GEAR_RATIO 51.46;//51.47 is a little big and 51.45 is a little smaill
#define GEAR_RATIO 52;//51.47 is a little big and 51.45 is a little smaill
pcnt_dev_t PCNT;
KF kf;
xQueueHandle pcnt_evt_queue;  /*A queue to handle pulse counter event*/
int16_t last_count = 0;
typedef struct {
  int unit;        /*pulse counter unit*/
  uint32_t status; /*pulse counter internal status*/
} pcnt_evt_t;

static void gpio_task_example(void* arg){
  nvs_flash_init();  
  //  static const char *tag = "FOR";
  int16_t count = 0;
  int16_t last_count = 0;
  Adafruit_DotStar strip = Adafruit_DotStar(200, DOTSTAR_BRG);
  uint32_t color;
  strip.begin();
  strip.show();
  int last_degree = 0;
  for(;;){
    kf._timeUpdate();
    int current_degree = (int)(kf.theta/360.0*200.0);

    if(last_degree == current_degree)
      continue;
    
    for(int i = 0; i < 200; i++){
      strip.setPixelColor(i, globepattern[current_degree*200+i]);
      //     strip.setPixelColor(i, 0x00000f);
      //      strip.setPixelColor(i, 0x4020b);
      //      strip.setPixelColor(i+100, 0xc0f08);
    }
    strip.show();
    last_degree = current_degree;

  }
    

}
 // Constructors
Encoder::Encoder()
{

}

 Encoder::Encoder(uint8_t PhaseA, uint8_t PhaseB, double coeff_step2rad)
 {
   _PhaseA = PhaseA;
   _PhaseB = PhaseB;
   _coeff_step2rad = coeff_step2rad;
   filter = &kf;
 }

// Public method
void Encoder::init()
{
  pcnt_config_t pcnt_config;
  pcnt_config.pulse_gpio_num = 32;
  pcnt_config.ctrl_gpio_num = 38;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = PCNT_UNIT_0;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_INC;
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.counter_h_lim = MAX_COUNT;
  pcnt_config.counter_l_lim = -1;
  pcnt_unit_config(&pcnt_config);
  
  pcnt_config.pulse_gpio_num = 33;
  pcnt_config.ctrl_gpio_num = 38;
  pcnt_config.channel = PCNT_CHANNEL_1;
  pcnt_unit_config(&pcnt_config);  
  
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  pcnt_config.pulse_gpio_num = 32;
  pcnt_config.ctrl_gpio_num = 38;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = PCNT_UNIT_1;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_INC;
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.counter_h_lim = 3;
  pcnt_config.counter_l_lim = -1;
  pcnt_unit_config(&pcnt_config);
  
  pcnt_config.pulse_gpio_num = 33;
  pcnt_config.ctrl_gpio_num = 38;
  pcnt_config.channel = PCNT_CHANNEL_1;
  pcnt_unit_config(&pcnt_config);  
  
  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_1);
  pcnt_counter_resume(PCNT_UNIT_1);
  

  xTaskCreatePinnedToCore(gpio_task_example, "gpio_task_example", 2048, NULL, 20, NULL, 1);
  _HandleEncoderUpdate = xTimerCreate("EncoderUpdate", 15, pdTRUE, NULL, _EncoderUpdatefunc);
  if(_HandleEncoderUpdate != NULL){
    xTimerStart(_HandleEncoderUpdate, 0);
  }

  pwm_decoder_0.init(34);
  pwm_decoder_1.init(35);  
}



void Encoder::_EncoderUpdatefunc(TimerHandle_t xTimer){
  int16_t count = 0;  
  pcnt_get_counter_value(PCNT_UNIT_0, &count);
  int16_t pulse_count;
  if(count < last_count){
    pulse_count = MAX_COUNT - last_count + count;
  }
  else{
    pulse_count = count - last_count;
  }
  kf.omega_raw = 1000.0*pulse_count/12.0/15*360;//degree per sec
  last_count = count;
  kf._measurementUpdate();
  kf.theta_encoder += pulse_count / 12.0 * 360.0 /GEAR_RATIO;
  while(kf.theta_encoder > 360){
    kf.theta_encoder -= 360;
  }
  gettimeofday(&(kf.last_update), NULL);
}
KF::KF(){
   P = 0.1;
   w = 0;
   theta = 0;
}
  
void KF::_measurementUpdate(){
  double R = 20;
  double Q = 20;
  
  double P_post = P + Q;
  double G = P_post / (P_post + R);
  w = w + G*(omega_raw - w); //w is degree per sec for the motor
  P = (1-G)*P_post;

  
}
void KF::_timeUpdate(){
  struct timeval current_time, time_delta;
  gettimeofday(&current_time, NULL);
  
  timersub(&current_time, &last_update, &time_delta);
  theta = theta_encoder + (time_delta.tv_usec)*(1.0e-6)*w/GEAR_RATIO;
  while(theta > 360){
    theta -= 360;
  }
  //  static const char *tag = "KF";
  //  ESP_LOGI(tag, "theta %f", theta);
  
}
