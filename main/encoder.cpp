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
#define MAX_COUNT 6000
pcnt_dev_t PCNT;
xQueueHandle pcnt_evt_queue;  /*A queue to handle pulse counter event*/
int16_t last_count = 0;
typedef struct {
  int unit;        /*pulse counter unit*/
  uint32_t status; /*pulse counter internal status*/
} pcnt_evt_t;

static void gpio_task_example(void* arg){
  nvs_flash_init();  
  static const char *tag = "FOR";
  int16_t count = 0;
  int16_t last_count = 0;
  Adafruit_DotStar strip = Adafruit_DotStar(200, DOTSTAR_BRG);
  uint32_t color;
  strip.begin();
  //  strip.show();
  
  for(;;){

    /*    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    count = count % 600;
    if(count %3 == 0){
      if(count == last_count)
	continue;
      int phase = count / 3;

      if(phase%3 == 0){
	color = 0x0f0000;
      }
      else if(phase%3 == 1){
	color = 0x000f00;
      }
      else{
	color = 0x00000f;
      }
      for(int i = 0; i < 200; i++){
	strip.setPixelColor(i, color);
      }
      strip.show();
      last_count = count;*/
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
   P = 0.1;
   w = 0;
 }

// Public method
void Encoder::init()
{
    // PhaseA

  //  gpio_initialize_interrupt(_PhaseB, GPIO_INTR_ANYEDGE, GPIO_PULLUP_ONLY);
  if(id==0) {
    /*
      gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));h
      xTaskCreate(gpio_task_example, "")
    */
    

  }
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
  

  //  pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
  xTaskCreatePinnedToCore(gpio_task_example, "gpio_task_example", 2048, NULL, 20, NULL, 1);
  _HandleEncoderUpdate = xTimerCreate("EncoderUpdate", 15, pdTRUE, (void*)this, _EncoderUpdatefunc);
  if(_HandleEncoderUpdate != NULL){
    xTimerStart(_HandleEncoderUpdate, 0);
  }
}

void Encoder::_calctheta()
{
    theta_processed = theta_raw - 2*M_PI*(int)(theta_raw/(2*M_PI));
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
  Encoder* encoder = (Encoder*)pvTimerGetTimerID(xTimer);
  encoder->theta_raw = 1000.0*pulse_count/12.0/15;
  static const char *tag = "Encoder";
  //  ESP_LOGI(tag, "RPS %f", encoder->theta_raw);
  //  ESP_LOGI(tag, "pulse count %d", pulse_count);  
  last_count = count;
  encoder->_updateKF();
}

void Encoder::_updateKF(){
  double R = 20;
  double Q = 20;
  
  double P_post = P + Q;
  double G = P_post / (P_post + R);
  w = w + G*(theta_raw - w);
  P = (1-G)*P_post;
  static const char *tag = "Encoder";
  //  ESP_LOGI(tag, "filtered %f", w);
  
}
