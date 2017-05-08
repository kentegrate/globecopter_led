#include "esp_log.h"
#include "pwm_decoder.hpp"
#include "utils.hpp"
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

int PWMDecoder::id = 0;

static xQueueHandle pwm_evt_queue = NULL;
PWMDecoder* PWMDecoder::pwm0 = NULL;
PWMDecoder* PWMDecoder::pwm1 = NULL;
static void IRAM_ATTR gpio_interrupt(void* arg){
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(pwm_evt_queue, &gpio_num, NULL);
}

static void decode(void* arg){
  uint32_t io_num;
  for(;;){
    if(xQueueReceive(pwm_evt_queue, &io_num, portMAX_DELAY)){
      PWMDecoder* decoder = PWMDecoder::getPWMDecoder(io_num);
      int8_t level = gpio_get_level(static_cast<gpio_num_t>(io_num));
      //      static const char *tag = "PWM";

      if (level){//HIGH, posedge
	gettimeofday(&(decoder->posedge_time), NULL);
      }
      else{ //LOW, negedge
	struct timeval negedge_time, time_delta;
	gettimeofday(&negedge_time, NULL);
	timersub(&negedge_time, &(decoder->posedge_time), &time_delta);
	decoder->width = 0.1*time_delta.tv_usec + 0.9*decoder->width;
	//	ESP_LOGI(tag, "tv_sec %ld", time_delta.tv_sec);
	//	ESP_LOGI(tag, "pin: %d width: %d", decoder->pin, decoder->width);
	
      }
    }
  }
}

void PWMDecoder::init(int _pin){
  width = 0;
  pin = _pin;
  if(pin == 34){
    pwm0 = this;    
    gpio_initialize_interrupt(pin, GPIO_INTR_ANYEDGE, GPIO_PULLUP_ONLY);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(static_cast<gpio_num_t>(pin), gpio_interrupt, (void*)pin);        

  }
  else{
    pwm1 = this;        
    gpio_isr_handler_add(static_cast<gpio_num_t>(pin), gpio_interrupt, (void*)pin);    
    pwm_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(decode, "pwm gpio interrupt", 2048, NULL, 20, NULL);

  }


}



PWMDecoder* PWMDecoder::getPWMDecoder(int pin){
  if(pin == 34)
    return pwm0;
  else if(pin == 35)
    return pwm1;
  else
    return NULL;
}
