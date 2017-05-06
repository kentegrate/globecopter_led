#include "encoder.hpp"
#include "utils.hpp"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"


int8_t Encoder::_encnum = 0;
const int8_t _encRef[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
static xQueueHandle gpio_evt_queue = NULL;
static long int pulse_count = 0;
static int8_t oldA, oldB;
static void IRAM_ATTR _updateEnc(void* arg)
{
  uint32_t gpio_num = (uint32_t) arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg){
  uint32_t io_num;
  static const char *tag = "Encoder";  
  for(;;){
    if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)){
      int8_t newA = gpio_get_level(static_cast<gpio_num_t>(ENCODER_PHASE_A));
      int8_t newB = gpio_get_level(static_cast<gpio_num_t>(ENCODER_PHASE_B));
      //      ESP_LOGI(tag, "INTR: [%d] GPIO[%d] LEVEL: %d",io_num,  ENCODER_PHASE_A, newA);
      //      ESP_LOGI(tag, "INTR: [%d] GPIO[%d] LEVEL: %d", io_num, ENCODER_PHASE_B, newB);

	int diff = _encRef[( oldA << 3 )
			   | ( oldB << 2 )
			   | ( newA << 1 )
			   | ( newB )];
	pulse_count += diff;

      oldA = newA;
      oldB = newB;
      ESP_LOGI(tag, "pulse diff %d", diff);
    }
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
   id = _encnum++;
 }

// Public method
void Encoder::init()
{
    // PhaseA
  oldA = gpio_get_level(static_cast<gpio_num_t>(_PhaseA));
  oldB = gpio_get_level(static_cast<gpio_num_t>(_PhaseB));
  
  gpio_initialize_interrupt(_PhaseA, GPIO_INTR_ANYEDGE, GPIO_PULLUP_ONLY);
  //  gpio_initialize_interrupt(_PhaseB, GPIO_INTR_ANYEDGE, GPIO_PULLUP_ONLY);
  if(id==0) {
    /*
      gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));h
      xTaskCreate(gpio_task_example, "")
    */
    

  }
  gpio_install_isr_service(0);

  gpio_isr_handler_add(static_cast<gpio_num_t>(_PhaseA),_updateEnc,(void*)_PhaseA);
  //  gpio_isr_register(_updateEnc, NULL, ESP_INTR_FLAG_EDGE,NULL);
  //  gpio_isr_handler_add(static_cast<gpio_num_t>(_PhaseB),_updateEnc,(void*)_PhaseB);
  absCount=0;
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 20, NULL);

  //  _HandleEncoderUpdate = xTimerCreate("EncoderUpdate", 100, pdTRUE, (void*)this, _EncoderUpdatefunc);
  if(_HandleEncoderUpdate != NULL){
    //    xTimerStart(_HandleEncoderUpdate, 0);
  }
}

void Encoder::_calctheta()
{
    theta_processed = theta_raw - 2*M_PI*(int)(theta_raw/(2*M_PI));
}

void Encoder::_EncoderUpdatefunc(TimerHandle_t xTimer){
  Encoder* encoder = (Encoder*)pvTimerGetTimerID(xTimer);
  encoder->theta_raw = 1000.0*pulse_count/12.0/100;
  static const char *tag = "Encoder";
  ESP_LOGI(tag, "RPS %f", encoder->theta_raw);
  ESP_LOGI(tag, "pulse count %ld", pulse_count);  
  pulse_count = 0;
}

