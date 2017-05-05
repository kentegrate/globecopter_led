#include "encoder.hpp"
#include "utils.hpp"
int8_t Encoder::_encnum = 0;
static void IRAM_ATTR _updateEnc(void* arg)
{
  /*    Encoder *enc = (Encoder*)arg;
    int8_t newA = gpio_get_level(static_cast<gpio_num_t>(enc->_PhaseA));
    int8_t newB = gpio_get_level(static_cast<gpio_num_t>(enc->_PhaseB));

    enc->absCount += _encRef[( enc->_oldA << 3 )
                       | ( enc->_oldB << 2 )
                       | ( newA << 1 )
                       | ( newB )];

    enc->theta_raw = enc->_coeff_step2rad * enc->absCount;
    enc->_calctheta();

    enc->_oldA = newA;
    enc->_oldB = newB;*/
}

const int8_t _encRef[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
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
  gpio_initialize_interrupt(_PhaseA, GPIO_INTR_ANYEDGE, GPIO_PULLUP_ONLY);
  gpio_initialize_interrupt(_PhaseB, GPIO_INTR_ANYEDGE, GPIO_PULLUP_ONLY);
  if(id==0) {
    /*
      gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));h
      xTaskCreate(gpio_task_example, "")
    */
    

  }
  //    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
  //  gpio_isr_handler_add(static_cast<gpio_num_t>(_PhaseA),_updateEnc,(void*)static_cast<gpio_num_t>(_PhaseA));
  gpio_isr_register(_updateEnc, NULL, ESP_INTR_FLAG_EDGE,NULL);
  //    gpio_isr_handler_add(static_cast<gpio_num_t>(_PhaseB),_updateEnc,this);
  absCount=0;
}

void Encoder::_calctheta()
{
    theta_processed = theta_raw - 2*M_PI*(int)(theta_raw/(2*M_PI));
}


