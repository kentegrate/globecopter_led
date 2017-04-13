#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include "nvs_flash.h"

#include "esp_event.h"
#include "esp_event_loop.h"
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "sdkconfig.h"
#include "spi.hpp"

#include "Adafruit_DotStar.hpp"

#include <driver/ledc.h>
extern "C" void app_main()
{
  //  static char tag[] = "led_dim";
  nvs_flash_init();

  //  ring_led_data led_data(200, 360);
  //  led_data.set_data(fullcolor);
  
  Adafruit_DotStar strip = Adafruit_DotStar(200, DOTSTAR_BRG);  
  int c = 0;
  uint32_t color;
  strip.begin();
  strip.show();

  while(1){// show red, green, and blue forever.
    for(int k = 0; k < 360; k++){
      for(int i = 0; i < 200; i++){
	if(c == 0){
	  color = 0x0f0000;
	}
	else if(c==1){
	  color = 0x000f00;
	}
	else{
	  color = 0x00000f;
	}
	strip.setPixelColor(i, color);
	c += 1;
	if(c > 2)
	  c = 0;
      }
      strip.show();                     // Refresh strip
    }
  }
  
}
