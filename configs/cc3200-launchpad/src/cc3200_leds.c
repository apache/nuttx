#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <debug.h>

#include <arch/board/board.h>
#include <chip/cc3200_memorymap.h>

#include "up_arch.h"

#include "cc3200_launchpad.h"
#include "cc3200_utils.h"

#define LED1_GPIO              9
#define LED2_GPIO              10
#define LED3_GPIO              11

/****************************************************************************
 * Name: cc3200_ledinit
 ****************************************************************************/

void cc3200_ledinit(void)
{

    uint32_t led1_port;
    uint8_t  led1_pin;
    uint32_t led2_port;
    uint8_t  led2_pin;
    uint32_t led3_port;
    uint8_t  led3_pin;
    uint8_t  x=16;

    putreg32(getreg32(0x44025000 + 0x00000058) | 0x00000001, 0x44025000 + 0x00000058);
    while(--x)
      ;
     
    cc3200_pin_type_gpio(PIN_01, PIN_MODE_0, false);
    cc3200_set_gpio_dir(TIVA_GPIOB_BASE, 0x4, GPIO_DIR_MODE_OUT);
    
    cc3200_pin_type_gpio(PIN_02, PIN_MODE_0, false);
    cc3200_set_gpio_dir(TIVA_GPIOB_BASE, 0x8, GPIO_DIR_MODE_OUT);

    cc3200_pin_type_gpio(PIN_64, PIN_MODE_0, false);
    cc3200_set_gpio_dir(TIVA_GPIOB_BASE, 0x2, GPIO_DIR_MODE_OUT);
    
    cc3200_get_gpio_port_pin(LED1_GPIO, &led1_port, &led1_pin);
    cc3200_get_gpio_port_pin(LED2_GPIO, &led2_port, &led2_pin);
    cc3200_get_gpio_port_pin(LED3_GPIO, &led3_port, &led3_pin);
    
    cc3200_set_gpio(LED1_GPIO, led1_port, led1_pin, 0);
    cc3200_set_gpio(LED2_GPIO, led2_port, led2_pin, 0);
    cc3200_set_gpio(LED3_GPIO, led3_port, led3_pin, 0);

}

/****************************************************************************
 * Name: cc3200_ledon
 ****************************************************************************/

void cc3200_ledon(int led)
{
    unsigned int  led1_port;
    unsigned char led1_pin;
    unsigned int  led2_port;
    unsigned char led2_pin;
    unsigned int  led3_port;
    unsigned char led3_pin;
    
    cc3200_get_gpio_port_pin(LED1_GPIO, &led1_port, &led1_pin);
    cc3200_get_gpio_port_pin(LED2_GPIO, &led2_port, &led2_pin);
    cc3200_get_gpio_port_pin(LED3_GPIO, &led3_port, &led3_pin);
    
    switch (led)
    {
      /* All */

      default:
      case 0:
	cc3200_set_gpio(LED1_GPIO, led1_port, led1_pin, 1);
	cc3200_set_gpio(LED2_GPIO, led2_port, led2_pin, 1);
	cc3200_set_gpio(LED3_GPIO, led3_port, led3_pin, 1);    	
        break;

      /* GREEN */

      case 1:
        cc3200_set_gpio(LED3_GPIO, led3_port, led3_pin, 1); 
        break;

      /*  YELLOW */

      case 2:
        cc3200_set_gpio(LED2_GPIO, led2_port, led2_pin, 1);
        break;

      /*  RED */

      case 3:
	cc3200_set_gpio(LED1_GPIO, led1_port, led1_pin, 1);
	break;

    }
}

/****************************************************************************
 * Name: cc3200_ledoff
 ****************************************************************************/

void cc3200_ledoff(int led)
{

    unsigned int  led1_port;
    unsigned char led1_pin;
    unsigned int  led2_port;
    unsigned char led2_pin;
    unsigned int  led3_port;
    unsigned char led3_pin;
    
    cc3200_get_gpio_port_pin(LED1_GPIO, &led1_port, &led1_pin);
    cc3200_get_gpio_port_pin(LED2_GPIO, &led2_port, &led2_pin);
    cc3200_get_gpio_port_pin(LED3_GPIO, &led3_port, &led3_pin);
    
    switch (led)
    {
      /* All */

      default:
      case 0:
	cc3200_set_gpio(LED1_GPIO, led1_port, led1_pin, 0);
	cc3200_set_gpio(LED2_GPIO, led2_port, led2_pin, 0);
	cc3200_set_gpio(LED3_GPIO, led3_port, led3_pin, 0);    	
        break;

      /* GREEN */

      case 1:
        cc3200_set_gpio(LED3_GPIO, led3_port, led3_pin, 0); 
        break;

      /*  YELLOW */

      case 2:
        cc3200_set_gpio(LED2_GPIO, led2_port, led2_pin, 0);
        break;

      /*  RED */

      case 3:
	cc3200_set_gpio(LED1_GPIO, led1_port, led1_pin, 0);
	break;

    }
}