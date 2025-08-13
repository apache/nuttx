#include <nuttx/config.h>
#include <sys/types.h>
#include <syslog.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/irq.h>
#include <arch/irq.h>
#include <nuttx/ioexpander/gpio.h>
#include "mindgrove_gpio.h"
#include "secure_iot_reg.h"
#include "plic.h"

#define MINDGROVE_IRQ_GPIO_INT0 (MINDGROVE_PLIC_START)
/****************************************************************************
 * MINDGROVE GPIO helper functions
 ****************************************************************************/

bool gpio_read(FAR struct gpio_dev_s *dev, FAR bool *value){
    FAR struct mindgrovegpio_dev_s *mindgrovegpio =
    (FAR struct mindgrovegpio_dev_s *)dev;
  uint32_t pin_status = (GPIO_REG->GPIO_DATA) & (mindgrovegpio->id);
  return (pin_status != 0) ? 1 : 0;
}

void gpio_set(uint32_t gpio_pins,bool high){
  if(high)
  {
    GPIO_REG->GPIO_SET = (gpio_pins);
  }
  else{
    GPIO_REG->GPIO_CLEAR = (gpio_pins);
  }
  
  
}

void configure_gpio(uint32_t gpio_pins,bool direction){
  
  for (uint8_t i = 0; i <= 7; i++)
  {
    if (gpio_pins & (1 << i))
    {
      if (((unsigned int *)(PINMUX0_BASE)) + i == 1)
      {
        
      }
    }
  }
  GPIO_REG->GPIO_DIRECTION &= ~(gpio_pins);
  if(direction)
  GPIO_REG->GPIO_DIRECTION |= (gpio_pins);
  else
  GPIO_REG->GPIO_DIRECTION &= ~(gpio_pins);
}

void gpio_interrupt_config(uint32_t gpio_pins, int low_ena) {
  if (low_ena)
  GPIO_REG->GPIO_INTR &= gpio_pins;        
  else
  GPIO_REG->GPIO_INTR |= gpio_pins;
  
}

// #if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)




/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mindgrovegpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
  pin_interrupt_t         callback;
};

static struct mindgrovegpio_dev_s g_mindgrove_gpio[32];
struct mindgrove_gpint_dev_s
{
  struct mindgrovegpio_dev_s mindgrovegpio;

};
static struct mindgrovegpio_dev_s g_gpout[32];

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct gpio_operations_s gpin_ops =
{
  .go_read       = gpio_read,
  .go_write      = gpio_set,
  .go_attach     = gpio_irq_attach,
  .go_enable     = gpio_interrupt_enable,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct mindgrovegpio_dev_s *mindgrovegpio =
    (FAR struct mindgrovegpio_dev_s *)dev;

  DEBUGASSERT(mindgrovegpio != NULL && value != NULL);
  DEBUGASSERT(mindgrovegpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading...\n");

  *value = (int) read_gpio(g_gpioinputs[mindgrovegpio->id]);
  return OK;
}



/****************************************************************************
 * Name: gpout_read
 ****************************************************************************/


static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct mindgrovegpio_dev_s *mindgrovegpio =
    (FAR struct mindgrovegpio_dev_s *)dev;

  DEBUGASSERT(mindgrovegpio != NULL && value != NULL);
  DEBUGASSERT(mindgrovegpio->id < 32);
  gpioinfo("Reading...\n");

  *value = (int) read_gpio(g_gpiooutputs[mindgrovegpio->id]);
  return OK;
}

/****************************************************************************
 * Name: gpout_write
 ****************************************************************************/

static int gpout_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct mindgrovegpio_dev_s *mindgrovegpio =
    (FAR struct mindgrovegpio_dev_s *)dev;

  DEBUGASSERT(mindgrovegpio != NULL);
  DEBUGASSERT(mindgrovegpio->id < 32);
  gpioinfo("Writing %d\n", (int)value);

  set_gpio( g_gpiooutputs[mindgrovegpio->id], value);

  return OK;
}




/****************************************************************************
 * Name: mindgrove_gpio_interrupt
 *
 * Description:
 *   gpio interrupt.
 *
 ****************************************************************************/

static int mindgrove_gpio_interrupt(int irq, void *context, void *arg)
{
  FAR struct mindgrove_gpint_dev_s *mindgrovexgpint =
    (FAR struct mindgrove_gpint_dev_s *)arg;

  uint32_t time_out = 0;
  uint8_t gpio_pin;

  DEBUGASSERT(mindgrovexgpint != NULL && mindgrovexgpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", mindgrovexgpint->callback);

  gpio_pin = g_gpiointinputs[mindgrovexgpint->mindgrovegpio.id];
  mindgrovexgpint->callback(&mindgrovexgpint->mindgrovegpio.gpio,
                        gpio_pin);

  return OK;
}

static int gpint_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct mindgrovegpio_dev_s *mindgrovegpio =
    (FAR struct mindgrovegpio_dev_s *)dev;

  DEBUGASSERT(mindgrovegpio != NULL && value != NULL);
  DEBUGASSERT(mindgrovegpio->id < BOARD_NGPIOINT);
  gpioinfo("Reading...\n");

  *value = (int) read_gpio(g_gpiointinputs[mindgrovegpio->id]);
  return OK;
}

static int gpio_irq_attach(FAR struct gpio_dev_s *dev,pin_interrupt_t callback){
  FAR struct mindgrove_gpint_dev_s *mindgrovexgpint =
    (FAR struct mindgrove_gpint_dev_s *)dev;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  mindgrovexgpint->callback = callback;

  irq_attach(mindgrovexgpint->, mindgrove_gpio_interrupt, dev);

  gpioinfo("Attach %p\n", callback);
  return OK;
}



static int gpio_interrupt_enable(FAR struct gpio_dev_s *dev, bool enable){
  FAR struct mindgrove_gpint_dev_s *mindgrove_gpint =
    (FAR struct mindgrove_gpint_dev_s *)dev;

  if (enable)
    {
      if (mindgrove_gpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");
          up_enable_irq(mindgrove_IRQ_GPIO_INT0);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      up_disable_irq(mindgrove_IRQ_GPIO_INT0);
    }

  return OK;

}




/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mindgrove_gpio_init
 ****************************************************************************/

int mindgrove_gpio_init(void)
{
  int i;
  int pincount = 0;


  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register(&g_gpin[i].gpio, pincount);

      /* Configure the pin that will be used as input */

      configure_gpio(g_gpioinputs[i],false);

      pincount++;
    }
  pincount=0;

#if 32 > 0
  for (i = 0; i < 32; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register(&g_gpout[i].gpio, pincount);

      /* Configure the pins that will be used as output */

      configure_gpio(g_gpiooutputs[i], true);
      set_gpio(g_gpiooutputs[i], false);

      pincount++;
    }

    pincount=0;
#endif



  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].mindgrovegpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].mindgrovegpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].mindgrovegpio.id              = i;
      gpio_pin_register(&g_gpint[i].mindgrovegpio.gpio, pincount);

      configure_gpio(g_gpiointinputs[i],false);
      gpiov2_interrupt_config(g_gpiointinputs[i],true);
      pincount++;
    }


  return OK;
}
 /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
