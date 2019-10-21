/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_altmdm.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/modem/altmdm.h>

#if defined(CONFIG_MODEM_ALTMDM) && defined(CONFIG_CXD56_GPIO_IRQ)
#include <arch/board/board.h>
#include <arch/board/cxd56_altmdm.h>

#include "cxd56_gpio.h"
#include "cxd56_gpioint.h"
#include "cxd56_pinconfig.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_MODEM_ALTMDM) && defined(CONFIG_CXD56_GPIO_IRQ)

#define ALTMDM_SHUTDOWN  (PIN_SPI2_MISO)
#define MODEM_WAKEUP     (PIN_SPI2_MOSI)
#define MASTER_REQUEST   (PIN_RTC_IRQ_OUT)
#define SLAVE_REQUEST    (PIN_SPI2_SCK)
#define LTE_POWER_BUTTON (PIN_AP_CLK)
#define NUM_OF_PINS      (sizeof(pincfg) / sizeof(struct altmdm_pincfg))

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(CONFIG_MODEM_ALTMDM) && defined(CONFIG_CXD56_GPIO_IRQ)

struct altmdm_pincfg
{
  uint32_t pin;
  bool input_enable;
  bool init_val;
};

#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_MODEM_ALTMDM) && defined(CONFIG_CXD56_GPIO_IRQ)

static const struct altmdm_pincfg pincfg[] =
{
  {MODEM_WAKEUP, false, false},    /* out, low */
  {MASTER_REQUEST, false, false},  /* out, low */
  {SLAVE_REQUEST, true, false},    /* in, low */
};

#endif

#if defined(CONFIG_MODEM_ALTMDM) && defined(CONFIG_CXD56_GPIO_IRQ)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_altmdm_poweron
 *
 * Description:
 *   Power on the Altair modem device on the board.
 *
 ****************************************************************************/

void board_altmdm_poweron(void)
{
  int i;

  /* Power on altair modem device */

  cxd56_gpio_config(ALTMDM_SHUTDOWN, false);
  cxd56_gpio_write(ALTMDM_SHUTDOWN, true);

  cxd56_gpio_config(LTE_POWER_BUTTON, false);
  cxd56_gpio_write(LTE_POWER_BUTTON, true);

  board_power_control(POWER_LTE, true);

  for (i = 0; i < NUM_OF_PINS; i++)
    {
      /* input pin: input enable */

      cxd56_gpio_config(pincfg[i].pin, pincfg[i].input_enable);

      /* if it is an output pin, write a default value */

      if (pincfg[i].input_enable == false)
        {
          cxd56_gpio_write(pincfg[i].pin, pincfg[i].init_val);
        }
    }

  /* Slave request seems to float in Lite Hibernation and becomes HIGH at some
   * times when it should stay LOW.
   */

  cxd56_pin_config(PINCONF_SET(SLAVE_REQUEST,
                               PINCONF_MODE0,
                               PINCONF_INPUT_ENABLE,
                               PINCONF_DRIVE_NORMAL, PINCONF_PULLDOWN));
}

/****************************************************************************
 * Name: board_altmdm_poweroff
 *
 * Description:
 *   Power off the Altair modem device on the board.
 *
 ****************************************************************************/

void board_altmdm_poweroff(void)
{
  int i;

  for (i = 0; i < NUM_OF_PINS; i++)
    {
      /* input disable, output disable(Hi-z) */

      cxd56_gpio_config(pincfg[i].pin, false);
    }

  /* power off Altair modem device */

  cxd56_gpio_write(ALTMDM_SHUTDOWN, true);

  board_power_control(POWER_LTE, false);

  cxd56_gpio_write(ALTMDM_SHUTDOWN, false);
  cxd56_gpio_write(LTE_POWER_BUTTON, false);
}

/****************************************************************************
 * Name: board_altmdm_gpio_write
 *
 * Description:
 *   Write GPIO pin.
 *
 ****************************************************************************/

void board_altmdm_gpio_write(uint32_t pin, bool value)
{
  if (pin < NUM_OF_PINS)
    {
      if (pincfg[pin].input_enable == false)
        {
          cxd56_gpio_write(pincfg[pin].pin, value);
        }
    }
}

/****************************************************************************
 * Name: board_altmdm_gpio_read
 *
 * Description:
 *   Read GPIO pin.
 *
 ****************************************************************************/

bool board_altmdm_gpio_read(uint32_t pin)
{
  bool val = false;

  if (pin < NUM_OF_PINS)
    {
      if (pincfg[pin].input_enable == true)
        {
          val = cxd56_gpio_read(pincfg[pin].pin);
        }
    }

  return val;
}

/****************************************************************************
 * Name: board_altmdm_gpio_irq
 *
 * Description:
 *   Register GPIO irq.
 *
 ****************************************************************************/

void board_altmdm_gpio_irq(uint32_t pin, uint32_t polarity,
                           uint32_t noise_filter, xcpt_t irqhandler)
{
  uint32_t pol;
  uint32_t nf;

  switch (polarity)
    {
    case ALTMDM_GPIOINT_LEVEL_HIGH:
      pol = GPIOINT_LEVEL_HIGH;
      break;

    case ALTMDM_GPIOINT_LEVEL_LOW:
      pol = GPIOINT_LEVEL_LOW;
      break;

    case ALTMDM_GPIOINT_EDGE_RISE:
      pol = GPIOINT_EDGE_RISE;
      break;

    case ALTMDM_GPIOINT_EDGE_FALL:
      pol = GPIOINT_EDGE_FALL;
      break;

    case ALTMDM_GPIOINT_EDGE_BOTH:
      pol = GPIOINT_EDGE_BOTH;
      break;

    default:
      return;
      break;
    }

  if (noise_filter == ALTMDM_GPIOINT_NOISE_FILTER_ENABLE)
    {
      nf = GPIOINT_NOISE_FILTER_ENABLE;
    }
  else
    {
      nf = GPIOINT_NOISE_FILTER_DISABLE;
    }

  if (pin < NUM_OF_PINS)
    {
      if (pincfg[pin].input_enable == true)
        {
          if (irqhandler)
            {
              /* Attach then enable the new interrupt handler */

              cxd56_gpioint_config(pincfg[pin].pin,
                                   (GPIOINT_TOGGLE_MODE_MASK | nf | pol),
                                   irqhandler, NULL);
            }
        }
    }
}

/****************************************************************************
 * Name: board_altmdm_gpio_int_control
 *
 * Description:
 *   Enable or disable GPIO interrupt.
 *
 ****************************************************************************/

void board_altmdm_gpio_int_control(uint32_t pin, bool en)
{
  if (pin < NUM_OF_PINS)
    {
      if (pincfg[pin].input_enable == true)
        {
          if (en)
            {
              /* enable interrupt */

              cxd56_gpioint_enable(pincfg[pin].pin);
            }
          else
            {
              /* disable interrupt */

              cxd56_gpioint_disable(pincfg[pin].pin);
            }
        }
    }
}

#endif
