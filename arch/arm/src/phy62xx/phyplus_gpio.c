/****************************************************************************
 * arch/arm/src/phy62xx/phyplus_gpio.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/ioexpander/gpio.h>
#include <arch/board/board.h>
#include "gpio.h"
#include "chip.h"
#include "phyplus_stub.h"
#include "mcu_phy_bumbee.h"
#include "pwrmgr.h"
#include "phyplus_gpio.h"
#include "errno.h"

#if defined(CONFIG_DEV_GPIO) 

/****************************************************************************
 * phy6222  internal used functions..
 ****************************************************************************/

static void hal_low_power_io_init(void)
{
    ioinit_cfg_t ioinit[] =
    {
        {GPIO_P02,   GPIO_FLOATING   },
        {GPIO_P03,   GPIO_FLOATING   },
        {GPIO_P09,   GPIO_PULL_UP    },
        {GPIO_P10,   GPIO_PULL_UP    },
        {GPIO_P11,   GPIO_PULL_DOWN  },
        {GPIO_P14,   GPIO_PULL_DOWN  },
        {GPIO_P15,   GPIO_PULL_DOWN  },
        {GPIO_P16,   GPIO_FLOATING   },
        {GPIO_P18,   GPIO_PULL_DOWN  },
        {GPIO_P20,   GPIO_PULL_DOWN  },
        {GPIO_P00,   GPIO_PULL_DOWN  },
        {GPIO_P01,   GPIO_PULL_DOWN  },
        {GPIO_P07,   GPIO_PULL_DOWN  },
        {GPIO_P17,   GPIO_FLOATING   },
        {GPIO_P23,   GPIO_PULL_DOWN  },
        {GPIO_P24,   GPIO_PULL_DOWN  },
        {GPIO_P25,   GPIO_PULL_DOWN  },
        {GPIO_P26,   GPIO_PULL_DOWN  },
        {GPIO_P27,   GPIO_PULL_DOWN  },
        {GPIO_P31,   GPIO_PULL_DOWN  },
        {GPIO_P32,   GPIO_PULL_DOWN  },
        {GPIO_P33,   GPIO_PULL_DOWN  },
        {GPIO_P34,   GPIO_PULL_DOWN  },
    };
  for (uint8_t i = 0; i < sizeof(ioinit) / sizeof(ioinit_cfg_t); i++)
    {
      hal_gpio_pull_set(ioinit[i].pin, ioinit[i].type);
    }
}

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct phyplus_gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t idx;
  pin_interrupt_t callback;
};

static struct phyplus_gpio_dev_s g_phyplus_gpio_dev[GPIO_NUM];
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int phyplus_gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int phyplus_gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int phyplus_gpout_write(FAR struct gpio_dev_s *dev, bool value);
static int phyplus_gpint_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int phyplus_gpint_attach(FAR struct gpio_dev_s *dev,
                FAR pin_interrupt_t callback);
static int phyplus_gpint_enable(FAR struct gpio_dev_s *dev, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s gpin_ops =
{
  .go_read   = phyplus_gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};

static const struct gpio_operations_s gpout_ops =
{
  .go_read   = phyplus_gpout_read,
  .go_write  = phyplus_gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

static const struct gpio_operations_s gpint_ops =
{
  .go_read   = phyplus_gpint_read,
  .go_write  = NULL,
  .go_attach = phyplus_gpint_attach,
  .go_enable = phyplus_gpint_enable,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*  static int phyplus_gpio_interrupt(int irq, void *context, void *arg)
 *  {
 *  #if 0
 *    FAR struct stm32gpint_dev_s *stm32gpint =
 *                          (FAR struct stm32gpint_dev_s *)arg;
 *
 *    DEBUGASSERT(stm32gpint != NULL && stm32gpint->callback != NULL);
 *    gpioinfo("Interrupt! callback=%p\n", stm32gpint->callback);
 *
 *    stm32gpint->callback(&stm32gpint->stm32gpio.gpio,
 *                         stm32gpint->stm32gpio.id);
 *  #endif
 *
 *    FAR struct phyplus_gpio_dev_s *phyplus_gpint =
 *                          (FAR struct phyplus_gpio_dev_s *)arg;
 *
 *    DEBUGASSERT(phyplus_gpint != NULL && phyplus_gpint->callback != NULL);
 *    gpioinfo("Interrupt! callback=%p\n", phyplus_gpint->callback);
 *
 *    phyplus_gpint->callback(&phyplus_gpint->gpio,
 *                         phyplus_gpint->idx);
 *
 *    return 0;
 *  }
 */

static int phyplus_gpin_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
#if 0
  FAR struct stm32gpio_dev_s *stm32gpio =
                        (FAR struct stm32gpio_dev_s *)dev;

  DEBUGASSERT(stm32gpio != NULL && value != NULL);
  DEBUGASSERT(stm32gpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading...\n");

  *value = stm32_gpioread(g_gpioinputs[stm32gpio->id]);
#endif 
  FAR struct phyplus_gpio_dev_s *phyplus_gpin =
                        (FAR struct phyplus_gpio_dev_s *)dev;

  DEBUGASSERT(phyplus_gpin != NULL && value != NULL);
  DEBUGASSERT(phyplus_gpin->idx < GPIO_NUM);
  gpioinfo("Reading...\n");

  *value = hal_gpio_read(phyplus_gpin->idx);

  return 0;
}

static int phyplus_gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
#if 0
  FAR struct stm32gpio_dev_s *stm32gpio =
                        (FAR struct stm32gpio_dev_s *)dev;

  DEBUGASSERT(stm32gpio != NULL && value != NULL);
  DEBUGASSERT(stm32gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = stm32_gpioread(g_gpiooutputs[stm32gpio->id]);
#endif	

  FAR struct phyplus_gpio_dev_s *phyplus_gpout =
                (FAR struct phyplus_gpio_dev_s *)dev;

  DEBUGASSERT(phyplus_gpout != NULL && value != NULL);
  DEBUGASSERT(phyplus_gpout->idx < GPIO_NUM);
  gpioinfo("Reading...\n");

  *value = hal_gpio_read(phyplus_gpout->idx);

  return 0;
}

static int phyplus_gpout_write(FAR struct gpio_dev_s *dev, bool value)
{
#if 0
  FAR struct stm32gpio_dev_s *stm32gpio =
                             (FAR struct stm32gpio_dev_s *)dev;

  DEBUGASSERT(stm32gpio != NULL);
  DEBUGASSERT(stm32gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  stm32_gpiowrite(g_gpiooutputs[stm32gpio->id], value);
#endif

  FAR struct phyplus_gpio_dev_s *phyplus_gpout =
                             (FAR struct phyplus_gpio_dev_s *)dev;

  DEBUGASSERT(phyplus_gpout != NULL);
  DEBUGASSERT(phyplus_gpout->idx < GPIO_NUM);
  gpioinfo("Writing %d\n", (int)value);

  hal_gpio_write(phyplus_gpout->idx, value);

  return 0;
}

static int phyplus_gpint_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
#if 0
  FAR struct stm32gpint_dev_s *stm32gpint =
                              (FAR struct stm32gpint_dev_s *)dev;

  DEBUGASSERT(stm32gpint != NULL && value != NULL);
  DEBUGASSERT(stm32gpint->stm32gpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = stm32_gpioread(g_gpiointinputs[stm32gpint->stm32gpio.id]);
#endif

  FAR struct phyplus_gpio_dev_s *phyplus_gpint =
                             (FAR struct phyplus_gpio_dev_s *)dev;

  DEBUGASSERT(phyplus_gpint != NULL && value != NULL);
  DEBUGASSERT(phyplus_gpint->idx < GPIO_NUM);
  gpioinfo("Reading int pin...\n");

  *value = hal_gpio_read(phyplus_gpint->idx);

  return 0;
}

/* static pin_interrupt_t phyplus_gpint_callback_register
 *                (FAR struct gpio_dev_s *dev, uint8_t pin)
 * {
 *  return 0;
 * }
 */

static int phyplus_gpint_attach(FAR struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
#if 0
  /* do the regist callback things... */

  FAR struct stm32gpint_dev_s *stm32gpint =
                             (FAR struct stm32gpint_dev_s *)dev;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  stm32_gpiosetevent(g_gpiointinputs[stm32gpint->stm32gpio.id], false,
                     false, false, NULL, NULL);

  gpioinfo("Attach %p\n", callback);
  stm32gpint->callback = callback;
#endif

  FAR struct phyplus_gpio_dev_s *phyplus_gpint =
                (FAR struct phyplus_gpio_dev_s *)dev;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  gpioinfo("Attach %p\n", callback);
  phyplus_gpint->callback = callback;

  return 0;
}

static int phyplus_gpint_enable(FAR struct gpio_dev_s *dev, bool enable)
{
#if 0
  FAR struct stm32gpint_dev_s *stm32gpint =
                              (FAR struct stm32gpint_dev_s *)dev;

  if (enable)
    {
      if (stm32gpint->callback != NULL)
        {
        gpioinfo("Enabling the interrupt\n");

  /* Configure the interrupt for rising edge */

        stm32_gpiosetevent(g_gpiointinputs[stm32gpint->stm32gpio.id],
                           true, false, false, stm32gpio_interrupt,
                           &g_gpint[stm32gpint->stm32gpio.id]);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      stm32_gpiosetevent(g_gpiointinputs[stm32gpint->stm32gpio.id],
                       false, false, false, NULL, NULL);
    }
#endif

  return 0;
}

static int phyplus_gpio_param_check(
                FAR struct phyplus_gpio_param_s *phyplus_gpio_param)
{
  if (phyplus_gpio_param->pin_idx > NUMBER_OF_PINS)
    {
      return -PPlus_ERR_INVALID_PARAM;
    }

  if (phyplus_gpio_param->mode >  PHYPLUS_GPIO_INTERRUPT)
    {
      return -PPlus_ERR_INVALID_PARAM;
    }

  if (phyplus_gpio_param->trig_mode > POL_RISING)
    {
      return -PPlus_ERR_INVALID_PARAM;
    }

  if (phyplus_gpio_param->default_val > PIN_HIGH)
    {
      return -PPlus_ERR_INVALID_PARAM;
    }

  if (phyplus_gpio_param->pin_pull > GPIO_PULL_DOWN)
    {
      return -PPlus_ERR_INVALID_PARAM;
    }

  return PPlus_SUCCESS;
}

int phyplus_gpio_register(FAR struct phyplus_gpio_param_s
                *phyplus_gpio_param)
{
  int ret;
  int pin_idx = 0;
  ret = phyplus_gpio_param_check(phyplus_gpio_param);
  if (0 != ret)
    {
      return -EPERM;
    }

  pin_idx = phyplus_gpio_param->pin_idx;
  if (phyplus_gpio_param->mode == PHYPLUS_GPIO_INPUT)
    {
      g_phyplus_gpio_dev[pin_idx].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_phyplus_gpio_dev[pin_idx].gpio.gp_ops = &gpin_ops;
      g_phyplus_gpio_dev[pin_idx].idx = pin_idx;
    }
  else if (phyplus_gpio_param->mode == PHYPLUS_GPIO_OUTPUT)
    {
      g_phyplus_gpio_dev[pin_idx].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_phyplus_gpio_dev[pin_idx].gpio.gp_ops = &gpout_ops;
      g_phyplus_gpio_dev[pin_idx].idx = pin_idx;
    }
  else if (phyplus_gpio_param->mode == PHYPLUS_GPIO_INTERRUPT)
    {
      g_phyplus_gpio_dev[pin_idx].gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_phyplus_gpio_dev[pin_idx].gpio.gp_ops = &gpint_ops;
      g_phyplus_gpio_dev[pin_idx].idx = pin_idx;
    }

  hal_gpio_pull_set(pin_idx, phyplus_gpio_param->pin_pull);
  gpio_pin_register(&g_phyplus_gpio_dev[pin_idx].gpio, pin_idx);
  return PPlus_SUCCESS;
}

int phyplus_gpio_unregister(FAR struct phyplus_gpio_param_s
                *phyplus_gpio_param)
{
  gpio_pin_unregister(
                &g_phyplus_gpio_dev[phyplus_gpio_param->pin_idx].gpio,
                phyplus_gpio_param->pin_idx);
  return PPlus_SUCCESS;
}

int phyplus_gpio_init(void)
{
  hal_low_power_io_init();

  hal_gpio_init();
  return PPlus_SUCCESS;
}

#endif
