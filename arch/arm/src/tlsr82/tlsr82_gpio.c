/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_gpio.c
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

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/tlsr82/chip.h>

#include "tlsr82_gpio.h"
#include "tlsr82_analog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_ENABLE             1
#define GPIO_DISABLE            0
#define CONFIG_GPIO_IRQ_MAX_NUM 5

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
struct tlsr82_gpio_irq_cb
{
  xcpt_t        callback;
  void         *arg;
  gpio_cfg_t    cfg;
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
struct tlsr82_gpio_irq_cb gpio_irq_cbs[CONFIG_GPIO_IRQ_MAX_NUM];
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_DUMPREGS
static void tlsr82_gpio_dumpregs(const char *msg);
#else
#  define tlsr82_gpio_dumpregs(msg)
#endif

static void tlsr82_gpio_ds_ctrl(gpio_cfg_t cfg, uint8_t ds);
static void tlsr82_gpio_pol_ctrl(gpio_cfg_t cfg, uint8_t pol);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_DUMPREGS
static void tlsr82_gpio_dumpregs(const char *msg)
{
  int i = 0;
  gpioinfo("%s\n", msg);
  for (i = 0; i < 5; i++)
    {
      gpioinfo("Group %d Gpio Register:\n", i);
      gpioinfo("  ACT=0x%02x\n", GPIO_SETTING_ACT_REG(i));
      gpioinfo("  IN =0x%02x\n", GPIO_SETTING_IN_REG(i));

      if (i != 3)
        {
          gpioinfo("  IE =0x%02x\n", GPIO_SETTING_IE_REG(i));
        }
      else
        {
          gpioinfo("  IE =0x%02x\n", tlsr82_analog_read(ANALOG_PC_IE_ADDR));
        }

      gpioinfo("  OEN=0x%02x\n", GPIO_SETTING_OEN_REG(i));
      gpioinfo("  OUT=0x%02x\n", GPIO_SETTING_OUT_REG(i));
      gpioinfo("  POL=0x%02x\n", GPIO_SETTING_POL_REG(i));

      if (i != 3)
        {
          gpioinfo("  DS =0x%02x\n", GPIO_SETTING_DS_REG(i));
        }
      else
        {
          gpioinfo("  DS =0x%02x\n", tlsr82_analog_read(ANALOG_PC_DS_ADDR));
        }
    }

  gpioinfo("Interrupt Register:\n");
  for (i = 0; i < 5; i++)
    {
      gpioinfo("  IRQ NORMAL %d: 0x%02x\n", i, GPIO_IRQ_NORMAL_REG(i));
    }

  gpioinfo("  IRQ NORMAL ALL 0x%02x\n", GPIO_IRQ_NORMAL_ALL_REG);
  gpioinfo("  IRQ RISC: 0x%02x\n", GPIO_IRQ_RISC_EN_REG);
  gpioinfo("  IRQ MASK: 0x%08lx\n", IRQ_MASK_REG);

  for (i = 0; i < CONFIG_GPIO_IRQ_MAX_NUM; i++)
    {
      gpioinfo("gpio_irq_cbs[%d]->arg     : 0x%08x\n",
               i, gpio_irq_cbs[i].arg);
      gpioinfo("gpio_irq_cbs[%d]->callback: 0x%08x\n",
               i, gpio_irq_cbs[i].callback);
      gpioinfo("gpio_irq_cbs[%d]->cfg     : 0x%08x\n",
               i, gpio_irq_cbs[i].cfg);
    }
}
#endif

/****************************************************************************
 * Name: tlsr82_gpio_ds_ctrl
 *
 * Description:
 *   Configurate the gpio drive strength be high/low.
 *
 * Input Parameters:
 *   gpio_cfg_t - GPIO config information
 *   ds         - 1 : high drive strength
 *                0 : low drive strength
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

static void tlsr82_gpio_ds_ctrl(gpio_cfg_t cfg, uint8_t ds)
{
  uint32_t group;
  uint32_t pin;
  uint32_t bit;

  DEBUGASSERT(GPIO_VALID(cfg));

  group   = (uint32_t)GPIO_GET(GROUP, cfg);
  pin     = (uint32_t)GPIO_GET(PIN, cfg);
  bit     = 1 << pin;

  if (group == GPIO_VAL(GROUP, C))
    {
      if (ds)
        {
          tlsr82_analog_write(ANALOG_PC_DS_ADDR,
                              tlsr82_analog_read(ANALOG_PC_DS_ADDR) |
                              bit);
        }
      else
        {
          tlsr82_analog_write(ANALOG_PC_DS_ADDR,
                              tlsr82_analog_read(ANALOG_PC_DS_ADDR) &
                              (~bit));
        }
    }
  else
    {
      if (ds)
        {
          BM_SET(GPIO_SETTING_DS_REG(group), bit);
        }
      else
        {
          BM_CLR(GPIO_SETTING_DS_REG(group), bit);
        }
    }
}

/****************************************************************************
 * Name: tlsr82_gpio_pol_ctrl
 *
 * Description:
 *   Configurate the gpio interrupt polarity (interruot trigger edge).
 *
 * Input Parameters:
 *   gpio_cfg_t - GPIO config information
 *   pol        - 0 : rising edge trigger
 *                1 : falling edge trigger
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

static void tlsr82_gpio_pol_ctrl(gpio_cfg_t cfg, uint8_t pol)
{
  uint32_t group;
  uint32_t pin;

  DEBUGASSERT(GPIO_VALID(cfg));

  group   = (uint32_t)GPIO_GET(GROUP, cfg);
  pin     = (uint32_t)GPIO_GET(PIN, cfg);

  if (pol)
    {
      BM_SET(GPIO_SETTING_POL_REG(group), 1 << pin);
    }
  else
    {
      BM_CLR(GPIO_SETTING_POL_REG(group), 1 << pin);
    }
}

/****************************************************************************
 * Name: tlsr82_gpio_irq
 *
 * Description:
 *   GPIO interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
static int tlsr82_gpio_irq(int irq, void *context, void *arg)
{
  int i;
  void *arg_cb;
  xcpt_t callback;
  int ret = OK;

  gpioinfo("Gpio irq entry!\n");

  /* Clear the GPIO interrupt status */

  BM_SET(IRQ_SRC_REG, 1 << NR_GPIO_IRQ);

  /* Dispatch pending interrupts in the lower GPIO status register */

  for (i = 0; i < CONFIG_GPIO_IRQ_MAX_NUM; ++i)
    {
      if (gpio_irq_cbs[i].callback != NULL &&
          GPIO_IS(IRQ, NORMAL, gpio_irq_cbs[i].cfg))
        {
          arg_cb   = gpio_irq_cbs[i].arg;
          callback = gpio_irq_cbs[i].callback;
          ret = callback(irq, context, arg_cb);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: tlsr82_gpio_risc1_irq
 *
 * Description:
 *   GPIO interrupt handler.
 *
 ****************************************************************************/

static int tlsr82_gpio_risc0_irq(int irq, void *context, void *arg)
{
  int i;
  void *arg_cb;
  xcpt_t callback;
  int ret = OK;

  gpioinfo("Gpio risc0 irq entry!\n");

  /* Clear the GPIO interrupt status */

  BM_SET(IRQ_SRC_REG, 1 << NR_GPIO_RISC0_IRQ);

  /* Dispatch pending interrupts in the lower GPIO status register */

  for (i = 0; i < CONFIG_GPIO_IRQ_MAX_NUM; ++i)
    {
      if (gpio_irq_cbs[i].callback != NULL &&
          GPIO_IS(IRQ, RISC0, gpio_irq_cbs[i].cfg))
        {
          arg_cb   = gpio_irq_cbs[i].arg;
          callback = gpio_irq_cbs[i].callback;
          ret = callback(irq, context, arg_cb);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: tlsr82_gpio_risc1_irq
 *
 * Description:
 *   GPIO interrupt handler.
 *
 ****************************************************************************/

static int tlsr82_gpio_risc1_irq(int irq, void *context, void *arg)
{
  int i;
  void *arg_cb;
  xcpt_t callback;
  int ret = OK;

  gpioinfo("Gpio risc1 irq entry!\n");

  /* Clear the GPIO interrupt status */

  BM_SET(IRQ_SRC_REG, 1 << NR_GPIO_RISC1_IRQ);

  /* Dispatch pending interrupts in the lower GPIO status register */

  for (i = 0; i < CONFIG_GPIO_IRQ_MAX_NUM; ++i)
    {
      if (gpio_irq_cbs[i].callback != NULL &&
          GPIO_IS(IRQ, RISC1, gpio_irq_cbs[i].cfg))
        {
          arg_cb   = gpio_irq_cbs[i].arg;
          callback = gpio_irq_cbs[i].callback;
          ret = callback(irq, context, arg_cb);
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_gpio_as_gpio
 *
 * Description:
 *   Config the gpio as a gpio or mux function
 *
 * Input Parameters:
 *   gpio_cfg_t - GPIO config information
 *   enable     - true : enable gpio as gpio
 *                false: enable gpio as mux function
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

void tlsr82_gpio_as_gpio(gpio_cfg_t cfg, bool enable)
{
  uint32_t group;
  uint32_t pin;

  group   = (uint32_t)GPIO_GET(GROUP, cfg);
  pin     = (uint32_t)GPIO_GET(PIN, cfg);

  if (enable)
    {
      BM_SET(GPIO_SETTING_ACT_REG(group), BIT(pin));
    }
  else
    {
      BM_CLR(GPIO_SETTING_ACT_REG(group), BIT(pin));
    }
}

/****************************************************************************
 * Name: tlsr82_gpio_input_ctrl
 *
 * Description:
 *   Enable/Disable the gpio input function.
 *
 * Input Parameters:
 *   gpio_cfg_t - GPIO config information
 *   enable     - true : enable gpio input function
 *                false: disable gpio input function
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

void tlsr82_gpio_input_ctrl(gpio_cfg_t cfg, bool enable)
{
  uint32_t group;
  uint32_t pin;
  uint32_t bit;

  DEBUGASSERT(GPIO_VALID(cfg));

  group   = (uint32_t)GPIO_GET(GROUP, cfg);
  pin     = (uint32_t)GPIO_GET(PIN, cfg);
  bit     = 1 << pin;

  if (group == GPIO_VAL(GROUP, C))
    {
      if (enable)
        {
          tlsr82_analog_write(ANALOG_PC_IE_ADDR,
                              tlsr82_analog_read(ANALOG_PC_IE_ADDR) |
                              bit);
        }
      else
        {
          tlsr82_analog_write(ANALOG_PC_IE_ADDR,
                              tlsr82_analog_read(ANALOG_PC_IE_ADDR) &
                              (~bit));
        }
    }
  else
    {
      if (enable)
        {
          BM_SET(GPIO_SETTING_IE_REG(group), bit);
        }
      else
        {
          BM_CLR(GPIO_SETTING_IE_REG(group), bit);
        }
    }
}

/****************************************************************************
 * Name: tlsr82_gpio_output_ctrl
 *
 * Description:
 *   Enable/Disable the gpio output function.
 *
 * Input Parameters:
 *   gpio_cfg_t - GPIO config information
 *   enable     - true : enable gpio output function
 *                false: disable gpio output function
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

void tlsr82_gpio_output_ctrl(gpio_cfg_t cfg, bool enable)
{
  uint32_t group;
  uint32_t pin;
  uint32_t bit;

  DEBUGASSERT(GPIO_VALID(cfg));

  group   = (uint32_t)GPIO_GET(GROUP, cfg);
  pin     = (uint32_t)GPIO_GET(PIN, cfg);
  bit     = 1 << pin;

  if (!enable)
    {
      BM_SET(GPIO_SETTING_OEN_REG(group), bit);
    }
  else
    {
      BM_CLR(GPIO_SETTING_OEN_REG(group), bit);
    }
}

/****************************************************************************
 * Name: tlsr82_gpio_pupd_ctrl
 *
 * Description:
 *   Configurate the gpio pull-up/pull-down.
 *
 * Input Parameters:
 *   gpio_cfg_t - GPIO config information
 *   pupd       - 0 : none/float
 *                1 : pull-up x100 - 1M
 *                2 : pull-down x10 - 100K
 *                3 : pull-dup x1 - 10K
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

void tlsr82_gpio_pupd_ctrl(gpio_cfg_t cfg, uint8_t pupd)
{
  uint32_t group;
  uint32_t pin;
  uint32_t basereg;
  uint32_t mask;
  uint32_t shift;

  DEBUGASSERT(GPIO_VALID(cfg));

  group   = (uint32_t)GPIO_GET(GROUP, cfg);
  pin     = (uint32_t)GPIO_GET(PIN, cfg);

  basereg = 0x0e + (group << 1) + ((pin >= 4) ? 1 : 0);

  shift   = (pin >= 4) ? (2 * pin - 8) : (2 * pin);
  mask    = ~(0x03 << shift);

  /* Usb dp pull-up/down config */

  if (GPIO_PIN_PA6 == GPIO_CFG2PIN(cfg))
    {
      /* usb_dp_pullup_en(0); */
    }

  tlsr82_analog_write(basereg,
                     (tlsr82_analog_read(basereg) & mask) |
                      (pupd << shift));
}

/****************************************************************************
 * Name: tlsr82_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin attributes.
 *
 ****************************************************************************/

int tlsr82_gpioconfig(gpio_cfg_t cfg)
{
  uint32_t group;
  uint32_t pin;
  uint8_t mux;
  uint8_t shift;

  /* Get the gpio group and pin */

  group   = (uint32_t)GPIO_GET(GROUP, cfg);
  pin     = (uint32_t)GPIO_GET(PIN, cfg);

  /* Pull-up and Pull-down config */

  if (GPIO_GET(PUPD, cfg) <= GPIO_VAL(PUPD, PU10K))
    {
      tlsr82_gpio_pupd_ctrl(cfg, GPIO_GET(PUPD, cfg));
    }

  /* Input, Output, and Mux function config */

  if (GPIO_IS(AF, INPUT, cfg))
    {
      /* IE and OEN should be set to 1 for Input usage */

      tlsr82_gpio_input_ctrl(cfg, true);
      tlsr82_gpio_output_ctrl(cfg, false);

      /* GPIO setting act as GPIO */

      BM_SET(GPIO_SETTING_ACT_REG(group), BIT(pin));
    }
  else if (GPIO_IS(AF, OUTPUT, cfg))
    {
      /* IE and OEN should be set to 0 for Output usage */

      tlsr82_gpio_input_ctrl(cfg, false);
      tlsr82_gpio_output_ctrl(cfg, true);

      /* Drive Strength config */

      tlsr82_gpio_ds_ctrl(cfg, (bool)GPIO_GET(DS, cfg));

      /* GPIO setting act as GPIO */

      BM_SET(GPIO_SETTING_ACT_REG(group), BIT(pin));
    }
  else if (GPIO_GET(AF, cfg) >= GPIO_VAL(AF, MUX0) &&
           GPIO_GET(AF, cfg) <= GPIO_VAL(AF, MUX3))
    {
      /* Clear mux first and then set mux value */

      shift = (pin >= 4) ? (2 * pin - 8) : (2 * pin);
      mux   = GPIO_GET(AF, cfg) - GPIO_VAL(AF, MUX0);

      BM_CLR(GPIO_MUX_REG(group, pin), 0x03 << shift);

      BM_SET(GPIO_MUX_REG(group, pin), mux << shift);

      /* GPIO setting act as MUX */

      BM_CLR(GPIO_SETTING_ACT_REG(group), BIT(pin));
    }

  return OK;
}

/****************************************************************************
 * Name: tlsr82_unconfiggpio
 *
 * Description:
 *   Un-configure a GPIO pin based on encoded pin attributes.
 *
 ****************************************************************************/

int tlsr82_gpiounconfig(gpio_cfg_t cfg)
{
  gpio_cfg_t pinset;

  /* Get the pin infomation in cfg */

  pinset = GPIO_CFG2PIN(cfg);

  /* Config gpio as input and float state */

  pinset |= GPIO_AF_INPUT | GPIO_PUPD_NONE;

  return tlsr82_gpioconfig(pinset);
}

/****************************************************************************
 * Name: tlsr82_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void tlsr82_gpiowrite(gpio_cfg_t cfg, bool value)
{
  uint32_t group;
  uint32_t pin;

  group = (uint16_t)GPIO_GET(GROUP, cfg);
  pin   = (uint16_t)GPIO_GET(PIN, cfg);

  if (value)
    {
      BM_SET(GPIO_SETTING_OUT_REG(group), BIT(pin));
    }
  else
    {
      BM_CLR(GPIO_SETTING_OUT_REG(group), BIT(pin));
    }
}

/****************************************************************************
 * Name: tlsr82_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool tlsr82_gpioread(gpio_cfg_t cfg)
{
  uint32_t group;
  uint32_t pin;

  group = (uint32_t)GPIO_GET(GROUP, cfg);
  pin   = (uint32_t)GPIO_GET(PIN, cfg);

  if (GPIO_IS(AF, OUTPUT, cfg))
    {
      return BM_IS_SET(GPIO_SETTING_OUT_REG(group), BIT(pin)) != 0;
    }
  else
    {
      return BM_IS_SET(GPIO_SETTING_IN_REG(group), BIT(pin)) != 0;
    }
}

/****************************************************************************
 * Name: tlsr82_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
void tlsr82_gpioirqinitialize(void)
{
  int ret;
  int i;
  int irqs[3] =
    {
      NR_GPIO_IRQ,
      NR_GPIO_RISC0_IRQ,
      NR_GPIO_RISC1_IRQ
    };

  xcpt_t isrs[3] =
    {
      tlsr82_gpio_irq,
      tlsr82_gpio_risc0_irq,
      tlsr82_gpio_risc1_irq
    };

  for (i = 0; i < 3; i++)
    {
      /* Disable interrupt */

      up_disable_irq(irqs[i]);

      /* Attach the gpio interrupt */

      ret = irq_attach(irqs[i], isrs[i], NULL);
      if (ret == OK)
        {
          up_enable_irq(irqs[i]);
        }
      else
        {
          gpioerr("ERROR: GPIO interrupt:%d not attached!\n", irqs[i]);
        }
    }

  for (i = 0; i < CONFIG_GPIO_IRQ_MAX_NUM; i++)
    {
      gpio_irq_cbs[i].callback = NULL;
      gpio_irq_cbs[i].arg      = NULL;
      gpio_irq_cbs[i].cfg      = GPIO_INVLD_CFG;
    }
}
#endif

/****************************************************************************
 * Name: tlsr82_gpioirqconfig
 *
 * Description:
 *   Config the gpio irq according to the cfg, and attach or
 *   detach the interrupt callback function according to value of func.
 *   High Level or Rising Edge trigger, should set the pull-down resistor
 *   Low Level or Failing Edge trigger, should set the pull-up resistor
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
int tlsr82_gpioirqconfig(gpio_cfg_t cfg, xcpt_t func, void *arg)
{
  int i;
  gpio_cfg_t pinset;

  gpioinfo("cfg=0x%lx, func=0x%lx\n", cfg, (uint32_t)func);

  /* Get pin information from cfg */

  pinset = GPIO_CFG2PIN(cfg);

  /* func == NULL indicates disable this interrupt */

  if (func == NULL)
    {
      /* Disable interrupt when modify the callback */

      tlsr82_gpioirqdisable_all();

      for (i = 0; i < CONFIG_GPIO_IRQ_MAX_NUM; ++i)
        {
          if (GPIO_CFG2PIN(gpio_irq_cbs[i].cfg) == pinset)
            {
              gpio_irq_cbs[i].callback = NULL;
              gpio_irq_cbs[i].arg      = NULL;
              gpio_irq_cbs[i].cfg      = GPIO_INVLD_CFG;
              break;
            }
        }

      /* Disable corresponding pin interrupt */

      tlsr82_gpioirqdisable(cfg);
    }
  else
    {
      /* Find an approprite callback object */

      for (i = 0; i < CONFIG_GPIO_IRQ_MAX_NUM; ++i)
        {
          if (GPIO_CFG2PIN(gpio_irq_cbs[i].cfg) == pinset ||
              gpio_irq_cbs[i].callback == NULL)
            {
              /* 1. Re-config the same pin before unconfig it, just direct
               *    use pervious callback object.
               * 2. New pinset, use a new callback object.
               */

              break;
            }
        }

      if (i == CONFIG_GPIO_IRQ_MAX_NUM)
        {
          /* Callback objects have been ran out */

          gpioerr("Config Fail, gpio_irq_cbs[] have been ran out.\n");
          return -ENOSPC;
        }

      /* Disable interrupt when modify the callback */

      tlsr82_gpioirqdisable_all();

      /* Find callback object success */

      gpio_irq_cbs[i].callback = func;
      gpio_irq_cbs[i].arg      = arg;
      gpio_irq_cbs[i].cfg      = cfg;

      /* Enable corresponding pin interrupt */

      tlsr82_gpioirqenable(cfg);
    }

  /* Configure the gpio */

  tlsr82_gpioconfig(cfg);

  /* Config gpio interrupt polarity */

  tlsr82_gpio_pol_ctrl(cfg, GPIO_GET(POL, cfg));

  /* Clear the interrupt flag */

  tlsr82_gpioirqclear(cfg);

  /* Enable corresponding gpio interrupt */

  tlsr82_gpioirqenable_all();

  /* Dump the gpio register for debug */

  tlsr82_gpio_dumpregs("tlsr82_gpioirqconfig()");

  return OK;
}
#endif

/****************************************************************************
 * Name: tlsr82_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
void tlsr82_gpioirqenable(gpio_cfg_t cfg)
{
  uint32_t group;
  uint32_t pin;
  uint32_t irqmode;

  group   = (uint32_t)GPIO_GET(GROUP, cfg);
  pin     = (uint32_t)GPIO_GET(PIN, cfg);
  irqmode = (uint32_t)GPIO_GET(IRQ, cfg);

  /* Enable corresponding gpio interrupt */

  if (irqmode == GPIO_IRQ_NORMAL_VAL)
    {
      /* Enable corresponding pin interrupt, irq */

      BM_SET(GPIO_IRQ_NORMAL_REG(group), BIT(pin));

      /* Enable the total gpio interrupt */

      BM_SET(GPIO_IRQ_NORMAL_ALL_REG, GPIO_IRQ_NORMAL_ALL_EN);
    }
  else if (irqmode == GPIO_IRQ_M0_VAL)
    {
      /* Set M0, the timer interrupt is controled by timer */

      BM_SET(GPIO_IRQ_M0_REG(group), BIT(pin));
    }
  else if (irqmode == GPIO_IRQ_M1_VAL)
    {
      /* Set M1, the timer interrupt is controled by timer */

      BM_SET(GPIO_IRQ_M1_REG(group), BIT(pin));
    }
  else if (irqmode == GPIO_IRQ_M2_VAL)
    {
      /* Set M2, the timer interrupt is controled by timer */

      BM_SET(GPIO_IRQ_M2_REG(group), BIT(pin));
    }
  else if (irqmode == GPIO_IRQ_RISC0_VAL)
    {
      /* Set M0, enable risc0 bit, enable gpio risc0 interrupt */

      BM_SET(GPIO_IRQ_M0_REG(group), BIT(pin));
      BM_SET(GPIO_IRQ_RISC_EN_REG, GPIO_IRQ_RISC0_EN);
    }
  else if (irqmode == GPIO_IRQ_RISC1_VAL)
    {
      /* Set M1, enable risc1 bit, enable gpio risc1 interrupt */

      BM_SET(GPIO_IRQ_M1_REG(group), BIT(pin));
      BM_SET(GPIO_IRQ_RISC_EN_REG, GPIO_IRQ_RISC1_EN);
    }
  else
    {
      gpioerr("GPIO irq type is not correct, cfg=0x%lx\n", cfg);
    }
}

#endif

/****************************************************************************
 * Name: tlsr82_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
void tlsr82_gpioirqdisable(gpio_cfg_t cfg)
{
  uint32_t group;
  uint32_t pin;
  uint32_t irqmode;

  group   = (uint32_t)GPIO_GET(GROUP, cfg);
  pin     = (uint32_t)GPIO_GET(PIN, cfg);
  irqmode = (uint32_t)GPIO_GET(IRQ, cfg);

  if (irqmode == GPIO_IRQ_NORMAL_VAL)
    {
      BM_CLR(GPIO_IRQ_NORMAL_REG(group), BIT(pin));
    }
  else if (irqmode == GPIO_IRQ_M0_VAL)
    {
      BM_CLR(GPIO_IRQ_M0_REG(group), BIT(pin));
    }
  else if (irqmode == GPIO_IRQ_M1_VAL)
    {
      BM_CLR(GPIO_IRQ_M1_REG(group), BIT(pin));
    }
  else if (irqmode == GPIO_IRQ_M2_VAL)
    {
      BM_CLR(GPIO_IRQ_M2_REG(group), BIT(pin));
    }
  else if (irqmode == GPIO_IRQ_RISC0_VAL)
    {
      BM_CLR(GPIO_IRQ_M0_REG(group), BIT(pin));
      BM_CLR(GPIO_IRQ_RISC_EN_REG, GPIO_IRQ_RISC0_EN);
    }
  else if (irqmode == GPIO_IRQ_RISC1_VAL)
    {
      BM_CLR(GPIO_IRQ_M1_REG(group), BIT(pin));
      BM_CLR(GPIO_IRQ_RISC_EN_REG, GPIO_IRQ_RISC1_EN);
    }
  else
    {
      gpioerr("GPIO irq type is not correct, cfg=0x%lx\n", cfg);
    }
}

#endif

/****************************************************************************
 * Name: tlsr82_gpioirqclear
 *
 * Description:
 *   Clear the interrupt flag for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
void tlsr82_gpioirqclear(gpio_cfg_t cfg)
{
  uint32_t irqmode;

  irqmode = (uint32_t)GPIO_GET(IRQ, cfg);

  /* Clear corresponding gpio interrupt flag */

  if (irqmode == GPIO_IRQ_NORMAL_VAL)
    {
      BM_SET(IRQ_SRC_REG, BIT(NR_GPIO_IRQ));
    }
  else if (irqmode == GPIO_IRQ_RISC0_VAL)
    {
      BM_SET(IRQ_SRC_REG, BIT(NR_GPIO_RISC0_IRQ));
    }
  else if (irqmode == GPIO_IRQ_RISC1_VAL)
    {
      BM_SET(IRQ_SRC_REG, BIT(NR_GPIO_RISC1_IRQ));
    }
  else
    {
      gpioerr("GPIO irq type is not correct, cfg=0x%lx\n", cfg);
    }
}

#endif

/****************************************************************************
 * Name: tlsr82_gpioirqdisable_all
 *
 * Description:
 *   Disable all the gpio interrupt (not clear gpio pin corresponding
 *   interrupt bit.)
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
void tlsr82_gpioirqdisable_all(void)
{
  up_disable_irq(NR_GPIO_IRQ);
  up_disable_irq(NR_GPIO_RISC0_IRQ);
  up_disable_irq(NR_GPIO_RISC1_IRQ);
}
#endif

/****************************************************************************
 * Name: tlsr82_gpioirqenable_all
 *
 * Description:
 *   Enable all the gpio interrupt (not set gpio pin corresponding
 *   interrupt bit.)
 *
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_IRQ
void tlsr82_gpioirqenable_all(void)
{
  up_enable_irq(NR_GPIO_IRQ);
  up_enable_irq(NR_GPIO_RISC0_IRQ);
  up_enable_irq(NR_GPIO_RISC1_IRQ);
}
#endif
