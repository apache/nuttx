/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_cs43l22.c
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
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/audio/cs43l22.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "stm32f4discovery.h"

#ifdef HAVE_CS43L22

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_mwinfo_s
{
  /* Standard CS43L22 interface */

  struct cs43l22_lower_s lower;

  /* Extensions for the stm32f4discovery board */

  cs43l22_handler_t handler;
  void *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/PIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the CS43L22 driver from differences in PIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   attach  - Attach the CS43L22 interrupt handler to the PIO interrupt
 *   enable  - Enable or disable the PIO interrupt
 */

static int  cs43l22_attach(const struct cs43l22_lower_s *lower,
                           cs43l22_handler_t isr, void *arg);
static bool cs43l22_enable(const struct cs43l22_lower_s *lower,
                           bool enable);
static void cs43l22_hw_reset(const struct cs43l22_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the CS43L22
 * driver.  This structure provides information about the configuration
 * of the CS43L22 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

#define CONFIG_STM32_CS43L22_I2CFREQUENCY 100000
#define BOARD_MAINCK_FREQUENCY 8000000

static struct stm32_mwinfo_s g_cs43l22info =
{
  .lower =
  {
    .address   = CS43L22_I2C_ADDRESS,
    .frequency = CONFIG_STM32_CS43L22_I2CFREQUENCY,
#ifdef CONFIG_STM32_CS43L22_SRCSCK
    .mclk      = BOARD_SLOWCLK_FREQUENCY,
#else
    .mclk      = BOARD_MAINCK_FREQUENCY,
#endif
    .attach    = cs43l22_attach,
    .enable    = cs43l22_enable,
    .reset     = cs43l22_hw_reset,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * IRQ/PIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the CS43L22 driver from differences in PIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   attach  - Attach the CS43L22 interrupt handler to the PIO interrupt
 *   enable  - Enable or disable the PIO interrupt
 *   clear   - Acknowledge/clear any pending PIO interrupt
 *
 ****************************************************************************/

static int cs43l22_attach(const struct cs43l22_lower_s *lower,
                         cs43l22_handler_t isr,  void *arg)
{
  if (isr)
    {
      /* Just save the address of the handler and its argument for now.  The
       * new handler will called via cs43l22_interrupt() when the interrupt
       * occurs.
       */

      audinfo("Attaching %p\n", isr);
      g_cs43l22info.handler = isr;
      g_cs43l22info.arg = arg;
    }
  else
    {
      audinfo("Detaching %p\n", g_cs43l22info.handler);
      cs43l22_enable(lower, false);
      g_cs43l22info.handler = NULL;
      g_cs43l22info.arg = NULL;
    }

  return OK;
}

static bool cs43l22_enable(const struct cs43l22_lower_s *lower,
                           bool enable)
{
  static bool enabled;
  irqstate_t flags;
  bool ret;

  /* Has the interrupt state changed */

  flags = enter_critical_section();
  if (enable != enabled)
    {
      /* Enable or disable interrupts */

      if (enable && g_cs43l22info.handler)
        {
          audinfo("Enabling\n");

          /* TODO: stm32_pioirqenable(IRQ_INT_CS43L22); */

          enabled = true;
        }
      else
        {
          audinfo("Disabling\n");

          /* TODO: stm32_pioirqdisable(IRQ_INT_CS43L22); */

          enabled = false;
        }
    }

  ret = enabled;
  leave_critical_section(flags);
  return ret;
}

#if 0
static int cs43l22_interrupt(int irq, void *context)
{
  /* Just forward the interrupt to the CS43L22 driver */

  audinfo("handler %p\n", g_cs43l22info.handler);
  if (g_cs43l22info.handler)
    {
      return g_cs43l22info.handler(&g_cs43l22info.lower, g_cs43l22info.arg);
    }

  /* We got an interrupt with no handler.  This should not
   * happen.
   */

  /* TODO: stm32_pioirqdisable(IRQ_INT_CS43L22); */

  return OK;
}
#endif

static void cs43l22_hw_reset(const struct cs43l22_lower_s *lower)
{
  int i;

  /* Reset the codec */

  stm32_gpiowrite(GPIO_CS43L22_RESET, false);
  for (i = 0; i < 0x4fff; i++)
    {
      __asm__ volatile("nop");
    }

  stm32_gpiowrite(GPIO_CS43L22_RESET, true);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_cs43l22_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the CS43L22 device.  This function will register the driver
 *   as /dev/audio/pcm[x] where x is determined by the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_cs43l22_initialize(int minor)
{
  struct audio_lowerhalf_s *cs43l22;
  struct audio_lowerhalf_s *pcm;
  struct i2c_master_s *i2c;
  struct i2s_dev_s *i2s;
  static bool initialized = false;
  char devname[12];
  int ret;

  audinfo("minor %d\n", minor);
  DEBUGASSERT(minor >= 0 && minor <= 25);

  /* Have we already initialized?  Since we never uninitialize we must
   * prevent multiple initializations.  This is necessary, for example,
   * when the touchscreen example is used as a built-in application in
   * NSH and can be called numerous time.  It will attempt to initialize
   * each time.
   */

  if (!initialized)
    {
      stm32_configgpio(GPIO_CS43L22_RESET);

      /* Configure the CS43L22 interrupt pin */

      /* TODO: stm32_configgpio(PIO_INT_CS43L22); */

      /* Get an instance of the I2C interface for the CS43L22 chip select */

      i2c = stm32_i2cbus_initialize(CS43L22_I2C_BUS);
      if (!i2c)
        {
          auderr("ERROR: Failed to initialize TWI%d\n", CS43L22_I2C_BUS);
          ret = -ENODEV;
          goto errout;
        }

      /* Get an instance of the I2S interface for the CS43L22 data channel */

      i2s = stm32_i2sbus_initialize(CS43L22_I2S_BUS);
      if (!i2s)
        {
          auderr("ERROR: Failed to initialize I2S%d\n", CS43L22_I2S_BUS);
          ret = -ENODEV;
          goto errout_with_i2c;
        }

      /* Configure the DAC master clock.  This clock is provided by
       * PCK2 (PB10) that is connected to the CS43L22 MCLK.
       */

      /* Configure CS43L22 interrupts */

#if 0  /* TODO: */
      stm32_pioirq(PIO_INT_CS43L22);
      ret = irq_attach(IRQ_INT_CS43L22, cs43l22_interrupt);
      if (ret < 0)
        {
          auderr("ERROR: Failed to attach CS43L22 interrupt: %d\n", ret);
          goto errout_with_i2s;
        }
#endif

      /* Now we can use these I2C and I2S interfaces to initialize the
       * CS43L22 which will return an audio interface.
       */

      cs43l22 = cs43l22_initialize(i2c, i2s, &g_cs43l22info.lower);
      if (!cs43l22)
        {
          auderr("ERROR: Failed to initialize the CS43L22\n");
          ret = -ENODEV;
          goto errout_with_irq;
        }

      /* No we can embed the CS43L22/I2C/I2S conglomerate into a PCM decoder
       * instance so that we will have a PCM front end for the the CS43L22
       * driver.
       */

      pcm = pcm_decode_initialize(cs43l22);
      if (!pcm)
        {
          auderr("ERROR: Failed create the PCM decoder\n");
          ret = -ENODEV;
          goto errout_with_cs43l22;
        }

      /* Create a device name */

      snprintf(devname, 12, "pcm%d",  minor);

      /* Finally, we can register the PCM/CS43L22/I2C/I2S audio device.
       *
       * Is anyone young enough to remember Rube Goldberg?
       */

      ret = audio_register(devname, pcm);
      if (ret < 0)
        {
          auderr("ERROR: Failed to register /dev/%s device: %d\n",
                 devname, ret);
          goto errout_with_pcm;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;

  /* Error exits.  Unfortunately there is no mechanism in place now to
   * recover resources from most errors on initialization failures.
   */

errout_with_pcm:
errout_with_cs43l22:
errout_with_irq:

#if 0
  irq_detach(IRQ_INT_CS43L22);
errout_with_i2s:
#endif

errout_with_i2c:
errout:
  return ret;
}

#endif /* HAVE_CS43L22 */
