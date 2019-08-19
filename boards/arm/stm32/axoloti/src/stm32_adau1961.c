/****************************************************************************
 * boards/arm/stm32/axoloti/src/stm32_adau1961.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Jason T. Harris <sirmanlypowers@gmail.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/adau1961.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "axoloti.h"

#ifdef HAVE_ADAU1961

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_mwinfo_s
{
  /* Standard ADAU1961 interface */

  struct adau1961_lower_s lower;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int adau1961_attach(FAR const struct adau1961_lower_s *lower,
                           adau1961_handler_t isr, FAR void *arg)
{
  audinfo("TODO\n");
  return 0;
}

static bool adau1961_enable(FAR const struct adau1961_lower_s *lower,
                            bool enable)
{
  audinfo("TODO\n");
  return 0;
}

static void adau1961_hw_reset(FAR const struct adau1961_lower_s *lower)
{
  audinfo("TODO\n");
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the ADAU1961
 * driver.  This structure provides information about the configuration
 * of the ADAU1961 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

static struct stm32_mwinfo_s g_adau1961info =
{
  .lower =
  {
    .address   = ADAU1961_I2C_ADDRESS,
    .frequency = I2C_SPEED_FAST,        /* 400 kHz */
    .mclk      = STM32_HSE_FREQUENCY,   /* see MCO1 configuration */
    .attach    = adau1961_attach,
    .enable    = adau1961_enable,
    .reset     = adau1961_hw_reset,
  }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adau1961_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the ADAU1961 device.  This function will register the driver
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

int stm32_adau1961_initialize(int minor)
{
  FAR struct audio_lowerhalf_s *adau1961;
  FAR struct i2c_master_s *i2c;
  FAR struct i2s_dev_s *i2s;
  static bool initialized = false;
  char devname[12];
  int ret;

  audinfo("minor %d\n", minor);
  DEBUGASSERT(minor >= 0 && minor <= 25);

  /* Initialize the CODEC if we have not already done so */

  if (!initialized)
    {
      /* Configure MC01 to drive the master clock of the CODEC at 8MHz */

      stm32_configgpio(GPIO_MCO1);
      stm32_mco1config(RCC_CFGR_MCO1_HSE, RCC_CFGR_MCO1PRE_NONE);

      /* Get an instance of the I2C interface for the CODEC */

      i2c = stm32_i2cbus_initialize(ADAU1961_I2C_BUS);
      if (!i2c)
        {
          auderr("stm32_i2cbus_initialize failed\n");
          ret = -ENODEV;
          goto error;
        }

      /* Get an instance of the I2S interface for the CODEC data streams */

      i2s = stm32_sai_initialize(ADAU1961_SAI_BUS);
      if (!i2s)
        {
          auderr("stm32_sai_initialize failed\n");
          ret = -ENODEV;
          goto error;
        }

      /* Now we can use these I2C and I2S interfaces to initialize the
       * CODEC which will return an audio interface.
       */

      adau1961 = adau1961_initialize(i2c, i2s, &g_adau1961info.lower);
      if (!adau1961)
        {
          auderr("adau1961_initialize failed\n");
          ret = -ENODEV;
          goto error;
        }

      /* Create a device name */

      snprintf(devname, 12, "pcm%d", minor);

      /* Finally, we can register the ADAU1961/I2C/I2S audio device. */

      ret = audio_register(devname, adau1961);
      if (ret < 0)
        {
          auderr("failed to register /dev/%s device: %d\n", devname, ret);
          goto error;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;

  /* Error exits. Unfortunately there is no mechanism in place now to
   * recover resources from most errors on initialization failures.
   */

error:
  return ret;
}

#endif /* HAVE_ADAU1961 */
