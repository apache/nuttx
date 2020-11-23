/****************************************************************************
 * boards/arm/stm32f7/stm32f746g-disco/src/stm32_wm8994.c
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
#include <nuttx/audio/wm8994.h>

#include <arch/board/board.h>

#include "stm32f746g-disco.h"
#include "stm32_i2c.h"
#include "stm32_sai.h"

#define HAVE_WM8994
#define WM8994_I2C_ADDRESS (0x34 >> 1)
#define WM8994_I2C_BUS  3
#define WM8994_SAI_BUS  2

#ifdef HAVE_WM8994

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_mwinfo_s
{
  /* Standard ADAU1961 interface */

  struct wm8994_lower_s lower;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int wm8994_attach(FAR const struct wm8994_lower_s *lower,
                           wm8994_handler_t isr, FAR void *arg)
{
  audinfo("TODO\n");
  return 0;
}

static bool wm8994_enable(FAR const struct wm8994_lower_s *lower,
                            bool enable)
{
  audinfo("TODO\n");
  return 0;
}

#if 0
static void wm8994_hw_reset(FAR const struct wm8994_lower_s *lower)
{
  audinfo("TODO\n");
}
#endif

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

static struct stm32_mwinfo_s g_wm8994 =
{
  .lower =
    {
    .address   = WM8994_I2C_ADDRESS,
    .frequency = I2C_SPEED_FAST,        /* 100 kHz */
    .mclk      = 12000000,              /* see MCO1 configuration */
    .attach    = wm8994_attach,
    .enable    = wm8994_enable,
    }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_wm8994_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the ADAU1961 device. This function will register the driver
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

int stm32_wm8994_initialize(int minor)
{
  FAR struct audio_lowerhalf_s *wm8994;
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

      /* stm32_configgpio(GPIO_MCO1);
       * stm32_mco1config(RCC_CFGR_MCO1_HSE, RCC_CFGR_MCO1PRE_NONE);
       */

      /* Get an instance of the I2C interface for the CODEC */

      i2c = stm32_i2cbus_initialize(WM8994_I2C_BUS);
      if (!i2c)
        {
          auderr("stm32_i2cbus_initialize failed\n");
          ret = -ENODEV;
          goto error;
        }

      /* Get an instance of the I2S interface for the CODEC data streams */

      i2s = stm32_sai_initialize(WM8994_SAI_BUS);
      if (!i2s)
        {
          auderr("stm32_sai_initialize failed\n");
          ret = -ENODEV;
          goto error;
        }

      /* Now we can use these I2C and I2S interfaces to initialize the
       * CODEC which will return an audio interface.
       */

      wm8994 = wm8994_initialize(i2c, i2s, &g_wm8994.lower);
      if (!wm8994)
        {
          auderr("wm8994_initialize failed\n");
          ret = -ENODEV;
          goto error;
        }

      /* Create a device name */

      snprintf(devname, 12, "pcm%d", minor);
#if 0
      /* Finally, we can register the ADAU1961/I2C/I2S audio device. */

      ret = audio_register(devname, wm8994);
      if (ret < 0)
        {
          auderr("failed to register /dev/%s device: %d\n", devname, ret);
          goto error;
        }

#endif

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

#endif /* HAVE_WM8994 */
