/****************************************************************************
 * boards/arm/stm32f7/stm32f746g-disco/src/stm32_wm8994.c
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
#include <nuttx/audio/wm8994.h>

#include <arch/board/board.h>

#include "chip.h"

#include "stm32f746g-disco.h"
#include "stm32_i2c.h"
#include "stm32_sai.h"
#include "stm32_pwr.h"
#include "stm32_rcc.h"

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

static int wm8994_attach(const struct wm8994_lower_s *lower,
                         wm8994_handler_t isr, void *arg)
{
  audinfo("TODO\n");
  return 0;
}

static bool wm8994_enable(const struct wm8994_lower_s *lower,
                          bool enable)
{
  audinfo("TODO\n");
  return 0;
}

#if 0
static void wm8994_hw_reset(const struct wm8994_lower_s *lower)
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

static uint32_t stm32_wm8994_sampleratecb(struct i2s_dev_s *dev,
                                          uint32_t rate)
{
  uint32_t frequency = 0;
  uint32_t regval = 0;
  uint32_t mask_divq = 0;
  uint32_t mask_i2s = 0;

  /* Table 210 in "STM32F75xxx and STM32F74xxx advanced Arm(r)-based 32-bit
   * MCUs - Reference manual" suggests the use of specific sai_x_ker_ck
   * frequencies for common audio sample frequencies:
   * For 44.1, 22.05, 11.025 KHz, 44.1 kHz * 256 = 11289600 Hz
   * For 192, 96, 48, 32, 16, 8 KHz, 192 kHz * 256 = 49152000 Hz
   *
   * The below configurations use 429000000 / 19 / 2 = 1128473.x
   * as approximation for the first group of sample frequencies and
   * 344000000 / 1 / 7 = 49142857.x as approximation for the second
   * group of sample frequencies
   */

  if ((rate == 44100) || (rate == 22050) || (rate == 11025))
    {
      /* Division factor has 1 offset! (i.e. 0 = /1, 1 = /2, etc. */

      mask_divq = RCC_DCKCFGR1_PLLI2SDIVQ(18);
      mask_i2s = RCC_PLLI2SCFGR_PLLI2SN(429) |
                 RCC_PLLI2SCFGR_PLLI2SQ(2);
      frequency = 11289473;
    }
  else
    {
      /* Division factor has 1 offset! (i.e. 0 = /1, 1 = /2, etc. */

      mask_divq = RCC_DCKCFGR1_PLLI2SDIVQ(0);
      mask_i2s = RCC_PLLI2SCFGR_PLLI2SN(344) |
                 RCC_PLLI2SCFGR_PLLI2SQ(7);
      frequency = 49142857;
    }

  /* Check if i2s PLL is already in correct configuration */

  if ((getreg32(STM32_RCC_DCKCFGR1) &
       (RCC_DCKCFGR1_PLLI2SDIVQ_MASK)) != mask_divq)
    {
      /* Disable the PLLI2S */

      regval = getreg32(STM32_RCC_CR);
      regval &= ~(RCC_CR_PLLI2SON);
      putreg32 (regval, STM32_RCC_CR);

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLI2SON))
        {
        }

      /* Set PLLI2S Configuration */

      regval  = getreg32(STM32_RCC_DCKCFGR1);
      regval &= ~(RCC_DCKCFGR1_PLLI2SDIVQ_MASK);
      regval |= mask_divq;
      putreg32(regval, STM32_RCC_DCKCFGR1);

      regval = getreg32(STM32_RCC_PLLI2SCFGR);
      regval &= ~(RCC_PLLI2SCFGR_PLLI2SN_MASK
                | RCC_PLLI2SCFGR_PLLI2SQ_MASK);
      regval |= mask_i2s;
      putreg32(regval, STM32_RCC_PLLI2SCFGR);

      /* Enable the PLLI2S */

      regval = getreg32(STM32_RCC_CR);
      regval |= (RCC_CR_PLLI2SON);
      putreg32 (regval, STM32_RCC_CR);

      while (!(getreg32(STM32_RCC_CR) & RCC_CR_PLLI2SRDY))
        {
        }
    }

  audinfo("i2s PLL configured for samplerate %lu\n", rate);
  return frequency;
}

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
  struct audio_lowerhalf_s *wm8994;
  struct audio_lowerhalf_s *pcm;
  struct i2c_master_s *i2c;
  struct i2s_dev_s *i2s;
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

      i2s = stm32_sai_initialize(WM8994_SAI_BUS, stm32_wm8994_sampleratecb);
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

      /* No we can embed the WM8994/I2C/I2S conglomerate into a PCM decoder
       * instance so that we will have a PCM front end for the the WM8994
       * driver.
       */

      pcm = pcm_decode_initialize(wm8994);
      if (pcm == NULL)
        {
          auderr("ERROR: Failed create the PCM decoder\n");
          ret = -ENODEV;
          goto error;
        }

      /* Create a device name */

      snprintf(devname, 12, "pcm%d", minor);

      /* Finally, we can register the ADAU1961/I2C/I2S audio device. */

      ret = audio_register(devname, pcm);
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

#endif /* HAVE_WM8994 */
