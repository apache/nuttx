/****************************************************************************
 * arch/risc-v/src/bl808/bl808_gpadc.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/arch.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "hardware/bl808_gpadc.h"
#include "riscv_internal.h"
#include "chip.h"
#include "bl808_gpadc.h"
#include "bl808_gpio.h"

#ifdef CONFIG_BL808_GPADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL808_GPADC_TOTAL_NCHANNELS 18
#define BL808_GPADC_SCAN_MAX_CHANNELS 12

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum bl808_gpadc_channel_e
  {
    GPADC_CH0,
    GPADC_CH1,
    GPADC_CH2,
    GPADC_CH3,
    GPADC_CH4,
    GPADC_CH5,
    GPADC_CH6,
    GPADC_CH7,
    GPADC_CH8,
    GPADC_CH9,
    GPADC_CH10,
    GPADC_CH11,
    GPADC_CH_DAC_OUTA,
    GPADC_CH_DAC_OUTB,
    GPADC_CH_TSEN,
    GPADC_CH_VREF = 16,
    GPADC_CH_HALF_VBAT = 18,
    GPADC_CH_GND = 23
  };

/* Values for resolution enum correspond to the register values
 * for each option.
 */

enum bl808_gpadc_resolution_e
  {
    GPADC_12_BIT = 0,
    GPADC_14_BIT = 1,
    GPADC_16_BIT = 3
  };

struct bl808_gpadc_s
{
  const struct adc_callback_s *callback;
  enum bl808_gpadc_channel_e enabled_channels[BL808_GPADC_SCAN_MAX_CHANNELS];
  uint8_t nchannels;
  enum bl808_gpadc_resolution_e resolution;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

uint8_t bl808_gpadc_get_count(void);

/* Character driver methods */

static int bl808_gpadc_bind(struct adc_dev_s *dev,
                            const struct adc_callback_s *callback);
static void bl808_gpadc_reset(struct adc_dev_s *dev);
static int bl808_gpadc_setup(struct adc_dev_s *dev);
static void bl808_gpadc_shutdown(struct adc_dev_s *dev);
static void bl808_gpadc_rxint(struct adc_dev_s *dev,
                              bool enable);
static int bl808_gpadc_ioctl(struct adc_dev_s *dev,
                             int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bl808_gpadc_s gpadc_priv =
  {
    .callback = NULL,

    .enabled_channels =
    {
      CONFIG_BL808_GPADC_SCAN_ORD0,
      CONFIG_BL808_GPADC_SCAN_ORD1,
      CONFIG_BL808_GPADC_SCAN_ORD2,
      CONFIG_BL808_GPADC_SCAN_ORD3,
      CONFIG_BL808_GPADC_SCAN_ORD4,
      CONFIG_BL808_GPADC_SCAN_ORD5,
      CONFIG_BL808_GPADC_SCAN_ORD6,
      CONFIG_BL808_GPADC_SCAN_ORD7,
      CONFIG_BL808_GPADC_SCAN_ORD8,
      CONFIG_BL808_GPADC_SCAN_ORD9,
      CONFIG_BL808_GPADC_SCAN_ORD10,
      CONFIG_BL808_GPADC_SCAN_ORD11
    },

    .nchannels = CONFIG_BL808_GPADC_NCHANNELS,

#ifdef CONFIG_BL808_GPADC_RES_12
    .resolution = GPADC_12_BIT,
#elif defined(CONFIG_BL808_GPADC_RES_14)
    .resolution = GPADC_14_BIT,
#elif defined(CONFIG_BL808_GPADC_RES_16)
    .resolution = GPADC_16_BIT,
#endif
  };

static struct adc_ops_s gpadc_ops =
  {
    .ao_bind = bl808_gpadc_bind,
    .ao_reset = bl808_gpadc_reset,
    .ao_setup = bl808_gpadc_setup,
    .ao_shutdown = bl808_gpadc_shutdown,
    .ao_rxint = bl808_gpadc_rxint,
    .ao_ioctl = bl808_gpadc_ioctl
  };

static uint8_t ch_to_gpio_map[12] =
  {
    [0] = 17,
    [1] = 5,
    [2] = 4,
    [3] = 11,
    [4] = 6,
    [5] = 40,
    [6] = 12,
    [7] = 13,
    [8] = 16,
    [9] = 18,
    [10] = 19,
    [11] = 34
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl808_gpadc_get_count
 *
 * Description:
 *   Gets the number of items available in the GPADC FIFO.
 *
 ****************************************************************************/

uint8_t bl808_gpadc_get_count()
{
  uint32_t status = getreg32(BL808_GPADC_CONFIG);
  uint8_t count = (status & GPADC_FIFO_DATA_COUNT_MASK)
    >> GPADC_FIFO_DATA_COUNT_SHIFT;
  return count;
}

/****************************************************************************
 * Name: gpadc_interrupt
 *
 * Description:
 *   GPADC interrupt handler. Passes ADC readings to upper half driver,
 *   then clears the FIFO.
 *
 ****************************************************************************/

static int __gpadc_interrupt(int irq, void *context, void *arg)
{
  struct adc_dev_s *dev = (struct adc_dev_s *)arg;
  struct bl808_gpadc_s *priv = dev->ad_priv;
  uint32_t status = getreg32(BL808_GPADC_CONFIG);

  if (status & GPADC_RDY)
    {
      if ((priv->callback != NULL)
          && (priv->callback->au_receive != NULL))
        {
          uint8_t count = bl808_gpadc_get_count();

          while (count != 0)
            {
              uint32_t result = getreg32(BL808_GPADC_DMA_RDATA);
              uint32_t channel = (result & GPADC_RESULT_POS_CHN_MASK)
                >> GPADC_RESULT_POS_CHN_SHIFT;
              uint32_t adc_val = result & GPADC_RESULT_RAW_VAL_MASK;

              int receive_ret =
                priv->callback->au_receive(dev, channel, adc_val);
              if (receive_ret)
                {
                  aerr("ADC driver upper half receive error");
                  return -EIO;
                }

              count = bl808_gpadc_get_count();
            }

          modifyreg32(BL808_GPADC_CONFIG, 0,
                      GPADC_FIFO_CLR);

          modifyreg32(BL808_GPADC_CONFIG, 0,
                      GPADC_RDY_CLR);

          modifyreg32(BL808_GPADC_CONFIG,
                      GPADC_RDY_CLR, 0);

          return OK;
        }
    }

  /* If we get here, there was an error */

  return -EIO;
}

/****************************************************************************
 * Name: bl808_gpadc_bind
 *
 * Description:
 *   Called when the driver is registered. Binds upper half callbacks
 *   to the private data structure.
 *
 ****************************************************************************/

static int bl808_gpadc_bind(struct adc_dev_s *dev,
                            const struct adc_callback_s *callback)
{
  ((struct bl808_gpadc_s *)(dev->ad_priv))->callback = callback;

  return OK;
}

/****************************************************************************
 * Name: bl808_gpadc_reset
 *
 * Description:
 *   Called as part of the device registration process. Resets the
 *   GPADC hardware block.
 *
 ****************************************************************************/

static void bl808_gpadc_reset(struct adc_dev_s *dev)
{
  modifyreg32(BL808_GPADC_CMD, 0, GPADC_SOFT_RST);
  modifyreg32(BL808_GPADC_CMD, GPADC_SOFT_RST, 0);
}

/****************************************************************************
 * Name: bl808_gpadc_setup
 *
 * Description:
 *   Called when the driver is first opened. Sets registers to allow
 *   scanning functionality, and sets the scan order registers.
 *
 ****************************************************************************/

static int bl808_gpadc_setup(struct adc_dev_s *dev)
{
  struct bl808_gpadc_s *priv = dev->ad_priv;

  /* This setup process is mostly taken from bouffalo_sdk */

  modifyreg32(BL808_GPADC_CMD, GPADC_GLOBAL_EN, 0);

  /* Soft reset */

  modifyreg32(BL808_GPADC_CMD, 0, GPADC_SOFT_RST);

  modifyreg32(BL808_GPADC_CMD, GPADC_SOFT_RST, 0);

  modifyreg32(BL808_GPADC_CONFIG1, GPADC_CONT_CONV_EN,
              (2 << GPADC_V18_SEL_SHIFT)
              | (1 << GPADC_V11_SEL_SHIFT)
              | GPADC_SCAN_EN
              | GPADC_CLK_ANA_INV
              | (priv->resolution << GPADC_RES_SEL_SHIFT));

  modifyreg32(BL808_GPADC_CONFIG2, 0,
              (2 << GPADC_DLY_SEL_SHIFT)
              | GPADC_VBAT_EN
              | GPADC_TS_EN);

  /* Use GND as negative channel for now */

  modifyreg32(BL808_GPADC_CMD, 0, GPADC_NEG_GND);

  /* Clear all interrupts and masked unused ones */

  modifyreg32(BL808_GPADC_CONFIG, 0,
              GPADC_RDY_CLR
              | GPADC_FIFO_OVERRUN_CLR
              | GPADC_FIFO_UNDERRUN_CLR
              | GPADC_FIFO_OVERRUN_MASK
              | GPADC_FIFO_UNDERRUN_MASK);

  modifyreg32(BL808_GPADC_ISR, 0,
              GPADC_NEG_SATUR_CLR
              | GPADC_POS_SATUR_CLR
              | GPADC_NEG_SATUR_MASK
              | GPADC_POS_SATUR_MASK);

  modifyreg32(BL808_GPADC_CONFIG,
              GPADC_RDY_CLR, 0);

  /* Set scan channels */

  modifyreg32(BL808_GPADC_SCAN_POS1,
              0xffffffff, 0);
  modifyreg32(BL808_GPADC_SCAN_POS2,
              0xffffffff, 0);

  for (int channel_idx = 0;
       channel_idx < priv->nchannels;
       channel_idx++)
    {
      uint8_t channel = priv->enabled_channels[channel_idx];

      /* If enabling an external channel, configure GPIO */

      if (channel < 12)
        {
          int pin = ch_to_gpio_map[channel];
          bl808_configgpio(pin, GPIO_INPUT
                           | GPIO_FLOAT
                           | GPIO_FUNC_ANA);
        }

      if (channel_idx < 6)
        {
          modifyreg32(BL808_GPADC_SCAN_POS1, 0,
                      (channel << GPADC_SCAN_SHIFT(channel_idx)));
        }
      else
        {
          modifyreg32(BL808_GPADC_SCAN_POS2, 0,
                      (channel << GPADC_SCAN_SHIFT(channel_idx)));
        }
    }

  modifyreg32(BL808_GPADC_CONFIG1, 0,
              ((priv->nchannels - 1) << GPADC_SCAN_LENGTH_SHIFT));

  modifyreg32(BL808_GPADC_CONFIG, 0,
              GPADC_FIFO_CLR);

  modifyreg32(BL808_GPADC_CMD, 0, GPADC_GLOBAL_EN);

  return OK;
}

/****************************************************************************
 * Name: bl808_gpadc_shutdown
 *
 * Description:
 *   Disables the GPADC hardware block.
 *
 ****************************************************************************/

static void bl808_gpadc_shutdown(struct adc_dev_s *dev)
{
  modifyreg32(BL808_GPADC_CMD, GPADC_GLOBAL_EN, 0);
}

/****************************************************************************
 * Name: bl808_gpadc_rxint
 *
 * Description:
 *   Enables or disables the conversion finished interrupt.
 *
 ****************************************************************************/

static void bl808_gpadc_rxint(struct adc_dev_s *dev,
                              bool enable)
{
  if (enable)
    {
      irq_attach(BL808_IRQ_GPADC, __gpadc_interrupt, (void *)dev);
      up_enable_irq(BL808_IRQ_GPADC);

      modifyreg32(BL808_GPADC_CMD, 0, GPADC_CONV_START);
    }
  else
    {
      up_disable_irq(BL808_IRQ_GPADC);
      irq_detach(BL808_IRQ_GPADC);

      modifyreg32(BL808_GPADC_CMD, GPADC_CONV_START, 0);
    }
}

/****************************************************************************
 * Name: bl808_gpadc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 ****************************************************************************/

static int bl808_gpadc_ioctl(struct adc_dev_s *dev,
                             int cmd, unsigned long arg)
{
  int ret;
  struct bl808_gpadc_s *priv = (struct bl808_gpadc_s *)dev->ad_priv;

  switch (cmd)
    {
    case ANIOC_TRIGGER:
      {
        modifyreg32(BL808_GPADC_CMD, 0, GPADC_CONV_START);
        ret = OK;
        break;
      }

    case ANIOC_GET_NCHANNELS:
      {
        ret = priv->nchannels;
        break;
      }

    case ANIOC_RESET_FIFO:
      {
        modifyreg32(BL808_GPADC_CONFIG, 0,
                    GPADC_FIFO_CLR);
        ret = OK;
        break;
      }

    case ANIOC_SAMPLES_ON_READ:
      {
        ret = bl808_gpadc_get_count();
        break;
      }

    default:
      {
        /* Other commands not implemented */

        ret = -ENOTTY;
        break;
      }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bl808_gpadc_init(void)
{
  struct adc_dev_s *dev = kmm_zalloc(sizeof(struct adc_dev_s));
  dev->ad_ops  = &gpadc_ops;
  dev->ad_priv = &gpadc_priv;

  int ret = adc_register("/dev/gpadc", dev);

  return ret;
}

#endif /* CONFIG_BL808_GPADC */
