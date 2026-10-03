/****************************************************************************
 * arch/arm/src/stm32h7/stm32_mdio_gpio.c
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
#include <nuttx/debug.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>

#include <nuttx/debug.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <inttypes.h>

#include <nuttx/net/mdio.h>
/* #include <arch/board/board.h> */

#include "arm_internal.h"
#include "chip.h"
#include "hardware/stm32_ethernet.h"
#include "stm32_gpio.h"
#include "stm32_mdio_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef CONFIG_DEBUG_STM32_MDIO_GPIO_INFO
#ifdef CONFIG_DEBUG_STM32_MDIO_GPIO_INFO
#define mg_info  _info
#else
#define mg_info  _none
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_mdiogpio_lowerhalf_s
{
  struct mdio_lowerhalf_s base;
  uint32_t mdc_out;  /* MDC pin config for output */
  uint32_t mdio_out; /* MDIO pin config for output */
  uint32_t mdio_in;  /* MDIO pin config for input */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_c22_read(struct mdio_lowerhalf_s *dev, uint8_t phydev,
                    uint8_t regaddr, uint16_t *value);

static int stm32_c22_write(struct mdio_lowerhalf_s *dev, uint8_t phydev,
                     uint8_t regaddr, uint16_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

const struct mdio_ops_s g_stm32_mdiogpio_ops =
{
  .read  = stm32_c22_read,
  .write = stm32_c22_write,
  .reset = NULL,
};

struct stm32_mdiogpio_lowerhalf_s g_stm32_mdiogpio_lowerhalf =
{
  .base =
    {
      .ops = &g_stm32_mdiogpio_ops
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#define MDIO_DELAY()           up_udelay(2) /* delay between clock edges */

#define MDC_HIGH(priv)         stm32_gpiowrite(priv->mdc_out, true)
#define MDC_LOW(priv)          stm32_gpiowrite(priv->mdc_out, false)
#define MDIO_WRITE(priv, val)  stm32_gpiowrite(priv->mdio_out, (val))
#define MDIO_READ(priv)        stm32_gpioread(priv->mdio_in)
#define MDIO_MODE_OUT(priv)    stm32_configgpio(priv->mdio_out)
#define MDIO_MODE_IN(priv)     stm32_configgpio(priv->mdio_in)

static void mdiogpio_write_bit(struct stm32_mdiogpio_lowerhalf_s *priv,
                               bool bit)
{
  MDIO_WRITE(priv, bit);
  MDIO_DELAY();
  MDC_HIGH(priv); /* rising MDC edge latches MDIO data */
  MDIO_DELAY();
  MDC_LOW(priv);
}

static bool mdiogpio_read_bit(struct stm32_mdiogpio_lowerhalf_s *priv)
{
  bool bit;

  MDC_HIGH(priv); /* PHY drives data on rising edge */
  MDIO_DELAY();
  bit = MDIO_READ(priv);
  MDC_LOW(priv);
  MDIO_DELAY();

  return bit;
}

static void mdiogpio_send_bits(struct stm32_mdiogpio_lowerhalf_s *priv,
                               uint32_t val, int count)
{
  for (int i = count - 1; i >= 0; i--)
    {
      mdiogpio_write_bit(priv, (val >> i) & 1);
    }
}

static int stm32_c22_read(struct mdio_lowerhalf_s *dev, uint8_t phydev,
                          uint8_t regaddr, uint16_t *value)
{
  struct stm32_mdiogpio_lowerhalf_s *priv =
         container_of(dev, struct stm32_mdiogpio_lowerhalf_s, base);
  uint16_t data = 0;

  if (!value)
    {
      return -EINVAL;
    }

  MDIO_MODE_OUT(priv);

  mdiogpio_send_bits(priv, 0xffffffff, 32); /* 32-bit preamble */

  mdiogpio_send_bits(priv, 0x1, 2); /* Start-of-frame */
  mdiogpio_send_bits(priv, 0x2, 2); /* OP: Read */

  mdiogpio_send_bits(priv, phydev & 0x1f, 5);  /* PHY Address */
  mdiogpio_send_bits(priv, regaddr & 0x1f, 5); /* PHY Register */

  MDIO_MODE_IN(priv); /* Turnaround (TA): Release MDIO line; switch pin to input */

  mdiogpio_read_bit(priv); /* First TA bit (expect hi-Z) */

  /* Read 16 bits of payload */

  for (int i = 15; i >= 0; i--)
    {
      if (mdiogpio_read_bit(priv))
        {
          data |= (1 << i);
        }
    }

  /* Extra clock cycle to clean up state */

  MDC_HIGH(priv);
  MDIO_DELAY();
  MDC_LOW(priv);
  MDIO_DELAY();

  *value = data;
  mg_info("PHY 0x%02X Reg 0x%02X -> 0x%04X\n", phydev, regaddr, data);
  return OK;
}

static int stm32_c22_write(struct mdio_lowerhalf_s *dev, uint8_t phydev,
                           uint8_t regaddr, uint16_t value)
{
  struct stm32_mdiogpio_lowerhalf_s *priv =
         container_of(dev, struct stm32_mdiogpio_lowerhalf_s, base);

  mg_info("PHY 0x%02X Reg 0x%02X <- 0x%04X\n", phydev, regaddr, value);

  MDIO_MODE_OUT(priv);

  mdiogpio_send_bits(priv, 0xffffffff, 32); /* 32-bit preamble */

  mdiogpio_send_bits(priv, 0x01, 2); /* Start-of-frame */
  mdiogpio_send_bits(priv, 0x01, 2); /* OP: Write */

  mdiogpio_send_bits(priv, phydev & 0x1f, 5);  /* PHY Address */
  mdiogpio_send_bits(priv, regaddr & 0x1f, 5); /* PHY Register */

  mdiogpio_send_bits(priv, 0x02, 2); /* Turnaround; driven as '10' by master */

  mdiogpio_send_bits(priv, value, 16); /* Write 16 bits of data */

  MDIO_MODE_IN(priv);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mdiogpio_bus_initialize
 *
 * Description:
 *   Initialize the MDIO bus to emulate STA via GPIO pins
 *
 * Input Parameters:
 *   mdc_out  - MDC GPIO pin config for output
 *   mdio_out - MDIO GPIO pin config for output
 *   mdio_in  - MDIO GPIO pin config for input
 *
 * Returned Value:
 *   Initialized MDIO GPIO bus structure or NULL on failure
 *
 ****************************************************************************/

struct mdio_bus_s *stm32_mdiogpio_bus_initialize(uint32_t mdc_out,
                                                 uint32_t mdio_out,
                                                 uint32_t mdio_in)
{
  struct stm32_mdiogpio_lowerhalf_s *priv = &g_stm32_mdiogpio_lowerhalf;

  mg_info("\n");

  priv->mdc_out = mdc_out;
  priv->mdio_out = mdio_out;
  priv->mdio_in = mdio_in;

  stm32_configgpio(priv->mdc_out);
  stm32_configgpio(priv->mdio_out);

  return mdio_register(&priv->base);
}
