/****************************************************************************
 * arch/arm/src/stm32h7/stm32_mdio.c
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

#include <arm_internal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_STM32H7_ETHMAC_REGDEBUG
static uint32_t stm32_getreg(uint32_t addr);
static void stm32_putreg(uint32_t val, uint32_t addr);
static void stm32_checksetup(void);
#else
#  define stm32_getreg(addr)     getreg32(addr)
#  define stm32_putreg(val,addr) putreg32(val,addr)
#  define stm32_checksetup()
#endif

/* PHY reset/configuration delays in milliseconds */

#define PHY_RESET_DELAY   (65)
#define PHY_CONFIG_DELAY  (1000)

/* PHY read/write delays in loop counts */

#define PHY_READ_TIMEOUT  (0x0004ffff)
#define PHY_WRITE_TIMEOUT (0x0004ffff)
#define PHY_RETRY_TIMEOUT (0x0001998)

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/mutex.h>
#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <inttypes.h>

#include "stm32_mdio.h"
#include "hardware/stm32_ethernet.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_mdio_bus_s
{
  int timeout;
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

struct stm32_mdio_bus_s g_stm32_mdio_priv =
{
  .timeout = 10
};

struct mdio_lowerhalf_s g_stm32_mdio_lowerhalf =
{
  .ops =
    {
      .read  = stm32_c22_read,
      .write = stm32_c22_write,
      .reset = NULL,
    },
  .priv = &g_stm32_mdio_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_c22_read(struct mdio_lowerhalf_s *dev, uint8_t phydev,
                          uint8_t regaddr, uint16_t *value)
{
  int to;
  uint32_t regval;

  int retval = -ETIMEDOUT;
  struct stm32_mdio_bus_s *priv = (struct stm32_mdio_bus_s *)dev->priv;

  /* Configure the MACMDIOAR register, preserving CSR Clock Range CR[3:0]
   * bits
   */

  regval  = stm32_getreg(STM32_ETH_MACMDIOAR);
  regval &= ETH_MACMDIOAR_CR_MASK;

  /* Set the PHY device address, PHY register address, and set the buy bit.
   * the ETH_MACMDIOAR_GOC == 3, indicating a read operation.
   */

  regval |= (((uint32_t)phydev << ETH_MACMDIOAR_PA_SHIFT) &
            ETH_MACMDIOAR_PA_MASK);
  regval |= (((uint32_t)regaddr << ETH_MACMDIOAR_RDA_SHIFT) &
            ETH_MACMDIOAR_RDA_MASK);
  regval |= ETH_MACMDIOAR_MB | ETH_MACMDIOAR_GOC_READ;

  stm32_putreg(regval, STM32_ETH_MACMDIOAR);

  /* Wait for the transfer to complete */

  for (to = priv->timeout; to >= 0; to--)
    {
      if ((stm32_getreg(STM32_ETH_MACMDIOAR) & ETH_MACMDIOAR_MB) == 0)
        {
          *value = (uint16_t)stm32_getreg(STM32_ETH_MACMDIODR);
          retval = OK;
          break;
        }

      up_mdelay(5);
    }

  if (to <= 0)
    {
      ninfo("MII transfer timed out: phydev: %04x regaddr: %04x\n",
            phydev, regaddr);
    }

  return retval;
}

static int stm32_c22_write(struct mdio_lowerhalf_s *dev, uint8_t phydev,
                           uint8_t regaddr, uint16_t value)
{
  int to;
  uint32_t regval;

  int retval = -ETIMEDOUT;
  struct stm32_mdio_bus_s *priv = (struct stm32_mdio_bus_s *)dev->priv;

  /* Configure the MACMDIOAR register, preserving CSR Clock Range CR[3:0]
   * bits
   */

  regval  = stm32_getreg(STM32_ETH_MACMDIOAR);
  regval &= ETH_MACMDIOAR_CR_MASK;

  /* Read the existing register value, if clear mask is given */

  /* Set the PHY device address, PHY register address, and set the busy bit.
   * the ETH_MACMDIOAR_GOC == 1, indicating a write operation.
   */

  regval |= (((uint32_t)phydev << ETH_MACMDIOAR_PA_SHIFT) &
            ETH_MACMDIOAR_PA_MASK);
  regval |= (((uint32_t)phydev << ETH_MACMDIOAR_RDA_SHIFT) &
            ETH_MACMDIOAR_RDA_MASK);
  regval |= (ETH_MACMDIOAR_MB | ETH_MACMDIOAR_GOC_WRITE);

  /* Write the value into the MACMDIODR register before setting the new
   * MACMDIOAR register value.
   */

  stm32_putreg(value, STM32_ETH_MACMDIODR);
  stm32_putreg(regval, STM32_ETH_MACMDIOAR);

  /* Wait for the transfer to complete */

  for (to = priv->timeout; to >= 0; to--)
    {
      if ((stm32_getreg(STM32_ETH_MACMDIOAR) & ETH_MACMDIOAR_MB) == 0)
        {
          retval = OK;
          break;
        }

      up_mdelay(5);
    }

  if (to <= 0)
    {
      ninfo("MII transfer timed out: phydevaddr: %04x phyregaddr: %04x"
            "value: %04x\n", phydev, regaddr, value);
    }

  return retval;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mdio_bus_initialize
 *
 * Description:
 *   Initialize the MDIO bus
 *
 * Returned Value:
 *   Initialized MDIO bus structure or NULL on failure
 *
 ****************************************************************************/

struct mdio_bus_s *stm32_mdio_bus_initialize(void)
{
  return mdio_register(&g_stm32_mdio_lowerhalf);
}
