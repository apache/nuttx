/****************************************************************************
 * arch/arm/src/am67/am67_pinmux.c
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
 *    http://www.apache.org/licenses/LICENSE-2.0
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
#include <assert.h>
#include <stdint.h>
#include <sys/types.h>

#include "am67_pinmux.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

static struct pinmux_conf_s g_am67_pinmux_conf[] =
{
  /* UART1_RXD -> MCASP0_AFSR (C27) */

  {
    PIN_MCASP0_AFSR,
    (PIN_MODE(2) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE)
  },

  /* UART1_TXD -> MCASP0_ACLKR (F24) */

  {
    PIN_MCASP0_ACLKR,
    (PIN_MODE(2) | PIN_PULL_DISABLE)
  },

  /* RED LED -> OLDI0_A0N (AF23) */

  {
    PIN_OLDI0_A0N,
    (PIN_MODE(7) | PIN_PULL_DISABLE)
  },

  /* GREEN LED -> OLDI0_A0P (AG24) */

  {
    PIN_OLDI0_A0P,
    (PIN_MODE(7) | PIN_PULL_DISABLE)
  },
  {PINMUX_END, PINMUX_END}
};

static struct pinmux_conf_s g_am67_epwm0_pinmux_conf[] =
{
  /* EPWM0 A output pin (B20, mode 2 = EHRPWM0_A) */

  {
    PIN_SPI0_CS0,
    (PIN_MODE(2) | PIN_PULL_DISABLE)
  },

  /* EPWM0 B output pin (C20, mode 2 = EHRPWM0_B) */

  {
    PIN_SPI0_CS1,
    (PIN_MODE(2) | PIN_PULL_DISABLE)
  },
  {PINMUX_END, PINMUX_END}
};

static struct pinmux_conf_s g_am67_epwm1_pinmux_conf[] =
{
  /* EPWM1 A output pin (D20, mode 2 = EHRPWM1_A) */

  {
    PIN_SPI0_CLK,
    (PIN_MODE(2) | PIN_PULL_DISABLE)
  },

  /* EPWM1 B output pin (E19, mode 2 = EHRPWM1_B) */

  {
    PIN_SPI0_D0,
    (PIN_MODE(2) | PIN_PULL_DISABLE)
  },
  {PINMUX_END, PINMUX_END}
};

/* eCAP0 pad unassigned: C20 stays with EPWM0_B (keeps the console free). */

static struct pinmux_conf_s g_am67_ecap0_pinmux_conf[] =
{
  {PINMUX_END, PINMUX_END}
};

static struct pinmux_conf_s g_am67_ecap1_pinmux_conf[] =
{
  /* eCAP1 APWM out: MCASP0_AXR3 mode 5 */

  {
    PIN_MCASP0_AXR3,
    (PIN_MODE(5) | PIN_PULL_DISABLE)
  },
  {PINMUX_END, PINMUX_END}
};

static struct pinmux_conf_s g_am67_ecap2_pinmux_conf[] =
{
  /* eCAP2 APWM out: MCASP0_ACLKX mode 2 */

  {
    PIN_MCASP0_ACLKX,
    (PIN_MODE(2) | PIN_PULL_DISABLE)
  },
  {PINMUX_END, PINMUX_END}
};

static struct pinmux_conf_s g_am67_mcu_spi_pinmux_conf[] =
{
  /* MCU_SPI0_CLK */

  {
    PIN_MCU_SPI0_CLK,
    (PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE)
  },

  /* MCU_SPI0_D0 (MOSI) - output to sensor SDI; INPUT_ENABLE allows the
   * D0 self-loopback test
   */

  {
    PIN_MCU_SPI0_D0,
    (PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE)
  },

  /* MCU_SPI0_D1 (MISO) - sensor SDO to D1 (ti,pindir-d0-out-d1-in) */

  {
    PIN_MCU_SPI0_D1,
    (PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE)
  },

  /* MCU_SPI0_CS0 (unused, configures pad to SPI CS0 function) */

  {
    PIN_MCU_SPI0_CS0,
    (PIN_MODE(0) | PIN_PULL_DISABLE)
  },

  /* MCU_SPI0_CS1 (LPS22DF, channel 1) */

  {
    PIN_MCU_SPI0_CS1,
    (PIN_MODE(0) | PIN_PULL_DISABLE)
  },

  /* MCU_SPI0_CS3 (ICM20948, channel 3) - MCU_MCAN0_TX pad, mode 2 */

  {
    PIN_MCU_MCAN0_TX,
    (PIN_MODE(2) | PIN_PULL_DISABLE)
  },

  /* IMU_EN -> MCU_GPIO0_12 (C3) - WKUP_UART0_RTSN pad at mode 7 */

  {
    PIN_WKUP_UART0_RTSN,
    (PIN_MODE(7) | PIN_PULL_DISABLE)
  },
  {PINMUX_END, PINMUX_END}
};

static struct pinmux_conf_s g_am67_mcu_i2c_pinmux_conf[] =
{
#ifdef CONFIG_AM67_I2C0
  /* MCU_I2C0_SCL */

  {
    PIN_MCU_I2C0_SCL,
    (PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE)
  },

  /* MCU_I2C0_SDA */

  {
    PIN_MCU_I2C0_SDA,
    (PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE)
  },
#endif

#ifdef CONFIG_AM67_WKUP_I2C0
  /* WKUP_I2C0_SCL */

  {
    PIN_WKUP_I2C0_SCL,
    (PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE)
  },

  /* WKUP_I2C0_SDA */

  {
    PIN_WKUP_I2C0_SDA,
    (PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE)
  },
#endif
  {PINMUX_END, PINMUX_END}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_pinmux_unlock
 *
 * Description:
 *   Unlock the pinmux configuration registers by writing unlock values to
 *   both lock registers (Lock0 and Lock1) kick registers.
 *
 ****************************************************************************/

static void am67_pinmux_unlock(void)
{
  uint32_t base_addr;
  uint32_t kick_addr;

  base_addr = CSL_PADCFG_CTRL0_CFG0_BASE;

  /* Lock 0 */

  kick_addr = base_addr + CSL_MAIN_PADCONFIG_LOCK0_KICK0_OFFSET;
  putreg32(KICK0_UNLOCK_VAL, kick_addr);
  kick_addr += 4;
  putreg32(KICK1_UNLOCK_VAL, kick_addr);

  /* Lock 1 */

  kick_addr = base_addr + CSL_MAIN_PADCONFIG_LOCK1_KICK0_OFFSET;
  putreg32(KICK0_UNLOCK_VAL, kick_addr);
  kick_addr += 4;
  putreg32(KICK1_UNLOCK_VAL, kick_addr);
}

static void am67_mcu_pinmux_unlock(void)
{
  uint32_t base_addr;
  uint32_t kick_addr;

  base_addr = CSL_MCU_PADCFG_CTRL0_CFG0_BASE;

  kick_addr = base_addr + CSL_MCU_PADCONFIG_LOCK0_KICK0_OFFSET;
  putreg32(KICK0_UNLOCK_VAL, kick_addr);
  kick_addr += 4;
  putreg32(KICK1_UNLOCK_VAL, kick_addr);

  kick_addr = base_addr + CSL_MCU_PADCONFIG_LOCK1_KICK0_OFFSET;
  putreg32(KICK0_UNLOCK_VAL, kick_addr);
  kick_addr += 4;
  putreg32(KICK1_UNLOCK_VAL, kick_addr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_pinmux_config
 *
 * Description:
 *   Configure pin multiplexing settings by writing configuration values to
 *   pad configuration registers after unlocking the pinmux registers.
 *
 ****************************************************************************/

void am67_pinmux_config(const struct pinmux_conf_s *pinmux_conf)
{
  if (pinmux_conf != NULL)
    {
      uint32_t base_addr = CSL_PADCFG_CTRL0_CFG0_BASE + PADCFG_PMUX_OFFSET;

      am67_pinmux_unlock();

      while (pinmux_conf->offset != PINMUX_END)
        {
          /* Set all the configuration fields */

          putreg32(pinmux_conf->setting, base_addr + pinmux_conf->offset);
          pinmux_conf++;
        }
    }
}

/****************************************************************************
 * Name: am67_mcu_pinmux_config
 ****************************************************************************/

void am67_mcu_pinmux_config(const struct pinmux_conf_s *pinmux_conf)
{
  if (pinmux_conf != NULL)
    {
      uint32_t base_addr = CSL_MCU_PADCFG_CTRL0_CFG0_BASE +
                           PADCFG_PMUX_OFFSET;

      am67_mcu_pinmux_unlock();

      while (pinmux_conf->offset != PINMUX_END)
        {
          putreg32(pinmux_conf->setting, base_addr + pinmux_conf->offset);
          pinmux_conf++;
        }
    }
}

/****************************************************************************
 * Name: am67_pinmux_init
 *
 * Description:
 *   Initialize pin multiplexing using the global pinmux configuration array.
 *
 ****************************************************************************/

void am67_pinmux_init(void)
{
  am67_pinmux_config(g_am67_pinmux_conf);
}

/****************************************************************************
 * Name: am67_spi_pinmux_init
 *
 * Description:
 *   Configure MCU_SPI0 pin multiplexing for onboard sensors.
 *
 ****************************************************************************/

void am67_spi_pinmux_init(void)
{
  am67_mcu_pinmux_config(g_am67_mcu_spi_pinmux_conf);
}

/****************************************************************************
 * Name: am67_i2c_pinmux_init
 *
 * Description:
 *   Configure MCU_I2C0 pin multiplexing for onboard sensors.
 *
 ****************************************************************************/

void am67_i2c_pinmux_init(void)
{
  am67_mcu_pinmux_config(g_am67_mcu_i2c_pinmux_conf);
}

/****************************************************************************
 * Name: am67_epwm_pinmux_init
 *
 * Description:
 *   Configure the output pin multiplexing (A and B pads) for the given
 *   EPWM instance.
 *
 ****************************************************************************/

void am67_epwm_pinmux_init(int epwm)
{
  if (epwm == 0)
    {
      am67_pinmux_config(g_am67_epwm0_pinmux_conf);
    }
  else
    {
      am67_pinmux_config(g_am67_epwm1_pinmux_conf);
    }
}

/****************************************************************************
 * Name: am67_ecap_pinmux_init
 *
 * Description:
 *   Configure the APWM output pad for the given eCAP module.  eCAP0 uses
 *   ball A23 (mode 8); eCAP1/eCAP2 pad tables are empty pending a board
 *   decision (see the table definitions above).
 *
 ****************************************************************************/

void am67_ecap_pinmux_init(int ecap)
{
  switch (ecap)
    {
      case 0:
        am67_pinmux_config(g_am67_ecap0_pinmux_conf);
        break;

      case 1:
        am67_pinmux_config(g_am67_ecap1_pinmux_conf);
        break;

      case 2:
        am67_pinmux_config(g_am67_ecap2_pinmux_conf);
        break;

      default:
        break;
    }
}
