/****************************************************************************
 * arch/arm/src/imxrt/imxrt_gpio.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/irq.h>

#include "chip.h"
#include "up_arch.h"
#include "imxrt_iomuxc.h"
#include "imxrt_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_PADMUX_INVALID    255

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_gpio1_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_AD_B0_00_INDEX,           /* GPIO1 Pin 0 */
  IMXRT_PADMUX_GPIO_AD_B0_01_INDEX,           /* GPIO1 Pin 1 */
  IMXRT_PADMUX_GPIO_AD_B0_02_INDEX,           /* GPIO1 Pin 2 */
  IMXRT_PADMUX_GPIO_AD_B0_03_INDEX,           /* GPIO1 Pin 3 */
  IMXRT_PADMUX_GPIO_AD_B0_04_INDEX,           /* GPIO1 Pin 4 */
  IMXRT_PADMUX_GPIO_AD_B0_05_INDEX,           /* GPIO1 Pin 5 */
  IMXRT_PADMUX_GPIO_AD_B0_06_INDEX,           /* GPIO1 Pin 6 */
  IMXRT_PADMUX_GPIO_AD_B0_07_INDEX,           /* GPIO1 Pin 7 */

  IMXRT_PADMUX_GPIO_AD_B0_08_INDEX,           /* GPIO1 Pin 8 */
  IMXRT_PADMUX_GPIO_AD_B0_09_INDEX,           /* GPIO1 Pin 9 */
  IMXRT_PADMUX_GPIO_AD_B0_10_INDEX,           /* GPIO1 Pin 10 */
  IMXRT_PADMUX_GPIO_AD_B0_11_INDEX,           /* GPIO1 Pin 11 */
  IMXRT_PADMUX_GPIO_AD_B0_12_INDEX,           /* GPIO1 Pin 12 */
  IMXRT_PADMUX_GPIO_AD_B0_13_INDEX,           /* GPIO1 Pin 13 */
  IMXRT_PADMUX_GPIO_AD_B0_14_INDEX,           /* GPIO1 Pin 14 */
  IMXRT_PADMUX_GPIO_AD_B0_15_INDEX,           /* GPIO1 Pin 15 */

  IMXRT_PADMUX_GPIO_AD_B1_00_INDEX,           /* GPIO1 Pin 16 */
  IMXRT_PADMUX_GPIO_AD_B1_01_INDEX,           /* GPIO1 Pin 17 */
  IMXRT_PADMUX_GPIO_AD_B1_02_INDEX,           /* GPIO1 Pin 18 */
  IMXRT_PADMUX_GPIO_AD_B1_03_INDEX,           /* GPIO1 Pin 19 */
  IMXRT_PADMUX_GPIO_AD_B1_04_INDEX,           /* GPIO1 Pin 20 */
  IMXRT_PADMUX_GPIO_AD_B1_05_INDEX,           /* GPIO1 Pin 21 */
  IMXRT_PADMUX_GPIO_AD_B1_06_INDEX,           /* GPIO1 Pin 22 */
  IMXRT_PADMUX_GPIO_AD_B1_07_INDEX,           /* GPIO1 Pin 23 */

  IMXRT_PADMUX_GPIO_AD_B1_08_INDEX,           /* GPIO1 Pin 24 */
  IMXRT_PADMUX_GPIO_AD_B1_09_INDEX,           /* GPIO1 Pin 25 */
  IMXRT_PADMUX_GPIO_AD_B1_10_INDEX,           /* GPIO1 Pin 26 */
  IMXRT_PADMUX_GPIO_AD_B1_11_INDEX,           /* GPIO1 Pin 27 */
  IMXRT_PADMUX_GPIO_AD_B1_12_INDEX,           /* GPIO1 Pin 28 */
  IMXRT_PADMUX_GPIO_AD_B1_13_INDEX,           /* GPIO1 Pin 29 */
  IMXRT_PADMUX_GPIO_AD_B1_14_INDEX,           /* GPIO1 Pin 30 */
  IMXRT_PADMUX_GPIO_AD_B1_15_INDEX            /* GPIO1 Pin 31 */
};

static const uint8_t g_gpio2_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_B0_00_INDEX,              /* GPIO2 Pin 0 */
  IMXRT_PADMUX_GPIO_B0_01_INDEX,              /* GPIO2 Pin 1 */
  IMXRT_PADMUX_GPIO_B0_02_INDEX,              /* GPIO2 Pin 2 */
  IMXRT_PADMUX_GPIO_B0_03_INDEX,              /* GPIO2 Pin 3 */
  IMXRT_PADMUX_GPIO_B0_04_INDEX,              /* GPIO2 Pin 4 */
  IMXRT_PADMUX_GPIO_B0_05_INDEX,              /* GPIO2 Pin 5 */
  IMXRT_PADMUX_GPIO_B0_06_INDEX,              /* GPIO2 Pin 6 */
  IMXRT_PADMUX_GPIO_B0_07_INDEX,              /* GPIO2 Pin 7 */

  IMXRT_PADMUX_GPIO_B0_08_INDEX,              /* GPIO2 Pin 8 */
  IMXRT_PADMUX_GPIO_B0_09_INDEX,              /* GPIO2 Pin 9 */
  IMXRT_PADMUX_GPIO_B0_10_INDEX,              /* GPIO2 Pin 10 */
  IMXRT_PADMUX_GPIO_B0_11_INDEX,              /* GPIO2 Pin 11 */
  IMXRT_PADMUX_GPIO_B0_12_INDEX,              /* GPIO2 Pin 12 */
  IMXRT_PADMUX_GPIO_B0_13_INDEX,              /* GPIO2 Pin 13 */
  IMXRT_PADMUX_GPIO_B0_14_INDEX,              /* GPIO2 Pin 14 */
  IMXRT_PADMUX_GPIO_B0_15_INDEX,              /* GPIO2 Pin 15 */

  IMXRT_PADMUX_GPIO_B1_00_INDEX,              /* GPIO2 Pin 16 */
  IMXRT_PADMUX_GPIO_B1_01_INDEX,              /* GPIO2 Pin 17 */
  IMXRT_PADMUX_GPIO_B1_02_INDEX,              /* GPIO2 Pin 18 */
  IMXRT_PADMUX_GPIO_B1_03_INDEX,              /* GPIO2 Pin 19 */
  IMXRT_PADMUX_GPIO_B1_04_INDEX,              /* GPIO2 Pin 20 */
  IMXRT_PADMUX_GPIO_B1_05_INDEX,              /* GPIO2 Pin 21 */
  IMXRT_PADMUX_GPIO_B1_06_INDEX,              /* GPIO2 Pin 22 */
  IMXRT_PADMUX_GPIO_B1_07_INDEX,              /* GPIO2 Pin 23 */

  IMXRT_PADMUX_GPIO_B1_08_INDEX,              /* GPIO2 Pin 24 */
  IMXRT_PADMUX_GPIO_B1_09_INDEX,              /* GPIO2 Pin 25 */
  IMXRT_PADMUX_GPIO_B1_10_INDEX,              /* GPIO2 Pin 26 */
  IMXRT_PADMUX_GPIO_B1_11_INDEX,              /* GPIO2 Pin 27 */
  IMXRT_PADMUX_GPIO_B1_12_INDEX,              /* GPIO2 Pin 28 */
  IMXRT_PADMUX_GPIO_B1_13_INDEX,              /* GPIO2 Pin 29 */
  IMXRT_PADMUX_GPIO_B1_14_INDEX,              /* GPIO2 Pin 30 */
  IMXRT_PADMUX_GPIO_B1_15_INDEX               /* GPIO2 Pin 31 */
};

static const uint8_t g_gpio3_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_SD_B1_00_INDEX,           /* GPIO3 Pin 0 */
  IMXRT_PADMUX_GPIO_SD_B1_01_INDEX,           /* GPIO3 Pin 1 */
  IMXRT_PADMUX_GPIO_SD_B1_02_INDEX,           /* GPIO3 Pin 2 */
  IMXRT_PADMUX_GPIO_SD_B1_03_INDEX,           /* GPIO3 Pin 3 */
  IMXRT_PADMUX_GPIO_SD_B1_04_INDEX,           /* GPIO3 Pin 4 */
  IMXRT_PADMUX_GPIO_SD_B1_05_INDEX,           /* GPIO3 Pin 5 */
  IMXRT_PADMUX_GPIO_SD_B1_06_INDEX,           /* GPIO3 Pin 6 */
  IMXRT_PADMUX_GPIO_SD_B1_07_INDEX,           /* GPIO3 Pin 7 */

  IMXRT_PADMUX_GPIO_SD_B1_08_INDEX,           /* GPIO3 Pin 8 */
  IMXRT_PADMUX_GPIO_SD_B1_09_INDEX,           /* GPIO3 Pin 9 */
  IMXRT_PADMUX_GPIO_SD_B1_10_INDEX,           /* GPIO3 Pin 10 */
  IMXRT_PADMUX_GPIO_SD_B1_11_INDEX,           /* GPIO3 Pin 11 */
  IMXRT_PADMUX_GPIO_SD_B0_00_INDEX,           /* GPIO3 Pin 12 */
  IMXRT_PADMUX_GPIO_SD_B0_01_INDEX,           /* GPIO3 Pin 13 */
  IMXRT_PADMUX_GPIO_SD_B0_02_INDEX,           /* GPIO3 Pin 14 */
  IMXRT_PADMUX_GPIO_SD_B0_03_INDEX,           /* GPIO3 Pin 15 */

  IMXRT_PADMUX_GPIO_SD_B0_04_INDEX,           /* GPIO3 Pin 16 */
  IMXRT_PADMUX_GPIO_SD_B0_05_INDEX,           /* GPIO3 Pin 17 */
  IMXRT_PADMUX_GPIO_EMC_32_INDEX,             /* GPIO3 Pin 18 */
  IMXRT_PADMUX_GPIO_EMC_33_INDEX,             /* GPIO3 Pin 19 */
  IMXRT_PADMUX_GPIO_EMC_34_INDEX,             /* GPIO3 Pin 20 */
  IMXRT_PADMUX_GPIO_EMC_35_INDEX,             /* GPIO3 Pin 21 */
  IMXRT_PADMUX_GPIO_EMC_36_INDEX,             /* GPIO3 Pin 22 */
  IMXRT_PADMUX_GPIO_EMC_37_INDEX,             /* GPIO3 Pin 23 */

  IMXRT_PADMUX_GPIO_EMC_38_INDEX,             /* GPIO3 Pin 24 */
  IMXRT_PADMUX_GPIO_EMC_39_INDEX,             /* GPIO3 Pin 25 */
  IMXRT_PADMUX_GPIO_EMC_40_INDEX,             /* GPIO3 Pin 26 */
  IMXRT_PADMUX_GPIO_EMC_41_INDEX,             /* GPIO3 Pin 27 */
  IMXRT_PADMUX_INVALID,                       /* GPIO3 Pin 28 */
  IMXRT_PADMUX_INVALID,                       /* GPIO3 Pin 29 */
  IMXRT_PADMUX_INVALID,                       /* GPIO3 Pin 30 */
  IMXRT_PADMUX_INVALID                        /* GPIO3 Pin 31 */
};

static const uint8_t g_gpio4_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_GPIO_EMC_00_INDEX,             /* GPIO4 Pin 0 */
  IMXRT_PADMUX_GPIO_EMC_01_INDEX,             /* GPIO4 Pin 1 */
  IMXRT_PADMUX_GPIO_EMC_02_INDEX,             /* GPIO4 Pin 2 */
  IMXRT_PADMUX_GPIO_EMC_03_INDEX,             /* GPIO4 Pin 3 */
  IMXRT_PADMUX_GPIO_EMC_04_INDEX,             /* GPIO4 Pin 4 */
  IMXRT_PADMUX_GPIO_EMC_05_INDEX,             /* GPIO4 Pin 5 */
  IMXRT_PADMUX_GPIO_EMC_06_INDEX,             /* GPIO4 Pin 6 */
  IMXRT_PADMUX_GPIO_EMC_07_INDEX,             /* GPIO4 Pin 7 */

  IMXRT_PADMUX_GPIO_EMC_08_INDEX,             /* GPIO4 Pin 8 */
  IMXRT_PADMUX_GPIO_EMC_09_INDEX,             /* GPIO4 Pin 9 */
  IMXRT_PADMUX_GPIO_EMC_10_INDEX,             /* GPIO4 Pin 10 */
  IMXRT_PADMUX_GPIO_EMC_11_INDEX,             /* GPIO4 Pin 11 */
  IMXRT_PADMUX_GPIO_EMC_12_INDEX,             /* GPIO4 Pin 12 */
  IMXRT_PADMUX_GPIO_EMC_13_INDEX,             /* GPIO4 Pin 13 */
  IMXRT_PADMUX_GPIO_EMC_14_INDEX,             /* GPIO4 Pin 14 */
  IMXRT_PADMUX_GPIO_EMC_15_INDEX,             /* GPIO4 Pin 15 */

  IMXRT_PADMUX_GPIO_EMC_16_INDEX,             /* GPIO4 Pin 16 */
  IMXRT_PADMUX_GPIO_EMC_17_INDEX,             /* GPIO4 Pin 17 */
  IMXRT_PADMUX_GPIO_EMC_18_INDEX,             /* GPIO4 Pin 18 */
  IMXRT_PADMUX_GPIO_EMC_19_INDEX,             /* GPIO4 Pin 19 */
  IMXRT_PADMUX_GPIO_EMC_20_INDEX,             /* GPIO4 Pin 20 */
  IMXRT_PADMUX_GPIO_EMC_21_INDEX,             /* GPIO4 Pin 21 */
  IMXRT_PADMUX_GPIO_EMC_22_INDEX,             /* GPIO4 Pin 22 */
  IMXRT_PADMUX_GPIO_EMC_23_INDEX,             /* GPIO4 Pin 23 */

  IMXRT_PADMUX_GPIO_EMC_24_INDEX,             /* GPIO4 Pin 24 */
  IMXRT_PADMUX_GPIO_EMC_25_INDEX,             /* GPIO4 Pin 25 */
  IMXRT_PADMUX_GPIO_EMC_26_INDEX,             /* GPIO4 Pin 26 */
  IMXRT_PADMUX_GPIO_EMC_27_INDEX,             /* GPIO4 Pin 27 */
  IMXRT_PADMUX_GPIO_EMC_28_INDEX,             /* GPIO4 Pin 28 */
  IMXRT_PADMUX_GPIO_EMC_29_INDEX,             /* GPIO4 Pin 29 */
  IMXRT_PADMUX_GPIO_EMC_30_INDEX,             /* GPIO4 Pin 30 */
  IMXRT_PADMUX_GPIO_EMC_31_INDEX              /* GPIO4 Pin 31 */
};

static const uint8_t g_gpio5_padmux[IMXRT_GPIO_NPINS] =
{
  IMXRT_PADMUX_WAKEUP_INDEX,                  /* GPIO5 Pin 0 */
  IMXRT_PADMUX_PMIC_ON_REQ_INDEX,             /* GPIO5 Pin 1 */
  IMXRT_PADMUX_PMIC_STBY_REQ_INDEX,           /* GPIO5 Pin 2 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 3 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 4 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 5 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 6 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 7 */

  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 8 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 9 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 10 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 11 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 12 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 13 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 14 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 15 */

  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 16 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 17 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 18 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 19 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 20 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 21 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 22 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 23 */

  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 24 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 25 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 26 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 27 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 28 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 29 */
  IMXRT_PADMUX_INVALID,                       /* GPIO5 Pin 30 */
  IMXRT_PADMUX_INVALID                        /* GPIO5 Pin 31 */
};

static FAR const uint8_t *g_gpio_padmux[IMXRT_GPIO_NPORTS + 1] =
{
  g_gpio1_padmux,                             /* GPIO1 */
  g_gpio2_padmux,                             /* GPIO2 */
  g_gpio3_padmux,                             /* GPIO3 */
  g_gpio4_padmux,                             /* GPIO4 */
  g_gpio5_padmux,                             /* GPIO5 */
  NULL                                        /* End of list */
};

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* Look-up table that maps GPIO1..GPIO5 indexes into GPIO register base addresses */

uintptr_t g_gpio_base[IMXRT_GPIO_NPORTS] =
{
  IMXRT_GPIO1_BASE
#if IMXRT_GPIO_NPORTS > 1
  , IMXRT_GPIO2_BASE
#endif
#if IMXRT_GPIO_NPORTS > 2
  , IMXRT_GPIO3_BASE
#endif
#if IMXRT_GPIO_NPORTS > 3
  , IMXRT_GPIO4_BASE
#endif
#if IMXRT_GPIO_NPORTS > 4
  , IMXRT_GPIO5_BASE
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_padmux_address
 ****************************************************************************/

static uintptr_t imxrt_padmux_address(unsigned int index)
{
  if (index >= IMXRT_PADMUX_WAKEUP_INDEX)
    {
      return (IMXRT_PADMUX_ADDRESS_SNVS(index - IMXRT_PADMUX_WAKEUP_INDEX));
    }

  return (IMXRT_PADMUX_ADDRESS(index));
}

/****************************************************************************
 * Name: imxrt_padctl_address
 ****************************************************************************/

static uintptr_t imxrt_padctl_address(unsigned int index)
{
  if (index >= IMXRT_PADCTL_WAKEUP_INDEX)
    {
      return (IMXRT_PADCTL_ADDRESS_SNVS(index - IMXRT_PADCTL_WAKEUP_INDEX));
    }

  return (IMXRT_PADCTL_ADDRESS(index));
}

/****************************************************************************
 * Name: imxrt_gpio_dirout
 ****************************************************************************/

static inline void imxrt_gpio_dirout(int port, int pin)
{
  uint32_t regval = getreg32(IMXRT_GPIO_GDIR(port));
  regval |= GPIO_PIN(pin);
  putreg32(regval, IMXRT_GPIO_GDIR(port));
}

/****************************************************************************
 * Name: imxrt_gpio_dirin
 ****************************************************************************/

static inline void imxrt_gpio_dirin(int port, int pin)
{
  uint32_t regval = getreg32(IMXRT_GPIO_GDIR(port));
  regval &= ~GPIO_PIN(pin);
  putreg32(regval, IMXRT_GPIO_GDIR(port));
}

/****************************************************************************
 * Name: imxrt_gpio_setoutput
 ****************************************************************************/

static void imxrt_gpio_setoutput(int port, int pin, bool value)
{
  uintptr_t regaddr = IMXRT_GPIO_DR(port);
  uint32_t regval;

  regval = getreg32(regaddr);
  if (value)
    {
      regval |= GPIO_PIN(pin);
    }
  else
    {
      regval &= ~GPIO_PIN(pin);
    }

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: imxrt_gpio_getinput
 ****************************************************************************/

static inline bool imxrt_gpio_getinput(int port, int pin)
{
  uintptr_t regaddr = IMXRT_GPIO_DR(port);
  uint32_t regval;

  regval = getreg32(regaddr);
  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: imxrt_gpio_configinput
 ****************************************************************************/

static int imxrt_gpio_configinput(gpio_pinset_t pinset)
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  FAR const uint8_t *table;
  iomux_pinset_t ioset;
  uintptr_t regaddr;
  unsigned int index;

  DEBUGASSERT((unsigned int)port < IMXRT_GPIO_NPORTS);

  /* Configure pin as in input */

  imxrt_gpio_dirin(port, pin);

  /* Configure pin as a GPIO */

  table = g_gpio_padmux[port];
  if (table == NULL)
    {
      return -EINVAL;
    }

  index = (unsigned int)table[pin];
  if (index >= IMXRT_PADMUX_NREGISTERS)
    {
      return -EINVAL;
    }

  regaddr = imxrt_padmux_address(index);
  putreg32(PADMUX_MUXMODE_ALT5, regaddr);

  /* Configure pin pad settings */

  index = imxrt_padmux_map(index);
  if (index >= IMXRT_PADCTL_NREGISTERS)
    {
      return -EINVAL;
    }

  regaddr = imxrt_padctl_address(index);
  ioset   = (iomux_pinset_t)((pinset & GPIO_IOMUX_MASK) >> GPIO_IOMUX_SHIFT);
  return imxrt_iomux_configure(regaddr, ioset);
}

/****************************************************************************
 * Name: imxrt_gpio_configoutput
 ****************************************************************************/

static inline int imxrt_gpio_configoutput(gpio_pinset_t pinset)
{
  int port   = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin    = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value = ((pinset & GPIO_OUTPUT_ONE) != 0);

  DEBUGASSERT((unsigned int)port < IMXRT_GPIO_NPORTS);

  /* Set the output value */

  imxrt_gpio_setoutput(port, pin, value);

  /* Convert the configured input GPIO to an output */

  imxrt_gpio_dirout(port, pin);
  return OK;
}

/****************************************************************************
 * Name: imxrt_gpio_configperiph
 ****************************************************************************/

static inline int imxrt_gpio_configperiph(gpio_pinset_t pinset)
{
  iomux_pinset_t ioset;
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t value;
  unsigned int index;

  /* Configure pin as a peripheral */

  index   = ((pinset & GPIO_PADMUX_MASK) >> GPIO_PADMUX_SHIFT);
  regaddr = imxrt_padmux_address(index);

  value   = ((pinset & GPIO_ALT_MASK) >> GPIO_ALT_SHIFT);
#if GPIO_SION_SHIFT >= PADMUX_SION_SHIFT
  value  |= ((pinset & GPIO_SION_MASK) >> (GPIO_SION_SHIFT - PADMUX_SION_SHIFT));
#else
  value  |= ((pinset & GPIO_SION_MASK) << (PADMUX_SION_SHIFT - GPIO_SION_SHIFT));
#endif
  regval  = (value << PADMUX_MUXMODE_SHIFT);

  putreg32(regval, regaddr);

  /* Configure pin pad settings */

  index = imxrt_padmux_map(index);
  if (index >= IMXRT_PADCTL_NREGISTERS)
    {
      return -EINVAL;
    }

  regaddr = imxrt_padctl_address(index);
  ioset   = (iomux_pinset_t)((pinset & GPIO_IOMUX_MASK) >> GPIO_IOMUX_SHIFT);
  return imxrt_iomux_configure(regaddr, ioset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on pin-encoded description of the pin.
 *
 ****************************************************************************/

int imxrt_config_gpio(gpio_pinset_t pinset)
{
  irqstate_t flags;
  int ret;

  /* Configure the pin as an input initially to avoid any spurious outputs */

  flags = enter_critical_section();

  /* Configure based upon the pin mode */

  switch (pinset & GPIO_MODE_MASK)
    {
      case GPIO_INPUT:
        {
          /* Configure the pin as a GPIO input */

          ret = imxrt_gpio_configinput(pinset);
        }
        break;

      case GPIO_OUTPUT:
        {
          /* First configure the pin as a GPIO input to avoid output
           * glitches.
           */

          ret = imxrt_gpio_configinput(pinset);
          if (ret >= 0)
            {
              /* Convert the input to an output */

              ret = imxrt_gpio_configoutput(pinset);
            }
        }
        break;

      case GPIO_PERIPH:
        {
          /* Configure the pin as a peripheral */

          ret = imxrt_gpio_configperiph(pinset);
        }
        break;

#ifdef CONFIG_IMXRT_GPIO_IRQ
      case GPIO_INTERRUPT:
        {
          /* Configure the pin as a GPIO input */

          ret = imxrt_gpio_configinput(pinset);
          if (ret == OK)
            {
              ret = imxrt_gpioirq_configure(pinset);
            }
        }
        break;
#endif

      default:
        ret = -EINVAL;
        break;
    }

  leave_critical_section(flags);
  return ret;
}

/************************************************************************************
 * Name: imxrt_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void imxrt_gpio_write(gpio_pinset_t pinset, bool value)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT((unsigned int)port < IMXRT_GPIO_NPORTS);

  flags = enter_critical_section();
  imxrt_gpio_setoutput(port, pin, value);
  leave_critical_section(flags);
}

/************************************************************************************
 * Name: imxrt_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool imxrt_gpio_read(gpio_pinset_t pinset)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value;

  DEBUGASSERT((unsigned int)port < IMXRT_GPIO_NPORTS);

  flags = enter_critical_section();
  value = imxrt_gpio_getinput(port, pin);
  leave_critical_section(flags);
  return value;
}
