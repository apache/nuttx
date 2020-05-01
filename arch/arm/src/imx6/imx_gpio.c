/****************************************************************************
 * arch/arm/src/imx6/imx_gpio.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/irq.h>

#include "chip.h"
#include "arm_arch.h"
#include "imx_iomuxc.h"
#include "imx_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX_PADMUX_INVALID           255

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_gpio1_padmux[IMX_GPIO_NPINS] =
{
  IMX_PADMUX_GPIO00_INDEX,           /* GPIO1 Pin 0 */
  IMX_PADMUX_GPIO01_INDEX,           /* GPIO1 Pin 1 */
  IMX_PADMUX_GPIO02_INDEX,           /* GPIO1 Pin 2 */
  IMX_PADMUX_GPIO03_INDEX,           /* GPIO1 Pin 3 */
  IMX_PADMUX_GPIO04_INDEX,           /* GPIO1 Pin 4 */
  IMX_PADMUX_GPIO05_INDEX,           /* GPIO1 Pin 5 */
  IMX_PADMUX_GPIO06_INDEX,           /* GPIO1 Pin 6 */
  IMX_PADMUX_GPIO07_INDEX,           /* GPIO1 Pin 7 */

  IMX_PADMUX_GPIO08_INDEX,           /* GPIO1 Pin 8 */
  IMX_PADMUX_GPIO09_INDEX,           /* GPIO1 Pin 9 */
  IMX_PADMUX_SD2_CLK_INDEX,          /* GPIO1 Pin 10 */
  IMX_PADMUX_SD2_CMD_INDEX,          /* GPIO1 Pin 11 */
  IMX_PADMUX_SD2_DATA3_INDEX,        /* GPIO1 Pin 12 */
  IMX_PADMUX_SD2_DATA2_INDEX,        /* GPIO1 Pin 13 */
  IMX_PADMUX_SD2_DATA1_INDEX,        /* GPIO1 Pin 14 */
  IMX_PADMUX_SD2_DATA0_INDEX,        /* GPIO1 Pin 15 */

  IMX_PADMUX_SD1_DATA0_INDEX,        /* GPIO1 Pin 16 */
  IMX_PADMUX_SD1_DATA1_INDEX,        /* GPIO1 Pin 17 */
  IMX_PADMUX_SD1_CMD_INDEX,          /* GPIO1 Pin 18 */
  IMX_PADMUX_SD1_DATA2_INDEX,        /* GPIO1 Pin 19 */
  IMX_PADMUX_SD1_CLK_INDEX,          /* GPIO1 Pin 20 */
  IMX_PADMUX_SD1_DATA3_INDEX,        /* GPIO1 Pin 21 */
  IMX_PADMUX_ENET_MDIO_INDEX,        /* GPIO1 Pin 22 */
  IMX_PADMUX_ENET_REF_CLK_INDEX,     /* GPIO1 Pin 23 */

  IMX_PADMUX_ENET_RX_ER_INDEX,       /* GPIO1 Pin 24 */
  IMX_PADMUX_ENET_CRS_DV_INDEX,      /* GPIO1 Pin 25 */
  IMX_PADMUX_ENET_RX_DATA1_INDEX,    /* GPIO1 Pin 26 */
  IMX_PADMUX_ENET_RX_DATA0_INDEX,    /* GPIO1 Pin 27 */
  IMX_PADMUX_ENET_TX_EN_INDEX,       /* GPIO1 Pin 28 */
  IMX_PADMUX_ENET_TX_DATA1_INDEX,    /* GPIO1 Pin 29 */
  IMX_PADMUX_ENET_TX_DATA0_INDEX,    /* GPIO1 Pin 30 */
  IMX_PADMUX_ENET_MDC_INDEX          /* GPIO1 Pin 31 */
};

static const uint8_t g_gpio2_padmux[IMX_GPIO_NPINS] =
{
  IMX_PADMUX_NAND_DATA00_INDEX,      /* GPIO2 Pin 0 */
  IMX_PADMUX_NAND_DATA01_INDEX,      /* GPIO2 Pin 1 */
  IMX_PADMUX_NAND_DATA02_INDEX,      /* GPIO2 Pin 2 */
  IMX_PADMUX_NAND_DATA03_INDEX,      /* GPIO2 Pin 3 */
  IMX_PADMUX_NAND_DATA04_INDEX,      /* GPIO2 Pin 4 */
  IMX_PADMUX_NAND_DATA05_INDEX,      /* GPIO2 Pin 5 */
  IMX_PADMUX_NAND_DATA06_INDEX,      /* GPIO2 Pin 6 */
  IMX_PADMUX_NAND_DATA07_INDEX,      /* GPIO2 Pin 7 */

  IMX_PADMUX_SD4_DATA0_INDEX,        /* GPIO2 Pin 8 */
  IMX_PADMUX_SD4_DATA1_INDEX,        /* GPIO2 Pin 9 */
  IMX_PADMUX_SD4_DATA2_INDEX,        /* GPIO2 Pin 10 */
  IMX_PADMUX_SD4_DATA3_INDEX,        /* GPIO2 Pin 11 */
  IMX_PADMUX_SD4_DATA4_INDEX,        /* GPIO2 Pin 12 */
  IMX_PADMUX_SD4_DATA5_INDEX,        /* GPIO2 Pin 13 */
  IMX_PADMUX_SD4_DATA6_INDEX,        /* GPIO2 Pin 14 */
  IMX_PADMUX_SD4_DATA7_INDEX,        /* GPIO2 Pin 15 */

  IMX_PADMUX_EIM_ADDR22_INDEX,       /* GPIO2 Pin 16 */
  IMX_PADMUX_EIM_ADDR21_INDEX,       /* GPIO2 Pin 17 */
  IMX_PADMUX_EIM_ADDR20_INDEX,       /* GPIO2 Pin 18 */
  IMX_PADMUX_EIM_ADDR19_INDEX,       /* GPIO2 Pin 19 */
  IMX_PADMUX_EIM_ADDR18_INDEX,       /* GPIO2 Pin 20 */
  IMX_PADMUX_EIM_ADDR17_INDEX,       /* GPIO2 Pin 21 */
  IMX_PADMUX_EIM_ADDR16_INDEX,       /* GPIO2 Pin 22 */
  IMX_PADMUX_EIM_CS0_INDEX,          /* GPIO2 Pin 23 */

  IMX_PADMUX_EIM_CS1_INDEX,          /* GPIO2 Pin 24 */
  IMX_PADMUX_EIM_OE_INDEX,           /* GPIO2 Pin 25 */
  IMX_PADMUX_EIM_RW_INDEX,           /* GPIO2 Pin 26 */
  IMX_PADMUX_EIM_LBA_INDEX,          /* GPIO2 Pin 27 */
  IMX_PADMUX_EIM_EB0_INDEX,          /* GPIO2 Pin 28 */
  IMX_PADMUX_EIM_EB1_INDEX,          /* GPIO2 Pin 29 */
  IMX_PADMUX_EIM_EB2_INDEX,          /* GPIO2 Pin 30 */
  IMX_PADMUX_EIM_EB3_INDEX,          /* GPIO2 Pin 31 */
};

static const uint8_t g_gpio3_padmux[IMX_GPIO_NPINS] =
{
  IMX_PADMUX_EIM_AD00_INDEX,         /* GPIO3 Pin 0 */
  IMX_PADMUX_EIM_AD01_INDEX,         /* GPIO3 Pin 1 */
  IMX_PADMUX_EIM_AD02_INDEX,         /* GPIO3 Pin 2 */
  IMX_PADMUX_EIM_AD03_INDEX,         /* GPIO3 Pin 3 */
  IMX_PADMUX_EIM_AD04_INDEX,         /* GPIO3 Pin 4 */
  IMX_PADMUX_EIM_AD05_INDEX,         /* GPIO3 Pin 5 */
  IMX_PADMUX_EIM_AD06_INDEX,         /* GPIO3 Pin 6 */
  IMX_PADMUX_EIM_AD07_INDEX,         /* GPIO3 Pin 7 */

  IMX_PADMUX_EIM_AD08_INDEX,         /* GPIO3 Pin 8 */
  IMX_PADMUX_EIM_AD09_INDEX,         /* GPIO3 Pin 9 */
  IMX_PADMUX_EIM_AD10_INDEX,         /* GPIO3 Pin 10 */
  IMX_PADMUX_EIM_AD11_INDEX,         /* GPIO3 Pin 11 */
  IMX_PADMUX_EIM_AD12_INDEX,         /* GPIO3 Pin 12 */
  IMX_PADMUX_EIM_AD13_INDEX,         /* GPIO3 Pin 13 */
  IMX_PADMUX_EIM_AD14_INDEX,         /* GPIO3 Pin 14 */
  IMX_PADMUX_EIM_AD15_INDEX,         /* GPIO3 Pin 15 */

  IMX_PADMUX_EIM_DATA16_INDEX,       /* GPIO3 Pin 16 */
  IMX_PADMUX_EIM_DATA17_INDEX,       /* GPIO3 Pin 17 */
  IMX_PADMUX_EIM_DATA18_INDEX,       /* GPIO3 Pin 18 */
  IMX_PADMUX_EIM_DATA19_INDEX,       /* GPIO3 Pin 19 */
  IMX_PADMUX_EIM_DATA20_INDEX,       /* GPIO3 Pin 20 */
  IMX_PADMUX_EIM_DATA21_INDEX,       /* GPIO3 Pin 21 */
  IMX_PADMUX_EIM_DATA22_INDEX,       /* GPIO3 Pin 22 */
  IMX_PADMUX_EIM_DATA23_INDEX,       /* GPIO3 Pin 23 */

  IMX_PADMUX_EIM_DATA24_INDEX,       /* GPIO3 Pin 24 */
  IMX_PADMUX_EIM_DATA25_INDEX,       /* GPIO3 Pin 25 */
  IMX_PADMUX_EIM_DATA26_INDEX,       /* GPIO3 Pin 26 */
  IMX_PADMUX_EIM_DATA27_INDEX,       /* GPIO3 Pin 27 */
  IMX_PADMUX_EIM_DATA28_INDEX,       /* GPIO3 Pin 28 */
  IMX_PADMUX_EIM_DATA29_INDEX,       /* GPIO3 Pin 29 */
  IMX_PADMUX_EIM_DATA30_INDEX,       /* GPIO3 Pin 30 */
  IMX_PADMUX_EIM_DATA31_INDEX,       /* GPIO3 Pin 31 */
};

static const uint8_t g_gpio4_padmux[IMX_GPIO_NPINS] =
{
  IMX_PADMUX_INVALID,                /* GPIO4 Pin 0 */
  IMX_PADMUX_INVALID,                /* GPIO4 Pin 1 */
  IMX_PADMUX_INVALID,                /* GPIO4 Pin 2 */
  IMX_PADMUX_INVALID,                /* GPIO4 Pin 3 */
  IMX_PADMUX_INVALID,                /* GPIO4 Pin 4 */
  IMX_PADMUX_GPIO19_INDEX,           /* GPIO4 Pin 5 */
  IMX_PADMUX_KEY_COL0_INDEX,         /* GPIO4 Pin 6 */
  IMX_PADMUX_KEY_ROW0_INDEX,         /* GPIO4 Pin 7 */

  IMX_PADMUX_KEY_COL1_INDEX,         /* GPIO4 Pin 8 */
  IMX_PADMUX_KEY_ROW1_INDEX,         /* GPIO4 Pin 9 */
  IMX_PADMUX_KEY_COL2_INDEX,         /* GPIO4 Pin 10 */
  IMX_PADMUX_KEY_ROW2_INDEX,         /* GPIO4 Pin 11 */
  IMX_PADMUX_KEY_COL3_INDEX,         /* GPIO4 Pin 12 */
  IMX_PADMUX_KEY_ROW3_INDEX,         /* GPIO4 Pin 13 */
  IMX_PADMUX_KEY_COL4_INDEX,         /* GPIO4 Pin 14 */
  IMX_PADMUX_KEY_ROW4_INDEX,         /* GPIO4 Pin 15 */

  IMX_PADMUX_DI0_DISP_CLK_INDEX,     /* GPIO4 Pin 16 */
  IMX_PADMUX_DI0_PIN15_INDEX,        /* GPIO4 Pin 17 */
  IMX_PADMUX_DI0_PIN02_INDEX,        /* GPIO4 Pin 18 */
  IMX_PADMUX_DI0_PIN03_INDEX,        /* GPIO4 Pin 19 */
  IMX_PADMUX_DI0_PIN04_INDEX,        /* GPIO4 Pin 20 */
  IMX_PADMUX_DISP0_DATA00_INDEX,     /* GPIO4 Pin 21 */
  IMX_PADMUX_DISP0_DATA01_INDEX,     /* GPIO4 Pin 22 */
  IMX_PADMUX_DISP0_DATA02_INDEX,     /* GPIO4 Pin 23 */

  IMX_PADMUX_DISP0_DATA03_INDEX,     /* GPIO4 Pin 24 */
  IMX_PADMUX_DISP0_DATA04_INDEX,     /* GPIO4 Pin 25 */
  IMX_PADMUX_DISP0_DATA05_INDEX,     /* GPIO4 Pin 26 */
  IMX_PADMUX_DISP0_DATA06_INDEX,     /* GPIO4 Pin 27 */
  IMX_PADMUX_DISP0_DATA07_INDEX,     /* GPIO4 Pin 28 */
  IMX_PADMUX_DISP0_DATA08_INDEX,     /* GPIO4 Pin 29 */
  IMX_PADMUX_DISP0_DATA09_INDEX,     /* GPIO4 Pin 30 */
  IMX_PADMUX_DISP0_DATA10_INDEX,     /* GPIO4 Pin 31 */
};

static const uint8_t g_gpio5_padmux[IMX_GPIO_NPINS] =
{
  IMX_PADMUX_EIM_WAIT_INDEX,         /* GPIO5 Pin 0 */
  IMX_PADMUX_INVALID,                /* GPIO5 Pin 1 */
  IMX_PADMUX_EIM_ADDR25_INDEX,       /* GPIO5 Pin 2 */
  IMX_PADMUX_INVALID,                /* GPIO5 Pin 3 */
  IMX_PADMUX_EIM_ADDR24_INDEX,       /* GPIO5 Pin 4 */
  IMX_PADMUX_DISP0_DATA11_INDEX,     /* GPIO5 Pin 5 */
  IMX_PADMUX_DISP0_DATA12_INDEX,     /* GPIO5 Pin 6 */
  IMX_PADMUX_DISP0_DATA13_INDEX,     /* GPIO5 Pin 7 */

  IMX_PADMUX_DISP0_DATA14_INDEX,     /* GPIO5 Pin 8 */
  IMX_PADMUX_DISP0_DATA15_INDEX,     /* GPIO5 Pin 9 */
  IMX_PADMUX_DISP0_DATA16_INDEX,     /* GPIO5 Pin 10 */
  IMX_PADMUX_DISP0_DATA17_INDEX,     /* GPIO5 Pin 11 */
  IMX_PADMUX_DISP0_DATA18_INDEX,     /* GPIO5 Pin 12 */
  IMX_PADMUX_DISP0_DATA19_INDEX,     /* GPIO5 Pin 13 */
  IMX_PADMUX_DISP0_DATA20_INDEX,     /* GPIO5 Pin 14 */
  IMX_PADMUX_DISP0_DATA21_INDEX,     /* GPIO5 Pin 15 */

  IMX_PADMUX_DISP0_DATA22_INDEX,     /* GPIO5 Pin 16 */
  IMX_PADMUX_DISP0_DATA23_INDEX,     /* GPIO5 Pin 17 */
  IMX_PADMUX_CSI0_PIXCLK_INDEX,      /* GPIO5 Pin 18 */
  IMX_PADMUX_CSI0_HSYNC_INDEX,       /* GPIO5 Pin 19 */
  IMX_PADMUX_CSI0_DATA_EN_INDEX,     /* GPIO5 Pin 20 */
  IMX_PADMUX_CSI0_VSYNC_INDEX,       /* GPIO5 Pin 21 */
  IMX_PADMUX_CSI0_DATA04_INDEX,      /* GPIO5 Pin 22 */
  IMX_PADMUX_CSI0_DATA05_INDEX,      /* GPIO5 Pin 23 */

  IMX_PADMUX_CSI0_DATA06_INDEX,      /* GPIO5 Pin 24 */
  IMX_PADMUX_CSI0_DATA07_INDEX,      /* GPIO5 Pin 25 */
  IMX_PADMUX_CSI0_DATA08_INDEX,      /* GPIO5 Pin 26 */
  IMX_PADMUX_CSI0_DATA09_INDEX,      /* GPIO5 Pin 27 */
  IMX_PADMUX_CSI0_DATA10_INDEX,      /* GPIO5 Pin 28 */
  IMX_PADMUX_CSI0_DATA11_INDEX,      /* GPIO5 Pin 29 */
  IMX_PADMUX_CSI0_DATA12_INDEX,      /* GPIO5 Pin 30 */
  IMX_PADMUX_CSI0_DATA13_INDEX,      /* GPIO5 Pin 31 */
};

static const uint8_t g_gpio6_padmux[IMX_GPIO_NPINS] =
{
  IMX_PADMUX_CSI0_DATA14_INDEX,      /* GPIO6 Pin 0 */
  IMX_PADMUX_CSI0_DATA15_INDEX,      /* GPIO6 Pin 1 */
  IMX_PADMUX_CSI0_DATA16_INDEX,      /* GPIO6 Pin 2 */
  IMX_PADMUX_CSI0_DATA17_INDEX,      /* GPIO6 Pin 3 */
  IMX_PADMUX_CSI0_DATA18_INDEX,      /* GPIO6 Pin 4 */
  IMX_PADMUX_CSI0_DATA19_INDEX,      /* GPIO6 Pin 5 */
  IMX_PADMUX_EIM_ADDR23_INDEX,       /* GPIO6 Pin 6 */
  IMX_PADMUX_NAND_CLE_INDEX,         /* GPIO6 Pin 7 */

  IMX_PADMUX_NAND_ALE_INDEX,         /* GPIO6 Pin 8 */
  IMX_PADMUX_NAND_WP_INDEX,          /* GPIO6 Pin 9 */
  IMX_PADMUX_NAND_READY_INDEX,       /* GPIO6 Pin 10 */
  IMX_PADMUX_NAND_CS0_INDEX,         /* GPIO6 Pin 11 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 12 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 13 */
  IMX_PADMUX_NAND_CS1_INDEX,         /* GPIO6 Pin 14 */
  IMX_PADMUX_NAND_CS2_INDEX,         /* GPIO6 Pin 15 */

  IMX_PADMUX_NAND_CS3_INDEX,         /* GPIO6 Pin 16 */
  IMX_PADMUX_SD3_DATA7_INDEX,        /* GPIO6 Pin 17 */
  IMX_PADMUX_SD3_DATA6_INDEX,        /* GPIO6 Pin 18 */
  IMX_PADMUX_RGMII_TXC_INDEX,        /* GPIO6 Pin 19 */
  IMX_PADMUX_RGMII_TD0_INDEX,        /* GPIO6 Pin 20 */
  IMX_PADMUX_RGMII_TD1_INDEX,        /* GPIO6 Pin 21 */
  IMX_PADMUX_RGMII_TD2_INDEX,        /* GPIO6 Pin 22 */
  IMX_PADMUX_RGMII_TD3_INDEX,        /* GPIO6 Pin 23 */

  IMX_PADMUX_RGMII_RX_CTL_INDEX,     /* GPIO6 Pin 24 */
  IMX_PADMUX_RGMII_RD0_INDEX,        /* GPIO6 Pin 25 */
  IMX_PADMUX_RGMII_TX_CTL_INDEX,     /* GPIO6 Pin 26 */
  IMX_PADMUX_RGMII_RD1_INDEX,        /* GPIO6 Pin 27 */
  IMX_PADMUX_RGMII_RD2_INDEX,        /* GPIO6 Pin 28 */
  IMX_PADMUX_RGMII_RD3_INDEX,        /* GPIO6 Pin 29 */
  IMX_PADMUX_RGMII_RXC_INDEX,        /* GPIO6 Pin 30 */
  IMX_PADMUX_EIM_BCLK_INDEX,         /* GPIO6 Pin 31 */
};

static const uint8_t g_gpio7_padmux[IMX_GPIO_NPINS] =
{
  IMX_PADMUX_SD3_DATA5_INDEX,        /* GPIO7 Pin 0 */
  IMX_PADMUX_SD3_DATA4_INDEX,        /* GPIO7 Pin 1 */
  IMX_PADMUX_SD3_CMD_INDEX,          /* GPIO7 Pin 2 */
  IMX_PADMUX_SD3_CLK_INDEX,          /* GPIO7 Pin 3 */
  IMX_PADMUX_SD3_DATA0_INDEX,        /* GPIO7 Pin 4 */
  IMX_PADMUX_SD3_DATA1_INDEX,        /* GPIO7 Pin 5 */
  IMX_PADMUX_SD3_DATA2_INDEX,        /* GPIO7 Pin 6 */
  IMX_PADMUX_SD3_DATA3_INDEX,        /* GPIO7 Pin 7 */

  IMX_PADMUX_SD3_RESET_INDEX,        /* GPIO7 Pin 8 */
  IMX_PADMUX_SD4_CMD_INDEX,          /* GPIO7 Pin 9 */
  IMX_PADMUX_SD4_CLK_INDEX,          /* GPIO7 Pin 10 */
  IMX_PADMUX_GPIO16_INDEX,           /* GPIO7 Pin 11 */
  IMX_PADMUX_GPIO17_INDEX,           /* GPIO7 Pin 12 */
  IMX_PADMUX_GPIO18_INDEX,           /* GPIO7 Pin 13 */
  IMX_PADMUX_INVALID,                /* GPIO7 Pin 14 */
  IMX_PADMUX_INVALID,                /* GPIO7 Pin 15 */

  IMX_PADMUX_INVALID,                /* GPIO6 Pin 16 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 17 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 18 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 19 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 20 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 21 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 22 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 23 */

  IMX_PADMUX_INVALID,                /* GPIO6 Pin 24 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 25 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 26 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 27 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 28 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 29 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 30 */
  IMX_PADMUX_INVALID,                /* GPIO6 Pin 31 */
};

static FAR const uint8_t *g_gpio_padmux[IMX_GPIO_NPORTS+1] =
{
  g_gpio1_padmux,                    /* GPIO1 */
  g_gpio2_padmux,                    /* GPIO2 */
  g_gpio3_padmux,                    /* GPIO3 */
  g_gpio4_padmux,                    /* GPIO4 */
  g_gpio5_padmux,                    /* GPIO5 */
  g_gpio6_padmux,                    /* GPIO6 */
  g_gpio7_padmux,                    /* GPIO7 */
  NULL                               /* GPIO8 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_gpio_dirout
 ****************************************************************************/

static inline void imx_gpio_dirout(int port, int pin)
{
  uint32_t regval = getreg32(IMX_GPIO_GDIR(port));
  regval |= GPIO_PIN(pin);
  putreg32(regval, IMX_GPIO_GDIR(port));
}

/****************************************************************************
 * Name: imx_gpio_dirin
 ****************************************************************************/

static inline void imx_gpio_dirin(int port, int pin)
{
  uint32_t regval = getreg32(IMX_GPIO_GDIR(port));
  regval &= ~GPIO_PIN(pin);
  putreg32(regval, IMX_GPIO_GDIR(port));
}

/****************************************************************************
 * Name: imx_gpio_setoutput
 ****************************************************************************/

static void imx_gpio_setoutput(int port, int pin, bool value)
{
  uintptr_t regaddr = IMX_GPIO_DR(port);
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
 * Name: imx_gpio_getinput
 ****************************************************************************/

static inline bool imx_gpio_getinput(int port, int pin)
{
  uintptr_t regaddr = IMX_GPIO_DR(port);
  uint32_t regval;

  regval = getreg32(regaddr);
  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: imx_gpio_configinput
 ****************************************************************************/

static int imx_gpio_configinput(gpio_pinset_t pinset)
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  FAR const uint8_t *table;
  iomux_pinset_t ioset;
  uintptr_t regaddr;
  unsigned int index;

  /* Configure pin as in input */

  imx_gpio_dirin(port, pin);

  /* Configure pin as a GPIO */

  table = g_gpio_padmux[port];
  if (table == NULL)
    {
      return -EINVAL;
    }

  index = (unsigned int)table[pin];
  if (index >= IMX_PADMUX_NREGISTERS)
    {
      return -EINVAL;
    }

  regaddr = IMX_PADMUX_ADDRESS(index);
  putreg32(PADMUX_MUXMODE_ALT5, regaddr);

  /* Configure pin pad settings */

  index = imx_padmux_map(index);
  if (index >= IMX_PADCTL_NREGISTERS)
    {
      return -EINVAL;
    }

  regaddr = IMX_PADCTL_ADDRESS(index);
  ioset   = (iomux_pinset_t)((pinset & GPIO_IOMUX_MASK) >> GPIO_IOMUX_SHIFT);
  return imx_iomux_configure(regaddr, ioset);
}

/****************************************************************************
 * Name: imx_gpio_configoutput
 ****************************************************************************/

static inline int imx_gpio_configoutput(gpio_pinset_t pinset)
{
  int port   = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin    = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value = ((pinset & GPIO_OUTPUT_ONE) != 0);

  /* Set the output value */

  imx_gpio_setoutput(port, pin, value);

  /* Convert the configured input GPIO to an output */

  imx_gpio_dirout(port, pin);
  return OK;
}

/****************************************************************************
 * Name: imx_gpio_configperiph
 ****************************************************************************/

static inline int imx_gpio_configperiph(gpio_pinset_t pinset)
{
  iomux_pinset_t ioset;
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t value;
  unsigned int index;

  /* Configure pin as a peripheral */

  index = ((pinset & GPIO_PADMUX_MASK) >> GPIO_PADMUX_SHIFT);
  regaddr = IMX_PADMUX_ADDRESS(index);

  value = ((pinset & GPIO_ALT_MASK) >> GPIO_ALT_SHIFT);
  regval = (value << PADMUX_MUXMODE_SHIFT);

  putreg32(regval, regaddr);

  /* Configure pin pad settings */

  index = imx_padmux_map(index);
  if (index >= IMX_PADCTL_NREGISTERS)
    {
      return -EINVAL;
    }

  regaddr = IMX_PADCTL_ADDRESS(index);
  ioset   = (iomux_pinset_t)((pinset & GPIO_IOMUX_MASK) >> GPIO_IOMUX_SHIFT);
  return imx_iomux_configure(regaddr, ioset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on pin-encoded description of the pin.
 *
 ****************************************************************************/

int imx_config_gpio(gpio_pinset_t pinset)
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

          ret = imx_gpio_configinput(pinset);
        }
        break;

      case GPIO_OUTPUT:
        {
          /* First coonfigure the pin as a GPIO input to avoid output
           * glitches.
           */

          ret = imx_gpio_configinput(pinset);
          if (ret >= 0)
            {
              /* Convert the input to an output */

              ret = imx_gpio_configoutput(pinset);
            }
        }
        break;

      case GPIO_PERIPH:
        {
          /* Configure the pin as a peripheral */

          ret = imx_gpio_configperiph(pinset);
        }
        break;

      default:
        ret = -EINVAL;
        break;
    }

  leave_critical_section(flags);
  return ret;
}

/************************************************************************************
 * Name: imx_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void imx_gpio_write(gpio_pinset_t pinset, bool value)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  flags = enter_critical_section();
  imx_gpio_setoutput(port, pin, value);
  leave_critical_section(flags);
}

/************************************************************************************
 * Name: imx_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool imx_gpio_read(gpio_pinset_t pinset)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value;

  flags = enter_critical_section();
  value = imx_gpio_getinput(port, pin);
  leave_critical_section(flags);
  return value;
}
