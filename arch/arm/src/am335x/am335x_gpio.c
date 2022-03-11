/****************************************************************************
 * arch/arm/src/am335x/am335x_gpio.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/irq.h>

#include "chip.h"
#include "arm_internal.h"
#include "am335x_pinmux.h"
#include "am335x_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AM335X_PADCTL_INVALID            255

/****************************************************************************
 * Private Data
 ****************************************************************************/

const uintptr_t g_gpiobase[AM335X_GPIO_NPORTS] =
{
  AM335X_GPIO0_VADDR,                    /* GPIO0 */
  AM335X_GPIO1_VADDR,                    /* GPIO1 */
  AM335X_GPIO2_VADDR,                    /* GPIO2 */
  AM335X_GPIO3_VADDR                     /* GPIO3 */
};

static const uint8_t g_gpio0_padctl[AM335X_GPIO_NPINS] =
{
  AM335X_PADCTL_MDIO_INDEX,              /* GPIO0 Pin 0 */
  AM335X_PADCTL_MDC_INDEX,               /* GPIO0 Pin 1 */
  AM335X_PADCTL_SPI0_SCLK_INDEX,         /* GPIO0 Pin 2 */
  AM335X_PADCTL_SPI0_D0_INDEX,           /* GPIO0 Pin 3 */
  AM335X_PADCTL_SPI0_D1_INDEX,           /* GPIO0 Pin 4 */
  AM335X_PADCTL_SPI0_CS0_INDEX,          /* GPIO0 Pin 5 */
  AM335X_PADCTL_SPI0_CS1_INDEX,          /* GPIO0 Pin 6 */
  AM335X_PADCTL_ECAP0_IN_PWM0_OUT_INDEX, /* GPIO0 Pin 7 */

  AM335X_PADCTL_LCD_DATA12_INDEX,        /* GPIO0 Pin 8 */
  AM335X_PADCTL_LCD_DATA13_INDEX,        /* GPIO0 Pin 9 */
  AM335X_PADCTL_LCD_DATA14_INDEX,        /* GPIO0 Pin 10 */
  AM335X_PADCTL_LCD_DATA15_INDEX,        /* GPIO0 Pin 11 */
  AM335X_PADCTL_UART1_CTSN_INDEX,        /* GPIO0 Pin 12 */
  AM335X_PADCTL_UART1_RTSN_INDEX,        /* GPIO0 Pin 13 */
  AM335X_PADCTL_UART1_RXD_INDEX,         /* GPIO0 Pin 14 */
  AM335X_PADCTL_UART1_TXD_INDEX,         /* GPIO0 Pin 15 */

  AM335X_PADCTL_MII1_TXD3_INDEX,         /* GPIO0 Pin 16 */
  AM335X_PADCTL_MII1_TXD2_INDEX,         /* GPIO0 Pin 17 */
  AM335X_PADCTL_USB0_DRVVBUS_INDEX,      /* GPIO0 Pin 18 */
  AM335X_PADCTL_XDMA_EVENT_INTR0_INDEX,  /* GPIO0 Pin 19 */
  AM335X_PADCTL_XDMA_EVENT_INTR1_INDEX,  /* GPIO0 Pin 20 */
  AM335X_PADCTL_MII1_TXD1_INDEX,         /* GPIO0 Pin 21 */
  AM335X_PADCTL_GPMC_AD8_INDEX,          /* GPIO0 Pin 22 */
  AM335X_PADCTL_GPMC_AD9_INDEX,          /* GPIO0 Pin 23 */

  AM335X_PADCTL_INVALID,                 /* GPIO0 Pin 24 */
  AM335X_PADCTL_INVALID,                 /* GPIO0 Pin 25 */
  AM335X_PADCTL_GPMC_AD10_INDEX,         /* GPIO0 Pin 26 */
  AM335X_PADCTL_GPMC_AD11_INDEX,         /* GPIO0 Pin 27 */
  AM335X_PADCTL_MII1_TXD0_INDEX,         /* GPIO0 Pin 28 */
  AM335X_PADCTL_RMII1_REF_CLK_INDEX,     /* GPIO0 Pin 29 */
  AM335X_PADCTL_GPMC_WAIT0_INDEX,        /* GPIO0 Pin 30 */
  AM335X_PADCTL_GPMC_WPN_INDEX           /* GPIO0 Pin 31 */
};

static const uint8_t g_gpio1_padctl[AM335X_GPIO_NPINS] =
{
  AM335X_PADCTL_GPMC_AD0_INDEX,          /* GPIO1 Pin 0 */
  AM335X_PADCTL_GPMC_AD1_INDEX,          /* GPIO1 Pin 1 */
  AM335X_PADCTL_GPMC_AD2_INDEX,          /* GPIO1 Pin 2 */
  AM335X_PADCTL_GPMC_AD3_INDEX,          /* GPIO1 Pin 3 */
  AM335X_PADCTL_GPMC_AD4_INDEX,          /* GPIO1 Pin 4 */
  AM335X_PADCTL_GPMC_AD5_INDEX,          /* GPIO1 Pin 5 */
  AM335X_PADCTL_GPMC_AD6_INDEX,          /* GPIO1 Pin 6 */
  AM335X_PADCTL_GPMC_AD7_INDEX,          /* GPIO1 Pin 7 */

  AM335X_PADCTL_UART0_CTSN_INDEX,        /* GPIO1 Pin 8 */
  AM335X_PADCTL_UART0_RTSN_INDEX,        /* GPIO1 Pin 9 */
  AM335X_PADCTL_UART0_RXD_INDEX,         /* GPIO1 Pin 10 */
  AM335X_PADCTL_UART0_TXD_INDEX,         /* GPIO1 Pin 11 */
  AM335X_PADCTL_GPMC_AD12_INDEX,         /* GPIO1 Pin 12 */
  AM335X_PADCTL_GPMC_AD13_INDEX,         /* GPIO1 Pin 13 */
  AM335X_PADCTL_GPMC_AD14_INDEX,         /* GPIO1 Pin 14 */
  AM335X_PADCTL_GPMC_AD15_INDEX,         /* GPIO1 Pin 15 */

  AM335X_PADCTL_GPMC_A0_INDEX,           /* GPIO1 Pin 16 */
  AM335X_PADCTL_GPMC_A1_INDEX,           /* GPIO1 Pin 17 */
  AM335X_PADCTL_GPMC_A2_INDEX,           /* GPIO1 Pin 18 */
  AM335X_PADCTL_GPMC_A3_INDEX,           /* GPIO1 Pin 19 */
  AM335X_PADCTL_GPMC_A4_INDEX,           /* GPIO1 Pin 20 */
  AM335X_PADCTL_GPMC_A5_INDEX,           /* GPIO1 Pin 21 */
  AM335X_PADCTL_GPMC_A6_INDEX,           /* GPIO1 Pin 22 */
  AM335X_PADCTL_GPMC_A7_INDEX,           /* GPIO1 Pin 23 */

  AM335X_PADCTL_GPMC_A8_INDEX,           /* GPIO1 Pin 24 */
  AM335X_PADCTL_GPMC_A9_INDEX,           /* GPIO1 Pin 25 */
  AM335X_PADCTL_GPMC_A10_INDEX,          /* GPIO1 Pin 26 */
  AM335X_PADCTL_GPMC_A11_INDEX,          /* GPIO1 Pin 27 */
  AM335X_PADCTL_GPMC_BEN1_INDEX,         /* GPIO1 Pin 28 */
  AM335X_PADCTL_GPMC_CSN0_INDEX,         /* GPIO1 Pin 29 */
  AM335X_PADCTL_GPMC_CSN1_INDEX,         /* GPIO1 Pin 30 */
  AM335X_PADCTL_GPMC_CSN2_INDEX,         /* GPIO1 Pin 31 */
};

static const uint8_t g_gpio2_padctl[AM335X_GPIO_NPINS] =
{
  AM335X_PADCTL_GPMC_CSN3_INDEX,         /* GPIO2 Pin 0 */
  AM335X_PADCTL_GPMC_CLK_INDEX,          /* GPIO2 Pin 1 */
  AM335X_PADCTL_GPMC_ADVN_ALE_INDEX,     /* GPIO2 Pin 2 */
  AM335X_PADCTL_GPMC_OEN_REN_INDEX,      /* GPIO2 Pin 3 */
  AM335X_PADCTL_GPMC_WEN_INDEX,          /* GPIO2 Pin 4 */
  AM335X_PADCTL_GPMC_BEN0_CLE_INDEX,     /* GPIO2 Pin 5 */
  AM335X_PADCTL_LCD_DATA0_INDEX,         /* GPIO2 Pin 6 */
  AM335X_PADCTL_LCD_DATA1_INDEX,         /* GPIO2 Pin 7 */

  AM335X_PADCTL_LCD_DATA2_INDEX,         /* GPIO2 Pin 8 */
  AM335X_PADCTL_LCD_DATA3_INDEX,         /* GPIO2 Pin 9 */
  AM335X_PADCTL_LCD_DATA4_INDEX,         /* GPIO2 Pin 10 */
  AM335X_PADCTL_LCD_DATA5_INDEX,         /* GPIO2 Pin 11 */
  AM335X_PADCTL_LCD_DATA6_INDEX,         /* GPIO2 Pin 12 */
  AM335X_PADCTL_LCD_DATA7_INDEX,         /* GPIO2 Pin 13 */
  AM335X_PADCTL_LCD_DATA8_INDEX,         /* GPIO2 Pin 14 */
  AM335X_PADCTL_LCD_DATA9_INDEX,         /* GPIO2 Pin 15 */

  AM335X_PADCTL_LCD_DATA10_INDEX,        /* GPIO2 Pin 16 */
  AM335X_PADCTL_LCD_DATA11_INDEX,        /* GPIO2 Pin 17 */
  AM335X_PADCTL_MII1_RXD3_INDEX,         /* GPIO2 Pin 18 */
  AM335X_PADCTL_MII1_RXD2_INDEX,         /* GPIO2 Pin 19 */
  AM335X_PADCTL_MII1_RXD1_INDEX,         /* GPIO2 Pin 20 */
  AM335X_PADCTL_MII1_RXD0_INDEX,         /* GPIO2 Pin 21 */
  AM335X_PADCTL_LCD_VSYNC_INDEX,         /* GPIO2 Pin 22 */
  AM335X_PADCTL_LCD_HSYNC_INDEX,         /* GPIO2 Pin 23 */

  AM335X_PADCTL_LCD_PCLK_INDEX,          /* GPIO2 Pin 24 */
  AM335X_PADCTL_LCD_AC_BIAS_EN_INDEX,    /* GPIO2 Pin 25 */
  AM335X_PADCTL_MMC0_DAT3_INDEX,         /* GPIO2 Pin 26 */
  AM335X_PADCTL_MMC0_DAT2_INDEX,         /* GPIO2 Pin 27 */
  AM335X_PADCTL_MMC0_DAT1_INDEX,         /* GPIO2 Pin 28 */
  AM335X_PADCTL_MMC0_DAT0_INDEX,         /* GPIO2 Pin 29 */
  AM335X_PADCTL_MMC0_CLK_INDEX,          /* GPIO2 Pin 30 */
  AM335X_PADCTL_MMC0_CMD_INDEX,          /* GPIO2 Pin 31 */
};

static const uint8_t g_gpio3_padctl[AM335X_GPIO_NPINS] =
{
  AM335X_PADCTL_MII1_COL_INDEX,          /* GPIO3 Pin 0 */
  AM335X_PADCTL_MII1_CRS_INDEX,          /* GPIO3 Pin 1 */
  AM335X_PADCTL_MII1_RX_ER_INDEX,        /* GPIO3 Pin 2 */
  AM335X_PADCTL_MII1_TX_EN_INDEX,        /* GPIO3 Pin 3 */
  AM335X_PADCTL_MII1_RX_DV_INDEX,        /* GPIO3 Pin 4 */
  AM335X_PADCTL_I2C0_SDA_INDEX,          /* GPIO3 Pin 5 */
  AM335X_PADCTL_I2C0_SCL_INDEX,          /* GPIO3 Pin 6 */
  AM335X_PADCTL_EMU0_INDEX,              /* GPIO3 Pin 7 */

  AM335X_PADCTL_EMU1_INDEX,              /* GPIO3 Pin 8 */
  AM335X_PADCTL_MII1_TX_CLK_INDEX,       /* GPIO3 Pin 9 */
  AM335X_PADCTL_MII1_RX_CLK_INDEX,       /* GPIO3 Pin 10 */
  AM335X_PADCTL_INVALID,                 /* GPIO3 Pin 11 */
  AM335X_PADCTL_INVALID,                 /* GPIO3 Pin 12 */
  AM335X_PADCTL_USB1_DRVVBUS_INDEX,      /* GPIO3 Pin 13 */
  AM335X_PADCTL_MCASP0_ACLKX_INDEX,      /* GPIO3 Pin 14 */
  AM335X_PADCTL_MCASP0_FSX_INDEX,        /* GPIO3 Pin 15 */

  AM335X_PADCTL_MCASP0_AXR0_INDEX,       /* GPIO3 Pin 16 */
  AM335X_PADCTL_MCASP0_AHCLKR_INDEX,     /* GPIO3 Pin 17 */
  AM335X_PADCTL_MCASP0_ACLKR_INDEX,      /* GPIO3 Pin 18 */
  AM335X_PADCTL_MCASP0_FSR_INDEX,        /* GPIO3 Pin 19 */
  AM335X_PADCTL_MCASP0_AXR1_INDEX,       /* GPIO3 Pin 20 */
  AM335X_PADCTL_MCASP0_AHCLKX_INDEX,     /* GPIO3 Pin 21 */
  AM335X_PADCTL_INVALID,                 /* GPIO3 Pin 22 */
  AM335X_PADCTL_INVALID,                 /* GPIO3 Pin 23 */

  AM335X_PADCTL_INVALID,                 /* GPIO3 Pin 24 */
  AM335X_PADCTL_INVALID,                 /* GPIO3 Pin 25 */
  AM335X_PADCTL_INVALID,                 /* GPIO3 Pin 26 */
  AM335X_PADCTL_INVALID,                 /* GPIO3 Pin 27 */
  AM335X_PADCTL_INVALID,                 /* GPIO3 Pin 28 */
  AM335X_PADCTL_INVALID,                 /* GPIO3 Pin 29 */
  AM335X_PADCTL_INVALID,                 /* GPIO3 Pin 30 */
  AM335X_PADCTL_INVALID,                 /* GPIO3 Pin 31 */
};

static const uint8_t *g_gpio_padctl[AM335X_GPIO_NPORTS] =
{
  g_gpio0_padctl,                    /* GPIO0 */
  g_gpio1_padctl,                    /* GPIO1 */
  g_gpio2_padctl,                    /* GPIO2 */
  g_gpio3_padctl,                    /* GPIO3 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_gpio_dirout
 ****************************************************************************/

static inline void am335x_gpio_dirout(int port, int pin)
{
  uint32_t regval = getreg32(AM335X_GPIO_OER(am335x_gpion_vbase(port)));
  regval &= ~GPIO_PIN(pin);
  putreg32(regval, AM335X_GPIO_OER(am335x_gpion_vbase(port)));
}

/****************************************************************************
 * Name: am335x_gpio_dirin
 ****************************************************************************/

static inline void am335x_gpio_dirin(int port, int pin)
{
  uint32_t regval = getreg32(AM335X_GPIO_OER(am335x_gpion_vbase(port)));
  regval |= GPIO_PIN(pin);
  putreg32(regval, AM335X_GPIO_OER(am335x_gpion_vbase(port)));
}

/****************************************************************************
 * Name: am335x_gpio_setoutput
 ****************************************************************************/

static void am335x_gpio_setoutput(int port, int pin, bool value)
{
  uint32_t regval = GPIO_PIN(pin);
  putreg32(regval, value ? AM335X_GPIO_SDOR(am335x_gpion_vbase(port)) :
                           AM335X_GPIO_CDOR(am335x_gpion_vbase(port)));
}

/****************************************************************************
 * Name: am335x_gpio_getinput
 ****************************************************************************/

static inline bool am335x_gpio_getinput(int port, int pin)
{
  uint32_t regval = getreg32(AM335X_GPIO_DIR(am335x_gpion_vbase(port)));
  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: am335x_gpio_configinput
 ****************************************************************************/

static int am335x_gpio_configinput(gpio_pinset_t pinset)
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  pinmux_pinset_t muxset;
  uintptr_t regaddr;
  uint8_t index;

  /* Configure pin as in input */

  am335x_gpio_dirin(port, pin);

  /* Configure pin interrupt levels */

  am335x_gpioirq(pinset);

  /* Configure pin as a GPIO */

  index = g_gpio_padctl[port][pin];
  if (index >= AM335X_PADCTL_NREGISTERS)
    {
      return -EINVAL;
    }

  /* Ensure that RX is enabled */

  pinset |= PINMUX_MODE7 | PINMUX_RX_ENABLE;

  /* Configure pin pad settings */

  regaddr = AM335X_PADCTL_ADDRESS(index);
  muxset  = (pinmux_pinset_t)((pinset & GPIO_PINMUX_MASK) >>
                               GPIO_PINMUX_SHIFT);
  return am335x_pinmux_configure(regaddr, muxset);
}

/****************************************************************************
 * Name: am335x_gpio_configoutput
 ****************************************************************************/

static inline int am335x_gpio_configoutput(gpio_pinset_t pinset)
{
  int port   = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin    = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value = ((pinset & GPIO_OUTPUT_ONE) != 0);

  /* Set the output value */

  am335x_gpio_setoutput(port, pin, value);

  /* Convert the configured input GPIO to an output */

  am335x_gpio_dirout(port, pin);
  return OK;
}

/****************************************************************************
 * Name: am335x_gpio_configperiph
 ****************************************************************************/

static inline int am335x_gpio_configperiph(gpio_pinset_t pinset)
{
  pinmux_pinset_t muxset;
  uintptr_t regaddr;
  unsigned int index;

  /* Configure pin as a peripheral */

  index   = ((pinset & GPIO_PADCTL_MASK) >> GPIO_PADCTL_SHIFT);
  regaddr = AM335X_PADCTL_ADDRESS(index);
  muxset  = (pinmux_pinset_t)((pinset & GPIO_PINMUX_MASK) >>
                               GPIO_PINMUX_SHIFT);
  return am335x_pinmux_configure(regaddr, muxset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on pin-encoded description of the pin.
 *
 ****************************************************************************/

int am335x_gpio_config(gpio_pinset_t pinset)
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

          ret = am335x_gpio_configinput(pinset);
        }
        break;

      case GPIO_OUTPUT:
        {
          /* First configure the pin as a GPIO input to avoid output
           * glitches.
           */

          ret = am335x_gpio_configinput(pinset);
          if (ret >= 0)
            {
              /* Convert the input to an output */

              ret = am335x_gpio_configoutput(pinset);
            }
        }
        break;

      case GPIO_PERIPH:
        {
          /* Configure the pin as a peripheral */

          ret = am335x_gpio_configperiph(pinset);
        }
        break;

      default:
        ret = -EINVAL;
        break;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: am335x_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void am335x_gpio_write(gpio_pinset_t pinset, bool value)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  flags = enter_critical_section();
  am335x_gpio_setoutput(port, pin, value);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: am335x_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool am335x_gpio_read(gpio_pinset_t pinset)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value;

  flags = enter_critical_section();
  value = am335x_gpio_getinput(port, pin);
  leave_critical_section(flags);
  return value;
}

/****************************************************************************
 * Name: am335x_periph_gpio
 *
 * Description:
 *   Return GPIO pinset that correspond to provided peripheral pinset.
 *
 ****************************************************************************/

gpio_pinset_t am335x_periph_gpio(gpio_pinset_t pinset)
{
  unsigned int index;
  int port;
  int pin;

  if ((pinset & GPIO_MODE_MASK) == GPIO_PERIPH)
    {
      index = ((pinset & GPIO_PADCTL_MASK) >> GPIO_PADCTL_SHIFT);
      for (port = 0; port < AM335X_GPIO_NPORTS; port++)
        {
          for (pin = 0; pin < AM335X_GPIO_NPINS; pin++)
            {
              if (index == g_gpio_padctl[port][pin])
                {
                  return (GPIO_INPUT | (port << GPIO_PORT_SHIFT) |
                          (pin << GPIO_PIN_SHIFT));
                }
            }
        }
    }

  return GPIO_MODE_MASK;
}
