/****************************************************************************
 * boards/arm/rm57/rm57l843-launchxl2/src/rm57l843-launchxl2.h
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

#ifndef __BOARDS_ARM_RM57_RM57L843_LAUNCHXL2_SRC_RM57L843_LAUNCHXL2_H
#define __BOARDS_ARM_RM57_RM57L843_LAUNCHXL2_SRC_RM57L843_LAUNCHXL2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "rm57_gio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs */
#define GIO_LED_B6    (GIO_OUTPUT | GIO_CFG_DEFAULT | GIO_OUTPUT_SET | \
                       GIO_PORT_GIOB | GIO_PIN6)
#define GIO_LED_B7    (GIO_OUTPUT | GIO_CFG_DEFAULT | GIO_OUTPUT_SET | \
                       GIO_PORT_GIOB | GIO_PIN7)

/* Buttons */
#define GIO_BUTTON_B4 (GIO_INPUT | GIO_CFG_PULLUP | GIO_INT_BOTHEDGES | \
                       GIO_PORT_GIOB | GIO_PIN4)
#define GIO_BUTTON_B5 (GIO_INPUT | GIO_CFG_PULLUP | GIO_INT_BOTHEDGES | \
                       GIO_PORT_GIOB | GIO_PIN5)

#ifdef CONFIG_RM57_EMAC

/* Ethernet PHY control lines.
 *
 * Both of the DP83630's control inputs are wired to GIOA on this board
 * (docs/sprr397.pdf sheet 12) and both default to the state that keeps
 * the PHY switched off, so firmware has to drive them - see
 * rm57_phy_powerup() in src/rm57_ethernet.c.
 *
 * PWRDOWN/INTN (PHY pin 7) comes from GIOA[3] (ball E1) and additionally
 * carries a hard 2.2k pulldown (RP11B), so the pin has to be a push-pull
 * output driven high to bring the PHY out of power-down.
 *
 * RESET_N (PHY pin 29) comes from GIOA[4] (ball A6).  Nothing else on
 * the board drives it: there is no RC power-on-reset, no supervisor and
 * no pull-up, and GIOA[4] powers up as an input with a 20uA internal
 * pulldown - so out of reset the PHY is held in reset indefinitely.
 * Configured asserted (low) here; rm57_phy_powerup() releases it.
 */

#  define GIO_PHY_PWRDOWN (GIO_OUTPUT | GIO_CFG_DEFAULT | GIO_OUTPUT_SET | \
                           GIO_PORT_GIOA | GIO_PIN3)
#  define GIO_PHY_RESET   (GIO_OUTPUT | GIO_CFG_DEFAULT | \
                           GIO_OUTPUT_CLEAR | GIO_PORT_GIOA | GIO_PIN4)

#endif /* CONFIG_RM57_EMAC */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_bringup
 *
 * Description:
 *   Bring up board features.
 *
 ****************************************************************************/

int rm57_bringup(void);

/****************************************************************************
 * Name: rm57_can_setup
 *
 * Description:
 *   Register the enabled DCAN instances (character device and/or
 *   SocketCAN, per the RM57_DCANn Kconfig choices).
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_DCAN
int rm57_can_setup(void);
#endif

/****************************************************************************
 * Name: rm57_adc_setup
 *
 * Description:
 *   Register every enabled MibADC conversion group as its own /dev/adcN
 *   character device - see src/rm57_adc.c for the board's analog wiring.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_ADC
int rm57_adc_setup(void);
#endif

/****************************************************************************
 * Name: rm57_spi_setup
 *
 * Description:
 *   Initialize every enabled MibSPI bus, claim its chip select pins and,
 *   with CONFIG_SPI_DRIVER, register it as /dev/spiN - see src/rm57_spi.c
 *   for where the signals come out on the BoosterPack headers.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_MIBSPI
int rm57_spi_setup(void);
#endif

/****************************************************************************
 * Name: rm57_i2c_setup
 *
 * Description:
 *   Initialize every enabled I2C bus, run the digital loopback self-test
 *   on it with CONFIG_RM57_I2C_LOOPBACK and, with CONFIG_I2C_DRIVER,
 *   register it as /dev/i2cN - see src/rm57_i2c.c.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_I2C
int rm57_i2c_setup(void);
#endif

/****************************************************************************
 * Name: rm57_eth_setup
 *
 * Description:
 *   Derive a locally-administered MAC address from the device's unique
 *   die ID and register the EMAC driver.  Only built/called under
 *   CONFIG_NETDEV_LATEINIT - see src/rm57_ethernet.c.
 *
 ****************************************************************************/

#if defined(CONFIG_RM57_EMAC) && defined(CONFIG_NETDEV_LATEINIT)
int rm57_eth_setup(void);
#endif

/****************************************************************************
 * Name: rm57_phy_powerup
 *
 * Description:
 *   Release the Ethernet PHY from power-down and reset.  Must run before
 *   the EMAC driver first touches the MDIO bus - see the GIO_PHY_*
 *   definitions above and src/rm57_ethernet.c.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_EMAC
void rm57_phy_powerup(void);
#endif

#endif /* __BOARDS_ARM_RM57_RM57L843_LAUNCHXL2_SRC_RM57L843_LAUNCHXL2_H */
