/****************************************************************************
 * boards/arm/lpc214x/zp214xpa/src/lpc2148_ug2864ambag01.c
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

#include <debug.h>
#include <inttypes.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ug-2864ambag01.h>

#include "arm_internal.h"
#include "chip.h"
#include "lpc214x_pinsel.h"
#include "lpc214x_spi.h"

#ifdef CONFIG_LCD_UG2864AMBAG01

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* The pin configurations here requires that SPI1 is available */

/* SPI should be configured with CMD/DATA support (and no transfer methods) */

#ifndef CONFIG_SPI_CMDDATA
#  error "The OLED driver requires CONFIG_SPI_CMDDATA in the configuration"
#endif

/* Pin Configuration ********************************************************/

/* UG-2864AMBAG01 OLED Display:
 *
 *   PIN NAME PIN CONFIGURATION
 *    1  3V3
 *    2  5V
 *    3  RESET P0.18/CAP1.3/MISO1/MAT1.3P0.18 RESET - General purpose output
 *    4  DI    P0.19/MAT1.2/MOSI1/CAP1.2P0.19 DI    - Alternate function 2
 *    5  CS    P0.20/MAT1.3/SSEL1/EINT3             - General purpose output
 *    6  SCK   P0.17/CAP1.2/SCK1/MAT1.2             - Alternate function 2
 *    7  A0    P0.23/VBUS                           - General purpose output
 *    8  N/C   LED-
 *    9  N/C   LED+ (BL)
 *   10  GND
 *
 * Definitions and configuration for DO, DI, CS, in up_spi1.c
 */

/* Use either FIO or legacy GPIO */

#ifdef CONFIG_LPC214x_FIO
#  define RESET_PIN_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_PIN_OFFSET)
#  define RESET_SET_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_SET_OFFSET)
#  define RESET_CLR_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_CLR_OFFSET)
#  define RESET_DIR_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_DIR_OFFSET)
#else
#  define RESET_PIN_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_PIN_OFFSET)
#  define RESET_SET_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_SET_OFFSET)
#  define RESET_CLR_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_CLR_OFFSET)
#  define RESET_DIR_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_DIR_OFFSET)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_graphics_setup
 *
 * Description:
 *   Called by NX initialization logic to configure the OLED.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *board_graphics_setup(unsigned int devno)
{
  FAR struct spi_dev_s *spi;
  FAR struct lcd_dev_s *dev;
  uint32_t regval32;
  uint32_t bits32;

  /* Configure multiplexed pins as connected on the ZP213X/4XPA board:
   *
   *   PINSEL1 P0.18/CAP1.3/MISO1/MAT1.3 Bits 4-5=00 for P0.18
   */

  regval32  = getreg32(LPC214X_PINSEL1);
  regval32 &= ~LPC214X_PINSEL1_P018_MASK;
  regval32 |= LPC214X_PINSEL1_P018_GPIO;
  putreg32(regval32, LPC214X_PINSEL1);

  /* Set the RESET line low, putting the OLED into the reset state. */

  bits32 = (1 << 18);
  putreg32(bits32, RESET_CLR_REGISTER);
  regval32 = getreg32(RESET_DIR_REGISTER);
  putreg32(regval32 | bits32, RESET_DIR_REGISTER);

  lcdinfo("RESET Pin Config: PINSEL1: %08" PRIx32
          " PIN: %08" PRIx32 " DIR: %08" PRIx32 "\n",
          getreg32(LPC214X_PINSEL1), getreg32(RESET_PIN_REGISTER),
          getreg32(RESET_DIR_REGISTER));

  /* Wait a bit then release the OLED from the reset state */

  up_mdelay(20);
  putreg32(bits32, RESET_SET_REGISTER);

  lcdinfo("RESET release: PIN: %08" PRIx32 " DIR: %08" PRIx32 "\n",
          getreg32(RESET_PIN_REGISTER), getreg32(RESET_DIR_REGISTER));

  /* Get the SPI1 port interface */

  spi = lpc214x_spibus_initialize(1);
  if (!spi)
    {
      lcderr("ERROR: Failed to initialize SPI port 1\n");
    }
  else
    {
      /* Bind the SPI port to the OLED */

      dev = ug2864ambag01_initialize(spi, devno);
      if (!dev)
        {
          lcderr("ERROR: Failed to bind SPI port 1 to OLED %d\n", devno);
        }
     else
        {
          lcdinfo("Bound SPI port 1 to OLED %d\n", devno);

          /* And turn the OLED on */

          dev->setpower(dev, CONFIG_LCD_MAXPOWER);
          return dev;
        }
    }

  return NULL;
}
#endif /* CONFIG_LCD_UG2864AMBAG01 */
