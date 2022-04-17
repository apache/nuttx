/****************************************************************************
 * boards/mips/pic32mx/mirtoo/src/pic32_spi2.c
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
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "mips_internal.h"
#include "chip.h"
#include "pic32mx.h"
#include "pic32mx_pps.h"
#include "mirtoo.h"

#ifdef CONFIG_PIC32MX_SPI2

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Mirtoo module as two on-board SPI devices:
 *
 * SST25VF032B - 32-Mbit SPI Serial FLASH
 *
 * PGA117 - Zero drift programmable gain amplifier with MUX.
 * The PGA117 offers 10 analog inputs, a four-pin SPI interface with
 * daisy-chain capability, and hardware and software shutdown in a TSSOP-20
 * package.
 * Only 8 of the analog inputs (PORT0-7) are used on the Mirtoo module.
 *
 * Chip selects:
 *
 * ------ -------- ------------------------- --------------------------------
 *  PIN   SIGNAL  BOARD CONNECTION           NOTES
 * ------ -------- ------------------------- --------------------------------
 *  RPA1   SI     PGA117 and SST25VF032B     SPI2 data OUT (SDO2)
 *  RPA2   SO     PGA117 and SST25VF032B     R1, SPI2 data IN (SDI2)
 *  RPA3   SO     PGA117 and SST25VF032B     R0, SPI2 data IN (SDI2)
 *  SCK2   SCK    PGA117 and SST25VF032B     SPI2 clock
 *
 *  RB7   ~CSAI   PGA117                     PGA117 chip select (active low)
 *  RB13  ~CSM    SST25VF032B                SST25VF032B chip select
 *                                           (active low)
 */

#define GPIO_SI             (GPIO_OUTPUT|GPIO_PORTA|GPIO_PIN1)
#if CONFIG_MIRTOO_RELEASE == 1
#  define GPIO_SO           (GPIO_INPUT|GPIO_PORTA|GPIO_PIN2)
#else
#  define GPIO_SO           (GPIO_INPUT|GPIO_PORTA|GPIO_PIN3)
#endif
#define GPIO_SCK            (GPIO_OUTPUT|GPIO_PORTB|GPIO_PIN15)

#define GPIO_PGA117_CS      (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTB|GPIO_PIN7)
#define GPIO_SST25VF032B_CS (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTB|GPIO_PIN13)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_spi2initialize
 *
 * Description:
 *   Called to configure SPI2 chip select GPIO pins for the Mirtoo module.
 *
 ****************************************************************************/

void weak_function pic32mx_spi2initialize(void)
{
  /* Make sure that TRIS pins are set correctly.
   * Configure the SPI pins as digital
   * inputs and outputs first.
   */

  pic32mx_configgpio(GPIO_SI);
  pic32mx_configgpio(GPIO_SO);
  pic32mx_configgpio(GPIO_SCK);

  /* Configure SPI2 data in and data out to use RPA2 and 1, respectively */

  putreg32(PPS_INSEL_RPA2,  PIC32MX_PPS_SDI2R);
  putreg32(PPS_OUTSEL_SDO2, PIC32MX_PPS_RPA1R);

  /* Configure the SPI chip select GPIOs */

  pic32mx_configgpio(GPIO_PGA117_CS);
  pic32mx_configgpio(GPIO_SST25VF032B_CS);
}

/****************************************************************************
 * Name:  pic32mx_spi2select, pic32mx_spi2status, and pic32mx_spi2cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.
 *   They are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s
 *   (see include/nuttx/spi/spi.h).
 *   All other methods including pic32mx_spibus_initialize()) are provided by
 *   common PIC32MX logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in pic32mx_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide pic32mx_spiNselect() and pic32mx_spiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      pic32mx_spiNcmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   3. Add a call to pic32mx_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by pic32mx_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

struct spi_dev_s;

void  pic32mx_spi2select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  if (devid == SPIDEV_FLASH(0))
    {
      pic32mx_gpiowrite(GPIO_SST25VF032B_CS, !selected);
    }
  else if (devid == SPIDEV_MUX(0))
    {
      pic32mx_gpiowrite(GPIO_PGA117_CS, !selected);
    }
}

uint8_t pic32mx_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mx_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return 0;
}
#endif

#endif /* CONFIG_PIC32MX_SPI2 */
