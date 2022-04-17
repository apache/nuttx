/****************************************************************************
 * boards/mips/pic32mx/sure-pic32mx/src/pic32mx_spi.c
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
#include "sure-pic32mx.h"

#if defined(CONFIG_PIC32MX_SPI2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_DBDP11215

/* The Sure DB_DP11215 PIC32 Storage Demo Board has an SD slot
 * connected on SPI2:
 *
 *  SCK2/PMA5/CN8/RG6    SCK    SD connector SCK, FLASH (U1) SCK*
 *  SDI2/PMA4/CN9/RG7    SDI    SD connector DO, FLASH (U1) SO*
 *  SDO2/PMA3/CN10/RG8   SDO    SD connector DI, FLASH (U1) SI*
 *
 * Chip Select.  Pulled up on-board
 *  TDO/AN11/PMA12/RB11  SD_CS  SD connector CS
 *
 * Status inputs.  All pulled up on-board
 *
 *  TCK/AN12/PMA11/RB12  SD_CD  SD connector CD
 *  TDI/AN13/PMA10/RB13  SD_WD  SD connector WD
 */

#  define PIC32_HAVE_SD 1

#  define GPIO_SD_CS (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTB|GPIO_PIN11)
#  define GPIO_SD_CD (GPIO_INPUT|GPIO_INT|GPIO_PORTB|GPIO_PIN12)
#  define GPIO_SD_WD (GPIO_INPUT|GPIO_PORTB|GPIO_PIN13)

/* The Sure DB_DP11215 PIC32 Storage Demo Board has pads an SOIC (Flash or
 * EEPROM) connected on SPI2, however, U4 is not populated on my board.
 *
 *
 *  TMS/AN10/CVREFOUT/PMA13/RB10  UTIL_WP        FLASH (U1) WP
 *  SS2/PMA2/CN11/RG9             UTIL_CS        FLASH (U1) CS
 */

#  undef PIC32_HAVE_SOIC

#  define GPIO_SOIC_WP (GPIO_INPUT|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_SOIC_CS (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTG|GPIO_PIN0)

/* Change notification numbers -- Not available for SD_CD. */

#endif

#ifdef CONFIG_ARCH_DBDP11212

/* The Sure DB-DP11212 PIC32 General Purpose Demo Board does not have an
 * SD slot.
 */

#  undef PIC32_HAVE_SD

/* The Sure DB-DP11212 PIC32 General Purpose Demo Board has an SOIC (Flash or
 * EEPROM) connected on SPI2:
 *
 *  TMS/AN10/PMA13/RB10   UTIL_WP FLASH (U4) WP
 *  TDO/AN11/PMA12/RB11   UTIL_CS FLASH (U4) CS
 */

#  define PIC32_HAVE_SOIC 1

#  define GPIO_SOIC_WP (GPIO_INPUT|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_SOIC_CS (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTB|GPIO_PIN11)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Sure PIC32MX
 *   board.
 *
 ****************************************************************************/

void weak_function pic32mx_spidev_initialize(void)
{
  /* Configure the SPI2 chip select (CS) GPIO output, and the card detect
   * (CD) and write protect (WP) inputs.
   */

#ifdef PIC32_HAVE_SD
  pic32mx_configgpio(GPIO_SD_CS);
  pic32mx_configgpio(GPIO_SD_CD);
  pic32mx_configgpio(GPIO_SD_WD);
#endif

#ifdef PIC32_HAVE_SOIC
  pic32mx_configgpio(GPIO_SOIC_WP);
  pic32mx_configgpio(GPIO_SOIC_CS);
#endif
}

/****************************************************************************
 * Name:  pic32mx_spi2select and pic32mx_spi2status
 *
 * Description:
 *   The external functions, pic32mx_spi2select and pic32mx_spi2status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including pic32mx_spibus_initialize())
 *   are provided by common PIC32MX logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in pic32mx_boardinitialize() to configure SPI/SPI chip
 *      select pins.
 *   2. Provide pic32mx_spi2select() and pic32mx_spi2status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to pic32mx_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by pic32mx_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_PIC32MX_SPI2
void pic32mx_spi2select(struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");

  /* The SD card chip select is pulled high and active low */

#ifdef PIC32_HAVE_SD
  if (devid == SPIDEV_MMCSD(0))
    {
      pic32mx_gpiowrite(GPIO_SD_CS, !selected);
    }
#endif

#ifdef PIC32_HAVE_SOIC
  if (devid == SPIDEV_FLASH(0))
    {
      pic32mx_gpiowrite(GPIO_SOIC_CS, !selected);
    }
#endif
}

uint8_t pic32mx_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;

  /* Card detect is pull up on-board.  If a low value is sensed then the
   * card must be present.
   */

#ifdef PIC32_HAVE_SD
  if (devid == SPIDEV_MMCSD(0))
    {
      if (!pic32mx_gpioread(GPIO_SD_CD))
        {
          ret = SPI_STATUS_PRESENT;

          /* It seems that a high value indicates the card is write
           * protected.
           */

          if (pic32mx_gpioread(GPIO_SD_WD))
            {
              ret |= SPI_STATUS_WRPROTECTED;
            }
        }
    }
#endif

#ifdef PIC32_HAVE_SOIC
  if (devid == SPIDEV_FLASH(0))
    {
      ret = SPI_STATUS_PRESENT;

      /* Write protect is indicated with a low value. */

      if (pic32mx_gpioread(GPIO_SOIC_WP))
        {
          ret |= SPI_STATUS_WRPROTECTED;
        }
    }
#endif

  spiinfo("Returning %d\n", ret);
  return ret;
}
#endif
#endif /* CONFIG_PIC32MX_SPI2 */
