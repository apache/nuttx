/****************************************************************************
 * boards/arm/sam34/arduino-due/src/sam_mmcsd.c
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

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_bitbang.h>

#include "arm_internal.h"
#include "sam_gpio.h"
#include "hardware/sam3u_pio.h"

#include "arduino-due.h"

/* Include the bit-band skeleton logic */

#include <nuttx/spi/spi_bitbang.c>

/* In order to use the SD card on the ITEAD shield, you must enable the SPI
 * bit-bang driver as well as support for SPI-based MMC/SD cards.
 */

#if defined(CONFIG_ARDUINO_ITHEAD_TFT) && defined(CONFIG_SPI_BITBANG) && \
    defined(CONFIG_MMCSD_SPI)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error Mountpoints are disabled (CONFIG_DISABLE_MOUNTPOINT=y)
#endif

/* Definitions for include/nuttx/spi/spi_bitbang.c. */

#define SPI_SETSCK  putreg32(1 << 27, SAM_PIOB_SODR)
#define SPI_CLRSCK  putreg32(1 << 27, SAM_PIOB_CODR)
#define SPI_SETMOSI putreg32(1 << 7,  SAM_PIOD_SODR)
#define SPI_CLRMOSI putreg32(1 << 7,  SAM_PIOD_CODR)
#define SPI_GETMISO ((getreg32(SAM_PIOD_PDSR) >> 8) & 1)
#define SPI_SETCS   putreg32(1 << 28, SAM_PIOA_SODR)
#define SPI_CLRCS   putreg32(1 << 28, SAM_PIOA_CODR)

/* Only mode 0 */

#undef  SPI_BITBANG_DISABLEMODE0
#define SPI_BITBANG_DISABLEMODE1 1
#define SPI_BITBANG_DISABLEMODE2 1
#define SPI_BITBANG_DISABLEMODE3 1

/* Only 8-bit data width */

#undef SPI_BITBANG_VARWIDTH

/* Calibration value for timing loop */

#define SPI_BITBANG_LOOPSPERMSEC CONFIG_BOARD_LOOPSPERMSEC

/* SPI_PERBIT_NSEC is the minimum time to transfer one bit.  This determines
 * the maximum frequency and is also used to calculate delays to achieve
 * other SPI frequencies.
 */

#define SPI_PERBIT_NSEC      100

/* Misc definitions */

#define SAM34_MMCSDSLOTNO    0    /* There is only one slot */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void spi_select(struct spi_bitbang_s *priv, uint32_t devid,
                       bool selected);
static uint8_t spi_status(struct spi_bitbang_s *priv, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(struct spi_bitbang_s *priv, uint32_t devid,
                       bool cmd);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Select or de-selected the SPI device specified by 'devid'
 *
 * Input Parameters:
 *   priv     - An instance of the bit-bang driver structure
 *   devid    - The device to select or de-select
 *   selected - True:select false:de-select
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(struct spi_bitbang_s *priv, uint32_t devid,
                       bool selected)
{
  if (devid == SPIDEV_MMCSD(0))
    {
      if (selected)
        {
          SPI_CLRCS;
        }
      else
        {
          SPI_SETCS;
        }
    }
}

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Return status of the SPI device specified by 'devid'
 *
 * Input Parameters:
 *   priv     - An instance of the bit-bang driver structure
 *   devid    - The device to select or de-select
 *
 * Returned Value:
 *   An 8-bit, bit-encoded status byte
 *
 ****************************************************************************/

static uint8_t spi_status(struct spi_bitbang_s *priv, uint32_t devid)
{
  if (devid == SPIDEV_MMCSD(0))
    {
      return SPI_STATUS_PRESENT;
    }

  return 0;
}

/****************************************************************************
 * Name: spi_cmddata
 *
 * Description:
 *   If there were was a CMD/DATA line, this function would manage it
 *
 * Input Parameters:
 *   priv  - An instance of the bit-bang driver structure
 *   devid - The device to use
 *   cmd   - True=MCD false=DATA
 *
 * Returned Value:
 *  OK
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(struct spi_bitbang_s *priv, uint32_t devid,
                       bool cmd)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: sam_mmcsd_spiinitialize
 *
 * Description:
 *   Initialize the SPI bit-bang driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A non-NULL reference to the SPI driver on success
 *
 ****************************************************************************/

static struct spi_dev_s *sam_mmcsd_spiinitialize(void)
{
  /* Initialize GPIOs */

  sam_configgpio(GPIO_SD_SCK);
  sam_configgpio(GPIO_SD_MISO);
  sam_configgpio(GPIO_SD_MOSI);
  sam_configgpio(GPIO_SD_CS);

  /* Create the SPI driver instance */

  return spi_create_bitbang(&g_spiops);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.
 *
 ****************************************************************************/

int sam_sdinitialize(int minor)
{
  struct spi_dev_s *spi;
  int ret;

  /* Get the SPI driver instance for the SD chip select */

  finfo("Initializing bit bang SPI for the MMC/SD slot\n");

  spi = sam_mmcsd_spiinitialize();
  if (!spi)
    {
      ferr("ERROR: Failed to bit bang SPI for the MMC/SD slot\n");
      return -ENODEV;
    }

  finfo("Successfully initialized bit bang SPI for the MMC/SD slot\n");

  /* Bind the SPI device for the chip select to the slot */

  finfo("Binding bit bang SPI device to MMC/SD slot %d\n",
        SAM34_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(minor, SAM34_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      ferr("ERROR: Failed to bind bit bang SPI device"
            " to MMC/SD slot %d: %d\n",
            SAM34_MMCSDSLOTNO, ret);
      return ret;
    }

  finfo("Successfully bound  bit bang SPI device to MMC/SD slot %d\n",
        SAM34_MMCSDSLOTNO);

  return OK;
}

#endif /* CONFIG_ARDUINO_ITHEAD_TFT && CONFIG_SPI_BITBANG && CONFIG_MMC_SPI */
