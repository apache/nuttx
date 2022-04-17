/****************************************************************************
 * boards/arm/kl/freedom-kl25z/src/kl_spi.c
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
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>

#include "kl_gpio.h"
#include "kl_spi.h"
#include "freedom-kl25z.h"

#if defined(CONFIG_KL_SPI0) || defined(CONFIG_KL_SPI1)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kl_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the FRDM-KL25Z board.
 *
 ****************************************************************************/

void weak_function kl_spidev_initialize(void)
{
  /* Configure SPI0 chip selects */

#ifdef CONFIG_KL_SPI0
# ifdef CONFIG_ADXL345_SPI
  kl_configgpio(GPIO_ADXL345_CS);
#endif
#endif

  /* Configure SPI1 chip selects */

#ifdef CONFIG_KL_SPI1
#endif
}

/****************************************************************************
 * Name:  kl_spi[n]select, kl_spi[n]status, and kl_spi[n]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They
 *   are implementations of the select, status, and cmddata methods of the
 *   SPI interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods including kl_spibus_initialize()) are provided by
 *   common Kinetis logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in kl_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide kl_spi[n]select() and kl_spi[n]status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board
 *      is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      kl_spi[n]cmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   3. Add a call to kl_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by kl_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

/****************************************************************************
 * Name: kl_spi[n]select
 *
 * Description:
 *   PIO chip select pins may be programmed by the board specific logic in
 *   one of two different ways.  First, the pins may be programmed as SPI
 *   peripherals.  In that case, the pins are completely controlled by the
 *   SPI driver.  This method still needs to be provided, but it may be only
 *   a stub.
 *
 *   An alternative way to program the PIO chip select pins is as a normal
 *   GPIO output.  In that case, the automatic control of the CS pins is
 *   bypassed and this function must provide control of the chip select.
 *   NOTE:  In this case, the GPIO output pin does *not* have to be the
 *   same as the NPCS pin normal associated with the chip select number.
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *   selected - TRUE:Select the device, FALSE:De-select the device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_KL_SPI0
void kl_spi0select(struct spi_dev_s *dev, uint32_t devid,
                   bool selected)
{
  spiinfo("devid: %d CS: %s\n",
           (int)devid, selected ? "assert" : "de-assert");

#ifdef CONFIG_ADXL345_SPI
  if (devid == SPIDEV_ACCELEROMETER(0))
    {
      /* Active low */

      kl_gpiowrite(GPIO_ADXL345_CS, !selected);
    }
#endif
}
#endif

#ifdef CONFIG_KL_SPI1
void kl_spi1select(struct spi_dev_s *dev, uint32_t devid,
                   bool selected)
{
  spiinfo("devid: %d CS: %s\n",
           (int)devid, selected ? "assert" : "de-assert");
}
#endif

/****************************************************************************
 * Name: kl_spi[n]status
 *
 * Description:
 *   Return status information associated with the SPI device.
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *
 * Returned Value:
 *   Bit-encoded SPI status (see include/nuttx/spi/spi.h.
 *
 ****************************************************************************/

#ifdef CONFIG_KL_SPI0
uint8_t kl_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_KL_SPI1
uint8_t kl_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: kl_spi[n]cmddata
 *
 * Description:
 *   Some SPI interfaces, particularly with LCDs, and an auxiliary 9th data
 *   input that determines where the other 8 data bits represent command or
 *   data.  These interfaces control that CMD/DATA GPIO output
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *   cmd - Determines where command or data should be selected.
 *
 * Returned Value:
 *   Bit-encoded SPI status (see include/nuttx/spi/spi.h.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_KL_SPI0
int kl_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return 0;
}
#endif

#ifdef CONFIG_KL_SPI1
int kl_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return 0;
}
#endif
#endif

#endif /* CONFIG_KL_SPI */
