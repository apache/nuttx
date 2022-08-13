/****************************************************************************
 * boards/arm/samd2l2/arduino-m0/src/sam_spi.c
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
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>

#include "sam_config.h"
#include "sam_port.h"
#include "sam_spi.h"

#include "arduino_m0.h"

#ifdef SAMD2L2_HAVE_SPI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select PORT pins for the SAMD21 Xplained
 *   Pro board.
 *
 ****************************************************************************/

void weak_function sam_spidev_initialize(void)
{
}

/****************************************************************************
 * Name:  sam_spi[n]select, sam_spi[n]status, and sam_spi[n]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They
 *   include:
 *
 *   o sam_spi[n]select is a functions to manage the board-specific chip
 *     selects
 *   o sam_spi[n]status and sam_spi[n]cmddata:  Implementations of the status
 *     and cmddata methods of the SPI interface defined by struct spi_ops_
 *     (see include/nuttx/spi/spi.h). All other methods including
 *     sam_spibus_initialize()) are provided by common SAMD/L logic.
 *
 *   Where [n] is the SERCOM number for the SPI module.
 *
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in sam_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide sam_spi[n]select() and sam_spi[n]status() functions in your
 *      board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is
 *      configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      sam_spi[n]cmddata() functions in your board-specific logic.  This
 *      function will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   3. Add a call to sam_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by sam_spibus_initialize() may then be used to
 *      bind the  SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spi[n]select
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
 *   dev - SPI device info
 *   devid - Identifies the (logical) device
 *   selected - TRUE:Select the device, FALSE:De-select the device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef SAMD2L2_HAVE_SPI0
void sam_spi0select(struct spi_dev_s *dev, uint32_t devid,
                    bool selected)
{
}
#endif

#ifdef SAMD2L2_HAVE_SPI1
void sam_spi1select(struct spi_dev_s *dev, uint32_t devid,
                    bool selected)
{
}
#endif

#ifdef SAMD2L2_HAVE_SPI2
void sam_spi2select(struct spi_dev_s *dev, uint32_t devid,
                    bool selected)
{
}
#endif

#ifdef SAMD2L2_HAVE_SPI3
void sam_spi3select(struct spi_dev_s *dev, uint32_t devid,
                    bool selected)
{
}
#endif

#ifdef SAMD2L2_HAVE_SPI4
void sam_spi4select(struct spi_dev_s *dev, uint32_t devid,
                    bool selected)
{
}
#endif

#ifdef SAMD2L2_HAVE_SPI5
void sam_spi5select(struct spi_dev_s *dev, uint32_t devid,
                    bool selected)
{
}
#endif

/****************************************************************************
 * Name: sam_spi[n]status
 *
 * Description:
 *   Return status information associated with the SPI device.
 *
 * Input Parameters:
 *   dev - SPI device info
 *   devid - Identifies the (logical) device
 *
 * Returned Value:
 *   Bit-encoded SPI status (see include/nuttx/spi/spi.h.
 *
 ****************************************************************************/

#ifdef SAMD2L2_HAVE_SPI0
uint8_t sam_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef SAMD2L2_HAVE_SPI1
uint8_t sam_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef SAMD2L2_HAVE_SPI2
uint8_t sam_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef SAMD2L2_HAVE_SPI3
uint8_t sam_spi3status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef SAMD2L2_HAVE_SPI4
uint8_t sam_spi4status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef SAMD2L2_HAVE_SPI5
uint8_t sam_spi5status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t ret = 0;
  return ret;
}
#endif

/****************************************************************************
 * Name: sam_spi[n]cmddata
 *
 * Description:
 *   Some SPI devices require an additional control to determine if the SPI
 *   data being sent is a command or is data.  If CONFIG_SPI_CMDDATA then
 *   this function will be called to different be command and data transfers.
 *
 *   This is often needed, for example, by LCD drivers.  Some LCD hardware
 *   may be configured to use 9-bit data transfers with the 9th bit
 *   indicating command or data.  That same hardware may be configurable,
 *   instead, to use 8-bit data but to require an additional, board-
 *   specific GPIO control to distinguish command and data.  This function
 *   would be needed in that latter case.
 *
 * Input Parameters:
 *   dev - SPI device info
 *   devid - Identifies the (logical) device
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef SAMD2L2_HAVE_SPI0
int sam_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#ifdef SAMD2L2_HAVE_SPI1
int sam_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#ifdef SAMD2L2_HAVE_SPI2
int sam_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#ifdef SAMD2L2_HAVE_SPI3
int sam_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#ifdef SAMD2L2_HAVE_SPI4
int sam_spi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#ifdef SAMD2L2_HAVE_SPI5
int sam_spi5cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return OK;
}
#endif

#endif /* CONFIG_SPI_CMDDATA */
#endif /* SAMD2L2_HAVE_SPI */
