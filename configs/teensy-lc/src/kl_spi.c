/****************************************************************************
 * configs/teensy-lc/src/kl_spi.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>

#include "kl_gpio.h"
#include "kl_spi.h"
#include "teensy-lc.h"

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
 *   are implementations of the select, status, and cmddata methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All
 *   other methods including kl_spibus_initialize()) are provided by common
 *   Kinetis logic.  To use this common SPI logic on your board:
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
 *   4. The handle returned by kl_spibus_initialize() may then be used to bind
 *      the SPI driver to higher level logic (e.g., calling
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
void kl_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                   bool selected)
{
  spiinfo("devid: %d CS: %s\n",
           (int)devid, selected ? "assert" : "de-assert");
}
#endif

#ifdef CONFIG_KL_SPI1
void kl_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
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
uint8_t kl_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_KL_SPI1
uint8_t kl_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
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
int kl_spi0cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return 0;
}
#endif

#ifdef CONFIG_KL_SPI1
int kl_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return 0;
}
#endif
#endif

#endif /* CONFIG_KL_SPI */
