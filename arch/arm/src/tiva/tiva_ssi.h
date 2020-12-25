/****************************************************************************
 * arch/arm/src/tiva/tiva_ssi.h
 *
 *   Copyright (C) 2009-2010, 2013, 2016 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_SSI_H
#define __ARCH_ARM_SRC_TIVA_TIVA_SSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: tiva_ssibus_initialize
 *
 * Description:
 *   Initialize common parts the selected SPI port.  Initialization of chip
 *   select GPIOs must have been performed by board specific logic prior to
 *   calling this function.  Specifically:  GPIOs should have been
 *   configured for output, and all chip selects disabled.
 *
 *   One GPIO, SS (PB2 on the eZ8F091) is reserved as a chip select.
 *   However, If multiple devices on on the bus, then multiple chip selects
 *   will be required.  Therefore, all GPIO chip management is deferred to
 *   board- specific logic.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SSI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s;
FAR struct spi_dev_s *tiva_ssibus_initialize(int port);

/****************************************************************************
 * The external functions, tiva_ssiselect, tiva_ssistatus, and
 * tiva_ssicmddata must be provided by board-specific logic.  These are
 * implementations of the select, status, and cmddata methods of the SPI
 * interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All
 * other methods (including tiva_ssibus_initialize()) are provided by common
 * logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in tiva_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide tiva_ssiselect() and tiva_ssistatus() functions in your
 *      board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is
 *      configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration,
 *      provide the tiva_ssicmddata() function in your board-specific logic.
 *      This functions will perform cmd/data selection operations using
 *      GPIOs in the way your board is configured.
 *   4. Add a call to tiva_ssibus_initialize() in your low level application
 *      initialization logic
 *   5. The handle returned by tiva_ssibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

struct spi_dev_s;
void tiva_ssiselect(FAR struct spi_dev_s *dev, uint32_t devid,
                    bool selected);
uint8_t tiva_ssistatus(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int tiva_ssicmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_TIVA_TIVA_SSI_H */
