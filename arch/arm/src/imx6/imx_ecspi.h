/************************************************************************************
 * arch/arm/src/imx6/imx_ecspi.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_IMX6_ECSPI_H
#define __ARCH_ARM_IMX6_ECSPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/spi/spi.h>

#include "chip/imx_ecspi.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif /* __cplusplus */

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

struct spi_dev_s; /* Forward reference */

/************************************************************************************
 * Name: imx_spibus_initialize
 *
 * Description:
 *   Initialize common parts the selected SPI port.  Initialization of
 *   chip select GPIOs must have been performed by board specific logic
 *   prior to calling this function.  Specifically:  GPIOs should have
 *   been configured for output, and all chip selects disabled.
 *
 *   One GPIO, SS (PB2 on the eZ8F091) is reserved as a chip select.  However,
 *   If multiple devices on on the bus, then multiple chip selects will be
 *   required.  Theregore, all GPIO chip management is deferred to board-
 *   specific logic.
 *
 * Input Parameters:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structre reference on succcess; a NULL on failure
 *
 ************************************************************************************/

FAR struct spi_dev_s *imx_spibus_initialize(int port);

/************************************************************************************
 * The external functions, imx_spiselect, imx_spistatus, and imx_cmddaa must be
 * provided by board-specific logic.  These are implementations of the select and
 * status methods of the SPI interface defined by struct spi_ops_s (see
 * include/nuttx/spi/spi.h).  All other methods (including imx_spibus_initialize()) are
 * provided by common logic.  To use this common SPI logic on your board:
 *
 *   1. Provide imx_spiselect() and imx_spistatus() functions in your board-specific
 *      logic.  This function will perform chip selection and status operations using
 *      GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration, provide the
 *      imx_spicmddata() function in your board-specific logic.  This function will
 *      perform cmd/data selection operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a call to imx_spibus_initialize() in your low level initialization logic
 *   4. The handle returned by imx_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling  mmcsd_spislotinitialize(),
 *      for example, will bind the SPI driver to the SPI MMC/SD driver).
 *
 ************************************************************************************/

#ifdef CONFIG_IMX6_ECSPI1
void imx_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t imx_spi1status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int imx_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_IMX6_ECSPI2
void imx_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t imx_spi2status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int imx_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_IMX6_ECSPI3
void imx_spi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t imx_spi3status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int imx_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_IMX6_ECSPI4
void imx_spi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t imx_spi4status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int imx_spi4cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_IMX6_ECSPI5
void imx_spi5select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t imx_spi5status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int imx_spi5cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_ARM_IMX6_ECSPI_H */
