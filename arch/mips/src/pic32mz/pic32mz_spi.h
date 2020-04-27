/************************************************************************************
 * arch/mips/src/pic32mz/pic32mz_spi.h
 *
 *   Copyright (C) 2015, 2016 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_SPI_H
#define __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_SPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

struct spi_dev_s;  /* Forward reference */

/****************************************************************************
 * Name: pic32mz_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *pic32mz_spibus_initialize(int port);

/************************************************************************************
 * Name:  pic32mz_spiNselect, pic32mz_spiNstatus, and pic32mz_spiNcmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   including pic32mz_spibus_initialize()) are provided by common PIC32MZ logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in pic32mz_boardinitialize() to configure SPI/SSP chip select
 *      pins.
 *   2. Provide pic32mz_spiNselect() and pic32mz_spiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      pic32mz_spiNcmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a call to pic32mz_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by pic32mz_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI1
void pic32mz_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected);
uint8_t pic32mz_spi1status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI2
void pic32mz_spi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected);
uint8_t pic32mz_spi2status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI3
void pic32mz_spi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected);
uint8_t pic32mz_spi3status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI4
void pic32mz_spi4select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected);
uint8_t pic32mz_spi4status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi4cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI5
void pic32mz_spi5select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected);
uint8_t pic32mz_spi5status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi5cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_PIC32MZ_SPI6
void pic32mz_spi6select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected);
uint8_t pic32mz_spi6status(FAR struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int pic32mz_spi6cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

/****************************************************************************
 * Name: pic32mz_spi1/2/...register
 *
 * Description:
 *   If the board supports a card detect callback to inform the SPI-based
 *   MMC/SD driver when an SD card is inserted or removed, then
 *   CONFIG_SPI_CALLBACK should be defined and the following function(s)
 *   must be implemented.  These functions implements the registercallback
 *   method of the SPI interface (see include/nuttx/spi/spi.h for details)
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The function to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CALLBACK
#ifdef CONFIG_PIC32MZ_SPI1
int pic32mz_spi1register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                         FAR void *arg);
#endif

#ifdef CONFIG_PIC32MZ_SPI2
int pic32mz_spi2register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                         FAR void *arg);
#endif

#ifdef CONFIG_PIC32MZ_SPI3
int pic32mz_spi3register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                         FAR void *arg);
#endif

#ifdef CONFIG_PIC32MZ_SPI4
int pic32mz_spi4register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                         FAR void *arg);
#endif

#ifdef CONFIG_PIC32MZ_SPI5
int pic32mz_spi5register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                         FAR void *arg);
#endif

#ifdef CONFIG_PIC32MZ_SPI6
int pic32mz_spi6register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                         FAR void *arg);
#endif
#endif /* CONFIG_SPI_CALLBACK */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_SPI_H */
