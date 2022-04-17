/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_spi_master.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC54_SPI_MASTER_H
#define __ARCH_ARM_SRC_LPC54XX_LPC54_SPI_MASTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

#ifdef HAVE_SPI_MASTER_DEVICE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This header file defines interfaces to common SPI logic.
 * To use this common SPI logic on your board:
 *
 * 1. Provide logic in lpc54_boardinitialize() to configure SPI chip select
 *    pins.
 * 2. Provide the lpc54_spiselect() and lpc54_spistatus() functions in your
 *    board-specific logic.  These functions will perform chip selection
 *    and status operations using GPIOs in the way your board is configured.
 * 3. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *    lpc54_spicmddata() functions in your board-specific logic.  This
 *    function will perform cmd/data selection operations using GPIOs in the
 *    way your board is configured.
 * 4. Your low level board initialization logic should call
 *    lpc54_spibus_initialize.
 * 5. The handle returned by lpc54_spibus_initialize() may then be used to
 *    bind the SPI driver to higher level logic (e.g., calling
 *    mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *    the SPI MMC/SD driver).
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *   0 - SPI0
 *   1 - SPI1
 *   ...
 *   9 - SPI9
 *
 * Input Parameters:
 *   port - SPI peripheral number, 0.. 9.
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *lpc54_spibus_initialize(int port);

/****************************************************************************
 * Name:  lpc54_spiN_select, lpc54_spiN_status, and lpc54_spiN_cmddata
 *
 * Description:
 *   These functions must be provided in your board-specific logic.  The
 *   lpc54_spiN_select function will perform chip selection and the
 *   lpc54_spiN_status will perform status operations using GPIOs in the
 *   way your board is configured.
 *
 *   If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, then
 *   lpc54_spiN_cmddata must also be provided.  This functions performs
 *   cmd/data selection operations using GPIOs in the way your board is
 *   configured.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC54_SPI0_MASTER
void  lpc54_spi0_select(struct spi_dev_s *dev,
                        uint32_t devid, bool selected);
uint8_t lpc54_spi0_status(struct spi_dev_s *dev, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA
int lpc54_spi0_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_LPC54_SPI1_MASTER
void  lpc54_spi1_select(struct spi_dev_s *dev,
                        uint32_t devid, bool selected);
uint8_t lpc54_spi1_status(struct spi_dev_s *dev, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA
int lpc54_spi1_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_LPC54_SPI2_MASTER
void  lpc54_spi2_select(struct spi_dev_s *dev,
                        uint32_t devid, bool selected);
uint8_t lpc54_spi2_status(struct spi_dev_s *dev, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA
int lpc54_spi2_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_LPC54_SPI3_MASTER
void  lpc54_spi3_select(struct spi_dev_s *dev,
                        uint32_t devid, bool selected);
uint8_t lpc54_spi3_status(struct spi_dev_s *dev, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA
int lpc54_spi3_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_LPC54_SPI4_MASTER
void  lpc54_spi4_select(struct spi_dev_s *dev,
                        uint32_t devid, bool selected);
uint8_t lpc54_spi4_status(struct spi_dev_s *dev, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA
int lpc54_spi4_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_LPC54_SPI5_MASTER
void  lpc54_spi5_select(struct spi_dev_s *dev,
                        uint32_t devid, bool selected);
uint8_t lpc54_spi5_status(struct spi_dev_s *dev, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA
int lpc54_spi5_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_LPC54_SPI6_MASTER
void  lpc54_spi6_select(struct spi_dev_s *dev,
                        uint32_t devid, bool selected);
uint8_t lpc54_spi6_status(struct spi_dev_s *dev, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA
int lpc54_spi6_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_LPC54_SPI7_MASTER
void  lpc54_spi7_select(struct spi_dev_s *dev,
                        uint32_t devid, bool selected);
uint8_t lpc54_spi7_status(struct spi_dev_s *dev, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA
int lpc54_spi7_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_LPC54_SPI8_MASTER
void  lpc54_spi8_select(struct spi_dev_s *dev,
                        uint32_t devid, bool selected);
uint8_t lpc54_spi8_status(struct spi_dev_s *dev, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA
int lpc54_spi8_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_LPC54_SPI9_MASTER
void  lpc54_spi9_select(struct spi_dev_s *dev,
                        uint32_t devid, bool selected);
uint8_t lpc54_spi9_status(struct spi_dev_s *dev, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA
int lpc54_spi9_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

/****************************************************************************
 * Name: lpc54_spiN_register
 *
 * Description:
 *   If the board supports a card detect callback to inform the SPI-based
 *   MMC/SD driver when an SD card is inserted or removed, then
 *   CONFIG_SPI_CALLBACK should be defined and the following function(s)
 *   must must be implemented.  These functions implements the
 *   registercallback method of the SPI interface
 *  (see include/nuttx/spi/spi.h for details)
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
#ifdef CONFIG_LPC54_SPI0_MASTER
int lpc54_spi0_register(struct spi_dev_s *dev,
                        spi_mediachange_t callback, void *arg);
#endif
#ifdef CONFIG_LPC54_SPI1_MASTER
int lpc54_spi1_register(struct spi_dev_s *dev,
                        spi_mediachange_t callback, void *arg);
#endif
#ifdef CONFIG_LPC54_SPI2_MASTER
int lpc54_spi2_register(struct spi_dev_s *dev,
                        spi_mediachange_t callback, void *arg);
#endif
#ifdef CONFIG_LPC54_SPI3_MASTER
int lpc54_spi3_register(struct spi_dev_s *dev,
                        spi_mediachange_t callback, void *arg);
#endif
#ifdef CONFIG_LPC54_SPI4_MASTER
int lpc54_spi4_register(struct spi_dev_s *dev,
                        spi_mediachange_t callback, void *arg);
#endif
#ifdef CONFIG_LPC54_SPI5_MASTER
int lpc54_spi5_register(struct spi_dev_s *dev,
                        spi_mediachange_t callback, void *arg);
#endif
#ifdef CONFIG_LPC54_SPI6_MASTER
int lpc54_spi6_register(struct spi_dev_s *dev,
                        spi_mediachange_t callback, void *arg);
#endif
#ifdef CONFIG_LPC54_SPI7_MASTER
int lpc54_spi7_register(struct spi_dev_s *dev,
                        spi_mediachange_t callback, void *arg);
#endif
#ifdef CONFIG_LPC54_SPI8_MASTER
int lpc54_spi8_register(struct spi_dev_s *dev,
                        spi_mediachange_t callback, void *arg);
#endif
#ifdef CONFIG_LPC54_SPI9_MASTER
int lpc54_spi9_register(struct spi_dev_s *dev,
                        spi_mediachange_t callback, void *arg);
#endif
#endif /* CONFIG_SPI_CALLBACK */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* HAVE_SPI_MASTER_DEVICE */
#endif /* __ARCH_ARM_SRC_LPC54XX_LPC54_SPI_MASTER_H */
