/****************************************************************************
 * arch/arm/src/xmc4/xmc4_spi.h
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

#ifndef __ARCH_ARM_SRC_XMC4_XMC4_SPI_H
#define __ARCH_ARM_SRC_XMC4_XMC4_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/xmc4_usic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XMC4_SPI_MODE0 USIC_BRG_SCLKCFG_NOINVDLY
#define XMC4_SPI_MODE1 USIC_BRG_SCLKCFG_NOINVNODLY
#define XMC4_SPI_MODE2 USIC_BRG_SCLKCFG_INVDLY
#define XMC4_SPI_MODE3 USIC_BRG_SCLKCFG_INVNODLY

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
 * Public Data
 ****************************************************************************/

struct spi_dev_s;
enum usic_channel_e;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameter:
 *   channel number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *xmc4_spibus_initialize(int channel);

/****************************************************************************
 * Name:  xmc4_spi[n]select, xmc4_spi[n]status, and xmc4_spi[n]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.
 *   They are implementations of the select, status, and cmddata methods of
 *   SPI interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods including xmc4_spibus_initialize()) are provided by
 *   common XMC4 logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in xmc4_board_initialize() to configure SPI chip
 *      select pins.
 *   2. Provide xmc4_spi[n]select() and xmc4_spi[n]status() functions
 *      in your board-specific logic. These functions w/ perform chip
 *      selection and status operations using GPIOs in the way your board
 *      is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      xmc4_spi[n]cmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   4. Add a call to xmc4_spibus_initialize() in your low level application
 *      initialization logic
 *   5. The handle returned by xmc4_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_XMC4_SPI0
void  xmc4_spi0select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t xmc4_spi0status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int xmc4_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_XMC4_SPI1
void  xmc4_spi1select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t xmc4_spi1status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int xmc4_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_XMC4_SPI2
void  xmc4_spi2select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t xmc4_spi2status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int xmc4_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_XMC4_SPI3
void  xmc4_spi3select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t xmc4_spi3status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int xmc4_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_XMC4_SPI4
void  xmc4_spi4select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t xmc4_spi4status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int xmc4_spi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_XMC4_SPI5
void  xmc4_spi5select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t xmc4_spi5status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int xmc4_spi5cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

/****************************************************************************
 * Name: spi_flush
 *
 * Description:
 *   Flush and discard any words left in the RX FIFO.  This can be called
 *   from spi[n]select after a device is deselected (if you worry about such
 *   things).
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_XMC4_XMC4_SPI_H */
