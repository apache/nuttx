/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_lpspi.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_S32K3XX_LPSPI_H
#define __ARCH_ARM_SRC_S32K3XX_S32K3XX_LPSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/spi/spi.h>

#include "chip.h"
#include "hardware/s32k3xx_lpspi.h"

/****************************************************************************
 * Public Functions Prototypes
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

struct spi_dev_s; /* Forward reference */

/****************************************************************************
 * Name: s32k3xx_lpspibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   bus number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *s32k3xx_lpspibus_initialize(int bus);

/****************************************************************************
 * Name:  s32k3xx_lpspi0/1/2/select and s32k3xx_lpspi0/1/2/status
 *
 * Description:
 *   The external functions, s32k3xx_lpspi001/2/select,
 *   s32k3xx_lpspi0/1/2/status, and s32k3xx_lpspi0/1/2/cmddata must be
 *   provided by board-specific logic. These are implementations of the
 *   select, status, and cmddata methods of the SPI interface defined by
 *   struct spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   (including s32k3xx_lpspibus_initialize()) are provided by common
 *   S32K3XX logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in s32k3xx_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide s32k3xx_lpspi0/1/2/select() and s32k3xx_lpspi0/1/2/status()
 *      functions in your  board-specific logic.  These functions will
 *      perform chip selection and status operations using GPIOs in the way
 *      your board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file,
 *      then provide s32k3xx_lpspi0/1/2/cmddata() functions in your
 *      board-specific logic. These functions will perform cmd/data selection
 *      operations using GPIOs in the way your board is configured.
 *   4. Add a calls to s32k3xx_lpspibus_initialize() in your low level
 *      application initialization logic
 *   5. The handle returned by s32k3xx_lpspibus_initialize() may then be
 *      used to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_LPSPI0
void s32k3xx_lpspi0select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected);
uint8_t s32k3xx_lpspi0status(struct spi_dev_s *dev, uint32_t devid);
int s32k3xx_lpspi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_S32K3XX_LPSPI1
void s32k3xx_lpspi1select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected);
uint8_t s32k3xx_lpspi1status(struct spi_dev_s *dev, uint32_t devid);
int s32k3xx_lpspi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_S32K3XX_LPSPI2
void s32k3xx_lpspi2select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected);
uint8_t s32k3xx_lpspi2status(struct spi_dev_s *dev, uint32_t devid);
int s32k3xx_lpspi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_S32K3XX_LPSPI3
void s32k3xx_lpspi3select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected);
uint8_t s32k3xx_lpspi3status(struct spi_dev_s *dev, uint32_t devid);
int s32k3xx_lpspi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_S32K3XX_LPSPI4
void s32k3xx_lpspi4select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected);
uint8_t s32k3xx_lpspi4status(struct spi_dev_s *dev, uint32_t devid);
int s32k3xx_lpspi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_S32K3XX_LPSPI5
void s32k3xx_lpspi5select(struct spi_dev_s *dev, uint32_t devid,
                          bool selected);
uint8_t s32k3xx_lpspi5status(struct spi_dev_s *dev, uint32_t devid);
int s32k3xx_lpspi5cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

/****************************************************************************
 * Name: s32k3xx_lpspi0/1/2/register
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
#ifdef CONFIG_S32K3XX_LPSPI0
int s32k3xx_lpspi0register(struct spi_dev_s *dev, spi_mediachange_t callback,
                           void *arg);
#endif

#ifdef CONFIG_S32K3XX_LPSPI1
int s32k3xx_lpspi1register(struct spi_dev_s *dev, spi_mediachange_t callback,
                           void *arg);
#endif

#ifdef CONFIG_S32K3XX_LPSPI2
int s32k3xx_lpspi2register(struct spi_dev_s *dev, spi_mediachange_t callback,
                           void *arg);
#endif
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_S32K3XX_S32K3XX_LPSPI_H */
