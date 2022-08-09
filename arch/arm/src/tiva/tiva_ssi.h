/****************************************************************************
 * arch/arm/src/tiva/tiva_ssi.h
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
struct spi_dev_s *tiva_ssibus_initialize(int port);

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
void tiva_ssiselect(struct spi_dev_s *dev, uint32_t devid,
                    bool selected);
uint8_t tiva_ssistatus(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int tiva_ssicmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_TIVA_TIVA_SSI_H */
