/************************************************************************************
 * arch/z80/src/ez80/ez80_spi.h
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
 ************************************************************************************/

#ifndef __ARCH_Z80_SRC_EZ80_EZ80_SPI_H
#define __ARCH_Z80_SRC_EZ80_EZ80_SPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <nuttx/spi/spi.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* SPIC Registers  ******************************************************************/

/* Provided in ez80f9x.h */

/* SPIC Register Bit Definitions  ***************************************************/

/* Baud Rate Generator (BRG) H/L Register Definitions
 *
 * No bit definitions -- These two 8-bit registers set the 16-bit BRG divider value
 */

/* SPI Control (CTL) Register Definitions */

#define SPI_CTL_IRQEN    (1 << 7) /* Bit 7: 1=SPI system interrupt is enabled */
                                  /* Bit 6: Reserved */
#define SPI_CTL_SPIEN    (1 << 5) /* Bit 5: 1=SPI is enabled */
#define SPI_CTL_MASTEREN (1 << 4) /* Bit 4: 1=SPI operates as a master */
#define SPI_CTL_CPOL     (1 << 3) /* Bit 3: 1=Master SCK pin idles in a high (1) state */
#define SPI_CTL_CPHA     (1 << 2) /* Bit 2: 1=SS remains Low to transfer any number of data bytes */
                                  /* Bits 0-1: Reserved */

/* SR Register Definitions */

#define SPI_SR_SPIF      (1 << 7) /* Bit 7: 1=SPI data transfer is finished */
#define SPI_SR_WCOL      (1 << 6) /* Bit 6: 1=SPI write collision is detected*/
                                  /* Bit 5: Reserved */
#define SPI_SR_MODF      (1 << 4) /* Bit 4: 1=Mode fault (multimaster conflict) is detected */
                                  /* Bits 0-3: Reserved */

/* RBR/TSR Register Definitions */

/* No definitions: 8-bit SPI receive/transmit data */

/************************************************************************************
 * Public Types
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

/************************************************************************************
 * Name: ez80_spibus_initialize
 *
 * Description:
 *   Initialize common parts the selected SPI port.  Initialization of
 *   chip select GPIOs must have been performed by board specific logic
 *   prior to calling this function.  Specifically:  GPIOs should have
 *   been configured for output, and all chip selects disabled.
 *
 *   One GPIO, SS (PB2 on the eZ8F091) is reserved as a chip select.  However,
 *   If multiple devices on on the bus, then multiple chip selects will be
 *   required.  Therefore, all GPIO chip management is deferred to board-
 *   specific logic.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ************************************************************************************/

FAR struct spi_dev_s *ez80_spibus_initialize(int port);

/************************************************************************************
 * The external functions, ez80_spiselect, ez80_spistatus, and ez80_spicmddata must
 * be provided by board-specific logic.  These are implementations of the select,
 * status, and cmddata methods of the SPI interface defined by struct spi_ops_s (see
 * include/nuttx/spi/spi.h).  All other methods (including ez80_spibus_initialize())
 * are provided by common logic.  To use this common SPI logic on your board:
 *
 *   1. Provide ez80_spiselect() and ez80_spistatus() functions in your board-
 *      specific logic.  This function will perform chip selection and status
 *      operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration, provide the
 *      ez80_spiscmddata() function in your board-specific logic.  This function
 *      will perform cmd/data selection operations using GPIOs in the way your board
 *      is configured.
 *   3. Add a call to ez80_spibus_initialize() in your low level initialization logic
 *   4. The handle returned by ez80_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling  mmcsd_spislotinitialize(),
 *      for example, will bind the SPI driver to the SPI MMC/SD driver).
 *
 ************************************************************************************/

void ez80_spiselect(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t ez80_spistatus(FAR struct spi_dev_s *dev, uint32_t devid);
int ez80_spicmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);

#undef EXTERN
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_Z80_SRC_EZ80_EZ80_SPI_H */
