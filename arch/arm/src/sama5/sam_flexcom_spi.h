/****************************************************************************
 * arch/arm/src/sama5/sam_flexcom_spi.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_FLEXCOM_SPI_H
#define __ARCH_ARM_SRC_SAMA5_SAM_FLEXCOM_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Flexcom SPI "port" number used as an input to sam_flexcom_spibus_
 * initialize encodes information about the Flexcom SPI controller (0 thru 4)
 * and the Flexcom SPI chip select (0 or 1)
 */

#define __FLEXCOM_SPI_CS_SHIFT  (0)      /* Bit 0: Flexcom SPI chip select number */
#define __FLEXCOM_SPI_CS_MASK   (1 << __FLEXCOM_SPI_CS_SHIFT)
#  define __FLEXCOM_SPI_CS0     (0 << __FLEXCOM_SPI_CS_SHIFT)
#  define __FLEXCOM_SPI_CS1     (1 << __FLEXCOM_SPI_CS_SHIFT)
#define __FLEXCOM_SPI_SPI_SHIFT (1)      /* Bit 1-3: Flexcom  number */
#define __FLEXCOM_SPI_SPI_MASK  (7 << __FLEXCOM_SPI_SPI_SHIFT)
#  define __FLEXCOM0_SPI        (0 << __FLEXCOM_SPI_SPI_SHIFT) /* FLEXCOM0 SPI */
#  define __FLEXCOM1_SPI        (1 << __FLEXCOM_SPI_SPI_SHIFT) /* FLEXCOM1 SPI */
#  define __FLEXCOM2_SPI        (2 << __FLEXCOM_SPI_SPI_SHIFT) /* FLEXCOM2 SPI */
#  define __FLEXCOM3_SPI        (3 << __FLEXCOM_SPI_SPI_SHIFT) /* FLEXCOM3 SPI */
#  define __FLEXCOM4_SPI        (4 << __FLEXCOM_SPI_SPI_SHIFT) /* FLEXCOM4 SPI */

#define FLEXCOM0_SPI_CS0 (__FLEXCOM0_SPI | __FLEXCOM_SPI_CS0)
#define FLEXCOM0_SPI_CS1 (__FLEXCOM0_SPI | __FLEXCOM_SPI_CS1)

#define FLEXCOM1_SPI_CS0 (__FLEXCOM1_SPI | __FLEXCOM_SPI_CS0)
#define FLEXCOM1_SPI_CS1 (__FLEXCOM1_SPI | __FLEXCOM_SPI_CS1)

#define FLEXCOM2_SPI_CS0 (__FLEXCOM2_SPI | __FLEXCOM_SPI_CS0)
#define FLEXCOM2_SPI_CS1 (__FLEXCOM3_SPI | __FLEXCOM_SPI_CS1)

#define FLEXCOM3_SPI_CS0 (__FLEXCOM3_SPI | __FLEXCOM_SPI_CS0)
#define FLEXCOM3_SPI_CS1 (__FLEXCOM3_SPI | __FLEXCOM_SPI_CS1)

#define FLEXCOM4_SPI_CS0 (__FLEXCOM4_SPI | __FLEXCOM_SPI_CS0)
#define FLEXCOM4_SPI_CS1 (__FLEXCOM4_SPI | __FLEXCOM_SPI_CS1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_flex_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   cs - Chip select number (identifying the "logical" SPI port)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *sam_flex_spibus_initialize(int port);

/****************************************************************************
 * Name:  sam_flexcom_spi[0|1|2|3|4]select, sam_flexcom_spi[0|1|2|3|4]status,
 * and sam_flexcom_spi[0|1|2|3|4]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic. They
 *   include:
 *
 *   o sam_flexcom_spi[0|1|2|3|4]select is a functions to manage the
 *     board-specific chip selects
 *   o sam_flexcom_spi[0|1|2|3|4]status and sam_flexcom_spi[0|1|2|3|4]cmddata
 *     implementations of the status and cmddata methods of the SPI interface
 *     defined by struct spi_ops_ (see include/nuttx/spi/spi.h).
 *     All other methods including sam_flex_spibus_initialize()) are provided
 *     by common SAM2/3/4 logic.
 *
 *  To use this common SPI logic on your board:
 *
 *   1. Provide logic in sam_boardinitialize() to configure FLEXCOM SPI
 *      chip select pins.
 *   2. Provide sam_flexcom_spi[0|1|2|3|4]select() and
 *      sam_flexcom_spi[0|1|2|3|4]status() functions in your board-specific
 *      logic.  These functions will perform chip selection and status
 *      operations using PIOs in the way your board is configured.
 *   2. If CONFIG_FLEXCOMx_SPI_CMDDATA is defined in the NuttX configuration,
 *      provide sam_flexcom_spi[0|1|2|3|4]cmddata() functions in your
 *      board-specific logic. This function will perform cmd/data selection
 *      operations using PIOs in the way your board is configured.
 *   3. Add a call to sam_flex_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by sam_flex_spibus_initialize() may then be used
 *      to bind the FLEXCOM SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

/****************************************************************************
 * Name: sam_flexcom_spi[0|1|2|3|4]select
 *
 * Description:
 *   PIO chip select pins may be programmed by the board specific logic in
 *   one of two different ways.  First, the pins may be programmed as SPI
 *   peripherals.  In that case, the pins are completely controlled by the
 *   SPI driver.  This method still needs to be provided, but it may be only
 *   a stub.
 *
 *   An alternative way to program the PIO chip select pins is as a normal
 *   PIO output.  In that case, the automatic control of the CS pins is
 *   bypassed and this function must provide control of the chip select.
 *   NOTE:  In this case, the PIO output pin does *not* have to be the
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

#ifdef CONFIG_SAMA5_FLEXCOM0_SPI
void sam_flexcom0_spiselect(uint32_t devid, bool selected);
#endif
#ifdef CONFIG_SAMA5_FLEXCOM1_SPI
void sam_flexcom1_spiselect(uint32_t devid, bool selected);
#endif
#ifdef CONFIG_SAMA5_FLEXCOM2_SPI
void sam_flexcom2_spiselect(uint32_t devid, bool selected);
#endif
#ifdef CONFIG_SAMA5_FLEXCOM3_SPI
void sam_flexcom3_spiselect(uint32_t devid, bool selected);
#endif
#ifdef CONFIG_SAMA5_FLEXCOM4_SPI
void sam_flexcom4_spiselect(uint32_t devid, bool selected);
#endif

/****************************************************************************
 * Name: sam_flexcom_spi[0|1|2|3|4]status
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

#ifdef CONFIG_SAMA5_FLEXCOM0_SPI
uint8_t sam_flexcom0_spistatus(struct spi_dev_s *dev, uint32_t devid);
#endif
#ifdef CONFIG_SAMA5_FLEXCOM1_SPI
uint8_t sam_flexcom1_spistatus(struct spi_dev_s *dev, uint32_t devid);
#endif
#ifdef CONFIG_SAMA5_FLEXCOM2_SPI
uint8_t sam_flexcom2_spistatus(struct spi_dev_s *dev, uint32_t devid);
#endif
#ifdef CONFIG_SAMA5_FLEXCOM3_SPI
uint8_t sam_flexcom3_spistatus(struct spi_dev_s *dev, uint32_t devid);
#endif
#ifdef CONFIG_SAMA5_FLEXCOM4_SPI
uint8_t sam_flexcom4_spistatus(struct spi_dev_s *dev, uint32_t devid);
#endif

/****************************************************************************
 * Name: sam_spi[0|1|2|3|4]cmddata
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
 *   specific PIO control to distinguish command and data.  This function
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

#ifdef CONFIG_FLEXCOM_SPI_CMDDATA
#ifdef CONFIG_SAMA5_FLEXCOM0_SPI
int sam_flexcom0_spicmddata(struct spi_dev_s *dev, uint32_t devid,
                            bool cmd);
#endif
#ifdef CONFIG_SAMA5_FLEXCOM1_SPI
int sam_flexcom1_spicmddata(struct spi_dev_s *dev, uint32_t devid,
                            bool cmd);
#endif
#ifdef CONFIG_SAMA5_FLEXCOM2_SPI
int sam_flexcom2_spicmddata(struct spi_dev_s *dev, uint32_t devid,
                            bool cmd);
#endif
#ifdef CONFIG_SAMA5_FLEXCOM3_SPI
int sam_flexcom3_spicmddata(struct spi_dev_s *dev, uint32_t devid,
                            bool cmd);
#endif
#ifdef CONFIG_SAMA5_FLEXCOM4_SPI
int sam_flexcom4_spicmddata(struct spi_dev_s *dev, uint32_t devid,
                            bool cmd);
#endif
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_FLEXCOM_SPI_H */
