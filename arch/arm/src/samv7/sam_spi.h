/****************************************************************************
 * arch/arm/src/samv7/sam_spi.h
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMV7_SAM_SPI_H
#define __ARCH_ARM_SRC_SAMV7_SAM_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "sam_config.h"

#ifdef CONFIG_SAMV7_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SPI port number used as an input to sam_spibus_initialize encodes
 * information about the SPI controller (0 or 1) and the SPI chip select
 * (0-3).
 *
 * NOTE that this is this is backward compatible with older implementations
 * that support only SPI0 and provide only the chip select number to
 * sam_spibus_initialize().
 */

#ifdef CONFIG_SAMV7_SPI_CS_DECODING

#  define __SPI_CS_SHIFT  (0)      /* Bits 0-3: SPI chip select number */
#  define __SPI_CS_MASK   (15 << __SPI_CS_SHIFT)

#  define __SPI_SPI_SHIFT (4)      /* Bit 4: SPI controller number */
#  define __SPI_SPI_MASK  (1 << __SPI_SPI_SHIFT)
#    define __SPI_SPI0    (0 << __SPI_SPI_SHIFT) /* SPI0 */
#    define __SPI_SPI1    (1 << __SPI_SPI_SHIFT) /* SPI1 */

#  define SPI0_CS1        (__SPI_SPI0 | 1)
#  define SPI0_CS2        (__SPI_SPI0 | 2)
#  define SPI0_CS3        (__SPI_SPI0 | 3)
#  define SPI0_CS4        (__SPI_SPI0 | 4)
#  define SPI0_CS5        (__SPI_SPI0 | 5)
#  define SPI0_CS6        (__SPI_SPI0 | 6)
#  define SPI0_CS7        (__SPI_SPI0 | 7)
#  define SPI0_CS8        (__SPI_SPI0 | 8)
#  define SPI0_CS9        (__SPI_SPI0 | 9)
#  define SPI0_CS10       (__SPI_SPI0 | 10)
#  define SPI0_CS11       (__SPI_SPI0 | 11)
#  define SPI0_CS12       (__SPI_SPI0 | 12)
#  define SPI0_CS13       (__SPI_SPI0 | 13)
#  define SPI0_CS14       (__SPI_SPI0 | 14)
#  define SPI0_CS15       (__SPI_SPI0 | 15)

#  define SPI1_CS1        (__SPI_SPI1 | 1)
#  define SPI1_CS2        (__SPI_SPI1 | 2)
#  define SPI1_CS3        (__SPI_SPI1 | 3)
#  define SPI1_CS4        (__SPI_SPI1 | 4)
#  define SPI1_CS5        (__SPI_SPI1 | 5)
#  define SPI1_CS6        (__SPI_SPI1 | 6)
#  define SPI1_CS7        (__SPI_SPI1 | 7)
#  define SPI1_CS8        (__SPI_SPI1 | 8)
#  define SPI1_CS9        (__SPI_SPI1 | 9)
#  define SPI1_CS10       (__SPI_SPI1 | 10)
#  define SPI1_CS11       (__SPI_SPI1 | 11)
#  define SPI1_CS12       (__SPI_SPI1 | 12)
#  define SPI1_CS13       (__SPI_SPI1 | 13)
#  define SPI1_CS14       (__SPI_SPI1 | 14)
#  define SPI1_CS15       (__SPI_SPI1 | 15)

#else /* CONFIG_SAMV7_SPI_CS_DECODING */

#  define __SPI_CS_SHIFT  (0)      /* Bits 0-1: SPI chip select number */
#  define __SPI_CS_MASK   (3 << __SPI_CS_SHIFT)
#    define __SPI_CS0     (0 << __SPI_CS_SHIFT)
#    define __SPI_CS1     (1 << __SPI_CS_SHIFT)
#    define __SPI_CS2     (2 << __SPI_CS_SHIFT)
#    define __SPI_CS3     (3 << __SPI_CS_SHIFT)
#  define __SPI_SPI_SHIFT (2)      /* Bit 2: SPI controller number */
#  define __SPI_SPI_MASK  (1 << __SPI_SPI_SHIFT)
#    define __SPI_SPI0    (0 << __SPI_SPI_SHIFT) /* SPI0 */
#    define __SPI_SPI1    (1 << __SPI_SPI_SHIFT) /* SPI1 */

#  define SPI0_CS0        (__SPI_SPI0 | __SPI_CS0)
#  define SPI0_CS1        (__SPI_SPI0 | __SPI_CS1)
#  define SPI0_CS2        (__SPI_SPI0 | __SPI_CS2)
#  define SPI0_CS3        (__SPI_SPI0 | __SPI_CS3)

#  define SPI1_CS0        (__SPI_SPI1 | __SPI_CS0)
#  define SPI1_CS1        (__SPI_SPI1 | __SPI_CS1)
#  define SPI1_CS2        (__SPI_SPI1 | __SPI_CS2)
#  define SPI1_CS3        (__SPI_SPI1 | __SPI_CS3)

#endif /* CONFIG_SAMV7_SPI_CS_DECODING */

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

struct spi_dev_s;    /* Forward reference */
struct spi_sctrlr_s; /* Forward reference */

/****************************************************************************
 * Name: sam_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port in master mode
 *
 * Input Parameters:
 *   cs - Chip select number (identifying the "logical" SPI port)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *sam_spibus_initialize(int port);

/****************************************************************************
 * Name: sam_spi_slave_initialize
 *
 * Description:
 *   Initialize the selected SPI port in slave mode.
 *
 * Input Parameters:
 *   port - Chip select number identifying the "logical" SPI port.  Includes
 *          encoded port and chip select information.
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_sctrlr_s *sam_spi_slave_initialize(int port);

/****************************************************************************
 * Name:  sam_spi[0|1]select, sam_spi[0|1]status, and sam_spi[0|1]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.
 *   They include:
 *
 *   o sam_spi[0|1]select is a functions to manage the board-specific chip
 *     selects
 *   o sam_spi[0|1]status and sam_spi[0|1]cmddata:  Implementations of the
 *     status and cmddata methods of the SPI interface defined by struct
 *     spi_ops_ (see include/nuttx/spi/spi.h). All other methods including
 *     sam_spibus_initialize()) are provided by common SAM3/4 logic.
 *
 *  To use this common SPI logic on your board:
 *
 *   1. Provide logic in sam_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide sam_spi[0|1]select() and sam_spi[0|1]status() functions in
 *      our board-specific logic.  These functions will perform chip selection
 *      and status operations using PIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      sam_spi[0|1]cmddata() functions in your board-specific logic.  This
 *      function will perform cmd/data selection operations using PIOs in
 *      the way your board is configured.
 *   3. Add a call to sam_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by sam_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spi[0|1]select
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

#ifdef CONFIG_SAMV7_SPI0_MASTER
void sam_spi0select(uint32_t devid, bool selected);
#endif
#ifdef CONFIG_SAMV7_SPI1_MASTER
void sam_spi1select(uint32_t devid, bool selected);
#endif

/****************************************************************************
 * Name: sam_spi[0|1]status
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

#ifdef CONFIG_SAMV7_SPI0
uint8_t sam_spi0status(FAR struct spi_dev_s *dev, uint32_t devid);
#endif
#ifdef CONFIG_SAMV7_SPI1
uint8_t sam_spi1status(FAR struct spi_dev_s *dev, uint32_t devid);
#endif

/****************************************************************************
 * Name: sam_spi[0|1]cmddata
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

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_SAMV7_SPI0_MASTER
int sam_spi0cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#ifdef CONFIG_SAMV7_SPI1_MASTER
int sam_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif /* CONFIG_SPI_CMDDATA */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SAMV7_SPI */
#endif /* __ARCH_ARM_SRC_SAMV7_SAM_SPI_H */
