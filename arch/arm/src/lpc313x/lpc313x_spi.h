/************************************************************************************************
 * arch/arm/src/lpc313x/lpc313x_spi.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC313X_SPI_H
#define __ARCH_ARM_SRC_LPC313X_SPI_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc313x_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* SPI register base address offset into the APB2 domain ****************************************/

#define LPC313X_SPI_VBASE                (LPC313X_APB2_VSECTION+LPC313X_APB2_SPI_OFFSET)
#define LPC313X_SPI_PBASE                (LPC313X_APB2_PSECTION+LPC313X_APB2_SPI_OFFSET)

/* SPI register offsets (with respect to the SPI base) ******************************************/
/* SPI configuration registers */

#define LPC313X_SPI_CONFIG_OFFSET        0x000 /* Configuration register */
#define LPC313X_SPI_SLVENABLE_OFFSET     0x004 /* Slave enable register */
#define LPC313X_SPI_TXFIFOFLUSH_OFFSET   0x008 /* Transmit FIFO flush register */
#define LPC313X_SPI_FIFODATA_OFFSET      0x00C /* FIFO data register */
#define LPC313X_SPI_NHPPOP_OFFSET        0x010 /* NHP pop register */
#define LPC313X_SPI_NHPMODE_OFFSET       0x014 /* NHP mode selection register */
#define LPC313X_SPI_DMA_OFFSET           0x018 /* DMA settings register */
#define LPC313X_SPI_STATUS_OFFSET        0x01c /* Status register */
#define LPC313X_SPI_HWINFO_OFFSET        0x020 /* Hardware information register */

/* SPI slave registers */

#define LPC313X_SPI_SLV0_1_OFFSET        0x024 /* Slave settings register 1 (for slave 0) */
#define LPC313X_SPI_SLV0_2_OFFSET        0x028 /* Slave settings register 2 (for slave 0) */
#define LPC313X_SPI_SLV1_1_OFFSET        0x02c /* Slave settings register 1 (for slave 1) */
#define LPC313X_SPI_SLV1_2_OFFSET        0x030 /* Slave settings register 2 (for slave 1) */
#define LPC313X_SPI_SLV2_1_OFFSET        0x034 /* Slave settings register 1 (for slave 2) */
#define LPC313X_SPI_SLV2_2_OFFSET        0x038 /* Slave settings register 2 (for slave 2) */
                                               /* 0x03c-0xfd0: Reserved */
/* SPI interrupt registers */

#define LPC313X_SPI_INTTHR_OFFSET        0xfd4 /* Tx/Rx threshold interrupt levels */
#define LPC313X_SPI_INTCLRENABLE_OFFSET  0xfd8 /* INT_ENABLE bits clear register */
#define LPC313X_SPI_INTSETENABLE_OFFSET  0xfdc /* INT_ENABLE bits set register */
#define LPC313X_SPI_INTSTATUS_OFFSET     0xfe0 /* Interrupt status register */
#define LPC313X_SPI_INTENABLE_OFFSET     0xfe4 /* Interrupt enable register */
#define LPC313X_SPI_INTCLRSTATUS_OFFSET  0xfe8 /* INT_STATUS bits clear register */
#define LPC313X_SPI_INTSETSTATUS_OFFSET  0xfec /* INT_STATUS bits set register */
                                               /* 0xff0-0xff8: Reserved */

/* SPI register (virtual) addresses *************************************************************/

/* SPI configuration registers */

#define LPC313X_SPI_CONFIG               (LPC313X_SPI_VBASE+LPC313X_SPI_CONFIG_OFFSET)
#define LPC313X_SPI_SLVENABLE            (LPC313X_SPI_VBASE+LPC313X_SPI_SLVENABLE_OFFSET)
#define LPC313X_SPI_TXFIFOFLUSH          (LPC313X_SPI_VBASE+LPC313X_SPI_TXFIFOFLUSH_OFFSET)
#define LPC313X_SPI_FIFODATA             (LPC313X_SPI_VBASE+LPC313X_SPI_FIFODATA_OFFSET)
#define LPC313X_SPI_NHPPOP               (LPC313X_SPI_VBASE+LPC313X_SPI_NHPPOP_OFFSET)
#define LPC313X_SPI_NHPMODE              (LPC313X_SPI_VBASE+LPC313X_SPI_NHPMODE_OFFSET)
#define LPC313X_SPI_DMA                  (LPC313X_SPI_VBASE+LPC313X_SPI_DMA_OFFSET)
#define LPC313X_SPI_STATUS               (LPC313X_SPI_VBASE+LPC313X_SPI_STATUS_OFFSET)
#define LPC313X_SPI_HWINFO               (LPC313X_SPI_VBASE+LPC313X_SPI_HWINFO_OFFSET)

/* SPI slave registers */

#define LPC313X_SPI_SLV0_1               (LPC313X_SPI_VBASE+LPC313X_SPI_SLV0_1_OFFSET)
#define LPC313X_SPI_SLV0_2               (LPC313X_SPI_VBASE+LPC313X_SPI_SLV0_2_OFFSET)
#define LPC313X_SPI_SLV1_1               (LPC313X_SPI_VBASE+LPC313X_SPI_SLV1_1_OFFSET)
#define LPC313X_SPI_SLV1_2               (LPC313X_SPI_VBASE+LPC313X_SPI_SLV1_2_OFFSET)
#define LPC313X_SPI_SLV2_1               (LPC313X_SPI_VBASE+LPC313X_SPI_SLV2_1_OFFSET)
#define LPC313X_SPI_SLV2_2               (LPC313X_SPI_VBASE+LPC313X_SPI_SLV2_2_OFFSET)

/* SPI interrupt registers */

#define LPC313X_SPI_INTTHR               (LPC313X_SPI_VBASE+LPC313X_SPI_INTTHR_OFFSET)
#define LPC313X_SPI_INTCLRENABLE         (LPC313X_SPI_VBASE+LPC313X_SPI_INTCLRENABLE_OFFSET)
#define LPC313X_SPI_INTSETENABLE         (LPC313X_SPI_VBASE+LPC313X_SPI_INTSETENABLE_OFFSET)
#define LPC313X_SPI_INTSTATUS            (LPC313X_SPI_VBASE+LPC313X_SPI_INTSTATUS_OFFSET)
#define LPC313X_SPI_INTENABLE            (LPC313X_SPI_VBASE+LPC313X_SPI_INTENABLE_OFFSET)
#define LPC313X_SPI_INTCLRSTATUS         (LPC313X_SPI_VBASE+LPC313X_SPI_INTCLRSTATUS_OFFSET)
#define LPC313X_SPI_INTSETSTATUS         (LPC313X_SPI_VBASE+LPC313X_SPI_INTSETSTATUS_OFFSET)

/* SPI register bit definitions *****************************************************************/

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC313X_SPI_H */
