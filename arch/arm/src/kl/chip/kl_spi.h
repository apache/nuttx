/********************************************************************************************
 * arch/arm/src/kl/kl_spi.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KL_CHIP_KL_SPI_H
#define __ARCH_ARM_SRC_KL_CHIP_KL_SPI_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KL_SPI_C1_OFFSET    0x0000 /* SPI control register 1 */
#define KL_SPI_C2_OFFSET    0x0001 /* SPI control register 2 */
#define KL_SPI_BR_OFFSET    0x0002 /* SPI baud rate register */
#define KL_SPI_S_OFFSET     0x0003 /* SPI status register */
#define KL_SPI_D_OFFSET     0x0005 /* SPI data register */
#define KL_SPI_M_OFFSET     0x0007 /* SPI match register */

/* Register Addresses ***********************************************************************/

#define KL_SPI0_C1          (KL_SPI0_BASE+KL_SPI_C1_OFFSET)
#define KL_SPI0_C2          (KL_SPI0_BASE+KL_SPI_C2_OFFSET)
#define KL_SPI0_BR          (KL_SPI0_BASE+KL_SPI_BR_OFFSET)
#define KL_SPI0_S           (KL_SPI0_BASE+KL_SPI_S_OFFSET)
#define KL_SPI0_D           (KL_SPI0_BASE+KL_SPI_D_OFFSET)
#define KL_SPI0_M           (KL_SPI0_BASE+KL_SPI_M_OFFSET)

#define KL_SPI1_C1          (KL_SPI1_BASE+KL_SPI_C1_OFFSET)
#define KL_SPI1_C2          (KL_SPI1_BASE+KL_SPI_C2_OFFSET)
#define KL_SPI1_BR          (KL_SPI1_BASE+KL_SPI_BR_OFFSET)
#define KL_SPI1_S           (KL_SPI1_BASE+KL_SPI_S_OFFSET)
#define KL_SPI1_D           (KL_SPI1_BASE+KL_SPI_D_OFFSET)
#define KL_SPI1_M           (KL_SPI1_BASE+KL_SPI_M_OFFSET)

/* Register Bit Definitions *****************************************************************/

/* SPI control register 1 */

#define SPI_C1_LSBFE        (1 << 0)  /* Bit 0:  LSB first (shifter direction) */
#define SPI_C1_SSOE         (1 << 1)  /* Bit 1:  Slave select output enable */
#define SPI_C1_CPHA         (1 << 2)  /* Bit 2:  Clock phase */
#define SPI_C1_CPOL         (1 << 3)  /* Bit 3:  Clock polarity */
#define SPI_C1_MSTR         (1 << 4)  /* Bit 4:  Master/slave mode select */
#define SPI_C1_SPTIE        (1 << 5)  /* Bit 5:  SPI transmit interrupt enable */
#define SPI_C1_SPE          (1 << 6)  /* Bit 6:  SPI system enable */
#define SPI_C1_SPIE         (1 << 7)  /* Bit 7:  SPI interrupt enable: for SPRF and MODF */

/* SPI control register 2 */

#define SPI_C2_SPC0         (1 << 0)  /* Bit 0:  SPI pin control 0 */
#define SPI_C2_SPISWAI      (1 << 1)  /* Bit 1:  SPI stop in wait mode */
#define SPI_C2_RXDMAE       (1 << 2)  /* Bit 2:  Receive DMA enable */
#define SPI_C2_BIDIROE      (1 << 3)  /* Bit 3:  Bidirectional mode output enable */
#define SPI_C2_MODFEN       (1 << 4)  /* Bit 4:  Master mode-fault function enable */
#define SPI_C2_TXDMAE       (1 << 5)  /* Bit 5:  Transmit DMA enable */
#define SPI_C2_SPMIE        (1 << 7)  /* Bit 7:  SPI match interrupt enable */

/* SPI baud rate register */

#define SPI_BR_SPR_SHIFT    (0)       /* Bits 0-3: SPI baud rate divisor */
#define SPI_BR_SPR_MASK     (15 << SPI_BR_SPR_SHIFT)
#  define SPI_BR_SPR_DIV(n) (((n)-1) << SPI_BR_SPR_SHIFT) /* Baud rate divisor is 2^(n-1) */
#  define SPI_BR_SPR_DIV2   (0 << SPI_BR_SPR_SHIFT) /* Baud rate divisor is 2 */
#  define SPI_BR_SPR_DIV4   (1 << SPI_BR_SPR_SHIFT) /* Baud rate divisor is 4 */
#  define SPI_BR_SPR_DIV8   (2 << SPI_BR_SPR_SHIFT) /* Baud rate divisor is 8 */
#  define SPI_BR_SPR_DIV16  (3 << SPI_BR_SPR_SHIFT) /* Baud rate divisor is 16 */
#  define SPI_BR_SPR_DIV32  (4 << SPI_BR_SPR_SHIFT) /* Baud rate divisor is 32 */
#  define SPI_BR_SPR_DIV64  (5 << SPI_BR_SPR_SHIFT) /* Baud rate divisor is 64 */
#  define SPI_BR_SPR_DIV128 (6 << SPI_BR_SPR_SHIFT) /* Baud rate divisor is 128 */
#  define SPI_BR_SPR_DIV256 (7 << SPI_BR_SPR_SHIFT) /* Baud rate divisor is 256 */
#  define SPI_BR_SPR_DIV512 (8 << SPI_BR_SPR_SHIFT) /* Baud rate divisor is 512 */
#define SPI_BR_SPPR_SHIFT   (4)       /* Bits 4-6: SPI baud rate prescale divisor */
#define SPI_BR_SPPR_MASK    (7 << SPI_BR_SPPR_SHIFT)
#  define SPI_BR_SPPR(n)    (((n)-1) << SPI_BR_SPPR_SHIFT) /* Prescaler=n, n=1-8 */
                                      /* Bit 7:  Reserved */
/* SPI status register */
                                      /* Bits 0-3: Reserved */
#define SPI_S_MODF          (1 << 4)  /* Bit 4:  Master mode fault flag */
#define SPI_S_SPTEF         (1 << 5)  /* Bit 5:  SPI transmit buffer empty flag */
#define SPI_S_SPMF          (1 << 6)  /* Bit 6:  SPI match flag */
#define SPI_S_SPRF          (1 << 7)  /* Bit 7:  SPI read buffer full flag */

/* SPI data register (8-bit data, low byte) */
/* SPI match register (8-bit match value) */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KL_CHIP_KL_SPI_H */
