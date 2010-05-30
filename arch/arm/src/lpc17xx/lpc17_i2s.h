/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_i2s
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_I2S_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_I2S_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "lp17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC17_I2S_DAO_OFFSET        0x0000 /* Digital Audio Output Register */
#define LPC17_I2S_DAI_OFFSET        0x0004 /* Digital Audio Input Register */
#define LPC17_I2S_TXFIFO_OFFSET     0x0008 /* Transmit FIFO */
#define LPC17_I2S_RXFIFO_OFFSET     0x000c /* Receive FIFO */
#define LPC17_I2S_STATE_OFFSET      0x0010 /* Status Feedback Register */
#define LPC17_I2S_DMA1_OFFSET       0x0014 /* DMA Configuration Register 1 */
#define LPC17_I2S_DMA2_OFFSET       0x0018 /* DMA Configuration Register 2 */
#define LPC17_I2S_IRQ_OFFSET        0x001c /* Interrupt Request Control Register */
#define LPC17_I2S_TXRATE_OFFSET     0x0020 /* Transmit MCLK divider */
#define LPC17_I2S_RXRATE_OFFSET     0x0024 /* Receive MCLK divider */
#define LPC17_I2S_TXBITRATE_OFFSET  0x0028 /* Transmit bit rate divider */
#define LPC17_I2S_RXBITRATE_OFFSET  0x002c /* Receive bit rate divider */
#define LPC17_I2S_TXMODE_OFFSET     0x0030 /* Transmit mode control */
#define LPC17_I2S_RXMODE_OFFSET     0x0034 /* Receive mode control */

/* Register addresses ***************************************************************/

#define LPC17_I2S_DAO               (LPC17_I2S_BASE+LPC17_I2S_DAO_OFFSET)
#define LPC17_I2S_DAI               (LPC17_I2S_BASE+LPC17_I2S_DAI_OFFSET)
#define LPC17_I2S_TXFIFO            (LPC17_I2S_BASE+LPC17_I2S_TXFIFO_OFFSET)
#define LPC17_I2S_RXFIFO            (LPC17_I2S_BASE+LPC17_I2S_RXFIFO_OFFSET)
#define LPC17_I2S_STATE             (LPC17_I2S_BASE+LPC17_I2S_STATE_OFFSET)
#define LPC17_I2S_DMA1              (LPC17_I2S_BASE+LPC17_I2S_DMA1_OFFSET)
#define LPC17_I2S_DMA2              (LPC17_I2S_BASE+LPC17_I2S_DMA2_OFFSET)
#define LPC17_I2S_IRQ               (LPC17_I2S_BASE+LPC17_I2S_IRQ_OFFSET)
#define LPC17_I2S_TXRATE            (LPC17_I2S_BASE+LPC17_I2S_TXRATE_OFFSET)
#define LPC17_I2S_RXRATE            (LPC17_I2S_BASE+LPC17_I2S_RXRATE_OFFSET)
#define LPC17_I2S_TXBITRATE         (LPC17_I2S_BASE+LPC17_I2S_TXBITRATE_OFFSET)
#define LPC17_I2S_RXBITRATE         (LPC17_I2S_BASE+LPC17_I2S_RXBITRATE_OFFSET)
#define LPC17_I2S_TXMODE            (LPC17_I2S_BASE+LPC17_I2S_TXMODE_OFFSET)
#define LPC17_I2S_RXMODE            (LPC17_I2S_BASE+LPC17_I2S_RXMODE_OFFSET)

/* Register bit definitions *********************************************************/

/* Digital Audio Output Register */
#define I2S_DAO_
/* Digital Audio Input Register */
#define I2S_DAI_
/* Transmit FIFO */
#define I2S_TXFIFO_
/* Receive FIFO */
#define I2S_RXFIFO_
/* Status Feedback Register */
#define I2S_STATE_
/* DMA Configuration Register 1 */
#define I2S_DMA1_
/* DMA Configuration Register 2 */
#define I2S_DMA2_
/* Interrupt Request Control Register */
#define I2S_IRQ_
/* Transmit MCLK divider */
#define I2S_TXRATE_
/* Receive MCLK divider */
#define I2S_RXRATE_
/* Transmit bit rate divider */
#define I2S_TXBITRATE_
/* Receive bit rate divider */
#define I2S_RXBITRATE_
/* Transmit mode control */
#define I2S_TXMODE_
/* Receive mode control */
#define I2S_RXMODE_

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_I2S_H */
