/************************************************************************************
 * arch/arm/src/kinetis/kinetis_i2s.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_I2S_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_I2S_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_I2S_TX0_OFFSET     0x000 /* I2S Transmit Data Registers 0 */
#define KINETIS_I2S_TX1_OFFSET     0x004 /* I2S Transmit Data Registers 1 */
#define KINETIS_I2S_RX0_OFFSET     0x008 /* I2S Receive Data Registers 0 */
#define KINETIS_I2S_RX1_OFFSET     0x00c /* I2S Receive Data Registers 1 */
#define KINETIS_I2S_CR_OFFSET      0x010 /* I2S Control Register */
#define KINETIS_I2S_ISR_OFFSET     0x014 /* I2S Interrupt Status Register */
#define KINETIS_I2S_IER_OFFSET     0x018 /* I2S Interrupt Enable Register */
#define KINETIS_I2S_TCR_OFFSET     0x01c /* I2S Transmit Configuration Register */
#define KINETIS_I2S_RCR_OFFSET     0x020 /* I2S Receive Configuration Register */
#define KINETIS_I2S_TCCR_OFFSET    0x024 /* I2S Transmit Clock Control Registers */
#define KINETIS_I2S_RCCR_OFFSET    0x028 /* I2S Receive Clock Control Registers */
#define KINETIS_I2S_FCSR_OFFSET    0x02c /* I2S FIFO Control/Status Register */
#define KINETIS_I2S_ACNT_OFFSET    0x038 /* I2S AC97 Control Register */
#define KINETIS_I2S_ACADD_OFFSET   0x03c /* I2S AC97 Command Address Register */
#define KINETIS_I2S_ACDAT_OFFSET   0x040 /* I2S AC97 Command Data Register */
#define KINETIS_I2S_ATAG_OFFSET    0x044 /* I2S AC97 Tag Register */
#define KINETIS_I2S_TMSK_OFFSET    0x048 /* I2S Transmit Time Slot Mask Register */
#define KINETIS_I2S_RMSK_OFFSET    0x04c /* I2S Receive Time Slot Mask Register */
#define KINETIS_I2S_ACCST_OFFSET   0x050 /* I2S AC97 Channel Status Register */
#define KINETIS_I2S_ACCEN_OFFSET   0x054 /* I2S AC97 Channel Enable Register */
#define KINETIS_I2S_ACCDIS_OFFSET  0x058 /* I2S AC97 Channel Disable Register */

/* Register Addresses ***************************************************************/

#define KINETIS_I2S0_TX0           (KINETIS_I2S0_BASE+KINETIS_I2S_TX0_OFFSET)
#define KINETIS_I2S0_TX1           (KINETIS_I2S0_BASE+KINETIS_I2S_TX1_OFFSET)
#define KINETIS_I2S0_RX0           (KINETIS_I2S0_BASE+KINETIS_I2S_RX0_OFFSET)
#define KINETIS_I2S0_RX1           (KINETIS_I2S0_BASE+KINETIS_I2S_RX1_OFFSET)
#define KINETIS_I2S0_CR            (KINETIS_I2S0_BASE+KINETIS_I2S_CR_OFFSET)
#define KINETIS_I2S0_ISR           (KINETIS_I2S0_BASE+KINETIS_I2S_ISR_OFFSET)
#define KINETIS_I2S0_IER           (KINETIS_I2S0_BASE+KINETIS_I2S_IER_OFFSET)
#define KINETIS_I2S0_TCR           (KINETIS_I2S0_BASE+KINETIS_I2S_TCR_OFFSET)
#define KINETIS_I2S0_RCR           (KINETIS_I2S0_BASE+KINETIS_I2S_RCR_OFFSET)
#define KINETIS_I2S0_TCCR          (KINETIS_I2S0_BASE+KINETIS_I2S_TCCR_OFFSET)
#define KINETIS_I2S0_RCCR          (KINETIS_I2S0_BASE+KINETIS_I2S_RCCR_OFFSET)
#define KINETIS_I2S0_FCSR          (KINETIS_I2S0_BASE+KINETIS_I2S_FCSR_OFFSET)
#define KINETIS_I2S0_ACNT          (KINETIS_I2S0_BASE+KINETIS_I2S_ACNT_OFFSET)
#define KINETIS_I2S0_ACADD         (KINETIS_I2S0_BASE+KINETIS_I2S_ACADD_OFFSET)
#define KINETIS_I2S0_ACDAT         (KINETIS_I2S0_BASE+KINETIS_I2S_ACDAT_OFFSET)
#define KINETIS_I2S0_ATAG          (KINETIS_I2S0_BASE+KINETIS_I2S_ATAG_OFFSET)
#define KINETIS_I2S0_TMSK          (KINETIS_I2S0_BASE+KINETIS_I2S_TMSK_OFFSET)
#define KINETIS_I2S0_RMSK          (KINETIS_I2S0_BASE+KINETIS_I2S_RMSK_OFFSET)
#define KINETIS_I2S0_ACCST         (KINETIS_I2S0_BASE+KINETIS_I2S_ACCST_OFFSET)
#define KINETIS_I2S0_ACCEN         (KINETIS_I2S0_BASE+KINETIS_I2S_ACCEN_OFFSET)
#define KINETIS_I2S0_ACCDIS        (KINETIS_I2S0_BASE+KINETIS_I2S_ACCDIS_OFFSET)

/* Register Bit Definitions *********************************************************/

/* I2S Transmit Data Registers 0 */
#define I2S_TX0_
/* I2S Transmit Data Registers 1 */
#define I2S_TX1_
/* I2S Receive Data Registers 0 */
#define I2S_RX0_
/* I2S Receive Data Registers 1 */
#define I2S_RX1_
/* I2S Control Register */
#define I2S_CR_
/* I2S Interrupt Status Register */
#define I2S_ISR_
/* I2S Interrupt Enable Register */
#define I2S_IER_
/* I2S Transmit Configuration Register */
#define I2S_TCR_
/* I2S Receive Configuration Register */
#define I2S_RCR_
/* I2S Transmit Clock Control Registers */
#define I2S_TCCR_
/* I2S Receive Clock Control Registers */
#define I2S_RCCR_
/* I2S FIFO Control/Status Register */
#define I2S_FCSR_
/* I2S AC97 Control Register */
#define I2S_ACNT_
/* I2S AC97 Command Address Register */
#define I2S_ACADD_
/* I2S AC97 Command Data Register */
#define I2S_ACDAT_
/* I2S AC97 Tag Register */
#define I2S_ATAG_
/* I2S Transmit Time Slot Mask Register */
#define I2S_TMSK_
/* I2S Receive Time Slot Mask Register */
#define I2S_RMSK_
/* I2S AC97 Channel Status Register */
#define I2S_ACCST_
/* I2S AC97 Channel Enable Register */
#define I2S_ACCEN_
/* I2S AC97 Channel Disable Register */
#define I2S_ACCDIS_

                (1 << nn)  /* Bit nn:  
_SHIFT          (nn)       /* Bits nn-nn: 
_MASK           (nn << nn)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_I2S_H */
