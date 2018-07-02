/************************************************************************************
 * arch/arm/src/stm32f7/stm32_fmc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_STM32F7_FMC_H
#define __ARCH_ARM_SRC_STM32_STM32F7_FMC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_FMC_SDCR1_OFFSET 0x0140 /* SDRAM Control Register, Bank 0 */
#define STM32_FMC_SDCR2_OFFSET 0x0144 /* SDRAM Control Register, Bank 1 */

#define STM32_FMC_SDTR1_OFFSET 0x0148 /* SDRAM Timing Register, Bank 0 */
#define STM32_FMC_SDTR2_OFFSET 0x014c /* SDRAM Timing Register, Bank 1 */

#define STM32_FMC_SDCMR_OFFSET 0x0150 /* SDRAM Config Memory register */
#define STM32_FMC_SDRTR_OFFSET 0x0154 /* SDRAM Refresh Timing Register maybe */
#define STM32_FMC_SDSR_OFFSET  0x0158 /* SDRAM Status Register */

/* Register Addresses ***************************************************************/

#define STM32_FMC_SDCR1        (STM32_FMC_BASE+STM32_FMC_SDCR1_OFFSET)
#define STM32_FMC_SDCR2        (STM32_FMC_BASE+STM32_FMC_SDCR2_OFFSET)

#define STM32_FMC_SDTR1        (STM32_FMC_BASE+STM32_FMC_SDTR1_OFFSET)
#define STM32_FMC_SDTR2        (STM32_FMC_BASE+STM32_FMC_SDTR2_OFFSET)

#define STM32_FMC_SDCMR        (STM32_FMC_BASE+STM32_FMC_SDCMR_OFFSET)
#define STM32_FMC_SDRTR        (STM32_FMC_BASE+STM32_FMC_SDRTR_OFFSET)
#define STM32_FMC_SDSR         (STM32_FMC_BASE+STM32_FMC_SDSR_OFFSET)

/* Register Bitfield Definitions ****************************************************/

#define FMC_SDRAM_CR_COLBITS_8                       0x00000000
#define FMC_SDRAM_CR_COLBITS_9                       0x00000001
#define FMC_SDRAM_CR_COLBITS_10                      0x00000002
#define FMC_SDRAM_CR_COLBITS_11                      0x00000003

#define FMC_SDRAM_CR_ROWBITS_11                      0x00000000
#define FMC_SDRAM_CR_ROWBITS_12                      0x00000004
#define FMC_SDRAM_CR_ROWBITS_13                      0x00000008

#define FMC_SDRAM_CR_WIDTH_8                         0x00000000
#define FMC_SDRAM_CR_WIDTH_16                        0x00000010
#define FMC_SDRAM_CR_WIDTH_32                        0x00000020

#define FMC_SDRAM_CR_BANKS_2                         0x00000000
#define FMC_SDRAM_CR_BANKS_4                         0x00000040

#define FMC_SDRAM_CR_CASLAT_1                        0x00000080
#define FMC_SDRAM_CR_CASLAT_2                        0x00000100
#define FMC_SDRAM_CR_CASLAT_3                        0x00000180

#define FMC_SDRAM_CR_WRITE_PROT                      0x00000200

#define FMC_SDRAM_CR_SDCLK_DISABLE                   0x00000000
#define FMC_SDRAM_CR_SDCLK_2X                        0x00000800
#define FMC_SDRAM_CR_SDCLK_3X                        0x00000C00

#define FMC_SDRAM_CR_BURST_READ                      0x00001000

#define FMC_SDRAM_CR_RPIPE_0                         0x00000000
#define FMC_SDRAM_CR_RPIPE_1                         0x00002000
#define FMC_SDRAM_CR_RPIPE_2                         0x00004000

#define FMC_SDRAM_TR_TMRD_SHIFT                      0
#define FMC_SDRAM_TR_TXSR_SHIFT                      4
#define FMC_SDRAM_TR_TRAS_SHIFT                      8
#define FMC_SDRAM_TR_TRC_SHIFT                       12
#define FMC_SDRAM_TR_TWR_SHIFT                       16
#define FMC_SDRAM_TR_TRP_SHIFT                       20
#define FMC_SDRAM_TR_TRCD_SHIFT                      24

#define FMC_SDRAM_MODE_CMD_NORMAL                    0
#define FMC_SDRAM_MODE_CMD_CLK_ENABLE                1
#define FMC_SDRAM_MODE_CMD_PALL                      2
#define FMC_SDRAM_MODE_CMD_AUTO_REFRESH              3
#define FMC_SDRAM_MODE_CMD_LOAD_MODE                 4
#define FMC_SDRAM_MODE_CMD_SELF_REFRESH              5
#define FMC_SDRAM_MODE_CMD_POWER_DOWN                6

#define FMC_SDRAM_CMD_BANK_1                         0x00000010
#define FMC_SDRAM_CMD_BANK_2                         0x00000008

#define FMC_SDRAM_AUTO_REFRESH_SHIFT                 5
#define FMC_SDRAM_MODEREG_SHIFT                      9

#define FMC_SDRAM_MODEREG_BURST_LENGTH_1             (0x0000 << FMC_SDRAM_MODEREG_SHIFT)
#define FMC_SDRAM_MODEREG_BURST_LENGTH_2             (0x0001 << FMC_SDRAM_MODEREG_SHIFT)
#define FMC_SDRAM_MODEREG_BURST_LENGTH_4             (0x0002 << FMC_SDRAM_MODEREG_SHIFT)
#define FMC_SDRAM_MODEREG_BURST_LENGTH_8             (0x0004 << FMC_SDRAM_MODEREG_SHIFT)
#define FMC_SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      (0x0000 << FMC_SDRAM_MODEREG_SHIFT)
#define FMC_SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     (0x0008 << FMC_SDRAM_MODEREG_SHIFT)
#define FMC_SDRAM_MODEREG_CAS_LATENCY_2              (0x0020 << FMC_SDRAM_MODEREG_SHIFT)
#define FMC_SDRAM_MODEREG_CAS_LATENCY_3              (0x0030 << FMC_SDRAM_MODEREG_SHIFT)
#define FMC_SDRAM_MODEREG_OPERATING_MODE_STANDARD    (0x0000 << FMC_SDRAM_MODEREG_SHIFT)
#define FMC_SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED (0x0000 << FMC_SDRAM_MODEREG_SHIFT)
#define FMC_SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     (0x0200 << FMC_SDRAM_MODEREG_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32_STM32F7_FMC_H */
