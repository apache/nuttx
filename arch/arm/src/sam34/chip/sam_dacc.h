/****************************************************************************************
 * arch/arm/src/sam34/chip/sam_dacc.h
 * Digital-to-Analog Converter Controller (DACC) for the SAM4E
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM_DACC_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM_DACC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* DACC register offsets *****************************************************************/

#define SAM_DACC_CR_OFFSET         0x0000 /* Control Register */
#define SAM_DACC_MR_OFFSET         0x0004 /* Mode Register */
#define SAM_DACC_CHER_OFFSET       0x0010 /* Channel Enable Register */
#define SAM_DACC_CHDR_OFFSET       0x0014 /* Channel Disable Register */
#define SAM_DACC_CHSR_OFFSET       0x0018 /* Channel Status Register */
#define SAM_DACC_CDR_OFFSET        0x0020 /* Conversion Data Register */
#define SAM_DACC_IER_OFFSET        0x0024 /* Interrupt Enable Register */
#define SAM_DACC_IDR_OFFSET        0x0028 /* Interrupt Disable Register */
#define SAM_DACC_IMR_OFFSET        0x002c /* Interrupt Mask Register */
#define SAM_DACC_ISR_OFFSET        0x0030 /* Interrupt Status Register */
#define SAM_DACC_ACR_OFFSET        0x0094 /* Analog Current Register */
#define SAM_DACC_WPMR_OFFSET       0x00e4 /* Write Protect Mode register */
#define SAM_DACC_WPSR_OFFSET       0x00e8 /* Write Protect Status register */

/* DACC register addresses **************************************************************/

#define SAM_DACC_CR                (SAM_DACC_BASE+SAM_DACC_CR_OFFSET)
#define SAM_DACC_MR                (SAM_DACC_BASE+SAM_DACC_MR_OFFSET)
#define SAM_DACC_CHER              (SAM_DACC_BASE+SAM_DACC_CHER_OFFSET)
#define SAM_DACC_CHDR              (SAM_DACC_BASE+SAM_DACC_CHDR_OFFSET)
#define SAM_DACC_CHSR              (SAM_DACC_BASE+SAM_DACC_CHSR_OFFSET)
#define SAM_DACC_CDR               (SAM_DACC_BASE+SAM_DACC_CDR_OFFSET)
#define SAM_DACC_IER               (SAM_DACC_BASE+SAM_DACC_IER_OFFSET)
#define SAM_DACC_IDR               (SAM_DACC_BASE+SAM_DACC_IDR_OFFSET)
#define SAM_DACC_IMR               (SAM_DACC_BASE+SAM_DACC_IMR_OFFSET)
#define SAM_DACC_ISR               (SAM_DACC_BASE+SAM_DACC_ISR_OFFSET)
#define SAM_DACC_ACR               (SAM_DACC_BASE+SAM_DACC_ACR_OFFSET)
#define SAM_DACC_WPMR              (SAM_DACC_BASE+SAM_DACC_WPMR_OFFSET)
#define SAM_DACC_WPSR              (SAM_DACC_BASE+SAM_DACC_WPSR_OFFSET)

/* DACC register bit definitions ********************************************************/

/* Control Register */

#define DACC_CR_SWRST              (1 << 0)  /* Bit 0:  Software reset */

/* Mode Register */

#define DACC_MR_TRGEN              (1 << 0)  /* Bit 0: Trigger Enable */
#define DACC_MR_TRGSEL_SHIFT       (1)       /* Bits 1-3: Trigger Selection */
#define DACC_MR_TRGSEL_MASK        (7 << DACC_MR_TRGSEL_SHIFT)
#  define DACC_MR_TRGSEL_EXTERN    (0 << DACC_MR_TRGSEL_SHIFT) /* External trigger */
#  define DACC_MR_TRGSEL_TIO0      (1 << DACC_MR_TRGSEL_SHIFT) /* TIO Output of the TC Channel 0 */
#  define DACC_MR_TRGSEL_TIO1      (2 << DACC_MR_TRGSEL_SHIFT) /* TIO Output of the TC Channel 1 */
#  define DACC_MR_TRGSEL_TIO2      (3 << DACC_MR_TRGSEL_SHIFT) /* TIO Output of the TC Channel 2 */
#  define DACC_MR_TRGSEL_PWM0      (4 << DACC_MR_TRGSEL_SHIFT) /* PWM Event Line 0 */
#  define DACC_MR_TRGSEL_PWM1      (5 << DACC_MR_TRGSEL_SHIFT) /* PWM Event Line 1 */
#define DACC_MR_WORD               (1 << 4)  /* Bit 4: Word Transfer */
#define DACC_MR_SLEEP              (1 << 5)  /* Bit 5: Sleep Mode */
#define DACC_MR_FASTWKUP           (1 << 6)  /* Bit 6: Fast Wake up Mode */
#define DACC_MR_REFRESH_SHIFT      (8)       /* Bits 8-15: Refresh Period */
#define DACC_MR_REFRESH_MASK       (0xff << DACC_MR_REFRESH_SHIFT)
#define DACC_MR_USERSEL_SHIFT      (16)       /* Bits 16-17: User Channel Selection */
#define DACC_MR_USERSEL_MASK       (3 << DACC_MR_USERSEL_SHIFT)
#  define DACC_MR_USERSEL_CHAN0    (0 << DACC_MR_USERSEL_SHIFT) /* Channel 0 */
#  define DACC_MR_USERSEL_CHAN1    (1 << DACC_MR_USERSEL_SHIFT) /* Channel 1 */
#define DACC_MR_TAG                (1 << 20)  /* Bit 20: Tag Selection Mode */
#define DACC_MR_MAXS               (1 << 21)  /* Bit 21: Max Speed Mode */
#define DACC_MR_CLKDIV             (1 << 22)  /* Bit 22: Clock Divider */
#  define DACC_MR_CLKDIV_2         (0)             /* DAC clock is MCK divided by 2 */
#  define DACC_MR_CLKDIV_4         DACC_MR_CLKDIV  /* DAC clock is MCK divided by 4 */
#define DACC_MR_STARTUP_SHIFT      (24)       /* Bits 24-29: Startup Time Select */
#define DACC_MR_STARTUP_MASK       (63 << DACC_MR_STARTUP_SHIFT)
#  define DACC_MR_STARTUP_0        (0 << DACC_MR_STARTUP_SHIFT)  /* 0 periods of DACClock */
#  define DACC_MR_STARTUP_8        (1 << DACC_MR_STARTUP_SHIFT)  /* 8 periods of DACClock */
#  define DACC_MR_STARTUP_16       (2 << DACC_MR_STARTUP_SHIFT)  /* 16 periods of DACClock */
#  define DACC_MR_STARTUP_24       (3 << DACC_MR_STARTUP_SHIFT)  /* 24 periods of DACClock */
#  define DACC_MR_STARTUP_64       (4 << DACC_MR_STARTUP_SHIFT)  /* 64 periods of DACClock */
#  define DACC_MR_STARTUP_80       (5 << DACC_MR_STARTUP_SHIFT)  /* 80 periods of DACClock */
#  define DACC_MR_STARTUP_96       (6 << DACC_MR_STARTUP_SHIFT)  /* 96 periods of DACClock */
#  define DACC_MR_STARTUP_112      (7 << DACC_MR_STARTUP_SHIFT)  /* 112 periods of DACClock */
#  define DACC_MR_STARTUP_512      (8 << DACC_MR_STARTUP_SHIFT)  /* 512 periods of DACClock */
#  define DACC_MR_STARTUP_576      (9 << DACC_MR_STARTUP_SHIFT)  /* 576 periods of DACClock */
#  define DACC_MR_STARTUP_640      (10 << DACC_MR_STARTUP_SHIFT) /* 640 periods of DACClock */
#  define DACC_MR_STARTUP_704      (11 << DACC_MR_STARTUP_SHIFT) /* 704 periods of DACClock */
#  define DACC_MR_STARTUP_768      (12 << DACC_MR_STARTUP_SHIFT) /* 768 periods of DACClock */
#  define DACC_MR_STARTUP_832      (13 << DACC_MR_STARTUP_SHIFT) /* 832 periods of DACClock */
#  define DACC_MR_STARTUP_896      (14 << DACC_MR_STARTUP_SHIFT) /* 896 periods of DACClock */
#  define DACC_MR_STARTUP_960      (51 << DACC_MR_STARTUP_SHIFT) /* 960 periods of DACClock */
#  define DACC_MR_STARTUP_1024     (16 << DACC_MR_STARTUP_SHIFT) /* 1024 periods of DACClock */
#  define DACC_MR_STARTUP_1088     (17 << DACC_MR_STARTUP_SHIFT) /* 1088 periods of DACClock */
#  define DACC_MR_STARTUP_1152     (18 << DACC_MR_STARTUP_SHIFT) /* 1152 periods of DACClock */
#  define DACC_MR_STARTUP_1216     (19 << DACC_MR_STARTUP_SHIFT) /* 1216 periods of DACClock */
#  define DACC_MR_STARTUP_1280     (20 << DACC_MR_STARTUP_SHIFT) /* 1280 periods of DACClock */
#  define DACC_MR_STARTUP_1344     (21 << DACC_MR_STARTUP_SHIFT) /* 1344 periods of DACClock */
#  define DACC_MR_STARTUP_1408     (22 << DACC_MR_STARTUP_SHIFT) /* 1408 periods of DACClock */
#  define DACC_MR_STARTUP_1472     (23 << DACC_MR_STARTUP_SHIFT) /* 1472 periods of DACClock */
#  define DACC_MR_STARTUP_1536     (24 << DACC_MR_STARTUP_SHIFT) /* 1536 periods of DACClock */
#  define DACC_MR_STARTUP_1600     (25 << DACC_MR_STARTUP_SHIFT) /* 1600 periods of DACClock */
#  define DACC_MR_STARTUP_1664     (26 << DACC_MR_STARTUP_SHIFT) /* 1664 periods of DACClock */
#  define DACC_MR_STARTUP_1728     (27 << DACC_MR_STARTUP_SHIFT) /* 1728 periods of DACClock */
#  define DACC_MR_STARTUP_1792     (28 << DACC_MR_STARTUP_SHIFT) /* 1792 periods of DACClock */
#  define DACC_MR_STARTUP_1856     (29 << DACC_MR_STARTUP_SHIFT) /* 1856 periods of DACClock */
#  define DACC_MR_STARTUP_1920     (30 << DACC_MR_STARTUP_SHIFT) /* 1920 periods of DACClock */
#  define DACC_MR_STARTUP_1984     (31 << DACC_MR_STARTUP_SHIFT) /* 1984 periods of DACClock */
#  define DACC_MR_STARTUP_2048     (32 << DACC_MR_STARTUP_SHIFT) /* 2048 periods of DACClock */
#  define DACC_MR_STARTUP_2112     (33 << DACC_MR_STARTUP_SHIFT) /* 2112 periods of DACClock */
#  define DACC_MR_STARTUP_2176     (34 << DACC_MR_STARTUP_SHIFT) /* 2176 periods of DACClock */
#  define DACC_MR_STARTUP_2240     (35 << DACC_MR_STARTUP_SHIFT) /* 2240 periods of DACClock */
#  define DACC_MR_STARTUP_2304     (36 << DACC_MR_STARTUP_SHIFT) /* 2304 periods of DACClock */
#  define DACC_MR_STARTUP_2368     (37 << DACC_MR_STARTUP_SHIFT) /* 2368 periods of DACClock */
#  define DACC_MR_STARTUP_2432     (38 << DACC_MR_STARTUP_SHIFT) /* 2432 periods of DACClock */
#  define DACC_MR_STARTUP_2496     (39 << DACC_MR_STARTUP_SHIFT) /* 2496 periods of DACClock */
#  define DACC_MR_STARTUP_2560     (40 << DACC_MR_STARTUP_SHIFT) /* 2560 periods of DACClock */
#  define DACC_MR_STARTUP_2624     (41 << DACC_MR_STARTUP_SHIFT) /* 2624 periods of DACClock */
#  define DACC_MR_STARTUP_2688     (42 << DACC_MR_STARTUP_SHIFT) /* 2688 periods of DACClock */
#  define DACC_MR_STARTUP_2752     (43 << DACC_MR_STARTUP_SHIFT) /* 2752 periods of DACClock */
#  define DACC_MR_STARTUP_2816     (44 << DACC_MR_STARTUP_SHIFT) /* 2816 periods of DACClock */
#  define DACC_MR_STARTUP_2880     (45 << DACC_MR_STARTUP_SHIFT) /* 2880 periods of DACClock */
#  define DACC_MR_STARTUP_2944     (46 << DACC_MR_STARTUP_SHIFT) /* 2944 periods of DACClock */
#  define DACC_MR_STARTUP_3008     (47 << DACC_MR_STARTUP_SHIFT) /* 3008 periods of DACClock */
#  define DACC_MR_STARTUP_3072     (48 << DACC_MR_STARTUP_SHIFT) /* 3072 periods of DACClock */
#  define DACC_MR_STARTUP_3136     (49 << DACC_MR_STARTUP_SHIFT) /* 3136 periods of DACClock */
#  define DACC_MR_STARTUP_3200     (50 << DACC_MR_STARTUP_SHIFT) /* 3200 periods of DACClock */
#  define DACC_MR_STARTUP_3264     (51 << DACC_MR_STARTUP_SHIFT) /* 3264 periods of DACClock */
#  define DACC_MR_STARTUP_3328     (52 << DACC_MR_STARTUP_SHIFT) /* 3328 periods of DACClock */
#  define DACC_MR_STARTUP_3392     (53 << DACC_MR_STARTUP_SHIFT) /* 3392 periods of DACClock */
#  define DACC_MR_STARTUP_3456     (54 << DACC_MR_STARTUP_SHIFT) /* 3456 periods of DACClock */
#  define DACC_MR_STARTUP_3520     (55 << DACC_MR_STARTUP_SHIFT) /* 3520 periods of DACClock */
#  define DACC_MR_STARTUP_3584     (56 << DACC_MR_STARTUP_SHIFT) /* 3584 periods of DACClock */
#  define DACC_MR_STARTUP_3648     (57 << DACC_MR_STARTUP_SHIFT) /* 3648 periods of DACClock */
#  define DACC_MR_STARTUP_3712     (58 << DACC_MR_STARTUP_SHIFT) /* 3712 periods of DACClock */
#  define DACC_MR_STARTUP_3776     (59 << DACC_MR_STARTUP_SHIFT) /* 3776 periods of DACClock */
#  define DACC_MR_STARTUP_3840     (60 << DACC_MR_STARTUP_SHIFT) /* 3840 periods of DACClock */
#  define DACC_MR_STARTUP_3940     (61 << DACC_MR_STARTUP_SHIFT) /* 3904 periods of DACClock */
#  define DACC_MR_STARTUP_3968     (62 << DACC_MR_STARTUP_SHIFT) /* 3968 periods of DACClock */
#  define DACC_MR_STARTUP_4032     (63 << DACC_MR_STARTUP_SHIFT) /* 4032 periods of DACClock */

/* Channel Enable, Channel Disable, and  Channel Status Registers */

#define DACC_CH0                   (1 << 0)  /* Bit 0: Channel 0 */
#define DACC_CH1                   (1 << 1)  /* Bit 1: Channel 1 */

/* Conversion Data Register -- 32-bit data */

/* Interrupt Enable, Interrupt Disable, Interrupt Mask, and Interrupt Status Register */

#define DACC_INT_TXRDY             (1 << 0)  /* Bit 0:  Transmit Ready Interrupt */
#define DACC_INT_EOC               (1 << 1)  /* Bit 1:  End of Conversion Interrupt Flag */
#define DACC_INT_ENDTX             (1 << 2)  /* Bit 2:  End of DMA Interrupt Flag */
#define DACC_INT_TXBUFE            (1 << 3)  /* Bit 3:  Transmit Buffer Empty */

/* Analog Current Register */

#define DACC_ACR_IBCTLCH_SHIFT     (0)       /* Bits 0-1: Analog Output Current Control */
#define DACC_ACR_IBCTLCH_MASK      (3 << DACC_ACR_IBCTLCH_SHIFT
#define DACC_ACR_IBCTLCH0          (1 << 0)  /* Bit 0:  Analog Output Current Control 0 */
#define DACC_ACR_IBCTLCH1          (1 << 1)  /* Bit 1:  Analog Output Current Control 1 */
#define DACC_ACR_IBCTLDACCORE_SHIFT (8)      /* Bits 8-9: Bias Current Control for DAC Core */
#define DACC_ACR_IBCTLDACCORE_MASK  (3 << DACC_ACR_IBCTLDACCORE_SHIFT)
#  define DACC_ACR_IBCTLDACCORE(n)  ((uint32_t)(n) << DACC_ACR_IBCTLDACCORE_SHIFT)

/* Write Protect Mode register */

#define DACC_WPMR_WPEN             (1 << 0)  /* Bit 0:  Write Protect Enable */
#define DACC_WPMR_WPKEY_SHIFT      (8)       /* Bits 8-31: Write Protect KEY */
#define DACC_WPMR_WPKEY_MASK       (0x00ffffff << DACC_WPMR_WPKEY_SHIFT)
#  define DACC_WPMR_WPKEY_MASK     (0x00444143 << DACC_WPMR_WPKEY_SHIFT)

/* Write Protect Status register */

#define DACC_WPSR_WPROTERR         (1 << 0)  /* Bit 0:  Write protection error */
#define DACC_WPSR_WPROTADDR_SHIFT  (8)       /* Bits 8-15: Write protection error address */
#define DACC_WPSR_WPROTADDR_MASK   (0xff << DACC_WPSR_WPROTADDR_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM_DACC_H */
