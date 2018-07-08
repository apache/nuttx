/************************************************************************************
 * arch/arm/src/stm32h7/chip/stm32h7x3xx_pwr.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Mateusz Szafoni <raiden00@railab.me>
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

#ifndef __ARCH_ARM_SRC_STM327_CHIP_STM32H7X3XX_PWR_H
#define __ARCH_ARM_SRC_STM327_CHIP_STM32H7X3XX_PWR_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_PWR_CR1_OFFSET     0x0000  /* Power control register 1 */
#define STM32_PWR_CSR1_OFFSET    0x0004  /* Power control/status register 1 */
#define STM32_PWR_CR2_OFFSET     0x0008  /* Power control register 2 */
#define STM32_PWR_CR3_OFFSET     0x000c  /* Power control register 3 */
#define STM32_PWR_CPUCR_OFFSET   0x0010  /* Power CPU control register */
                                         /* 0x014: Reserved */
#define STM32_PWR_D3CR_OFFSET    0x0018  /* Power D3 domain control register */
#define STM32_PWR_WKUPCR_OFFSET  0x0020  /* Power wakeup clear register */
#define STM32_PWR_WKUPFR_OFFSET  0x0024  /* Power wakeup flag register */
#define STM32_PWR_WKUPEPR_OFFSET 0x0028  /* Power wakeup enable and polarity register*/
                                         /* 0x030: Reserved */

/* Register Addresses ***************************************************************/

#define STM32_PWR_CR1            (STM32_PWR_BASE+STM32_PWR_CR1_OFFSET)
#define STM32_PWR_CSR1           (STM32_PWR_BASE+STM32_PWR_CSR1_OFFSET)
#define STM32_PWR_CR2            (STM32_PWR_BASE+STM32_PWR_CR2_OFFSET)
#define STM32_PWR_CR3            (STM32_PWR_BASE+STM32_PWR_CR3_OFFSET)
#define STM32_PWR_CPUCR          (STM32_PWR_BASE+STM32_PWR_CPUCR_OFFSET)
#define STM32_PWR_D3CR           (STM32_PWR_BASE+STM32_PWR_D3CR_OFFSET)
#define STM32_PWR_WKUPCR         (STM32_PWR_BASE+STM32_PWR_WKUPCR_OFFSET)
#define STM32_PWR_WKUPFR         (STM32_PWR_BASE+STM32_PWR_WKUPFR_OFFSET)
#define STM32_PWR_WKUPEOR        (STM32_PWR_BASE+STM32_PWR_WKUPEOR_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* Power control register 1 (CR1) */

#define PWR_CR1_LPDS             (1 << 0) /* Bit 0: Low-power Deepsleep with SVOS3 */
                                          /* Bits 1-3: Reserved */
#define PWR_CR1_PVDE             (1 << 4) /* Bit 4: Programmable voltage detector enable */


#define PWR_CR1_PLS_SHIFT         (5)    /* Bits 5-7: Programmable voltage detector level */
#define PWR_CR1_PLS_MASK          (7 << PWR_CR1_PLS_SHIFT)
#  define PWR_CR1_PLS_1V95        (0 << PWR_CR1_PLS_SHIFT) /* 000: */
#  define PWR_CR1_PLS_2V1         (1 << PWR_CR1_PLS_SHIFT) /* 001: */
#  define PWR_CR1_PLS_2V25        (2 << PWR_CR1_PLS_SHIFT) /* 010: */
#  define PWR_CR1_PLS_2V4         (3 << PWR_CR1_PLS_SHIFT) /* 011: */
#  define PWR_CR1_PLS_2V55        (4 << PWR_CR1_PLS_SHIFT) /* 100: */
#  define PWR_CR1_PLS_2V7         (5 << PWR_CR1_PLS_SHIFT) /* 101: */
#  define PWR_CR1_PLS_2V85        (6 << PWR_CR1_PLS_SHIFT) /* 110: */
#  define PWR_CR1_PLS_EXT         (7 << PWR_CR1_PLS_SHIFT) /* 111: */
#define PWR_CR1_DBP               (1 << 8)  /* Bit 8: Disable backup domain write protection */
#define PWR_CR1_FLPS              (1 << 9)  /* Bit 9: */
                                            /* Bits 10-13: Reserved */
#define PWR_CR1_SVOS_SHIFT        (1 << 14) /* Bits 14-15: */
#define PWR_CR1_SVOS_MASK         (3 << PWR_CR1_SVOS_SHIFT)
                                                            /* 00: Reserved */
#  define PWR_CR1_SVOS_S5         (1 << PWR_CR1_SVOS_SHIFT) /* 01:  */
#  define PWR_CR1_SVOS_S4         (2 << PWR_CR1_SVOS_SHIFT) /* 10: */
#  define PWR_CR1_SVOS_S3         (3 << PWR_CR1_SVOS_SHIFT) /* 11: */
#define PWR_CR1_AVDEN             (1 << 16) /* Bit 16: */
#define PWR_CR1_ALS_SHIFT         (17)      /* Bits 17-18: Analog voltage detector level selection */
#define PWR_CR1_ALS_MASK          (3 << PWR_CR1_ALS_SHIFT)
#  define PWR_CR1_ALS_1V7         (0 << PWR_CR1_ALS_SHIFT) /* 00: */
#  define PWR_CR1_ALS_2V1         (1 << PWR_CR1_ALS_SHIFT) /* 01 */
#  define PWR_CR1_ALS_2V5         (2 << PWR_CR1_ALS_SHIFT) /* 10: */
#  define PWR_CR1_ALS_2V8         (3 << PWR_CR1_ALS_SHIFT) /* 11: */
                                            /* Bits 19-31: Reserved */

/* Power control/status register 1 (CRS1) */

/* Power control register 2 (CR2) */

/* Power control register 3 (CR3) */

/* Power CPU control register (CPUCR) */

/* Power D3 domain control register (D3CR) */

/* Power wakeup clear register (WKUPCR) */

/* Power wakeup flag register (WKUPFR) */

/* Power wakeup enable and polarity register (WKUPEPR) */

#endif /* __ARCH_ARM_SRC_STM327_CHIP_STM32H7X3XX_PWR_H */
