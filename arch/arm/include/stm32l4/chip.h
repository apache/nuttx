/************************************************************************************
 * arch/arm/include/stm32l4/chip.h
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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

#ifndef __ARCH_ARM_INCLUDE_STM32L4_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32L4_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* STM32F476, STM32F486.  Differences between family members: 486 has AES.
 * 
 *   ----------- ---------------- ----- ------ ------ ---- ---- -----
 *   PART        PACKAGE          GPIOs LCD    Tamper FSMC CapS AdcCh
 *   ----------- ---------------- ----- ------ ------ ---- ---- -----
 *   STM32L4x6Jx WLCSP72L           57   8x28   2     No    12   16
 *   STM32L476Mx WLCSP81L           65   ?      ?     ?     ?    ?
 *   STM32L4x6Qx UFBGA132L         109   8x40   3     Yes   24   16
 *   STM32L4x6Rx LQFP64             51   8x28   2     No    12   16
 *   STM32L4x6Vx LQFP100            82   8x40   3     Yes   21   16
 *   STM32L4x6Zx LQFP144           114   8x40   3     Yes   24   24
 *   ----------- ---------------- ----- ------ ------ ---- ---- -----
 *
 * Parts STM32L4x6xC have 256Kb of FLASH
 * Parts STM32L4x6xE have 512Kb of FLASH
 * Parts STM32L4x6xG have 1024Kb of FLASH
 *
 * The correct FLASH size must be set with a CONFIG_STM32L4_FLASH_*KB
 * selection.
 */

#  define STM32L4_SRAM1_SIZE       (96*1024)   /* 96Kb SRAM1 on AHB bus Matrix */
#  define STM32L4_SRAM2_SIZE       (32*1024)   /* 32Kb SRAM2 on AHB bus Matrix */

#  define STM32L4_NFSMC                    1   /* Have FSMC memory controller */
#  define STM32L4_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32L4_NGTIM32                  2   /* 32-bit general timers TIM2 and 5 with DMA */
#  define STM32L4_NGTIM16                  2   /* 16-bit general timers TIM3 and 4 with DMA */
#  define STM32L4_NGTIMNDMA                3   /* 16-bit general timers TIM15-17 without DMA */
#  define STM32L4_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32L4_NLPTIM                   2   /* Two low-power timers, LPTIM1-2 */
#  define STM32L4_NRNG                     1   /* Random number generator (RNG) */
#  define STM32L4_NUART                    4   /* UART 4-5 */
#  define STM32L4_NUSART                   3   /* USART 1-3 */
#  define STM32L4_NLPUART                  1   /* LPUART 1 */
#  define STM32L4_NSPI                     3   /* SPI1-3 */
#  define STM32L4_NI2C                     3   /* I2C1-3 */
#  define STM32L4_NUSBOTGFS                1   /* USB OTG FS */
#  define STM32L4_NCAN                     1   /* CAN1 */
#  define STM32L4_NSAI                     2   /* SAI1-2 */
#  define STM32L4_NSDMMC                   1   /* SDMMC interface */
#  define STM32L4_NDMA                     2   /* DMA1-2 */
#  define STM32L4_NPORTS                   8   /* 8 GPIO ports, GPIOA-H */
#  define STM32L4_NADC                     3   /* 12-bit ADC1-3, 24 channels *except V series) */
#  define STM32L4_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32L4_NCRC                     1   /* CRC */
#  define STM32L4_NCOMP                    2   /* Comparators */
#  define STM32L4_NOPAMP                   2   /* Operational Amplifiers */

/* NVIC priority levels *************************************************************/
/* 16 Programmable interrupt levels */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

/* If CONFIG_ARMV7M_USEBASEPRI is selected, then interrupts will be disabled
 * by setting the BASEPRI register to NVIC_SYSH_DISABLE_PRIORITY so that most
 * interrupts will not have execution priority.  SVCall must have execution
 * priority in all cases.
 *
 * In the normal cases, interrupts are not nest-able and all interrupts run
 * at an execution priority between NVIC_SYSH_PRIORITY_MIN and
 * NVIC_SYSH_PRIORITY_MAX (with NVIC_SYSH_PRIORITY_MAX reserved for SVCall).
 *
 * If, in addition, CONFIG_ARCH_HIPRI_INTERRUPT is defined, then special
 * high priority interrupts are supported.  These are not "nested" in the
 * normal sense of the word.  These high priority interrupts can interrupt
 * normal processing but execute outside of OS (although they can "get back
 * into the game" via a PendSV interrupt).
 *
 * In the normal course of things, interrupts must occasionally be disabled
 * using the irqsave() inline function to prevent contention in use of
 * resources that may be shared between interrupt level and non-interrupt
 * level logic.  Now the question arises, if CONFIG_ARCH_HIPRI_INTERRUPT,
 * do we disable all interrupts (except SVCall), or do we only disable the
 * "normal" interrupts.  Since the high priority interrupts cannot interact
 * with the OS, you may want to permit the high priority interrupts even if
 * interrupts are disabled.  The setting CONFIG_ARCH_INT_DISABLEALL can be
 * used to select either behavior:
 *
 *   ----------------------------+--------------+----------------------------
 *   CONFIG_ARCH_HIPRI_INTERRUPT |      NO      |             YES
 *   ----------------------------+--------------+--------------+-------------
 *   CONFIG_ARCH_INT_DISABLEALL  |     N/A      |     YES      |      NO
 *   ----------------------------+--------------+--------------+-------------
 *                               |              |              |    SVCall
 *                               |    SVCall    |    SVCall    |    HIGH
 *   Disable here and below --------> MAXNORMAL ---> HIGH --------> MAXNORMAL
 *                               |              |    MAXNORMAL |
 *   ----------------------------+--------------+--------------+-------------
 */

#if defined(CONFIG_ARCH_HIPRI_INTERRUPT) && defined(CONFIG_ARCH_INT_DISABLEALL)
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + 2*NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_HIGH_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#else
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       NVIC_SYSH_PRIORITY_MAX
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_MAXNORMAL_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#endif

#endif /* __ARCH_ARM_INCLUDE_STM32L4_CHIP_H */
