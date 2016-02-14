/************************************************************************************
 * arch/arm/include/stm32f7/chip.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_STM32F7_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32F7_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* STM32F745xx, STM32F746xx, and STM32F56xx.  Differences between family members:
 *
 *   ----------- ---------------- ----- -------- ------------ --------
 *   PART        PACKAGE          GPIOs SPI/I2S  ADC CHANNELS LCD-TFT?
 *   ----------- ---------------- ----- -------- ------------ --------
 *   STM32F745Vx LQFP100            82   4/3         16         No
 *   STM32F745Zx WLCSP143/LQFP144  114   6/3         24         No
 *   STM32F745Ix UFBGA176/LQFP176  140   6/3         24         No
 *   STM32F745Bx LQFP208           168   6/3         24         No
 *   STM32F745Nx TFBGA216           68   6/3         24         No
 *
 *   STM32F746Vx LQFP100            82   4/3         16         Yes
 *   STM32F746Zx WLCSP143/LQFP144  114   6/3         24         Yes
 *   STM32F746Ix UFBGA176/LQFP176  140   6/3         24         Yes
 *   STM32F746Bx LQFP208           168   6/3         24         Yes
 *   STM32F746Nx TFBGA216          168   6/3         24         Yes
 *
 *   STM32F756Vx LQFP100            82   4/3         16         Yes
 *   STM32F756Zx WLCSP143/LQFP144  114   6/3         24         Yes
 *   STM32F756Ix UFBGA176/LQFP176  140   6/3         24         Yes
 *   STM32F756Bx LQFP208           168   6/3         24         Yes
 *   STM32F756Nx TFBGA216          168   6/3         24         Yes
 *   ----------- ---------------- ----- -------- ------------ --------
 *
 * Parts STM32F74xxE have 512Kb of FLASH
 * Parts STM32F74xxG have 1024Kb of FLASH
 *
 * The correct FLASH size must be set with a CONFIG_STM32F7_FLASH_*KB
 * selection.
 */

#if defined(CONFIG_ARCH_CHIP_STM32F745) || defined(CONFIG_ARCH_CHIP_STM32F746) || \
    defined(CONFIG_ARCH_CHIP_STM32F756)

#if defined(CONFIG_ARCH_CHIP_STM32F745)
#  define STM32F7_STM32F745XX              1   /* STM32F745xx family */
#  undef  STM32F7_STM32F746XX                  /* Not STM32F746xx family */
#  undef  STM32F7_STM32F756XX                  /* Not STM32F756xx family */

#  define STM32F7_NLCDTFT                  0   /* No LCD-TFT */

#elif  defined(CONFIG_ARCH_CHIP_STM32F746)

#  undef  STM32F7_STM32F745XX                  /* Not STM32F745xx family */
#  define STM32F7_STM32F746XX              1   /* STM32F746xx family */
#  undef  STM32F7_STM32F756XX                  /* Not STM32F756xx family */

#  define STM32F7_NLCDTFT                  1   /* One LCD-TFT */

#else /* if  defined(CONFIG_ARCH_CHIP_STM32F746) */

#  undef  STM32F7_STM32F745XX                  /* Not STM32F745xx family */
#  undef  STM32F7_STM32F746XX                  /* Not STM32F746xx family */
#  define STM32F7_STM32F756XX              1   /* STM32F756xx family */

#  define STM32F7_NLCDTFT                  1   /* One LCD-TFT */
#endif

#  define STM32F7_SRAM1_SIZE      (240*1024)   /* 240Kb SRAM1 on AHB bus Matrix */
#  define STM32F7_SRAM2_SIZE       (16*1024)   /* 16Kb SRAM2 on AHB bus Matrix */
#  define STM32F7_DTCM_SRAM_SIZE   (64*1024)   /* 64Kb DTCM SRAM on TCM inerface */
#  define STM32F7_ITCM_SRAM_SIZE   (16*1024)   /* 16Kb ITCM SRAM on TCM inerface */

#  define STM32F7_NFSMC                    1   /* Have FSMC memory controller */
#  define STM32F7_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32F7_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32F7_NGTIM32                  2   /* 32-bit general timers TIM2 and 5 with DMA */
#  define STM32F7_NGTIM16                  2   /* 16-bit general timers TIM3 and 4 with DMA */
#  define STM32F7_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32F7_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32F7_NRNG                     1   /* Random number generator (RNG) */
#  define STM32F7_NUART                    4   /* UART 4-5 and 7-8 */
#  define STM32F7_NUSART                   4   /* USART1-3 and 6 */
#  define STM32F7_NSPI                     6   /* SPI1-6 (Except V series) */
#  define STM32F7_NI2S                     3   /* I2S1-2 (multiplexed with SPI1-3) */
#  define STM32F7_NI2C                     4   /* I2C1-4 */
#  define STM32F7_NUSBOTGFS                1   /* USB OTG FS */
#  define STM32F7_NUSBOTGHS                1   /* USB OTG HS */
#  define STM32F7_NCAN                     2   /* CAN1-2 */
#  define STM32F7_NSAI                     2   /* SAI1-2 */
#  define STM32F7_NSPDIFRX                 4   /* 4 SPDIFRX inputs */
#  define STM32F7_NSDMMC                   1   /* SDMMC interface */
#  define STM32F7_NDCMI                    1   /* Digital camera interface (DCMI) */
#  define STM32F7_NDMA                     2   /* DMA1-2 */
#  define STM32F7_NDMA2D                   1   /* DChrom-ART Accelerator™ (DMA2D) */
#  define STM32F7_NGPIO                   11   /* 11 GPIO ports, GPIOA-K */
#  define STM32F7_NADC                     3   /* 12-bit ADC1-3, 24 channels *except V series) */
#  define STM32F7_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32F7_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32F7_NCRC                     1   /* CRC */

#else
#  error STM32 F7 chip not identified
#endif

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
 * using the up_irq_save() inline function to prevent contention in use of
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

#endif /* __ARCH_ARM_INCLUDE_STM32F7_CHIP_H */
