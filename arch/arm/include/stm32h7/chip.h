/************************************************************************************
 * arch/arm/include/stm32h7/chip.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Simon Laube <simon@leitwert.ch>
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

#ifndef __ARCH_ARM_INCLUDE_STM32H7_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32H7_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* STM32H7x3xx  Differences between family members:
 *
 *   ----------- ----------------
 *
 *   PART        PACKAGE
 *   ----------- ----------------
 *   STM32H7x3Zx LQFP144
 *   ----------- ----------------
 *
 * Parts STM32H7xxxI have 2048Kb of FLASH
 *
 * The correct FLASH size will be set CONFIG_STM32H7_FLASH_CONFIG_x or overridden
 * with CONFIG_STM32H7_FLASH_OVERRIDE_x
 */

#if defined(CONFIG_ARCH_CHIP_STM32H743ZI)
#else
#  error STM32 H7 chip not identified
#endif

/* Size SRAM */

#if defined(CONFIG_STM32H7_STM32H7X3XX)
/* Memory */

#    define STM32H7_SRAM_SIZE             (512*1024)  /* 512Kb SRAM on AXI bus Matrix (D1) */
#    define STM32H7_SRAM1_SIZE            (128*1024)  /* 128Kb SRAM1 on AHB bus Matrix (D2) */
#    define STM32H7_SRAM2_SIZE            (128*1024)  /* 128Kb SRAM2 on AHB bus Matrix (D2) */
#    define STM32H7_SRAM3_SIZE            (32*1024)   /*  32Kb SRAM3 on AHB bus Matrix (D2) */
#    define STM32H7_SRAM123_SIZE          (288*1024)  /* 128Kb SRAM123 on AHB bus Matrix (D2) */
#    define STM32H7_SRAM4_SIZE            (64*1024)   /*  64Kb SRAM2 on AHB bus Matrix (D3) */
#  if defined(CONFIG_ARMV7M_HAVE_DTCM)
#      define STM32H7_DTCM_SRAM_SIZE      (128*1024)  /* 128Kb DTCM SRAM on TCM interface */
#  else
#      define STM32H7_DTCM_SRAM_SIZE      (0)         /* No DTCM SRAM on TCM interface */
#  endif
#  if defined(CONFIG_ARMV7M_HAVE_ITCM)
#      define STM32H7_ITCM_SRAM_SIZE      (64*1024)   /*  64b ITCM SRAM on TCM interface */
#  else
#      define STM32H7_ITCM_SRAM_SIZE      (0)         /* No ITCM SRAM on TCM interface */
#  endif

/* Peripherals */

#  define STM32H7_NGPIO                   (11)        /* GPIOA-GPIOK */
#  define STM32H7_NUSART                  (4)         /* USART1-3, 6 */
#  define STM32H7_NUART                   (4)         /* UART4-5, 7-8 */
#else
#  error STM32 H7 chip Family not identified
#endif

/* TBD FPU Configuration */

#if defined(CONFIG_ARCH_HAVE_FPU)
#else
#endif

#if defined(CONFIG_ARCH_HAVE_DPFPU)
#else
#endif

/* Diversification based on Family and package */

// TODO:
// #if defined(CONFIG_STM32F7_HAVE_FMC)
// #  define STM32F7_NFMC                 1           /* Have FMC memory controller */
// #else
// #  define STM32F7_NFMC                 0           /* No FMC memory controller */
// #endif

/* NVIC priority levels *************************************************************/
/* 16 Programmable interrupt levels */

// TODO: check this
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

// TODO: check this
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

#endif /* __ARCH_ARM_INCLUDE_STM32H7_CHIP_H */
