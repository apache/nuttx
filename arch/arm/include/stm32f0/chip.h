/************************************************************************************
 * arch/arm/include/stm32f0/chip.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Alan Carvalho de Assis <acassis@gmail.com>
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

#ifndef __ARCH_ARM_INCLUDE_STM32F0_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32F0_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_STM32F051R8)
#  define STM32F051x              1  /* STM32F051x family */
#  undef  STM32F072x                 /* Not STM32F072x family */
#  undef  STM32F091x                 /* Not STM32F091x family */

#  define STM32F0_FLASH_SIZE      (64*1024) /* 64Kb */
#  define STM32F0_SRAM_SIZE       (8*1024)  /*  8Kb */

#  define STM32F0_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32F0_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32F0_NI2C            2  /* Two I2C modules */
#  define STM32F0_NUSART          2  /* Two USARTs modules */
#  define STM32F0_NCAN            0  /* No CAN controllers */
#  define STM32F0_NUSBDEV         1  /* One USB device controller */
#  define STM32F0_NDAC            1  /* One DAC module */
#  define STM32F0_NDACCHAN        1  /* One DAC channels */
#  define STM32F0_NCOMP           2  /* Two Analog Comparators */
#  define STM32F0_NCAP            13 /* Capacitive sensing channels (14 on UFQFPN32)) */
#  define STM32F0_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F072C8) || defined(CONFIG_ARCH_CHIP_STM32F072CB)
#  undef  STM32F051x                 /* Not STM32F051x family */
#  define STM32F072x              1  /* STM32F072x family */
#  undef  STM32F091x                 /* Not STM32F091x family */

#  ifdef CONFIG_ARCH_CHIP_STM32F072C8
#    define STM32F0_FLASH_SIZE    (64*1024)  /*  64Kb */
#  else
#    define STM32F0_FLASH_SIZE    (128*1024) /* 128Kb */
#  endif
#  define STM32F0_SRAM_SIZE       (16*1024)  /*  16Kb */

#  define STM32F0_NATIM           1  /* One advanced timer TIM1 */
#  define STM32F0_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14-17 */
#  define STM32F0_NGTIM32         1  /* 32-bit general up/down timers TIM2 */
#  define STM32F0_NBTIM           2  /* 2 basic timers: TIM6, TIM7 */
#  define STM32F0_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32F0_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32F0_NI2C            2  /* Two I2C modules */
#  define STM32F0_NUSART          4  /* Four USARTs module */
#  define STM32F0_NCAN            1  /* One CAN controller */
#  define STM32F0_NUSBDEV         1  /* One USB device controller */
#  define STM32F0_NCEC            1  /* One HDMI-CEC controller */
#  define STM32F0_NADC12          1  /* One 12-bit module */
#  define STM32F0_NADCCHAN        10 /* Ten external channels */
#  define STM32F0_NADCINT         3  /* Three internal channels */
#  define STM32F0_NDAC            1  /* One DAC module */
#  define STM32F0_NDACCHAN        2  /* Two DAC channels */
#  define STM32F0_NCOMP           2  /* Two Analog Comparators */
#  define STM32F0_NCAP            17 /* Capacitive sensing channels */
#  define STM32F0_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F072R8) || defined(CONFIG_ARCH_CHIP_STM32F072RB)
#  undef  STM32F051x                 /* Not STM32F051x family */
#  define STM32F072x              1  /* STM32F072x family */
#  undef  STM32F091x                 /* Not STM32F091x family */

#  ifdef CONFIG_ARCH_CHIP_STM32F072R8
#    define STM32F0_FLASH_SIZE    (64*1024)  /*  64Kb */
#  else
#    define STM32F0_FLASH_SIZE    (128*1024) /* 128Kb */
#  endif
#  define STM32F0_SRAM_SIZE       (16*1024)  /*  16Kb */

#  define STM32F0_NATIM           1  /* One advanced timer TIM1 */
#  define STM32F0_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14-17 */
#  define STM32F0_NGTIM32         1  /* 32-bit general up/down timers TIM2 */
#  define STM32F0_NBTIM           2  /* 2 basic timers: TIM6, TIM7 */
#  define STM32F0_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32F0_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32F0_NI2C            2  /* Two I2C modules */
#  define STM32F0_NUSART          4  /* Four USARTs module */
#  define STM32F0_NCAN            1  /* One CAN controller */
#  define STM32F0_NUSBDEV         1  /* One USB device controller */
#  define STM32F0_NCEC            1  /* One HDMI-CEC controller */
#  define STM32F0_NADC12          1  /* One 12-bit module */
#  define STM32F0_NADCCHAN        16 /* 16 external channels */
#  define STM32F0_NADCINT         3  /* Three internal channels */
#  define STM32F0_NDAC            1  /* One DAC module */
#  define STM32F0_NDACCHAN        2  /* Two DAC channels */
#  define STM32F0_NCOMP           2  /* Two Analog Comparators */
#  define STM32F0_NCAP            18 /* Capacitive sensing channels */
#  define STM32F0_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F072V8) || defined(CONFIG_ARCH_CHIP_STM32F072VB)
#  undef  STM32F051x                 /* Not STM32F051x family */
#  define STM32F072x              1  /* STM32F072x family */
#  undef  STM32F091x                 /* Not STM32F091x family */

#  ifdef CONFIG_ARCH_CHIP_STM32F072V8
#    define STM32F0_FLASH_SIZE    (64*1024)  /*  64Kb */
#  else
#    define STM32F0_FLASH_SIZE    (128*1024) /* 128Kb */
#  endif
#  define STM32F0_SRAM_SIZE       (16*1024)  /*  16Kb */

#  define STM32F0_NATIM           1  /* One advanced timer TIM1 */
#  define STM32F0_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14-17 */
#  define STM32F0_NGTIM32         1  /* 32-bit general up/down timers TIM2 */
#  define STM32F0_NBTIM           2  /* 2 basic timers: TIM6, TIM7 */
#  define STM32F0_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32F0_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32F0_NI2C            2  /* Two I2C modules */
#  define STM32F0_NUSART          4  /* Four USARTs module */
#  define STM32F0_NCAN            1  /* One CAN controller */
#  define STM32F0_NUSBDEV         1  /* One USB device controller */
#  define STM32F0_NCEC            1  /* One HDMI-CEC controller */
#  define STM32F0_NADC12          1  /* One 12-bit module */
#  define STM32F0_NADCCHAN        16 /* 16 external channels */
#  define STM32F0_NADCINT         3  /* Three internal channels */
#  define STM32F0_NDAC            1  /* One DAC module */
#  define STM32F0_NDACCHAN        2  /* Two DAC channels */
#  define STM32F0_NCOMP           2  /* Two Analog Comparators */
#  define STM32F0_NCAP            24 /* Capacitive sensing channels */
#  define STM32F0_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F091CB) || defined(CONFIG_ARCH_CHIP_STM32F091CC)
#  undef  STM32F051x                 /* Not STM32F051x family */
#  undef  STM32F072x                 /* Not STM32F072x family */
#  define STM32F091x              1  /* STM32F091x family */

#  ifdef CONFIG_ARCH_CHIP_STM32F091CB
#    define STM32F0_FLASH_SIZE    (128*1024) /* 128Kb */
#  else
#    define STM32F0_FLASH_SIZE    (256*1024) /* 256Kb */
#  endif
#  define STM32F0_SRAM_SIZE       (32*1024)  /*  32Kb */

#  define STM32F0_NATIM           1  /* One advanced timer TIM1 */
#  define STM32F0_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14-17 */
#  define STM32F0_NGTIM32         1  /* 32-bit general up/down timers TIM2 */
#  define STM32F0_NBTIM           2  /* 2 basic timers: TIM6, TIM7 */
#  define STM32F0_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32F0_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32F0_NI2C            2  /* Two I2C modules */
#  define STM32F0_NUSART          6  /* Six USARTs modules */
#  define STM32F0_NCAN            1  /* One CAN controller */
#  define STM32F0_NUSBDEV         0  /* No USB device controller */
#  define STM32F0_NCEC            1  /* One HDMI-CEC controller */
#  define STM32F0_NADC12          1  /* One 12-bit module */
#  define STM32F0_NADCCHAN        10 /* 10 external channels */
#  define STM32F0_NADCINT         3  /* Three internal channels */
#  define STM32F0_NDAC            1  /* One DAC module */
#  define STM32F0_NDACCHAN        2  /* Two DAC channels */
#  define STM32F0_NCOMP           2  /* Two Analog Comparators */
#  define STM32F0_NCAP            17 /* Capacitive sensing channels */
#  define STM32F0_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#elif defined(CONFIG_ARCH_CHIP_STM32F091RB) || defined(CONFIG_ARCH_CHIP_STM32F091RC) \
   || defined(CONFIG_ARCH_CHIP_STM32F091VB) || defined(CONFIG_ARCH_CHIP_STM32F091VC)
#  undef  STM32F051x                 /* Not STM32F051x family */
#  undef  STM32F072x                 /* Not STM32F072x family */
#  define STM32F091x              1  /* STM32F091x family */

#  if defined(CONFIG_ARCH_CHIP_STM32F091RB) || defined(CONFIG_ARCH_CHIP_STM32F091VB)
#    define STM32F0_FLASH_SIZE    (128*1024) /* 128Kb */
#  else
#    define STM32F0_FLASH_SIZE    (256*1024) /* 256Kb */
#  endif
#  define STM32F0_SRAM_SIZE       (32*1024)  /*  32Kb */

#  define STM32F0_NATIM           1  /* One advanced timer TIM1 */
#  define STM32F0_NGTIM16         5  /* 16-bit general up/down timers TIM3, TIM14-17 */
#  define STM32F0_NGTIM32         1  /* 32-bit general up/down timers TIM2 */
#  define STM32F0_NBTIM           2  /* 2 basic timers: TIM6, TIM7 */
#  define STM32F0_NSPI            2  /* Two SPI modules (SPI or I2S) */
#  define STM32F0_NI2S            2  /* Two I2S modules (SPI or I2S) */
#  define STM32F0_NI2C            2  /* Two I2C modules */
#  define STM32F0_NUSART          8  /* Eight USARTs modules */
#  define STM32F0_NCAN            1  /* One CAN controller */
#  define STM32F0_NUSBDEV         0  /* No USB device controller */
#  define STM32F0_NCEC            1  /* One HDMI-CEC controller */
#  define STM32F0_NADC12          1  /* One 12-bit module */
#  define STM32F0_NADCCHAN        16 /* 16 external channels */
#  define STM32F0_NADCINT         3  /* Three internal channels */
#  define STM32F0_NDAC            1  /* One DAC module */
#  define STM32F0_NDACCHAN        2  /* Two DAC channels */
#  define STM32F0_NCOMP           2  /* Two Analog Comparators */
#  if defined(CONFIG_ARCH_CHIP_STM32F091VB) || defined(CONFIG_ARCH_CHIP_STM32F091VC)
#    define STM32F0_NCAP          24 /* Capacitive sensing channels */
#  else
#    define STM32F0_NCAP          18 /* Capacitive sensing channels */
#  endif
#  define STM32F0_NPORTS          6  /* Six GPIO ports, GPIOA-F */

#else
#  error "Unsupported STM32F0xx chip"
#endif

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0-31. The lower the value, the greater
 * the priority of the corresponding interrupt. The processor implements only
 * bits[7:6] of each field, bits[5:0] read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xc0 /* All bits[7:6] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x40 /* Two bits of interrupt priority used */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_STM32F0_CHIP_H */
