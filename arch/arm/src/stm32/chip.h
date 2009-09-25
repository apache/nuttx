/************************************************************************************
 * arch/arm/src/stm32/chip.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_H
#define __ARCH_ARM_SRC_STM32_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Get customizations for each supported chip (only the STM32F103Z right now) */

#ifdef CONFIG_ARCH_CHIP_STM32F103ZET6
#  undef CONFIG_STM32_LOWDENSITY            /* STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY        /* STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  define CONFIG_STM32_HIGHDENSITY      1   /* STM32F101x  and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_CONNECTIVITYLINE     /* STM32F105x and STM32F107x */
#  define STM32_NATIM                   2   /* Advanced timers TIM1,8 */
#  define STM32_NGTIM                   4   /* General timers TIM2,3,4,5 */
#  define STM32 NBTIM                   2   /* Basic timers TIM6,7 */
#  define STM32_NSPI                    1   /* SPI1 */
#  define STM32_NUSART                  5   /* USART1-3, UART4-5 */
#  define STM32_NI2C                    2   /* I2C1-2 */
#  define STM32_NCAN                    1   /* bxCAN1 */
#  define STM32_NSDIO                   1   /* 1 */
#  define STM32_NGPIO                   112 /* GPIOA-G */
#  define STM32_NADC                    3   /* ADC 1-3 */
#  define STM32_NDAC                    2   /* No DAC */
#  define STM32_NCRC                    0   /* No CRC */
#  define STM32_NTHERNET                0   /* No ethernet */
#else
#  error "Unsupported STM32 chip"
#endif

/* Include only the memory map.  Other chip hardware files should then include this
 * file for the proper setup
 */

#include "stm32_memorymap.h"

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_CHIP_H */
