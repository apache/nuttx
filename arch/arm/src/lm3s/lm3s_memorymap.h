/************************************************************************************
 * arch/arm/src/lm3s/lm3s_memorymap.h
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

#ifndef __ARCH_ARM_SRC_LM3S_LM3S_MEMORYMAP_H
#define __ARCH_ARM_SRC_LM3S_LM3S_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Memory map ***********************************************************************/
 
#ifdef CONFIG_ARCH_CHIP_LM3S6918
#  define LM3S_FLASH_BASE     0x00000000 /* -0x0003ffff: On-chip FLASH */
                                         /* -0x1fffffff: Reserved */
#  define LM3S_SRAM_BASE      0x20000000 /* -0x2000ffff: Bit-banded on-chip SRAM */
                                         /* -0x21ffffff: Reserved */
#  define LM3S_ASRAM_BASE     0x22000000 /* -0x221fffff: Bit-band alias of 20000000- */
                                         /* -0x3fffffff: Reserved */
#  define LM3S_FPERIPH_BASE   0x40000000 /* -0x4001ffff: FiRM Peripherals */
#  define LM3S_PERIPH_BASE    0x40020000 /* -0x41ffffff: Peripherals */
#  define LM3S_APERIPH_BASE   0x42000000 /* -0x43ffffff: Bit-band alise of 40000000- */
                                         /* -0xdfffffff: Reserved */
#  define LM3S_ITM_BASE       0xe0000000 /* -0xe0000fff: Instrumentation Trace Macrocell */
#  define LM3S_DWT_BASE       0xe0001000 /* -0xe0001fff: Data Watchpoint and Trace */
#  define LM3S_FPB_BASE       0xe0002000 /* -0xe0002fff: Flash Patch and Breakpoint */
                                         /* -0xe000dfff: Reserved */
#  define LM3S_NVIC_BASE      0xe000e000 /* -0xe000efff: Nested Vectored Interrupt Controller */
                                         /* -0xe003ffff: Reserved */
#  define LM3S_TPIU_BASE      0xe0040000 /* -0xe0040fff: Trace Port Interface Unit */
                                         /* -0xffffffff: Reserved */
#else
#  error "Memory map not specified for this LM3S chip"
#endif

/* Peripheral base addresses ********************************************************/

#ifdef CONFIG_ARCH_CHIP_LM3S6918
/* FiRM Peripheral Base Addresses */

#  define LM3S_WDOG_BASE      (LM3S_FPERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer */
                                                          /* -0x03fff: Reserved */
#  define LM3S_GPIOA_BASE     (LM3S_FPERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define LM3S_GPIOB_BASE     (LM3S_FPERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define LM3S_GPIOC_BASE     (LM3S_FPERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define LM3S_GPIOD_BASE     (LM3S_FPERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define LM3S_SSI0_BASE      (LM3S_FPERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
#  define LM3S_SSI1_BASE      (LM3S_FPERIPH_BASE + 0x09000) /* -0x09fff: SSI1 */
                                                            /* -0x0bfff: Reserved */
#  define LM3S_UART0_BASE     (LM3S_FPERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define LM3S_UART1_BASE     (LM3S_FPERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
                                                            /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define LM3S_I2CM0_BASE     (LM3S_PERIPH_BASE + 0x00000)  /* -0x007ff: I2C Master 0 */
#  define LM3S_I2CS0_BASE     (LM3S_PERIPH_BASE + 0x00800)  /* -0x00fff: I2C Slave 0 */
#  define LM3S_I2CM1_BASE     (LM3S_PERIPH_BASE + 0x01000)  /* -0x017ff: I2C Master 1 */
#  define LM3S_I2CS1_BASE     (LM3S_PERIPH_BASE + 0x01800)  /* -0x01fff: I2C Slave 1 */
                                                            /* -0x03fff: Reserved */
#  define LM3S_GPIOE_BASE     (LM3S_PERIPH_BASE + 0x04000)  /* -0x04fff: GPIO Port E */
#  define LM3S_GPIOF_BASE     (LM3S_PERIPH_BASE + 0x05000)  /* -0x05fff: GPIO Port F */
#  define LM3S_GPIOG_BASE     (LM3S_PERIPH_BASE + 0x06000)  /* -0x06fff: GPIO Port G */
#  define LM3S_GPIOH_BASE     (LM3S_PERIPH_BASE + 0x07000)  /* -0x07fff: GPIO Port H */
                                                            /* -0x0ffff: Reserved */
#  define LM3S_TIMER0_BASE    (LM3S_PERIPH_BASE + 0x10000)  /* -0x10fff: Timer 0 */
#  define LM3S_TIMER1_BASE    (LM3S_PERIPH_BASE + 0x11000)  /* -0x11fff: Timer 1 */
#  define LM3S_TIMER2_BASE    (LM3S_PERIPH_BASE + 0x12000)  /* -0x12fff: Timer 2 */
#  define LM3S_TIMER3_BASE    (LM3S_PERIPH_BASE + 0x13000)  /* -0x13fff: Timer 3 */
                                                            /* -0x17fff: Reserved */
#  define LM3S_ADC_BASE       (LM3S_PERIPH_BASE + 0x18000)  /* -0x18fff: ADC */
                                                            /* -0x1bfff: Reserved */
#  define LM3S_COMPARE_BASE   (LM3S_PERIPH_BASE + 0x1c000)  /* -0x1cfff: Analog Comparators */
                                                            /* -0x27fff: Reserved */
#  define LM3S_ETHCON_BASE    (LM3S_PERIPH_BASE + 0x28000)  /* -0x28fff: Ethernet Controller */
                                                            /* -0xdcfff: Reserved */
#  define LM3S_HIBERNATE_BASE (LM3S_PERIPH_BASE + 0xdc000)  /* -0xdcfff: Ethernet Controller */
#  define LM3S_FLASHCON_BASE  (LM3S_PERIPH_BASE + 0xdd000)  /* -0xddfff: FLASH Control */
#  define LM3S_SYSCON_BASE    (LM3S_PERIPH_BASE + 0xde000)  /* -0xdefff: System Control */
                                                            /* -0x1fdffff: Reserved */
#else
#  error "Peripheral base addresses not specified for this LM3S chip"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LM3S_LM3S_MEMORYMAP_H */
