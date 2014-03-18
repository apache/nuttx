/************************************************************************************
 * arch/arm/src/tiva/chip/tm4c_memorymap.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_TIVA_CHIP_TM4C_MEMORYMAP_H
#define __ARCH_ARM_SRC_TIVA_CHIP_TM4C_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Memory map ***********************************************************************/

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)
#  define TIVA_FLASH_BASE     0x00000000 /* -0x0003ffff: On-chip FLASH */
                                         /* -0x00ffffff: Reserved */
#  define TIVA_ROM_BASE       0x01000000 /* -0x1fffffff: Reserved for ROM */
#  define TIVA_SRAM_BASE      0x20000000 /* -0x20007fff: Bit-banded on-chip SRAM */
                                         /* -0x21ffffff: Reserved */
#  define TIVA_ASRAM_BASE     0x22000000 /* -0x220fffff: Bit-band alias of 20000000- */
                                         /* -0x3fffffff: Reserved */
#  define TIVA_PERIPH_BASE    0x40000000 /* -0x4001ffff: FiRM Peripherals */
                                         /* -0x41ffffff: Peripherals */
#  define TIVA_APERIPH_BASE   0x42000000 /* -0x43ffffff: Bit-band alias of 40000000- */
                                         /* -0xdfffffff: Reserved */
#  define TIVA_ITM_BASE       0xe0000000 /* -0xe0000fff: Instrumentation Trace Macrocell */
#  define TIVA_DWT_BASE       0xe0001000 /* -0xe0001fff: Data Watchpoint and Trace */
#  define TIVA_FPB_BASE       0xe0002000 /* -0xe0002fff: Flash Patch and Breakpoint */
                                         /* -0xe000dfff: Reserved */
#  define TIVA_NVIC_BASE      0xe000e000 /* -0xe000efff: Nested Vectored Interrupt Controller */
                                         /* -0xe003ffff: Reserved */
#  define TIVA_TPIU_BASE      0xe0040000 /* -0xe0040fff: Trace Port Interface Unit */
#  define TIVA_ETM_BASE       0xe0041000 /* -0xe0041fff: Embedded Trace Macrocell */
                                         /* -0xffffffff: Reserved */

#elif defined(CONFIG_ARCH_CHIP_TM4C123GH6PMI)
#  define TIVA_FLASH_BASE     0x00000000 /* -0x0003ffff: On-chip FLASH */
                                         /* -0x00ffffff: Reserved */
#  define TIVA_ROM_BASE       0x01000000 /* -0x1fffffff: Reserved for ROM */
#  define TIVA_SRAM_BASE      0x20000000 /* -0x20007fff: Bit-banded on-chip SRAM */
                                         /* -0x21ffffff: Reserved */
#  define TIVA_ASRAM_BASE     0x22000000 /* -0x220fffff: Bit-band alias of 20000000- */
                                         /* -0x3fffffff: Reserved */
#  define TIVA_PERIPH_BASE    0x40000000 /* -0x4001ffff: FiRM Peripherals */
                                         /* -0x41ffffff: Peripherals */
#  define TIVA_APERIPH_BASE   0x42000000 /* -0x43ffffff: Bit-band alias of 40000000- */
                                         /* -0xdfffffff: Reserved */
#  define TIVA_ITM_BASE       0xe0000000 /* -0xe0000fff: Instrumentation Trace Macrocell */
#  define TIVA_DWT_BASE       0xe0001000 /* -0xe0001fff: Data Watchpoint and Trace */
#  define TIVA_FPB_BASE       0xe0002000 /* -0xe0002fff: Flash Patch and Breakpoint */
                                         /* -0xe000dfff: Reserved */
#  define TIVA_NVIC_BASE      0xe000e000 /* -0xe000efff: Nested Vectored Interrupt Controller */
                                         /* -0xe003ffff: Reserved */
#  define TIVA_TPIU_BASE      0xe0040000 /* -0xe0040fff: Trace Port Interface Unit */
#  define TIVA_ETM_BASE       0xe0041000 /* -0xe0041fff: Embedded Trace Macrocell */
                                         /* -0xffffffff: Reserved */
#else
#  error "Memory map not specified for this TM4C chip"
#endif

/* Peripheral base addresses ********************************************************/

#if defined(CONFIG_ARCH_CHIP_TM4C123GH6ZRB)

#  define TIVA_WDOG0_BASE     (TIVA_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer 0 */
#  define TIVA_WDOG1_BASE     (TIVA_PERIPH_BASE + 0x01000) /* -0x00fff: Watchdog Timer 1 */
                                                           /* -0x03fff: Reserved */
#  define TIVA_GPIOA_BASE     (TIVA_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define TIVA_GPIOB_BASE     (TIVA_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define TIVA_GPIOC_BASE     (TIVA_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define TIVA_GPIOD_BASE     (TIVA_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define TIVA_SSI0_BASE      (TIVA_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
#  define TIVA_SSI1_BASE      (TIVA_PERIPH_BASE + 0x09000) /* -0x09fff: SSI1 */
#  define TIVA_SSI2_BASE      (TIVA_PERIPH_BASE + 0x0a000) /* -0x0afff: SSI2 */
#  define TIVA_SSI3_BASE      (TIVA_PERIPH_BASE + 0x0b000) /* -0x0bfff: SSI3 */
#  define TIVA_UART0_BASE     (TIVA_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define TIVA_UART1_BASE     (TIVA_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
#  define TIVA_UART2_BASE     (TIVA_PERIPH_BASE + 0x0e000) /* -0x0efff: UART2 */
#  define TIVA_UART3_BASE     (TIVA_PERIPH_BASE + 0x0f000) /* -0x0ffff: UART3 */
#  define TIVA_UART4_BASE     (TIVA_PERIPH_BASE + 0x10000) /* -0x10fff: UART4 */
#  define TIVA_UART5_BASE     (TIVA_PERIPH_BASE + 0x11000) /* -0x11fff: UART5 */
#  define TIVA_UART6_BASE     (TIVA_PERIPH_BASE + 0x12000) /* -0x12fff: UART6 */
#  define TIVA_UART7_BASE     (TIVA_PERIPH_BASE + 0x13000) /* -0x13fff: UART7 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define TIVA_I2CM0_BASE     (TIVA_PERIPH_BASE + 0x20000)  /* -0x207ff: I2C Master 0 */
#  define TIVA_I2CS0_BASE     (TIVA_PERIPH_BASE + 0x20800)  /* -0x20fbf: I2C Slave 0 */
#  define TIVA_I2CSC0_BASE    (TIVA_PERIPH_BASE + 0x20fc0)  /* -0x20fff: I2C Status and Control 0 */
#  define TIVA_I2CM1_BASE     (TIVA_PERIPH_BASE + 0x21000)  /* -0x217ff: I2C Master 1 */
#  define TIVA_I2CS1_BASE     (TIVA_PERIPH_BASE + 0x21800)  /* -0x21fbf: I2C Slave 1 */
#  define TIVA_I2CSC1_BASE    (TIVA_PERIPH_BASE + 0x21fc0)  /* -0x21fff: I2C Status and Control 1 */
#  define TIVA_I2CM2_BASE     (TIVA_PERIPH_BASE + 0x22000)  /* -0x227ff: I2C Master 2 */
#  define TIVA_I2CS2_BASE     (TIVA_PERIPH_BASE + 0x22800)  /* -0x22fbf: I2C Slave 2 */
#  define TIVA_I2CSC2_BASE    (TIVA_PERIPH_BASE + 0x22fc0)  /* -0x22fff: I2C Status and Control 2 */
#  define TIVA_I2CM3_BASE     (TIVA_PERIPH_BASE + 0x23000)  /* -0x237ff: I2C Master 3 */
#  define TIVA_I2CS3_BASE     (TIVA_PERIPH_BASE + 0x23800)  /* -0x23fbf: I2C Slave 3 */
#  define TIVA_I2CSC3_BASE    (TIVA_PERIPH_BASE + 0x23fc0)  /* -0x23fff: I2C Status and Control 3 */
#  define TIVA_GPIOE_BASE     (TIVA_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define TIVA_GPIOF_BASE     (TIVA_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define TIVA_GPIOG_BASE     (TIVA_PERIPH_BASE + 0x26000)  /* -0x26fff: GPIO Port G */
#  define TIVA_GPIOH_BASE     (TIVA_PERIPH_BASE + 0x27000)  /* -0x27fff: GPIO Port H */
#  define TIVA_PWM0_BASE      (TIVA_PERIPH_BASE + 0x28000)  /* -0x28fff: PWM 0 */
#  define TIVA_PWM1_BASE      (TIVA_PERIPH_BASE + 0x29000)  /* -0x29fff: PWM 1 */
                                                            /* -0x2ffff: Reserved */
#  define TIVA_TIMER0_BASE    (TIVA_PERIPH_BASE + 0x30000)  /* -0x30fff: 16/32 Timer 0 */
#  define TIVA_TIMER1_BASE    (TIVA_PERIPH_BASE + 0x31000)  /* -0x31fff: 16/32 Timer 1 */
#  define TIVA_TIMER2_BASE    (TIVA_PERIPH_BASE + 0x32000)  /* -0x32fff: 16/32 Timer 2 */
#  define TIVA_TIMER3_BASE    (TIVA_PERIPH_BASE + 0x33000)  /* -0x33fff: 16/32 Timer 3 */
#  define TIVA_TIMER4_BASE    (TIVA_PERIPH_BASE + 0x34000)  /* -0x34fff: 16/32 Timer 4 */
#  define TIVA_TIMER5_BASE    (TIVA_PERIPH_BASE + 0x35000)  /* -0x35fff: 16/32 Timer 5 */
#  define TIVA_WTIMER0_BASE   (TIVA_PERIPH_BASE + 0x36000)  /* -0x36fff: 32/64 Wide Timer 0 */
#  define TIVA_WTIMER1_BASE   (TIVA_PERIPH_BASE + 0x37000)  /* -0x37fff: 32/64 Wide Timer 1 */
#  define TIVA_ADC0_BASE      (TIVA_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC 0 */
#  define TIVA_ADC1_BASE      (TIVA_PERIPH_BASE + 0x39000)  /* -0x39fff: ADC 1 */
                                                            /* -0x3bfff: Reserved */
#  define TIVA_CMP_BASE       (TIVA_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
#  define TIVA_GPIOJ_BASE     (TIVA_PERIPH_BASE + 0x3d000)  /* -0x3dfff: GPIO Port J */
                                                            /* -0x3ffff: Reserved */
#  define TIVA_CAN0_BASE      (TIVA_PERIPH_BASE + 0x40000)  /* -0x40fff: CAN Controller 0 */
#  define TIVA_CAN1_BASE      (TIVA_PERIPH_BASE + 0x41000)  /* -0x41fff: CAN Controller 1 */
                                                            /* -0x4bfff: Reserved */
#  define TIVA_WTIMER2_BASE   (TIVA_PERIPH_BASE + 0x4c000)  /* -0x4cfff: 32/64 Wide Timer 2 */
#  define TIVA_WTIMER3_BASE   (TIVA_PERIPH_BASE + 0x4d000)  /* -0x4dfff: 32/64 Wide Timer 3 */
#  define TIVA_WTIMER4_BASE   (TIVA_PERIPH_BASE + 0x4e000)  /* -0x4efff: 32/64 Wide Timer 4 */
#  define TIVA_WTIMER5_BASE   (TIVA_PERIPH_BASE + 0x4f000)  /* -0x4ffff: 32/64 Wide Timer 5 */
#  define TIVA_USB_BASE       (TIVA_PERIPH_BASE + 0x50000)  /* -0x50fff: USB */
                                                            /* -0x57fff: Reserved */
#  define TIVA_GPIOAAHB_BASE  (TIVA_PERIPH_BASE + 0x58000)  /* -0x58fff: GPIO Port A (AHB aperture) */
#  define TIVA_GPIOBAHB_BASE  (TIVA_PERIPH_BASE + 0x59000)  /* -0x59fff: GPIO Port B (AHB aperture) */
#  define TIVA_GPIOCAHB_BASE  (TIVA_PERIPH_BASE + 0x5a000)  /* -0x5afff: GPIO Port C (AHB aperture) */
#  define TIVA_GPIODAHB_BASE  (TIVA_PERIPH_BASE + 0x5b000)  /* -0x5bfff: GPIO Port D (AHB aperture) */
#  define TIVA_GPIOEAHB_BASE  (TIVA_PERIPH_BASE + 0x5c000)  /* -0x5cfff: GPIO Port E (AHB aperture) */
#  define TIVA_GPIOFAHB_BASE  (TIVA_PERIPH_BASE + 0x5d000)  /* -0x5dfff: GPIO Port F (AHB aperture) */
#  define TIVA_GPIOGAHB_BASE  (TIVA_PERIPH_BASE + 0x5e000)  /* -0x5efff: GPIO Port G (AHB aperture) */
#  define TIVA_GPIOHAHB_BASE  (TIVA_PERIPH_BASE + 0x5f000)  /* -0x5ffff: GPIO Port H (AHB aperture) */
#  define TIVA_GPIOJAHB_BASE  (TIVA_PERIPH_BASE + 0x60000)  /* -0x60fff: GPIO Port J (AHB aperture) */
#  define TIVA_GPIOKAHB_BASE  (TIVA_PERIPH_BASE + 0x61000)  /* -0x61fff: GPIO Port K (AHB aperture) */
#  define TIVA_GPIOLAHB_BASE  (TIVA_PERIPH_BASE + 0x62000)  /* -0x62fff: GPIO Port L (AHB aperture) */
#  define TIVA_GPIOMAHB_BASE  (TIVA_PERIPH_BASE + 0x63000)  /* -0x63fff: GPIO Port M (AHB aperture) */
#  define TIVA_GPIONAHB_BASE  (TIVA_PERIPH_BASE + 0x64000)  /* -0x64fff: GPIO Port N (AHB aperture) */
#  define TIVA_GPIOPAHB_BASE  (TIVA_PERIPH_BASE + 0x65000)  /* -0x65fff: GPIO Port P (AHB aperture) */
#  define TIVA_GPIOQAHB_BASE  (TIVA_PERIPH_BASE + 0x66000)  /* -0x66fff: GPIO Port Q (AHB aperture) */
                                                            /* -0xaefff: Reserved */
#  define TIVA_EEPROM_BASE    (TIVA_PERIPH_BASE + 0xaf000)  /* -0xaffff: EEPROM and Key Locker */
                                                            /* -0xbffff: Reserved */
#  define TIVA_I2CM4_BASE     (TIVA_PERIPH_BASE + 0xc0000)  /* -0x207ff: I2C Master 4 */
#  define TIVA_I2CS4_BASE     (TIVA_PERIPH_BASE + 0xc0800)  /* -0x20fbf: I2C Slave 4 */
#  define TIVA_I2CSC4_BASE    (TIVA_PERIPH_BASE + 0xc0fc0)  /* -0x20fff: I2C Status and Control 4 */
#  define TIVA_I2CM5_BASE     (TIVA_PERIPH_BASE + 0xc1000)  /* -0x207ff: I2C Master 5 */
#  define TIVA_I2CS5_BASE     (TIVA_PERIPH_BASE + 0xc1800)  /* -0x20fbf: I2C Slave 5 */
#  define TIVA_I2CSC5_BASE    (TIVA_PERIPH_BASE + 0xc1fc0)  /* -0x20fff: I2C Status and Control 5 */
                                                            /* -0xf8fff: Reserved */
#  define TIVA_SYSEXC_BASE    (TIVA_PERIPH_BASE + 0xf9000)  /* -0xf9fff: System Exception Control */
                                                            /* -0xfbfff: Reserved */
#  define TIVA_HIBERNATE_BASE (TIVA_PERIPH_BASE + 0xfc000)  /* -0xfcfff: Hibernation Controller */
#  define TIVA_FLASHCON_BASE  (TIVA_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define TIVA_SYSCON_BASE    (TIVA_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
#  define TIVA_UDMA_BASE      (TIVA_PERIPH_BASE + 0xff000)  /* -0xfffff: Micro Direct Memory Access */

#elif defined(CONFIG_ARCH_CHIP_TM4C123GH6PMI)

#  define TIVA_WDOG0_BASE     (TIVA_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer 0 */
#  define TIVA_WDOG1_BASE     (TIVA_PERIPH_BASE + 0x01000) /* -0x00fff: Watchdog Timer 1 */
                                                           /* -0x03fff: Reserved */
#  define TIVA_GPIOA_BASE     (TIVA_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define TIVA_GPIOB_BASE     (TIVA_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define TIVA_GPIOC_BASE     (TIVA_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define TIVA_GPIOD_BASE     (TIVA_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define TIVA_SSI0_BASE      (TIVA_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
#  define TIVA_SSI1_BASE      (TIVA_PERIPH_BASE + 0x09000) /* -0x09fff: SSI1 */
#  define TIVA_SSI2_BASE      (TIVA_PERIPH_BASE + 0x0a000) /* -0x0afff: SSI2 */
#  define TIVA_SSI3_BASE      (TIVA_PERIPH_BASE + 0x0b000) /* -0x0bfff: SSI3 */
#  define TIVA_UART0_BASE     (TIVA_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define TIVA_UART1_BASE     (TIVA_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
#  define TIVA_UART2_BASE     (TIVA_PERIPH_BASE + 0x0e000) /* -0x0efff: UART2 */
#  define TIVA_UART3_BASE     (TIVA_PERIPH_BASE + 0x0f000) /* -0x0ffff: UART3 */
#  define TIVA_UART4_BASE     (TIVA_PERIPH_BASE + 0x10000) /* -0x10fff: UART4 */
#  define TIVA_UART5_BASE     (TIVA_PERIPH_BASE + 0x11000) /* -0x11fff: UART5 */
#  define TIVA_UART6_BASE     (TIVA_PERIPH_BASE + 0x12000) /* -0x12fff: UART6 */
#  define TIVA_UART7_BASE     (TIVA_PERIPH_BASE + 0x13000) /* -0x13fff: UART7 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define TIVA_I2CM0_BASE     (TIVA_PERIPH_BASE + 0x20000)  /* -0x207ff: I2C Master 0 */
#  define TIVA_I2CS0_BASE     (TIVA_PERIPH_BASE + 0x20800)  /* -0x20fbf: I2C Slave 0 */
#  define TIVA_I2CSC0_BASE    (TIVA_PERIPH_BASE + 0x20fc0)  /* -0x20fff: I2C Status and Control 0 */
#  define TIVA_I2CM1_BASE     (TIVA_PERIPH_BASE + 0x21000)  /* -0x217ff: I2C Master 1 */
#  define TIVA_I2CS1_BASE     (TIVA_PERIPH_BASE + 0x21800)  /* -0x21fbf: I2C Slave 1 */
#  define TIVA_I2CSC1_BASE    (TIVA_PERIPH_BASE + 0x21fc0)  /* -0x21fff: I2C Status and Control 1 */
#  define TIVA_I2CM2_BASE     (TIVA_PERIPH_BASE + 0x22000)  /* -0x227ff: I2C Master 2 */
#  define TIVA_I2CS2_BASE     (TIVA_PERIPH_BASE + 0x22800)  /* -0x22fbf: I2C Slave 2 */
#  define TIVA_I2CSC2_BASE    (TIVA_PERIPH_BASE + 0x22fc0)  /* -0x22fff: I2C Status and Control 2 */
#  define TIVA_I2CM3_BASE     (TIVA_PERIPH_BASE + 0x23000)  /* -0x237ff: I2C Master 3 */
#  define TIVA_I2CS3_BASE     (TIVA_PERIPH_BASE + 0x23800)  /* -0x23fbf: I2C Slave 3 */
#  define TIVA_I2CSC3_BASE    (TIVA_PERIPH_BASE + 0x23fc0)  /* -0x23fff: I2C Status and Control 3 */
#  define TIVA_GPIOE_BASE     (TIVA_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define TIVA_GPIOF_BASE     (TIVA_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define TIVA_PWM0_BASE      (TIVA_PERIPH_BASE + 0x28000)  /* -0x28fff: PWM 0 */
#  define TIVA_PWM1_BASE      (TIVA_PERIPH_BASE + 0x29000)  /* -0x29fff: PWM 1 */
                                                            /* -0x2ffff: Reserved */
#  define TIVA_TIMER0_BASE    (TIVA_PERIPH_BASE + 0x30000)  /* -0x30fff: 16/32 Timer 0 */
#  define TIVA_TIMER1_BASE    (TIVA_PERIPH_BASE + 0x31000)  /* -0x31fff: 16/32 Timer 1 */
#  define TIVA_TIMER2_BASE    (TIVA_PERIPH_BASE + 0x32000)  /* -0x32fff: 16/32 Timer 2 */
#  define TIVA_TIMER3_BASE    (TIVA_PERIPH_BASE + 0x33000)  /* -0x33fff: 16/32 Timer 3 */
#  define TIVA_TIMER4_BASE    (TIVA_PERIPH_BASE + 0x34000)  /* -0x34fff: 16/32 Timer 4 */
#  define TIVA_TIMER5_BASE    (TIVA_PERIPH_BASE + 0x35000)  /* -0x35fff: 16/32 Timer 5 */
#  define TIVA_WTIMER0_BASE   (TIVA_PERIPH_BASE + 0x36000)  /* -0x36fff: 32/64 Wide Timer 0 */
#  define TIVA_WTIMER1_BASE   (TIVA_PERIPH_BASE + 0x37000)  /* -0x37fff: 32/64 Wide Timer 1 */
#  define TIVA_ADC0_BASE      (TIVA_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC 0 */
#  define TIVA_ADC1_BASE      (TIVA_PERIPH_BASE + 0x39000)  /* -0x39fff: ADC 1 */
                                                            /* -0x3bfff: Reserved */
#  define TIVA_CMP_BASE       (TIVA_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
                                                            /* -0x3ffff: Reserved */
#  define TIVA_CAN0_BASE      (TIVA_PERIPH_BASE + 0x40000)  /* -0x40fff: CAN Controller 0 */
#  define TIVA_CAN1_BASE      (TIVA_PERIPH_BASE + 0x41000)  /* -0x41fff: CAN Controller 1 */
                                                            /* -0x4bfff: Reserved */
#  define TIVA_WTIMER2_BASE   (TIVA_PERIPH_BASE + 0x4c000)  /* -0x4cfff: 32/64 Wide Timer 2 */
#  define TIVA_WTIMER3_BASE   (TIVA_PERIPH_BASE + 0x4d000)  /* -0x4dfff: 32/64 Wide Timer 3 */
#  define TIVA_WTIMER4_BASE   (TIVA_PERIPH_BASE + 0x4e000)  /* -0x4efff: 32/64 Wide Timer 4 */
#  define TIVA_WTIMER5_BASE   (TIVA_PERIPH_BASE + 0x4f000)  /* -0x4ffff: 32/64 Wide Timer 5 */
#  define TIVA_USB_BASE       (TIVA_PERIPH_BASE + 0x50000)  /* -0x50fff: USB */
                                                            /* -0x57fff: Reserved */
#  define TIVA_GPIOAAHB_BASE  (TIVA_PERIPH_BASE + 0x58000)  /* -0x58fff: GPIO Port A (AHB aperture) */
#  define TIVA_GPIOBAHB_BASE  (TIVA_PERIPH_BASE + 0x59000)  /* -0x59fff: GPIO Port B (AHB aperture) */
#  define TIVA_GPIOCAHB_BASE  (TIVA_PERIPH_BASE + 0x5a000)  /* -0x5afff: GPIO Port C (AHB aperture) */
#  define TIVA_GPIODAHB_BASE  (TIVA_PERIPH_BASE + 0x5b000)  /* -0x5bfff: GPIO Port D (AHB aperture) */
#  define TIVA_GPIOEAHB_BASE  (TIVA_PERIPH_BASE + 0x5c000)  /* -0x5cfff: GPIO Port E (AHB aperture) */
#  define TIVA_GPIOFAHB_BASE  (TIVA_PERIPH_BASE + 0x5d000)  /* -0x5dfff: GPIO Port F (AHB aperture) */
                                                            /* -0xaefff: Reserved */
#  define TIVA_EEPROM_BASE    (TIVA_PERIPH_BASE + 0xaf000)  /* -0xaffff: EEPROM and Key Locker */
                                                            /* -0xf8fff: Reserved */
#  define TIVA_SYSEXC_BASE    (TIVA_PERIPH_BASE + 0xf9000)  /* -0xf9fff: System Exception Control */
                                                            /* -0xfbfff: Reserved */
#  define TIVA_HIBERNATE_BASE (TIVA_PERIPH_BASE + 0xfc000)  /* -0xfcfff: Hibernation Controller */
#  define TIVA_FLASHCON_BASE  (TIVA_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define TIVA_SYSCON_BASE    (TIVA_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
#  define TIVA_UDMA_BASE      (TIVA_PERIPH_BASE + 0xff000)  /* -0xfffff: Micro Direct Memory Access */
#elif defined(CONFIG_ARCH_CHIP_TM4C123GH6PM)

#  define TIVA_WDOG0_BASE     (TIVA_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer 0 */
#  define TIVA_WDOG1_BASE     (TIVA_PERIPH_BASE + 0x01000) /* -0x00fff: Watchdog Timer 1 */
                                                           /* -0x03fff: Reserved */
#  define TIVA_GPIOA_BASE     (TIVA_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define TIVA_GPIOB_BASE     (TIVA_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define TIVA_GPIOC_BASE     (TIVA_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define TIVA_GPIOD_BASE     (TIVA_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define TIVA_SSI0_BASE      (TIVA_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
#  define TIVA_SSI1_BASE      (TIVA_PERIPH_BASE + 0x09000) /* -0x09fff: SSI1 */
#  define TIVA_SSI2_BASE      (TIVA_PERIPH_BASE + 0x0a000) /* -0x0afff: SSI2 */
#  define TIVA_SSI3_BASE      (TIVA_PERIPH_BASE + 0x0b000) /* -0x0bfff: SSI3 */
#  define TIVA_UART0_BASE     (TIVA_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define TIVA_UART1_BASE     (TIVA_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
#  define TIVA_UART2_BASE     (TIVA_PERIPH_BASE + 0x0e000) /* -0x0efff: UART2 */
#  define TIVA_UART3_BASE     (TIVA_PERIPH_BASE + 0x0f000) /* -0x0ffff: UART3 */
#  define TIVA_UART4_BASE     (TIVA_PERIPH_BASE + 0x10000) /* -0x10fff: UART4 */
#  define TIVA_UART5_BASE     (TIVA_PERIPH_BASE + 0x11000) /* -0x11fff: UART5 */
#  define TIVA_UART6_BASE     (TIVA_PERIPH_BASE + 0x12000) /* -0x12fff: UART6 */
#  define TIVA_UART7_BASE     (TIVA_PERIPH_BASE + 0x13000) /* -0x13fff: UART7 */
                                                           /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define TIVA_I2CM0_BASE     (TIVA_PERIPH_BASE + 0x20000)  /* -0x207ff: I2C Master 0 */
#  define TIVA_I2CS0_BASE     (TIVA_PERIPH_BASE + 0x20800)  /* -0x20fbf: I2C Slave 0 */
#  define TIVA_I2CSC0_BASE    (TIVA_PERIPH_BASE + 0x20fc0)  /* -0x20fff: I2C Status and Control 0 */
#  define TIVA_I2CM1_BASE     (TIVA_PERIPH_BASE + 0x21000)  /* -0x217ff: I2C Master 1 */
#  define TIVA_I2CS1_BASE     (TIVA_PERIPH_BASE + 0x21800)  /* -0x21fbf: I2C Slave 1 */
#  define TIVA_I2CSC1_BASE    (TIVA_PERIPH_BASE + 0x21fc0)  /* -0x21fff: I2C Status and Control 1 */
#  define TIVA_I2CM2_BASE     (TIVA_PERIPH_BASE + 0x22000)  /* -0x227ff: I2C Master 2 */
#  define TIVA_I2CS2_BASE     (TIVA_PERIPH_BASE + 0x22800)  /* -0x22fbf: I2C Slave 2 */
#  define TIVA_I2CSC2_BASE    (TIVA_PERIPH_BASE + 0x22fc0)  /* -0x22fff: I2C Status and Control 2 */
#  define TIVA_I2CM3_BASE     (TIVA_PERIPH_BASE + 0x23000)  /* -0x237ff: I2C Master 3 */
#  define TIVA_I2CS3_BASE     (TIVA_PERIPH_BASE + 0x23800)  /* -0x23fbf: I2C Slave 3 */
#  define TIVA_I2CSC3_BASE    (TIVA_PERIPH_BASE + 0x23fc0)  /* -0x23fff: I2C Status and Control 3 */
#  define TIVA_GPIOE_BASE     (TIVA_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define TIVA_GPIOF_BASE     (TIVA_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
#  define TIVA_PWM0_BASE      (TIVA_PERIPH_BASE + 0x28000)  /* -0x28fff: PWM 0 */
#  define TIVA_PWM1_BASE      (TIVA_PERIPH_BASE + 0x29000)  /* -0x29fff: PWM 1 */
                                                            /* -0x2ffff: Reserved */
#  define TIVA_TIMER0_BASE    (TIVA_PERIPH_BASE + 0x30000)  /* -0x30fff: 16/32 Timer 0 */
#  define TIVA_TIMER1_BASE    (TIVA_PERIPH_BASE + 0x31000)  /* -0x31fff: 16/32 Timer 1 */
#  define TIVA_TIMER2_BASE    (TIVA_PERIPH_BASE + 0x32000)  /* -0x32fff: 16/32 Timer 2 */
#  define TIVA_TIMER3_BASE    (TIVA_PERIPH_BASE + 0x33000)  /* -0x33fff: 16/32 Timer 3 */
#  define TIVA_TIMER4_BASE    (TIVA_PERIPH_BASE + 0x34000)  /* -0x34fff: 16/32 Timer 4 */
#  define TIVA_TIMER5_BASE    (TIVA_PERIPH_BASE + 0x35000)  /* -0x35fff: 16/32 Timer 5 */
#  define TIVA_WTIMER0_BASE   (TIVA_PERIPH_BASE + 0x36000)  /* -0x36fff: 32/64 Wide Timer 0 */
#  define TIVA_WTIMER1_BASE   (TIVA_PERIPH_BASE + 0x37000)  /* -0x37fff: 32/64 Wide Timer 1 */
#  define TIVA_ADC0_BASE      (TIVA_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC 0 */
#  define TIVA_ADC1_BASE      (TIVA_PERIPH_BASE + 0x39000)  /* -0x39fff: ADC 1 */
                                                            /* -0x3bfff: Reserved */
#  define TIVA_CMP_BASE       (TIVA_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
                                                            /* -0x3ffff: Reserved */
#  define TIVA_CAN0_BASE      (TIVA_PERIPH_BASE + 0x40000)  /* -0x40fff: CAN Controller 0 */
#  define TIVA_CAN1_BASE      (TIVA_PERIPH_BASE + 0x41000)  /* -0x41fff: CAN Controller 1 */
                                                            /* -0x4bfff: Reserved */
#  define TIVA_WTIMER2_BASE   (TIVA_PERIPH_BASE + 0x4c000)  /* -0x4cfff: 32/64 Wide Timer 2 */
#  define TIVA_WTIMER3_BASE   (TIVA_PERIPH_BASE + 0x4d000)  /* -0x4dfff: 32/64 Wide Timer 3 */
#  define TIVA_WTIMER4_BASE   (TIVA_PERIPH_BASE + 0x4e000)  /* -0x4efff: 32/64 Wide Timer 4 */
#  define TIVA_WTIMER5_BASE   (TIVA_PERIPH_BASE + 0x4f000)  /* -0x4ffff: 32/64 Wide Timer 5 */
#  define TIVA_USB_BASE       (TIVA_PERIPH_BASE + 0x50000)  /* -0x50fff: USB */
                                                            /* -0x57fff: Reserved */
#  define TIVA_GPIOAAHB_BASE  (TIVA_PERIPH_BASE + 0x58000)  /* -0x58fff: GPIO Port A (AHB aperture) */
#  define TIVA_GPIOBAHB_BASE  (TIVA_PERIPH_BASE + 0x59000)  /* -0x59fff: GPIO Port B (AHB aperture) */
#  define TIVA_GPIOCAHB_BASE  (TIVA_PERIPH_BASE + 0x5a000)  /* -0x5afff: GPIO Port C (AHB aperture) */
#  define TIVA_GPIODAHB_BASE  (TIVA_PERIPH_BASE + 0x5b000)  /* -0x5bfff: GPIO Port D (AHB aperture) */
#  define TIVA_GPIOEAHB_BASE  (TIVA_PERIPH_BASE + 0x5c000)  /* -0x5cfff: GPIO Port E (AHB aperture) */
#  define TIVA_GPIOFAHB_BASE  (TIVA_PERIPH_BASE + 0x5d000)  /* -0x5dfff: GPIO Port F (AHB aperture) */
                                                            /* -0xaefff: Reserved */
#  define TIVA_EEPROM_BASE    (TIVA_PERIPH_BASE + 0xaf000)  /* -0xaffff: EEPROM and Key Locker */
                                                            /* -0xf8fff: Reserved */
#  define TIVA_SYSEXC_BASE    (TIVA_PERIPH_BASE + 0xf9000)  /* -0xf9fff: System Exception Control */
                                                            /* -0xfbfff: Reserved */
#  define TIVA_HIBERNATE_BASE (TIVA_PERIPH_BASE + 0xfc000)  /* -0xfcfff: Hibernation Controller */
#  define TIVA_FLASHCON_BASE  (TIVA_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define TIVA_SYSCON_BASE    (TIVA_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
#  define TIVA_UDMA_BASE      (TIVA_PERIPH_BASE + 0xff000)  /* -0xfffff: Micro Direct Memory Access */
#else
#  error "Peripheral base addresses not specified for this Stellaris chip"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_CHIP_TM4C_MEMORYMAP_H */
