/************************************************************************************
 * arch/arm/src/kl/hardware/kl_memorymap.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_KL_HARDWARE_KL_MEMORYMAP_H
#define __ARCH_ARM_SRC_KL_HARDWARE_KL_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Memory Map ***********************************************************************/
/* K40 Family
 *
 * The memory map for the following parts is defined in Freescale document
 * K40P144M100SF2RM
 */

# define KL_FLASH_BASE     0x00000000 /* -0x0fffffff Program flash and read-
                                       *             only data (Includes exception
                                       *             vectors in first 1024 bytes) */
# define KL_SRAML_BASE     0x18000000 /* -0x1fffffff SRAM_L: Lower SRAM
                                       *             (ICODE/DCODE) */
# define KL_SRAMU_BASE     0x20000000 /* -0x200fffff SRAM_U: Upper SRAM bitband
                                       *             region */
                        /* 0x20100000  * -0x3fffffff Reserved */
# define KIP_AIPS_BASE     0x40000000 /* -0x4007ffff AIPS Peripherals */
                        /* 0x40080000  * -0x400fffff Reserved */
# define KL_GPIO_BASE(n)   (0x400ff000 + ((n) << 6))
                        /* 0x40100000  * -0x43ffffff Reserved */
# define KL_BME_BASE       0x44000000 /* -0x5fffffff Bit Manipulation Engine (BME) access
                                       *             to AIPS Peripherals for slots 0-127 */
                        /* 0x60000000  * -0xdfffffff Reserved */
# define KL_PERIPH_BASE    0xe0000000 /* -0xe00fffff Private peripherals */
                        /* 0xe0100000  * -0xefffffff Reserved */
# define KL_MTB_BASE       0xf0000000 /* -0xffffffff Micro Trace Buffer (MTB) registers */

/* AIPS Memory Map ******************************************************************/

# define KL_DMAC_BASE      0x40008000 /* DMA controller */
# define KL_AIPSGPIO_BASE  0x4000f000 /* GPIO controller (aliased to 0x400ff000) */
# define KL_FTFL_BASE      0x40020000 /* Flash memory */
# define KL_DMAMUX0_BASE   0x40021000 /* DMA channel multiplexer 0 */
# define KL_PIT_BASE       0x40037000 /* Periodic interrupt timers (PIT) */
# define KL_TPM0_BASE      0x40038000 /* Timer/PWM (TPM) 0 */
# define KL_TPM1_BASE      0x40039000 /* Timer/PWM (TPM) 1 */
# define KL_TPM2_BASE      0x4003a000 /* Timer/PWM (TPM) 2 */
# define KL_ADC0_BASE      0x4003b000 /* Analog-to-digital converter (ADC) 0 */
# define KL_RTC_BASE       0x4003d000 /* Real time clock */
# define KL_DAC0_BASE      0x4003f000 /* Digital-to-analog convert (DAC) 0 */
# define KL_LPTMR_BASE     0x40040000 /* Low power timer */
# define KL_TSI_BASE       0x40045000 /* Touch sense interface */
# define KL_SIMLP_BASE     0x40047000 /* SIM low-power logic */
# define KL_SIM_BASE       0x40048000 /* System integration module (SIM) */
# define KL_PORT_BASE(n)   (0x40049000 + ((n) << 12))
# define KL_PORTA_BASE     0x40049000 /* Port A multiplexing control */
# define KL_PORTB_BASE     0x4004a000 /* Port B multiplexing control */
# define KL_PORTC_BASE     0x4004b000 /* Port C multiplexing control */
# define KL_PORTD_BASE     0x4004c000 /* Port D multiplexing control */
# define KL_PORTE_BASE     0x4004d000 /* Port E multiplexing control */
# define KL_MCG_BASE       0x40064000 /* Multi-purpose Clock Generator (MCG) */
# define KL_OSC_BASE       0x40065000 /* System oscillator (OSC) */
# define KL_I2C0_BASE      0x40066000 /* I2C 0 */
# define KL_I2C1_BASE      0x40067000 /* I2C 1 */
# define KL_UART0_BASE     0x4006a000 /* UART0 */
# define KL_UART1_BASE     0x4006b000 /* UART1 */
# define KL_UART2_BASE     0x4006c000 /* UART2 */
# define KL_USB0_BASE      0x40072000 /* USB OTG FS/LS */
# define KL_CMP_BASE       0x40073000 /* Analog comparator (CMP) / 6-bit digital-to-analog converter (DAC) */
# define KL_SPI0_BASE      0x40076000 /* SPI 0 */
# define KL_SPI1_BASE      0x40077000 /* SPI 1 */
# define KL_LLWU_BASE      0x4007c000 /* Low-leakage wakeup unit (LLWU) */
# define KL_PMC_BASE       0x4007d000 /* Power management controller (PMC) */
# define KL_SMC_BASE       0x4007e000 /* System Mode controller (SMC) */
# define KL_RCM_BASE       0x4007f000 /* Reset Control Module (RCM) */
                        /* 0x400ff000  * GPIO Controller */
# define KL_GPIOn_BASE(n) (0x400ff000 + ((n) << 6))
# define KL_GPIOA_BASE     0x400ff000 /* GPIO PORTA registers */
# define KL_GPIOB_BASE     0x400ff040 /* GPIO PORTB registers */
# define KL_GPIOC_BASE     0x400ff080 /* GPIO PORTC registers */
# define KL_GPIOD_BASE     0x400ff0c0 /* GPIO PORTD registers */
# define KL_GPIOE_BASE     0x400ff100 /* GPIO PORTE registers */

/* Private Peripheral Bus (PPB) Memory Map ******************************************/

# define KL_SCS_BASE       0xe000e000 /* System Control Space (SCS) (for NVIC) */
# define KL_ROMTAB_BASE    0xe00ff000 /* ROM Table - allows auto-detection of debug components */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KL_HARDWARE_KL_MEMORYMAP_H */
