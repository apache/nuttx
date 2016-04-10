/************************************************************************************
 * arch/arm/src/tiva/chip/cc3200_memorymap.h
 *
 *   Copyright (C) 2014 Droidifi LLC. All rights reserved.
 *            Jim Ewing <jim@droidifi.com>
 *
 *   Adapted for the cc3200 from code:
 *
 *   Copyright (C) Gregory Nutt.
 *   Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name Droidifi nor the names of its contributors may be
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

#ifndef __ARCH_ARM_SRC_TIVA_CHIP_CC3200_MEMORYMAP_H
#define __ARCH_ARM_SRC_TIVA_CHIP_CC3200_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Memory map ***********************************************************************/

#if defined(CONFIG_ARCH_CHIP_CC3200)
#  define TIVA_FLASH_BASE     0x00000000 /* -0x0007ffff: On-chip FLASH */
                                         /* -0x00ffffff: Reserved */
#  define TIVA_ROM_BASE       0x01000000 /* -0x1fffffff: Reserved for ROM */
#  define TIVA_SRAM_BASE      0x20000000 /* -0x2003ffff: Bit-banded on-chip SRAM */
                                         /* -0x21ffffff: Reserved */
#  define TIVA_ASRAM_BASE     0x22000000 /* -0x23ffffff: Bit-band alias of 20000000- */
                                         /* -0x3fffffff: Reserved */
#  define TIVA_PERIPH_BASE    0x40000000 /* -0x4001ffff: FiRM Peripherals */
                                         /* -0x41ffffff: Peripherals */
#  define TIVA_APERIPH_BASE   0x42000000 /* -0x43ffffff: Bit-band alias of 40000000- */

#  define TIVA_CRYPTO_BASE    0x44030000 /* -0x44039fff: Crypto HW base 44030000- */
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

#if defined(CONFIG_ARCH_CHIP_CC3200)

#  define TIVA_WDOG0_BASE     (TIVA_PERIPH_BASE + 0x00000)  /* -0x00fff: Watchdog Timer 0 */

#  define TIVA_GPIOA_BASE     (TIVA_PERIPH_BASE + 0x04000)  /* -0x04fff: GPIO Port A */
#  define TIVA_GPIOB_BASE     (TIVA_PERIPH_BASE + 0x05000)  /* -0x05fff: GPIO Port B */
#  define TIVA_GPIOC_BASE     (TIVA_PERIPH_BASE + 0x06000)  /* -0x06fff: GPIO Port C */
#  define TIVA_GPIOD_BASE     (TIVA_PERIPH_BASE + 0x07000)  /* -0x07fff: GPIO Port D */

#  define TIVA_UART0_BASE     (TIVA_PERIPH_BASE + 0x0c000)  /* -0x0cfff: UART0 */
#  define TIVA_UART1_BASE     (TIVA_PERIPH_BASE + 0x0d000)  /* -0x0dfff: UART1 */

#  define TIVA_I2C0_BASE      (TIVA_PERIPH_BASE + 0x20000)  /* -0x207ff: I2C0 */

#  define TIVA_TIMER0_BASE    (TIVA_PERIPH_BASE + 0x30000)  /* -0x30fff: 16/32 Timer 0 */
#  define TIVA_TIMER1_BASE    (TIVA_PERIPH_BASE + 0x31000)  /* -0x31fff: 16/32 Timer 1 */
#  define TIVA_TIMER2_BASE    (TIVA_PERIPH_BASE + 0x32000)  /* -0x32fff: 16/32 Timer 2 */
#  define TIVA_TIMER3_BASE    (TIVA_PERIPH_BASE + 0x33000)  /* -0x33fff: 16/32 Timer 3 */

// NOTE: ADC memory location not listed in CC3200 DS
#  define TIVA_ADC0_BASE      (TIVA_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC 0 */

#  define TIVA_CONF_REG       (TIVA_PERIPH_BASE + 0xf7000)  /* -0xf7fff: Configuration registers */
#  define TIVA_SYSCON_BASE    (TIVA_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */

#  define TIVA_UDMA_BASE      (TIVA_PERIPH_BASE + 0xff000)  /* -0xfffff: Micro Direct Memory Access */
#  define TIVA_ASP_A0         (TIVA_PERIPH_BASE + 0x401c000)/* -0x401efff: McASP A0 */
#  define TIVA_SPI_A0         (TIVA_PERIPH_BASE + 0x4020000)/* -0x4020fff: McSPI A0 */
#  define TIVA_SPI_A1         (TIVA_PERIPH_BASE + 0x4021000)/* -0x4022fff: McSPI A1 */

#  define TIVA_APP_CLK_BASE   (TIVA_PERIPH_BASE + 0x4025000)/* -0x4025fff: App Clk */
#  define TIVA_APP_CFG_BASE   (TIVA_PERIPH_BASE + 0x4026000)/* -0x4026fff: App config */
#  define TIVA_OCP_GPRCM_BASE (TIVA_PERIPH_BASE + 0x402d000)/* -0x402dfff: global reset, pwr, clk */
#  define TIVA_OCP_SHR_BASE   (TIVA_PERIPH_BASE + 0x402e000)/* -0x402efff: OCP shared config */
#  define TIVA_HIBERNATE_BASE (TIVA_PERIPH_BASE + 0x402f000)/* -0x402ffff: Hibernation Controller */

/* Crypto Base Addresses */

#  define TIVA_TCP_DTHE_BASE  (TIVA_CRYPTO_BASE + 0x0000)  /* -0x0fff: TCP Checksum & DTHE regs */
#  define TIVA_CCM_BASE       TIVA_TCP_DTHE_BASE
#  define TIVA_SHA_BASE       (TIVA_CRYPTO_BASE + 0x5000)  /* -0x5fff: MD5/SHA */
#  define TIVA_AES_BASE       (TIVA_CRYPTO_BASE + 0x7000)  /* -0x7fff: AES */
#  define TIVA_DES_BASE       (TIVA_CRYPTO_BASE + 0x9000)  /* -0x9fff: DES */

// NOTE: the following locations are not listed in CC3200 DS
#  define TIVA_GPIOAAHB_BASE  (TIVA_PERIPH_BASE + 0x58000)  /* -0x58fff: GPIO Port A (AHB aperture) */
#  define TIVA_GPIOBAHB_BASE  (TIVA_PERIPH_BASE + 0x59000)  /* -0x59fff: GPIO Port B (AHB aperture) */
#  define TIVA_GPIOCAHB_BASE  (TIVA_PERIPH_BASE + 0x5a000)  /* -0x5afff: GPIO Port C (AHB aperture) */
#  define TIVA_GPIODAHB_BASE  (TIVA_PERIPH_BASE + 0x5b000)  /* -0x5bfff: GPIO Port D (AHB aperture) */

#  define TIVA_EPI0_BASE      (TIVA_PERIPH_BASE + 0xd0000)  /* -0xd0fff: EPI 0 */
#  define TIVA_EEPROM_BASE    (TIVA_PERIPH_BASE + 0xaf000)  /* -0xaffff: EEPROM and Key Locker */
#  define TIVA_SYSEXC_BASE    (TIVA_PERIPH_BASE + 0xf9000)  /* -0xf9fff: System Exception Control */
#  define TIVA_FLASHCON_BASE  (TIVA_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */

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

#endif /* __ARCH_ARM_SRC_TIVA_CHIP_CC3200_MEMORYMAP_H */
