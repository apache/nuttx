/****************************************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13xx_enableclks.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_CC13XX_CC13XX_ENABLECLKS_H
#define __ARCH_ARM_SRC_TIVA_CC13XX_CC13XX_ENABLECLKS_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "cc13xx/cc13xx_prcm.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

#define CC13XX_RUNMODE_CLOCK             (1 << 0)
#define CC13XX_SLEEPMODE_CLOCK           (1 << 1)
#define CC13XX_DEEPSLEEPMODE_CLOCK       (1 << 2)
#define CC13XX_ALLMODE_CLOCKS            (7)

/* Watchdog Timer Clock Control */

#define tiva_wdt0_enableclk()
#define tiva_wdt0_disableclk()

/* 16/32-Bit Timer Clock Control */

#define tiva_gptm0_runenable()           cc13xx_periph_enableclk(PRCM_PERIPH_TIMER0, CC13XX_RUNMODE_CLOCK)
#define tiva_gptm0_rundisable()          cc13xx_periph_disableclk(PRCM_PERIPH_TIMER0, CC13XX_RUNMODE_CLOCK)
#define tiva_gptm0_sleepenable()         cc13xx_periph_enableclk(PRCM_PERIPH_TIMER0, CC13XX_SLEEPMODE_CLOCK)
#define tiva_gptm0_sleepdisable()        cc13xx_periph_disableclk(PRCM_PERIPH_TIMER0, CC13XX_SLEEPMODE_CLOCK)
#define tiva_gptm0_deepsleepenable()     cc13xx_periph_enableclk(PRCM_PERIPH_TIMER0, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_gptm0_deepsleepdisable()    cc13xx_periph_disableclk(PRCM_PERIPH_TIMER0, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_gptm0_enableclk()           cc13xx_periph_enableclk(PRCM_PERIPH_TIMER0, CC13XX_ALLMODE_CLOCKS)
#define tiva_gptm0_disableclk()          cc13xx_periph_disableclk(PRCM_PERIPH_TIMER0, CC13XX_ALLMODE_CLOCKS)

#define tiva_gptm1_runenable()           cc13xx_periph_enableclk(PRCM_PERIPH_TIMER1, CC13XX_RUNMODE_CLOCK)
#define tiva_gptm1_rundisable()          cc13xx_periph_disableclk(PRCM_PERIPH_TIMER1, CC13XX_RUNMODE_CLOCK)
#define tiva_gptm1_sleepenable()         cc13xx_periph_enableclk(PRCM_PERIPH_TIMER1, CC13XX_SLEEPMODE_CLOCK)
#define tiva_gptm1_sleepdisable()        cc13xx_periph_disableclk(PRCM_PERIPH_TIMER1, CC13XX_SLEEPMODE_CLOCK)
#define tiva_gptm1_deepsleepenable()     cc13xx_periph_enableclk(PRCM_PERIPH_TIMER1, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_gptm1_deepsleepdisable()    cc13xx_periph_disableclk(PRCM_PERIPH_TIMER1, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_gptm1_enableclk()           cc13xx_periph_enableclk(PRCM_PERIPH_TIMER1, CC13XX_ALLMODE_CLOCKS)
#define tiva_gptm1_disableclk()          cc13xx_periph_disableclk(PRCM_PERIPH_TIMER1, CC13XX_ALLMODE_CLOCKS)

#define tiva_gptm2_runenable()           cc13xx_periph_enableclk(PRCM_PERIPH_TIMER2, CC13XX_RUNMODE_CLOCK)
#define tiva_gptm2_rundisable()          cc13xx_periph_disableclk(PRCM_PERIPH_TIMER2, CC13XX_RUNMODE_CLOCK)
#define tiva_gptm2_sleepenable()         cc13xx_periph_enableclk(PRCM_PERIPH_TIMER2, CC13XX_SLEEPMODE_CLOCK)
#define tiva_gptm2_sleepdisable()        cc13xx_periph_disableclk(PRCM_PERIPH_TIMER2, CC13XX_SLEEPMODE_CLOCK)
#define tiva_gptm2_deepsleepenable()     cc13xx_periph_enableclk(PRCM_PERIPH_TIMER2, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_gptm2_deepsleepdisable()    cc13xx_periph_disableclk(PRCM_PERIPH_TIMER2, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_gptm2_enableclk()           cc13xx_periph_enableclk(PRCM_PERIPH_TIMER2, CC13XX_ALLMODE_CLOCKS)
#define tiva_gptm2_disableclk()          cc13xx_periph_disableclk(PRCM_PERIPH_TIMER2, CC13XX_ALLMODE_CLOCKS)

#define tiva_gptm3_runenable()           cc13xx_periph_enableclk(PRCM_PERIPH_TIMER3, CC13XX_RUNMODE_CLOCK)
#define tiva_gptm3_rundisable()          cc13xx_periph_disableclk(PRCM_PERIPH_TIMER3, CC13XX_RUNMODE_CLOCK)
#define tiva_gptm3_sleepenable()         cc13xx_periph_enableclk(PRCM_PERIPH_TIMER3, CC13XX_SLEEPMODE_CLOCK)
#define tiva_gptm3_sleepdisable()        cc13xx_periph_disableclk(PRCM_PERIPH_TIMER3, CC13XX_SLEEPMODE_CLOCK)
#define tiva_gptm3_deepsleepenable()     cc13xx_periph_enableclk(PRCM_PERIPH_TIMER3, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_gptm3_deepsleepdisable()    cc13xx_periph_disableclk(PRCM_PERIPH_TIMER3, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_gptm3_enableclk()           cc13xx_periph_enableclk(PRCM_PERIPH_TIMER3, CC13XX_ALLMODE_CLOCKS)
#define tiva_gptm3_disableclk()          cc13xx_periph_disableclk(PRCM_PERIPH_TIMER3, CC13XX_ALLMODE_CLOCKS)

/* GPIO Clock Control */

#define tiva_gpio_runenable()            cc13xx_periph_enableclk(PRCM_PERIPH_GPIO, CC13XX_RUNMODE_CLOCK)
#define tiva_gpio_rundisable()           cc13xx_periph_disableclk(PRCM_PERIPH_GPIO, CC13XX_RUNMODE_CLOCK)
#define tiva_gpio_sleepenable()          cc13xx_periph_enableclk(PRCM_PERIPH_GPIO, CC13XX_SLEEPMODE_CLOCK)
#define tiva_gpio_sleepdisable()         cc13xx_periph_disableclk(PRCM_PERIPH_GPIO, CC13XX_SLEEPMODE_CLOCK)
#define tiva_gpio_deepsleepenable()      cc13xx_periph_enableclk(PRCM_PERIPH_GPIO, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_gpio_deepsleepdisable()     cc13xx_periph_disableclk(PRCM_PERIPH_GPIO, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_gpio_enableclk()            cc13xx_periph_enableclk(PRCM_PERIPH_GPIO, CC13XX_ALLMODE_CLOCKS)
#define tiva_gpio_disableclk()           cc13xx_periph_disableclk(PRCM_PERIPH_GPIO, CC13XX_ALLMODE_CLOCKS)

#define tiva_gpioa_runenable()           tiva_gpio_runenable()
#define tiva_gpioa_rundisable()          tiva_gpio_rundisable()
#define tiva_gpioa_sleepenable()         tiva_gpio_sleepenable()
#define tiva_gpioa_sleepdisable()        tiva_gpio_sleepdisable()
#define tiva_gpioa_deepsleepenable()     tiva_gpio_deepsleepenable()
#define tiva_gpioa_deepsleepdisable()    tiva_gpio_deepsleepdisable()
#define tiva_gpioa_enableclk()           tiva_gpio_enableclk()
#define tiva_gpioa_disableclk()          tiva_gpio_disableclk()

/* μDMA Clock Control */

#define tiva_udma_runenable()            cc13xx_periph_enableclk(PRCM_PERIPH_UDMA, CC13XX_RUNMODE_CLOCK)
#define tiva_udma_rundisable()           cc13xx_periph_disableclk(PRCM_PERIPH_UDMA, CC13XX_RUNMODE_CLOCK)
#define tiva_udma_sleepenable()          cc13xx_periph_enableclk(PRCM_PERIPH_UDMA, CC13XX_SLEEPMODE_CLOCK)
#define tiva_udma_sleepdisable()         cc13xx_periph_disableclk(PRCM_PERIPH_UDMA, CC13XX_SLEEPMODE_CLOCK)
#define tiva_udma_deepsleepenable()      cc13xx_periph_enableclk(PRCM_PERIPH_UDMA, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_udma_deepsleepdisable()     cc13xx_periph_disableclk(PRCM_PERIPH_UDMA, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_udma_enableclk()            cc13xx_periph_enableclk(PRCM_PERIPH_UDMA, CC13XX_ALLMODE_CLOCKS)
#define tiva_udma_disableclk()           cc13xx_periph_disableclk(PRCM_PERIPH_UDMA, CC13XX_ALLMODE_CLOCKS)

/* UART Clock Control */

#define tiva_uart0_runenable()           cc13xx_periph_enableclk(PRCM_PERIPH_UART0, CC13XX_RUNMODE_CLOCK)
#define tiva_uart0_rundisable()          cc13xx_periph_disableclk(PRCM_PERIPH_UART0, CC13XX_RUNMODE_CLOCK)
#define tiva_uart0_sleepenable()         cc13xx_periph_enableclk(PRCM_PERIPH_UART0, CC13XX_SLEEPMODE_CLOCK)
#define tiva_uart0_sleepdisable()        cc13xx_periph_disableclk(PRCM_PERIPH_UART0, CC13XX_SLEEPMODE_CLOCK)
#define tiva_uart0_deepsleepenable()     cc13xx_periph_enableclk(PRCM_PERIPH_UART0, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_uart0_deepsleepdisable()    cc13xx_periph_disableclk(PRCM_PERIPH_UART0, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_uart0_enableclk()           cc13xx_periph_enableclk(PRCM_PERIPH_UART0, CC13XX_ALLMODE_CLOCKS)
#define tiva_uart0_disableclk()          cc13xx_periph_disableclk(PRCM_PERIPH_UART0, CC13XX_ALLMODE_CLOCKS)

#ifdef CONFIG_ARCH_CHIP_CC13X2
#  define tiva_uart1_runenable()         cc13xx_periph_enableclk(PRCM_PERIPH_UART1, CC13XX_RUNMODE_CLOCK)
#  define tiva_uart1_rundisable()        cc13xx_periph_disableclk(PRCM_PERIPH_UART1, CC13XX_RUNMODE_CLOCK)
#  define tiva_uart1_sleepenable()       cc13xx_periph_enableclk(PRCM_PERIPH_UART1, CC13XX_SLEEPMODE_CLOCK)
#  define tiva_uart1_sleepdisable()      cc13xx_periph_disableclk(PRCM_PERIPH_UART1, CC13XX_SLEEPMODE_CLOCK)
#  define tiva_uart1_deepsleepenable()   cc13xx_periph_enableclk(PRCM_PERIPH_UART1, CC13XX_DEEPSLEEPMODE_CLOCK)
#  define tiva_uart1_deepsleepdisable()  cc13xx_periph_disableclk(PRCM_PERIPH_UART1, CC13XX_DEEPSLEEPMODE_CLOCK)
#  define tiva_uart1_enableclk()         cc13xx_periph_enableclk(PRCM_PERIPH_UART1, CC13XX_ALLMODE_CLOCKS)
#  define tiva_uart1_disableclk()        cc13xx_periph_disableclk(PRCM_PERIPH_UART1, CC13XX_ALLMODE_CLOCKS)
#endif

/* SSI Clock Control */

#define tiva_ssi0_runenable()            cc13xx_periph_enableclk(PRCM_PERIPH_SSI0, CC13XX_RUNMODE_CLOCK)
#define tiva_ssi0_rundisable()           cc13xx_periph_disableclk(PRCM_PERIPH_SSI0, CC13XX_RUNMODE_CLOCK)
#define tiva_ssi0_sleepenable()          cc13xx_periph_enableclk(PRCM_PERIPH_SSI0, CC13XX_SLEEPMODE_CLOCK)
#define tiva_ssi0_sleepdisable()         cc13xx_periph_disableclk(PRCM_PERIPH_SSI0, CC13XX_SLEEPMODE_CLOCK)
#define tiva_ssi0_deepsleepenable()      cc13xx_periph_enableclk(PRCM_PERIPH_SSI0, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_ssi0_deepsleepdisable()     cc13xx_periph_disableclk(PRCM_PERIPH_SSI0, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_ssi0_enableclk()            cc13xx_periph_enableclk(PRCM_PERIPH_SSI0, CC13XX_ALLMODE_CLOCKS)
#define tiva_ssi0_disableclk()           cc13xx_periph_disableclk(PRCM_PERIPH_SSI0, CC13XX_ALLMODE_CLOCKS)

#define tiva_ssi1_runenable()            cc13xx_periph_enableclk(PRCM_PERIPH_SSI1, CC13XX_RUNMODE_CLOCK)
#define tiva_ssi1_rundisable()           cc13xx_periph_disableclk(PRCM_PERIPH_SSI1, CC13XX_RUNMODE_CLOCK)
#define tiva_ssi1_sleepenable()          cc13xx_periph_enableclk(PRCM_PERIPH_SSI1, CC13XX_SLEEPMODE_CLOCK)
#define tiva_ssi1_sleepdisable()         cc13xx_periph_disableclk(PRCM_PERIPH_SSI1, CC13XX_SLEEPMODE_CLOCK)
#define tiva_ssi1_deepsleepenable()      cc13xx_periph_enableclk(PRCM_PERIPH_SSI1, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_ssi1_deepsleepdisable()     cc13xx_periph_disableclk(PRCM_PERIPH_SSI1, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_ssi1_enableclk()            cc13xx_periph_enableclk(PRCM_PERIPH_SSI1, CC13XX_ALLMODE_CLOCKS)
#define tiva_ssi1_disableclk()           cc13xx_periph_disableclk(PRCM_PERIPH_SSI1, CC13XX_ALLMODE_CLOCKS)

/* I2C Clock Control */

#define tiva_i2c0_runenable()            cc13xx_periph_enableclk(PRCM_PERIPH_I2C0, CC13XX_RUNMODE_CLOCK)
#define tiva_i2c0_rundisable()           cc13xx_periph_disableclk(PRCM_PERIPH_I2C0, CC13XX_RUNMODE_CLOCK)
#define tiva_i2c0_sleepenable()          cc13xx_periph_enableclk(PRCM_PERIPH_I2C0, CC13XX_SLEEPMODE_CLOCK)
#define tiva_i2c0_sleepdisable()         cc13xx_periph_disableclk(PRCM_PERIPH_I2C0, CC13XX_SLEEPMODE_CLOCK)
#define tiva_i2c0_deepsleepenable()      cc13xx_periph_enableclk(PRCM_PERIPH_I2C0, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_i2c0_deepsleepdisable()     cc13xx_periph_disableclk(PRCM_PERIPH_I2C0, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_i2c0_enableclk()            cc13xx_periph_enableclk(PRCM_PERIPH_I2C0, CC13XX_ALLMODE_CLOCKS)
#define tiva_i2c0_disableclk()           cc13xx_periph_disableclk(PRCM_PERIPH_I2C0, CC13XX_ALLMODE_CLOCKS)

/* I2S Clock Control */

#define tiva_i2s_runenable()             cc13xx_periph_enableclk(PRCM_PERIPH_I2S, CC13XX_RUNMODE_CLOCK)
#define tiva_i2s_rundisable()            cc13xx_periph_disableclk(PRCM_PERIPH_I2S, CC13XX_RUNMODE_CLOCK)
#define tiva_i2s_sleepenable()           cc13xx_periph_enableclk(PRCM_PERIPH_I2S, CC13XX_SLEEPMODE_CLOCK)
#define tiva_i2s_sleepdisable()          cc13xx_periph_disableclk(PRCM_PERIPH_I2S, CC13XX_SLEEPMODE_CLOCK)
#define tiva_i2s_deepsleepenable()       cc13xx_periph_enableclk(PRCM_PERIPH_I2S, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_i2s_deepsleepdisable()      cc13xx_periph_disableclk(PRCM_PERIPH_I2S, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_i2s_enableclk()             cc13xx_periph_enableclk(PRCM_PERIPH_I2S, CC13XX_ALLMODE_CLOCKS)
#define tiva_i2s_disableclk()            cc13xx_periph_disableclk(PRCM_PERIPH_I2S, CC13XX_ALLMODE_CLOCKS)

#define tiva_i2s0_runenable()            tiva_i2s_runenable()
#define tiva_i2s0_rundisable()           tiva_i2s_rundisable()
#define tiva_i2s0_sleepenable()          tiva_i2s_sleepenable()
#define tiva_i2s0_sleepdisable()         tiva_i2s_sleepdisable()
#define tiva_i2s0_deepsleepenable()      tiva_i2s_deepsleepenable()
#define tiva_i2s0_deepsleepdisable()     tiva_i2s_deepsleepdisable()
#define tiva_i2s0_enableclk()            tiva_i2s_enableclk()
#define tiva_i2s0_disableclk()           tiva_i2s_disableclk()

/* CRC/PKA Clock Control */

#define tiva_crypto_runenable()          cc13xx_periph_enableclk(PRCM_PERIPH_CRYPTO, CC13XX_RUNMODE_CLOCK)
#define tiva_crypto_rundisable()         cc13xx_periph_disableclk(PRCM_PERIPH_CRYPTO, CC13XX_RUNMODE_CLOCK)
#define tiva_crypto_sleepenable()        cc13xx_periph_enableclk(PRCM_PERIPH_CRYPTO, CC13XX_SLEEPMODE_CLOCK)
#define tiva_crypto_sleepdisable()       cc13xx_periph_disableclk(PRCM_PERIPH_CRYPTO, CC13XX_SLEEPMODE_CLOCK)
#define tiva_crypto_deepsleepenable()    cc13xx_periph_enableclk(PRCM_PERIPH_CRYPTO, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_crypto_deepsleepdisable()   cc13xx_periph_disableclk(PRCM_PERIPH_CRYPTO, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_crypto_enableclk()          cc13xx_periph_enableclk(PRCM_PERIPH_CRYPTO, CC13XX_ALLMODE_CLOCKS)
#define tiva_crypto_disableclk()         cc13xx_periph_disableclk(PRCM_PERIPH_CRYPTO, CC13XX_ALLMODE_CLOCKS)

#ifdef CONFIG_ARCH_CHIP_CC13X2
#  define tiva_pka_runenable()           cc13xx_periph_enableclk(PRCM_PERIPH_PKA, CC13XX_RUNMODE_CLOCK)
#  define tiva_pka_rundisable()          cc13xx_periph_disableclk(PRCM_PERIPH_PKA, CC13XX_RUNMODE_CLOCK)
#  define tiva_pka_sleepenable()         cc13xx_periph_enableclk(PRCM_PERIPH_PKA, CC13XX_SLEEPMODE_CLOCK)
#  define tiva_pka_sleepdisable()        cc13xx_periph_disableclk(PRCM_PERIPH_PKA, CC13XX_SLEEPMODE_CLOCK)
#  define tiva_pka_deepsleepenable()     cc13xx_periph_enableclk(PRCM_PERIPH_PKA, CC13XX_DEEPSLEEPMODE_CLOCK)
#  define tiva_pka_deepsleepdisable()    cc13xx_periph_disableclk(PRCM_PERIPH_PKA, CC13XX_DEEPSLEEPMODE_CLOCK)
#  define tiva_pka_enableclk()           cc13xx_periph_enableclk(PRCM_PERIPH_PKA, CC13XX_ALLMODE_CLOCKS)
#  define tiva_pka_disableclk()          cc13xx_periph_disableclk(PRCM_PERIPH_PKA, CC13XX_ALLMODE_CLOCKS)
#endif

/* TRNG Clock Control */

#define tiva_trng_runenable()            cc13xx_periph_enableclk(PRCM_PERIPH_TRNG, CC13XX_RUNMODE_CLOCK)
#define tiva_trng_rundisable()           cc13xx_periph_disableclk(PRCM_PERIPH_TRNG, CC13XX_RUNMODE_CLOCK)
#define tiva_trng_sleepenable()          cc13xx_periph_enableclk(PRCM_PERIPH_TRNG, CC13XX_SLEEPMODE_CLOCK)
#define tiva_trng_sleepdisable()         cc13xx_periph_disableclk(PRCM_PERIPH_TRNG, CC13XX_SLEEPMODE_CLOCK)
#define tiva_trng_deepsleepenable()      cc13xx_periph_enableclk(PRCM_PERIPH_TRNG, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_trng_deepsleepdisable()     cc13xx_periph_disableclk(PRCM_PERIPH_TRNG, CC13XX_DEEPSLEEPMODE_CLOCK)
#define tiva_trng_enableclk()            cc13xx_periph_enableclk(PRCM_PERIPH_TRNG, CC13XX_ALLMODE_CLOCKS)
#define tiva_trng_disableclk()           cc13xx_periph_disableclk(PRCM_PERIPH_TRNG, CC13XX_ALLMODE_CLOCKS)

/****************************************************************************************************
 * Public Function Prototypes
 ****************************************************************************************************/

/****************************************************************************************************
 * Name:  cc13xx_periph_enableclks
 *
 * Description:
 *   Enable clocking in the selected modes for this peripheral.
 *
 ****************************************************************************************************/

void cc13xx_periph_enableclk(uint32_t peripheral, uint32_t modeset);

/****************************************************************************************************
 * Name:  cc13xx_periph_disableclk
 *
 * Description:
 *   Disable clocking in the selected modes for this peripheral.
 *
 ****************************************************************************************************/

void cc13xx_periph_disableclk(uint32_t peripheral, uint32_t modeset);

#endif /* __ARCH_ARM_SRC_TIVA_CC13XX_CC13XX_ENABLECLKS_H */
