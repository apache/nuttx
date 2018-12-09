/************************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13xx_enablepwr.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_CC13XX_CC13XX_ENABLEPWR_H
#define __ARCH_ARM_SRC_TIVA_CC13XX_CC13XX_ENABLEPWR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "cc13xx/cc13xx_prcm.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* CC13xx Power Domains:
 *
 * 1) PRCM_DOMAIN_RFCORE : RF Core
 * 2) PRCM_DOMAIN_SERIAL : SSI0, UART0, I2C0
 * 3) PRCM_DOMAIN_PERIPH : GPT0, GPT1, GPT2, GPT3, GPIO, SSI1, I2S, DMA, UART1
 * 4) PRCM_DOMAIN_VIMS   : SRAM, FLASH, ROM
 * 5) PRCM_DOMAIN_SYSBUS
 * 6) PRCM_DOMAIN_CPU
 */

/* Watchdog Timer Power Control */

#define tiva_wdt0_enablepwr()
#define tiva_wdt0_disablepwr()

/* 16/32-Bit Timer Power Control */

#define tiva_gptm0_enablepwr()     cc13xx_periph_enablepwr(PRCM_PERIPH_TIMER0)
#define tiva_gptm0_disablepwr()    cc13xx_periph_disablepwr(PRCM_PERIPH_TIMER0)

#define tiva_gptm1_enablepwr()     cc13xx_periph_enablepwr(PRCM_PERIPH_TIMER1)
#define tiva_gptm1_disablepwr()    cc13xx_periph_disablepwr(PRCM_PERIPH_TIMER1)

#define tiva_gptm2_enablepwr()     cc13xx_periph_enablepwr(PRCM_PERIPH_TIMER2)
#define tiva_gptm2_disablepwr()    cc13xx_periph_disablepwr(PRCM_PERIPH_TIMER2)

#define tiva_gptm3_enablepwr()     cc13xx_periph_enablepwr(PRCM_PERIPH_TIMER3)
#define tiva_gptm3_disablepwr()    cc13xx_periph_disablepwr(PRCM_PERIPH_TIMER3)

/* GPIO Power Control */

#define tiva_gpio_enablepwr()      cc13xx_periph_enablepwr(PRCM_PERIPH_GPIO)
#define tiva_gpio_disablepwr()     cc13xx_periph_disablepwr(PRCM_PERIPH_GPIO)

#define tiva_gpioa_enablepwr()     tiva_gpio_enablepwr()
#define tiva_gpioa_disablepwr()    tiva_gpio_disablepwr()

/* μDMA Power Control */

#define tiva_udma_enablepwr()      cc13xx_periph_enablepwr(PRCM_PERIPH_UDMA)
#define tiva_udma_disablepwr()     cc13xx_periph_disablepwr(PRCM_PERIPH_UDMA)

/* UART Power Control */

#define tiva_uart0_enablepwr()     cc13xx_periph_enablepwr(PRCM_PERIPH_UART0)
#define tiva_uart0_disablepwr()    cc13xx_periph_disablepwr(PRCM_PERIPH_UART0)

#ifdef CONFIG_ARCH_CHIP_CC13X2
#  define tiva_uart1_enablepwr()   cc13xx_periph_enablepwr(PRCM_PERIPH_UART1)
#  define tiva_uart1_disablepwr()  cc13xx_periph_disablepwr(PRCM_PERIPH_UART1)
#endif

/* SSI Power Control */

#define tiva_ssi0_enablepwr()      cc13xx_periph_enablepwr(PRCM_PERIPH_SSI0)
#define tiva_ssi0_disablepwr()     cc13xx_periph_disablepwr(PRCM_PERIPH_SSI0)

#define tiva_ssi1_enablepwr()      cc13xx_periph_enablepwr(PRCM_PERIPH_SSI1)
#define tiva_ssi1_disablepwr()     cc13xx_periph_disablepwr(PRCM_PERIPH_SSI1)

/* I2C Power Control */

#define tiva_i2c0_enablepwr()      cc13xx_periph_enablepwr(PRCM_PERIPH_I2C0)
#define tiva_i2c0_disablepwr()     cc13xx_periph_disablepwr(PRCM_PERIPH_I2C0)

/* I2S Power Control */

#define tiva_i2s_enablepwr()       cc13xx_periph_enablepwr(PRCM_PERIPH_I2S)
#define tiva_i2s_disablepwr()      cc13xx_periph_disablepwr(PRCM_PERIPH_I2S)

#define tiva_i2s0_enablepwr()      tiva_i2s_enablepwr()
#define tiva_i2s0_disablepwr()     tiva_i2s_disablepwr()

/* CRC/PKA Power Control */

#define tiva_crypto_enablepwr()    cc13xx_periph_enablepwr(PRCM_PERIPH_CRYPTO)
#define tiva_crypto_disablepwr()   cc13xx_periph_disablepwr(PRCM_PERIPH_CRYPTO)

#ifdef CONFIG_ARCH_CHIP_CC13X2
#  define tiva_pka_enablepwr()     cc13xx_periph_enablepwr(PRCM_PERIPH_PKA)
#  define tiva_pka_disablepwr()    cc13xx_periph_disablepwr(PRCM_PERIPH_PKA)
#endif

/* TRNG Power Control */

#define tiva_trng_enablepwr()      cc13xx_periph_enablepwr(PRCM_PERIPH_TRNG)
#define tiva_trng_disablepwr()     cc13xx_periph_disablepwr(PRCM_PERIPH_TRNG)

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name:  cc13xx_periph_enablepwr
 *
 * Description:
 *   Enable the power domain associated with the peripheral.
 *
 ************************************************************************************/

void cc13xx_periph_enablepwr(uint32_t peripheral);

/************************************************************************************
 * Name:  cc13xx_periph_disablepwr
 *
 * Description:
 *   Disable the power domain associated with the peripheral if and only if all
 *   peripherals using that power domain no longer need power.
 *
 ************************************************************************************/

void cc13xx_periph_disablepwr(uint32_t peripheral);

#endif /* __ARCH_ARM_SRC_TIVA_CC13XX_CC13XX_ENABLEPWR_H */
