/************************************************************************************
 * arch/arm/src/tiva/tiva_periphrdy.h
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVAPERIPHRDY_H
#define __ARCH_ARM_SRC_TIVA_TIVAPERIPHRDY_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "up_arch.h"
#include "chip.h"
#include "chip/tiva_syscontrol.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* We determine if a peripheral is ready by testing a bit (b) in system control
 * register (a).
 */

#define tiva_periphrdy(a,b)        ((getreg32(a) & (b)) != 0)

/* Watchdog Timer Power Control */

#ifdef TIVA_SYSCON_PRWD
#  define tiva_wdt_periphrdy(p)    tiva_periphrdy(TIVA_SYSCON_PRWD,SYSCON_PRWD(p))
#else
#  define tiva_wdt_periphrdy(p)    (true)
#endif

#define tiva_wdt0_periphrdy()      tiva_wdt_periphrdy(0)
#define tiva_wdt1_periphrdy()      tiva_wdt_periphrdy(1)

/* 16/32-Bit Timer Power Control */

#ifdef TIVA_SYSCON_PRTIMER
#  define tiva_gptm_periphrdy(p)   tiva_periphrdy(TIVA_SYSCON_PRTIMER,SYSCON_PRTIMER(p))
#else
#  define tiva_gptm_periphrdy(p)   (true)
#endif

#define tiva_gptm0_periphrdy()     tiva_gptm_periphrdy(0)
#define tiva_gptm1_periphrdy()     tiva_gptm_periphrdy(1)
#define tiva_gptm2_periphrdy()     tiva_gptm_periphrdy(2)
#define tiva_gptm3_periphrdy()     tiva_gptm_periphrdy(3)
#define tiva_gptm4_periphrdy()     tiva_gptm_periphrdy(4)
#define tiva_gptm5_periphrdy()     tiva_gptm_periphrdy(5)
#define tiva_gptm6_periphrdy()     tiva_gptm_periphrdy(6)
#define tiva_gptm7_periphrdy()     tiva_gptm_periphrdy(7)

/* GPIO Power Control */

#ifdef TIVA_SYSCON_PRGPIO
#  define tiva_gpio_periphrdy(p)   tiva_periphrdy(TIVA_SYSCON_PRGPIO,SYSCON_PRGPIO(p))
#else
#  define tiva_gpio_periphrdy(p)   (true)
#endif

#define tiva_gpioa_periphrdy()     tiva_gpio_periphrdy(0)
#define tiva_gpiob_periphrdy()     tiva_gpio_periphrdy(1)
#define tiva_gpioc_periphrdy()     tiva_gpio_periphrdy(2)
#define tiva_gpiod_periphrdy()     tiva_gpio_periphrdy(3)
#define tiva_gpioe_periphrdy()     tiva_gpio_periphrdy(4)
#define tiva_gpiof_periphrdy()     tiva_gpio_periphrdy(5)
#define tiva_gpiog_periphrdy()     tiva_gpio_periphrdy(6)
#define tiva_gpioh_periphrdy()     tiva_gpio_periphrdy(7)
#define tiva_gpioj_periphrdy()     tiva_gpio_periphrdy(8)
#define tiva_gpiok_periphrdy()     tiva_gpio_periphrdy(9)
#define tiva_gpiol_periphrdy()     tiva_gpio_periphrdy(10)
#define tiva_gpiom_periphrdy()     tiva_gpio_periphrdy(11)
#define tiva_gpion_periphrdy()     tiva_gpio_periphrdy(12)
#define tiva_gpiop_periphrdy()     tiva_gpio_periphrdy(13)
#define tiva_gpioq_periphrdy()     tiva_gpio_periphrdy(14)
#define tiva_gpior_periphrdy()     tiva_gpio_periphrdy(15)
#define tiva_gpios_periphrdy()     tiva_gpio_periphrdy(16)
#define tiva_gpiot_periphrdy()     tiva_gpio_periphrdy(17)

/* μDMA Power Control */

#ifdef TIVA_SYSCON_PRDMA
#  define tiva_udma_periphrdy()    tiva_periphrdy(TIVA_SYSCON_PRDMA,SYSCON_PRDMA_R0)
#else
#  define tiva_udma_periphrdy()    (true)
#endif

/* EPI Power Control */

#ifdef TIVA_SYSCON_PREPI
#  define tiva_epi_periphrdy()     tiva_periphrdy(TIVA_SYSCON_PREPI,SYSCON_PREPI_R0)
#else
#  define tiva_epi_periphrdy()     (true)
#endif

/* Hibernation Power Control */

#ifdef TIVA_SYSCON_PRHIB
#  define tiva_hib_periphrdy()     tiva_periphrdy(TIVA_SYSCON_PRHIB,SYSCON_PRHIB_R0)
#else
#  define tiva_hib_periphrdy()     (true)
#endif

/* UART Power Control */

#ifdef TIVA_SYSCON_PRUART
#  define tiva_uart_periphrdy(p)   tiva_periphrdy(TIVA_SYSCON_PRUART,SYSCON_PRUART(p))
#else
#  define tiva_uart_periphrdy(p)   (true)
#endif

#define tiva_uart0_periphrdy()     tiva_uart_periphrdy(0)
#define tiva_uart1_periphrdy()     tiva_uart_periphrdy(1)
#define tiva_uart2_periphrdy()     tiva_uart_periphrdy(2)
#define tiva_uart3_periphrdy()     tiva_uart_periphrdy(3)
#define tiva_uart4_periphrdy()     tiva_uart_periphrdy(4)
#define tiva_uart5_periphrdy()     tiva_uart_periphrdy(5)
#define tiva_uart6_periphrdy()     tiva_uart_periphrdy(6)
#define tiva_uart7_periphrdy()     tiva_uart_periphrdy(7)

/* SSI Power Control */

#ifdef TIVA_SYSCON_PRSSI
#  define tiva_ssi_periphrdy(p)    tiva_periphrdy(TIVA_SYSCON_PRSSI,SYSCON_PRSSI(p))
#else
#  define tiva_ssi_periphrdy(p)    (true)
#endif

#define tiva_ssi0_periphrdy()      tiva_ssi_periphrdy(0)
#define tiva_ssi1_periphrdy()      tiva_ssi_periphrdy(1)
#define tiva_ssi2_periphrdy()      tiva_ssi_periphrdy(2)
#define tiva_ssi3_periphrdy()      tiva_ssi_periphrdy(3)

/* I2C Power Control */

#ifdef TIVA_SYSCON_PRI2C
#  define tiva_i2c_periphrdy(p)    tiva_periphrdy(TIVA_SYSCON_PRI2C,SYSCON_PRI2C(p))
#else
#  define tiva_i2c_periphrdy(p)    (true)
#endif

#define tiva_i2c0_periphrdy()      tiva_i2c_periphrdy(0)
#define tiva_i2c1_periphrdy()      tiva_i2c_periphrdy(1)
#define tiva_i2c2_periphrdy()      tiva_i2c_periphrdy(2)
#define tiva_i2c3_periphrdy()      tiva_i2c_periphrdy(3)
#define tiva_i2c4_periphrdy()      tiva_i2c_periphrdy(4)
#define tiva_i2c5_periphrdy()      tiva_i2c_periphrdy(5)
#define tiva_i2c6_periphrdy()      tiva_i2c_periphrdy(6)
#define tiva_i2c7_periphrdy()      tiva_i2c_periphrdy(7)
#define tiva_i2c8_periphrdy()      tiva_i2c_periphrdy(8)
#define tiva_i2c9_periphrdy()      tiva_i2c_periphrdy(9)

/* USB Power Control */

#ifdef TIVA_SYSCON_PRUSB
#  define tiva_usb_periphrdy()     tiva_periphrdy(TIVA_SYSCON_PRUSB,SYSCON_PRUSB_R0)
#else
#  define tiva_usb_periphrdy()     (true)
#endif

/* Ethernet PHY Power Control */

#ifdef TIVA_SYSCON_PREPHY
#  define tiva_ephy_periphrdy()    tiva_periphrdy(TIVA_SYSCON_PREPHY,SYSCON_PREPHY_R0)
#else
#  define tiva_ephy_periphrdy()    (true)
#endif

/* CAN RunMode Clock Gating Control */

#ifdef TIVA_SYSCON_PRCAN
#  define tiva_can_periphrdy(p)    tiva_periphrdy(TIVA_SYSCON_PRCAN,SYSCON_PRCAN(p))
#else
#  define tiva_can_periphrdy(p)    (true)
#endif

#define tiva_can0_periphrdy()      tiva_can_periphrdy(0)
#define tiva_can1_periphrdy()      tiva_can_periphrdy(1)

/* ADC Power Control */

#ifdef TIVA_SYSCON_PRADC
#  define tiva_adc_periphrdy(p)    tiva_periphrdy(TIVA_SYSCON_PRADC,SYSCON_PRADC(p))
#else
#  define tiva_adc_periphrdy(p)    (true)
#endif

#define tiva_adc0_periphrdy()      tiva_adc_periphrdy(0)
#define tiva_adc1_periphrdy()      tiva_adc_periphrdy(1)

/* ACMP Power Control */

#ifdef TIVA_SYSCON_PRACMP
#  define tiva_acmp_periphrdy()    tiva_periphrdy(TIVA_SYSCON_PRACMP,SYSCON_PRACMP_R0)
#else
#  define tiva_acmp_periphrdy()    (true)
#endif

/* PWM Power Control */

#ifdef TIVA_SYSCON_PRPWM
#  define tiva_pwm_periphrdy(p)    tiva_periphrdy(TIVA_SYSCON_PRPWM,SYSCON_PRPWM(p))
#else
#  define tiva_pwm_periphrdy(p)    (true)
#endif

#define tiva_pwm0_periphrdy()      tiva_pwm_periphrdy(0)
#define tiva_pwm1_periphrdy()      tiva_pwm_periphrdy(1)

/* QE Interface Power Control */

#ifdef TIVA_SYSCON_PRQEI
#  define tiva_qei_periphrdy(p)    tiva_periphrdy(TIVA_SYSCON_PRQEI,SYSCON_PRQEI(p))
#else
#  define tiva_qei_periphrdy(p)    (true)
#endif

#define tiva_qei0_periphrdy()      tiva_qei_periphrdy(0)
#define tiva_qei1_periphrdy()      tiva_qei_periphrdy(1)

/* EEPROM Power Control */

#ifdef TIVA_SYSCON_PREEPROM
#  define tiva_eeprom_periphrdy()   tiva_periphrdy(TIVA_SYSCON_PREEPROM,SYSCON_PREEPROM_R0)
#else
#  define tiva_eeprom_periphrdy()   (true)
#endif

/* 32/64-Bit Wide Timer Power Control */

#ifdef TIVA_SYSCON_PRWTIMER
#  define tiva_wtm_periphrdy(p)    tiva_periphrdy(TIVA_SYSCON_PRWTIMER,SYSCON_PRWTIMER(p))
#else
#  define tiva_wtm_periphrdy(p)    (true)
#endif

#define tiva_wtm0_periphrdy()      tiva_wtm_periphrdy(0)
#define tiva_wtm1_periphrdy()      tiva_wtm_periphrdy(1)
#define tiva_wtm2_periphrdy()      tiva_wtm_periphrdy(2)
#define tiva_wtm3_periphrdy()      tiva_wtm_periphrdy(3)
#define tiva_wtm4_periphrdy()      tiva_wtm_periphrdy(4)
#define tiva_wtm5_periphrdy()      tiva_wtm_periphrdy(5)

/* CRC/Crypto Modules RunMode ClockGating Control */

#ifdef TIVA_SYSCON_PRCCM
#  define tiva_ccm_periphrdy()     tiva_periphrdy(TIVA_SYSCON_PRCCM,SYSCON_PRCCM_R0)
#else
#  define tiva_ccm_periphrdy()     (true)
#endif

/* LCD Controller Power Control */

#ifdef TIVA_SYSCON_PRLCD
#  define tiva_lcd_periphrdy()     tiva_periphrdy(TIVA_SYSCON_PRLCD,SYSCON_PRLCD_R0)
#else
#  define tiva_lcd_periphrdy()    (true)
#endif

/* 1-Wire Power Control */

#ifdef TIVA_SYSCON_PROWIRE
#  define tiva_owire_periphrdy()   tiva_periphrdy(TIVA_SYSCON_PROWIRE,SYSCON_PROWIRE_R0)
#else
#  define tiva_owire_periphrdy()   (true)
#endif

/* Ethernet MAC Power Control */

#ifdef TIVA_SYSCON_PREMAC
#  define tiva_emac_periphrdy()    tiva_periphrdy(TIVA_SYSCON_PREMAC,SYSCON_PREMAC_R0)
#else
#  define tiva_emac_periphrdy()    (true)
#endif

#endif /* __ARCH_ARM_SRC_TIVA_TIVAPERIPHRDY_H */
