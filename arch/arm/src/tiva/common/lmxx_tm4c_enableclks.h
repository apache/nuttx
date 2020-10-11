/************************************************************************************
 * arch/arm/src/tiva/common/lmxx_tm4c_enableclks.h
 *
 *   Copyright (C) 2014, 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_COMMON_LMXX_TM4C_ENABLECLKS_H
#define __ARCH_ARM_SRC_TIVA_COMMON_LMXX_TM4C_ENABLECLKS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "arm_arch.h"
#include "chip.h"
#include "hardware/tiva_sysctrl.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocks are enabled or disabled by setting or clearing a bit (b) in a system
 * control register (a))
 */

#define tiva_enableclk(a,b)        modifyreg32((a),0,(b))
#define tiva_disableclk(a,b)       modifyreg32((a),(b),0)

/* Watchdog Timer Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCWD
#  define tiva_wdt_enableclk(p)    tiva_enableclk(TIVA_SYSCON_RCGCWD,SYSCON_RCGCWD(p))
#  define tiva_wdt_disableclk(p)   tiva_disableclk(TIVA_SYSCON_RCGCWD,SYSCON_RCGCWD(p))

#  define tiva_wdt0_enableclk()    tiva_wdt_enableclk(0)
#  define tiva_wdt1_enableclk()    tiva_wdt_enableclk(1)

#  define tiva_wdt0_disableclk()   tiva_wdt_disableclk(0)
#  define tiva_wdt1_disableclk()   tiva_wdt_disableclk(1)
#else
#endif

/* 16/32-Bit Timer Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCTIMER
#  define tiva_gptm_enableclk(p)   tiva_enableclk(TIVA_SYSCON_RCGCTIMER,SYSCON_RCGCTIMER(p))
#  define tiva_gptm_disableclk(p)  tiva_disableclk(TIVA_SYSCON_RCGCTIMER,SYSCON_RCGCTIMER(p))

#  define tiva_gptm0_enableclk()   tiva_gptm_enableclk(0)
#  define tiva_gptm1_enableclk()   tiva_gptm_enableclk(1)
#  define tiva_gptm2_enableclk()   tiva_gptm_enableclk(2)
#  define tiva_gptm3_enableclk()   tiva_gptm_enableclk(3)
#  define tiva_gptm4_enableclk()   tiva_gptm_enableclk(4)
#  define tiva_gptm5_enableclk()   tiva_gptm_enableclk(5)
#  define tiva_gptm6_enableclk()   tiva_gptm_enableclk(6)
#  define tiva_gptm7_enableclk()   tiva_gptm_enableclk(7)

#  define tiva_gptm0_disableclk()  tiva_gptm_disableclk(0)
#  define tiva_gptm1_disableclk()  tiva_gptm_disableclk(1)
#  define tiva_gptm2_disableclk()  tiva_gptm_disableclk(2)
#  define tiva_gptm3_disableclk()  tiva_gptm_disableclk(3)
#  define tiva_gptm4_disableclk()  tiva_gptm_disableclk(4)
#  define tiva_gptm5_disableclk()  tiva_gptm_disableclk(5)
#  define tiva_gptm6_disableclk()  tiva_gptm_disableclk(6)
#  define tiva_gptm7_disableclk()  tiva_gptm_disableclk(7)
#else
#endif

/* GPIO Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCGPIO
#  define tiva_gpio_enableclk(p)   tiva_enableclk(TIVA_SYSCON_RCGCGPIO,SYSCON_RCGCGPIO(p))
#  define tiva_gpio_disableclk(p)  tiva_disableclk(TIVA_SYSCON_RCGCGPIO,SYSCON_RCGCGPIO(p))

#  define tiva_gpioa_enableclk()   tiva_gpio_enableclk(0)
#  define tiva_gpiob_enableclk()   tiva_gpio_enableclk(1)
#  define tiva_gpioc_enableclk()   tiva_gpio_enableclk(2)
#  define tiva_gpiod_enableclk()   tiva_gpio_enableclk(3)
#  define tiva_gpioe_enableclk()   tiva_gpio_enableclk(4)
#  define tiva_gpiof_enableclk()   tiva_gpio_enableclk(5)
#  define tiva_gpiog_enableclk()   tiva_gpio_enableclk(6)
#  define tiva_gpioh_enableclk()   tiva_gpio_enableclk(7)
#  define tiva_gpioj_enableclk()   tiva_gpio_enableclk(8)
#  define tiva_gpiok_enableclk()   tiva_gpio_enableclk(9)
#  define tiva_gpiol_enableclk()   tiva_gpio_enableclk(10)
#  define tiva_gpiom_enableclk()   tiva_gpio_enableclk(11)
#  define tiva_gpion_enableclk()   tiva_gpio_enableclk(12)
#  define tiva_gpiop_enableclk()   tiva_gpio_enableclk(13)
#  define tiva_gpioq_enableclk()   tiva_gpio_enableclk(14)
#  define tiva_gpior_enableclk()   tiva_gpio_enableclk(15)
#  define tiva_gpios_enableclk()   tiva_gpio_enableclk(16)
#  define tiva_gpiot_enableclk()   tiva_gpio_enableclk(17)

#  define tiva_gpioa_disableclk()  tiva_gpio_disableclk(0)
#  define tiva_gpiob_disableclk()  tiva_gpio_disableclk(1)
#  define tiva_gpioc_disableclk()  tiva_gpio_disableclk(2)
#  define tiva_gpiod_disableclk()  tiva_gpio_disableclk(3)
#  define tiva_gpioe_disableclk()  tiva_gpio_disableclk(4)
#  define tiva_gpiof_disableclk()  tiva_gpio_disableclk(5)
#  define tiva_gpiog_disableclk()  tiva_gpio_disableclk(6)
#  define tiva_gpioh_disableclk()  tiva_gpio_disableclk(7)
#  define tiva_gpioj_disableclk()  tiva_gpio_disableclk(8)
#  define tiva_gpiok_disableclk()  tiva_gpio_disableclk(9)
#  define tiva_gpiol_disableclk()  tiva_gpio_disableclk(10)
#  define tiva_gpiom_disableclk()  tiva_gpio_disableclk(11)
#  define tiva_gpion_disableclk()  tiva_gpio_disableclk(12)
#  define tiva_gpiop_disableclk()  tiva_gpio_disableclk(13)
#  define tiva_gpioq_disableclk()  tiva_gpio_disableclk(14)
#  define tiva_gpior_disableclk()  tiva_gpio_disableclk(15)
#  define tiva_gpios_disableclk()  tiva_gpio_disableclk(16)
#  define tiva_gpiot_disableclk()  tiva_gpio_disableclk(17)

#else
#  define tiva_gpio_enableclk(p)   tiva_enableclk(TIVA_SYSCON_RCGC2,SYSCON_RCGC2_GPIO(p))
#  define tiva_gpio_disableclk(p)  tiva_disableclk(TIVA_SYSCON_RCGC2,SYSCON_RCGC2_GPIO(p))

#  define tiva_gpioa_enableclk()   tiva_gpio_enableclk(0)
#  define tiva_gpiob_enableclk()   tiva_gpio_enableclk(1)
#  define tiva_gpioc_enableclk()   tiva_gpio_enableclk(2)
#  define tiva_gpiod_enableclk()   tiva_gpio_enableclk(3)
#  define tiva_gpioe_enableclk()   tiva_gpio_enableclk(4)
#  define tiva_gpiof_enableclk()   tiva_gpio_enableclk(5)
#  define tiva_gpiog_enableclk()   tiva_gpio_enableclk(6)
#  define tiva_gpioh_enableclk()   tiva_gpio_enableclk(7)
#  define tiva_gpioj_enableclk()   tiva_gpio_enableclk(8)
#  define tiva_gpiok_enableclk()   tiva_gpio_enableclk(9)
#  define tiva_gpiol_enableclk()   tiva_gpio_enableclk(10)
#  define tiva_gpiom_enableclk()   tiva_gpio_enableclk(11)
#  define tiva_gpion_enableclk()   tiva_gpio_enableclk(12)
#  define tiva_gpiop_enableclk()   tiva_gpio_enableclk(13)
#  define tiva_gpioq_enableclk()   tiva_gpio_enableclk(14)

#  define tiva_gpioa_disableclk()  tiva_gpio_disableclk(0)
#  define tiva_gpiob_disableclk()  tiva_gpio_disableclk(1)
#  define tiva_gpioc_disableclk()  tiva_gpio_disableclk(2)
#  define tiva_gpiod_disableclk()  tiva_gpio_disableclk(3)
#  define tiva_gpioe_disableclk()  tiva_gpio_disableclk(4)
#  define tiva_gpiof_disableclk()  tiva_gpio_disableclk(5)
#  define tiva_gpiog_disableclk()  tiva_gpio_disableclk(6)
#  define tiva_gpioh_disableclk()  tiva_gpio_disableclk(7)
#  define tiva_gpioj_disableclk()  tiva_gpio_disableclk(8)
#  define tiva_gpiok_disableclk()  tiva_gpio_disableclk(9)
#  define tiva_gpiol_disableclk()  tiva_gpio_disableclk(10)
#  define tiva_gpiom_disableclk()  tiva_gpio_disableclk(11)
#  define tiva_gpion_disableclk()  tiva_gpio_disableclk(12)
#  define tiva_gpiop_disableclk()  tiva_gpio_disableclk(13)
#  define tiva_gpioq_disableclk()  tiva_gpio_disableclk(14)

#endif

/* μDMA Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCDMA
#  define tiva_udma_enableclk()    tiva_enableclk(TIVA_SYSCON_RCGCDMA,SYSCON_RCGCDMA_R0)
#  define tiva_udma_disableclk()   tiva_disableclk(TIVA_SYSCON_RCGCDMA,SYSCON_RCGCDMA_R0)
#else
#endif

/* EPI Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCEPI
#  define tiva_epi_enableclk()     tiva_enableclk(TIVA_SYSCON_RCGCEPI,SYSCON_RCGCEPI_R0)
#  define tiva_epi_disableclk()    tiva_disableclk(TIVA_SYSCON_RCGCEPI,SYSCON_RCGCEPI_R0)
#else
#endif

/* Hibernation Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCHIB
#  define tiva_hib_enableclk()     tiva_enableclk(TIVA_SYSCON_RCGCHIB,SYSCON_RCGCHIB_R0)
#  define tiva_hib_disableclk()    tiva_disableclk(TIVA_SYSCON_RCGCHIB,SYSCON_RCGCHIB_R0)
#else
#endif

/* UART Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCUART
#  define tiva_uart_enableclk(p)   tiva_enableclk(TIVA_SYSCON_RCGCUART,SYSCON_RCGCUART(p))
#  define tiva_uart_disableclk(p)  tiva_disableclk(TIVA_SYSCON_RCGCUART,SYSCON_RCGCUART(p))

#  define tiva_uart0_enableclk()   tiva_uart_enableclk(0)
#  define tiva_uart1_enableclk()   tiva_uart_enableclk(1)
#  define tiva_uart2_enableclk()   tiva_uart_enableclk(2)
#  define tiva_uart3_enableclk()   tiva_uart_enableclk(3)
#  define tiva_uart4_enableclk()   tiva_uart_enableclk(4)
#  define tiva_uart5_enableclk()   tiva_uart_enableclk(5)
#  define tiva_uart6_enableclk()   tiva_uart_enableclk(6)
#  define tiva_uart7_enableclk()   tiva_uart_enableclk(7)

#  define tiva_uart0_disableclk()  tiva_uart_disableclk(0)
#  define tiva_uart1_disableclk()  tiva_uart_disableclk(1)
#  define tiva_uart2_disableclk()  tiva_uart_disableclk(2)
#  define tiva_uart3_disableclk()  tiva_uart_disableclk(3)
#  define tiva_uart4_disableclk()  tiva_uart_disableclk(4)
#  define tiva_uart5_disableclk()  tiva_uart_disableclk(5)
#  define tiva_uart6_disableclk()  tiva_uart_disableclk(6)
#  define tiva_uart7_disableclk()  tiva_uart_disableclk(7)
#else
#if defined(CONFIG_ARCH_CHIP_LM3S9B92)
#  define tiva_uart_enableclk(p)   tiva_enableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGCUART(p))
#  define tiva_uart_disableclk(p)  tiva_disableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGCUART(p))
#endif

#  define tiva_uart0_enableclk()   tiva_enableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_UART0)
#  define tiva_uart1_enableclk()   tiva_enableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_UART1)
#  define tiva_uart2_enableclk()   tiva_enableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_UART2)

#  define tiva_uart0_disableclk()  tiva_disableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_UART0)
#  define tiva_uart1_disableclk()  tiva_disableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_UART1)
#  define tiva_uart2_disableclk()  tiva_disableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_UART2)
#endif

/* SSI Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCSSI
#  define tiva_ssi_enableclk(p)    tiva_enableclk(TIVA_SYSCON_RCGCSSI,SYSCON_RCGCSSI(p))
#  define tiva_ssi_disableclk(p)   tiva_disableclk(TIVA_SYSCON_RCGCSSI,SYSCON_RCGCSSI(p))

#  define tiva_ssi0_enableclk()    tiva_ssi_enableclk(0)
#  define tiva_ssi1_enableclk()    tiva_ssi_enableclk(1)
#  define tiva_ssi2_enableclk()    tiva_ssi_enableclk(2)
#  define tiva_ssi3_enableclk()    tiva_ssi_enableclk(3)

#  define tiva_ssi0_disableclk()   tiva_ssi_disableclk(0)
#  define tiva_ssi1_disableclk()   tiva_ssi_disableclk(1)
#  define tiva_ssi2_disableclk()   tiva_ssi_disableclk(2)
#  define tiva_ssi3_disableclk()   tiva_ssi_disableclk(3)
#else
#  define tiva_ssi0_enableclk()    tiva_enableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_SSI0)
#  define tiva_ssi1_enableclk()    tiva_enableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_SSI1)

#  define tiva_ssi0_disableclk()   tiva_disableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_SSI0)
#  define tiva_ssi1_disableclk()   tiva_disableclk(TIVA_SYSCON_RCGC1,SYSCON_RCGC1_SSI1)
#endif

/* I2C Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCI2C
#  define tiva_i2c_enableclk(p)    tiva_enableclk(TIVA_SYSCON_RCGCI2C,SYSCON_RCGCI2C(p))
#  define tiva_i2c_disableclk(p)   tiva_disableclk(TIVA_SYSCON_RCGCI2C,SYSCON_RCGCI2C(p))

#  define tiva_i2c0_enableclk()    tiva_i2c_enableclk(0)
#  define tiva_i2c1_enableclk()    tiva_i2c_enableclk(1)
#  define tiva_i2c2_enableclk()    tiva_i2c_enableclk(2)
#  define tiva_i2c3_enableclk()    tiva_i2c_enableclk(3)
#  define tiva_i2c4_enableclk()    tiva_i2c_enableclk(4)
#  define tiva_i2c5_enableclk()    tiva_i2c_enableclk(5)
#  define tiva_i2c6_enableclk()    tiva_i2c_enableclk(6)
#  define tiva_i2c7_enableclk()    tiva_i2c_enableclk(7)
#  define tiva_i2c8_enableclk()    tiva_i2c_enableclk(8)
#  define tiva_i2c9_enableclk()    tiva_i2c_enableclk(9)

#  define tiva_i2c0_disableclk()   tiva_i2c_disableclk(0)
#  define tiva_i2c1_disableclk()   tiva_i2c_disableclk(1)
#  define tiva_i2c2_disableclk()   tiva_i2c_disableclk(2)
#  define tiva_i2c3_disableclk()   tiva_i2c_disableclk(3)
#  define tiva_i2c4_disableclk()   tiva_i2c_disableclk(4)
#  define tiva_i2c5_disableclk()   tiva_i2c_disableclk(5)
#  define tiva_i2c6_disableclk()   tiva_i2c_disableclk(6)
#  define tiva_i2c7_disableclk()   tiva_i2c_disableclk(7)
#  define tiva_i2c8_disableclk()   tiva_i2c_disableclk(8)
#  define tiva_i2c9_disableclk()   tiva_i2c_disableclk(9)
#else
#endif

/* USB Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCUSB
#  define tiva_usb_enableclk()     tiva_enableclk(TIVA_SYSCON_RCGCUSB,SYSCON_RCGCUSB_R0)
#  define tiva_usb_disableclk()    tiva_disableclk(TIVA_SYSCON_RCGCUSB,SYSCON_RCGCUSB_R0)
#else
#endif

/* Ethernet PHY Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCEPHY
#  define tiva_ephy_enableclk()    tiva_enableclk(TIVA_SYSCON_RCGCEPHY,SYSCON_RCGCEPHY_R0)
#  define tiva_ephy_disableclk()   tiva_disableclk(TIVA_SYSCON_RCGCEPHY,SYSCON_RCGCEPHY_R0)
#else
#endif

/* CAN RunMode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCCAN
#  define tiva_can_enableclk(p)    tiva_enableclk(TIVA_SYSCON_RCGCCAN,SYSCON_RCGCCAN(p))
#  define tiva_can_disableclk(p)   tiva_disableclk(TIVA_SYSCON_RCGCCAN,SYSCON_RCGCCAN(p))

#  define tiva_can0_enableclk()    tiva_can_enableclk(0)
#  define tiva_can1_enableclk()    tiva_can_enableclk(1)

#  define tiva_can0_disableclk()   tiva_can_disableclk(0)
#  define tiva_can1_disableclk()   tiva_can_disableclk(1)
#else
#endif

/* ADC Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCADC
#  define tiva_adc_enableclk(p)    tiva_enableclk(TIVA_SYSCON_RCGCADC,SYSCON_RCGCADC(p))
#  define tiva_adc_disableclk(p)   tiva_disableclk(TIVA_SYSCON_RCGCADC,SYSCON_RCGCADC(p))

#  define tiva_adc0_enableclk()    tiva_adc_enableclk(0)
#  define tiva_adc1_enableclk()    tiva_adc_enableclk(1)

#  define tiva_adc0_disableclk()   tiva_adc_disableclk(0)
#  define tiva_adc1_disableclk()   tiva_adc_disableclk(1)
#else
#endif

/* ACMP Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCACMP
#  define tiva_acmp_enableclk()    tiva_enableclk(TIVA_SYSCON_RCGCACMP,SYSCON_RCGCACMP_R0)
#  define tiva_acmp_disableclk()   tiva_disableclk(TIVA_SYSCON_RCGCACMP,SYSCON_RCGCACMP_R0)
#else
#endif

/* PWM Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCPWM
#  define tiva_pwm_enableclk(p)    tiva_enableclk(TIVA_SYSCON_RCGCPWM,SYSCON_RCGCPWM(p))
#  define tiva_pwm_disableclk(p)   tiva_disableclk(TIVA_SYSCON_RCGCPWM,SYSCON_RCGCPWM(p))

#  define tiva_pwm0_enableclk()    tiva_pwm_enableclk(0)
#  define tiva_pwm1_enableclk()    tiva_pwm_enableclk(1)

#  define tiva_pwm0_disableclk()   tiva_pwm_disableclk(0)
#  define tiva_pwm1_disableclk()   tiva_pwm_disableclk(1)
#else
#endif

/* QE Interface Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCQEI
#  define tiva_qei_enableclk(p)    tiva_enableclk(TIVA_SYSCON_RCGCQEI,SYSCON_RCGCQEI(p))
#  define tiva_qei_disableclk(p)   tiva_disableclk(TIVA_SYSCON_RCGCQEI,SYSCON_RCGCQEI(p))

#  define tiva_qei0_enableclk()    tiva_qei_enableclk(0)
#  define tiva_qei1_enableclk()    tiva_qei_enableclk(1)

#  define tiva_qei0_disableclk()   tiva_qei_disableclk(0)
#  define tiva_qei1_disableclk()   tiva_qei_disableclk(1)
#else
#endif

/* EEPROM Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCEEPROM
#  define tiva_eeprom_enableclk()   tiva_enableclk(TIVA_SYSCON_RCGCEEPROM,SYSCON_RCGCEEPROM_R0)
#  define tiva_eeprom_disableclk()  tiva_disableclk(TIVA_SYSCON_RCGCEEPROM,SYSCON_RCGCEEPROM_R0)
#else
#endif

/* 32/64-Bit Wide Timer Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCWTIMER
#  define tiva_wtm_enableclk(p)    tiva_enableclk(TIVA_SYSCON_RCGCWTIMER,SYSCON_RCGCWTIMER(p))
#  define tiva_wtm_disableclk(p)   tiva_disableclk(TIVA_SYSCON_RCGCWTIMER,SYSCON_RCGCWTIMER(p))

#  define tiva_wtm0_enableclk()    tiva_wtm_enableclk(0)
#  define tiva_wtm1_enableclk()    tiva_wtm_enableclk(1)
#  define tiva_wtm2_enableclk()    tiva_wtm_enableclk(2)
#  define tiva_wtm3_enableclk()    tiva_wtm_enableclk(3)
#  define tiva_wtm4_enableclk()    tiva_wtm_enableclk(4)
#  define tiva_wtm5_enableclk()    tiva_wtm_enableclk(5)

#  define tiva_wtm0_disableclk()   tiva_wtm_disableclk(0)
#  define tiva_wtm1_disableclk()   tiva_wtm_disableclk(1)
#  define tiva_wtm2_disableclk()   tiva_wtm_disableclk(2)
#  define tiva_wtm3_disableclk()   tiva_wtm_disableclk(3)
#  define tiva_wtm4_disableclk()   tiva_wtm_disableclk(4)
#  define tiva_wtm5_disableclk()   tiva_wtm_disableclk(5)
#else
#endif

/* CRC/Crypto Modules RunMode ClockGating Control */

#ifdef TIVA_SYSCON_RCGCCCM
#  define tiva_ccm_enableclk()     tiva_enableclk(TIVA_SYSCON_RCGCCCM,SYSCON_RCGCCCM_R0)
#  define tiva_ccm_disableclk()    tiva_disableclk(TIVA_SYSCON_RCGCCCM,SYSCON_RCGCCCM_R0)
#else
#endif

/* LCD Controller Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCLCD
#  define tiva_lcd_enableclk()     tiva_enableclk(TIVA_SYSCON_RCGCLCD,SYSCON_RCGCLCD_R0)
#  define tiva_lcd_disableclk()    tiva_disableclk(TIVA_SYSCON_RCGCLCD,SYSCON_RCGCLCD_R0)
#else
#endif

/* 1-Wire Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCOWIRE
#  define tiva_owire_enableclk()   tiva_enableclk(TIVA_SYSCON_RCGCOWIRE,SYSCON_RCGCOWIRE_R0)
#  define tiva_owire_disableclk()  tiva_disableclk(TIVA_SYSCON_RCGCOWIRE,SYSCON_RCGCOWIRE_R0)
#else
#endif

/* Ethernet MAC Run Mode Clock Gating Control */

#ifdef TIVA_SYSCON_RCGCEMAC
#  define tiva_emac_enableclk()    tiva_enableclk(TIVA_SYSCON_RCGCEMAC,SYSCON_RCGCEMAC_R0)
#  define tiva_emac_disableclk()   tiva_disableclk(TIVA_SYSCON_RCGCEMAC,SYSCON_RCGCEMAC_R0)
#else
#endif

#endif /* __ARCH_ARM_SRC_TIVA_COMMON_LMXX_TM4C_ENABLECLKS_H */
