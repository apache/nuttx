/************************************************************************************
 * arch/arm/src/tiva/tiva_enablepwr.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_ENABLEPWR_H
#define __ARCH_ARM_SRC_TIVA_TIVA_ENABLEPWR_H

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
/* Power control is enabled or disabled by setting or clearing a bit (b) in a system
 * control register (a))
 */

#define tiva_enablepwr(a,b)        modifyreg32((a),0,(b))
#define tiva_disablepwr(a,b)       modifyreg32((a),(b),0)

/* Watchdog Timer Power Control */

#ifdef TIVA_SYSCON_PCWD
#  define tiva_wdt_enablepwr(p)    tiva_enablepwr(TIVA_SYSCON_PCWD,SYSCON_PCWD(p))
#  define tiva_wdt_disablepwr(p)   tiva_disablepwr(TIVA_SYSCON_PCWD,SYSCON_PCWD(p))
#else
#  define tiva_wdt_enablepwr(p)
#  define tiva_wdt_disablepwr(p)
#endif

#define tiva_wdt0_enablepwr()      tiva_wdt_enablepwr(0)
#define tiva_wdt1_enablepwr()      tiva_wdt_enablepwr(1)

#define tiva_wdt0_disablepwr()     tiva_wdt_disablepwr(0)
#define tiva_wdt1_disablepwr()     tiva_wdt_disablepwr(1)

/* 16/32-Bit Timer Power Control */

#ifdef TIVA_SYSCON_PCTIMER
#  define tiva_gptm_enablepwr(p)   tiva_enablepwr(TIVA_SYSCON_PCTIMER,SYSCON_PCTIMER(p))
#  define tiva_gptm_disablepwr(p)  tiva_disablepwr(TIVA_SYSCON_PCTIMER,SYSCON_PCTIMER(p))
#else
#  define tiva_gptm_enablepwr(p)
#  define tiva_gptm_disablepwr(p)
#endif

#define tiva_gptm0_enablepwr()     tiva_gptm_enablepwr(0)
#define tiva_gptm1_enablepwr()     tiva_gptm_enablepwr(1)
#define tiva_gptm2_enablepwr()     tiva_gptm_enablepwr(2)
#define tiva_gptm3_enablepwr()     tiva_gptm_enablepwr(3)
#define tiva_gptm4_enablepwr()     tiva_gptm_enablepwr(4)
#define tiva_gptm5_enablepwr()     tiva_gptm_enablepwr(5)
#define tiva_gptm6_enablepwr()     tiva_gptm_enablepwr(6)
#define tiva_gptm7_enablepwr()     tiva_gptm_enablepwr(7)

#define tiva_gptm0_disablepwr()    tiva_gptm_disablepwr(0)
#define tiva_gptm1_disablepwr()    tiva_gptm_disablepwr(1)
#define tiva_gptm2_disablepwr()    tiva_gptm_disablepwr(2)
#define tiva_gptm3_disablepwr()    tiva_gptm_disablepwr(3)
#define tiva_gptm4_disablepwr()    tiva_gptm_disablepwr(4)
#define tiva_gptm5_disablepwr()    tiva_gptm_disablepwr(5)
#define tiva_gptm6_disablepwr()    tiva_gptm_disablepwr(6)
#define tiva_gptm7_disablepwr()    tiva_gptm_disablepwr(7)

/* GPIO Power Control */

#ifdef TIVA_SYSCON_PCGPIO
#  define tiva_gpio_enablepwr(p)   tiva_enablepwr(TIVA_SYSCON_PCGPIO,SYSCON_PCGPIO(p))
#  define tiva_gpio_disablepwr(p)  tiva_disablepwr(TIVA_SYSCON_PCGPIO,SYSCON_PCGPIO(p))
#else
#  define tiva_gpio_enablepwr(p)
#  define tiva_gpio_disablepwr(p)
#endif

#define tiva_gpioa_enablepwr()     tiva_gpio_enablepwr(0)
#define tiva_gpiob_enablepwr()     tiva_gpio_enablepwr(1)
#define tiva_gpioc_enablepwr()     tiva_gpio_enablepwr(2)
#define tiva_gpiod_enablepwr()     tiva_gpio_enablepwr(3)
#define tiva_gpioe_enablepwr()     tiva_gpio_enablepwr(4)
#define tiva_gpiof_enablepwr()     tiva_gpio_enablepwr(5)
#define tiva_gpiog_enablepwr()     tiva_gpio_enablepwr(6)
#define tiva_gpioh_enablepwr()     tiva_gpio_enablepwr(7)
#define tiva_gpioj_enablepwr()     tiva_gpio_enablepwr(8)
#define tiva_gpiok_enablepwr()     tiva_gpio_enablepwr(9)
#define tiva_gpiol_enablepwr()     tiva_gpio_enablepwr(10)
#define tiva_gpiom_enablepwr()     tiva_gpio_enablepwr(11)
#define tiva_gpion_enablepwr()     tiva_gpio_enablepwr(12)
#define tiva_gpiop_enablepwr()     tiva_gpio_enablepwr(13)
#define tiva_gpioq_enablepwr()     tiva_gpio_enablepwr(14)
#define tiva_gpior_enablepwr()     tiva_gpio_enablepwr(15)
#define tiva_gpios_enablepwr()     tiva_gpio_enablepwr(16)
#define tiva_gpiot_enablepwr()     tiva_gpio_enablepwr(17)

#define tiva_gpioa_disablepwr()    tiva_gpio_disablepwr(0)
#define tiva_gpiob_disablepwr()    tiva_gpio_disablepwr(1)
#define tiva_gpioc_disablepwr()    tiva_gpio_disablepwr(2)
#define tiva_gpiod_disablepwr()    tiva_gpio_disablepwr(3)
#define tiva_gpioe_disablepwr()    tiva_gpio_disablepwr(4)
#define tiva_gpiof_disablepwr()    tiva_gpio_disablepwr(5)
#define tiva_gpiog_disablepwr()    tiva_gpio_disablepwr(6)
#define tiva_gpioh_disablepwr()    tiva_gpio_disablepwr(7)
#define tiva_gpioj_disablepwr()    tiva_gpio_disablepwr(8)
#define tiva_gpiok_disablepwr()    tiva_gpio_disablepwr(9)
#define tiva_gpiol_disablepwr()    tiva_gpio_disablepwr(10)
#define tiva_gpiom_disablepwr()    tiva_gpio_disablepwr(11)
#define tiva_gpion_disablepwr()    tiva_gpio_disablepwr(12)
#define tiva_gpiop_disablepwr()    tiva_gpio_disablepwr(13)
#define tiva_gpioq_disablepwr()    tiva_gpio_disablepwr(14)
#define tiva_gpior_disablepwr()    tiva_gpio_disablepwr(15)
#define tiva_gpios_disablepwr()    tiva_gpio_disablepwr(16)
#define tiva_gpiot_disablepwr()    tiva_gpio_disablepwr(17)

/* μDMA Power Control */

#ifdef TIVA_SYSCON_PCDMA
#  define tiva_udma_enablepwr()    tiva_enablepwr(TIVA_SYSCON_PCDMA,SYSCON_PCDMA_P0)
#  define tiva_udma_disablepwr()   tiva_disablepwr(TIVA_SYSCON_PCDMA,SYSCON_PCDMA_P0)
#else
#  define tiva_udma_enablepwr()
#  define tiva_udma_disablepwr()
#endif

/* EPI Power Control */

#ifdef TIVA_SYSCON_PCEPI
#  define tiva_epi_enablepwr()     tiva_enablepwr(TIVA_SYSCON_PCEPI,SYSCON_PCEPI_P0)
#  define tiva_epi_disablepwr()    tiva_disablepwr(TIVA_SYSCON_PCEPI,SYSCON_PCEPI_P0)
#else
#  define tiva_epi_enablepwr()
#  define tiva_epi_disablepwr()
#endif

/* Hibernation Power Control */

#ifdef TIVA_SYSCON_PCHIB
#  define tiva_hib_enablepwr()     tiva_enablepwr(TIVA_SYSCON_PCHIB,SYSCON_PCHIB_P0)
#  define tiva_hib_disablepwr()    tiva_disablepwr(TIVA_SYSCON_PCHIB,SYSCON_PCHIB_P0)
#else
#  define tiva_hib_enablepwr()
#  define tiva_hib_disablepwr()
#endif

/* UART Power Control */

#ifdef TIVA_SYSCON_PCUART
#  define tiva_uart_enablepwr(p)   tiva_enablepwr(TIVA_SYSCON_PCUART,SYSCON_PCUART(p))
#  define tiva_uart_disablepwr(p)  tiva_disablepwr(TIVA_SYSCON_PCUART,SYSCON_PCUART(p))
#else
#  define tiva_uart_enablepwr(p)
#  define tiva_uart_disablepwr(p)
#endif

#define tiva_uart0_enablepwr()     tiva_uart_enablepwr(0)
#define tiva_uart1_enablepwr()     tiva_uart_enablepwr(1)
#define tiva_uart2_enablepwr()     tiva_uart_enablepwr(2)
#define tiva_uart3_enablepwr()     tiva_uart_enablepwr(3)
#define tiva_uart4_enablepwr()     tiva_uart_enablepwr(4)
#define tiva_uart5_enablepwr()     tiva_uart_enablepwr(5)
#define tiva_uart6_enablepwr()     tiva_uart_enablepwr(6)
#define tiva_uart7_enablepwr()     tiva_uart_enablepwr(7)

#define tiva_uart0_disablepwr()    tiva_uart_disablepwr(0)
#define tiva_uart1_disablepwr()    tiva_uart_disablepwr(1)
#define tiva_uart2_disablepwr()    tiva_uart_disablepwr(2)
#define tiva_uart3_disablepwr()    tiva_uart_disablepwr(3)
#define tiva_uart4_disablepwr()    tiva_uart_disablepwr(4)
#define tiva_uart5_disablepwr()    tiva_uart_disablepwr(5)
#define tiva_uart6_disablepwr()    tiva_uart_disablepwr(6)
#define tiva_uart7_disablepwr()    tiva_uart_disablepwr(7)

/* SSI Power Control */

#ifdef TIVA_SYSCON_PCSSI
#  define tiva_ssi_enablepwr(p)    tiva_enablepwr(TIVA_SYSCON_PCSSI,SYSCON_PCSSI(p))
#  define tiva_ssi_disablepwr(p)   tiva_disablepwr(TIVA_SYSCON_PCSSI,SYSCON_PCSSI(p))
#else
#  define tiva_ssi_enablepwr(p)
#  define tiva_ssi_disablepwr(p)
#endif

#define tiva_ssi0_enablepwr()      tiva_ssi_enablepwr(0)
#define tiva_ssi1_enablepwr()      tiva_ssi_enablepwr(1)
#define tiva_ssi2_enablepwr()      tiva_ssi_enablepwr(2)
#define tiva_ssi3_enablepwr()      tiva_ssi_enablepwr(3)

#define tiva_ssi0_disablepwr()     tiva_ssi_disablepwr(0)
#define tiva_ssi1_disablepwr()     tiva_ssi_disablepwr(1)
#define tiva_ssi2_disablepwr()     tiva_ssi_disablepwr(2)
#define tiva_ssi3_disablepwr()     tiva_ssi_disablepwr(3)

/* I2C Power Control */

#ifdef TIVA_SYSCON_PCI2C
#  define tiva_i2c_enablepwr(p)    tiva_enablepwr(TIVA_SYSCON_PCI2C,SYSCON_PCI2C(p))
#  define tiva_i2c_disablepwr(p)   tiva_disablepwr(TIVA_SYSCON_PCI2C,SYSCON_PCI2C(p))
#else
#  define tiva_i2c_enablepwr(p)
#  define tiva_i2c_disablepwr(p)
#endif

#define tiva_i2c0_enablepwr()      tiva_i2c_enablepwr(0)
#define tiva_i2c1_enablepwr()      tiva_i2c_enablepwr(1)
#define tiva_i2c2_enablepwr()      tiva_i2c_enablepwr(2)
#define tiva_i2c3_enablepwr()      tiva_i2c_enablepwr(3)
#define tiva_i2c4_enablepwr()      tiva_i2c_enablepwr(4)
#define tiva_i2c5_enablepwr()      tiva_i2c_enablepwr(5)
#define tiva_i2c6_enablepwr()      tiva_i2c_enablepwr(6)
#define tiva_i2c7_enablepwr()      tiva_i2c_enablepwr(7)
#define tiva_i2c8_enablepwr()      tiva_i2c_enablepwr(8)
#define tiva_i2c9_enablepwr()      tiva_i2c_enablepwr(9)

#define tiva_i2c0_disablepwr()     tiva_i2c_disablepwr(0)
#define tiva_i2c1_disablepwr()     tiva_i2c_disablepwr(1)
#define tiva_i2c2_disablepwr()     tiva_i2c_disablepwr(2)
#define tiva_i2c3_disablepwr()     tiva_i2c_disablepwr(3)
#define tiva_i2c4_disablepwr()     tiva_i2c_disablepwr(4)
#define tiva_i2c5_disablepwr()     tiva_i2c_disablepwr(5)
#define tiva_i2c6_disablepwr()     tiva_i2c_disablepwr(6)
#define tiva_i2c7_disablepwr()     tiva_i2c_disablepwr(7)
#define tiva_i2c8_disablepwr()     tiva_i2c_disablepwr(8)
#define tiva_i2c9_disablepwr()     tiva_i2c_disablepwr(9)

/* USB Power Control */

#ifdef TIVA_SYSCON_PCUSB
#  define tiva_usb_enablepwr()     tiva_enablepwr(TIVA_SYSCON_PCUSB,SYSCON_PCUSB_P0)
#  define tiva_usb_disablepwr()    tiva_disablepwr(TIVA_SYSCON_PCUSB,SYSCON_PCUSB_P0)
#else
#  define tiva_usb_enablepwr()
#  define tiva_usb_disablepwr()
#endif

/* Ethernet PHY Power Control */

#ifdef TIVA_SYSCON_PCEPHY
#  define tiva_ephy_enablepwr()    tiva_enablepwr(TIVA_SYSCON_PCEPHY,SYSCON_PCEPHY_P0)
#  define tiva_ephy_disablepwr()   tiva_disablepwr(TIVA_SYSCON_PCEPHY,SYSCON_PCEPHY_P0)
#else
#  define tiva_ephy_enablepwr()
#  define tiva_ephy_disablepwr()
#endif

/* CAN RunMode Clock Gating Control */

#ifdef TIVA_SYSCON_PCCAN
#  define tiva_can_enablepwr(p)    tiva_enablepwr(TIVA_SYSCON_PCCAN,SYSCON_PCCAN(p))
#  define tiva_can_disablepwr(p)   tiva_disablepwr(TIVA_SYSCON_PCCAN,SYSCON_PCCAN(p))
#else
#  define tiva_can_enablepwr(p)
#  define tiva_can_disablepwr(p)
#endif

#define tiva_can0_enablepwr()      tiva_can_enablepwr(0)
#define tiva_can1_enablepwr()      tiva_can_enablepwr(1)

#define tiva_can0_disablepwr()     tiva_can_disablepwr(0)
#define tiva_can1_disablepwr()     tiva_can_disablepwr(1)

/* ADC Power Control */

#ifdef TIVA_SYSCON_PCADC
#  define tiva_adc_enablepwr(p)    tiva_enablepwr(TIVA_SYSCON_PCADC,SYSCON_PCADC(p))
#  define tiva_adc_disablepwr(p)   tiva_disablepwr(TIVA_SYSCON_PCADC,SYSCON_PCADC(p))
#else
#  define tiva_adc_enablepwr(p)
#  define tiva_adc_disablepwr(p)
#endif

#define tiva_adc0_enablepwr()      tiva_adc_enablepwr(0)
#define tiva_adc1_enablepwr()      tiva_adc_enablepwr(1)

#define tiva_adc0_disablepwr()     tiva_adc_disablepwr(0)
#define tiva_adc1_disablepwr()     tiva_adc_disablepwr(1)

/* ACMP Power Control */

#ifdef TIVA_SYSCON_PCACMP
#  define tiva_acmp_enablepwr()    tiva_enablepwr(TIVA_SYSCON_PCACMP,SYSCON_PCACMP_P0)
#  define tiva_acmp_disablepwr()   tiva_disablepwr(TIVA_SYSCON_PCACMP,SYSCON_PCACMP_P0)
#else
#  define tiva_acmp_enablepwr()
#  define tiva_acmp_disablepwr()
#endif

/* PWM Power Control */

#ifdef TIVA_SYSCON_PCPWM
#  define tiva_pwm_enablepwr(p)    tiva_enablepwr(TIVA_SYSCON_PCPWM,SYSCON_PCPWM(p))
#  define tiva_pwm_disablepwr(p)   tiva_disablepwr(TIVA_SYSCON_PCPWM,SYSCON_PCPWM(p))
#else
#  define tiva_pwm_enablepwr(p)
#  define tiva_pwm_disablepwr(p)
#endif

#define tiva_pwm0_enablepwr()      tiva_pwm_enablepwr(0)
#define tiva_pwm1_enablepwr()      tiva_pwm_enablepwr(1)

#define tiva_pwm0_disablepwr()     tiva_pwm_disablepwr(0)
#define tiva_pwm1_disablepwr()     tiva_pwm_disablepwr(1)

/* QE Interface Power Control */

#ifdef TIVA_SYSCON_PCQEI
#  define tiva_qei_enablepwr(p)    tiva_enablepwr(TIVA_SYSCON_PCQEI,SYSCON_PCQEI(p))
#  define tiva_qei_disablepwr(p)   tiva_disablepwr(TIVA_SYSCON_PCQEI,SYSCON_PCQEI(p))
#else
#  define tiva_qei_enablepwr(p)
#  define tiva_qei_disablepwr(p)
#endif

#define tiva_qei0_enablepwr()      tiva_qei_enablepwr(0)
#define tiva_qei1_enablepwr()      tiva_qei_enablepwr(1)

#define tiva_qei0_disablepwr()     tiva_qei_disablepwr(0)
#define tiva_qei1_disablepwr()     tiva_qei_disablepwr(1)

/* EEPROM Power Control */

#ifdef TIVA_SYSCON_PCEEPROM
#  define tiva_eeprom_enablepwr()   tiva_enablepwr(TIVA_SYSCON_PCEEPROM,SYSCON_PCEEPROM_P0)
#  define tiva_eeprom_disablepwr()  tiva_disablepwr(TIVA_SYSCON_PCEEPROM,SYSCON_PCEEPROM_P0)
#else
#  define tiva_eeprom_enablepwr()
#  define tiva_eeprom_disablepwr()
#endif

/* 32/64-Bit Wide Timer Power Control */

#ifdef TIVA_SYSCON_PCWTIMER
#  define tiva_wtm_enablepwr(p)    tiva_enablepwr(TIVA_SYSCON_PCWTIMER,SYSCON_PCWTIMER(p))
#  define tiva_wtm_disablepwr(p)   tiva_disablepwr(TIVA_SYSCON_PCWTIMER,SYSCON_PCWTIMER(p))
#else
#  define tiva_wtm_enablepwr(p)
#  define tiva_wtm_disablepwr(p)
#endif

#define tiva_wtm0_enablepwr()      tiva_wtm_enablepwr(0)
#define tiva_wtm1_enablepwr()      tiva_wtm_enablepwr(1)
#define tiva_wtm2_enablepwr()      tiva_wtm_enablepwr(2)
#define tiva_wtm3_enablepwr()      tiva_wtm_enablepwr(3)
#define tiva_wtm4_enablepwr()      tiva_wtm_enablepwr(4)
#define tiva_wtm5_enablepwr()      tiva_wtm_enablepwr(5)

#define tiva_wtm0_disablepwr()     tiva_wtm_disablepwr(0)
#define tiva_wtm1_disablepwr()     tiva_wtm_disablepwr(1)
#define tiva_wtm2_disablepwr()     tiva_wtm_disablepwr(2)
#define tiva_wtm3_disablepwr()     tiva_wtm_disablepwr(3)
#define tiva_wtm4_disablepwr()     tiva_wtm_disablepwr(4)
#define tiva_wtm5_disablepwr()     tiva_wtm_disablepwr(5)

/* CRC/Crypto Modules RunMode ClockGating Control */

#ifdef TIVA_SYSCON_PCCCM
#  define tiva_ccm_enablepwr()     tiva_enablepwr(TIVA_SYSCON_PCCCM,SYSCON_PCCCM_P0)
#  define tiva_ccm_disablepwr()    tiva_disablepwr(TIVA_SYSCON_PCCCM,SYSCON_PCCCM_P0)
#else
#  define tiva_ccm_enablepwr()
#  define tiva_ccm_disablepwr()
#endif

/* LCD Controller Power Control */

#ifdef TIVA_SYSCON_PCLCD
#  define tiva_lcd_enablepwr()     tiva_enablepwr(TIVA_SYSCON_PCLCD,SYSCON_PCLCD_P0)
#  define tiva_lcd_disablepwr()    tiva_disablepwr(TIVA_SYSCON_PCLCD,SYSCON_PCLCD_P0)
#else
#  define tiva_lcd_enablepwr()
#  define tiva_lcd_disablepwr()
#endif

/* 1-Wire Power Control */

#ifdef TIVA_SYSCON_PCOWIRE
#  define tiva_owire_enablepwr()   tiva_enablepwr(TIVA_SYSCON_PCOWIRE,SYSCON_PCOWIRE_P0)
#  define tiva_owire_disablepwr()  tiva_disablepwr(TIVA_SYSCON_PCOWIRE,SYSCON_PCOWIRE_P0)
#else
#  define tiva_owire_enablepwr()
#  define tiva_owire_disablepwr()
#endif

/* Ethernet MAC Power Control */

#ifdef TIVA_SYSCON_PCEMAC
#  define tiva_emac_enablepwr()    tiva_enablepwr(TIVA_SYSCON_PCEMAC,SYSCON_PCEMAC_P0)
#  define tiva_emac_disablepwr()   tiva_disablepwr(TIVA_SYSCON_PCEMAC,SYSCON_PCEMAC_P0)
#else
#  define tiva_emac_enablepwr()
#  define tiva_emac_disablepwr()
#endif

#endif /* __ARCH_ARM_SRC_TIVA_TIVA_ENABLEPWR_H */
