/********************************************************************************************
 * arch/arm/src/nuc1xx/chip/nuc_clk.h
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_NUC1XX_CHIP_NUC_CLK_H
#define __ARCH_ARM_SRC_NUC1XX_CHIP_NUC_CLK_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* Well-known clock frequencies *************************************************************/

#define NUC_INTHI_FREQUENCY        22118400
#define NUC_INTLO_FREQUENCY        10000

/* Register offsets *************************************************************************/

#define NUC_CLK_PWRCON_OFFSET      0x0000 /* System power down control register */
#define NUC_CLK_AHBCLK_OFFSET      0x0004 /* AHB devices clock enable control register */
#define NUC_CLK_APBCLK_OFFSET      0x0008 /* APB devices clock enable control register */
#define NUC_CLK_CLKSTATUS_OFFSET   0x000c /* Clock status monitor register */
#define NUC_CLK_CLKSEL0_OFFSET     0x0010 /* Clock source select control register 0 */
#define NUC_CLK_CLKSEL1_OFFSET     0x0014 /* Clock source select control register 1 */
#define NUC_CLK_CLKSEL2_OFFSET     0x001c /* Clock source select control register 2 */
#define NUC_CLK_CLKDIV_OFFSET      0x0018 /* Clock divider number register */
#define NUC_CLK_PLLCON_OFFSET      0x0020 /* PLL control register */
#define NUC_CLK_FRQDIV_OFFSET      0x0024 /* Frequency divider control register */

/* Register addresses ***********************************************************************/

#define NUC_CLK_PWRCON             (NUC_CLK_BASE+NUC_CLK_PWRCON_OFFSET)
#define NUC_CLK_AHBCLK             (NUC_CLK_BASE+NUC_CLK_AHBCLK_OFFSET)
#define NUC_CLK_APBCLK             (NUC_CLK_BASE+NUC_CLK_APBCLK_OFFSET)
#define NUC_CLK_CLKSTATUS          (NUC_CLK_BASE+NUC_CLK_CLKSTATUS_OFFSET)
#define NUC_CLK_CLKSEL0            (NUC_CLK_BASE+NUC_CLK_CLKSEL0_OFFSET)
#define NUC_CLK_CLKSEL1            (NUC_CLK_BASE+NUC_CLK_CLKSEL1_OFFSET)
#define NUC_CLK_CLKSEL2            (NUC_CLK_BASE+NUC_CLK_CLKSEL2_OFFSET)
#define NUC_CLK_CLKDIV             (NUC_CLK_BASE+NUC_CLK_CLKDIV_OFFSET)
#define NUC_CLK_PLLCON             (NUC_CLK_BASE+NUC_CLK_PLLCON_OFFSET)
#define NUC_CLK_FRQDIV             (NUC_CLK_BASE+NUC_CLK_FRQDIV_OFFSET)

/* Register bit-field definitions ***********************************************************/

/* System power down control register */

#define CLK_PWRCON_XTL12M_EN       (1 << 0)  /* Bit 0:  External 4~24 mhz high speed crystal enable */
#define CLK_PWRCON_XTL32K_EN       (1 << 1)  /* Bit 1:  External 32.768 khz low speed crystal enable */
#define CLK_PWRCON_OSC22M_EN       (1 << 2)  /* Bit 2:  Internal 22.1184 MHz high speed oscillator enable */
#define CLK_PWRCON_OSC10K_EN       (1 << 3)  /* Bit 3:  Internal 10KHz low speed oscillator enable */
#define CLK_PWRCON_PD_WU_DLY       (1 << 4)  /* Bit 4:  Enable the wake-up delay counter */
#define CLK_PWRCON_PD_WU_INT_EN    (1 << 5)  /* Bit 5:  Power down mode wake-up interrupt status */
#define CLK_PWRCON_PD_WU_STS       (1 << 6)  /* Bit 6:  Power down mode wake-up interrupt status */
#define CLK_PWRCON_PWR_DOWN_EN     (1 << 7)  /* Bit 7:  System power down enable bit */
#define CLK_PWRCON_PD_WAIT_CPU     (1 << 8)  /* Bit 8:  Power down entry condition */

/* AHB devices clock enable control register */

#define CLK_AHBCLK_PDMA_EN         (1 << 1)  /* Bit 1:  PDMA acontroller clock enable control */
#define CLK_AHBCLK_ISP_EN          (1 << 2)  /* Bit 2:  FLASH ISPO controller clock enable control */
#define CLK_AHBCLK_EBI_EN          (1 << 3)  /* Bit 3:  EBI controller clock enable control */

/* APB devices clock enable control register */

#define CLK_APBCLK_WDT_EN          (1 << 0)  /* Bit 0:  Watchdog time clock enable */
#define CLK_APBCLK_RTC_EN          (1 << 1)  /* Bit 1:  Real time clock clock enable */
#define CLK_APBCLK_TMR0_EN         (1 << 2)  /* Bit 2:  Timer0 clock enable */
#define CLK_APBCLK_TMR1_EN         (1 << 3)  /* Bit 3:  Timer1 clock enable */
#define CLK_APBCLK_TMR2_EN         (1 << 4)  /* Bit 4:  Timer2 clock enable */
#define CLK_APBCLK_TMR3_EN         (1 << 5)  /* Bit 5:  Timer3 clock enable */
#define CLK_APBCLK_FDIV_EN         (1 << 6)  /* Bit 6:  Frequency divider output clock enable */
#define CLK_APBCLK_I2C0_EN         (1 << 8)  /* Bit 8:  I2C0 clock enable */
#define CLK_APBCLK_I2C1_EN         (1 << 9)  /* Bit 9:  I2C1 clock enable */
#define CLK_APBCLK_SPI0_EN         (1 << 12) /* Bit 12: SPI0 clock enable */
#define CLK_APBCLK_SPI1_EN         (1 << 13) /* Bit 13: SPI1 clock enable */
#define CLK_APBCLK_SPI2_EN         (1 << 14) /* Bit 14: SPI2 clock enable */
#define CLK_APBCLK_SPI3_EN         (1 << 15) /* Bit 15: SPI3 clock enable */
#define CLK_APBCLK_UART0_EN        (1 << 16) /* Bit 16: UART0 clock enable */
#define CLK_APBCLK_UART1_EN        (1 << 17) /* Bit 17: UART1 clock enable */
#define CLK_APBCLK_UART2_EN        (1 << 18) /* Bit 18: UART2 clock enable */
#define CLK_APBCLK_PWM01_EN        (1 << 20) /* Bit 20: PWM_01 clock enable */
#define CLK_APBCLK_PWM23_EN        (1 << 21) /* Bit 21: PWM_23 clock enable */
#define CLK_APBCLK_PWM45_EN        (1 << 22) /* Bit 22: PWM_45 clock enable */
#define CLK_APBCLK_PWM67_EN        (1 << 23) /* Bit 23: PWM_67 clock enable */
#define CLK_APBCLK_USBD_EN         (1 << 27) /* Bit 27: USB 2.0 FS device controller clock enable */
#define CLK_APBCLK_ADC_EN          (1 << 28) /* Bit 28: Analog-digital-converter clock enable */
#define CLK_APBCLK_I2S_EN          (1 << 29) /* Bit 29: I2S clock enable */
#define CLK_APBCLK_ACMP_EN         (1 << 30) /* Bit 30: Analog comparator clock enable */
#define CLK_APBCLK_PS2_EN          (1 << 31) /* Bit 31: PS/2 clock enable */

/* Clock status monitor register */

#define CLK_CLKSTATUS_XTL12M_STB   (1 << 0)  /* Bit 0:  External 4~24 mhz high speed crystal
                                              *         clock source stable flag */
#define CLK_CLKSTATUS_STL32K_STB   (1 << 1)  /* Bit 1:  External 32.768 kHz low speed crystal
                                              *         clock source stable flag */
#define CLK_CLKSTATUS_PLL_STB      (1 << 2)  /* Bit 2:  Internal PLL clock source stable flag */
#define CLK_CLKSTATUS_OSC10K_STB   (1 << 3)  /* Bit 3:  Internal 10kHz low speed clock source
                                              *         stable flag */
#define CLK_CLKSTATUS_OSC22M_STB   (1 << 4)  /* Bit 4:  Internal 22.1184MHz high speed
                                              *         osciallator clock source stable flag */
#define CLK_CLKSTATUS_CLK_SW_FAIL  (1 << 7)  /* Bit 7:  Clock switching fail flag */

/* Clock source select control register 0 */

#define CLK_CLKSEL0_HCLK_S_SHIFT   (0)       /* Bits 0-2: HCLK clock source select */
#define CLK_CLKSEL0_HCLK_S_MASK    (7 << CLK_CLKSEL0_HCLK_S_SHIFT)
#  define CLK_CLKSEL0_HCLK_S_XTALHI    (0 << CLK_CLKSEL0_HCLK_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL0_HCLK_S_XTALLO    (1 << CLK_CLKSEL0_HCLK_S_SHIFT) /* Low speed XTAL clock */
#  define CLK_CLKSEL0_HCLK_S_PLL       (2 << CLK_CLKSEL0_HCLK_S_SHIFT) /* PLL clock */
#  define CLK_CLKSEL0_HCLK_S_INTLO     (3 << CLK_CLKSEL0_HCLK_S_SHIFT) /* Internal low speed clock */
#  define CLK_CLKSEL0_HCLK_S_INTHI     (7 << CLK_CLKSEL0_HCLK_S_SHIFT) /* Internal high speed clock */
#define CLK_CLKSEL0_STCLK_S_SHIFT  (3)       /* Bits 3-5: Cortex M0 Systick clock source select */
#define CLK_CLKSEL0_STCLK_S_MASK   (7 << CLK_CLKSEL0_STCLK_S_SHIFT)
#  define CLK_CLKSEL0_STCLK_S_XTALHI   (0 << CLK_CLKSEL0_STCLK_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL0_STCLK_S_XTALLO   (1 << CLK_CLKSEL0_STCLK_S_SHIFT) /* Low speed XTAL clock */
#  define CLK_CLKSEL0_STCLK_S_XTALDIV2 (2 << CLK_CLKSEL0_STCLK_S_SHIFT) /* High speed XTAL clock/2 */
#  define CLK_CLKSEL0_STCLK_S_HCLKDIV2 (3 << CLK_CLKSEL0_STCLK_S_SHIFT) /* HCLK/2 */
#  define CLK_CLKSEL0_STCLK_S_INTDIV2  (7 << CLK_CLKSEL0_STCLK_S_SHIFT) /* Internal high speed clock/2 */

/* Clock source select control register 1 */

#define CLK_CLKSEL1_WDT_S_SHIFT    (0)       /* Bits 0-1: Watchdog timer clock source select */
#define CLK_CLKSEL1_WDT_S_MASK     (3 << CLK_CLKSEL1_WDT_S_SHIFT)
#  define CLK_CLKSEL1_ADC_S_HCLKDIV    (2 << CLK_CLKSEL1_WDT_S_SHIFT) /* HCLK / 2048 */
#  define CLK_CLKSEL1_ADC_S_INTLO      (3 << CLK_CLKSEL1_WDT_S_SHIFT) /* Internal low speed clock */
#define CLK_CLKSEL1_ADC_S_SHIFT    (2)       /* Bits 2-3: ADC clock source select */
#define CLK_CLKSEL1_ADC_S_MASK     (3 << CLK_CLKSEL1_ADC_S_SHIFT)
#  define CLK_CLKSEL1_ADC_S_XTALHI     (0 << CLK_CLKSEL1_ADC_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL1_ADC_S_PLL        (1 << CLK_CLKSEL1_ADC_S_SHIFT) /* PLL */
#  define CLK_CLKSEL1_ADC_S_INTHI      (3 << CLK_CLKSEL1_ADC_S_SHIFT) /* Internal high speed clock */
#define CLK_CLKSEL1_TMR0_S_SHIFT   (8)       /* Bits 8-10: Timer0 clock source select */
#define CLK_CLKSEL1_TMR0_S_MASK    (7 << CLK_CLKSEL1_TMR0_S_SHIFT)
#  define CLK_CLKSEL1_TMR0_S_XTALHI    (0 << CLK_CLKSEL1_TMR0_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL1_TMR0_S_XTALLO    (1 << CLK_CLKSEL1_TMR0_S_SHIFT) /* Low speed XTAL clock */
#  define CLK_CLKSEL1_TMR0_S_HCLK      (2 << CLK_CLKSEL1_TMR0_S_SHIFT) /* HCLK */
#  define CLK_CLKSEL1_TMR0_S_INTHI     (7 << CLK_CLKSEL1_TMR0_S_SHIFT) /* Internal high speed clock */
#define CLK_CLKSEL1_TMR1_S_SHIFT   (12)      /* Bits 12-14: Timer1 clock source select */
#define CLK_CLKSEL1_TMR1_S_MASK    (7 << CLK_CLKSEL1_TMR1_S_SHIFT)
#  define CLK_CLKSEL1_TMR1_S_XTALHI    (0 << CLK_CLKSEL1_TMR1_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL1_TMR1_S_XTALLO    (1 << CLK_CLKSEL1_TMR1_S_SHIFT) /* Low speed XTAL clock */
#  define CLK_CLKSEL1_TMR1_S_HCLK      (2 << CLK_CLKSEL1_TMR1_S_SHIFT) /* HCLK */
#  define CLK_CLKSEL1_TMR1_S_INTHI     (7 << CLK_CLKSEL1_TMR1_S_SHIFT) /* Internal high speed clock */
#define CLK_CLKSEL1_TMR2_S_SHIFT   (16)      /* Bits 16-18: Timer2 clock source select */
#define CLK_CLKSEL1_TMR2_S_MASK    (7 << CLK_CLKSEL1_TMR2_S_SHIFT)
#  define CLK_CLKSEL1_TMR2_S_XTALHI    (0 << CLK_CLKSEL1_TMR2_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL1_TMR2_S_XTALLO    (1 << CLK_CLKSEL1_TMR2_S_SHIFT) /* Low speed XTAL clock */
#  define CLK_CLKSEL1_TMR2_S_HCLK      (2 << CLK_CLKSEL1_TMR2_S_SHIFT) /* HCLK */
#  define CLK_CLKSEL1_TMR2_S_INTHI     (7 << CLK_CLKSEL1_TMR2_S_SHIFT) /* Internal high speed clock */
#define CLK_CLKSEL1_TMR3_S_SHIFT   (20)      /* Bits 20-22: Timer3 clock source select */
#define CLK_CLKSEL1_TMR3_S_MASK    (7 << CLK_CLKSEL1_TMR3_S_SHIFT)
#  define CLK_CLKSEL1_TMR3_S_XTALHI    (0 << CLK_CLKSEL1_TMR3_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL1_TMR3_S_XTALLO    (1 << CLK_CLKSEL1_TMR3_S_SHIFT) /* Low speed XTAL clock */
#  define CLK_CLKSEL1_TMR3_S_HCLK      (2 << CLK_CLKSEL1_TMR3_S_SHIFT) /* HCLK */
#  define CLK_CLKSEL1_TMR3_S_INTHI     (7 << CLK_CLKSEL1_TMR3_S_SHIFT) /* Internal high speed clock */
#define CLK_CLKSEL1_UART_S_SHIFT   (24)      /* Bits 24-25: UART clock source select */
#define CLK_CLKSEL1_UART_S_MASK    (3 << CLK_CLKSEL1_UART_S_SHIFT)
#  define CLK_CLKSEL1_UART_S_XTALHI    (0 << CLK_CLKSEL1_UART_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL1_UART_S_PLL       (1 << CLK_CLKSEL1_UART_S_SHIFT) /* PLL */
#  define CLK_CLKSEL1_UART_S_INTHI     (3 << CLK_CLKSEL1_UART_S_SHIFT) /* Internal high speed clock */
#define CLK_CLKSEL1_PWM01_S_SHIFT  (28)      /* Bits 28-29: PWM0 and PWM1 clock source select */
#define CLK_CLKSEL1_PWM01_S_MASK   (3 << CLK_CLKSEL1_PWM01_S_SHIFT)
#  define CLK_CLKSEL1_PWM01_S_XTALHI   (0 << CLK_CLKSEL1_PWM01_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL1_PWM01_S_XTALLO   (1 << CLK_CLKSEL1_PWM01_S_SHIFT) /* Low speed XTAL clock */
#  define CLK_CLKSEL1_PWM01_S_HCLK     (2 << CLK_CLKSEL1_PWM01_S_SHIFT) /* HCLK */
#  define CLK_CLKSEL1_PWM01_S_INTHI    (3 << CLK_CLKSEL1_PWM01_S_SHIFT) /* Internal high speed clock */
#define CLK_CLKSEL1_PWM23_S_SHIFT  (30)      /* Bits 30-31: PWM2 and PWM3 clock source select */
#define CLK_CLKSEL1_PWM23_S_MASK   (3 << CLK_CLKSEL1_PWM23_S_SHIFT)
#  define CLK_CLKSEL1_PWM23_S_XTALHI   (0 << CLK_CLKSEL1_PWM23_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL1_PWM23_S_XTALLO   (1 << CLK_CLKSEL1_PWM23_S_SHIFT) /* Low speed XTAL clock */
#  define CLK_CLKSEL1_PWM23_S_HCLK     (2 << CLK_CLKSEL1_PWM23_S_SHIFT) /* HCLK */
#  define CLK_CLKSEL1_PWM23_S_INTHI    (3 << CLK_CLKSEL1_PWM23_S_SHIFT) /* Internal high speed clock */

/* Clock source select control register 2 */

#define CLK_CLKSEL2_I2S_S_SHIFT    (0)       /* Bits 0-1: I2S clock source select */
#define CLK_CLKSEL2_I2S_S_MASK     (3 << CLK_CLKSEL2_I2S_S_SHIFT)
#  define CLK_CLKSEL1_I2S_S_XTALHI     (0 << CLK_CLKSEL2_I2S_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL1_I2S_S_XTALLO     (1 << CLK_CLKSEL2_I2S_S_SHIFT) /* Low speed XTAL clock */
#  define CLK_CLKSEL1_I2S_S_HCLK       (2 << CLK_CLKSEL2_I2S_S_SHIFT) /* HCLK */
#  define CLK_CLKSEL1_I2S_S_INTHI      (3 << CLK_CLKSEL2_I2S_S_SHIFT) /* Internal high speed clock */
#define CLK_CLKSEL2_FRQDIV_S_SHIFT (2)       /* Bits 2-3: Frequency divider clock source select */
#define CLK_CLKSEL2_FRQDIV_S_MASK  (3 << CLK_CLKSEL2_FRQDIV_S_SHIFT)
#  define CLK_CLKSEL1_FRQDIV_S_XTALHI  (0 << CLK_CLKSEL2_FRQDIV_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL1_FRQDIV_S_XTALLO  (1 << CLK_CLKSEL2_FRQDIV_S_SHIFT) /* Low speed XTAL clock */
#  define CLK_CLKSEL1_FRQDIV_S_HCLK    (2 << CLK_CLKSEL2_FRQDIV_S_SHIFT) /* HCLK */
#define CLK_CLKSEL2_PWM45_S_SHIFT  (4)       /* Bits 4-5: PWM4 and PWM5 clock source select */
#define CLK_CLKSEL2_PWM45_S_MASK   (3 << CLK_CLKSEL2_PWM45_S_SHIFT)
#  define CLK_CLKSEL1_PWM45_S_XTALHI   (0 << CLK_CLKSEL2_PWM45_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL1_PWM45_S_XTALLO   (1 << CLK_CLKSEL2_PWM45_S_SHIFT) /* Low speed XTAL clock */
#  define CLK_CLKSEL1_PWM45_S_HCLK     (2 << CLK_CLKSEL2_PWM45_S_SHIFT) /* HCLK */
#  define CLK_CLKSEL1_PWM45_S_INTHI    (3 << CLK_CLKSEL2_PWM45_S_SHIFT) /* Internal high speed clock */
#define CLK_CLKSEL2_PWM67_S_SHIFT  (6)       /* Bits 6-7: PWM6 and PWM7 clock source select */
#define CLK_CLKSEL2_PWM67_S_MASK   (3 << CLK_CLKSEL2_PWM67_S_SHIFT)
#  define CLK_CLKSEL1_PWM67_S_XTALHI   (0 << CLK_CLKSEL2_PWM67_S_SHIFT) /* High speed XTAL clock */
#  define CLK_CLKSEL1_PWM67_S_XTALLO   (1 << CLK_CLKSEL2_PWM67_S_SHIFT) /* Low speed XTAL clock */
#  define CLK_CLKSEL1_PWM67_S_HCLK     (2 << CLK_CLKSEL2_PWM67_S_SHIFT) /* HCLK */
#  define CLK_CLKSEL1_PWM67_S_INTHI    (3 << CLK_CLKSEL2_PWM67_S_SHIFT) /* Internal high speed clock */

/* Clock divider number register */

#define CLK_CLKDIV_HCLK_N_SHIFT    (0) /* Bits 0-3: HCLCK divide from clock source */
#define CLK_CLKDIV_HCLK_N_MASK     (15 << CLK_CLKDIV_HCLK_N_SHIFT)
#  define CLK_CLKDIV_HCLK_N(n)     (((n)-1) << CLK_CLKDIV_HCLK_N_SHIFT) /* n=1..16 */
#define CLK_CLKDIV_USB_N_SHIFT     (4) /* Bits 4-7: USBD divide from clock source */
#define CLK_CLKDIV_USB_N_MASK      (15 << CLK_CLKDIV_USB_N_SHIFT)
#  define CLK_CLKDIV_USB_N(n)      (((n)-1) << CLK_CLKDIV_USB_N_SHIFT) /* n=1..16 */
#define CLK_CLKDIV_UART_N_SHIFT    (8) /* Bits 8-11 UART divide from clock source */
#define CLK_CLKDIV_UART_N_MASK     (15 << CLK_CLKDIV_UART_N_SHIFT)
#  define CLK_CLKDIV_UART_N(n)     (((n)-1) << CLK_CLKDIV_UART_N_SHIFT) /* n=1..16 */
#define CLK_CLKDIV_ADC_N_SHIFT     (16) /* Bits 16-23: ADC divide from clock source */
#define CLK_CLKDIV_ADC_N_MASK      (255 << CLK_CLKDIV_ADC_N_SHIFT)
#  define CLK_CLKDIV_ADC_N(n)      (((n)-1) << CLK_CLKDIV_UART_N_SHIFT) /* n=1..256 */

/* PLL control register */

#define CLK_PLLCON_FB_DV_SHIFT     (0)       /* Bits 0-8: PLL feedback divider control pins */
#define CLK_PLLCON_FB_DV_MASK      (0x1ff << CLK_PLLCON_FB_DV_SHIFT)
#  define CLK_PLLCON_FB_DV(n)      ((n) << CLK_PLLCON_FB_DV_SHIFT)
#define CLK_PLLCON_IN_DV_SHIFT     (9)       /* bits 9-13: PLL input divider control pins */
#define CLK_PLLCON_IN_DV_MASK      (0x1f << CLK_PLLCON_IN_DV_SHIFT)
#  define CLK_PLLCON_IN_DV(n)      ((n) << CLK_PLLCON_IN_DV_SHIFT)
#define CLK_PLLCON_OUT_DV_SHIFT    (14)      /* Bits 14-15: PLL output divider control pins */
#define CLK_PLLCON_OUT_DV_MASK     (3 << CLK_PLLCON_OUT_DV_SHIFT)
#  define CLK_PLLCON_OUT_DV(n)     ((n) << CLK_PLLCON_OUT_DV_SHIFT)
#define CLK_PLLCON_PD              (1 << 16) /* Bit 16: Power down mode  */
#define CLK_PLLCON_BP              (1 << 17) /* Bit 17: PLL bypass control */
#define CLK_PLLCON_OE              (1 << 18) /* Bit 18: PLL OE (FOUT enable) pin control */
#define CLK_PLLCON_PLL_SRC         (1 << 19) /* Bit 19: PLL source clock select */

/* Frequency divider control register */

#define CLK_FRQDIV_FSEL_SHIFT      (0)       /* Bits 0-3: Divider output frequency selection bits */
#define CLK_FRQDIV_FSEL_MASK       (15 << CLK_FRQDIV_FSEL_SHIFT)
#  define CLK_FRQDIV_FSEL(n)       ((n) << CLK_FRQDIV_FSEL_SHIFT) /* fout = fin / (2^(n+1)) */
#define CLK_FRQDIV_DIVIDER_EN      (1 << 4)  /* Bit 4: Frequency divider enable bit */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_NUC1XX_CHIP_NUC_CLK_H */
