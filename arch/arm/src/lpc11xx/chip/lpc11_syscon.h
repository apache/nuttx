/********************************************************************************************
 * arch/arm/src/lpc11xx/chip/lpc11_syscon.h
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC11XX_CHIP_LPC11_SYSCON_H
#define __ARCH_ARM_SRC_LPC11XX_CHIP_LPC11_SYSCON_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc11_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register offsets *************************************************************************/

#define LPC11_SYSCON_SYSMEMREMAP_OFFSET       0x0000 /* System memory remap */
#define LPC11_SYSCON_PRESETCTRL_OFFSET        0x0004 /* Pefipheral reset control */
#define LPC11_SYSCON_SYSPLLCTRL_OFFSET        0x0008 /* System PLL control */
#define LPC11_SYSCON_SYSPLLSTAT_OFFSET        0x000C /* System PLL status */
                                                     /* 0x010 - 0x01c: Reserved */

#define LPC11_SYSCON_SYSOSCCTRL_OFFSET        0x0020 /* System oscillator control */
#define LPC11_SYSCON_WDTOSCCTRL_OFFSET        0x0024 /* Watchdog oscillator control */
#define LPC11_SYSCON_IRCCTRL_OFFSET           0x0028 /* IRC control */
                                                     /* 0x02c: Reserved */

#define LPC11_SYSCON_SYSRSTSTAT_OFFSET        0x0030 /* System reset status register */
                                                     /* 0x034 - 0x03c: Reserved */

#define LPC11_SYSCON_SYSPLLCLKSEL_OFFSET      0x0040 /* System PLL clock source select */
#define LPC11_SYSCON_SYSPLLCLKUEN_OFFSET      0x0044 /* System PLL clock source update enable */
                                                     /* 0x048 - 0x06c: Reserved */

#define LPC11_SYSCON_MAINCLKSEL_OFFSET        0x0070 /* Main clock source select */
#define LPC11_SYSCON_MAINCLKUEN_OFFSET        0x0074 /* Main clock source update enable */
#define LPC11_SYSCON_SYSAHBCLKDIV_OFFSET      0x0078 /* System AHB clock divider */
                                                     /* 0x07c: Reserved */

#define LPC11_SYSCON_SYSAHBCLKCTRL_OFFSET     0x0080 /* System AHB clock control */
                                                     /* 0x084 - 0x090: Reserved */

#define LPC11_SYSCON_SSP0CLKDIV_OFFSET        0x0094 /* SPI0 clock divider */
#define LPC11_SYSCON_UARTCLKDIV_OFFSET        0x0098 /* UART clock divider */
#define LPC11_SYSCON_SSP1CLKDIV_OFFSET        0x009c /* SPI1 clock divider */
                                                     /* 0x0a0 - 0x0cc: Reserved */

#define LPC11_SYSCON_WDTCLKSEL_OFFSET         0x00d0 /* WDT clock source select */
#define LPC11_SYSCON_WDTCLKUEN_OFFSET         0x00d4 /* WDT clock source update enable */
#define LPC11_SYSCON_WDTCLKDIV_OFFSET         0x00d8 /* WDT clock divider */
                                                     /* 0x0dc: Reserved */

#define LPC11_SYSCON_CLKOUTCLKSEL_OFFSET      0x00e0 /* CLKOUT clock source select */
#define LPC11_SYSCON_CLKOUTUEN_OFFSET         0x00e4 /* CLKOUT clock source update enable */
#define LPC11_SYSCON_CLKOUTCLKDIV_OFFSET      0x00e8 /* CLKOUT clock divider */
                                                     /* 0x0ec - 0x0fc: Reserved */

#define LPC11_SYSCON_PIOPORCAP0_OFFSET        0x0100 /* POR captured PIO status 0 */
#define LPC11_SYSCON_PIOPORCAP1_OFFSET        0x0104 /* POR captured PIO status 1 */
                                                     /* 0x108 - 0x14c: Reserved */

#define LPC11_SYSCON_BODCTRL_OFFSET           0x0150 /* BOD control */
#define LPC11_SYSCON_SYSTCKCAL_OFFSET         0x0154 /* System tick counter calibration */
                                                     /* 0x158 - 0x16c: Reserved */

#define LPC11_SYSCON_IRQLATENCY_OFFSET        0x0170 /* IRQ delay */
#define LPC11_SYSCON_NMISRC_OFFSET            0x0174 /* NMI source selection */
                                                     /* 0x178 - 0x1fc: Reserved */

#define LPC11_SYSCON_STARTAPRP0_OFFSET        0x0200 /* Start logic edge control register 0 */
#define LPC11_SYSCON_STARTERP0_OFFSET         0x0204 /* Start logic signal enable register 0 */
#define LPC11_SYSCON_STARTRSRP0CLR_OFFSET     0x0208 /* Start logic reset register 0 */
#define LPC11_SYSCON_STARTSRP0_OFFSET         0x020c /* Start logic status register 0 */
                                                     /* 0x210 - 0x22c: Reserved */

#define LPC11_SYSCON_PDSLEEPCFG_OFFSET        0x0230 /* Power-down states in Deep-sleep mode */
#define LPC11_SYSCON_PDAWAKECFG_OFFSET        0x0234 /* Power-down states after wake-up from Deep-sleep mode */
#define LPC11_SYSCON_PDRUNCFG_OFFSET          0x0238 /* Power-down configuration register */
                                                     /* 0x023c - 0x3f0: Reserved */
#define LPC11_SYSCON_DEVICE_ID_OFFSET         0x03f4 /* Device ID register 0 for parts LPC1100, LPC1100C, LPC1100L */


/* Register addresses ***********************************************************************/

#define LPC11_SYSCON_SYSMEMREMAP              (LPC11_SYSCON_BASE + LPC11_SYSCON_SYSMEMREMAP_OFFSET)

#define LPC11_SYSCON_PRESETCTRL               (LPC11_SYSCON_BASE + LPC11_SYSCON_PRESETCTRL_OFFSET)
#define LPC11_SYSCON_SYSPLLCTRL               (LPC11_SYSCON_BASE + LPC11_SYSCON_SYSPLLCTRL_OFFSET)
#define LPC11_SYSCON_SYSPLLSTAT               (LPC11_SYSCON_BASE + LPC11_SYSCON_SYSPLLSTAT_OFFSET)

#define LPC11_SYSCON_SYSOSCCTRL               (LPC11_SYSCON_BASE + LPC11_SYSCON_SYSOSCCTRL_OFFSET)
#define LPC11_SYSCON_WDTOSCCTRL               (LPC11_SYSCON_BASE + LPC11_SYSCON_WDTOSCCTRL_OFFSET)
#define LPC11_SYSCON_IRCCTRL                  (LPC11_SYSCON_BASE + LPC11_SYSCON_IRCCTRL_OFFSET)

#define LPC11_SYSCON_SYSRSTSTAT               (LPC11_SYSCON_BASE + LPC11_SYSCON_SYSRSTSTAT_OFFSET)

#define LPC11_SYSCON_SYSPLLCLKSEL             (LPC11_SYSCON_BASE + LPC11_SYSCON_SYSPLLCLKSEL_OFFSET)
#define LPC11_SYSCON_SYSPLLCLKUEN             (LPC11_SYSCON_BASE + LPC11_SYSCON_SYSPLLCLKUEN_OFFSET)

#define LPC11_SYSCON_MAINCLKSEL               (LPC11_SYSCON_BASE + LPC11_SYSCON_MAINCLKSEL_OFFSET)
#define LPC11_SYSCON_MAINCLKUEN               (LPC11_SYSCON_BASE + LPC11_SYSCON_MAINCLKUEN_OFFSET)

#define LPC11_SYSCON_SYSAHBCLKDIV             (LPC11_SYSCON_BASE + LPC11_SYSCON_SYSAHBCLKDIV_OFFSET)
#define LPC11_SYSCON_SYSAHBCLKCTRL            (LPC11_SYSCON_BASE + LPC11_SYSCON_SYSAHBCLKCTRL_OFFSET)

#define LPC11_SYSCON_SSP0CLKDIV               (LPC11_SYSCON_BASE + LPC11_SYSCON_SSP0CLKDIV_OFFSET)
#define LPC11_SYSCON_UARTCLKDIV               (LPC11_SYSCON_BASE + LPC11_SYSCON_UARTCLKDIV_OFFSET)
#define LPC11_SYSCON_SSP1CLKDIV               (LPC11_SYSCON_BASE + LPC11_SYSCON_SSP1CLKDIV_OFFSET)

#define LPC11_SYSCON_WDTCLKSEL                (LPC11_SYSCON_BASE + LPC11_SYSCON_WDTCLKSEL_OFFSET)
#define LPC11_SYSCON_WDTCLKUEN                (LPC11_SYSCON_BASE + LPC11_SYSCON_WDTCLKUEN_OFFSET)
#define LPC11_SYSCON_WDTCLKDIV                (LPC11_SYSCON_BASE + LPC11_SYSCON_WDTCLKDIV_OFFSET)

#define LPC11_SYSCON_CLKOUTCLKSEL             (LPC11_SYSCON_BASE + LPC11_SYSCON_CLKOUTCLKSEL_OFFSET)
#define LPC11_SYSCON_CLKOUTUEN                (LPC11_SYSCON_BASE + LPC11_SYSCON_CLKOUTUEN_OFFSET_OFFSET)
#define LPC11_SYSCON_CLKOUTCLKDIV             (LPC11_SYSCON_BASE + LPC11_SYSCON_CLKOUTCLKDIV_OFFSET)

#define LPC11_SYSCON_PIOPORCAP0               (LPC11_SYSCON_BASE + LPC11_SYSCON_PIOPORCAP0_OFFSET)
#define LPC11_SYSCON_PIOPORCAP1               (LPC11_SYSCON_BASE + LPC11_SYSCON_PIOPORCAP1_OFFSET)

#define LPC11_SYSCON_BODCTRL                  (LPC11_SYSCON_BASE + LPC11_SYSCON_BODCTRL_OFFSET)
#define LPC11_SYSCON_SYSTCKCAL                (LPC11_SYSCON_BASE + LPC11_SYSCON_SYSTCKCAL_OFFSET)

#define LPC11_SYSCON_IRQLATENCY               (LPC11_SYSCON_BASE + LPC11_SYSCON_IRQLATENCY_OFFSET)
#define LPC11_SYSCON_NMISRC                   (LPC11_SYSCON_BASE + LPC11_SYSCON_NMISRC_OFFSET)

#define LPC11_SYSCON_STARTAPRP0               (LPC11_SYSCON_BASE + LPC11_SYSCON_STARTAPRP0_OFFSET)
#define LPC11_SYSCON_STARTERP0                (LPC11_SYSCON_BASE + LPC11_SYSCON_STARTERP0_OFFSET)
#define LPC11_SYSCON_STARTRSRP0CLR            (LPC11_SYSCON_BASE + LPC11_SYSCON_STARTRSRP0CLR_OFFSET)
#define LPC11_SYSCON_STARTSRP0                (LPC11_SYSCON_BASE + LPC11_SYSCON_STARTSRP0_OFFSET)

#define LPC11_SYSCON_PDSLEEPCFG               (LPC11_SYSCON_BASE + LPC11_SYSCON_PDSLEEPCFG_OFFSET)
#define LPC11_SYSCON_PDAWAKECFG               (LPC11_SYSCON_BASE + LPC11_SYSCON_PDAWAKECFG_OFFSET)
#define LPC11_SYSCON_PDRUNCFG                 (LPC11_SYSCON_BASE + LPC11_SYSCON_PDRUNCFG_OFFSET)

#define LPC11_SYSCON_DEVICE_ID                (LPC11_SYSCON_BASE + LPC11_SYSCON_DEVICE_ID_OFFSET)

/* Register bit definitions *****************************************************************/

#define SYSCON_SYSMEMREMAP_MAP_SHIFT          (0)       /* Bits 0-1: System memory remap register */
#define SYSCON_SYSMEMREMAP_MAP_MASK           (3 << SYSCON_SYSMEMREMAP_MAP_SHIFT)
#  define SYSCON_SYSMEMREMAP_MAP_BOOTLOADER   (0 << SYSCON_SYSMEMREMAP_MAP_SHIFT) /* Interrupt vectors are re-mapped to Boot ROM */
#  define SYSCON_SYSMEMREMAP_MAP_RAM          (1 << SYSCON_SYSMEMREMAP_MAP_SHIFT) /* Interrupt vectors are re-mapped to Static RAM */
#  define SYSCON_SYSMEMREMAP_MAP_FLASH        (2 << SYSCON_SYSMEMREMAP_MAP_SHIFT) /* Interrupt vectors are keeped in flash */
                                                                                  /* Bits 2-31:  Reserved */

#define SYSCON_PRESETCTRL_SSP0_RST_N          (1 << 0)  /* SPI0 reset control */
#define SYSCON_PRESETCTRL_I2C_RST_N           (1 << 1)  /* I2C reset control */
#define SYSCON_PRESETCTRL_SSP1_RST_N          (1 << 2)  /* SPI1 reset control */
#define SYSCON_PRESETCTRL_CAN_RST_N           (1 << 3)  /* C_CAN reset control */
                                                        /* Bits 4-31:  Reserved */

#define SYSCON_SYSPLLCTRL_MSEL_SHIFT          (0)       /* Bits 0-4: Feedback divider value. */
#define SYSCON_SYSPLLCTRL_MSEL_MASK           (0x1f << SYSCON_SYSPLLCTRL_MSEL_SHIFT)
#  define SYSCON_SYSPLLCTRL_MSEL_DIV(n)       ((n-1) << SYSCON_SYSPLLCTRL_MSEL_SHIFT) /* n=1,2,3,..32 */
#define SYSCON_SYSPLLCTRL_PSEL_SHIFT          (5)       /* Bits 5-6: Post divider ratio P. The division ratio is 2 x P */
#define SYSCON_SYSPLLCTRL_PSEL_MASK           (3 << SYSCON_SYSPLLCTRL_PSEL_SHIFT)
#  define SYSCON_SYSPLLCTRL_PSEL_DIV1         (0 << SYSCON_SYSPLLCTRL_PSEL_SHIFT)
#  define SYSCON_SYSPLLCTRL_PSEL_DIV2         (1 << SYSCON_SYSPLLCTRL_PSEL_SHIFT)
#  define SYSCON_SYSPLLCTRL_PSEL_DIV4         (2 << SYSCON_SYSPLLCTRL_PSEL_SHIFT)
#  define SYSCON_SYSPLLCTRL_PSEL_DIV8         (3 << SYSCON_SYSPLLCTRL_PSEL_SHIFT)
                                                                                  /* Bits 7-31:  Reserved */

#define SYSCON_SYSPLLSTAT_LOCK                (1 << 0)  /* PLL lock status. 0 = PLL not locked, 1 = PLL locked */
                                                        /* Bits 1-31: Reserved */

#define SYSCON_SYSOSCCTRL_BYPASS              (1 << 0)  /* Bypass system oscillator */
#define SYSCON_SYSOSCCTRL_FREQRANGE           (1 << 1)  /* Determines freq. range for low-power oscillator */
                                                        /* Bits 2-31: Reserved */
#define SYSCON_WDTOSCCTRL_DIVSEL_SHIFT        (0)       /* Bits 0-4: Select divider for Fclkana. wdt_osc_clk = Fclkana/(2x(1+DIVSEL)) */
#define SYSCON_WDTOSCCTRL_DIVSEL_MASK         (0x1f << SYSCON_WDTOSCCTRL_DIVSEL_SHIFT)
#  define SYSCON_WDTOSCCTRL_DIVSEL(n)         (((n-2)/2) << SYSCON_WDTOSCCTRL_DIVSEL_SHIFT) /* n = 2,4,8,..64 */
#define SYSCON_WDTOSCCTRL_FREQSEL_SHIFT       (5)       /* Bits 5-8: Select watchdog oscillator analog output frequency */
#define SYSCON_WDTOSCCTRL_FREQSEL_MASK        (15 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)
#  define SYSCON_WDTOSCCTRL_FREQSEL_0p6Mhz    (1 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 0.6 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_1p05Mhz   (2 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 1.05 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_1p4Mhz    (3 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 1.4 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_1p75Mhz   (4 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 1.75 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p1Mhz    (5 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 2.1 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p4Mhz    (6 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 2.4 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_2p7Mhz    (7 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 2.7 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_3Mhz      (8 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 3.0 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_3p25Mhz   (9 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 3.25 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_3p5Mhz    (10 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 3.5 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_3p75Mhz   (11 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 3.75 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_4Mhz      (12 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 4 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_4p2Mhz    (13 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 4.2 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_4p4Mhz    (14 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 4.4 MHz */
#  define SYSCON_WDTOSCCTRL_FREQSEL_4p6Mhz    (15 << SYSCON_WDTOSCCTRL_FREQSEL_SHIFT) /* Select watchdog osc analog freq 4.6 MHz */
                                                                                      /* Bits 9-31: Reserved */

#define SYSCON_IRCCTRL_TRIM_MASK              (0xff)    /* Bits 0-7: Trim value used to adjust on-chip 12 MHz oscillator */
                                                        /* Bits 8-31: Reserved */

#define SYSCON_SYSRSTSTAT_POR                 (1 << 0)  /* POR reset status */
#define SYSCON_SYSRSTSTAT_EXTRST              (1 << 1)  /* Status of the external /RESET pin */
#define SYSCON_SYSRSTSTAT_WDT                 (1 << 2)  /* Status of the Watchdog reset */
#define SYSCON_SYSRSTSTAT_BOD                 (1 << 3)  /* Status of Brown-out detect reset */
#define SYSCON_SYSRSTSTAT_SYSRST              (1 << 4)  /* Status of the software system reset */
                                                        /* Bits 5-31: Reserved */

#define SYSCON_SYSPLLCLKSEL_SHIFT             (0) /* Bits 0-1: System PLL clock source */
#define SYSCON_SYSPLLCLKSEL_MASK              (3 << SYSCON_SYSPLLCLKSEL_SHIFT)
#  define SYSCON_SYSPLLCLKSEL_IRCOSC          (0 << SYSCON_SYSPLLCLKSEL_SHIFT)
#  define SYSCON_SYSPLLCLKSEL_SYSOSC          (1 << SYSCON_SYSPLLCLKSEL_SHIFT)
                                                                               /* Bits 2-31: Reserved */

#define SYSCON_SYSPLLCLKUEN_ENA               (1 << 0)  /* Bit 0: Enable system PLL clock source update */
                                                        /* Bits 1-31: Reserved */

#define SYSCON_MAINCLKSEL_SHIFT               (0)       /* Bits 0-1: Clock source for main clock */
#define SYSCON_MAINCLKSEL_MASK                (3 << SYSCON_MAINCLKSEL_SHIFT)
#  define SYSCON_MAINCLKSEL_IRCOSC            (0 << SYSCON_MAINCLKSEL_SHIFT) /* IRC oscillator */
#  define SYSCON_MAINCLKSEL_PLLOSC            (1 << SYSCON_MAINCLKSEL_SHIFT) /* Input clock to system PLL */
#  define SYSCON_MAINCLKSEL_WDTOSC            (2 << SYSCON_MAINCLKSEL_SHIFT) /* WDT oscillator */
#  define SYSCON_MAINCLKSEL_SYSPLLCLKOUT      (3 << SYSCON_MAINCLKSEL_SHIFT) /* System PLL clock out */
                                                                             /* Bits 2-31: Reserved */

#define SYSCON_MAINCLKUEN_ENA                 (1 << 0)  /* Bit 0: Enable main clock source update */
                                                        /* Bits 1-31: Reserved */

#define SYSCON_SYSAHBCLKDIV_SHIFT             (0)       /* Bits 0-7: 0=System clock disabled, 1=Divide by 1 ... 255 = Divide by 255 */
#define SYSCON_SYSAHBCLKDIV_MASK              (0xff << SYSCON_SYSAHBCLKDIV_SHIFT)
                                                        /* Bits 8-31: Reserved */
//#  define SYSCON_CCLKCFG_DIV(n)       ((n-1) << SYSCON_CCLKCFG_SHIFT) /* n=2,3,..255 */

#define SYSCON_SYSAHBCLKCTRL_SYS              (1 << 0)  /* Bit 0:  Enables clock for AHB to APB bridge */
#define SYSCON_SYSAHBCLKCTRL_ROM              (1 << 1)  /* Bit 1:  Enables clock for ROM */
#define SYSCON_SYSAHBCLKCTRL_RAM              (1 << 2)  /* Bit 2:  Enables clock for RAM */
#define SYSCON_SYSAHBCLKCTRL_FLASHREG         (1 << 3)  /* Bit 3:  Enables clock for flash register interface */
#define SYSCON_SYSAHBCLKCTRL_FLASHARRAY       (1 << 4)  /* Bit 4:  Enables clock for flash array access */
#define SYSCON_SYSAHBCLKCTRL_I2C              (1 << 5)  /* Bit 5:  Enables clock for I2C */
#define SYSCON_SYSAHBCLKCTRL_GPIO             (1 << 6)  /* Bit 6:  Enables clock for GPIO */
#define SYSCON_SYSAHBCLKCTRL_CT16B0           (1 << 7)  /* Bit 7:  Enables clock for 16-bit counter/timer 0 */
#define SYSCON_SYSAHBCLKCTRL_CT16B1           (1 << 8)  /* Bit 8:  Enables clock for 16-bit counter/timer 1 */
#define SYSCON_SYSAHBCLKCTRL_CT32B0           (1 << 9)  /* Bit 9:  Enables clock for 32-bit counter/timer 0 */
#define SYSCON_SYSAHBCLKCTRL_CT32B1           (1 << 10) /* Bit 10: Enables clock for 32-bit counter/timer 1 */
#define SYSCON_SYSAHBCLKCTRL_SSP0             (1 << 11) /* Bit 11: Enables clock for SPI0 */
#define SYSCON_SYSAHBCLKCTRL_UART             (1 << 12) /* Bit 12: Enables clock for UART */
#define SYSCON_SYSAHBCLKCTRL_ADC              (1 << 13) /* Bit 13: Enables clock for ADC */
                                                        /* Bit 14: Reserved */
#define SYSCON_SYSAHBCLKCTRL_WDT              (1 << 15) /* Bit 15: Enables clock for WDT */
#define SYSCON_SYSAHBCLKCTRL_IOCON            (1 << 16) /* Bit 16: Enables clock for I/O configuration block */
#define SYSCON_SYSAHBCLKCTRL_CAN              (1 << 17) /* Bit 17: Enables clock for C_CAN */
#define SYSCON_SYSAHBCLKCTRL_SSP1             (1 << 18) /* Bit 18: Enables clock for SPI1 */
                                                        /* Bits 19-31: Reserved */

#define SYSCON_SSP0CLKDIV_MASK                (0xff)    /* Bits 0-7: 0=Disable SPI0_PCLK, 1=Divide by 1 ... 255 = Divide by 255 */
                                                        /* Bits 8-31: Reserved */

#define SYSCON_UARTCLKDIV_MASK                (0xff)    /* Bits 0-7: 0=Disable UART_PCLK, 1=Divide by 1 ... 255 = Divide by 255 */
                                                        /* Bits 8-31: Reserved */

#define SYSCON_SSP1CLKDIV_MASK                (0xff)    /* Bits 0-7: 0=Disable SPI1_PCLK, 1=Divide by 1 ... 255 = Divide by 255 */
                                                        /* Bits 8-31: Reserved */

#define SYSCON_WDTCLKSEL_SHIFT                (0)       /* Bits 0-1: WDT clock source */
#define SYSCON_WDTCLKSEL_MASK                 (3 << SYSCON_WDTCLKSEL_SHIFT)
#  define SYSCON_WDTCLKSEL_IRCOSC             (0 << SYSCON_WDTCLKSEL_SHIFT) /* IRC oscillator */
#  define SYSCON_WDTCLKSEL_MAINCLK            (1 << SYSCON_WDTCLKSEL_SHIFT) /* Main clock */
#  define SYSCON_WDTCLKSEL_WDTOSC             (2 << SYSCON_WDTCLKSEL_SHIFT) /* Watchdog oscillator */
                                                                            /* Bits 2-31: reserved */

#define SYSCON_WDTCLKUEN_ENA                  (1 << 0)  /* Bit 0: Enable WDT clock source update */
                                                        /* Bits 1-31: Reserved */

#define SYSCON_WDTCLKDIV_MASK                 (0xff)    /* Bits 0-7: 0=Disable WDCLK, 1=Divide by 1 ... 255 = Divide by 255 */
                                                        /* Bits 8-31: Reserved */

#define SYSCON_CLKOUTCLKSEL_SHIFT             (0) /* Bits 0-1: CLKOUT clock source */
#define SYSCON_CLKOUTCLKSEL_MASK              (3 << SYSCON_CLKOUTCLKSEL_SHIFT)
#  define SYSCON_CLKOUTCLKSEL_IRCOSC          (0 << SYSCON_CLKOUTCLKSEL_SHIFT) /* IRC oscillator */
#  define SYSCON_CLKOUTCLKSEL_SYSOSC          (1 << SYSCON_CLKOUTCLKSEL_SHIFT) /* System oscillator */
#  define SYSCON_CLKOUTCLKSEL_WDTOSC          (2 << SYSCON_CLKOUTCLKSEL_SHIFT) /* Watchdog oscillator */
#  define SYSCON_CLKOUTCLKSEL_MAINCLK         (3 << SYSCON_CLKOUTCLKSEL_SHIFT) /* Main clock */
                                                                               /* Bits 2-31: Reserved */

#define SYSCON_CLKOUTUEN_ENA                  (1 << 0)  /* Bit 0: Enable CLKOUT clock source update */
                                                        /* Bits 1-31: Reserved */

#define SYSCON_CLKOUTCLKDIV_MASK              (0xff)    /* Bits 0-7: 0=Disable CLKOUT, 1=Divide by 1 ... 255 = Divide by 255 */
                                                        /* Bits 1-31: Reserved */

#define SYSCON_PIOPORCAP0_CAPPIO0_SHIFT       (0)       /* Bits 0-11: Raw reset stats input PIO0_n: PIO0_11 to PIO0_0 */
#define SYSCON_PIOPORCAP0_CAPPIO0_MASK        (0xfff << SYSCON_PIOPORCAP0_CAPPIO0_SHIFT)
#  define SYSCON_PIOPORCAP0_CAPPIO0_BIT(n)    ((1 << n) << SYSCON_PIOPORCAP0_CAPPIO0_SHIFT) /* n = 0 to 11 */

#define SYSCON_PIOPORCAP0_CAPPIO1_SHIFT       (12)      /* Bits 12-23: Raw reset stats input PIO0_n: PIO1_11 to PIO1_0 */
#define SYSCON_PIOPORCAP0_CAPPIO1_MASK        (0xfff << SYSCON_PIOPORCAP0_CAPPIO1_SHIFT)
#  define SYSCON_PIOPORCAP0_CAPPIO1_BIT(n)    ((1 << n) << SYSCON_PIOPORCAP0_CAPPIO1_SHIFT) /* n = 0 to 11 */

#define SYSCON_PIOPORCAP0_CAPPIO2_SHIFT       (24)      /* Bits 24-31: Raw reset stats input PIO0_n: PIO2_11 to PIO2_0 */
#define SYSCON_PIOPORCAP0_CAPPIO2_MASK        (0xfff << SYSCON_PIOPORCAP0_CAPPIO2_SHIFT)
#  define SYSCON_PIOPORCAP0_CAPPIO2_BIT(n)    ((1 << n) << SYSCON_PIOPORCAP0_CAPPIO2_SHIFT) /* n = 0 to 11 */


#define SYSCON_PIOPORCAP1_CAPPIO2_8           (1 << 0)  /* Bit 0: Raw reset status input PIO2_8 */
#define SYSCON_PIOPORCAP1_CAPPIO2_9           (1 << 1)  /* Bit 1: Raw reset status input PIO2_9 */
#define SYSCON_PIOPORCAP1_CAPPIO2_10          (1 << 2)  /* Bit 2: Raw reset status input PIO2_10 */
#define SYSCON_PIOPORCAP1_CAPPIO2_11          (1 << 3)  /* Bit 3: Raw reset status input PIO2_11 */
#define SYSCON_PIOPORCAP1_CAPPIO3_0           (1 << 4)  /* Bit 4: Raw reset status input PIO3_0 */
#define SYSCON_PIOPORCAP1_CAPPIO3_1           (1 << 5)  /* Bit 5: Raw reset status input PIO3_1 */
#define SYSCON_PIOPORCAP1_CAPPIO3_2           (1 << 6)  /* Bit 6: Raw reset status input PIO3_2 */
#define SYSCON_PIOPORCAP1_CAPPIO3_3           (1 << 7)  /* Bit 7: Raw reset status input PIO3_3 */
#define SYSCON_PIOPORCAP1_CAPPIO3_4           (1 << 8)  /* Bit 8: Raw reset status input PIO3_4 */
#define SYSCON_PIOPORCAP1_CAPPIO3_5           (1 << 9)  /* Bit 9: Raw reset status input PIO3_5 */
                                                        /* Bits 10-31: Reserved */

#define SYSCON_BODCTRL_BODRSTLEV_SHIFT        (0)       /* Bits 0-1: BOD reset level */
#define SYSCON_BODCTRL_BODRSTLEV_MASK         (3 << SYSCON_BODCTRL_BODRSTLEV_SHIFT)
#  define SYSCON_BODCTRL_BODRSTLEV_LEVEL0     (0 << SYSCON_BODCTRL_BODRSTLEV_SHIFT) /* Level 0: assert 1.46V, de-assert 1.63V */
#  define SYSCON_BODCTRL_BODRSTLEV_LEVEL1     (1 << SYSCON_BODCTRL_BODRSTLEV_SHIFT) /* Level 1: assert 2.06V, de-assert 2.15V */
#  define SYSCON_BODCTRL_BODRSTLEV_LEVEL2     (2 << SYSCON_BODCTRL_BODRSTLEV_SHIFT) /* Level 2: assert 2.35V, de-assert 2.43V */
#  define SYSCON_BODCTRL_BODRSTLEV_LEVEL3     (3 << SYSCON_BODCTRL_BODRSTLEV_SHIFT) /* Level 3: assert 2.63V, de-assert 2.71V */
#define SYSCON_BODCTRL_BODINTVAL_SHIFT        (2)       /* Bits 2-3: BOD interrupt level */
#define SYSCON_BODCTRL_BODINTVAL_MASK         (3 << SYSCON_BODCTRL_BODRSTLEV_BODINTVAL_SHIFT)
#  define SYSCON_BODCTRL_BODINTVAL_LEVEL0     (0 << SYSCON_BODCTRL_BODINTVAL_SHIFT) /* Level 0: Reserved */
#  define SYSCON_BODCTRL_BODINTVAL_LEVEL1     (1 << SYSCON_BODCTRL_BODINTVAL_SHIFT) /* Level 1: int. assert 2.22V,de-a. 2.35V */
#  define SYSCON_BODCTRL_BODINTVAL_LEVEL2     (2 << SYSCON_BODCTRL_BODINTVAL_SHIFT) /* Level 2: int. assert 2.52V,de-a. 2.66V */
#  define SYSCON_BODCTRL_BODINTVAL_LEVEL3     (3 << SYSCON_BODCTRL_BODINTVAL_SHIFT) /* Level 3: int. assert 2.80V,de-a. 2.90V */
#define SYSCON_BODCTRL_BODRSTENA              (1 << 4)  /* BOD reset enable */
                                                        /* Bits 5-31: Reserved */

#define SYSCON_SYSTCKCAL_CAL                  0x3ffffff /* Bits 0-25: System tick timer calibration value */
                                                        /* Bits 26-31: Reserved */

#define SYSCON_IRQLATENCY_LATENCY_MASK        (0xff)    /* Bits 0-7: 8-bit latency value */
                                                        /* Bits 8-31: Reserved */

#define SYSCON_NMISRC_IRQNO_SHIFT             (0)       /* Bits 0-4: The IRQ number of interrupt that acts as NMI if bit 31 is 1 */
#define SYSCON_NMISRC_IRQNO_MASK              (31 << SYSCON_NMISRC_IRQNO_SHIFT)
                                                        /* Bits 5-30: Reserved */
#define SYSCON_NMISRC_NMIEN                   (1 << 31) /* Write 1 to this bit to enable NMI source selected by bits 4:0 */

#define SYSCON_STARTAPRP0_APRPIO0_SHIFT       (0)       /* Bits 0-11: Edge select for start logic input PIO0_[0-11], 0=fall/1=rise */
#define SYSCON_STARTAPRP0_APRPIO0_MASK        (0xfff << SYSCON_STARTAPRP0_APRPIO0_SHIFT)
#  define SYSCON_STARTAPRP0_APRPIO0_BIT(n)    ((1 << n) << SYSCON_STARTAPRP0_APRPIO0_SHIFT) /* n = 0 to 11 */
#define SYSCON_STARTAPRP0_APRPIO1_0           (1 << 12) /* Bit 12: Edge select start logic input PIO1_0, 0=falling/1=rising */
                                                        /* Bits 13-31: Reserved */

#define SYSCON_STARTERP0_ERPIO0_SHIFT         (0)       /* Bits 0-11: Enable start signal for start logic input PIO0[0-11] */
#define SYSCON_STARTERP0_ERPIO0_MASK          (0xfff << SYSCON_STARTERP0_ERPIO0_SHIFT)
#  define SYSCON_STARTERP0_ERPIO0_BIT(n)      ((1 << n) << SYSCON_STARTERP0_ERPIO0_SHIFT) /* n = 0 to 11 */
#define SYSCON_STARTERP0_ERPIO1_0             (1 << 12) /* Bit 12: Enable start signal for start logic input PIO1_0 */
                                                        /* Bits 13-31: Reserved */

#define SYSCON_STARTRSRP0CLR_RSRPIO0_SHIFT    (0)       /* Bits 0-11: Start logic reset register 0 */
#define SYSCON_STARTRSRP0CLR_RSRPIO0_MASK     (0xfff << SYSCON_STARTRSRP0CLR_RSRPIO0_SHIFT)
#  define SYSCON_STARTRSRP0CLR_RSRPIO0_BIT(n) ((1 << n) << SYSCON_STARTRSRP0CLR_RSRPIO0_SHIFT) /* n = 0 to 11 */
#define SYSCON_STARTRSRP0CLR_RSRPIO1_0        (1 << 12) /* Bit 12: Start signal reset for start logic input PIO1_0 */
                                                        /* Bits 13-31: Reserved */

#define SYSCON_STARTSRP0_SRPIO0_SHIFT         (0)       /* Bits 0-11: Start logic status register 0 */
#define SYSCON_STARTSRP0_SRPIO0_MASK          (0xfff << SYSCON_STARTSRP0_SRPIO0_SHIFT)
#  define SYSCON_STARTSRP0_SRPIO0_BIT(n)      ((1 << n) << SYSCON_STARTSRP0_SRPIO0_SHIFT) /* n = 0 to 11 */
#define SYSCON_STARTSRP0_SRPIO1_0             (1 << 12) /* Bit 12: Start signal status for start logic input PIO1_0 */
                                                        /* Bits 13-31: Reserved */

                                                        /* Bits 0-2: Reserved. NOTE: Always write these bits as 111 */
#define SYSCON_PDSLEEPCFG_BOD_PD              (1 << 3)  /* BOD power-down control in Deep-sleep mode */
                                                        /* Bits 4-5: Reserved. NOTE: Always write these bits as 11 */
#define SYSCON_PDSLEEPCFG_WDTOSC_PD           (1 << 6)  /* Watchdog oscillator power control in Deep-sleep mode */
                                                        /* Bit 7:  Reserved. NOTE: Always write this bit as 1 */
                                                        /* Bits 8-10: Reserved NOTE: Always write these bits as 000 */
                                                        /* Bits 11-12: Reserved. NOTE: Always write these bits as 11 */
                                                        /* Bits 13-31: Reserved */

#define SYSCON_PDAWAKECFG_IRCOUT_PD           (1 << 0)  /* Bit 0:  IRC oscillator output wake-up configuration */
#define SYSCON_PDAWAKECFG_IRC_PD              (1 << 1)  /* Bit 1:  IRC oscillator wake-up configuration */
#define SYSCON_PDAWAKECFG_FLASH_PD            (1 << 2)  /* Bit 2:  Flash wake-up configuration */
#define SYSCON_PDAWAKECFG_BOD_PD              (1 << 3)  /* Bit 3:  Brownout Detection wake-up configuration */
#define SYSCON_PDAWAKECFG_ADC_PD              (1 << 4)  /* Bit 4:  ADC wake-up configuration */
#define SYSCON_PDAWAKECFG_SYSOSC_PD           (1 << 5)  /* Bit 5:  System oscillator wake-up configuration */
#define SYSCON_PDAWAKECFG_WDTOSC_PD           (1 << 6)  /* Bit 6:  Watchdog oscillator wake-up configuration */
#define SYSCON_PDAWAKECFG_SYSPLL_PD           (1 << 7)  /* Bit 7:  System PLL wake-up configuration */
                                                        /* Bit 8:  Reserved. NOTE: Always write this bit as 1 */
                                                        /* Bit 9:  Reserved. NOTE: Always write this bit as 0 */
                                                        /* Bit 10: Reserved. NOTE: Always write this bit as 1 */
                                                        /* Bit 11: Reserved. NOTE: Always write this bit as 1 */
                                                        /* Bit 12: Reserved. NOTE: Always write this bit as 0 */
                                                        /* Bits 13-15: Reserved. NOTE: Always write these bits as 111 */
                                                        /* Bits 16-31: Reserved */

#define SYSCON_PDRUNCFG_IRCOUT_PD             (1 << 0)  /* Bit 0:  IRC oscillator output power-down */
#define SYSCON_PDRUNCFG_IRC_PD                (1 << 1)  /* Bit 1:  IRC oscillator power-down */
#define SYSCON_PDRUNCFG_FLASH_PD              (1 << 2)  /* Bit 2:  Flash power-down */
#define SYSCON_PDRUNCFG_BOD_PD                (1 << 3)  /* Bit 3:  Brownout Detection power-down */
#define SYSCON_PDRUNCFG_ADC_PD                (1 << 4)  /* Bit 4:  ADC power-down */
#define SYSCON_PDRUNCFG_SYSOSC_PD             (1 << 5)  /* Bit 5:  System oscillator power-down */
#define SYSCON_PDRUNCFG_WDTOSC_PD             (1 << 6)  /* Bit 6:  Watchdog oscillator power-down */
#define SYSCON_PDRUNCFG_SYSPLL_PD             (1 << 7)  /* Bit 7:  System PLL power-down */
                                                        /* Bit 8:  Reserved. NOTE: Always write this bit as 1 */
                                                        /* Bit 9:  Reserved. NOTE: Always write this bit as 0 */
                                                        /* Bit 10: Reserved. NOTE: Always write this bit as 1 */
                                                        /* Bit 11: Reserved. NOTE: Always write this bit as 1 */
                                                        /* Bit 12: Reserved. NOTE: Always write this bit as 0 */
                                                        /* Bits 13-15: Reserved. NOTE: Always write these bits as 111 */
                                                        /* Bits 16-31: Reserved */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC11XX_CHIP_LPC11_SYSCON_H */
