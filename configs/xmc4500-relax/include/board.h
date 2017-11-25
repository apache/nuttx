/************************************************************************************
 * configs/xmc4500-relax/include/board.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIG_XMC4500_RELAX_INCLUDE_BOARD_H
#define __CONFIG_XMC4500_RELAX_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* The maximum frequency for the XMC4500 is 120MHz. */

#undef  BOARD_FCPU_144MHZ
#define BOARD_FCPU_120MHZ     1

/* Watchdog clock source selection */

#define WDT_CLKSRC_FOFI       0              /* fOFI clock  */
#define WDT_CLKSRC_FSTDY      1              /* fSTDY clock */
#define WDT_CLKSRC_FPLL       2              /* fPLL clock  */

/* External Clock source selection */

#define EXT_CLKSRC_FSYS       0              /* fSYS clock                   */
#define EXT_CLKSRC_FUSB       2              /* fUSB clock divided by ECKDIV */
#define EXT_CLKSRC_FPLL       3              /* fPLL clock divided by ECKDIV */

/* Factory Calibration */

#undef BOARD_FOFI_CALIBRATION                /* Enable factory calibration */

/* On-board crystals
 *
 *   NOTE: Only the XMC4500 Relax Kit-V1 provides the 32.768KHz RTC crystal.  It
 *   is not available on XMC4500 Relax Lite Kit-V1.
 */

#define BOARD_XTAL_FREQUENCY        12000000 /* 12MHz XTAL */
#undef  BOARD_RTC_XTAL_FRQUENCY              /* 32.768KHz RTC XTAL not available on the Relax Lite */

#if defined(BOARD_FCPU_144MHZ)
/* Default clock initialization
 *
 *   fXTAL = 12Mhz
 *      -> fPLL = (fXTAL / (2 * 1) * 48) = 288MHz
 *         -> fSYS = (fPLL / 1)          = 288MHz
 *            -> fCPU = (fSYS / 2)       = 144MHz
 *               -> fPERIPH = (fCPU / 1) = 144MHz
 *            -> fCCU = (fSYS / 2)       = 144MHz
 *            -> fETH                    = 72MHz   (REVISIT)
 *         -> fUSB                       = 48MHz   (REVISIT)
 *         -> fEBU                       = 72MHz   (REVISIT)
 *
 * fUSBPLL Disabled, only enabled if SCU_CLK_USBCLKCR_USBSEL_USBPLL is selected
 *
 *   fOFI                                = 24MHz
 *      -> fWDT                          = 24MHz   (REVISIT)
 */

/* Select the external crystal as the PLL clock source */

#  define BOARD_PLL_CLOCKSRC_XTAL   1        /* PLL Clock source == extnernal crystal */
#  undef  BOARD_PLL_CLOCKSRC_OFI             /* PLL Clock source != internal fast oscillator */

/* PLL Configuration:
 *
 * fPLL = (fPLLSRC / (pdiv * k2div) * ndiv
 *
 * fPLL = (12000000 / (2 * 1)) * 48
 *      = 288MHz
 */

#  define BOARD_ENABLE_PLL          1
#  define BOARD_PLL_PDIV            2
#  define BOARD_PLL_NDIV            48
#  define BOARD_PLL_K2DIV           1
#  define BOARD_PLL_FREQUENCY       288000000

/* System frequency, fSYS, is divided down from PLL output */

#  define BOARD_SYSDIV              1        /* PLL Output divider to get fSYS */
#  define BOARD_SYS_FREQUENCY       288000000

/* CPU frequency, fCPU, may be divided down from system frequency */

#  define BOARD_CPUDIV_ENABLE       1        /* Enable PLL divide by 2 for fCPU */
#  define BOARD_CPU_FREQUENCY       144000000

/* CCU frequency may be divided down from system frequency */

#  define BOARD_CCUDIV_ENABLE       1        /* Enable PLL div by 2 */
#  define BOARD_CCU_FREQUENCY       144000000

/* Watchdog clock settings */

#  define BOARD_WDT_SOURCE          WDT_CLKSRC_FOFI
#  define BOARD_WDTDIV              1
#  define BOARD_WDT_FREQUENCY       24000000

/* EBU frequency may be divided down from system frequency */

#  define BOARD_EBUDIV              2        /* fSYS / 2 */
#  define BOARD_EBU_FREQUENCY       72000000

/* EXT clock settings */

#  define BOARD_EXT_SOURCE          EXT_CLKSRC_FPLL
#  define BOARD_EXTDIV              289      /* REVISIT */
#  define BOARD_EXT_FREQUENCY       498270   /* REVISIT */

/* The peripheral clock, fPERIPH, derives from fCPU with no division */

#  define BOARD_PBDIV               1        /* No division */
#  define BOARD_PERIPH_FREQUENCY    144000000

#elif defined(BOARD_FCPU_120MHZ)
/* Default clock initialization
 *
 *   fXTAL = 12Mhz
 *      -> fPLL = (fXTAL / (2 * 4) * 80) = 120
 *         -> fSYS = (fPLL / 1)          = 120MHz
 *            -> fCPU = (fSYS / 1)       = 120MHz
 *               -> fPERIPH = (fCPU / 1) = 120MHz
 *            -> fCCU = (fSYS / 1)       = 120MHz
 *            -> fETH                    = 60MHz   (REVISIT)
 *         -> fUSB                       = 48MHz   (REVISIT)
 *         -> fEBU                       = 60MHz   (REVISIT)
 *
 * fUSBPLL Disabled, only enabled if SCU_CLK_USBCLKCR_USBSEL_USBPLL is selected
 *
 *   fOFI                                = 24MHz
 *      -> fWDT                          = 24MHz   (REVISIT)
 */

/* Select the external crystal as the PLL clock source */

#  define BOARD_PLL_CLOCKSRC_XTAL   1        /* PLL Clock source == extnernal crystal */
#  undef  BOARD_PLL_CLOCKSRC_OFI             /* PLL Clock source != internal fast oscillator */

/* PLL Configuration:
 *
 * fPLL = (fPLLSRC / (pdiv * k2div) * ndiv
 *
 * fPLL = (12000000 / (2 * 4)) * 80
 *      = 120MHz
 */

#  define BOARD_ENABLE_PLL          1
#  define BOARD_PLL_PDIV            2
#  define BOARD_PLL_NDIV            80
#  define BOARD_PLL_K2DIV           4
#  define BOARD_PLL_FREQUENCY       120000000

/* System frequency, fSYS, is divided down from PLL output */

#  define BOARD_SYSDIV              1        /* No division */
#  define BOARD_SYS_FREQUENCY       120000000

/* CPU frequency, fCPU, may be divided down from system frequency */

#  define BOARD_CPUDIV_ENABLE       0        /* No divison */
#  define BOARD_CPU_FREQUENCY       120000000

/* CCU frequency may be divided down from system frequency */

#  define BOARD_CCUDIV_ENABLE       0        /* No divison */
#  define BOARD_CCU_FREQUENCY       120000000

/* Watchdog clock setting */

#  define BOARD_WDT_SOURCE          WDT_CLKSRC_FOFI
#  define BOARD_WDTDIV              1
#  define BOARD_WDT_FREQUENCY       24000000

/* EBU frequency may be divided down from system frequency */

#  define BOARD_EBUDIV              2       /* fSYS/2 */
#  define BOARD_EBU_FREQUENCY       60000000

/* EXT clock settings */

#  define BOARD_EXT_SOURCE          EXT_CLKSRC_FPLL
#  define BOARD_EXTDIV              289      /* REVISIT */
#  define BOARD_EXT_FREQUENCY       415225   /* REVISIT */

/* The peripheral clock, fPERIPH, derives from fCPU with no division */

#  define BOARD_PBDIV               1        /* No division */
#  define BOARD_PERIPH_FREQUENCY    120000000
#endif

/* Standby clock source selection
 *
 * BOARD_STDBY_CLOCKSRC_OSI    - Internal 32.768KHz slow oscillator
 * BOARD_STDBY_CLOCKSRC_OSCULP - External 32.768KHz crystal
 */

#define BOARD_STDBY_CLOCKSRC_OSI   1
#undef  BOARD_STDBY_CLOCKSRC_OSCULP
#define BOARD_STDBY_FREQUENCY     32768

/* USB PLL settings.
 *
 *   fUSBPLL = 48MHz and fUSBPLLVCO = 384 MHz
 *
 * Note: Implicit divider of 2 and fUSBPLLVCO >= 260 MHz and
 * fUSBPLLVCO <= 520 MHz
 */

#undef  BOARD_ENABLE_USBPLL
#define BOARD_USB_PDIV            2
#define BOARD_USB_NDIV            64

/* FLASH wait states */

#define BOARD_FLASH_WS            5

/* LED definitions ******************************************************************/
/* The XMC4500 Relax Lite v1 board has two LEDs:
 *
 * LED1 P1.1 High output illuminates
 * LED2 P1.0 High output illuminates
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED0        0
#define BOARD_LED1        1
#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_LED0_BIT    (1 << BOARD_LED0)
#define BOARD_LED1_BIT    (1 << BOARD_LED1)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *   SYMBOL                  Meaning                     LED state
 *                                                      LED2   LED1
 *   ---------------------  --------------------------  ------ ------ */

#define LED_STARTED       0 /* NuttX has been started   OFF    OFF    */
#define LED_HEAPALLOCATE  0 /* Heap has been allocated  OFF    OFF    */
#define LED_IRQSENABLED   0 /* Interrupts enabled       OFF    OFF    */
#define LED_STACKCREATED  1 /* Idle stack created       ON     OFF    */
#define LED_INIRQ         2 /* In an interrupt           No change    */
#define LED_SIGNAL        2 /* In a signal handler       No change    */
#define LED_ASSERTION     2 /* An assertion failed       No change    */
#define LED_PANIC         3 /* The system has crashed   N/C  Blinking */
#undef  LED_IDLE            /* MCU is is sleep mode      Not used     */

/* Thus if LED1 is statically on, NuttX has successfully booted and is,
 * apparently, running normally.  If LED2 is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/* Button definitions ***************************************************************/
/* The XMC4500 Relax Lite v1 board has two buttons:
 *
 * BUTTON1 P1.14 Low input sensed when button pressed
 * BUTTON2 P1.15 Low input sensed when button pressed
 */

#define BUTTON_0          0
#define BUTTON_1          1
#define NUM_BUTTONS       2

#define BUTTON_0_BIT      (1 << BUTTON_0)
#define BUTTON_1_BIT      (1 << BUTTON_1)

/* USIC0 ****************************************************************************/
/* USIC0 CH0 is used as UART0
 *
 *  RX - P1.4
 *  TX - P1.5
 */

#define BOARD_UART0_DX    USIC_DXB
#define GPIO_UART0_RXD    GPIO_U0C0_DX0B
#define GPIO_UART0_TXD    (GPIO_U0C0_DOUT0_3 | GPIO_PADA1P_STRONGSOFT | GPIO_OUTPUT_SET)

/* USIC1 CH1 is used as UART3
 *
 *  RX - P0.0
 *  TX - P0.1
 */

#define BOARD_UART3_DX    USIC_DXD
#define GPIO_UART3_RXD    (GPIO_U1C1_DX0D | GPIO_INPUT_PULLUP)
#define GPIO_UART3_TXD    (GPIO_U1C1_DOUT0_2 | GPIO_PADA1P_STRONGSOFT | GPIO_OUTPUT_SET)

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_XMC4500_RELAX_INCLUDE_BOARD_H */
