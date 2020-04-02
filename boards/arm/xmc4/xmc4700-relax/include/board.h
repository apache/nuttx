/****************************************************************************
 * boards/arm/xmc4/xmc4700-relax/include/board.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_XMC4_XMC4700_RELAX_INCLUDE_BOARD_H
#define __BOARDS_ARM_XMC4_XMC4700_RELAX_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The maximum frequency for the XMC4700 is 144MHz. */

#undef  BOARD_FCPU_120MHZ
#define BOARD_FCPU_144MHZ    1

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

/* On-board crystals */

#define BOARD_XTAL_FREQUENCY        12000000 /* 12MHz XTAL */
#define BOARD_RTC_XTAL_FRQUENCY     32769    /* 32.768KHz RTC XTAL */

/* TODO: enable the RTC osc, use RTC for time/date
 */

/* Select the external crystal as the PLL clock source */

#define BOARD_PLL_CLOCKSRC_XTAL   1        /* PLL Clock source == extnernal crystal */
#undef  BOARD_PLL_CLOCKSRC_OFI             /* PLL Clock source != internal fast oscillator */

/* PLL Configuration:
 *
 *   fXTAL = 12Mhz
 *   260 MHz <= fVCO <= 520 MHz
 *
 * fVCO = fXTAL * ndiv / pdiv
 * fPLL = fVCO / k2div
 * fSYS = fPLL / sysdiv
 * fETH = fSYS / 2          (fixed div by 2)
 * fCCU = fSYS / ccudiv     (div by 1 or 2)
 * fCPU = fSYS / cpudiv     (div by 1 or 2)
 * fPERIPH = fCPU / pbdiv   (div by 1 or 2)
 */

#define BOARD_ENABLE_PLL          1   /* enable the PLL */
#define CPU_FREQ                  120 /* MHz */

/* TODO: Automate PLL calculations */

#if CPU_FREQ == 120

/*      120 MHz
 *
 * fVCO = 12MHz * 40 / 2  = 480MHz
 * fPLL = 480MHz / 2  = 240MHz
 * fSYS = fPLL / 2    = 120MHz
 * fCCU = fSYS / 2    =  60MHz
 * fCPU = fSYS / 1    = 120MHz
 * fPB  = fCPU / 2    =  60MHz
 * fETH = fSYS / 2    =  60MHz
 */

#  define BOARD_PLL_NDIV            40
#  define BOARD_PLL_PDIV            1
#  define BOARD_PLL_K2DIV           4
#  define BOARD_PLL_SYSDIV          1
#  define BOARD_PLL_CPUDIV          1
#  define BOARD_PLL_PBDIV           2
#  define BOARD_PLL_CCUDIV          2
#  define BOARD_PLL_EBUDIV          4

#elif CPU_FREQ == 144

/*     144 MHz
 *
 * fVCO = 12MHz * 36 / 1  = 432MHz
 * fPLL = 432MHz / 3  = 144MHz
 * fSYS = fPLL / 1    = 144MHz
 * fCCU = fSYS / 2    =  72MHz
 * fCPU = fSYS / 1    = 144MHz
 * fPB  = fCPU / 2    =  72MHz
 * fETH = fSYS / 2    =  72MHz
 */

#  define BOARD_PLL_NDIV            36
#  define BOARD_PLL_PDIV            1
#  define BOARD_PLL_K2DIV           3
#  define BOARD_PLL_SYSDIV          1
#  define BOARD_PLL_CPUDIV          1
#  define BOARD_PLL_PBDIV           2
#  define BOARD_PLL_CCUDIV          2
#  define BOARD_PLL_EBUDIV          2

#else
#  error "Illegal or Unsupported CPU Frequency"
#endif

#  define BOARD_CCUDIV_ENABLE       (BOARD_PLL_CCUDIV - 1)
#  define BOARD_CPUDIV_ENABLE       (BOARD_PLL_CPUDIV - 1)

#  define BOARD_VCO_FREQUENCY       (BOARD_XTAL_FREQUENCY * BOARD_PLL_NDIV / BOARD_PLL_PDIV)
#  define BOARD_PLL_FREQUENCY       (BOARD_VCO_FREQUENCY / BOARD_PLL_K2DIV)
#  define BOARD_SYS_FREQUENCY       (BOARD_PLL_FREQUENCY / BOARD_PLL_SYSDIV)
#  define BOARD_CCU_FREQUENCY       (BOARD_SYS_FREQUENCY / BOARD_PLL_CCUDIV)
#  define BOARD_CPU_FREQUENCY       (BOARD_SYS_FREQUENCY / BOARD_PLL_CPUDIV)
#  define BOARD_PERIPH_FREQUENCY    (BOARD_CPU_FREQUENCY / BOARD_PLL_PBDIV)
#  define BOARD_ETH_FREQUENCY       (BOARD_SYS_FREQUENCY / 2)

#  define BOARD_WDT_SOURCE          WDT_CLKSRC_FOFI
#  define BOARD_WDTDIV              1
#  define BOARD_WDT_FREQUENCY       24000000

#  define BOARD_EXT_SOURCE          EXT_CLKSRC_FPLL
#  define BOARD_PLL_ECKDIV          480     /* [1,512] */

#  define kHz_1     1000
#  define MHz_1     (kHz_1 * kHz_1)
#  define MHz_50    ( 50 * MHz_1)
#  define MHz_260   (260 * MHz_1)
#  define MHz_520   (520 * MHz_1)

/* range check VCO frequency */

#  if (BOARD_VCO_FREQUENCY < MHz_260)
#     error "VCO freq must be >= 260 MHz"
#  endif

#  if (BOARD_VCO_FREQUENCY > MHz_520)
#     error "VCO freq must be <= 520 MHz"
#  endif

/* range check Ethernet MAC frequency */

#  if (BOARD_ETH_FREQUENCY <= MHz_50)
#     error "ETH freq must be > 50 MHz"
#  endif

/* check ccudiv cpudiv pbdiv against Table 11-5
 * of XMC4700 User Manual
 */

#define CLKDIV_INDEX        (4 * (BOARD_PLL_CCUDIV-1) + \
                             2 * (BOARD_PLL_CPUDIV-1) + \
                                 (BOARD_PLL_PBDIV-1) )

#if (CLKDIV_INDEX == 3) || (CLKDIV_INDEX == 4) || (CLKDIV_INDEX > 6)
#  error "Illegal combination of dividers!  Ref: Table 11-5 of UM"
#endif

/* EXT clock settings */

#define BOARD_EXTCKL_ENABLE         1   /* 0 disables output */

#if BOARD_EXTCKL_ENABLE
#  define EXTCLK_PIN_P0_8           8
#  define EXTCLK_PIN_P1_15          15
#  define BOARD_EXTCLK_PIN          EXTCLK_PIN_P0_8
#  define BOARD_EXT_SOURCE          EXT_CLKSRC_FPLL
#  define BOARD_EXT_FREQUENCY       (250 * kHz_1)   /* Desired output freq */
#  define BOARD_EXTDIV              (BOARD_PLL_FREQUENCY / BOARD_EXT_FREQUENCY)

/* range check EXTDIV */

#  if BOARD_EXTDIV > 512
#    error "EXTCLK Divisor out of range!"
#  endif
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

/* LED definitions **********************************************************/

/* The XMC4700 Relax board has two LEDs:
 *
 * LED1 P5.9 High output illuminates
 * LED2 P5.8 High output illuminates
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.
 * The following definitions are used to access individual LEDs.
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
 *   ---------------------  --------------------------  ------ ------
 */

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

/* Button definitions *******************************************************/

/* The XMC4700 Relax board has two buttons:
 *
 * BUTTON1 P15.13 Low input sensed when button pressed
 * BUTTON2 P15.12 Low input sensed when button pressed
 */

#define BUTTON_0          0
#define BUTTON_1          1
#define NUM_BUTTONS       2

#define BUTTON_0_BIT      (1 << BUTTON_0)
#define BUTTON_1_BIT      (1 << BUTTON_1)

/* USIC0 ********************************************************************/

/* USIC0 CH0 is used as UART0
 *
 *  RX - P1.4
 *  TX - P1.5
 */

#define BOARD_UART0_DX    USIC_DXB
#define GPIO_UART0_RXD    GPIO_U0C0_DX0B
#define GPIO_UART0_TXD    (GPIO_U0C0_DOUT0_3 | GPIO_PADA1P_STRONGSOFT | GPIO_OUTPUT_SET)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_XMC4_XMC4700_RELAX_INCLUDE_BOARD_H */
