/****************************************************************************
 * boards/arm/ra8m1/ek-ra8m1/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM_RA8M1_EK_RA8M1_INCLUDE_BOARD_H
#define __BOARDS_ARM_RA8M1_EK_RA8M1_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Do not include RA8M1 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Every BOARD_* clock option below has to be defined by the board: the
 * RA8M1 code (arch/arm/src/ra8m1/ra_clockconfig.h) derives all the clock
 * frequencies from them and checks them against the limits of the RA8M1
 * User's Manual, chapter 8.  A missing or invalid value stops the build
 * with an #error that says what is wrong.  The MCU comes out of reset
 * running from the 8 MHz MOCO with every divider at 1; the options here
 * change that.
 *
 * The EK-RA8M1 clock sources are:
 *
 *   MOCO: 8 MHz internal oscillator (the reset system clock)
 *   HOCO: internal oscillator, 16/18/20/32/48 MHz (BOARD_HOCO_FREQUENCY)
 *   MOSC: 20 MHz resonator on EXTAL/XTAL
 *
 * The *_SOURCE options below take the real register-field values from
 * arch/arm/src/ra8m1/hardware/ra8m1_system.h, not a value invented for
 * this file: whichever register the option ends up in is named in each
 * option's own description.  This file does not include that header (see
 * the note above); the names are resolved later, when ra_clockconfig.h
 * and ra_clockconfig.c use them, since both already include it themselves.
 */

/* Oscillators
 *
 * BOARD_HOCO_FREQUENCY
 *   HOCO frequency in Hz: 16000000, 18000000, 20000000, 32000000 or
 *   48000000.  This is the frequency the MCU starts the HOCO at after
 *   reset, so it is also programmed into OFS1 (option bytes below).
 *
 * BOARD_MOSC_FREQUENCY
 *   Frequency of the resonator, or of the external clock, on EXTAL/XTAL,
 *   8000000 to 48000000 Hz.  Only needed when the MOSC is used, directly
 *   or as a PLL source.
 *
 * BOARD_MOSC_EXTERNAL_CLOCK
 *   0: a resonator is connected to EXTAL/XTAL.
 *   1: EXTAL is driven by an external clock (no stabilization wait).
 *
 * BOARD_MOSC_STABILIZATION_US
 *   How long to wait for the resonator to stabilize, in microseconds
 *   (12 to 31139).  Use at least what the resonator manufacturer
 *   recommends; it is rounded up to the next value the MOSC wait register
 *   supports.  Not used with an external clock.
 */

#define BOARD_HOCO_FREQUENCY      48000000

#define BOARD_MOSC_FREQUENCY      20000000
#define BOARD_MOSC_EXTERNAL_CLOCK 0
#define BOARD_MOSC_STABILIZATION_US 10000

/* System clock source
 *
 * BOARD_SYSCLK_SOURCE feeds CPUCLK, ICLK, PCLKA-E, FCLK and BCLK (before
 * their dividers).  It is the SCKSCR.CKSEL value: one of the
 * R_SYSTEM_SCKSCR_CKSEL_* codes --
 *
 *   R_SYSTEM_SCKSCR_CKSEL_MOCO                  8 MHz MOCO (reset default)
 *   R_SYSTEM_SCKSCR_CKSEL_HOCO                  HOCO, BOARD_HOCO_FREQUENCY
 *   R_SYSTEM_SCKSCR_CKSEL_MAIN_CLOCK_OSCILLATOR Main oscillator (MOSC)
 *   R_SYSTEM_SCKSCR_CKSEL_PLL                   PLL1 output P
 *
 * PLL2 can not be the system clock; it only feeds the dedicated peripheral
 * clocks (SCICLK below).
 */

#define BOARD_SYSCLK_SOURCE       R_SYSTEM_SCKSCR_CKSEL_PLL

/* PLL1 and PLL2
 *
 * A PLL is started only if something uses it: PLL1 when it is the system
 * clock or a dedicated clock source, PLL2 when a dedicated clock uses it.
 * The BOARD_PLL2_* options are only needed then.  For each PLL:
 *
 *   BOARD_PLLn_SOURCE
 *     0 for the main oscillator (MOSC), or the PLLnCCR PLnSRCSEL bit
 *     (R_SYSTEM_PLLCCR_PLSRCSEL for PLL1, R_SYSTEM_PLL2CCR_PL2SRCSEL for
 *     PLL2) for the HOCO -- either source is 8 MHz to 48 MHz.  With the
 *     HOCO as the source CPUCLK is limited to 360 MHz.
 *   BOARD_PLLn_INPUT_DIVIDER
 *     1 to 4.  Source / divider must be 6 MHz to 12 MHz.
 *   BOARD_PLLn_MULTIPLIER
 *     53 to 180.  Input x (multiplier + fraction) is the VCO frequency,
 *     which must be 640 MHz to 1440 MHz.
 *   BOARD_PLLn_MULTIPLIER_FRACTION
 *     One of the PLLnCCR PLLnMULNF codes:
 *     R_SYSTEM_PLL[2]CCR_PLL[2]MULNF_V0_00_VALUE_AFTER_RESET (+0),
 *     _V0_33_1_3 (+1/3), _V0_50_1_2 (+1/2) or _V0_66_2_3 (+2/3).
 *   BOARD_PLLn_DIVIDER_P
 *     2, 4, 6, 8 or 16.  VCO / divider must be 40 MHz to 480 MHz.
 *   BOARD_PLLn_DIVIDER_Q, BOARD_PLLn_DIVIDER_R
 *     2, 3, 4, 5, 6, 8 or 9.  VCO / divider must be 71 MHz to 480 MHz.
 *
 * Here: MOSC 20 MHz / 2 = 10 MHz, x 96 = 960 MHz VCO, and P, Q and R at
 * 960 MHz / 2 = 480 MHz.
 */

#define BOARD_PLL1_SOURCE         0
#define BOARD_PLL1_INPUT_DIVIDER  2
#define BOARD_PLL1_MULTIPLIER     96
#define BOARD_PLL1_MULTIPLIER_FRACTION \
  R_SYSTEM_PLLCCR_PLLMULNF_V0_00_VALUE_AFTER_RESET
#define BOARD_PLL1_DIVIDER_P      2
#define BOARD_PLL1_DIVIDER_Q      2
#define BOARD_PLL1_DIVIDER_R      2

#define BOARD_PLL2_SOURCE         0
#define BOARD_PLL2_INPUT_DIVIDER  2
#define BOARD_PLL2_MULTIPLIER     96
#define BOARD_PLL2_MULTIPLIER_FRACTION \
  R_SYSTEM_PLL2CCR_PLL2MULNF_V0_00_VALUE_AFTER_RESET
#define BOARD_PLL2_DIVIDER_P      2
#define BOARD_PLL2_DIVIDER_Q      2
#define BOARD_PLL2_DIVIDER_R      2

/* System clock dividers
 *
 * Each clock is the system clock divided by 1, 2, 3, 4, 6, 8, 12, 16, 32
 * or 64.  Dividers of 3, 6 and 12 can not be mixed with 2, 4, 8, 16, 32
 * and 64 across these nine options.  The limits, and the ratio rules that
 * are checked at build time:
 *
 *   Option                Clock                              Maximum
 *   --------------------  ---------------------------------  -------
 *   BOARD_CPUCLK_DIVIDER  CPU clock (also SysTick)           480 MHz
 *   BOARD_ICLK_DIVIDER    System clock (bus, flash cache)    240 MHz
 *   BOARD_PCLKA_DIVIDER   Peripheral clock A (SPI, GPT, ..)  120 MHz
 *   BOARD_PCLKB_DIVIDER   Peripheral clock B (WDT, ..)        60 MHz
 *   BOARD_PCLKC_DIVIDER   Peripheral clock C (ADC12)          60 MHz
 *   BOARD_PCLKD_DIVIDER   Peripheral clock D (GPT count)     120 MHz
 *   BOARD_PCLKE_DIVIDER   Peripheral clock E                 240 MHz
 *   BOARD_FCLK_DIVIDER    Flash interface clock (>=4 MHz)     60 MHz
 *   BOARD_BCLK_DIVIDER    External bus clock                 120 MHz
 *
 * CPUCLK:ICLK and ICLK:PCLKA/PCLKB/FCLK/BCLK must be integer ratios, PCLKA
 * must not be slower than PCLKB and PCLKD not slower than PCLKA.  If the
 * system clock is the PLL and CPUCLK is above what the package allows at
 * the moment of the switch (240 MHz for the BGA package) the start-up code
 * switches with a slower CPU divider first and raises CPUCLK afterwards.
 *
 * Here: CPUCLK 480 MHz, ICLK 240 MHz, PCLKA 120 MHz, PCLKB 60 MHz,
 * PCLKC 60 MHz, PCLKD 120 MHz, PCLKE 240 MHz, FCLK 60 MHz, BCLK 120 MHz.
 */

#define BOARD_CPUCLK_DIVIDER      1
#define BOARD_ICLK_DIVIDER        2
#define BOARD_PCLKA_DIVIDER       4
#define BOARD_PCLKB_DIVIDER       8
#define BOARD_PCLKC_DIVIDER       8
#define BOARD_PCLKD_DIVIDER       4
#define BOARD_PCLKE_DIVIDER       2
#define BOARD_FCLK_DIVIDER        8
#define BOARD_BCLK_DIVIDER        4

/* SCI clock (SCICLK)
 *
 * The baud rate generator of every SCI runs from SCICLK, which is
 * configured whenever an SCI UART is enabled (CONFIG_RA_SCIn_UART); these
 * two options are only needed then.  SCICLK does not depend on PCLKA.
 *
 *   BOARD_SCICLK_SOURCE
 *     RA_SCICLK_SRC_MOCO, _HOCO, _MOSC, _PLL1P, _PLL1Q, _PLL1R, _PLL2P,
 *     _PLL2Q or _PLL2R (arch/arm/src/ra8m1/ra_clockconfig.h).  Unlike the
 *     sources above, SCICKCR.SCICKSEL has no per-value name generated
 *     from the SVD, so this one selector is defined by ra_clockconfig.h
 *     itself rather than reusing a hardware register header.  The PLL
 *     chosen is started if it is not already.
 *   BOARD_SCICLK_DIVIDER
 *     1, 2, 3, 4, 5, 6 or 8.  SCICLK must be at most 120 MHz.
 *
 * Here: PLL1Q (480 MHz) / 4 = 120 MHz, which gives 115200 baud with 0.16 %
 * error.  The MOCO (8 MHz) would give 3.5 % error.
 */

#define BOARD_SCICLK_SOURCE       RA_SCICLK_SRC_PLL1Q
#define BOARD_SCICLK_DIVIDER      4

/* Option bytes *************************************************************/

/* The option-setting memory is flash that the MCU reads at reset, before
 * any code runs.  ra_option_setting.c programs it with the image from the
 * options below (flash the image as ELF, HEX or SREC, not as a raw binary:
 * these words sit far above the code flash).  This is a flat image, so the
 * secure registers are used: OFS0 (0x0300_A100), OFS2 (0x0300_A104) and
 * OFS1 (0x0300_A200).  Options are 0 (off) or 1 (on) unless stated.
 *
 * OFS0: watchdogs
 *
 *   BOARD_OFS0_IWDT_AUTOSTART, BOARD_OFS0_WDT_AUTOSTART
 *     1 starts the independent watchdog (IWDT) or the watchdog (WDT)
 *     counting right after reset with the options below.  Nothing
 *     refreshes them yet on RA8M1 (there is no NuttX watchdog driver), so
 *     leave them 0 unless other code does; otherwise the MCU resets over
 *     and over.  With 0 the options below are not needed.
 *   BOARD_OFS0_IWDT_TIMEOUT, BOARD_OFS0_WDT_TIMEOUT
 *     Cycles of the divided clock before the counter underflows.
 *     IWDT: 128, 512, 1024 or 2048.  WDT: 1024, 4096, 8192 or 16384.
 *   BOARD_OFS0_IWDT_CLKDIV, BOARD_OFS0_WDT_CLKDIV
 *     Division ratio of the counter clock.  IWDT (IWDTCLK divided by):
 *     1, 16, 32, 64, 128 or 256.  WDT (PCLKB divided by): 4, 64, 128,
 *     512, 2048 or 8192.
 *   BOARD_OFS0_IWDT_WINDOW_END, BOARD_OFS0_WDT_WINDOW_END
 *     Where the refresh window ends, in percent of the counter: 75, 50, 25
 *     or 0 (no window end).  It has to be smaller than the window start.
 *   BOARD_OFS0_IWDT_WINDOW_START, BOARD_OFS0_WDT_WINDOW_START
 *     Where the refresh window starts: 25, 50, 75 or 100 (no window
 *     start).  The counter starts at 100 % and underflows at 0 %.
 *   BOARD_OFS0_IWDT_RESET, BOARD_OFS0_WDT_RESET
 *     1: an underflow or refresh error resets the MCU.  0: it raises an
 *     interrupt request instead.
 *   BOARD_OFS0_IWDT_STOP_IN_SLEEP, BOARD_OFS0_WDT_STOP_IN_SLEEP
 *     1: stop counting in the low power modes (IWDT: Sleep, Deep Sleep,
 *     Software Standby, Deep Software Standby; WDT: Sleep and Deep Sleep).
 */

#define BOARD_OFS0_IWDT_AUTOSTART     0
#define BOARD_OFS0_IWDT_TIMEOUT       2048
#define BOARD_OFS0_IWDT_CLKDIV        128
#define BOARD_OFS0_IWDT_WINDOW_END    0
#define BOARD_OFS0_IWDT_WINDOW_START  100
#define BOARD_OFS0_IWDT_RESET         1
#define BOARD_OFS0_IWDT_STOP_IN_SLEEP 1

#define BOARD_OFS0_WDT_AUTOSTART      0
#define BOARD_OFS0_WDT_TIMEOUT        16384
#define BOARD_OFS0_WDT_CLKDIV         128
#define BOARD_OFS0_WDT_WINDOW_END     0
#define BOARD_OFS0_WDT_WINDOW_START   100
#define BOARD_OFS0_WDT_RESET          1
#define BOARD_OFS0_WDT_STOP_IN_SLEEP  1

/* OFS1: voltage detection, HOCO, debug and ECC
 *
 *   BOARD_OFS1_VDSEL_MV
 *     Voltage detection 0 level (PVD0) in millivolts: 2850, 2580, 2150,
 *     2000, 1900, 1800, 1700 or 1600.  1600 is prohibited when the VBATT
 *     function is enabled, both BOARD_OFS1_LVD0_RESET and
 *     BOARD_OFS1_PVD0_LOWPOWER are 0, and Deep Software Standby mode 1 or
 *     2 is used.
 *   BOARD_OFS1_LVD0_RESET
 *     1: hold the MCU in reset while VCC is below the level above,
 *     starting right after reset.
 *   BOARD_OFS1_PVD0_LOWPOWER
 *     1: PVD0 draws less current in Deep Software Standby mode 1 and 2,
 *     with a slower response.
 *   BOARD_OFS1_HOCO_START
 *     1: the HOCO starts oscillating before the CPU runs, shortening the
 *     wait for it to stabilize.  The system clock is not switched to it by
 *     this.  Required when the Flexible Software Boot Loader (FSBL) runs.
 *     The HOCO frequency is BOARD_HOCO_FREQUENCY above.
 *   BOARD_OFS1_SWDBG
 *     1: the MCU follows DBGAUTH0/DBGAUTH1, and the IWDT and WDT stop
 *     automatically while the CPU is in the debug state.
 *   BOARD_OFS1_INITECC
 *     1: the ECC function of the TCM and the cache is on at start-up.
 *     Changing it from 1 to 0 needs a power-on reset to take effect.
 *
 * OFS2: DCDC
 *
 *   BOARD_OFS2_DCDC
 *     1: enable the on-chip DCDC regulator at start-up.  Set 0 only if the
 *     board does not fit the DCDC external components.  It also sets the
 *     settling time after the system clock is switched to the PLL (150 us
 *     with the DCDC, 10 us without).
 */

#define BOARD_OFS1_VDSEL_MV           1600
#define BOARD_OFS1_LVD0_RESET         0
#define BOARD_OFS1_PVD0_LOWPOWER      0
#define BOARD_OFS1_HOCO_START         0
#define BOARD_OFS1_SWDBG              0
#define BOARD_OFS1_INITECC            0

#define BOARD_OFS2_DCDC               1

/* Alternate function pin selections ****************************************/

/* The on-board debugger (J-Link OB) provides a virtual COM port on SCI9:
 * RXD9 on PA15 and TXD9 on PA14 (pin group 3 of SCI9).
 */

#define GPIO_SCI9_RX   GPIO_RXD9_MISO9_SCL9_3  /* PA15 */
#define GPIO_SCI9_TX   GPIO_TXD9_MOSI9_SDA9_3  /* PA14 */

/* LED pin selections *******************************************************/

/* The EK-RA8M1 has three user LEDs.  They are driven active high. */

#define GPIO_LED1  (gpio_pinset_t){ PORT6, PIN0,  (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW) }  /* P600, blue */
#define GPIO_LED2  (gpio_pinset_t){ PORT4, PIN14, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW) }  /* P414, green */
#define GPIO_LED3  (gpio_pinset_t){ PORT1, PIN7,  (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW) }  /* P107, red */

#define LED_DRIVER_PATH "/dev/userleds"

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case the LEDs are used to encode OS-related events as
 * follows (see ra8m1_autoleds.c):
 *
 *   SYMBOL                Meaning                      LED1 LED2 LED3
 *   --------------------  ---------------------------  ---- ---- ----
 *   LED_STARTED           NuttX has been started       OFF  OFF  OFF
 *   LED_HEAPALLOCATE      Heap has been allocated      OFF  OFF  OFF
 *   LED_IRQSENABLED       Interrupts enabled           OFF  OFF  OFF
 *   LED_STACKCREATED      Idle stack created           ON   OFF  OFF
 *   LED_INIRQ             In an interrupt              N/C  ON   N/C
 *   LED_SIGNAL            In a signal handler          N/C  ON   N/C
 *   LED_ASSERTION         An assertion failed          N/C  ON   N/C
 *   LED_PANIC             The system has crashed       N/C  N/C  ON
 */

#define LED_STARTED       0
#define LED_HEAPALLOCATE  0
#define LED_IRQSENABLED   0
#define LED_STACKCREATED  1
#define LED_INIRQ         2
#define LED_SIGNAL        2
#define LED_ASSERTION     2
#define LED_PANIC         3

#endif /* __BOARDS_ARM_RA8M1_EK_RA8M1_INCLUDE_BOARD_H */
