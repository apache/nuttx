/************************************************************************************
 * configs/sam4l-xplained/include/board.h
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
 ************************************************************************************/

#ifndef __CONFIGS_SAM4L_XPLAINED_INCLUDE_BOARD_H
#define __CONFIGS_SAM4L_XPLAINED_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  ifdef CONFIG_GPIO_IRQ
#    include <arch/irq.h>
#  endif
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* Select the DFLL as the source of the system clock.
 *
 * Options (define one):
 *   BOARD_SYSCLK_SOURCE_RCSYS     - System RC oscillator
 *   BOARD_SYSCLK_SOURCE_OSC0      - Oscillator 0
 *   BOARD_SYSCLK_SOURCE_PLL0      - Phase Locked Loop 0
 *   BOARD_SYSCLK_SOURCE_DFLL0     - Digital Frequency Locked Loop
 *   BOARD_SYSCLK_SOURCE_RC80M     - 80 MHz RC oscillator
 *   BOARD_SYSCLK_SOURCE_FCFAST12M - 12 MHz RC oscillator
 *   BOARD_SYSCLK_SOURCE_FCFAST8M  - 8 MHz RC oscillator
 *   BOARD_SYSCLK_SOURCE_FCFAST4M  - 4 MHz RC oscillator
 *   BOARD_SYSCLK_SOURCE_RC1M      - 1 MHz RC oscillator
 */

#define BOARD_SYSCLK_SOURCE_DFLL0  1

/* Nominal frequencies in on-chip RC oscillators.  These are *not* configurable
 * but appear here for use in frequency calculations.  NOTE: These may frequencies
 * may vary with temperature changes.
 */

#define BOARD_RCSYS_FREQUENCY      115000   /* Nominal frequency of RCSYS (Hz) */
#define BOARD_RC32K_FREQUENCY      32768    /* Nominal frequency of RC32K (Hz) */
#define BOARD_RC80M_FREQUENCY      80000000 /* Nominal frequency of RC80M (Hz) */
#define BOARD_RCFAST4M_FREQUENCY   4000000  /* Nominal frequency of RCFAST4M (Hz) */
#define BOARD_RCFAST8M_FREQUENCY   8000000  /* Nominal frequency of RCFAST8M (Hz) */
#define BOARD_RCFAST12M_FREQUENCY  12000000 /* Nominal frequency of RCFAST12M (Hz) */
#define BOARD_RC1M_FREQUENCY       1000000  /* Nominal frequency of RC1M (Hz) */

/* The SAM4L Xplained Pro has two on-board crystals:
 *   XC100 12MHz     OSC0
 *   XC101 32.768KHz OSC32
 */

/* OSC0 Configuration */

#define BOARD_OSC0_FREQUENCY       12000000 /* 12MHz XTAL */

/* OSC32 Configuration */

#define BOARD_OSC32_FREQUENCY      32768    /* 32.768KHz XTAL */
#define BOARD_OSC32_STARTUP_US     6100
#define BOARD_OSC32_SELCURR        BSCIF_OSCCTRL32_SELCURR_300
#define BOARD_OSC32_ISXTAL         1        /* OSC32 is a crystal */

/* Digital Frequency Locked Loop configuration
 *  Fdfll = (Fclk * DFLLmul) / DFLLdiv
 *        = 32768 * (48000000/32768) / 1 = 48MHz
 *
 * The actual frequency is 47.97MHz due to truncation of the multiplier.
 * The 48MHz target value is treated as "not-to-exceed" value).  Use OSC0
 * if you need more accuracy (12MHz with a multiplier of 4).
 *
 * DFLL0 source options (select one):
 *   BOARD_DFLL0_SOURCE_RCSYS      - System RC oscillator
 *   BOARD_DFLL0_SOURCE_OSC32K     - 32.768KHz oscillator
 *   BOARD_DFLL0_SOURCE_OSC0       - Oscillator 0
 *   BOARD_DFLL0_SOURCE_RC80M      - 80 MHz RC oscillator
 *   BOARD_DFLL0_SOURCE_RC32K      - 32 kHz RC oscillator
 *
 * NOTE: Nothing must be defined if the DFPLL is not used
 */

#define BOARD_DFLL0_SOURCE_OSC32K  1
#define BOARD_DFLL0_TARGET         48000000
#define BOARD_DFLL0_MUL            (BOARD_DFLL0_TARGET / BOARD_OSC32_FREQUENCY)
#define BOARD_DFLL0_DIV            1
#define BOARD_DFLL0_FREQUENCY      (BOARD_OSC32_FREQUENCY * BOARD_DFLL0_MUL / BOARD_DFLL0_DIV)

/* Phase Locked Loop configuration
 *  Fdfll = (Fclk * PLLmul) / PLLdiv
 *
 * PLL0 source options (select one):
 *   BOARD_PLL0_SOURCE_OSC0        - Oscillator 0
 *   BOARD_PLL0_SOURCE_GCLK9       - General clock 9
 *
 *   BOARD_GLCK9_SOURCE_RCSYS      - System RC oscillator
 *   BOARD_GLCK9_SOURCE_OSC32K     - Output from OSC32K
 *   BOARD_GLCK9_SOURCE_DFLL0      - Output from DFLL0
 *   BOARD_GLCK9_SOURCE_OSC0       - Output from Oscillator0
 *   BOARD_GLCK9_SOURCE_RC80M      - Output from 80MHz RCOSC
 *   BOARD_GLCK9_SOURCE_RCFAST     - Output from 4,8,12MHz RCFAST
 *   BOARD_GLCK9_SOURCE_RC1M       - Output from 1MHz RC1M
 *   BOARD_GLCK9_SOURCE_CPUCLK     - The CPU clock
 *   BOARD_GLCK9_SOURCE_HSBCLK     - High Speed Bus clock
 *   BOARD_GLCK9_SOURCE_PBACLK     - Peripheral Bus A clock
 *   BOARD_GLCK9_SOURCE_PBBCLK     - Peripheral Bus B clock
 *   BOARD_GLCK9_SOURCE_PBCCLK     - Peripheral Bus C clock
 *   BOARD_GLCK9_SOURCE_PBDCLK     - Peripheral Bus D clock
 *   BOARD_GLCK9_SOURCE_RC32K      - Output from 32kHz RCOSC
 *
 * NOTE: Nothing must be defined if the PLL0 is not used
 */

/* System clock dividers: Fbus = Fmck >> BUSshift */

#define BOARD_CPU_SHIFT            0 /* Fcpu = Fmck = 48MHz */
#define BOARD_PBA_SHIFT            0 /* Fpba = Fmck = 48MHz */
#define BOARD_PBB_SHIFT            0 /* Fpbb = Fmck = 48MHz */
#define BOARD_PBC_SHIFT            0 /* Fpbc = Fmck = 48MHz */
#define BOARD_PBD_SHIFT            0 /* Fpbd = Fmck = 48MHz */

/* Resulting frequencies */

#define BOARD_MCK_FREQUENCY        (BOARD_DFLL0_FREQUENCY)
#define BOARD_CPU_FREQUENCY        (BOARD_MCK_FREQUENCY >> BOARD_CPU_SHIFT)
#define BOARD_PBA_FREQUENCY        (BOARD_MCK_FREQUENCY >> BOARD_PBA_SHIFT)
#define BOARD_PBB_FREQUENCY        (BOARD_MCK_FREQUENCY >> BOARD_PBB_SHIFT)
#define BOARD_PBC_FREQUENCY        (BOARD_MCK_FREQUENCY >> BOARD_PBC_SHIFT)
#define BOARD_PBD_FREQUENCY        (BOARD_MCK_FREQUENCY >> BOARD_PBD_SHIFT)

/* USBC.
 *
 * "The USBC has two bus clocks connected: One High Speed Bus clock
 *  (CLK_USBC_AHB) and one Peripheral Bus clock (CLK_USBC_APB). These clocks
 *  are generated by the Power Manager.  Both clocks are enabled at reset
 *  and can be disabled by the Power Manager. It is recommended to disable
 *  the USBC before disabling the clocks, to avoid freezing the USBC in
 *  an undefined state.
 *
 * "To follow the usb data rate at 12Mbit/s in full-speed mode, the
 *  CLK_USBC_AHB clock should be at minimum 12MHz.
 *
 * "The 48MHz USB clock is generated by a dedicated generic clock from
 *  the SCIF module. Before using the USB, the user must ensure that the
 *  USB generic clock (GCLK_USBC) is enabled at 48MHz in the SCIF module."
 *
 * USB Generic Clock 7 (GCLK_USBC) source selection (one only)
 *
 *   BOARD_USBC_SRC_OSC0
 *   BOARD_USBC_SRC_PLL0
 *   BOARD_USBC_SRC_DFLL
 *   BOARD_USBC_SRC_GCLKIN0
 */

#define BOARD_USBC_SRC_DFLL        1 /* Source DFLL0 at 48MHz */
#define BOARD_USBC_GCLK_DIV        1 /* Fusb = Fdfll / 1 = 48MHz */

/* LED definitions ******************************************************************/
/* There are three LEDs on board the SAM4L Xplained Pro board:  The EDBG
 * controls two of the LEDs, a power LED and a status LED.  There is only
 * one user controllable LED, a yellow LED labeled LED0 near the SAM4L USB
 * connector.
 *
 * This LED is controlled by PC07 and LED0 can be activated by driving the
 * PC07 to GND.
 */

/* LED index values for use with sam_setled() */

#define BOARD_LED0        0
#define BOARD_NLEDS       1

/* LED bits for use with sam_setleds() */

#define BOARD_LED0_BIT    (1 << BOARD_LED0)

/* When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
 * control LED0 as defined below.  Thus if LED0 is statically on, NuttX has
 * successfully booted and is, apparently, running normmally.  If LED0 is
 * flashing at approximately 2Hz, then a fatal error has been detected and the
 * system has halted.
 */

#define LED_STARTED       0 /* LED0=OFF */
#define LED_HEAPALLOCATE  0 /* LED0=OFF */
#define LED_IRQSENABLED   0 /* LED0=OFF */
#define LED_STACKCREATED  1 /* LED0=ON */
#define LED_INIRQ         2 /* LED0=no change */
#define LED_SIGNAL        2 /* LED0=no change */
#define LED_ASSERTION     2 /* LED0=no change */
#define LED_PANIC         3 /* LED0=flashing */

/* Button definitions ***************************************************************/
/* QTouch button: The SAM4L Xplained Pro kit has one QTouch button.  The connection
 * to the SAM4L is:
 *
 *   PC13 CATB_SENSE15
 *   PC14 CATB_DIS
 */

/* Mechanical buttons:
 *
 * The SAM4L Xplained Pro contains two mechanical buttons. One button is the
 * RESET button connected to the SAM4L reset line and the other is a generic user
 * configurable button. When a button is pressed it will drive the I/O line to GND.
 *
 *   PC24 SW0
 */

/* The STM32F4 Discovery supports one button: */

#define BUTTON_SW0         0
#define NUM_BUTTONS        1

#define BUTTON_SW0_BIT     (1 << BUTTON_SW0)

/* Alternate Function Disambiguation ************************************************/
/* USART0 is also available on connectors EXT1 and EXT4:
 *
 * EXT1  TXT4  GPIO  Function
 * ----  ---- ------ -----------
 *  13    13   PB00  USART0_RXD
 *  14    14   PB01  USART0_TXD
 */

#define GPIO_USART0_RXD    GPIO_USART0_RXD_4
#define GPIO_USART0_TXD    GPIO_USART0_TXD_4

/* The SAM4L Xplained Pro contains an Embedded Debugger (EDBG) that can be used to
 * program and debug the ATSAM4LC4C using Serial Wire Debug (SWD).  The Embedded
 * debugger also include a Virtual Com port interface over USART1.  Virtual COM
 * port connections:
 *
 *   PC26 USART1 RXD
 *   PC27 USART1 TXD
 */

#define GPIO_USART1_RXD    GPIO_USART1_RXD_2
#define GPIO_USART1_TXD    GPIO_USART1_TXD_2

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: sam_boardinitialize
 *
 * Description:
 *   All SAM3U architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void sam_boardinitialize(void);

/************************************************************************************
 * Name:  sam_ledinit, sam_setled, and sam_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LEDs.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfaces are available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void sam_ledinit(void);
void sam_setled(int led, bool ledon);
void sam_setleds(uint8_t ledset);
#endif

/************************************************************************************
 * Name: up_buttoninit
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After that,
 *   up_buttons() may be called to collect the current state of all buttons or
 *   up_irqbutton() may be called to register button interrupt handlers.
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
void up_buttoninit(void);

/************************************************************************************
 * Name: up_buttons
 *
 * Description:
 *   After up_buttoninit() has been called, up_buttons() may be called to collect
 *   the state of all buttons.  up_buttons() returns an 8-bit bit set with each bit
 *   associated with a button.  See the BUTTON* definitions above for the meaning of
 *   each bit in the returned value.
 *
 ************************************************************************************/

uint8_t up_buttons(void);

/************************************************************************************
 * Name: up_irqbutton
 *
 * Description:
 *   This function may be called to register an interrupt handler that will be
 *   called when a button is depressed or released.  The ID value is one of the
 *   BUTTON* definitions provided above. The previous interrupt handler address is
 *   returned (so that it may restored, if so desired).
 *
 ************************************************************************************/

#ifdef CONFIG_GPIOA_IRQ
xcpt_t up_irqbutton(int id, xcpt_t irqhandler);
#endif
#endif /* CONFIG_ARCH_BUTTONS */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_SAM4L_XPLAINED_INCLUDE_BOARD_H */
