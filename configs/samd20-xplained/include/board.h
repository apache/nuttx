/************************************************************************************
 * configs/samd20-xplained/include/board.h
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

#ifndef __CONFIGS_SAMD20_XPLAINED_INCLUDE_BOARD_H
#define __CONFIGS_SAMD20_XPLAINED_INCLUDE_BOARD_H

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

/* Nominal frequencies of on-chip RC oscillators.  These are *not* configurable
 * but appear here for use in frequency calculations.  NOTE: These frequencies
 * may vary with temperature changes.
 */

#define BOARD_OSCULP32K_FREQUENCY  32000    /* 32kHz ultra-low-power internal oscillator */
#define BOARD_OSC8M_FREQUENCY      8000000  /* 8MHz high-accuracy internal oscillator */
#define BOARD_DFLL48M_FREQUENCY    48000000 /* 48MHz Digital Frequency Locked Loop */

/* The SAMD20 Xplained Pro has one on-board crystal:
 *
 *   XC101 32.768KHz XOSC32
 */

/* XOSC Configuration -- Not available
 *
 *   BOARD_XOSC_ENABLE          - Boolean (defined / not defined)
 *   BOARD_XOSC_FREQUENCY       - In Hz
 *   BOARD_XOSC_STARTUPTIME     - See SYSCTRL_XOSC_STARTUP_* definitions
 *   BOARD_XOSC_ISCRYSTAL       - Boolean (defined / not defined)
 *   BOARD_XOSC_AMPGC           - Boolean (defined / not defined)
 *   BOARD_XOSC_ONDEMAND        - Boolean (defined / not defined)
 *   BOARD_XOSC_RUNINSTANDBY    - Boolean (defined / not defined)
 */

#undef  BOARD_XOSC_ENABLE
#define BOARD_XOSC_FREQUENCY       12000000UL
#define BOARD_XOSC_STARTUPTIME     SYSCTRL_XOSC_STARTUP_1S
#define BOARD_XOSC_ISCRYSTAL       1
#define BOARD_XOSC_AMPGC           1
#define BOARD_XOSC_ONDEMAND        1
#undef  BOARD_XOSC_RUNINSTANDBY

/* XOSC32 Configuration -- Not used
 *
 *   BOARD_XOSC32K_ENABLE       - Boolean (defined / not defined)
 *   BOARD_XOSC32K_FREQUENCY    - In Hz
 *   BOARD_XOSC32K_STARTUPTIME  - See SYSCTRL_XOSC32K_STARTUP_* definitions
 *   BOARD_XOSC32K_ISCRYSTAL    - Boolean (defined / not defined)
 *   BOARD_XOSC32K_AAMPEN       - Boolean (defined / not defined)
 *   BOARD_XOSC32K_EN1KHZ       - Boolean (defined / not defined)
 *   BOARD_XOSC32K_EN32KHZ      - Boolean (defined / not defined)
 *   BOARD_XOSC32K_ONDEMAND     - Boolean (defined / not defined)
 *   BOARD_XOSC32K_RUNINSTANDBY - Boolean (defined / not defined)
 */

#undef  BOARD_XOSC32K_ENABLE
#define BOARD_XOSC32K_FREQUENCY    32768    /* 32.768KHz XTAL */
#define BOARD_XOSC32K_STARTUPTIME  SYSCTRL_XOSC32K_STARTUP_2S
#define BOARD_XOSC32K_ISCRYSTAL    1
#define BOARD_XOSC32K_AAMPEN       1
#undef  BOARD_XOSC32K_EN1KHZ
#define BOARD_XOSC32K_EN32KHZ      1
#define BOARD_XOSC32K_ONDEMAND     1
#undef  BOARD_XOSC32K_RUNINSTANDBY

/* OSC32 Configuration -- not used
 *
 *   BOARD_OSC32K_ENABLE        - Boolean (defined / not defined)
 *   BOARD_OSC32K_FREQUENCY     - In Hz
 *   BOARD_OSC32K_STARTUPTIME   - See SYSCTRL_OSC32K_STARTUP_* definitions
 *   BOARD_OSC32K_EN1KHZ        - Boolean (defined / not defined)
 *   BOARD_OSC32K_EN32KHZ       - Boolean (defined / not defined)
 *   BOARD_OSC32K_ONDEMAND      - Boolean (defined / not defined)
 *   BOARD_OSC32K_RUNINSTANDBY  - Boolean (defined / not defined)
 */

#undef  BOARD_OSC32K_ENABLE
#define BOARD_OSC32K_FREQUENCY     32768    /* 32.768kHz internal oscillator */
#define BOARD_OSC32K_STARTUPTIME   SYSCTRL_OSC32K_STARTUP_4MS
#define BOARD_OSC32K_EN1KHZ        1
#define BOARD_OSC32K_EN32KHZ       1
#define BOARD_OSC32K_ONDEMAND      1
#undef  BOARD_OSC32K_RUNINSTANDBY

/* OSC8M Configuration -- always enabled
 *
 *   BOARD_OSC8M_PRESCALER      - See SYSCTRL_OSC8M_PRESC_DIV* definitions
 *   BOARD_OSC8M_ONDEMAND       - Boolean (defined / not defined)
 *   BOARD_OSC8M_RUNINSTANDBY   - Boolean (defined / not defined)
 */

#define BOARD_OSC8M_PRESCALER      SYSCTRL_OSC8M_PRESC_DIV1
#define BOARD_OSC8M_ONDEMAND       1
#undef BOARD_OSC8M_RUNINSTANDBY

/* The source of the main clock is always GLCK_MAIN.  Also called GCLKGEN[0], this is
 * the clock feeding the Power Manager. The Power Manager, in turn, generates main
 * clock which is divided down to produce the CPU, AHB, and APB clocks.
 *
 * The main clock is initially OSC8M divided by 8.  But will be reconfigured here to
 * be DFLL48M.
 *
 * Select the OSC8M as the source of the GLCK_MAIN. Options (define one):
 *
 *   BOARD_GLCK_MAIN_SRC_XOSC      - XOSC oscillator output
 *   BOARD_GLCK_MAIN_SRC_GCLKIN    - Generator input pad
 *   BOARD_GLCK_MAIN_SRC_GCLKGEN1  - Generic clock generator 1 output
 *   BOARD_GLCK_MAIN_SRC_OSCULP32K - OSCULP32K oscillator output
 *   BOARD_GLCK_MAIN_SRC_OSC32K    - OSC32K oscillator output
 *   BOARD_GLCK_MAIN_SRC_XOSC32K   - XOSC32K oscillator output
 *   BOARD_GLCK_MAIN_SRC_OSC8M     - OSC8M oscillator output
 *   BOARD_GLCK_MAIN_SRC_DFLL48M   - DFLL48M output
 *
 * Fglckmain = Frefclk / Divider
 */

#define BOARD_GLCK_MAIN_SRC_OSC8M  1
#define BOARD_GLCK_MAIN_DIVIDER    1
#define BOARD_GLCK_MAIN_FREQUENCY  (BOARD_OSC8M_FREQUENCY / BOARD_GLCK_MAIN_DIVIDER)

/* Digital Frequency Locked Loop configuration.  In closed-loop mode, the
 * DFLL output frequency (Fdfll) is given by:
 *
 *  Fdfll = DFLLmul * Frefclk
 *        = (48000000/32768) * 32768 = 48MHz
 *
 * Where the reference clock is always the Generic Clock Channel 0 output.
 *
 * NOTE: Nothing must be defined if the DFPLL is not used
 */

#define BOARD_DFLL48M_TARGET       48000000
#define BOARD_DFLL48M_MUL          (BOARD_DFLL0_TARGET / BOARD_GCK_MAIN_FREQUENCY)
#define BOARD_DFLL48M_FREQUENCY    (BOARD_DFLL48M_MUL * BOARD_GCK_MAIN_FREQUENCY)

/* Main clock dividers
 *
 *    BOARD_CPU_DIVIDER   - See PM_CPUSEL_CPUDIV_* definitions
 *    BOARD_CPU_FRQUENCY  - In Hz
 *    BOARD_CPU_FAILDECT  - Boolean (defined / not defined)
 *    BOARD_APBA_DIVIDER  - See M_APBASEL_APBADIV_* definitions
 *    BOARD_APBA_FRQUENCY - In Hz
 *    BOARD_APBB_DIVIDER  - See M_APBBSEL_APBBDIV_* definitions
 *    BOARD_APBB_FRQUENCY - In Hz
 *    BOARD_APBC_DIVIDER  - See M_APBCSEL_APBCDIV_* definitions
 *    BOARD_APBC_FRQUENCY - In Hz
 */

#define BOARD_CPU_FAILDECT         1
#define BOARD_CPU_DIVIDER          PM_CPUSEL_CPUDIV_1
#define BOARD_APBA_DIVIDER         PM_APBASEL_APBADIV_1
#define BOARD_APBB_DIVIDER         PM_APBBSEL_APBBDIV_1
#define BOARD_APBC_DIVIDER         PM_APBCSEL_APBCDIV_1

/* Resulting frequencies */

#define BOARD_MCK_FREQUENCY        (BOARD_GLCK_MAIN_FREQUENCY)
#define BOARD_CPU_FREQUENCY        (BOARD_MCK_FREQUENCY)
#define BOARD_PBA_FREQUENCY        (BOARD_MCK_FREQUENCY)
#define BOARD_PBB_FREQUENCY        (BOARD_MCK_FREQUENCY)
#define BOARD_PBC_FREQUENCY        (BOARD_MCK_FREQUENCY)
#define BOARD_PBD_FREQUENCY        (BOARD_MCK_FREQUENCY)

/* FLASH wait states */

#define BOARD_FLASH_WAITSTATES     0

/* LED definitions ******************************************************************/
/* There are three LEDs on board the SAMD20 Xplained Pro board:  The EDBG
 * controls two of the LEDs, a power LED and a status LED.  There is only
 * one user controllable LED, a yellow LED labelled STATUS near the SAMD20 USB
 * connector.
 *
 * This LED is controlled by PC07 and the LED can be activated by driving the
 * PA14 to GND.
 */

/* LED index values for use with sam_setled() */

#define BOARD_STATUS_LED     0
#define BOARD_NLEDS          1

/* LED bits for use with sam_setleds() */

#define BOARD_STATUS LED_BIT (1 << BOARD_STATUS_LED)

/* When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
 * control the LED as defined below.  Thus if the LED is statically on, NuttX has
 * successfully booted and is, apparently, running normally.  If the LED is
 * flashing at approximately 2Hz, then a fatal error has been detected and the
 * system has halted.
 */

#define LED_STARTED       0 /* STATUS LED=OFF */
#define LED_HEAPALLOCATE  0 /* STATUS LED=OFF */
#define LED_IRQSENABLED   0 /* STATUS LED=OFF */
#define LED_STACKCREATED  1 /* STATUS LED=ON */
#define LED_INIRQ         2 /* STATUS LED=no change */
#define LED_SIGNAL        2 /* STATUS LED=no change */
#define LED_ASSERTION     2 /* STATUS LED=no change */
#define LED_PANIC         3 /* STATUS LED=flashing */

/* Button definitions ***************************************************************/
/* Mechanical buttons:
 *
 * The SAMD20 Xplained Pro contains two mechanical buttons. One button is the
 * RESET button connected to the SAMD20 reset line and the other is a generic user
 * configurable button. When a button is pressed it will drive the I/O line to GND.
 *
 *   PA15 SW0
 */

/* The SAMD20 Xplained Pro supports one button: */

#define BUTTON_SW0         0
#define NUM_BUTTONS        1

#define BUTTON_SW0_BIT     (1 << BUTTON_SW0)

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
/************************************************************************************
 * Name: sam_boardinitialize
 *
 * Description:
 *   All SAM3U architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_SAMD20_XPLAINED_INCLUDE_BOARD_H */
