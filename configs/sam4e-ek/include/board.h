/************************************************************************************
 * configs/sam4e-ek/include/board.h
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

#ifndef __ARCH_SAM4E_EK_INCLUDE_BOARD_H
#define __ARCH_SAM4E_EK_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#  ifdef CONFIG_SAM34_GPIO_IRQ
#    include <arch/irq.h>
#  endif
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* After power-on reset, the SAM4E16 device is running out of the Master Clock using
 * the Fast RC Oscillator running at 4 MHz.
 *
 *   MAINOSC:  Frequency = 12MHz (crystal)
 *
 * CONFIG_SAM4EEK_120MHZ
 *   PLLA: PLL Divider = 1, Multiplier = 20 to generate PLLACK = 240MHz
 *   Master Clock (MCK): Source = PLLACK, Prescalar = 1 to generate MCK = 120MHz
 *   CPU clock: 120MHz
 *
 * CONFIG_SAM4EEK_96MHZ
 *   PLLA: PLL Divider = 1, Multiplier = 16 to generate PLLACK = 192MHz
 *   Master Clock (MCK): Source = PLLACK, Prescalar = 1 to generate MCK = 96MHz
 *   CPU clock: 96MHz
 */

/* Main oscillator register settings.
 *
 *   The start up time should be should be:
 *   Start Up Time = 8 * MOSCXTST / SLCK = 56 Slow Clock Cycles.
 */

#define BOARD_CKGR_MOR_MOSCXTST    (62 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

/* PLLA configuration.
 *
 *   Divider = 1
 *   Multipler = 16 or 20
 */

#ifdef CONFIG_SAM4EEK_120MHZ
#  define BOARD_CKGR_PLLAR_MUL     (19 << PMC_CKGR_PLLAR_MUL_SHIFT)
#else
#  define BOARD_CKGR_PLLAR_MUL     (15 << PMC_CKGR_PLLAR_MUL_SHIFT)
#endif

#define BOARD_CKGR_PLLAR_STMODE    PMC_CKGR_PLLAR_STMODE_FAST
#define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#define BOARD_CKGR_PLLAR_DIV       PMC_CKGR_PLLAR_DIV_BYPASS

/* PMC master clock register settings.
 *
 *  Source = PLLA
 *  Divider = 2
 */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV2

/* The PLL clock (USB_48M or UDPCK) is driven from the output of the PLL,
 * PLLACK.  The PLL clock must be 48MHz.  PLLACK can be divided down via the
 * PMC USB register to provide the PLL clock.  So in order to use the USB
 * feature, the PLL output must be a multiple of 48MHz.
 *
 * PLLACK = 240MHz, USBDIV=4, USB_48M = 240 MHz / (4 + 1) = 48MHz
 * PLLACK = 192MHz, USBDIV=5, USB_48M = 192 MHz / (3 + 1) = 48MHz
 */

#define BOARD_PMC_USBS            (0)

#ifdef CONFIG_SAM4EEK_120MHZ
#  define BOARD_PMC_USBDIV        (4 << PMC_USB_USBDIV_SHIFT)
#else
#  define BOARD_PMC_USBDIV        (3 << PMC_USB_USBDIV_SHIFT)
#endif

/* Resulting frequencies */

#define BOARD_MAINOSC_FREQUENCY    (12000000)  /* MAINOSC: 12MHz crystal on-board */

#ifdef CONFIG_SAM4EEK_120MHZ
#  define BOARD_PLLA_FREQUENCY     (240000000) /* PLLACK:  20 * 12Mhz / 1 */
#  define BOARD_MCK_FREQUENCY      (120000000) /* MCK:     PLLACK / 2 */
#  define BOARD_CPU_FREQUENCY      (120000000) /* CPU:     MCK */
#else
#  define BOARD_PLLA_FREQUENCY     (192000000) /* PLLACK:  16 * 12Mhz / 1 */
#  define BOARD_MCK_FREQUENCY      (96000000)  /* MCK:     PLLACK / 2 */
#  define BOARD_CPU_FREQUENCY      (96000000)  /* CPU:     MCK */
#endif

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV+1)).
 *
 *   MCI_SPEED = MCK / (2*(CLKDIV+1))
 *   CLKDIV = MCK / MCI_SPEED / 2 - 1
 *
 * Where CLKDIV has a range of 0-255.
 */

#ifdef CONFIG_SAM4EEK_120MHZ
  /* MCK = 120MHz, CLKDIV = 149 w/o CLKODD, MCI_SPEED = 120MHz / (2*149 + 0 + 2) = 400 KHz */

#  define HSMCI_INIT_CLKDIV        (149 << HSMCI_MR_CLKDIV_SHIFT)

  /* MCK = 120MHz, CLKDIV = 2 w/o CLKODD, MCI_SPEED = 120MHz / (2*2 + 0 + 2) = 20 MHz */

#  define HSMCI_MMCXFR_CLKDIV      (3 << HSMCI_MR_CLKDIV_SHIFT)

  /* MCK = 120MHz, CLKDIV = 1 w/ CLKODD, MCI_SPEED = 120MHz / (2*1 + 1 + 2) = 24 MHz */

#  define HSMCI_SDXFR_CLKDIV       ((1 << HSMCI_MR_CLKDIV_SHIFT) | HSMCI_MR_CLKODD)

#else
/* MCK = 96MHz, CLKDIV = 119, w/o CLKODD, MCI_SPEED = 96MHz / (2 * 119 + 0 + 2) = 400 KHz */

#  define HSMCI_INIT_CLKDIV        (119 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 96MHz, CLKDIV = 1 w/ CLKODD, MCI_SPEED = 96MHz / (2*1 + 1 + 2) = 19.2 MHz */

#  define HSMCI_MMCXFR_CLKDIV      ((3 << HSMCI_MR_CLKDIV_SHIFT) | HSMCI_MR_CLKODD)

/* MCK = 96MHz, CLKDIV = 1 w/o CLKODD, MCI_SPEED = 96MHz / (2*1 + 0 + 2) = 24 MHz */

#  define HSMCI_SDXFR_CLKDIV       (1 << HSMCI_MR_CLKDIV_SHIFT)
#endif

#define HSMCI_SDWIDEXFR_CLKDIV     HSMCI_SDXFR_CLKDIV

/* FLASH wait states.
 *
 * SAM4E-EK documetion says:
 * VDDCORE: "The voltage ranges from 1.08V to 1.32V."
 * VDDIO:   Looks like it is at 3.3V
 *
 * FWS Max frequency
 *      (1)      (2)      (3)      (4)
 * --- -------  -------  -------  -------
 *  0   17 MHz   20 MHz   17 MHz   21 MHz
 *  1   34 MHz   41 MHz   35 MHz   43 MHz
 *  2   51 MHz   62 MHz   53 MHz   64 MHz
 *  3   69 MHz   83 MHz   71 MHz   86 MHz
 *  4   86 MHz   96 MHz   88 MHz  107 MHz
 *  5  100 MHz  104 MHz  106 MHz  129 MHz
 *  6                    124 MHz

 * (1) VDDCORE set at 1.08V and VDDIO 1.62V to 3.6V @105C
 * (2) VDDCORE set at 1.08V and VDDIO 2.7V to 3.6V @105C
 * (3) VDDCORE set at 1.20V and VDDIO 1.62V to 3.6V @ 105C
 * (4) VDDCORE set at 1.20V and VDDIO 2.7V to 3.6V @ 105C
 */

#ifdef CONFIG_SAM4EEK_120MHZ
#  define BOARD_FWS                  5
#else
#  define BOARD_FWS                  4
#endif

/* LED definitions ******************************************************************/
/* The SAM4E-EK board has three, user-controllable LEDs labelled D2 (blue),
 * D3 (amber), and D4 (green) on the board.  Usage of these LEDs is defined
 * in include/board.h and src/up_leds.c. They are encoded as follows:
 *
 *   SYMBOL              Meaning                 D3*     D2      D4
 *   ------------------- ----------------------- ------- ------- -------
 *   LED_STARTED         NuttX has been started  OFF     OFF     OFF
 *   LED_HEAPALLOCATE    Heap has been allocated OFF     OFF     ON
 *   LED_IRQSENABLED     Interrupts enabled      OFF     ON      OFF
 *   LED_STACKCREATED    Idle stack created      OFF     ON      ON
 *   LED_INIRQ           In an interrupt**       N/C     FLASH   N/C
 *   LED_SIGNAL          In a signal handler***  N/C     N/C     FLASH
 *   LED_ASSERTION       An assertion failed     FLASH   N/C     N/C
 *   LED_PANIC           The system has crashed  FLASH   N/C     N/C
 *
 *   * If D2 and D4 are statically on, then NuttX probably failed to boot
 *     and these LEDs will give you some indication of where the failure was
 *  ** The normal state is D3=OFF, D4=ON and D2 faintly glowing.  This faint
 *     glow is because of timer interrupts that result in the LED being
 *     illuminated on a small proportion of the time.
 * *** D4 may also flicker normally if signals are processed.
 */

#define LED_STARTED                0 /* LED0=OFF LED1=OFF LED2=OFF */
#define LED_HEAPALLOCATE           1 /* LED0=OFF LED1=OFF LED2=ON */
#define LED_IRQSENABLED            2 /* LED0=OFF LED1=ON  LED2=OFF */
#define LED_STACKCREATED           3 /* LED0=OFF LED1=ON  LED2=ON */

#define LED_INIRQ                  4 /* LED0=XXX LED1=TOG LED2=XXX */
#define LED_SIGNAL                 5 /* LED0=XXX LED1=XXX LED2=TOG */
#define LED_ASSERTION              6 /* LED0=TOG LED1=XXX LED2=XXX */
#define LED_PANIC                  7 /* LED0=TOG LED1=XXX LED2=XXX */

/* LED index values for use with sam_setled() */

#define BOARD_LED_D3               0
#define BOARD_LED_D2               1
#define BOARD_LED_D4               2
#define BOARD_NLEDS                3

/* LED bits for use with sam_setleds() */

#define BOARD_LED_D3_BIT           (1 << BOARD_LED_D3)
#define BOARD_LED_D2_BIT           (1 << BOARD_LED_D2)
#define BOARD_LED_D4_BIT           (1 << BOARD_LED_D4)

/* Button definitions ***************************************************************/
/* Four buttons for software inputs:
 *
 *   PA1  BUTTON_SCROLL-UP    Grounded
 *   PA2  BUTTON_SCROLL-DOWN  Grounded
 *   PA19 BUTTON_WAKU         Grounded
 *   PA20 BUTTON_TAMP         Grounded
 */

#define BUTTON_SCROLLUP            1 /* Bit 0: Scroll-up button */
#define BUTTON_SCROLLDOWN          2 /* Bit 1: Scroll-down button */
#define BUTTON_WAKU                4 /* Bit 2: Waku button */
#define BUTTON_TAMP                8 /* Bit 3: Tamp button */

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
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void sam_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

/************************************************************************************
 * Name:  sam_ledinit, sam_setled, and sam_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LEDs.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfacesare available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void sam_ledinit(void);
void sam_setled(int led, bool ledon);
void sam_setleds(uint8_t ledset);
#endif

/************************************************************************************
 * Name:  stm32_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the Shenzhou board.  Because
 *   of the various rotations, clearing the display in the normal way by writing a
 *   sequences of runs that covers the entire display can be very slow.  Here the
 *   display is cleared by simply setting all GRAM memory to the specified color.
 *
 ************************************************************************************/

#if defined(CONFIG_SAM4EEK_LCD_RGB565)
void sam_lcdclear(uint16_t color);
#else /* if defined(CONFIG_SAM4EEK_LCD_RGB24) defined(CONFIG_SAM4EEK_LCD_RGB32) */
void sam_lcdclear(uint32_t color);
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_SAM4E_EK_INCLUDE_BOARD_H */
