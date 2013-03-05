/************************************************************************************
 * configs/open1788/include/board.h
 * include/arch/board/board.h
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

#ifndef __CONFIG_OPEN1788_INCLUDE_BOARD_H
#define __CONFIG_OPEN1788_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_GPIO_IRQ)
#  include <nuttx/irq.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Clocking *************************************************************************/
/* NOTE:  The following definitions require lpc17_syscon.h.  It is not included here
 * because the including C file may not have that file in its include path.
 */

#define BOARD_XTAL_FREQUENCY       (12000000)            /* XTAL oscillator frequency */
#define BOARD_OSCCLK_FREQUENCY     BOARD_XTAL_FREQUENCY  /* Main oscillator frequency */
#define BOARD_RTCCLK_FREQUENCY     (32768)               /* RTC oscillator frequency */
#define BOARD_INTRCOSC_FREQUENCY   (4000000)             /* Internal RC oscillator frequency */
#define BOARD_WDTOSC_FREQUENCY     (500000)              /* WDT oscillator frequency */

/* This is the clock setup we configure for:
 *
 *   SYSCLK = BOARD_OSCCLK_FREQUENCY = 12MHz  -> Select Main oscillator for source
 *   PLL0CLK = (10 * SYSCLK) / 1 = 120MHz -> PLL0 multipler=10, pre-divider=1
 *   CCLCK = 120MHz  -> CCLK divider = 1
 */

#define LPC17_CCLK                 120000000 /* 120Mhz */
#define BOARD_PCLKDIV              2         /* Peripheral clock = LPC17_CCLK/2 */
#define BOARD_PCLK_FREQUENCY       (LPC17_CCLK / BOARD_PCLKDIV)

/* Select the main oscillator as the frequency source.  SYSCLK is then the frequency
 * of the main oscillator.
 */

#undef CONFIG_LPC17_MAINOSC
#define CONFIG_LPC17_MAINOSC       1
#define BOARD_SCS_VALUE            SYSCON_SCS_OSCEN

/* Select the main oscillator and CCLK divider. The output of the divider is CCLK.
 * The input to the divider (PLLCLK) will be determined by the PLL output.
 */

#define BOARD_CCLKSEL_DIVIDER      1
#define BOARD_CCLKSEL_VALUE        (BOARD_CCLKSEL_DIVIDER | SYSCON_CCLKSEL_CCLKSEL)

/* PLL0.  PLL0 is used to generate the CPU clock (PLLCLK).
 *
 *  Source clock:               Main oscillator
 *  PLL0 Multiplier value (M):  10
 *  PLL0 Pre-divider value (P): 1
 *
 *  PLL0CLK = (M * SYSCLK) = 120MHz
 */

#undef CONFIG_LPC17_PLL0
#define CONFIG_LPC17_PLL0          1
#define BOARD_CLKSRCSEL_VALUE      SYSCON_CLKSRCSEL_MAIN

#define BOARD_PLL0CFG_MSEL         10
#define BOARD_PLL0CFG_PSEL         1
#define BOARD_PLL0CFG_VALUE \
  (((BOARD_PLL0CFG_MSEL-1) << SYSCON_PLLCFG_MSEL_SHIFT) | \
   ((BOARD_PLL0CFG_PSEL-1) << SYSCON_PLLCFG_PSEL_SHIFT))

/* PLL1 : PLL1 is used to generate clock for the USB */

#undef  CONFIG_LPC17_PLL1
//~ #define CONFIG_LPC17_PLL1         1
#define BOARD_PLL1CFG_MSEL        4
#define BOARD_PLL1CFG_PSEL        2
#define BOARD_PLL1CFG_VALUE \
  (((BOARD_PLL1CFG_MSEL-1) << SYSCON_PLLCFG_MSEL_SHIFT) | \
   ((BOARD_PLL1CFG_PSEL-1) << SYSCON_PLLCFG_PSEL_SHIFT))

#if defined(CONFIG_LPC17_USBHOST) || (CONFIG_LPC17_USBDEV)

 /* USB divider.  The output of the PLL is used as the USB clock
 *
 *  USBCLK = PLL1CLK = (SYSCLK * 4)  = 48MHz
 */

#define BOARD_USBCLKSEL_VALUE      (SYSCON_USBCLKSEL_USBDIV_DIV1 | \
                                    SYSCON_USBCLKSEL_USBSEL_PLL1)
#endif

/* FLASH Configuration */

#undef  CONFIG_LPC17_FLASH
#define CONFIG_LPC17_FLASH         1

/* Flash access use 6 CPU clocks - Safe for any allowed conditions */

#define BOARD_FLASHCFG_VALUE       (SYSCON_FLASHCFG_TIM_5 | 0x03a)

/* Ethernet configuration */

#define ETH_MCFG_CLKSEL_DIV        ETH_MCFG_CLKSEL_DIV20

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled 
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(118+2)=400 KHz
 */
  
#define SDCARD_INIT_CLKDIV         (118 << SDCARD_CLOCK_CLKDIV_SHIFT)

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDCARD_MMCXFR_CLKDIV     (1 << SDCARD_CLOCK_CLKDIV_SHIFT) 
#else
#  define SDCARD_MMCXFR_CLKDIV     (2 << SDCARD_CLOCK_CLKDIV_SHIFT) 
#endif

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDCARD_SDXFR_CLKDIV      (1 << SDCARD_CLOCK_CLKDIV_SHIFT)
#else
#  define SDCARD_SDXFR_CLKDIV      (2 << SDCARD_CLOCK_CLKDIV_SHIFT)
#endif

/* Set EMC delay values:
 *
 * CMDDLY: Programmable delay value for EMC outputs in command delayed
 *   mode.  The delay amount is roughly CMDDLY * 250 picoseconds.
 * FBCLKDLY: Programmable delay value for the feedback clock that controls
 *   input data sampling.  The delay amount is roughly (FBCLKDLY+1) * 250
 *   picoseconds.
 * CLKOUT0DLY: Programmable delay value for the CLKOUT0 output. This would
 *   typically be used in clock delayed mode.  The delay amount is roughly
 *  (CLKOUT0DLY+1) * 250 picoseconds.
 * CLKOUT1DLY: Programmable delay value for the CLKOUT1 output. This would
 *  typically be used in clock delayed mode.  The delay amount is roughly
 *  (CLKOUT1DLY+1) * 250 picoseconds.
 *
 * Optimal for NOR: {1,1,1,1}
 * Needed for NAND and SDRAM: {17,1,2,1}
 */

#if defined(CONFIG_LPC17_EMC_NAND) || defined(CONFIG_LPC17_EMC_SDRAM)
#  define BOARD_CMDDLY             17
#  define BOARD_FBCLKDLY           17
#  define BOARD_CLKOUT0DLY         1
#  define BOARD_CLKOUT1DLY         1
#else
#  define BOARD_CMDDLY             1
#  define BOARD_FBCLKDLY           1
#  define BOARD_CLKOUT0DLY         1
#  define BOARD_CLKOUT1DLY         1
#endif

/* LED definitions ******************************************************************/
/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 *
 * LED1 -- Connected to P1[14]
 * LED2 -- Connected to P0[16]
 * LED3 -- Connected to P1[13]
 * LED4 -- Connected to P4[27]
 *
 * These LEDs are connecte to ground so a high output value will illuminate them.
 */

/* LED index values for use with lpc17_setled() */

#define BOARD_LED1                 0
#define BOARD_LED2                 1
#define BOARD_LED3                 2
#define BOARD_LED4                 3
#define BOARD_NLEDS                4

/* LED bits for use with lpc17_setleds() */

#define BOARD_LED1_BIT             (1 << BOARD_LED1)
#define BOARD_LED2_BIT             (1 << BOARD_LED2)
#define BOARD_LED3_BIT             (1 << BOARD_LED3)
#define BOARD_LED4_BIT             (1 << BOARD_LED4)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the four LEDs
 * on the WaveShare Open1788K.  The following definitions describe how NuttX
 * controls the LEDs:
 */
                                      /* LED1 LED2 LED3 LED4                        */
#define LED_STARTED                0  /*  OFF  OFF  OFF  OFF                        */
#define LED_HEAPALLOCATE           1  /*  ON   OFF  OFF  OFF                        */
#define LED_IRQSENABLED            2  /*  OFF   ON  OFF  OFF                        */
#define LED_STACKCREATED           3  /*  ON    ON  OFF  OFF                        */
#define LED_INIRQ                  4  /*  LED3 glows, on while in interupt          */
#define LED_SIGNAL                 4  /*  LED3 glows, on while in signal handler    */
#define LED_ASSERTION              4  /*  LED3 glows, on while in assertion         */
#define LED_PANIC                  4  /*  LED3 Flashes at 2Hz                       */
#define LED_IDLE                   5  /*  LED4 glows, ON while sleeping             */

/* Button definitions ***************************************************************/
/* The Open1788 supports several buttons.  All will read "1" when open and "0"
 * when closed
 *
 * USER1           -- Connected to P4[26]
 * USER2           -- Connected to P2[22]
 * USER3           -- Connected to P0[10]
 *
 * And a Joystick
 *
 * JOY_A           -- Connected to P2[25]
 * JOY_B           -- Connected to P2[26]
 * JOY_C           -- Connected to P2[23]
 * JOY_D           -- Connected to P2[19]
 * JOY_CTR         -- Connected to P0[14]
 *
 * The switches are all connected to ground and should be pulled up and sensed
 * with a value of '0' when closed.
 */

#define BOARD_BUTTON_USER1         0
#define BOARD_BUTTON_USER2         1
#define BOARD_BUTTON_USER3         2

#define BOARD_JOYSTICK_A           3
#define BOARD_JOYSTICK_B           4
#define BOARD_JOYSTICK_C           5
#define BOARD_JOYSTICK_D           6
#define BOARD_JOYSTICK_CTR         7

#define BOARD_NUM_BUTTONS          8

#define BOARD_BUTTON_USER1_BIT     (1 << BOARD_BUTTON_USER1)
#define BOARD_BUTTON_USER2_BIT     (1 << BOARD_BUTTON_USER2)
#define BOARD_BUTTON_USER3_BIT     (1 << BOARD_BUTTON_USER3)

#define BOARD_JOYSTICK_A_BIT       (1 << BOARD_JOYSTICK_A)
#define BOARD_JOYSTICK_B_BIT       (1 << BOARD_JOYSTICK_B)
#define BOARD_JOYSTICK_C_BIT       (1 << BOARD_JOYSTICK_C)
#define BOARD_JOYSTICK_D_BIT       (1 << BOARD_JOYSTICK_D)
#define BOARD_JOYSTICK_CTR_BIT     (1 << BOARD_JOYSTICK_CTR)

/* Alternate pin selections *********************************************************/

#define GPIO_UART0_TXD             GPIO_UART0_TXD_2
#define GPIO_UART0_RXD             GPIO_UART0_RXD_2

#define GPIO_SD_DAT0               GPIO_SD_DAT0_1 /* REVISIT */
#define GPIO_SD_DAT1               GPIO_SD_DAT1_1
#define GPIO_SD_DAT2               GPIO_SD_DAT2_1
#define GPIO_SD_DAT3               GPIO_SD_DAT3_1
#define GPIO_SD_CLK                GPIO_SD_CLK_1
#define GPIO_SD_CMD                GPIO_SD_CMD_1

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

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
 * Name: lpc17_boardinitialize
 *
 * Description:
 *   All LPC17xx architectures must provide the following entry point.  This entry
 *   point is called early in the intitialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void lpc17_boardinitialize(void);

/************************************************************************************
 * Name:  lpc17_ledinit, lpc17_setled, and lpc17_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LEDs.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfacesare available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
EXTERN void lpc17_ledinit(void);
EXTERN void lpc17_setled(int led, bool ledon);
EXTERN void lpc17_setleds(uint8_t ledset);
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
EXTERN void up_buttoninit(void);

/************************************************************************************
 * Name: up_buttons
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After that,
 *   up_buttons() may be called to collect the current state of all buttons.
 *
 *   After up_buttoninit() has been called, up_buttons() may be called to collect
 *   the state of all buttons.  up_buttons() returns an 8-bit bit set with each bit
 *   associated with a button.  See the BOARD_BUTTON_*_BIT and BOARD_JOYSTICK_*_BIT
 *   definitions above for the meaning of each bit.
 *
 ************************************************************************************/

EXTERN uint8_t up_buttons(void);

/************************************************************************************
 * Button support.
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After that,
 *   up_irqbutton() may be called to register button interrupt handlers.
 *
 *   up_irqbutton() may be called to register an interrupt handler that will be called
 *   when a button is depressed or released.  The ID value is a button enumeration
 *   value that uniquely identifies a button resource. See the BOARD_BUTTON_* and
 *   BOARD_JOYSTICK_* definitions in above for the meaning of enumeration values
 *   The previous interrupt handler address is returned (so that it may restored, if
 *   so desired).
 *
 *   Note that up_irqbutton() also enables button interrupts.  Button interrupts
 *   will remain enabled after the interrupt handler is attached. Interrupts may
 *   be disabled (and detached) by calling up_irqbutton with irqhandler equal to
 *   NULL.
 *
 ************************************************************************************/

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_GPIO_IRQ)
EXTERN xcpt_t up_irqbutton(int id, xcpt_t irqhandler);
#endif
#endif /* CONFIG_ARCH_BUTTONS */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_OPEN1788_INCLUDE_BOARD_H */
