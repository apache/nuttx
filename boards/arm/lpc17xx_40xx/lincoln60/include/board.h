/****************************************************************************
 * boards/arm/lpc17xx_40xx/lincoln60/include/board.h
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

#ifndef __BOARDS_ARM_LPC17XX_40XX_LINCOLN60_INCLUDE_BOARD_H
#define __BOARDS_ARM_LPC17XX_40XX_LINCOLN60_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_LPC17_40_GPIOIRQ)
#  include <nuttx/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* NOTE:  The following definitions require lpc17_40_syscon.h.
 * It is not included here because the including C file may not have that
 * file in its include path.
 */

#define BOARD_XTAL_FREQUENCY        (12000000)            /* XTAL oscillator frequency */
#define BOARD_OSCCLK_FREQUENCY      BOARD_XTAL_FREQUENCY  /* Main oscillator frequency */
#define BOARD_RTCCLK_FREQUENCY      (32768)               /* RTC oscillator frequency */
#define BOARD_INTRCOSC_FREQUENCY    (4000000)             /* Internal RC oscillator frequency */

/* This is the clock setup we configure for:
 *
 *   SYSCLK = BOARD_OSCCLK_FREQUENCY = 12MHz  -> Main oscillator for source
 *   PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz -> multipler=20, pre-divider=1
 *   CCLCK = 480MHz / 6 = 80MHz               -> divider = 6
 */

#define LPC17_40_CCLK                 80000000 /* 80Mhz*/

/* Select the main oscillator as the frequency source.  SYSCLK is then the
 * frequency of the main oscillator.
 */

#undef CONFIG_LPC17_40_MAINOSC
#define CONFIG_LPC17_40_MAINOSC       1
#define BOARD_SCS_VALUE            SYSCON_SCS_OSCEN

/* Select the main oscillator and CCLK divider. The output of the divider is
 * CCLK. The input to the divider (PLLCLK) will be determined by the PLL
 * output.
 */

#define BOARD_CCLKCFG_DIVIDER      6
#define BOARD_CCLKCFG_VALUE        ((BOARD_CCLKCFG_DIVIDER-1) << SYSCON_CCLKCFG_SHIFT)

/* PLL0.  PLL0 is used to generate the CPU clock divider input (PLLCLK).
 *
 *  Source clock:               Main oscillator
 *  PLL0 Multiplier value (M):  20
 *  PLL0 Pre-divider value (N): 1
 *
 *  PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz
 */

#undef CONFIG_LPC17_40_PLL0
#define CONFIG_LPC17_40_PLL0          1
#define BOARD_CLKSRCSEL_VALUE      SYSCON_CLKSRCSEL_MAIN

#define BOARD_PLL0CFG_MSEL         20
#define BOARD_PLL0CFG_NSEL         1
#define BOARD_PLL0CFG_VALUE \
  (((BOARD_PLL0CFG_MSEL-1) << SYSCON_PLL0CFG_MSEL_SHIFT) | \
   ((BOARD_PLL0CFG_NSEL-1) << SYSCON_PLL0CFG_NSEL_SHIFT))

/* PLL1 -- Not used. */

#undef CONFIG_LPC17_40_PLL1
#define BOARD_PLL1CFG_MSEL         36
#define BOARD_PLL1CFG_NSEL         1
#define BOARD_PLL1CFG_VALUE \
  (((BOARD_PLL1CFG_MSEL-1) << SYSCON_PLL1CFG_MSEL_SHIFT) | \
   ((BOARD_PLL1CFG_NSEL-1) << SYSCON_PLL1CFG_NSEL_SHIFT))

/* USB divider.  This divider is used when PLL1 is not enabled to get the
 * USB clock from PLL0:
 *
 *  USBCLK = PLL0CLK / 10 = 48MHz
 */

#define BOARD_USBCLKCFG_VALUE      SYSCON_USBCLKCFG_DIV10

/* FLASH Configuration */

#undef  CONFIG_LPC17_40_FLASH
#define CONFIG_LPC17_40_FLASH         1
#define BOARD_FLASHCFG_VALUE       0x0000303a

/* Ethernet configuration */

#define ETH_MCFG_CLKSEL_DIV ETH_MCFG_CLKSEL_DIV20

/* Clocking *****************************************************************/

/* The Lincoln 60 has 2 LEDs along the bottom of the board. Green or off.
 * If CONFIG_ARCH_LEDS is defined, the LEDs will be controlled as follows
 * for NuttX debug functionality (where NC means "No Change").
 *
 * During the boot phases.  LED1 and LED2 will show boot status.
 */

                                      /* LED1   LED2    */
#define LED_STARTED                0  /* OFF    OFF     */
#define LED_HEAPALLOCATE           1  /* GREEN  OFF     */
#define LED_IRQSENABLED            2  /* OFF    GREEN   */
#define LED_STACKCREATED           3  /* OFF    OFF     */

/* After the system is booted, this logic will no longer use LEDs 1 & 2.
 * They are available for use the application software using lpc17_40_led
 * (prototyped below)
 */

                                      /* LED1   LED2   LED3 LED4 */
#define LED_INIRQ                  4  /*  NC     NC    NC   ON  (momentary) */
#define LED_SIGNAL                 5  /*  NC     NC    NC   ON  (momentary) */
#define LED_ASSERTION              6  /*  NC     NC    NC   ON  (momentary) */
#define LED_PANIC                  7  /*  NC     NC    NC   ON  (1Hz flashing) */

#define GPIO_SSP0_SCK              GPIO_SSP0_SCK_1
#define GPIO_SSP0_SSEL             GPIO_SSP0_SSEL_1
#define GPIO_SSP0_MISO             GPIO_SSP0_MISO_1
#define GPIO_SSP0_MOSI             GPIO_SSP0_MOSI_1

/* We need to redefine USB_PWRD as GPIO to get USB Host working
 * Also remember to add 2 resistors of 15K to D+ and D- pins.
 */

#ifdef CONFIG_USBHOST
#  ifdef GPIO_USB_PWRD
#    undef  GPIO_USB_PWRD
#    define GPIO_USB_PWRD          (GPIO_INPUT | GPIO_PORT1 | GPIO_PIN22)
#  endif
#endif

/* Ethernet PHY */

#define GPIO_ENET_MDC              GPIO_ENET_MDC_1
#define GPIO_ENET_MDIO             GPIO_ENET_MDIO_1

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

/****************************************************************************
 * Name: lpc17_40_led
 *
 * Description:
 *   Once the system has booted, these functions can be used to control
 *   LEDs 1 & 2
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void lpc17_40_led(int lednum, int state);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LPC17XX_40XX_LINCOLN60_INCLUDE_BOARD_H */
