/****************************************************************************
 * configs/pic32mz-starterkit/src/pic32mz-starterkit.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __CONFIGS_PIC32MZ_STARTERKIT_SRC_PIC32MZ_STARTERKIT_H
#define __CONFIGS_PIC32MZ_STARTERKIT_SRC_PIC32MZ_STARTERKIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Assume that we have MMC/SD */

#define PIC32MZ_HAVE_MMCSD   1

/* The PIC32 Ethernet Starter Kit does not have an SD slot on board.  If one
 * is added, then it must be specified by defining which SPI bus that it
 * is connected on.
 */

#ifndef CONFIG_PIC32MZ_MMCSDSPIPORTNO
#  undef PIC32MZ_HAVE_MMCSD
#endif

#define PIC32MZ_MMCSDSPIPORTNO CONFIG_PIC32MZ_MMCSDSPIPORTNO
#define PIC32MZ_MMCSDSLOTNO    0
#define PIC32MZ_MMCSDMINOR     0

/* Assume /dev/mmcsd0 */

#define PIC32MZ_MMCSDMINOR 0

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef PIC32MZ_HAVE_MMCSD
#endif

/* Make sure that the NSH configuration will support the SD card */

#if defined(PIC32MZ_HAVE_MMCSD) && defined(CONFIG_NSH_ARCHINIT)

   /* Make sure that the NSH configuration uses the correct SPI */

#  if !defined(CONFIG_NSH_MMCSDSPIPORTNO)
#    define CONFIG_NSH_MMCSDSPIPORTNO PIC32MZ_MMCSDSPIPORTNO
#  elif CONFIG_NSH_MMCSDSPIPORTNO != PIC32MZ_MMCSDSPIPORTNO
#    warning "CONFIG_PIC32MZ_MMCSDSPIPORTNO does not match CONFIG_NSH_MMCSDSPIPORTNO"
#    undef CONFIG_NSH_MMCSDSPIPORTNO
#    define CONFIG_NSH_MMCSDSPIPORTNO PIC32MZ_MMCSDSPIPORTNO
#  endif

   /* Make sure that the NSH configuration uses the slot */

#  if !defined(CONFIG_NSH_MMCSDSLOTNO) || CONFIG_NSH_MMCSDSLOTNO != 0
#    warning "The PIC32 Starter Kit has only one slot (0)"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif

   /* Make sure that the correct SPI is enabled in the configuration */

#  if PIC32MZ_MMCSDSPIPORTNO == 1 && !defined(CONFIG_PIC32MZ_SPI1)
#    warning "CONFIG_PIC32MZ_SPI1 is not enabled"
#    undef PIC32MZ_HAVE_MMCSD
#  elif PIC32MZ_MMCSDSPIPORTNO == 2 && !defined(CONFIG_PIC32MZ_SPI2)
#    warning "CONFIG_PIC32MZ_SPI2 is not enabled"
#    undef PIC32MZ_HAVE_MMCSD
#  elif PIC32MZ_MMCSDSPIPORTNO == 3 && !defined(CONFIG_PIC32MZ_SPI3)
#    warning "CONFIG_PIC32MZ_SPI3 is not enabled"
#    undef PIC32MZ_HAVE_MMCSD
#  elif PIC32MZ_MMCSDSPIPORTNO == 4 && !defined(CONFIG_PIC32MZ_SPI4)
#    warning "CONFIG_PIC32MZ_SPI4 is not enabled"
#    undef PIC32MZ_HAVE_MMCSD
#  endif

 /* Use the minor number selected in the NSH configuration */

#  ifdef CONFIG_NSH_MMCSDMINOR
#    define PIC32MZ_MMCSDMINOR CONFIG_NSH_MMCSDMINOR
#  endif
#endif

/* LEDs *********************************************************************/
/* The PIC32 starter kit has 3 user LEDs
 *
 *   PIN  LED   Notes
 *   ---  ----- -------------------------
 *   RH0  LED1  High illuminates (RED)
 *   RH1  LED3  High illuminates (YELLOW)
 *   RH2  LED2  High illuminates (GREEN)
 */

#define GPIO_LED_1  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTH | GPIO_PIN0)
#define GPIO_LED_2  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTH | GPIO_PIN1)
#define GPIO_LED_3  (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTH | GPIO_PIN2)

/* The PIC32MZ Ethernet Starter kit has 3 user push buttons labelled SW1-3
 * on the board:
 *
 *   PIN   LED  Notes
 *   ----  ---- -------------------------
 *   RB12  SW1  Active-low
 *   RB13  SW2  Active-low
 *   RB14  SW3  Active-low
 *
 * The switches do not have any debounce circuitry and require internal pull-
 * up resistors. When Idle, the switches are pulled high (+3.3V), and they
 * are grounded when pressed.
 */

#define GPIO_SW_1   (GPIO_INPUT | GPIO_INTERRUPT | GPIO_PULLUP | \
                     GPIO_PORTB | GPIO_PIN12)
#define GPIO_SW_2   (GPIO_INPUT | GPIO_INTERRUPT | GPIO_PULLUP | \
                     GPIO_PORTB | GPIO_PIN13)
#define GPIO_SW_3   (GPIO_INPUT | GPIO_INTERRUPT | GPIO_PULLUP | \
                     GPIO_PORTB | GPIO_PIN14)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Name: pic32mz_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PCB Logic board.
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MZ_SPI
void weak_function pic32mz_spiinitialize(void);
#endif

/************************************************************************************
 * Name: pic32mz_ledinit
 *
 * Description:
 *   Configure on-board LEDs if LED support has been selected.
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void pic32mz_ledinit(void);
#endif

/****************************************************************************
 * Name: pic32mz_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int pic32mz_bringup(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_PIC32MZ_STARTERKIT_SRC_PIC32MZ_STARTERKIT_H */
