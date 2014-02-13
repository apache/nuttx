/************************************************************************************
 * configs/samd20-xplained/src/samd20-xplained.h
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

#ifndef __CONFIGS_SAMD20_XPLAINED_SRC_SAMD20_XPLAINED_H
#define __CONFIGS_SAMD20_XPLAINED_SRC_SAMD20_XPLAINED_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "sam_config.h"
#include "chip/sam_pinmap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* LEDs: There are three LEDs on board the SAMD20 Xplained Pro board:  The EDBG
 * controls two of the LEDs, a power LED and a status LED.  There is only
 * one user controllable LED, a yellow LED labelled STATIS near the SAMD20 USB
 * connector.
 *
 * This LED is controlled by PA14 and the LED can be activated by driving PA14
 * to GND.
 *
 * When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
 * control the LED as follows:
 *
 *   SYMBOL              Meaning                 LED
 *   ------------------- ----------------------- ------
 *   LED_STARTED         NuttX has been started  OFF
 *   LED_HEAPALLOCATE    Heap has been allocated OFF
 *   LED_IRQSENABLED     Interrupts enabled      OFF
 *   LED_STACKCREATED    Idle stack created      ON
 *   LED_INIRQ           In an interrupt         N/C
 *   LED_SIGNAL          In a signal handler     N/C
 *   LED_ASSERTION       An assertion failed     N/C
 *   LED_PANIC           The system has crashed  FLASH
 *
 * Thus if the LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If the LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

#define GPIO_STATUS_LED (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_SET | \
                         GPIO_PORTA | GPIO_PIN14)

/* Mechanical buttons:
 *
 * The SAMD20 Xplained Pro contains two mechanical buttons. One button is the
 * RESET button connected to the SAMD20 reset line and the other is a generic user
 * configurable button. When a button is pressed it will drive the I/O line to GND.
 *
 *   PA15 SW0
 */

#define GPIO_SW0      (GPIO_INTERRUPT | GPIO_PULL_UP | GPIO_GLITCH_FILTER | \
                       GPIO_PORTA | GPIO_PIN15)
#define IRQ_SW0       SAM_IRQ_PA15

/* I/O1
 *
 * Support for the microSD card slot on the I/O1 module.  The I/O1 requires
 * SPI support and two GPIOs.    These the GPIOs will vary if the I/O1
 * is installed on the EXT1 or EXT2 connector:
 *
 *   --- ------------------ ---------------------- -------------------------------
 *   PIN EXT1               EXT2                   Description
 *   --- ------------------ ---------------------- -------------------------------
 *   15 PA05 SERCOM0 PAD[1] 15 PA17 SERCOM1 PAD[1]  Active low chip select OUTPUT,
 *           SPI SS                 SPI SS          pulled high on board.
 *   --- ------------------ ---------------------- -------------------------------
 *   10 PB05 GPIO           10 PB15 GPIO            Active low card detect INPUT,
 *                                                  must use internal pull-up.
 *   --- ------------------ ---------------------- -------------------------------
 */

#ifdef CONFIG_SAMD20_XPLAINED_IOMODULE

#  ifndef SAMD_HAVE_SPI0
#    error SAMD_HAVE_SPI0 is required to use the I/O1 module
#  endif

#  if defined(CONFIG_SAMD20_XPLAINED_IOMODULE_EXT1)

#    if defined(CONFIG_SAMD20_XPLAINED_OLED1MODULE) && \
        defined(CONFIG_SAMD20_XPLAINED_OLED1MODULE_EXT1)
#      error I/O1 and OLED1 modules cannot both reside in EXT1
#    endif

#    define GPIO_SD_CD (GPIO_INTERRUPT | GPIO_INT_CHANGE | GPIO_PULL_UP | \
                        GPIO_GLITCH_FILTER | GPIO_PORTF | GPIO_PIN5)
#    define IRQ_SD_CD   SAM_IRQ_PB5

#    define GPIO_SD_CS (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_SET | \
                        GPIO_PORTA | GPIO_PIN5)
#    define SD_CSNO    0

#  elif defined(CONFIG_SAMD20_XPLAINED_IOMODULE_EXT2)

#    if defined(CONFIG_SAMD20_XPLAINED_OLED1MODULE) && \
        defined(CONFIG_SAMD20_XPLAINED_OLED1MODULE_EXT2)
#      error I/O1 and OLED1 modules cannot both reside in EXT2
#    endif

#    define GPIO_CD   (GPIO_INTERRUPT | GPIO_INT_CHANGE | GPIO_PULL_UP | \
                       GPIO_GLITCH_FILTER | GPIO_PORTB | GPIO_PIN15)
#    define IRQ_CD     SAM_IRQ_PB15

#    define GPIO_SD_CS (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_SET | \
                        GPIO_PORTA | GPIO_PIN17)
#    define SD_CSNO    2

#  else
#    error Which connector is the I/O1 module installed in?
#  endif
#endif

/* OLED1
 *
 * Support for the microSD card slot on the I/O1 module.  The I/O1 requires
 * SPI support and three output GPIOs.  These the GPIOs will vary if the OLED1
 * is installed on the EXT1 or EXT2 connector:
 *
 *
 *   PIN EXT1                EXT2                 Description
 *   --- ------------------- -------------------- -------------------------------------
 *   5   PB06 GPIO           PA20 GPIO            DATA_CMD_SEL
 *   10  PB05 GPIO           PB15 GPIO            DISPLAY_RESET. Active low.
 *   15  PA05 SERCOM0 PAD[1] PA17 SERCOM1 PAD[1]  DISPLAY_SS.  Active low.
 *            SPI SS              SPI SS
 */

#ifdef CONFIG_SAMD20_XPLAINED_OLED1MODULE

#  ifndef SAMD_HAVE_SPI0
#    error SAMD_HAVE_SPI0 is required to use the OLED1 module
#  endif

#  ifndef CONFIG_SPI_CMDDATA
#    error CONFIG_SPI_CMDDATA is required to use the OLED1 module
#  endif

#  ifndef CONFIG_LCD_SSD1306
#    error CONFIG_LCD_SSD1306 is required to use the OLED1 module
#  endif

#  ifndef CONFIG_LCD_UG2832HSWEG04
#    error CONFIG_LCD_UG2832HSWEG04 is required to use the OLED1 module
#  endif

#  if defined(CONFIG_SAMD20_XPLAINED_OLED1MODULE_EXT1)

#    if defined(CONFIG_SAMD20_XPLAINED_IOMODULE) && \
        defined(CONFIG_SAMD20_XPLAINED_IOMODULE_EXT1)
#      error OLED1 and I/O1 modules cannot both reside in EXT1
#    endif

#    define GPIO_OLED_DATA (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTB | GPIO_PIN6)
#    define GPIO_OLED_RST  (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTB | GPIO_PIN5)
#    define GPIO_OLED_CS   (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_SET | \
                            GPIO_PORTA | GPIO_PIN5)
#    define OLED_CSNO       0

#  elif defined(CONFIG_SAMD20_XPLAINED_OLED1MODULE_EXT2)

#    if defined(CONFIG_SAMD20_XPLAINED_IOMODULE) && \
        defined(CONFIG_SAMD20_XPLAINED_IOMODULE_EXT2)
#      error OLED1 and I/O1 modules cannot both reside in EXT2
#    endif

#    define GPIO_OLED_DATA (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTA | GPIO_PIN20)
#    define GPIO_OLED_RST  (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTB | GPIO_PIN15)
#    define GPIO_OLED_CS   (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_SET | \
                            GPIO_PORTA | GPIO_PIN17)
#    define OLED_CSNO      2

#  else
#    error Which connector is the OLED1 module installed in?
#  endif
#endif

#if defined(CONFIG_LCD_UG2864AMBAG01) || defined(CONFIG_LCD_UG2864HSWEG01)
#    define GPIO_SD_CS (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_SET | \
                        GPIO_PORTB | GPIO_PIN11) /* REVISIT */
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAM3U-EK board.
 *
 ************************************************************************************/

void weak_function sam_spiinitialize(void);

/************************************************************************************
 * Name: sam_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.  Requires CONFIG_SAMD20_XPLAINED_IOMODULE=y,
 *   CONFIG_DISABLE_MOUNTPOINT=n, CONFIG_MMCSD=y, and SAMD_HAVE_SPI0
 *
 ************************************************************************************/

#if defined(SAMD_HAVE_SPI0) && defined(CONFIG_SAMD20_XPLAINED_IOMODULE)
int sam_sdinitialize(int minor);
#endif

/************************************************************************************
 * Name: board_led_initialize
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_led_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SAMD20_XPLAINED_SRC_SAMD20_XPLAINED_H */

