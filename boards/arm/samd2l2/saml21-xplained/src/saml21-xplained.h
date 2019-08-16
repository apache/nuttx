/****************************************************************************
 * boards/arm/samd2l2/saml21-xplained/src/saml21-xplained.h
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
 ****************************************************************************/

#ifndef __BOARDS_ARM_SAMD2L2_SAML21_XPLAINED_SRC_SAML21_XPLAINED_H
#define __BOARDS_ARM_SAMD2L2_SAML21_XPLAINED_SRC_SAML21_XPLAINED_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "sam_config.h"
#include "sam_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs: There are three LEDs on board the SAML21 Xplained Pro board:
 * The EDBG controls two of the LEDs, a power LED and a status LED.
 * There is only one user controllable LED, a yellow LED labelled STATIS
 * near the SAML21 USB connector.
 *
 * This LED is controlled by PB10 and the LED can be activated by driving
 * PB10 to GND.
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

#define PORT_STATUS_LED (PORT_OUTPUT | PORT_PULL_NONE | PORT_OUTPUT_SET | \
                         PORTB | PORT_PIN10)

/* Mechanical buttons:
 *
 * The SAML21 Xplained Pro contains two mechanical buttons. One button is the
 * RESET button connected to the SAML21 reset line and the other is a generic
 * user configurable button.
 * When a button is pressed it will drive the I/O line to GND.
 *
 *   PA02 SW0
 */

#define PORT_SW0      (PORT_INTERRUPT | PORT_PULL_UP | PORTA | PORT_PIN2)
#define IRQ_SW0       SAM_IRQ_PA2

/* I/O1
 *
 * Support for the microSD card slot on the I/O1 module.  The I/O1 requires
 * SPI support and two PORTs.    These the PORTs will vary if the I/O1
 * is installed on the EXT1 or EXT2 connector:
 *
 *   --- ------------------ ---------------------- --------------------------
 *   PIN EXT1               EXT2                   Description
 *   --- ------------------ ---------------------- --------------------------
 *   15 PA05 SERCOM0 PAD[1] 15 PA17 GPIO]          Active low chip select
 *                             SPI SS              OUTPUT, pulled high on
 *                                                 board.
 *   10 PA02 GPIO           10 PB15 GPIO           Active low card detect
 *                                                 INPUT, must use internal
 *                                                 pull-up.
 *   --- ------------------ ---------------------- --------------------------
 */

#ifdef CONFIG_SAML21_XPLAINED_IOMODULE

#  ifndef SAMD2L2_HAVE_SPI0
#    error SAMD2L2_HAVE_SPI0 is required to use the I/O1 module
#  endif

#  if defined(CONFIG_SAML21_XPLAINED_IOMODULE_EXT1)

#    if defined(CONFIG_SAML21_XPLAINED_OLED1MODULE) && \
        defined(CONFIG_SAML21_XPLAINED_OLED1MODULE_EXT1)
#      error I/O1 and OLED1 modules cannot both reside in EXT1
#    endif

#    define PORT_SD_CD (PORT_INTERRUPT | PORT_INT_CHANGE | PORT_PULL_UP | \
                        PORTA | PORT_PIN2)

#    define PORT_SD_CS (PORT_OUTPUT | PORT_PULL_NONE | PORT_OUTPUT_SET | \
                        PORTA | PORT_PIN5)

#  elif defined(CONFIG_SAML21_XPLAINED_IOMODULE_EXT2)

#    if defined(CONFIG_SAML21_XPLAINED_OLED1MODULE) && \
        defined(CONFIG_SAML21_XPLAINED_OLED1MODULE_EXT2)
#      error I/O1 and OLED1 modules cannot both reside in EXT2
#    endif

#    define PORT_SD_CD (PORT_INTERRUPT | PORT_INT_CHANGE | PORT_PULL_UP | \
                        PORTB | PORT_PIN15)

#    define PORT_SD_CS (PORT_OUTPUT | PORT_PULL_NONE | PORT_OUTPUT_SET | \
                        PORTA | PORT_PIN17)

#  else
#    error Which connector is the I/O1 module installed in?
#  endif
#endif

/* OLED1
 *
 * Support for the microSD card slot on the I/O1 module.
 * The I/O1 requires SPI support and three output PORTs.
 * These the PORTs will vary if the OLED1 is installed on the EXT1
 * or EXT2 connector:
 *
 *   --- ------------------- -------------------- ---------------------------
 *   PIN EXT1                EXT2                 Description
 *   --- ------------------- -------------------- ---------------------------
 *   5   PB06 PA02           PA20 PA02            DATA_CMD_SEL
 *   10  PA02 PA02           PB15 PA02            DISPLAY_RESET. Active low.
 *   15  PA05 SERCOM0 PAD[1] PA17 SERCOM1 PAD[1]  DISPLAY_SS.  Active low.
 *            SPI SS              SPI SS
 *   --- ------------------- -------------------- ---------------------------
 */

#ifdef CONFIG_SAML21_XPLAINED_OLED1MODULE

#  ifndef SAMD2L2_HAVE_SPI0
#    error SAMD2L2_HAVE_SPI0 is required to use the OLED1 module
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

#  if defined(CONFIG_SAML21_XPLAINED_OLED1MODULE_EXT1)

#    if defined(CONFIG_SAML21_XPLAINED_IOMODULE) && \
        defined(CONFIG_SAML21_XPLAINED_IOMODULE_EXT1)
#      error OLED1 and I/O1 modules cannot both reside in EXT1
#    endif

#    define PORT_OLED_DATA (PORT_OUTPUT | PORT_PULL_NONE | PORT_OUTPUT_CLEAR | \
                            PORTB | PORT_PIN6)
#    define PORT_OLED_RST  (PORT_OUTPUT | PORT_PULL_NONE | PORT_OUTPUT_CLEAR | \
                            PORTA | PORT_PIN2)
#    define PORT_OLED_CS   (PORT_OUTPUT | PORT_PULL_NONE | PORT_OUTPUT_SET | \
                            PORTA | PORT_PIN5)

#  elif defined(CONFIG_SAML21_XPLAINED_OLED1MODULE_EXT2)

#    if defined(CONFIG_SAML21_XPLAINED_IOMODULE) && \
        defined(CONFIG_SAML21_XPLAINED_IOMODULE_EXT2)
#      error OLED1 and I/O1 modules cannot both reside in EXT2
#    endif

#    define PORT_OLED_DATA (PORT_OUTPUT | PORT_PULL_NONE | PORT_OUTPUT_CLEAR | \
                            PORTA | PORT_PIN20)
#    define PORT_OLED_RST  (PORT_OUTPUT | PORT_PULL_NONE | PORT_OUTPUT_CLEAR | \
                            PORTB | PORT_PIN15)
#    define PORT_OLED_CS   (PORT_OUTPUT | PORT_PULL_NONE | PORT_OUTPUT_SET | \
                            PORTA | PORT_PIN17)

#  else
#    error Which connector is the OLED1 module installed in?
#  endif
#endif

#if defined(CONFIG_LCD_UG2864AMBAG01) || defined(CONFIG_LCD_UG2864HSWEG01)
#    define PORT_SD_CS (PORT_OUTPUT | PORT_PULL_NONE | PORT_OUTPUT_SET | \
                        PORTB | PORT_PIN11) /* REVISIT */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select PORT pins for the SAM3U-EK board.
 *
 ****************************************************************************/

void weak_function sam_spidev_initialize(void);

/****************************************************************************
 * Name: sam_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.
 *   Requires CONFIG_SAML21_XPLAINED_IOMODULE=y,
 *   CONFIG_DISABLE_MOUNTPOINT=n, CONFIG_MMCSD=y,
 *   and the appropriate SERCOM SPI port enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SAML21_XPLAINED_IOMODULE
int sam_sdinitialize(int port, int minor);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMD2L2_SAML21_XPLAINED_SRC_SAML21_XPLAINED_H */
