/****************************************************************************
 * boards/arm/sam34/sam4l-xplained/src/sam4l-xplained.h
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

#ifndef __BOARDS_ARM_SAM34_SAM4L_XPLAINED_SRC_SAM4L_XPLAINED_H
#define __BOARDS_ARM_SAM34_SAM4L_XPLAINED_SRC_SAM4L_XPLAINED_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "hardware/sam_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs: There are three LEDs on board the SAM4L Xplained Pro board:
 * The EDBG controls two of the LEDs, a power LED and a status LED.
 * There is only one user controllable LED, a yellow LED labeled LED0 near
 * the SAM4L USB connector.
 *
 * This LED is controlled by PC07 and LED0 can be activated by driving the
 * PC07 to GND.
 *
 * When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
 * control LED0 as follows:
 *
 *   SYMBOL              Meaning                 LED0
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
 * Thus is LED0 is statically on, NuttX has successfully  booted and is,
 * apparently, running normmally.  If LED0 is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

#define GPIO_LED0     (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_SET | \
                       GPIO_PORTC | GPIO_PIN7)

/* QTouch button: The SAM4L Xplained Pro kit has one QTouch button.
 * The connection to the SAM4L is:
 *
 *   PC13 CATB_SENSE15
 *   PC14 CATB_DIS
 */

/* Mechanical buttons:
 *
 * The SAM4L Xplained Pro contains two mechanical buttons.
 * One button is the RESET button connected to the SAM4L reset line and the
 * other is a generic user configurable button.
 * When a button is pressed it will drive the I/O line to GND.
 *
 *   PC24 SW0
 */

#define GPIO_SW0      (GPIO_INTERRUPT | GPIO_PULL_UP | GPIO_GLITCH_FILTER | \
                       GPIO_PORTC | GPIO_PIN24)
#define IRQ_SW0       SAM_IRQ_PC24

/* LCD1
 *
 * EXT5  SAM4L BOARD     LCD1     SHARED
 * PIN    PIN  FUNCTION  FUNCTION WITH
 *  1    PA09  COM3      COM3     EXT3
 *  2    PA10  COM2      COM2     EXT3
 *  3    PA11  COM1      COM1     EXT4
 *  4    PA12  COM0      COM0     EXT4
 *  5    PC15  SEG0      SEG0     EXT3
 *  6    PC16  SEG1      SEG1     EXT3
 *  7    PC17  SEG2      SEG2     EXT4
 *  8    PC18  SEG3      SEG3     EXT4
 *  9    PC19  SEG4      SEG4
 *  10   PA13  SEG5      SEG5     EXT4
 *  11   PA14  SEG6      SEG6
 *  12   PA15  SEG7      SEG7     EXT4
 *  13   PA16  SEG8      SEG8     EXT4
 *  14   PA17  SEG9      SEG9     EXT3
 *  15   PC20  SEG10     SEG10
 *  16   PC21  SEG11     SEG11
 *  17   PC22  SEG12     SEG12
 *  18   PC23  SEG13     SEG13
 *  19   PB08  SEG14     SEG14
 *  20   PB09  SEG15     SEG15
 *  21   PB10  SEG16     SEG16    EXT2
 *  22   PB11  SEG17     SEG17    EXT2
 *  23   PA18  SEG18     SEG18    EXT3-4
 *  24   PA19  SEG19     SEG19    EXT3-4
 *  25   PA20  SEG20     SEG20    EXT3-4
 *  26   PB07  SEG21     SEG21
 *  27   PB06  SEG22     SEG22
 *  28   PA08  SEG23     SEG32    EXT3
 *  29   PC24  SEG24     N/C
 *  30   PC25  SEG25     N/C      EXT1
 *  31   PC26  SEG26     N/C      EXT2-3
 *  32   PC27  SEG27     N/C      EXT2-3
 *  33   PC28  SEG28     N/C
 *  34   PC29  SEG29     N/C
 *  35   PC30  SEG30     N/C      EXT1-2
 *  36   PC31  SEG31     N/C
 *  37   PB12  SEG32     N/C      EXT1
 *  38   PB13  SEG33     N/C      EXT1
 *  39   PA21  SEG34     N/C      EXT1-2
 *  40   PA22  SEG35     N/C      EXT1-2
 *  41   PB14  SEG36     N/C      EXT2-4
 *  42   PB15  SEG37     N/C      EXT2-4
 *  43   PA23  SEG38     N/C      EXT1
 *  44   PA24  SEG39     N/C      EXT1
 *  45   ---   N/C       N/C
 *  46   ---   N/C       N/C
 *  47   ---   VCC_P3V3  BL V+
 *  48   ---   GND       BL V-
 *  49   PC05  BL        BL CTRL  EXT2
 *  50   ---   ID        ID
 *  51   ---   GND       GND
 *
 * The backlight control is active high.
 */

#ifdef CONFIG_SAM4L_XPLAINED_SLCD1MODULE

#  ifndef CONFIG_SAM34_LCDCA
#    error CONFIG_SAM34_LCDCA is required to use the LCD1 module
#  endif

#  define GPIO_LCD1_BL (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_CLEAR | \
                        GPIO_PORTC | GPIO_PIN5)
#endif

/* I/O1
 *
 * Support for the microSD card slot on the I/O1 module.
 * The I/O1 requires SPI support and two GPIOs.
 * These the GPIOs will vary if the I/O1 is installed on the EXT1 or EXT2
 * connector:
 *
 *
 *   PIN EXT1           EXT2            Description
 *   --- -------------- --------------- -------------------------------------
 *   15  PC03 SPI/NPCS0 PB11 SPI/NPCS2  Active low chip select OUTPUT, pulled
 *                                      high on board.
 *   10  PB13 SPI/NPCS1 PC09 GPIO       Active low card detect INPUT, must
 *                                      use internal pull-up.
 */

#ifdef CONFIG_SAM4L_XPLAINED_IOMODULE

#  ifndef CONFIG_SAM34_SPI0
#    error CONFIG_SAM34_SPI0 is required to use the I/O1 module
#  endif

#  if defined(CONFIG_SAM4L_XPLAINED_IOMODULE_EXT1)

#    if defined(CONFIG_SAM4L_XPLAINED_OLED1MODULE) && \
        defined(CONFIG_SAM4L_XPLAINED_OLED1MODULE_EXT1)
#      error I/O1 and OLED1 modules cannot both reside in EXT1
#    endif

#    define GPIO_SD_CD (GPIO_INTERRUPT | GPIO_INT_CHANGE | GPIO_PULL_UP | \
                        GPIO_GLITCH_FILTER | GPIO_PORTB | GPIO_PIN13)
#    define IRQ_SD_CD   SAM_IRQ_PB13

#    define GPIO_SD_CS (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_SET | \
                        GPIO_PORTC | GPIO_PIN3)
#    define SD_CSNO    0

#  elif defined(CONFIG_SAM4L_XPLAINED_IOMODULE_EXT2)

#    ifndef CONFIG_SAM4L_XPLAINED_SLCD1MODULE
#      error I/O1 cannot be in EXT2 if the LCD1 module is connected
#    endif

#    if defined(CONFIG_SAM4L_XPLAINED_OLED1MODULE) && \
        defined(CONFIG_SAM4L_XPLAINED_OLED1MODULE_EXT2)
#      error I/O1 and OLED1 modules cannot both reside in EXT2
#    endif

#    define GPIO_CD   (GPIO_INTERRUPT | GPIO_INT_CHANGE | GPIO_PULL_UP | \
                       GPIO_GLITCH_FILTER | GPIO_PORTC | GPIO_PIN9)
#    define IRQ_CD     SAM_IRQ_PC9

#    define GPIO_SD_CS (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_SET | \
                        GPIO_PORTB | GPIO_PIN11)
#    define SD_CSNO    2

#  else
#    error Which connector is the I/O1 module installed in?
#  endif
#endif

/* OLED1
 *
 * Support for the microSD card slot on the I/O1 module.
 * The I/O1 requires SPI support and three output GPIOs.
 * These the GPIOs will vary if the OLED1 is installed on the EXT1 or EXT2
 * connector:
 *
 *
 *   PIN EXT1           EXT2            Description
 *   --- -------------- --------------- -------------------------------------
 *   5   PB12 GPIO      PC08 GPIO       DATA_CMD_SEL
 *   10  PB13 SPI/NPCS1 PC09 GPIO       DISPLAY_RESET. Active low.
 *   15  PC03 SPI/NPCS0 PB11 SPI/NPCS2  DISPLAY_SS.  Active low.
 */

#ifdef CONFIG_SAM4L_XPLAINED_OLED1MODULE

#  ifndef CONFIG_SAM34_SPI0
#    error CONFIG_SAM34_SPI0 is required to use the OLED1 module
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

#  if defined(CONFIG_SAM4L_XPLAINED_OLED1MODULE_EXT1)

#    if defined(CONFIG_SAM4L_XPLAINED_IOMODULE) && \
        defined(CONFIG_SAM4L_XPLAINED_IOMODULE_EXT1)
#      error OLED1 and I/O1 modules cannot both reside in EXT1
#    endif

#    define GPIO_OLED_DATA (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTB | GPIO_PIN12)
#    define GPIO_OLED_RST  (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTB | GPIO_PIN13)
#    define GPIO_OLED_CS   (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_SET | \
                            GPIO_PORTC | GPIO_PIN3)
#    define OLED_CSNO       0

#  elif defined(CONFIG_SAM4L_XPLAINED_OLED1MODULE_EXT2)

#    ifndef CONFIG_SAM4L_XPLAINED_SLCD1MODULE
#      error OLED1 cannot be in EXT2 if the LCD1 module is connected
#    endif

#    if defined(CONFIG_SAM4L_XPLAINED_IOMODULE) && \
        defined(CONFIG_SAM4L_XPLAINED_IOMODULE_EXT2)
#      error OLED1 and I/O1 modules cannot both reside in EXT2
#    endif

#    define GPIO_OLED_DATA (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTC | GPIO_PIN8)
#    define GPIO_OLED_RST  (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTc | GPIO_PIN9)
#    define GPIO_OLED_CS   (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_SET | \
                            GPIO_PORTB | GPIO_PIN11)
#    define OLED_CSNO      2

#  else
#    error Which connector is the OLED1 module installed in?
#  endif
#endif

#if defined(CONFIG_LCD_UG2864AMBAG01) || defined(CONFIG_LCD_UG2864HSWEG01)
#    define GPIO_SD_CS (GPIO_OUTPUT | GPIO_PULL_NONE | GPIO_OUTPUT_SET | \
                        GPIO_PORTB | GPIO_PIN11)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAM3U-EK board.
 *
 ****************************************************************************/

void weak_function sam_spidev_initialize(void);

/****************************************************************************
 * Name: sam_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.
 *   Requires CONFIG_SAM4L_XPLAINED_IOMODULE=y,
 *   CONFIG_DISABLE_MOUNTPOINT=n, CONFIG_MMCSD=y, and CONFIG_SAM34_SPI0=y
 *
 ****************************************************************************/

#if defined(CONFIG_SAM34_SPI0) && defined(CONFIG_SAM4L_XPLAINED_IOMODULE)
int sam_sdinitialize(int minor);
#endif

/****************************************************************************
 * Name: sam_slcd_initialize
 *
 * Description:
 *   Initialize the SAM4L Xplained Pro LCD hardware and register the
 *   character driver as /dev/slcd0.
 *
 ****************************************************************************/

#if defined(CONFIG_SAM34_LCDCA) && defined(CONFIG_SAM4L_XPLAINED_SLCD1MODULE)
int sam_slcd_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAM34_SAM4L_XPLAINED_SRC_SAM4L_XPLAINED_H */
