/****************************************************************************
 * boards/arm/sam34/arduino-due/src/arduino-due.h
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

#ifndef __BOARDS_ARM_SAM34_ARDUINO_DUE_SRC_ARDUNO_DUE_H
#define __BOARDS_ARM_SAM34_ARDUINO_DUE_SRC_ARDUNO_DUE_H

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

/*  There are three user-controllable LEDs on board the Arduino Due board:
 *
 *     LED              GPIO
 *     ---------------- -----
 *     L   Amber LED    PB27
 *     TX  Yellow LED   PA21
 *     RX  Yellow LED   PC30
 *
 * LED L is connected to ground and can be illuminated by driving the PB27
 * output high. The TX and RX LEDs are pulled high and can be illuminated by
 * driving the corresponding
 * GPIO output to low.
 *
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *   SYMBOL                MEANING                         LED STATE
 *                                                   L         TX       RX
 *   -------------------  -----------------------  -------- -------- --------
 *   LED_STARTED          NuttX has been started     OFF      OFF      OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF      OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF      OFF      OFF
 *   LED_STACKCREATED     Idle stack created         ON       OFF      OFF
 *   LED_INIRQ            In an interrupt            N/C      GLOW     OFF
 *   LED_SIGNAL           In a signal handler        N/C      GLOW     OFF
 *   LED_ASSERTION        An assertion failed        N/C      GLOW     OFF
 *   LED_PANIC            The system has crashed     N/C      N/C    Blinking
 *   LED_IDLE             MCU is is sleep mode       ------ Not used --------
 *
 * Thus if LED L is statically on, NuttX has successfully booted and is,
 * apparently, running normmally.  If LED RX is glowing, then NuttX is
 * handling interrupts (and also signals and assertions).  If TX is flashing
 * at approximately 2Hz, then a fatal error has been detected and the system
 */

#define GPIO_LED_L   (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_CLEAR | \
                      GPIO_PORT_PIOB | GPIO_PIN27)
#define GPIO_LED_RX  (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOC | GPIO_PIN30)
#define GPIO_LED_TX  (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOA | GPIO_PIN21)

/* ITEAD 2.4" TFT with Touch
 *
 * The Arduino 2.4" TFT Touch Shield is designed for all the Arduino
 * compatible boards. It works in 3.3V voltage level. It can be directly
 * plugged on the Arduino and other compatible boards. It will offer
 * display, touch and storage functions for the Arduino board
 *
 * Features:
 *
 *   1. Compatible with 3.3/5V operation voltage level
 *   2. Compatible with UTFT library
 *   3. With SD Card Socket
 *
 * The Arduino 2.4" TFT Touch shield uses the S6D1121 controller , it
 * supports 8-bit data interface. The touch IC is TSC2046.
 *
 * --------- ------------------- ------- ------------------------- -----
 * Arduino           ATSAM3X              Due             ITHEAD
 * Due PIN   GPIO FUNCTION       SIGNA   PIN              SIGNAL   NOTES
 * --------- ---- ------------ ------- ---------------- ---------- -----
 * PWMH
 * 10  SCL1  PA18 TWCK0/A20/
 *                WKUP9       SCL1      ---      ---   ---      SCL N/A
 *  9  SDA1  PA17 TWD0SPCK0   SDA1      ---      ---   ---      SDA N/A
 *  8  Aref  ---  ---         AREF      J2 pin 8 Vref  N/C      ---
 *  7  GND   ---  ---         GND       J2 pin 7 GND   ---      ---
 *  6  PWM13 PB27 SPI0_SPCK/
 *                A20/WKUP10  PWM13     J2 pin 6 D13   SD_SCK   SCK,
 *                                                              also LED "L",
 *                                                              Pulled low
 *  5  PWM12 PD8  A21/
 *                NANDALE/
 *                TIOB8       PWM12     J2 pin 5 D12   SD_MISO  MISO N/A
 *  4  PWM11 PD7  A17/BA1/
 *                TIOA8       PWM11     J2 pin 4 D11   SD_MOSI  MOSI N/A,
 *                                                              Pulled low
 *  3  PWM10 PA28 SPI0_NPCS0/
 *                PCK2/
 *                WKUP11     SS0/PWM10  J2 pin 3 D10   SD_CS    Pulled low
 *                                                              on-board
 *  2  PWM9 PC21  A0/NBS0/
 *                PWML4       PWM9      J2 pin 2 D9  Touch_Dout ---
 *  1  PWM8 PC22  A1/PWML5    PWM8      J2 pin 1 D8  Touch_IRQ  ---
 *
 * PWML
 *  8  PWM7 PC23 A2/PWML6     PWM7      J3 pin 8 D7   DB15      ---
 *  7  PWM6 PC24 A3/PWML7     PWM6      J3 pin 7 D6   DB14      ---
 *  6  PWM5 PC25 A4/TIOA6     PWM5      J3 pin 6 D5   DB13      ---
 *  5  PWM4 PC26 A5/TIOB6     SS1/PWM4  J3 pin 5 D4   DB12      ---
 *  4  PWM3 PC28 A7/TIOA7     PWM3      J3 pin 4 D3   DB11      ---
 *  3  PWM2 PB25 RTS0/
 *               TIOA0        PWM2      J3 pin 3 D2   DB10      ---
 *  2  PWM1 PA9  UTXD/
 *               PWMH3        TX        J3 pin 2 D1   DB9       UART0 TX
 *  1  PWM0 PA8  URXD/
 *               PWMH0/
 *               WKUP4        RX        J3 pin 1 D0   DB8       UART0 RX
 * ---------- ---- --------------- ----------- ---------------- --------- ---
 * POWER
 *  1  ---    ---  ---             ---         ---      ---    ---        ---
 *  2  IOref  ---  ---            IOREF +3V3   ---      ---    ---        ---
 *  3  RESET  ---  ---            MASTER_RESET J4 pin 1 RST    ---        ---
 *  5  5V     ---  ---            +5V          J4 pin 2 3.3V   ---        ---
 *  4  3.3V   ---  ---            +3V3         J4 pin 3 5V     ---        ---
 *  6  GND    ---  ---            GND          J4 pin 4 GND    ---        ---
 *  7  GND    ---  ---            GND          J4 pin 5 GND    ---        ---
 *  8  Vin    ---  ---            VIN          J4 pin 6 Vin    ---        ---
 * ADCL
 *  1  A0     PA16 SPCK1/
 *                 TD/AD7         AD0          J1 pin 1 A0/D14 Touch_Din  ---
 *  2  A1     PA24 MCDA3/
 *                 PCK1/AD6       AD1          J1 pin 1 A1/D15 Touch_CLK  ---
 *  3  A2     PA23 MCDA2/
 *                 TCLK4/AD5      AD2          J1 pin 1 A2/D16 ---        ---
 *  4  A3     PA22 MCDA1/
 *                 TCLK3/AD4      AD3          J1 pin 1 A3/D17 TFT_CS     ---
 *  5  A4     PA6  TIOB2/
 *                 NCS0/AD3       AD4          J1 pin 1 A4/D18 TFT_WR     ---
 *  6  A5     PA4  TCLK1/
 *                 NWAIT/AD2      AD5          J1 pin 1 A5/D19 TFT_RS     ---
 *  7  A6     PA3  TIOB1/
 *                 PWMFI1/
 *                 AD1/WKUP1      AD6          ---      ---    ---        ---
 *  8  A7     PA2  TIOA1/
 *                 NANDRDY/
 *                 AD0            AD7          ---      ---    ---        ---
 * ---------- ---- ---------- ----------- -------------------- ---------- ---
 *
 * NOTES:
 *
 * 1. It is not possible to use any of the SPI devices on the Shield unless
 *    a bit-bang SPI interface is used.  This includes the touch controller
 *    and the SD card.
 * 2. UART0 cannot be used.  USARTs on the COMM connector should be
 *    available.
 * 3. Parallel data is not contiguous in the PIO register
 * 4. Touch controller /CS pin is connected to ground (always selected).
 * 5. Either PA28 or PC29 may drive PWM10
 */

#ifdef CONFIG_ARDUINO_ITHEAD_TFT
/* In order to use the SD card on the ITEAD shield, you must enable the
 * SPI bit-bang driver as well as support for SPI-based MMC/SD cards.
 */

#  if defined(CONFIG_SPI_BITBANG) && defined(CONFIG_MMCSD_SPI)

/* SD Interface:
 *
 * ------------ ------------------ -------
 * SD CONNECTOR ARDUINO CONNECTORS AT91SAM
 * PIN SIGNAL   PIN      SIGNAL    GPIO
 * --- -------- -------- --------- --------
 *  1   /CS      J2 pin 3 D10       PA28
 *  2   DI       J2 pin 4 D11       PD7
 *  3   GND      ---      ---       ---
 *  4   VCC      ---      ---       ---
 *  5   CLK      J2 pin 6 D13       PB27
 *  6   GND      ---      ---       ---
 *  7   DO       J2 pin 5 D12       PD8
 *  8   IRQ      N/C      ---       ---
 *  9   N/C      ---      ---       ---
 * 10   SW       N/C      ---       ---
 * 11   WP       N/C      ---       ---
 * 12   CD       N/C      ---       ---
 * 13   CD       N/C      ---       ---
 * 14   GND      ---      ---       ---
 * 15   GND      ---      ---       ---
 * 16   GND      ---      ---       ---
 * --- -------- -------- --------- --------
 *
 * NOTES:
 * - The SD slot shares the pin with LED "L" so LED support must be disabled
 *   to use the MMC/SD card on the ITEAD shield.
 * - Either PA28 or PC29 may drive D10
 */

#    ifdef CONFIG_ARCH_LEDs
#      error LEDs may not be used with the ITEAD SD card
#    endif

#    define GPIO_SD_SCK   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOB | GPIO_PIN27)
#    define GPIO_SD_MISO  (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_PORT_PIOD | \
                           GPIO_PIN8)
#    define GPIO_SD_MOSI  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOD | GPIO_PIN7)

#    define GPIO_SD_CS    (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOA | GPIO_PIN28)
#  endif

/* In order to use the touchscreen on the ITEAD shield, you must enable the
 * SPI bit-bang driver and ADS7843E/XPT2046 device support.
 */

#  if defined(CONFIG_SPI_BITBANG) && defined(CONFIG_INPUT_ADS7843E)

/* Touch Controller Interface:
 *
 * ----------- ------------------ --------
 *   XPT2046   ARDUINO CONNECTORS AT91SAM
 * PIN SIGNAL  PIN      SIGNAL    GPIO
 * --- ------- -------- --------- --------
 *  1  VCC        ---      ---       ---
 *  2   X+        ---      ---       ---
 *  3   Y+        ---      ---       ---
 *  4   X-        ---      ---       ---
 *  5   Y-        ---      ---       ---
 *  6  GND        ---      ---       ---
 *  7  IN3        N/C      ---       ---
 *  8  IN4        N/C      ---       ---
 *  9  VREF       ---      ---       ---
 *  10 VCC        ---      ---       ---
 *  11 IRQ        J2 pin 2 D9        PC21
 *  12 DOUT       J2 pin 1 D8        PC22
 *  13 BUSY       N/C      ---       ---
 *  14 DIN        J1 pin 1 A0/D14    PA16
 *  15 /C  S      ---      ---       ---
 *  16 DCLK       J1 pin 2 A1/D15    PA24
 * --- ------- -------- --------- --------
 *
 * NOTE: /CS is connected to ground (XPT2046 is always selected)
 */

#    define GPIO_TSC_SCK  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOA | GPIO_PIN24)
#    define GPIO_TSC_MISO (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_PORT_PIOC | \
                           GPIO_PIN22)
#    define GPIO_TSC_MOSI (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOA | GPIO_PIN16)

#    define GPIO_TSC_IRQ  (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_INT_BOTHEDGES | \
                           GPIO_PORT_PIOC | GPIO_PIN21)
#    define SAM_TSC_IRQ   SAM_IRQ_PC21
#  endif

/* Only CONFIG_LCD is expected to enable the TFT LCD */

#  ifdef CONFIG_LCD

/* UART0 cannot be used with the LCD because the UART0 pins are used
 * by the LCD.
 */

#    ifdef CONFIG_SAM34_UART0
#      error "UART0 cannot be used with the ITEAD LCD"
#    endif

/* Data pins are initially configured but may be switched dynamically to
 * either inputs or outputs as needed.
 */

#    define GPIO_LCD_D0IN (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOA | GPIO_PIN8)
#    define GPIO_LCD_D1IN (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOA | GPIO_PIN9)
#    define GPIO_LCD_D2IN (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOB | GPIO_PIN25)
#    define GPIO_LCD_D3IN (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOC | GPIO_PIN28)
#    define GPIO_LCD_D4IN (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOC | GPIO_PIN26)
#    define GPIO_LCD_D5IN (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOC | GPIO_PIN25)
#    define GPIO_LCD_D6IN (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOC | GPIO_PIN24)
#    define GPIO_LCD_D7IN (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOC | GPIO_PIN23)
#    define GPIO_LCD_D7IN (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOC | GPIO_PIN23)
#    define GPIO_LCD_CS   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOA | GPIO_PIN22)
#    define GPIO_LCD_WR   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOA | GPIO_PIN6)
#    define GPIO_LCD_RS   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                           GPIO_PORT_PIOA | GPIO_PIN4)
#  endif
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
 * Name: sam_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int sam_bringup(void);

/****************************************************************************
 * Name: sam_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.
 *
 ****************************************************************************/

#if defined(CONFIG_ARDUINO_ITHEAD_TFT) && defined(CONFIG_SPI_BITBANG) && \
    defined(CONFIG_MMCSD_SPI)
int sam_sdinitialize(int minor);
#endif

/****************************************************************************
 * Name: sam_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.
 *   This function will register the driver as /dev/inputN where N is the
 *   minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.
 *   Otherwise, a negated errno value is returned to indicate the nature
 *   of the failure.
 *
 ****************************************************************************/

#if defined(CONFIG_ARDUINO_ITHEAD_TFT) && defined(CONFIG_SPI_BITBANG) && \
    defined(CONFIG_INPUT_ADS7843E)
int sam_tsc_setup(int minor);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAM34_ARDUINO_DUE_SRC_ARDUNO_DUE_H */
