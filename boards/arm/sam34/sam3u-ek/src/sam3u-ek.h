/****************************************************************************
 * boards/arm/sam34/sam3u-ek/src/sam3u-ek.h
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

#ifndef __BOARDS_ARM_SAM34_SAM3U_EK_SRC_SAM3U_EK_H
#define __BOARDS_ARM_SAM34_SAM3U_EK_SRC_SAM3U_EK_H

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

/* External Memory Usage ****************************************************/

/* LCD on CS2 */

#define LCD_BASE              SAM_EXTCS2_BASE

/* Touchscreen controller (TSC) */

#define CONFIG_TSC_ADS7843    1   /* ADS7843 present on board */
#define CONFIG_TSC_SPI        0   /* On SPI0 */

/* SAM3U-EK GPIO Pin Definitions ********************************************/

/* LCD:
 *   LCD Module Pin Out:                       SAM3U PIO:
 *  ----------------------------------- ------------------------------------
 *   Pin Symbol Function                   LCD        PeriphA  PeriphB Extra
 *  ---- ------ ----------------------- ------------ -------- ------ ------
 *   1   GND    Ground                  N/A          ---      ---    ---
 *   2   CS     Chip Select             PC16         NCS2     PWML3  AD12BAD5
 *   3   RS     Register select signal  PB8 (see A1) CTS0     A1     AD3
 *   4   WR     Write operation signal  PB23 (NWE)   NWR0/NEW PCK1   ---
 *   5   RD     Read operation signal   PB19 (NRD)   NRD      PWML2  ---
 *   6   DB0    Data bus                PB9          D0       DTR0   ---
 *   7   DB1    Data bus                PB10         D1       DSR0   ---
 *   8   DB2    Data bus                PB11         D2       DCD0   ---
 *   9   DB3    Data bus                PB12         D3       RI0    ---
 *   10  DB4    Data bus                PB13         D4       PWMH0  ---
 *   11  DB5    Data bus                PB14         D5       PWMH1  ---
 *   12  DB6    Data bus                PB15         D6       PWMH2  ---
 *   13  DB7    Data bus                PB16         D7       PMWH3  ---
 *   14  DB8    Data bus                PB25         D8       PWML0  ---
 *   15  DB9    Data bus                PB26         D9       PWML1  ---
 *   16  DB10   Data bus                PB27         D10      PWML2  ---
 *   17  DB11   Data bus                PB28         D11      PWML3  ---
 *   18  DB12   Data bus                PB29         D12      ---    ---
 *   19  DB13   Data bus                PB30         D13      ---    ---
 *   20  DB14   Data bus                PB31         D14      ---    ---
 *   21  DB15   Data bus                PB6          TIOA1    D15    AD1
 *   22  NC     No connection           N/A          ---      ---    ---
 *   23  NC     No connection           N/A          ---      ---    ---
 *   24  RESET  Reset signal            N/A          ---      ---    ---
 *   25  GND    Ground                  N/A          ---      ---    ---
 *   26  X+     Touch panel X_RIGHT     PA15         SPCK     PWMH2  ---
 *   27  Y+     Touch panel Y_UP        PA14         MOSI     ---    ---
 *   28  X-     Touch panel X_LEFT      PA13         MISO     ---    ---
 *   29  Y-     Touch panel Y_DOWN      PC14         A3       NPCS2  ---
 *   30  GND    Ground                  N/A          ---      ---    ---
 *   31  VDD1   Power supply for
 *              digital IO Pad          N/A          ---      ---    ---
 *   32  VDD2   Power supply for
 *              analog circuit          N/A          ---      ---    ---
 *   33  A1     Power supply for
 *              backlight               PB8 (see RS) CTS0     A1     AD3
 *   34  A2     Power supply for
 *              backlight               N/A          ---      ---    ---
 *   35  A3     Power supply for
 *              backlight               N/A          ---      ---    ---
 *   36  A4     Power supply for
 *              backlight               N/A          ---      ---    ---
 *   37  NC     No connection           N/A          ---      ---    ---
 *   38  NC     No connection           N/A          ---      ---    ---
 *   39  K      Backlight ground        N/A          ---      ---    ---
 */

#define GPIO_LCD_NCS2 (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOC | GPIO_PIN16)
#define GPIO_LCD_RS   (GPIO_PERIPHB | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN8)
#define GPIO_LCD_NWE  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN23)
#define GPIO_LCD_NRD  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN19)

#define GPIO_LCD_D0   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN9)
#define GPIO_LCD_D1   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN10)
#define GPIO_LCD_D2   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN11)
#define GPIO_LCD_D3   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN12)
#define GPIO_LCD_D4   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN13)
#define GPIO_LCD_D5   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN14)
#define GPIO_LCD_D6   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN15)
#define GPIO_LCD_D7   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN16)
#define GPIO_LCD_D8   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN25)
#define GPIO_LCD_D9   (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN26)
#define GPIO_LCD_D10  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN27)
#define GPIO_LCD_D11  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN28)
#define GPIO_LCD_D12  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN29)
#define GPIO_LCD_D13  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN30)
#define GPIO_LCD_D14  (GPIO_PERIPHA | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN31)
#define GPIO_LCD_D15  (GPIO_PERIPHB | GPIO_CFG_PULLUP | GPIO_PORT_PIOB | GPIO_PIN6)

/* LCD Backlight pin definition. */

#define GPIO_LCD_BKL  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                       GPIO_PORT_PIOC | GPIO_PIN19)

/* Touchscreen controller (TSC)
 *
 * The IRQ is active low and pulled up.
 *
 * Pen Interrupt. Open anode output, requires 10kO to 100kO pull-up resistor
 * externally.  There is a 100KO pull-up on the SAM3U-EK board so no
 * additional pull-up should be required.
 *
 * BUSY is high impedance when CS is high (not selected).  When CS is
 * is low, BUSY is active high.  Since the pin is pulled up, it will appear
 * busy if CS is not selected (there is no pull-up onboard).
 */

#define GPIO_TCS_IRQ  (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_INT_BOTHEDGES | \
                       GPIO_PORT_PIOA | GPIO_PIN24)
#define GPIO_TCS_BUSY (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_PORT_PIOA | \
                       GPIO_PIN2)

#define SAM_TCS_IRQ   SAM_IRQ_PA24

/* LEDs */

#define GPIO_LED0     (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOB | \
                       GPIO_OUTPUT_CLEAR | GPIO_PIN0)
#define GPIO_LED1     (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOB | \
                       GPIO_OUTPUT_SET | GPIO_PIN1)
#define GPIO_LED2     (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_PORT_PIOB | \
                       GPIO_OUTPUT_SET | GPIO_PIN2)

/* BUTTONS */

#define GPIO_BUTTON1  (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                       GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN18)
#define GPIO_BUTTON2  (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                       GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN19)

#define IRQ_BUTTON1   SAM_IRQ_PA18
#define IRQ_BUTTON2   SAM_IRQ_PA19

/* SD Card Detect */

#define GPIO_MCI_CD   (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_PORT_PIOA | GPIO_PIN25)

/* SPI Chip Selects */

/* Chip select pin connected to the touchscreen controller and to the ZigBee
 * module connector.
 * Notice that the touchscreen chip select is implemented as a GPIO OUTPUT
 * that must be controlled by board-specific.
 * This is because the ADS7843E driver must be able to sample the device BUSY
 * GPIO input between SPI transfers.
 * However, the AD7843E will tri-state the BUSY input whenever the chip
 * select is de-asserted.
 * So the only option is to control the chip select manually and hold it low
 * throughout the SPI transfer.
 */

#define GPIO_TSC_NPCS2 (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
                        GPIO_PORT_PIOC | GPIO_PIN14)
#define TSC_CSNUM      2

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
 * Name: sam_hsmciinit
 *
 * Description:
 *   Initialize HSMCI support
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_HSMCI
int weak_function sam_hsmciinit(void);
#else
# define sam_hsmciinit()
#endif

/****************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_HSMCI
bool sam_cardinserted(unsigned char slot);
#else
#  define sam_cardinserted(slot) (false)
#endif

/****************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_HSMCI
bool sam_writeprotected(unsigned char slot);
#else
#  define sam_writeprotected(slot) (false)
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

#ifdef CONFIG_INPUT_ADS7843E
int sam_tsc_setup(int minor);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAM34_SAM3U_EK_SRC_SAM3U_EK_H */
