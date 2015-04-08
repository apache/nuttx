/****************************************************************************************************
 * configs/stm32ldiscovery/src/stm32ldiscover.h
 * arch/arm/src/board/stm32ldiscover.n
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
 ****************************************************************************************************/

#ifndef __CONFIGS_STM32F3DISCOVERY_SRC_STM32F3DISCOVERY_INTERNAL_H
#define __CONFIGS_STM32F3DISCOVERY_SRC_STM32F3DISCOVERY_INTERNAL_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/
/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#endif

/* STM32L-Discovery GPIOs ***************************************************************************/
/* The STM32L-Discovery board has four LEDs.  Two of these are controlled by logic on the board and
 * are not available for software control:
 *
 * LD1 COM:   LD2 default status is red. LD2 turns to green to indicate that communications are in
 *            progress between the PC and the ST-LINK/V2.
 * LD2 PWR:   Red LED indicates that the board is powered.
 *
 * And two LEDs can be controlled by software:
 *
 * User LD3:  Green LED is a user LED connected to the I/O PB7 of the STM32L152 MCU.
 * User LD4:  Blue LED is a user LED connected to the I/O PB6 of the STM32L152 MCU.
 *
 * The other side of the LED connects to ground so high value will illuminate the LED.
 */

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_10MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN7)
#define GPIO_LED2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_10MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN6)

/* Button definitions *******************************************************************************/
/* The STM32L-Discovery supports two buttons; only one button is controllable by software:
 *
 *   B1 USER: user and wake-up button connected to the I/O PA0 of the STM32F303VCT6.
 *   B2 RESET: pushbutton connected to NRST is used to RESET the STM32F303VCT6.
 *
 * NOTE that  EXTI interrupts are configured
 */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)

/* LCD definitions **********************************************************************************/
/* LCD.  The STM32L152RBT6 supports either a 4x32 or 8x28.  The STM32L-Discovery
 * has an LCD 24 segments, 4 commons.  On that board, LCD pins are mapped as
 * follows:
 *
 * The 24 segments are represented by the letters A, B, C, D, E, F, G, H, J,
 * K, M, N, P, Q, COL (colon), and DP (decimal point)
 *
 *              A
 *          ---------    _
 *         |\   |J  /|  |_| COL
 *        F| H  |  K |B
 *         |  \ | /  |   _
 *         --G-- --M-+  |_| COL
 *         |   /| \  |
 *        E|  Q |  N |C
 *         | /  |P  \|   _
 *          ---------   |_| DP
 *              D
 *
 * Plus BAR0-3.  The following is of each segment of each of the 6 characters 4 x 24:
 *
 * ---- ----- ----- ----- ----- ---------------
 * GPIO COM3  COM2  COM1  COM0  SIGNAL NAME
 * ---- ----- ----- ----- ----- ---------------
 * PA1   1N    1P   1D    1E    LCD SEG0
 * PA2   1DP   1COL 1C    1M    LCD SEG1
 * PA3   2N    2P   2D    2E    LCD SEG2
 * PB3   2DP   2COL 2C    2M    LCD SEG3
 * PB4   3N    3P   3D    3E    LCD SEG4
 * PB5   3DP   3COL 3C    3M    LCD SEG5
 * PB10  4N    4P   4D    4E    LCD SEG6
 * PB11  4DP   4COL 4C    4M    LCD SEG7
 * PB12  5N    5P   5D    5E    LCD SEG8
 * PB13  BAR2  BAR3 5C    5M    LCD SEG9
 * PB14  6N    6P   6D    6E    LCD SEG10
 * PB15  BAR0  BAR1 6C    6M    LCD SEG11
 * PB9   COM3                   LCD glass COM3
 * PA10        COM2             LCD glass COM2
 * PA9              COM1        LCD glass COM1
 * PA8                    COM0  LCD glass COM0
 * PA15  6J    6K   6A    6B    LCD SEG12
 * PB8   6H    6Q   6F    6G    LCD SEG13
 * PC0   5J    5K   5A    5B    LCD SEG14
 * PC1   5H    5Q   5F    5G    LCD SEG15
 * PC2   4J    4K   4A    4B    LCD SEG16
 * PC3   4H    4Q   4F    4G    LCD SEG17
 * PC6   3J    3K   3A    3B    LCD SEG18
 * PC7   3H    3Q   3F    3G    LCD SEG19
 * PC8   2J    2K   2A    2B    LCD SEG20
 * PC9   2H    2Q   2F    2G    LCD SEG21
 * PC10  1J    1K   1A    1B    LCD SEG22
 * PC11  1H    1Q   1F    1G    LCD SEG23
 * ---- ----- ----- ----- ----- ---------------
 *
 * ----- --------------------- ----------------
 * GPIO  ALTERNATE FUNCTION    BOARD SIGNAL
 * ----- --------------------- ----------------
 * PA8   LCD_COM0              LCD glass COM0
 * PA9   LCD_COM1              LCD glass COM1
 * PA10  LCD_COM2              LCD glass COM2
 * PB9   LCD_COM3              LCD glass COM3
 * PA1   LCD_SEG0              LCD SEG0
 * PA2   LCD_SEG1              LCD SEG1
 * PA3   LCD_SEG2              LCD SEG2
 * PB3   LCD_SEG7              LCD_SEG3
 * PB4   LCD_SEG8              LCD_SEG4
 * PB5   LCD_SEG9              LCD SEG5
 * PB10  LCD_SEG10             LCD SEG6
 * PB11  LCD_SEG11             LCD SEG7
 * PB12  LCD_SEG12             LCD SEG8
 * PB13  LCD_SEG13             LCD SEG9
 * PB14  LCD_SEG14             LCD SEG10
 * PB15  LCD_SEG15             LCD SEG11
 * PA15  LCD_SEG12             LCD SEG12
 * PB8   LCD_SEG16             LCD SEG13
 * PC0   LCD_SEG18             LCD SEG14
 * PC1   LCD_SEG19             LCD SEG15
 * PC2   LCD_SEG20             LCD SEG16
 * PC3   LCD_SEG21             LCD SEG17
 * PC6   LCD_SEG24             LCD SEG18
 * PC7   LCD_SEG25             LCD SEG19
 * PC8   LCD_SEG26             LCD SEG20
 * PC9   LCD_SEG27             LCD SEG21
 * PC10  LCD_SEG40             LCD SEG22
 * PC11  LCD_SEG41             LCD SEG23
 */

#define BOARD_SLCD_COM0  GPIO_LCD_COM0   /* PA8 */
#define BOARD_SLCD_COM1  GPIO_LCD_COM1   /* PA9 */
#define BOARD_SLCD_COM2  GPIO_LCD_COM2   /* PA10 */
#define BOARD_SLCD_COM3  GPIO_LCD_COM3   /* PB9 */
#define BOARD_SLCD_SEG0  GPIO_LCD_SEG0   /* PA1 */
#define BOARD_SLCD_SEG1  GPIO_LCD_SEG1   /* PA2 */
#define BOARD_SLCD_SEG2  GPIO_LCD_SEG2   /* PA3 */
#define BOARD_SLCD_SEG3  GPIO_LCD_SEG7   /* PB3 */
#define BOARD_SLCD_SEG4  GPIO_LCD_SEG8   /* PB4 */
#define BOARD_SLCD_SEG5  GPIO_LCD_SEG9   /* PB5 */
#define BOARD_SLCD_SEG6  GPIO_LCD_SEG10  /* PB10 */
#define BOARD_SLCD_SEG7  GPIO_LCD_SEG11  /* PB11 */
#define BOARD_SLCD_SEG8  GPIO_LCD_SEG12  /* PB12 */
#define BOARD_SLCD_SEG9  GPIO_LCD_SEG13  /* PB13 */
#define BOARD_SLCD_SEG10 GPIO_LCD_SEG14  /* PB14 */
#define BOARD_SLCD_SEG11 GPIO_LCD_SEG15  /* PB15 */
#define BOARD_SLCD_SEG12 GPIO_LCD_SEG17  /* PA15 */
#define BOARD_SLCD_SEG13 GPIO_LCD_SEG16  /* PB8 */
#define BOARD_SLCD_SEG14 GPIO_LCD_SEG18  /* PC0 */
#define BOARD_SLCD_SEG15 GPIO_LCD_SEG19  /* PC1 */
#define BOARD_SLCD_SEG16 GPIO_LCD_SEG20  /* PC2 */
#define BOARD_SLCD_SEG17 GPIO_LCD_SEG21  /* PC3 */
#define BOARD_SLCD_SEG18 GPIO_LCD_SEG24  /* PC6 */
#define BOARD_SLCD_SEG19 GPIO_LCD_SEG25  /* PC7 */
#define BOARD_SLCD_SEG20 GPIO_LCD_SEG26  /* PC8 */
#define BOARD_SLCD_SEG21 GPIO_LCD_SEG27  /* PC9 */
#define BOARD_SLCD_SEG22 GPIO_LCD_SEG40  /* PC10 */
#define BOARD_SLCD_SEG23 GPIO_LCD_SEG41  /* PC11 */

#define BOARD_SLCD_NCOM    4
#define BOARD_SLCD_NSEG   24
#define BOARD_SLCD_NGPIOS 28

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32ldiscovery board.
 *
 ****************************************************************************************************/

void weak_function stm32_spiinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_STM32F3DISCOVERY_SRC_STM32F3DISCOVERY_INTERNAL_H */

