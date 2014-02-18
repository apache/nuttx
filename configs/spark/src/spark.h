/************************************************************************************
 * configs/spark/src/spark.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Laurent Latil <laurent@latil.nom.fr>
 *           Librae <librae8226@gmail.com>
 *           David_s5 <david_s5@nscdg.com>
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

#ifndef __CONFIGS_SPARK_SRC_SPARK_H
#define __CONFIGS_SPARK_SRC_SPARK_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* During the development of the SparkCore, the hardware was in limited supply
 * As a work around david_s5 created a SparkCore Big board (http://nscdg.com/spark/sparkBB.png)
 * that will interface with a maple mini (http://leaflabs.com/docs/hardware/maple-mini.html),
 * and a CC3000BOOST (https://estore.ti.com/CC3000BOOST-CC3000-BoosterPack-P4258.aspx)
 *
 * It breaks out the Tx, Rx to connect to a FTDI TTL-232RG-VREG3V3-WE for the console and
 * wires in the spark LEDs and serial flash to the same I/O as the sparkcore. It has a Jlink
 * compatible Jtag connector on it.
 *
 *
 * Board GPIO Usage:
 *
 *   GPIO      Function                                     MPU        Core        Core       Maple       Maple
 *                                                          Pin #      Name        Pin #      Name        Pin #
 *  ----- --------------------------------             --------------------------------------------------------
 *  PA[00] WKUP/USART2_CTS/ADC12_IN0/TIM2_CH1_ETR            10       A0          JP1-12                 J1-8
 *  PA[01] USART2_RTS/ADC12_IN1/TIM2_CH2                     11       A1          JP1-11                 J1-9
 *  PA[02] USART2_TX/ADC12_IN2/TIM2_CH3                      12       TX          JP1-3                  J1-10
 *  PA[03] USART2_RX/ADC12_IN3/TIM2_CH4                      13       RX          JP1-4                  J1-11
 *  PA[04] SPI1_NSS/USART2_CK/ADC12_IN4                      14       A2          JP1-10                 J1-12
 *  PA[05] SPI1_SCK/ADC12_IN5                                15       A3          JP1-9                  J1-13
 *  PA[06] SPI1_MISO/ADC12_IN6/TIM3_CH1                      16       A4          JP1-8                  J1-14
 *  PA[07] SPI1_MOSI/ADC12_IN7/TIM3_CH2                      17       A5          JP1-7                  J1-15
 *  PA[08] USART1_CK/TIM1_CH1/MCO                            29       LED2                               J2-5
 *  PA[09] USART1_TX/TIM1_CH2                                30       LED3                               J2-6
 *  PA[10] USART1_RX/TIM1_CH3                                31       LED4                               J2-7
 *  PA[11] USART1_CTS/CAN_RX/TIM1_CH4/USBDM                  32       USBM                   USBDM       J2-8
 *  PA[12] USART1_RTS/CAN_TX/TIM1_ETR/USBDP                  33       USBP                   USBDP       J2-9
 *  PA[13] JTMS/SWDIO                                        34       D7,LED1     JP2-5                  J2-10
 *  PA[14] JTCK/SWCLK                                        37       D6          JP2-6                  J2-11
 *  PA[15] JTDI                                              38       D5          JP2-7                  J2-12
 *
 *  PB[00] ADC12_IN8/TIM3_CH3                                18       A6          JP1-6                  J1-16
 *  PB[01] ADC12_IN9/TIM3_CH4                                19       A7          JP1-5      R1-LED
 *  PB[02] BOOT1                                             20       BTN         BTN                    J1-17
 *  PB[03] JTDO                                              39       D4          JP2-8                  J2-13
 *  PB[04] NJTRST                                            40       D3          JP2-9                  J2-14
 *  PB[05] I2C1_SMBA                                         41       D2          JP2-10                 J2-15
 *  PB[06] I2C1_SCL/TIM4_CH1                                 42       D1          JP2-11                 J2-16
 *  PB[07] I2C1_SDA/TIM4_CH2                                 43       D0          JP2-12                 J2-17
 *  PB[08] TIM4_CH3                                          45       WIFI_EN                BOOT0       J2-18
 *  PB[09] TIM4_CH4                                          46       MEM_CS                 DISC
 *  PB[10] I2C2_SCL/USART3_TX                                21       USB_DISC                           J1-18
 *  PB[11] I2C2_SDA/USART3_RX                                22       WIFI_INT                           J1-19
 *  PB[12] SPI2_NSS/I2C2_SMBA/USART3_CK/TIM1_BKIN            25       WIFI_CS                            J2-1
 *  PB[13] SPI2_SCK/USART3_CTS/TIM1_CH1N                     26       SPI_CLK                            J2-2
 *  PB[14] SPI2_MISO/USART3_RTS/TIM1_CH2N                    27       SPI_MISO                           J2-3
 *  PB[15] SPI2_MOSI/TIM1_CH3N                               28       SPI_MOSI                           J2-4
 *
 *  PC[13] TAMPER-RTC                                        2        N.C.                               J1-4
 *  PC[14] OSC32_IN                                          3        OSC32_IN    Y1         N.C.
 *  PC[15] OSC32_OUT                                         4        OSC32_OUT   Y1         N.C.
 *
 *  PD[00] OSC_IN                                            5        OSC_IN      Y2         8MHZ OSC    8MHZ OSC
 *  PD[01] OSC_OUT                                           6        OSC_OUT     Y2         8MHZ OSC    8MHZ OSC
 *
 *  Spark Core pin Mapping
 *
 *  GPIO   ADC  Timer     I2C     UART   SPI       JTAG    Other    5V?  STM Pin# Core HW Core SW
 *  PA0    CH0  2_CH1_ETR         2_CTS                                     10    A0      10
 *  PA1    CH1  2_CH2             2_RTS                                     11    A1      11
 *  PA2    CH2  2_CH3             2_TX                                      12    TX      19
 *  PA3    CH3  2_CH4             2_RX                                      13    RX      18
 *  PA4    CH4                    2_CK   1_NSS                              14    A2      12
 *  PA5    CH5                           1_SCK                              15    A3      13
 *  PA6    CH6  3_CH1                    1_MISO                             16    A4      14
 *  PA7    CH7  3_CH2                    1_MOSI                             17    A5      15
 *  PA8         1_CH1             1_CK             MCO               Yes    29    LED2
 *  PA9         1_CH2             1_TX                               Yes    30    LED3
 *  PA10        1_CH3             1_RX                               Yes    31    LED4
 *  PA11        1_CH4             1_CTS                    USB-      Yes    32    USBM
 *  PA12        1_ETR             1_RTS                    USB+      Yes    33    USBP
 *  PA13                                           JTMS              Yes    34    D7       7
 *  PA14                                           JTCK              Yes    37    D6       6
 *  PA15                                           JTDI              Yes    38    D5       5
 *
 *  PB0    CH8  3_CH3                                                       18    A6      16
 *  PB1    CH9  3_CH4                                                       19    A7      17
 *  PB2                                                    BOOT1     Yes    20    BTN
 *  PB3                                            JTDO              Yes    39    D4       4
 *  PB4                                            NJTRST            Yes    40    D3       3
 *  PB5                   1_SMBA                                            41    D2       2
 *  PB6         4_CH1     1_SCL                                      Yes    42    D1       1
 *  PB7         4_CH2     1_SDA                                      Yes    43    D0       0
 *  PB8         4_CH3                                                Yes    45    WIFI_EN
 *  PB9         4_CH4                                                Yes    46    MEM_CS
 *  PB10                  2_SCL   3_TX                               Yes    21    USB_DISC
 *  PB11                  2_SDA   3_RX                               Yes    22    WIFI_INT
 *  PB12        1_BKIN    2_SMBA  3_CK   2_NSS                       Yes    25    WIFI_CS
 *  PB13                          3_CTS  2_SCK                       Yes    26    SPI_SCK
 *  PB14                          3_RTS  2_MISO                      Yes    27    SPI_MISO
 *  PB15                                 2_MOSI                      Yes    28    SPI_MOSI
 *
 *  PC13                                                                    2
 *  PC14                                                   RTC Oscillator   3     OSC32IN
 *  PC15                                                   RTC Oscillator   4     OSC32OUT
 *
 *  PD0                                                    Oscillator <=    5     OSC
 *  PD1                                                    Oscillator =>    6     OSC
 */

/* How many SPI modules does this chip support? The LM3S6918 supports 2 SPI
 * modules (others may support more -- in such case, the following must be
 * expanded).
 */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#endif

/* LEDs *****************************************************************************/
/*
 *   GPIO      Function                                     MPU        Core        Core       Maple       Maple
 *                                                          Pin #      Name        Pin #      Name        Pin #
 *  ----- --------------------------------             --------------------------------------------------------
 *
 *  PA[08] USART1_CK/TIM1_CH1/MCO                            29       LED2                               J2-5
 *  PA[09] USART1_TX/TIM1_CH2                                30       LED3                               J2-6
 *  PA[10] USART1_RX/TIM1_CH3                                31       LED4                               J2-7
 *  PA[13] JTMS/SWDIO                                        34       D7,LED1     JP2-5                  J2-10
 */

#define GPIO_LED1     (GPIO_PORTA | GPIO_PIN13 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)
#define GPIO_LED_USR  GPIO_LED1
#define GPIO_LED2     (GPIO_PORTA | GPIO_PIN8  | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)
#define GPIO_LED3     (GPIO_PORTA | GPIO_PIN9  | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)
#define GPIO_LED4     (GPIO_PORTA | GPIO_PIN10 | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)

#define GPIO_USB_PULLUP (GPIO_PORTB | GPIO_PIN10 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)

/* BUTTON ***************************************************************************/
/*
 *   GPIO      Function                                     MPU        Core        Core       Maple       Maple
 *                                                          Pin #      Name        Pin #      Name        Pin #
 *  ----- --------------------------------             --------------------------------------------------------
 *  PB[02] BOOT1                                             20       BTN         BTN                    J1-17
 */
#define IRQBUTTON     BUTTON_USER

#define GPIO_BTN      (GPIO_PORTB | GPIO_PIN2 | GPIO_INPUT | GPIO_CNF_INPULLUP | GPIO_EXTI)


/* MEMORYs **************************************************************************/
/*
 *   GPIO      Function                                     MPU        Core        Core       Maple       Maple
 *                                                          Pin #      Name        Pin #      Name        Pin #
 *  ----- --------------------------------             --------------------------------------------------------
 *  PB[09] TIM4_CH4                                          46       MEM_CS                 DISC
*/

#define GPIO_MEM_CS   (GPIO_PORTB | GPIO_PIN9 |  GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)

/* CCS3000 **************************************************************************/
/*
 *   GPIO      Function                                     MPU        Core        Core       Maple       Maple
 *                                                          Pin #      Name        Pin #      Name        Pin #
 *  ----- --------------------------------             --------------------------------------------------------
 *  PB[08] TIM4_CH3                                          45       WIFI_EN                BOOT0       J2-18
 *  PB[11] I2C2_SDA/USART3_RX                                22       WIFI_INT                           J1-19
 *  PB[12] SPI2_NSS/I2C2_SMBA/USART3_CK/TIM1_BKIN            25       WIFI_CS                            J2-1
*/

#define GPIO_WIFI_EN  (GPIO_PORTB | GPIO_PIN8  | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)
#define GPIO_WIFI_CS  (GPIO_PORTB | GPIO_PIN12 | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)

#define GPIO_WIFI_INT (GPIO_PORTB | GPIO_PIN11 | GPIO_INPUT        | GPIO_CNF_INPULLUP | GPIO_EXTI)

#if defined(CONFIG_CC3000_PROBES)
#define GPIO_D0       (GPIO_PORTB | GPIO_PIN7 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)
#define GPIO_D1       (GPIO_PORTB | GPIO_PIN6 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)
#else
#define GPIO_D0       (GPIO_PORTB | GPIO_PIN7 | GPIO_INPUT        | GPIO_CNF_INPULLUP | GPIO_EXTI)
#define GPIO_D1       (GPIO_PORTB | GPIO_PIN6 | GPIO_INPUT        | GPIO_CNF_INPULLUP | GPIO_EXTI)
#define GPIO_D2       (GPIO_PORTB | GPIO_PIN5 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)
#define GPIO_A0       (GPIO_PORTA | GPIO_PIN0 | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)
#define GPIO_A1       (GPIO_PORTA | GPIO_PIN1 | GPIO_OUTPUT_SET   | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz)
#define GPIO_A2       (GPIO_PORTA | GPIO_PIN4 | GPIO_INPUT        | GPIO_CNF_INPULLUP )
#define GPIO_A3       (GPIO_PORTA | GPIO_PIN5 | GPIO_INPUT        | GPIO_CNF_INPULLUP )
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
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ************************************************************************************/

void stm32_spiinitialize(void);

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ************************************************************************************/

void stm32_usbinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SPARK_SRC_SPARK_H */
