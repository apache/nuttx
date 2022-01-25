/****************************************************************************
 * boards/arm/kinetis/kwikstik-k40/src/kwikstik-k40.h
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

#ifndef __BOARDS_ARM_KINETIS_KWIKSTK_K40_SRC_KWIKSTIK_K40_H
#define __BOARDS_ARM_KINETIS_KWIKSTK_K40_SRC_KWIKSTIK_K40_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/kinetis/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many SPI modules does this chip support? The LM3S6918 supports 2 SPI
 * modules (others may support more -- in such case, the following must be
 * expanded).
 */

#if KINETIS_NSPI < 1
#  undef CONFIG_KINETIS_SPI1
#  undef CONFIG_KINETIS_SPI2
#elif KINETIS_NSPI < 2
#  undef CONFIG_KINETIS_SPI2
#endif

/* KwikStik-K40 GPIOs *******************************************************/

/* On-Board Connections
 *
 * ------------------- -------------------------- -------- ------------------
 * FEATURE             CONNECTION                 PORT/PIN PIN FUNCTION
 * ------------------- -------------------------- -------- ------------------
 * Audio Jack Output   Audio Amp On               PTE28    PTE28
 *                     Audio Output               DAC1_OUT DAC1_OUT
 *                     Volume Up                  PTD10    PTD10
 *                     Volume Down                PTD11    PTD11
 * Buzzer              Audio Out                  PTA8     FTM1_CH0
 * Microphone          Microphone input           PTA7     ADC0_SE10
 * SD Card Slot        SD Clock                   PTE2     SDHC0_DCLK
 *                     SD Command                 PTE3     SDHC0_CMD
 *                     SD Data0                   PTD12    SDHC0_D4
 *                     SD Data1                   PTD13    SDHC0_D5
 *                     SD Data2                   PTD14    SDHC0_D6
 *                     SD Data3                   PTD15    SDHC0_D7
 *                     SD Card Detect             PTE27    PTE27
 *                     SD Card On                 PTE6     PTE6
 * Infrared Port       IR Transmit                PTE4     IR_TX
 *                     IR Receive                 PTA13    CMP2_IN0
 * Touch Pads          E1 / Touch                 PTB0     TSI0_CH0
 *                     E2 / Touch                 PTA4     TSI0_CH5
 *                     E3 / Touch                 PTA24    PTA24
 *                     E4 / Touch                 PTA25    PTA25
 *                     E5 / Touch                 PTA26    PTA26
 *                     E6 / Touch                 PTA27    PTA27
 */

#define GPIO_SD_CARDDETECT (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTE | PIN27)
#define GPIO_SD_CARDON     (GPIO_HIGHDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTE | PIN6)

/* Connections via the General Purpose Tower Plug-in (TWRPI) Socket
 * ------------------- -------------------------- -------- ------------------
 * FEATURE             CONNECTION                 PORT/PIN PIN FUNCTION
 * ------------------- -------------------------- -------- ------------------
 * General Purpose     TWRPI AN0 (J8 Pin 8)       ?        ADC0_DP0/ADC1_DP3
 * TWRPI Socket        TWRPI AN1 (J8 Pin 9)       ?        ADC0_DM0/ADC1_DM3
 *                     TWRPI AN2 (J8 Pin 12)      ?        ADC1_DP0/ADC0_DP3
 *                     TWRPI ID0 (J8 Pin 17)      ?        ADC0_DP1
 *                     TWRPI ID1 (J8 Pin 18)      ?        ADC0_DM1
 *                     TWRPI I2C SCL (J9 Pin 3)   PTC10    I2C1_SCL
 *                     TWRPI I2C SDA (J9 Pin 4)   PTC11    I2C1_SDA
 *                     TWRPI SPI MISO (J9 Pin 9)  PTB23    SPI2_SIN
 *                     TWRPI SPI MOSI (J9 Pin 10) PTB22    SPI2_SOUT
 *                     TWRPI SPI SS (J9 Pin 11)   PTB20    SPI2_PCS0
 *                     TWRPI SPI CLK (J9 Pin 12)  PTB21    SPI2_SCK
 *                     TWRPI GPIO0 (J9 Pin 15)    PTC12    PTC12
 *                     TWRPI GPIO1 (J9 Pin 16)    PTB9     PTB9
 *                     TWRPI GPIO2 (J9 Pin 17)    PTB10    PTB10
 *                     TWRPI GPIO3 (J9 Pin 18)    PTC5     PTC5
 *                     TWRPI GPIO4 (J9 Pin 19)    PTA5     PTA5
 */

/* Connections via the Tower Primary Connector Side A
 * --- -------------------- --------------------------------
 * PIN NAME                 USAGE
 * --- -------------------- --------------------------------
 * A9  GPIO9 / CTS1         PTE10/UART_CTS
 * A43 RXD1                 PTE9/UART_RX
 * A44 TXD1                 PTE8/UART_TX
 * A63 RSTOUT_b             PTA9/FTM1_CH1
 */

/* Connections via the Tower Primary Connector Side B
 * --- -------------------- --------------------------------
 * PIN NAME                 USAGE
 * --- -------------------- --------------------------------
 * B21 GPIO1 / RTS1         PTE7/UART_RTS
 * B37 PWM7                 PTA8/FTM1_CH0
 * B38 PWM6                 PTA9/FTM1_CH1
 * B41 CANRX0               PTE25/CAN1_RX
 * B42 CANTX0               PTE24/CAN1_TX
 * B44 SPI0_MISO            PTA17/SPI0_SIN
 * B45 SPI0_MOSI            PTA16/SPI0_SOUT
 * B46 SPI0_CS0_b           PTA14/SPI0_PCS0
 * B48 SPI0_CLK             PTA15/SPI0_SCK
 * B50 SCL1                 PTE1/I2C1_SCL
 * B51 SDA1                 PTE0/I2C1_SDA
 * B52 GPIO5 / SD_CARD_DET  PTA16
 */

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
 * Name: kinetis_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the KwikStik-K40
 *   board.
 *
 ****************************************************************************/

extern void weak_function kinetis_spidev_initialize(void);

/****************************************************************************
 * Name: kinetis_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the KwikStik-K40 board.
 *
 ****************************************************************************/

extern void weak_function kinetis_usbinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_KINETIS_KWIKSTK_K40_SRC_KWIKSTIK_K40_H */
