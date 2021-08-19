/****************************************************************************
 * boards/arm/stm32l4/b-l475e-iot01a/include/board.h
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

#ifndef __BOARDS_ARM_STM32L4_B_L475E_IOT01A_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32L4_B_L475E_IOT01A_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#include <arch/board/b-l475e-iot01a_clock.h>

/* LED definitions **********************************************************/

/* LEDs
 *
 * Two user LEDs are available on pins PA5 and PB14.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *   ------------------- ---------------------------- ------
 *   SYMBOL                  Meaning                  LED
 *   ------------------- ---------------------------- ------
 */

#define LED_STARTED      0 /* NuttX has been started  OFF      */
#define LED_HEAPALLOCATE 0 /* Heap has been allocated OFF      */
#define LED_IRQSENABLED  0 /* Interrupts enabled      OFF      */
#define LED_STACKCREATED 1 /* Idle stack created      ON       */
#define LED_INIRQ        2 /* In an interrupt         N/C      */
#define LED_SIGNAL       2 /* In a signal handler     N/C      */
#define LED_ASSERTION    2 /* An assertion failed     N/C      */
#define LED_PANIC        3 /* The system has crashed  FLASH    */
#undef  LED_IDLE           /* MCU is is sleep mode    Not used */

/* Thus if LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/* SPSGRF-915 Spirit1 library definitions ***********************************/

/* The TX_WAIT_PCKT_PERIOD should equal the max packet tx time. */

#define SPIRIT_TX_WAIT_PCKT_PERIOD    50

/* The RX_WAIT_ACK_PERIOD is the period within which the ACK packet must be
 * received.
 * ca PACKET_LENGTH / DATA_RATE + ELABORATION_TIME + SPI_OPERATIONS_USED.
 */

#define SPIRIT_RX_WAIT_ACK_PERIOD     50

#define SPIRIT_CCA_THRESHOLD          -90.0     /* dBm */
#define SPIRIT_XTAL_FREQUENCY         50000000  /* Hz */
#define SPIRIT_XTAL_OFFSET_PPM        0
#define SPIRIT_BASE_FREQUENCY         868.0e6
#define SPIRIT_CHANNEL_SPACE          20e3
#define SPIRIT_CHANNEL_NUMBER         0
#define SPIRIT_MODULATION_SELECT      FSK
#define SPIRIT_DATARATE               38400
#define SPIRIT_FREQ_DEVIATION         20e3
#define SPIRIT_BANDWIDTH              100.5E3
#define SPIRIT_POWER_DBM              10.0
#define SPIRIT_PREAMBLE_LENGTH        PKT_PREAMBLE_LENGTH_04BYTES
#define SPIRIT_SYNC_LENGTH            PKT_SYNC_LENGTH_4BYTES
#define SPIRIT_SYNC_WORD              0x88888888
#define SPIRIT_CRC_MODE               PKT_CRC_MODE_16BITS_2
#define SPIRIT_CONTROL_LENGTH         PKT_CONTROL_LENGTH_0BYTES
#define SPIRIT_EN_FEC                 S_DISABLE
#define SPIRIT_EN_WHITENING           S_DISABLE
#define SPIRIT_RANGE_TYPE             RANGE_EXT_NONE /* RANGE_EXT_SKYWORKS */

/* Spirit1 IC version */

#define SPIRIT_VERSION                SPIRIT_VERSION_3_0

/* Alternate function pin selections ****************************************/

/* USART1: Connected to STLink Debug via PB6, PB7 */

#define GPIO_USART1_RX GPIO_USART1_RX_2
#define GPIO_USART1_TX GPIO_USART1_TX_2

/* UART4: Connected to arduino compatible pins DO/D1 via PA0, PA1 */

#define GPIO_UART4_RX GPIO_UART4_RX_1
#define GPIO_UART4_TX GPIO_UART4_TX_1

/* SPSGRF
 *
 * -------- ----------------------- ----------------
 * SPSGRF   Board Signal            STM32L4 pin
 * -------- ----------------------- ----------------
 * SPI_CLK  INTERNAL-SPI3_SCK       PC10 SPI3_SCK
 * SPI_MISO INTERNAL-SPI3_MISO      PC11 SPI3_MISO
 * SPI_MOSI INTERNAL-SPI3_MOSI      PC12 SPI3_MOSI
 * SPI_CS   SPSGRF-915-SPI3_CSN     PB5  GPIO_Output
 * GPIO(3)  SPSGRF-915-GPIO3_EXTI5  PE5  GPIO_EXTI5
 * GPIO(2)  N/C                     N/A
 * GPIO(1)  N/C                     N/A
 * GPIO(0)  N/C                     N/A
 * SDN      SPSGRF-915-SDN          PB15 GPIO_Output
 * -------- ----------------------- ----------------
 */

#define GPIO_SPI3_SCK  GPIO_SPI3_SCK_2
#define GPIO_SPI3_MISO GPIO_SPI3_MISO_2
#define GPIO_SPI3_MOSI GPIO_SPI3_MOSI_2

/* Quad SPI: connected to MX25R6435F external flash memory */

#define GPIO_QSPI_CS  (GPIO_QSPI_NCS_2     | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_IO0 (GPIO_QSPI_BK1_IO0_2 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_IO1 (GPIO_QSPI_BK1_IO1_2 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_IO2 (GPIO_QSPI_BK1_IO2_2 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_IO3 (GPIO_QSPI_BK1_IO3_2 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_SCK (GPIO_QSPI_CLK_2     | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32L4_B_L475E_IOT01A_INCLUDE_BOARD_H */
