/****************************************************************************
 * boards/xtensa/esp32/heltec_wifi_lora32/src/heltec_wifi_lora32.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_XTENSA_ESP32_HELTEC_WIFI_LORA32_SRC_HELTEC_WIFI_LORA32_H
#define __BOARDS_XTENSA_ESP32_HELTEC_WIFI_LORA32_SRC_HELTEC_WIFI_LORA32_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Heltec WiFi LoRa32 GPIOs *************************************************/

/* SX1276 pins */

#define GPIO_SX127X_RESET   14    /* RESET connected to IO14 */
#define GPIO_SX127X_DIO0    26    /* DIO0 connected to IO26  */

/* BOOT Button */

#define BUTTON_BOOT  0

/* LED
 *
 * This is an externally connected LED used for testing.
 */

#define GPIO_LED1             2

/* PCNT Quadrature Encoder IDs */

#define PCNT_QE0_ID           0

/* MCP2515 Interrupt pin */

#define GPIO_MCP2515_IRQ      22

/* TIMERS */

#define TIMER0 0
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3

/* ONESHOT */

#define ONESHOT_TIMER         1
#define ONESHOT_RESOLUTION_US 1

/* RMT gpio */

#define RMT_RXCHANNEL       1
#define RMT_TXCHANNEL       0

#ifdef CONFIG_RMT_LOOP_TEST_MODE
#  define RMT_INPUT_PIN     0
#  define RMT_OUTPUT_PIN    0
#else
#  define RMT_INPUT_PIN     2
#  define RMT_OUTPUT_PIN    4
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library via board_app_initialize()
 *
 ****************************************************************************/

int esp32_bringup(void);

/****************************************************************************
 * Name: esp32_lpwaninitialize
 *
 * Description:
 *   Initialize the SX127x driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X
int esp32_lpwaninitialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32_HELTEC_WIFI_LORA32_SRC_HELTEC_WIFI_LORA32_H */
