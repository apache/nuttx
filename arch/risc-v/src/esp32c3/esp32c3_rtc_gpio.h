/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_rtc_gpio.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_RTC_GPIO_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_RTC_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "hardware/esp32c3_rtccntl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTC GPIO channels */

#define RTC_GPIO_NUMBER             5

#define RTCIO_GPIO0_CHANNEL         0   /* RTCIO_CHANNEL_0 */
#define RTCIO_CHANNEL_0_GPIO_NUM    4

#define RTCIO_GPIO1_CHANNEL         1   /* RTCIO_CHANNEL_1 */
#define RTCIO_CHANNEL_1_GPIO_NUM    5

#define RTCIO_GPIO2_CHANNEL         2   /* RTCIO_CHANNEL_2 */
#define RTCIO_CHANNEL_2_GPIO_NUM    6

#define RTCIO_GPIO3_CHANNEL         3   /* RTCIO_CHANNEL_3 */
#define RTCIO_CHANNEL_3_GPIO_NUM    8

#define RTCIO_GPIO4_CHANNEL         4   /* RTCIO_CHANNEL_4 */
#define RTCIO_CHANNEL_4_GPIO_NUM    9

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_RTCIO_IRQ
static const int rtc_irq_reg_shift[ESP32C3_NIRQ_RTCIO] =
{
  RTC_CNTL_SLP_WAKEUP_INT_ENA_S,
  RTC_CNTL_SLP_REJECT_INT_ENA_S,
  RTC_CNTL_WDT_INT_ENA_S,
  RTC_CNTL_BROWN_OUT_INT_ENA_S,
  RTC_CNTL_MAIN_TIMER_INT_ENA_S,
  RTC_CNTL_SWD_INT_ENA_S,
  RTC_CNTL_XTAL32K_DEAD_INT_ENA_S,
  RTC_CNTL_GLITCH_DET_INT_ENA_S,
  RTC_CNTL_BBPLL_CAL_INT_ENA_S
};
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_rtcioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   RTC IRQs.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_RTCIO_IRQ
void esp32c3_rtcioirqinitialize(void);
#else
#  define esp32c3_rtcioirqinitialize()
#endif

/****************************************************************************
 * Name: esp32c3_rtcioirqenable
 *
 * Description:
 *   Enable the interrupt for the specified RTC peripheral IRQ.
 *
 * Input Parameters:
 *   irq - The IRQ number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_RTCIO_IRQ
void esp32c3_rtcioirqenable(int irq);
#else
#  define esp32c3_rtcioirqenable(irq)
#endif

/****************************************************************************
 * Name: esp32c3_rtcioirqdisable
 *
 * Description:
 *   Disable the interrupt for the specified RTC peripheral IRQ.
 *
 * Input Parameters:
 *   irq - The IRQ number.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_RTCIO_IRQ
void esp32c3_rtcioirqdisable(int irq);
#else
#  define esp32c3_rtcioirqdisable(irq)
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_RTC_GPIO_H */
