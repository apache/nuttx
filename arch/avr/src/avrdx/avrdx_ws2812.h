/****************************************************************************
 * arch/avr/src/avrdx/avrdx_ws2812.h
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

#ifndef __ARCH_AVR_SRC_AVRDX_AVRDX_WS2812_H
#define __ARCH_AVR_SRC_AVRDX_AVRDX_WS2812_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <assert.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Lane size 1 is guaranteed, others are not. (That's why first #if
 * does not check the size but others do.) Each lane has a static
 * value (or is not defined if not enabled.)
 *
 * For future edits: lane numbers MUST NOT be redefined, the code uses
 * the numeric values for various decisions, offset calculations etc.
 *
 * This enum is internally converted to a pointer, its size must fit
 * pointer size. Asserted below the definition.
 */

enum avrdx_ws2812_lane_e
{
#ifdef CONFIG_AVR_AVRDX_WS2812_USART0
  AVRDX_WS2812_LANE_USART0 = 0,
#endif
#ifdef CONFIG_AVR_AVRDX_WS2812_USART1
  AVRDX_WS2812_LANE_USART1 = 1,
#endif
#ifdef CONFIG_AVR_AVRDX_WS2812_USART2
  AVRDX_WS2812_LANE_USART2 = 2,
#endif
#ifdef CONFIG_AVR_AVRDX_WS2812_USART3
  AVRDX_WS2812_LANE_USART3 = 3,
#endif
#ifdef CONFIG_AVR_AVRDX_WS2812_USART4
  AVRDX_WS2812_LANE_USART4 = 4,
#endif
#ifdef CONFIG_AVR_AVRDX_WS2812_USART5
  AVRDX_WS2812_LANE_USART5 = 5
#endif
};

static_assert(sizeof(enum avrdx_ws2812_lane_e) <= sizeof(FAR void *),
              "Lane enum does not fit into pointer size.");

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: <Inline function name>
 *
 * Description:
 *   Description of the operation of the inline function.
 *
 * Input Parameters:
 *   A list of input parameters, one-per-line, appears here along with a
 *   description of each input parameter.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: <Global function name>
 *
 * Description:
 *   Description of the operation of the function.
 *
 * Input Parameters:
 *   A list of input parameters, one-per-line, appears here along with a
 *   description of each input parameter.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_ws2812_register
 *
 *   Initializer for one WS2812 lane. Lane is a set of WS2812 LEDs
 *   connected serially and driven from single hardware device
 *   on the MCU side.
 *
 *   Note that this function triggers transmission of its freshly
 *   allocated and zeroed buffer, thus resetting the LEDs
 *   and turning them all of. Since the function does not know
 *   if it runs after a power-on or after a reset (and, therefore,
 *   how much time elapsed from last transmission), it will simply
 *   wait.
 *
 * Input Parameters:
 *   path - device path for this LED lane
 *   count - LED count. Limited to 255 regardless of upper half setting
 *           (The device does not have memory for more.)
 *   lane - enum avrdx_ws2812_lane_e value. Selects peripheral to which
 *          are the LEDs connected on the board
 *   has_white - bool. True if the lane has LEDs with white component.
 *               Ignored if CONFIG_WS2812_HAS_WHITE is unset.
 *
 * Returned Value:
 *   OK or negated errno.
 *   ENOMEM - if the LED count exceeds limit (in addition to memory
 *            allocation failure.)
 *
 ****************************************************************************/

int avrdx_ws2812_register(const char *path, uint8_t count,
                          enum avrdx_ws2812_lane_e lane,
                          bool has_white);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */

#endif
