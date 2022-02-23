/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_bcm20706.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_BCM20706_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_BCM20706_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

#ifdef CONFIG_BCM20706

/****************************************************************************
 * Name: board_bluetooth_pin_cfg
 *
 * Description:
 *   Initialize bcm20707 control pins, it must be called before any operation
 *   to do power control, wake up and reset.
 *
 ****************************************************************************/

int board_bluetooth_pin_cfg(void);

/****************************************************************************
 * Name: board_bluetooth_uart_pin_cfg
 *
 * Description:
 *   Setup UART pin configuration for bcm20706.
 *
 ****************************************************************************/

int board_bluetooth_uart_pin_cfg(void);

/****************************************************************************
 * Name: board_bluetooth_reset
 *
 * Description:
 *   Reset bcm20707 chip
 *
 ****************************************************************************/

void board_bluetooth_reset(void);

/****************************************************************************
 * Name: board_bluetooth_power_control
 *
 * Description:
 *   Power on/off bcm20707 chip
 *
 ****************************************************************************/

int board_bluetooth_power_control(bool en);

/****************************************************************************
 * Name: board_bluetooth_enable_sleep
 *
 * Description:
 *   Enable/disable bcm20707 enters sleep mode
 *
 ****************************************************************************/

void board_bluetooth_enable_sleep(bool en);
#endif /* CONFIG_BCM20706 */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_BCM20706_H */
