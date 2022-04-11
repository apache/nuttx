/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_sdcard.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_SDCARD_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_SDCARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

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

/****************************************************************************
 * Name: board_sdcard_initialize
 *
 * Description:
 *   Initialize SD Card on the board.
 *
 ****************************************************************************/

int board_sdcard_initialize(void);

/****************************************************************************
 * Name: board_sdcard_pin_initialize
 *
 * Description:
 *   Initialize SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_initialize(void);

/****************************************************************************
 * Name: board_sdcard_pin_finalize
 *
 * Description:
 *   Finalize SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_finalize(void);

/****************************************************************************
 * Name: board_sdcard_pin_configuraton
 *
 * Description:
 *   Configure SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_configuraton(void);

/****************************************************************************
 * Name: board_sdcard_pin_enable
 *
 * Description:
 *   Enable SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_enable(void);

/****************************************************************************
 * Name: board_sdcard_pin_disable
 *
 * Description:
 *   Disable SD Card pins on the board.
 *
 ****************************************************************************/

void board_sdcard_pin_disable(void);

/****************************************************************************
 * Name: board_sdcard_set_high_voltage
 *
 * Description:
 *   Set SD Card IO voltage to 3.3V
 *
 ****************************************************************************/

void board_sdcard_set_high_voltage(void);

/****************************************************************************
 * Name: board_sdcard_set_low_voltage
 *
 * Description:
 *   Set SD Card IO voltage to 1.8V
 *
 ****************************************************************************/

void board_sdcard_set_low_voltage(void);

/****************************************************************************
 * Name: board_sdcard_set_state_cb
 *
 * Description:
 *   Register callback function to notify state change of card slot.
 *   This function is called by board_ioctl()
 *    as BOARDIOC_SDCARD_SETNOTIFYCB command.
 *
 ****************************************************************************/

int board_sdcard_set_state_cb(uintptr_t cb);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_SDCARD_H */
