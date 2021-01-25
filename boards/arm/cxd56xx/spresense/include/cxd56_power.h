/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_power.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_POWER_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_POWER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PMIC_NONE           (0)
#define PMIC_TYPE_LSW       (1u << 8)
#define PMIC_TYPE_GPO       (1u << 9)
#define PMIC_TYPE_DDCLDO    (1u << 10)
#define PMIC_GET_TYPE(v)    ((v) & 0xff00)
#define PMIC_GET_CH(v)      ((v) & 0x00ff)
#define PMIC_LSW(n)         (PMIC_TYPE_LSW | (1u << (n)))
#define PMIC_GPO(n)         (PMIC_TYPE_GPO | (1u << (n)))
#define PMIC_DDCLDO(n)      (PMIC_TYPE_DDCLDO | (1u << (n)))

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
 * Name: board_pmic_read
 *
 * Description:
 *   Read the value from the specified sub address
 *
 * Input Parameter:
 *   addr - sub address
 *   buf - pointer to read buffer
 *   size - byte count of read
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int board_pmic_read(uint8_t addr, void *buf, uint32_t size);

/****************************************************************************
 * Name: board_pmic_write
 *
 * Description:
 *   Write the value to the specified sub address
 *
 * Input Parameter:
 *   addr - sub address
 *   buf - pointer to write buffer
 *   size - byte count of write
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

int board_pmic_write(uint8_t addr, void *buf, uint32_t size);

/****************************************************************************
 * Name: board_power_setup
 *
 * Description:
 *   Initial setup for board-specific power control
 *
 ****************************************************************************/

int board_power_setup(int status);

/****************************************************************************
 * Name: board_power_control
 *
 * Description:
 *   Power on/off the device on the board.
 *
 ****************************************************************************/

int board_power_control(int target, bool en);

/****************************************************************************
 * Name: board_power_control_tristate
 *
 * Description:
 *   Power on/off/HiZ the device on the board.
 *   (HiZ is available only for PMIC_TYPE_GPO.)
 *
 * Input Parameter:
 *   target : PMIC channel
 *   value : 1 (ON), 0 (OFF), -1(HiZ)
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

int board_power_control_tristate(int target, int value);

/****************************************************************************
 * Name: board_power_monitor
 *
 * Description:
 *   Get status of Power on/off the device on the board.
 *
 ****************************************************************************/

bool board_power_monitor(int target);

/****************************************************************************
 * Name: board_power_monitor_tristate
 *
 * Description:
 *   Get status of Power on/off/HiZ the device on the board.
 *
 * Input Parameter:
 *   target : PMIC channel
 *
 * Returned Value:
 *   1 (ON), 0 (OFF), -1(HiZ)
 *
 ****************************************************************************/

int board_power_monitor_tristate(int target);

/****************************************************************************
 * Name: board_flash_power_control
 *
 * Description:
 *   Power on/off the flash device on the board.
 *
 ****************************************************************************/

int board_flash_power_control(bool en);

/****************************************************************************
 * Name: board_flash_power_monitor
 *
 * Description:
 *   Get status of Power on/off the flash device on the board.
 *
 ****************************************************************************/

bool board_flash_power_monitor(void);

/****************************************************************************
 * Name: board_xtal_power_control
 *
 * Description:
 *   Power on/off the Xtal device on the board.
 *
 ****************************************************************************/

int board_xtal_power_control(bool en);

/****************************************************************************
 * Name: board_xtal_power_monitor
 *
 * Description:
 *   Get status of Power on/off the Xtal device on the board.
 *
 ****************************************************************************/

bool board_xtal_power_monitor(void);

/****************************************************************************
 * Name: board_lna_power_control
 *
 * Description:
 *   Power on/off the LNA device on the board.
 *
 ****************************************************************************/

int board_lna_power_control(bool en);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_POWER_H */
