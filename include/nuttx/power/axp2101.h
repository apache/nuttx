/****************************************************************************
 * include/nuttx/power/axp2101.h
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

#ifndef __INCLUDE_NUTTX_POWER_AXP2101_H
#define __INCLUDE_NUTTX_POWER_AXP2101_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_AXP2101

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AXP2101_I2C_ADDR        0x34  /* 7-bit I2C address */
#define AXP2101_CHIP_ID         0x4a  /* Content of the chip ID register */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

struct i2c_master_s;
struct battery_charger_dev_s;
struct battery_gauge_dev_s;

/****************************************************************************
 * Name: axp2101_initialize
 *
 * Description:
 *   Initialize the AXP2101 battery charger lower half.  The chip ID is
 *   checked and the VBAT, VBUS, VSYS and die temperature ADC channels are
 *   enabled.  The power rails are not changed.
 *
 * Input Parameters:
 *   i2c       - The I2C bus of the AXP2101
 *   addr      - The I2C address (AXP2101_I2C_ADDR)
 *   frequency - The I2C frequency
 *
 * Returned Value:
 *   An instance to register with battery_charger_register(), or NULL on
 *   failure.
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
axp2101_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                   uint32_t frequency);

/****************************************************************************
 * Name: axp2101_poweroff
 *
 * Description:
 *   Turn off all the outputs of the AXP2101.  The system starts again only
 *   on a power-on event (PWRON key, VBUS insertion, ...).
 *
 * Input Parameters:
 *   dev - The instance returned by axp2101_initialize()
 *
 * Returned Value:
 *   Zero on success, the outputs turn off right after.  A negated errno
 *   value on failure.
 *
 ****************************************************************************/

int axp2101_poweroff(FAR struct battery_charger_dev_s *dev);

#ifdef CONFIG_AXP2101_GAUGE

/****************************************************************************
 * Name: axp2101_gauge_initialize
 *
 * Description:
 *   Initialize the AXP2101 fuel gauge lower half.  It may be used with or
 *   without the charger lower half on the same chip.
 *
 * Input Parameters:
 *   i2c       - The I2C bus of the AXP2101
 *   addr      - The I2C address (AXP2101_I2C_ADDR)
 *   frequency - The I2C frequency
 *
 * Returned Value:
 *   An instance to register with battery_gauge_register(), or NULL on
 *   failure.
 *
 ****************************************************************************/

FAR struct battery_gauge_dev_s *
axp2101_gauge_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                         uint32_t frequency);

#endif /* CONFIG_AXP2101_GAUGE */

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AXP2101 */
#endif /* __INCLUDE_NUTTX_POWER_AXP2101_H */
