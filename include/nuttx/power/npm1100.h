/****************************************************************************
 * include/nuttx/power/npm1100.h
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

#ifndef __INCLUDE_NUTTX_POWER_NPM1100_H
#define __INCLUDE_NUTTX_POWER_NPM1100_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The nPM1100 has no register interface; its charge/error status is exposed
 * on two open-drain, active-low GPIOs. The board supplies callbacks that
 * return the raw pin level (true = high/de-asserted, false = low/asserted);
 * this driver applies the active-low semantics.
 */

struct npm1100_config_s
{
  CODE bool (*read_chg)(void);  /* CHG status pin level (low = charging) */
  CODE bool (*read_err)(void);  /* ERR status pin level (low = error)    */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: npm1100_initialize
 *
 * Description:
 *   Initialize the nPM1100 battery charger driver and return an instance of
 *   the lower-half interface that may be used with
 *   battery_charger_register().
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY_CHARGER - Upper-half battery charger driver support
 *
 * Input Parameters:
 *   config - Board-specific GPIO read callbacks.
 *
 * Returned Value:
 *   A pointer to the initialized lower-half driver instance, or NULL on
 *   failure.
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
  npm1100_initialize(FAR const struct npm1100_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_POWER_NPM1100_H */
