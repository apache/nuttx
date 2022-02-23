/****************************************************************************
 * include/nuttx/power/mcp73871.h
 * Lower half driver for MCP73871 battery charger
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

#ifndef __INCLUDE_NUTTX_POWER_MCP73871_H
#define __INCLUDE_NUTTX_POWER_MCP73871_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is the TABLE 5-1 STATUS OUTPUTS from MCP73871 datasheet
 * (20002090C.pdf)
 */

#define MCP73871_FAULT          0
#define MCP73871_CHARGING       2
#define MCP73871_BAT_LOW        3
#define MCP73871_CHG_COMPLETE   4
#define MCP73871_NO_BATTERY     6 /* Also could be Shutdown mode */
#define MCP73871_NO_INPUT_PWR   7 /* Also could be Shutdown mode */

/* MCP73871 configuration data structure */

struct mcp73871_config_s
{
  CODE int (*read_stat1)(void);
  CODE int (*read_stat2)(void);
  CODE int (*read_pg)(void);
  CODE void (*set_chg_ce)(bool on);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mcp73871_initialize
 *
 * Description:
 *   Initialize the BQ2425x battery driver and return an instance of the
 *   lower_half interface that may be used with battery_charger_register();
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY_CHARGER - Upper half battery driver support
 *
 * Input Parameters:
 *   config   - The config structure with function pointers to read/write.
 *
 * Returned Value:
 *   A pointer to the initialized lower-half driver instance.  A NULL
 *   pointer is returned on a failure to initialize the BQ2425x lower half.
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
           mcp73871_initialize(FAR struct mcp73871_config_s *config);

#endif /* __INCLUDE_NUTTX_POWER_MCP73871_H */
