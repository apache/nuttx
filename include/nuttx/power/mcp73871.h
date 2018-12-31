/****************************************************************************
 * drivers/power/mcp73871.h
 * Lower half driver for MCP73871 battery charger
 *
 *   Copyright (C) 2018 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __DRIVERS_POWER_MCP73871_H
#define __DRIVERS_POWER_MCP73871_H

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

#endif /* __DRIVERS_POWER_MCP73871_H */
