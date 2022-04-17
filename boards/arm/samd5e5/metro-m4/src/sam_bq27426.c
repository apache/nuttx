/****************************************************************************
 * boards/arm/samd5e5/metro-m4/src/sam_bq27426.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_gauge.h>

#include <nuttx/power/bq27426.h>
#include <nuttx/power/battery_ioctl.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/sam_i2c_master.h"
#include "metro-m4.h"

#include <arch/board/board.h>
#include "sam_config.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_BQ27426

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bq24298_initialize
 *
 * Description:
 *   Called to configure bq24298
 *
 ****************************************************************************/

int sam_bq27426_initialize(const char *devname)
{
    struct battery_gauge_dev_s *bq27426_m4;

    bq27426_m4 = (struct battery_gauge_dev_s *)bq27426_initialize(
                                                   g_i2c5_dev,
                                                   BQ27426_I2C_ADDRESS,
                                                   100000);
  return battery_gauge_register(devname, bq27426_m4);
}

#endif /* CONFIG_BQ27426 */