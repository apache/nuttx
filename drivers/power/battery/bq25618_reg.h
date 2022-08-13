/****************************************************************************
 * nuttx/drivers/power/battery/bq25618_reg.h
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

#ifndef __BQ25618_REG_H__
#define __BQ25618_REG_H__

#define BQ25618_MANUFACTURER "Texas Instruments"

#define BQ25618_INPUT_CURRENT_LIMIT       0x00
#define BQ25618_CHARGER_CONTROL_0         0x01
#define BQ25618_CHARGE_CURRENT_LIMIT      0x02
#define BQ25618_PRECHG_AND_TERM_CURR_LIM  0x03
#define BQ25618_BATTERY_VOLTAGE_LIMIT     0x04
#define BQ25618_CHARGER_CONTROL_1         0x05
#define BQ25618_CHARGER_CONTROL_2         0x06
#define BQ25618_CHARGER_CONTROL_3         0x07
#define BQ25618_CHARGER_STATUS_0          0x08
#define BQ25618_CHARGER_STATUS_1          0x09
#define BQ25618_CHARGER_STATUS_2          0x0a
#define BQ25618_PART_INFORMATION          0x0b
#define BQ25618_CHARGER_CONTROL_4         0x0c

#define BQ25618_DEV_ID_SHIFT              3
#define BQ25618_DEV_ID_MASK               0x28

#define BQ25618_RESET                     (1 << 7) /* Write 1 to Reset all register to default values */
#define BQ25618_WD_EN                     (3 << 4) /* Set 1 will enable and reset timeout */

#define BQ25618_INP_CURR_LIM_100MA        0x01     /* 100ma */
#define BQ25618_INP_CURR_LIM_200MA        0x02     /* 200ma */
#define BQ25618_INP_CURR_LIM_500MA        0x05     /* 500ma */
#define BQ25618_INP_CURR_LIM_800MA        0x08     /* 800ma */
#define BQ25618_INP_CURR_LIM_1600MA       0x10     /* 1600ma */
#define BQ25618_INP_CURR_LIM_2400MA       0x17     /* 2400ma */
#define BQ25618_INP_CURR_LIM_MASK         0x1f
#define BQ25618_IINDPM_STEP_UA            100
#define BQ25618_IINDPM_OFFSET_UA          100

#define BQ25618_VINDPM_MASK               0x0f
#define BQ25618_VINDPM_SHIFT              0
#define BQ25618_VINDPM_OFFSET_UV          3900
#define BQ25618_VINDPM_DEFAULT_UV         4500
#define BQ25618_VINDPM_STEP_UV            100

#define BQ25618_CHG_CURRENT_MASK          0x3f
#define BQ25618_CHG_CURRENT_SHIFT         0
#define BQ25618_ICHG_THRESH               0x3c
#define BQ25618_ICHG_STEP_UA              20
#define BQ25618_ICHG_THRESH_UA            1180
#define BQ25618_ICHG_DEFAULT_UA           340

#define BQ25618_CURR_MIN                  100
#define BQ25618_CURR_MAX                  1500

#define BQ25618_BATTERY_VOLT_ILIM_4V1     0x30
#define BQ25618_BATTERY_VOLT_ILIM_4V2     0x40
#define BQ25618_BATTERY_VOLT_ILIM_4V3     0x48
#define BQ25618_BATTERY_VOLT_ILIM_4V4     0x98
#define BQ25618_BATTERY_VOLT_ILIM_4V45    0xc0
#define BQ25618_BATTERY_VOLT_ILIM_4V5     0xe8

#define BQ25618_VBATREG_MASK              0xf8
#define BQ25618_VBATREG_BIT_SHIFT         3
#define BQ25618_VBATREG_THRESH            0x8
#define BQ25618_VBATREG_STEP_UV           100
#define BQ25618_VBATREG_THRESH_UV         4300
#define BQ25618_VBAT_DEFAULT_UV           4400
#define BQ25618_VOLT_MIN                  4100
#define BQ25618_VOLT_MAX                  4500

#define BQ25618_VBUS_STAT_MASK            0xd0
#define BQ25618_CHRG_STAT_MASK            0x18
#define BQ25618_PG_STAT_MASK              0x04
#define BQ25618_WDT_FAULT_MASK            0x80
#define BQ25618_BAT_FAULT_MASK            0x08
#define BQ25618_CHRG_FAULT_MASK           0x30
#define BQ25618_NTC_FAULT_MASK            0x07

#define BQ25618_VBUS_STAT_NO_INPUT        0
#define BQ25618_VBUS_STAT_USB_OTG         0xd0
#define BQ25618_CHRG_STAT_NOT_CHRGING     0
#define BQ25618_CHRG_STAT_CHRG_TERM       0x18

#define BQ25618_PRE_CURR_MIN              20
#define BQ25618_PRE_CURR_MASK             0xf0
#define BQ25618_PRE_CURR_SHIFT            4
#define BQ25618_PRE_CURR_STEP_UA          20
#define BQ25618_PRE_CURR_DEFAULT_UA       40

#define BQ25618_ITERM_CURR_MIN            20
#define BQ25618_ITERM_CURR_MASK           0x0f
#define BQ25618_ITERM_CURR_SHIFT          0
#define BQ25618_ITERM_STEP_UA             20
#define BQ25618_ITERM_DEFAULT_UA          60

#define BQ25618_CHRG_FAULT_INPUT          0x10
#define BQ25618_CHRG_FAULT_THERM          0x20
#define BQ25618_CHRG_FAULT_CST_EXPIRE     0x30
#define BQ25618_NTC_FAULT_WARM            0x02
#define BQ25618_NTC_FAULT_COOL            0x03
#define BQ25618_NTC_FAULT_COLD            0x05
#define BQ25618_NTC_FAULT_HOT             0x06

#define BQ25618_SHIPMODE_MASK             0x20
#define BQ25618_SHIPMODE_SHIFT            5

#define BQ25618_CONTROL_CHARGE_MASK       0x10
#define BQ25618_CONTROL_CHARGE_SHIFT      4
#endif
