/****************************************************************************
 * drivers/power/battery/cw2218.h
 * Lower half driver for cw2218 battery fuel gauge.
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

#ifndef __DRIVERS_POWER_CW2218_H
#define __DRIVERS_POWER_CW2218_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General Constants */
#define CW2218_DEVICE_ID                0xa0      /* chip id */

/* Standard Commands */
#define CW2218_COMMAND_VERSION          0x00      /* IC firmware version */
#define CW2218_COMMAND_VCELL            0x02      /* VCELL voltage conversion result */
#define CW2218_COMMAND_SOC              0x04      /* SOC result*/
#define CW2218_COMMAND_TEMP             0x06      /* Battery NTC thermistor temperature */
#define CW2218_COMMAND_CONFIG           0x08      /* IC configuration */
#define CW2218_COMMAND_INT_CONFIG       0x0a      /* Interrupt configuration */
#define CW2218_COMMAND_SOC_ALERT        0x0b      /* SOC interrupt threshold */
#define CW2218_COMMAND_TEMP_MAX         0x0c      /* Maximum temperature threshold */
#define CW2218_COMMAND_TEMP_MIN         0x0d      /* Minimum temperature threshold */
#define CW2218_COMMAND_CURRENT          0x0e      /* Current conversion result */
#define CW2218_COMMAND_T_HOST           0xa0      /* Host reported temperature */
#define CW2218_COMMAND_USER_CONF        0xa2      /* Configurable register for developer */
#define CW2218_COMMAND_CYCLECNT         0xa4      /* Battery charging cycles */
#define CW2218_COMMAND_SOH              0xa6      /* Battery state of health */

#define CW2218_COMMAND_BATINFO          0x10      /* battery profile reg */

/* Auxiliary Definitions */
#define SIZE_OF_PROFILE                 80        /* battery profile data size */

#define CW2218_RSENSE                   10        /* sampling resistor value */

#define COMPLEMENT_CODE_U16             0x8000    /* sign bit */

#define CW2218_CUR_MAGIC_PART1          763       /* current calculation factor */
#define CW2218_CUR_MAGIC_PART2          2         /* current calculation factor */
#define CW2218_CUR_MAGIC_PART3          100       /* current calculation factor */

#define CW2218_TEMP_MAGIC_PART1         10        /* temp calculation factor */
#define CW2218_TEMP_MAGIC_PART2         2         /* temp calculation factor */
#define CW2218_TEMP_MAGIC_PART3         400       /* temp calculation factor */

#define CW2218_VOL_MAGIC_PART1          5         /* voltage calculation factor */
#define CW2218_VOL_MAGIC_PART2          16        /* voltage calculation factor */

#define CW2218_CYCLE_MAGIC              16        /* cycle calculation factor */

#define CONFIG_MODE_RESTART             0x30      /* restart work mode command */
#define CONFIG_MODE_ACTIVE              0x00      /* active work mode command */
#define CONFIG_MODE_SLEEP               0xf0      /* sleep work mode command */
#define CONFIG_UPDATE_FLG               0x80      /* update flag bit */

#define CW2218_UI_FULL                  100       /* ui full soc value */
#define CW2218_SOC_MAGIC_BASE           256       /* current calculation factor */
#define CW2218_SOC_MAGIC_100            100       /* current calculation factor */

#define CW2218_SLEEP_20MS               20000     /* delat time */
#define CW2218_SLEEP_10MS               10000     /* delay time */
#define CW2218_SLEEP_100MS              100000    /* delay time */
#define CW2218_SLEEP_200MS              200000    /* delay time */

#define CW2218_NOT_ACTIVE               1         /* cw2218 not active mode flag */
#define CW2218_PROFILE_NOT_READY        2         /* cw2218 battery profile not ready flag */
#define CW2218_PROFILE_NEED_UPDATE      3         /* cw2218 battery profile need update flag */

#define GPIO_SOC_IRQ_VALUE              0x7F      /* cw2218 soc irq value */
#define TEMP_MAX_INT_VALUE              0xC8      /* cw2218 high temp interrupt value */
#define TEMP_MIN_INT_VALUE              0x28      /* cw2218 low temp interrupt value */

#define CW2218_SLEEP_COUNTS_SOC         70        /* cw2218 sleep count soc value */
#define CW2218_SLEEP_COUNTS             60        /* cw2218 sleep count value */

#define CW2218_BATTERY_INIT_5S          5000000   /* delay time */
#define CW_IIC_RETRY_NUM                3
#define CW2218_UI_FULL_TIME             10000000   /* delay time */
#define CW2218_UI_FULL_START_TIME       2000000    /* delay time */
#define FULL_WORK_NO_EXIT               0
#define FULL_WORK_EXIT                  1

#endif /* __DRIVERS_POWER_CW2218_H */
