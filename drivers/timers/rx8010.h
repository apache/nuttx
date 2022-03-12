/****************************************************************************
 * drivers/timers/rx8010.h
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

#ifndef __DRIVERS_TIMERS_RX8010_H
#define __DRIVERS_TIMERS_RX8010_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define RX8010_TIME_SECR            0x10
#define RX8010_MIN                  0x11
#define RX8010_HOUR                 0x12
#define RX8010_WDAY                 0x13
#define RX8010_MDAY                 0x14
#define RX8010_MONTH                0x15
#define RX8010_YEAR                 0x16
#define RX8010_RESV17               0x17
#define RX8010_ALMIN                0x18
#define RX8010_ALHOUR               0x19
#define RX8010_ALWDAY               0x1A
#define RX8010_TCOUNT0              0x1B
#define RX8010_TCOUNT1              0x1C
#define RX8010_EXT                  0x1D
#define RX8010_FLAG                 0x1E
#define RX8010_CTRL                 0x1F
/* 0x20 to 0x2F are user registers */
#define RX8010_RESV30               0x30
#define RX8010_RESV31               0x31
#define RX8010_IRQ                  0x32

#define RX8010_TIME_SEC_BCDMASK     0x7f
#define RX8010_TIME_MIN_BCDMASK     0x7f
#define RX8010_TIME_HOUR24_BCDMASK  0x3f
#define RX8010_TIME_DAY_MASK        0x07
#define RX8010_TIME_DATE_BCDMASK    0x3f
#define RX8010_TIME_MONTH_BCDMASK   0x1f
#define RX8010_TIME_YEAR_BCDMASK    0xff

#define RX8010_EXT_WADA             BIT(3)

#define RX8010_FLAG_VLF             (1 << 1)
#define RX8010_FLAG_AF              (1 << 3)
#define RX8010_FLAG_TF              (1 << 4)
#define RX8010_FLAG_UF              (1 << 5)

#define RX8010_CTRL_AIE             BIT(3)
#define RX8010_CTRL_UIE             BIT(5)
#define RX8010_CTRL_STOP            BIT(6)
#define RX8010_CTRL_TEST            BIT(7)

#define RX8010_ALARM_AE             IT(7)

#endif
