/****************************************************************************
 * arch/arm/src/str71x/str71x_rtc.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_RTC_H
#define __ARCH_ARM_SRC_STR71X_STR71X_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTC Registers ************************************************************/

#define STR71X_RTC_CRH     (STR71X_RTC_BASE + 0x0000) /* 16-bits wide */
#define STR71X_RTC_CRL     (STR71X_RTC_BASE + 0x0004) /* 16-bits wide */
#define STR71X_RTC_PRLH    (STR71X_RTC_BASE + 0x0008) /* 16-bits wide */
#define STR71X_RTC_PRLL    (STR71X_RTC_BASE + 0x000c) /* 16-bits wide */
#define STR71X_RTC_DIVH    (STR71X_RTC_BASE + 0x0010) /* 16-bits wide */
#define STR71X_RTC_DIVL    (STR71X_RTC_BASE + 0x0014) /* 16-bits wide */
#define STR71X_RTC_CNTH    (STR71X_RTC_BASE + 0x0018) /* 16-bits wide */
#define STR71X_RTC_CNTL    (STR71X_RTC_BASE + 0x001c) /* 16-bits wide */
#define STR71X_RTC_ALRH    (STR71X_RTC_BASE + 0x0020) /* 16-bits wide */
#define STR71X_RTC_ALRL    (STR71X_RTC_BASE + 0x0024) /* 16-bits wide */

/* Register bit settings ****************************************************/

/* RTC control register */

#define STR71X_RTCCRH_SEN    (0x0001) /* Bit 0: Second interrupt enable */
#define STR71X_RTCCRH_AEN    (0x0002) /* Bit 1: Alarm interrupt enable */
#define STR71X_RTCCRH_OWEN   (0x0004) /* Bit 2: Overflow interrupt enable */
#define STR71X_RTCCRH_GEN    (0x0008) /* Bit 3: Global interrupt enable */

#define STR71X_RTCCRL_SIR    (0x0001) /* Bit 0: Second interrupt request */
#define STR71X_RTCCRL_AIR    (0x0002) /* Bit 1: Alarm interrupt request */
#define STR71X_RTCCRL_OWIR   (0x0004) /* Bit 2: Overflow interrupt request */
#define STR71X_RTCCRL_GIR    (0x0008) /* Bit 3: Global interrupt request */
#define STR71X_RTCCRL_CNF    (0x0010) /* Bit 4: Enter configuration mode */
#define STR71X_RTCCRL_RTOFF  (0x0020) /* Bit 5: RTC Operation Off */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_RTC_H */
