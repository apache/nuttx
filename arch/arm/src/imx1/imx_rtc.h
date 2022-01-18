/****************************************************************************
 * arch/arm/src/imx1/imx_rtc.h
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

#ifndef __ARCH_ARM_SRC_IMX1_IMX_RTC_H
#define __ARCH_ARM_SRC_IMX1_IMX_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTC Register Offsets *****************************************************/

#define RTC_HOURMIN_OFFSET           0x0000
#define RTC_SECOND_OFFSET            0x0004
#define RTC_ALRM_HM_OFFSET           0x0008
#define RTC_ALRM_SEC_OFFSET          0x000c
#define RTC_RTCCTL_OFFSET            0x0010
#define RTC_RTCISR_OFFSET            0x0014
#define RTC_RTCIENR_OFFSET           0x0018
#define RTC_STPWCH_OFFSET            0x001c
#define RTC_DAYR_OFFSET              0x0020
#define RTC_DAYALARM_OFFSET          0x0024
#define RTC_TEST1_OFFSET             0x0028
#define RTC_TEST2_OFFSET             0x002c
#define RTC_TEST3_OFFSET             0x0030

/* RTC Register Addresses ***************************************************/

#define IMX_RTC_HOURMIN             (IMX_RTC_VBASE + RTC_HOURMIN_OFFSET)
#define IMX_RTC_SECOND              (IMX_RTC_VBASE + RTC_SECOND_OFFSET)
#define IMX_RTC_ALRM_HM             (IMX_RTC_VBASE + RTC_ALRM_HM_OFFSET)
#define IMX_RTC_ALRM_SEC            (IMX_RTC_VBASE + RTC_ALRM_SEC_OFFSET)
#define IMX_RTC_RTCCTL              (IMX_RTC_VBASE + RTC_RTCCTL_OFFSET)
#define IMX_RTC_RTCISR              (IMX_RTC_VBASE + RTC_RTCISR_OFFSET)
#define IMX_RTC_RTCIENR             (IMX_RTC_VBASE + RTC_RTCIENR_OFFSET)
#define IMX_RTC_STPWCH              (IMX_RTC_VBASE + RTC_STPWCH_OFFSET)
#define IMX_RTC_DAYR                (IMX_RTC_VBASE + RTC_DAYR_OFFSET)
#define IMX_RTC_DAYALARM            (IMX_RTC_VBASE + RTC_DAYALARM_OFFSET)
#define IMX_RTC_TEST1               (IMX_RTC_VBASE + RTC_TEST1_OFFSET)
#define IMX_RTC_TEST2               (IMX_RTC_VBASE + RTC_TEST2_OFFSET)
#define IMX_RTC_TEST3               (IMX_RTC_VBASE + RTC_TEST3_OFFSET)

/* RTC Register Bit Definitions *********************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_IMX1_IMX_RTC_H */
