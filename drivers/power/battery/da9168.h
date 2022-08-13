/****************************************************************************
 * nuttx/drivers/power/battery/da9168.h
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

#ifndef __DRIVERS_POWER_DA9168_H
#define __DRIVERS_POWER_DA9168_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Defines */

#define DA9168_PMC_STATUS_00                    0x0000
#define DA9168_PMC_STATUS_01                    0x0001
#define DA9168_PMC_STATUS_02                    0x0002
#define DA9168_PMC_STATUS_03                    0x0003
#define DA9168_PMC_STATUS_04                    0x0004
#define DA9168_PMC_EVENT_00                     0x0005
#define DA9168_PMC_EVENT_01                     0x0006
#define DA9168_PMC_EVENT_02                     0x0007
#define DA9168_PMC_EVENT_03                     0x0008
#define DA9168_PMC_EVENT_04                     0x0009
#define DA9168_PMC_MASK_00                      0x000a
#define DA9168_PMC_MASK_01                      0x000b
#define DA9168_PMC_MASK_02                      0x000c
#define DA9168_PMC_MASK_03                      0x000d
#define DA9168_PMC_MASK_04                      0x000e
#define DA9168_PMC_SYS_00                       0x000f
#define DA9168_PMC_SYS_01                       0x0010
#define DA9168_PMC_SYS_02                       0x0011
#define DA9168_PMC_SYS_03                       0x0012
#define DA9168_PMC_SYS_04                       0x0013
#define DA9168_PMC_SYS_05                       0x0014
#define DA9168_PMC_SYS_06                       0x0015
#define DA9168_PMC_CHG_00                       0x0016
#define DA9168_PMC_CHG_01                       0x0017
#define DA9168_PMC_CHG_02                       0x0018
#define DA9168_PMC_CHG_03                       0x0019
#define DA9168_PMC_CHG_04                       0x001a
#define DA9168_PMC_CHG_05                       0x001b
#define DA9168_PMC_CHG_06                       0x001c
#define DA9168_PMC_LDO_00                       0x001d
#define DA9168_PMC_LDO_01                       0x001e
#define DA9168_PMC_LDO_02                       0x001f
#define DA9168_OTP_DEVICE_ID                    0x0042
#define DA9168_OTP_VARIANT_ID                   0x0043
#define DA9168_OTP_CONFIG_ID                    0x0044

/* Bit Fields */

/* DA9168_PMC_STATUS_00 = 0x0000 */

#define DA9168_S_VBAT_OK_SHIFT                  0
#define DA9168_S_VBAT_OK_MASK                   (0x01 << 0)
#define DA9168_S_VSYS_OK_SHIFT                  1
#define DA9168_S_VSYS_OK_MASK                   (0x01 << 1)
#define DA9168_S_VMID_OK_SHIFT                  2
#define DA9168_S_VMID_OK_MASK                   (0x01 << 2)
#define DA9168_S_VBUS_OK_SHIFT                  3
#define DA9168_S_VBUS_OK_MASK                   (0x01 << 3)
#define DA9168_S_VBAT_OC_SHIFT                  4
#define DA9168_S_VBAT_OC_MASK                   (0x01 << 4)
#define DA9168_S_VMID_OC_SHIFT                  5
#define DA9168_S_VMID_OC_MASK                   (0x01 << 5)
#define DA9168_S_VBUS_IINDPM_SHIFT              6
#define DA9168_S_VBUS_IINDPM_MASK               (0x01 << 6)
#define DA9168_S_VBUS_VINDPM_SHIFT              7
#define DA9168_S_VBUS_VINDPM_MASK               (0x01 << 7)

/* DA9168_PMC_STATUS_01 = 0x0001 */

#define DA9168_S_VBAT_UV_SHIFT                  0
#define DA9168_S_VBAT_UV_MASK                   (0x01 << 0)
#define DA9168_S_VSYS_UV_SHIFT                  1
#define DA9168_S_VSYS_UV_MASK                   (0x01 << 1)
#define DA9168_S_VMID_UV_SHIFT                  2
#define DA9168_S_VMID_UV_MASK                   (0x01 << 2)
#define DA9168_S_VBUS_UV_SHIFT                  3
#define DA9168_S_VBUS_UV_MASK                   (0x01 << 3)
#define DA9168_S_VBAT_OV_SHIFT                  4
#define DA9168_S_VBAT_OV_MASK                   (0x01 << 4)
#define DA9168_S_VSYS_OV_SHIFT                  5
#define DA9168_S_VSYS_OV_MASK                   (0x01 << 5)
#define DA9168_S_VMID_OV_SHIFT                  6
#define DA9168_S_VMID_OV_MASK                   (0x01 << 6)
#define DA9168_S_VBUS_OV_SHIFT                  7
#define DA9168_S_VBUS_OV_MASK                   (0x01 << 7)

/* DA9168_PMC_STATUS_02 = 0x0002 */

#define DA9168_S_TS_OFF_SHIFT                   0
#define DA9168_S_TS_OFF_MASK                    (0x01 << 0)
#define DA9168_S_TS_COLD_SHIFT                  1
#define DA9168_S_TS_COLD_MASK                   (0x01 << 1)
#define DA9168_S_TS_COOL_SHIFT                  2
#define DA9168_S_TS_COOL_MASK                   (0x01 << 2)
#define DA9168_S_TS_WARM_SHIFT                  3
#define DA9168_S_TS_WARM_MASK                   (0x01 << 3)
#define DA9168_S_TS_HOT_SHIFT                   4
#define DA9168_S_TS_HOT_MASK                    (0x01 << 4)
#define DA9168_S_WD_TIMER_SHIFT                 5
#define DA9168_S_WD_TIMER_MASK                  (0x01 << 5)
#define DA9168_S_TSD_WARN_SHIFT                 6
#define DA9168_S_TSD_WARN_MASK                  (0x01 << 6)
#define DA9168_S_TSD_CRIT_SHIFT                 7
#define DA9168_S_TSD_CRIT_MASK                  (0x01 << 7)

/* DA9168_PMC_STATUS_03 = 0x0003 */

#define DA9168_S_CHG_DONE_SHIFT                 0
#define DA9168_S_CHG_DONE_MASK                  (0x01 << 0)
#define DA9168_S_CHG_CV_SHIFT                   1
#define DA9168_S_CHG_CV_MASK                    (0x01 << 1)
#define DA9168_S_CHG_CC_SHIFT                   2
#define DA9168_S_CHG_CC_MASK                    (0x01 << 2)
#define DA9168_S_CHG_PRE_SHIFT                  3
#define DA9168_S_CHG_PRE_MASK                   (0x01 << 3)
#define DA9168_S_CHG_TRICKLE_SHIFT              4
#define DA9168_S_CHG_TRICKLE_MASK               (0x01 << 4)
#define DA9168_S_CHG_TIMER_SHIFT                5
#define DA9168_S_CHG_TIMER_MASK                 (0x01 << 5)
#define DA9168_S_CHG_SPLMT_SHIFT                6
#define DA9168_S_CHG_SPLMT_MASK                 (0x01 << 6)
#define DA9168_S_CHG_SLEEP_SHIFT                7
#define DA9168_S_CHG_SLEEP_MASK                 (0x01 << 7)

/* DA9168_PMC_STATUS_04 = 0x0004 */

#define DA9168_S_LDO1_OC_SHIFT                  0
#define DA9168_S_LDO1_OC_MASK                   (0x01 << 0)
#define DA9168_S_LDO1_IMON2_SHIFT               1
#define DA9168_S_LDO1_IMON2_MASK                (0x01 << 1)
#define DA9168_S_LDO1_IMON1_SHIFT               2
#define DA9168_S_LDO1_IMON1_MASK                (0x01 << 2)
#define DA9168_S_LDO2_OC_SHIFT                  3
#define DA9168_S_LDO2_OC_MASK                   (0x01 << 3)
#define DA9168_S_LDO2_IMON2_SHIFT               4
#define DA9168_S_LDO2_IMON2_MASK                (0x01 << 4)
#define DA9168_S_LDO2_IMON1_SHIFT               5
#define DA9168_S_LDO2_IMON1_MASK                (0x01 << 5)
#define DA9168_S_REF_OC_SHIFT                   6
#define DA9168_S_REF_OC_MASK                    (0x01 << 6)
#define DA9168_S_VSYS_SHUTDOWN_SHIFT            7
#define DA9168_S_VSYS_SHUTDOWN_MASK             (0x01 << 7)

/* DA9168_PMC_EVENT_00 = 0x0005 */

#define DA9168_E_VBAT_OK_SHIFT                  0
#define DA9168_E_VBAT_OK_MASK                   (0x01 << 0)
#define DA9168_E_VSYS_OK_SHIFT                  1
#define DA9168_E_VSYS_OK_MASK                   (0x01 << 1)
#define DA9168_E_VMID_OK_SHIFT                  2
#define DA9168_E_VMID_OK_MASK                   (0x01 << 2)
#define DA9168_E_VBUS_OK_SHIFT                  3
#define DA9168_E_VBUS_OK_MASK                   (0x01 << 3)
#define DA9168_E_VBAT_OC_SHIFT                  4
#define DA9168_E_VBAT_OC_MASK                   (0x01 << 4)
#define DA9168_E_VMID_OC_SHIFT                  5
#define DA9168_E_VMID_OC_MASK                   (0x01 << 5)
#define DA9168_E_VBUS_IINDPM_SHIFT              6
#define DA9168_E_VBUS_IINDPM_MASK               (0x01 << 6)
#define DA9168_E_VBUS_VINDPM_SHIFT              7
#define DA9168_E_VBUS_VINDPM_MASK               (0x01 << 7)

/* DA9168_PMC_EVENT_01 = 0x0006 */

#define DA9168_E_VBAT_UV_SHIFT                  0
#define DA9168_E_VBAT_UV_MASK                   (0x01 << 0)
#define DA9168_E_VSYS_UV_SHIFT                  1
#define DA9168_E_VSYS_UV_MASK                   (0x01 << 1)
#define DA9168_E_VMID_UV_SHIFT                  2
#define DA9168_E_VMID_UV_MASK                   (0x01 << 2)
#define DA9168_E_VBUS_UV_SHIFT                  3
#define DA9168_E_VBUS_UV_MASK                   (0x01 << 3)
#define DA9168_E_VBAT_OV_SHIFT                  4
#define DA9168_E_VBAT_OV_MASK                   (0x01 << 4)
#define DA9168_E_VSYS_OV_SHIFT                  5
#define DA9168_E_VSYS_OV_MASK                   (0x01 << 5)
#define DA9168_E_VMID_OV_SHIFT                  6
#define DA9168_E_VMID_OV_MASK                   (0x01 << 6)
#define DA9168_E_VBUS_OV_SHIFT                  7
#define DA9168_E_VBUS_OV_MASK                   (0x01 << 7)

/* DA9168_PMC_EVENT_02 = 0x0007 */

#define DA9168_E_TS_OFF_SHIFT                   0
#define DA9168_E_TS_OFF_MASK                    (0x01 << 0)
#define DA9168_E_TS_COLD_SHIFT                  1
#define DA9168_E_TS_COLD_MASK                   (0x01 << 1)
#define DA9168_E_TS_COOL_SHIFT                  2
#define DA9168_E_TS_COOL_MASK                   (0x01 << 2)
#define DA9168_E_TS_WARM_SHIFT                  3
#define DA9168_E_TS_WARM_MASK                   (0x01 << 3)
#define DA9168_E_TS_HOT_SHIFT                   4
#define DA9168_E_TS_HOT_MASK                    (0x01 << 4)
#define DA9168_E_WD_TIMER_SHIFT                 5
#define DA9168_E_WD_TIMER_MASK                  (0x01 << 5)
#define DA9168_E_TSD_WARN_SHIFT                 6
#define DA9168_E_TSD_WARN_MASK                  (0x01 << 6)
#define DA9168_E_TSD_CRIT_SHIFT                 7
#define DA9168_E_TSD_CRIT_MASK                  (0x01 << 7)

/* DA9168_PMC_EVENT_03 = 0x0008 */

#define DA9168_E_CHG_DONE_SHIFT                 0
#define DA9168_E_CHG_DONE_MASK                  (0x01 << 0)
#define DA9168_E_CHG_CV_SHIFT                   1
#define DA9168_E_CHG_CV_MASK                    (0x01 << 1)
#define DA9168_E_CHG_CC_SHIFT                   2
#define DA9168_E_CHG_CC_MASK                    (0x01 << 2)
#define DA9168_E_CHG_PRE_SHIFT                  3
#define DA9168_E_CHG_PRE_MASK                   (0x01 << 3)
#define DA9168_E_CHG_TRICKLE_SHIFT              4
#define DA9168_E_CHG_TRICKLE_MASK               (0x01 << 4)
#define DA9168_E_CHG_TIMER_SHIFT                5
#define DA9168_E_CHG_TIMER_MASK                 (0x01 << 5)
#define DA9168_E_CHG_SPLMT_SHIFT                6
#define DA9168_E_CHG_SPLMT_MASK                 (0x01 << 6)
#define DA9168_E_CHG_SLEEP_SHIFT                7
#define DA9168_E_CHG_SLEEP_MASK                 (0x01 << 7)

/* DA9168_PMC_EVENT_04 = 0x0009 */

#define DA9168_E_LDO1_OC_SHIFT                  0
#define DA9168_E_LDO1_OC_MASK                   (0x01 << 0)
#define DA9168_E_LDO1_IMON2_SHIFT               1
#define DA9168_E_LDO1_IMON2_MASK                (0x01 << 1)
#define DA9168_E_LDO1_IMON1_SHIFT               2
#define DA9168_E_LDO1_IMON1_MASK                (0x01 << 2)
#define DA9168_E_LDO2_OC_SHIFT                  3
#define DA9168_E_LDO2_OC_MASK                   (0x01 << 3)
#define DA9168_E_LDO2_IMON2_SHIFT               4
#define DA9168_E_LDO2_IMON2_MASK                (0x01 << 4)
#define DA9168_E_LDO2_IMON1_SHIFT               5
#define DA9168_E_LDO2_IMON1_MASK                (0x01 << 5)
#define DA9168_E_REF_OC_SHIFT                   6
#define DA9168_E_REF_OC_MASK                    (0x01 << 6)
#define DA9168_E_VSYS_SHUTDOWN_SHIFT            7
#define DA9168_E_VSYS_SHUTDOWN_MASK             (0x01 << 7)

/* DA9168_PMC_MASK_00 = 0x000a */

#define DA9168_M_VBAT_OK_SHIFT                  0
#define DA9168_M_VBAT_OK_MASK                   (0x01 << 0)
#define DA9168_M_VSYS_OK_SHIFT                  1
#define DA9168_M_VSYS_OK_MASK                   (0x01 << 1)
#define DA9168_M_VMID_OK_SHIFT                  2
#define DA9168_M_VMID_OK_MASK                   (0x01 << 2)
#define DA9168_M_VBUS_OK_SHIFT                  3
#define DA9168_M_VBUS_OK_MASK                   (0x01 << 3)
#define DA9168_M_VBAT_OC_SHIFT                  4
#define DA9168_M_VBAT_OC_MASK                   (0x01 << 4)
#define DA9168_M_VMID_OC_SHIFT                  5
#define DA9168_M_VMID_OC_MASK                   (0x01 << 5)
#define DA9168_M_VBUS_IINDPM_SHIFT              6
#define DA9168_M_VBUS_IINDPM_MASK               (0x01 << 6)
#define DA9168_M_VBUS_VINDPM_SHIFT              7
#define DA9168_M_VBUS_VINDPM_MASK               (0x01 << 7)

/* DA9168_PMC_MASK_01 = 0x000b */

#define DA9168_M_VBAT_UV_SHIFT                  0
#define DA9168_M_VBAT_UV_MASK                   (0x01 << 0)
#define DA9168_M_VSYS_UV_SHIFT                  1
#define DA9168_M_VSYS_UV_MASK                   (0x01 << 1)
#define DA9168_M_VMID_UV_SHIFT                  2
#define DA9168_M_VMID_UV_MASK                   (0x01 << 2)
#define DA9168_M_VBUS_UV_SHIFT                  3
#define DA9168_M_VBUS_UV_MASK                   (0x01 << 3)
#define DA9168_M_VBAT_OV_SHIFT                  4
#define DA9168_M_VBAT_OV_MASK                   (0x01 << 4)
#define DA9168_M_VSYS_OV_SHIFT                  5
#define DA9168_M_VSYS_OV_MASK                   (0x01 << 5)
#define DA9168_M_VMID_OV_SHIFT                  6
#define DA9168_M_VMID_OV_MASK                   (0x01 << 6)
#define DA9168_M_VBUS_OV_SHIFT                  7
#define DA9168_M_VBUS_OV_MASK                   (0x01 << 7)

/* DA9168_PMC_MASK_02 = 0x000c */

#define DA9168_M_TS_OFF_SHIFT                   0
#define DA9168_M_TS_OFF_MASK                    (0x01 << 0)
#define DA9168_M_TS_COLD_SHIFT                  1
#define DA9168_M_TS_COLD_MASK                   (0x01 << 1)
#define DA9168_M_TS_COOL_SHIFT                  2
#define DA9168_M_TS_COOL_MASK                   (0x01 << 2)
#define DA9168_M_TS_WARM_SHIFT                  3
#define DA9168_M_TS_WARM_MASK                   (0x01 << 3)
#define DA9168_M_TS_HOT_SHIFT                   4
#define DA9168_M_TS_HOT_MASK                    (0x01 << 4)
#define DA9168_M_WD_TIMER_SHIFT                 5
#define DA9168_M_WD_TIMER_MASK                  (0x01 << 5)
#define DA9168_M_TSD_WARN_SHIFT                 6
#define DA9168_M_TSD_WARN_MASK                  (0x01 << 6)
#define DA9168_M_TSD_CRIT_SHIFT                 7
#define DA9168_M_TSD_CRIT_MASK                  (0x01 << 7)

/* DA9168_PMC_MASK_03 = 0x000d */

#define DA9168_M_CHG_DONE_SHIFT                 0
#define DA9168_M_CHG_DONE_MASK                  (0x01 << 0)
#define DA9168_M_CHG_CV_SHIFT                   1
#define DA9168_M_CHG_CV_MASK                    (0x01 << 1)
#define DA9168_M_CHG_CC_SHIFT                   2
#define DA9168_M_CHG_CC_MASK                    (0x01 << 2)
#define DA9168_M_CHG_PRE_SHIFT                  3
#define DA9168_M_CHG_PRE_MASK                   (0x01 << 3)
#define DA9168_M_CHG_TRICKLE_SHIFT              4
#define DA9168_M_CHG_TRICKLE_MASK               (0x01 << 4)
#define DA9168_M_CHG_TIMER_SHIFT                5
#define DA9168_M_CHG_TIMER_MASK                 (0x01 << 5)
#define DA9168_M_CHG_SPLMT_SHIFT                6
#define DA9168_M_CHG_SPLMT_MASK                 (0x01 << 6)
#define DA9168_M_CHG_SLEEP_SHIFT                7
#define DA9168_M_CHG_SLEEP_MASK                 (0x01 << 7)

/* DA9168_PMC_MASK_04 = 0x000e */

#define DA9168_M_LDO1_OC_SHIFT                  0
#define DA9168_M_LDO1_OC_MASK                   (0x01 << 0)
#define DA9168_M_LDO1_IMON2_SHIFT               1
#define DA9168_M_LDO1_IMON2_MASK                (0x01 << 1)
#define DA9168_M_LDO1_IMON1_SHIFT               2
#define DA9168_M_LDO1_IMON1_MASK                (0x01 << 2)
#define DA9168_M_LDO2_OC_SHIFT                  3
#define DA9168_M_LDO2_OC_MASK                   (0x01 << 3)
#define DA9168_M_LDO2_IMON2_SHIFT               4
#define DA9168_M_LDO2_IMON2_MASK                (0x01 << 4)
#define DA9168_M_LDO2_IMON1_SHIFT               5
#define DA9168_M_LDO2_IMON1_MASK                (0x01 << 5)
#define DA9168_M_REF_OC_SHIFT                   6
#define DA9168_M_REF_OC_MASK                    (0x01 << 6)
#define DA9168_M_VSYS_SHUTDOWN_SHIFT            7
#define DA9168_M_VSYS_SHUTDOWN_MASK             (0x01 << 7)

/* DA9168_PMC_SYS_00 = 0x000f */

#define DA9168_VINDPM_SHIFT                     0
#define DA9168_VINDPM_MASK                      (0x0f << 0)
#define DA9168_VINDPM_OFFSET                    4
#define DA9168_VSYS_MIN_SHIFT                   4
#define DA9168_VSYS_MIN_MASK                    (0x07 << 4)
#define DA9168_E_RD_CLR_DIS_SHIFT               7
#define DA9168_E_RD_CLR_DIS_MASK                (0x01 << 7)

/* DA9168_PMC_SYS_01 = 0x0010 */

#define DA9168_IINDPM_SHIFT                     0
#define DA9168_IINDPM_MASK                      (0x1f << 0)
#define DA9168_ILIMIT_EN_SHIFT                  5
#define DA9168_ILIMIT_EN_MASK                   (0x01 << 5)
#define DA9168_VSYS_UV_SHUTDOWN_DIS_SHIFT       6
#define DA9168_VSYS_UV_SHUTDOWN_DIS_MASK        (0x01 << 6)
#define DA9168_VSYS_OV_SHUTDOWN_DIS_SHIFT       7
#define DA9168_VSYS_OV_SHUTDOWN_DIS_MASK        (0x01 << 7)

/* DA9168_PMC_SYS_02 = 0x0011 */

#define DA9168_VBUS_DEB_SHIFT                   0
#define DA9168_VBUS_DEB_MASK                    (0x03 << 0)
#define DA9168_VBAT_DEB_SHIFT                   2
#define DA9168_VBAT_DEB_MASK                    (0x03 << 2)
#define DA9168_BTS_VBUS_EN_SHIFT                4
#define DA9168_BTS_VBUS_EN_MASK                 (0x01 << 4)
#define DA9168_BTS_VBUS_RATE_SHIFT              5
#define DA9168_BTS_VBUS_RATE_MASK               (0x01 << 5)
#define DA9168_BTS_VBAT_EN_SHIFT                6
#define DA9168_BTS_VBAT_EN_MASK                 (0x01 << 6)
#define DA9168_BTS_VBAT_RATE_SHIFT              7
#define DA9168_BTS_VBAT_RATE_MASK               (0x01 << 7)

/* DA9168_PMC_SYS_03 = 0x0012 */

#define DA9168_RST_REG_SHIFT                    0
#define DA9168_RST_REG_MASK                     (0x01 << 0)
#define DA9168_RST_TMR_SHIFT                    1
#define DA9168_RST_TMR_MASK                     (0x03 << 1)
#define DA9168_WD_EN_SHIFT                      3
#define DA9168_WD_EN_MASK                       (0x01 << 3)
#define DA9168_WD_TMR_SHIFT                     4
#define DA9168_WD_TMR_MASK                      (0x03 << 4)
#define DA9168_SYS_WAIT_SHIFT                   6
#define DA9168_SYS_WAIT_MASK                    (0x03 << 6)

/* DA9168_PMC_SYS_04 = 0x0013 */

#define DA9168_BOOST_EN_SHIFT                   0
#define DA9168_BOOST_EN_MASK                    (0x01 << 0)
#define DA9168_REV_VBUS_EN_SHIFT                1
#define DA9168_REV_VBUS_EN_MASK                 (0x01 << 1)
#define DA9168_DLOAD_VMID_EN_SHIFT              2
#define DA9168_DLOAD_VMID_EN_MASK               (0x01 << 2)
#define DA9168_DLOAD_VMID_SEL_SHIFT             3
#define DA9168_DLOAD_VMID_SEL_MASK              (0x03 << 3)
#define DA9168_SEQ_BOOST_SHIFT                  5
#define DA9168_SEQ_BOOST_MASK                   (0x01 << 5)
#define DA9168_BOOST_PWM_SHIFT                  6
#define DA9168_BOOST_PWM_MASK                   (0x01 << 6)

/* DA9168_PMC_SYS_05 = 0x0014 */

#define DA9168_BOOST_VOUT_SHIFT                 0
#define DA9168_BOOST_VOUT_MASK                  (0x0f << 0)
#define DA9168_REV_VBUS_ILIM_SHIFT              4
#define DA9168_REV_VBUS_ILIM_MASK               (0x0f << 4)

/* DA9168_PMC_SYS_06 = 0x0015 */

#define DA9168_SHIP_MODE_SHIFT                  0
#define DA9168_SHIP_MODE_MASK                   (0x01 << 0)
#define DA9168_SHIP_DLY_SHIFT                   1
#define DA9168_SHIP_DLY_MASK                    (0x03 << 1)
#define DA9168_HIZ_MODE_SHIFT                   3
#define DA9168_HIZ_MODE_MASK                    (0x01 << 3)
#define DA9168_VBUS_OVSEL_SHIFT                 4
#define DA9168_VBUS_OVSEL_MASK                  (0x03 << 4)
#define DA9168_RIN_N_SHIP_EXIT_TMR_SHIFT        6
#define DA9168_RIN_N_SHIP_EXIT_TMR_MASK         (0x01 << 6)
#define DA9168_RST_SYS_SHIFT                    7
#define DA9168_RST_SYS_MASK                     (0x01 << 7)

/* DA9168_PMC_CHG_00 = 0x0016 */

#define DA9168_CHG_EN_SHIFT                     0
#define DA9168_CHG_EN_MASK                      (0x01 << 0)
#define DA9168_CHG_TERM_EN_SHIFT                1
#define DA9168_CHG_TERM_EN_MASK                 (0x01 << 1)
#define DA9168_CHG_TMR_EN_SHIFT                 2
#define DA9168_CHG_TMR_EN_MASK                  (0x01 << 2)
#define DA9168_CHG_TMR_HALF_EN_SHIFT            3
#define DA9168_CHG_TMR_HALF_EN_MASK             (0x01 << 3)
#define DA9168_CHG_VRCHG_SHIFT                  4
#define DA9168_CHG_VRCHG_MASK                   (0x01 << 4)
#define DA9168_BUCK_PWM_SHIFT                   5
#define DA9168_BUCK_PWM_MASK                    (0x01 << 5)

/* DA9168_PMC_CHG_01 = 0x0017 */

#define DA9168_CHG_TOPOFF_SHIFT                 0
#define DA9168_CHG_TOPOFF_MASK                  (0x0f << 0)
#define DA9168_CHG_TOPOFF_OFF                   (0x0 << 0)
#define DA9168_CHG_TOPOFF_15_SEC                (0x1 << 0)
#define DA9168_CHG_TOPOFF_30_SEC                (0x2 << 0)
#define DA9168_CHG_TOPOFF_60_SEC                (0x3 << 0)
#define DA9168_CHG_TOPOFF_OFFSET                4
#define DA9168_CHG_TMR_PRE_SHIFT                4
#define DA9168_CHG_TMR_PRE_MASK                 (0x03 << 4)
#define DA9168_CHG_TMR_SAFE_SHIFT               6
#define DA9168_CHG_TMR_SAFE_MASK                (0x03 << 6)

/* DA9168_PMC_CHG_02 = 0x0018 */

#define DA9168_CHG_IPRE_SHIFT                   0
#define DA9168_CHG_IPRE_MASK                    (0x03 << 0)
#define DA9168_CHG_IPRE_OFFSET                  1
#define DA9168_CHG_ITERM_SHIFT                  2
#define DA9168_CHG_ITERM_MASK                   (0x07 << 2)
#define DA9168_CHG_ITERM_OFFSET                 1
#define DA9168_CHG_RANGE_PRE_SHIFT              5
#define DA9168_CHG_RANGE_PRE_MASK               (0x01 << 5)
#define DA9168_CHG_RANGE_TERM_SHIFT             6
#define DA9168_CHG_RANGE_TERM_MASK              (0x01 << 6)
#define DA9168_CHG_IPRE_MSB_SHIFT               7
#define DA9168_CHG_IPRE_MSB_MASK                (0x01 << 7)

/* DA9168_PMC_CHG_03 = 0x0019 */

#define DA9168_CHG_ICHG_SHIFT                   0
#define DA9168_CHG_ICHG_MASK                    (0x7f << 0)
#define DA9168_CHG_ICHG_OFFSET                  1
#define DA9168_CHG_RANGE_SHIFT                  7
#define DA9168_CHG_RANGE_MASK                   (0x01 << 7)

/* DA9168_PMC_CHG_04 = 0x001a */

#define DA9168_CHG_VBATREG_SHIFT                0
#define DA9168_CHG_VBATREG_MASK                 (0x3f << 0)
#define DA9168_CHG_VBATREG_OFFSET               3

/* DA9168_PMC_CHG_05 = 0x001b */

#define DA9168_CHG_TS_COOL_I_SHIFT              0
#define DA9168_CHG_TS_COOL_I_MASK               (0x01 << 0)
#define DA9168_CHG_TS_WARM_V_SHIFT              1
#define DA9168_CHG_TS_WARM_V_MASK               (0x01 << 1)
#define DA9168_TS_VBATREG_SHIFT_SHIFT           2
#define DA9168_TS_VBATREG_SHIFT_MASK            (0x01 << 2)
#define DA9168_IO_CHG_EN_N_PD_SHIFT             5
#define DA9168_IO_CHG_EN_N_PD_MASK              (0x01 << 5)
#define DA9168_IO_EN_PD_SHIFT                   6
#define DA9168_IO_EN_PD_MASK                    (0x01 << 6)
#define DA9168_IO_INT_N_PU_SHIFT                7
#define DA9168_IO_INT_N_PU_MASK                 (0x01 << 7)

/* DA9168_PMC_CHG_06 = 0x001c */

#define DA9168_TS_ICHG_SHIFT                    0
#define DA9168_TS_ICHG_MASK                     (0x7f << 0)
#define DA9168_TS_ICHG_OFFSET                   1

/* DA9168_PMC_LDO_00 = 0x001d */

#define DA9168_LDO1_EN_SHIFT                    0
#define DA9168_LDO1_EN_MASK                     (0x01 << 0)
#define DA9168_LDO1_LSW_SHIFT                   1
#define DA9168_LDO1_LSW_MASK                    (0x01 << 1)
#define DA9168_LDO1_PD_SHIFT                    2
#define DA9168_LDO1_PD_MASK		        (0x01 << 2)
#define DA9168_LDO1_OCP_SHIFT                   3
#define DA9168_LDO1_OCP_MASK                    (0x01 << 3)
#define DA9168_LDO2_EN_SHIFT                    4
#define DA9168_LDO2_EN_MASK                     (0x01 << 4)
#define DA9168_LDO2_LSW_SHIFT                   5
#define DA9168_LDO2_LSW_MASK                    (0x01 << 5)
#define DA9168_LDO2_PD_SHIFT                    6
#define DA9168_LDO2_PD_MASK                     (0x01 << 6)
#define DA9168_LDO2_OCP_SHIFT                   7
#define DA9168_LDO2_OCP_MASK                    (0x01 << 7)

/* DA9168_PMC_LDO_01 = 0x001e */

#define DA9168_LDO1_VOUT_SHIFT                  0
#define DA9168_LDO1_VOUT_MASK                   (0x3f << 0)
#define DA9168_LDO_VOUT_SHIFT                   DA9168_LDO1_VOUT_SHIFT
#define DA9168_LDO_VOUT_MASK                    DA9168_LDO1_VOUT_MASK
#define DA9168_SEQ_LDO1_SHIFT                   6
#define DA9168_SEQ_LDO1_MASK                    (0x03 << 6)

/* DA9168_PMC_LDO_02 = 0x001f */

#define DA9168_LDO2_VOUT_SHIFT                  0
#define DA9168_LDO2_VOUT_MASK                   (0x3f << 0)
#define DA9168_SEQ_LDO2_SHIFT                   6
#define DA9168_SEQ_LDO2_MASK                    (0x03 << 6)

/* DA9168_PMC_LDO_05 = 0x0022 */

#define DA9168_LDO1_ILIM_SHIFT                  0
#define DA9168_LDO1_ILIM_MASK                   (0x0f << 0)
#define DA9168_LDO_ILIM_SHIFT                   DA9168_LDO1_ILIM_SHIFT
#define DA9168_LDO_ILIM_MASK                    DA9168_LDO1_ILIM_MASK
#define DA9168_LDO2_ILIM_SHIFT                  4
#define DA9168_LDO2_ILIM_MASK                   (0x0f << 4)

/* DA9168_OTP_DEVICE_ID = 0x0042 */

#define DA9168_DEV_ID_SHIFT                     0
#define DA9168_DEV_ID_MASK                      (0xff << 0)
#define DA9168_OTP_DEVICE_ID_OFFSET             0

/* DA9168_OTP_VARIANT_ID = 0x0043 */

#define DA9168_VRC_SHIFT                        0
#define DA9168_VRC_MASK	                        (0x0f << 0)
#define DA9168_MRC_SHIFT                        4
#define DA9168_MRC_MASK                         (0x0f << 4)
#define DA9168_OTP_VARIANT_ID_OFFSET            1

/* DA9168_OTP_CONFIG_ID = 0x0044 */

#define DA9168_CONFIG_REV_SHIFT                 0
#define DA9168_CONFIG_REV_MASK                  (0xff << 0)
#define DA9168_OTP_CONFIG_ID_OFFSET             2

/* Input current limit */

#define DA9168_IBUS_LIM_STEP_MA                         100
#define DA9168_IBUS_LIM_MIN_MA                          100
#define DA9168_IBUS_LIM_MAX_MA                          2500

/* charge current cfg value */

#define DA9168_PCFG_CHG_RANGE_STEPS_5MA                 0
#define	DA9168_PCFG_CHG_RANGE_STEPS_20MA                1
#define DA9168_PCFG_CHG_CURR_LOW_STEP_MA                5
#define DA9168_PCFG_CHG_CURR_HIGH_STEP_MA               20
#define DA9168_PCFG_CHG_CURR_PRE_LOW_MIN_MA             5
#define DA9168_PCFG_CHG_CURR_PRE_LOW_MAX_MA             15
#define DA9168_PCFG_CHG_CURR_PRE_LOW_OFFSET_MIN_MA      20
#define DA9168_PCFG_CHG_CURR_PRE_LOW_OFFSET_MAX_MA      35
#define DA9168_PCFG_CHG_CURR_TERM_LOW_MIN_MA            5
#define DA9168_PCFG_CHG_CURR_TERM_LOW_MAX_MA            25
#define DA9168_PCFG_CHG_CURR_PRE_HIGH_MIN_MA            20
#define DA9168_PCFG_CHG_CURR_PRE_HIGH_MAX_MA            60
#define DA9168_PCFG_CHG_CURR_PRE_HIGH_OFFSET_MIN_MA     80
#define DA9168_PCFG_CHG_CURR_PRE_HIGH_OFFSET_MAX_MA     140
#define DA9168_PCFG_CHG_CURR_TERM_HIGH_MIN_MA           20
#define DA9168_PCFG_CHG_CURR_TERM_HIGH_MAX_MA           100
#define DA9168_PCFG_CHG_CURR_CC_LOW_MIN_MA              5
#define DA9168_PCFG_CHG_CURR_CC_LOW_MAX_MA              500
#define DA9168_PCFG_CHG_CURR_CC_HIGH_MIN_MA             20
#define DA9168_PCFG_CHG_CURR_CC_HIGH_MAX_MA             2000
#define DA9168_ICHG_DEFAULT_UA                          140
#define DA9168_PRE_CURR_DEFAULT_UA                      20
#define DA9168_ITERM_DEFAULT_UA                         60

/* charge voltage cfg value */

#define DA9168_PCFG_CHG_VOLT_CV_STEP_MV                 10
#define DA9168_PCFG_CHG_VOLT_CV_MIN_MV                  4000
#define DA9168_PCFG_CHG_VOLT_CV_MAX_MV                  4500
#define DA9168_VBAT_DEFAULT_UV                          4200

/* Vindpm cfg value */

#define DA9168_PCFG_VBUS_STEP_MV                        100
#define DA9168_PCFG_VBUS_DPM_MIN_MV                     3800
#define DA9168_PCFG_VBUS_DPM_MAX_MV                     4800
#define DA9168_VINDPM_DEFAULT_UV                        4300

/* Vindpm cfg value */

#define DA9168_VUBS_DEFAULT_OV                          5800

/* Device id */

#define DA9168_DEV_ID                                   0xE7

#define DA_IIC_RETRY_NUM                                3

/* Ship mode entry delay time */

enum da9168_ship_mode_entry_delay_sel
{
  DA9168_SHIP_MODE_ENTRY_DELAY_2S,          /* 2 second delay */
  DA9168_SHIP_MODE_ENTRY_DELAY_4S,          /* 4 second delay */
  DA9168_SHIP_MODE_ENTRY_DELAY_7S,          /* 7 second delay */
  DA9168_SHIP_MODE_ENTRY_DELAY_10S,         /* 10 second delay */
};

/* Ship mode exit delay time */

enum da9168_ship_mode_exit_deb_sel
{
  DA9168_SHIP_MODE_EXIT_DEB_20MS,           /**< 20 millisecond delay */
  DA9168_SHIP_MODE_EXIT_DEB_2S,             /**< 2 second delay */
};

/* Charging phases during DA9168_CHG_STATE_SINK_VBUS state */

enum da9168_chg_phases_e
{
  DA9168_CHG_PHASE_UNKNOWN = -1,
  DA9168_CHG_PHASE_OFF = 0,                 /**< Charging disabled */
  DA9168_CHG_PHASE_TRICKLE,                 /**< Trickle-charge */
  DA9168_CHG_PHASE_PRE,                     /**< Pre-charge */
  DA9168_CHG_PHASE_CC,                      /**< Constant Current */
  DA9168_CHG_PHASE_CV,                      /**< Constant Voltage */
  DA9168_CHG_PHASE_FULL,                    /**< Battery full */
  DA9168_CHG_PHASE_MAX,
};

/* Charger operational faults */

enum da9168_chg_faults_e
{
  DA9168_CHG_FAULT_UNKNOWN = -1,
  DA9168_CHG_FAULT_NONE = 0,                 /**< No faults present */
  DA9168_CHG_FAULT_VBUS_OV,                  /**< VBUS over-voltage */
  DA9168_CHG_FAULT_VBUS_UV,                  /**< VBUS under-voltage */
  DA9168_CHG_FAULT_VBUS_DPM,                 /**< VBUS DPM in operation */
  DA9168_CHG_FAULT_IBUS_DPM,                 /**< IBUS DPM in operation */
  DA9168_CHG_FAULT_VBAT_OC,                  /**< VBAT over-current */
  DA9168_CHG_FAULT_VBAT_OV,                  /**< VBAT over-voltage */
  DA9168_CHG_FAULT_VBAT_UV,                  /**< VBAT under-voltage */
  DA9168_CHG_FAULT_TBAT_HOT,                 /**< TBAT over HOT threshold */
  DA9168_CHG_FAULT_TBAT_COLD,                /**< TBAT under COLD threshold */
  DA9168_CHG_FAULT_TBAT_WARM,                /**< TBAT over WARM threshold */
  DA9168_CHG_FAULT_TBAT_COOL,                /**< TBAT under COOL threshold */
  DA9168_CHG_FAULT_VSYS_SHUTDOWN,            /**< VSYS shutdown */
  DA9168_CHG_FAULT_VSYS_OV,                  /**< VSYS over-voltage */
  DA9168_CHG_FAULT_VSYS_UV,                  /**< VSYS under-voltage */
  DA9168_CHG_FAULT_CHG_TIMER,                /**< Charging safety timer expired */
  DA9168_CHG_FAULT_WD_TIMER,                 /* Watchdog timer expired */
  DA9168_CHG_FAULT_TJUNC_CRIT,               /* TJUNC over CRIT threshold */
  DA9168_CHG_FAULT_TJUNC_WARN,               /* TJUNC over WARN threshold */
  DA9168_CHG_FAULT_MAX,
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

FAR struct da9168_ic_state_s
{
  uint8_t chg_stat;
  uint8_t vsys_fault;
  uint8_t vbus_fault;
  uint8_t timer_fault;
  uint8_t bat_fault;
};

#define DA9168_VINDPM_INT_FLAG    (1<<0)
#define DA9168_VBUS_UV_INT_FLAG   (1<<9)

#endif/* __DA9168_H */
