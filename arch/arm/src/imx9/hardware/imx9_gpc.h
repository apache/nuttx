/****************************************************************************
 * arch/arm/src/imx9/hardware/imx9_gpc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#ifndef __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_GPC_H
#define __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_GPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets */

#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_OFFSET          0x0004 /* CMC Authentication Control */
#define IMX9_GPC_CTRL_CMC_MISC_OFFSET                 0x000c /* Miscellaneous */
#define IMX9_GPC_CTRL_CMC_MODE_CTRL_OFFSET            0x0010 /* CPU mode control */
#define IMX9_GPC_CTRL_CMC_MODE_STAT_OFFSET            0x0014 /* CPU mode Status */
#define IMX9_GPC_CTRL_CMC_PIN_STAT_OFFSET             0x0018 /* CMC pin Status */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_1_OFFSET    0x0100 /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_2_OFFSET    0x0104 /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_3_OFFSET    0x0108 /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_4_OFFSET    0x010c /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_5_OFFSET    0x0110 /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_6_OFFSET    0x0114 /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_7_OFFSET    0x0118 /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_8_OFFSET    0x011c /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_9_OFFSET    0x0120 /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_10_OFFSET   0x0124 /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_11_OFFSET   0x0128 /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_12_OFFSET   0x012c /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_MASK_OFFSET  0x0140 /* CMC non-IRQ wakeup mask */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_1_OFFSET    0x0150 /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_2_OFFSET    0x0154 /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_3_OFFSET    0x0158 /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_4_OFFSET    0x015c /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_5_OFFSET    0x0160 /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_6_OFFSET    0x0164 /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_7_OFFSET    0x0168 /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_8_OFFSET    0x016c /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_9_OFFSET    0x0170 /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_10_OFFSET   0x0174 /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_11_OFFSET   0x0178 /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_12_OFFSET   0x017c /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_STAT_OFFSET  0x0190 /* CMC non-IRQ wakeup status */
#define IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_CTRL_OFFSET  0x0200 /* CMC sleep A55_HDSK control */
#define IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_STAT_OFFSET  0x0204 /* CMC sleep A55_HDSK status */
#define IMX9_GPC_CTRL_CMC_SLEEP_SSAR_CTRL_OFFSET      0x0208 /* CMC sleep SSAR control */
#define IMX9_GPC_CTRL_CMC_SLEEP_SSAR_STAT_OFFSET      0x020c /* CMC sleep SSAR status */
#define IMX9_GPC_CTRL_CMC_SLEEP_RESET_CTRL_OFFSET     0x0230 /* CMC sleep reset control */
#define IMX9_GPC_CTRL_CMC_SLEEP_RESET_STAT_OFFSET     0x0234 /* CMC sleep reset status */
#define IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_CTRL_OFFSET    0x0248 /* CMC sleep Sysman control */
#define IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_STAT_OFFSET    0x024c /* CMC Sleep Sysman status */
#define IMX9_GPC_CTRL_CMC_WAKEUP_POWER_CTRL_OFFSET    0x0290 /* CMC wakeup power control */
#define IMX9_GPC_CTRL_CMC_WAKEUP_POWER_STAT_OFFSET    0x0294 /* CMC wakeup power status */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_CTRL_OFFSET     0x02c8 /* CMC wakeup SSAR control */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_STAT_OFFSET     0x02cc /* CMC wakeup SSAR status */
#define IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_CTRL_OFFSET 0x02d0 /* CMC wakeup A55_HDSK control */
#define IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_STAT_OFFSET 0x02d4 /* CMC wakeup A55_HDSK status */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_CTRL_OFFSET   0x02d8 /* CMC wakeup Sysman control */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_STAT_OFFSET   0x02dc /* CMC wakeup Sysman status */
#define IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_OFFSET       0x0380 /* CMC system sleep control */
#define IMX9_GPC_CTRL_CMC_DEBUG_OFFSET                0x0390 /* CMC debug */

/* Register macros */

#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL(n)          ((n) + IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_OFFSET)          /* CMC Authentication Control */
#define IMX9_GPC_CTRL_CMC_MISC(n)                 ((n) + IMX9_GPC_CTRL_CMC_MISC_OFFSET)                 /* Miscellaneous */
#define IMX9_GPC_CTRL_CMC_MODE_CTRL(n)            ((n) + IMX9_GPC_CTRL_CMC_MODE_CTRL_OFFSET)            /* CPU mode control */
#define IMX9_GPC_CTRL_CMC_MODE_STAT(n)            ((n) + IMX9_GPC_CTRL_CMC_MODE_STAT_OFFSET)            /* CPU mode Status */
#define IMX9_GPC_CTRL_CMC_PIN_STAT(n)             ((n) + IMX9_GPC_CTRL_CMC_PIN_STAT_OFFSET)             /* CMC pin Status */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_1(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_1_OFFSET)    /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_2(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_2_OFFSET)    /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_3(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_3_OFFSET)    /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_4(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_4_OFFSET)    /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_5(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_5_OFFSET)    /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_6(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_6_OFFSET)    /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_7(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_7_OFFSET)    /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_8(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_8_OFFSET)    /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_9(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_9_OFFSET)    /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_10(n)   ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_10_OFFSET)   /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_11(n)   ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_11_OFFSET)   /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_12(n)   ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK_12_OFFSET)   /* IRQ wake-up mask register */
#define IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_MASK(n)  ((n) + IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_MASK_OFFSET)  /* CMC non-IRQ wakeup mask */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_1(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_1_OFFSET)    /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_2(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_2_OFFSET)    /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_3(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_3_OFFSET)    /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_4(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_4_OFFSET)    /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_5(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_5_OFFSET)    /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_6(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_6_OFFSET)    /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_7(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_7_OFFSET)    /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_8(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_8_OFFSET)    /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_9(n)    ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_9_OFFSET)    /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_10(n)   ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_10_OFFSET)   /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_11(n)   ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_11_OFFSET)   /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_12(n)   ((n) + IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT_12_OFFSET)   /* IRQ status register */
#define IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_STAT(n)  ((n) + IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_STAT_OFFSET)  /* CMC non-IRQ wakeup status */
#define IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_CTRL(n)  ((n) + IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_CTRL_OFFSET)  /* CMC sleep A55_HDSK control */
#define IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_STAT(n)  ((n) + IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_STAT_OFFSET)  /* CMC sleep A55_HDSK status */
#define IMX9_GPC_CTRL_CMC_SLEEP_SSAR_CTRL(n)      ((n) + IMX9_GPC_CTRL_CMC_SLEEP_SSAR_CTRL_OFFSET)      /* CMC sleep SSAR control */
#define IMX9_GPC_CTRL_CMC_SLEEP_SSAR_STAT(n)      ((n) + IMX9_GPC_CTRL_CMC_SLEEP_SSAR_STAT_OFFSET)      /* CMC sleep SSAR status */
#define IMX9_GPC_CTRL_CMC_SLEEP_RESET_CTRL(n)     ((n) + IMX9_GPC_CTRL_CMC_SLEEP_RESET_CTRL_OFFSET)     /* CMC sleep reset control */
#define IMX9_GPC_CTRL_CMC_SLEEP_RESET_STAT(n)     ((n) + IMX9_GPC_CTRL_CMC_SLEEP_RESET_STAT_OFFSET)     /* CMC sleep reset status */
#define IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_CTRL(n)    ((n) + IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_CTRL_OFFSET)    /* CMC sleep Sysman control */
#define IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_STAT(n)    ((n) + IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_STAT_OFFSET)    /* CMC Sleep Sysman status */
#define IMX9_GPC_CTRL_CMC_WAKEUP_POWER_CTRL(n)    ((n) + IMX9_GPC_CTRL_CMC_WAKEUP_POWER_CTRL_OFFSET)    /* CMC wakeup power control */
#define IMX9_GPC_CTRL_CMC_WAKEUP_POWER_STAT(n)    ((n) + IMX9_GPC_CTRL_CMC_WAKEUP_POWER_STAT_OFFSET)    /* CMC wakeup power status */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_CTRL(n)     ((n) + IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_CTRL_OFFSET)     /* CMC wakeup SSAR control */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_STAT(n)     ((n) + IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_STAT_OFFSET)     /* CMC wakeup SSAR status */
#define IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_CTRL(n) ((n) + IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_CTRL_OFFSET) /* CMC wakeup A55_HDSK control */
#define IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_STAT(n) ((n) + IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_STAT_OFFSET) /* CMC wakeup A55_HDSK status */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_CTRL(n)   ((n) + IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_CTRL_OFFSET)   /* CMC wakeup Sysman control */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_STAT(n)   ((n) + IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_STAT_OFFSET)   /* CMC wakeup Sysman status */
#define IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL(n)       ((n) + IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_OFFSET)       /* CMC system sleep control */
#define IMX9_GPC_CTRL_CMC_DEBUG(n)                ((n) + IMX9_GPC_CTRL_CMC_DEBUG_OFFSET)                /* CMC debug */

/* Field definitions */

/* CMC_AUTHEN_CTRL register */

#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_LOCK_CFG_SHIFT 7                                                   /* Configuration lock */
#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_LOCK_CFG_FLAG  (1 << IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_LOCK_CFG_SHIFT) /* Configuration lock */

#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_USER_SHIFT 8                                               /* Allow user mode access */
#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_USER_FLAG  (1 << IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_USER_SHIFT) /* Allow user mode access */

#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_NONSECURE_SHIFT 9                                                    /* Allow non-secure mode access */
#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_NONSECURE_FLAG  (1 << IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_NONSECURE_SHIFT) /* Allow non-secure mode access */

#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_LOCK_SETTING_SHIFT 11                                                      /* Lock NONSECURE and USER */
#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_LOCK_SETTING_FLAG  (1 << IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_LOCK_SETTING_SHIFT) /* Lock NONSECURE and USER */

#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_LOCK_LIST_SHIFT 15                                                   /* White list lock */
#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_LOCK_LIST_FLAG  (1 << IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_LOCK_LIST_SHIFT) /* White list lock */

#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_WHITE_LIST_SHIFT 16     /* Domain ID white list */
#define IMX9_GPC_CTRL_CMC_AUTHEN_CTRL_WHITE_LIST_MASK  0xffff /* Domain ID white list */

/* CMC_MISC register */

#define IMX9_GPC_CTRL_CMC_MISC_NMI_STAT_SHIFT 0                                            /* Non-masked interrupt status */
#define IMX9_GPC_CTRL_CMC_MISC_NMI_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_MISC_NMI_STAT_SHIFT) /* Non-masked interrupt status */

#define IMX9_GPC_CTRL_CMC_MISC_SLEEP_HOLD_EN_SHIFT 1                                                 /* Allow cpu_sleep_hold_req assert during CPU low power status */
#define IMX9_GPC_CTRL_CMC_MISC_SLEEP_HOLD_EN_FLAG  (1 << IMX9_GPC_CTRL_CMC_MISC_SLEEP_HOLD_EN_SHIFT) /* Allow cpu_sleep_hold_req assert during CPU low power status */

#define IMX9_GPC_CTRL_CMC_MISC_SLEEP_HOLD_STAT_SHIFT 2                                                   /* Status of cpu_sleep_hold_ack_b */
#define IMX9_GPC_CTRL_CMC_MISC_SLEEP_HOLD_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_MISC_SLEEP_HOLD_STAT_SHIFT) /* Status of cpu_sleep_hold_ack_b */

#define IMX9_GPC_CTRL_CMC_MISC_GIC_WAKEUP_STAT_SHIFT 4                                                   /* GIC wakeup request status */
#define IMX9_GPC_CTRL_CMC_MISC_GIC_WAKEUP_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_MISC_GIC_WAKEUP_STAT_SHIFT) /* GIC wakeup request status */

#define IMX9_GPC_CTRL_CMC_MISC_IRQ_MUX_SHIFT 5                                           /* IRQ select */
#define IMX9_GPC_CTRL_CMC_MISC_IRQ_MUX_FLAG  (1 << IMX9_GPC_CTRL_CMC_MISC_IRQ_MUX_SHIFT) /* IRQ select */

#define IMX9_GPC_CTRL_CMC_MISC_SW_WAKEUP_SHIFT 6                                             /* Software wakeup. Used for CPU hotplug. */
#define IMX9_GPC_CTRL_CMC_MISC_SW_WAKEUP_FLAG  (1 << IMX9_GPC_CTRL_CMC_MISC_SW_WAKEUP_SHIFT) /* Software wakeup. Used for CPU hotplug. */

/* CMC_MODE_CTRL register */

#define IMX9_GPC_CTRL_CMC_MODE_CTRL_CPU_MODE_TARGET_SHIFT 0   /* The CPU mode the CPU platform should transit to on next sleep event */
#define IMX9_GPC_CTRL_CMC_MODE_CTRL_CPU_MODE_TARGET_MASK  0x3 /* The CPU mode the CPU platform should transit to on next sleep event */

#define IMX9_GPC_CTRL_CMC_MODE_CTRL_CPU_MODE_TARGET_STAY_IN_RUN_MODE    0
#define IMX9_GPC_CTRL_CMC_MODE_CTRL_CPU_MODE_TARGET_TRANSIT_TO_WAIT     1
#define IMX9_GPC_CTRL_CMC_MODE_CTRL_CPU_MODE_TARGET_TRANSIT_TO_STOP     2
#define IMX9_GPC_CTRL_CMC_MODE_CTRL_CPU_MODE_TARGET_TRANSIT_TO_SUSPEND  3

#define IMX9_GPC_CTRL_CMC_MODE_CTRL_CPU_MODE_TARGET(n) (n << IMX9_GPC_CTRL_CMC_MODE_CTRL_CPU_MODE_TARGET_SHIFT)

#define IMX9_GPC_CTRL_CMC_MODE_CTRL_WFE_EN_SHIFT 4                                               /* WFE assertion can be sleep event */
#define IMX9_GPC_CTRL_CMC_MODE_CTRL_WFE_EN_FLAG  (1 << IMX9_GPC_CTRL_CMC_MODE_CTRL_WFE_EN_SHIFT) /* WFE assertion can be sleep event */

/* CMC_MODE_STAT register */

#define IMX9_GPC_CTRL_CMC_MODE_STAT_CPU_MODE_CURRENT_SHIFT 0   /* Current CPU mode */
#define IMX9_GPC_CTRL_CMC_MODE_STAT_CPU_MODE_CURRENT_MASK  0x3 /* Current CPU mode */

#define IMX9_GPC_CTRL_CMC_MODE_STAT_CPU_MODE_PREVIOUS_SHIFT 2   /* Previous CPU mode */
#define IMX9_GPC_CTRL_CMC_MODE_STAT_CPU_MODE_PREVIOUS_MASK  0x3 /* Previous CPU mode */

#define IMX9_GPC_CTRL_CMC_MODE_STAT_SLEEP_TRANS_BUSY_SHIFT 8                                                         /* Busy on CPU mode transition of sleep, not include set point trans busy. */
#define IMX9_GPC_CTRL_CMC_MODE_STAT_SLEEP_TRANS_BUSY_FLAG  (1 << IMX9_GPC_CTRL_CMC_MODE_STAT_SLEEP_TRANS_BUSY_SHIFT) /* Busy on CPU mode transition of sleep, not include set point trans busy. */

#define IMX9_GPC_CTRL_CMC_MODE_STAT_WAKEUP_TRANS_BUSY_SHIFT 9                                                          /* Busy on CPU mode transition of wakeup, not include set point trans busy. */
#define IMX9_GPC_CTRL_CMC_MODE_STAT_WAKEUP_TRANS_BUSY_FLAG  (1 << IMX9_GPC_CTRL_CMC_MODE_STAT_WAKEUP_TRANS_BUSY_SHIFT) /* Busy on CPU mode transition of wakeup, not include set point trans busy. */

#define IMX9_GPC_CTRL_CMC_MODE_STAT_SLEEPING_IDLE_SHIFT 10                                                     /* Completed CPU mode and set point transition of sleep sequence, in a sleeping_idle state. */
#define IMX9_GPC_CTRL_CMC_MODE_STAT_SLEEPING_IDLE_FLAG  (1 << IMX9_GPC_CTRL_CMC_MODE_STAT_SLEEPING_IDLE_SHIFT) /* Completed CPU mode and set point transition of sleep sequence, in a sleeping_idle state. */

#define IMX9_GPC_CTRL_CMC_MODE_STAT_SLEEP_REQUEST_SHIFT 16                                                     /* Status of sleep_request input port */
#define IMX9_GPC_CTRL_CMC_MODE_STAT_SLEEP_REQUEST_FLAG  (1 << IMX9_GPC_CTRL_CMC_MODE_STAT_SLEEP_REQUEST_SHIFT) /* Status of sleep_request input port */

#define IMX9_GPC_CTRL_CMC_MODE_STAT_WFE_REQUEST_SHIFT 17                                                   /* Status of standby_wfe input port */
#define IMX9_GPC_CTRL_CMC_MODE_STAT_WFE_REQUEST_FLAG  (1 << IMX9_GPC_CTRL_CMC_MODE_STAT_WFE_REQUEST_SHIFT) /* Status of standby_wfe input port */

#define IMX9_GPC_CTRL_CMC_MODE_STAT_WAKEUP_REQUEST_SHIFT 18                                                      /* "ORed" of all unmasked IRQ */
#define IMX9_GPC_CTRL_CMC_MODE_STAT_WAKEUP_REQUEST_FLAG  (1 << IMX9_GPC_CTRL_CMC_MODE_STAT_WAKEUP_REQUEST_SHIFT) /* "ORed" of all unmasked IRQ */

#define IMX9_GPC_CTRL_CMC_MODE_STAT_FSM_STATE_SHIFT 24   /* CPU mode trans FSM state. */
#define IMX9_GPC_CTRL_CMC_MODE_STAT_FSM_STATE_MASK  0x1f /* CPU mode trans FSM state. */

/* CMC_PIN_STAT register */

#define IMX9_GPC_CTRL_CMC_PIN_STAT_A55_HDSK_REQUEST_STAT_SHIFT 0                                                             /* cpu_mode_trans_a55_hdsk_request pin status */
#define IMX9_GPC_CTRL_CMC_PIN_STAT_A55_HDSK_REQUEST_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_PIN_STAT_A55_HDSK_REQUEST_STAT_SHIFT) /* cpu_mode_trans_a55_hdsk_request pin status */

#define IMX9_GPC_CTRL_CMC_PIN_STAT_SSAR_REQUEST_STAT_SHIFT 1                                                         /* cpu_mode_trans_ssar_request pin status */
#define IMX9_GPC_CTRL_CMC_PIN_STAT_SSAR_REQUEST_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_PIN_STAT_SSAR_REQUEST_STAT_SHIFT) /* cpu_mode_trans_ssar_request pin status */

#define IMX9_GPC_CTRL_CMC_PIN_STAT_RESET_REQUEST_STAT_SHIFT 6                                                          /* cpu_mode_trans_reset_request pin status */
#define IMX9_GPC_CTRL_CMC_PIN_STAT_RESET_REQUEST_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_PIN_STAT_RESET_REQUEST_STAT_SHIFT) /* cpu_mode_trans_reset_request pin status */

#define IMX9_GPC_CTRL_CMC_PIN_STAT_POWER_REQUEST_STAT_SHIFT 7                                                          /* cpu_mode_trans_power_request pin status */
#define IMX9_GPC_CTRL_CMC_PIN_STAT_POWER_REQUEST_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_PIN_STAT_POWER_REQUEST_STAT_SHIFT) /* cpu_mode_trans_power_request pin status */

#define IMX9_GPC_CTRL_CMC_PIN_STAT_SYSMAN_REQUEST_STAT_SHIFT 9                                                           /* cpu_mode_trans_sysman_request pin status */
#define IMX9_GPC_CTRL_CMC_PIN_STAT_SYSMAN_REQUEST_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_PIN_STAT_SYSMAN_REQUEST_STAT_SHIFT) /* cpu_mode_trans_sysman_request pin status */

#define IMX9_GPC_CTRL_CMC_PIN_STAT_A55_HDSK_DONE_STAT_SHIFT 16                                                         /* cpu_mode_trans_a55_hdsk_done pin status */
#define IMX9_GPC_CTRL_CMC_PIN_STAT_A55_HDSK_DONE_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_PIN_STAT_A55_HDSK_DONE_STAT_SHIFT) /* cpu_mode_trans_a55_hdsk_done pin status */

#define IMX9_GPC_CTRL_CMC_PIN_STAT_SSAR_DONE_STAT_SHIFT 17                                                     /* cpu_mode_trans_ssar_done pin status */
#define IMX9_GPC_CTRL_CMC_PIN_STAT_SSAR_DONE_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_PIN_STAT_SSAR_DONE_STAT_SHIFT) /* cpu_mode_trans_ssar_done pin status */

#define IMX9_GPC_CTRL_CMC_PIN_STAT_RESET_DONE_STAT_SHIFT 22                                                      /* cpu_mode_trans_reset_done pin status */
#define IMX9_GPC_CTRL_CMC_PIN_STAT_RESET_DONE_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_PIN_STAT_RESET_DONE_STAT_SHIFT) /* cpu_mode_trans_reset_done pin status */

#define IMX9_GPC_CTRL_CMC_PIN_STAT_POWER_DONE_STAT_SHIFT 23                                                      /* cpu_mode_trans_power_done pin status */
#define IMX9_GPC_CTRL_CMC_PIN_STAT_POWER_DONE_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_PIN_STAT_POWER_DONE_STAT_SHIFT) /* cpu_mode_trans_power_done pin status */

#define IMX9_GPC_CTRL_CMC_PIN_STAT_SYSMAN_DONE_STAT_SHIFT 25                                                       /* cpu_mode_trans_sysman_done pin status. */
#define IMX9_GPC_CTRL_CMC_PIN_STAT_SYSMAN_DONE_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_PIN_STAT_SYSMAN_DONE_STAT_SHIFT) /* cpu_mode_trans_sysman_done pin status. */

#define IMX9_GPC_CTRL_CMC_PIN_STAT_CPU_MODE_STAT_SHIFT 29  /* cpu_power_mode pin status */
#define IMX9_GPC_CTRL_CMC_PIN_STAT_CPU_MODE_STAT_MASK  0x3 /* cpu_power_mode pin status */

#define IMX9_GPC_CTRL_CMC_PIN_STAT_DEBUG_WAKEUP_ACK_STAT_SHIFT 31                                                            /* Debug wakeup acknowledge pin status */
#define IMX9_GPC_CTRL_CMC_PIN_STAT_DEBUG_WAKEUP_ACK_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_PIN_STAT_DEBUG_WAKEUP_ACK_STAT_SHIFT) /* Debug wakeup acknowledge pin status */

/* CMC_NON_IRQ_WAKEUP_MASK register */

#define IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_MASK_EVENT_WAKEUP_MASK_SHIFT 0                                                                    /* "1" means the event cannot wakeup CPU platform */
#define IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_MASK_EVENT_WAKEUP_MASK_FLAG  (1 << IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_MASK_EVENT_WAKEUP_MASK_SHIFT) /* "1" means the event cannot wakeup CPU platform */

#define IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_MASK_DEBUG_WAKEUP_MASK_SHIFT 1                                                                    /* "1" means the debug_wakeup_request cannot wakeup CPU platform */
#define IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_MASK_DEBUG_WAKEUP_MASK_FLAG  (1 << IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_MASK_DEBUG_WAKEUP_MASK_SHIFT) /* "1" means the debug_wakeup_request cannot wakeup CPU platform */

/* CMC_NON_IRQ_WAKEUP_STAT register */

#define IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_STAT_EVENT_WAKEUP_STAT_SHIFT 0                                                                    /* Event wakeup status */
#define IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_STAT_EVENT_WAKEUP_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_STAT_EVENT_WAKEUP_STAT_SHIFT) /* Event wakeup status */

#define IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_STAT_DEBUG_WAKEUP_STAT_SHIFT 1                                                                    /* Debug wakeup status */
#define IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_STAT_DEBUG_WAKEUP_STAT_FLAG  (1 << IMX9_GPC_CTRL_CMC_NON_IRQ_WAKEUP_STAT_DEBUG_WAKEUP_STAT_SHIFT) /* Debug wakeup status */

/* CMC_SLEEP_A55_HDSK_CTRL register */

#define IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_CTRL_STEP_CNT_SHIFT 0        /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE. */
#define IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_CTRL_STEP_CNT_MASK  0xffffff /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE. */

#define IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_CTRL_CNT_MODE_SHIFT 28  /* (keep==0 and invisible on customer RM)Count mode */
#define IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_CTRL_CNT_MODE_MASK  0x3 /* (keep==0 and invisible on customer RM)Count mode */

#define IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_CTRL_DISABLE_SHIFT 31                                                         /* Disable this step */
#define IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_CTRL_DISABLE_FLAG  (1 << IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_CTRL_DISABLE_SHIFT) /* Disable this step */

/* CMC_SLEEP_A55_HDSK_STAT register */

#define IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_STAT_RSP_CNT_SHIFT 0        /* Response count, record the delay from step start to step_done received */
#define IMX9_GPC_CTRL_CMC_SLEEP_A55_HDSK_STAT_RSP_CNT_MASK  0xffffff /* Response count, record the delay from step start to step_done received */

/* CMC_SLEEP_SSAR_CTRL register */

#define IMX9_GPC_CTRL_CMC_SLEEP_SSAR_CTRL_STEP_CNT_SHIFT 0        /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE. */
#define IMX9_GPC_CTRL_CMC_SLEEP_SSAR_CTRL_STEP_CNT_MASK  0xffffff /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE. */

#define IMX9_GPC_CTRL_CMC_SLEEP_SSAR_CTRL_CNT_MODE_SHIFT 28  /* (keep==0 and invisible on customer RM)Count mode */
#define IMX9_GPC_CTRL_CMC_SLEEP_SSAR_CTRL_CNT_MODE_MASK  0x3 /* (keep==0 and invisible on customer RM)Count mode */

#define IMX9_GPC_CTRL_CMC_SLEEP_SSAR_CTRL_DISABLE_SHIFT 31                                                     /* Disable this step */
#define IMX9_GPC_CTRL_CMC_SLEEP_SSAR_CTRL_DISABLE_FLAG  (1 << IMX9_GPC_CTRL_CMC_SLEEP_SSAR_CTRL_DISABLE_SHIFT) /* Disable this step */

/* CMC_SLEEP_SSAR_STAT register */

#define IMX9_GPC_CTRL_CMC_SLEEP_SSAR_STAT_RSP_CNT_SHIFT 0        /* Response count, record the delay from step start to step_done received */
#define IMX9_GPC_CTRL_CMC_SLEEP_SSAR_STAT_RSP_CNT_MASK  0xffffff /* Response count, record the delay from step start to step_done received */

/* CMC_SLEEP_RESET_CTRL register */

#define IMX9_GPC_CTRL_CMC_SLEEP_RESET_CTRL_STEP_CNT_SHIFT 0        /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE */
#define IMX9_GPC_CTRL_CMC_SLEEP_RESET_CTRL_STEP_CNT_MASK  0xffffff /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE */

#define IMX9_GPC_CTRL_CMC_SLEEP_RESET_CTRL_CNT_MODE_SHIFT 28  /* (keep==0 and invisible on customer RM)Count mode */
#define IMX9_GPC_CTRL_CMC_SLEEP_RESET_CTRL_CNT_MODE_MASK  0x3 /* (keep==0 and invisible on customer RM)Count mode */

#define IMX9_GPC_CTRL_CMC_SLEEP_RESET_CTRL_DISABLE_SHIFT 31                                                      /* Disable this step */
#define IMX9_GPC_CTRL_CMC_SLEEP_RESET_CTRL_DISABLE_FLAG  (1 << IMX9_GPC_CTRL_CMC_SLEEP_RESET_CTRL_DISABLE_SHIFT) /* Disable this step */

/* CMC_SLEEP_RESET_STAT register */

#define IMX9_GPC_CTRL_CMC_SLEEP_RESET_STAT_RSP_CNT_SHIFT 0        /* Response count, record the delay from step start to step_done received */
#define IMX9_GPC_CTRL_CMC_SLEEP_RESET_STAT_RSP_CNT_MASK  0xffffff /* Response count, record the delay from step start to step_done received */

/* CMC_SLEEP_SYSMAN_CTRL register */

#define IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_CTRL_STEP_CNT_SHIFT 0        /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE. */
#define IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_CTRL_STEP_CNT_MASK  0xffffff /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE. */

#define IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_CTRL_CNT_MODE_SHIFT 28  /* (keep==0 and invisible on customer RM)Count mode */
#define IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_CTRL_CNT_MODE_MASK  0x3 /* (keep==0 and invisible on customer RM)Count mode */

#define IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_CTRL_DISABLE_SHIFT 31                                                       /* Disable this step */
#define IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_CTRL_DISABLE_FLAG  (1 << IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_CTRL_DISABLE_SHIFT) /* Disable this step */

/* CMC_SLEEP_SYSMAN_STAT register */

#define IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_STAT_RSP_CNT_SHIFT 0        /* Response count, record the delay from step start to step_done received */
#define IMX9_GPC_CTRL_CMC_SLEEP_SYSMAN_STAT_RSP_CNT_MASK  0xffffff /* Response count, record the delay from step start to step_done received */

/* CMC_WAKEUP_POWER_CTRL register */

#define IMX9_GPC_CTRL_CMC_WAKEUP_POWER_CTRL_STEP_CNT_SHIFT 0        /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE */
#define IMX9_GPC_CTRL_CMC_WAKEUP_POWER_CTRL_STEP_CNT_MASK  0xffffff /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE */

#define IMX9_GPC_CTRL_CMC_WAKEUP_POWER_CTRL_CNT_MODE_SHIFT 28  /* (keep==0 and invisible on customer RM)Count mode */
#define IMX9_GPC_CTRL_CMC_WAKEUP_POWER_CTRL_CNT_MODE_MASK  0x3 /* (keep==0 and invisible on customer RM)Count mode */

#define IMX9_GPC_CTRL_CMC_WAKEUP_POWER_CTRL_DISABLE_SHIFT 31                                                       /* Disable this step */
#define IMX9_GPC_CTRL_CMC_WAKEUP_POWER_CTRL_DISABLE_FLAG  (1 << IMX9_GPC_CTRL_CMC_WAKEUP_POWER_CTRL_DISABLE_SHIFT) /* Disable this step */

/* CMC_WAKEUP_POWER_STAT register */

#define IMX9_GPC_CTRL_CMC_WAKEUP_POWER_STAT_RSP_CNT_SHIFT 0        /* Response count, record the delay from step start to step_done received */
#define IMX9_GPC_CTRL_CMC_WAKEUP_POWER_STAT_RSP_CNT_MASK  0xffffff /* Response count, record the delay from step start to step_done received */

/* CMC_WAKEUP_SSAR_CTRL register */

#define IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_CTRL_STEP_CNT_SHIFT 0        /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_CTRL_STEP_CNT_MASK  0xffffff /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE */

#define IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_CTRL_CNT_MODE_SHIFT 28  /* (keep==0 and invisible on customer RM)Count mode */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_CTRL_CNT_MODE_MASK  0x3 /* (keep==0 and invisible on customer RM)Count mode */

#define IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_CTRL_DISABLE_SHIFT 31                                                      /* Disable this step */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_CTRL_DISABLE_FLAG  (1 << IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_CTRL_DISABLE_SHIFT) /* Disable this step */

/* CMC_WAKEUP_SSAR_STAT register */

#define IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_STAT_RSP_CNT_SHIFT 0        /* Response count, record the delay from step start to step_done received */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SSAR_STAT_RSP_CNT_MASK  0xffffff /* Response count, record the delay from step start to step_done received */

/* CMC_WAKEUP_A55_HDSK_CTRL register */

#define IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_CTRL_STEP_CNT_SHIFT 0        /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE */
#define IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_CTRL_STEP_CNT_MASK  0xffffff /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE */

#define IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_CTRL_CNT_MODE_SHIFT 28  /* (keep==0 and invisible on customer RM)Count mode */
#define IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_CTRL_CNT_MODE_MASK  0x3 /* (keep==0 and invisible on customer RM)Count mode */

#define IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_CTRL_DISABLE_SHIFT 31                                                          /* Disable this step */
#define IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_CTRL_DISABLE_FLAG  (1 << IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_CTRL_DISABLE_SHIFT) /* Disable this step */

/* CMC_WAKEUP_A55_HDSK_STAT register */

#define IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_STAT_RSP_CNT_SHIFT 0        /* Response count, record the delay from step start to step_done received */
#define IMX9_GPC_CTRL_CMC_WAKEUP_A55_HDSK_STAT_RSP_CNT_MASK  0xffffff /* Response count, record the delay from step start to step_done received */

/* CMC_WAKEUP_SYSMAN_CTRL register */

#define IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_CTRL_STEP_CNT_SHIFT 0        /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_CTRL_STEP_CNT_MASK  0xffffff /* (invalid when CNT_MODE==0 and invisible on customer RM)Step count, usage depends on CNT_MODE */

#define IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_CTRL_CNT_MODE_SHIFT 28  /* (keep==0 and invisible on customer RM)Count mode */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_CTRL_CNT_MODE_MASK  0x3 /* (keep==0 and invisible on customer RM)Count mode */

#define IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_CTRL_DISABLE_SHIFT 31                                                        /* Disable this step */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_CTRL_DISABLE_FLAG  (1 << IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_CTRL_DISABLE_SHIFT) /* Disable this step */

/* CMC_WAKEUP_SYSMAN_STAT register */

#define IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_STAT_RSP_CNT_SHIFT 0        /* Response count, record the delay from step start to step_done received */
#define IMX9_GPC_CTRL_CMC_WAKEUP_SYSMAN_STAT_RSP_CNT_MASK  0xffffff /* Response count, record the delay from step start to step_done received */

/* CMC_SYS_SLEEP_CTRL register */

#define IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SS_WAIT_SHIFT 0                                                     /* Request system sleep when CPU is in WAIT mode */
#define IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SS_WAIT_FLAG  (1 << IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SS_WAIT_SHIFT) /* Request system sleep when CPU is in WAIT mode */

#define IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SS_STOP_SHIFT 1                                                     /* Request system sleep when CPU is in STOP mode */
#define IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SS_STOP_FLAG  (1 << IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SS_STOP_SHIFT) /* Request system sleep when CPU is in STOP mode */

#define IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SS_SUSPEND_SHIFT 2                                                        /* Request system sleep when CPU is in SUSPEND mode */
#define IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SS_SUSPEND_FLAG  (1 << IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SS_SUSPEND_SHIFT) /* Request system sleep when CPU is in SUSPEND mode */

#define IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SYS_SLEEP_BUSY_SHIFT 16                                                           /* Indicates the CPU is busy entering system sleep mode. */
#define IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SYS_SLEEP_BUSY_FLAG  (1 << IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SYS_SLEEP_BUSY_SHIFT) /* Indicates the CPU is busy entering system sleep mode. */

#define IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SYS_WAKEUP_BUSY_SHIFT 17                                                            /* Indicates the CPU is busy exiting system sleep mode. */
#define IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SYS_WAKEUP_BUSY_FLAG  (1 << IMX9_GPC_CTRL_CMC_SYS_SLEEP_CTRL_SYS_WAKEUP_BUSY_SHIFT) /* Indicates the CPU is busy exiting system sleep mode. */

/* CMC_DEBUG register */

#define IMX9_GPC_CTRL_CMC_DEBUG_PRETEND_SLEEP_SHIFT 0                                                  /* Write 1 to force CMC into sleep. Used to debug GPC status. Locked by LOCK_CFG field. */
#define IMX9_GPC_CTRL_CMC_DEBUG_PRETEND_SLEEP_FLAG  (1 << IMX9_GPC_CTRL_CMC_DEBUG_PRETEND_SLEEP_SHIFT) /* Write 1 to force CMC into sleep. Used to debug GPC status. Locked by LOCK_CFG field. */

/* Register array dimensions */

#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_MASK__REGARRAY_SIZE 12
#define IMX9_GPC_CTRL_CMC_IRQ_WAKEUP_STAT__REGARRAY_SIZE 12

#endif /* __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_GPC_H */
