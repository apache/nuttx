/****************************************************************************
 * arch/arm/src/max326xx/hardware/max326_pwrseq.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_PWRSEQ_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_PWRSEQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define MAX326_PWRSEQ_LPCTRL_OFFSET   0x0000 /* POR Low Power Voltage Control Register */
#define MAX326_PWRSEQ_LPWAKEFL_OFFSET 0x0004 /* POR Low Power Mode Wakeup Flags for GPIO0 */
#define MAX326_PWRSEQ_LPWKEN_OFFSET   0x0008 /* POR GPIO0 Wakeup Enable */
#define MAX326_PWRSEQ_LPMEMSD_OFFSET  0x0020 /* POR RAM Shut Down Control */

/* Register Addresses *******************************************************/

#define MAX326_PWRSEQ_LPCTRL          (MAX326_PWRSEQ_BASE + MAX326_PWRSEQ_LPCTRL_OFFSET)
#define MAX326_PWRSEQ_LPWAKEFL        (MAX326_PWRSEQ_BASE + MAX326_PWRSEQ_LPWAKEFL_OFFSET)
#define MAX326_PWRSEQ_LPWKEN          (MAX326_PWRSEQ_BASE + MAX326_PWRSEQ_LPWKEN_OFFSET)
#define MAX326_PWRSEQ_LPMEMSD         (MAX326_PWRSEQ_BASE + MAX326_PWRSEQ_LPMEMSD_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* POR Low Power Voltage Control Register */

#define PWRSEQ_LPCTRL_RAMRETSEL0      (1 << 0)  /* Bit 0:  System RAM 0 Data Retention
                                                 *         Enable for BACKUP Mode */
#define PWRSEQ_LPCTRL_RAMRETSEL1      (1 << 1)  /* Bit 1:  System RAM 1 Data Retention
                                                 *         Enable for BACKUP Mode */
#define PWRSEQ_LPCTRL_RAMRETSEL2      (1 << 2)  /* Bit 2:  System RAM 2 Data Retention
                                                 *         Enable for BACKUP Mode */
#define PWRSEQ_LPCTRL_RAMRETSEL3      (1 << 3)  /* Bit 3:  System RAM 3 Data Retention
                                                 *         Enable for BACKUP Mode */
#define PWRSEQ_LPCTRL_OVR_SHIFT       (4)       /* Output Voltage Range */
#define PWRSEQ_LPCTRL_OVR_MASK        (3 << PWRSEQ_LPCTRL_OVR_SHIFT)
#  define PWRSEQ_LPCTRL_OVR(n)        ((uint32_t)(n) << PWRSEQ_LPCTRL_OVR_SHIFT)
#  define PWRSEQ_LPCTRL_OVR_1p1V      (2 << PWRSEQ_LPCTRL_OVR_SHIFT) /* VCORE=1.1V fINTCLK=96MHz */
#  define PWRSEQ_LPCTRL_OVR_1p0V      (1 << PWRSEQ_LPCTRL_OVR_SHIFT) /* VCORE=1.0V fINTCLK=48MHz */
#  define PWRSEQ_LPCTRL_OVR_0p9V      (0 << PWRSEQ_LPCTRL_OVR_SHIFT) /* VCORE=0.9V fINTCLK=24MHz */

#define PWRSEQ_LPCTRL_VCORE_DETBYPASS (1 << 6)  /* Bit 6:  Bypass VCORE External Supply
                                                 *         Detection */
#define PWRSEQ_LPCTRL_RETREGEN        (1 << 8)  /* Bit 8:  RAM Retention Regulator
                                                 *         Enable for BACKUP Mode */
#define PWRSEQ_LPCTRL_FASTWKEN        (1 << 10) /* Bit 10: Fast Wakeup Enable for
                                                 *         DEEPSLEEP Mode */
#define PWRSEQ_LPCTRL_BGOFF           (1 << 11) /* Bit 11: Band Gap Disable for
                                                 *         DEEPSLEEP and BACKUP Mode */
#define PWRSEQ_LPCTRL_VCORE_PORDIS    (1 << 12) /* Bit 12: VCORE POR Disable for
                                                 *         DEEPSLEEP and BACKUP Mode */
#define PWRSEQ_LPCTRL_LDODIS          (1 << 16) /* Bit 16: LDO Disable */
#define PWRSEQ_LPCTRL_VCORE_SVMDIS    (1 << 20) /* Bit 20: VCORE Supply Voltage Monitor
                                                 *         Disable */
#define PWRSEQ_LPCTRL_VDDIO_PORDIS    (1 << 25) /* Bit 25: VDDIO Power-On-Reset Monitor
                                                 *         Disable */

/* POR Low Power Mode Wakeup Flags for GPIO0 */

#define PWRSEQ_LPWAKEFL(n)            (1 << (n)) /* Bits 0-13: GPIO Pin n Wakeup
                                                  *            Status Flag */

/* POR GPIO0 Wakeup Enable */

#define PWRSEQ_LPWKEN(n)              (1 << (n)) /* Bits 0-13: GPIO Pin n Wakeup
                                                  *            Interrupt Enable */

/* POR RAM Shut Down Control */

#define PWRSEQ_LPMEMSD_SRAM0OFF       (1 << 0)  /* Bit 0:  System RAM 0 Shut Down
                                                 *         (0x20000000–0x20003fff) */
#define PWRSEQ_LPMEMSD_SRAM1OFF       (1 << 1)  /* Bit 1:  System RAM 1 Shut Down
                                                 *         (0x20004000-0x20007fff) */
#define PWRSEQ_LPMEMSD_SRAM2OFF       (1 << 2)  /* Bit 2:  System RAM 2 Shut Down
                                                 *         (0x20008000-0x2000ffff) */
#define PWRSEQ_LPMEMSD_SRAM3OFF       (1 << 3)  /* Bit 3:  System RAM 3 Shut Down
                                                 *         (0x20010000-0x20017fff) */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_PWRSEQ_H */
