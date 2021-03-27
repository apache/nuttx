/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_ftm.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FTM_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FTM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/s32k1xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define S32K1XX_FTM_SC_OFFSET            0x0000 /* Status And Control register offset */
#define S32K1XX_FTM_CNT_OFFSET           0x0004 /* Counter register offset */
#define S32K1XX_FTM_MOD_OFFSET           0x0008 /* Modulo register offset */
#define S32K1XX_FTM_C0SC_OFFSET          0x000c /* Channel 0 Status And Control register offset */
#define S32K1XX_FTM_C0V_OFFSET           0x0010 /* Channel 0 Value register offset */
#define S32K1XX_FTM_C1SC_OFFSET          0x0014 /* Channel 1 Status And Control register offset */
#define S32K1XX_FTM_C1V_OFFSET           0x0018 /* Channel 1 Value register offset */
#define S32K1XX_FTM_C2SC_OFFSET          0x001c /* Channel 2 Status And Control register offset */
#define S32K1XX_FTM_C2V_OFFSET           0x0020 /* Channel 2 Value register offset */
#define S32K1XX_FTM_C3SC_OFFSET          0x0024 /* Channel 3 Status And Control register offset */
#define S32K1XX_FTM_C3V_OFFSET           0x0028 /* Channel 3 Value register offset */
#define S32K1XX_FTM_C4SC_OFFSET          0x002c /* Channel 4 Status And Control register offset */
#define S32K1XX_FTM_C4V_OFFSET           0x0030 /* Channel 4 Value register offset */
#define S32K1XX_FTM_C5SC_OFFSET          0x0034 /* Channel 5 Status And Control register offset */
#define S32K1XX_FTM_C5V_OFFSET           0x0038 /* Channel 5 Value register offset */
#define S32K1XX_FTM_C6SC_OFFSET          0x003c /* Channel 6 Status And Control register offset */
#define S32K1XX_FTM_C6V_OFFSET           0x0040 /* Channel 6 Value register offset */
#define S32K1XX_FTM_C7SC_OFFSET          0x0044 /* Channel 7 Status And Control register offset */
#define S32K1XX_FTM_C7V_OFFSET           0x0048 /* Channel 7 Value register offset */
#define S32K1XX_FTM_CNTIN_OFFSET         0x004c /* Counter Initial Value register offset */
#define S32K1XX_FTM_STATUS_OFFSET        0x0050 /* Capture And Compare Status register offset */
#define S32K1XX_FTM_MODE_OFFSET          0x0054 /* Features Mode Selection register offset */
#define S32K1XX_FTM_SYNC_OFFSET          0x0058 /* Synchronization register offset */
#define S32K1XX_FTM_OUTINIT_OFFSET       0x005c /* Initial State For Channels Output register offset */
#define S32K1XX_FTM_OUTMASK_OFFSET       0x0060 /* Output Mask register offset */
#define S32K1XX_FTM_COMBINE_OFFSET       0x0064 /* Function For Linked Channels register offset */
#define S32K1XX_FTM_DEADTIME_OFFSET      0x0068 /* Deadtime Configuration register offset */
#define S32K1XX_FTM_EXTTRIG_OFFSET       0x006c /* FTM External Trigger register offset */
#define S32K1XX_FTM_POL_OFFSET           0x0070 /* Channel Polarity register offset */
#define S32K1XX_FTM_FMS_OFFSET           0x0074 /* Fault Mode Status register offset */
#define S32K1XX_FTM_FILTER_OFFSET        0x0078 /* Input Capture Filter Control register offset */
#define S32K1XX_FTM_FLTCTRL_OFFSET       0x007c /* Fault Control register offset */
#define S32K1XX_FTM_QDCTRL_OFFSET        0x0080 /* Quadrature Decoder Control And Status register offset */
#define S32K1XX_FTM_CONF_OFFSET          0x0084 /* Configuration register offset */
#define S32K1XX_FTM_FLTPOL_OFFSET        0x0088 /* FTM Fault Input Polarity register offset */
#define S32K1XX_FTM_SYNCONF_OFFSET       0x008c /* Synchronization Configuration register offset */
#define S32K1XX_FTM_INVCTRL_OFFSET       0x0090 /* FTM Inverting Control register offset */
#define S32K1XX_FTM_SWOCTRL_OFFSET       0x0094 /* FTM Software Output Control register offset */
#define S32K1XX_FTM_PWMLOAD_OFFSET       0x0098 /* FTM PWM Load register offset */
#define S32K1XX_FTM_HCR_OFFSET           0x009c /* Half Cycle Register offset */

#define S32K1XX_FTM_PAIR0DEADTIME_OFFSET 0x00a0 /* Pair 0 Deadtime Configuration register offset */
#define S32K1XX_FTM_PAIR1DEADTIME_OFFSET 0x00a8 /* Pair 1 Deadtime Configuration register offset */
#define S32K1XX_FTM_PAIR2DEADTIME_OFFSET 0x00b0 /* Pair 2 Deadtime Configuration register offset */
#define S32K1XX_FTM_PAIR3DEADTIME_OFFSET 0x00b8 /* Pair 3 Deadtime Configuration register offset */

#define S32K1XX_FTM_MOD_MIRROR_OFFSET    0x0200 /* Mirror of Modulo Value register offset */
#define S32K1XX_FTM_C0V_MIRROR_OFFSET    0x0204 /* Mirror of Channel 0 Match Value register offset */
#define S32K1XX_FTM_C1V_MIRROR_OFFSET    0x0208 /* Mirror of Channel 1 Match Value register offset */
#define S32K1XX_FTM_C2V_MIRROR_OFFSET    0x020c /* Mirror of Channel 2 Match Value register offset */
#define S32K1XX_FTM_C3V_MIRROR_OFFSET    0x0210 /* Mirror of Channel 3 Match Value register offset */
#define S32K1XX_FTM_C4V_MIRROR_OFFSET    0x0214 /* Mirror of Channel 4 Match Value register offset */
#define S32K1XX_FTM_C5V_MIRROR_OFFSET    0x0218 /* Mirror of Channel 5 Match Value register offset */
#define S32K1XX_FTM_C6V_MIRROR_OFFSET    0x021c /* Mirror of Channel 6 Match Value register offset */
#define S32K1XX_FTM_C7V_MIRROR_OFFSET    0x0220 /* Mirror of Channel 7 Match Value register offset */

#define S32K1XX_FTM_CNSC_OFFSET(n)       (0x000c + (n) * 0x0008) /* Channel (n) Status And Control register offset */
#define S32K1XX_FTM_CNV_OFFSET(n)        (0x0010 + (n) * 0x0008) /* Channel (n) Value register offset */

/* Register Addresses *******************************************************/

/* FTM0 registers */

#define S32K1XX_FTM0_SC                  (S32K1XX_FTM0_BASE + S32K1XX_FTM_SC_OFFSET)       /* FTM0 Status And Control register */
#define S32K1XX_FTM0_CNT                 (S32K1XX_FTM0_BASE + S32K1XX_FTM_CNT_OFFSET)      /* FTM0 Counter register */
#define S32K1XX_FTM0_MOD                 (S32K1XX_FTM0_BASE + S32K1XX_FTM_MOD_OFFSET)      /* FTM0 Modulo register */
#define S32K1XX_FTM0_C0SC                (S32K1XX_FTM0_BASE + S32K1XX_FTM_C0SC_OFFSET)     /* FTM0 Channel 0 Status And Control register */
#define S32K1XX_FTM0_C0V                 (S32K1XX_FTM0_BASE + S32K1XX_FTM_C0V_OFFSET)      /* FTM0 Channel 0 Value register */
#define S32K1XX_FTM0_C1SC                (S32K1XX_FTM0_BASE + S32K1XX_FTM_C1SC_OFFSET)     /* FTM0 Channel 1 Status And Control register */
#define S32K1XX_FTM0_C1V                 (S32K1XX_FTM0_BASE + S32K1XX_FTM_C1V_OFFSET)      /* FTM0 Channel 1 Value register */
#define S32K1XX_FTM0_C2SC                (S32K1XX_FTM0_BASE + S32K1XX_FTM_C2SC_OFFSET)     /* FTM0 Channel 2 Status And Control register */
#define S32K1XX_FTM0_C2V                 (S32K1XX_FTM0_BASE + S32K1XX_FTM_C2V_OFFSET)      /* FTM0 Channel 2 Value register */
#define S32K1XX_FTM0_C3SC                (S32K1XX_FTM0_BASE + S32K1XX_FTM_C3SC_OFFSET)     /* FTM0 Channel 3 Status And Control register */
#define S32K1XX_FTM0_C3V                 (S32K1XX_FTM0_BASE + S32K1XX_FTM_C3V_OFFSET)      /* FTM0 Channel 3 Value register */
#define S32K1XX_FTM0_C4SC                (S32K1XX_FTM0_BASE + S32K1XX_FTM_C4SC_OFFSET)     /* FTM0 Channel 4 Status And Control register */
#define S32K1XX_FTM0_C4V                 (S32K1XX_FTM0_BASE + S32K1XX_FTM_C4V_OFFSET)      /* FTM0 Channel 4 Value register */
#define S32K1XX_FTM0_C5SC                (S32K1XX_FTM0_BASE + S32K1XX_FTM_C5SC_OFFSET)     /* FTM0 Channel 5 Status And Control register */
#define S32K1XX_FTM0_C5V                 (S32K1XX_FTM0_BASE + S32K1XX_FTM_C5V_OFFSET)      /* FTM0 Channel 5 Value register */
#define S32K1XX_FTM0_C6SC                (S32K1XX_FTM0_BASE + S32K1XX_FTM_C6SC_OFFSET)     /* FTM0 Channel 6 Status And Control register */
#define S32K1XX_FTM0_C6V                 (S32K1XX_FTM0_BASE + S32K1XX_FTM_C6V_OFFSET)      /* FTM0 Channel 6 Value register */
#define S32K1XX_FTM0_C7SC                (S32K1XX_FTM0_BASE + S32K1XX_FTM_C7SC_OFFSET)     /* FTM0 Channel 7 Status And Control register */
#define S32K1XX_FTM0_C7V                 (S32K1XX_FTM0_BASE + S32K1XX_FTM_C7V_OFFSET)      /* FTM0 Channel 7 Value register */
#define S32K1XX_FTM0_CNTIN               (S32K1XX_FTM0_BASE + S32K1XX_FTM_CNTIN_OFFSET)    /* FTM0 Counter Initial Value register */
#define S32K1XX_FTM0_STATUS              (S32K1XX_FTM0_BASE + S32K1XX_FTM_STATUS_OFFSET)   /* FTM0 Capture And Compare Status register */
#define S32K1XX_FTM0_MODE                (S32K1XX_FTM0_BASE + S32K1XX_FTM_MODE_OFFSET)     /* FTM0 Features Mode Selection register */
#define S32K1XX_FTM0_SYNC                (S32K1XX_FTM0_BASE + S32K1XX_FTM_SYNC_OFFSET)     /* FTM0 Synchronization register */
#define S32K1XX_FTM0_OUTINIT             (S32K1XX_FTM0_BASE + S32K1XX_FTM_OUTINIT_OFFSET)  /* FTM0 Initial State For Channels Output register */
#define S32K1XX_FTM0_OUTMASK             (S32K1XX_FTM0_BASE + S32K1XX_FTM_OUTMASK_OFFSET)  /* FTM0 Output Mask register */
#define S32K1XX_FTM0_COMBINE             (S32K1XX_FTM0_BASE + S32K1XX_FTM_COMBINE_OFFSET)  /* FTM0 Function For Linked Channels register */
#define S32K1XX_FTM0_DEADTIME            (S32K1XX_FTM0_BASE + S32K1XX_FTM_DEADTIME_OFFSET) /* FTM0 Deadtime Configuration register */
#define S32K1XX_FTM0_EXTTRIG             (S32K1XX_FTM0_BASE + S32K1XX_FTM_EXTTRIG_OFFSET)  /* FTM0 External Trigger register */
#define S32K1XX_FTM0_POL                 (S32K1XX_FTM0_BASE + S32K1XX_FTM_POL_OFFSET)      /* FTM0 Channel Polarity register */
#define S32K1XX_FTM0_FMS                 (S32K1XX_FTM0_BASE + S32K1XX_FTM_FMS_OFFSET)      /* FTM0 Fault Mode Status register */
#define S32K1XX_FTM0_FILTER              (S32K1XX_FTM0_BASE + S32K1XX_FTM_FILTER_OFFSET)   /* FTM0 Input Capture Filter Control register */
#define S32K1XX_FTM0_FLTCTRL             (S32K1XX_FTM0_BASE + S32K1XX_FTM_FILTCTRL_OFFSET) /* FTM0 Fault Control register */
#define S32K1XX_FTM0_QDCTRL              (S32K1XX_FTM0_BASE + S32K1XX_FTM_QDCTRL_OFFSET)   /* FTM0 Quadrature Decoder Control And Status register */
#define S32K1XX_FTM0_CONF                (S32K1XX_FTM0_BASE + S32K1XX_FTM_CONF_OFFSET)     /* FTM0 Configuration register */
#define S32K1XX_FTM0_FLTPOL              (S32K1XX_FTM0_BASE + S32K1XX_FTM_FLTPOL_OFFSET)   /* FTM0 Fault Input Polarity register */
#define S32K1XX_FTM0_SYNCONF             (S32K1XX_FTM0_BASE + S32K1XX_FTM_SYNCONF_OFFSET)  /* FTM0 Synchronization Configuration register */
#define S32K1XX_FTM0_INVCTRL             (S32K1XX_FTM0_BASE + S32K1XX_FTM_INVCTRL_OFFSET)  /* FTM0 Inverting Control register */
#define S32K1XX_FTM0_SWOCTRL             (S32K1XX_FTM0_BASE + S32K1XX_FTM_SWOCTRL_OFFSET)  /* FTM0 Software Output Control register */
#define S32K1XX_FTM0_PWMLOAD             (S32K1XX_FTM0_BASE + S32K1XX_FTM_PWMLOAD_OFFSET)  /* FTM0 PWM Load register */
#define S32K1XX_FTM0_HCR                 (S32K1XX_FTM0_BASE + S32K1XX_FTM_HCR_OFFSET)      /* FTM0 Half Cycle Register */

#define S32K1XX_FTM0_PAIR0DEADTIME       (S32K1XX_FTM0_BASE + S32K1XX_FTM_PAIR0DEADTIME_OFFSET) /* FTM0 Pair 0 Deadtime Configuration register */
#define S32K1XX_FTM0_PAIR1DEADTIME       (S32K1XX_FTM0_BASE + S32K1XX_FTM_PAIR1DEADTIME_OFFSET) /* FTM0 Pair 1 Deadtime Configuration register */
#define S32K1XX_FTM0_PAIR2DEADTIME       (S32K1XX_FTM0_BASE + S32K1XX_FTM_PAIR2DEADTIME_OFFSET) /* FTM0 Pair 2 Deadtime Configuration register */
#define S32K1XX_FTM0_PAIR3DEADTIME       (S32K1XX_FTM0_BASE + S32K1XX_FTM_PAIR3DEADTIME_OFFSET) /* FTM0 Pair 3 Deadtime Configuration register oset */

#define S32K1XX_FTM0_MOD_MIRROR          (S32K1XX_FTM0_BASE + S32K1XX_FTM_MOD_MIRROR_OFFSET) /* FTM0 Mirror of Modulo Value register */
#define S32K1XX_FTM0_C0V_MIRROR          (S32K1XX_FTM0_BASE + S32K1XX_FTM_C0V_MIRROR_OFFSET) /* FTM0 Mirror of Channel 0 Match Value register */
#define S32K1XX_FTM0_C1V_MIRROR          (S32K1XX_FTM0_BASE + S32K1XX_FTM_C1V_MIRROR_OFFSET) /* FTM0 Mirror of Channel 1 Match Value register */
#define S32K1XX_FTM0_C2V_MIRROR          (S32K1XX_FTM0_BASE + S32K1XX_FTM_C2V_MIRROR_OFFSET) /* FTM0 Mirror of Channel 2 Match Value register */
#define S32K1XX_FTM0_C3V_MIRROR          (S32K1XX_FTM0_BASE + S32K1XX_FTM_C3V_MIRROR_OFFSET) /* FTM0 Mirror of Channel 3 Match Value register */
#define S32K1XX_FTM0_C4V_MIRROR          (S32K1XX_FTM0_BASE + S32K1XX_FTM_C4V_MIRROR_OFFSET) /* FTM0 Mirror of Channel 4 Match Value register */
#define S32K1XX_FTM0_C5V_MIRROR          (S32K1XX_FTM0_BASE + S32K1XX_FTM_C5V_MIRROR_OFFSET) /* FTM0 Mirror of Channel 5 Match Value register */
#define S32K1XX_FTM0_C6V_MIRROR          (S32K1XX_FTM0_BASE + S32K1XX_FTM_C6V_MIRROR_OFFSET) /* FTM0 Mirror of Channel 6 Match Value register */
#define S32K1XX_FTM0_C7V_MIRROR          (S32K1XX_FTM0_BASE + S32K1XX_FTM_C7V_MIRROR_OFFSET) /* FTM0 Mirror of Channel 7 Match Value register */

/* FTM1 registers */

#define S32K1XX_FTM1_SC                  (S32K1XX_FTM1_BASE + S32K1XX_FTM_SC_OFFSET)       /* FTM1 Status And Control register */
#define S32K1XX_FTM1_CNT                 (S32K1XX_FTM1_BASE + S32K1XX_FTM_CNT_OFFSET)      /* FTM1 Counter register */
#define S32K1XX_FTM1_MOD                 (S32K1XX_FTM1_BASE + S32K1XX_FTM_MOD_OFFSET)      /* FTM1 Modulo register */
#define S32K1XX_FTM1_C0SC                (S32K1XX_FTM1_BASE + S32K1XX_FTM_C0SC_OFFSET)     /* FTM1 Channel 0 Status And Control register */
#define S32K1XX_FTM1_C0V                 (S32K1XX_FTM1_BASE + S32K1XX_FTM_C0V_OFFSET)      /* FTM1 Channel 0 Value register */
#define S32K1XX_FTM1_C1SC                (S32K1XX_FTM1_BASE + S32K1XX_FTM_C1SC_OFFSET)     /* FTM1 Channel 1 Status And Control register */
#define S32K1XX_FTM1_C1V                 (S32K1XX_FTM1_BASE + S32K1XX_FTM_C1V_OFFSET)      /* FTM1 Channel 1 Value register */
#define S32K1XX_FTM1_C2SC                (S32K1XX_FTM1_BASE + S32K1XX_FTM_C2SC_OFFSET)     /* FTM1 Channel 2 Status And Control register */
#define S32K1XX_FTM1_C2V                 (S32K1XX_FTM1_BASE + S32K1XX_FTM_C2V_OFFSET)      /* FTM1 Channel 2 Value register */
#define S32K1XX_FTM1_C3SC                (S32K1XX_FTM1_BASE + S32K1XX_FTM_C3SC_OFFSET)     /* FTM1 Channel 3 Status And Control register */
#define S32K1XX_FTM1_C3V                 (S32K1XX_FTM1_BASE + S32K1XX_FTM_C3V_OFFSET)      /* FTM1 Channel 3 Value register */
#define S32K1XX_FTM1_C4SC                (S32K1XX_FTM1_BASE + S32K1XX_FTM_C4SC_OFFSET)     /* FTM1 Channel 4 Status And Control register */
#define S32K1XX_FTM1_C4V                 (S32K1XX_FTM1_BASE + S32K1XX_FTM_C4V_OFFSET)      /* FTM1 Channel 4 Value register */
#define S32K1XX_FTM1_C5SC                (S32K1XX_FTM1_BASE + S32K1XX_FTM_C5SC_OFFSET)     /* FTM1 Channel 5 Status And Control register */
#define S32K1XX_FTM1_C5V                 (S32K1XX_FTM1_BASE + S32K1XX_FTM_C5V_OFFSET)      /* FTM1 Channel 5 Value register */
#define S32K1XX_FTM1_C6SC                (S32K1XX_FTM1_BASE + S32K1XX_FTM_C6SC_OFFSET)     /* FTM1 Channel 6 Status And Control register */
#define S32K1XX_FTM1_C6V                 (S32K1XX_FTM1_BASE + S32K1XX_FTM_C6V_OFFSET)      /* FTM1 Channel 6 Value register */
#define S32K1XX_FTM1_C7SC                (S32K1XX_FTM1_BASE + S32K1XX_FTM_C7SC_OFFSET)     /* FTM1 Channel 7 Status And Control register */
#define S32K1XX_FTM1_C7V                 (S32K1XX_FTM1_BASE + S32K1XX_FTM_C7V_OFFSET)      /* FTM1 Channel 7 Value register */
#define S32K1XX_FTM1_CNTIN               (S32K1XX_FTM1_BASE + S32K1XX_FTM_CNTIN_OFFSET)    /* FTM1 Counter Initial Value register */
#define S32K1XX_FTM1_STATUS              (S32K1XX_FTM1_BASE + S32K1XX_FTM_STATUS_OFFSET)   /* FTM1 Capture And Compare Status register */
#define S32K1XX_FTM1_MODE                (S32K1XX_FTM1_BASE + S32K1XX_FTM_MODE_OFFSET)     /* FTM1 Features Mode Selection register */
#define S32K1XX_FTM1_SYNC                (S32K1XX_FTM1_BASE + S32K1XX_FTM_SYNC_OFFSET)     /* FTM1 Synchronization register */
#define S32K1XX_FTM1_OUTINIT             (S32K1XX_FTM1_BASE + S32K1XX_FTM_OUTINIT_OFFSET)  /* FTM1 Initial State For Channels Output register */
#define S32K1XX_FTM1_OUTMASK             (S32K1XX_FTM1_BASE + S32K1XX_FTM_OUTMASK_OFFSET)  /* FTM1 Output Mask register */
#define S32K1XX_FTM1_COMBINE             (S32K1XX_FTM1_BASE + S32K1XX_FTM_COMBINE_OFFSET)  /* FTM1 Function For Linked Channels register */
#define S32K1XX_FTM1_DEADTIME            (S32K1XX_FTM1_BASE + S32K1XX_FTM_DEADTIME_OFFSET) /* FTM1 Deadtime Configuration register */
#define S32K1XX_FTM1_EXTTRIG             (S32K1XX_FTM1_BASE + S32K1XX_FTM_EXTTRIG_OFFSET)  /* FTM1 External Trigger register */
#define S32K1XX_FTM1_POL                 (S32K1XX_FTM1_BASE + S32K1XX_FTM_POL_OFFSET)      /* FTM1 Channel Polarity register */
#define S32K1XX_FTM1_FMS                 (S32K1XX_FTM1_BASE + S32K1XX_FTM_FMS_OFFSET)      /* FTM1 Fault Mode Status register */
#define S32K1XX_FTM1_FILTER              (S32K1XX_FTM1_BASE + S32K1XX_FTM_FILTER_OFFSET)   /* FTM1 Input Capture Filter Control register */
#define S32K1XX_FTM1_FLTCTRL             (S32K1XX_FTM1_BASE + S32K1XX_FTM_FILTCTRL_OFFSET) /* FTM1 Fault Control register */
#define S32K1XX_FTM1_QDCTRL              (S32K1XX_FTM1_BASE + S32K1XX_FTM_QDCTRL_OFFSET)   /* FTM1 Quadrature Decoder Control And Status register */
#define S32K1XX_FTM1_CONF                (S32K1XX_FTM1_BASE + S32K1XX_FTM_CONF_OFFSET)     /* FTM1 Configuration register */
#define S32K1XX_FTM1_FLTPOL              (S32K1XX_FTM1_BASE + S32K1XX_FTM_FLTPOL_OFFSET)   /* FTM1 Fault Input Polarity register */
#define S32K1XX_FTM1_SYNCONF             (S32K1XX_FTM1_BASE + S32K1XX_FTM_SYNCONF_OFFSET)  /* FTM1 Synchronization Configuration register */
#define S32K1XX_FTM1_INVCTRL             (S32K1XX_FTM1_BASE + S32K1XX_FTM_INVCTRL_OFFSET)  /* FTM1 Inverting Control register */
#define S32K1XX_FTM1_SWOCTRL             (S32K1XX_FTM1_BASE + S32K1XX_FTM_SWOCTRL_OFFSET)  /* FTM1 Software Output Control register */
#define S32K1XX_FTM1_PWMLOAD             (S32K1XX_FTM1_BASE + S32K1XX_FTM_PWMLOAD_OFFSET)  /* FTM1 PWM Load register */
#define S32K1XX_FTM1_HCR                 (S32K1XX_FTM1_BASE + S32K1XX_FTM_HCR_OFFSET)      /* FTM1 Half Cycle Register */

#define S32K1XX_FTM1_PAIR0DEADTIME       (S32K1XX_FTM1_BASE + S32K1XX_FTM_PAIR0DEADTIME_OFFSET) /* FTM1 Pair 0 Deadtime Configuration register */
#define S32K1XX_FTM1_PAIR1DEADTIME       (S32K1XX_FTM1_BASE + S32K1XX_FTM_PAIR1DEADTIME_OFFSET) /* FTM1 Pair 1 Deadtime Configuration register */
#define S32K1XX_FTM1_PAIR2DEADTIME       (S32K1XX_FTM1_BASE + S32K1XX_FTM_PAIR2DEADTIME_OFFSET) /* FTM1 Pair 2 Deadtime Configuration register */
#define S32K1XX_FTM1_PAIR3DEADTIME       (S32K1XX_FTM1_BASE + S32K1XX_FTM_PAIR3DEADTIME_OFFSET) /* FTM1 Pair 3 Deadtime Configuration register oset */

#define S32K1XX_FTM1_MOD_MIRROR          (S32K1XX_FTM1_BASE + S32K1XX_FTM_MOD_MIRROR_OFFSET) /* FTM1 Mirror of Modulo Value register */
#define S32K1XX_FTM1_C0V_MIRROR          (S32K1XX_FTM1_BASE + S32K1XX_FTM_C0V_MIRROR_OFFSET) /* FTM1 Mirror of Channel 0 Match Value register */
#define S32K1XX_FTM1_C1V_MIRROR          (S32K1XX_FTM1_BASE + S32K1XX_FTM_C1V_MIRROR_OFFSET) /* FTM1 Mirror of Channel 1 Match Value register */
#define S32K1XX_FTM1_C2V_MIRROR          (S32K1XX_FTM1_BASE + S32K1XX_FTM_C2V_MIRROR_OFFSET) /* FTM1 Mirror of Channel 2 Match Value register */
#define S32K1XX_FTM1_C3V_MIRROR          (S32K1XX_FTM1_BASE + S32K1XX_FTM_C3V_MIRROR_OFFSET) /* FTM1 Mirror of Channel 3 Match Value register */
#define S32K1XX_FTM1_C4V_MIRROR          (S32K1XX_FTM1_BASE + S32K1XX_FTM_C4V_MIRROR_OFFSET) /* FTM1 Mirror of Channel 4 Match Value register */
#define S32K1XX_FTM1_C5V_MIRROR          (S32K1XX_FTM1_BASE + S32K1XX_FTM_C5V_MIRROR_OFFSET) /* FTM1 Mirror of Channel 5 Match Value register */
#define S32K1XX_FTM1_C6V_MIRROR          (S32K1XX_FTM1_BASE + S32K1XX_FTM_C6V_MIRROR_OFFSET) /* FTM1 Mirror of Channel 6 Match Value register */
#define S32K1XX_FTM1_C7V_MIRROR          (S32K1XX_FTM1_BASE + S32K1XX_FTM_C7V_MIRROR_OFFSET) /* FTM1 Mirror of Channel 7 Match Value register */

/* FTM2 registers */

#define S32K1XX_FTM2_SC                  (S32K1XX_FTM2_BASE + S32K1XX_FTM_SC_OFFSET)       /* FTM2 Status And Control register */
#define S32K1XX_FTM2_CNT                 (S32K1XX_FTM2_BASE + S32K1XX_FTM_CNT_OFFSET)      /* FTM2 Counter register */
#define S32K1XX_FTM2_MOD                 (S32K1XX_FTM2_BASE + S32K1XX_FTM_MOD_OFFSET)      /* FTM2 Modulo register */
#define S32K1XX_FTM2_C0SC                (S32K1XX_FTM2_BASE + S32K1XX_FTM_C0SC_OFFSET)     /* FTM2 Channel 0 Status And Control register */
#define S32K1XX_FTM2_C0V                 (S32K1XX_FTM2_BASE + S32K1XX_FTM_C0V_OFFSET)      /* FTM2 Channel 0 Value register */
#define S32K1XX_FTM2_C1SC                (S32K1XX_FTM2_BASE + S32K1XX_FTM_C1SC_OFFSET)     /* FTM2 Channel 1 Status And Control register */
#define S32K1XX_FTM2_C1V                 (S32K1XX_FTM2_BASE + S32K1XX_FTM_C1V_OFFSET)      /* FTM2 Channel 1 Value register */
#define S32K1XX_FTM2_C2SC                (S32K1XX_FTM2_BASE + S32K1XX_FTM_C2SC_OFFSET)     /* FTM2 Channel 2 Status And Control register */
#define S32K1XX_FTM2_C2V                 (S32K1XX_FTM2_BASE + S32K1XX_FTM_C2V_OFFSET)      /* FTM2 Channel 2 Value register */
#define S32K1XX_FTM2_C3SC                (S32K1XX_FTM2_BASE + S32K1XX_FTM_C3SC_OFFSET)     /* FTM2 Channel 3 Status And Control register */
#define S32K1XX_FTM2_C3V                 (S32K1XX_FTM2_BASE + S32K1XX_FTM_C3V_OFFSET)      /* FTM2 Channel 3 Value register */
#define S32K1XX_FTM2_C4SC                (S32K1XX_FTM2_BASE + S32K1XX_FTM_C4SC_OFFSET)     /* FTM2 Channel 4 Status And Control register */
#define S32K1XX_FTM2_C4V                 (S32K1XX_FTM2_BASE + S32K1XX_FTM_C4V_OFFSET)      /* FTM2 Channel 4 Value register */
#define S32K1XX_FTM2_C5SC                (S32K1XX_FTM2_BASE + S32K1XX_FTM_C5SC_OFFSET)     /* FTM2 Channel 5 Status And Control register */
#define S32K1XX_FTM2_C5V                 (S32K1XX_FTM2_BASE + S32K1XX_FTM_C5V_OFFSET)      /* FTM2 Channel 5 Value register */
#define S32K1XX_FTM2_C6SC                (S32K1XX_FTM2_BASE + S32K1XX_FTM_C6SC_OFFSET)     /* FTM2 Channel 6 Status And Control register */
#define S32K1XX_FTM2_C6V                 (S32K1XX_FTM2_BASE + S32K1XX_FTM_C6V_OFFSET)      /* FTM2 Channel 6 Value register */
#define S32K1XX_FTM2_C7SC                (S32K1XX_FTM2_BASE + S32K1XX_FTM_C7SC_OFFSET)     /* FTM2 Channel 7 Status And Control register */
#define S32K1XX_FTM2_C7V                 (S32K1XX_FTM2_BASE + S32K1XX_FTM_C7V_OFFSET)      /* FTM2 Channel 7 Value register */
#define S32K1XX_FTM2_CNTIN               (S32K1XX_FTM2_BASE + S32K1XX_FTM_CNTIN_OFFSET)    /* FTM2 Counter Initial Value register */
#define S32K1XX_FTM2_STATUS              (S32K1XX_FTM2_BASE + S32K1XX_FTM_STATUS_OFFSET)   /* FTM2 Capture And Compare Status register */
#define S32K1XX_FTM2_MODE                (S32K1XX_FTM2_BASE + S32K1XX_FTM_MODE_OFFSET)     /* FTM2 Features Mode Selection register */
#define S32K1XX_FTM2_SYNC                (S32K1XX_FTM2_BASE + S32K1XX_FTM_SYNC_OFFSET)     /* FTM2 Synchronization register */
#define S32K1XX_FTM2_OUTINIT             (S32K1XX_FTM2_BASE + S32K1XX_FTM_OUTINIT_OFFSET)  /* FTM2 Initial State For Channels Output register */
#define S32K1XX_FTM2_OUTMASK             (S32K1XX_FTM2_BASE + S32K1XX_FTM_OUTMASK_OFFSET)  /* FTM2 Output Mask register */
#define S32K1XX_FTM2_COMBINE             (S32K1XX_FTM2_BASE + S32K1XX_FTM_COMBINE_OFFSET)  /* FTM2 Function For Linked Channels register */
#define S32K1XX_FTM2_DEADTIME            (S32K1XX_FTM2_BASE + S32K1XX_FTM_DEADTIME_OFFSET) /* FTM2 Deadtime Configuration register */
#define S32K1XX_FTM2_EXTTRIG             (S32K1XX_FTM2_BASE + S32K1XX_FTM_EXTTRIG_OFFSET)  /* FTM2 External Trigger register */
#define S32K1XX_FTM2_POL                 (S32K1XX_FTM2_BASE + S32K1XX_FTM_POL_OFFSET)      /* FTM2 Channel Polarity register */
#define S32K1XX_FTM2_FMS                 (S32K1XX_FTM2_BASE + S32K1XX_FTM_FMS_OFFSET)      /* FTM2 Fault Mode Status register */
#define S32K1XX_FTM2_FILTER              (S32K1XX_FTM2_BASE + S32K1XX_FTM_FILTER_OFFSET)   /* FTM2 Input Capture Filter Control register */
#define S32K1XX_FTM2_FLTCTRL             (S32K1XX_FTM2_BASE + S32K1XX_FTM_FILTCTRL_OFFSET) /* FTM2 Fault Control register */
#define S32K1XX_FTM2_QDCTRL              (S32K1XX_FTM2_BASE + S32K1XX_FTM_QDCTRL_OFFSET)   /* FTM2 Quadrature Decoder Control And Status register */
#define S32K1XX_FTM2_CONF                (S32K1XX_FTM2_BASE + S32K1XX_FTM_CONF_OFFSET)     /* FTM2 Configuration register */
#define S32K1XX_FTM2_FLTPOL              (S32K1XX_FTM2_BASE + S32K1XX_FTM_FLTPOL_OFFSET)   /* FTM2 Fault Input Polarity register */
#define S32K1XX_FTM2_SYNCONF             (S32K1XX_FTM2_BASE + S32K1XX_FTM_SYNCONF_OFFSET)  /* FTM2 Synchronization Configuration register */
#define S32K1XX_FTM2_INVCTRL             (S32K1XX_FTM2_BASE + S32K1XX_FTM_INVCTRL_OFFSET)  /* FTM2 Inverting Control register */
#define S32K1XX_FTM2_SWOCTRL             (S32K1XX_FTM2_BASE + S32K1XX_FTM_SWOCTRL_OFFSET)  /* FTM2 Software Output Control register */
#define S32K1XX_FTM2_PWMLOAD             (S32K1XX_FTM2_BASE + S32K1XX_FTM_PWMLOAD_OFFSET)  /* FTM2 PWM Load register */
#define S32K1XX_FTM2_HCR                 (S32K1XX_FTM2_BASE + S32K1XX_FTM_HCR_OFFSET)      /* FTM2 Half Cycle Register */

#define S32K1XX_FTM2_PAIR0DEADTIME       (S32K1XX_FTM2_BASE + S32K1XX_FTM_PAIR0DEADTIME_OFFSET) /* FTM2 Pair 0 Deadtime Configuration register */
#define S32K1XX_FTM2_PAIR1DEADTIME       (S32K1XX_FTM2_BASE + S32K1XX_FTM_PAIR1DEADTIME_OFFSET) /* FTM2 Pair 1 Deadtime Configuration register */
#define S32K1XX_FTM2_PAIR2DEADTIME       (S32K1XX_FTM2_BASE + S32K1XX_FTM_PAIR2DEADTIME_OFFSET) /* FTM2 Pair 2 Deadtime Configuration register */
#define S32K1XX_FTM2_PAIR3DEADTIME       (S32K1XX_FTM2_BASE + S32K1XX_FTM_PAIR3DEADTIME_OFFSET) /* FTM2 Pair 3 Deadtime Configuration register oset */

#define S32K1XX_FTM2_MOD_MIRROR          (S32K1XX_FTM2_BASE + S32K1XX_FTM_MOD_MIRROR_OFFSET) /* FTM2 Mirror of Modulo Value register */
#define S32K1XX_FTM2_C0V_MIRROR          (S32K1XX_FTM2_BASE + S32K1XX_FTM_C0V_MIRROR_OFFSET) /* FTM2 Mirror of Channel 0 Match Value register */
#define S32K1XX_FTM2_C1V_MIRROR          (S32K1XX_FTM2_BASE + S32K1XX_FTM_C1V_MIRROR_OFFSET) /* FTM2 Mirror of Channel 1 Match Value register */
#define S32K1XX_FTM2_C2V_MIRROR          (S32K1XX_FTM2_BASE + S32K1XX_FTM_C2V_MIRROR_OFFSET) /* FTM2 Mirror of Channel 2 Match Value register */
#define S32K1XX_FTM2_C3V_MIRROR          (S32K1XX_FTM2_BASE + S32K1XX_FTM_C3V_MIRROR_OFFSET) /* FTM2 Mirror of Channel 3 Match Value register */
#define S32K1XX_FTM2_C4V_MIRROR          (S32K1XX_FTM2_BASE + S32K1XX_FTM_C4V_MIRROR_OFFSET) /* FTM2 Mirror of Channel 4 Match Value register */
#define S32K1XX_FTM2_C5V_MIRROR          (S32K1XX_FTM2_BASE + S32K1XX_FTM_C5V_MIRROR_OFFSET) /* FTM2 Mirror of Channel 5 Match Value register */
#define S32K1XX_FTM2_C6V_MIRROR          (S32K1XX_FTM2_BASE + S32K1XX_FTM_C6V_MIRROR_OFFSET) /* FTM2 Mirror of Channel 6 Match Value register */
#define S32K1XX_FTM2_C7V_MIRROR          (S32K1XX_FTM2_BASE + S32K1XX_FTM_C7V_MIRROR_OFFSET) /* FTM2 Mirror of Channel 7 Match Value register */

/* FTM3 registers */

#define S32K1XX_FTM3_SC                  (S32K1XX_FTM3_BASE + S32K1XX_FTM_SC_OFFSET)       /* FTM3 Status And Control register */
#define S32K1XX_FTM3_CNT                 (S32K1XX_FTM3_BASE + S32K1XX_FTM_CNT_OFFSET)      /* FTM3 Counter register */
#define S32K1XX_FTM3_MOD                 (S32K1XX_FTM3_BASE + S32K1XX_FTM_MOD_OFFSET)      /* FTM3 Modulo register */
#define S32K1XX_FTM3_C0SC                (S32K1XX_FTM3_BASE + S32K1XX_FTM_C0SC_OFFSET)     /* FTM3 Channel 0 Status And Control register */
#define S32K1XX_FTM3_C0V                 (S32K1XX_FTM3_BASE + S32K1XX_FTM_C0V_OFFSET)      /* FTM3 Channel 0 Value register */
#define S32K1XX_FTM3_C1SC                (S32K1XX_FTM3_BASE + S32K1XX_FTM_C1SC_OFFSET)     /* FTM3 Channel 1 Status And Control register */
#define S32K1XX_FTM3_C1V                 (S32K1XX_FTM3_BASE + S32K1XX_FTM_C1V_OFFSET)      /* FTM3 Channel 1 Value register */
#define S32K1XX_FTM3_C2SC                (S32K1XX_FTM3_BASE + S32K1XX_FTM_C2SC_OFFSET)     /* FTM3 Channel 2 Status And Control register */
#define S32K1XX_FTM3_C2V                 (S32K1XX_FTM3_BASE + S32K1XX_FTM_C2V_OFFSET)      /* FTM3 Channel 2 Value register */
#define S32K1XX_FTM3_C3SC                (S32K1XX_FTM3_BASE + S32K1XX_FTM_C3SC_OFFSET)     /* FTM3 Channel 3 Status And Control register */
#define S32K1XX_FTM3_C3V                 (S32K1XX_FTM3_BASE + S32K1XX_FTM_C3V_OFFSET)      /* FTM3 Channel 3 Value register */
#define S32K1XX_FTM3_C4SC                (S32K1XX_FTM3_BASE + S32K1XX_FTM_C4SC_OFFSET)     /* FTM3 Channel 4 Status And Control register */
#define S32K1XX_FTM3_C4V                 (S32K1XX_FTM3_BASE + S32K1XX_FTM_C4V_OFFSET)      /* FTM3 Channel 4 Value register */
#define S32K1XX_FTM3_C5SC                (S32K1XX_FTM3_BASE + S32K1XX_FTM_C5SC_OFFSET)     /* FTM3 Channel 5 Status And Control register */
#define S32K1XX_FTM3_C5V                 (S32K1XX_FTM3_BASE + S32K1XX_FTM_C5V_OFFSET)      /* FTM3 Channel 5 Value register */
#define S32K1XX_FTM3_C6SC                (S32K1XX_FTM3_BASE + S32K1XX_FTM_C6SC_OFFSET)     /* FTM3 Channel 6 Status And Control register */
#define S32K1XX_FTM3_C6V                 (S32K1XX_FTM3_BASE + S32K1XX_FTM_C6V_OFFSET)      /* FTM3 Channel 6 Value register */
#define S32K1XX_FTM3_C7SC                (S32K1XX_FTM3_BASE + S32K1XX_FTM_C7SC_OFFSET)     /* FTM3 Channel 7 Status And Control register */
#define S32K1XX_FTM3_C7V                 (S32K1XX_FTM3_BASE + S32K1XX_FTM_C7V_OFFSET)      /* FTM3 Channel 7 Value register */
#define S32K1XX_FTM3_CNTIN               (S32K1XX_FTM3_BASE + S32K1XX_FTM_CNTIN_OFFSET)    /* FTM3 Counter Initial Value register */
#define S32K1XX_FTM3_STATUS              (S32K1XX_FTM3_BASE + S32K1XX_FTM_STATUS_OFFSET)   /* FTM3 Capture And Compare Status register */
#define S32K1XX_FTM3_MODE                (S32K1XX_FTM3_BASE + S32K1XX_FTM_MODE_OFFSET)     /* FTM3 Features Mode Selection register */
#define S32K1XX_FTM3_SYNC                (S32K1XX_FTM3_BASE + S32K1XX_FTM_SYNC_OFFSET)     /* FTM3 Synchronization register */
#define S32K1XX_FTM3_OUTINIT             (S32K1XX_FTM3_BASE + S32K1XX_FTM_OUTINIT_OFFSET)  /* FTM3 Initial State For Channels Output register */
#define S32K1XX_FTM3_OUTMASK             (S32K1XX_FTM3_BASE + S32K1XX_FTM_OUTMASK_OFFSET)  /* FTM3 Output Mask register */
#define S32K1XX_FTM3_COMBINE             (S32K1XX_FTM3_BASE + S32K1XX_FTM_COMBINE_OFFSET)  /* FTM3 Function For Linked Channels register */
#define S32K1XX_FTM3_DEADTIME            (S32K1XX_FTM3_BASE + S32K1XX_FTM_DEADTIME_OFFSET) /* FTM3 Deadtime Configuration register */
#define S32K1XX_FTM3_EXTTRIG             (S32K1XX_FTM3_BASE + S32K1XX_FTM_EXTTRIG_OFFSET)  /* FTM3 External Trigger register */
#define S32K1XX_FTM3_POL                 (S32K1XX_FTM3_BASE + S32K1XX_FTM_POL_OFFSET)      /* FTM3 Channel Polarity register */
#define S32K1XX_FTM3_FMS                 (S32K1XX_FTM3_BASE + S32K1XX_FTM_FMS_OFFSET)      /* FTM3 Fault Mode Status register */
#define S32K1XX_FTM3_FILTER              (S32K1XX_FTM3_BASE + S32K1XX_FTM_FILTER_OFFSET)   /* FTM3 Input Capture Filter Control register */
#define S32K1XX_FTM3_FLTCTRL             (S32K1XX_FTM3_BASE + S32K1XX_FTM_FILTCTRL_OFFSET) /* FTM3 Fault Control register */
#define S32K1XX_FTM3_QDCTRL              (S32K1XX_FTM3_BASE + S32K1XX_FTM_QDCTRL_OFFSET)   /* FTM3 Quadrature Decoder Control And Status register */
#define S32K1XX_FTM3_CONF                (S32K1XX_FTM3_BASE + S32K1XX_FTM_CONF_OFFSET)     /* FTM3 Configuration register */
#define S32K1XX_FTM3_FLTPOL              (S32K1XX_FTM3_BASE + S32K1XX_FTM_FLTPOL_OFFSET)   /* FTM3 Fault Input Polarity register */
#define S32K1XX_FTM3_SYNCONF             (S32K1XX_FTM3_BASE + S32K1XX_FTM_SYNCONF_OFFSET)  /* FTM3 Synchronization Configuration register */
#define S32K1XX_FTM3_INVCTRL             (S32K1XX_FTM3_BASE + S32K1XX_FTM_INVCTRL_OFFSET)  /* FTM3 Inverting Control register */
#define S32K1XX_FTM3_SWOCTRL             (S32K1XX_FTM3_BASE + S32K1XX_FTM_SWOCTRL_OFFSET)  /* FTM3 Software Output Control register */
#define S32K1XX_FTM3_PWMLOAD             (S32K1XX_FTM3_BASE + S32K1XX_FTM_PWMLOAD_OFFSET)  /* FTM3 PWM Load register */
#define S32K1XX_FTM3_HCR                 (S32K1XX_FTM3_BASE + S32K1XX_FTM_HCR_OFFSET)      /* FTM3 Half Cycle Register */

#define S32K1XX_FTM3_PAIR0DEADTIME       (S32K1XX_FTM3_BASE + S32K1XX_FTM_PAIR0DEADTIME_OFFSET) /* FTM3 Pair 0 Deadtime Configuration register */
#define S32K1XX_FTM3_PAIR1DEADTIME       (S32K1XX_FTM3_BASE + S32K1XX_FTM_PAIR1DEADTIME_OFFSET) /* FTM3 Pair 1 Deadtime Configuration register */
#define S32K1XX_FTM3_PAIR2DEADTIME       (S32K1XX_FTM3_BASE + S32K1XX_FTM_PAIR2DEADTIME_OFFSET) /* FTM3 Pair 2 Deadtime Configuration register */
#define S32K1XX_FTM3_PAIR3DEADTIME       (S32K1XX_FTM3_BASE + S32K1XX_FTM_PAIR3DEADTIME_OFFSET) /* FTM3 Pair 3 Deadtime Configuration register oset */

#define S32K1XX_FTM3_MOD_MIRROR          (S32K1XX_FTM3_BASE + S32K1XX_FTM_MOD_MIRROR_OFFSET) /* FTM3 Mirror of Modulo Value register */
#define S32K1XX_FTM3_C0V_MIRROR          (S32K1XX_FTM3_BASE + S32K1XX_FTM_C0V_MIRROR_OFFSET) /* FTM3 Mirror of Channel 0 Match Value register */
#define S32K1XX_FTM3_C1V_MIRROR          (S32K1XX_FTM3_BASE + S32K1XX_FTM_C1V_MIRROR_OFFSET) /* FTM3 Mirror of Channel 1 Match Value register */
#define S32K1XX_FTM3_C2V_MIRROR          (S32K1XX_FTM3_BASE + S32K1XX_FTM_C2V_MIRROR_OFFSET) /* FTM3 Mirror of Channel 2 Match Value register */
#define S32K1XX_FTM3_C3V_MIRROR          (S32K1XX_FTM3_BASE + S32K1XX_FTM_C3V_MIRROR_OFFSET) /* FTM3 Mirror of Channel 3 Match Value register */
#define S32K1XX_FTM3_C4V_MIRROR          (S32K1XX_FTM3_BASE + S32K1XX_FTM_C4V_MIRROR_OFFSET) /* FTM3 Mirror of Channel 4 Match Value register */
#define S32K1XX_FTM3_C5V_MIRROR          (S32K1XX_FTM3_BASE + S32K1XX_FTM_C5V_MIRROR_OFFSET) /* FTM3 Mirror of Channel 5 Match Value register */
#define S32K1XX_FTM3_C6V_MIRROR          (S32K1XX_FTM3_BASE + S32K1XX_FTM_C6V_MIRROR_OFFSET) /* FTM3 Mirror of Channel 6 Match Value register */
#define S32K1XX_FTM3_C7V_MIRROR          (S32K1XX_FTM3_BASE + S32K1XX_FTM_C7V_MIRROR_OFFSET) /* FTM3 Mirror of Channel 7 Match Value register */

/* FTM4 registers */

#define S32K1XX_FTM4_SC                  (S32K1XX_FTM4_BASE + S32K1XX_FTM_SC_OFFSET)       /* FTM4 Status And Control register */
#define S32K1XX_FTM4_CNT                 (S32K1XX_FTM4_BASE + S32K1XX_FTM_CNT_OFFSET)      /* FTM4 Counter register */
#define S32K1XX_FTM4_MOD                 (S32K1XX_FTM4_BASE + S32K1XX_FTM_MOD_OFFSET)      /* FTM4 Modulo register */
#define S32K1XX_FTM4_C0SC                (S32K1XX_FTM4_BASE + S32K1XX_FTM_C0SC_OFFSET)     /* FTM4 Channel 0 Status And Control register */
#define S32K1XX_FTM4_C0V                 (S32K1XX_FTM4_BASE + S32K1XX_FTM_C0V_OFFSET)      /* FTM4 Channel 0 Value register */
#define S32K1XX_FTM4_C1SC                (S32K1XX_FTM4_BASE + S32K1XX_FTM_C1SC_OFFSET)     /* FTM4 Channel 1 Status And Control register */
#define S32K1XX_FTM4_C1V                 (S32K1XX_FTM4_BASE + S32K1XX_FTM_C1V_OFFSET)      /* FTM4 Channel 1 Value register */
#define S32K1XX_FTM4_C2SC                (S32K1XX_FTM4_BASE + S32K1XX_FTM_C2SC_OFFSET)     /* FTM4 Channel 2 Status And Control register */
#define S32K1XX_FTM4_C2V                 (S32K1XX_FTM4_BASE + S32K1XX_FTM_C2V_OFFSET)      /* FTM4 Channel 2 Value register */
#define S32K1XX_FTM4_C3SC                (S32K1XX_FTM4_BASE + S32K1XX_FTM_C3SC_OFFSET)     /* FTM4 Channel 3 Status And Control register */
#define S32K1XX_FTM4_C3V                 (S32K1XX_FTM4_BASE + S32K1XX_FTM_C3V_OFFSET)      /* FTM4 Channel 3 Value register */
#define S32K1XX_FTM4_C4SC                (S32K1XX_FTM4_BASE + S32K1XX_FTM_C4SC_OFFSET)     /* FTM4 Channel 4 Status And Control register */
#define S32K1XX_FTM4_C4V                 (S32K1XX_FTM4_BASE + S32K1XX_FTM_C4V_OFFSET)      /* FTM4 Channel 4 Value register */
#define S32K1XX_FTM4_C5SC                (S32K1XX_FTM4_BASE + S32K1XX_FTM_C5SC_OFFSET)     /* FTM4 Channel 5 Status And Control register */
#define S32K1XX_FTM4_C5V                 (S32K1XX_FTM4_BASE + S32K1XX_FTM_C5V_OFFSET)      /* FTM4 Channel 5 Value register */
#define S32K1XX_FTM4_C6SC                (S32K1XX_FTM4_BASE + S32K1XX_FTM_C6SC_OFFSET)     /* FTM4 Channel 6 Status And Control register */
#define S32K1XX_FTM4_C6V                 (S32K1XX_FTM4_BASE + S32K1XX_FTM_C6V_OFFSET)      /* FTM4 Channel 6 Value register */
#define S32K1XX_FTM4_C7SC                (S32K1XX_FTM4_BASE + S32K1XX_FTM_C7SC_OFFSET)     /* FTM4 Channel 7 Status And Control register */
#define S32K1XX_FTM4_C7V                 (S32K1XX_FTM4_BASE + S32K1XX_FTM_C7V_OFFSET)      /* FTM4 Channel 7 Value register */
#define S32K1XX_FTM4_CNTIN               (S32K1XX_FTM4_BASE + S32K1XX_FTM_CNTIN_OFFSET)    /* FTM4 Counter Initial Value register */
#define S32K1XX_FTM4_STATUS              (S32K1XX_FTM4_BASE + S32K1XX_FTM_STATUS_OFFSET)   /* FTM4 Capture And Compare Status register */
#define S32K1XX_FTM4_MODE                (S32K1XX_FTM4_BASE + S32K1XX_FTM_MODE_OFFSET)     /* FTM4 Features Mode Selection register */
#define S32K1XX_FTM4_SYNC                (S32K1XX_FTM4_BASE + S32K1XX_FTM_SYNC_OFFSET)     /* FTM4 Synchronization register */
#define S32K1XX_FTM4_OUTINIT             (S32K1XX_FTM4_BASE + S32K1XX_FTM_OUTINIT_OFFSET)  /* FTM4 Initial State For Channels Output register */
#define S32K1XX_FTM4_OUTMASK             (S32K1XX_FTM4_BASE + S32K1XX_FTM_OUTMASK_OFFSET)  /* FTM4 Output Mask register */
#define S32K1XX_FTM4_COMBINE             (S32K1XX_FTM4_BASE + S32K1XX_FTM_COMBINE_OFFSET)  /* FTM4 Function For Linked Channels register */
#define S32K1XX_FTM4_DEADTIME            (S32K1XX_FTM4_BASE + S32K1XX_FTM_DEADTIME_OFFSET) /* FTM4 Deadtime Configuration register */
#define S32K1XX_FTM4_EXTTRIG             (S32K1XX_FTM4_BASE + S32K1XX_FTM_EXTTRIG_OFFSET)  /* FTM4 External Trigger register */
#define S32K1XX_FTM4_POL                 (S32K1XX_FTM4_BASE + S32K1XX_FTM_POL_OFFSET)      /* FTM4 Channel Polarity register */
#define S32K1XX_FTM4_FMS                 (S32K1XX_FTM4_BASE + S32K1XX_FTM_FMS_OFFSET)      /* FTM4 Fault Mode Status register */
#define S32K1XX_FTM4_FILTER              (S32K1XX_FTM4_BASE + S32K1XX_FTM_FILTER_OFFSET)   /* FTM4 Input Capture Filter Control register */
#define S32K1XX_FTM4_FLTCTRL             (S32K1XX_FTM4_BASE + S32K1XX_FTM_FILTCTRL_OFFSET) /* FTM4 Fault Control register */
#define S32K1XX_FTM4_QDCTRL              (S32K1XX_FTM4_BASE + S32K1XX_FTM_QDCTRL_OFFSET)   /* FTM4 Quadrature Decoder Control And Status register */
#define S32K1XX_FTM4_CONF                (S32K1XX_FTM4_BASE + S32K1XX_FTM_CONF_OFFSET)     /* FTM4 Configuration register */
#define S32K1XX_FTM4_FLTPOL              (S32K1XX_FTM4_BASE + S32K1XX_FTM_FLTPOL_OFFSET)   /* FTM4 Fault Input Polarity register */
#define S32K1XX_FTM4_SYNCONF             (S32K1XX_FTM4_BASE + S32K1XX_FTM_SYNCONF_OFFSET)  /* FTM4 Synchronization Configuration register */
#define S32K1XX_FTM4_INVCTRL             (S32K1XX_FTM4_BASE + S32K1XX_FTM_INVCTRL_OFFSET)  /* FTM4 Inverting Control register */
#define S32K1XX_FTM4_SWOCTRL             (S32K1XX_FTM4_BASE + S32K1XX_FTM_SWOCTRL_OFFSET)  /* FTM4 Software Output Control register */
#define S32K1XX_FTM4_PWMLOAD             (S32K1XX_FTM4_BASE + S32K1XX_FTM_PWMLOAD_OFFSET)  /* FTM4 PWM Load register */
#define S32K1XX_FTM4_HCR                 (S32K1XX_FTM4_BASE + S32K1XX_FTM_HCR_OFFSET)      /* FTM4 Half Cycle Register */

#define S32K1XX_FTM4_PAIR0DEADTIME       (S32K1XX_FTM4_BASE + S32K1XX_FTM_PAIR0DEADTIME_OFFSET) /* FTM4 Pair 0 Deadtime Configuration register */
#define S32K1XX_FTM4_PAIR1DEADTIME       (S32K1XX_FTM4_BASE + S32K1XX_FTM_PAIR1DEADTIME_OFFSET) /* FTM4 Pair 1 Deadtime Configuration register */
#define S32K1XX_FTM4_PAIR2DEADTIME       (S32K1XX_FTM4_BASE + S32K1XX_FTM_PAIR2DEADTIME_OFFSET) /* FTM4 Pair 2 Deadtime Configuration register */
#define S32K1XX_FTM4_PAIR3DEADTIME       (S32K1XX_FTM4_BASE + S32K1XX_FTM_PAIR3DEADTIME_OFFSET) /* FTM4 Pair 3 Deadtime Configuration register oset */

#define S32K1XX_FTM4_MOD_MIRROR          (S32K1XX_FTM4_BASE + S32K1XX_FTM_MOD_MIRROR_OFFSET) /* FTM4 Mirror of Modulo Value register */
#define S32K1XX_FTM4_C0V_MIRROR          (S32K1XX_FTM4_BASE + S32K1XX_FTM_C0V_MIRROR_OFFSET) /* FTM4 Mirror of Channel 0 Match Value register */
#define S32K1XX_FTM4_C1V_MIRROR          (S32K1XX_FTM4_BASE + S32K1XX_FTM_C1V_MIRROR_OFFSET) /* FTM4 Mirror of Channel 1 Match Value register */
#define S32K1XX_FTM4_C2V_MIRROR          (S32K1XX_FTM4_BASE + S32K1XX_FTM_C2V_MIRROR_OFFSET) /* FTM4 Mirror of Channel 2 Match Value register */
#define S32K1XX_FTM4_C3V_MIRROR          (S32K1XX_FTM4_BASE + S32K1XX_FTM_C3V_MIRROR_OFFSET) /* FTM4 Mirror of Channel 3 Match Value register */
#define S32K1XX_FTM4_C4V_MIRROR          (S32K1XX_FTM4_BASE + S32K1XX_FTM_C4V_MIRROR_OFFSET) /* FTM4 Mirror of Channel 4 Match Value register */
#define S32K1XX_FTM4_C5V_MIRROR          (S32K1XX_FTM4_BASE + S32K1XX_FTM_C5V_MIRROR_OFFSET) /* FTM4 Mirror of Channel 5 Match Value register */
#define S32K1XX_FTM4_C6V_MIRROR          (S32K1XX_FTM4_BASE + S32K1XX_FTM_C6V_MIRROR_OFFSET) /* FTM4 Mirror of Channel 6 Match Value register */
#define S32K1XX_FTM4_C7V_MIRROR          (S32K1XX_FTM4_BASE + S32K1XX_FTM_C7V_MIRROR_OFFSET) /* FTM4 Mirror of Channel 7 Match Value register */

/* FTM5 registers */

#define S32K1XX_FTM5_SC                  (S32K1XX_FTM5_BASE + S32K1XX_FTM_SC_OFFSET)       /* FTM5 Status And Control register */
#define S32K1XX_FTM5_CNT                 (S32K1XX_FTM5_BASE + S32K1XX_FTM_CNT_OFFSET)      /* FTM5 Counter register */
#define S32K1XX_FTM5_MOD                 (S32K1XX_FTM5_BASE + S32K1XX_FTM_MOD_OFFSET)      /* FTM5 Modulo register */
#define S32K1XX_FTM5_C0SC                (S32K1XX_FTM5_BASE + S32K1XX_FTM_C0SC_OFFSET)     /* FTM5 Channel 0 Status And Control register */
#define S32K1XX_FTM5_C0V                 (S32K1XX_FTM5_BASE + S32K1XX_FTM_C0V_OFFSET)      /* FTM5 Channel 0 Value register */
#define S32K1XX_FTM5_C1SC                (S32K1XX_FTM5_BASE + S32K1XX_FTM_C1SC_OFFSET)     /* FTM5 Channel 1 Status And Control register */
#define S32K1XX_FTM5_C1V                 (S32K1XX_FTM5_BASE + S32K1XX_FTM_C1V_OFFSET)      /* FTM5 Channel 1 Value register */
#define S32K1XX_FTM5_C2SC                (S32K1XX_FTM5_BASE + S32K1XX_FTM_C2SC_OFFSET)     /* FTM5 Channel 2 Status And Control register */
#define S32K1XX_FTM5_C2V                 (S32K1XX_FTM5_BASE + S32K1XX_FTM_C2V_OFFSET)      /* FTM5 Channel 2 Value register */
#define S32K1XX_FTM5_C3SC                (S32K1XX_FTM5_BASE + S32K1XX_FTM_C3SC_OFFSET)     /* FTM5 Channel 3 Status And Control register */
#define S32K1XX_FTM5_C3V                 (S32K1XX_FTM5_BASE + S32K1XX_FTM_C3V_OFFSET)      /* FTM5 Channel 3 Value register */
#define S32K1XX_FTM5_C4SC                (S32K1XX_FTM5_BASE + S32K1XX_FTM_C4SC_OFFSET)     /* FTM5 Channel 4 Status And Control register */
#define S32K1XX_FTM5_C4V                 (S32K1XX_FTM5_BASE + S32K1XX_FTM_C4V_OFFSET)      /* FTM5 Channel 4 Value register */
#define S32K1XX_FTM5_C5SC                (S32K1XX_FTM5_BASE + S32K1XX_FTM_C5SC_OFFSET)     /* FTM5 Channel 5 Status And Control register */
#define S32K1XX_FTM5_C5V                 (S32K1XX_FTM5_BASE + S32K1XX_FTM_C5V_OFFSET)      /* FTM5 Channel 5 Value register */
#define S32K1XX_FTM5_C6SC                (S32K1XX_FTM5_BASE + S32K1XX_FTM_C6SC_OFFSET)     /* FTM5 Channel 6 Status And Control register */
#define S32K1XX_FTM5_C6V                 (S32K1XX_FTM5_BASE + S32K1XX_FTM_C6V_OFFSET)      /* FTM5 Channel 6 Value register */
#define S32K1XX_FTM5_C7SC                (S32K1XX_FTM5_BASE + S32K1XX_FTM_C7SC_OFFSET)     /* FTM5 Channel 7 Status And Control register */
#define S32K1XX_FTM5_C7V                 (S32K1XX_FTM5_BASE + S32K1XX_FTM_C7V_OFFSET)      /* FTM5 Channel 7 Value register */
#define S32K1XX_FTM5_CNTIN               (S32K1XX_FTM5_BASE + S32K1XX_FTM_CNTIN_OFFSET)    /* FTM5 Counter Initial Value register */
#define S32K1XX_FTM5_STATUS              (S32K1XX_FTM5_BASE + S32K1XX_FTM_STATUS_OFFSET)   /* FTM5 Capture And Compare Status register */
#define S32K1XX_FTM5_MODE                (S32K1XX_FTM5_BASE + S32K1XX_FTM_MODE_OFFSET)     /* FTM5 Features Mode Selection register */
#define S32K1XX_FTM5_SYNC                (S32K1XX_FTM5_BASE + S32K1XX_FTM_SYNC_OFFSET)     /* FTM5 Synchronization register */
#define S32K1XX_FTM5_OUTINIT             (S32K1XX_FTM5_BASE + S32K1XX_FTM_OUTINIT_OFFSET)  /* FTM5 Initial State For Channels Output register */
#define S32K1XX_FTM5_OUTMASK             (S32K1XX_FTM5_BASE + S32K1XX_FTM_OUTMASK_OFFSET)  /* FTM5 Output Mask register */
#define S32K1XX_FTM5_COMBINE             (S32K1XX_FTM5_BASE + S32K1XX_FTM_COMBINE_OFFSET)  /* FTM5 Function For Linked Channels register */
#define S32K1XX_FTM5_DEADTIME            (S32K1XX_FTM5_BASE + S32K1XX_FTM_DEADTIME_OFFSET) /* FTM5 Deadtime Configuration register */
#define S32K1XX_FTM5_EXTTRIG             (S32K1XX_FTM5_BASE + S32K1XX_FTM_EXTTRIG_OFFSET)  /* FTM5 External Trigger register */
#define S32K1XX_FTM5_POL                 (S32K1XX_FTM5_BASE + S32K1XX_FTM_POL_OFFSET)      /* FTM5 Channel Polarity register */
#define S32K1XX_FTM5_FMS                 (S32K1XX_FTM5_BASE + S32K1XX_FTM_FMS_OFFSET)      /* FTM5 Fault Mode Status register */
#define S32K1XX_FTM5_FILTER              (S32K1XX_FTM5_BASE + S32K1XX_FTM_FILTER_OFFSET)   /* FTM5 Input Capture Filter Control register */
#define S32K1XX_FTM5_FLTCTRL             (S32K1XX_FTM5_BASE + S32K1XX_FTM_FILTCTRL_OFFSET) /* FTM5 Fault Control register */
#define S32K1XX_FTM5_QDCTRL              (S32K1XX_FTM5_BASE + S32K1XX_FTM_QDCTRL_OFFSET)   /* FTM5 Quadrature Decoder Control And Status register */
#define S32K1XX_FTM5_CONF                (S32K1XX_FTM5_BASE + S32K1XX_FTM_CONF_OFFSET)     /* FTM5 Configuration register */
#define S32K1XX_FTM5_FLTPOL              (S32K1XX_FTM5_BASE + S32K1XX_FTM_FLTPOL_OFFSET)   /* FTM5 Fault Input Polarity register */
#define S32K1XX_FTM5_SYNCONF             (S32K1XX_FTM5_BASE + S32K1XX_FTM_SYNCONF_OFFSET)  /* FTM5 Synchronization Configuration register */
#define S32K1XX_FTM5_INVCTRL             (S32K1XX_FTM5_BASE + S32K1XX_FTM_INVCTRL_OFFSET)  /* FTM5 Inverting Control register */
#define S32K1XX_FTM5_SWOCTRL             (S32K1XX_FTM5_BASE + S32K1XX_FTM_SWOCTRL_OFFSET)  /* FTM5 Software Output Control register */
#define S32K1XX_FTM5_PWMLOAD             (S32K1XX_FTM5_BASE + S32K1XX_FTM_PWMLOAD_OFFSET)  /* FTM5 PWM Load register */
#define S32K1XX_FTM5_HCR                 (S32K1XX_FTM5_BASE + S32K1XX_FTM_HCR_OFFSET)      /* FTM5 Half Cycle Register */

#define S32K1XX_FTM5_PAIR0DEADTIME       (S32K1XX_FTM5_BASE + S32K1XX_FTM_PAIR0DEADTIME_OFFSET) /* FTM5 Pair 0 Deadtime Configuration register */
#define S32K1XX_FTM5_PAIR1DEADTIME       (S32K1XX_FTM5_BASE + S32K1XX_FTM_PAIR1DEADTIME_OFFSET) /* FTM5 Pair 1 Deadtime Configuration register */
#define S32K1XX_FTM5_PAIR2DEADTIME       (S32K1XX_FTM5_BASE + S32K1XX_FTM_PAIR2DEADTIME_OFFSET) /* FTM5 Pair 2 Deadtime Configuration register */
#define S32K1XX_FTM5_PAIR3DEADTIME       (S32K1XX_FTM5_BASE + S32K1XX_FTM_PAIR3DEADTIME_OFFSET) /* FTM5 Pair 3 Deadtime Configuration register oset */

#define S32K1XX_FTM5_MOD_MIRROR          (S32K1XX_FTM5_BASE + S32K1XX_FTM_MOD_MIRROR_OFFSET) /* FTM5 Mirror of Modulo Value register */
#define S32K1XX_FTM5_C0V_MIRROR          (S32K1XX_FTM5_BASE + S32K1XX_FTM_C0V_MIRROR_OFFSET) /* FTM5 Mirror of Channel 0 Match Value register */
#define S32K1XX_FTM5_C1V_MIRROR          (S32K1XX_FTM5_BASE + S32K1XX_FTM_C1V_MIRROR_OFFSET) /* FTM5 Mirror of Channel 1 Match Value register */
#define S32K1XX_FTM5_C2V_MIRROR          (S32K1XX_FTM5_BASE + S32K1XX_FTM_C2V_MIRROR_OFFSET) /* FTM5 Mirror of Channel 2 Match Value register */
#define S32K1XX_FTM5_C3V_MIRROR          (S32K1XX_FTM5_BASE + S32K1XX_FTM_C3V_MIRROR_OFFSET) /* FTM5 Mirror of Channel 3 Match Value register */
#define S32K1XX_FTM5_C4V_MIRROR          (S32K1XX_FTM5_BASE + S32K1XX_FTM_C4V_MIRROR_OFFSET) /* FTM5 Mirror of Channel 4 Match Value register */
#define S32K1XX_FTM5_C5V_MIRROR          (S32K1XX_FTM5_BASE + S32K1XX_FTM_C5V_MIRROR_OFFSET) /* FTM5 Mirror of Channel 5 Match Value register */
#define S32K1XX_FTM5_C6V_MIRROR          (S32K1XX_FTM5_BASE + S32K1XX_FTM_C6V_MIRROR_OFFSET) /* FTM5 Mirror of Channel 6 Match Value register */
#define S32K1XX_FTM5_C7V_MIRROR          (S32K1XX_FTM5_BASE + S32K1XX_FTM_C7V_MIRROR_OFFSET) /* FTM5 Mirror of Channel 7 Match Value register */

/* FTM6 registers */

#define S32K1XX_FTM6_SC                  (S32K1XX_FTM6_BASE + S32K1XX_FTM_SC_OFFSET)       /* FTM6 Status And Control register */
#define S32K1XX_FTM6_CNT                 (S32K1XX_FTM6_BASE + S32K1XX_FTM_CNT_OFFSET)      /* FTM6 Counter register */
#define S32K1XX_FTM6_MOD                 (S32K1XX_FTM6_BASE + S32K1XX_FTM_MOD_OFFSET)      /* FTM6 Modulo register */
#define S32K1XX_FTM6_C0SC                (S32K1XX_FTM6_BASE + S32K1XX_FTM_C0SC_OFFSET)     /* FTM6 Channel 0 Status And Control register */
#define S32K1XX_FTM6_C0V                 (S32K1XX_FTM6_BASE + S32K1XX_FTM_C0V_OFFSET)      /* FTM6 Channel 0 Value register */
#define S32K1XX_FTM6_C1SC                (S32K1XX_FTM6_BASE + S32K1XX_FTM_C1SC_OFFSET)     /* FTM6 Channel 1 Status And Control register */
#define S32K1XX_FTM6_C1V                 (S32K1XX_FTM6_BASE + S32K1XX_FTM_C1V_OFFSET)      /* FTM6 Channel 1 Value register */
#define S32K1XX_FTM6_C2SC                (S32K1XX_FTM6_BASE + S32K1XX_FTM_C2SC_OFFSET)     /* FTM6 Channel 2 Status And Control register */
#define S32K1XX_FTM6_C2V                 (S32K1XX_FTM6_BASE + S32K1XX_FTM_C2V_OFFSET)      /* FTM6 Channel 2 Value register */
#define S32K1XX_FTM6_C3SC                (S32K1XX_FTM6_BASE + S32K1XX_FTM_C3SC_OFFSET)     /* FTM6 Channel 3 Status And Control register */
#define S32K1XX_FTM6_C3V                 (S32K1XX_FTM6_BASE + S32K1XX_FTM_C3V_OFFSET)      /* FTM6 Channel 3 Value register */
#define S32K1XX_FTM6_C4SC                (S32K1XX_FTM6_BASE + S32K1XX_FTM_C4SC_OFFSET)     /* FTM6 Channel 4 Status And Control register */
#define S32K1XX_FTM6_C4V                 (S32K1XX_FTM6_BASE + S32K1XX_FTM_C4V_OFFSET)      /* FTM6 Channel 4 Value register */
#define S32K1XX_FTM6_C5SC                (S32K1XX_FTM6_BASE + S32K1XX_FTM_C5SC_OFFSET)     /* FTM6 Channel 5 Status And Control register */
#define S32K1XX_FTM6_C5V                 (S32K1XX_FTM6_BASE + S32K1XX_FTM_C5V_OFFSET)      /* FTM6 Channel 5 Value register */
#define S32K1XX_FTM6_C6SC                (S32K1XX_FTM6_BASE + S32K1XX_FTM_C6SC_OFFSET)     /* FTM6 Channel 6 Status And Control register */
#define S32K1XX_FTM6_C6V                 (S32K1XX_FTM6_BASE + S32K1XX_FTM_C6V_OFFSET)      /* FTM6 Channel 6 Value register */
#define S32K1XX_FTM6_C7SC                (S32K1XX_FTM6_BASE + S32K1XX_FTM_C7SC_OFFSET)     /* FTM6 Channel 7 Status And Control register */
#define S32K1XX_FTM6_C7V                 (S32K1XX_FTM6_BASE + S32K1XX_FTM_C7V_OFFSET)      /* FTM6 Channel 7 Value register */
#define S32K1XX_FTM6_CNTIN               (S32K1XX_FTM6_BASE + S32K1XX_FTM_CNTIN_OFFSET)    /* FTM6 Counter Initial Value register */
#define S32K1XX_FTM6_STATUS              (S32K1XX_FTM6_BASE + S32K1XX_FTM_STATUS_OFFSET)   /* FTM6 Capture And Compare Status register */
#define S32K1XX_FTM6_MODE                (S32K1XX_FTM6_BASE + S32K1XX_FTM_MODE_OFFSET)     /* FTM6 Features Mode Selection register */
#define S32K1XX_FTM6_SYNC                (S32K1XX_FTM6_BASE + S32K1XX_FTM_SYNC_OFFSET)     /* FTM6 Synchronization register */
#define S32K1XX_FTM6_OUTINIT             (S32K1XX_FTM6_BASE + S32K1XX_FTM_OUTINIT_OFFSET)  /* FTM6 Initial State For Channels Output register */
#define S32K1XX_FTM6_OUTMASK             (S32K1XX_FTM6_BASE + S32K1XX_FTM_OUTMASK_OFFSET)  /* FTM6 Output Mask register */
#define S32K1XX_FTM6_COMBINE             (S32K1XX_FTM6_BASE + S32K1XX_FTM_COMBINE_OFFSET)  /* FTM6 Function For Linked Channels register */
#define S32K1XX_FTM6_DEADTIME            (S32K1XX_FTM6_BASE + S32K1XX_FTM_DEADTIME_OFFSET) /* FTM6 Deadtime Configuration register */
#define S32K1XX_FTM6_EXTTRIG             (S32K1XX_FTM6_BASE + S32K1XX_FTM_EXTTRIG_OFFSET)  /* FTM6 External Trigger register */
#define S32K1XX_FTM6_POL                 (S32K1XX_FTM6_BASE + S32K1XX_FTM_POL_OFFSET)      /* FTM6 Channel Polarity register */
#define S32K1XX_FTM6_FMS                 (S32K1XX_FTM6_BASE + S32K1XX_FTM_FMS_OFFSET)      /* FTM6 Fault Mode Status register */
#define S32K1XX_FTM6_FILTER              (S32K1XX_FTM6_BASE + S32K1XX_FTM_FILTER_OFFSET)   /* FTM6 Input Capture Filter Control register */
#define S32K1XX_FTM6_FLTCTRL             (S32K1XX_FTM6_BASE + S32K1XX_FTM_FILTCTRL_OFFSET) /* FTM6 Fault Control register */
#define S32K1XX_FTM6_QDCTRL              (S32K1XX_FTM6_BASE + S32K1XX_FTM_QDCTRL_OFFSET)   /* FTM6 Quadrature Decoder Control And Status register */
#define S32K1XX_FTM6_CONF                (S32K1XX_FTM6_BASE + S32K1XX_FTM_CONF_OFFSET)     /* FTM6 Configuration register */
#define S32K1XX_FTM6_FLTPOL              (S32K1XX_FTM6_BASE + S32K1XX_FTM_FLTPOL_OFFSET)   /* FTM6 Fault Input Polarity register */
#define S32K1XX_FTM6_SYNCONF             (S32K1XX_FTM6_BASE + S32K1XX_FTM_SYNCONF_OFFSET)  /* FTM6 Synchronization Configuration register */
#define S32K1XX_FTM6_INVCTRL             (S32K1XX_FTM6_BASE + S32K1XX_FTM_INVCTRL_OFFSET)  /* FTM6 Inverting Control register */
#define S32K1XX_FTM6_SWOCTRL             (S32K1XX_FTM6_BASE + S32K1XX_FTM_SWOCTRL_OFFSET)  /* FTM6 Software Output Control register */
#define S32K1XX_FTM6_PWMLOAD             (S32K1XX_FTM6_BASE + S32K1XX_FTM_PWMLOAD_OFFSET)  /* FTM6 PWM Load register */
#define S32K1XX_FTM6_HCR                 (S32K1XX_FTM6_BASE + S32K1XX_FTM_HCR_OFFSET)      /* FTM6 Half Cycle Register */

#define S32K1XX_FTM6_PAIR0DEADTIME       (S32K1XX_FTM6_BASE + S32K1XX_FTM_PAIR0DEADTIME_OFFSET) /* FTM6 Pair 0 Deadtime Configuration register */
#define S32K1XX_FTM6_PAIR1DEADTIME       (S32K1XX_FTM6_BASE + S32K1XX_FTM_PAIR1DEADTIME_OFFSET) /* FTM6 Pair 1 Deadtime Configuration register */
#define S32K1XX_FTM6_PAIR2DEADTIME       (S32K1XX_FTM6_BASE + S32K1XX_FTM_PAIR2DEADTIME_OFFSET) /* FTM6 Pair 2 Deadtime Configuration register */
#define S32K1XX_FTM6_PAIR3DEADTIME       (S32K1XX_FTM6_BASE + S32K1XX_FTM_PAIR3DEADTIME_OFFSET) /* FTM6 Pair 3 Deadtime Configuration register oset */

#define S32K1XX_FTM6_MOD_MIRROR          (S32K1XX_FTM6_BASE + S32K1XX_FTM_MOD_MIRROR_OFFSET) /* FTM6 Mirror of Modulo Value register */
#define S32K1XX_FTM6_C0V_MIRROR          (S32K1XX_FTM6_BASE + S32K1XX_FTM_C0V_MIRROR_OFFSET) /* FTM6 Mirror of Channel 0 Match Value register */
#define S32K1XX_FTM6_C1V_MIRROR          (S32K1XX_FTM6_BASE + S32K1XX_FTM_C1V_MIRROR_OFFSET) /* FTM6 Mirror of Channel 1 Match Value register */
#define S32K1XX_FTM6_C2V_MIRROR          (S32K1XX_FTM6_BASE + S32K1XX_FTM_C2V_MIRROR_OFFSET) /* FTM6 Mirror of Channel 2 Match Value register */
#define S32K1XX_FTM6_C3V_MIRROR          (S32K1XX_FTM6_BASE + S32K1XX_FTM_C3V_MIRROR_OFFSET) /* FTM6 Mirror of Channel 3 Match Value register */
#define S32K1XX_FTM6_C4V_MIRROR          (S32K1XX_FTM6_BASE + S32K1XX_FTM_C4V_MIRROR_OFFSET) /* FTM6 Mirror of Channel 4 Match Value register */
#define S32K1XX_FTM6_C5V_MIRROR          (S32K1XX_FTM6_BASE + S32K1XX_FTM_C5V_MIRROR_OFFSET) /* FTM6 Mirror of Channel 5 Match Value register */
#define S32K1XX_FTM6_C6V_MIRROR          (S32K1XX_FTM6_BASE + S32K1XX_FTM_C6V_MIRROR_OFFSET) /* FTM6 Mirror of Channel 6 Match Value register */
#define S32K1XX_FTM6_C7V_MIRROR          (S32K1XX_FTM6_BASE + S32K1XX_FTM_C7V_MIRROR_OFFSET) /* FTM6 Mirror of Channel 7 Match Value register */

/* FTM7 registers */

#define S32K1XX_FTM7_SC                  (S32K1XX_FTM7_BASE + S32K1XX_FTM_SC_OFFSET)       /* FTM7 Status And Control register */
#define S32K1XX_FTM7_CNT                 (S32K1XX_FTM7_BASE + S32K1XX_FTM_CNT_OFFSET)      /* FTM7 Counter register */
#define S32K1XX_FTM7_MOD                 (S32K1XX_FTM7_BASE + S32K1XX_FTM_MOD_OFFSET)      /* FTM7 Modulo register */
#define S32K1XX_FTM7_C0SC                (S32K1XX_FTM7_BASE + S32K1XX_FTM_C0SC_OFFSET)     /* FTM7 Channel 0 Status And Control register */
#define S32K1XX_FTM7_C0V                 (S32K1XX_FTM7_BASE + S32K1XX_FTM_C0V_OFFSET)      /* FTM7 Channel 0 Value register */
#define S32K1XX_FTM7_C1SC                (S32K1XX_FTM7_BASE + S32K1XX_FTM_C1SC_OFFSET)     /* FTM7 Channel 1 Status And Control register */
#define S32K1XX_FTM7_C1V                 (S32K1XX_FTM7_BASE + S32K1XX_FTM_C1V_OFFSET)      /* FTM7 Channel 1 Value register */
#define S32K1XX_FTM7_C2SC                (S32K1XX_FTM7_BASE + S32K1XX_FTM_C2SC_OFFSET)     /* FTM7 Channel 2 Status And Control register */
#define S32K1XX_FTM7_C2V                 (S32K1XX_FTM7_BASE + S32K1XX_FTM_C2V_OFFSET)      /* FTM7 Channel 2 Value register */
#define S32K1XX_FTM7_C3SC                (S32K1XX_FTM7_BASE + S32K1XX_FTM_C3SC_OFFSET)     /* FTM7 Channel 3 Status And Control register */
#define S32K1XX_FTM7_C3V                 (S32K1XX_FTM7_BASE + S32K1XX_FTM_C3V_OFFSET)      /* FTM7 Channel 3 Value register */
#define S32K1XX_FTM7_C4SC                (S32K1XX_FTM7_BASE + S32K1XX_FTM_C4SC_OFFSET)     /* FTM7 Channel 4 Status And Control register */
#define S32K1XX_FTM7_C4V                 (S32K1XX_FTM7_BASE + S32K1XX_FTM_C4V_OFFSET)      /* FTM7 Channel 4 Value register */
#define S32K1XX_FTM7_C5SC                (S32K1XX_FTM7_BASE + S32K1XX_FTM_C5SC_OFFSET)     /* FTM7 Channel 5 Status And Control register */
#define S32K1XX_FTM7_C5V                 (S32K1XX_FTM7_BASE + S32K1XX_FTM_C5V_OFFSET)      /* FTM7 Channel 5 Value register */
#define S32K1XX_FTM7_C6SC                (S32K1XX_FTM7_BASE + S32K1XX_FTM_C6SC_OFFSET)     /* FTM7 Channel 6 Status And Control register */
#define S32K1XX_FTM7_C6V                 (S32K1XX_FTM7_BASE + S32K1XX_FTM_C6V_OFFSET)      /* FTM7 Channel 6 Value register */
#define S32K1XX_FTM7_C7SC                (S32K1XX_FTM7_BASE + S32K1XX_FTM_C7SC_OFFSET)     /* FTM7 Channel 7 Status And Control register */
#define S32K1XX_FTM7_C7V                 (S32K1XX_FTM7_BASE + S32K1XX_FTM_C7V_OFFSET)      /* FTM7 Channel 7 Value register */
#define S32K1XX_FTM7_CNTIN               (S32K1XX_FTM7_BASE + S32K1XX_FTM_CNTIN_OFFSET)    /* FTM7 Counter Initial Value register */
#define S32K1XX_FTM7_STATUS              (S32K1XX_FTM7_BASE + S32K1XX_FTM_STATUS_OFFSET)   /* FTM7 Capture And Compare Status register */
#define S32K1XX_FTM7_MODE                (S32K1XX_FTM7_BASE + S32K1XX_FTM_MODE_OFFSET)     /* FTM7 Features Mode Selection register */
#define S32K1XX_FTM7_SYNC                (S32K1XX_FTM7_BASE + S32K1XX_FTM_SYNC_OFFSET)     /* FTM7 Synchronization register */
#define S32K1XX_FTM7_OUTINIT             (S32K1XX_FTM7_BASE + S32K1XX_FTM_OUTINIT_OFFSET)  /* FTM7 Initial State For Channels Output register */
#define S32K1XX_FTM7_OUTMASK             (S32K1XX_FTM7_BASE + S32K1XX_FTM_OUTMASK_OFFSET)  /* FTM7 Output Mask register */
#define S32K1XX_FTM7_COMBINE             (S32K1XX_FTM7_BASE + S32K1XX_FTM_COMBINE_OFFSET)  /* FTM7 Function For Linked Channels register */
#define S32K1XX_FTM7_DEADTIME            (S32K1XX_FTM7_BASE + S32K1XX_FTM_DEADTIME_OFFSET) /* FTM7 Deadtime Configuration register */
#define S32K1XX_FTM7_EXTTRIG             (S32K1XX_FTM7_BASE + S32K1XX_FTM_EXTTRIG_OFFSET)  /* FTM7 External Trigger register */
#define S32K1XX_FTM7_POL                 (S32K1XX_FTM7_BASE + S32K1XX_FTM_POL_OFFSET)      /* FTM7 Channel Polarity register */
#define S32K1XX_FTM7_FMS                 (S32K1XX_FTM7_BASE + S32K1XX_FTM_FMS_OFFSET)      /* FTM7 Fault Mode Status register */
#define S32K1XX_FTM7_FILTER              (S32K1XX_FTM7_BASE + S32K1XX_FTM_FILTER_OFFSET)   /* FTM7 Input Capture Filter Control register */
#define S32K1XX_FTM7_FLTCTRL             (S32K1XX_FTM7_BASE + S32K1XX_FTM_FILTCTRL_OFFSET) /* FTM7 Fault Control register */
#define S32K1XX_FTM7_QDCTRL              (S32K1XX_FTM7_BASE + S32K1XX_FTM_QDCTRL_OFFSET)   /* FTM7 Quadrature Decoder Control And Status register */
#define S32K1XX_FTM7_CONF                (S32K1XX_FTM7_BASE + S32K1XX_FTM_CONF_OFFSET)     /* FTM7 Configuration register */
#define S32K1XX_FTM7_FLTPOL              (S32K1XX_FTM7_BASE + S32K1XX_FTM_FLTPOL_OFFSET)   /* FTM7 Fault Input Polarity register */
#define S32K1XX_FTM7_SYNCONF             (S32K1XX_FTM7_BASE + S32K1XX_FTM_SYNCONF_OFFSET)  /* FTM7 Synchronization Configuration register */
#define S32K1XX_FTM7_INVCTRL             (S32K1XX_FTM7_BASE + S32K1XX_FTM_INVCTRL_OFFSET)  /* FTM7 Inverting Control register */
#define S32K1XX_FTM7_SWOCTRL             (S32K1XX_FTM7_BASE + S32K1XX_FTM_SWOCTRL_OFFSET)  /* FTM7 Software Output Control register */
#define S32K1XX_FTM7_PWMLOAD             (S32K1XX_FTM7_BASE + S32K1XX_FTM_PWMLOAD_OFFSET)  /* FTM7 PWM Load register */
#define S32K1XX_FTM7_HCR                 (S32K1XX_FTM7_BASE + S32K1XX_FTM_HCR_OFFSET)      /* FTM7 Half Cycle Register */

#define S32K1XX_FTM7_PAIR0DEADTIME       (S32K1XX_FTM7_BASE + S32K1XX_FTM_PAIR0DEADTIME_OFFSET) /* FTM7 Pair 0 Deadtime Configuration register */
#define S32K1XX_FTM7_PAIR1DEADTIME       (S32K1XX_FTM7_BASE + S32K1XX_FTM_PAIR1DEADTIME_OFFSET) /* FTM7 Pair 1 Deadtime Configuration register */
#define S32K1XX_FTM7_PAIR2DEADTIME       (S32K1XX_FTM7_BASE + S32K1XX_FTM_PAIR2DEADTIME_OFFSET) /* FTM7 Pair 2 Deadtime Configuration register */
#define S32K1XX_FTM7_PAIR3DEADTIME       (S32K1XX_FTM7_BASE + S32K1XX_FTM_PAIR3DEADTIME_OFFSET) /* FTM7 Pair 3 Deadtime Configuration register oset */

#define S32K1XX_FTM7_MOD_MIRROR          (S32K1XX_FTM7_BASE + S32K1XX_FTM_MOD_MIRROR_OFFSET) /* FTM7 Mirror of Modulo Value register */
#define S32K1XX_FTM7_C0V_MIRROR          (S32K1XX_FTM7_BASE + S32K1XX_FTM_C0V_MIRROR_OFFSET) /* FTM7 Mirror of Channel 0 Match Value register */
#define S32K1XX_FTM7_C1V_MIRROR          (S32K1XX_FTM7_BASE + S32K1XX_FTM_C1V_MIRROR_OFFSET) /* FTM7 Mirror of Channel 1 Match Value register */
#define S32K1XX_FTM7_C2V_MIRROR          (S32K1XX_FTM7_BASE + S32K1XX_FTM_C2V_MIRROR_OFFSET) /* FTM7 Mirror of Channel 2 Match Value register */
#define S32K1XX_FTM7_C3V_MIRROR          (S32K1XX_FTM7_BASE + S32K1XX_FTM_C3V_MIRROR_OFFSET) /* FTM7 Mirror of Channel 3 Match Value register */
#define S32K1XX_FTM7_C4V_MIRROR          (S32K1XX_FTM7_BASE + S32K1XX_FTM_C4V_MIRROR_OFFSET) /* FTM7 Mirror of Channel 4 Match Value register */
#define S32K1XX_FTM7_C5V_MIRROR          (S32K1XX_FTM7_BASE + S32K1XX_FTM_C5V_MIRROR_OFFSET) /* FTM7 Mirror of Channel 5 Match Value register */
#define S32K1XX_FTM7_C6V_MIRROR          (S32K1XX_FTM7_BASE + S32K1XX_FTM_C6V_MIRROR_OFFSET) /* FTM7 Mirror of Channel 6 Match Value register */
#define S32K1XX_FTM7_C7V_MIRROR          (S32K1XX_FTM7_BASE + S32K1XX_FTM_C7V_MIRROR_OFFSET) /* FTM7 Mirror of Channel 7 Match Value register */

/* Register Bit Definitions *************************************************/

/* Status And Control register */

#define FTM_SC_PS_SHIFT                  (0)       /* Bits 0-2: Prescale Factor Selection */
#define FTM_SC_PS_MASK                   (0x07 << FTM_SC_PS_SHIFT)
#  define FTP_SC_PS_DIV1                 (0 << FTM_SC_PS_SHIFT) /* Divide by 1 */
#  define FTP_SC_PS_DIV2                 (1 << FTM_SC_PS_SHIFT) /* Divide by 2 */
#  define FTP_SC_PS_DIV4                 (2 << FTM_SC_PS_SHIFT) /* Divide by 4 */
#  define FTP_SC_PS_DIV8                 (3 << FTM_SC_PS_SHIFT) /* Divide by 8 */
#  define FTP_SC_PS_DIV16                (4 << FTM_SC_PS_SHIFT) /* Divide by 16 */
#  define FTP_SC_PS_DIV32                (5 << FTM_SC_PS_SHIFT) /* Divide by 32 */
#  define FTP_SC_PS_DIV64                (6 << FTM_SC_PS_SHIFT) /* Divide by 64 */
#  define FTP_SC_PS_DIV128               (7 << FTM_SC_PS_SHIFT) /* Divide by 128 */

#define FTM_SC_CLKS_SHIFT                (3)       /* Bits 3-4: Clock Source Selection */
#define FTM_SC_CLKS_MASK                 (0x03 << FTM_SC_CLKS_SHIFT)
#  define FTM_SC_CLKS_DIS                (0 << FTM_SC_CLKS_SHIFT) /* No clock selected. This in effect disables the FTM counter */
#  define FTM_SC_CLKS_FTM                (1 << FTM_SC_CLKS_SHIFT) /* FTM input clock */
#  define FTM_SC_CLKS_FIXED              (2 << FTM_SC_CLKS_SHIFT) /* Fixed frequency clock */
#  define FTM_SC_CLKS_EXTCLK             (3 << FTM_SC_CLKS_SHIFT) /* External clock */

#define FTM_SC_CPWMS                     (1 << 5)  /* Bit 5: Center-Aligned PWM Select */
#define FTM_SC_RIE                       (1 << 6)  /* Bit 6: Reload Point Interrupt Enable */
#define FTM_SC_RF                        (1 << 7)  /* Bit 7: Reload Flag */
#define FTM_SC_TOIE                      (1 << 8)  /* Bit 8: Timer Overflow Interrupt Enable */
#define FTM_SC_TOF                       (1 << 9)  /* Bit 9: Timer Overflow Flag */

                                                   /* Bits 10-15: Reserved */

#define FTM_SC_PWMEN_SHIFT               (16)      /* Bits 16-23: Channel n PWM enable bit */
#define FTM_SC_PWMEN_MASK                (0xff << FTM_SC_PWMEN_SHIFT)
#define FTM_SC_PWMEN(n)                  (1 << FTM_SC_PWMEN_SHIFT + (n))
#define FTM_SC_FLTPS_SHIFT               (24)      /* Bits 24-27: Filter Prescaler */
#define FTM_SC_FLTPS_MASK                (0x07 << FTM_SC_FLTPS_SHIFT)
#  define FTM_SC_FLTPS_DIV1              (0 << FTM_SC_FLTPS_SHIFT)  /* Divide by 1 */
#  define FTM_SC_FLTPS_DIV2              (1 << FTM_SC_FLTPS_SHIFT)  /* Divide by 2 */
#  define FTM_SC_FLTPS_DIV3              (2 << FTM_SC_FLTPS_SHIFT)  /* Divide by 3 */
#  define FTM_SC_FLTPS_DIV4              (3 << FTM_SC_FLTPS_SHIFT)  /* Divide by 4 */
#  define FTM_SC_FLTPS_DIV5              (4 << FTM_SC_FLTPS_SHIFT)  /* Divide by 5 */
#  define FTM_SC_FLTPS_DIV6              (5 << FTM_SC_FLTPS_SHIFT)  /* Divide by 6 */
#  define FTM_SC_FLTPS_DIV7              (6 << FTM_SC_FLTPS_SHIFT)  /* Divide by 7 */
#  define FTM_SC_FLTPS_DIV8              (7 << FTM_SC_FLTPS_SHIFT)  /* Divide by 8 */
#  define FTM_SC_FLTPS_DIV9              (8 << FTM_SC_FLTPS_SHIFT)  /* Divide by 9 */
#  define FTM_SC_FLTPS_DIV10             (9 << FTM_SC_FLTPS_SHIFT)  /* Divide by 10 */
#  define FTM_SC_FLTPS_DIV11             (10 << FTM_SC_FLTPS_SHIFT) /* Divide by 11 */
#  define FTM_SC_FLTPS_DIV12             (11 << FTM_SC_FLTPS_SHIFT) /* Divide by 12 */
#  define FTM_SC_FLTPS_DIV13             (12 << FTM_SC_FLTPS_SHIFT) /* Divide by 13 */
#  define FTM_SC_FLTPS_DIV14             (13 << FTM_SC_FLTPS_SHIFT) /* Divide by 14 */
#  define FTM_SC_FLTPS_DIV15             (14 << FTM_SC_FLTPS_SHIFT) /* Divide by 15 */
#  define FTM_SC_FLTPS_DIV16             (15 << FTM_SC_FLTPS_SHIFT) /* Divide by 16 */

                                                   /* Bits 28-31: Reserved */

/* Counter register */

#define FTM_CNT_COUNT_SHIFT              (0)       /* Bits 0-15: Counter Value */
#define FTM_CNT_COUNT_MASK               (0xff << FTM_CNT_COUNT_SHIFT)

                                                   /* Bits 16-31: Reserved */

/* Modulo register */

#define FTM_MOD_MOD_SHIFT                (0)       /* Bits 0-15: Modulo Value */
#define FTM_MOD_MOD_MASK                 (0xff << FTM_MOD_MOD_SHIFT)

                                                   /* Bits 16-31: Reserved */

/* Channel (n) Status And Control register */

#define FTM_CNSC_DMA                     (1 << 0)  /* Bit 0: DMA Enable */
#define FTM_CNSC_ICRST                   (1 << 1)  /* Bit 1: FTM Counter reset by the selected input capture event */
#define FTM_CNSC_ELSA                    (1 << 2)  /* Bit 2: Channel (n) Edge or Level Select */
#define FTM_CNSC_ELSB                    (1 << 3)  /* Bit 3: Channel (n) Edge or Level Select */
#define FTM_CNSC_MSA                     (1 << 4)  /* Bit 4: Channel (n) Mode Select */
#define FTM_CNSC_MSB                     (1 << 5)  /* Bit 5: Channel (n) Mode Select */
#define FTM_CNSC_CHIE                    (1 << 6)  /* Bit 6: Channel (n) Interrupt Enable */
#define FTM_CNSC_CHF                     (1 << 7)  /* Bit 7: Channel (n) flag */
#define FTM_CNSC_TRIGMODE                (1 << 8)  /* Bit 8: Trigger Mode Control */
#define FTM_CNSC_CHIS                    (1 << 9)  /* Bit 9: Channel (n) Input State */
#define FTM_CNSC_CHOV                    (1 << 10) /* Bit 10: Channel (n) Output Value */

                                                   /* Bits 11-31: Reserved */

/* Channel (n) Value register */

#define FTM_CNV_VAL_SHIFT                (0)       /* Bits 0-15: Channel Value */
#define FTM_CNV_VAL_MASK                 (0xff << FTM_CNV_VAL_SHIFT)

                                                   /* Bits 16-31: Reserved */

/* Counter Initial Value register */

#define FTM_CNTIN_INIT_SHIFT             (0)       /* Bits 0-15: Initial Value of the FTM Counter */
#define FTM_CNTIN_VAL_MASK               (0xff << FTM_CNTIN_INIT_SHIFT)

                                                   /* Bits 16-31: Reserved */

/* Capture And Compare Status register */

#define FTM_STATUS_CH0F                  (1 << 0)  /* Bit 0: Channel 0 Flag */
#define FTM_STATUS_CH1F                  (1 << 1)  /* Bit 1: Channel 1 Flag */
#define FTM_STATUS_CH2F                  (1 << 2)  /* Bit 2: Channel 2 Flag */
#define FTM_STATUS_CH3F                  (1 << 3)  /* Bit 3: Channel 3 Flag */
#define FTM_STATUS_CH4F                  (1 << 4)  /* Bit 4: Channel 4 Flag */
#define FTM_STATUS_CH5F                  (1 << 5)  /* Bit 5: Channel 5 Flag */
#define FTM_STATUS_CH6F                  (1 << 6)  /* Bit 6: Channel 6 Flag */
#define FTM_STATUS_CH7F                  (1 << 7)  /* Bit 7: Channel 7 Flag */

                                                   /* Bits 8-31: Reserved */

/* Features Mode Selection register */

#define FTM_MODE_FTMEN                   (1 << 0)  /* Bit 0: FTM Enable */
#define FTM_MODE_INIT                    (1 << 1)  /* Bit 1: Initialize the Channels Output */
#define FTM_MODE_WPDIS                   (1 << 2)  /* Bit 2: Write Protection Disable */
#define FTM_MODE_PWMSYNC                 (1 << 3)  /* Bit 3: PWM Synchronization Mode */
#define FTM_MODE_CAPTEST                 (1 << 4)  /* Bit 4: Capture Test Mode Enable */
#define FTM_MODE_FAULTM_SHIFT            (5)       /* Bits 5-6: Fault Control Mode */
#define FTM_MODE_FAULTM_MASK             (0x03 << FTM_MODE_FAULTM_SHIFT)
#  define FTM_MODE_FAULTM_DIS            (0 << FTM_MODE_FAULTM_SHIFT) /* Fault control is disabled for all channels. */
#  define FTM_MODE_FAULTM_EVEN_MAN       (1 << FTM_MODE_FAULTM_SHIFT) /* Fault control is enabled for even channels only (channels 0, 2, 4, and 6), and the selected mode is the manual fault clearing. */
#  define FTM_MODE_FAULTM_ALL_MAN        (2 << FTM_MODE_FAULTM_SHIFT) /* Fault control is enabled for all channels, and the selected mode is the manual fault clearing. */
#  define FTM_MODE_FAULTM_ALL_AUTO       (3 << FTM_MODE_FAULTM_SHIFT) /* Fault control is enabled for all channels, and the selected mode is the automatic fault clearing. */

#define FTM_MODE_FAULTIE                 (1 << 7)  /* Bit 7: Fault Interrupt Enable */

                                                   /* Bits 8-31: Reserved */

/* Synchronization register */

#define FTM_SYNC_CNTMIN                  (1 << 0)  /* Bit 0: Minimum Loading Point Enable */
#define FTM_SYNC_CNTMAX                  (1 << 1)  /* Bit 1: Maximum Loading Point Enable */
#define FTM_SYNC_REINIT                  (1 << 2)  /* Bit 2: FTM Counter Reinitialization by Synchronization */
#define FTM_SYNC_SYNCHOM                 (1 << 3)  /* Bit 3: Output Mask Synchronization */
#define FTM_SYNC_TRIG0                   (1 << 4)  /* Bit 4: PWM Synchronization Hardware Trigger 0 */
#define FTM_SYNC_TRIG1                   (1 << 5)  /* Bit 5: PWM Synchronization Hardware Trigger 1 */
#define FTM_SYNC_TRIG2                   (1 << 6)  /* Bit 6: PWM Synchronization Hardware Trigger 2 */
#define FTM_SYNC_SWSYNC                  (1 << 7)  /* Bit 7: PWM Synchronization Software Trigger */

                                                   /* Bit 8-31: Reserved */

/* Initial State For Channels Output register */

#define FTM_OUTINIT_CH0OI                (1 << 0)  /* Bit 0: Channel 0 Output Initialization Value */
#define FTM_OUTINIT_CH1OI                (1 << 1)  /* Bit 1: Channel 1 Output Initialization Value */
#define FTM_OUTINIT_CH2OI                (1 << 2)  /* Bit 2: Channel 2 Output Initialization Value */
#define FTM_OUTINIT_CH3OI                (1 << 3)  /* Bit 3: Channel 3 Output Initialization Value */
#define FTM_OUTINIT_CH4OI                (1 << 4)  /* Bit 4: Channel 4 Output Initialization Value */
#define FTM_OUTINIT_CH5OI                (1 << 5)  /* Bit 5: Channel 5 Output Initialization Value */
#define FTM_OUTINIT_CH6OI                (1 << 6)  /* Bit 6: Channel 6 Output Initialization Value */
#define FTM_OUTINIT_CH7OI                (1 << 7)  /* Bit 7: Channel 7 Output Initialization Value */

                                                   /* Bit 8-31: Reserved */

/* Output Mask register */

#define FTM_OUTMASK_CH0OM                (1 << 0)  /* Bit 0: Channel 0 Ouput Mask */
#define FTM_OUTMASK_CH1OM                (1 << 1)  /* Bit 1: Channel 1 Ouput Mask */
#define FTM_OUTMASK_CH2OM                (1 << 2)  /* Bit 2: Channel 2 Ouput Mask */
#define FTM_OUTMASK_CH3OM                (1 << 3)  /* Bit 3: Channel 3 Ouput Mask */
#define FTM_OUTMASK_CH4OM                (1 << 4)  /* Bit 4: Channel 4 Ouput Mask */
#define FTM_OUTMASK_CH5OM                (1 << 5)  /* Bit 5: Channel 5 Ouput Mask */
#define FTM_OUTMASK_CH6OM                (1 << 6)  /* Bit 6: Channel 6 Ouput Mask */
#define FTM_OUTMASK_CH7OM                (1 << 7)  /* Bit 7: Channel 7 Ouput Mask */

                                                   /* Bit 8-31: Reserved */

/* Function For Linked Channels register */

#define FTM_COMBINE_COMBINE0             (1 << 0)  /* Bit 0: Combine Mode for Channels 0 and 1 */
#define FTM_COMBINE_COMP0                (1 << 1)  /* Bit 1: Channel 1 is Complement of Channel 0 */
#define FTM_COMBINE_DECAPEN0             (1 << 2)  /* Bit 2: Dual Edge Capture Mode Enable for Channel 0 and 1 */
#define FTM_COMBINE_DECAP0               (1 << 3)  /* Bit 3: Dual Edge Capture Mode Capture for Channel 0 and 1 */
#define FTM_COMBINE_DTEN0                (1 << 4)  /* Bit 4: Deadtime Enable for Channel 0 and 1 */
#define FTM_COMBINE_SYNCEN0              (1 << 5)  /* Bit 5: Synchronization Enable for Channel 0 and 1 */
#define FTM_COMBINE_FAULTEN0             (1 << 6)  /* Bit 6: Fault Control Enable for Channel 0 and 1 */
#define FTM_COMBINE_MCOMBINE0            (1 << 7)  /* Bit 7: Modified Combine Mode for Channels 0 and 1 */
#define FTM_COMBINE_COMBINE1             (1 << 8)  /* Bit 8: Combine Mode for Channels 2 and 3 */
#define FTM_COMBINE_COMP1                (1 << 9)  /* Bit 9: Channel 3 is Complement of Channel 2 */
#define FTM_COMBINE_DECAPEN1             (1 << 10) /* Bit 10: Dual Edge Capture Mode Enable for Channel 2 and 3 */
#define FTM_COMBINE_DECAP1               (1 << 11) /* Bit 11: Dual Edge Capture Mode Capture for Channel 2 and 3 */
#define FTM_COMBINE_DTEN1                (1 << 12) /* Bit 12: Deadtime Enable for Channel 2 and 3 */
#define FTM_COMBINE_SYNCEN1              (1 << 13) /* Bit 13: Synchronization Enable for Channel 2 and 3 */
#define FTM_COMBINE_FAULTEN1             (1 << 14) /* Bit 14: Fault Control Enable for Channel 2 and 3 */
#define FTM_COMBINE_MCOMBINE1            (1 << 15) /* Bit 15: Modified Combine Mode for Channels 2 and 3 */
#define FTM_COMBINE_COMBINE2             (1 << 16) /* Bit 16: Combine Mode for Channels 4 and 5 */
#define FTM_COMBINE_COMP2                (1 << 17) /* Bit 17: Channel 5 is Complement of Channel 4 */
#define FTM_COMBINE_DECAPEN2             (1 << 18) /* Bit 18: Dual Edge Capture Mode Enable for Channel 4 and 5 */
#define FTM_COMBINE_DECAP2               (1 << 19) /* Bit 19: Dual Edge Capture Mode Capture for Channel 4 and 5 */
#define FTM_COMBINE_DTEN2                (1 << 20) /* Bit 20: Deadtime Enable for Channel 4 and 5 */
#define FTM_COMBINE_SYNCEN2              (1 << 21) /* Bit 21: Synchronization Enable for Channel 4 and 5 */
#define FTM_COMBINE_FAULTEN2             (1 << 22) /* Bit 22: Fault Control Enable for Channel 4 and 5 */
#define FTM_COMBINE_MCOMBINE2            (1 << 23) /* Bit 23: Modified Combine Mode for Channels 4 and 5 */
#define FTM_COMBINE_COMBINE3             (1 << 24) /* Bit 24: Combine Mode for Channels 6 and 7 */
#define FTM_COMBINE_COMP3                (1 << 25) /* Bit 25: Channel 7 is Complement of Channel 6 */
#define FTM_COMBINE_DECAPEN3             (1 << 26) /* Bit 26: Dual Edge Capture Mode Enable for Channel 6 and 7 */
#define FTM_COMBINE_DECAP3               (1 << 27) /* Bit 27: Dual Edge Capture Mode Capture for Channel 6 and 7 */
#define FTM_COMBINE_DTEN3                (1 << 28) /* Bit 28: Deadtime Enable for Channel 6 and 7 */
#define FTM_COMBINE_SYNCEN3              (1 << 29) /* Bit 29: Synchronization Enable for Channel 6 and 7 */
#define FTM_COMBINE_FAULTEN3             (1 << 30) /* Bit 30: Fault Control Enable for Channel 6 and 7 */
#define FTM_COMBINE_MCOMBINE3            (1 << 31) /* Bit 31: Modified Combine Mode for Channels 6 and 7 */

/* Deadtime Configuration register */

#define FTM_DEADTIME_DTVAL_SHIFT         (0)       /* Bits 0-5: Deadtime Value */
#define FTM_DEADTIME_DTVAL_MASK          (0x1f << FTM_DEADTIME_DTVAL_SHIFT)
#define FTM_DEADTIME_DTPS_SHIFT          (6)       /* Bits 6-7: Deadtime Prescaler Value */
#define FTM_DEADTIME_DTPS_MASK           (0x03 << FTM_DEADTIME_DTPS_SHIFT)
#  define FTM_DEADTIME_DTPS_DIV1         (1 << FTM_DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 1 */
#  define FTM_DEADTIME_DTPS_DIV4         (2 << FTM_DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 4 */
#  define FTM_DEADTIME_DTPS_DIV16        (3 << FTM_DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 16 */

                                                   /* Bits 8-15: Reserved */

#define FTM_DEADTIME_DTVALEX_SHIFT       (16)      /* Bits 16-19: Extended Deadtime Value */
#define FTM_DEADTIME_DTVALEX_MASK        (0x0f << FTM_DEADTIME_DTVALEX_SHIFT)

                                                   /* Bits 20-31: Reserved */

/* FTM External Trigger register */

#define FTM_EXTTRIG_CH2TRIG              (1 << 0)  /* Bit 0: Channel 2 External Trigger Enable */
#define FTM_EXTTRIG_CH3TRIG              (1 << 1)  /* Bit 1: Channel 3 External Trigger Enable */
#define FTM_EXTTRIG_CH4TRIG              (1 << 2)  /* Bit 2: Channel 4 External Trigger Enable */
#define FTM_EXTTRIG_CH5TRIG              (1 << 3)  /* Bit 3: Channel 5 External Trigger Enable */
#define FTM_EXTTRIG_CH0TRIG              (1 << 4)  /* Bit 4: Channel 0 External Trigger Enable */
#define FTM_EXTTRIG_CH1TRIG              (1 << 5)  /* Bit 5: Channel 1 External Trigger Enable */
#define FTM_EXTTRIG_INITTRIGEN           (1 << 6)  /* Bit 6: Initialization Trigger Enable */
#define FTM_EXTTRIG_TRIGF                (1 << 7)  /* Bit 7: Channel Trigger Flag */
#define FTM_EXTTRIG_CH6TRIG              (1 << 8)  /* Bit 8: Channel 6 External Trigger */
#define FTM_EXTTRIG_CH7TRIG              (1 << 9)  /* Bit 9: Channel 7 External Trigger */

                                                   /* Bits 10-31: Reserved */

/* Channels Polarity register */

#define FTM_POL_POL0                     (1 << 0)  /* Bit 0: Channel 0 Polarity */
#define FTM_POL_POL1                     (1 << 1)  /* Bit 1: Channel 1 Polarity */
#define FTM_POL_POL2                     (1 << 2)  /* Bit 2: Channel 2 Polarity */
#define FTM_POL_POL3                     (1 << 3)  /* Bit 3: Channel 3 Polarity */
#define FTM_POL_POL4                     (1 << 4)  /* Bit 4: Channel 4 Polarity */
#define FTM_POL_POL5                     (1 << 5)  /* Bit 5: Channel 5 Polarity */
#define FTM_POL_POL6                     (1 << 6)  /* Bit 6: Channel 6 Polarity */
#define FTM_POL_POL7                     (1 << 7)  /* Bit 7: Channel 7 Polarity */

                                                   /* Bits 8-31: Reserved */

/* Fault Mode Status register */

#define FTM_FMS_FAULTF0                  (1 << 0)  /* Bit 0: Fault Detection Flag 0 */
#define FTM_FMS_FAULTF1                  (1 << 1)  /* Bit 1: Fault Detection Flag 1 */
#define FTM_FMS_FAULTF2                  (1 << 2)  /* Bit 2: Fault Detection Flag 2 */
#define FTM_FMS_FAULTF3                  (1 << 3)  /* Bit 3: Fault Detection Flag 3 */

                                                   /* Bit 4: Reserved */

#define FTM_FMS_FAULTIN                  (1 << 5)  /* Bit 5: Fault Inputs */
#define FTM_FMS_WPEN                     (1 << 6)  /* Bit 6: Write Protection Enable */
#define FTM_FMS_FAULTF                   (1 << 7)  /* Bit 7: Fault Detection Flag */

                                                   /* Bits 8-31: Reserved */

/* Input Capture Filter Control register */

#define FTM_FILTER_CH0FVAL_SHIFT         (0)       /* Bits 0-3: Channel 0 Input Filter */
#define FTM_FILTER_CH0FVAL_MASK          (0x0f << FTM_FILTER_CH0FVAL_SHIFT)
#define FTM_FILTER_CH1FVAL_SHIFT         (4)       /* Bits 4-7: Channel 1 Input Filter */
#define FTM_FILTER_CH1FVAL_MASK          (0x0f << FTM_FILTER_CH1FVAL_SHIFT)
#define FTM_FILTER_CH2FVAL_SHIFT         (8)       /* Bits 8-11: Channel 2 Input Filter */
#define FTM_FILTER_CH2FVAL_MASK          (0x0f << FTM_FILTER_CH2FVAL_SHIFT)
#define FTM_FILTER_CH3FVAL_SHIFT         (12)      /* Bits 12-15: Channel 3 Input Filter */
#define FTM_FILTER_CH3FVAL_MASK          (0x0f << FTM_FILTER_CH3FVAL_SHIFT)

                                                   /* Bits 16-31: Reserved */

/* Fault Control register */

#define FTM_FLTCTRL_FAULT0EN             (1 << 0)  /* Bit 0: Fault Input 0 Enable */
#define FTM_FLTCTRL_FAULT1EN             (1 << 1)  /* Bit 1: Fault Input 1 Enable */
#define FTM_FLTCTRL_FAULT2EN             (1 << 2)  /* Bit 2: Fault Input 2 Enable */
#define FTM_FLTCTRL_FAULT3EN             (1 << 3)  /* Bit 3: Fault Input 3 Enable */
#define FTM_FLTCTRL_FFLTR0EN             (1 << 4)  /* Bit 4: Fault Input 0 Filter Enable */
#define FTM_FLTCTRL_FFLTR1EN             (1 << 5)  /* Bit 5: Fault Input 1 Filter Enable */
#define FTM_FLTCTRL_FFLTR2EN             (1 << 6)  /* Bit 6: Fault Input 2 Filter Enable */
#define FTM_FLTCTRL_FFLTR3EN             (1 << 7)  /* Bit 7: Fault Input 3 Filter Enable */
#define FTM_FLTCTRL_FFVAL_SHIFT          (8)       /* Bits 8-11: Fault Input Filter */
#define FTM_FLTCTRL_FFVAL_MASK           (0x0f << FTM_FLTCTRL_FFVAL_SHIFT)

                                                   /* Bits 12-14: Reserved */

#define FTM_FLTCTRL_FSTATE               (1 << 15) /* Bit 15: Fault Output State */

                                                   /* Bits 16-31: Reserved */

/* Quadrature Decoder Control And Status register */

#define FTM_QDCTRL_QUADEN                (1 << 0)  /* Bit 0: Quadrature Decoder Mode Enable */
#define FTM_QDCTRL_TOFDIR                (1 << 1)  /* Bit 1: Timer Overflow Direction in Quadrature Decoder Mode */
#define FTM_QDCTRL_QUADIR                (1 << 2)  /* Bit 2: FTM Counter Direction in Quadrature Decoder Mode */
#define FTM_QDCTRL_QUADMODE              (1 << 3)  /* Bit 3: Quadrature Decoder Mode */
#define FTM_QDCTRL_PHBPOL                (1 << 4)  /* Bit 4: Phase B Input Polarity */
#define FTM_QDCTRL_PHAPOL                (1 << 5)  /* Bit 5: Phase A Input Polarity */
#define FTM_QDCTRL_PHBFLTREN             (1 << 6)  /* Bit 6: Phase B Input Filter Enable */
#define FTM_QDCTRL_PHAFLTREN             (1 << 7)  /* Bit 7: Phase A Input Filter Enable */

                                                   /* Bits 8-31: Reserved */

/* Configuration register */

#define FTM_CONF_LDFQ_SHIFT              (0)       /* Bits 0-4: Frequency of the Reload Opportunities */
#define FTM_CONF_LDFQ_MASK               (0x1f << FTM_CONF_LDFQ_SHIFT)

                                                   /* Bit 5: Reserved */

#define FTM_CONF_BDMMODE_SHIFT           (6)       /* Bits 6-7: Debug Mode */
#define FTM_CONF_BDMMODE_MASK            (0x03 << FTM_CONF_BDMMODE_SHIFT)

                                                   /* Bit 8: Reserved */

#define FTM_CONF_GTBEEN                  (1 << 9)  /* Bit 9:  Global Time Base Enable */
#define FTM_CONF_GTBEOUT                 (1 << 10) /* Bit 10: Global Time Base Output */
#define FTM_CONF_ITRIGR                  (1 << 11) /* Bit 11: Initialization trigger on Reload Point */

                                                   /* Bits 12-31: Reserved */

/* FTM Fault Input Polarity register */

#define FTM_FLTPOL_FLT0POL               (1 << 0)  /* Bit 0: Fault Input 0 Polarity */
#define FTM_FLTPOL_FLT1POL               (1 << 1)  /* Bit 1: Fault Input 1 Polarity */
#define FTM_FLTPOL_FLT2POL               (1 << 2)  /* Bit 2: Fault Input 2 Polarity */
#define FTM_FLTPOL_FLT3POL               (1 << 3)  /* Bit 3: Fault Input 3 Polarity */

                                                   /* Bits 4-31: Reserved */

/* Synchronization Configuration register */

#define FTM_SYNCONF_HWTRIGMODE           (1 << 0)  /* Bit 0: Hardware Trigger Mode */

                                                   /* Bit 1: Reserved */

#define FTM_SYNCONF_CNTINC               (1 << 2)  /* Bit 2: CNTIN Register Synchronization */

                                                   /* Bit 3: Reserved */

#define FTM_SYNCONF_INVC                 (1 << 4)  /* Bit 4: INVCTRL Register Synchronization */
#define FTM_SYNCONF_SWOC                 (1 << 5)  /* Bit 5: SWOCTRL Register Synchronization */

                                                   /* Bit 6:  Reserved */

#define FTM_SYNCONF_SYNCMODE             (1 << 7)  /* Bit 7: Synchronization Mode */
#define FTM_SYNCONF_SWRSTCNT             (1 << 8)  /* Bit 8: FTM counter synchronization is activated by the software trigger */
#define FTM_SYNCONF_SWWRBUF              (1 << 9)  /* Bit 9: MOD, HCR, CNTIN, and CV registers synchronization is activated by the software trigger */
#define FTM_SYNCONF_SWOM                 (1 << 10) /* Bit 10: Output mask synchronization is activated by the software trigger */
#define FTM_SYNCONF_SWINVC               (1 << 11) /* Bit 11: Inverting control synchronization is activated by the software trigger */
#define FTM_SYNCONF_SWSOC                (1 << 12) /* Bit 12: Software output control synchronization is activated by the software trigger */

                                                   /* Bits 13-15: Reserved */

#define FTM_SYNCONF_HWRSTCNT             (1 << 16) /* Bit 16: FTM counter synchronization is activated by a hardware trigger */
#define FTM_SYNCONF_HWWRBUF              (1 << 17) /* Bit 17: MOD, HCR, CNTIN, and CV registers synchronization is activated by a hardware trigger */
#define FTM_SYNCONF_HWOM                 (1 << 18) /* Bit 18: Output mask synchronization is activated by a hardware trigger */
#define FTM_SYNCONF_HWINVC               (1 << 19) /* Bit 19: Inverting control synchronization is activated by a hardware trigger */
#define FTM_SYNCONF_HWSOC                (1 << 20) /* Bit 20: Software output control synchronization is activated by a hardware trigger */

                                                   /* Bits 21-31: Reserved */

/* FTM Inverting Control register */

#define FTM_INVCTRL_INV0EN               (1 << 0)  /* Bit 0: Pair Channels 0 Inverting Enable */
#define FTM_INVCTRL_INV1EN               (1 << 1)  /* Bit 1: Pair Channels 1 Inverting Enable */
#define FTM_INVCTRL_INV2EN               (1 << 2)  /* Bit 2: Pair Channels 2 Inverting Enable */
#define FTM_INVCTRL_INV3EN               (1 << 3)  /* Bit 3: Pair Channels 3 Inverting Enable */

                                                   /* Bits 4-31: Reserved */

/* FTM Software Output Control register */

#define FTM_SWOCTRL_CH0OC                (1 << 0)  /* Bit 0: Channel 0 Software Output Control Enable */
#define FTM_SWOCTRL_CH1OC                (1 << 1)  /* Bit 1: Channel 1 Software Output Control Enable */
#define FTM_SWOCTRL_CH2OC                (1 << 2)  /* Bit 2: Channel 2 Software Output Control Enable */
#define FTM_SWOCTRL_CH3OC                (1 << 3)  /* Bit 3: Channel 3 Software Output Control Enable */
#define FTM_SWOCTRL_CH4OC                (1 << 4)  /* Bit 4: Channel 4 Software Output Control Enable */
#define FTM_SWOCTRL_CH5OC                (1 << 5)  /* Bit 5: Channel 5 Software Output Control Enable */
#define FTM_SWOCTRL_CH6OC                (1 << 6)  /* Bit 6: Channel 6 Software Output Control Enable */
#define FTM_SWOCTRL_CH7OC                (1 << 7)  /* Bit 7: Channel 7 Software Output Control Enable */
#define FTM_SWOCTRL_CH0OCV               (1 << 8)  /* Bit 8: Channel 0 Software Output Control Value */
#define FTM_SWOCTRL_CH1OCV               (1 << 9)  /* Bit 9: Channel 1 Software Output Control Value */
#define FTM_SWOCTRL_CH2OCV               (1 << 10) /* Bit 10: Channel 2 Software Output Control Value */
#define FTM_SWOCTRL_CH3OCV               (1 << 11) /* Bit 11: Channel 3 Software Output Control Value */
#define FTM_SWOCTRL_CH4OCV               (1 << 12) /* Bit 12: Channel 4 Software Output Control Value */
#define FTM_SWOCTRL_CH5OCV               (1 << 13) /* Bit 13: Channel 5 Software Output Control Value */
#define FTM_SWOCTRL_CH6OCV               (1 << 14) /* Bit 14: Channel 6 Software Output Control Value */
#define FTM_SWOCTRL_CH7OCV               (1 << 15) /* Bit 15: Channel 7 Software Output Control Value */

                                                   /* Bits 16-31: Reserved */

/* FTM PWM Load register */

#define FTM_PWMLOAD_CH0SEL               (1 << 0)  /* Bit 0: Channel 0 Select */
#define FTM_PWMLOAD_CH1SEL               (1 << 1)  /* Bit 1: Channel 1 Select */
#define FTM_PWMLOAD_CH2SEL               (1 << 2)  /* Bit 2: Channel 2 Select */
#define FTM_PWMLOAD_CH3SEL               (1 << 3)  /* Bit 3: Channel 3 Select */
#define FTM_PWMLOAD_CH4SEL               (1 << 4)  /* Bit 4: Channel 4 Select */
#define FTM_PWMLOAD_CH5SEL               (1 << 5)  /* Bit 5: Channel 5 Select */
#define FTM_PWMLOAD_CH6SEL               (1 << 6)  /* Bit 6: Channel 6 Select */
#define FTM_PWMLOAD_CH7SEL               (1 << 7)  /* Bit 7: Channel 7 Select */
#define FTM_PWMLOAD_HCSEL                (1 << 8)  /* Bit 8: Half Cycle Select */
#define FTM_PWMLOAD_LDOK                 (1 << 9)  /* Bit 9: Load Enable */
#define FTM_PWMLOAD_GLEN                 (1 << 10) /* Bit 10: Global Load Enable */
#define FTM_PWMLOAD_GLDOK                (1 << 11) /* Bit 11: Global Load OK */

                                                   /* Bits 12-31: Reserved */

/* Half Cycle Register */

#define FTM_HCR_HCVAL_SHIFT              (0)       /* Bits 0-15: Half Cycle Value */
#define FTM_HCR_HCVAL_MASK               (0xff << FTM_HCR_HCVAL_SHIFT)

                                                   /* Bits 16-31: Reserved */

/* Pair 0 Deadtime Configuration register */

#define FTM_PAIR0DEADTIME_DTVAL_SHIFT    (0)       /* Bits 0-5: Deadtime Value */
#define FTM_PAIR0DEADTIME_DTVAL_MASK     (0x1f << FTM_PAIR0DEADTIME_DTVAL_SHIFT)
#define FTM_PAIR0DEADTIME_DTPS_SHIFT     (6)       /* Bits 6-7: Deadtime Prescaler Value */
#define FTM_PAIR0DEADTIME_DTPS_MASK      (0x03 << FTM_PAIR0DEADTIME_DTPS_SHIFT)
#  define FTM_PAIR0DEADTIME_DTPS_DIV1    (1 << FTM_PAIR0DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 1 */
#  define FTM_PAIR0DEADTIME_DTPS_DIV4    (2 << FTM_PAIR0DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 4 */
#  define FTM_PAIR0DEADTIME_DTPS_DIV16   (3 << FTM_PAIR0DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 16 */

                                                   /* Bits 8-15: Reserved */

#define FTM_PAIR0DEADTIME_DTVALEX_SHIFT  (16)      /* Bits 16-19: Extended Deadtime Value */
#define FTM_PAIR0DEADTIME_DTVALEX_MASK   (0x0f << FTM_PAIR0DEADTIME_DTVALEX_SHIFT)

                                                   /* Bits 20-31: Reserved */

/* Pair 1 Deadtime Configuration register */

#define FTM_PAIR1DEADTIME_DTVAL_SHIFT    (0)       /* Bits 0-5: Deadtime Value */
#define FTM_PAIR1DEADTIME_DTVAL_MASK     (0x1f << FTM_PAIR1DEADTIME_DTVAL_SHIFT)
#define FTM_PAIR1DEADTIME_DTPS_SHIFT     (6)       /* Bits 6-7: Deadtime Prescaler Value */
#define FTM_PAIR1DEADTIME_DTPS_MASK      (0x03 << FTM_PAIR1DEADTIME_DTPS_SHIFT)
#  define FTM_PAIR1DEADTIME_DTPS_DIV1    (1 << FTM_PAIR1DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 1 */
#  define FTM_PAIR1DEADTIME_DTPS_DIV4    (2 << FTM_PAIR1DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 4 */
#  define FTM_PAIR1DEADTIME_DTPS_DIV16   (3 << FTM_PAIR1DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 16 */

                                                   /* Bits 8-15: Reserved */

#define FTM_PAIR1DEADTIME_DTVALEX_SHIFT  (16)      /* Bits 16-19: Extended Deadtime Value */
#define FTM_PAIR1DEADTIME_DTVALEX_MASK   (0x0f << FTM_PAIR1DEADTIME_DTVALEX_SHIFT)

                                                   /* Bits 20-31: Reserved */

/* Pair 2 Deadtime Configuration register */

#define FTM_PAIR2DEADTIME_DTVAL_SHIFT    (0)       /* Bits 0-5: Deadtime Value */
#define FTM_PAIR2DEADTIME_DTVAL_MASK     (0x1f << FTM_PAIR2DEADTIME_DTVAL_SHIFT)
#define FTM_PAIR2DEADTIME_DTPS_SHIFT     (6)       /* Bits 6-7: Deadtime Prescaler Value */
#define FTM_PAIR2DEADTIME_DTPS_MASK      (0x03 << FTM_PAIR2DEADTIME_DTPS_SHIFT)
#  define FTM_PAIR2DEADTIME_DTPS_DIV1    (1 << FTM_PAIR2DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 1 */
#  define FTM_PAIR2DEADTIME_DTPS_DIV4    (2 << FTM_PAIR2DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 4 */
#  define FTM_PAIR2DEADTIME_DTPS_DIV16   (3 << FTM_PAIR2DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 16 */

                                                   /* Bits 8-15: Reserved */

#define FTM_PAIR2DEADTIME_DTVALEX_SHIFT  (16)      /* Bits 16-19: Extended Deadtime Value */
#define FTM_PAIR2DEADTIME_DTVALEX_MASK   (0x0f << FTM_PAIR2DEADTIME_DTVALEX_SHIFT)

                                                   /* Bits 20-31: Reserved */

/* Pair 3 Deadtime Configuration register */

#define FTM_PAIR3DEADTIME_DTVAL_SHIFT    (0)       /* Bits 0-5: Deadtime Value */
#define FTM_PAIR3DEADTIME_DTVAL_MASK     (0x1f << FTM_PAIR3DEADTIME_DTVAL_SHIFT)
#define FTM_PAIR3DEADTIME_DTPS_SHIFT     (6)       /* Bits 6-7: Deadtime Prescaler Value */
#define FTM_PAIR3DEADTIME_DTPS_MASK      (0x03 << FTM_PAIR3DEADTIME_DTPS_SHIFT)
#  define FTM_PAIR3DEADTIME_DTPS_DIV1    (1 << FTM_PAIR3DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 1 */
#  define FTM_PAIR3DEADTIME_DTPS_DIV4    (2 << FTM_PAIR3DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 4 */
#  define FTM_PAIR3DEADTIME_DTPS_DIV16   (3 << FTM_PAIR3DEADTIME_DTPS_SHIFT) /* Divide the FTM input clock by 16 */

                                                   /* Bits 8-15: Reserved */

#define FTM_PAIR3DEADTIME_DTVALEX_SHIFT  (16)      /* Bits 16-19: Extended Deadtime Value */
#define FTM_PAIR3DEADTIME_DTVALEX_MASK   (0x0f << FTM_PAIR3DEADTIME_DTVALEX_SHIFT)

                                                   /* Bits 20-31: Reserved */

/* Mirror of Modulo Value register */

                                                   /* Bits 0-10: Reserved */

#define FTM_MOD_MIRROR_FRACMOD_SHIFT     (11)      /* Bits 11-15: Modulo Fractional Value */
#define FTM_MOD_MIRROR_FRACMOD_MASK      (0x1f << FTM_MOD_MIRROR_FRACMOD_SHIFT)
#define FTM_MOD_MIRROR_MOD_SHIFT         (16)      /* Bits 16-31: Mirror of the Modulo Integer Value */
#define FTM_MOD_MIRROR_MOD_MASK          (0xff << FTM_MOD_MIRROR_MOD_SHIFT)

/* Mirror of Channel 0 Match Value register */

                                                   /* Bits 0-10: Reserved */

#define FTM_C0V_MIRROR_FRACVAL_SHIFT     (11)      /* Bits 11-15: Channel 0 Match Fractional Value */
#define FTM_C0V_MIRROR_FRACVAL_MASK      (0x1f << FTM_C0V_MIRROR_FRACVAL)
#define FTM_C0V_MIRROR_VAL_SHIFT         (16)      /* Bits 16-31: Mirror of the Channel 0 Match Integer Value */
#define FTM_C0V_MIRROR_VAL_MASK          (0xff << FTM_C0V_MIRROR_VAL_SHIFT)

/* Mirror of Channel 1 Match Value register */

                                                   /* Bits 0-10: Reserved */

#define FTM_C1V_MIRROR_FRACVAL_SHIFT     (11)      /* Bits 11-15: Channel 1 Match Fractional Value */
#define FTM_C1V_MIRROR_FRACVAL_MASK      (0x1f << FTM_C1V_MIRROR_FRACVAL)
#define FTM_C1V_MIRROR_VAL_SHIFT         (16)      /* Bits 16-31: Mirror of the Channel 1 Match Integer Value */
#define FTM_C1V_MIRROR_VAL_MASK          (0xff << FTM_C1V_MIRROR_VAL_SHIFT)

/* Mirror of Channel 2 Match Value register */

                                                   /* Bits 0-10: Reserved */

#define FTM_C2V_MIRROR_FRACVAL_SHIFT     (11)      /* Bits 11-15: Channel 2 Match Fractional Value */
#define FTM_C2V_MIRROR_FRACVAL_MASK      (0x1f << FTM_C2V_MIRROR_FRACVAL)
#define FTM_C2V_MIRROR_VAL_SHIFT         (16)      /* Bits 16-31: Mirror of the Channel 2 Match Integer Value */
#define FTM_C2V_MIRROR_VAL_MASK          (0xff << FTM_C2V_MIRROR_VAL_SHIFT)

/* Mirror of Channel 3 Match Value register */

                                                   /* Bits 0-10: Reserved */

#define FTM_C3V_MIRROR_FRACVAL_SHIFT     (11)      /* Bits 11-15: Channel 3 Match Fractional Value */
#define FTM_C3V_MIRROR_FRACVAL_MASK      (0x1f << FTM_C3V_MIRROR_FRACVAL)
#define FTM_C3V_MIRROR_VAL_SHIFT         (16)      /* Bits 16-31: Mirror of the Channel 3 Match Integer Value */
#define FTM_C3V_MIRROR_VAL_MASK          (0xff << FTM_C3V_MIRROR_VAL_SHIFT)

/* Mirror of Channel 4 Match Value register */

                                                   /* Bits 0-10: Reserved */

#define FTM_C4V_MIRROR_FRACVAL_SHIFT     (11)      /* Bits 11-15: Channel 4 Match Fractional Value */
#define FTM_C4V_MIRROR_FRACVAL_MASK      (0x1f << FTM_C4V_MIRROR_FRACVAL)
#define FTM_C4V_MIRROR_VAL_SHIFT         (16)      /* Bits 16-31: Mirror of the Channel 4 Match Integer Value */
#define FTM_C4V_MIRROR_VAL_MASK          (0xff << FTM_C4V_MIRROR_VAL_SHIFT)

/* Mirror of Channel 5 Match Value register */

                                                   /* Bits 0-10: Reserved */

#define FTM_C5V_MIRROR_FRACVAL_SHIFT     (11)      /* Bits 11-15: Channel 5 Match Fractional Value */
#define FTM_C5V_MIRROR_FRACVAL_MASK      (0x1f << FTM_C5V_MIRROR_FRACVAL)
#define FTM_C5V_MIRROR_VAL_SHIFT         (16)      /* Bits 16-31: Mirror of the Channel 5 Match Integer Value */
#define FTM_C5V_MIRROR_VAL_MASK          (0xff << FTM_C5V_MIRROR_VAL_SHIFT)

/* Mirror of Channel 6 Match Value register */

                                                   /* Bits 0-10: Reserved */

#define FTM_C6V_MIRROR_FRACVAL_SHIFT     (11)      /* Bits 11-15: Channel 6 Match Fractional Value */
#define FTM_C6V_MIRROR_FRACVAL_MASK      (0x1f << FTM_C6V_MIRROR_FRACVAL)
#define FTM_C6V_MIRROR_VAL_SHIFT         (16)      /* Bits 16-31: Mirror of the Channel 6 Match Integer Value */
#define FTM_C6V_MIRROR_VAL_MASK          (0xff << FTM_C6V_MIRROR_VAL_SHIFT)

/* Mirror of Channel 7 Match Value register */

                                                   /* Bits 0-10: Reserved */

#define FTM_C7V_MIRROR_FRACVAL_SHIFT     (11)      /* Bits 11-15: Channel 7 Match Fractional Value */
#define FTM_C7V_MIRROR_FRACVAL_MASK      (0x1f << FTM_C7V_MIRROR_FRACVAL)
#define FTM_C7V_MIRROR_VAL_SHIFT         (16)      /* Bits 16-31: Mirror of the Channel 7 Match Integer Value */
#define FTM_C7V_MIRROR_VAL_MASK          (0xff << FTM_C7V_MIRROR_VAL_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_FTM_H */
