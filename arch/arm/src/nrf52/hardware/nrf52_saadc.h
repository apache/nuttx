/***************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_saadc.h
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
 ***************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_SAADC_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_SAADC_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* Register offsets for SAADC **********************************************/

#define NRF52_SAADC_TASKS_START_OFFSET        0x0000                 /* Start the SAADCM */
#define NRF52_SAADC_TASKS_SAMPLE_OFFSET       0x0004                 /* Takes one SAADC sample */
#define NRF52_SAADC_TASKS_STOP_OFFSET         0x0008                 /* Stop the SAADC */
#define NRF52_SAADC_TASKS_CALOFFSET_OFFSET    0x000c                 /* Starts offset auto-calibration */
#define NRF52_SAADC_EVENTS_STARTED_OFFSET     0x0100                 /* The SAADC has started */
#define NRF52_SAADC_EVENTS_END_OFFSET         0x0104                 /* The SAADC has filled up the result buffer */
#define NRF52_SAADC_EVENTS_DONE_OFFSET        0x0108                 /* A conversion task has been completed */
#define NRF52_SAADC_EVENTS_RESDONE_OFFSET     0x010c                 /* Result ready for transfer to RAM */
#define NRF52_SAADC_EVENTS_CALDONE_OFFSET     0x0110                 /* Calibration is complete */
#define NRF52_SAADC_EVENTS_STOPPED_OFFSET     0x0110                 /* The SAADC has stopped */
#define NRF52_SAADC_EVENTS_CHLIMH_OFFSET(x)   (0x118 + ((x) * 0x8))  /* Limit high event for channel x */
#define NRF52_SAADC_EVENTS_CHLIML_OFFSET(x)   (0x11c + ((x) * 0x8))  /* Limit low event for channel x */
#define NRF52_SAADC_INTEN_OFFSET              0x0300                 /* Enable or disable interrupt */
#define NRF52_SAADC_INTENSET_OFFSET           0x0304                 /* Enable interrupt */
#define NRF52_SAADC_INTENCLR_OFFSET           0x0308                 /* Disable interrupt */
#define NRF52_SAADC_STATUS_OFFSET             0x0400                 /* Status */
#define NRF52_SAADC_ENABLE_OFFSET             0x0500                 /* Enable or disable SAADC */
#define NRF52_SAADC_CHPSELP_OFFSET(x)         (0x510 + ((x) * 0x10)) /* Input positive pin for CH[x] */
#define NRF52_SAADC_CHPSELN_OFFSET(x)         (0x514 + ((x) * 0x10)) /* Input negative pin for CH[x] */
#define NRF52_SAADC_CHCONFIG_OFFSET(x)        (0x518 + ((x) * 0x10)) /* Input configuration for CH[x] */
#define NRF52_SAADC_CHLIMIT_OFFSET(x)         (0x51c + ((x) * 0x10)) /* High/low limits for event monitoring of a CH[x] */
#define NRF52_SAADC_RESOLUTION_OFFSET         0x05f0                 /* Resolution configuration */
#define NRF52_SAADC_OVERSAMPLE_OFFSET         0x05f4                 /* Oversampling configuration */
#define NRF52_SAADC_SAMPLERATE_OFFSET         0x05f8                 /* Controls normal or continuous sample rate */
#define NRF52_SAADC_PTR_OFFSET                0x062c                 /* Data pointer */
#define NRF52_SAADC_MAXCNT_OFFSET             0x0630                 /* Maximum number of 16-bit samples */
#define NRF52_SAADC_AMOUNT_OFFSET             0x0634                 /* Number of 16-bit samples written to buffer */

/* Register Bitfield Definitions for SAADC *********************************/

/* INTEN/INTENSET/INTENCLR Register */

#define SAADC_INT_STARTED              (1 << 0)         /* Bit 0: Interrupt for event STARTED */
#define SAADC_INT_END                  (1 << 1)         /* Bit 1: Interrupt for event END */
#define SAADC_INT_DONE                 (1 << 2)         /* Bit 2: Interrupt for event DONE */
#define SAADC_INT_RESDONE              (1 << 3)         /* Bit 3: Interrupt for event RESULTDONE */
#define SAADC_INT_CALDONE              (1 << 4)         /* Bit 4: Interrupt for event CALIBRATEDONE */
#define SAADC_INT_STOPPED              (1 << 5)         /* Bit 5: Interrupt for event STOPPED */
#define SAADC_INT_CHXLIMH(x)           (1 << (x + 0x6)) /* Bit (x+6): Interrupt for event CHxLIMITH */
#define SAADC_INT_CHXLIML(x)           (1 << (x + 0x7)) /* Bit (x+7): Interrupt for event CHxLIMITL */

/* STATUS Register */

#define SAADC_STATUS_READY             (0)       /* Bit 0: SAADC is ready */
#define SAADC_STATUS_BUSY              (1 << 0)  /* Bit 0: SAADC is busy */

/* ENABLE Register */

#define SAADC_ENABLE_DIS               (0)       /* Bit 0: Disable SAADC */
#define SAADC_ENABLE_EN                (1 << 0)  /* Bit 0: Enable SAADC */

/* CH[n] PSELx Register */

#define SAADC_CHPSEL_SHIFT            (0)       /* Bits 0-4: Input positive pin selection for CH[x] */
#define SAADC_CHPSEL_MASK             (0xf << SAADC_CHPSEL_SHIFT)
#  define SAADC_CHPSEL_NC             (0x0 << SAADC_CHPSEL_SHIFT)
#  define SAADC_CHPSEL_IN0            (0x1 << SAADC_CHPSEL_SHIFT)
#  define SAADC_CHPSEL_IN1            (0x2 << SAADC_CHPSEL_SHIFT)
#  define SAADC_CHPSEL_IN2            (0x3 << SAADC_CHPSEL_SHIFT)
#  define SAADC_CHPSEL_IN3            (0x4 << SAADC_CHPSEL_SHIFT)
#  define SAADC_CHPSEL_IN4            (0x5 << SAADC_CHPSEL_SHIFT)
#  define SAADC_CHPSEL_IN5            (0x6 << SAADC_CHPSEL_SHIFT)
#  define SAADC_CHPSEL_IN6            (0x7 << SAADC_CHPSEL_SHIFT)
#  define SAADC_CHPSEL_IN7            (0x8 << SAADC_CHPSEL_SHIFT)
#  define SAADC_CHPSEL_VDD            (0x9 << SAADC_CHPSEL_SHIFT)
#  define SAADC_CHPSEL_VDDHDIV5       (0xd << SAADC_CHPSEL_SHIFT)

/* CH[n] CONFIG Register */

#define SAADC_CONFIG_RESP_SHIFT        (0)       /* Bits 0-2: Positive channel resistor control */
#define SAADC_CONFIG_RESP_MASK         (0x3 << SAADC_CONFIG_RESP_SHIFT)
#  define SAADC_CONFIG_RESP_NONE       (0x0 << SAADC_CONFIG_RESP_SHIFT)
#  define SAADC_CONFIG_RESP_PD         (0x1 << SAADC_CONFIG_RESP_SHIFT)
#  define SAADC_CONFIG_RESP_PU         (0x2 << SAADC_CONFIG_RESP_SHIFT)
#  define SAADC_CONFIG_RESP_VDD1P2     (0x3 << SAADC_CONFIG_RESP_SHIFT)
#define SAADC_CONFIG_RESN_SHIFT        (4)       /* Bits 4-5: Negative channel resistor control */
#define SAADC_CONFIG_RESN_MASK         (0x3 << SAADC_CONFIG_RESN_SHIFT)
#  define SAADC_CONFIG_RESN_NONE       (0x0 << SAADC_CONFIG_RESN_SHIFT)
#  define SAADC_CONFIG_RESN_PD         (0x1 << SAADC_CONFIG_RESN_SHIFT)
#  define SAADC_CONFIG_RESN_PU         (0x2 << SAADC_CONFIG_RESN_SHIFT)
#  define SAADC_CONFIG_RESN_VDD1P2     (0x3 << SAADC_CONFIG_RESN_SHIFT)
#define SAADC_CONFIG_GAIN_SHIFT        (8)       /* Bits 8-10: Gain control */
#define SAADC_CONFIG_GAIN_MASK         (0x7 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_1P6        (0x0 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_1P5        (0x1 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_1P4        (0x2 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_1P3        (0x3 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_1P2        (0x4 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_1          (0x5 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_2          (0x6 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_4          (0x7 << SAADC_CONFIG_GAIN_SHIFT)
#define SAADC_CONFIG_REFSEL_INTERNAL   (0 << 12) /* Bit 12: Internal reference (0.6V) */
#define SAADC_CONFIG_REFSEL_VDD1P4     (1 << 12) /* Bit 12: VDD/4 as reference */
#define SAADC_CONFIG_TACQ_SHIFT        (16)      /* Bits 16-18: Acquisition time */
#define SAADC_CONFIG_TACQ_MASK         (0x7 << SAADC_CONFIG_TACQ_SHIFT)
#  define SAADC_CONFIG_TACQ_3US        (0x0 << SAADC_CONFIG_TACQ_SHIFT)
#  define SAADC_CONFIG_TACQ_5US        (0x1 << SAADC_CONFIG_TACQ_SHIFT)
#  define SAADC_CONFIG_TACQ_10US       (0x2 << SAADC_CONFIG_TACQ_SHIFT)
#  define SAADC_CONFIG_TACQ_15US       (0x3 << SAADC_CONFIG_TACQ_SHIFT)
#  define SAADC_CONFIG_TACQ_20US       (0x4 << SAADC_CONFIG_TACQ_SHIFT)
#  define SAADC_CONFIG_TACQ_40US       (0x5 << SAADC_CONFIG_TACQ_SHIFT)
#define SAADC_CONFIG_MODE_SE           (0 << 20) /* Bit 20: Single-ended */
#define SAADC_CONFIG_MODE_DIFF         (1 << 20) /* Bit 20: Differential */
#define SAADC_CONFIG_BURS_DIS          (0 << 24) /* Bit 24: Burst mode is disabled */
#define SAADC_CONFIG_BURS_EN           (1 << 24) /* Bit 24: Burst mode is enabled  */

/* CH[n] LIMIT Register */

#define SAADC_CHLIMIT_LOW_SHIFT        (0)       /* Bits 0-15: Low level limit */
#define SAADC_CHLIMIT_LOW_MASK         (0xffff << SAADC_CHLIMIT_LOW_SHIFT)
#define SAADC_CHLIMIT_HIGH_SHIFT       (16)      /* Bits 0-15: High level limit */
#define SAADC_CHLIMIT_HIGH_MASK        (0xffff << SAADC_CHLIMIT_HIGH_SHIFT)

/* RESOLUTION Register */

#define SAADC_RESOLUTION_SHIFT         (0)       /* Bits 0-2: SAADC resolution */
#define SAADC_RESOLUTION_MASK          (0xf << SAADC_RESOLUTION_SHIFT)
#  define SAADC_RESOLUTION_8BIT        (0x0 << SAADC_RESOLUTION_SHIFT)
#  define SAADC_RESOLUTION_10BIT       (0x1 << SAADC_RESOLUTION_SHIFT)
#  define SAADC_RESOLUTION_12BIT       (0x2 << SAADC_RESOLUTION_SHIFT)
#  define SAADC_RESOLUTION_14BIT       (0x3 << SAADC_RESOLUTION_SHIFT)

/* OVERSAMPLE Register */

#define SAADC_OVERSAMPLE_SHIFT         (0)       /* Bit 0-3: Oversample control */
#define SAADC_OVERSAMPLE_MASK          (0xf << SAADC_OVERSAMPLE_SHIFT)
#  define SAADC_OVERSAMPLE_NONE        (0x0 << SAADC_OVERSAMPLE_SHIFT)
#  define SAADC_OVERSAMPLE_2X          (0x1 << SAADC_OVERSAMPLE_SHIFT)
#  define SAADC_OVERSAMPLE_4X          (0x2 << SAADC_OVERSAMPLE_SHIFT)
#  define SAADC_OVERSAMPLE_8X          (0x3 << SAADC_OVERSAMPLE_SHIFT)
#  define SAADC_OVERSAMPLE_16X         (0x4 << SAADC_OVERSAMPLE_SHIFT)
#  define SAADC_OVERSAMPLE_32X         (0x5 << SAADC_OVERSAMPLE_SHIFT)
#  define SAADC_OVERSAMPLE_64X         (0x6 << SAADC_OVERSAMPLE_SHIFT)
#  define SAADC_OVERSAMPLE_128X        (0x7 << SAADC_OVERSAMPLE_SHIFT)
#  define SAADC_OVERSAMPLE_256X        (0x8 << SAADC_OVERSAMPLE_SHIFT)

/* SAMPLERATE Register */

#define SAADC_SAMPLERATE_CC_SHIFT      (0)       /* Bits 0-10: Capture and compare value */
#define SAADC_SAMPLERATE_CC_MASK       (0x7ff << SAADC_SAMPLERATE_CC_SHIFT)
#define SAADC_SAMPLERATE_MODE_TASK     (0 << 12) /* Bit 12: Rate is controlled from SAMPLE task */
#define SAADC_SAMPLERATE_MODE_TIMERS   (1 << 12) /* Bit 12: Rate is controlled from local timer */

/* MAXCNT Register */

#define SAADC_MAXCNT_SHIFT             (0)       /* Bits 0-14: Maximum of 16-bit samples written to output buffer */
#define SAADC_MAXCNT_MASK              (0x7fff)

/* AMOUNT Register */

#define SAADC_AMOUNT_SHIFT             (0)       /* Bits 0-14: Number of 16-bit samples written to output buffer */
#define SAADC_AMOUNT_MASK              (0x7fff)

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_SAADC_H */
