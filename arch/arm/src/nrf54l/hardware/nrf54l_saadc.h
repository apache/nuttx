/***************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_saadc.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_SAADC_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_SAADC_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* Register offsets for SAADC **********************************************/

#define NRF54L_SAADC_TASKS_START_OFFSET        0x0000                 /* Start the SAADC */
#define NRF54L_SAADC_TASKS_SAMPLE_OFFSET       0x0004                 /* Takes one SAADC sample */
#define NRF54L_SAADC_TASKS_STOP_OFFSET         0x0008                 /* Stop the SAADC */
#define NRF54L_SAADC_TASKS_CALOFFSET_OFFSET    0x000c                 /* Starts offset auto-calibration */
#define NRF54L_SAADC_SUB_START_OFFSET          0x0080                 /* Subscribe START */
#define NRF54L_SAADC_SUB_SAMPLE_OFFSET         0x0084                 /* Subscribe SAMPLE */
#define NRF54L_SAADC_SUB_STOP_OFFSET           0x0088                 /* Subscribe STOP */
#define NRF54L_SAADC_SUB_CALOFFSET_OFFSET      0x008c                 /* Subscribe CALIBRATEOFFSET */
#define NRF54L_SAADC_EVENTS_STARTED_OFFSET     0x0100                 /* The SAADC has started */
#define NRF54L_SAADC_EVENTS_END_OFFSET         0x0104                 /* The SAADC has filled up the result buffer */
#define NRF54L_SAADC_EVENTS_DONE_OFFSET        0x0108                 /* A conversion task has been completed */
#define NRF54L_SAADC_EVENTS_RESDONE_OFFSET     0x010c                 /* Result ready for transfer to RAM */
#define NRF54L_SAADC_EVENTS_CALDONE_OFFSET     0x0110                 /* Calibration is complete */
#define NRF54L_SAADC_EVENTS_STOPPED_OFFSET     0x0114                 /* The SAADC has stopped */
#define NRF54L_SAADC_EVENTS_CHLIMH_OFFSET(x)   (0x118 + ((x) * 0x8))  /* Limit high event for channel x */
#define NRF54L_SAADC_EVENTS_CHLIML_OFFSET(x)   (0x11c + ((x) * 0x8))  /* Limit low event for channel x */
#define NRF54L_SAADC_PUB_STARTED_OFFSET        0x0180                 /* Publish STARTED */
#define NRF54L_SAADC_PUB_END_OFFSET            0x0184                 /* Publish END */
#define NRF54L_SAADC_PUB_DONE_OFFSET           0x0188                 /* Publish DONE */
#define NRF54L_SAADC_PUB_RESDONE_OFFSET        0x018c                 /* Publish RESULTDONE */
#define NRF54L_SAADC_PUB_CALDONE_OFFSET        0x0190                 /* Publish CALIBRATEDONE */
#define NRF54L_SAADC_PUB_STOPPED_OFFSET        0x0194                 /* Publish STOPPED */
#define NRF54L_SAADC_PUB_CHLIMH_OFFSET(x)      (0x198 + ((x) * 0x8))  /* Publish channel high limit */
#define NRF54L_SAADC_PUB_CHLIML_OFFSET(x)      (0x19c + ((x) * 0x8))  /* Publish channel low limit */
#define NRF54L_SAADC_SHORTS_OFFSET             0x0200                 /* Shortcuts */
#define NRF54L_SAADC_INTEN_OFFSET              0x0300                 /* Enable or disable interrupt */
#define NRF54L_SAADC_INTENSET_OFFSET           0x0304                 /* Enable interrupt */
#define NRF54L_SAADC_INTENCLR_OFFSET           0x0308                 /* Disable interrupt */
#define NRF54L_SAADC_STATUS_OFFSET             0x0400                 /* Status */
#define NRF54L_SAADC_LINCALCOEFF_OFFSET(x)     (0x440 + ((x) * 0x4))  /* Linearity calibration coefficient */
#define NRF54L_SAADC_ENABLE_OFFSET             0x0500                 /* Enable or disable SAADC */
#define NRF54L_SAADC_CHPSELP_OFFSET(x)         (0x510 + ((x) * 0x10)) /* Input positive pin for CH[x] */
#define NRF54L_SAADC_CHPSELN_OFFSET(x)         (0x514 + ((x) * 0x10)) /* Input negative pin for CH[x] */
#define NRF54L_SAADC_CHCONFIG_OFFSET(x)        (0x518 + ((x) * 0x10)) /* Input configuration for CH[x] */
#define NRF54L_SAADC_CHLIMIT_OFFSET(x)         (0x51c + ((x) * 0x10)) /* High/low limits for event monitoring of a CH[x] */
#define NRF54L_SAADC_BURST_OFFSET              0x05e8                 /* Global burst control (LM20) */
#define NRF54L_SAADC_RESOLUTION_OFFSET         0x05f0                 /* Resolution configuration */
#define NRF54L_SAADC_OVERSAMPLE_OFFSET         0x05f4                 /* Oversampling configuration */
#define NRF54L_SAADC_SAMPLERATE_OFFSET         0x05f8                 /* Controls normal or continuous sample rate */
#define NRF54L_SAADC_PTR_OFFSET                0x062c                 /* Data pointer */
#define NRF54L_SAADC_MAXCNT_OFFSET             0x0630                 /* Maximum number of bytes */
#define NRF54L_SAADC_AMOUNT_OFFSET             0x0634                 /* Bytes written at END or STOPPED */
#define NRF54L_SAADC_CURRENTAMOUNT_OFFSET      0x0638                 /* Bytes written since START */
#define NRF54L_SAADC_NOISESHAPE_OFFSET         0x0654                 /* Noise shaping mode */

/* Register Bitfield Definitions for SAADC *********************************/

/* SUBSCRIBE/PUBLISH Registers */

#define SAADC_SUBPUB_CHIDX_SHIFT        (0)
#define SAADC_SUBPUB_CHIDX_MASK         (0xff << SAADC_SUBPUB_CHIDX_SHIFT)
#define SAADC_SUBPUB_ENABLE             (0x80000000)

/* SHORTS Register */

#define SAADC_SHORT_DONE_SAMPLE        (1 << 0)
#define SAADC_SHORT_END_START          (1 << 1)

/* INTEN/INTENSET/INTENCLR Register */

#define SAADC_INT_STARTED              (1 << 0)         /* Bit 0: Interrupt for event STARTED */
#define SAADC_INT_END                  (1 << 1)         /* Bit 1: Interrupt for event END */
#define SAADC_INT_DONE                 (1 << 2)         /* Bit 2: Interrupt for event DONE */
#define SAADC_INT_RESDONE              (1 << 3)         /* Bit 3: Interrupt for event RESULTDONE */
#define SAADC_INT_CALDONE              (1 << 4)         /* Bit 4: Interrupt for event CALIBRATEDONE */
#define SAADC_INT_STOPPED              (1 << 5)         /* Bit 5: Interrupt for event STOPPED */
#define SAADC_INT_CHXLIMH(x)           (1 << (2 * (x) + 6))
#define SAADC_INT_CHXLIML(x)           (1 << (2 * (x) + 7))
#define SAADC_INT_ALL                  (0x003fffff)

/* STATUS Register */

#define SAADC_STATUS_READY             (0)       /* Bit 0: SAADC is ready */
#define SAADC_STATUS_BUSY              (1 << 0)  /* Bit 0: SAADC is busy */

/* ENABLE Register */

#define SAADC_ENABLE_DIS               (0)       /* Bit 0: Disable SAADC */
#define SAADC_ENABLE_EN                (1 << 0)  /* Bit 0: Enable SAADC */

/* CH[n] PSELx Register */

#define SAADC_CHPSEL_PIN_SHIFT         (0)       /* Bits 0-4: GPIO pin */
#define SAADC_CHPSEL_PIN_MASK          (0x1f << SAADC_CHPSEL_PIN_SHIFT)
#define SAADC_CHPSEL_PORT_SHIFT        (8)       /* Bits 8-11: GPIO port */
#define SAADC_CHPSEL_PORT_MASK         (0xf << SAADC_CHPSEL_PORT_SHIFT)
#define SAADC_CHPSEL_INTERNAL_SHIFT    (12)      /* Bits 12-13: Internal input */
#define SAADC_CHPSEL_INTERNAL_MASK     (0x3 << SAADC_CHPSEL_INTERNAL_SHIFT)
#  define SAADC_CHPSEL_AVDD            (0x0 << SAADC_CHPSEL_INTERNAL_SHIFT)
#  define SAADC_CHPSEL_DVDD            (0x1 << SAADC_CHPSEL_INTERNAL_SHIFT)
#  define SAADC_CHPSEL_VDD             (0x2 << SAADC_CHPSEL_INTERNAL_SHIFT)
#define SAADC_CHPSEL_CONNECT_MASK      (0xc0000000)
#  define SAADC_CHPSEL_NC              (0x00000000)
#  define SAADC_CHPSEL_ANALOG          (0x40000000)
#  define SAADC_CHPSEL_INTERNAL        (0x80000000)

/* CH[n] CONFIG Register */

#define SAADC_CONFIG_CHOPPING_DIS      (0 << 0)  /* Bit 0: Chopping disabled (LM20) */
#define SAADC_CONFIG_CHOPPING_EN       (1 << 0)  /* Bit 0: Chopping enabled (LM20) */
#define SAADC_CONFIG_GAIN_SHIFT        (8)       /* Bits 8-10: Gain control */
#define SAADC_CONFIG_GAIN_MASK         (0x7 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_2          (0x0 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_1          (0x1 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_2P3        (0x2 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_1P2        (0x3 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_2P5        (0x4 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_1P3        (0x5 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_2P7        (0x6 << SAADC_CONFIG_GAIN_SHIFT)
#  define SAADC_CONFIG_GAIN_1P4        (0x7 << SAADC_CONFIG_GAIN_SHIFT)
#define SAADC_CONFIG_BURS_DIS          (0 << 11) /* Bit 11: Burst disabled (L15) */
#define SAADC_CONFIG_BURS_EN           (1 << 11) /* Bit 11: Burst enabled (L15) */
#define SAADC_CONFIG_REFSEL_INTERNAL   (0 << 12) /* Bit 12: Internal reference (0.9V) */
#define SAADC_CONFIG_REFSEL_EXTERNAL   (1 << 12) /* Bit 12: External reference */
#define SAADC_CONFIG_MODE_SE           (0 << 15) /* Bit 15: Single-ended */
#define SAADC_CONFIG_MODE_DIFF         (1 << 15) /* Bit 15: Differential */
#define SAADC_CONFIG_TACQ_SHIFT        (16)      /* Bits 16-24: Acquisition time */
#define SAADC_CONFIG_TACQ_MASK         (0x1ff << SAADC_CONFIG_TACQ_SHIFT)
#  define SAADC_CONFIG_TACQ_3US        (23 << SAADC_CONFIG_TACQ_SHIFT)
#  define SAADC_CONFIG_TACQ_5US        (39 << SAADC_CONFIG_TACQ_SHIFT)
#  define SAADC_CONFIG_TACQ_10US       (79 << SAADC_CONFIG_TACQ_SHIFT)
#  define SAADC_CONFIG_TACQ_15US       (119 << SAADC_CONFIG_TACQ_SHIFT)
#  define SAADC_CONFIG_TACQ_20US       (159 << SAADC_CONFIG_TACQ_SHIFT)
#  define SAADC_CONFIG_TACQ_40US       (319 << SAADC_CONFIG_TACQ_SHIFT)
#define SAADC_CONFIG_TCONV_SHIFT       (28)      /* Bits 28-30: Conversion time */
#define SAADC_CONFIG_TCONV_MASK        (0x7 << SAADC_CONFIG_TCONV_SHIFT)
#  define SAADC_CONFIG_TCONV_2US       (7 << SAADC_CONFIG_TCONV_SHIFT)

/* CH[n] LIMIT Register */

#define SAADC_CHLIMIT_LOW_SHIFT        (0)       /* Bits 0-15: Low level limit */
#define SAADC_CHLIMIT_LOW_MASK         (0xffff << SAADC_CHLIMIT_LOW_SHIFT)
#define SAADC_CHLIMIT_HIGH_SHIFT       (16)      /* Bits 16-31: High level limit */
#define SAADC_CHLIMIT_HIGH_MASK        (0xffff << SAADC_CHLIMIT_HIGH_SHIFT)

/* BURST Register (LM20) */

#define SAADC_BURST_DIS                (0 << 0)
#define SAADC_BURST_EN                 (1 << 0)

/* RESOLUTION Register */

#define SAADC_RESOLUTION_SHIFT         (0)       /* Bits 0-2: SAADC resolution */
#define SAADC_RESOLUTION_MASK          (0x7 << SAADC_RESOLUTION_SHIFT)
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

#define SAADC_MAXCNT_SHIFT             (0)       /* Bits 0-14: Maximum byte count */
#define SAADC_MAXCNT_MASK              (0x7fff)

/* AMOUNT Register */

#define SAADC_AMOUNT_SHIFT             (0)       /* Bits 0-14: Byte count */
#define SAADC_AMOUNT_MASK              (0x7fff)

/* CURRENTAMOUNT Register */

#define SAADC_CURRENTAMOUNT_MASK       (0x7fff)

/* TRIM.LINCALCOEFF Registers */

#define SAADC_LINCALCOEFF_MASK         (0xffff)

/* NOISESHAPE Register */

#define SAADC_NOISESHAPE_MASK          (0x3)
#  define SAADC_NOISESHAPE_DISABLE     (0x0)
#  define SAADC_NOISESHAPE_NS1         (0x1)
#  define SAADC_NOISESHAPE_NS2         (0x2)
#  define SAADC_NOISESHAPE_NS3         (0x3)      /* LM20 only */

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_SAADC_H */
