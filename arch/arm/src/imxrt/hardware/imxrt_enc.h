/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_enc.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ENC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ENC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define IMXRT_ENC_CTRL_OFFSET           0x0000  /* Control Register Offset */
#define IMXRT_ENC_FILT_OFFSET           0x0002  /* Input Filter Register Offset */
#define IMXRT_ENC_WTR_OFFSET            0x0004  /* Watchdog Timeout Register Offset */
#define IMXRT_ENC_POSD_OFFSET           0x0006  /* Position Difference Counter Register Offset */
#define IMXRT_ENC_POSDH_OFFSET          0x0008  /* Position Difference Hold Register Offset */
#define IMXRT_ENC_REV_OFFSET            0x000A  /* Revolution Counter Register Offset */
#define IMXRT_ENC_REVH_OFFSET           0x000C  /* Revolution Hold Register Offset */
#define IMXRT_ENC_UPOS_OFFSET           0x000E  /* Upper Position Counter Register Offset */
#define IMXRT_ENC_LPOS_OFFSET           0x0010  /* Lower Position Counter Register Offset */
#define IMXRT_ENC_UPOSH_OFFSET          0x0012  /* Upper Position Hold Register Offset */
#define IMXRT_ENC_LPOSH_OFFSET          0x0014  /* Lower Position Hold Register Offset */
#define IMXRT_ENC_UINIT_OFFSET          0x0016  /* Upper Initialization Register Offset */
#define IMXRT_ENC_LINIT_OFFSET          0x0018  /* Lower Initialization Register Offset */
#define IMXRT_ENC_IMR_OFFSET            0x001A  /* Input Monitor Register Offset */
#define IMXRT_ENC_TST_OFFSET            0x001C  /* Test Register Offset */
#define IMXRT_ENC_CTRL2_OFFSET          0x001E  /* Control 2 Register Offset */
#define IMXRT_ENC_UMOD_OFFSET           0x0020  /* Upper Modulus Register Offset */
#define IMXRT_ENC_LMOD_OFFSET           0x0022  /* Lower Modulus Register Offset */
#define IMXRT_ENC_UCOMP_OFFSET          0x0024  /* Upper Compare Register Offset */
#define IMXRT_ENC_LCOMP_OFFSET          0x0026  /* Lower Compare Register Offset */

/* Register Addresses *******************************************************/

/* ENC1 Registers */

#define IMXRT_ENC1_CTRL                 (IMXRT_ENC1_BASE + IMXRT_ENC_CTRL_OFFSET)   /* Control Register */
#define IMXRT_ENC1_FILT                 (IMXRT_ENC1_BASE + IMXRT_ENC_FILT_OFFSET)   /* Input Filter Register */
#define IMXRT_ENC1_WTR                  (IMXRT_ENC1_BASE + IMXRT_ENC_WTR_OFFSET)    /* Watchdog Timeout Register */
#define IMXRT_ENC1_POSD                 (IMXRT_ENC1_BASE + IMXRT_ENC_POSD_OFFSET)   /* Position Difference Counter Register */
#define IMXRT_ENC1_POSDH                (IMXRT_ENC1_BASE + IMXRT_ENC_POSDH_OFFSET)  /* Position Difference Hold Register */
#define IMXRT_ENC1_REV                  (IMXRT_ENC1_BASE + IMXRT_ENC_REV_OFFSET)    /* Revolution Counter Register */
#define IMXRT_ENC1_REVH                 (IMXRT_ENC1_BASE + IMXRT_ENC_REVH_OFFSET)   /* Revolution Hold Register */
#define IMXRT_ENC1_UPOS                 (IMXRT_ENC1_BASE + IMXRT_ENC_UPOS_OFFSET)   /* Upper Position Counter Register */
#define IMXRT_ENC1_LPOS                 (IMXRT_ENC1_BASE + IMXRT_ENC_LPOS_OFFSET)   /* Lower Position Counter Register */
#define IMXRT_ENC1_UPOSH                (IMXRT_ENC1_BASE + IMXRT_ENC_UPOSH_OFFSET)  /* Upper Position Hold Register */
#define IMXRT_ENC1_LPOSH                (IMXRT_ENC1_BASE + IMXRT_ENC_LPOSH_OFFSET)  /* Lower Position Hold Register */
#define IMXRT_ENC1_UINIT                (IMXRT_ENC1_BASE + IMXRT_ENC_UINIT_OFFSET)  /* Upper Initialization Register */
#define IMXRT_ENC1_LINIT                (IMXRT_ENC1_BASE + IMXRT_ENC_LINIT_OFFSET)  /* Lower Initialization Register */
#define IMXRT_ENC1_IMR                  (IMXRT_ENC1_BASE + IMXRT_ENC_IMR_OFFSET)    /* Input Monitor Register */
#define IMXRT_ENC1_TST                  (IMXRT_ENC1_BASE + IMXRT_ENC_TST_OFFSET)    /* Test Register */
#define IMXRT_ENC1_CTRL2                (IMXRT_ENC1_BASE + IMXRT_ENC_CTRL2_OFFSET)  /* Control 2 Register Offset */
#define IMXRT_ENC1_UMOD                 (IMXRT_ENC1_BASE + IMXRT_ENC_UMOD_OFFSET)   /* Upper Modulus Register */
#define IMXRT_ENC1_LMOD                 (IMXRT_ENC1_BASE + IMXRT_ENC_LMOD_OFFSET)   /* Lower Modulus Register */
#define IMXRT_ENC1_UCOMP                (IMXRT_ENC1_BASE + IMXRT_ENC_UCOMP_OFFSET)  /* Upper Compare Register */
#define IMXRT_ENC1_LCOMP                (IMXRT_ENC1_BASE + IMXRT_ENC_LCOMP_OFFSET)  /* Lower Compare Register */

/* ENC2 Registers */

#define IMXRT_ENC2_CTRL                 (IMXRT_ENC2_BASE + IMXRT_ENC_CTRL_OFFSET)   /* Control Register */
#define IMXRT_ENC2_FILT                 (IMXRT_ENC2_BASE + IMXRT_ENC_FILT_OFFSET)   /* Input Filter Register */
#define IMXRT_ENC2_WTR                  (IMXRT_ENC2_BASE + IMXRT_ENC_WTR_OFFSET)    /* Watchdog Timeout Register */
#define IMXRT_ENC2_POSD                 (IMXRT_ENC2_BASE + IMXRT_ENC_POSD_OFFSET)   /* Position Difference Counter Register */
#define IMXRT_ENC2_POSDH                (IMXRT_ENC2_BASE + IMXRT_ENC_POSDH_OFFSET)  /* Position Difference Hold Register */
#define IMXRT_ENC2_REV                  (IMXRT_ENC2_BASE + IMXRT_ENC_REV_OFFSET)    /* Revolution Counter Register */
#define IMXRT_ENC2_REVH                 (IMXRT_ENC2_BASE + IMXRT_ENC_REVH_OFFSET)   /* Revolution Hold Register */
#define IMXRT_ENC2_UPOS                 (IMXRT_ENC2_BASE + IMXRT_ENC_UPOS_OFFSET)   /* Upper Position Counter Register */
#define IMXRT_ENC2_LPOS                 (IMXRT_ENC2_BASE + IMXRT_ENC_LPOS_OFFSET)   /* Lower Position Counter Register */
#define IMXRT_ENC2_UPOSH                (IMXRT_ENC2_BASE + IMXRT_ENC_UPOSH_OFFSET)  /* Upper Position Hold Register */
#define IMXRT_ENC2_LPOSH                (IMXRT_ENC2_BASE + IMXRT_ENC_LPOSH_OFFSET)  /* Lower Position Hold Register */
#define IMXRT_ENC2_UINIT                (IMXRT_ENC2_BASE + IMXRT_ENC_UINIT_OFFSET)  /* Upper Initialization Register */
#define IMXRT_ENC2_LINIT                (IMXRT_ENC2_BASE + IMXRT_ENC_LINIT_OFFSET)  /* Lower Initialization Register */
#define IMXRT_ENC2_IMR                  (IMXRT_ENC2_BASE + IMXRT_ENC_IMR_OFFSET)    /* Input Monitor Register */
#define IMXRT_ENC2_TST                  (IMXRT_ENC2_BASE + IMXRT_ENC_TST_OFFSET)    /* Test Register */
#define IMXRT_ENC2_CTRL2                (IMXRT_ENC2_BASE + IMXRT_ENC_CTRL2_OFFSET)  /* Control 2 Register Offset */
#define IMXRT_ENC2_UMOD                 (IMXRT_ENC2_BASE + IMXRT_ENC_UMOD_OFFSET)   /* Upper Modulus Register */
#define IMXRT_ENC2_LMOD                 (IMXRT_ENC2_BASE + IMXRT_ENC_LMOD_OFFSET)   /* Lower Modulus Register */
#define IMXRT_ENC2_UCOMP                (IMXRT_ENC2_BASE + IMXRT_ENC_UCOMP_OFFSET)  /* Upper Compare Register */
#define IMXRT_ENC2_LCOMP                (IMXRT_ENC2_BASE + IMXRT_ENC_LCOMP_OFFSET)  /* Lower Compare Register */

/* ENC3 Registers */

#define IMXRT_ENC3_CTRL                 (IMXRT_ENC3_BASE + IMXRT_ENC_CTRL_OFFSET)   /* Control Register */
#define IMXRT_ENC3_FILT                 (IMXRT_ENC3_BASE + IMXRT_ENC_FILT_OFFSET)   /* Input Filter Register */
#define IMXRT_ENC3_WTR                  (IMXRT_ENC3_BASE + IMXRT_ENC_WTR_OFFSET)    /* Watchdog Timeout Register */
#define IMXRT_ENC3_POSD                 (IMXRT_ENC3_BASE + IMXRT_ENC_POSD_OFFSET)   /* Position Difference Counter Register */
#define IMXRT_ENC3_POSDH                (IMXRT_ENC3_BASE + IMXRT_ENC_POSDH_OFFSET)  /* Position Difference Hold Register */
#define IMXRT_ENC3_REV                  (IMXRT_ENC3_BASE + IMXRT_ENC_REV_OFFSET)    /* Revolution Counter Register */
#define IMXRT_ENC3_REVH                 (IMXRT_ENC3_BASE + IMXRT_ENC_REVH_OFFSET)   /* Revolution Hold Register */
#define IMXRT_ENC3_UPOS                 (IMXRT_ENC3_BASE + IMXRT_ENC_UPOS_OFFSET)   /* Upper Position Counter Register */
#define IMXRT_ENC3_LPOS                 (IMXRT_ENC3_BASE + IMXRT_ENC_LPOS_OFFSET)   /* Lower Position Counter Register */
#define IMXRT_ENC3_UPOSH                (IMXRT_ENC3_BASE + IMXRT_ENC_UPOSH_OFFSET)  /* Upper Position Hold Register */
#define IMXRT_ENC3_LPOSH                (IMXRT_ENC3_BASE + IMXRT_ENC_LPOSH_OFFSET)  /* Lower Position Hold Register */
#define IMXRT_ENC3_UINIT                (IMXRT_ENC3_BASE + IMXRT_ENC_UINIT_OFFSET)  /* Upper Initialization Register */
#define IMXRT_ENC3_LINIT                (IMXRT_ENC3_BASE + IMXRT_ENC_LINIT_OFFSET)  /* Lower Initialization Register */
#define IMXRT_ENC3_IMR                  (IMXRT_ENC3_BASE + IMXRT_ENC_IMR_OFFSET)    /* Input Monitor Register */
#define IMXRT_ENC3_TST                  (IMXRT_ENC3_BASE + IMXRT_ENC_TST_OFFSET)    /* Test Register */
#define IMXRT_ENC3_CTRL2                (IMXRT_ENC3_BASE + IMXRT_ENC_CTRL2_OFFSET)  /* Control 2 Register Offset */
#define IMXRT_ENC3_UMOD                 (IMXRT_ENC3_BASE + IMXRT_ENC_UMOD_OFFSET)   /* Upper Modulus Register */
#define IMXRT_ENC3_LMOD                 (IMXRT_ENC3_BASE + IMXRT_ENC_LMOD_OFFSET)   /* Lower Modulus Register */
#define IMXRT_ENC3_UCOMP                (IMXRT_ENC3_BASE + IMXRT_ENC_UCOMP_OFFSET)  /* Upper Compare Register */
#define IMXRT_ENC3_LCOMP                (IMXRT_ENC3_BASE + IMXRT_ENC_LCOMP_OFFSET)  /* Lower Compare Register */

/* ENC4 Registers */

#define IMXRT_ENC4_CTRL                 (IMXRT_ENC4_BASE + IMXRT_ENC_CTRL_OFFSET)   /* Control Register */
#define IMXRT_ENC4_FILT                 (IMXRT_ENC4_BASE + IMXRT_ENC_FILT_OFFSET)   /* Input Filter Register */
#define IMXRT_ENC4_WTR                  (IMXRT_ENC4_BASE + IMXRT_ENC_WTR_OFFSET)    /* Watchdog Timeout Register */
#define IMXRT_ENC4_POSD                 (IMXRT_ENC4_BASE + IMXRT_ENC_POSD_OFFSET)   /* Position Difference Counter Register */
#define IMXRT_ENC4_POSDH                (IMXRT_ENC4_BASE + IMXRT_ENC_POSDH_OFFSET)  /* Position Difference Hold Register */
#define IMXRT_ENC4_REV                  (IMXRT_ENC4_BASE + IMXRT_ENC_REV_OFFSET)    /* Revolution Counter Register */
#define IMXRT_ENC4_REVH                 (IMXRT_ENC4_BASE + IMXRT_ENC_REVH_OFFSET)   /* Revolution Hold Register */
#define IMXRT_ENC4_UPOS                 (IMXRT_ENC4_BASE + IMXRT_ENC_UPOS_OFFSET)   /* Upper Position Counter Register */
#define IMXRT_ENC4_LPOS                 (IMXRT_ENC4_BASE + IMXRT_ENC_LPOS_OFFSET)   /* Lower Position Counter Register */
#define IMXRT_ENC4_UPOSH                (IMXRT_ENC4_BASE + IMXRT_ENC_UPOSH_OFFSET)  /* Upper Position Hold Register */
#define IMXRT_ENC4_LPOSH                (IMXRT_ENC4_BASE + IMXRT_ENC_LPOSH_OFFSET)  /* Lower Position Hold Register */
#define IMXRT_ENC4_UINIT                (IMXRT_ENC4_BASE + IMXRT_ENC_UINIT_OFFSET)  /* Upper Initialization Register */
#define IMXRT_ENC4_LINIT                (IMXRT_ENC4_BASE + IMXRT_ENC_LINIT_OFFSET)  /* Lower Initialization Register */
#define IMXRT_ENC4_IMR                  (IMXRT_ENC4_BASE + IMXRT_ENC_IMR_OFFSET)    /* Input Monitor Register */
#define IMXRT_ENC4_TST                  (IMXRT_ENC4_BASE + IMXRT_ENC_TST_OFFSET)    /* Test Register */
#define IMXRT_ENC4_CTRL2                (IMXRT_ENC4_BASE + IMXRT_ENC_CTRL2_OFFSET)  /* Control 2 Register Offset */
#define IMXRT_ENC4_UMOD                 (IMXRT_ENC4_BASE + IMXRT_ENC_UMOD_OFFSET)   /* Upper Modulus Register */
#define IMXRT_ENC4_LMOD                 (IMXRT_ENC4_BASE + IMXRT_ENC_LMOD_OFFSET)   /* Lower Modulus Register */
#define IMXRT_ENC4_UCOMP                (IMXRT_ENC4_BASE + IMXRT_ENC_UCOMP_OFFSET)  /* Upper Compare Register */
#define IMXRT_ENC4_LCOMP                (IMXRT_ENC4_BASE + IMXRT_ENC_LCOMP_OFFSET)  /* Lower Compare Register */

/* Register Bit Definitions *************************************************/

/* Control Register */

#define ENC_CTRL_CMPIE                  (1 << 0)    /* Bit 0: Compare Interrupt Enable */
#define ENC_CTRL_CMPIRQ                 (1 << 1)    /* Bit 1: Compare Interrupt Request */
#define ENC_CTRL_WDE                    (1 << 2)    /* Bit 2: Watchdog Enable */
#define ENC_CTRL_DIE                    (1 << 3)    /* Bit 3: Watchdog Timeout Interrupt Enable */
#define ENC_CTRL_DIRQ                   (1 << 4)    /* Bit 4: Watchdog Timeout Interrupt Request */
#define ENC_CTRL_XNE                    (1 << 5)    /* Bit 5: Use Negative Edge of INDEX Pulse */
#define ENC_CTRL_XIP                    (1 << 6)    /* Bit 6: INDEX Triggered Initialization of Position Counters */
#define ENC_CTRL_XIE                    (1 << 7)    /* Bit 7: INDEX Pulse Interrupt Enable */
#define ENC_CTRL_XIRQ                   (1 << 8)    /* Bit 8: INDEX Pulse Interrupt Request */
#define ENC_CTRL_PH1                    (1 << 9)    /* Bit 9: Signal Phase Count Mode */
#define ENC_CTRL_REV                    (1 << 10)   /* Bit 10: Reverse Direction of Counting */
#define ENC_CTRL_SWIP                   (1 << 11)   /* Bit 11: Software-Triggered Initialization of Position Counters */
#define ENC_CTRL_HNE                    (1 << 12)   /* Bit 12: Use Negative Edge of HOME Input */
#define ENC_CTRL_HIP                    (1 << 13)   /* Bit 13: HOME Initializes Position Counters Enable */
#define ENC_CTRL_HIE                    (1 << 14)   /* Bit 14: HOME Interrupt Enable */
#define ENC_CTRL_HIRQ                   (1 << 15)   /* Bit 15: HOME Signal Transition Interrupt Request */

/* Input Filter Register */

#define ENC_FILT_PER_SHIFT              (0)         /* Bits 0-7: Input Filter Sample Period */
#define ENC_FILT_PER_MASK               (0xff << ENC_FILT_PER_SHIFT)
#define ENC_FILT_CNT_SHIFT              (8)         /* Bits 8-10: Input Filter Sample Count */
#define ENC_FILT_CNT_MASK               (0x7 << ENC_FILT_CNT_SHIFT)

/* Input Monitor Register */

#define ENC_IMR_HOME                    (1 << 0)    /* Bit 0: Raw HOME input */
#define ENC_IMR_INDEX                   (1 << 1)    /* Bit 1: Raw INDEX input */
#define ENC_IMR_PHB                     (1 << 2)    /* Bit 2: Raw PHASEB input */
#define ENC_IMR_PHA                     (1 << 3)    /* Bit 3: Raw PHASEA input */
#define ENC_IMR_FHOM                    (1 << 4)    /* Bit 4: Filtered HOME input */
#define ENC_IMR_FIND                    (1 << 5)    /* Bit 5: Filtered INDEX input */
#define ENC_IMR_FPHB                    (1 << 6)    /* Bit 6: Filtered PHASEB input */
#define ENC_IMR_FPHA                    (1 << 7)    /* Bit 7: Filtered PHASEA input */
                                                    /* Bits 8-15: Reserved */

/* Test Register */

#define ENC_TST_COUNT_SHIFT             (0)         /* Bits 0-7: # of quadrature advances to generate */
#define ENC_TST_COUNT_MASK              (0xff << ENC_TST_COUNT_SHIFT)
#define ENC_TST_PERIOD_SHIFT            (8)         /* Bits 8-12: Period of Quadrature Phase */
#define ENC_TST_PERIOD_MASK             (0x1F << ENC_TST_PERIOD_SHIFT)
#define ENC_TST_QDN                     (1 << 13)   /* Bit 13: Generate Negative Quadrature Decoder Signal */
#define ENC_TST_TCE                     (1 << 14)   /* Bit 14: Test Counter Enable */
#define ENC_TST_TEN                     (1 << 15)   /* Bit 15: Test Mode Enable */

/* Control 2 Register Offset */

#define ENC_CTRL2_UPDHLD                (1 << 0)    /* Bit 0: Enable TRIGGER to update Hold Registers */
#define ENC_CTRL2_UPDPOS                (1 << 1)    /* Bit 1: Enable TRIGGER to clear Position, Position Difference, and Revolution Registers */
#define ENC_CTRL2_MOD                   (1 << 2)    /* Bit 2: Enable Modulo Counting */
#define ENC_CTRL2_DIR                   (1 << 3)    /* Bit 3: Count Direction Flag */
#define ENC_CTRL2_RUIE                  (1 << 4)    /* Bit 4: Roll-under Interrupt Enable */
#define ENC_CTRL2_RUIRQ                 (1 << 5)    /* Bit 5: Roll-under Interrupt Request */
#define ENC_CTRL2_ROIE                  (1 << 6)    /* Bit 6: Roll-over Interrupt Enable */
#define ENC_CTRL2_ROIRQ                 (1 << 7)    /* Bit 7: Roll-over Interrupt Request */
#define ENC_CTRL2_REVMOD                (1 << 8)    /* Bit 8: Revolution Counter Modulus Enable */
#define ENC_CTRL2_OUTCTL                (1 << 9)    /* Bit 9: POSMATCH Output Control */
                                                    /* Bits 10-15: Reserved */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ENC_H */
