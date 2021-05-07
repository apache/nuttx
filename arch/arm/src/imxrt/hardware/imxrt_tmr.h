/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_tmr.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_TMR_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_TMR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define IMXRT_TMR_COMP1_OFFSET               0x0000  /* Timer Channel Compare Register 1 */
#define IMXRT_TMR_COMP2_OFFSET               0x0002  /* Timer Channel Compare Register 2 */
#define IMXRT_TMR_CAPT_OFFSET                0x0004  /* Timer Channel Capture Register */
#define IMXRT_TMR_LOAD_OFFSET                0x0006  /* Timer Channel Load Register */
#define IMXRT_TMR_HOLD_OFFSET                0x0008  /* Timer Channel Hold Register */
#define IMXRT_TMR_CNTR_OFFSET                0x000a  /* Timer Channel Counter Register */
#define IMXRT_TMR_CTRL_OFFSET                0x000c  /* Timer Channel Control Register */
#define IMXRT_TMR_SCTRL_OFFSET               0x000e  /* Timer Channel Status and Control Register */
#define IMXRT_TMR_CMPLD1_OFFSET              0x0010  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR_CMPLD2_OFFSET              0x0012  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR_CSCTRL_OFFSET              0x0014  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR_FILT_OFFSET                0x0016  /* Timer Channel Input Filter Register */
#define IMXRT_TMR_DMA_OFFSET                 0x0018  /* Timer Channel DMA Enable Register */
#define IMXRT_TMR_ENBL_OFFSET                0x001e  /* Timer Channel Enable Register */

#define IMXRT_TMR_CHANNEL_SPACING            0x20    /* Each timer has 4 Channels spaced 0x20 apart */
#define IMXRT_TMR_CH0                        (0 * IMXRT_TMR_CHANNEL_SPACING)
#define IMXRT_TMR_CH1                        (1 * IMXRT_TMR_CHANNEL_SPACING)
#define IMXRT_TMR_CH2                        (2 * IMXRT_TMR_CHANNEL_SPACING)
#define IMXRT_TMR_CH3                        (3 * IMXRT_TMR_CHANNEL_SPACING)

#define IMXRT_TMR_OFFSET(ch,r)               ((r) + (ch))

#define IMXRT_TMR1_COMP10_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR1_COMP20_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR1_CAPT0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR1_LOAD0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR1_HOLD0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR1_CNTR0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR1_CTRL0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR1_SCTRL0_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR1_CMPLD10_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR1_CMPLD20_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR1_CSCTRL0_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR1_FILT0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR1_DMA0_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR1_ENBL_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_ENBL_OFFSET)    /* Timer Channel Enable Register */
#define IMXRT_TMR1_COMP11_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR1_COMP21_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR1_CAPT1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR1_LOAD1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR1_HOLD1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR1_CNTR1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR1_CTRL1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR1_SCTRL1_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR1_CMPLD11_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR1_CMPLD21_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR1_CSCTRL1_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR1_FILT1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR1_DMA1_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR1_COMP12_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR1_COMP22_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR1_CAPT2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR1_LOAD2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR1_HOLD2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR1_CNTR2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR1_CTRL2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR1_SCTRL2_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR1_CMPLD12_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR1_CMPLD22_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR1_CSCTRL2_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR1_FILT2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR1_DMA2_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR1_COMP13_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR1_COMP23_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR1_CAPT3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR1_LOAD3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR1_HOLD3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR1_CNTR3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR1_CTRL3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR1_SCTRL3_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR1_CMPLD13_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR1_CMPLD23_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR1_CSCTRL3_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR1_FILT3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR1_DMA3_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */

/* Timer 2 offset */

#define IMXRT_TMR2_COMP10_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR2_COMP20_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR2_CAPT0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR2_LOAD0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR2_HOLD0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR2_CNTR0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR2_CTRL0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR2_SCTRL0_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR2_CMPLD10_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR2_CMPLD20_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR2_CSCTRL0_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR2_FILT0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR2_DMA0_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR2_ENBL_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_ENBL_OFFSET)    /* Timer Channel Enable Register */
#define IMXRT_TMR2_COMP11_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR2_COMP21_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR2_CAPT1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR2_LOAD1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR2_HOLD1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR2_CNTR1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR2_CTRL1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR2_SCTRL1_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR2_CMPLD11_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR2_CMPLD21_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR2_CSCTRL1_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR2_FILT1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR2_DMA1_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR2_COMP12_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR2_COMP22_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR2_CAPT2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR2_LOAD2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR2_HOLD2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR2_CNTR2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR2_CTRL2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR2_SCTRL2_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR2_CMPLD12_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR2_CMPLD22_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR2_CSCTRL2_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR2_FILT2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR2_DMA2_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR2_COMP13_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR2_COMP23_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR2_CAPT3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR2_LOAD3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR2_HOLD3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR2_CNTR3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR2_CTRL3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR2_SCTRL3_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR2_CMPLD13_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR2_CMPLD23_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR2_CSCTRL3_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR2_FILT3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR2_DMA3_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */

/* Timer 3 offset */

#define IMXRT_TMR3_COMP10_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR3_COMP20_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR3_CAPT0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR3_LOAD0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR3_HOLD0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR3_CNTR0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR3_CTRL0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR3_SCTRL0_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR3_CMPLD10_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR3_CMPLD20_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR3_CSCTRL0_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR3_FILT0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR3_DMA0_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR3_ENBL_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_ENBL_OFFSET)    /* Timer Channel Enable Register */
#define IMXRT_TMR3_COMP11_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR3_COMP21_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR3_CAPT1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR3_LOAD1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR3_HOLD1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR3_CNTR1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR3_CTRL1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR3_SCTRL1_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR3_CMPLD11_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR3_CMPLD21_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR3_CSCTRL1_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR3_FILT1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR3_DMA1_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR3_COMP12_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR3_COMP22_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR3_CAPT2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR3_LOAD2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR3_HOLD2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR3_CNTR2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR3_CTRL2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR3_SCTRL2_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR3_CMPLD12_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR3_CMPLD22_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR3_CSCTRL2_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR3_FILT2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR3_DMA2_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR3_COMP13_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR3_COMP23_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR3_CAPT3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR3_LOAD3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR3_HOLD3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR3_CNTR3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR3_CTRL3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR3_SCTRL3_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR3_CMPLD13_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR3_CMPLD23_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR3_CSCTRL3_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR3_FILT3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR3_DMA3_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */

/* Timer 4 offset */

#define IMXRT_TMR4_COMP10_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR4_COMP20_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR4_CAPT0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR4_LOAD0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR4_HOLD0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR4_CNTR0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR4_CTRL0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR4_SCTRL0_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR4_CMPLD10_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR4_CMPLD20_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR4_CSCTRL0_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR4_FILT0_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR4_DMA0_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR4_ENBL_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH0, IMXRT_TMR_ENBL_OFFSET)    /* Timer Channel Enable Register */
#define IMXRT_TMR4_COMP11_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR4_COMP21_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR4_CAPT1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR4_LOAD1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR4_HOLD1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR4_CNTR1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR4_CTRL1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR4_SCTRL1_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR4_CMPLD11_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR4_CMPLD21_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR4_CSCTRL1_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR4_FILT1_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR4_DMA1_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH1, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR4_COMP12_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR4_COMP22_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR4_CAPT2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR4_LOAD2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR4_HOLD2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR4_CNTR2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR4_CTRL2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR4_SCTRL2_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR4_CMPLD12_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR4_CMPLD22_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR4_CSCTRL2_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR4_FILT2_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR4_DMA2_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH2, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR4_COMP13_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_COMP1_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR4_COMP23_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_COMP2_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR4_CAPT3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CAPT_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR4_LOAD3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_LOAD_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR4_HOLD3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_HOLD_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR4_CNTR3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CNTR_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR4_CTRL3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CTRL_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR4_SCTRL3_OFFSET             IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_SCTRL_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR4_CMPLD13_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CMPLD1_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR4_CMPLD23_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CMPLD2_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR4_CSCTRL3_OFFSET            IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_CSCTRL_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR4_FILT3_OFFSET              IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_FILT_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR4_DMA3_OFFSET               IMXRT_TMR_OFFSET(IMXRT_TMR_CH3, IMXRT_TMR_DMA_OFFSET)     /* Timer Channel DMA Enable Register */

/* Register addresses *******************************************************/

#define IMXRT_TMR1_COMP10                   (IMXRT_QTIMER1_BASE + IMXRT_TMR1_COMP10_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR1_COMP20                   (IMXRT_QTIMER1_BASE + IMXRT_TMR1_COMP20_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR1_CAPT0                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CAPT0_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR1_LOAD0                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_LOAD0_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR1_HOLD0                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_HOLD0_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR1_CNTR0                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CNTR0_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR1_CTRL0                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CTRL0_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR1_SCTRL0                   (IMXRT_QTIMER1_BASE + IMXRT_TMR1_SCTRL0_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR1_CMPLD10                  (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CMPLD10_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR1_CMPLD20                  (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CMPLD20_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR1_CSCTRL0                  (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CSCTRL0_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR1_FILT0                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_FILT0_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR1_DMA0                     (IMXRT_QTIMER1_BASE + IMXRT_TMR1_DMA0_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR1_ENBL                     (IMXRT_QTIMER1_BASE + IMXRT_TMR1_ENBL_OFFSET)     /* Timer Channel Enable Register */
#define IMXRT_TMR1_COMP11                   (IMXRT_QTIMER1_BASE + IMXRT_TMR1_COMP11_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR1_COMP21                   (IMXRT_QTIMER1_BASE + IMXRT_TMR1_COMP21_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR1_CAPT1                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CAPT1_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR1_LOAD1                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_LOAD1_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR1_HOLD1                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_HOLD1_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR1_CNTR1                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CNTR1_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR1_CTRL1                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CTRL1_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR1_SCTRL1                   (IMXRT_QTIMER1_BASE + IMXRT_TMR1_SCTRL1_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR1_CMPLD11                  (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CMPLD11_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR1_CMPLD21                  (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CMPLD21_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR1_CSCTRL1                  (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CSCTRL1_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR1_FILT1                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_FILT1_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR1_DMA1                     (IMXRT_QTIMER1_BASE + IMXRT_TMR1_DMA1_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR1_COMP12                   (IMXRT_QTIMER1_BASE + IMXRT_TMR1_COMP12_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR1_COMP22                   (IMXRT_QTIMER1_BASE + IMXRT_TMR1_COMP22_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR1_CAPT2                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CAPT2_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR1_LOAD2                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_LOAD2_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR1_HOLD2                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_HOLD2_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR1_CNTR2                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CNTR2_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR1_CTRL2                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CTRL2_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR1_SCTRL2                   (IMXRT_QTIMER1_BASE + IMXRT_TMR1_SCTRL2_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR1_CMPLD12                  (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CMPLD12_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR1_CMPLD22                  (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CMPLD22_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR1_CSCTRL2                  (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CSCTRL2_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR1_FILT2                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_FILT2_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR1_DMA2                     (IMXRT_QTIMER1_BASE + IMXRT_TMR1_DMA2_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR1_COMP13                   (IMXRT_QTIMER1_BASE + IMXRT_TMR1_COMP13_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR1_COMP23                   (IMXRT_QTIMER1_BASE + IMXRT_TMR1_COMP23_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR1_CAPT3                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CAPT3_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR1_LOAD3                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_LOAD3_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR1_HOLD3                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_HOLD3_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR1_CNTR3                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CNTR3_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR1_CTRL3                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CTRL3_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR1_SCTRL3                   (IMXRT_QTIMER1_BASE + IMXRT_TMR1_SCTRL3_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR1_CMPLD13                  (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CMPLD13_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR1_CMPLD23                  (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CMPLD23_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR1_CSCTRL3                  (IMXRT_QTIMER1_BASE + IMXRT_TMR1_CSCTRL3_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR1_FILT3                    (IMXRT_QTIMER1_BASE + IMXRT_TMR1_FILT3_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR1_DMA3                     (IMXRT_QTIMER1_BASE + IMXRT_TMR1_DMA3_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR2_COMP10                   (IMXRT_QTIMER2_BASE + IMXRT_TMR2_COMP10_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR2_COMP20                   (IMXRT_QTIMER2_BASE + IMXRT_TMR2_COMP20_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR2_CAPT0                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CAPT0_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR2_LOAD0                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_LOAD0_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR2_HOLD0                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_HOLD0_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR2_CNTR0                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CNTR0_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR2_CTRL0                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CTRL0_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR2_SCTRL0                   (IMXRT_QTIMER2_BASE + IMXRT_TMR2_SCTRL0_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR2_CMPLD10                  (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CMPLD10_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR2_CMPLD20                  (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CMPLD20_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR2_CSCTRL0                  (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CSCTRL0_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR2_FILT0                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_FILT0_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR2_DMA0                     (IMXRT_QTIMER2_BASE + IMXRT_TMR2_DMA0_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR2_ENBL                     (IMXRT_QTIMER2_BASE + IMXRT_TMR2_ENBL_OFFSET)     /* Timer Channel Enable Register */
#define IMXRT_TMR2_COMP11                   (IMXRT_QTIMER2_BASE + IMXRT_TMR2_COMP11_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR2_COMP21                   (IMXRT_QTIMER2_BASE + IMXRT_TMR2_COMP21_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR2_CAPT1                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CAPT1_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR2_LOAD1                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_LOAD1_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR2_HOLD1                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_HOLD1_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR2_CNTR1                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CNTR1_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR2_CTRL1                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CTRL1_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR2_SCTRL1                   (IMXRT_QTIMER2_BASE + IMXRT_TMR2_SCTRL1_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR2_CMPLD11                  (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CMPLD11_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR2_CMPLD21                  (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CMPLD21_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR2_CSCTRL1                  (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CSCTRL1_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR2_FILT1                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_FILT1_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR2_DMA1                     (IMXRT_QTIMER2_BASE + IMXRT_TMR2_DMA1_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR2_COMP12                   (IMXRT_QTIMER2_BASE + IMXRT_TMR2_COMP12_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR2_COMP22                   (IMXRT_QTIMER2_BASE + IMXRT_TMR2_COMP22_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR2_CAPT2                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CAPT2_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR2_LOAD2                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_LOAD2_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR2_HOLD2                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_HOLD2_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR2_CNTR2                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CNTR2_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR2_CTRL2                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CTRL2_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR2_SCTRL2                   (IMXRT_QTIMER2_BASE + IMXRT_TMR2_SCTRL2_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR2_CMPLD12                  (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CMPLD12_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR2_CMPLD22                  (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CMPLD22_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR2_CSCTRL2                  (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CSCTRL2_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR2_FILT2                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_FILT2_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR2_DMA2                     (IMXRT_QTIMER2_BASE + IMXRT_TMR2_DMA2_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR2_COMP13                   (IMXRT_QTIMER2_BASE + IMXRT_TMR2_COMP13_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR2_COMP23                   (IMXRT_QTIMER2_BASE + IMXRT_TMR2_COMP23_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR2_CAPT3                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CAPT3_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR2_LOAD3                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_LOAD3_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR2_HOLD3                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_HOLD3_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR2_CNTR3                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CNTR3_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR2_CTRL3                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CTRL3_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR2_SCTRL3                   (IMXRT_QTIMER2_BASE + IMXRT_TMR2_SCTRL3_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR2_CMPLD13                  (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CMPLD13_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR2_CMPLD23                  (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CMPLD23_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR2_CSCTRL3                  (IMXRT_QTIMER2_BASE + IMXRT_TMR2_CSCTRL3_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR2_FILT3                    (IMXRT_QTIMER2_BASE + IMXRT_TMR2_FILT3_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR2_DMA3                     (IMXRT_QTIMER2_BASE + IMXRT_TMR2_DMA3_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR3_COMP10                   (IMXRT_QTIMER3_BASE + IMXRT_TMR3_COMP10_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR3_COMP20                   (IMXRT_QTIMER3_BASE + IMXRT_TMR3_COMP20_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR3_CAPT0                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CAPT0_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR3_LOAD0                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_LOAD0_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR3_HOLD0                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_HOLD0_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR3_CNTR0                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CNTR0_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR3_CTRL0                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CTRL0_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR3_SCTRL0                   (IMXRT_QTIMER3_BASE + IMXRT_TMR3_SCTRL0_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR3_CMPLD10                  (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CMPLD10_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR3_CMPLD20                  (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CMPLD20_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR3_CSCTRL0                  (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CSCTRL0_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR3_FILT0                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_FILT0_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR3_DMA0                     (IMXRT_QTIMER3_BASE + IMXRT_TMR3_DMA0_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR3_ENBL                     (IMXRT_QTIMER3_BASE + IMXRT_TMR3_ENBL_OFFSET)     /* Timer Channel Enable Register */
#define IMXRT_TMR3_COMP11                   (IMXRT_QTIMER3_BASE + IMXRT_TMR3_COMP11_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR3_COMP21                   (IMXRT_QTIMER3_BASE + IMXRT_TMR3_COMP21_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR3_CAPT1                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CAPT1_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR3_LOAD1                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_LOAD1_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR3_HOLD1                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_HOLD1_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR3_CNTR1                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CNTR1_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR3_CTRL1                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CTRL1_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR3_SCTRL1                   (IMXRT_QTIMER3_BASE + IMXRT_TMR3_SCTRL1_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR3_CMPLD11                  (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CMPLD11_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR3_CMPLD21                  (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CMPLD21_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR3_CSCTRL1                  (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CSCTRL1_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR3_FILT1                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_FILT1_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR3_DMA1                     (IMXRT_QTIMER3_BASE + IMXRT_TMR3_DMA1_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR3_COMP12                   (IMXRT_QTIMER3_BASE + IMXRT_TMR3_COMP12_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR3_COMP22                   (IMXRT_QTIMER3_BASE + IMXRT_TMR3_COMP22_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR3_CAPT2                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CAPT2_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR3_LOAD2                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_LOAD2_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR3_HOLD2                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_HOLD2_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR3_CNTR2                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CNTR2_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR3_CTRL2                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CTRL2_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR3_SCTRL2                   (IMXRT_QTIMER3_BASE + IMXRT_TMR3_SCTRL2_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR3_CMPLD12                  (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CMPLD12_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR3_CMPLD22                  (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CMPLD22_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR3_CSCTRL2                  (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CSCTRL2_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR3_FILT2                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_FILT2_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR3_DMA2                     (IMXRT_QTIMER3_BASE + IMXRT_TMR3_DMA2_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR3_COMP13                   (IMXRT_QTIMER3_BASE + IMXRT_TMR3_COMP13_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR3_COMP23                   (IMXRT_QTIMER3_BASE + IMXRT_TMR3_COMP23_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR3_CAPT3                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CAPT3_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR3_LOAD3                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_LOAD3_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR3_HOLD3                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_HOLD3_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR3_CNTR3                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CNTR3_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR3_CTRL3                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CTRL3_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR3_SCTRL3                   (IMXRT_QTIMER3_BASE + IMXRT_TMR3_SCTRL3_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR3_CMPLD13                  (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CMPLD13_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR3_CMPLD23                  (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CMPLD23_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR3_CSCTRL3                  (IMXRT_QTIMER3_BASE + IMXRT_TMR3_CSCTRL3_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR3_FILT3                    (IMXRT_QTIMER3_BASE + IMXRT_TMR3_FILT3_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR3_DMA3                     (IMXRT_QTIMER3_BASE + IMXRT_TMR3_DMA3_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR4_COMP10                   (IMXRT_QTIMER4_BASE + IMXRT_TMR4_COMP10_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR4_COMP20                   (IMXRT_QTIMER4_BASE + IMXRT_TMR4_COMP20_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR4_CAPT0                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CAPT0_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR4_LOAD0                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_LOAD0_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR4_HOLD0                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_HOLD0_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR4_CNTR0                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CNTR0_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR4_CTRL0                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CTRL0_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR4_SCTRL0                   (IMXRT_QTIMER4_BASE + IMXRT_TMR4_SCTRL0_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR4_CMPLD10                  (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CMPLD10_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR4_CMPLD20                  (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CMPLD20_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR4_CSCTRL0                  (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CSCTRL0_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR4_FILT0                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_FILT0_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR4_DMA0                     (IMXRT_QTIMER4_BASE + IMXRT_TMR4_DMA0_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR4_ENBL                     (IMXRT_QTIMER4_BASE + IMXRT_TMR4_ENBL_OFFSET)     /* Timer Channel Enable Register */
#define IMXRT_TMR4_COMP11                   (IMXRT_QTIMER4_BASE + IMXRT_TMR4_COMP11_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR4_COMP21                   (IMXRT_QTIMER4_BASE + IMXRT_TMR4_COMP21_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR4_CAPT1                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CAPT1_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR4_LOAD1                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_LOAD1_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR4_HOLD1                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_HOLD1_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR4_CNTR1                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CNTR1_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR4_CTRL1                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CTRL1_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR4_SCTRL1                   (IMXRT_QTIMER4_BASE + IMXRT_TMR4_SCTRL1_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR4_CMPLD11                  (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CMPLD11_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR4_CMPLD21                  (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CMPLD21_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR4_CSCTRL1                  (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CSCTRL1_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR4_FILT1                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_FILT1_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR4_DMA1                     (IMXRT_QTIMER4_BASE + IMXRT_TMR4_DMA1_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR4_COMP12                   (IMXRT_QTIMER4_BASE + IMXRT_TMR4_COMP12_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR4_COMP22                   (IMXRT_QTIMER4_BASE + IMXRT_TMR4_COMP22_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR4_CAPT2                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CAPT2_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR4_LOAD2                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_LOAD2_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR4_HOLD2                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_HOLD2_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR4_CNTR2                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CNTR2_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR4_CTRL2                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CTRL2_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR4_SCTRL2                   (IMXRT_QTIMER4_BASE + IMXRT_TMR4_SCTRL2_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR4_CMPLD12                  (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CMPLD12_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR4_CMPLD22                  (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CMPLD22_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR4_CSCTRL2                  (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CSCTRL2_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR4_FILT2                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_FILT2_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR4_DMA2                     (IMXRT_QTIMER4_BASE + IMXRT_TMR4_DMA2_OFFSET)     /* Timer Channel DMA Enable Register */
#define IMXRT_TMR4_COMP13                   (IMXRT_QTIMER4_BASE + IMXRT_TMR4_COMP13_OFFSET)   /* Timer Channel Compare Register 1 */
#define IMXRT_TMR4_COMP23                   (IMXRT_QTIMER4_BASE + IMXRT_TMR4_COMP23_OFFSET)   /* Timer Channel Compare Register 2 */
#define IMXRT_TMR4_CAPT3                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CAPT3_OFFSET)    /* Timer Channel Capture Register */
#define IMXRT_TMR4_LOAD3                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_LOAD3_OFFSET)    /* Timer Channel Load Register */
#define IMXRT_TMR4_HOLD3                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_HOLD3_OFFSET)    /* Timer Channel Hold Register */
#define IMXRT_TMR4_CNTR3                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CNTR3_OFFSET)    /* Timer Channel Counter Register */
#define IMXRT_TMR4_CTRL3                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CTRL3_OFFSET)    /* Timer Channel Control Register */
#define IMXRT_TMR4_SCTRL3                   (IMXRT_QTIMER4_BASE + IMXRT_TMR4_SCTRL3_OFFSET)   /* Timer Channel Status and Control Register */
#define IMXRT_TMR4_CMPLD13                  (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CMPLD13_OFFSET)  /* Timer Channel Comparator Load Register 1 */
#define IMXRT_TMR4_CMPLD23                  (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CMPLD23_OFFSET)  /* Timer Channel Comparator Load Register 2 */
#define IMXRT_TMR4_CSCTRL3                  (IMXRT_QTIMER4_BASE + IMXRT_TMR4_CSCTRL3_OFFSET)  /* Timer Channel Comparator Status and Control Register */
#define IMXRT_TMR4_FILT3                    (IMXRT_QTIMER4_BASE + IMXRT_TMR4_FILT3_OFFSET)    /* Timer Channel Input Filter Register */
#define IMXRT_TMR4_DMA3                     (IMXRT_QTIMER4_BASE + IMXRT_TMR4_DMA3_OFFSET)     /* Timer Channel DMA Enable Register */

/* Register Bit Definitions *************************************************/

/* Timer Channel Control Register */

#define TMR_CTRL_OUTMODE_SHIFT         (0)        /* Bits: 0-2  Output Mode */
#define TMR_CTRL_OUTMODE_MASK          (7 << TMR_CTRL_OUTMODE_SHIFT)
#  define TMR_CTRL_OUTMODE(n)          ((uint32_t)(n) << TMR_CTRL_OUTMODE_SHIFT)
#  define TMR_CTRL_OUTMODE_ON          (0 << TMR_CTRL_OUTMODE_SHIFT)  /* Asserted while counter is active */
#  define TMR_CTRL_OUTMODE_CLEAR       (1 << TMR_CTRL_OUTMODE_SHIFT)  /* Clear OFLAG output on successful compare */
#  define TMR_CTRL_OUTMODE_SET         (2 << TMR_CTRL_OUTMODE_SHIFT)  /* Set OFLAG output on successful compare */
#  define TMR_CTRL_OUTMODE_TOGGLE      (3 << TMR_CTRL_OUTMODE_SHIFT)  /* Toggle OFLAG output on successful compare */
#  define TMR_CTRL_OUTMODE_TOG_ALT     (4 << TMR_CTRL_OUTMODE_SHIFT)  /* Toggle OFLAG output using alternating compare registers */
#  define TMR_CTRL_OUTMODE_SET_CLR     (5 << TMR_CTRL_OUTMODE_SHIFT)  /* Set on compare, cleared on secondary source input edge */
#  define TMR_CTRL_OUTMODE_SET_CLR_ROL (6 << TMR_CTRL_OUTMODE_SHIFT)  /* Set on compare, cleared on counter rollover */
#  define TMR_CTRL_OUTMODE_GATED       (7 << TMR_CTRL_OUTMODE_SHIFT)  /* Enable gated clock output while counter is active */

#define TMR_CTRL_COINIT                (1 << 3)   /* Bit: 3  Co-Channel Initialization */
#define TMR_CTRL_DIR                   (1 << 4)   /* Bit: 4  Count Direction */
#define TMR_CTRL_LENGTH                (1 << 5)   /* Bit: 5  Count Length */
#define TMR_CTRL_ONCE                  (1 << 6)   /* Bit: 6  Count Once */
#define TMR_CTRL_SCS_SHIFT             (7)        /* Bits: 7-8  Secondary Count Source */
#define TMR_CTRL_SCS_MASK              (3 << TMR_CTRL_SCS_SHIFT)
#  define TMR_CTRL_SCS(n)              ((uint32_t)(n) << TMR_CTRL_SCS_SHIFT)
#  define TMR_CTRL_SCS_CNTR0           (0 << TMR_CTRL_SCS_SHIFT)  /* Counter 0 input pin */
#  define TMR_CTRL_SCS_CNTR1           (1 << TMR_CTRL_SCS_SHIFT)  /* Counter 1 input pin */
#  define TMR_CTRL_SCS_CNTR2           (2 << TMR_CTRL_SCS_SHIFT)  /* Counter 2 input pin */
#  define TMR_CTRL_SCS_CNTR3           (3 << TMR_CTRL_SCS_SHIFT)  /* Counter 3 input pin */

#define TMR_CTRL_PCS_SHIFT             (9)        /* Bits: 9-12  Primary Count Source */
#define TMR_CTRL_PCS_MASK              (15 << TMR_CTRL_PCS_SHIFT)
#  define TMR_CTRL_PCS(n)              ((uint32_t)(n) << TMR_CTRL_PCS_SHIFT)
#  define TMR_CTRL_PCS_CNTR0           (0 << TMR_CTRL_PCS_SHIFT)  /* Counter 0 input pin */
#  define TMR_CTRL_PCS_CNTR1           (1 << TMR_CTRL_PCS_SHIFT)  /* Counter 1 input pin */
#  define TMR_CTRL_PCS_CNTR2           (2 << TMR_CTRL_PCS_SHIFT)  /* Counter 2 input pin */
#  define TMR_CTRL_PCS_CNTR3           (3 << TMR_CTRL_PCS_SHIFT)  /* Counter 3 input pin */
#  define TMR_CTRL_PCS_OUT0            (4 << TMR_CTRL_PCS_SHIFT)  /* Counter 0 output */
#  define TMR_CTRL_PCS_OUT1            (5 << TMR_CTRL_PCS_SHIFT)  /* Counter 1 output */
#  define TMR_CTRL_PCS_OUT2            (6 << TMR_CTRL_PCS_SHIFT)  /* Counter 2 output */
#  define TMR_CTRL_PCS_OUT3            (7 << TMR_CTRL_PCS_SHIFT)  /* Counter 3 output */
#  define TMR_CTRL_PCS_DIV1            (8 << TMR_CTRL_PCS_SHIFT)  /* IP bus clock divide by 1 prescaler */
#  define TMR_CTRL_PCS_DIV2            (9 << TMR_CTRL_PCS_SHIFT)  /* IP bus clock divide by 2 prescaler */
#  define TMR_CTRL_PCS_DIV4            (10 << TMR_CTRL_PCS_SHIFT) /* IP bus clock divide by 4 prescaler */
#  define TMR_CTRL_PCS_DIV8            (11 << TMR_CTRL_PCS_SHIFT) /* IP bus clock divide by 8 prescaler */
#  define TMR_CTRL_PCS_DIV16           (12 << TMR_CTRL_PCS_SHIFT) /* IP bus clock divide by 16 prescaler */
#  define TMR_CTRL_PCS_DIV32           (13 << TMR_CTRL_PCS_SHIFT) /* IP bus clock divide by 32 prescaler */
#  define TMR_CTRL_PCS_DIV64           (14 << TMR_CTRL_PCS_SHIFT) /* IP bus clock divide by 64 prescaler */
#  define TMR_CTRL_PCS_DIV128          (15 << TMR_CTRL_PCS_SHIFT) /* IP bus clock divide by 128 prescaler */

#define TMR_CTRL_CM_SHIFT              (13)       /* Bits: 13-15  Count Mode */
#define TMR_CTRL_CM_MASK               (7 << TMR_CTRL_CM_SHIFT)
#  define TMR_CTRL_CM(n)               ((uint32_t)(n) << TMR_CTRL_CM_SHIFT)
#  define TMR_CTRL_CM_MODE0            (0 << TMR_CTRL_CM_SHIFT)  /* No operation */
#  define TMR_CTRL_CM_MODE1            (1 << TMR_CTRL_CM_SHIFT)  /* Count rising edges of primary source */
#  define TMR_CTRL_CM_MODE2            (2 << TMR_CTRL_CM_SHIFT)  /* Count rising and falling edges of primary source */
#  define TMR_CTRL_CM_MODE3            (3 << TMR_CTRL_CM_SHIFT)  /* Count rising edges of primary source while secondary input high active */
#  define TMR_CTRL_CM_MODE4            (4 << TMR_CTRL_CM_SHIFT)  /* Quadrature count mode, uses primary and secondary sources */
#  define TMR_CTRL_CM_MODE5            (5 << TMR_CTRL_CM_SHIFT)  /* Count rising edges of primary source; secondary source specifies direction */
#  define TMR_CTRL_CM_MODE6            (6 << TMR_CTRL_CM_SHIFT)  /* Edge of secondary source triggers primary count until compare */
#  define TMR_CTRL_CM_MODE7            (7 << TMR_CTRL_CM_SHIFT)  /* Cascaded counter mode (up/down)*/

/* Timer Channel Status and Control Register */

#define TMR_SCTRL_OEN                  (1 << 0)   /* Bit: 0  Output Enable */
#define TMR_SCTRL_OPS                  (1 << 1)   /* Bit: 1  Output Polarity Select */
#define TMR_SCTRL_FORCE                (1 << 2)   /* Bit: 2  Force OFLAG Output */
#define TMR_SCTRL_VAL                  (1 << 3)   /* Bit: 3  Forced OFLAG Value */
#define TMR_SCTRL_EEOF                 (1 << 4)   /* Bit: 4  Enable External OFLAG Force */
#define TMR_SCTRL_MSTR                 (1 << 5)   /* Bit: 5  Master Mode */
#define TMR_SCTRL_CAPTURE_MODE_SHIFT   (6)        /* Bits: 6-7  Input Capture Mode */
#define TMR_SCTRL_CAPTURE_MODE_MASK    (3 << TMR_SCTRL_CAPTURE_MODE_SHIFT)
#  define TMR_SCTRL_CAPTURE_MODE(n)    ((uint32_t)(n) << TMR_SCTRL_CAPTURE_MODE_SHIFT)
#  define TMR_SCTRL_CAPTURE_DIS        (0 << TMR_SCTRL_CAPTURE_MODE_SHIFT)  /* Capture function is disabled */
#  define TMR_SCTRL_CAPTURE_RISING     (1 << TMR_SCTRL_CAPTURE_MODE_SHIFT)  /* Load capture register on rising edge (when IPS=0) or falling edge (when IPS=1) of input */
#  define TMR_SCTRL_CAPTURE_FALLING    (2 << TMR_SCTRL_CAPTURE_MODE_SHIFT)  /* Load capture register on falling edge (when IPS=0) or rising edge (when IPS=1) of input */
#  define TMR_SCTRL_CAPTURE_BOTH       (3 << TMR_SCTRL_CAPTURE_MODE_SHIFT)  /* Load capture register on both edges of input */

#define TMR_SCTRL_INPUT                (1 << 8)   /* Bit: 8  External Input Signal */
#define TMR_SCTRL_IPS                  (1 << 9)   /* Bit: 9  Input Polarity Select */
#define TMR_SCTRL_IEFIE                (1 << 10)  /* Bit: 10 Input Edge Flag Interrupt Enable */
#define TMR_SCTRL_IEF                  (1 << 11)  /* Bit: 11 Input Edge Flag */
#define TMR_SCTRL_TOFIE                (1 << 12)  /* Bit: 12 Timer Overflow Flag Interrupt Enable */
#define TMR_SCTRL_TOF                  (1 << 13)  /* Bit: 13 Timer Overflow Flag */
#define TMR_SCTRL_TCFIE                (1 << 14)  /* Bit: 14 Timer Compare Flag Interrupt Enable */
#define TMR_SCTRL_TCF                  (1 << 15)  /* Bit: 15 Timer Compare Flag */

/* Timer Channel Comparator Status and Control Register */

#define TMR_CSCTRL_CL1_SHIFT           (0)        /* Bits: 0-1  Compare Load Control 1 */
#define TMR_CSCTRL_CL1_MASK            (3 << TMR_CSCTRL_CL1_SHIFT)
#  define TMR_CSCTRL_CL1(n)            ((uint32_t)(n) << TMR_CSCTRL_CL1_SHIFT)
#  define TMR_CSCTRL_CL1_DIS           (0 << TMR_CSCTRL_CL1_SHIFT)  /* Never preload */
#  define TMR_CSCTRL_CL1_COMP1         (1 << TMR_CSCTRL_CL1_SHIFT)  /* Load upon successful compare with the value in COMP1 */
#  define TMR_CSCTRL_CL1_COMP2         (2 << TMR_CSCTRL_CL1_SHIFT)  /* Load upon successful compare with the value in COMP2 */

#define TMR_CSCTRL_CL2_SHIFT           (2)        /* Bits: 2-3  Compare Load Control 2 */
#define TMR_CSCTRL_CL2_MASK            (3 << TMR_CSCTRL_CL2_SHIFT)
#  define TMR_CSCTRL_CL2(n)            ((uint32_t)(n) << TMR_CSCTRL_CL2_SHIFT)
#  define TMR_CSCTRL_CL2_DIS           (0 << TMR_CSCTRL_CL2_SHIFT)  /* Never preload */
#  define TMR_CSCTRL_CL2_COMP1         (1 << TMR_CSCTRL_CL2_SHIFT)  /* Load upon successful compare with the value in COMP1 */
#  define TMR_CSCTRL_CL2_COMP2         (2 << TMR_CSCTRL_CL2_SHIFT)  /* Load upon successful compare with the value in COMP2 */

#define TMR_CSCTRL_TCF1                (1 << 4)   /* Bit: 4  Timer Compare 1 Interrupt Flag */
#define TMR_CSCTRL_TCF2                (1 << 5)   /* Bit: 5  Timer Compare 2 Interrupt Flag */
#define TMR_CSCTRL_TCF1EN              (1 << 6)   /* Bit: 6  Timer Compare 1 Interrupt Enable */
#define TMR_CSCTRL_TCF2EN              (1 << 7)   /* Bit: 7  Timer Compare 2 Interrupt Enable */
                                                  /* Bit: 8  This field is reserved. */
#define TMR_CSCTRL_UP                  (1 << 9)   /* Bit: 9  Counting Direction Indicator */
#define TMR_CSCTRL_TCI                 (1 << 10)  /* Bit: 10 Triggered Count Initialization Control */
#define TMR_CSCTRL_ROC                 (1 << 11)  /* Bit: 11 Reload on Capture */
#define TMR_CSCTRL_ALT_LOAD            (1 << 12)  /* Bit: 12 Alternative Load Enable */
#define TMR_CSCTRL_FAULT               (1 << 13)  /* Bit: 13 Fault Enable */
#define TMR_CSCTRL_DBG_EN_SHIFT        (14)       /* Bits: 14-15  Debug Actions Enable */
#define TMR_CSCTRL_DBG_EN_MASK         (3 << TMR_CSCTRL_DBG_EN_SHIFT)
#  define TMR_CSCTRL_DBG_EN(n)         ((uint32_t)(n) << TMR_CSCTRL_DBG_EN_SHIFT)
#  define TMR_CSCTRL_DBG_EN_NORMAL     (0 << TMR_CSCTRL_DBG_EN_SHIFT)  /* Continue with normal operation during debug mode. (default) */
#  define TMR_CSCTRL_DBG_EN_HALT       (1 << TMR_CSCTRL_DBG_EN_SHIFT)  /* Halt TMR counter during debug mode. */
#  define TMR_CSCTRL_DBG_EN_FORCE      (2 << TMR_CSCTRL_DBG_EN_SHIFT)  /* Force TMR output to logic 0 (prior to consideration of SCTRL[OPS]). */
#  define TMR_CSCTRL_DBG_EN_HALT_FORCE (3 << TMR_CSCTRL_DBG_EN_SHIFT)  /* Both halt counter and force output to 0 during debug mode.*/

/* Timer Channel Input Filter Register */

#define TMR_FILT_FILT_PER_SHIFT        (0)        /* Bits: 0-7  Input Filter Sample Period */
#define TMR_FILT_FILT_PER_MASK         (0xff << TMR_FILT_FILT_PER_SHIFT)
#  define TMR_FILT_FILT_PER(n)         ((uint32_t)(n) << TMR_FILT_FILT_PER_SHIFT)
#define TMR_FILT_FILT_CNT_SHIFT        (8)        /* Bits: 8-10  Input Filter Sample Count */
#define TMR_FILT_FILT_CNT_MASK         (7 << TMR_FILT_FILT_CNT_SHIFT)
#  define TMR_FILT_FILT_CNT(n)         ((uint32_t)(n) << TMR_FILT_FILT_CNT_SHIFT)
#  define TMR_FILT_FILT_CNT_3          (0 << TMR_FILT_FILT_CNT_SHIFT)  /* These bits represent the number of consecutive */
#  define TMR_FILT_FILT_CNT_4          (1 << TMR_FILT_FILT_CNT_SHIFT)  /* samples that must agree prior to the input */
#  define TMR_FILT_FILT_CNT_5          (2 << TMR_FILT_FILT_CNT_SHIFT)  /* filter accepting an input transition. A value */
#  define TMR_FILT_FILT_CNT_6          (3 << TMR_FILT_FILT_CNT_SHIFT)  /* of 0x0 represents 3 samples. A value of 0x7 */
#  define TMR_FILT_FILT_CNT_7          (4 << TMR_FILT_FILT_CNT_SHIFT)  /* represents 10 samples. The value */
#  define TMR_FILT_FILT_CNT_8          (5 << TMR_FILT_FILT_CNT_SHIFT)  /* of FILT_CNT affects the input latency. */
#  define TMR_FILT_FILT_CNT_9          (6 << TMR_FILT_FILT_CNT_SHIFT)
#  define TMR_FILT_FILT_CNT_10         (7 << TMR_FILT_FILT_CNT_SHIFT)
                                                  /* Bits: 11-15  Reserved */

/* Timer Channel DMA Enable Register */

#define TMR_DMA_IEFDE                  (1 << 0)   /* Bit: 0  Input Edge Flag DMA Enable */
#define TMR_DMA_CMPLD1DE               (1 << 1)   /* Bit: 1  Comparator Preload Register 1 DMA Enable */
#define TMR_DMA_CMPLD2DE               (1 << 2)   /* Bit: 2  Comparator Preload Register 2 DMA Enable */
                                                  /* Bits: 3-15  Reserved */

/* Timer Channel Enable Register */

#define TMR_ENBL_ENBL_SHIFT            (0)        /* Bits: 0-3  Timer Channel Enable */
#define TMR_ENBL_ENBL_MASK             (15 << TMR_ENBL_ENBL_SHIFT)
#  define TMR_ENBL_ENBL(n)             ((uint32_t)(n) << TMR_ENBL_ENBL_SHIFT)
#  define TMR_ENBL_CHN0                (1 << TMR_ENBL_ENBL_SHIFT)  /* Channel 0 enable */
#  define TMR_ENBL_CHN1                (2 << TMR_ENBL_ENBL_SHIFT)  /* Channel 1 enable */
#  define TMR_ENBL_CHN3                (4 << TMR_ENBL_ENBL_SHIFT)  /* Channel 2 enable */
#  define TMR_ENBL_CHN4                (8 << TMR_ENBL_ENBL_SHIFT)  /* Channel 3 enable */

                                                  /* Bits: 4-15  Reserved */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_TMR_H */
