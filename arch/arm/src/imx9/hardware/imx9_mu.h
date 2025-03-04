/****************************************************************************
 * arch/arm/src/imx9/hardware/imx9_mu.h
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets */

#define IMX9_MU_VER_OFFSET  0x0000 /* Version ID */
#define IMX9_MU_PAR_OFFSET  0x0004 /* Parameter */
#define IMX9_MU_CR_OFFSET   0x0008 /* Control */
#define IMX9_MU_SR_OFFSET   0x000c /* Status */
#define IMX9_MU_FCR_OFFSET  0x0100 /* Flag Control */
#define IMX9_MU_FSR_OFFSET  0x0104 /* Flag Status */
#define IMX9_MU_GIER_OFFSET 0x0110 /* General-Purpose Interrupt Enable */
#define IMX9_MU_GCR_OFFSET  0x0114 /* General-Purpose Control */
#define IMX9_MU_GSR_OFFSET  0x0118 /* General-purpose Status */
#define IMX9_MU_TCR_OFFSET  0x0120 /* Transmit Control */
#define IMX9_MU_TSR_OFFSET  0x0124 /* Transmit Status */
#define IMX9_MU_RCR_OFFSET  0x0128 /* Receive Control */
#define IMX9_MU_RSR_OFFSET  0x012c /* Receive Status */
#define IMX9_MU_TR1_OFFSET  0x0200 /* Transmit */
#define IMX9_MU_TR2_OFFSET  0x0200 /* Transmit */
#define IMX9_MU_TR3_OFFSET  0x0200 /* Transmit */
#define IMX9_MU_TR4_OFFSET  0x0200 /* Transmit */
#define IMX9_MU_RR1_OFFSET  0x0280 /* Receive */
#define IMX9_MU_RR2_OFFSET  0x0280 /* Receive */
#define IMX9_MU_RR3_OFFSET  0x0280 /* Receive */
#define IMX9_MU_RR4_OFFSET  0x0280 /* Receive */

/* Register macros */

#define IMX9_MU_VER(n)  ((n) + IMX9_MU_VER_OFFSET)  /* Version ID */
#define IMX9_MU_PAR(n)  ((n) + IMX9_MU_PAR_OFFSET)  /* Parameter */
#define IMX9_MU_CR(n)   ((n) + IMX9_MU_CR_OFFSET)   /* Control */
#define IMX9_MU_SR(n)   ((n) + IMX9_MU_SR_OFFSET)   /* Status */
#define IMX9_MU_FCR(n)  ((n) + IMX9_MU_FCR_OFFSET)  /* Flag Control */
#define IMX9_MU_FSR(n)  ((n) + IMX9_MU_FSR_OFFSET)  /* Flag Status */
#define IMX9_MU_GIER(n) ((n) + IMX9_MU_GIER_OFFSET) /* General-Purpose Interrupt Enable */
#define IMX9_MU_GCR(n)  ((n) + IMX9_MU_GCR_OFFSET)  /* General-Purpose Control */
#define IMX9_MU_GSR(n)  ((n) + IMX9_MU_GSR_OFFSET)  /* General-purpose Status */
#define IMX9_MU_TCR(n)  ((n) + IMX9_MU_TCR_OFFSET)  /* Transmit Control */
#define IMX9_MU_TSR(n)  ((n) + IMX9_MU_TSR_OFFSET)  /* Transmit Status */
#define IMX9_MU_RCR(n)  ((n) + IMX9_MU_RCR_OFFSET)  /* Receive Control */
#define IMX9_MU_RSR(n)  ((n) + IMX9_MU_RSR_OFFSET)  /* Receive Status */
#define IMX9_MU_TR1(n)  ((n) + IMX9_MU_TR1_OFFSET)  /* Transmit */
#define IMX9_MU_TR2(n)  ((n) + IMX9_MU_TR2_OFFSET)  /* Transmit */
#define IMX9_MU_TR3(n)  ((n) + IMX9_MU_TR3_OFFSET)  /* Transmit */
#define IMX9_MU_TR4(n)  ((n) + IMX9_MU_TR4_OFFSET)  /* Transmit */
#define IMX9_MU_RR1(n)  ((n) + IMX9_MU_RR1_OFFSET)  /* Receive */
#define IMX9_MU_RR2(n)  ((n) + IMX9_MU_RR2_OFFSET)  /* Receive */
#define IMX9_MU_RR3(n)  ((n) + IMX9_MU_RR3_OFFSET)  /* Receive */
#define IMX9_MU_RR4(n)  ((n) + IMX9_MU_RR4_OFFSET)  /* Receive */

/* Field definitions */

/* VER register */

#define IMX9_MU_VER_FEATURE_SHIFT 0      /* Feature Set Number */
#define IMX9_MU_VER_FEATURE_MASK  0xffff /* Feature Set Number */

#define IMX9_MU_VER_MINOR_SHIFT 16   /* Minor Version Number */
#define IMX9_MU_VER_MINOR_MASK  0xff /* Minor Version Number */

#define IMX9_MU_VER_MAJOR_SHIFT 24   /* Major Version Number */
#define IMX9_MU_VER_MAJOR_MASK  0xff /* Major Version Number */

/* PAR register */

#define IMX9_MU_PAR_TR_NUM_SHIFT 0    /* Transmit Register Number */
#define IMX9_MU_PAR_TR_NUM_MASK  0xff /* Transmit Register Number */

#define IMX9_MU_PAR_RR_NUM_SHIFT 8    /* Receive Register Number */
#define IMX9_MU_PAR_RR_NUM_MASK  0xff /* Receive Register Number */

#define IMX9_MU_PAR_GIR_NUM_SHIFT 16   /* General-Purpose Interrupt Request Number */
#define IMX9_MU_PAR_GIR_NUM_MASK  0xff /* General-Purpose Interrupt Request Number */

#define IMX9_MU_PAR_FLAG_WIDTH_SHIFT 24   /* Flag Width */
#define IMX9_MU_PAR_FLAG_WIDTH_MASK  0xff /* Flag Width */

/* CR register */

#define IMX9_MU_CR_MUR_SHIFT 0                           /* MU Reset */
#define IMX9_MU_CR_MUR_FLAG  (1 << IMX9_MU_CR_MUR_SHIFT) /* MU Reset */

#define IMX9_MU_CR_MURIE_SHIFT 1                             /* MUA Reset Interrupt Enable */
#define IMX9_MU_CR_MURIE_FLAG  (1 << IMX9_MU_CR_MURIE_SHIFT) /* MUA Reset Interrupt Enable */

/* SR register */

#define IMX9_MU_SR_MURS_SHIFT 0                            /* MUA and MUB Reset State */
#define IMX9_MU_SR_MURS_FLAG  (1 << IMX9_MU_SR_MURS_SHIFT) /* MUA and MUB Reset State */

#define IMX9_MU_SR_MURIP_SHIFT 1                             /* MU Reset Interrupt Pending Flag */
#define IMX9_MU_SR_MURIP_FLAG  (1 << IMX9_MU_SR_MURIP_SHIFT) /* MU Reset Interrupt Pending Flag */

#define IMX9_MU_SR_EP_SHIFT 2                          /* MUA Side Event Pending */
#define IMX9_MU_SR_EP_FLAG  (1 << IMX9_MU_SR_EP_SHIFT) /* MUA Side Event Pending */

#define IMX9_MU_SR_FUP_SHIFT 3                           /* MUA Flag Update Pending */
#define IMX9_MU_SR_FUP_FLAG  (1 << IMX9_MU_SR_FUP_SHIFT) /* MUA Flag Update Pending */

#define IMX9_MU_SR_GIRP_SHIFT 4                            /* MUA General-Purpose Interrupt Pending */
#define IMX9_MU_SR_GIRP_FLAG  (1 << IMX9_MU_SR_GIRP_SHIFT) /* MUA General-Purpose Interrupt Pending */

#define IMX9_MU_SR_TEP_SHIFT 5                           /* MUA Transmit Empty Pending */
#define IMX9_MU_SR_TEP_FLAG  (1 << IMX9_MU_SR_TEP_SHIFT) /* MUA Transmit Empty Pending */

#define IMX9_MU_SR_RFP_SHIFT 6                           /* MUA Receive Full Pending */
#define IMX9_MU_SR_RFP_FLAG  (1 << IMX9_MU_SR_RFP_SHIFT) /* MUA Receive Full Pending */

/* FCR register */

#define IMX9_MU_FCR_F0_SHIFT 0                           /* MUA to MUB Flag */
#define IMX9_MU_FCR_F0_FLAG  (1 << IMX9_MU_FCR_F0_SHIFT) /* MUA to MUB Flag */

#define IMX9_MU_FCR_F1_SHIFT 1                           /* MUA to MUB Flag */
#define IMX9_MU_FCR_F1_FLAG  (1 << IMX9_MU_FCR_F1_SHIFT) /* MUA to MUB Flag */

#define IMX9_MU_FCR_F2_SHIFT 2                           /* MUA to MUB Flag */
#define IMX9_MU_FCR_F2_FLAG  (1 << IMX9_MU_FCR_F2_SHIFT) /* MUA to MUB Flag */

/* FSR register */

#define IMX9_MU_FSR_F0_SHIFT 0                           /* MUB to MUA-Side Flag */
#define IMX9_MU_FSR_F0_FLAG  (1 << IMX9_MU_FSR_F0_SHIFT) /* MUB to MUA-Side Flag */

#define IMX9_MU_FSR_F1_SHIFT 1                           /* MUB to MUA-Side Flag */
#define IMX9_MU_FSR_F1_FLAG  (1 << IMX9_MU_FSR_F1_SHIFT) /* MUB to MUA-Side Flag */

#define IMX9_MU_FSR_F2_SHIFT 2                           /* MUB to MUA-Side Flag */
#define IMX9_MU_FSR_F2_FLAG  (1 << IMX9_MU_FSR_F2_SHIFT) /* MUB to MUA-Side Flag */

/* GIER register */

#define IMX9_MU_GIER_GIE0_SHIFT 0                              /* MUA General-purpose Interrupt Enable */
#define IMX9_MU_GIER_GIE0_FLAG  (1 << IMX9_MU_GIER_GIE0_SHIFT) /* MUA General-purpose Interrupt Enable */

#define IMX9_MU_GIER_GIE1_SHIFT 1                              /* MUA General-purpose Interrupt Enable */
#define IMX9_MU_GIER_GIE1_FLAG  (1 << IMX9_MU_GIER_GIE1_SHIFT) /* MUA General-purpose Interrupt Enable */

#define IMX9_MU_GIER_GIE2_SHIFT 2                              /* MUA General-purpose Interrupt Enable */
#define IMX9_MU_GIER_GIE2_FLAG  (1 << IMX9_MU_GIER_GIE2_SHIFT) /* MUA General-purpose Interrupt Enable */

#define IMX9_MU_GIER_GIE3_SHIFT 3                              /* MUA General-purpose Interrupt Enable */
#define IMX9_MU_GIER_GIE3_FLAG  (1 << IMX9_MU_GIER_GIE3_SHIFT) /* MUA General-purpose Interrupt Enable */

/* GCR register */

#define IMX9_MU_GCR_GIR0_SHIFT 0                             /* MUA General-Purpose Interrupt Request */
#define IMX9_MU_GCR_GIR0_FLAG  (1 << IMX9_MU_GCR_GIR0_SHIFT) /* MUA General-Purpose Interrupt Request */

#define IMX9_MU_GCR_GIR1_SHIFT 1                             /* MUA General-Purpose Interrupt Request */
#define IMX9_MU_GCR_GIR1_FLAG  (1 << IMX9_MU_GCR_GIR1_SHIFT) /* MUA General-Purpose Interrupt Request */

#define IMX9_MU_GCR_GIR2_SHIFT 2                             /* MUA General-Purpose Interrupt Request */
#define IMX9_MU_GCR_GIR2_FLAG  (1 << IMX9_MU_GCR_GIR2_SHIFT) /* MUA General-Purpose Interrupt Request */

#define IMX9_MU_GCR_GIR3_SHIFT 3                             /* MUA General-Purpose Interrupt Request */
#define IMX9_MU_GCR_GIR3_FLAG  (1 << IMX9_MU_GCR_GIR3_SHIFT) /* MUA General-Purpose Interrupt Request */

/* GSR register */

#define IMX9_MU_GSR_GIP0_SHIFT 0                             /* MUA General-Purpose Interrupt Request Pending */
#define IMX9_MU_GSR_GIP0_FLAG  (1 << IMX9_MU_GSR_GIP0_SHIFT) /* MUA General-Purpose Interrupt Request Pending */

#define IMX9_MU_GSR_GIP1_SHIFT 1                             /* MUA General-Purpose Interrupt Request Pending */
#define IMX9_MU_GSR_GIP1_FLAG  (1 << IMX9_MU_GSR_GIP1_SHIFT) /* MUA General-Purpose Interrupt Request Pending */

#define IMX9_MU_GSR_GIP2_SHIFT 2                             /* MUA General-Purpose Interrupt Request Pending */
#define IMX9_MU_GSR_GIP2_FLAG  (1 << IMX9_MU_GSR_GIP2_SHIFT) /* MUA General-Purpose Interrupt Request Pending */

#define IMX9_MU_GSR_GIP3_SHIFT 3                             /* MUA General-Purpose Interrupt Request Pending */
#define IMX9_MU_GSR_GIP3_FLAG  (1 << IMX9_MU_GSR_GIP3_SHIFT) /* MUA General-Purpose Interrupt Request Pending */

/* TCR register */

#define IMX9_MU_TCR_TIE0_SHIFT 0                             /* MUA Transmit Interrupt Enable */
#define IMX9_MU_TCR_TIE0_FLAG  (1 << IMX9_MU_TCR_TIE0_SHIFT) /* MUA Transmit Interrupt Enable */

#define IMX9_MU_TCR_TIE1_SHIFT 1                             /* MUA Transmit Interrupt Enable */
#define IMX9_MU_TCR_TIE1_FLAG  (1 << IMX9_MU_TCR_TIE1_SHIFT) /* MUA Transmit Interrupt Enable */

#define IMX9_MU_TCR_TIE2_SHIFT 2                             /* MUA Transmit Interrupt Enable */
#define IMX9_MU_TCR_TIE2_FLAG  (1 << IMX9_MU_TCR_TIE2_SHIFT) /* MUA Transmit Interrupt Enable */

#define IMX9_MU_TCR_TIE3_SHIFT 3                             /* MUA Transmit Interrupt Enable */
#define IMX9_MU_TCR_TIE3_FLAG  (1 << IMX9_MU_TCR_TIE3_SHIFT) /* MUA Transmit Interrupt Enable */

/* TSR register */

#define IMX9_MU_TSR_TE0_SHIFT 0                            /* MUA Transmit Empty */
#define IMX9_MU_TSR_TE0_FLAG  (1 << IMX9_MU_TSR_TE0_SHIFT) /* MUA Transmit Empty */

#define IMX9_MU_TSR_TE1_SHIFT 1                            /* MUA Transmit Empty */
#define IMX9_MU_TSR_TE1_FLAG  (1 << IMX9_MU_TSR_TE1_SHIFT) /* MUA Transmit Empty */

#define IMX9_MU_TSR_TE2_SHIFT 2                            /* MUA Transmit Empty */
#define IMX9_MU_TSR_TE2_FLAG  (1 << IMX9_MU_TSR_TE2_SHIFT) /* MUA Transmit Empty */

#define IMX9_MU_TSR_TE3_SHIFT 3                            /* MUA Transmit Empty */
#define IMX9_MU_TSR_TE3_FLAG  (1 << IMX9_MU_TSR_TE3_SHIFT) /* MUA Transmit Empty */

/* RCR register */

#define IMX9_MU_RCR_RIE0_SHIFT 0                             /* MUA Receive Interrupt Enable */
#define IMX9_MU_RCR_RIE0_FLAG  (1 << IMX9_MU_RCR_RIE0_SHIFT) /* MUA Receive Interrupt Enable */

#define IMX9_MU_RCR_RIE1_SHIFT 1                             /* MUA Receive Interrupt Enable */
#define IMX9_MU_RCR_RIE1_FLAG  (1 << IMX9_MU_RCR_RIE1_SHIFT) /* MUA Receive Interrupt Enable */

#define IMX9_MU_RCR_RIE2_SHIFT 2                             /* MUA Receive Interrupt Enable */
#define IMX9_MU_RCR_RIE2_FLAG  (1 << IMX9_MU_RCR_RIE2_SHIFT) /* MUA Receive Interrupt Enable */

#define IMX9_MU_RCR_RIE3_SHIFT 3                             /* MUA Receive Interrupt Enable */
#define IMX9_MU_RCR_RIE3_FLAG  (1 << IMX9_MU_RCR_RIE3_SHIFT) /* MUA Receive Interrupt Enable */

/* RSR register */

#define IMX9_MU_RSR_RF0_SHIFT 0                            /* MUA Receive Register Full */
#define IMX9_MU_RSR_RF0_FLAG  (1 << IMX9_MU_RSR_RF0_SHIFT) /* MUA Receive Register Full */

#define IMX9_MU_RSR_RF1_SHIFT 1                            /* MUA Receive Register Full */
#define IMX9_MU_RSR_RF1_FLAG  (1 << IMX9_MU_RSR_RF1_SHIFT) /* MUA Receive Register Full */

#define IMX9_MU_RSR_RF2_SHIFT 2                            /* MUA Receive Register Full */
#define IMX9_MU_RSR_RF2_FLAG  (1 << IMX9_MU_RSR_RF2_SHIFT) /* MUA Receive Register Full */

#define IMX9_MU_RSR_RF3_SHIFT 3                            /* MUA Receive Register Full */
#define IMX9_MU_RSR_RF3_FLAG  (1 << IMX9_MU_RSR_RF3_SHIFT) /* MUA Receive Register Full */

/* Register array dimensions */

#define IMX9_MU_TR_REGARRAY_SIZE 4
#define IMX9_MU_RR_REGARRAY_SIZE 4
