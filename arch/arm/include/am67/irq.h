/****************************************************************************
 * arch/arm/include/am67/irq.h
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
 ****************************************************************************/

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_AM67_IRQ_H
#define __ARCH_ARM_INCLUDE_AM67_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CSLR_R5FSS0_CORE0_INTR_SMS0_AESEIP38T_0_AES_SINTREQUEST_P_0                        (1U)
#define CSLR_R5FSS0_CORE0_INTR_SMS0_AESEIP38T_0_AES_SINTREQUEST_S_0                        (2U)
#define CSLR_R5FSS0_CORE0_INTR_R5FSS0_CORE0_EXP_INTR_0                                     (4U)
#define CSLR_R5FSS0_CORE0_INTR_DMPAC0_DMPAC_LEVEL_0                                        (5U)
#define CSLR_R5FSS0_CORE0_INTR_DMPAC0_DMPAC_LEVEL_1                                        (6U)
#define CSLR_R5FSS0_CORE0_INTR_SA3_SS0_INTAGGR_0_INTAGGR_VINTR_7                           (7U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_32                      (8U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_33                      (9U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_34                      (10U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_35                      (11U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_36                      (12U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_37                      (13U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_38                      (14U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_39                      (15U)
#define CSLR_R5FSS0_CORE0_INTR_SA3_SS0_SA_UL_0_SA_UL_PKA_0                                 (16U)
#define CSLR_R5FSS0_CORE0_INTR_SA3_SS0_SA_UL_0_SA_UL_TRNG_0                                (17U)
#define CSLR_R5FSS0_CORE0_INTR_SMS0_TIFS_CBASS_0_FW_EXCEPTION_INTR_0                       (19U)
#define CSLR_R5FSS0_CORE0_INTR_SMS0_COMMON_0_COMBINED_SEC_IN_0                             (20U)
#define CSLR_R5FSS0_CORE0_INTR_SMS0_HSM_CBASS_0_FW_EXCEPTION_INTR_0                        (21U)
#define CSLR_R5FSS0_CORE0_INTR_GLUELOGIC_GPU_GPIO_REQACK_GLUE_GPU_GPIO_ACKINT_LVL_0        (22U)
#define CSLR_R5FSS0_CORE0_INTR_GLUELOGIC_GPU_GPIO_REQACK_GLUE_GPU_GPIO_REQINT_LVL_0        (23U)
#define CSLR_R5FSS0_CORE0_INTR_TIMER0_INTR_PEND_0                                          (24U)
#define CSLR_R5FSS0_CORE0_INTR_TIMER1_INTR_PEND_0                                          (25U)
#define CSLR_R5FSS0_CORE0_INTR_TIMER2_INTR_PEND_0                                          (26U)
#define CSLR_R5FSS0_CORE0_INTR_TIMER3_INTR_PEND_0                                          (27U)
#define CSLR_R5FSS0_CORE0_INTR_TIMER4_INTR_PEND_0                                          (28U)
#define CSLR_R5FSS0_CORE0_INTR_TIMER5_INTR_PEND_0                                          (29U)
#define CSLR_R5FSS0_CORE0_INTR_RTI8_INTR_WWD_0                                             (30U)
#define CSLR_R5FSS0_CORE0_INTR_GLUELOGIC_MCU_CBASS_INTR_OR_GLUE_OUT_0                      (31U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_MCU_GPIOMUX_INTROUTER0_OUTP_12                         (32U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_MCU_GPIOMUX_INTROUTER0_OUTP_13                         (33U)
#define CSLR_R5FSS0_CORE0_INTR_TIMER6_INTR_PEND_0                                          (34U)
#define CSLR_R5FSS0_CORE0_INTR_TIMER7_INTR_PEND_0                                          (35U)
#define CSLR_R5FSS0_CORE0_INTR_EPWM0_EPWM_ETINT_0                                          (36U)
#define CSLR_R5FSS0_CORE0_INTR_EPWM1_EPWM_ETINT_0                                          (37U)
#define CSLR_R5FSS0_CORE0_INTR_EPWM2_EPWM_ETINT_0                                          (38U)
#define CSLR_R5FSS0_CORE0_INTR_GLUELOGIC_MCU_ACCESS_ERR_INTR_GLUE_OUT_0                    (39U)
#define CSLR_R5FSS0_CORE0_INTR_DSS0_DISPC_INTR_REQ_0_0                                     (40U)
#define CSLR_R5FSS0_CORE0_INTR_DSS0_DISPC_INTR_REQ_1_0                                     (41U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_MCAN0_MCANSS_EXT_TS_ROLLOVER_LVL_INT_0                  (42U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_MCAN0_MCANSS_MCAN_LVL_INT_0                             (43U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_MCAN0_MCANSS_MCAN_LVL_INT_1                             (44U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_MCAN1_MCANSS_EXT_TS_ROLLOVER_LVL_INT_0                  (45U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_MCAN1_MCANSS_MCAN_LVL_INT_0                             (46U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_MCAN1_MCANSS_MCAN_LVL_INT_1                             (47U)
#define CSLR_R5FSS0_CORE0_INTR_CPSW0_CPTS_COMP_0                                           (48U)
#define CSLR_R5FSS0_CORE0_INTR_PCIE0_PCIE_CPTS_COMP_0                                      (49U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_TX_IF0_CSI_INTERRUPT_0                                  (50U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_TX_IF0_CSI_LEVEL_0                                      (51U)
#define CSLR_R5FSS0_CORE0_INTR_USB1_OTGIRQ_0                                               (52U)
#define CSLR_R5FSS0_CORE0_INTR_PCIE0_PCIE_CPTS_PEND_0                                      (53U)
#define CSLR_R5FSS0_CORE0_INTR_PCIE0_PCIE_PWR_STATE_PULSE_0                                (54U)
#define CSLR_R5FSS0_CORE0_INTR_PCIE0_PCIE_HOT_RESET_PULSE_0                                (55U)
#define CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_20                             (56U)
#define CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_21                             (57U)
#define CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_32                             (58U)
#define CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_33                             (59U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_MCU_GPIOMUX_INTROUTER0_OUTP_14                         (60U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_MCU_GPIOMUX_INTROUTER0_OUTP_15                         (61U)
#define CSLR_R5FSS0_CORE0_INTR_MCAN1_MCANSS_MCAN_LVL_INT_1                                 (63U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_152                     (64U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_153                     (65U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_154                     (66U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_155                     (67U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_156                     (68U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_157                     (69U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_158                     (70U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_159                     (71U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_160                     (72U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_161                     (73U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_162                     (74U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_163                     (75U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_164                     (76U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_165                     (77U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_166                     (78U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS0_INTAGGR_0_INTAGGR_VINTR_PEND_167                     (79U)
#define CSLR_R5FSS0_CORE0_INTR_EPWM0_EPWM_TRIPZINT_0                                       (80U)
#define CSLR_R5FSS0_CORE0_INTR_EPWM1_EPWM_TRIPZINT_0                                       (81U)
#define CSLR_R5FSS0_CORE0_INTR_EPWM2_EPWM_TRIPZINT_0                                       (82U)
#define CSLR_R5FSS0_CORE0_INTR_ECAP0_ECAP_INT_0                                            (83U)
#define CSLR_R5FSS0_CORE0_INTR_ECAP1_ECAP_INT_0                                            (84U)
#define CSLR_R5FSS0_CORE0_INTR_ECAP2_ECAP_INT_0                                            (85U)
#define CSLR_R5FSS0_CORE0_INTR_EQEP0_EQEP_INT_0                                            (86U)
#define CSLR_R5FSS0_CORE0_INTR_EQEP1_EQEP_INT_0                                            (87U)
#define CSLR_R5FSS0_CORE0_INTR_EQEP2_EQEP_INT_0                                            (88U)
#define CSLR_R5FSS0_CORE0_INTR_R5FSS0_COMMON0_COMMRX_LEVEL_0_0                             (90U)
#define CSLR_R5FSS0_CORE0_INTR_R5FSS0_COMMON0_COMMTX_LEVEL_0_0                             (91U)
#define CSLR_R5FSS0_CORE0_INTR_DSS1_DISPC_INTR_REQ_0_0                                     (92U)
#define CSLR_R5FSS0_CORE0_INTR_DSS1_DISPC_INTR_REQ_1_0                                     (93U)
#define CSLR_R5FSS0_CORE0_INTR_R5FSS0_CORE0_PMU_0                                          (94U)
#define CSLR_R5FSS0_CORE0_INTR_R5FSS0_CORE0_VALFIQ_0                                       (95U)
#define CSLR_R5FSS0_CORE0_INTR_R5FSS0_CORE0_VALIRQ_0                                       (96U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_RTCSS0_RTC_EVENT_PEND_0                                (97U)
#define CSLR_R5FSS0_CORE0_INTR_PCIE0_PCIE_DOWNSTREAM_PULSE_0                               (99U)
#define CSLR_R5FSS0_CORE0_INTR_JPGENC0_IRQ_0                                               (100U)
#define CSLR_R5FSS0_CORE0_INTR_GPMC0_GPMC_SINTERRUPT_0                                     (103U)
#define CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_16                             (104U)
#define CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_17                             (105U)
#define CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_18                             (106U)
#define CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_19                             (107U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_DCC0_INTR_DONE_LEVEL_0                                  (108U)
#define CSLR_R5FSS0_CORE0_INTR_GLUELOGIC_MAIN_DCC_DONE_GLUE_DCC_DONE_0                     (109U)
#define CSLR_R5FSS0_CORE0_INTR_MCAN1_MCANSS_MCAN_LVL_INT_0                                 (110U)
#define CSLR_R5FSS0_CORE0_INTR_SMS0_RAT_1_EXP_INTR_0                                       (111U)
#define CSLR_R5FSS0_CORE0_INTR_SMS0_RAT_0_EXP_INTR_0                                       (112U)
#define CSLR_R5FSS0_CORE0_INTR_GLUELOGICN_MAIN_PBIST_CPU_GLUE_OUT_0                        (113U)
#define CSLR_R5FSS0_CORE0_INTR_GLUELOGIC_WKUP_PBIST_CPUINTR_OUT_0                          (114U)
#define CSLR_R5FSS0_CORE0_INTR_MAILBOX0_MAILBOX_CLUSTER_2_MAILBOX_CLUSTER_PEND_3           (115U)
#define CSLR_R5FSS0_CORE0_INTR_MAILBOX0_MAILBOX_CLUSTER_3_MAILBOX_CLUSTER_PEND_3           (116U)
#define CSLR_R5FSS0_CORE0_INTR_MCASP3_XMIT_INTR_PEND_0                                     (117U)
#define CSLR_R5FSS0_CORE0_INTR_MCASP3_REC_INTR_PEND_0                                      (118U)
#define CSLR_R5FSS0_CORE0_INTR_MCRC64_0_INT_MCRC_0                                         (119U)
#define CSLR_R5FSS0_CORE0_INTR_MCASP0_REC_INTR_PEND_0                                      (120U)
#define CSLR_R5FSS0_CORE0_INTR_MCASP0_XMIT_INTR_PEND_0                                     (121U)
#define CSLR_R5FSS0_CORE0_INTR_MCASP1_REC_INTR_PEND_0                                      (122U)
#define CSLR_R5FSS0_CORE0_INTR_MCASP1_XMIT_INTR_PEND_0                                     (123U)
#define CSLR_R5FSS0_CORE0_INTR_MCASP2_REC_INTR_PEND_0                                      (124U)
#define CSLR_R5FSS0_CORE0_INTR_MCASP2_XMIT_INTR_PEND_0                                     (125U)
#define CSLR_R5FSS0_CORE0_INTR_PCIE0_PCIE_DPA_PULSE_0                                      (126U)
#define CSLR_R5FSS0_CORE0_INTR_GLUELOGIC_SOC_ACCESS_ERR_INTR_GLUE_OUT_0                    (128U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_24                      (129U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_25                      (130U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_26                      (131U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_27                      (132U)
#define CSLR_R5FSS0_CORE0_INTR_CODEC0_VPU_WAVE521CL_INTR_0                                 (133U)
#define CSLR_R5FSS0_CORE0_INTR_CPSW0_EVNT_PEND_0                                           (134U)
#define CSLR_R5FSS0_CORE0_INTR_CPSW0_MDIO_PEND_0                                           (135U)
#define CSLR_R5FSS0_CORE0_INTR_CPSW0_STAT_PEND_0                                           (136U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_DCC1_INTR_DONE_LEVEL_0                                  (137U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_28                      (138U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_29                      (139U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_ESM0_ESM_INT_CFG_LVL_0                                 (140U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_ESM0_ESM_INT_HI_LVL_0                                  (141U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_ESM0_ESM_INT_LOW_LVL_0                                 (142U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_30                      (143U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_31                      (144U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_PSC0_PSC_ALLINT_0                                      (145U)
#define CSLR_R5FSS0_CORE0_INTR_PSC0_PSC_ALLINT_0                                           (146U)
#define CSLR_R5FSS0_CORE0_INTR_GLUELOGIC_SOC_CBASS_ERR_INTR_GLUE_MAIN_CBASS_AGG_ERR_INTR_0 (147U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_PBIST0_DFT_PBIST_CPU_0                                  (149U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_23                      (150U)
#define CSLR_R5FSS0_CORE0_INTR_DDR32SS0_DDRSS_CONTROLLER_0                                 (151U)
#define CSLR_R5FSS0_CORE0_INTR_GLUELOGIC_MGASKET_INTR_GLUE_OUT_0                           (152U)
#define CSLR_R5FSS0_CORE0_INTR_GLUELOGIC_SGASKET_INTR_GLUE_OUT_0                           (153U)
#define CSLR_R5FSS0_CORE0_INTR_SERDES_10G1_PHY_PWR_TIMEOUT_LVL_0                           (154U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_RX_IF3_CSI_ERR_IRQ_0                                    (155U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_RX_IF3_CSI_IRQ_0                                        (156U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_RX_IF3_CSI_LEVEL_0                                      (157U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_14                      (158U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_15                      (159U)
#define CSLR_R5FSS0_CORE0_INTR_DMASS1_INTAGGR_0_INTAGGR_VINTR_PEND_22                      (160U)
#define CSLR_R5FSS0_CORE0_INTR_MMCSD0_EMMCSS_INTR_0                                        (161U)
#define CSLR_R5FSS0_CORE0_INTR_MMCSD1_EMMCSDSS_INTR_0                                      (162U)
#define CSLR_R5FSS0_CORE0_INTR_MMCSD2_EMMCSDSS_INTR_0                                      (163U)
#define CSLR_R5FSS0_CORE0_INTR_ELM0_ELM_POROCPSINTERRUPT_LVL_0                             (164U)
#define CSLR_R5FSS0_CORE0_INTR_PCIE0_PCIE_PTM_VALID_PULSE_0                                (165U)
#define CSLR_R5FSS0_CORE0_INTR_SERDES_10G0_PHY_PWR_TIMEOUT_LVL_0                           (166U)
#define CSLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_CFG_LVL_0                                      (167U)
#define CSLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_HI_LVL_0                                       (168U)
#define CSLR_R5FSS0_CORE0_INTR_ESM0_ESM_INT_LOW_LVL_0                                      (169U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_RX_IF0_CSI_ERR_IRQ_0                                    (170U)
#define CSLR_R5FSS0_CORE0_INTR_FSS0_OSPI_0_OSPI_LVL_INTR_0                                 (171U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_RX_IF1_CSI_ERR_IRQ_0                                    (172U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_RX_IF0_CSI_IRQ_0                                        (173U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_RX_IF0_CSI_LEVEL_0                                      (174U)
#define CSLR_R5FSS0_CORE0_INTR_R5FSS0_CORE0_CTI_0                                          (175U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_RX_IF1_CSI_IRQ_0                                        (176U)
#define CSLR_R5FSS0_CORE0_INTR_DDPA0_DDPA_INTR_0                                           (177U)
#define CSLR_R5FSS0_CORE0_INTR_VPAC0_VPAC_LEVEL_0                                          (178U)
#define CSLR_R5FSS0_CORE0_INTR_VPAC0_VPAC_LEVEL_1                                          (179U)
#define CSLR_R5FSS0_CORE0_INTR_VPAC0_VPAC_LEVEL_2                                          (180U)
#define CSLR_R5FSS0_CORE0_INTR_DDR32SS0_DDRSS_PLL_FREQ_CHANGE_REQ_0                        (181U)
#define CSLR_R5FSS0_CORE0_INTR_VPAC0_VPAC_LEVEL_3                                          (182U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_VTM0_THERM_LVL_GT_TH1_INTR_0                           (183U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_VTM0_THERM_LVL_GT_TH2_INTR_0                           (184U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_VTM0_THERM_LVL_LT_TH0_INTR_0                           (185U)
#define CSLR_R5FSS0_CORE0_INTR_MCAN0_MCANSS_EXT_TS_ROLLOVER_LVL_INT_0                      (186U)
#define CSLR_R5FSS0_CORE0_INTR_MCAN0_MCANSS_MCAN_LVL_INT_0                                 (187U)
#define CSLR_R5FSS0_CORE0_INTR_MCAN0_MCANSS_MCAN_LVL_INT_1                                 (188U)
#define CSLR_R5FSS0_CORE0_INTR_VPAC0_VPAC_LEVEL_4                                          (189U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_I2C0_POINTRPEND_0                                      (190U)
#define CSLR_R5FSS0_CORE0_INTR_VPAC0_VPAC_LEVEL_5                                          (191U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_MCRC64_0_INT_MCRC_0                                     (192U)
#define CSLR_R5FSS0_CORE0_INTR_I2C0_POINTRPEND_0                                           (193U)
#define CSLR_R5FSS0_CORE0_INTR_I2C1_POINTRPEND_0                                           (194U)
#define CSLR_R5FSS0_CORE0_INTR_I2C2_POINTRPEND_0                                           (195U)
#define CSLR_R5FSS0_CORE0_INTR_I2C3_POINTRPEND_0                                           (196U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_I2C0_POINTRPEND_0                                       (197U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_RX_IF2_CSI_ERR_IRQ_0                                    (198U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_RX_IF2_CSI_IRQ_0                                        (199U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_RX_IF2_CSI_LEVEL_0                                      (200U)
#define CSLR_R5FSS0_CORE0_INTR_DEBUGSS0_AQCMPINTR_LEVEL_0                                  (201U)
#define CSLR_R5FSS0_CORE0_INTR_DEBUGSS0_CTM_LEVEL_0                                        (202U)
#define CSLR_R5FSS0_CORE0_INTR_GLUELOGIC_GLUE_EXT_INTN_OUT_0                               (203U)
#define CSLR_R5FSS0_CORE0_INTR_MCSPI0_INTR_SPI_0                                           (204U)
#define CSLR_R5FSS0_CORE0_INTR_MCSPI1_INTR_SPI_0                                           (205U)
#define CSLR_R5FSS0_CORE0_INTR_MCSPI2_INTR_SPI_0                                           (206U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_MCSPI0_INTR_SPI_0                                       (207U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_MCSPI1_INTR_SPI_0                                       (208U)
#define CSLR_R5FSS0_CORE0_INTR_CSI_RX_IF1_CSI_LEVEL_0                                      (209U)
#define CSLR_R5FSS0_CORE0_INTR_UART0_USART_IRQ_0                                           (210U)
#define CSLR_R5FSS0_CORE0_INTR_UART1_USART_IRQ_0                                           (211U)
#define CSLR_R5FSS0_CORE0_INTR_UART2_USART_IRQ_0                                           (212U)
#define CSLR_R5FSS0_CORE0_INTR_UART3_USART_IRQ_0                                           (213U)
#define CSLR_R5FSS0_CORE0_INTR_UART4_USART_IRQ_0                                           (214U)
#define CSLR_R5FSS0_CORE0_INTR_UART5_USART_IRQ_0                                           (215U)
#define CSLR_R5FSS0_CORE0_INTR_UART6_USART_IRQ_0                                           (216U)
#define CSLR_R5FSS0_CORE0_INTR_MCU_UART0_USART_IRQ_0                                       (217U)
#define CSLR_R5FSS0_CORE0_INTR_DSS_DSI0_DSI_0_FUNC_INTR_0                                  (218U)
#define CSLR_R5FSS0_CORE0_INTR_WKUP_UART0_USART_IRQ_0                                      (219U)
#define CSLR_R5FSS0_CORE0_INTR_USB0_IRQ_0                                                  (220U)
#define CSLR_R5FSS0_CORE0_INTR_USB0_IRQ_1                                                  (221U)
#define CSLR_R5FSS0_CORE0_INTR_USB0_IRQ_2                                                  (222U)
#define CSLR_R5FSS0_CORE0_INTR_USB0_IRQ_3                                                  (223U)
#define CSLR_R5FSS0_CORE0_INTR_USB0_IRQ_4                                                  (224U)
#define CSLR_R5FSS0_CORE0_INTR_USB0_IRQ_5                                                  (225U)
#define CSLR_R5FSS0_CORE0_INTR_USB0_IRQ_6                                                  (226U)
#define CSLR_R5FSS0_CORE0_INTR_USB0_IRQ_7                                                  (227U)
#define CSLR_R5FSS0_CORE0_INTR_USB0_MISC_LEVEL_0                                           (228U)
#define CSLR_R5FSS0_CORE0_INTR_MCAN1_MCANSS_EXT_TS_ROLLOVER_LVL_INT_0                      (229U)
#define CSLR_R5FSS0_CORE0_INTR_USB1_IRQ_0                                                  (230U)
#define CSLR_R5FSS0_CORE0_INTR_USB1_IRQ_1                                                  (231U)
#define CSLR_R5FSS0_CORE0_INTR_USB1_IRQ_2                                                  (232U)
#define CSLR_R5FSS0_CORE0_INTR_USB1_IRQ_3                                                  (233U)
#define CSLR_R5FSS0_CORE0_INTR_USB1_IRQ_4                                                  (234U)
#define CSLR_R5FSS0_CORE0_INTR_USB1_IRQ_5                                                  (235U)
#define CSLR_R5FSS0_CORE0_INTR_USB1_IRQ_6                                                  (236U)
#define CSLR_R5FSS0_CORE0_INTR_USB1_IRQ_7                                                  (237U)
#define CSLR_R5FSS0_CORE0_INTR_USB1_HOST_SYSTEM_ERROR_0                                    (238U)
#define CSLR_R5FSS0_CORE0_INTR_I2C4_POINTRPEND_0                                           (239U)
#define CSLR_R5FSS0_CORE0_INTR_MAILBOX0_MAILBOX_CLUSTER_4_MAILBOX_CLUSTER_PEND_3           (240U)
#define CSLR_R5FSS0_CORE0_INTR_MAILBOX0_MAILBOX_CLUSTER_5_MAILBOX_CLUSTER_PEND_3           (241U)
#define CSLR_R5FSS0_CORE0_INTR_MAILBOX0_MAILBOX_CLUSTER_6_MAILBOX_CLUSTER_PEND_3           (242U)
#define CSLR_R5FSS0_CORE0_INTR_MAILBOX0_MAILBOX_CLUSTER_7_MAILBOX_CLUSTER_PEND_3           (243U)
#define CSLR_R5FSS0_CORE0_INTR_C7X256V1_CLEC_SOC_EVENTS_OUT_LEVEL_24                       (244U)
#define CSLR_R5FSS0_CORE0_INTR_C7X256V1_CLEC_SOC_EVENTS_OUT_LEVEL_25                       (245U)
#define CSLR_R5FSS0_CORE0_INTR_C7X256V1_CLEC_SOC_EVENTS_OUT_LEVEL_26                       (246U)
#define CSLR_R5FSS0_CORE0_INTR_C7X256V1_CLEC_SOC_EVENTS_OUT_LEVEL_27                       (247U)
#define CSLR_R5FSS0_CORE0_INTR_GPU0_GPU_PWRCTRL_REQ_0                                      (248U)
#define CSLR_R5FSS0_CORE0_INTR_C7X256V0_CLEC_SOC_EVENTS_OUT_LEVEL_24                       (249U)
#define CSLR_R5FSS0_CORE0_INTR_C7X256V0_CLEC_SOC_EVENTS_OUT_LEVEL_25                       (250U)
#define CSLR_R5FSS0_CORE0_INTR_MCASP4_XMIT_INTR_PEND_0                                     (251U)
#define CSLR_R5FSS0_CORE0_INTR_MCASP4_REC_INTR_PEND_0                                      (252U)
#define CSLR_R5FSS0_CORE0_INTR_C7X256V0_CLEC_SOC_EVENTS_OUT_LEVEL_26                       (254U)
#define CSLR_R5FSS0_CORE0_INTR_C7X256V0_CLEC_SOC_EVENTS_OUT_LEVEL_27                       (255U)

#define NR_IRQS 256

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_AM67_IRQ_H */
