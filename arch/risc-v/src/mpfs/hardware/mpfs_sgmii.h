/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_sgmii.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_SGMII_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_SGMII_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/mpfs_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_MPUCFG_SEG0_REG0_OFFSET 0xd00
#define MPFS_MPUCFG_SEG0_REG1_OFFSET 0xd08
#define MPFS_MPUCFG_SEG0_REG2_OFFSET 0xd10
#define MPFS_MPUCFG_SEG0_REG3_OFFSET 0xd18
#define MPFS_MPUCFG_SEG0_REG4_OFFSET 0xd20
#define MPFS_MPUCFG_SEG0_REG5_OFFSET 0xd28
#define MPFS_MPUCFG_SEG0_REG6_OFFSET 0xd30

#define MPFS_MPUCFG_SEG1_REG0_OFFSET 0xe00
#define MPFS_MPUCFG_SEG1_REG1_OFFSET 0xe08
#define MPFS_MPUCFG_SEG1_REG2_OFFSET 0xe10
#define MPFS_MPUCFG_SEG1_REG3_OFFSET 0xe18
#define MPFS_MPUCFG_SEG1_REG4_OFFSET 0xe20
#define MPFS_MPUCFG_SEG1_REG5_OFFSET 0xe28
#define MPFS_MPUCFG_SEG1_REG6_OFFSET 0xe30

#define MPFS_MPUCFG_SEG0_REG0 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG0_REG0_OFFSET)
#define MPFS_MPUCFG_SEG0_REG1 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG0_REG1_OFFSET)
#define MPFS_MPUCFG_SEG0_REG2 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG0_REG2_OFFSET)
#define MPFS_MPUCFG_SEG0_REG3 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG0_REG3_OFFSET)
#define MPFS_MPUCFG_SEG0_REG4 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG0_REG4_OFFSET)
#define MPFS_MPUCFG_SEG0_REG5 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG0_REG5_OFFSET)
#define MPFS_MPUCFG_SEG0_REG6 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG0_REG6_OFFSET)

#define MPFS_MPUCFG_SEG1_REG0 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG1_REG0_OFFSET)
#define MPFS_MPUCFG_SEG1_REG1 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG1_REG1_OFFSET)
#define MPFS_MPUCFG_SEG1_REG2 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG1_REG2_OFFSET)
#define MPFS_MPUCFG_SEG1_REG3 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG1_REG3_OFFSET)
#define MPFS_MPUCFG_SEG1_REG4 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG1_REG4_OFFSET)
#define MPFS_MPUCFG_SEG1_REG5 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG1_REG5_OFFSET)
#define MPFS_MPUCFG_SEG1_REG6 (MPFS_MPUCFG_BASE + MPFS_MPUCFG_SEG1_REG6_OFFSET)

#define MPFS_IOSCBCFG_TIMER_OFFSET                                 0x08

#define MPFS_SYSREGSCB_MSS_RESET_CR_OFFSET                         0x100
#define MPFS_SYSREGSCB_MSSIO_CONTROL_CR_OFFSET                     0x1bc

#define MPFS_IOSCB_DLL_SGMII_SOFT_RESET_OFFSET                     0x000
#define MPFS_IOSCB_BANK_CNTL_SGMII_SOFT_RESET_OFFSET               0x000
#define MPFS_IOSCB_BANK_CNTL_DDR_SOFT_RESET_OFFSET                 0x000

#define MPFS_IOSCB_CALIB_SGMII_SOFT_RESET_OFFSET                   0x000
#define MPFS_IOSCB_CALIB_SGMII_IOC_REG0_OFFSET                     0x004

#define MPFS_IOSCB_CALIB_DDR_SOFT_RESET_OFFSET                     0x000
#define MPFS_IOSCB_CALIB_DDR_IOC_REG0_OFFSET                       0x004
#define MPFS_IOSCB_CALIB_DDR_IOC_REG1_OFFSET                       0x008
#define MPFS_IOSCB_CALIB_DDR_IOC_REG2_OFFSET                       0x00c
#define MPFS_IOSCB_CALIB_DDR_IOC_REG3_OFFSET                       0x010
#define MPFS_IOSCB_CALIB_DDR_IOC_REG4_OFFSET                       0x014
#define MPFS_IOSCB_CALIB_DDR_IOC_REG5_OFFSET                       0x018
#define MPFS_IOSCB_CALIB_DDR_IOC_REG6_OFFSET                       0x01c

#define MPFS_IOSCB_SGMII_LANE01_SOFT_RESET_OFFSET                  0x000
#define MPFS_IOSCB_SGMII_LANE23_SOFT_RESET_OFFSET                  0x000

#define MPFS_IOSCB_SGMII_MUX_SOFT_RESET_OFFSET                     0x000
#define MPFS_IOSCB_SGMII_MUX_RFCKMUX_OFFSET                        0x004
#define MPFS_IOSCB_SGMII_MUX_SGMII_CLKMUX_OFFSET                   0x008
#define MPFS_IOSCB_SGMII_MUX_SPARE0_OFFSET                         0x00c
#define MPFS_IOSCB_SGMII_MUX_CLK_XCVR_OFFSET                       0x010
#define MPFS_IOSCB_SGMII_MUX_TEST_CTRL_OFFSET                      0x014

#define MPFS_IOSCB_MSS_MUX_SOFT_RESET_OFFSET                       0x000
#define MPFS_IOSCB_MSS_MUX_BCLKMUX_OFFSET                          0x004
#define MPFS_IOSCB_MSS_MUX_PLL_CKMUX_OFFSET                        0x008
#define MPFS_IOSCB_MSS_MUX_MSSCLKMUX_OFFSET                        0x00c
#define MPFS_IOSCB_MSS_MUX_SPARE0_OFFSET                           0x010
#define MPFS_IOSCB_MSS_MUX_FMETER_ADDR_OFFSET                      0x014
#define MPFS_IOSCB_MSS_MUX_FMETER_DATAW_OFFSET                     0x018
#define MPFS_IOSCB_MSS_MUX_FMETER_DATAR_OFFSET                     0x01c
#define MPFS_IOSCB_MSS_MUX_TEST_CTRL_OFFSET                        0x020

#define MPFS_IOSCB_SGMII_PLL_SOFT_RESET_OFFSET                     0x000
#define MPFS_IOSCB_SGMII_PLL_CTRL_OFFSET                           0x004
#define MPFS_IOSCB_SGMII_PLL_REF_FB_OFFSET                         0x008
#define MPFS_IOSCB_SGMII_PLL_FRACN_OFFSET                          0x00c
#define MPFS_IOSCB_SGMII_PLL_DIV_0_1_OFFSET                        0x010
#define MPFS_IOSCB_SGMII_PLL_DIV_2_3_OFFSET                        0x014
#define MPFS_IOSCB_SGMII_PLL_CTRL2_OFFSET                          0x018
#define MPFS_IOSCB_SGMII_PLL_CAL_OFFSET                            0x01c
#define MPFS_IOSCB_SGMII_PLL_PHADJ_OFFSET                          0x020
#define MPFS_IOSCB_SGMII_PLL_SSCG_REG_0_OFFSET                     0x024
#define MPFS_IOSCB_SGMII_PLL_SSCG_REG_1_OFFSET                     0x028
#define MPFS_IOSCB_SGMII_PLL_SSCG_REG_2_OFFSET                     0x02c
#define MPFS_IOSCB_SGMII_PLL_SSCG_REG_3_OFFSET                     0x030

#define MPFS_IOSCB_MSS_PLL_SOFT_RESET_OFFSET                       0x000
#define MPFS_IOSCB_MSS_PLL_CTRL_OFFSET                             0x004
#define MPFS_IOSCB_MSS_PLL_REF_FB_OFFSET                           0x008
#define MPFS_IOSCB_MSS_PLL_FRACN_OFFSET                            0x00c
#define MPFS_IOSCB_MSS_PLL_DIV_0_1_OFFSET                          0x010
#define MPFS_IOSCB_MSS_PLL_DIV_2_3_OFFSET                          0x014
#define MPFS_IOSCB_MSS_PLL_CTRL2_OFFSET                            0x018
#define MPFS_IOSCB_MSS_PLL_CAL_OFFSET                              0x01c
#define MPFS_IOSCB_MSS_PLL_PHADJ_OFFSET                            0x020
#define MPFS_IOSCB_MSS_PLL_SSCG_REG_0_OFFSET                       0x024
#define MPFS_IOSCB_MSS_PLL_SSCG_REG_1_OFFSET                       0x028
#define MPFS_IOSCB_MSS_PLL_SSCG_REG_2_OFFSET                       0x02c
#define MPFS_IOSCB_MSS_PLL_SSCG_REG_3_OFFSET                       0x030

#define MPFS_IOSCB_DDR_PLL_SOFT_RESET_OFFSET                       0x000
#define MPFS_IOSCB_DDR_PLL_CTRL_OFFSET                             0x004
#define MPFS_IOSCB_DDR_PLL_REF_FB_OFFSET                           0x008
#define MPFS_IOSCB_DDR_PLL_FRACN_OFFSET                            0x00c
#define MPFS_IOSCB_DDR_PLL_DIV_0_1_OFFSET                          0x010
#define MPFS_IOSCB_DDR_PLL_DIV_2_3_OFFSET                          0x014
#define MPFS_IOSCB_DDR_PLL_CTRL2_OFFSET                            0x018
#define MPFS_IOSCB_DDR_PLL_CAL_OFFSET                              0x01c
#define MPFS_IOSCB_DDR_PLL_PHADJ_OFFSET                            0x020
#define MPFS_IOSCB_DDR_PLL_SSCG_REG_0_OFFSET                       0x024
#define MPFS_IOSCB_DDR_PLL_SSCG_REG_1_OFFSET                       0x028
#define MPFS_IOSCB_DDR_PLL_SSCG_REG_2_OFFSET                       0x02c
#define MPFS_IOSCB_DDR_PLL_SSCG_REG_3_OFFSET                       0x030

#define MPFS_GEM0_NETWORK_CONFIG_OFFSET                            0x004
#define MPFS_GEM1_NETWORK_CONFIG_OFFSET                            0x004

#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DDR_PHY_OFFSET           0x000
#define MPFS_CFG_DDR_SGMII_PHY_DDRPHY_MODE_OFFSET                  0x004
#define MPFS_CFG_DDR_SGMII_PHY_STARTUP_OFFSET                      0x008
#define MPFS_CFG_DDR_SGMII_PHY_SPARE_0_OFFSET                      0x00c
#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_MAIN_PLL_OFFSET          0x080
#define MPFS_CFG_DDR_SGMII_PHY_PLL_CTRL_MAIN_OFFSET                0x084
#define MPFS_CFG_DDR_SGMII_PHY_PLL_REF_FB_MAIN_OFFSET              0x088
#define MPFS_CFG_DDR_SGMII_PHY_PLL_FRACN_MAIN_OFFSET               0x08c
#define MPFS_CFG_DDR_SGMII_PHY_PLL_DIV_0_1_MAIN_OFFSET             0x090
#define MPFS_CFG_DDR_SGMII_PHY_PLL_DIV_2_3_MAIN_OFFSET             0x094
#define MPFS_CFG_DDR_SGMII_PHY_PLL_CTRL2_MAIN_OFFSET               0x098
#define MPFS_CFG_DDR_SGMII_PHY_PLL_CAL_MAIN_OFFSET                 0x09c
#define MPFS_CFG_DDR_SGMII_PHY_PLL_PHADJ_MAIN_OFFSET               0x0a0
#define MPFS_CFG_DDR_SGMII_PHY_SSCG_REG_0_MAIN_OFFSET              0x0a4
#define MPFS_CFG_DDR_SGMII_PHY_SSCG_REG_1_MAIN_OFFSET              0x0a8
#define MPFS_CFG_DDR_SGMII_PHY_SSCG_REG_2_MAIN_OFFSET              0x0ac
#define MPFS_CFG_DDR_SGMII_PHY_SSCG_REG_3_MAIN_OFFSET              0x0b0
#define MPFS_CFG_DDR_SGMII_PHY_RPC_RESET_MAIN_PLL_OFFSET           0x0b4
#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_IOSCB_PLL_OFFSET         0x100
#define MPFS_CFG_DDR_SGMII_PHY_PLL_CTRL_IOSCB_OFFSET               0x104
#define MPFS_CFG_DDR_SGMII_PHY_PLL_REF_FB_IOSCB_OFFSET             0x108
#define MPFS_CFG_DDR_SGMII_PHY_PLL_FRACN_IOSCB_OFFSET              0x10c
#define MPFS_CFG_DDR_SGMII_PHY_PLL_DIV_0_1_IOSCB_OFFSET            0x110
#define MPFS_CFG_DDR_SGMII_PHY_PLL_DIV_2_3_IOSCB_OFFSET            0x114
#define MPFS_CFG_DDR_SGMII_PHY_PLL_CTRL2_IOSCB_OFFSET              0x118
#define MPFS_CFG_DDR_SGMII_PHY_PLL_CAL_IOSCB_OFFSET                0x11c
#define MPFS_CFG_DDR_SGMII_PHY_PLL_PHADJ_IOSCB_OFFSET              0x120
#define MPFS_CFG_DDR_SGMII_PHY_SSCG_REG_0_IOSCB_OFFSET             0x124
#define MPFS_CFG_DDR_SGMII_PHY_SSCG_REG_1_IOSCB_OFFSET             0x128
#define MPFS_CFG_DDR_SGMII_PHY_SSCG_REG_2_IOSCB_OFFSET             0x12c
#define MPFS_CFG_DDR_SGMII_PHY_SSCG_REG_3_IOSCB_OFFSET             0x130
#define MPFS_CFG_DDR_SGMII_PHY_RPC_RESET_IOSCB_OFFSET              0x134
#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_BANK_CTRL_OFFSET         0x180
#define MPFS_CFG_DDR_SGMII_PHY_DPC_BITS_OFFSET                     0x184
#define MPFS_CFG_DDR_SGMII_PHY_BANK_STATUS_OFFSET                  0x188
#define MPFS_CFG_DDR_SGMII_PHY_RPC_RESET_BANK_CTRL_OFFSET          0x18c
#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_IOCALIB_OFFSET           0x200
#define MPFS_CFG_DDR_SGMII_PHY_IOC_REG0_OFFSET                     0x204
#define MPFS_CFG_DDR_SGMII_PHY_IOC_REG1_OFFSET                     0x208
#define MPFS_CFG_DDR_SGMII_PHY_IOC_REG2_OFFSET                     0x20c
#define MPFS_CFG_DDR_SGMII_PHY_IOC_REG3_OFFSET                     0x210
#define MPFS_CFG_DDR_SGMII_PHY_IOC_REG4_OFFSET                     0x214
#define MPFS_CFG_DDR_SGMII_PHY_IOC_REG5_OFFSET                     0x218
#define MPFS_CFG_DDR_SGMII_PHY_IOC_REG6_OFFSET                     0x21c
#define MPFS_CFG_DDR_SGMII_PHY_RPC_RESET_IOCALIB_OFFSET            0x220
#define MPFS_CFG_DDR_SGMII_PHY_RPC_CALIB_OFFSET                    0x224
#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_OFFSET                   0x280
#define MPFS_CFG_DDR_SGMII_PHY_BCLKMUX_OFFSET                      0x284
#define MPFS_CFG_DDR_SGMII_PHY_PLL_CKMUX_OFFSET                    0x288
#define MPFS_CFG_DDR_SGMII_PHY_MSSCLKMUX_OFFSET                    0x28c
#define MPFS_CFG_DDR_SGMII_PHY_SPARE0_OFFSET                       0x290
#define MPFS_CFG_DDR_SGMII_PHY_FMETER_ADDR_OFFSET                  0x294
#define MPFS_CFG_DDR_SGMII_PHY_FMETER_DATAW_OFFSET                 0x298
#define MPFS_CFG_DDR_SGMII_PHY_FMETER_DATAR_OFFSET                 0x29c
#define MPFS_CFG_DDR_SGMII_PHY_TEST_CTRL_OFFSET                    0x2a0
#define MPFS_CFG_DDR_SGMII_PHY_RPC_RESET_OFFSET                    0x2a4
#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_DRIVER_OFFSET    0x300
#define MPFS_CFG_DDR_SGMII_PHY_RPC1_DRV_OFFSET                     0x304
#define MPFS_CFG_DDR_SGMII_PHY_RPC2_DRV_OFFSET                     0x308
#define MPFS_CFG_DDR_SGMII_PHY_RPC3_DRV_OFFSET                     0x30c
#define MPFS_CFG_DDR_SGMII_PHY_RPC4_DRV_OFFSET                     0x310
#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_ODT_OFFSET       0x380
#define MPFS_CFG_DDR_SGMII_PHY_RPC1_ODT_OFFSET                     0x384
#define MPFS_CFG_DDR_SGMII_PHY_RPC2_ODT_OFFSET                     0x388
#define MPFS_CFG_DDR_SGMII_PHY_RPC3_ODT_OFFSET                     0x38c
#define MPFS_CFG_DDR_SGMII_PHY_RPC4_ODT_OFFSET                     0x390
#define MPFS_CFG_DDR_SGMII_PHY_RPC5_ODT_OFFSET                     0x394
#define MPFS_CFG_DDR_SGMII_PHY_RPC6_ODT_OFFSET                     0x398
#define MPFS_CFG_DDR_SGMII_PHY_RPC7_ODT_OFFSET                     0x39c
#define MPFS_CFG_DDR_SGMII_PHY_RPC8_ODT_OFFSET                     0x3a0
#define MPFS_CFG_DDR_SGMII_PHY_RPC9_ODT_OFFSET                     0x3a4
#define MPFS_CFG_DDR_SGMII_PHY_RPC10_ODT_OFFSET                    0x3a8
#define MPFS_CFG_DDR_SGMII_PHY_RPC11_ODT_OFFSET                    0x3ac
#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_IO_OFFSET        0x400
#define MPFS_CFG_DDR_SGMII_PHY_OVRT1_OFFSET                        0x404
#define MPFS_CFG_DDR_SGMII_PHY_OVRT2_OFFSET                        0x408
#define MPFS_CFG_DDR_SGMII_PHY_OVRT3_OFFSET                        0x40c
#define MPFS_CFG_DDR_SGMII_PHY_OVRT4_OFFSET                        0x410
#define MPFS_CFG_DDR_SGMII_PHY_OVRT5_OFFSET                        0x414
#define MPFS_CFG_DDR_SGMII_PHY_OVRT6_OFFSET                        0x418
#define MPFS_CFG_DDR_SGMII_PHY_OVRT7_OFFSET                        0x41c
#define MPFS_CFG_DDR_SGMII_PHY_OVRT8_OFFSET                        0x420
#define MPFS_CFG_DDR_SGMII_PHY_OVRT9_OFFSET                        0x424
#define MPFS_CFG_DDR_SGMII_PHY_OVRT10_OFFSET                       0x428
#define MPFS_CFG_DDR_SGMII_PHY_OVRT11_OFFSET                       0x42c
#define MPFS_CFG_DDR_SGMII_PHY_OVRT12_OFFSET                       0x430
#define MPFS_CFG_DDR_SGMII_PHY_OVRT13_OFFSET                       0x434
#define MPFS_CFG_DDR_SGMII_PHY_OVRT14_OFFSET                       0x438
#define MPFS_CFG_DDR_SGMII_PHY_OVRT15_OFFSET                       0x43c
#define MPFS_CFG_DDR_SGMII_PHY_OVRT16_OFFSET                       0x440
#define MPFS_CFG_DDR_SGMII_PHY_RPC17_OFFSET                        0x444
#define MPFS_CFG_DDR_SGMII_PHY_RPC18_OFFSET                        0x448
#define MPFS_CFG_DDR_SGMII_PHY_RPC19_OFFSET                        0x44c
#define MPFS_CFG_DDR_SGMII_PHY_RPC20_OFFSET                        0x450
#define MPFS_CFG_DDR_SGMII_PHY_RPC21_OFFSET                        0x454
#define MPFS_CFG_DDR_SGMII_PHY_RPC22_OFFSET                        0x458
#define MPFS_CFG_DDR_SGMII_PHY_RPC23_OFFSET                        0x45c
#define MPFS_CFG_DDR_SGMII_PHY_RPC24_OFFSET                        0x460
#define MPFS_CFG_DDR_SGMII_PHY_RPC25_OFFSET                        0x464
#define MPFS_CFG_DDR_SGMII_PHY_RPC26_OFFSET                        0x468
#define MPFS_CFG_DDR_SGMII_PHY_RPC27_OFFSET                        0x46c
#define MPFS_CFG_DDR_SGMII_PHY_RPC28_OFFSET                        0x470
#define MPFS_CFG_DDR_SGMII_PHY_RPC29_OFFSET                        0x474
#define MPFS_CFG_DDR_SGMII_PHY_RPC30_OFFSET                        0x478
#define MPFS_CFG_DDR_SGMII_PHY_RPC31_OFFSET                        0x47c
#define MPFS_CFG_DDR_SGMII_PHY_RPC32_OFFSET                        0x480
#define MPFS_CFG_DDR_SGMII_PHY_RPC33_OFFSET                        0x484
#define MPFS_CFG_DDR_SGMII_PHY_RPC34_OFFSET                        0x488
#define MPFS_CFG_DDR_SGMII_PHY_RPC35_OFFSET                        0x48c
#define MPFS_CFG_DDR_SGMII_PHY_RPC36_OFFSET                        0x490
#define MPFS_CFG_DDR_SGMII_PHY_RPC37_OFFSET                        0x494
#define MPFS_CFG_DDR_SGMII_PHY_RPC38_OFFSET                        0x498
#define MPFS_CFG_DDR_SGMII_PHY_RPC39_OFFSET                        0x49c
#define MPFS_CFG_DDR_SGMII_PHY_RPC40_OFFSET                        0x4a0
#define MPFS_CFG_DDR_SGMII_PHY_RPC41_OFFSET                        0x4a4
#define MPFS_CFG_DDR_SGMII_PHY_RPC42_OFFSET                        0x4a8
#define MPFS_CFG_DDR_SGMII_PHY_RPC43_OFFSET                        0x4ac
#define MPFS_CFG_DDR_SGMII_PHY_RPC44_OFFSET                        0x4b0
#define MPFS_CFG_DDR_SGMII_PHY_RPC45_OFFSET                        0x4b4
#define MPFS_CFG_DDR_SGMII_PHY_RPC46_OFFSET                        0x4b8
#define MPFS_CFG_DDR_SGMII_PHY_RPC47_OFFSET                        0x4bc
#define MPFS_CFG_DDR_SGMII_PHY_RPC48_OFFSET                        0x4c0
#define MPFS_CFG_DDR_SGMII_PHY_RPC49_OFFSET                        0x4c4
#define MPFS_CFG_DDR_SGMII_PHY_RPC50_OFFSET                        0x4c8
#define MPFS_CFG_DDR_SGMII_PHY_RPC51_OFFSET                        0x4cc
#define MPFS_CFG_DDR_SGMII_PHY_RPC52_OFFSET                        0x4d0
#define MPFS_CFG_DDR_SGMII_PHY_RPC53_OFFSET                        0x4d4
#define MPFS_CFG_DDR_SGMII_PHY_RPC54_OFFSET                        0x4d8
#define MPFS_CFG_DDR_SGMII_PHY_RPC55_OFFSET                        0x4dc
#define MPFS_CFG_DDR_SGMII_PHY_RPC56_OFFSET                        0x4e0
#define MPFS_CFG_DDR_SGMII_PHY_RPC57_OFFSET                        0x4e4
#define MPFS_CFG_DDR_SGMII_PHY_RPC58_OFFSET                        0x4e8
#define MPFS_CFG_DDR_SGMII_PHY_RPC59_OFFSET                        0x4ec
#define MPFS_CFG_DDR_SGMII_PHY_RPC60_OFFSET                        0x4f0
#define MPFS_CFG_DDR_SGMII_PHY_RPC61_OFFSET                        0x4f4
#define MPFS_CFG_DDR_SGMII_PHY_RPC62_OFFSET                        0x4f8
#define MPFS_CFG_DDR_SGMII_PHY_RPC63_OFFSET                        0x4fc
#define MPFS_CFG_DDR_SGMII_PHY_RPC64_OFFSET                        0x500
#define MPFS_CFG_DDR_SGMII_PHY_RPC65_OFFSET                        0x504
#define MPFS_CFG_DDR_SGMII_PHY_RPC66_OFFSET                        0x508
#define MPFS_CFG_DDR_SGMII_PHY_RPC67_OFFSET                        0x50c
#define MPFS_CFG_DDR_SGMII_PHY_RPC68_OFFSET                        0x510
#define MPFS_CFG_DDR_SGMII_PHY_RPC69_OFFSET                        0x514
#define MPFS_CFG_DDR_SGMII_PHY_RPC70_OFFSET                        0x518
#define MPFS_CFG_DDR_SGMII_PHY_RPC71_OFFSET                        0x51c
#define MPFS_CFG_DDR_SGMII_PHY_RPC72_OFFSET                        0x520
#define MPFS_CFG_DDR_SGMII_PHY_RPC73_OFFSET                        0x524
#define MPFS_CFG_DDR_SGMII_PHY_RPC74_OFFSET                        0x528
#define MPFS_CFG_DDR_SGMII_PHY_RPC75_OFFSET                        0x52c
#define MPFS_CFG_DDR_SGMII_PHY_RPC76_OFFSET                        0x530
#define MPFS_CFG_DDR_SGMII_PHY_RPC77_OFFSET                        0x534
#define MPFS_CFG_DDR_SGMII_PHY_RPC78_OFFSET                        0x538
#define MPFS_CFG_DDR_SGMII_PHY_RPC79_OFFSET                        0x53c
#define MPFS_CFG_DDR_SGMII_PHY_RPC80_OFFSET                        0x540
#define MPFS_CFG_DDR_SGMII_PHY_RPC81_OFFSET                        0x544
#define MPFS_CFG_DDR_SGMII_PHY_RPC82_OFFSET                        0x548
#define MPFS_CFG_DDR_SGMII_PHY_RPC83_OFFSET                        0x54c
#define MPFS_CFG_DDR_SGMII_PHY_RPC84_OFFSET                        0x550
#define MPFS_CFG_DDR_SGMII_PHY_RPC85_OFFSET                        0x554
#define MPFS_CFG_DDR_SGMII_PHY_RPC86_OFFSET                        0x558
#define MPFS_CFG_DDR_SGMII_PHY_RPC87_OFFSET                        0x55c
#define MPFS_CFG_DDR_SGMII_PHY_RPC88_OFFSET                        0x560
#define MPFS_CFG_DDR_SGMII_PHY_RPC89_OFFSET                        0x564
#define MPFS_CFG_DDR_SGMII_PHY_RPC90_OFFSET                        0x568
#define MPFS_CFG_DDR_SGMII_PHY_RPC91_OFFSET                        0x56c
#define MPFS_CFG_DDR_SGMII_PHY_RPC92_OFFSET                        0x570
#define MPFS_CFG_DDR_SGMII_PHY_RPC93_OFFSET                        0x574
#define MPFS_CFG_DDR_SGMII_PHY_RPC94_OFFSET                        0x578
#define MPFS_CFG_DDR_SGMII_PHY_RPC95_OFFSET                        0x57c
#define MPFS_CFG_DDR_SGMII_PHY_RPC96_OFFSET                        0x580
#define MPFS_CFG_DDR_SGMII_PHY_RPC97_OFFSET                        0x584
#define MPFS_CFG_DDR_SGMII_PHY_RPC98_OFFSET                        0x588
#define MPFS_CFG_DDR_SGMII_PHY_RPC99_OFFSET                        0x58c
#define MPFS_CFG_DDR_SGMII_PHY_RPC100_OFFSET                       0x590
#define MPFS_CFG_DDR_SGMII_PHY_RPC101_OFFSET                       0x594
#define MPFS_CFG_DDR_SGMII_PHY_RPC102_OFFSET                       0x598
#define MPFS_CFG_DDR_SGMII_PHY_RPC103_OFFSET                       0x59c
#define MPFS_CFG_DDR_SGMII_PHY_RPC104_OFFSET                       0x5a0
#define MPFS_CFG_DDR_SGMII_PHY_RPC105_OFFSET                       0x5a4
#define MPFS_CFG_DDR_SGMII_PHY_RPC106_OFFSET                       0x5a8
#define MPFS_CFG_DDR_SGMII_PHY_RPC107_OFFSET                       0x5ac
#define MPFS_CFG_DDR_SGMII_PHY_RPC108_OFFSET                       0x5b0
#define MPFS_CFG_DDR_SGMII_PHY_RPC109_OFFSET                       0x5b4
#define MPFS_CFG_DDR_SGMII_PHY_RPC110_OFFSET                       0x5b8
#define MPFS_CFG_DDR_SGMII_PHY_RPC111_OFFSET                       0x5bc
#define MPFS_CFG_DDR_SGMII_PHY_RPC112_OFFSET                       0x5c0
#define MPFS_CFG_DDR_SGMII_PHY_RPC113_OFFSET                       0x5c4
#define MPFS_CFG_DDR_SGMII_PHY_RPC114_OFFSET                       0x5c8
#define MPFS_CFG_DDR_SGMII_PHY_RPC115_OFFSET                       0x5cc
#define MPFS_CFG_DDR_SGMII_PHY_RPC116_OFFSET                       0x5d0
#define MPFS_CFG_DDR_SGMII_PHY_RPC117_OFFSET                       0x5d4
#define MPFS_CFG_DDR_SGMII_PHY_RPC118_OFFSET                       0x5d8
#define MPFS_CFG_DDR_SGMII_PHY_RPC119_OFFSET                       0x5dc
#define MPFS_CFG_DDR_SGMII_PHY_RPC120_OFFSET                       0x5e0
#define MPFS_CFG_DDR_SGMII_PHY_RPC121_OFFSET                       0x5e4
#define MPFS_CFG_DDR_SGMII_PHY_RPC122_OFFSET                       0x5e8
#define MPFS_CFG_DDR_SGMII_PHY_RPC123_OFFSET                       0x5ec
#define MPFS_CFG_DDR_SGMII_PHY_RPC124_OFFSET                       0x5f0
#define MPFS_CFG_DDR_SGMII_PHY_RPC125_OFFSET                       0x5f4
#define MPFS_CFG_DDR_SGMII_PHY_RPC126_OFFSET                       0x5f8
#define MPFS_CFG_DDR_SGMII_PHY_RPC127_OFFSET                       0x5fc
#define MPFS_CFG_DDR_SGMII_PHY_RPC128_OFFSET                       0x600
#define MPFS_CFG_DDR_SGMII_PHY_RPC129_OFFSET                       0x604
#define MPFS_CFG_DDR_SGMII_PHY_RPC130_OFFSET                       0x608
#define MPFS_CFG_DDR_SGMII_PHY_RPC131_OFFSET                       0x60c
#define MPFS_CFG_DDR_SGMII_PHY_RPC132_OFFSET                       0x610
#define MPFS_CFG_DDR_SGMII_PHY_RPC133_OFFSET                       0x614
#define MPFS_CFG_DDR_SGMII_PHY_RPC134_OFFSET                       0x618
#define MPFS_CFG_DDR_SGMII_PHY_RPC135_OFFSET                       0x61c
#define MPFS_CFG_DDR_SGMII_PHY_RPC136_OFFSET                       0x620
#define MPFS_CFG_DDR_SGMII_PHY_RPC137_OFFSET                       0x624
#define MPFS_CFG_DDR_SGMII_PHY_RPC138_OFFSET                       0x628
#define MPFS_CFG_DDR_SGMII_PHY_RPC139_OFFSET                       0x62c
#define MPFS_CFG_DDR_SGMII_PHY_RPC140_OFFSET                       0x630
#define MPFS_CFG_DDR_SGMII_PHY_RPC141_OFFSET                       0x634
#define MPFS_CFG_DDR_SGMII_PHY_RPC142_OFFSET                       0x638
#define MPFS_CFG_DDR_SGMII_PHY_RPC143_OFFSET                       0x63c
#define MPFS_CFG_DDR_SGMII_PHY_RPC144_OFFSET                       0x640
#define MPFS_CFG_DDR_SGMII_PHY_RPC145_OFFSET                       0x644
#define MPFS_CFG_DDR_SGMII_PHY_RPC146_OFFSET                       0x648
#define MPFS_CFG_DDR_SGMII_PHY_RPC147_OFFSET                       0x64c
#define MPFS_CFG_DDR_SGMII_PHY_RPC148_OFFSET                       0x650
#define MPFS_CFG_DDR_SGMII_PHY_RPC149_OFFSET                       0x654
#define MPFS_CFG_DDR_SGMII_PHY_RPC150_OFFSET                       0x658
#define MPFS_CFG_DDR_SGMII_PHY_RPC151_OFFSET                       0x65c
#define MPFS_CFG_DDR_SGMII_PHY_RPC152_OFFSET                       0x660
#define MPFS_CFG_DDR_SGMII_PHY_RPC153_OFFSET                       0x664
#define MPFS_CFG_DDR_SGMII_PHY_RPC154_OFFSET                       0x668
#define MPFS_CFG_DDR_SGMII_PHY_RPC155_OFFSET                       0x66c
#define MPFS_CFG_DDR_SGMII_PHY_RPC156_OFFSET                       0x670
#define MPFS_CFG_DDR_SGMII_PHY_RPC157_OFFSET                       0x674
#define MPFS_CFG_DDR_SGMII_PHY_RPC158_OFFSET                       0x678
#define MPFS_CFG_DDR_SGMII_PHY_RPC159_OFFSET                       0x67c
#define MPFS_CFG_DDR_SGMII_PHY_RPC160_OFFSET                       0x680
#define MPFS_CFG_DDR_SGMII_PHY_RPC161_OFFSET                       0x684
#define MPFS_CFG_DDR_SGMII_PHY_RPC162_OFFSET                       0x688
#define MPFS_CFG_DDR_SGMII_PHY_RPC163_OFFSET                       0x68c
#define MPFS_CFG_DDR_SGMII_PHY_RPC164_OFFSET                       0x690
#define MPFS_CFG_DDR_SGMII_PHY_RPC165_OFFSET                       0x694
#define MPFS_CFG_DDR_SGMII_PHY_RPC166_OFFSET                       0x698
#define MPFS_CFG_DDR_SGMII_PHY_RPC167_OFFSET                       0x69c
#define MPFS_CFG_DDR_SGMII_PHY_RPC168_OFFSET                       0x6a0
#define MPFS_CFG_DDR_SGMII_PHY_RPC169_OFFSET                       0x6a4
#define MPFS_CFG_DDR_SGMII_PHY_RPC170_OFFSET                       0x6a8
#define MPFS_CFG_DDR_SGMII_PHY_RPC171_OFFSET                       0x6ac
#define MPFS_CFG_DDR_SGMII_PHY_RPC172_OFFSET                       0x6b0
#define MPFS_CFG_DDR_SGMII_PHY_RPC173_OFFSET                       0x6b4
#define MPFS_CFG_DDR_SGMII_PHY_RPC174_OFFSET                       0x6b8
#define MPFS_CFG_DDR_SGMII_PHY_RPC175_OFFSET                       0x6bc
#define MPFS_CFG_DDR_SGMII_PHY_RPC176_OFFSET                       0x6c0
#define MPFS_CFG_DDR_SGMII_PHY_RPC177_OFFSET                       0x6c4
#define MPFS_CFG_DDR_SGMII_PHY_RPC178_OFFSET                       0x6c8
#define MPFS_CFG_DDR_SGMII_PHY_RPC179_OFFSET                       0x6cc
#define MPFS_CFG_DDR_SGMII_PHY_RPC180_OFFSET                       0x6d0
#define MPFS_CFG_DDR_SGMII_PHY_RPC181_OFFSET                       0x6d4
#define MPFS_CFG_DDR_SGMII_PHY_RPC182_OFFSET                       0x6d8
#define MPFS_CFG_DDR_SGMII_PHY_RPC183_OFFSET                       0x6dc
#define MPFS_CFG_DDR_SGMII_PHY_RPC184_OFFSET                       0x6e0
#define MPFS_CFG_DDR_SGMII_PHY_RPC185_OFFSET                       0x6e4
#define MPFS_CFG_DDR_SGMII_PHY_RPC186_OFFSET                       0x6e8
#define MPFS_CFG_DDR_SGMII_PHY_RPC187_OFFSET                       0x6ec
#define MPFS_CFG_DDR_SGMII_PHY_RPC188_OFFSET                       0x6f0
#define MPFS_CFG_DDR_SGMII_PHY_RPC189_OFFSET                       0x6f4
#define MPFS_CFG_DDR_SGMII_PHY_RPC190_OFFSET                       0x6f8
#define MPFS_CFG_DDR_SGMII_PHY_RPC191_OFFSET                       0x6fc
#define MPFS_CFG_DDR_SGMII_PHY_RPC192_OFFSET                       0x700
#define MPFS_CFG_DDR_SGMII_PHY_RPC193_OFFSET                       0x704
#define MPFS_CFG_DDR_SGMII_PHY_RPC194_OFFSET                       0x708
#define MPFS_CFG_DDR_SGMII_PHY_RPC195_OFFSET                       0x70c
#define MPFS_CFG_DDR_SGMII_PHY_RPC196_OFFSET                       0x710
#define MPFS_CFG_DDR_SGMII_PHY_RPC197_OFFSET                       0x714
#define MPFS_CFG_DDR_SGMII_PHY_RPC198_OFFSET                       0x718
#define MPFS_CFG_DDR_SGMII_PHY_RPC199_OFFSET                       0x71c
#define MPFS_CFG_DDR_SGMII_PHY_RPC200_OFFSET                       0x720
#define MPFS_CFG_DDR_SGMII_PHY_RPC201_OFFSET                       0x724
#define MPFS_CFG_DDR_SGMII_PHY_RPC202_OFFSET                       0x728
#define MPFS_CFG_DDR_SGMII_PHY_RPC203_OFFSET                       0x72c
#define MPFS_CFG_DDR_SGMII_PHY_RPC204_OFFSET                       0x730
#define MPFS_CFG_DDR_SGMII_PHY_RPC205_OFFSET                       0x734
#define MPFS_CFG_DDR_SGMII_PHY_RPC206_OFFSET                       0x738
#define MPFS_CFG_DDR_SGMII_PHY_RPC207_OFFSET                       0x73c
#define MPFS_CFG_DDR_SGMII_PHY_RPC208_OFFSET                       0x740
#define MPFS_CFG_DDR_SGMII_PHY_RPC209_OFFSET                       0x744
#define MPFS_CFG_DDR_SGMII_PHY_RPC210_OFFSET                       0x748
#define MPFS_CFG_DDR_SGMII_PHY_RPC211_OFFSET                       0x74c
#define MPFS_CFG_DDR_SGMII_PHY_RPC212_OFFSET                       0x750
#define MPFS_CFG_DDR_SGMII_PHY_RPC213_OFFSET                       0x754
#define MPFS_CFG_DDR_SGMII_PHY_RPC214_OFFSET                       0x758
#define MPFS_CFG_DDR_SGMII_PHY_RPC215_OFFSET                       0x75c
#define MPFS_CFG_DDR_SGMII_PHY_RPC216_OFFSET                       0x760
#define MPFS_CFG_DDR_SGMII_PHY_RPC217_OFFSET                       0x764
#define MPFS_CFG_DDR_SGMII_PHY_RPC218_OFFSET                       0x768
#define MPFS_CFG_DDR_SGMII_PHY_RPC219_OFFSET                       0x76c
#define MPFS_CFG_DDR_SGMII_PHY_RPC220_OFFSET                       0x770
#define MPFS_CFG_DDR_SGMII_PHY_RPC221_OFFSET                       0x774
#define MPFS_CFG_DDR_SGMII_PHY_RPC222_OFFSET                       0x778
#define MPFS_CFG_DDR_SGMII_PHY_RPC223_OFFSET                       0x77c
#define MPFS_CFG_DDR_SGMII_PHY_RPC224_OFFSET                       0x780
#define MPFS_CFG_DDR_SGMII_PHY_RPC225_OFFSET                       0x784
#define MPFS_CFG_DDR_SGMII_PHY_RPC226_OFFSET                       0x788
#define MPFS_CFG_DDR_SGMII_PHY_RPC227_OFFSET                       0x78c
#define MPFS_CFG_DDR_SGMII_PHY_RPC228_OFFSET                       0x790
#define MPFS_CFG_DDR_SGMII_PHY_RPC229_OFFSET                       0x794
#define MPFS_CFG_DDR_SGMII_PHY_RPC230_OFFSET                       0x798
#define MPFS_CFG_DDR_SGMII_PHY_RPC231_OFFSET                       0x79c
#define MPFS_CFG_DDR_SGMII_PHY_RPC232_OFFSET                       0x7a0
#define MPFS_CFG_DDR_SGMII_PHY_RPC233_OFFSET                       0x7a4
#define MPFS_CFG_DDR_SGMII_PHY_RPC234_OFFSET                       0x7a8
#define MPFS_CFG_DDR_SGMII_PHY_RPC235_OFFSET                       0x7ac
#define MPFS_CFG_DDR_SGMII_PHY_RPC236_OFFSET                       0x7b0
#define MPFS_CFG_DDR_SGMII_PHY_RPC237_OFFSET                       0x7b4
#define MPFS_CFG_DDR_SGMII_PHY_RPC238_OFFSET                       0x7b8
#define MPFS_CFG_DDR_SGMII_PHY_RPC239_OFFSET                       0x7bc
#define MPFS_CFG_DDR_SGMII_PHY_RPC240_OFFSET                       0x7c0
#define MPFS_CFG_DDR_SGMII_PHY_RPC241_OFFSET                       0x7c4
#define MPFS_CFG_DDR_SGMII_PHY_RPC242_OFFSET                       0x7c8
#define MPFS_CFG_DDR_SGMII_PHY_RPC243_OFFSET                       0x7cc
#define MPFS_CFG_DDR_SGMII_PHY_RPC244_OFFSET                       0x7d0
#define MPFS_CFG_DDR_SGMII_PHY_RPC245_OFFSET                       0x7d4
#define MPFS_CFG_DDR_SGMII_PHY_RPC246_OFFSET                       0x7d8
#define MPFS_CFG_DDR_SGMII_PHY_RPC247_OFFSET                       0x7dc
#define MPFS_CFG_DDR_SGMII_PHY_RPC248_OFFSET                       0x7e0
#define MPFS_CFG_DDR_SGMII_PHY_RPC249_OFFSET                       0x7e4
#define MPFS_CFG_DDR_SGMII_PHY_RPC250_OFFSET                       0x7e8
#define MPFS_CFG_DDR_SGMII_PHY_SPIO251_OFFSET                      0x7ec
#define MPFS_CFG_DDR_SGMII_PHY_SPIO252_OFFSET                      0x7f0
#define MPFS_CFG_DDR_SGMII_PHY_SPIO253_OFFSET                      0x7f4
#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_TIP_OFFSET               0x800
#define MPFS_CFG_DDR_SGMII_PHY_RANK_SELECT_OFFSET                  0x804
#define MPFS_CFG_DDR_SGMII_PHY_LANE_SELECT_OFFSET                  0x808
#define MPFS_CFG_DDR_SGMII_PHY_TRAINING_SKIP_OFFSET                0x80c
#define MPFS_CFG_DDR_SGMII_PHY_TRAINING_START_OFFSET               0x810
#define MPFS_CFG_DDR_SGMII_PHY_TRAINING_STATUS_OFFSET              0x814
#define MPFS_CFG_DDR_SGMII_PHY_TRAINING_RESET_OFFSET               0x818
#define MPFS_CFG_DDR_SGMII_PHY_GT_ERR_COMB_OFFSET                  0x81c
#define MPFS_CFG_DDR_SGMII_PHY_GT_CLK_SEL_OFFSET                   0x820
#define MPFS_CFG_DDR_SGMII_PHY_GT_TXDLY_OFFSET                     0x824
#define MPFS_CFG_DDR_SGMII_PHY_GT_STEPS_180_OFFSET                 0x828
#define MPFS_CFG_DDR_SGMII_PHY_GT_STATE_OFFSET                     0x82c
#define MPFS_CFG_DDR_SGMII_PHY_WL_DELAY_0_OFFSET                   0x830
#define MPFS_CFG_DDR_SGMII_PHY_DQ_DQS_ERR_DONE_OFFSET              0x834
#define MPFS_CFG_DDR_SGMII_PHY_DQDQS_WINDOW_OFFSET                 0x838
#define MPFS_CFG_DDR_SGMII_PHY_DQDQS_STATE_OFFSET                  0x83c
#define MPFS_CFG_DDR_SGMII_PHY_DELTA0_OFFSET                       0x840
#define MPFS_CFG_DDR_SGMII_PHY_DELTA1_OFFSET                       0x844
#define MPFS_CFG_DDR_SGMII_PHY_DQDQS_STATUS0_OFFSET                0x848
#define MPFS_CFG_DDR_SGMII_PHY_DQDQS_STATUS1_OFFSET                0x84c
#define MPFS_CFG_DDR_SGMII_PHY_DQDQS_STATUS2_OFFSET                0x850
#define MPFS_CFG_DDR_SGMII_PHY_DQDQS_STATUS3_OFFSET                0x854
#define MPFS_CFG_DDR_SGMII_PHY_DQDQS_STATUS4_OFFSET                0x858
#define MPFS_CFG_DDR_SGMII_PHY_DQDQS_STATUS5_OFFSET                0x85c
#define MPFS_CFG_DDR_SGMII_PHY_DQDQS_STATUS6_OFFSET                0x860
#define MPFS_CFG_DDR_SGMII_PHY_ADDCMD_STATUS0_OFFSET               0x864
#define MPFS_CFG_DDR_SGMII_PHY_ADDCMD_STATUS1_OFFSET               0x868
#define MPFS_CFG_DDR_SGMII_PHY_ADDCMD_ANSWER_OFFSET                0x86c
#define MPFS_CFG_DDR_SGMII_PHY_BCLKSCLK_ANSWER_OFFSET              0x870
#define MPFS_CFG_DDR_SGMII_PHY_DQDQS_WRCALIB_OFFSET_OFFSET         0x874
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_MODE_EN_OFFSET               0x878
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG0_OFFSET      0x87c
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1_OFFSET      0x880
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_DIRECTION_REG0_OFFSET 0x884
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_DIRECTION_REG1_OFFSET 0x888
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG0_OFFSET      0x88c
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1_OFFSET      0x890
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_OOR_REG0_OFFSET       0x894
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_OOR_REG1_OFFSET       0x898
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MV_RD_DLY_REG_OFFSET  0x89c
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_PAUSE_OFFSET          0x8a0
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT_OFFSET                0x8a4
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DQLANE_READBACK_OFFSET       0x8a8
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_ADDCMD_LN_READBACK_OFFSET    0x8ac
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_READ_GATE_CONTROLS_OFFSET    0x8b0
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DQ_DQS_OPTIMIZATION0_OFFSET  0x8b4
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DQ_DQS_OPTIMIZATION1_OFFSET  0x8b8
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_WRCALIB_OFFSET               0x8bc
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_CALIF_OFFSET                 0x8c0
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_CALIF_READBACK_OFFSET        0x8c4
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_CALIF_READBACK1_OFFSET       0x8c8
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DFI_STATUS_OVERRIDE_OFFSET   0x8cc
#define MPFS_CFG_DDR_SGMII_PHY_TIP_CFG_PARAMS_OFFSET               0x8d0
#define MPFS_CFG_DDR_SGMII_PHY_TIP_VREF_PARAM_OFFSET               0x8d4
#define MPFS_CFG_DDR_SGMII_PHY_LANE_ALIGNMENT_FIFO_CONTROL_OFFSET  0x8d8
#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_SGMII_OFFSET             0xc00
#define MPFS_CFG_DDR_SGMII_PHY_SGMII_MODE_OFFSET                   0xc04
#define MPFS_CFG_DDR_SGMII_PHY_PLL_CNTL_OFFSET                     0xc08
#define MPFS_CFG_DDR_SGMII_PHY_CH0_CNTL_OFFSET                     0xc0c
#define MPFS_CFG_DDR_SGMII_PHY_CH1_CNTL_OFFSET                     0xc10
#define MPFS_CFG_DDR_SGMII_PHY_RECAL_CNTL_OFFSET                   0xc14
#define MPFS_CFG_DDR_SGMII_PHY_CLK_CNTL_OFFSET                     0xc18
#define MPFS_CFG_DDR_SGMII_PHY_DYN_CNTL_OFFSET                     0xc1c
#define MPFS_CFG_DDR_SGMII_PHY_PVT_STAT_OFFSET                     0xc20
#define MPFS_CFG_DDR_SGMII_PHY_SPARE_CNTL_OFFSET                   0xc24
#define MPFS_CFG_DDR_SGMII_PHY_SPARE_STAT_OFFSET                   0xc28

#define MPFS_SYSREG_CLOCK_CONFIG_CR (MPFS_SYSREG_BASE + MPFS_SYSREG_CLOCK_CONFIG_CR_OFFSET)
#define MPFS_SYSREG_RTC_CLOCK_CR    (MPFS_SYSREG_BASE + MPFS_SYSREG_RTC_CLOCK_CR_OFFSET)
#define MPFS_SYSREG_ENVM_CR         (MPFS_SYSREG_BASE + MPFS_SYSREG_ENVM_CR_OFFSET)
#define MPFS_SYSREG_DFIAPB_CR       (MPFS_SYSREG_BASE + MPFS_SYSREG_DFIAPB_CR_OFFSET)

#define MPFS_IOSCBCFG_TIMER         (MPFS_IOSCBCFG_BASE + MPFS_IOSCBCFG_TIMER_OFFSET)

#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DDR_PHY (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DDR_PHY_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_STARTUP            (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_STARTUP_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_DYN_CNTL           (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_DYN_CNTL_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_SGMII_MODE         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_SGMII_MODE_OFFSET)

#define MPFS_CFG_DDR_SGMII_PHY_CH0_CNTL   (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_CH0_CNTL_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_CH1_CNTL   (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_CH1_CNTL_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RECAL_CNTL (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RECAL_CNTL_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_CLK_CNTL   (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_CLK_CNTL_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_PLL_CNTL   (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_PLL_CNTL_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_PVT_STAT   (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_PVT_STAT_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_SPARE_CNTL (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_SPARE_CNTL_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_SPARE_STAT (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_SPARE_STAT_OFFSET)

/* SGMII register definitions */

#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_DRIVER (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_DRIVER_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_ODT    (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_ODT_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_IO     (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DECODER_IO_OFFSET)

#define MPFS_CFG_DDR_SGMII_PHY_TRAINING_START (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_TRAINING_START_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_DDRPHY_MODE    (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_DDRPHY_MODE_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_DPC_BITS       (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_DPC_BITS_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC95          (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC95_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC96          (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC96_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC97          (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC97_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC98          (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC98_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_SPARE_0        (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_SPARE_0_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_SPIO253        (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_SPIO253_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC10_ODT      (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC10_ODT_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC11_ODT      (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC11_ODT_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_OVRT9          (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_OVRT9_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_OVRT10         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_OVRT10_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC245         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC245_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC237         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC237_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_OVRT11         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_OVRT11_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_OVRT12         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_OVRT12_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_OVRT13         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_OVRT13_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_OVRT14         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_OVRT14_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_OVRT15         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_OVRT15_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_OVRT16         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_OVRT16_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC27          (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC27_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC203         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC203_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC1_ODT       (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC1_ODT_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC2_ODT       (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC2_ODT_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC3_ODT       (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC3_ODT_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC4_ODT       (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC4_ODT_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC19          (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC19_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC20          (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC20_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC145         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC145_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC147         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC147_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC166         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC166_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC168         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC168_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC220         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC220_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC235         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC235_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC236         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC236_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC237         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC237_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC238         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC238_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC239         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC239_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC240         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC240_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC241         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC241_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC242         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC242_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC243         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC243_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC244         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC244_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC245         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC245_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC246         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC246_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC247         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC247_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC248         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC248_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC249         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC249_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC250         (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC250_OFFSET)

#define MPFS_CFG_DDR_SGMII_PHY_PLL_CTRL_MAIN    (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_PLL_CTRL_MAIN_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_PLL_REF_FB_MAIN  (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_PLL_REF_FB_MAIN_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_PLL_DIV_0_1_MAIN (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_PLL_DIV_0_1_MAIN_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_PLL_DIV_2_3_MAIN (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_PLL_DIV_2_3_MAIN_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_PLL_CTRL2_MAIN   (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_PLL_CTRL2_MAIN_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_PLL_PHADJ_MAIN   (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_PLL_PHADJ_MAIN_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_SSCG_REG_2_MAIN  (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_SSCG_REG_2_MAIN_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_TRAINING_RESET   (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_TRAINING_RESET_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_TRAINING_SKIP    (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_TRAINING_SKIP_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_TRAINING_STATUS  (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_TRAINING_STATUS_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_MODE_EN   (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_EXPERT_MODE_EN_OFFSET)

#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG0      (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG0_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1      (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT                (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_EXPERT_PLLCNT_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MV_RD_DLY_REG  (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MV_RD_DLY_REG_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG0      (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG0_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1      (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_LOAD_REG1_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_PAUSE          (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_PAUSE_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1      (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_MOVE_REG1_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_DIRECTION_REG1 (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_EXPERT_DLYCNT_DIRECTION_REG1_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_WRCALIB               (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_EXPERT_WRCALIB_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_TIP_CFG_PARAMS               (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_TIP_CFG_PARAMS_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_DFI_STATUS_OVERRIDE   (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_EXPERT_DFI_STATUS_OVERRIDE_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_EXPERT_ADDCMD_LN_READBACK    (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_EXPERT_ADDCMD_LN_READBACK_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_RPC1_DRV                     (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_RPC1_DRV_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_LANE_SELECT                  (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_LANE_SELECT_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_ADDCMD_STATUS0               (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_ADDCMD_STATUS0_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_ADDCMD_STATUS1               (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_ADDCMD_STATUS1_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_GT_ERR_COMB                  (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_GT_ERR_COMB_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_DQ_DQS_ERR_DONE              (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_DQ_DQS_ERR_DONE_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_DQDQS_STATUS1                (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_DQDQS_STATUS1_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_GT_CLK_SEL                   (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_GT_CLK_SEL_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_GT_TXDLY                     (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_GT_TXDLY_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_DYN_CNTL                     (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_DYN_CNTL_OFFSET)

#define MPFS_IOSCB_BANK_CNTL_SGMII_SOFT_RESET (MPFS_IOSCB_BANK_SGMII_BASE + MPFS_IOSCB_BANK_CNTL_SGMII_SOFT_RESET_OFFSET)
#define MPFS_IOSCB_BANK_CNTL_DDR_SOFT_RESET   (MOFS_IOSCB_BANK_DDR_BASE + MPFS_IOSCB_BANK_CNTL_DDR_SOFT_RESET_OFFSET)

#define MPFS_CFG_DDR_SGMII_PHY_IOC_REG0    (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_IOC_REG0_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_IOC_REG1    (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_IOC_REG1_OFFSET)
#define MPFS_CFG_DDR_SGMII_PHY_IOC_REG6    (MPFS_CFG_DDR_SGMII_PHY_BASE + MPFS_CFG_DDR_SGMII_PHY_IOC_REG6_OFFSET)

#define MPFS_SYSREGSCB_MSS_RESET_CR        (MPFS_SYSREGSCB_BASE + MPFS_SYSREGSCB_MSS_RESET_CR_OFFSET)
#define MPFS_SYSREGSCB_MSSIO_CONTROL_CR    (MPFS_SYSREGSCB_BASE + MPFS_SYSREGSCB_MSSIO_CONTROL_CR_OFFSET)

#define MPFS_SYSREGSCB_MSSIO_BANK2_CFG_CR  (MPFS_SYSREGSCB_BASE + MPFS_SYSREGSCB_MSSIO_BANK2_CFG_CR_OFFSET)
#define MPFS_SYSREGSCB_MSSIO_BANK4_CFG_CR  (MPFS_SYSREGSCB_BASE + MPFS_SYSREGSCB_MSSIO_BANK4_CFG_CR_OFFSET)

#define MPFS_IOSCB_DLL_SGMII_SOFT_RESET    (MPFS_IOSCB_DLL_SGMII_BASE + MPFS_IOSCB_DLL_SGMII_SOFT_RESET_OFFSET)

#define MPFS_IOSCB_SGMII_LANE01_SOFT_RESET (MPFS_IOSCB_SGMII_LANE01_BASE + MPFS_IOSCB_SGMII_LANE01_SOFT_RESET_OFFSET)
#define MPFS_IOSCB_SGMII_LANE23_SOFT_RESET (MPFS_IOSCB_SGMII_LANE23_BASE + MPFS_IOSCB_SGMII_LANE23_SOFT_RESET_OFFSET)

#define MPFS_IOSCB_CALIB_SGMII_SOFT_RESET  (MPFS_IOSCB_IO_CALIB_SGMII_BASE + MPFS_IOSCB_CALIB_SGMII_SOFT_RESET_OFFSET)
#define MPFS_IOSCB_CALIB_SGMII_IOC_REG0    (MPFS_IOSCB_IO_CALIB_SGMII_BASE + MPFS_IOSCB_CALIB_SGMII_IOC_REG0_OFFSET)

#define MPFS_IOSCB_CALIB_DDR_SOFT_RESET (MPFS_IOSCB_IO_CALIB_DDR_BASE + MPFS_IOSCB_CALIB_DDR_SOFT_RESET_OFFSET)
#define MPFS_IOSCB_CALIB_DDR_IOC_REG0   (MPFS_IOSCB_IO_CALIB_DDR_BASE + MPFS_IOSCB_CALIB_DDR_IOC_REG0_OFFSET)
#define MPFS_IOSCB_CALIB_DDR_IOC_REG1   (MPFS_IOSCB_IO_CALIB_DDR_BASE + MPFS_IOSCB_CALIB_DDR_IOC_REG1_OFFSET)
#define MPFS_IOSCB_CALIB_DDR_IOC_REG2   (MPFS_IOSCB_IO_CALIB_DDR_BASE + MPFS_IOSCB_CALIB_DDR_IOC_REG2_OFFSET)
#define MPFS_IOSCB_CALIB_DDR_IOC_REG3   (MPFS_IOSCB_IO_CALIB_DDR_BASE + MPFS_IOSCB_CALIB_DDR_IOC_REG3_OFFSET)
#define MPFS_IOSCB_CALIB_DDR_IOC_REG4   (MPFS_IOSCB_IO_CALIB_DDR_BASE + MPFS_IOSCB_CALIB_DDR_IOC_REG4_OFFSET)
#define MPFS_IOSCB_CALIB_DDR_IOC_REG5   (MPFS_IOSCB_IO_CALIB_DDR_BASE + MPFS_IOSCB_CALIB_DDR_IOC_REG5_OFFSET)
#define MPFS_IOSCB_CALIB_DDR_IOC_REG6   (MPFS_IOSCB_IO_CALIB_DDR_BASE + MPFS_IOSCB_CALIB_DDR_IOC_REG6_OFFSET)

#define MPFS_IOSCB_SGMII_MUX_SOFT_RESET   (MPFS_IOSCB_SGMII_MUX_BASE + MPFS_IOSCB_SGMII_MUX_SOFT_RESET_OFFSET)
#define MPFS_IOSCB_SGMII_MUX_SGMII_CLKMUX (MPFS_IOSCB_SGMII_MUX_BASE + MPFS_IOSCB_SGMII_MUX_SGMII_CLKMUX_OFFSET)
#define MPFS_IOSCB_SGMII_MUX_RFCKMUX      (MPFS_IOSCB_SGMII_MUX_BASE + MPFS_IOSCB_SGMII_MUX_RFCKMUX_OFFSET)
#define MPFS_IOSCB_SGMII_MUX_CLK_XCVR     (MPFS_IOSCB_SGMII_MUX_BASE + MPFS_IOSCB_SGMII_MUX_CLK_XCVR_OFFSET)

#define MPFS_IOSCB_MSS_MUX_SOFT_RESET    (MPFS_IOSCB_MSS_MUX_BASE + MPFS_IOSCB_MSS_MUX_SOFT_RESET_OFFSET)
#define MPFS_IOSCB_MSS_MUX_BCLKMUX       (MPFS_IOSCB_MSS_MUX_BASE + MPFS_IOSCB_MSS_MUX_BCLKMUX_OFFSET)
#define MPFS_IOSCB_MSS_MUX_PLL_CKMUX     (MPFS_IOSCB_MSS_MUX_BASE + MPFS_IOSCB_MSS_MUX_PLL_CKMUX_OFFSET)
#define MPFS_IOSCB_MSS_MUX_MSSCLKMUX     (MPFS_IOSCB_MSS_MUX_BASE + MPFS_IOSCB_MSS_MUX_MSSCLKMUX_OFFSET)
#define MPFS_IOSCB_MSS_MUX_SPARE0        (MPFS_IOSCB_MSS_MUX_BASE + MPFS_IOSCB_MSS_MUX_SPARE0_OFFSET)
#define MPFS_IOSCB_MSS_MUX_FMETER_ADDR   (MPFS_IOSCB_MSS_MUX_BASE + MPFS_IOSCB_MSS_MUX_FMETER_ADDR_OFFSET)
#define MPFS_IOSCB_MSS_MUX_FMETER_DATAW  (MPFS_IOSCB_MSS_MUX_BASE + MPFS_IOSCB_MSS_MUX_FMETER_DATAW_OFFSET)
#define MPFS_IOSCB_MSS_MUX_FMETER_DATAR  (MPFS_IOSCB_MSS_MUX_BASE + MPFS_IOSCB_MSS_MUX_FMETER_DATAR_OFFSET)
#define MPFS_IOSCB_MSS_MUX_TEST_CTRL     (MPFS_IOSCB_MSS_MUX_BASE + MPFS_IOSCB_MSS_MUX_TEST_CTRL_OFFSET)

#define MPFS_IOSCB_SGMII_PLL_SOFT_RESET  (MPFS_IOSCB_SGMII_PLL_BASE + MPFS_IOSCB_SGMII_PLL_SOFT_RESET_OFFSET)
#define MPFS_IOSCB_SGMII_PLL_CTRL        (MPFS_IOSCB_SGMII_PLL_BASE + MPFS_IOSCB_SGMII_PLL_CTRL_OFFSET)
#define MPFS_IOSCB_SGMII_PLL_CTRL2       (MPFS_IOSCB_SGMII_PLL_BASE + MPFS_IOSCB_SGMII_PLL_CTRL2_OFFSET)
#define MPFS_IOSCB_SGMII_PLL_REF_FB      (MPFS_IOSCB_SGMII_PLL_BASE + MPFS_IOSCB_SGMII_PLL_REF_FB_OFFSET)
#define MPFS_IOSCB_SGMII_PLL_DIV_0_1     (MPFS_IOSCB_SGMII_PLL_BASE + MPFS_IOSCB_SGMII_PLL_DIV_0_1_OFFSET)
#define MPFS_IOSCB_SGMII_PLL_DIV_2_3     (MPFS_IOSCB_SGMII_PLL_BASE + MPFS_IOSCB_SGMII_PLL_DIV_2_3_OFFSET)
#define MPFS_IOSCB_SGMII_PLL_FRACN       (MPFS_IOSCB_SGMII_PLL_BASE + MPFS_IOSCB_SGMII_PLL_FRACN_OFFSET)
#define MPFS_IOSCB_SGMII_PLL_PHADJ       (MPFS_IOSCB_SGMII_PLL_BASE + MPFS_IOSCB_SGMII_PLL_PHADJ_OFFSET)
#define MPFS_IOSCB_SGMII_PLL_SSCG_REG_0  (MPFS_IOSCB_SGMII_PLL_BASE + MPFS_IOSCB_SGMII_PLL_SSCG_REG_0_OFFSET)
#define MPFS_IOSCB_SGMII_PLL_SSCG_REG_1  (MPFS_IOSCB_SGMII_PLL_BASE + MPFS_IOSCB_SGMII_PLL_SSCG_REG_1_OFFSET)
#define MPFS_IOSCB_SGMII_PLL_SSCG_REG_2  (MPFS_IOSCB_SGMII_PLL_BASE + MPFS_IOSCB_SGMII_PLL_SSCG_REG_2_OFFSET)
#define MPFS_IOSCB_SGMII_PLL_SSCG_REG_3  (MPFS_IOSCB_SGMII_PLL_BASE + MPFS_IOSCB_SGMII_PLL_SSCG_REG_3_OFFSET)

#define MPFS_IOSCB_MSS_PLL_SOFT_RESET  (MPFS_IOSCB_MSS_PLL_BASE + MPFS_IOSCB_MSS_PLL_SOFT_RESET_OFFSET)
#define MPFS_IOSCB_MSS_PLL_CTRL        (MPFS_IOSCB_MSS_PLL_BASE + MPFS_IOSCB_MSS_PLL_CTRL_OFFSET)
#define MPFS_IOSCB_MSS_PLL_CTRL2       (MPFS_IOSCB_MSS_PLL_BASE + MPFS_IOSCB_MSS_PLL_CTRL2_OFFSET)
#define MPFS_IOSCB_MSS_PLL_REF_FB      (MPFS_IOSCB_MSS_PLL_BASE + MPFS_IOSCB_MSS_PLL_REF_FB_OFFSET)
#define MPFS_IOSCB_MSS_PLL_DIV_0_1     (MPFS_IOSCB_MSS_PLL_BASE + MPFS_IOSCB_MSS_PLL_DIV_0_1_OFFSET)
#define MPFS_IOSCB_MSS_PLL_DIV_2_3     (MPFS_IOSCB_MSS_PLL_BASE + MPFS_IOSCB_MSS_PLL_DIV_2_3_OFFSET)
#define MPFS_IOSCB_MSS_PLL_FRACN       (MPFS_IOSCB_MSS_PLL_BASE + MPFS_IOSCB_MSS_PLL_FRACN_OFFSET)
#define MPFS_IOSCB_MSS_PLL_PHADJ       (MPFS_IOSCB_MSS_PLL_BASE + MPFS_IOSCB_MSS_PLL_PHADJ_OFFSET)
#define MPFS_IOSCB_MSS_PLL_SSCG_REG_0  (MPFS_IOSCB_MSS_PLL_BASE + MPFS_IOSCB_MSS_PLL_SSCG_REG_0_OFFSET)
#define MPFS_IOSCB_MSS_PLL_SSCG_REG_1  (MPFS_IOSCB_MSS_PLL_BASE + MPFS_IOSCB_MSS_PLL_SSCG_REG_1_OFFSET)
#define MPFS_IOSCB_MSS_PLL_SSCG_REG_2  (MPFS_IOSCB_MSS_PLL_BASE + MPFS_IOSCB_MSS_PLL_SSCG_REG_2_OFFSET)
#define MPFS_IOSCB_MSS_PLL_SSCG_REG_3  (MPFS_IOSCB_MSS_PLL_BASE + MPFS_IOSCB_MSS_PLL_SSCG_REG_3_OFFSET)

#define MPFS_IOSCB_DDR_PLL_SOFT_RESET  (MPFS_IOSCB_DDR_PLL_BASE + MPFS_IOSCB_DDR_PLL_SOFT_RESET_OFFSET)
#define MPFS_IOSCB_DDR_PLL_CTRL        (MPFS_IOSCB_DDR_PLL_BASE + MPFS_IOSCB_DDR_PLL_CTRL_OFFSET)
#define MPFS_IOSCB_DDR_PLL_CTRL2       (MPFS_IOSCB_DDR_PLL_BASE + MPFS_IOSCB_DDR_PLL_CTRL2_OFFSET)
#define MPFS_IOSCB_DDR_PLL_REF_FB      (MPFS_IOSCB_DDR_PLL_BASE + MPFS_IOSCB_DDR_PLL_REF_FB_OFFSET)
#define MPFS_IOSCB_DDR_PLL_DIV_0_1     (MPFS_IOSCB_DDR_PLL_BASE + MPFS_IOSCB_DDR_PLL_DIV_0_1_OFFSET)
#define MPFS_IOSCB_DDR_PLL_DIV_2_3     (MPFS_IOSCB_DDR_PLL_BASE + MPFS_IOSCB_DDR_PLL_DIV_2_3_OFFSET)
#define MPFS_IOSCB_DDR_PLL_FRACN       (MPFS_IOSCB_DDR_PLL_BASE + MPFS_IOSCB_DDR_PLL_FRACN_OFFSET)
#define MPFS_IOSCB_DDR_PLL_PHADJ       (MPFS_IOSCB_DDR_PLL_BASE + MPFS_IOSCB_DDR_PLL_PHADJ_OFFSET)
#define MPFS_IOSCB_DDR_PLL_SSCG_REG_0  (MPFS_IOSCB_DDR_PLL_BASE + MPFS_IOSCB_DDR_PLL_SSCG_REG_0_OFFSET)
#define MPFS_IOSCB_DDR_PLL_SSCG_REG_1  (MPFS_IOSCB_DDR_PLL_BASE + MPFS_IOSCB_DDR_PLL_SSCG_REG_1_OFFSET)
#define MPFS_IOSCB_DDR_PLL_SSCG_REG_2  (MPFS_IOSCB_DDR_PLL_BASE + MPFS_IOSCB_DDR_PLL_SSCG_REG_2_OFFSET)
#define MPFS_IOSCB_DDR_PLL_SSCG_REG_3  (MPFS_IOSCB_DDR_PLL_BASE + MPFS_IOSCB_DDR_PLL_SSCG_REG_3_OFFSET)

#define MPFS_GEM0_NETWORK_CONFIG       (MPFS_GEM0_LO_BASE + MPFS_GEM0_NETWORK_CONFIG_OFFSET)
#define MPFS_GEM1_NETWORK_CONFIG       (MPFS_GEM1_LO_BASE + MPFS_GEM0_NETWORK_CONFIG_OFFSET)

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_SGMII_H */
