/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_fpga_canfd.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_FPGA_CANFD_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_FPGA_CANFD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define MPFS_CANFD_DEVICE_ID_OFFSET                             (0x00)
#define MPFS_CANFD_VERSION_OFFSET                               (0x02)

#define MPFS_CANFD_MODE_OFFSET                                  (0x04)
#define MPFS_CANFD_SETTINGS_OFFSET                              (0x06)

#define MPFS_CANFD_STATUS_OFFSET                                (0x08)

#define MPFS_CANFD_COMMAND_OFFSET                               (0x0c)

#define MPFS_CANFD_INT_STAT_OFFSET                              (0x10)

#define MPFS_CANFD_INT_ENA_SET_OFFSET                           (0x14)

#define MPFS_CANFD_INT_ENA_CLR_OFFSET                           (0x18)

#define MPFS_CANFD_INT_MASK_SET_OFFSET                          (0x1c)

#define MPFS_CANFD_INT_MASK_CLR_OFFSET                          (0x20)

#define MPFS_CANFD_BTR_OFFSET                                   (0x24)

#define MPFS_CANFD_BTR_FD_OFFSET                                (0x28)

#define MPFS_CANFD_EWL_OFFSET                                   (0x2c)
#define MPFS_CANFD_ERP_OFFSET                                   (0x2d)
#define MPFS_CANFD_FAULT_STATE_OFFSET                           (0x2e)

#define MPFS_CANFD_REC_OFFSET                                   (0x30)
#define MPFS_CANFD_TEC_OFFSET                                   (0x32)

#define MPFS_CANFD_ERR_NORM_OFFSET                              (0x34)
#define MPFS_CANFD_ERR_FD_OFFSET                                (0x36)

#define MPFS_CANFD_CTR_PRES_OFFSET                              (0x38)

#define MPFS_CANFD_FILTER_A_MASK_OFFSET                         (0x3c)

#define MPFS_CANFD_FILTER_A_VAL_OFFSET                          (0x40)

#define MPFS_CANFD_FILTER_B_MASK_OFFSET                         (0x44)

#define MPFS_CANFD_FILTER_B_VAL_OFFSET                          (0x48)

#define MPFS_CANFD_FILTER_C_MASK_OFFSET                         (0x4c)

#define MPFS_CANFD_FILTER_C_VAL_OFFSET                          (0x50)

#define MPFS_CANFD_FILTER_RAN_LOW_OFFSET                        (0x54)

#define MPFS_CANFD_FILTER_RAN_HIGH_OFFSET                       (0x58)

#define MPFS_CANFD_FILTER_CONTROL_OFFSET                        (0x5c)
#define MPFS_CANFD_FILTER_STATUS_OFFSET                         (0x5e)

/*  RX registers */
#define MPFS_CANFD_RX_MEM_INFO_OFFSET                           (0x60)

#define MPFS_CANFD_RX_POINTERS_OFFSET                           (0x64)

#define MPFS_CANFD_RX_STATUS_OFFSET                             (0x68)
#define MPFS_CANFD_RX_SETTINGS_OFFSET                           (0x6a)

#define MPFS_CANFD_RX_DATA_OFFSET                               (0x6c)

/*  TX registers */
#define MPFS_CANFD_TX_STATUS_OFFSET                             (0x70)

#define MPFS_CANFD_TX_COMMAND_OFFSET                            (0x74)
#define MPFS_CANFD_TXTB_INFO_OFFSET                             (0x76)

#define MPFS_CANFD_TX_PRIORITY_OFFSET                           (0x78)

#define MPFS_CANFD_ERR_CAPT_OFFSET                              (0x7c)
#define MPFS_CANFD_RETR_CTR_OFFSET                              (0x7d)
#define MPFS_CANFD_ALC_OFFSET                                   (0x7e)

#define MPFS_CANFD_TRV_DELAY_OFFSET                             (0x80)
#define MPFS_CANFD_SSP_CFG_OFFSET                               (0x82)

#define MPFS_CANFD_RX_FR_CTR_OFFSET                             (0x84)

#define MPFS_CANFD_TX_FR_CTR_OFFSET                             (0x88)

#define MPFS_CANFD_DEBUG_REGISTER_OFFSET                        (0x8c)

#define MPFS_CANFD_YOLO_OFFSET                                  (0x90)

#define MPFS_CANFD_TIMESTAMP_LOW_OFFSET                         (0x94)

#define MPFS_CANFD_TIMESTAMP_HIGH_OFFSET                        (0x98)

# define MPFS_CANFD_CTUCANFD_TXTB1_DATA_1                       (0x100)
# define MPFS_CANFD_CTUCANFD_TXTB1_DATA_2                       (0x104)
# define MPFS_CANFD_CTUCANFD_TXTB1_DATA_20                      (0x14c)

# define MPFS_CANFD_CTUCANFD_TXTB2_DATA_1                       (0x200)
# define MPFS_CANFD_CTUCANFD_TXTB2_DATA_2                       (0x204)
# define MPFS_CANFD_CTUCANFD_TXTB2_DATA_20                      (0x24c)

# define MPFS_CANFD_CTUCANFD_TXTB3_DATA_1                       (0x300)
# define MPFS_CANFD_CTUCANFD_TXTB3_DATA_2                       (0x304)
# define MPFS_CANFD_CTUCANFD_TXTB3_DATA_20                      (0x34c)

# define MPFS_CANFD_CTUCANFD_TXTB4_DATA_1                       (0x400)
# define MPFS_CANFD_CTUCANFD_TXTB4_DATA_2                       (0x404)
# define MPFS_CANFD_CTUCANFD_TXTB4_DATA_20                      (0x44c)

# define MPFS_CANFD_CTUCANFD_TXTB5_DATA_1                       (0x500)
# define MPFS_CANFD_CTUCANFD_TXTB5_DATA_2                       (0x504)
# define MPFS_CANFD_CTUCANFD_TXTB5_DATA_20                      (0x54c)

# define MPFS_CANFD_CTUCANFD_TXTB6_DATA_1                       (0x600)
# define MPFS_CANFD_CTUCANFD_TXTB6_DATA_2                       (0x604)
# define MPFS_CANFD_CTUCANFD_TXTB6_DATA_20                      (0x64c)

# define MPFS_CANFD_CTUCANFD_TXTB7_DATA_1                       (0x700)
# define MPFS_CANFD_CTUCANFD_TXTB7_DATA_2                       (0x704)
# define MPFS_CANFD_CTUCANFD_TXTB7_DATA_20                      (0x74c)

# define MPFS_CANFD_CTUCANFD_TXTB8_DATA_1                       (0x800)
# define MPFS_CANFD_CTUCANFD_TXTB8_DATA_2                       (0x804)
# define MPFS_CANFD_CTUCANFD_TXTB8_DATA_20                      (0x84c)

# define MPFS_CANFD_CTUCANFD_TST_CONTROL                        (0x900)
# define MPFS_CANFD_CTUCANFD_TST_DEST                           (0x904)
# define MPFS_CANFD_CTUCANFD_TST_WDATA                          (0x908)
# define MPFS_CANFD_CTUCANFD_TST_RDATA                          (0x90c)

/* Control_registers memory region ******************************************/

/*  DEVICE ID / VERSION registers */
#define MPFS_CANFD_DEVICE_ID_DEVICE_ID_SHIFT                    (0)
#define MPFS_CANFD_DEVICE_ID_DEVICE_ID                          (0xffff << MPFS_CANFD_DEVICE_ID_DEVICE_ID_SHIFT)
#define MPFS_CANFD_DEVICE_ID_VER_MINOR_SHIFT                    (16)
#define MPFS_CANFD_DEVICE_ID_VER_MINOR                          (0xff << MPFS_CANFD_DEVICE_ID_VER_MINOR_SHIFT)
#define MPFS_CANFD_DEVICE_ID_VER_MAJOR_SHIFT                    (24)
#define MPFS_CANFD_DEVICE_ID_VER_MAJOR                          (0xff << MPFS_CANFD_DEVICE_ID_VER_MAJOR_SHIFT)

/*  MODE / SETTINGS registers */
#define MPFS_CANFD_MODE_RST                                     (1 << 0)
#define MPFS_CANFD_MODE_BMM                                     (1 << 1)
#define MPFS_CANFD_MODE_STM                                     (1 << 2)
#define MPFS_CANFD_MODE_AFM                                     (1 << 3)
#define MPFS_CANFD_MODE_FDE                                     (1 << 4)
#define MPFS_CANFD_MODE_TTTM                                    (1 << 5)
#define MPFS_CANFD_MODE_ROM                                     (1 << 6)
#define MPFS_CANFD_MODE_ACF                                     (1 << 7)
#define MPFS_CANFD_MODE_TSTM                                    (1 << 8)
#define MPFS_CANFD_MODE_RXBAM                                   (1 << 9)
#define MPFS_CANFD_MODE_RTRLE                                   (1 << 16)
#define MPFS_CANFD_MODE_RTRTH_SHIFT                             (17)
#define MPFS_CANFD_MODE_RTRTH                                   (0x0f << MPFS_CANFD_MODE_RTRTH_SHIFT)
#define MPFS_CANFD_MODE_ILBP                                    (1 << 21)
#define MPFS_CANFD_MODE_ENA                                     (1 << 22)
#define MPFS_CANFD_MODE_NISOFD                                  (1 << 23)
#define MPFS_CANFD_MODE_PEX                                     (1 << 24)
#define MPFS_CANFD_MODE_TBFBO                                   (1 << 25)
#define MPFS_CANFD_MODE_FDRF                                    (1 << 26)

/*  STATUS registers */
#define MPFS_CANFD_STATUS_RXNE                                  (1 << 0)
#define MPFS_CANFD_STATUS_DOR                                   (1 << 1)
#define MPFS_CANFD_STATUS_TXNF                                  (1 << 2)
#define MPFS_CANFD_STATUS_EFT                                   (1 << 3)
#define MPFS_CANFD_STATUS_RXS                                   (1 << 4)
#define MPFS_CANFD_STATUS_TXS                                   (1 << 5)
#define MPFS_CANFD_STATUS_EWL                                   (1 << 6)
#define MPFS_CANFD_STATUS_IDLE                                  (1 << 7)
#define MPFS_CANFD_STATUS_PEXS                                  (1 << 8)
#define MPFS_CANFD_STATUS_STCNT                                 (1 << 16)
#define MPFS_CANFD_STATUS_STRGS                                 (1 << 17)

/*  COMMAND registers */
#define MPFS_CANFD_COMMAND_RXRPMV                               (1 << 1)
#define MPFS_CANFD_COMMAND_RRB                                  (1 << 2)
#define MPFS_CANFD_COMMAND_CDO                                  (1 << 3)
#define MPFS_CANFD_COMMAND_ERCRST                               (1 << 4)
#define MPFS_CANFD_COMMAND_RXFCRST                              (1 << 5)
#define MPFS_CANFD_COMMAND_TXFCRST                              (1 << 6)
#define MPFS_CANFD_COMMAND_CPEXS                                (1 << 7)

/*  INT_STAT registers */
#define MPFS_CANFD_INT_STAT_RXI                                 (1 << 0)
#define MPFS_CANFD_INT_STAT_TXI                                 (1 << 1)
#define MPFS_CANFD_INT_STAT_EWLI                                (1 << 2)
#define MPFS_CANFD_INT_STAT_DOI                                 (1 << 3)
#define MPFS_CANFD_INT_STAT_FCSI                                (1 << 4)
#define MPFS_CANFD_INT_STAT_ALI                                 (1 << 5)
#define MPFS_CANFD_INT_STAT_BEI                                 (1 << 6)
#define MPFS_CANFD_INT_STAT_OFI                                 (1 << 7)
#define MPFS_CANFD_INT_STAT_RXFI                                (1 << 8)
#define MPFS_CANFD_INT_STAT_BSI                                 (1 << 9)
#define MPFS_CANFD_INT_STAT_RBNEI                               (1 << 10)
#define MPFS_CANFD_INT_STAT_TXBHCI                              (1 << 11)

/*  INT_ENA_SET registers */
#define MPFS_CANFD_INT_ENA_SET_INT_ENA_SET                      (0x0fff << 0)

/*  INT_ENA_CLR registers */
#define MPFS_CANFD_INT_ENA_CLR_INT_ENA_CLR                      (0x0fff << 0)

/*  INT_MASK_SET registers */
#define MPFS_CANFD_INT_MASK_SET_INT_MASK_SET                    (0x0fff << 0)

/*  INT_MASK_CLR registers */
#define MPFS_CANFD_INT_MASK_CLR_INT_MASK_CLR                    (0x0fff << 0)

/*  BTR registers */
#define MPFS_CANFD_BTR_PROP_SHIFT                               (0)
#define MPFS_CANFD_BTR_PROP                                     (0x7f << MPFS_CANFD_BTR_PROP_SHIFT)
#define MPFS_CANFD_BTR_PH1_SHIFT                                (7)
#define MPFS_CANFD_BTR_PH1                                      (0x3f << MPFS_CANFD_BTR_PH1_SHIFT)
#define MPFS_CANFD_BTR_PH2_SHIFT                                (13)
#define MPFS_CANFD_BTR_PH2                                      (0x3f << MPFS_CANFD_BTR_PH2_SHIFT)
#define MPFS_CANFD_BTR_BRP_SHIFT                                (19)
#define MPFS_CANFD_BTR_BRP                                      (0xff << MPFS_CANFD_BTR_BRP_SHIFT)
#define MPFS_CANFD_BTR_SJW_SHIFT                                (27)
#define MPFS_CANFD_BTR_SJW                                      (0x1f << MPFS_CANFD_BTR_SJW_SHIFT)

/*  BTR_FD registers */
#define MPFS_CANFD_BTR_FD_PROP_FD_SHIFT                         (0)
#define MPFS_CANFD_BTR_FD_PROP_FD                               (0x3f << MPFS_CANFD_BTR_FD_PROP_FD_SHIFT)
#define MPFS_CANFD_BTR_FD_PH1_FD_SHIFT                          (7)
#define MPFS_CANFD_BTR_FD_PH1_FD                                (0x1f << MPFS_CANFD_BTR_FD_PH1_FD_SHIFT)
#define MPFS_CANFD_BTR_FD_PH2_FD_SHIFT                          (13)
#define MPFS_CANFD_BTR_FD_PH2_FD                                (0x1f << MPFS_CANFD_BTR_FD_PH2_FD_SHIFT)
#define MPFS_CANFD_BTR_FD_BRP_FD_SHIFT                          (19)
#define MPFS_CANFD_BTR_FD_BRP_FD                                (0xff << MPFS_CANFD_BTR_FD_BRP_FD_SHIFT)
#define MPFS_CANFD_BTR_FD_SJW_FD_SHIFT                          (27)
#define MPFS_CANFD_BTR_FD_SJW_FD                                (0x1f << MPFS_CANFD_BTR_FD_SJW_FD_SHIFT)

/*  EWL / ERP / FAULT_STATE registers */
#define MPFS_CANFD_EWL_EW_LIMIT_SHIFT                           (0)
#define MPFS_CANFD_EWL_EW_LIMIT                                 (0xff << MPFS_CANFD_EWL_EW_LIMIT_SHIFT)
#define MPFS_CANFD_EWL_ERP_LIMIT_SHIFT                          (8)
#define MPFS_CANFD_EWL_ERP_LIMIT                                (0xff << MPFS_CANFD_EWL_ERP_LIMIT_SHIFT)
#define MPFS_CANFD_EWL_ERA                                      (1 << 16)
#define MPFS_CANFD_EWL_ERP                                      (1 << 17)
#define MPFS_CANFD_EWL_BOF                                      (1 << 18)

/*  REC / TEC registers */
#define MPFS_CANFD_REC_REC_VAL_SHIFT                            (0)
#define MPFS_CANFD_REC_REC_VAL                                  (0x01ff << MPFS_CANFD_REC_REC_VAL_SHIFT)
#define MPFS_CANFD_REC_TEC_VAL_SHIFT                            (16)
#define MPFS_CANFD_REC_TEC_VAL                                  (0x01ff << MPFS_CANFD_REC_TEC_VAL_SHIFT)

/*  ERR_NORM ERR_FD registers */
#define MPFS_CANFD_ERR_NORM_ERR_NORM_VAL_SHIFT                  (0)
#define MPFS_CANFD_ERR_NORM_ERR_NORM_VAL                        (0xffff << MPFS_CANFD_ERR_NORM_ERR_NORM_VAL_SHIFT)
#define MPFS_CANFD_ERR_NORM_ERR_FD_VAL_SHIFT                    (16)
#define MPFS_CANFD_ERR_NORM_ERR_FD_VAL                          (0xffff << MPFS_CANFD_ERR_NORM_ERR_FD_VAL_SHIFT)

/*  CTR_PRES registers */
#define MPFS_CANFD_CTR_PRES_CTPV_SHIFT                          (0)
#define MPFS_CANFD_CTR_PRES_CTPV                                (0x01ff << MPFS_CANFD_CTR_PRES_CTPV_SHIFT)
#define MPFS_CANFD_CTR_PRES_PTX                                 (1 << 9)
#define MPFS_CANFD_CTR_PRES_PRX                                 (1 << 10)
#define MPFS_CANFD_CTR_PRES_ENORM                               (1 << 11)
#define MPFS_CANFD_CTR_PRES_EFD                                 (1 << 12)

/*  FILTER_A_MASK registers */
#define MPFS_CANFD_FILTER_A_MASK_BIT_MASK_A_VAL                 (0x1fffffff << 0)

/*  FILTER_A_VAL registers */
#define MPFS_CANFD_FILTER_A_VAL_BIT_VAL_A_VAL                   (0x1fffffff << 0)

/*  FILTER_B_MASK registers */
#define MPFS_CANFD_FILTER_B_MASK_BIT_MASK_B_VAL                 (0x1fffffff << 0)

/*  FILTER_B_VAL registers */
#define MPFS_CANFD_FILTER_B_VAL_BIT_VAL_B_VAL                   (0x1fffffff << 0)

/*  FILTER_C_MASK registers */
#define MPFS_CANFD_FILTER_C_MASK_BIT_MASK_C_VAL                 (0x1fffffff << 0)

/*  FILTER_C_VAL registers */
#define MPFS_CANFD_FILTER_C_VAL_BIT_VAL_C_VAL                   (0x1fffffff << 0)

/*  FILTER_RAN_LOW registers */
#define MPFS_CANFD_FILTER_RAN_LOW_BIT_RAN_LOW_VAL               (0x1fffffff << 0)

/*  FILTER_RAN_HIGH registers */
#define MPFS_CANFD_FILTER_RAN_HIGH_BIT_RAN_HIGH_VAL             (0x1fffffff << 0)

/*  FILTER_CONTROL / FILTER_STATUS registers */
#define MPFS_CANFD_FILTER_CONTROL_FANB                          (1 << 0)
#define MPFS_CANFD_FILTER_CONTROL_FANE                          (1 << 1)
#define MPFS_CANFD_FILTER_CONTROL_FAFB                          (1 << 2)
#define MPFS_CANFD_FILTER_CONTROL_FAFE                          (1 << 3)
#define MPFS_CANFD_FILTER_CONTROL_FBNB                          (1 << 4)
#define MPFS_CANFD_FILTER_CONTROL_FBNE                          (1 << 5)
#define MPFS_CANFD_FILTER_CONTROL_FBFB                          (1 << 6)
#define MPFS_CANFD_FILTER_CONTROL_FBFE                          (1 << 7)
#define MPFS_CANFD_FILTER_CONTROL_FCNB                          (1 << 8)
#define MPFS_CANFD_FILTER_CONTROL_FCNE                          (1 << 9)
#define MPFS_CANFD_FILTER_CONTROL_FCFB                          (1 << 10)
#define MPFS_CANFD_FILTER_CONTROL_FCFE                          (1 << 11)
#define MPFS_CANFD_FILTER_CONTROL_FRNB                          (1 << 12)
#define MPFS_CANFD_FILTER_CONTROL_FRNE                          (1 << 13)
#define MPFS_CANFD_FILTER_CONTROL_FRFB                          (1 << 14)
#define MPFS_CANFD_FILTER_CONTROL_FRFE                          (1 << 15)
#define MPFS_CANFD_FILTER_CONTROL_SFA                           (1 << 16)
#define MPFS_CANFD_FILTER_CONTROL_SFB                           (1 << 17)
#define MPFS_CANFD_FILTER_CONTROL_SFC                           (1 << 18)
#define MPFS_CANFD_FILTER_CONTROL_SFR                           (1 << 19)

/*  RX_MEM_INFO registers */
#define MPFS_CANFD_RX_MEM_INFO_RX_BUFF_SIZE_SHIFT               (0)
#define MPFS_CANFD_RX_MEM_INFO_RX_BUFF_SIZE                     (0x1fff << MPFS_CANFD_RX_MEM_INFO_RX_BUFF_SIZE_SHIFT)
#define MPFS_CANFD_RX_MEM_INFO_RX_MEM_FREE_SHIFT                (16)
#define MPFS_CANFD_RX_MEM_INFO_RX_MEM_FREE                      (0x1fff << MPFS_CANFD_RX_MEM_INFO_RX_MEM_FREE_SHIFT)

/*  RX_POINTERS registers */
#define MPFS_CANFD_RX_POINTERS_RX_WPP_SHIFT                     (0)
#define MPFS_CANFD_RX_POINTERS_RX_WPP                           (0x0fff << MPFS_CANFD_RX_POINTERS_RX_WPP_SHIFT)
#define MPFS_CANFD_RX_POINTERS_RX_RPP_SHIFT                     (16)
#define MPFS_CANFD_RX_POINTERS_RX_RPP                           (0x0fff << MPFS_CANFD_RX_POINTERS_RX_RPP_SHIFT)

/*  RX_STATUS / RX_SETTINGS registers */
#define MPFS_CANFD_RX_STATUS_RXE                                (1 << 0)
#define MPFS_CANFD_RX_STATUS_RXF                                (1 << 1)
#define MPFS_CANFD_RX_STATUS_RXMOF                              (1 << 2)
#define MPFS_CANFD_RX_STATUS_RXFRC_SHIFT                        (4)
#define MPFS_CANFD_RX_STATUS_RXFRC                              (0x07ff << MPFS_CANFD_RX_STATUS_RXFRC_SHIFT)
#define MPFS_CANFD_RX_STATUS_RTSOP                              (1 << 16)

/*  RX_DATA registers */
#define MPFS_CANFD_RX_DATA_RX_DATA                              (0xffffffff << 0)

/*  TX_STATUS registers */
#define MPFS_CANFD_TX_STATUS_TX1S_SHIFT                         (0)
#define MPFS_CANFD_TX_STATUS_TX1S                               (0x0f << MPFS_CANFD_TX_STATUS_TX1S_SHIFT)
#define MPFS_CANFD_TX_STATUS_TX2S_SHIFT                         (4)
#define MPFS_CANFD_TX_STATUS_TX2S                               (0x0f << MPFS_CANFD_TX_STATUS_TX2S_SHIFT)
#define MPFS_CANFD_TX_STATUS_TX3S_SHIFT                         (8)
#define MPFS_CANFD_TX_STATUS_TX3S                               (0x0f << MPFS_CANFD_TX_STATUS_TX3S_SHIFT)
#define MPFS_CANFD_TX_STATUS_TX4S_SHIFT                         (12)
#define MPFS_CANFD_TX_STATUS_TX4S                               (0x0f << MPFS_CANFD_TX_STATUS_TX4S_SHIFT)
#define MPFS_CANFD_TX_STATUS_TX5S_SHIFT                         (16)
#define MPFS_CANFD_TX_STATUS_TX5S                               (0x0f << MPFS_CANFD_TX_STATUS_TX5S_SHIFT)
#define MPFS_CANFD_TX_STATUS_TX6S_SHIFT                         (20)
#define MPFS_CANFD_TX_STATUS_TX6S                               (0x0f << MPFS_CANFD_TX_STATUS_TX6S_SHIFT)
#define MPFS_CANFD_TX_STATUS_TX7S_SHIFT                         (24)
#define MPFS_CANFD_TX_STATUS_TX7S                               (0x0f << MPFS_CANFD_TX_STATUS_TX7S_SHIFT)
#define MPFS_CANFD_TX_STATUS_TX8S_SHIFT                         (28)
#define MPFS_CANFD_TX_STATUS_TX8S                               (0x0f << MPFS_CANFD_TX_STATUS_TX8S_SHIFT)

/*  TX_COMMAND TXTB_INFO registers */
#define MPFS_CANFD_TX_COMMAND_TXCE                              (1 << 0)
#define MPFS_CANFD_TX_COMMAND_TXCR                              (1 << 1)
#define MPFS_CANFD_TX_COMMAND_TXCA                              (1 << 2)
#define MPFS_CANFD_TX_COMMAND_TXB1                              (1 << 8)
#define MPFS_CANFD_TX_COMMAND_TXB2                              (1 << 9)
#define MPFS_CANFD_TX_COMMAND_TXB3                              (1 << 10)
#define MPFS_CANFD_TX_COMMAND_TXB4                              (1 << 11)
#define MPFS_CANFD_TX_COMMAND_TXB5                              (1 << 12)
#define MPFS_CANFD_TX_COMMAND_TXB6                              (1 << 13)
#define MPFS_CANFD_TX_COMMAND_TXB7                              (1 << 14)
#define MPFS_CANFD_TX_COMMAND_TXB8                              (1 << 15)
#define MPFS_CANFD_TX_COMMAND_TXT_BUFFER_COUNT_SHIFT            (16)
#define MPFS_CANFD_TX_COMMAND_TXT_BUFFER_COUNT                  (0x0f << MPFS_CANFD_TX_COMMAND_TXT_BUFFER_COUNT_SHIFT)

/*  TX_PRIORITY registers */
#define MPFS_CANFD_TX_PRIORITY_TXT1P_SHIFT                      (0)
#define MPFS_CANFD_TX_PRIORITY_TXT1P                            (0x07 << )MPFS_CANFD_TX_PRIORITY_TXT1P_SHIFT
#define MPFS_CANFD_TX_PRIORITY_TXT2P_SHIFT                      (4)
#define MPFS_CANFD_TX_PRIORITY_TXT2P                            (0x07 << MPFS_CANFD_TX_PRIORITY_TXT2P_SHIFT)
#define MPFS_CANFD_TX_PRIORITY_TXT3P_SHIFT                      (8)
#define MPFS_CANFD_TX_PRIORITY_TXT3P                            (0x07 <<  MPFS_CANFD_TX_PRIORITY_TXT3P_SHIFT)
#define MPFS_CANFD_TX_PRIORITY_TXT4P_SHIFT                      (12)
#define MPFS_CANFD_TX_PRIORITY_TXT4P                            (0x07 <<  MPFS_CANFD_TX_PRIORITY_TXT4P_SHIFT)
#define MPFS_CANFD_TX_PRIORITY_TXT5P_SHIFT                      (16)
#define MPFS_CANFD_TX_PRIORITY_TXT5P                            (0x07 <<  MPFS_CANFD_TX_PRIORITY_TXT5P_SHIFT)
#define MPFS_CANFD_TX_PRIORITY_TXT6P_SHIFT                      (20)
#define MPFS_CANFD_TX_PRIORITY_TXT6P                            (0x07 <<  MPFS_CANFD_TX_PRIORITY_TXT6P_SHIFT)
#define MPFS_CANFD_TX_PRIORITY_TXT7P_SHIFT                      (24)
#define MPFS_CANFD_TX_PRIORITY_TXT7P                            (0x07 <<  MPFS_CANFD_TX_PRIORITY_TXT7P_SHIFT)
#define MPFS_CANFD_TX_PRIORITY_TXT8P_SHIFT                      (28)
#define MPFS_CANFD_TX_PRIORITY_TXT8P                            (0x07 <<  MPFS_CANFD_TX_PRIORITY_TXT8P_SHIFT)

/*  ERR_CAPT RETR_CTR ALC registers */
#define MPFS_CANFD_ERR_CAPT_ERR_POS_SHIFT                       (0)
#define MPFS_CANFD_ERR_CAPT_ERR_POS                             (0x1f << MPFS_CANFD_ERR_CAPT_ERR_POS_SHIFT)
#define MPFS_CANFD_ERR_CAPT_ERR_TYPE_SHIFT                      (5)
#define MPFS_CANFD_ERR_CAPT_ERR_TYPE                            (0x07 << MPFS_CANFD_ERR_CAPT_ERR_TYPE_SHIFT)
#define MPFS_CANFD_ERR_CAPT_RETR_CTR_VAL_SHIFT                  (8)
#define MPFS_CANFD_ERR_CAPT_RETR_CTR_VAL                        (0x0f << MPFS_CANFD_ERR_CAPT_RETR_CTR_VAL_SHIFT)
#define MPFS_CANFD_ERR_CAPT_ALC_BIT_SHIFT                       (16)
#define MPFS_CANFD_ERR_CAPT_ALC_BIT                             (0x1f << MPFS_CANFD_ERR_CAPT_ALC_BIT_SHIFT)
#define MPFS_CANFD_ERR_CAPT_ALC_ID_FIELD_SHIFT                  (21)
#define MPFS_CANFD_ERR_CAPT_ALC_ID_FIELD                        (0x07 << MPFS_CANFD_ERR_CAPT_ALC_ID_FIELD_SHIFT)

/*  TRV_DELAY SSP_CFG registers */
#define MPFS_CANFD_TRV_DELAY_TRV_DELAY_VALUE_SHIFT              (0)
#define MPFS_CANFD_TRV_DELAY_TRV_DELAY_VALUE                    (0x7f << MPFS_CANFD_TRV_DELAY_TRV_DELAY_VALUE_SHIFT)
#define MPFS_CANFD_TRV_DELAY_SSP_OFFSET_SHIFT                   (16)
#define MPFS_CANFD_TRV_DELAY_SSP_OFFSET                         (0xff << MPFS_CANFD_TRV_DELAY_SSP_OFFSET_SHIFT)
#define MPFS_CANFD_TRV_DELAY_SSP_SRC_SHIFT                      (24)
#define MPFS_CANFD_TRV_DELAY_SSP_SRC                            (0x03 << MPFS_CANFD_TRV_DELAY_SSP_SRC_SHIFT)

/*  RX_FR_CTR registers */
#define MPFS_CANFD_RX_FR_CTR_RX_FR_CTR_VAL                      (0xffffffff << 0)

/*  TX_FR_CTR registers */
#define MPFS_CANFD_TX_FR_CTR_TX_FR_CTR_VAL                      (0xffffffff << 0)

/*  DEBUG_REGISTER registers */
#define MPFS_CANFD_DEBUG_REGISTER_STUFF_COUNT_SHIFT             (0)
#define MPFS_CANFD_DEBUG_REGISTER_STUFF_COUNT                   (0x07 << MPFS_CANFD_DEBUG_REGISTER_STUFF_COUNT_SHIFT)
#define MPFS_CANFD_DEBUG_REGISTER_DESTUFF_COUNT_SHIFT           (3)
#define MPFS_CANFD_DEBUG_REGISTER_DESTUFF_COUNT                 (0x07 << MPFS_CANFD_DEBUG_REGISTER_DESTUFF_COUNT_SHIFT)
#define MPFS_CANFD_DEBUG_REGISTER_PC_ARB                        (1 << 6)
#define MPFS_CANFD_DEBUG_REGISTER_PC_CON                        (1 << 7)
#define MPFS_CANFD_DEBUG_REGISTER_PC_DAT                        (1 << 8)
#define MPFS_CANFD_DEBUG_REGISTER_PC_STC                        (1 << 9)
#define MPFS_CANFD_DEBUG_REGISTER_PC_CRC                        (1 << 10)
#define MPFS_CANFD_DEBUG_REGISTER_PC_CRCD                       (1 << 11)
#define MPFS_CANFD_DEBUG_REGISTER_PC_ACK                        (1 << 12)
#define MPFS_CANFD_DEBUG_REGISTER_PC_ACKD                       (1 << 13)
#define MPFS_CANFD_DEBUG_REGISTER_PC_EOF                        (1 << 14)
#define MPFS_CANFD_DEBUG_REGISTER_PC_INT                        (1 << 15)
#define MPFS_CANFD_DEBUG_REGISTER_PC_SUSP                       (1 << 16)
#define MPFS_CANFD_DEBUG_REGISTER_PC_OVR                        (1 << 17)
#define MPFS_CANFD_DEBUG_REGISTER_PC_SOF                        (1 << 18)

/*  YOLO_REG registers */
#define MPFS_CANFD_YOLO_REG_YOLO_VAL                            (0xffffffff << 0)

/*  TIMESTAMP_LOW registers */
#define MPFS_CANFD_TIMESTAMP_LOW_TIMESTAMP_LOW                  (0xffffffff << 0)

/*  TIMESTAMP_HIGH registers */
#define MPFS_CANFD_TIMESTAMP_HIGH_TIMESTAMP_HIGH                (0xffffffff << 0)

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_FPGA_CANFD_H */