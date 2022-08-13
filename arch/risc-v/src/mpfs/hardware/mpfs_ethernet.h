/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_ethernet.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_ETHERNET_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/mpfs_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NETWORK_CONTROL                     0x0000
#define NETWORK_CONFIG                      0x0004
#define NETWORK_STATUS                      0x0008
#define DMA_CONFIG                          0x0010
#define TRANSMIT_STATUS                     0x0014
#define RECEIVE_Q_PTR                       0x0018
#define TRANSMIT_Q_PTR                      0x001C
#define RECEIVE_STATUS                      0x0020
#define INT_STATUS                          0x0024
#define INT_ENABLE                          0x0028
#define INT_DISABLE                         0x002C
#define INT_MASK                            0x0030
#define PHY_MANAGEMENT                      0x0034
#define PAUSE_TIME                          0x0038
#define TX_PAUSE_QUANTUM                    0x003C
#define PBUF_TXCUTTHRU                      0x0040
#define PBUF_RXCUTTHRU                      0x0044
#define JUMBO_MAX_LENGTH                    0x0048
#define AXI_MAX_PIPELINE                    0x0054
#define RSC_CONTROL                         0x0058
#define INT_MODERATION                      0x005C
#define SYS_WAKE_TIME                       0x0060
#define LOCKUP_CONFIG                       0x0068
#define MAC_LOCKUP_TIME                     0x006C
#define LOCKUP_CONFIG3                      0x0070
#define RX_WATER_MARK                       0x007C
#define HASH_BOTTOM                         0x0080
#define HASH_TOP                            0x0084
#define SPEC_ADD1_BOTTOM                    0x0088
#define SPEC_ADD1_TOP                       0x008C
#define SPEC_ADD2_BOTTOM                    0x0090
#define SPEC_ADD2_TOP                       0x0094
#define SPEC_ADD3_BOTTOM                    0x0098
#define SPEC_ADD3_TOP                       0x009C
#define SPEC_ADD4_BOTTOM                    0x00A0
#define SPEC_ADD4_TOP                       0x00A4
#define SPEC_TYPE1                          0x00A8
#define SPEC_TYPE2                          0x00AC
#define SPEC_TYPE3                          0x00B0
#define SPEC_TYPE4                          0x00B4
#define WOL_REGISTER                        0x00B8
#define STRETCH_RATIO                       0x00BC
#define STACKED_VLAN                        0x00C0
#define TX_PFC_PAUSE                        0x00C4
#define MASK_ADD1_BOTTOM                    0x00C8
#define MASK_ADD1_TOP                       0x00CC
#define DMA_ADDR_OR_MASK                    0x00D0
#define RX_PTP_UNICAST                      0x00D4
#define TX_PTP_UNICAST                      0x00D8
#define TSU_NSEC_CMP                        0x00DC
#define TSU_SEC_CMP                         0x00E0
#define TSU_MSB_SEC_CMP                     0x00E4
#define TSU_PTP_TX_MSB_SEC                  0x00E8
#define TSU_PTP_RX_MSB_SEC                  0x00EC
#define TSU_PEER_TX_MSB_SEC                 0x00F0
#define TSU_PEER_RX_MSB_SEC                 0x00F4
#define DPRAM_FILL_DBG                      0x00F8
#define REVISION_REG                        0x00FC
#define OCTETS_TXED_BOTTOM                  0x0100
#define OCTETS_TXED_TOP                     0x0104
#define FRAMES_TXED_OK                      0x0108
#define BROADCAST_TXED                      0x010C
#define MULTICAST_TXED                      0x0110
#define PAUSE_FRAMES_TXED                   0x0114
#define FRAMES_TXED_64                      0x0118
#define FRAMES_TXED_65                      0x011C
#define FRAMES_TXED_128                     0x0120
#define FRAMES_TXED_256                     0x0124
#define FRAMES_TXED_512                     0x0128
#define FRAMES_TXED_1024                    0x012C
#define FRAMES_TXED_1519                    0x0130
#define TX_UNDERRUNS                        0x0134
#define SINGLE_COLLISIONS                   0x0138
#define MULTIPLE_COLLISIONS                 0x013C
#define EXCESSIVE_COLLISIONS                0x0140
#define LATE_COLLISIONS                     0x0144
#define DEFERRED_FRAMES                     0x0148
#define CRS_ERRORS                          0x014C
#define OCTETS_RXED_BOTTOM                  0x0150
#define OCTETS_RXED_TOP                     0x0154
#define FRAMES_RXED_OK                      0x0158
#define BROADCAST_RXED                      0x015C
#define MULTICAST_RXED                      0x0160
#define PAUSE_FRAMES_RXED                   0x0164
#define FRAMES_RXED_64                      0x0168
#define FRAMES_RXED_65                      0x016C
#define FRAMES_RXED_128                     0x0170
#define FRAMES_RXED_256                     0x0174
#define FRAMES_RXED_512                     0x0178
#define FRAMES_RXED_1024                    0x017C
#define FRAMES_RXED_1519                    0x0180
#define UNDERSIZE_FRAMES                    0x0184
#define EXCESSIVE_RX_LENGTH                 0x0188
#define RX_JABBERS                          0x018C
#define FCS_ERRORS                          0x0190
#define RX_LENGTH_ERRORS                    0x0194
#define RX_SYMBOL_ERRORS                    0x0198
#define ALIGNMENT_ERRORS                    0x019C
#define RX_RESOURCE_ERRORS                  0x01A0
#define RX_OVERRUNS                         0x01A4
#define RX_IP_CK_ERRORS                     0x01A8
#define RX_TCP_CK_ERRORS                    0x01AC
#define RX_UDP_CK_ERRORS                    0x01B0
#define AUTO_FLUSHED_PKTS                   0x01B4
#define TSU_TIMER_INCR_SUB_NSEC             0x01BC
#define TSU_TIMER_MSB_SEC                   0x01C0
#define TSU_STROBE_MSB_SEC                  0x01C4
#define TSU_STROBE_SEC                      0x01C8
#define TSU_STROBE_NSEC                     0x01CC
#define TSU_TIMER_SEC                       0x01D0
#define TSU_TIMER_NSEC                      0x01D4
#define TSU_TIMER_ADJUST                    0x01D8
#define TSU_TIMER_INCR                      0x01DC
#define TSU_PTP_TX_SEC                      0x01E0
#define TSU_PTP_TX_NSEC                     0x01E4
#define TSU_PTP_RX_SEC                      0x01E8
#define TSU_PTP_RX_NSEC                     0x01EC
#define TSU_PEER_TX_SEC                     0x01F0
#define TSU_PEER_TX_NSEC                    0x01F4
#define TSU_PEER_RX_SEC                     0x01F8
#define TSU_PEER_RX_NSEC                    0x01FC
#define PCS_CONTROL                         0x0200
#define PFC_STATUS                          0x026C
#define RX_LPI                              0x0270
#define RX_LPI_TIME                         0x0274
#define TX_LPI                              0x0278
#define TX_LPI_TIME                         0x027C
#define DESIGNCFG_DEBUG1                    0x0280
#define DESIGNCFG_DEBUG2                    0x0284
#define DESIGNCFG_DEBUG3                    0x0288
#define DESIGNCFG_DEBUG4                    0x028C
#define DESIGNCFG_DEBUG5                    0x0290
#define DESIGNCFG_DEBUG6                    0x0294
#define DESIGNCFG_DEBUG7                    0x0298
#define DESIGNCFG_DEBUG8                    0x029C
#define DESIGNCFG_DEBUG9                    0x02A0
#define DESIGNCFG_DEBUG10                   0x02A4
#define DESIGNCFG_DEBUG11                   0x02A8
#define DESIGNCFG_DEBUG12                   0x02AC
#define AXI_QOS_CFG_0                       0x02E0
#define AXI_QOS_CFG_1                       0x02E4
#define AXI_QOS_CFG_2                       0x02E8
#define AXI_QOS_CFG_3                       0x02EC
#define INT_Q1_STATUS                       0x0400
#define INT_Q2_STATUS                       0x0404
#define INT_Q3_STATUS                       0x0408
#define INT_Q4_STATUS                       0x040C
#define INT_Q5_STATUS                       0x0410
#define INT_Q6_STATUS                       0x0414
#define INT_Q7_STATUS                       0x0418
#define INT_Q8_STATUS                       0x041C
#define INT_Q9_STATUS                       0x0420
#define INT_Q10_STATUS                      0x0424
#define INT_Q11_STATUS                      0x0428
#define INT_Q12_STATUS                      0x042C
#define INT_Q13_STATUS                      0x0430
#define INT_Q14_STATUS                      0x0434
#define INT_Q15_STATUS                      0x0438
#define TRANSMIT_Q1_PTR                     0x0440
#define TRANSMIT_Q2_PTR                     0x0444
#define TRANSMIT_Q3_PTR                     0x0448
#define TRANSMIT_Q4_PTR                     0x044C
#define TRANSMIT_Q5_PTR                     0x0450
#define TRANSMIT_Q6_PTR                     0x0454
#define TRANSMIT_Q7_PTR                     0x0458
#define TRANSMIT_Q8_PTR                     0x045C
#define TRANSMIT_Q9_PTR                     0x0460
#define TRANSMIT_Q10_PTR                    0x0464
#define TRANSMIT_Q11_PTR                    0x0468
#define TRANSMIT_Q12_PTR                    0x046C
#define TRANSMIT_Q13_PTR                    0x0470
#define TRANSMIT_Q14_PTR                    0x0474
#define TRANSMIT_Q15_PTR                    0x0478
#define RECEIVE_Q1_PTR                      0x0480
#define RECEIVE_Q2_PTR                      0x0484
#define RECEIVE_Q3_PTR                      0x0488
#define RECEIVE_Q4_PTR                      0x048C
#define RECEIVE_Q5_PTR                      0x0490
#define RECEIVE_Q6_PTR                      0x0494
#define RECEIVE_Q7_PTR                      0x0498
#define DMA_RXBUF_SIZE_Q1                   0x04A0
#define DMA_RXBUF_SIZE_Q2                   0x04A4
#define DMA_RXBUF_SIZE_Q3                   0x04A8
#define DMA_RXBUF_SIZE_Q4                   0x04AC
#define DMA_RXBUF_SIZE_Q5                   0x04B0
#define DMA_RXBUF_SIZE_Q6                   0x04B4
#define DMA_RXBUF_SIZE_Q7                   0x04B8
#define CBS_CONTROL                         0x04BC
#define CBS_IDLESLOPE_Q_A                   0x04C0
#define CBS_IDLESLOPE_Q_B                   0x04C4
#define UPPER_TX_Q_BASE_ADDR                0x04C8
#define TX_BD_CONTROL                       0x04CC
#define RX_BD_CONTROL                       0x04D0
#define UPPER_RX_Q_BASE_ADDR                0x04D4
#define WD_COUNTER                          0x04EC
#define AXI_TX_FULL_THRESH0                 0x04F8
#define AXI_TX_FULL_THRESH1                 0x04FC
#define SCREENING_TYPE_1_REGISTER_0         0x0500
#define SCREENING_TYPE_1_REGISTER_1         0x0504
#define SCREENING_TYPE_1_REGISTER_2         0x0508
#define SCREENING_TYPE_1_REGISTER_3         0x050C
#define SCREENING_TYPE_1_REGISTER_4         0x0510
#define SCREENING_TYPE_1_REGISTER_5         0x0514
#define SCREENING_TYPE_1_REGISTER_6         0x0518
#define SCREENING_TYPE_1_REGISTER_7         0x051C
#define SCREENING_TYPE_1_REGISTER_8         0x0520
#define SCREENING_TYPE_1_REGISTER_9         0x0524
#define SCREENING_TYPE_1_REGISTER_10        0x0528
#define SCREENING_TYPE_1_REGISTER_11        0x052C
#define SCREENING_TYPE_1_REGISTER_12        0x0530
#define SCREENING_TYPE_1_REGISTER_13        0x0534
#define SCREENING_TYPE_1_REGISTER_14        0x0538
#define SCREENING_TYPE_1_REGISTER_15        0x053C
#define SCREENING_TYPE_2_REGISTER_0         0x0540
#define SCREENING_TYPE_2_REGISTER_1         0x0544
#define SCREENING_TYPE_2_REGISTER_2         0x0548
#define SCREENING_TYPE_2_REGISTER_3         0x054C
#define SCREENING_TYPE_2_REGISTER_4         0x0550
#define SCREENING_TYPE_2_REGISTER_5         0x0554
#define SCREENING_TYPE_2_REGISTER_6         0x0558
#define SCREENING_TYPE_2_REGISTER_7         0x055C
#define SCREENING_TYPE_2_REGISTER_8         0x0560
#define SCREENING_TYPE_2_REGISTER_9         0x0564
#define SCREENING_TYPE_2_REGISTER_10        0x0568
#define SCREENING_TYPE_2_REGISTER_11        0x056C
#define SCREENING_TYPE_2_REGISTER_12        0x0570
#define SCREENING_TYPE_2_REGISTER_13        0x0574
#define SCREENING_TYPE_2_REGISTER_14        0x0578
#define SCREENING_TYPE_2_REGISTER_15        0x057C
#define TX_SCHED_CTRL                       0x0580
#define BW_RATE_LIMIT_Q0TO3                 0x0590
#define BW_RATE_LIMIT_Q4TO7                 0x0594
#define BW_RATE_LIMIT_Q8TO11                0x0598
#define BW_RATE_LIMIT_Q12TO15               0x059C
#define TX_Q_SEG_ALLOC_Q_LOWER              0x05A0
#define TX_Q_SEG_ALLOC_Q_UPPER              0x05A4
#define RECEIVE_Q8_PTR                      0x05C0
#define RECEIVE_Q9_PTR                      0x05C4
#define RECEIVE_Q10_PTR                     0x05C8
#define RECEIVE_Q11_PTR                     0x05CC
#define RECEIVE_Q12_PTR                     0x05D0
#define RECEIVE_Q13_PTR                     0x05D4
#define RECEIVE_Q14_PTR                     0x05D8
#define RECEIVE_Q15_PTR                     0x05DC
#define DMA_RXBUF_SIZE_Q8                   0x05E0
#define DMA_RXBUF_SIZE_Q9                   0x05E4
#define DMA_RXBUF_SIZE_Q10                  0x05E8
#define DMA_RXBUF_SIZE_Q11                  0x05EC
#define DMA_RXBUF_SIZE_Q12                  0x05F0
#define DMA_RXBUF_SIZE_Q13                  0x05F4
#define DMA_RXBUF_SIZE_Q14                  0x05F8
#define DMA_RXBUF_SIZE_Q15                  0x05FC
#define INT_Q1_ENABLE                       0x0600
#define INT_Q2_ENABLE                       0x0604
#define INT_Q3_ENABLE                       0x0608
#define INT_Q4_ENABLE                       0x060C
#define INT_Q5_ENABLE                       0x0610
#define INT_Q6_ENABLE                       0x0614
#define INT_Q7_ENABLE                       0x0618
#define INT_Q1_DISABLE                      0x0620
#define INT_Q2_DISABLE                      0x0624
#define INT_Q3_DISABLE                      0x0628
#define INT_Q4_DISABLE                      0x062C
#define INT_Q5_DISABLE                      0x0630
#define INT_Q6_DISABLE                      0x0634
#define INT_Q7_DISABLE                      0x0638
#define INT_Q1_MASK                         0x0640
#define INT_Q2_MASK                         0x0644
#define INT_Q3_MASK                         0x0648
#define INT_Q4_MASK                         0x064C
#define INT_Q5_MASK                         0x0650
#define INT_Q6_MASK                         0x0654
#define INT_Q7_MASK                         0x0658
#define INT_Q8_ENABLE                       0x0660
#define INT_Q9_ENABLE                       0x0664
#define INT_Q10_ENABLE                      0x0668
#define INT_Q11_ENABLE                      0x066C
#define INT_Q12_ENABLE                      0x0670
#define INT_Q13_ENABLE                      0x0674
#define INT_Q14_ENABLE                      0x0678
#define INT_Q15_ENABLE                      0x067C
#define INT_Q8_DISABLE                      0x0680
#define INT_Q9_DISABLE                      0x0684
#define INT_Q10_DISABLE                     0x0688
#define INT_Q11_DISABLE                     0x068C
#define INT_Q12_DISABLE                     0x0690
#define INT_Q13_DISABLE                     0x0694
#define INT_Q14_DISABLE                     0x0698
#define INT_Q15_DISABLE                     0x069C
#define INT_Q8_MASK                         0x06A0
#define INT_Q9_MASK                         0x06A4
#define INT_Q10_MASK                        0x06A8
#define INT_Q11_MASK                        0x06AC
#define INT_Q12_MASK                        0x06B0
#define INT_Q13_MASK                        0x06B4
#define INT_Q14_MASK                        0x06B8
#define INT_Q15_MASK                        0x06BC
#define SCREENING_TYPE_2_ETHERTYPE_REG_0    0x06E0
#define SCREENING_TYPE_2_ETHERTYPE_REG_1    0x06E4
#define SCREENING_TYPE_2_ETHERTYPE_REG_2    0x06E8
#define SCREENING_TYPE_2_ETHERTYPE_REG_3    0x06EC
#define SCREENING_TYPE_2_ETHERTYPE_REG_4    0x06F0
#define SCREENING_TYPE_2_ETHERTYPE_REG_5    0x06F4
#define SCREENING_TYPE_2_ETHERTYPE_REG_6    0x06F8
#define SCREENING_TYPE_2_ETHERTYPE_REG_7    0x06FC
#define TYPE2_COMPARE_0_WORD_0              0x0700
#define TYPE2_COMPARE_0_WORD_1              0x0704
#define TYPE2_COMPARE_1_WORD_0              0x0708
#define TYPE2_COMPARE_1_WORD_1              0x070C
#define TYPE2_COMPARE_2_WORD_0              0x0710
#define TYPE2_COMPARE_2_WORD_1              0x0714
#define TYPE2_COMPARE_3_WORD_0              0x0718
#define TYPE2_COMPARE_3_WORD_1              0x071C
#define TYPE2_COMPARE_4_WORD_0              0x0720
#define TYPE2_COMPARE_4_WORD_1              0x0724
#define TYPE2_COMPARE_5_WORD_0              0x0728
#define TYPE2_COMPARE_5_WORD_1              0x072C
#define TYPE2_COMPARE_6_WORD_0              0x0730
#define TYPE2_COMPARE_6_WORD_1              0x0734
#define TYPE2_COMPARE_7_WORD_0              0x0738
#define TYPE2_COMPARE_7_WORD_1              0x073C
#define TYPE2_COMPARE_8_WORD_0              0x0740
#define TYPE2_COMPARE_8_WORD_1              0x0744
#define TYPE2_COMPARE_9_WORD_0              0x0748
#define TYPE2_COMPARE_9_WORD_1              0x074C
#define TYPE2_COMPARE_10_WORD_0             0x0750
#define TYPE2_COMPARE_10_WORD_1             0x0754
#define TYPE2_COMPARE_11_WORD_0             0x0758
#define TYPE2_COMPARE_11_WORD_1             0x075C
#define TYPE2_COMPARE_12_WORD_0             0x0760
#define TYPE2_COMPARE_12_WORD_1             0x0764
#define TYPE2_COMPARE_13_WORD_0             0x0768
#define TYPE2_COMPARE_13_WORD_1             0x076C
#define TYPE2_COMPARE_14_WORD_0             0x0770
#define TYPE2_COMPARE_14_WORD_1             0x0774
#define TYPE2_COMPARE_15_WORD_0             0x0778
#define TYPE2_COMPARE_15_WORD_1             0x077C
#define TYPE2_COMPARE_16_WORD_0             0x0780
#define TYPE2_COMPARE_16_WORD_1             0x0784
#define TYPE2_COMPARE_17_WORD_0             0x0788
#define TYPE2_COMPARE_17_WORD_1             0x078C
#define TYPE2_COMPARE_18_WORD_0             0x0790
#define TYPE2_COMPARE_18_WORD_1             0x0794
#define TYPE2_COMPARE_19_WORD_0             0x0798
#define TYPE2_COMPARE_19_WORD_1             0x079C
#define TYPE2_COMPARE_20_WORD_0             0x07A0
#define TYPE2_COMPARE_20_WORD_1             0x07A4
#define TYPE2_COMPARE_21_WORD_0             0x07A8
#define TYPE2_COMPARE_21_WORD_1             0x07AC
#define TYPE2_COMPARE_22_WORD_0             0x07B0
#define TYPE2_COMPARE_22_WORD_1             0x07B4
#define TYPE2_COMPARE_23_WORD_0             0x07B8
#define TYPE2_COMPARE_23_WORD_1             0x07BC
#define TYPE2_COMPARE_24_WORD_0             0x07C0
#define TYPE2_COMPARE_24_WORD_1             0x07C4
#define TYPE2_COMPARE_25_WORD_0             0x07C8
#define TYPE2_COMPARE_25_WORD_1             0x07CC
#define TYPE2_COMPARE_26_WORD_0             0x07D0
#define TYPE2_COMPARE_26_WORD_1             0x07D4
#define TYPE2_COMPARE_27_WORD_0             0x07D8
#define TYPE2_COMPARE_27_WORD_1             0x07DC
#define TYPE2_COMPARE_28_WORD_0             0x07E0
#define TYPE2_COMPARE_28_WORD_1             0x07E4
#define TYPE2_COMPARE_29_WORD_0             0x07E8
#define TYPE2_COMPARE_29_WORD_1             0x07EC
#define TYPE2_COMPARE_30_WORD_0             0x07F0
#define TYPE2_COMPARE_30_WORD_1             0x07F4
#define TYPE2_COMPARE_31_WORD_0             0x07F8
#define TYPE2_COMPARE_31_WORD_1             0x07FC
#define ENST_START_TIME_Q8                  0x0800
#define ENST_START_TIME_Q9                  0x0804
#define ENST_START_TIME_Q10                 0x0808
#define ENST_START_TIME_Q11                 0x080C
#define ENST_START_TIME_Q12                 0x0810
#define ENST_START_TIME_Q13                 0x0814
#define ENST_START_TIME_Q14                 0x0818
#define ENST_START_TIME_Q15                 0x081C
#define ENST_ON_TIME_Q8                     0x0820
#define ENST_ON_TIME_Q9                     0x0824
#define ENST_ON_TIME_Q10                    0x0828
#define ENST_ON_TIME_Q11                    0x082C
#define ENST_ON_TIME_Q12                    0x0830
#define ENST_ON_TIME_Q13                    0x0834
#define ENST_ON_TIME_Q14                    0x0838
#define ENST_ON_TIME_Q15                    0x083C
#define ENST_OFF_TIME_Q8                    0x0840
#define ENST_OFF_TIME_Q9                    0x0844
#define ENST_OFF_TIME_Q10                   0x0848
#define ENST_OFF_TIME_Q11                   0x084C
#define ENST_OFF_TIME_Q12                   0x0850
#define ENST_OFF_TIME_Q13                   0x0854
#define ENST_OFF_TIME_Q14                   0x0858
#define ENST_OFF_TIME_Q15                   0x085C
#define ENST_CONTROL                        0x0880
#define RX_Q0_FLUSH                         0x0B00
#define RX_Q1_FLUSH                         0x0B04
#define RX_Q2_FLUSH                         0x0B08
#define RX_Q3_FLUSH                         0x0B0C
#define RX_Q4_FLUSH                         0x0B10
#define RX_Q5_FLUSH                         0x0B14
#define RX_Q6_FLUSH                         0x0B18
#define RX_Q7_FLUSH                         0x0B1C
#define RX_Q8_FLUSH                         0x0B20
#define RX_Q9_FLUSH                         0x0B24
#define RX_Q10_FLUSH                        0x0B28
#define RX_Q11_FLUSH                        0x0B2C
#define RX_Q12_FLUSH                        0x0B30
#define RX_Q13_FLUSH                        0x0B34
#define RX_Q14_FLUSH                        0x0B38
#define RX_Q15_FLUSH                        0x0B3C
#define SCR2_REG0_RATE_LIMIT                0x0B40
#define SCR2_REG1_RATE_LIMIT                0x0B44
#define SCR2_REG2_RATE_LIMIT                0x0B48
#define SCR2_REG3_RATE_LIMIT                0x0B4C
#define SCR2_REG4_RATE_LIMIT                0x0B50
#define SCR2_REG5_RATE_LIMIT                0x0B54
#define SCR2_REG6_RATE_LIMIT                0x0B58
#define SCR2_REG7_RATE_LIMIT                0x0B5C
#define SCR2_REG8_RATE_LIMIT                0x0B60
#define SCR2_REG9_RATE_LIMIT                0x0B64
#define SCR2_REG10_RATE_LIMIT               0x0B68
#define SCR2_REG11_RATE_LIMIT               0x0B6C
#define SCR2_REG12_RATE_LIMIT               0x0B70
#define SCR2_REG13_RATE_LIMIT               0x0B74
#define SCR2_REG14_RATE_LIMIT               0x0B78
#define SCR2_REG15_RATE_LIMIT               0x0B7C
#define SCR2_RATE_STATUS                    0x0B80
#define ASF_INT_STATUS                      0x0E00
#define ASF_INT_RAW_STATUS                  0x0E04
#define ASF_INT_MASK                        0x0E08
#define ASF_INT_TEST                        0x0E0C
#define ASF_FATAL_NONFATAL_SELECT           0x0E10
#define ASF_TRANS_TO_FAULT_MASK             0x0E34
#define ASF_TRANS_TO_FAULT_STATUS           0x0E38
#define ASF_PROTOCOL_FAULT_MASK             0x0E40
#define ASF_PROTOCOL_FAULT_STATUS           0x0E44

/* Register bit field definitions *******************************************/

/* NETWORK_CONTROL:
 * The network control register contains general MAC control functions
 * for both receiver and transmitter.
 */

#define NETWORK_CONTROL_LOOPBACK                                (1 << 0)
#define NETWORK_CONTROL_LOOPBACK_LOCAL                          (1 << 1)
#define NETWORK_CONTROL_ENABLE_RECEIVE                          (1 << 2)
#define NETWORK_CONTROL_ENABLE_TRANSMIT                         (1 << 3)
#define NETWORK_CONTROL_MAN_PORT_EN                             (1 << 4)
#define NETWORK_CONTROL_CLEAR_ALL_STATS_REGS                    (1 << 5)
#define NETWORK_CONTROL_INC_ALL_STATS_REGS                      (1 << 6)
#define NETWORK_CONTROL_STATS_WRITE_EN                          (1 << 7)
#define NETWORK_CONTROL_BACK_PRESSURE                           (1 << 8)
#define NETWORK_CONTROL_TRANSMIT_START                          (1 << 9)
#define NETWORK_CONTROL_TRANSMIT_HALT                           (1 << 10)
#define NETWORK_CONTROL_TX_PAUSE_FRAME_REQ                      (1 << 11)
#define NETWORK_CONTROL_TX_PAUSE_FRAME_ZERO                     (1 << 12)
#define NETWORK_CONTROL_STORE_RX_TS                             (1 << 15)
#define NETWORK_CONTROL_PFC_ENABLE                              (1 << 16)
#define NETWORK_CONTROL_TRANSMIT_PFC_PRIORITY_BASED_PAUSE_FRAME (1 << 17)
#define NETWORK_CONTROL_FLUSH_RX_PKT_PCLK                       (1 << 18)
#define NETWORK_CONTROL_TX_LPI_EN                               (1 << 19)
#define NETWORK_CONTROL_PTP_UNICAST_ENA                         (1 << 20)
#define NETWORK_CONTROL_ALT_SGMII_MODE                          (1 << 21)
#define NETWORK_CONTROL_STORE_UDP_OFFSET                        (1 << 22)
#define NETWORK_CONTROL_EXT_TSU_PORT_ENABLE                     (1 << 23)
#define NETWORK_CONTROL_ONE_STEP_SYNC_MODE                      (1 << 24)
#define NETWORK_CONTROL_PFC_CTRL                                (1 << 25)
#define NETWORK_CONTROL_EXT_RXQ_SEL_EN                          (1 << 26)
#define NETWORK_CONTROL_OSS_CORRECTION_FIELD                    (1 << 27)
#define NETWORK_CONTROL_SEL_MII_ON_RGMII                        (1 << 28)
#define NETWORK_CONTROL_TWO_PT_FIVE_GIG                         (1 << 29)
#define NETWORK_CONTROL_IFG_EATS_QAV_CREDIT                     (1 << 30)

/* NETWORK_CONFIG:
 * The network configuration register contains functions for
 * setting the mode of operation for the Gigabit Ethernet MAC.
 */

#define NETWORK_CONFIG_SPEED                                    (1 << 0)
#define NETWORK_CONFIG_FULL_DUPLEX                              (1 << 1)
#define NETWORK_CONFIG_DISCARD_NON_VLAN_FRAMES                  (1 << 2)
#define NETWORK_CONFIG_JUMBO_FRAMES                             (1 << 3)
#define NETWORK_CONFIG_COPY_ALL_FRAMES                          (1 << 4)
#define NETWORK_CONFIG_NO_BROADCAST                             (1 << 5)
#define NETWORK_CONFIG_MULTICAST_HASH_ENABLE                    (1 << 6)
#define NETWORK_CONFIG_UNICAST_HASH_ENABLE                      (1 << 7)
#define NETWORK_CONFIG_RECEIVE_1536_BYTE_FRAMES                 (1 << 8)
#define NETWORK_CONFIG_EXTERNAL_ADDRESS_MATCH_ENABLE            (1 << 9)
#define NETWORK_CONFIG_GIGABIT_MODE_ENABLE                      (1 << 10)
#define NETWORK_CONFIG_PCS_SELECT                               (1 << 11)
#define NETWORK_CONFIG_RETRY_TEST                               (1 << 12)
#define NETWORK_CONFIG_PAUSE_ENABLE                             (1 << 13)
#define NETWORK_CONFIG_RECEIVE_BUFFER_OFFSET_MASK               (3)
#define NETWORK_CONFIG_RECEIVE_BUFFER_OFFSET_SHIFT              (14)
#define NETWORK_CONFIG_LENGTH_FIELD_ERROR_FRAME_DISCARD         (1 << 16)
#define NETWORK_CONFIG_FCS_REMOVE                               (1 << 17)
#define NETWORK_CONFIG_MDC_CLOCK_DIVISOR_SHIFT                  (18)
#define NETWORK_CONFIG_MDC_CLOCK_DIVISOR_MASK                   (7)
#define   NETWORK_CONFIG_MDC_CLOCK_DIVISOR_8                    (0 << NETWORK_CONFIG_MDC_CLOCK_DIVISOR_SHIFT)
#define   NETWORK_CONFIG_MDC_CLOCK_DIVISOR_16                   (1 << NETWORK_CONFIG_MDC_CLOCK_DIVISOR_SHIFT)
#define   NETWORK_CONFIG_MDC_CLOCK_DIVISOR_32                   (2 << NETWORK_CONFIG_MDC_CLOCK_DIVISOR_SHIFT)
#define   NETWORK_CONFIG_MDC_CLOCK_DIVISOR_48                   (3 << NETWORK_CONFIG_MDC_CLOCK_DIVISOR_SHIFT)
#define   NETWORK_CONFIG_MDC_CLOCK_DIVISOR_64                   (4 << NETWORK_CONFIG_MDC_CLOCK_DIVISOR_SHIFT)
#define   NETWORK_CONFIG_MDC_CLOCK_DIVISOR_96                   (5 << NETWORK_CONFIG_MDC_CLOCK_DIVISOR_SHIFT)
#define NETWORK_CONFIG_DATA_BUS_WIDTH_SHIFT                     (21)
#define NETWORK_CONFIG_DATA_BUS_WIDTH_MASK                      (3)
#define NETWORK_CONFIG_DISABLE_COPY_OF_PAUSE_FRAMES             (1 << 23)
#define NETWORK_CONFIG_RECEIVE_CHECKSUM_OFFLOAD_ENABLE          (1 << 24)
#define NETWORK_CONFIG_EN_HALF_DUPLEX_RX                        (1 << 25)
#define NETWORK_CONFIG_IGNORE_RX_FCS                            (1 << 26)
#define NETWORK_CONFIG_SGMII_MODE_ENABLE                        (1 << 27)
#define NETWORK_CONFIG_IPG_STRETCH_ENABLE                       (1 << 28)
#define NETWORK_CONFIG_NSP_CHANGE                               (1 << 29)
#define NETWORK_CONFIG_IGNORE_IPG_RX_ER                         (1 << 30)
#define NETWORK_CONFIG_UNI_DIRECTION_ENABLE                     (1 << 31)

/* NETWORK_STATUS:
 * The network status register returns status information with respect
 * to the PHY management MDIO interface, the PCS, priority flow control,
 * LPI and other status.
 */

#define NETWORK_STATUS_PCS_LINK_STATE                           (1 << 0)
#define NETWORK_STATUS_MDIO_IN                                  (1 << 1)
#define NETWORK_STATUS_MAN_DONE                                 (1 << 2)
#define NETWORK_STATUS_MAC_FULL_DUPLEX                          (1 << 3)
#define NETWORK_STATUS_MAC_PAUSE_RX_EN                          (1 << 4)
#define NETWORK_STATUS_MAC_PAUSE_TX_EN                          (1 << 5)
#define NETWORK_STATUS_PFC_NEGOTIATE_PCLK                       (1 << 6)
#define NETWORK_STATUS_LPI_INDICATE_PCLK                        (1 << 7)
#define NETWORK_STATUS_AXI_XACTION_OUTSTANDING                  (1 << 8)
#define NETWORK_STATUS_LINK_FAULT_INDICATION_SHIFT              (1 << 9)
#define NETWORK_STATUS_LINK_FAULT_INDICATION_MASK               (7)

/* DMA_CONFIG:
 * DMA Configuration Register
 */

#define DMA_CONFIG_AMBA_BURST_LENGTH_SHIFT          (0)       /* Bits: 0-4 */
#define DMA_CONFIG_AMBA_BURST_LENGTH_MASK           (0x1f)
#define DMA_CONFIG_HDR_DATA_SPLITTING_EN            (1 << 5)  /* Bit 5 */
#define DMA_CONFIG_ENDIAN_SWAP_MANAGEMENT           (1 << 6)  /* Bit 6 */
#define DMA_CONFIG_ENDIAN_SWAP_PACKET               (1 << 7)  /* Bit 7 */
#define DMA_CONFIG_RX_PBUF_SIZE_SHIFT               (8)       /* Bits: 8-9 */
#define DMA_CONFIG_RX_PBUF_SIZE_MASK                (3)
#define DMA_CONFIG_TX_PBUF_SIZE                     (1 << 10) /* Bit 10 */
#define DMA_CONFIG_TX_PBUF_TCP_EN                   (1 << 11) /* Bit 11 */
#define DMA_CONFIG_INFINITE_LAST_DBUF_SIZE_EN       (1 << 12) /* Bit 12 */
#define DMA_CONFIG_CRC_ERROR_REPORT                 (1 << 13) /* Bit 13 */
#define DMA_CONFIG_RX_BUF_SIZE_MASK                 (0xff)
#define DMA_CONFIG_RX_BUF_SIZE_SHIFT                (16)
#define DMA_CONFIG_FORCE_DISCARD_ON_ERR             (1 << 24) /* Bit 24 */
#define DMA_CONFIG_FORCE_MAX_AMBA_BURST_RX          (1 << 25) /* Bit 25 */
#define DMA_CONFIG_FORCE_MAX_AMBA_BURST_TX          (1 << 26) /* Bit 26 */
                                                              /* Bit 27 reserved */
#define DMA_CONFIG_RX_BD_EXTENDED_MODE_EN           (1 << 28) /* Bit 28 */
#define DMA_CONFIG_TX_BD_EXTENDED_MODE_EN           (1 << 29) /* Bit 29 */
#define DMA_CONFIG_DMA_ADDR_BUS_WIDTH_1             (1 << 30) /* Bit 30 */
                                                              /* Bit 31 reserved */

/* TRANSMIT_STATUS:
 * This register, when read, provides details of the status
 * of the transmit path. Once read, individual bits may be cleared
 * by writing 1 to them. It is not possible to set a bit to 1 by writing
 * to the register.
 */

#define TRANSMIT_STATUS_USED_BIT_READ               (1 << 0)
#define TRANSMIT_STATUS_COLLISION_OCCURED           (1 << 1)
#define TRANSMIT_STATUS_RETRY_LIMIT_EXCEEDED        (1 << 2)
#define TRANSMIT_STATUS_TRANSMIT_GO                 (1 << 3)
#define TRANSMIT_STATUS_AMBA_ERROR                  (1 << 4)
#define TRANSMIT_STATUS_TRANSMIT_COMPLETE           (1 << 5)
#define TRANSMIT_STATUS_UNDERRUN                    (1 << 6)
#define TRANSMIT_STATUS_LATE_COLLISION              (1 << 7)
#define TRANSMIT_STATUS_RESP_NOT_OK                 (1 << 8)
#define TRANSMIT_STATUS_TX_MAC_LOCKUP               (1 << 9)
#define TRANSMIT_STATUS_TX_DMA_LOCKUP               (1 << 10)

/* RECEIVE_STATUS:
 * This register, when read, provides details of the status
 * of the receive path. Once read, individual bits may be cleared
 * by writing 1 to them. It is not possible to set a bit to 1 by writing
 * to the register.
 */
#define RECEIVE_STATUS_BUFFER_NOT_AVAILABLE         (1 << 0)
#define RECEIVE_STATUS_FRAME_RECEIVED               (1 << 1)
#define RECEIVE_STATUS_RECEIVE_OVERRUN              (1 << 2)
#define RECEIVE_STATUS_RESP_NOT_OK                  (1 << 3)
#define RECEIVE_STATUS_RX_MAC_LOCKUP                (1 << 4)
#define RECEIVE_STATUS_RX_DMA_LOCKUP                (1 << 5)

/* PHY_MANAGEMENT:
 */

#define PHY_MANAGEMENT_PHY_DATA_SHIFT               (0)
#define PHY_MANAGEMENT_PHY_DATA_MASK                (0xffff)
#define PHY_MANAGEMENT_PHY_DATA(n)                  ((n & PHY_MANAGEMENT_PHY_DATA_MASK) << PHY_MANAGEMENT_PHY_DATA_SHIFT)
#define PHY_MANAGEMENT_WRITE10                      (2 << PHY_MANAGEMENT_WRITE10_SHIFT)
#define PHY_MANAGEMENT_WRITE10_SHIFT                (16)
#define PHY_MANAGEMENT_WRITE10_MASK                 (3)
#define PHY_MANAGEMENT_REG_ADDRESS_SHIFT            (18)
#define PHY_MANAGEMENT_REG_ADDRESS_MASK             (0x1f)
#define PHY_MANAGEMENT_REG_ADDRESS(n)               ((n & PHY_MANAGEMENT_REG_ADDRESS_MASK ) << PHY_MANAGEMENT_REG_ADDRESS_SHIFT)
#define PHY_MANAGEMENT_PHY_ADDRESS_SHIFT            (23)
#define PHY_MANAGEMENT_PHY_ADDRESS_MASK             (0x1f)
#define PHY_MANAGEMENT_PHY_ADDRESS(n)               ((n & PHY_MANAGEMENT_PHY_ADDRESS_MASK ) << PHY_MANAGEMENT_PHY_ADDRESS_SHIFT)
#define PHY_MANAGEMENT_OPERATION_SHIFT              (28)
#define PHY_MANAGEMENT_OPERATION_MASK               (3)
#  define PHY_MANAGEMENT_OPERATION_READ             (2 << PHY_MANAGEMENT_OPERATION_SHIFT)
#  define PHY_MANAGEMENT_OPERATION_WRITE            (1 << PHY_MANAGEMENT_OPERATION_SHIFT)
#define PHY_MANAGEMENT_WRITE_1                      (1 << 30)
#define PHY_MANAGEMENT_WRITE_0                      (1 << 31)

/* irq status, enable and disable */

#define GEM_INT_MANAGEMENT_DONE                     (1 << 0)
#define GEM_INT_RECEIVE_COMPLETE                    (1 << 1)
#define GEM_INT_RECEIVE_BUFFER_USED_BIT_READ        (1 << 2)
#define GEM_INT_TRANSMIT_BUFFER_USED_BIT_READ       (1 << 3)
#define GEM_INT_TRANSMIT_BUFFER_UNDERRUN            (1 << 4)
#define GEM_INT_RETRY_LIMIT_EXCEEDED                (1 << 5)
#define GEM_INT_AMBA_ERROR                          (1 << 6)
#define GEM_INT_TRANSMIT_COMPLETE                   (1 << 7)
#define GEM_INT_RECEIVE_OVERRUN                     (1 << 10)
#define GEM_INT_RESP_NOT_OK                         (1 << 11)

/* PCS_CONTROL: */

#define PCS_CONTROL_SPEED_SELECT_BIT_0              (1 << 6)
#define PCS_CONTROL_COLLISION_TEST                  (1 << 7)
#define PCS_CONTROL_COLLISION_TEST                  (1 << 7)
#define PCS_CONTROL_MAC_DUPLEX_STATE                (1 << 8)
#define PCS_CONTROL_RESTART_AUTO_NEG                (1 << 9)
#define PCS_CONTROL_RESTART_AUTO_NEG                (1 << 9)
#define PCS_CONTROL_ENABLE_AUTO_NEG                 (1 << 12)
#define PCS_CONTROL_SPEED_SELECT_BIT_1              (1 << 13)
#define PCS_CONTROL_LOOPBACK_MODE                   (1 << 14)
#define PCS_CONTROL_LOOPBACK_MODE                   (1 << 14)
#define PCS_CONTROL_SOFTWARE_RESET                  (1 << 15)

/* Receive buffer descriptor:  Address word */

#define GEM_RX_DMA_ADDR_OWNER              (1 << 0)     /* Bit 0:  1=Software owns; 0=GMAC owns */
#define GEM_RX_DMA_ADDR_WRAP               (1 << 1)     /* Bit 1:  Last descriptor in list */
#define GEM_RX_DMA_ADDR_MASK               (0xfffffffc) /* Bits 2-31: Aligned buffer address */

/* Receive buffer descriptor:  status word */

#define GEM_RX_DMA_STATUS_FRAME_LEN_SHIFT  (0)         /* Bits 0-12: Frame length */
#define GEM_RX_DMA_STATUS_FRAME_LEN_MASK   (0x00000fff)
#define GEM_RX_DMA_STATUS_SOF              (1 << 14)   /* Bit 14: Start of frame */
#define GEM_RX_DMA_STATUS_EOF              (1 << 15)   /* Bit 15: End of frame */

/* Transmit buffer descriptor:  status word */

#define GEM_TX_DMA_LAST                    (1 << 15)  /* Bit 15: the last buffer in a frame */
#define GEM_TX_DMA_NO_CRC                  (1 << 16)  /* Bit 16: don't calc and append crc */
                                                      /* Bits: 17 - 19: reserved */
#define GEM_TX_DMA_OFFLOAD_ERRORS_SHIFT    (20)       /* Bits: 20 - 22 Transmit checksum generation offload errors */
#define GEM_TX_DMA_OFFLOAD_ERRORS_MASK     (7)
#define GEM_TX_DMA_OFFLOAD_ERROR_OK        (0 << GEM_TX_DMA_OFFLOAD_ERRORS_SHIFT)
#define GEM_TX_DMA_OFFLOAD_ERROR_VLAN      (1 << GEM_TX_DMA_OFFLOAD_ERRORS_SHIFT)
#define GEM_TX_DMA_OFFLOAD_ERROR_SNAP      (2 << GEM_TX_DMA_OFFLOAD_ERRORS_SHIFT)
#define GEM_TX_DMA_OFFLOAD_ERROR_IP        (3 << GEM_TX_DMA_OFFLOAD_ERRORS_SHIFT)
#define GEM_TX_DMA_OFFLOAD_ERROR_UNK       (4 << GEM_TX_DMA_OFFLOAD_ERRORS_SHIFT)
#define GEM_TX_DMA_OFFLOAD_ERROR_FRAG      (5 << GEM_TX_DMA_OFFLOAD_ERRORS_SHIFT)
#define GEM_TX_DMA_OFFLOAD_ERROR_PROTO     (6 << GEM_TX_DMA_OFFLOAD_ERRORS_SHIFT)
#define GEM_TX_DMA_OFFLOAD_ERROR_END       (7 << GEM_TX_DMA_OFFLOAD_ERRORS_SHIFT)
#define GEM_TX_DMA_TS_PRESENT              (1 << 23)  /* Bit: 23 This descriptor holds a valid time stamp which indicates the transmit time. */
                                                      /* Bits: 24 - 25 reserved */
#define GEM_TX_DMA_LATE_COL_ERROR          (1 << 26)  /* Bit 26: Late collision transmit error detected. */
#define GEM_TX_DMA_BUS_ERROR               (1 << 27)  /* Bit 27: Transmit frame corruption due to AHB/AXI error. */
#define GEM_TX_DMA_UNDERRUN                (1 << 28)  /* Bit 28: Transmit underrun */
#define GEM_TX_DMA_RETRY_ERROR             (1 << 29)  /* Bit 29: Retry limit exceeded, tx error detected. */
#define GEM_TX_DMA_WRAP                    (1 << 30)  /* Bit 30: This is the last descriptor in the chain and the DMA controller will next access the first descriptor in the chain. */
#define GEM_TX_DMA_USED                    (1 << 31)  /* Bit 31: Set to 0 by the driver to indicate buffer contains data to be transmitted.
                                                       * Set to 1 by the GEM once packet or packet fragment has been sent from the buffer. */

#define GEM_DMA_STATUS_LENGTH_MASK   0x3fffu

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_ETHERNET_H */
