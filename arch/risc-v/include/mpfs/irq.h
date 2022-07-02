/****************************************************************************
 * arch/risc-v/include/mpfs/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_MPFS_IRQ_H
#define __ARCH_RISCV_INCLUDE_MPFS_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

#define MPFS_IRQ_ASYNC RISCV_IRQ_ASYNC

#define MPFS_IRQ_LOCAL_START                (MPFS_IRQ_ASYNC + 16)
#define MPFS_IRQ_LOCAL_0                    (MPFS_IRQ_LOCAL_START + 0)   /* Local 0 spare */
#define MPFS_IRQ_LOCAL_1                    (MPFS_IRQ_LOCAL_START + 1)   /* Local 1 spare */
#define MPFS_IRQ_LOCAL_2                    (MPFS_IRQ_LOCAL_START + 2)   /* Local 2 spare */
#define MPFS_IRQ_LOCAL_U54_MAC_MMSL         (MPFS_IRQ_LOCAL_START + 3)   /* check hartid for mac source */
#define MPFS_IRQ_LOCAL_U54_MAC_EMAC         (MPFS_IRQ_LOCAL_START + 4)
#define MPFS_IRQ_LOCAL_U54_MAC_QUE3         (MPFS_IRQ_LOCAL_START + 5)
#define MPFS_IRQ_LOCAL_U54_MAC_QUE2         (MPFS_IRQ_LOCAL_START + 6)
#define MPFS_IRQ_LOCAL_U54_MAC_QUE1         (MPFS_IRQ_LOCAL_START + 7)
#define MPFS_IRQ_LOCAL_U54_MAC_INT          (MPFS_IRQ_LOCAL_START + 8)
#define MPFS_IRQ_LOCAL_U54_WDOG_TOUT        (MPFS_IRQ_LOCAL_START + 9)   /* check hartid for wdog source */
#define MPFS_IRQ_LOCAL_U54_MVRP             (MPFS_IRQ_LOCAL_START + 10)
#define MPFS_IRQ_LOCAL_E51_MMUART0          (MPFS_IRQ_LOCAL_START + 11)
#define MPFS_IRQ_LOCAL_U54_H1_MMUART1       (MPFS_IRQ_LOCAL_START + 11)
#define MPFS_IRQ_LOCAL_U54_H2_MMUART2       (MPFS_IRQ_LOCAL_START + 11)
#define MPFS_IRQ_LOCAL_U54_H3_MMUART3       (MPFS_IRQ_LOCAL_START + 11)
#define MPFS_IRQ_LOCAL_U54_H4_MMUART4       (MPFS_IRQ_LOCAL_START + 11)
#define MPFS_IRQ_LOCAL_12                   (MPFS_IRQ_LOCAL_START + 12)   /* Local 12 spare */
#define MPFS_IRQ_LOCAL_13                   (MPFS_IRQ_LOCAL_START + 13)   /* Local 13 spare */
#define MPFS_IRQ_LOCAL_14                   (MPFS_IRQ_LOCAL_START + 14)   /* Local 14 spare */
#define MPFS_IRQ_LOCAL_15                   (MPFS_IRQ_LOCAL_START + 15)   /* Local 15 spare */
#define MPFS_IRQ_LOCAL_U54_F2H_0            (MPFS_IRQ_LOCAL_START + 16)   /* Fabric 0 */
#define MPFS_IRQ_LOCAL_U54_F2H_1            (MPFS_IRQ_LOCAL_START + 17)   /* Fabric 1 */
#define MPFS_IRQ_LOCAL_U54_F2H_2            (MPFS_IRQ_LOCAL_START + 18)   /* Fabric 2 */
#define MPFS_IRQ_LOCAL_U54_F2H_3            (MPFS_IRQ_LOCAL_START + 19)   /* Fabric 3 */
#define MPFS_IRQ_LOCAL_U54_F2H_4            (MPFS_IRQ_LOCAL_START + 20)   /* Fabric 4 */
#define MPFS_IRQ_LOCAL_U54_F2H_5            (MPFS_IRQ_LOCAL_START + 21)   /* Fabric 5 */
#define MPFS_IRQ_LOCAL_U54_F2H_6            (MPFS_IRQ_LOCAL_START + 22)   /* Fabric 6 */
#define MPFS_IRQ_LOCAL_U54_F2H_7            (MPFS_IRQ_LOCAL_START + 23)   /* Fabric 7 */
#define MPFS_IRQ_LOCAL_U54_F2H_8            (MPFS_IRQ_LOCAL_START + 24)   /* Fabric 8 */
#define MPFS_IRQ_LOCAL_U54_F2H_9            (MPFS_IRQ_LOCAL_START + 25)   /* Fabric 9 */
#define MPFS_IRQ_LOCAL_U54_F2H_10           (MPFS_IRQ_LOCAL_START + 26)   /* Fabric 10 */
#define MPFS_IRQ_LOCAL_U54_F2H_11           (MPFS_IRQ_LOCAL_START + 27)   /* Fabric 11 */
#define MPFS_IRQ_LOCAL_U54_F2H_12           (MPFS_IRQ_LOCAL_START + 28)   /* Fabric 12 */
#define MPFS_IRQ_LOCAL_U54_F2H_13           (MPFS_IRQ_LOCAL_START + 29)   /* Fabric 13 */
#define MPFS_IRQ_LOCAL_U54_F2H_14           (MPFS_IRQ_LOCAL_START + 30)   /* Fabric 14 */
#define MPFS_IRQ_LOCAL_U54_F2H_15           (MPFS_IRQ_LOCAL_START + 31)   /* Fabric 15 */
#define MPFS_IRQ_LOCAL_U54_F2H_16           (MPFS_IRQ_LOCAL_START + 32)   /* Fabric 16 */
#define MPFS_IRQ_LOCAL_U54_F2H_17           (MPFS_IRQ_LOCAL_START + 33)   /* Fabric 17 */
#define MPFS_IRQ_LOCAL_U54_F2H_18           (MPFS_IRQ_LOCAL_START + 34)   /* Fabric 18 */
#define MPFS_IRQ_LOCAL_U54_F2H_19           (MPFS_IRQ_LOCAL_START + 35)   /* Fabric 19 */
#define MPFS_IRQ_LOCAL_U54_F2H_20           (MPFS_IRQ_LOCAL_START + 36)   /* Fabric 20 */
#define MPFS_IRQ_LOCAL_U54_F2H_21           (MPFS_IRQ_LOCAL_START + 37)   /* Fabric 21 */
#define MPFS_IRQ_LOCAL_U54_F2H_22           (MPFS_IRQ_LOCAL_START + 38)   /* Fabric 22 */
#define MPFS_IRQ_LOCAL_U54_F2H_23           (MPFS_IRQ_LOCAL_START + 39)   /* Fabric 23 */
#define MPFS_IRQ_LOCAL_U54_F2H_24           (MPFS_IRQ_LOCAL_START + 40)   /* Fabric 24 */
#define MPFS_IRQ_LOCAL_U54_F2H_25           (MPFS_IRQ_LOCAL_START + 41)   /* Fabric 25 */
#define MPFS_IRQ_LOCAL_U54_F2H_26           (MPFS_IRQ_LOCAL_START + 42)   /* Fabric 26 */
#define MPFS_IRQ_LOCAL_U54_F2H_27           (MPFS_IRQ_LOCAL_START + 43)   /* Fabric 27 */
#define MPFS_IRQ_LOCAL_U54_F2H_28           (MPFS_IRQ_LOCAL_START + 44)   /* Fabric 28 */
#define MPFS_IRQ_LOCAL_U54_F2H_29           (MPFS_IRQ_LOCAL_START + 45)   /* Fabric 29 */
#define MPFS_IRQ_LOCAL_U54_F2H_30           (MPFS_IRQ_LOCAL_START + 46)   /* Fabric 30 */
#define MPFS_IRQ_LOCAL_U54_F2H_31           (MPFS_IRQ_LOCAL_START + 47)   /* Fabric 31 */

/* External Interrupts. if irq is MPFS_IRQ_MEXT or MPFS_IRQ_SEXT */

#define MPFS_IRQ_EXT_START                  (MPFS_IRQ_ASYNC + 80U)
#define MPFS_IRQ_INVALID                    (MPFS_IRQ_EXT_START + 0)
#define MPFS_IRQ_L2_METADATA_CORR           (MPFS_IRQ_EXT_START + 1)
#define MPFS_IRQ_L2_METADATA_UNCORR         (MPFS_IRQ_EXT_START + 2)
#define MPFS_IRQ_L2_DATA_CORR               (MPFS_IRQ_EXT_START + 3)
#define MPFS_IRQ_L2_DATA_UNCORR             (MPFS_IRQ_EXT_START + 4)
#define MPFS_IRQ_DMA_CH0_DONE               (MPFS_IRQ_EXT_START + 5)
#define MPFS_IRQ_DMA_CH0_ERR                (MPFS_IRQ_EXT_START + 6)
#define MPFS_IRQ_DMA_CH1_DONE               (MPFS_IRQ_EXT_START + 7)
#define MPFS_IRQ_DMA_CH1_ERR                (MPFS_IRQ_EXT_START + 8)
#define MPFS_IRQ_DMA_CH2_DONE               (MPFS_IRQ_EXT_START + 9)
#define MPFS_IRQ_DMA_CH2_ERR                (MPFS_IRQ_EXT_START + 10)
#define MPFS_IRQ_DMA_CH3_DONE               (MPFS_IRQ_EXT_START + 11)
#define MPFS_IRQ_DMA_CH3_ERR                (MPFS_IRQ_EXT_START + 12)

/* Global Interrupts */

#define OFFSET_TO_MSS_GLOBAL_INTS           (13U)
#define MPFS_IRQ_GLOBAL_START               (MPFS_IRQ_EXT_START + OFFSET_TO_MSS_GLOBAL_INTS)
#define MPFS_IRQ_GPIO02_BIT0                (MPFS_IRQ_GLOBAL_START + 0)
#define MPFS_IRQ_GPIO02_BIT1                (MPFS_IRQ_GLOBAL_START + 1)
#define MPFS_IRQ_GPIO02_BIT2                (MPFS_IRQ_GLOBAL_START + 2)
#define MPFS_IRQ_GPIO02_BIT3                (MPFS_IRQ_GLOBAL_START + 3)
#define MPFS_IRQ_GPIO02_BIT4                (MPFS_IRQ_GLOBAL_START + 4)
#define MPFS_IRQ_GPIO02_BIT5                (MPFS_IRQ_GLOBAL_START + 5)
#define MPFS_IRQ_GPIO02_BIT6                (MPFS_IRQ_GLOBAL_START + 6)
#define MPFS_IRQ_GPIO02_BIT7                (MPFS_IRQ_GLOBAL_START + 7)
#define MPFS_IRQ_GPIO02_BIT8                (MPFS_IRQ_GLOBAL_START + 8)
#define MPFS_IRQ_GPIO02_BIT9                (MPFS_IRQ_GLOBAL_START + 9)
#define MPFS_IRQ_GPIO02_BIT10               (MPFS_IRQ_GLOBAL_START + 10)
#define MPFS_IRQ_GPIO02_BIT11               (MPFS_IRQ_GLOBAL_START + 11)
#define MPFS_IRQ_GPIO02_BIT12               (MPFS_IRQ_GLOBAL_START + 12)
#define MPFS_IRQ_GPIO02_BIT13               (MPFS_IRQ_GLOBAL_START + 13)
#define MPFS_IRQ_GPIO1_BIT0_OR_GPIO2_BIT14  (MPFS_IRQ_GLOBAL_START + 14)
#define MPFS_IRQ_GPIO1_BIT1_OR_GPIO2_BIT15  (MPFS_IRQ_GLOBAL_START + 15)
#define MPFS_IRQ_GPIO1_BIT2_OR_GPIO2_BIT16  (MPFS_IRQ_GLOBAL_START + 16)
#define MPFS_IRQ_GPIO1_BIT3_OR_GPIO2_BIT17  (MPFS_IRQ_GLOBAL_START + 17)
#define MPFS_IRQ_GPIO1_BIT4_OR_GPIO2_BIT18  (MPFS_IRQ_GLOBAL_START + 18)
#define MPFS_IRQ_GPIO1_BIT5_OR_GPIO2_BIT19  (MPFS_IRQ_GLOBAL_START + 19)
#define MPFS_IRQ_GPIO1_BIT6_OR_GPIO2_BIT20  (MPFS_IRQ_GLOBAL_START + 20)
#define MPFS_IRQ_GPIO1_BIT7_OR_GPIO2_BIT21  (MPFS_IRQ_GLOBAL_START + 21)
#define MPFS_IRQ_GPIO1_BIT8_OR_GPIO2_BIT22  (MPFS_IRQ_GLOBAL_START + 22)
#define MPFS_IRQ_GPIO1_BIT9_OR_GPIO2_BIT23  (MPFS_IRQ_GLOBAL_START + 23)
#define MPFS_IRQ_GPIO1_BIT10_OR_GPIO2_BIT24 (MPFS_IRQ_GLOBAL_START + 24)
#define MPFS_IRQ_GPIO1_BIT11_OR_GPIO2_BIT25 (MPFS_IRQ_GLOBAL_START + 25)
#define MPFS_IRQ_GPIO1_BIT12_OR_GPIO2_BIT26 (MPFS_IRQ_GLOBAL_START + 26)
#define MPFS_IRQ_GPIO1_BIT13_OR_GPIO2_BIT27 (MPFS_IRQ_GLOBAL_START + 27)
#define MPFS_IRQ_GPIO1_BIT14_OR_GPIO2_BIT28 (MPFS_IRQ_GLOBAL_START + 28)
#define MPFS_IRQ_GPIO1_BIT15_OR_GPIO2_BIT29 (MPFS_IRQ_GLOBAL_START + 29)
#define MPFS_IRQ_GPIO1_BIT16_OR_GPIO2_BIT30 (MPFS_IRQ_GLOBAL_START + 30)
#define MPFS_IRQ_GPIO1_BIT17_OR_GPIO2_BIT31 (MPFS_IRQ_GLOBAL_START + 31)
#define MPFS_IRQ_GPIO1_BIT18                (MPFS_IRQ_GLOBAL_START + 32)
#define MPFS_IRQ_GPIO1_BIT19                (MPFS_IRQ_GLOBAL_START + 33)
#define MPFS_IRQ_GPIO1_BIT20                (MPFS_IRQ_GLOBAL_START + 34)
#define MPFS_IRQ_GPIO1_BIT21                (MPFS_IRQ_GLOBAL_START + 35)
#define MPFS_IRQ_GPIO1_BIT22                (MPFS_IRQ_GLOBAL_START + 36)
#define MPFS_IRQ_GPIO1_BIT23                (MPFS_IRQ_GLOBAL_START + 37)
#define MPFS_IRQ_GPIO0_NON_DIRECT           (MPFS_IRQ_GLOBAL_START + 38)
#define MPFS_IRQ_GPIO1_NON_DIRECT           (MPFS_IRQ_GLOBAL_START + 39)
#define MPFS_IRQ_GPIO2_NON_DIRECT           (MPFS_IRQ_GLOBAL_START + 40)
#define MPFS_IRQ_SPI0                       (MPFS_IRQ_GLOBAL_START + 41)
#define MPFS_IRQ_SPI1                       (MPFS_IRQ_GLOBAL_START + 42)
#define MPFS_IRQ_CAN0                       (MPFS_IRQ_GLOBAL_START + 43)
#define MPFS_IRQ_CAN1                       (MPFS_IRQ_GLOBAL_START + 44)
#define MPFS_IRQ_I2C0_MAIN                  (MPFS_IRQ_GLOBAL_START + 45)
#define MPFS_IRQ_I2C0_ALERT                 (MPFS_IRQ_GLOBAL_START + 46)
#define MPFS_IRQ_I2C0_SUS                   (MPFS_IRQ_GLOBAL_START + 47)
#define MPFS_IRQ_I2C1_MAIN                  (MPFS_IRQ_GLOBAL_START + 48)
#define MPFS_IRQ_I2C1_ALERT                 (MPFS_IRQ_GLOBAL_START + 49)
#define MPFS_IRQ_I2C1_SUS                   (MPFS_IRQ_GLOBAL_START + 50)
#define MPFS_IRQ_MAC0_INT                   (MPFS_IRQ_GLOBAL_START + 51)
#define MPFS_IRQ_MAC0_QUEUE1                (MPFS_IRQ_GLOBAL_START + 52)
#define MPFS_IRQ_MAC0_QUEUE2                (MPFS_IRQ_GLOBAL_START + 53)
#define MPFS_IRQ_MAC0_QUEUE3                (MPFS_IRQ_GLOBAL_START + 54)
#define MPFS_IRQ_MAC0_EMAC                  (MPFS_IRQ_GLOBAL_START + 55)
#define MPFS_IRQ_MAC0_MMSL                  (MPFS_IRQ_GLOBAL_START + 56)
#define MPFS_IRQ_MAC1_INT                   (MPFS_IRQ_GLOBAL_START + 57)
#define MPFS_IRQ_MAC1_QUEUE1                (MPFS_IRQ_GLOBAL_START + 58)
#define MPFS_IRQ_MAC1_QUEUE2                (MPFS_IRQ_GLOBAL_START + 59)
#define MPFS_IRQ_MAC1_QUEUE3                (MPFS_IRQ_GLOBAL_START + 60)
#define MPFS_IRQ_MAC1_EMAC                  (MPFS_IRQ_GLOBAL_START + 61)
#define MPFS_IRQ_MAC1_MMSL                  (MPFS_IRQ_GLOBAL_START + 62)
#define MPFS_IRQ_DDRC_TRAIN                 (MPFS_IRQ_GLOBAL_START + 63)
#define MPFS_IRQ_SCB_INTERRUPT              (MPFS_IRQ_GLOBAL_START + 64)
#define MPFS_IRQ_ECC_ERROR                  (MPFS_IRQ_GLOBAL_START + 65)
#define MPFS_IRQ_ECC_CORRECT                (MPFS_IRQ_GLOBAL_START + 66)
#define MPFS_IRQ_RTC_WAKEUP                 (MPFS_IRQ_GLOBAL_START + 67)
#define MPFS_IRQ_RTC_MATCH                  (MPFS_IRQ_GLOBAL_START + 68)
#define MPFS_IRQ_TIMER1                     (MPFS_IRQ_GLOBAL_START + 69)
#define MPFS_IRQ_TIMER2                     (MPFS_IRQ_GLOBAL_START + 70)
#define MPFS_IRQ_ENVM                       (MPFS_IRQ_GLOBAL_START + 71)
#define MPFS_IRQ_QSPI                       (MPFS_IRQ_GLOBAL_START + 72)
#define MPFS_IRQ_USB_DMA                    (MPFS_IRQ_GLOBAL_START + 73)
#define MPFS_IRQ_USB_MC                     (MPFS_IRQ_GLOBAL_START + 74)
#define MPFS_IRQ_MMC_MAIN                   (MPFS_IRQ_GLOBAL_START + 75)
#define MPFS_IRQ_MMC_WAKEUP                 (MPFS_IRQ_GLOBAL_START + 76)
#define MPFS_IRQ_MMUART0                    (MPFS_IRQ_GLOBAL_START + 77)
#define MPFS_IRQ_MMUART1                    (MPFS_IRQ_GLOBAL_START + 78)
#define MPFS_IRQ_MMUART2                    (MPFS_IRQ_GLOBAL_START + 79)
#define MPFS_IRQ_MMUART3                    (MPFS_IRQ_GLOBAL_START + 80)
#define MPFS_IRQ_MMUART4                    (MPFS_IRQ_GLOBAL_START + 81)
#define MPFS_IRQ_WDOG0_MRVP                 (MPFS_IRQ_GLOBAL_START + 87)
#define MPFS_IRQ_WDOG1_MRVP                 (MPFS_IRQ_GLOBAL_START + 88)
#define MPFS_IRQ_WDOG2_MRVP                 (MPFS_IRQ_GLOBAL_START + 89)
#define MPFS_IRQ_WDOG3_MRVP                 (MPFS_IRQ_GLOBAL_START + 90)
#define MPFS_IRQ_WDOG4_MRVP                 (MPFS_IRQ_GLOBAL_START + 91)
#define MPFS_IRQ_WDOG0_TOUT                 (MPFS_IRQ_GLOBAL_START + 92)
#define MPFS_IRQ_WDOG1_TOUT                 (MPFS_IRQ_GLOBAL_START + 93)
#define MPFS_IRQ_WDOG2_TOUT                 (MPFS_IRQ_GLOBAL_START + 94)
#define MPFS_IRQ_WDOG3_TOUT                 (MPFS_IRQ_GLOBAL_START + 95)
#define MPFS_IRQ_WDOG4_TOUT                 (MPFS_IRQ_GLOBAL_START + 96)
#define MPFS_IRQ_FABRIC_F2H_0               (MPFS_IRQ_GLOBAL_START + 105)
#define MPFS_IRQ_FABRIC_F2H_1               (MPFS_IRQ_GLOBAL_START + 106)
#define MPFS_IRQ_FABRIC_F2H_2               (MPFS_IRQ_GLOBAL_START + 107)
#define MPFS_IRQ_FABRIC_F2H_3               (MPFS_IRQ_GLOBAL_START + 108)
#define MPFS_IRQ_FABRIC_F2H_4               (MPFS_IRQ_GLOBAL_START + 109)
#define MPFS_IRQ_FABRIC_F2H_5               (MPFS_IRQ_GLOBAL_START + 110)
#define MPFS_IRQ_FABRIC_F2H_6               (MPFS_IRQ_GLOBAL_START + 111)
#define MPFS_IRQ_FABRIC_F2H_7               (MPFS_IRQ_GLOBAL_START + 112)
#define MPFS_IRQ_FABRIC_F2H_8               (MPFS_IRQ_GLOBAL_START + 113)
#define MPFS_IRQ_FABRIC_F2H_9               (MPFS_IRQ_GLOBAL_START + 114)
#define MPFS_IRQ_FABRIC_F2H_10              (MPFS_IRQ_GLOBAL_START + 115)
#define MPFS_IRQ_FABRIC_F2H_11              (MPFS_IRQ_GLOBAL_START + 116)
#define MPFS_IRQ_FABRIC_F2H_12              (MPFS_IRQ_GLOBAL_START + 117)
#define MPFS_IRQ_FABRIC_F2H_13              (MPFS_IRQ_GLOBAL_START + 118)
#define MPFS_IRQ_FABRIC_F2H_14              (MPFS_IRQ_GLOBAL_START + 119)
#define MPFS_IRQ_FABRIC_F2H_15              (MPFS_IRQ_GLOBAL_START + 120)
#define MPFS_IRQ_FABRIC_F2H_16              (MPFS_IRQ_GLOBAL_START + 121)
#define MPFS_IRQ_FABRIC_F2H_17              (MPFS_IRQ_GLOBAL_START + 122)
#define MPFS_IRQ_FABRIC_F2H_18              (MPFS_IRQ_GLOBAL_START + 123)
#define MPFS_IRQ_FABRIC_F2H_19              (MPFS_IRQ_GLOBAL_START + 124)
#define MPFS_IRQ_FABRIC_F2H_20              (MPFS_IRQ_GLOBAL_START + 125)
#define MPFS_IRQ_FABRIC_F2H_21              (MPFS_IRQ_GLOBAL_START + 126)
#define MPFS_IRQ_FABRIC_F2H_22              (MPFS_IRQ_GLOBAL_START + 127)
#define MPFS_IRQ_FABRIC_F2H_23              (MPFS_IRQ_GLOBAL_START + 128)
#define MPFS_IRQ_FABRIC_F2H_24              (MPFS_IRQ_GLOBAL_START + 129)
#define MPFS_IRQ_FABRIC_F2H_25              (MPFS_IRQ_GLOBAL_START + 130)
#define MPFS_IRQ_FABRIC_F2H_26              (MPFS_IRQ_GLOBAL_START + 131)
#define MPFS_IRQ_FABRIC_F2H_27              (MPFS_IRQ_GLOBAL_START + 132)
#define MPFS_IRQ_FABRIC_F2H_28              (MPFS_IRQ_GLOBAL_START + 133)
#define MPFS_IRQ_FABRIC_F2H_29              (MPFS_IRQ_GLOBAL_START + 134)
#define MPFS_IRQ_FABRIC_F2H_30              (MPFS_IRQ_GLOBAL_START + 135)
#define MPFS_IRQ_FABRIC_F2H_31              (MPFS_IRQ_GLOBAL_START + 136)
#define MPFS_IRQ_FABRIC_F2H_32              (MPFS_IRQ_GLOBAL_START + 137)
#define MPFS_IRQ_FABRIC_F2H_33              (MPFS_IRQ_GLOBAL_START + 138)
#define MPFS_IRQ_FABRIC_F2H_34              (MPFS_IRQ_GLOBAL_START + 139)
#define MPFS_IRQ_FABRIC_F2H_35              (MPFS_IRQ_GLOBAL_START + 140)
#define MPFS_IRQ_FABRIC_F2H_36              (MPFS_IRQ_GLOBAL_START + 141)
#define MPFS_IRQ_FABRIC_F2H_37              (MPFS_IRQ_GLOBAL_START + 142)
#define MPFS_IRQ_FABRIC_F2H_38              (MPFS_IRQ_GLOBAL_START + 143)
#define MPFS_IRQ_FABRIC_F2H_39              (MPFS_IRQ_GLOBAL_START + 144)
#define MPFS_IRQ_FABRIC_F2H_40              (MPFS_IRQ_GLOBAL_START + 145)
#define MPFS_IRQ_FABRIC_F2H_41              (MPFS_IRQ_GLOBAL_START + 146)
#define MPFS_IRQ_FABRIC_F2H_42              (MPFS_IRQ_GLOBAL_START + 147)
#define MPFS_IRQ_FABRIC_F2H_43              (MPFS_IRQ_GLOBAL_START + 148)
#define MPFS_IRQ_FABRIC_F2H_44              (MPFS_IRQ_GLOBAL_START + 149)
#define MPFS_IRQ_FABRIC_F2H_45              (MPFS_IRQ_GLOBAL_START + 150)
#define MPFS_IRQ_FABRIC_F2H_46              (MPFS_IRQ_GLOBAL_START + 151)
#define MPFS_IRQ_FABRIC_F2H_47              (MPFS_IRQ_GLOBAL_START + 152)
#define MPFS_IRQ_FABRIC_F2H_48              (MPFS_IRQ_GLOBAL_START + 153)
#define MPFS_IRQ_FABRIC_F2H_49              (MPFS_IRQ_GLOBAL_START + 154)
#define MPFS_IRQ_FABRIC_F2H_50              (MPFS_IRQ_GLOBAL_START + 155)
#define MPFS_IRQ_FABRIC_F2H_51              (MPFS_IRQ_GLOBAL_START + 156)
#define MPFS_IRQ_FABRIC_F2H_52              (MPFS_IRQ_GLOBAL_START + 157)
#define MPFS_IRQ_FABRIC_F2H_53              (MPFS_IRQ_GLOBAL_START + 158)
#define MPFS_IRQ_FABRIC_F2H_54              (MPFS_IRQ_GLOBAL_START + 159)
#define MPFS_IRQ_FABRIC_F2H_55              (MPFS_IRQ_GLOBAL_START + 160)
#define MPFS_IRQ_FABRIC_F2H_56              (MPFS_IRQ_GLOBAL_START + 161)
#define MPFS_IRQ_FABRIC_F2H_57              (MPFS_IRQ_GLOBAL_START + 162)
#define MPFS_IRQ_FABRIC_F2H_58              (MPFS_IRQ_GLOBAL_START + 163)
#define MPFS_IRQ_FABRIC_F2H_59              (MPFS_IRQ_GLOBAL_START + 164)
#define MPFS_IRQ_FABRIC_F2H_60              (MPFS_IRQ_GLOBAL_START + 165)
#define MPFS_IRQ_FABRIC_F2H_61              (MPFS_IRQ_GLOBAL_START + 166)
#define MPFS_IRQ_FABRIC_F2H_62              (MPFS_IRQ_GLOBAL_START + 167)
#define MPFS_IRQ_FABRIC_F2H_63              (MPFS_IRQ_GLOBAL_START + 168)
#define MPFS_IRQ_BUS_ERROR_UNIT_HART_0      (MPFS_IRQ_GLOBAL_START + 169)
#define MPFS_IRQ_BUS_ERROR_UNIT_HART_1      (MPFS_IRQ_GLOBAL_START + 170)
#define MPFS_IRQ_BUS_ERROR_UNIT_HART_2      (MPFS_IRQ_GLOBAL_START + 171)
#define MPFS_IRQ_BUS_ERROR_UNIT_HART_3      (MPFS_IRQ_GLOBAL_START + 172)
#define MPFS_IRQ_BUS_ERROR_UNIT_HART_4      (MPFS_IRQ_GLOBAL_START + 173)

/* Total number of IRQs */

#define NR_IRQS               (MPFS_IRQ_BUS_ERROR_UNIT_HART_4 + 1)

#endif /* __ARCH_RISCV_INCLUDE_MPFS_IRQ_H */
