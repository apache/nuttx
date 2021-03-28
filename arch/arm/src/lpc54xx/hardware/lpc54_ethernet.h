/****************************************************************************
 * arch/arm/src/lpc54xx/hardware/lpc54_ethernet.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_ETHERNET_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define LPC54_ETH_MAC_CONFIG_OFFSET                      0x0000  /* MAC configuration */
#define LPC54_ETH_MAC_EXT_CONFIG_OFFSET                  0x0004  /* MAC extended configuration */
#define LPC54_ETH_MAC_FRAME_FILTER_OFFSET                0x0008  /* MAC frame filter */
#define LPC54_ETH_MAC_WD_TIMEROUT_OFFSET                 0x000c  /* MAC watchdog timeout */
#define LPC54_ETH_MAC_VLAN_TAG_OFFSET                    0x0050  /* VLAN tag */
#define LPC54_ETH_MAC_TX_FLOW_CTRL_Q0_OFFSET             0x0070  /* Transmit flow control 0 */
#define LPC54_ETH_MAC_TX_FLOW_CTRL_Q1_OFFSET             0x0074  /* Transmit flow control 1 */
#define LPC54_ETH_MAC_RX_FLOW_CTRL_OFFSET                0x0090  /* Receive flow control */
#define LPC54_ETH_MAC_TXQ_PRIO_MAP_OFFSET                0x0098  /* Transmit Queue priority mapping */
#define LPC54_ETH_MAC_RXQ_CTRL0_OFFSET                   0x00a0  /* Receive Queue control 0 */
#define LPC54_ETH_MAC_RXQ_CTRL1_OFFSET                   0x00a4  /* Receive Queue control 1 */
#define LPC54_ETH_MAC_RXQ_CTRL2_OFFSET                   0x00a8  /* Receive Queue control 2 */
#define LPC54_ETH_MAC_INTR_STAT_OFFSET                   0x00b0  /* Interrupt status */
#define LPC54_ETH_MAC_INTR_EN_OFFSET                     0x00b4  /* Interrupt enable */
#define LPC54_ETH_MAC_RXTX_STAT_OFFSET                   0x00b8  /* Receive transmit status */
#define LPC54_ETH_MAC_PMT_CRTL_STAT_OFFSET               0x00c0  /* MAC PMT control and status */
#define LPC54_ETH_MAC_RWK_PKT_FLT_OFFSET                 0x00c4  /* Wake-up packet filter */
#define LPC54_ETH_MAC_LPI_CTRL_STAT_OFFSET               0x00d0  /* LPI control and status */
#define LPC54_ETH_MAC_LPI_TIMER_CTRL_OFFSET              0x00d4  /* LPI timers control */
#define LPC54_ETH_MAC_LPI_ENTR_TIMR_OFFSET               0x00d8  /* LPI entry timer */
#define LPC54_ETH_MAC_1US_TIC_COUNTR_OFFSET              0x00dc  /* MAC 1 usec tick counter */
#define LPC54_ETH_MAC_VERSION_OFFSET                     0x0110  /* MAC version */
#define LPC54_ETH_MAC_DBG_OFFSET                         0x0114  /* MAC debug */
#define LPC54_ETH_MAC_HW_FEAT0_OFFSET                    0x011c  /* MAC hardware feature 0 */
#define LPC54_ETH_MAC_HW_FEAT1_OFFSET                    0x0120  /* MAC hardware feature 1 */
#define LPC54_ETH_MAC_HW_FEAT2_OFFSET                    0x0124  /* MAC hardware feature 2 */

#define LPC54_ETH_MAC_MDIO_ADDR_OFFSET                   0x0200  /* MIDO address */
#define LPC54_ETH_MAC_MDIO_DATA_OFFSET                   0x0204  /* MDIO data */
#define LPC54_ETH_MAC_ADDR_HIGH_OFFSET                   0x0300  /* MAC address0 high */
#define LPC54_ETH_MAC_ADDR_LOW_OFFSET                    0x0304  /* MAC address0 low */

#define LPC54_ETH_MAC_TIMESTAMP_CTRL_OFFSET              0x0b00  /* Timestamp control */
#define LPC54_ETH_MAC_SUB_SCND_INCR_OFFSET               0x0b04  /* Sub-second increment */
#define LPC54_ETH_MAC_SYS_TIME_SCND_OFFSET               0x0b08  /* System time seconds */
#define LPC54_ETH_MAC_SYS_TIME_NSCND_OFFSET              0x0b0c  /* System time nanoseconds */
#define LPC54_ETH_MAC_SYS_TIME_SCND_UPD_OFFSET           0x0b10  /* System time seconds update */
#define LPC54_ETH_MAC_SYS_TIME_NSCND_UPD_OFFSET          0x0b14  /* System time nanoseconds update */
#define LPC54_ETH_MAC_SYS_TIMESTMP_ADDEND_OFFSET         0x0b18  /* Timestamp addend */
#define LPC54_ETH_MAC_SYS_TIME_HWORD_SCND_OFFSET         0x0b1c  /* System time-higher word seconds */
#define LPC54_ETH_MAC_SYS_TIMESTMP_STAT_OFFSET           0x0b20  /* Timestamp status */
#define LPC54_ETH_MAC_TX_TIMESTAMP_STATUS_NSECS_OFFSET   0x0b30  /* Tx timestamp status nanoseconds */
#define LPC54_ETH_MAC_TX_TIMESTAMP_STATUS_SECS_OFFSET    0x0b34  /* Tx timestamp status seconds */
#define LPC54_ETH_MAC_TIMESTAMP_INGRESS_CORR_NSEC_OFFSET 0x0b58  /* Timestamp ingress correction */
#define LPC54_ETH_MAC_TIMESTAMP_EGRESS_CORR_NSEC_OFFSET  0x0b5c  /* Timestamp egress correction */

#define LPC54_ETH_MTL_OP_MODE_OFFSET                     0x0c00  /* MTL operation mode */
#define LPC54_ETH_MTL_INTR_STAT_OFFSET                   0x0c20  /* MTL interrupt status */
#define LPC54_ETH_MTL_RXQ_DMA_MAP_OFFSET                 0x0c30  /* MTL Rx Queue and DMA channel mapping */

#define LPC54_ETH_MTL_Qn_OFFSET(n)                       (0x0d00 + ((n) << 6))

#define LPC54_ETH_MTL_TXQ_OP_MODE_OFFSET                 0x0000  /* MTL TxQn operation mode */
#define LPC54_ETH_MTL_TXQ_UNDRFLW_OFFSET                 0x0004  /* MTL TxQn underflow */
#define LPC54_ETH_MTL_TXQ_DBG_OFFSET                     0x0008  /* MTL TxQn debug */
#define LPC54_ETH_MTL_TXQ_ETS_CTRL_OFFSET                0x0010  /* MTL TxQ1 (only) ETS control */
#define LPC54_ETH_MTL_TXQ_ETS_STAT_OFFSET                0x0014  /* MTL TxQn ETS status */
#define LPC54_ETH_MTL_TXQ_QNTM_WGHT_OFFSET               0x0018  /* MTL TxQn idleSlopeCredit, quantum or weights */
#define LPC54_ETH_MTL_TXQ_SNDSLP_CRDT_OFFSET             0x001c  /* MTL TxQ1 (only) SendSlopCredit */
#define LPC54_ETH_MTL_TXQ_HI_CRDT_OFFSET                 0x0020  /* MTL TxQ1 (only) hiCredit */
#define LPC54_ETH_MTL_TXQ_LO_CRDT_OFFSET                 0x0024  /* MTL TxQ1 (only) loCredit */
#define LPC54_ETH_MTL_TXQ_INTCTRL_STAT_OFFSET            0x002c  /* MTL TxQn interrupt control status */
#define LPC54_ETH_MTL_RXQ_OP_MODE_OFFSET                 0x0030  /* MTL RxQn operation mode */
#define LPC54_ETH_MTL_RXQ_MISSPKT_OVRFLW_CNT_OFFSET      0x0034  /* MTL RxQn missed packet overflow counter */
#define LPC54_ETH_MTL_RXQ_DBG_OFFSET                     0x0038  /* MTL RxQn debug */
#define LPC54_ETH_MTL_RXQ_CTRL_OFFSET                    0x003c  /* MTL RxQn control */

#define LPC54_ETH_DMA_MODE_OFFSET                        0x1000  /* DMA mode */
#define LPC54_ETH_DMA_SYSBUS_MODE_OFFSET                 0x1004  /* DMA system bus mode */
#define LPC54_ETH_DMA_INTR_STAT_OFFSET                   0x1008  /* DMA interrupt status */
#define LPC54_ETH_DMA_DBG_STAT_OFFSET                    0x100c  /* DMA debug status */

#define LPC54_ETH_DMACH_OFFSET(n)                        (0x1100 + ((n) << 7))

#define LPC54_ETH_DMACH_CTRL_OFFSET                      0x0000  /* DMA channel n control */
#define LPC54_ETH_DMACH_TX_CTRL_OFFSET                   0x0004  /* DMA channel n transmit control */
#define LPC54_ETH_DMACH_RX_CTRL_OFFSET                   0x0008  /* DMA channel n receive control */
#define LPC54_ETH_DMACH_TXDESC_LIST_ADDR_OFFSET          0x0014  /* DMA channel n Tx descriptor list address */
#define LPC54_ETH_DMACH_RXDESC_LIST_ADDR_OFFSET          0x001c  /* DMA channel n Rx descriptor list address */
#define LPC54_ETH_DMACH_TXDESC_TAIL_PTR_OFFSET           0x0020  /* DMA channel n Tx descriptor tail pointer */
#define LPC54_ETH_DMACH_RXDESC_TAIL_PTR_OFFSET           0x0028  /* DMA channel n Rx descriptor tail pointer */
#define LPC54_ETH_DMACH_TXDESC_RING_LENGTH_OFFSET        0x002c  /* DMA channel n Tx descriptor ring length */
#define LPC54_ETH_DMACH_RXDESC_RING_LENGTH_OFFSET        0x0030  /* DMA channel n Rx descriptor ring length */
#define LPC54_ETH_DMACH_INT_EN_OFFSET                    0x0034  /* DMA channel n interrupt enable */
#define LPC54_ETH_DMACH_RX_INT_WDTIMER_OFFSET            0x0038  /* DMA channel n receive interrupt watchdog timer */
#define LPC54_ETH_DMACH_SLOT_FUNC_CTRL_STAT_OFFSET       0x003c  /* DMA channel n slot function control and status */
#define LPC54_ETH_DMACH_CUR_HST_TXDESC_OFFSET            0x0044  /* DMA channel n current host transmit descriptor */
#define LPC54_ETH_DMACH_CUR_HST_RXDESC_OFFSET            0x004c  /* DMA channel n current host receive descriptor */
#define LPC54_ETH_DMACH_CUR_HST_TXBUF_OFFSET             0x0054  /* DMA channel n current host transmit buffer address */
#define LPC54_ETH_DMACH_CUR_HST_RXBUF_OFFSET             0x005c  /* DMA channel n current application receive buffer address */
#define LPC54_ETH_DMACH_STAT_OFFSET                      0x0060  /* DMA channel n DMA status */
#define LPC54_ETH_DMACH_MISS_FRAME_CNT_OFFSET            0x006c  /* DMA channel n missed frame count */

/* Register addresses *******************************************************/

#define LPC54_ETH_MAC_CONFIG                             (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_CONFIG_OFFSET)
#define LPC54_ETH_MAC_EXT_CONFIG                         (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_EXT_CONFIG_OFFSET)
#define LPC54_ETH_MAC_FRAME_FILTER                       (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_FRAME_FILTER_OFFSET)
#define LPC54_ETH_MAC_WD_TIMEROUT                        (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_WD_TIMEROUT_OFFSET)
#define LPC54_ETH_MAC_VLAN_TAG                           (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_VLAN_TAG_OFFSET)
#define LPC54_ETH_MAC_TX_FLOW_CTRL_Q0                    (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_TX_FLOW_CTRL_Q0_OFFSET)
#define LPC54_ETH_MAC_TX_FLOW_CTRL_Q1                    (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_TX_FLOW_CTRL_Q1_OFFSET)
#define LPC54_ETH_MAC_RX_FLOW_CTRL                       (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_RX_FLOW_CTRL_OFFSET)
#define LPC54_ETH_MAC_TXQ_PRIO_MAP                       (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_TXQ_PRIO_MAP_OFFSET)
#define LPC54_ETH_MAC_RXQ_CTRL0                          (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_RXQ_CTRL0_OFFSET)
#define LPC54_ETH_MAC_RXQ_CTRL1                          (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_RXQ_CTRL1_OFFSET)
#define LPC54_ETH_MAC_RXQ_CTRL2                          (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_RXQ_CTRL2_OFFSET)
#define LPC54_ETH_MAC_INTR_STAT                          (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_INTR_STAT_OFFSET)
#define LPC54_ETH_MAC_INTR_EN                            (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_INTR_EN_OFFSET)
#define LPC54_ETH_MAC_RXTX_STAT                          (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_RXTX_STAT_OFFSET)
#define LPC54_ETH_MAC_PMT_CRTL_STAT                      (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_PMT_CRTL_STAT_OFFSET)
#define LPC54_ETH_MAC_RWK_PKT_FLT                        (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_RWK_PKT_FLT_OFFSET)
#define LPC54_ETH_MAC_LPI_CTRL_STAT                      (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_LPI_CTRL_STAT_OFFSET)
#define LPC54_ETH_MAC_LPI_TIMER_CTRL                     (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_LPI_TIMER_CTRL_OFFSET)
#define LPC54_ETH_MAC_LPI_ENTR_TIMR                      (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_LPI_ENTR_TIMR_OFFSET)
#define LPC54_ETH_MAC_1US_TIC_COUNTR                     (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_1US_TIC_COUNTR_OFFSET)
#define LPC54_ETH_MAC_VERSION                            (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_VERSION_OFFSET)
#define LPC54_ETH_MAC_DBG                                (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_DBG_OFFSET)
#define LPC54_ETH_MAC_HW_FEAT0                           (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_HW_FEAT0_OFFSET)
#define LPC54_ETH_MAC_HW_FEAT1                           (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_HW_FEAT1_OFFSET)
#define LPC54_ETH_MAC_HW_FEAT2                           (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_HW_FEAT2_OFFSET)

#define LPC54_ETH_MAC_MDIO_ADDR                          (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_MDIO_ADDR_OFFSET)
#define LPC54_ETH_MAC_MDIO_DATA                          (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_MDIO_DATA_OFFSET)
#define LPC54_ETH_MAC_ADDR_HIGH                          (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_ADDR_HIGH_OFFSET)
#define LPC54_ETH_MAC_ADDR_LOW                           (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_ADDR_LOW_OFFSET)

#define LPC54_ETH_MAC_TIMESTAMP_CTRL                     (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_TIMESTAMP_CTRL_OFFSET)
#define LPC54_ETH_MAC_SUB_SCND_INCR                      (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_SUB_SCND_INCR_OFFSET)
#define LPC54_ETH_MAC_SYS_TIME_SCND                      (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_SYS_TIME_SCND_OFFSET)
#define LPC54_ETH_MAC_SYS_TIME_NSCND                     (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_SYS_TIME_NSCND_OFFSET)
#define LPC54_ETH_MAC_SYS_TIME_SCND_UPD                  (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_SYS_TIME_SCND_UPD_OFFSET)
#define LPC54_ETH_MAC_SYS_TIME_NSCND_UPD                 (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_SYS_TIME_NSCND_UPD_OFFSET)
#define LPC54_ETH_MAC_SYS_TIMESTMP_ADDEND                (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_SYS_TIMESTMP_ADDEND_OFFSET)
#define LPC54_ETH_MAC_SYS_TIME_HWORD_SCND                (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_SYS_TIME_HWORD_SCND_OFFSET)
#define LPC54_ETH_MAC_SYS_TIMESTMP_STAT                  (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_SYS_TIMESTMP_STAT_OFFSET)
#define LPC54_ETH_MAC_TX_TIMESTAMP_STATUS_NSECS          (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_TX_TIMESTAMP_STATUS_NSECS_OFFSET)
#define LPC54_ETH_MAC_TX_TIMESTAMP_STATUS_SECS           (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_TX_TIMESTAMP_STATUS_SECS_OFFSET)
#define LPC54_ETH_MAC_TIMESTAMP_INGRESS_CORR_NSEC        (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_TIMESTAMP_INGRESS_CORR_NSEC_OFFSET)
#define LPC54_ETH_MAC_TIMESTAMP_EGRESS_CORR_NSEC         (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_TIMESTAMP_EGRESS_CORR_NSEC_OFFSET)

#define LPC54_ETH_MTL_OP_MODE                            (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_OP_MODE_OFFSET)
#define LPC54_ETH_MTL_INTR_STAT                          (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_INTR_STAT_OFFSET)
#define LPC54_ETH_MTL_RXQ_DMA_MAP                        (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_RXQ_DMA_MAP_OFFSET)

#define LPC54_ETH_MTL_Q_BASE(n)                          (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_Qn_OFFSET(n))

#define LPC54_ETH_MTL_TXQ_OP_MODE(n)                     (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_TXQ_OP_MODE_OFFSET)
#define LPC54_ETH_MTL_TXQ_UNDRFLW(n)                     (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_TXQ_UNDRFLW_OFFSET)
#define LPC54_ETH_MTL_TXQ_DBG(n)                         (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_TXQ_DBG_OFFSET)
#define LPC54_ETH_MTL_TXQ_ETS_CTRL(n)                    (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_TXQ_ETS_CTRL_OFFSET)
#define LPC54_ETH_MTL_TXQ_ETS_STAT(n)                    (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_TXQ_ETS_STAT_OFFSET)
#define LPC54_ETH_MTL_TXQ_QNTM_WGHT(n)                   (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_TXQ_QNTM_WGHT_OFFSET)
#define LPC54_ETH_MTL_TXQ_SNDSLP_CRDT(n)                 (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_TXQ_SNDSLP_CRDT_OFFSET)
#define LPC54_ETH_MTL_TXQ_HI_CRDT(n)                     (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_TXQ_HI_CRDT_OFFSET)
#define LPC54_ETH_MTL_TXQ_LO_CRDT(n)                     (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_TXQ_LO_CRDT_OFFSET)
#define LPC54_ETH_MTL_TXQ_INTCTRL_STAT(n)                (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_TXQ_INTCTRL_STAT_OFFSET)
#define LPC54_ETH_MTL_RXQ_OP_MODE(n)                     (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_RXQ_OP_MODE_OFFSET)
#define LPC54_ETH_MTL_RXQ_MISSPKT_OVRFLW_CNT(n)          (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_RXQ_MISSPKT_OVRFLW_CNT_OFFSET)
#define LPC54_ETH_MTL_RXQ_DBG(n)                         (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_RXQ_DBG_OFFSET)
#define LPC54_ETH_MTL_RXQ_CTRL(n)                        (LPC54_ETH_MTL_Q_BASE(n) + LPC54_ETH_MTL_RXQ_CTRL_OFFSET)

#define LPC54_ETH_DMA_MODE                               (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_MODE_OFFSET)
#define LPC54_ETH_DMA_SYSBUS_MODE                        (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_SYSBUS_MODE_OFFSET)
#define LPC54_ETH_DMA_INTR_STAT                          (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_INTR_STAT_OFFSET)
#define LPC54_ETH_DMA_DBG_STAT                           (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_DBG_STAT_OFFSET)

#define LPC54_ETH_DMACH_BASE(n)                          (LPC54_ETHERNET_BASE + LPC54_ETH_DMACH_OFFSET(n))

#define LPC54_ETH_DMACH_CTRL(n)                          (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_CTRL_OFFSET)
#define LPC54_ETH_DMACH_TX_CTRL(n)                       (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_TX_CTRL_OFFSET)
#define LPC54_ETH_DMACH_RX_CTRL(n)                       (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_RX_CTRL_OFFSET)
#define LPC54_ETH_DMACH_TXDESC_LIST_ADDR(n)              (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_TXDESC_LIST_ADDR_OFFSET)
#define LPC54_ETH_DMACH_RXDESC_LIST_ADDR(n)              (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_RXDESC_LIST_ADDR_OFFSET)
#define LPC54_ETH_DMACH_TXDESC_TAIL_PTR(n)               (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_TXDESC_TAIL_PTR_OFFSET)
#define LPC54_ETH_DMACH_RXDESC_TAIL_PTR(n)               (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_RXDESC_TAIL_PTR_OFFSET)
#define LPC54_ETH_DMACH_TXDESC_RING_LENGTH(n)            (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_TXDESC_RING_LENGTH_OFFSET)
#define LPC54_ETH_DMACH_RXDESC_RING_LENGTH(n)            (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_RXDESC_RING_LENGTH_OFFSET)
#define LPC54_ETH_DMACH_INT_EN(n)                        (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_INT_EN_OFFSET)
#define LPC54_ETH_DMACH_RX_INT_WDTIMER(n)                (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_RX_INT_WDTIMER_OFFSET)
#define LPC54_ETH_DMACH_SLOT_FUNC_CTRL_STAT(n)           (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_SLOT_FUNC_CTRL_STAT_OFFSET)
#define LPC54_ETH_DMACH_CUR_HST_TXDESC(n)                (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_CUR_HST_TXDESC_OFFSET)
#define LPC54_ETH_DMACH_CUR_HST_RXDESC(n)                (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_CUR_HST_RXDESC_OFFSET)
#define LPC54_ETH_DMACH_CUR_HST_TXBUF(n)                 (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_CUR_HST_TXBUF_OFFSET)
#define LPC54_ETH_DMACH_CUR_HST_RXBUF(n)                 (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_CUR_HST_RXBUF_OFFSET)
#define LPC54_ETH_DMACH_STAT(n)                          (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_STAT_OFFSET)
#define LPC54_ETH_DMACH_MISS_FRAME_CNT(n)                (LPC54_ETH_DMACH_BASE(n) + LPC54_ETH_DMACH_MISS_FRAME_CNT_OFFSET)

/* Register bit definitions *************************************************/

/* MAC configuration */

#define ETH_MAC_CONFIG_RE                                (1 << 0)  /* Bit 0:  Receiver enable */
#define ETH_MAC_CONFIG_TE                                (1 << 1)  /* Bit 1:  Transmitter enable */
#define ETH_MAC_CONFIG_PRELEN_SHIFT                      (2)       /* Bits 2-3: Preamble length for transmit packets */
#define ETH_MAC_CONFIG_PRELEN_MASK                       (3 << ETH_MAC_CONFIG_PRELEN_SHIFT)
#  define ETH_MAC_CONFIG_PRELEN_7                        (0 << ETH_MAC_CONFIG_PRELEN_SHIFT) /* 7 bytes of preamble */
#  define ETH_MAC_CONFIG_PRELEN_5                        (1 << ETH_MAC_CONFIG_PRELEN_SHIFT) /* 5 bytes of preamble */
#  define ETH_MAC_CONFIG_PRELEN_3                        (2 << ETH_MAC_CONFIG_PRELEN_SHIFT) /* 3 bytes of preamble */
#define ETH_MAC_CONFIG_DC                                (1 << 4)                           /* Bit 4: Deferral check */
#define ETH_MAC_CONFIG_BL_SHIFT                          (5)                                /* Bits 5-6: Back-off limit */
#define ETH_MAC_CONFIG_BL_MASK                           (3 << ETH_MAC_CONFIG_BL_SHIFT)
#  define ETH_MAC_CONFIG_BL_10                           (0 << ETH_MAC_CONFIG_BL_SHIFT) /* k = min (n, 10) */
#  define ETH_MAC_CONFIG_BL_8                            (1 << ETH_MAC_CONFIG_BL_SHIFT) /* k = min (n, 8) */
#  define ETH_MAC_CONFIG_BL_4                            (2 << ETH_MAC_CONFIG_BL_SHIFT) /* k = min (n, 4) */
#  define ETH_MAC_CONFIG_BL_1                            (3 << ETH_MAC_CONFIG_BL_SHIFT) /* k = min (n, 1) */
#define ETH_MAC_CONFIG_DR                                (1 << 8)                       /* Bit 8:  Disable retry */
#define ETH_MAC_CONFIG_DCRS                              (1 << 9)                       /* Bit 9:  Disable carrier sense during transmission */
#define ETH_MAC_CONFIG_DO                                (1 << 10)                      /* Bit 10: Disable receive own */
#define ETH_MAC_CONFIG_ECRSFD                            (1 << 11)                      /* Bit 11: Enable carrier sense full-duplex mode before transmission */
#define ETH_MAC_CONFIG_LM                                (1 << 12)                      /* Bit 12: Loopback mode */
#define ETH_MAC_CONFIG_DM                                (1 << 13)                      /* Bit 13: Duplex mode */
#define ETH_MAC_CONFIG_FES                               (1 << 14)                      /* Bit 14: Speed */
#define ETH_MAC_CONFIG_PS                                (1 << 15)                      /* Bit 15: Port select */
#define ETH_MAC_CONFIG_JE                                (1 << 16)                      /* Bit 16: Jumbo frame enable */
#define ETH_MAC_CONFIG_JD                                (1 << 17)                      /* Bit 17: Jabber disable */
#define ETH_MAC_CONFIG_BE                                (1 << 18)                      /* Bit 18: Packet burst enable */
#define ETH_MAC_CONFIG_WD                                (1 << 19)                      /* Bit 19: Watchdog disable */
#define ETH_MAC_CONFIG_ACS                               (1 << 20)                      /* Bit 20: Automatic pad or CRC stripping */
#define ETH_MAC_CONFIG_CST                               (1 << 21)                      /* Bit 21: CRC stripping for type packets */
#define ETH_MAC_CONFIG_S2KP                              (1 << 22)                      /* Bit 22: IEEE 802.3as support for 2K packets */
#define ETH_MAC_CONFIG_GPSLCE                            (1 << 23)                      /* Bit 23: Giant packet size limit control enable */
#define ETH_MAC_CONFIG_IPG_SHIFT                         (24)                           /* Bits 24-26: Inter-packet gap */
#define ETH_MAC_CONFIG_IPG_MASK                          (7 << ETH_MAC_CONFIG_IPG_SHIFT)
#  define ETH_MAC_CONFIG_IPG_96                          (0 << ETH_MAC_CONFIG_IPG_SHIFT) /* 96 bit times */
#  define ETH_MAC_CONFIG_IPG_88                          (1 << ETH_MAC_CONFIG_IPG_SHIFT) /* 88 bit times */
#  define ETH_MAC_CONFIG_IPG_80                          (2 << ETH_MAC_CONFIG_IPG_SHIFT) /* 80 bit times */
#  define ETH_MAC_CONFIG_IPG_72                          (3 << ETH_MAC_CONFIG_IPG_SHIFT) /* 72 bit times */
#  define ETH_MAC_CONFIG_IPG_64                          (4 << ETH_MAC_CONFIG_IPG_SHIFT) /* 64 bit times */
#  define ETH_MAC_CONFIG_IPG_56                          (5 << ETH_MAC_CONFIG_IPG_SHIFT) /* 56 bit times */
#  define ETH_MAC_CONFIG_IPG_48                          (6 << ETH_MAC_CONFIG_IPG_SHIFT) /* 48 bit times */
#  define ETH_MAC_CONFIG_IPG_40                          (7 << ETH_MAC_CONFIG_IPG_SHIFT) /* 40 bit times */
#define ETH_MAC_CONFIG_IPC                               (1 << 27)                       /* Bit 27: Checksum offload */

/* MAC extended configuration */
#define ETH_MAC_EXT_CONFIG_

/* MAC frame filter */

#define ETH_MAC_FRAME_FILTER_PR                          (1 << 0)  /* Bit 0:  Promiscuous mode */
#define ETH_MAC_FRAME_FILTER_DAIF                        (1 << 3)  /* Bit 3:  DA inverse filtering */
#define ETH_MAC_FRAME_FILTER_PM                          (1 << 4)  /* Bit 4:  Pass all multicast */
#define ETH_MAC_FRAME_FILTER_DBF                         (1 << 5)  /* Bit 5:  Disable broadcast frames */
#define ETH_MAC_FRAME_FILTER_PCF_SHIFT                   (6)       /* Bits 6-7: Pass control frames */
#define ETH_MAC_FRAME_FILTER_PCF_MASK                    (3 << ETH_MAC_FRAME_FILTER_PCF_SHIFT)
#  define ETH_MAC_FRAME_FILTER_PCF_NONE                  (0 << ETH_MAC_FRAME_FILTER_PCF_SHIFT) /* All control frames filtered */
#  define ETH_MAC_FRAME_FILTER_PCF_PAUSE                 (1 << ETH_MAC_FRAME_FILTER_PCF_SHIFT) /* All but pause control frames accepted */
#  define ETH_MAC_FRAME_FILTER_PCF_ALL                   (2 << ETH_MAC_FRAME_FILTER_PCF_SHIFT) /* All control frames accepted */
#  define ETH_MAC_FRAME_FILTER_PCF_FILTERED              (3 << ETH_MAC_FRAME_FILTER_PCF_SHIFT) /* Control frames accepted if pass the address filter */
#define ETH_MAC_FRAME_FILTER_SAIF                        (1 << 8)                              /* Bit 8:  SA inverse filtering */
#define ETH_MAC_FRAME_FILTER_SAF                         (1 << 9)                              /* Bit 9:  Source address filter enable */
#define ETH_MAC_FRAME_FILTER_RA                          (1 << 31)                             /* Bit 31: Receive all */

/* MAC watchdog timeout */
#define ETH_MAC_WD_TIMEROUT_
/* VLAN tag */
#define ETH_MAC_VLAN_TAG_

/* Transmit flow control 0/1 */

#define ETH_MAC_TX_FLOW_CTRL_Q_FCB                       (1 << 0)  /* Bit 0:  Flow control busy/backpressure activate */
#define ETH_MAC_TX_FLOW_CTRL_Q_TFE                       (1 << 1)  /* Bit 1:  Transmit flow control enable */
#define ETH_MAC_TX_FLOW_CTRL_Q_PLT_SHIFT                 (4)       /* Bits 4-6: Pause low threshold */
#define ETH_MAC_TX_FLOW_CTRL_Q_PLT_MASK                  (7 << ETH_MAC_TX_FLOW_CTRL_Q_PLT_SHIFT)
#  define ETH_MAC_TX_FLOW_CTRL_Q_PLT(n)                  ((uint32_t)(n) << ETH_MAC_TX_FLOW_CTRL_Q_PLT_SHIFT)
#define ETH_MAC_TX_FLOW_CTRL_Q_DZPQ                      (1 << 7)  /* Bit 7:  Disable zero-quanta pause */
#define ETH_MAC_TX_FLOW_CTRL_Q_PT_SHIFT                  (16)      /* Bits 16-31: Pause time */
#define ETH_MAC_TX_FLOW_CTRL_Q_PT_MASK                   (0xffff << ETH_MAC_TX_FLOW_CTRL_Q_PT_SHIFT)
#  define ETH_MAC_TX_FLOW_CTRL_Q_PT(n)                   ((uint32_t)(n) << ETH_MAC_TX_FLOW_CTRL_Q_PT_SHIFT)

/* Receive flow control */

#define ETH_MAC_RX_FLOW_CTRL_RFE                         (1 << 0)  /* Bit 0:  Receive flow control enable */
#define ETH_MAC_RX_FLOW_CTRL_UP                          (1 << 1)  /* Bit 1:  Unicast pause packet detect */

/* Transmit Queue priority mapping */

#define ETH_MAC_TXQ_PRIO_MAP_PSTQ0_SHIFT                 (0)       /* Bits 0-7: Priorities selected in Tx Queue 0 */
#define ETH_MAC_TXQ_PRIO_MAP_PSTQ0_MASK                  (0xff << ETH_MAC_TXQ_PRIO_MAP_PSTQ0_SHIFT)
#  define ETH_MAC_TXQ_PRIO_MAP_PSTQ0(n)                  ((uint32_t)(n) << ETH_MAC_TXQ_PRIO_MAP_PSTQ0_SHIFT)
#define ETH_MAC_TXQ_PRIO_MAP_PSTQ1_SHIFT                 (8)       /* Bits 8-15: Priorities selected in Tx Queue 1 */
#define ETH_MAC_TXQ_PRIO_MAP_PSTQ1_MASK                  (0xff << ETH_MAC_TXQ_PRIO_MAP_PSTQ1_SHIFT)
#  define ETH_MAC_TXQ_PRIO_MAP_PSTQ1(n)                  ((uint32_t)(n) << ETH_MAC_TXQ_PRIO_MAP_PSTQ1_SHIFT)

/* Receive Queue control 0 */

#define ETH_MAC_RXQ_CTRL0_RXQ0EN_SHIFT                   (0)       /* Bits 0-1: Rx Queue 0 enable */
#define ETH_MAC_RXQ_CTRL0_RXQ0EN_MASK                    (3 << ETH_MAC_RXQ_CTRL0_RXQ0EN_SHIFT)
#  define ETH_MAC_RXQ_CTRL0_RXQ0EN_DISABLE               (0 << ETH_MAC_RXQ_CTRL0_RXQ0EN_SHIFT) /* Disable */
#  define ETH_MAC_RXQ_CTRL0_RXQ0EN_ENABLE                (1 << ETH_MAC_RXQ_CTRL0_RXQ0EN_SHIFT) /* Queue 0 enabled for AV  */
#define ETH_MAC_RXQ_CTRL0_RXQ1EN_SHIFT                   (2)                                   /* Bits 2-3: Rx Queue 1 enable */
#define ETH_MAC_RXQ_CTRL0_RXQ1EN_MASK                    (3 << ETH_MAC_RXQ_CTRL0_RXQ1EN_SHIFT)
#  define ETH_MAC_RXQ_CTRL0_RXQ1EN_DISABLE               (0 << ETH_MAC_RXQ_CTRL0_RXQ1EN_SHIFT) /* Disable */
#  define ETH_MAC_RXQ_CTRL0_RXQ1EN_ENABLE                (1 << ETH_MAC_RXQ_CTRL0_RXQ1EN_SHIFT) /* Queue 1 enabled for AV */

/* Receive Queue control 1 */

#define ETH_MAC_RXQ_CTRL1_AVCPQ_SHIFT                    (0)       /* Bits 0-2: AV untagged control packets queue */
#define ETH_MAC_RXQ_CTRL1_AVCPQ_MASK                     (7 < ETH_MAC_RXQ_CTRL1_AVCPQ_SHIFT)
#  define ETH_MAC_RXQ_CTRL1_AVCPQ(n)                     ((uint32_t)(n) < ETH_MAC_RXQ_CTRL1_AVCPQ_SHIFT) /* Rx Queue n, n=0..1 */
#define ETH_MAC_RXQ_CTRL1_AVPTPQ_SHIFT                   (4)                                             /* Bits 4-6: AV PTP packets queue */
#define ETH_MAC_RXQ_CTRL1_AVPTPQ_MASK                    (7 < ETH_MAC_RXQ_CTRL1_AVCPQ_SHIFT)
#  define ETH_MAC_RXQ_CTRL1_AVPTPQ(n)                    ((uint32_t)(n) < ETH_MAC_RXQ_CTRL1_AVCPQ_SHIFT) /* Rx Queue n, n=0..1 */
#define ETH_MAC_RXQ_CTRL1_UPQ_SHIFT                      (12)                                            /* Bits 12-14: Untagged packet queue */
#define ETH_MAC_RXQ_CTRL1_UPQ_MASK                       (7 < ETH_MAC_RXQ_CTRL1_AVCPQ_SHIFT)
#  define ETH_MAC_RXQ_CTRL1_UPQ(n)                       ((uint32_t)(n) < ETH_MAC_RXQ_CTRL1_AVCPQ_SHIFT) /* Rx Queue n, n=0..1 */
#define ETH_MAC_RXQ_CTRL1_MCBCQ_SHIFT                    (16)                                            /* Bits 16-18: Multicast and broadcast queue */
#define ETH_MAC_RXQ_CTRL1_MCBCQ_MASK                     (7 < ETH_MAC_RXQ_CTRL1_AVCPQ_SHIFT)
#  define ETH_MAC_RXQ_CTRL1_MCBCQ(n)                     ((uint32_t)(n) < ETH_MAC_RXQ_CTRL1_AVCPQ_SHIFT) /* Rx Queue n, n=0..1 */
#define ETH_MAC_RXQ_CTRL1_MCBCQEN                        (1 << 20)                                       /* Bit 20: Multicast and broadcast queue enable */

/* Receive Queue control 2 */

#define ETH_MAC_RXQ_CTRL2_PSRQ0_SHIFT                    (0)       /* Bits 0-7:   Priorities selected in the Rx Queue 0 */
#define ETH_MAC_RXQ_CTRL2_PSRQ0_MASK                     (0xff << ETH_MAC_RXQ_CTRL2_PSRQ0_SHIFT)
#  define ETH_MAC_RXQ_CTRL2_PSRQ0(n)                     ((uint32_t)(n) << ETH_MAC_RXQ_CTRL2_PSRQ0_SHIFT)
#define ETH_MAC_RXQ_CTRL2_PSRQ1_SHIFT                    (8)       /* Bits 8-15:  Priorities selected in the Rx Queue 1 */
#define ETH_MAC_RXQ_CTRL2_PSRQ1_MASK                     (0xff << ETH_MAC_RXQ_CTRL2_PSRQ1_SHIFT)
#  define ETH_MAC_RXQ_CTRL2_PSRQ1(n)                     ((uint32_t)(n) << ETH_MAC_RXQ_CTRL2_PSRQ1_SHIFT)
#define ETH_MAC_RXQ_CTRL2_PSRQ2_SHIFT                    (16)      /* Bits 16-23: Priorities selected in the Rx Queue 2 */
#define ETH_MAC_RXQ_CTRL2_PSRQ2_MASK                     (0xff << ETH_MAC_RXQ_CTRL2_PSRQ2_SHIFT)
#  define ETH_MAC_RXQ_CTRL2_PSRQ2(n)                     ((uint32_t)(n) << ETH_MAC_RXQ_CTRL2_PSRQ2_SHIFT)
#define ETH_MAC_RXQ_CTRL2_PSRQ3_SHIFT                    (24)      /* Bits 24-31: Priorities selected in the Rx Queue 3 */
#define ETH_MAC_RXQ_CTRL2_PSRQ3_MASK                     (0xff << ETH_MAC_RXQ_CTRL2_PSRQ3_SHIFT)
#  define ETH_MAC_RXQ_CTRL2_PSRQ3(n)                     ((uint32_t)(n) << ETH_MAC_RXQ_CTRL2_PSRQ3_SHIFT)

/* Interrupt enable and interrupt status */

#define ETH_MAC_INTR_PHYI                                (1 << 3)  /* Bit 3:  PHY interrupt */
#define ETH_MAC_INTR_PMTI                                (1 << 4)  /* Bit 4:  PMT interrupt */
#define ETH_MAC_INTR_LPII                                (1 << 5)  /* Bit 5:  LPI interrupt */
#define ETH_MAC_INTR_TSI                                 (1 << 12) /* Bit 12: Timestamp interrupt */
#define ETH_MAC_INTR_TXSTSI                              (1 << 13) /* Bit 13: Transmit status interrupt */
#define ETH_MAC_INTR_RXSTSI                              (1 << 14) /* Bit 14: Receive status interrupt */

/* Receive transmit status */
#define ETH_MAC_RXTX_STAT_
/* MAC PMT control and status */
#define ETH_MAC_PMT_CRTL_STAT_
/* Wake-up packet filter */
#define ETH_MAC_RWK_PKT_FLT_
/* LPI control and status */
#define ETH_MAC_LPI_CTRL_STAT_
/* LPI timers control */
#define ETH_MAC_LPI_TIMER_CTRL_
/* LPI entry timer */
#define ETH_MAC_LPI_ENTR_TIMR_

/* MAC 1 usec tick counter */

#define ETH_MAC_1US_TIC_COUNTR_SHIFT                     (0)       /* Bits 0-11: 1uS TIC counter */
#define ETH_MAC_1US_TIC_COUNTR_MASK                      (0xfff << ETH_MAC_1US_TIC_COUNTR_SHIFT)
#  define ETH_MAC_1US_TIC_COUNTR(n)                      ((uint32_t)((n)-1) << ETH_MAC_1US_TIC_COUNTR_SHIFT)

/* MAC version */
#define ETH_MAC_VERSION_
/* MAC debug */
#define ETH_MAC_DBG_
/* MAC hardware feature 0 */
#define ETH_MAC_HW_FEAT0_
/* MAC hardware feature 1 */
#define ETH_MAC_HW_FEAT1_
/* MAC hardware feature 2 */
#define ETH_MAC_HW_FEAT2_

/* MIDO address */

#define ETH_MAC_MDIO_ADDR_MB                             (1 << 0)  /* Bit 0  MII busy */
#define ETH_MAC_MDIO_ADDR_MOC_SHIFT                      (2)       /* Bits 2-3: MII operation command */
#define ETH_MAC_MDIO_ADDR_MOC_MASK                       (3 << ETH_MAC_MDIO_ADDR_MOC_SHIFT)
#  define ETH_MAC_MDIO_ADDR_MOC_WRITE                    (1 << ETH_MAC_MDIO_ADDR_MOC_SHIFT) /* Write */
#  define ETH_MAC_MDIO_ADDR_MOC_READ                     (3 << ETH_MAC_MDIO_ADDR_MOC_SHIFT) /* Read */
#define ETH_MAC_MDIO_ADDR_CR_SHIFT                       (8)                                /* Bits 8-11: CSR clock range */
#define ETH_MAC_MDIO_ADDR_CR_MASK                        (15 << ETH_MAC_MDIO_ADDR_CR_SHIFT)
#  define ETH_MAC_MDIO_ADDR_CR_DIV42                     (0 << ETH_MAC_MDIO_ADDR_CR_SHIFT) /* CSR=60-100 MHz; MDC=CSR/42 */
#  define ETH_MAC_MDIO_ADDR_CR_DIV62                     (1 << ETH_MAC_MDIO_ADDR_CR_SHIFT) /* CSR=100-150 MHz; MDC=CSR/62 */
#  define ETH_MAC_MDIO_ADDR_CR_DIV16                     (2 << ETH_MAC_MDIO_ADDR_CR_SHIFT) /* CSR=20-35 MHz; MDC=CSR/16 */
#  define ETH_MAC_MDIO_ADDR_CR_DIV26                     (3 << ETH_MAC_MDIO_ADDR_CR_SHIFT) /* CSR=35-60 MHz; MDC=CSR/26 */
#define ETH_MAC_MDIO_ADDR_NTC_SHIFT                      (12)                              /* Bits 12-14: Number of training clocks */
#define ETH_MAC_MDIO_ADDR_NTC_MASK                       (7 << ETH_MAC_MDIO_ADDR_NTC_SHIFT)
#  define ETH_MAC_MDIO_ADDR_NTC(n)                       ((uint32_t)(n) << ETH_MAC_MDIO_ADDR_NTC_SHIFT)
#define ETH_MAC_MDIO_ADDR_RDA_SHIFT                      (16)      /* Bits 16-20: Register/device address */
#define ETH_MAC_MDIO_ADDR_RDA_MASK                       (31 << ETH_MAC_MDIO_ADDR_RDA_SHIFT)
#  define ETH_MAC_MDIO_ADDR_RDA(n)                       ((uint32_t)(n) << ETH_MAC_MDIO_ADDR_RDA_SHIFT)
#define ETH_MAC_MDIO_ADDR_PA_SHIFT                       (21)      /* Bits  21-25: Physical layer address */
#define ETH_MAC_MDIO_ADDR_PA_MASK                        (31 << ETH_MAC_MDIO_ADDR_PA_SHIFT)
#  define ETH_MAC_MDIO_ADDR_PA(n)                        ((uint32_t)(n) << ETH_MAC_MDIO_ADDR_PA_SHIFT)
#define ETH_MAC_MDIO_ADDR_BTB                            (1 << 26) /* Bit 26  Back to back transactions */
#define ETH_MAC_MDIO_ADDR_PSE                            (1 << 27) /* Bit 27  Preamble suppression enable */

/* MDIO data */

#define ETH_MAC_MDIO_DATA_MASK                           0xffff    /* Bits 0-15: 16 bit PHY data */

/* MAC address0 high */

#define ETH_MAC_ADDR_HIGH_A32_47_SHIFT                   (0)       /* Bits 9-15: MAC address 32-47 */
#define ETH_MAC_ADDR_HIGH_A32_47_MASK                    (0xffff << ETH_MAC_ADDR_HIGH_A32_47_SHIFT)
#  define ETH_MAC_ADDR_HIGH_A32_47(n)                    ((uint32_t)(n) << ETH_MAC_ADDR_HIGH_A32_47_SHIFT)
#define ETH_MAC_ADDR_HIGH_DCS                            (1 << 16) /* Bit 16: DMA channel select */

/* MAC address0 low (32-bit MAC address 0-31) */

/* Timestamp control */
#define ETH_MAC_TIMESTAMP_CTRL_
/* Sub-second increment */
#define ETH_MAC_SUB_SCND_INCR_
/* System time seconds */
#define ETH_MAC_SYS_TIME_SCND_
/* System time nanoseconds */
#define ETH_MAC_SYS_TIME_NSCND_
/* System time seconds update */
#define ETH_MAC_SYS_TIME_SCND_UPD_
/* System time nanoseconds update */
#define ETH_MAC_SYS_TIME_NSCND_UPD_
/* Timestamp addend */
#define ETH_MAC_SYS_TIMESTMP_ADDEND_
/* System time-higher word seconds */
#define ETH_MAC_SYS_TIME_HWORD_SCND_
/* Timestamp status */
#define ETH_MAC_SYS_TIMESTMP_STAT_
/* Tx timestamp status nanoseconds */
#define ETH_MAC_TX_TIMESTAMP_STATUS_NSECS_
/* Tx timestamp status seconds */
#define ETH_MAC_TX_TIMESTAMP_STATUS_SECS_
/* Timestamp ingress correction */
#define ETH_MAC_TIMESTAMP_INGRESS_CORR_NSEC_
/* Timestamp egress correction */
#define ETH_MAC_TIMESTAMP_EGRESS_CORR_NSEC_

/* MTL operation mode */

#define ETH_MTL_OP_MODE_DTXSTS                           (1 << 1)  /* Bit 1:  Drop transmit status */
#define ETH_MTL_OP_MODE_RAA                              (1 << 1)  /* Bit 2:  Receive arbitration algorithm */
#  define ETH_MTL_OP_MODE_RAA_SP                         (0)       /*         Strict priority */
#  define ETH_MTL_OP_MODE_RAA_WSP                        (1 << 1)  /*         Weighted Strict Priority */
#define ETH_MTL_OP_MODE_SHALG_SHIFT                      (5)       /* Bits 5-6:  Tx Scheduling Algorithm */
#define ETH_MTL_OP_MODE_SHALG_MASK                       (3 << ETH_MTL_OP_MODE_SHALG_SHIFT)
#  define ETH_MTL_OP_MODE_SHALG_SP                       (0 << ETH_MTL_OP_MODE_SHALG_SHIFT) /* Strict priority */
#  define ETH_MTL_OP_MODE_SHALG_WSP                      (3 << ETH_MTL_OP_MODE_SHALG_SHIFT) /* Weighted Strict */

/* MTL interrupt status */
#define ETH_MTL_INTR_STAT_

/* MTL Rx Queue and DMA channel mapping */

#define ETH_MTL_RXQ_DMA_MAP_Q0MDMACH                     (1 << 0)  /* Bit 0:  Queue 0 mapped to DMA channel 1 */
#define ETH_MTL_RXQ_DMA_MAP_Q0DDMACH                     (1 << 4)  /* Bit 4:  Queue 0 enabled for DA-based DMA channel selection */
#define ETH_MTL_RXQ_DMA_MAP_Q1MDMACH                     (1 << 8)  /* Bit 8:  Queue 1 mapped to DMA channel 1 */
#define ETH_MTL_RXQ_DMA_MAP_Q1DDMACH                     (1 << 12) /* Bit 12: Queue 1 enabled for DA-based DMA channel selection */

/* MTL TxQn operation mode */

#define ETH_MTL_TXQ_OP_MODE_FTQ                          (1 << 0)  /* Bit 0:  Flush Tx Queue */
#define ETH_MTL_TXQ_OP_MODE_TSF                          (1 << 1)  /* Bit 1:  Transmit store and forward */
#define ETH_MTL_TXQ_OP_MODE_TXQEN_SHIFT                  (2)       /* Bits 2-3: Tx Queue enable */
#define ETH_MTL_TXQ_OP_MODE_TXQEN_MASK                   (3 << ETH_MTL_TXQ_OP_MODE_TXQEN_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TXQEN_DISABLE              (0 << ETH_MTL_TXQ_OP_MODE_TXQEN_SHIFT) /* Not enabled */
#  define ETH_MTL_TXQ_OP_MODE_TXQEN_ENABLE               (2 << ETH_MTL_TXQ_OP_MODE_TXQEN_SHIFT) /* Enabled */
#define ETH_MTL_TXQ_OP_MODE_TTC_SHIFT                    (4)                                    /* Bits 4-6: Transmit threshold control */
#define ETH_MTL_TXQ_OP_MODE_TTC_MASK                     (7 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_32                     (0 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_64                     (1 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_96                     (2 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_128                    (3 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_192                    (4 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_256                    (5 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_384                    (6 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_512                    (7 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#define ETH_MTL_TXQ_OP_MODE_TQS_SHIFT                    (16)      /* Bits 16-18: Tx Queue size (x256) */
#define ETH_MTL_TXQ_OP_MODE_TQS_MASK                     (7 << ETH_MTL_TXQ_OP_MODE_TQS_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TQS(n)                     ((uint32_t)((n)-1) << ETH_MTL_TXQ_OP_MODE_TQS_SHIFT)

/* MTL TxQn underflow */
#define ETH_MTL_TXQ_UNDRFLW_
/* MTL TxQn debug */
#define ETH_MTL_TXQ_DBG_
/* MTL TxQ1 (only) ETS control */
#define ETH_MTL_TXQ1_ETS_CTRL_
/* MTL TxQn ETS status */
#define ETH_MTL_TXQ_ETS_STAT_

/* MTL TxQn idleSlopeCredit,quantum or weights */

#define ETH_MTL_TXQ_QNTM_WGHT_MASK                       0x001fffff /* Bits 0-20: IdleSlopeCredit, quantum or weights */

/* MTL TxQ1 (only) SendSlopCredit */
#define ETH_MTL_TXQ1_SNDSLP_CRDT_
/* MTL TxQ1 (only) hiCredit */
#define ETH_MTL_TXQ1_HI_CRDT_
/* MTL TxQ1 (only) loCredit */
#define ETH_MTL_TXQ1_LO_CRDT_
/* MTL TxQn interrupt control status */
#define ETH_MTL_TXQ_INTCTRL_STAT_

/* MTL RxQn operation mode */

#define ETH_MTL_RXQ_OP_MODE_RTC_SHIFT                    (0)       /* Bits 0-1:  Rx Queue threshold control */
#define ETH_MTL_RXQ_OP_MODE_RTC_MASK                     (3 << ETH_MTL_RXQ_OP_MODE_RTC_SHIFT)
#  define ETH_MTL_RXQ_OP_MODE_RTC_64                     (0 << ETH_MTL_RXQ_OP_MODE_RTC_SHIFT)
#  define ETH_MTL_RXQ_OP_MODE_RTC_32                     (1 << ETH_MTL_RXQ_OP_MODE_RTC_SHIFT)
#  define ETH_MTL_RXQ_OP_MODE_RTC_96                     (2 << ETH_MTL_RXQ_OP_MODE_RTC_SHIFT)
#  define ETH_MTL_RXQ_OP_MODE_RTC_128                    (3 << ETH_MTL_RXQ_OP_MODE_RTC_SHIFT)
#define ETH_MTL_RXQ_OP_MODE_FUP                          (1 << 3)  /* Bit 3  Forward undersized good packets */
#define ETH_MTL_RXQ_OP_MODE_FEP                          (1 << 4)  /* Bit 4  Forward error packets */
#define ETH_MTL_RXQ_OP_MODE_RSF                          (1 << 5)  /* Bit 5  Rx Queue store and forward */
#define ETH_MTL_RXQ_OP_MODE_DIS_TCP_EF                   (1 << 6)  /* Bit 6  Disable dropping of TCP/IP checksum error packets */
#define ETH_MTL_RXQ_OP_MODE_RQS_SHIFT                    (20)      /* Bits 20-22: Rx Queue size (x256) */
#define ETH_MTL_RXQ_OP_MODE_RQS_MASK                     (7 << ETH_MTL_RXQ_OP_MODE_RQS_SHIFT)
#  define ETH_MTL_RXQ_OP_MODE_RQS(n)                     ((uint32_t)((n)-1) << ETH_MTL_RXQ_OP_MODE_RQS_SHIFT)

/* MTL RxQn missed packet overflow counter */
#define ETH_MTL_RXQ_MISSPKT_OVRFLW_CNT_
/* MTL RxQn debug */
#define ETH_MTL_RXQ_DBG_

/* MTL RxQn control */

#define ETH_MTL_RXQ_CTRL_WEGT_SHIFT                      (0)       /* Bits 0-2: Rx Queue weight */
#define ETH_MTL_RXQ_CTRL_WEGT_MASK                       (7 << ETH_MTL_RXQ_CTRL_WEGT)
#  define ETH_MTL_RXQ_CTRL_WEGT(n)                       ((uint32_t)(n) << ETH_MTL_RXQ_CTRL_WEGT)
#define ETH_MTL_RXQ_CTRL_FRM_ARBIT                       (1 << 3)  /* Bit 3: Rx Queue packet arbitration */

/* DMA mode */

#define ETH_DMA_MODE_SWR                                 (1 << 0)  /* Bit 0:  Software reset */
#define ETH_DMA_MODE_DA_MASK                             (1 << 1)  /* Bit 1:  DMA Tx or Rx arbitration scheme */
#  define ETH_DMA_MODE_DA_WRR                            (0)       /* Weighted round-robin with Rx:Tx or Tx:Rx */
#  define ETH_DMA_MODE_DA_FIXED                          (1 << 1)  /* Fixed priority */
#define ETH_DMA_MODE_TAA_SHIFT                           (2)       /* Bits 2-4: Transmit arbitration algorithm */
#define ETH_DMA_MODE_TAA_MASK                            (7 << ETH_DMA_MODE_TAA_SHIFT)
#  define ETH_DMA_MODE_TAA_FIXED                         (0 << ETH_DMA_MODE_TAA_SHIFT) /* Fixed priority */
#  define ETH_DMA_MODE_TAA_WSP                           (1 << ETH_DMA_MODE_TAA_SHIFT) /* Weighted strict priority */
#  define ETH_DMA_MODE_TAA_WRR                           (2 << ETH_DMA_MODE_TAA_SHIFT) /* Weighted round-robin */
#define ETH_DMA_MODE_TXPR                                (1 << 11)                     /* Bit 11: Transmit priority */
#define ETH_DMA_MODE_PR_SHIFT                            (12)                          /* Bits 12-14: Priority ratio */
#define ETH_DMA_MODE_PR_MASK                             (7 << ETH_DMA_MODE_PR_SHIFT)
# define ETH_DMA_MODE_PR_1TO1                            (0 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 1:1 */
# define ETH_DMA_MODE_PR_3TO1                            (2 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 3:1 */
# define ETH_DMA_MODE_PR_4TO1                            (3 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 4:1 */
# define ETH_DMA_MODE_PR_5TO1                            (4 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 5:1 */
# define ETH_DMA_MODE_PR_6TO1                            (5 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 6:1 */
# define ETH_DMA_MODE_PR_7TO1                            (6 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 7:1 */
# define ETH_DMA_MODE_PR_8TO1                            (7 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 8:1 */

/* DMA system bus mode */

#define ETH_DMA_SYSBUS_MODE_FB                           (1 << 0)  /* Bit 0:  Fixed burst length */
#define ETH_DMA_SYSBUS_MODE_AAL                          (1 << 12) /* Bit 12: Address-aligned beats */
#define ETH_DMA_SYSBUS_MODE_MB                           (1 << 14) /* Bit 14: Mixed burst */
#define ETH_DMA_SYSBUS_MODE_RB                           (1 << 15) /* Bit 15: Rebuild INCRx burst */

/* DMA interrupt status */

#define ETH_DMA_INTR_STAT_DC0IS                          (1 << 0)  /* Bit 0:  DMA channel 0 interrupt status */
#define ETH_DMA_INTR_STAT_DC1IS                          (1 << 1)  /* Bit 1:  DMA channel 1 interrupt status */
#define ETH_DMA_INTR_STAT_MTLIS                          (1 << 16) /* Bit 16: MTL interrupt status */
#define ETH_DMA_INTR_STAT_MACIS                          (1 << 17) /* Bit 17: MAC interrupt status */

/* DMA debug status */
#define ETH_DMA_DBG_STAT_

/* DMA channel n control */

#define ETH_DMACH_CTRL_PBLx8                             (1 << 16) /* Bit 16: 8xPBL mode */
#define ETH_DMACH_CTRL_DSL_SHIFT                         (18)      /* Bits 18-20: Skip length */

/* DMA channel n transmit control */

#define ETH_DMACH_TX_CTRL_ST                             (1 << 0)  /* Bit 0:  Start or stop transmission command */
#define ETH_DMACH_TX_CTRL_TCW_SHIFT                      (1)       /* Bits 1-3: Transmit channel weight */
#define ETH_DMACH_TX_CTRL_TCW_MASK                       (7 << ETH_DMACH_TX_CTRL_TCW_SHIFT)
#  define ETH_DMACH_TX_CTRL_TCW(n)                       ((uint32_t)(n) << ETH_DMACH_TX_CTRL_TCW_SHIFT)
#define ETH_DMACH_TX_CTRL_OSF                            (1 << 4)  /* Bit 4:  Operate on second frame */
#define ETH_DMACH_TX_CTRL_TXPBL_SHIFT                    (16)      /* Bits 16-21: Transmit programmable burst length */
#define ETH_DMACH_TX_CTRL_TXPBL_MASK                     (0x3f << ETH_DMACH_TX_CTRL_TXPBL_SHIFT)
#  define ETH_DMACH_TX_CTRL_TXPBL(n)                     ((uint32_t)(n) << ETH_DMACH_TX_CTRL_TXPBL_SHIFT)

/* DMA channel n receive control */

#define ETH_DMACH_RX_CTRL_SR                             (1 << 0)  /* Bit 0:  Start or stop receive command */
#define ETH_DMACH_RX_CTRL_RBSZ_SHIFT                     (3)       /* Bits 3-14: Receive buffer size */
#define ETH_DMACH_RX_CTRL_RBSZ_MASK                      (0xfff << ETH_DMACH_RX_CTRL_RBSZ_SHIFT)
#  define ETH_DMACH_RX_CTRL_RBSZ(n)                      ((uint32_t)(n) << ETH_DMACH_RX_CTRL_RBSZ_SHIFT)
#define ETH_DMACH_RX_CTRL_RXPBL_SHIFT                    (16)      /* Bits 16-21: Receive programmable burst length */
#define ETH_DMACH_RX_CTRL_RXPBL_MASK                     (0x3f << ETH_DMACH_RX_CTRL_RXPBL_SHIFT)
#  define ETH_DMACH_RX_CTRL_RXPBL(n)                     ((uint32_t)(n) << ETH_DMACH_RX_CTRL_RXPBL_SHIFT)
#define ETH_DMACH_RX_CTRL_RPF                            (1 << 31) /* Bit 31: DMA Rx channel n packet flush */

/* DMA channel n Tx descriptor list address (32-bit, word-aligned address) */

/* DMA channel n Rx descriptor list address (32-bit, word-aligned address) */

/* DMA channel n Tx descriptor tail pointer (32-bit, word-aligned address) */

/* DMA channel n Rx descriptor tail pointer (32-bit, word-aligned address) */

/* DMA channel n Tx descriptor ring length */

#define ETH_DMACH_TXDESC_RING_LENGTH_SHIFT               (0)       /* Bits 0-9: Transmit ring length */
#define ETH_DMACH_TXDESC_RING_LENGTH_MASK                (0x3ff << ETH_DMACH_TXDESC_RING_LENGTH_SHIFT)
#  define ETH_DMACH_TXDESC_RING_LENGTH(n)                ((uint32_t)((n)-1) << ETH_DMACH_TXDESC_RING_LENGTH_SHIFT)

/* DMA channel n Rx descriptor ring length */

#define ETH_DMACH_RXDESC_RING_LENGTH_SHIFT               (0)       /* Bits 0-9: Receive ring length */
#define ETH_DMACH_RXDESC_RING_LENGTH_MASK                (0x3ff << ETH_DMACH_RXDESC_RING_LENGTH_SHIFT)
#  define ETH_DMACH_RXDESC_RING_LENGTH(n)                ((uint32_t)((n)-1) << ETH_DMACH_RXDESC_RING_LENGTH_SHIFT)

/* DMA channel n interrupt enable and DMA channel n DMA status */

#define ETH_DMACH_INT_TI                                 (1 << 0)  /* Bit 0:  Transmit interrupt */
#define ETH_DMACH_INT_TS                                 (1 << 1)  /* Bit 1:  Transmitter stopped */
#define ETH_DMACH_INT_TBU                                (1 << 2)  /* Bit 2:  Transmit buffer unavailable */
#define ETH_DMACH_INT_RI                                 (1 << 6)  /* Bit 6:  Receive interrupt */
#define ETH_DMACH_INT_RBU                                (1 << 7)  /* Bit 7:  Receive buffer unavailable */
#define ETH_DMACH_INT_RS                                 (1 << 8)  /* Bit 8:  Receiver stopped */
#define ETH_DMACH_INT_RWT                                (1 << 9)  /* Bit 9:  Receive watchdog timeout */
#define ETH_DMACH_INT_ETI                                (1 << 10) /* Bit 10: Early transmit interrupt */
#define ETH_DMACH_INT_ERI                                (1 << 11) /* Bit 11: Early receive interrupt */
#define ETH_DMACH_INT_FBE                                (1 << 12) /* Bit 12: Fatal bus error */
#define ETH_DMACH_INT_AI                                 (1 << 14) /* Bit 14: Abnormal interrupt summary */
#define ETH_DMACH_INT_NI                                 (1 << 15) /* Bit 15: Normal interrupt summary */

/* DMA channel n receive interrupt watchdog timer */
#define ETH_DMACH_RX_INT_WDTIMER_
/* DMA channel n slot function control and status */
#define ETH_DMACH_SLOT_FUNC_CTRL_STAT_
/* DMA channel n current host transmit descriptor */
#define ETH_DMACH_CUR_HST_TXDESC_
/* DMA channel n current host receive descriptor */
#define ETH_DMACH_CUR_HST_RXDESC_
/* DMA channel n current host transmit buffer address */
#define ETH_DMACH_CUR_HST_TXBUF_
/* DMA channel n current application receive buffer address */
#define ETH_DMACH_CUR_HST_RXBUF_

/* DMA channel n missed frame count */
#define ETH_DMACH_MISS_FRAME_CNT_

/* DMA descriptors **********************************************************/

/* Receive descriptor (read-format) */

/* RDES0: 32-bit address */

/* RDES1: Reserved */

/* RDES2: 32-bit address */

/* RDES3: */

#define ETH_RXDES3_BUF1V                                 (1 << 24) /* Bit 24:  Buffer 1 address valid */
#define ETH_RXDES3_BUF2V                                 (1 << 25) /* Bit 25:  Buffer 1 address valid */
#define ETH_RXDES3_IOC                                   (1 << 30) /* Bit 30:  Interrupt enabled on completion */
#define ETH_RXDES3_OWN                                   (1 << 31) /* Bit 31:  Own bit */

/* Receive descriptor (writeback-format) */

/* RDES0: Reserved */

/* RDES1: */

#define ETH_RXDES1_PT_SHIFT                              (0)       /* Bits 0-2: Payload type */
#define ETH_RXDES1_PT_MASK                               (7 << ETH_RXDES1_PT_SHIFT)
#  define ETH_RXDES1_PT_UNKNOWN                          (0 << ETH_RXDES1_PT_SHIFT) /* Unknown */
#  define ETH_RXDES1_PT_UDP                              (1 << ETH_RXDES1_PT_SHIFT) /* UDP */
#  define ETH_RXDES1_PT_TCP                              (2 << ETH_RXDES1_PT_SHIFT) /* TCP */
#  define ETH_RXDES1_PT_ICMP                             (3 << ETH_RXDES1_PT_SHIFT) /* ICMP */
#  define ETH_RXDES1_PT_IGMP                             (4 << ETH_RXDES1_PT_SHIFT) /* IGMP */
#  define ETH_RXDES1_PT_AVUCP                            (5 << ETH_RXDES1_PT_SHIFT) /* AV untagged control packet */
#  define ETH_RXDES1_PT_AVTDP                            (6 << ETH_RXDES1_PT_SHIFT) /* AV tagged data packet */
#  define ETH_RXDES1_PT_AVTCP                            (7 << ETH_RXDES1_PT_SHIFT) /* AV tagged control packet */
#define ETH_RXDES1_IPHE                                  (1 << 3)                   /* Bit 3:  IP header error */
#define ETH_RXDES1_IPV4                                  (1 << 4)                   /* Bit 4:  IPV4 header present */
#define ETH_RXDES1_IPV6                                  (1 << 5)                   /* Bit 5:  IPv6 header present */
#define ETH_RXDES1_IPCB                                  (1 << 6)                   /* Bit 6:  IP checksum bypassed */
#define ETH_RXDES1_IPCE                                  (1 << 7)                   /* Bit 7:  IP payload error */
#define ETH_RXDES1_PMT_SHIFT                             (8)                        /* Bits 8-11: PTP message type */
#define ETH_RXDES1_PMT_MASK                              (15 << ETH_RXDES1_PMT_SHIFT)
#  define ETH_RXDES1_PMT_NONE                            (0 << ETH_RXDES1_PMT_SHIFT)  /* No PTP message received */
#  define ETH_RXDES1_PMT_SYNC                            (1 << ETH_RXDES1_PMT_SHIFT)  /* SYNC */
#  define ETH_RXDES1_PMT_FOLLOWUP                        (2 << ETH_RXDES1_PMT_SHIFT)  /* Follow_Up */
#  define ETH_RXDES1_PMT_DELAYREQ                        (3 << ETH_RXDES1_PMT_SHIFT)  /* Delay Req */
#  define ETH_RXDES1_PMT_DELAYRESP                       (4 << ETH_RXDES1_PMT_SHIFT)  /* Delay Resp */
#  define ETH_RXDES1_PMT_PDELAYREQ                       (5 << ETH_RXDES1_PMT_SHIFT)  /* Pdelay Req */
#  define ETH_RXDES1_PMT_PDELAYRESP                      (6 << ETH_RXDES1_PMT_SHIFT)  /* Pdelay Resp */
#  define ETH_RXDES1_PMT_PDELAYFOLLOWUP                  (7 << ETH_RXDES1_PMT_SHIFT)  /* Pdelay Resp follow-up */
#  define ETH_RXDES1_PMT_ANNOUNCE                        (8 << ETH_RXDES1_PMT_SHIFT)  /* Announce */
#  define ETH_RXDES1_PMT_MGMNT                           (9 << ETH_RXDES1_PMT_SHIFT)  /* Management */
#  define ETH_RXDES1_PMT_SIGNALING                       (10 << ETH_RXDES1_PMT_SHIFT) /* Signaling */
#  define ETH_RXDES1_PMT_RESERVERD                       (15 << ETH_RXDES1_PMT_SHIFT) /* Reserved message type */
#define ETH_RXDES1_PFT                                   (1 << 12)                    /* Bit 12: PTP packet type */
#define ETH_RXDES1_PV                                    (1 << 13)                    /* Bit 13: PTP version */
#define ETH_RXDES1_TSA                                   (1 << 14)                    /* Bit 14: Timestamp available */
#define ETH_RXDES1_TD                                    (1 << 15)                    /* Bit 15: Timestamp dropped */
#define ETH_RXDES1_OPC_SHIFT                             (16)                         /* Bits 16-31: OAM sub-type code */

/* RXDES2: */

#define ETH_RXDES2_SAF                                   (1 << 16) /* Bit 16: SA address filter fail */
#define ETH_RXDES2_DAF                                   (1 << 17) /* Bit 17: Destination address filter fail */
#define ETH_RXDES2_MADRM_SHIFT                           (19)      /* Bits 19-26: MAC address match */
#define ETH_RXDES2_MADRM_MASK                            (0xff << ETH_RXDES2_MADRM_SHIFT)

/* RXDES3: */

#define ETH_RXDES3_PL_SHIFT                              (0)       /* Bits 0-14: Packet length */
#define ETH_RXDES3_PL_MASK                               (0x7fff << ETH_RXDES3_PL_SHIFT)
#define ETH_RXDES3_ES                                    (1 << 15) /* Bit 15: Error summary */
#define ETH_RXDES3_LT_SHIFT                              (16)      /* Bits 16-18: Length/type */
#define ETH_RXDES3_LT_MASK                               (7 << ETH_RXDES3_LT_SHIFT)
#  define ETH_RXDES3_LT_PKTLEN                           (0 << ETH_RXDES3_LT_SHIFT) /* Packet is a length packet */
#  define ETH_RXDES3_LT_PKTTYPE                          (1 << ETH_RXDES3_LT_SHIFT) /* Packet is a type packet */
#  define ETH_RXDES3_LT_ARPREQ                           (3 << ETH_RXDES3_LT_SHIFT) /* Packet is a ARP request packet type */
#  define ETH_RXDES3_LT_VLAN                             (4 << ETH_RXDES3_LT_SHIFT) /* Packet is a type packet with VLAN tag */
#  define ETH_RXDES3_LT_DVLAN                            (5 << ETH_RXDES3_LT_SHIFT) /* Packet is a type packet with double VLAN tag */
#  define ETH_RXDES3_LT_CTRLPKT                          (6 << ETH_RXDES3_LT_SHIFT) /* Packet is a MAC control packet type */
#  define ETH_RXDES3_LT_OAM                              (7 << ETH_RXDES3_LT_SHIFT) /* Packet is a OAM packet type */
#define ETH_RXDES3_DE                                    (1 << 19)                  /* Bit 19: Dribble bit error */
#define ETH_RXDES3_RE                                    (1 << 20)                  /* Bit 20: Receive error */
#define ETH_RXDES3_OE                                    (1 << 21)                  /* Bit 21: Overflow error */
#define ETH_RXDES3_RWT                                   (1 << 22)                  /* Bit 22: Receive watchdog timeout */
#define ETH_RXDES3_GP                                    (1 << 23)                  /* Bit 23: Giant packet */
#define ETH_RXDES3_CE                                    (1 << 24)                  /* Bit 24: CRC error */
#define ETH_RXDES3_RS0V                                  (1 << 25)                  /* Bit 25: Receive status RDES0 valid */
#define ETH_RXDES3_RS1V                                  (1 << 26)                  /* Bit 26: Receive status RDES1 valid */
#define ETH_RXDES3_RS2V                                  (1 << 27)                  /* Bit 27: Receive status RDES2 valid */
#define ETH_RXDES3_LD                                    (1 << 28)                  /* Bit 28: Last descriptor */
#define ETH_RXDES3_FD                                    (1 << 29)                  /* Bit 29: First descriptor */
#define ETH_RXDES3_CTXT                                  (1 << 30)                  /* Bit 30: Receive context descriptor */
                                                                                    /* Bit 31: Own bit (see read-format) */

/* Transmit normal descriptor (read-format) */

/* TDES0/1: 32-bit address */

/* TDES2: */

#define ETH_TXDES2_B1L_SHIFT                             (0)       /* Bits 0-13: Buffer 1 length */
#define ETH_TXDES2_B1L_MASK                              (0x3fff << ETH_TXDES2_B1L_SHIFT)
#  define ETH_TXDES2_B1L(n)                              ((uint32_t)(n) << ETH_TXDES2_B1L_SHIFT)
#define ETH_TXDES2_B2L_SHIFT                             (16)      /* Bits 16-29: Buffer 2 length */
#define ETH_TXDES2_B2L_MASK                              (0x3fff << ETH_TXDES2_B2L_SHIFT)
#  define ETH_TXDES2_B2L(n)                              ((uint32_t)(n) << ETH_TXDES2_B2L_SHIFT)
#define ETH_TXDES2_TTSE                                  (1 << 30) /* Bit 30: Transmit timestamp enable */
#define ETH_TXDES2_IOC                                   (1 << 31) /* Bit 31: Interrupt on completion */

/* TDES3: */

#define ETH_TXDES3_FL_SHIFT                              (0)       /* Bits 0-14: Frame length */
#define ETH_TXDES3_FL_MASK                               (0x7fff << ETH_TXDES3_FL_SHIFT)
#  define ETH_TXDES3_FL(n)                               ((uint32_t)(n) << ETH_TXDES3_FL_SHIFT)
#define ETH_TXDES3_CIC_SHIFT                             (16)      /* Bits 16-17: Checksum insertion control */
#define ETH_TXDES3_CIC_MASK                              (3 << ETH_TXDES3_CIC_SHIFT)
#  define ETH_TXDES3_CIC_DISABLED                        (0 << ETH_TXDES3_CIC_SHIFT) /* Checksum insertion disabled */
#  define ETH_TXDES3_CIC_IPHDR                           (1 << ETH_TXDES3_CIC_SHIFT) /* Only IP header checksum */
#  define ETH_TXDES3_CIC_PAYLOAD                         (2 << ETH_TXDES3_CIC_SHIFT) /* IP header checksum and payload checksum */
#  define ETH_TXDES3_CIC_ALL                             (3 << ETH_TXDES3_CIC_SHIFT) /* IP Header checksum, payload, and pseudo-header checksum */
#define ETH_TXDES3_SLOTNUM_SHIFT                         (19)                        /* Bits 19-22: Slot number control bits in AV mode */
#define ETH_TXDES3_SLOTNUM_MASK                          (15 << ETH_TXDES3_SLOTNUM_SHIFT)
#define ETH_TXDES3_CPC_SHIFT                             (26)      /* Bits 26-27: CRC pad control */
#define ETH_TXDES3_CPC_MASK                              (3 << ETH_TXDES2_B1L_SHIFT)
#  define ETH_TXDES3_CPC_CRCPAD                          (0 << ETH_TXDES2_B1L_SHIFT) /* CRC and pad insertion */
#  define ETH_TXDES3_CPC_CRC                             (1 << ETH_TXDES2_B1L_SHIFT) /* CRC insertion (disable pad insertion) */
#  define ETH_TXDES3_CPC_DISABLED                        (2 << ETH_TXDES2_B1L_SHIFT) /* Disable CRC insertion */
#  define ETH_TXDES3_CPC_REPLACMENT                      (3 << ETH_TXDES2_B1L_SHIFT) /* CRC replacement */
#define ETH_TXDES3_LD                                    (1 << 28)                   /* Bit 28: Last descriptor */
#define ETH_TXDES3_FD                                    (1 << 29)                   /* Bit 29: First descriptor */
#define ETH_TXDES3_CTXT                                  (1 << 30)                   /* Bit 30: Context type */
#define ETH_TXDES3_OWN                                   (1 << 31)                   /* Bit 31: Own bit */

/* Transmit normal descriptor (writeback-format) */

/* TDES0/1: 64-bit transmit packet timestamp */

/* TDES2: Reserved */

/* TDES3: */

#define ETH_TXDES3_IHE                                   (1 << 0)  /* Bit 0:  IP header error */
#define ETH_TXDES3_DB                                    (1 << 1)  /* Bit 1:  Deferred bit */
#define ETH_TXDES3_UF                                    (1 << 2)  /* Bit 2:  Underflow error */
#define ETH_TXDES3_ED                                    (1 << 3)  /* Bit 3:  Excessive deferral */
#define ETH_TXDES3_CC_SHIFT                              (4)       /* Bits 4-7: Collision count */
#define ETH_TXDES3_CC_MASK                               (15 << ETH_TXDES3_CC_SHIFT)
#define ETH_TXDES3_EC                                    (1 << 8)  /* Bit 8:  Excessive collision */
#define ETH_TXDES3_LC                                    (1 << 9)  /* Bit 9:  Late collision */
#define ETH_TXDES3_NC                                    (1 << 10) /* Bit 10: No carrier */
#define ETH_TXDES3_LOC                                   (1 << 11) /* Bit 11: Loss of carrier */
#define ETH_TXDES3_PCE                                   (1 << 12) /* Bit 12: Payload checksum error */
#define ETH_TXDES3_FF                                    (1 << 13) /* Bit 13: Packet flushed */
#define ETH_TXDES3_JT                                    (1 << 14) /* Bit 14: Jabber timeout */
#define ETH_TXDES3_ES                                    (1 << 15) /* Bit 15: Error summary */
#define ETH_TXDES3_TTSS                                  (1 << 17) /* Bit 17: Tx timestamp status */
                                                                   /* Bit 28: (see read format) */
                                                                   /* Bit 29: (see read format) */
                                                                   /* Bit 30: (see read format) */
                                                                   /* Bit 31: (see read format) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Receive descriptor structure (read format) */

struct enet_rxdesc_s
{
  uint32_t buffer1;   /* Buffer 1 address */
  uint32_t reserved;  /* Reserved */
  uint32_t buffer2;   /* Buffer 2 or next descriptor address */
  uint32_t ctrl;      /* Buffer 1/2 byte counts and control */
};

/* Transmit descriptor structure (read format) */

struct enet_txdesc_s
{
  uint32_t buffer1;   /* TDES0 Buffer 1 address */
  uint32_t buffer2;   /* TDES1 Buffer 2 address */
  uint32_t buflen;    /* TDES2 Buffer 1/2 byte counts */
  uint32_t ctrlstat;  /* TDES3 Control and status word */
};

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_ETHERNET_H */
