/************************************************************************************************************
 * arch/arm/src/lpc54xx/lpc54_ethernet.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_ETHERNET_H
#define __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_ETHERNET_H

/************************************************************************************************************
 * Included Files
 ************************************************************************************************************/

#include <nuttx/config.h>
#include "chip/lpc54_memorymap.h"

/************************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************************/

/* Register offsets *****************************************************************************************/

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
#define LPC54_ETH_MAC_Tx_TIMESTAMP_STATUS_NSECS_OFFSET   0x0b30  /* Tx timestamp status nanoseconds */
#define LPC54_ETH_MAC_Tx_TIMESTAMP_STATUS_SECS_OFFSET    0x0b34  /* Tx timestamp status seconds */
#define LPC54_ETH_MAC_TIMESTAMP_INGRESS_CORR_NSEC_OFFSET 0x0b58  /* Timestamp ingress correction */
#define LPC54_ETH_MAC_TIMESTAMP_EGRESS_CORR_NSEC_OFFSET  0x0b5c  /* Timestamp egress correction */

#define LPC54_ETH_MTL_OP_MODE_OFFSET                     0x0c00  /* MTL operation mode */
#define LPC54_ETH_MTL_INTR_STAT_OFFSET                   0x0c20  /* MTL interrupt status */
#define LPC54_ETH_MTL_RXQ_DMA_MAP_OFFSET                 0x0c30  /* MTL Rx Queue and DMA channel mapping */

#define LPC54_ETH_MTL_Q_OFFSET(n)                        (0x0d00 + ((n) << 6))

#define LPC54_ETH_MTL_TXQ_OP_MODE_OFFSET                 0x0000  /* MTL TxQn operation mode */
#define LPC54_ETH_MTL_TXQ_UNDRFLW_OFFSET                 0x0004  /* MTL TxQn underflow */
#define LPC54_ETH_MTL_TXQ_DBG_OFFSET                     0x0008  /* MTL TxQn debug */
#define LPC54_ETH_MTL_TXQ_ETS_CTRL_OFFSET                0x0010  /* MTL TxQ1 (only) ETS control */
#define LPC54_ETH_MTL_TXQ_ETS_STAT_OFFSET                0x0014  /* MTL TxQn ETS status */
#define LPC54_ETH_MTL_TXQ_QNTM_WGHT_OFFSET               0x0018  /* MTL TxQn quantum or weights */
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

#define LPC54_ETH_DMACH_CTRL_OFFSET(n)                   (0x1100 + ((n) << 7))

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

/* Register addresses ***************************************************************************************/

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
#define LPC54_ETH_MAC_Tx_TIMESTAMP_STATUS_NSECS          (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_Tx_TIMESTAMP_STATUS_NSECS_OFFSET)
#define LPC54_ETH_MAC_Tx_TIMESTAMP_STATUS_SECS           (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_Tx_TIMESTAMP_STATUS_SECS_OFFSET)
#define LPC54_ETH_MAC_TIMESTAMP_INGRESS_CORR_NSEC        (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_TIMESTAMP_INGRESS_CORR_NSEC_OFFSET)
#define LPC54_ETH_MAC_TIMESTAMP_EGRESS_CORR_NSEC         (LPC54_ETHERNET_BASE + LPC54_ETH_MAC_TIMESTAMP_EGRESS_CORR_NSEC_OFFSET)

#define LPC54_ETH_MTL_OP_MODE                            (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_OP_MODE_OFFSET)
#define LPC54_ETH_MTL_INTR_STAT                          (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_INTR_STAT_OFFSET)
#define LPC54_ETH_MTL_RXQ_DMA_MAP                        (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_RXQ_DMA_MAP_OFFSET)

#define LPC54_ETH_MTL_Q_BASE(n)                          (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_Q_OFFSET(n))

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

#define LPC54_ETH_DMACH_CTRL_BASE(n)                     (LPC54_ETHERNET_BASE + LPC54_ETH_DMACH_CTRL_OFFSET(n))

#define LPC54_ETH_DMACH_CTRL(n)                          (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_CTRL_OFFSET)
#define LPC54_ETH_DMACH_TX_CTRL(n)                       (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_TX_CTRL_OFFSET)
#define LPC54_ETH_DMACH_RX_CTRL(n)                       (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_RX_CTRL_OFFSET)
#define LPC54_ETH_DMACH_TXDESC_LIST_ADDR(n)              (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_TXDESC_LIST_ADDR_OFFSET)
#define LPC54_ETH_DMACH_RXDESC_LIST_ADDR(n)              (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_RXDESC_LIST_ADDR_OFFSET)
#define LPC54_ETH_DMACH_TXDESC_TAIL_PTR(n)               (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_TXDESC_TAIL_PTR_OFFSET)
#define LPC54_ETH_DMACH_RXDESC_TAIL_PTR(n)               (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_RXDESC_TAIL_PTR_OFFSET)
#define LPC54_ETH_DMACH_TXDESC_RING_LENGTH(n)            (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_TXDESC_RING_LENGTH_OFFSET)
#define LPC54_ETH_DMACH_RXDESC_RING_LENGTH(n)            (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_RXDESC_RING_LENGTH_OFFSET)
#define LPC54_ETH_DMACH_INT_EN(n)                        (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_INT_EN_OFFSET)
#define LPC54_ETH_DMACH_RX_INT_WDTIMER(n)                (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_RX_INT_WDTIMER_OFFSET)
#define LPC54_ETH_DMACH_SLOT_FUNC_CTRL_STAT(n)           (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_SLOT_FUNC_CTRL_STAT_OFFSET)
#define LPC54_ETH_DMACH_CUR_HST_TXDESC(n)                (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_CUR_HST_TXDESC_OFFSET)
#define LPC54_ETH_DMACH_CUR_HST_RXDESC(n)                (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_CUR_HST_RXDESC_OFFSET)
#define LPC54_ETH_DMACH_CUR_HST_TXBUF(n)                 (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_CUR_HST_TXBUF_OFFSET)
#define LPC54_ETH_DMACH_CUR_HST_RXBUF(n)                 (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_CUR_HST_RXBUF_OFFSET)
#define LPC54_ETH_DMACH_STAT(n)                          (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_STAT_OFFSET)
#define LPC54_ETH_DMACH_MISS_FRAME_CNT(n)                (LPC54_ETH_DMACH_CTRL_BASE(n) + LPC54_ETH_DMACH0_MISS_FRAME_CNT_OFFSET)

/* Register bit definitions *********************************************************************************/

/* MAC configuration */
#define ETH_MAC_CONFIG_
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
#define ETH_MAC_FRAME_FILTER_SAIF                        (1 << 8)  /* Bit 8:  SA inverse filtering */
#define ETH_MAC_FRAME_FILTER_SAF                         (1 << 9)  /* Bit 9:  Source address filter enable */
#define ETH_MAC_FRAME_FILTER_RA                          (1 << 31) /* Bit 31:  Receive all */

/* MAC watchdog timeout */
#define ETH_MAC_WD_TIMEROUT_
/* VLAN tag */
#define ETH_MAC_VLAN_TAG_
/* Transmit flow control 0 */
#define ETH_MAC_TX_FLOW_CTRL_Q0_
/* Transmit flow control 1 */
#define ETH_MAC_TX_FLOW_CTRL_Q1_
/* Receive flow control */
#define ETH_MAC_RX_FLOW_CTRL_
/* Transmit Queue priority mapping */
#define ETH_MAC_TXQ_PRIO_MAP_
/* Receive Queue control 0 */
#define ETH_MAC_RXQ_CTRL0_
/* Receive Queue control 1 */
#define ETH_MAC_RXQ_CTRL1_
/* Receive Queue control 2 */
#define ETH_MAC_RXQ_CTRL2_
/* Interrupt status */
#define ETH_MAC_INTR_STAT_
/* Interrupt enable */
#define ETH_MAC_INTR_EN_
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
#define ETH_MAC_1US_TIC_COUNTR_
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

#define ETH_MAC_MDIO_ADDR_MB                   (1 << 0)  /* Bit 0  MII busy */
#define ETH_MAC_MDIO_ADDR_MOC_SHIFT            (2)       /* Bits 2-3: MII operation command */
#define ETH_MAC_MDIO_ADDR_MOC_MASK             (3 << ETH_MAC_MDIO_ADDR_MOC_SHIFT)
#  define ETH_MAC_MDIO_ADDR_MOC_WRITE          (1 << ETH_MAC_MDIO_ADDR_MOC_SHIFT) /* Write */
#  define ETH_MAC_MDIO_ADDR_MOC_READ           (3 << ETH_MAC_MDIO_ADDR_MOC_SHIFT) /* Read */
#define ETH_MAC_MDIO_ADDR_CR_SHIFT             (8)       /* Bits 8-11: CSR clock range */
#define ETH_MAC_MDIO_ADDR_CR_MASK              (15 << ETH_MAC_MDIO_ADDR_CR_SHIFT)
#  define ETH_MAC_MDIO_ADDR_CR_DIV42           (0 << ETH_MAC_MDIO_ADDR_CR_SHIFT) /* CSR=60-100 MHz; MDC=CSR/42 */
#  define ETH_MAC_MDIO_ADDR_CR_DIV62           (1 << ETH_MAC_MDIO_ADDR_CR_SHIFT) /* CSR=100-150 MHz; MDC=CSR/62 */
#  define ETH_MAC_MDIO_ADDR_CR_DIV16           (2 << ETH_MAC_MDIO_ADDR_CR_SHIFT) /* CSR=20-35 MHz; MDC=CSR/16 */
#  define ETH_MAC_MDIO_ADDR_CR_DIV26           (3 << ETH_MAC_MDIO_ADDR_CR_SHIFT) /* CSR=35-60 MHz; MDC=CSR/26 */
#define ETH_MAC_MDIO_ADDR_NTC_SHIFT            (12)      /* Bits 12-14: Number of training clocks */
#define ETH_MAC_MDIO_ADDR_NTC_MASK             (7 << ETH_MAC_MDIO_ADDR_NTC_SHIFT)
#  define ETH_MAC_MDIO_ADDR_NTC(n)             ((uint32_t)(n) << ETH_MAC_MDIO_ADDR_NTC_SHIFT)
#define ETH_MAC_MDIO_ADDR_RDA_SHIFT            (16)      /* Bits 16-20: Register/device address */
#define ETH_MAC_MDIO_ADDR_RDA_MASK             (31 << ETH_MAC_MDIO_ADDR_RDA_SHIFT)
#  define ETH_MAC_MDIO_ADDR_RDA(n)             ((uint32_t)(n) << ETH_MAC_MDIO_ADDR_RDA_SHIFT)
#define ETH_MAC_MDIO_ADDR_PA_SHIFT             (21)      /* Bits  21-25: Physical layer address */
#define ETH_MAC_MDIO_ADDR_PA_MASK              (31 << ETH_MAC_MDIO_ADDR_PA_SHIFT)
#  define ETH_MAC_MDIO_ADDR_PA(n)              ((uint32_t)(n) << ETH_MAC_MDIO_ADDR_PA_SHIFT)
#define ETH_MAC_MDIO_ADDR_BTB                  (1 << 26) /* Bit 26  Back to back transactions */
#define ETH_MAC_MDIO_ADDR_PSE                  (1 << 27) /* Bit 27  Preamble suppression enable */

/* MDIO data */

#define ETH_MAC_MDIO_DATA_MASK                 0xffff    /* Bits 0-15: 16 bit PHY data */

/* MAC address0 high */

#define ETH_MAC_ADDR_HIGH_A32_47_SHIFT         (0)       /* MAC address 32-47 */
#define ETH_MAC_ADDR_HIGH_A32_47_MASK          (0xffff << ETH_MAC_ADDR_HIGH_A32_47_SHIFT)
#  define ETH_MAC_ADDR_HIGH_A32_47(n)          ((uint32_t)(n) << ETH_MAC_ADDR_HIGH_A32_47_SHIFT)
#define ETH_MAC_ADDR_HIGH_DCS                  (1 << 16) /* Bit 16: DMA channel select */

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
#define ETH_MAC_Tx_TIMESTAMP_STATUS_NSECS_
/* Tx timestamp status seconds */
#define ETH_MAC_Tx_TIMESTAMP_STATUS_SECS_
/* Timestamp ingress correction */
#define ETH_MAC_TIMESTAMP_INGRESS_CORR_NSEC_
/* Timestamp egress correction */
#define ETH_MAC_TIMESTAMP_EGRESS_CORR_NSEC_

/* MTL operation mode */
#define ETH_MTL_OP_MODE_
/* MTL interrupt status */
#define ETH_MTL_INTR_STAT_
/* MTL Rx Queue and DMA channel mapping */
#define ETH_MTL_RXQ_DMA_MAP_

/* MTL TxQn operation mode */

#define ETH_MTL_TXQ_OP_MODE_FTQ                (1 << 0)  /* Bit 0:  Flush Tx Queue */
#define ETH_MTL_TXQ_OP_MODE_TSF                (1 << 1)  /* Bit 1:  Transmit store and forward */
#define ETH_MTL_TXQ_OP_MODE_TXQEN_SHIFT        (2)       /* Bits 2-3: Tx Queue enable */
#define ETH_MTL_TXQ_OP_MODE_TXQEN_MASK         (3 << ETH_MTL_TXQ_OP_MODE_TXQEN_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TXQEN_ENABLE     (0 << ETH_MTL_TXQ_OP_MODE_TXQEN_SHIFT) /* Not enabled */
#  define ETH_MTL_TXQ_OP_MODE_TXQEN_DISABLE    (2 << ETH_MTL_TXQ_OP_MODE_TXQEN_SHIFT) /* Enabled */
#define ETH_MTL_TXQ_OP_MODE_TTC_SHIFT          (4)       /* Bits 4-6: Transmit threshold control */
#define ETH_MTL_TXQ_OP_MODE_TTC_MASK           (7 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_32           (0 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_64           (1 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_96           (2 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_128          (3 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_192          (4 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_256          (5 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_384          (6 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TTC_512          (7 << ETH_MTL_TXQ_OP_MODE_TTC_SHIFT)
#define ETH_MTL_TXQ_OP_MODE_TQS_SHIFT          (16)      /* Bits 16-18: Tx Queue size (x256) */
#define ETH_MTL_TXQ_OP_MODE_TQS_MASK           (7 << ETH_MTL_TXQ_OP_MODE_TQS_SHIFT)
#  define ETH_MTL_TXQ_OP_MODE_TQS(n)           ((uint32_t)((n)-1) << ETH_MTL_TXQ_OP_MODE_TQS_SHIFT)

/* MTL TxQn underflow */
#define ETH_MTL_TXQ_UNDRFLW_
/* MTL TxQn debug */
#define ETH_MTL_TXQ_DBG_
/* MTL TxQ1 (only) ETS control */
#define ETH_MTL_TXQ1_ETS_CTRL_
/* MTL TxQn ETS status */
#define ETH_MTL_TXQ_ETS_STAT_
/* Queue 0 quantum or weights */
#define ETH_MTL_TXQ_QNTM_WGHT_
/* MTL TxQ1 (only) SendSlopCredit */
#define ETH_MTL_TXQ1_SNDSLP_CRDT_
/* MTL TxQ1 (only) hiCredit */
#define ETH_MTL_TXQ1_HI_CRDT_
/* MTL TxQ1 (only) loCredit */
#define ETH_MTL_TXQ1_LO_CRDT_
/* MTL TxQn interrupt control status */
#define ETH_MTL_TXQ_INTCTRL_STAT_

/* MTL RxQn operation mode */

#define ETH_MTL_RXQ_OP_MODE_RTC_SHIFT          (0)       /* Bits 0-1:  Rx Queue threshold control */
#define ETH_MTL_RXQ_OP_MODE_RTC_MASK           (3 << ETH_MTL_RXQ_OP_MODE_RTC_SHIFT)
#  define ETH_MTL_RXQ_OP_MODE_RTC_64           (0 << ETH_MTL_RXQ_OP_MODE_RTC_SHIFT)
#  define ETH_MTL_RXQ_OP_MODE_RTC_32           (1 << ETH_MTL_RXQ_OP_MODE_RTC_SHIFT)
#  define ETH_MTL_RXQ_OP_MODE_RTC_96           (2 << ETH_MTL_RXQ_OP_MODE_RTC_SHIFT)
#  define ETH_MTL_RXQ_OP_MODE_RTC_128          (3 << ETH_MTL_RXQ_OP_MODE_RTC_SHIFT)
#define ETH_MTL_RXQ_OP_MODE_FUP                (1 << 3)  /* Bit 3  Forward undersized good packets */
#define ETH_MTL_RXQ_OP_MODE_FEP                (1 << 4)  /* Bit 4  Forward error packets */
#define ETH_MTL_RXQ_OP_MODE_RSF                (1 << 5)  /* Bit 5  Rx Queue store and forward */
#define ETH_MTL_RXQ_OP_MODE_DIS_TCP_EF         (1 << 6)  /* Bit 6  Disable dropping of TCP/IP checksum error packets */
#define ETH_MTL_RXQ_OP_MODE_RQS_SHIFT          (20)      /* Bits 20-22: Rx Queue size (x256) */
#define ETH_MTL_RXQ_OP_MODE_RQS_MASK           (7 << ETH_MTL_RXQ_OP_MODE_RQS_SHIFT)
#  define ETH_MTL_RXQ_OP_MODE_RQS(n)           ((uint32_t)((n)-1) << ETH_MTL_RXQ_OP_MODE_RQS_SHIFT)

/* MTL RxQn missed packet overflow counter */
#define ETH_MTL_RXQ_MISSPKT_OVRFLW_CNT_
/* MTL RxQn debug */
#define ETH_MTL_RXQ_DBG_
/* MTL RxQn control */
#define ETH_MTL_RXQ_CTRL_

/* DMA mode */

#define ETH_DMA_MODE_SWR                       (1 << 0)  /* Bit 0:  Software reset */
#define ETH_DMA_MODE_DA_MASK                   (1 << 1)  /* Bit 1:  DMA Tx or Rx arbitration scheme */
#  define ETH_DMA_MODE_DA_WRR                  (0)       /* Weighted round-robin with Rx:Tx or Tx:Rx */
#  define ETH_DMA_MODE_DA_FIXED                (1 << 1)  /* Fixed priority */
#define ETH_DMA_MODE_TAA_SHIFT                 (2)       /* Bits 2-4: Transmit arbitration algorithm */
#define ETH_DMA_MODE_TAA_MASK                  (7 << ETH_DMA_MODE_TAA_SHIFT)
#  define ETH_DMA_MODE_TAA_FIXED               (0 << ETH_DMA_MODE_TAA_SHIFT) /* Fixed priority */
#  define ETH_DMA_MODE_TAA_WSP                 (1 << ETH_DMA_MODE_TAA_SHIFT) /* Weighted strict priority */
#  define ETH_DMA_MODE_TAA_WRR                 (2 << ETH_DMA_MODE_TAA_SHIFT) /* Weighted round-robin */
#define ETH_DMA_MODE_TXPR                      (1 << 11) /* Bit 11: Transmit priority */
#define ETH_DMA_MODE_PR_SHIFT                  (12)      /* Bits 12-14: Priority ratio */
#define ETH_DMA_MODE_PR_MASK                   (7 << ETH_DMA_MODE_PR_SHIFT)
# define ETH_DMA_MODE_PR_1TO1                  (0 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 1:1 */
# define ETH_DMA_MODE_PR_3TO1                  (2 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 3:1 */
# define ETH_DMA_MODE_PR_4TO1                  (3 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 4:1 */
# define ETH_DMA_MODE_PR_5TO1                  (4 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 5:1 */
# define ETH_DMA_MODE_PR_6TO1                  (5 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 6:1 */
# define ETH_DMA_MODE_PR_7TO1                  (6 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 7:1 */
# define ETH_DMA_MODE_PR_8TO1                  (7 << ETH_DMA_MODE_PR_SHIFT) /* Priority ratio is 8:1 */

/* DMA system bus mode */

#define ETH_DMA_SYSBUS_MODE_FB                 (1 << 0)  /* Bit 0:  Fixed burst length */
#define ETH_DMA_SYSBUS_MODE_AAL                (1 << 12) /* Bit 12: Address-aligned beats */
#define ETH_DMA_SYSBUS_MODE_MB                 (1 << 14) /* Bit 14: Mixed burst */
#define ETH_DMA_SYSBUS_MODE_RB                 (1 << 15) /* Bit 15: Rebuild INCRx burst */

/* DMA interrupt status */

#define ETH_DMA_INTR_STAT_DC0IS                (1 << 0)  /* Bit 0:  DMA channel 0 interrupt status */
#define ETH_DMA_INTR_STAT_DC1IS                (1 << 1)  /* Bit 1:  DMA channel 1 interrupt status */
#define ETH_DMA_INTR_STAT_MTLIS                (1 << 16) /* Bit 16: MTL interrupt status */
#define ETH_DMA_INTR_STAT_MACIS                (1 << 17) /* Bit 17: MAC interrupt status */

/* DMA debug status */
#define ETH_DMA_DBG_STAT_

/* DMA channel n control */

#define ETH_DMACH_CTRL_PBLx8                   (1 << 16) /* Bit 16: 8xPBL mode */
#define ETH_DMACH_CTRL_DSL_SHIFT               (18)      /* Bits 18-20: Skip length */

/* DMA channel n transmit control */

#define ETH_DMACH_TX_CTRL_ST                   (1 << 0)  /* Bit 0:  Start or stop transmission command */
#define ETH_DMACH_TX_CTRL_TCW_SHIFT            (1)       /* Bits 1-3: Transmit channel weight */
#define ETH_DMACH_TX_CTRL_TCW_MASK             (7 << ETH_DMACH_TX_CTRL_TCW_SHIFT)
#  define ETH_DMACH_TX_CTRL_TCW(n)             ((uint32_t)(n) << ETH_DMACH_TX_CTRL_TCW_SHIFT)
#define ETH_DMACH_TX_CTRL_OSF                  (1 << 4)  /* Bit 4:  Operate on second frame */
#define ETH_DMACH_TX_CTRL_TxPBL_SHIFT          (16)      /* Bits 16-21: Transmit programmable burst length */
#define ETH_DMACH_TX_CTRL_TxPBL_MASK           (0x3f << ETH_DMACH_TX_CTRL_TxPBL_SHIFT)
#  define ETH_DMACH_TX_CTRL_TxPBL(n)           ((uint32_t)(n) << ETH_DMACH_TX_CTRL_TxPBL_SHIFT)

/* DMA channel n receive control */

#define ETH_DMACH_RX_CTRL_SR                   (1 << 0)  /* Bit 0:  Start or stop receive command */
#define ETH_DMACH_RX_CTRL_RBSZ_SHIFT           (3)       /* Bits 3-14: Receive buffer size */
#define ETH_DMACH_RX_CTRL_RBSZ_MASK            (0xfff << ETH_DMACH_RX_CTRL_RBSZ_SHIFT)
#  define ETH_DMACH_RX_CTRL(n)                 ((uint32_t)(n) << ETH_DMACH_RX_CTRL_RBSZ_SHIFT)
#define ETH_DMACH_RX_CTRL_RxPBL_SHIFT          (16)      /* Bits 16-21: Receive programmable burst length */
#define ETH_DMACH_RX_CTRL_RxPBL_MASK           (0x3f << ETH_DMACH_RX_CTRL_RxPBL_SHIFT)
#  define ETH_DMACH_RX_CTRL_RxPBL(n)           ((uint32_t)(n) << ETH_DMACH_RX_CTRL_RxPBL_SHIFT)
#define ETH_DMACH_RX_CTRL_RPF                  (1 << 31) /* Bit 31: DMA Rx channel n packet flush */

/* DMA channel n Tx descriptor list address */
#define ETH_DMACH_TXDESC_LIST_ADDR_
/* DMA channel n Rx descriptor list address */
#define ETH_DMACH_RXDESC_LIST_ADDR_
/* DMA channel n Tx descriptor tail pointer */
#define ETH_DMACH_TXDESC_TAIL_PTR_
/* DMA channel n Rx descriptor tail pointer */
#define ETH_DMACH_RXDESC_TAIL_PTR_
/* DMA channel n Tx descriptor ring length */
#define ETH_DMACH_TXDESC_RING_LENGTH_
/* DMA channel n Rx descriptor ring length */
#define ETH_DMACH_RXDESC_RING_LENGTH_
/* DMA channel n interrupt enable */
#define ETH_DMACH_INT_EN_
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
/* DMA channel n DMA status */
#define ETH_DMACH_STAT_
/* DMA channel n missed frame count */
#define ETH_DMACH_MISS_FRAME_CNT_

#endif /* __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_ETHERNET_H */
