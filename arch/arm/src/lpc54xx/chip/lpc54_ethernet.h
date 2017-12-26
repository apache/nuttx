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

#define LPC54_ETH_MTL_TXQ0_OP_MODE_OFFSET                0x0d00  /* MTL TxQ0 operation mode */
#define LPC54_ETH_MTL_TXQ0_UNDRFLW_OFFSET                0x0d04  /* MTL TxQ0 underflow */
#define LPC54_ETH_MTL_TXQ0_DBG_OFFSET                    0x0d08  /* MTL TxQ0 debug */
#define LPC54_ETH_MTL_TXQ0_ETS_STAT_OFFSET               0x0d14  /* MTL TxQ0 ETS status */
#define LPC54_ETH_MTL_TXQ0_QNTM_WGHT_OFFSET              0x0d18  /* Queue 0 quantum or weights */
#define LPC54_ETH_MTL_TXQ0_INTCTRL_STAT_OFFSET           0x0d2c  /* MTL TxQ0 interrupt control status */
#define LPC54_ETH_MTL_RXQ0_OP_MODE_OFFSET                0x0d30  /* MTL RxQ0 operation mode */
#define LPC54_ETH_MTL_RXQ0_MISSPKT_OVRFLW_CNT_OFFSET     0x0d34  /* MTL RxQ0 missed packet overflow counter */
#define LPC54_ETH_MTL_RXQ0_DBG_OFFSET                    0x0d38  /* MTL RxQ0 debug */
#define LPC54_ETH_MTL_RXQ0_CTRL_OFFSET                   0x0d3c  /* MTL RxQ0 control */
#define LPC54_ETH_MTL_TXQ1_OP_MODE_OFFSET                0x0d40  /* MTL TxQ1 operation mode */
#define LPC54_ETH_MTL_TXQ1_UNDRFLW_OFFSET                0x0d44  /* MTL TxQ1 underflow */
#define LPC54_ETH_MTL_TXQ1_DBG_OFFSET                    0x0d48  /* MTL TxQ1 debug */
#define LPC54_ETH_MTL_TXQ1_ETS_CTRL_OFFSET               0x0d50  /* MTL TxQ1 ETS control */
#define LPC54_ETH_MTL_TXQ1_ETS_STAT_OFFSET               0x0d54  /* MTL TxQ1 ETS status */
#define LPC54_ETH_MTL_TXQ1_QNTM_WGHT_OFFSET              0x0d58  /* MTL TxQ1 quantum Weight */
#define LPC54_ETH_MTL_TXQ1_SNDSLP_CRDT_OFFSET            0x0d5c  /* MTL TxQ1 SendSlopCredit */
#define LPC54_ETH_MTL_TXQ1_HI_CRDT_OFFSET                0x0d60  /* MTL TxQ1 hiCredit */
#define LPC54_ETH_MTL_TXQ1_LO_CRDT_OFFSET                0x0d64  /* MTL TxQ1 loCredit */
#define LPC54_ETH_MTL_TXQ1_INTCTRL_STAT_OFFSET           0x0d6c  /* MTL TxQ1 interrupt control status */
#define LPC54_ETH_MTL_RXQ1_OP_MODE_OFFSET                0x0d70  /* MTL RxQ1 operation mode */
#define LPC54_ETH_MTL_RXQ1_MISSPKT_OVRFLW_CNT_OFFSET     0x0d74  /* MTL RxQ1 missed packet overflow counter */
#define LPC54_ETH_MTL_RXQ1_DBG_OFFSET                    0x0d78  /* MTL RxQ1 debug */
#define LPC54_ETH_MTL_RXQ1_CTRL_OFFSET                   0x0d7c  /* MTL RxQ1 control */

#define LPC54_ETH_DMA_MODE_OFFSET                        0x1000  /* DMA mode */
#define LPC54_ETH_DMA_SYSBUS_MODE_OFFSET                 0x1004  /* DMA system bus mode */
#define LPC54_ETH_DMA_INTR_STAT_OFFSET                   0x1008  /* DMA interrupt status */
#define LPC54_ETH_DMA_DBG_STAT_OFFSET                    0x100c  /* DMA debug status */

#define LPC54_ETH_DMA_CH0_CTRL_OFFSET                    0x1100  /* DMA channel 0 control */
#define LPC54_ETH_CH0_TX_CTRL_OFFSET                     0x1104  /* DMA channel 0 transmit control */
#define LPC54_ETH_DMA_CH0_RX_CTRL_OFFSET                 0x1108  /* DMA channel 0 receive control */
#define LPC54_ETH_DMA_CH0_TXDESC_LIST_ADDR_OFFSET        0x1114  /* Channel 0 Tx descriptor list address */
#define LPC54_ETH_DMA_CH0_RXDESC_LIST_ADDR_OFFSET        0x111c  /* Channel 0 Rx descriptor list address */
#define LPC54_ETH_DMA_CH0_TXDESC_TAIL_PTR_OFFSET         0x1120  /* Channel 0 Tx descriptor tail pointer */
#define LPC54_ETH_DMA_CH0_RXDESC_TAIL_PTR_OFFSET         0x1128  /* Channel 0 Rx descriptor tail pointer */
#define LPC54_ETH_DMA_CH0_TXDESC_RING_LENGTH_OFFSET      0x112c  /* Channel 0 Tx descriptor ring length */
#define LPC54_ETH_DMA_CH0_RXDESC_RING_LENGTH_OFFSET      0x1130  /* Channel 0 Rx descriptor ring length */
#define LPC54_ETH_DMA_CH0_INT_EN_OFFSET                  0x1134  /* Channel 0 interrupt enable */
#define LPC54_ETH_DMA_CH0_RX_INT_WDTIMER_OFFSET          0x1138  /* Receive interrupt watchdog timer */
#define LPC54_ETH_DMA_CH0_SLOT_FUNC_CTRL_STAT_OFFSET     0x113c  /** Slot function control and status */
#define LPC54_ETH_DMA_CH0_CUR_HST_TXDESC_OFFSET          0x1144  /* Channel 0 current host transmit descriptor */
#define LPC54_ETH_DMA_CH0_CUR_HST_RXDESC_OFFSET          0x114c  /** Channel 0 current host receive descriptor */
#define LPC54_ETH_DMA_CH0_CUR_HST_TXBUF_OFFSET           0x1154  /* Channel 0 current host transmit buffer address */
#define LPC54_ETH_DMA_CH0_CUR_HST_RXBUF_OFFSET           0x115c  /* Channel 0 current application receive buffer address */
#define LPC54_ETH_DMA_CH0_STAT_OFFSET                    0x1160  /* Channel 0 DMA status */
#define LPC54_ETH_DMA_CH0_MISS_FRAME_CNT_OFFSET          0x116c  /** Channel 0 missed frame count */
#define LPC54_ETH_DMA_CH1_CTRL_OFFSET                    0x1180  /* DMA channel 1 control */
#define LPC54_ETH_DMA_CH1_TX_CTRL_OFFSET                 0x1184  /* DMA channel 1 transmit control */
#define LPC54_ETH_DMA_CH1_RX_CTRL_OFFSET                 0x1188  /* The DMA channel 1 receive control */
#define LPC54_ETH_DMA_CH1_TXDESC_LIST_ADDR_OFFSET        0x1194  /* The channel 1 Tx descriptor list address */
#define LPC54_ETH_DMA_CH1_RXDESC_LIST_ADDR_OFFSET        0x119c  /* The channel 1 Rx descriptor list address */
#define LPC54_ETH_DMA_CH1_TXDESC_TAIL_PTR_OFFSET         0x11a0  /* The channel 1 Tx descriptor tail pointer */
#define LPC54_ETH_DMA_CH1_RXDESC_TAIL_PTR_OFFSET         0x11a8  /* The channel 1 Rx descriptor tail pointer */
#define LPC54_ETH_DMA_CH1_TXDESC_RING_LENGTH_OFFSET      0x11ac  /* Channel 1 Tx descriptor ring length */
#define LPC54_ETH_DMA_CH1_RXDESC_RING_LENGTH_OFFSET      0x11b0  /* The channel 1 Rx descriptor ring length */
#define LPC54_ETH_DMA_CH1_INT_EN_OFFSET                  0x11b4  /* The channel 1 interrupt enable */
#define LPC54_ETH_DMA_CH1_RX_INT_WDTIMER_OFFSET          0x11b8  /* The channel 1 receive interrupt watchdog timer */
#define LPC54_ETH_DMA_CH1_SLOT_FUNC_CTRL_STAT_OFFSET     0x11bc  /* The channel 1 slot function control and status */
#define LPC54_ETH_DMA_CH1_CUR_HST_TXDESC_OFFSET          0x11c4  /* The channel 1 current host transmit descriptor */
#define LPC54_ETH_DMA_CH1_CUR_HST_RXDESC_OFFSET          0x11cc  /* The channel 1 current host receive descriptor */
#define LPC54_ETH_DMA_CH1_CUR_HST_TXBUF_OFFSET           0x11d4  /* The channel 1 current host transmit buffer address */
#define LPC54_ETH_DMA_CH1_CUR_HST_RXBUF_OFFSET           0x11dc  /** The channel 1 current host receive buffer address */
#define LPC54_ETH_DMA_CH1_STAT_OFFSET                    0x11e0  /* Channel 1 DMA status */
#define LPC54_ETH_DMA_CH1_MISS_FRAME_CNT_OFFSET          0x11ec  /* Channel 1 missed frame count */

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

#define LPC54_ETH_MTL_TXQ0_OP_MODE                       (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ0_OP_MODE_OFFSET)
#define LPC54_ETH_MTL_TXQ0_UNDRFLW                       (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ0_UNDRFLW_OFFSET)
#define LPC54_ETH_MTL_TXQ0_DBG                           (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ0_DBG_OFFSET)
#define LPC54_ETH_MTL_TXQ0_ETS_STAT                      (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ0_ETS_STAT_OFFSET)
#define LPC54_ETH_MTL_TXQ0_QNTM_WGHT                     (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ0_QNTM_WGHT_OFFSET)
#define LPC54_ETH_MTL_TXQ0_INTCTRL_STAT                  (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ0_INTCTRL_STAT_OFFSET)
#define LPC54_ETH_MTL_RXQ0_OP_MODE                       (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_RXQ0_OP_MODE_OFFSET)
#define LPC54_ETH_MTL_RXQ0_MISSPKT_OVRFLW_CNT            (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_RXQ0_MISSPKT_OVRFLW_CNT_OFFSET)
#define LPC54_ETH_MTL_RXQ0_DBG                           (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_RXQ0_DBG_OFFSET)
#define LPC54_ETH_MTL_RXQ0_CTRL                          (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_RXQ0_CTRL_OFFSET)
#define LPC54_ETH_MTL_TXQ1_OP_MODE                       (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ1_OP_MODE_OFFSET)
#define LPC54_ETH_MTL_TXQ1_UNDRFLW                       (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ1_UNDRFLW_OFFSET)
#define LPC54_ETH_MTL_TXQ1_DBG                           (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ1_DBG_OFFSET)
#define LPC54_ETH_MTL_TXQ1_ETS_CTRL                      (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ1_ETS_CTRL_OFFSET)
#define LPC54_ETH_MTL_TXQ1_ETS_STAT                      (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ1_ETS_STAT_OFFSET)
#define LPC54_ETH_MTL_TXQ1_QNTM_WGHT                     (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ1_QNTM_WGHT_OFFSET)
#define LPC54_ETH_MTL_TXQ1_SNDSLP_CRDT                   (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ1_SNDSLP_CRDT_OFFSET)
#define LPC54_ETH_MTL_TXQ1_HI_CRDT                       (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ1_HI_CRDT_OFFSET)
#define LPC54_ETH_MTL_TXQ1_LO_CRDT                       (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ1_LO_CRDT_OFFSET)
#define LPC54_ETH_MTL_TXQ1_INTCTRL_STAT                  (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_TXQ1_INTCTRL_STAT_OFFSET)
#define LPC54_ETH_MTL_RXQ1_OP_MODE                       (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_RXQ1_OP_MODE_OFFSET)
#define LPC54_ETH_MTL_RXQ1_MISSPKT_OVRFLW_CNT            (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_RXQ1_MISSPKT_OVRFLW_CNT_OFFSET)
#define LPC54_ETH_MTL_RXQ1_DBG                           (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_RXQ1_DBG_OFFSET)
#define LPC54_ETH_MTL_RXQ1_CTRL                          (LPC54_ETHERNET_BASE + LPC54_ETH_MTL_RXQ1_CTRL_OFFSET)

#define LPC54_ETH_DMA_MODE                               (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_MODE_OFFSET)
#define LPC54_ETH_DMA_SYSBUS_MODE                        (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_SYSBUS_MODE_OFFSET)
#define LPC54_ETH_DMA_INTR_STAT                          (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_INTR_STAT_OFFSET)
#define LPC54_ETH_DMA_DBG_STAT                           (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_DBG_STAT_OFFSET)

#define LPC54_ETH_DMA_CH0_CTRL                           (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_CTRL_OFFSET)
#define LPC54_ETH_CH0_TX_CTRL                            (LPC54_ETHERNET_BASE + LPC54_ETH_CH0_TX_CTRL_OFFSET)
#define LPC54_ETH_DMA_CH0_RX_CTRL                        (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_RX_CTRL_OFFSET)
#define LPC54_ETH_DMA_CH0_TXDESC_LIST_ADDR               (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_TXDESC_LIST_ADDR_OFFSET)
#define LPC54_ETH_DMA_CH0_RXDESC_LIST_ADDR               (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_RXDESC_LIST_ADDR_OFFSET)
#define LPC54_ETH_DMA_CH0_TXDESC_TAIL_PTR                (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_TXDESC_TAIL_PTR_OFFSET)
#define LPC54_ETH_DMA_CH0_RXDESC_TAIL_PTR                (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_RXDESC_TAIL_PTR_OFFSET)
#define LPC54_ETH_DMA_CH0_TXDESC_RING_LENGTH             (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_TXDESC_RING_LENGTH_OFFSET)
#define LPC54_ETH_DMA_CH0_RXDESC_RING_LENGTH             (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_RXDESC_RING_LENGTH_OFFSET)
#define LPC54_ETH_DMA_CH0_INT_EN                         (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_INT_EN_OFFSET)
#define LPC54_ETH_DMA_CH0_RX_INT_WDTIMER                 (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_RX_INT_WDTIMER_OFFSET)
#define LPC54_ETH_DMA_CH0_SLOT_FUNC_CTRL_STAT            (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_SLOT_FUNC_CTRL_STAT_OFFSET)
#define LPC54_ETH_DMA_CH0_CUR_HST_TXDESC                 (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_CUR_HST_TXDESC_OFFSET)
#define LPC54_ETH_DMA_CH0_CUR_HST_RXDESC                 (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_CUR_HST_RXDESC_OFFSET)
#define LPC54_ETH_DMA_CH0_CUR_HST_TXBUF                  (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_CUR_HST_TXBUF_OFFSET)
#define LPC54_ETH_DMA_CH0_CUR_HST_RXBUF                  (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_CUR_HST_RXBUF_OFFSET)
#define LPC54_ETH_DMA_CH0_STAT                           (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_STAT_OFFSET)
#define LPC54_ETH_DMA_CH0_MISS_FRAME_CNT                 (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH0_MISS_FRAME_CNT_OFFSET)
#define LPC54_ETH_DMA_CH1_CTRL                           (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_CTRL_OFFSET)
#define LPC54_ETH_DMA_CH1_TX_CTRL                        (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_TX_CTRL_OFFSET)
#define LPC54_ETH_DMA_CH1_RX_CTRL                        (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_RX_CTRL_OFFSET)
#define LPC54_ETH_DMA_CH1_TXDESC_LIST_ADDR               (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_TXDESC_LIST_ADDR_OFFSET)
#define LPC54_ETH_DMA_CH1_RXDESC_LIST_ADDR               (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_RXDESC_LIST_ADDR_OFFSET)
#define LPC54_ETH_DMA_CH1_TXDESC_TAIL_PTR                (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_TXDESC_TAIL_PTR_OFFSET)
#define LPC54_ETH_DMA_CH1_RXDESC_TAIL_PTR                (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_RXDESC_TAIL_PTR_OFFSET)
#define LPC54_ETH_DMA_CH1_TXDESC_RING_LENGTH             (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_TXDESC_RING_LENGTH_OFFSET)
#define LPC54_ETH_DMA_CH1_RXDESC_RING_LENGTH             (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_RXDESC_RING_LENGTH_OFFSET)
#define LPC54_ETH_DMA_CH1_INT_EN                         (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_INT_EN_OFFSET)
#define LPC54_ETH_DMA_CH1_RX_INT_WDTIMER                 (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_RX_INT_WDTIMER_OFFSET)
#define LPC54_ETH_DMA_CH1_SLOT_FUNC_CTRL_STAT            (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_SLOT_FUNC_CTRL_STAT_OFFSET)
#define LPC54_ETH_DMA_CH1_CUR_HST_TXDESC                 (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_CUR_HST_TXDESC_OFFSET)
#define LPC54_ETH_DMA_CH1_CUR_HST_RXDESC                 (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_CUR_HST_RXDESC_OFFSET)
#define LPC54_ETH_DMA_CH1_CUR_HST_TXBUF                  (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_CUR_HST_TXBUF_OFFSET)
#define LPC54_ETH_DMA_CH1_CUR_HST_RXBUF                  (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_CUR_HST_RXBUF_OFFSET)
#define LPC54_ETH_DMA_CH1_STAT                           (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_STAT_OFFSET)
#define LPC54_ETH_DMA_CH1_MISS_FRAME_CNT                 (LPC54_ETHERNET_BASE + LPC54_ETH_DMA_CH1_MISS_FRAME_CNT_OFFSET)

/* Register bit definitions *********************************************************************************/

/* MAC configuration */
#define ETH_MAC_CONFIG_
/* MAC extended configuration */
#define ETH_MAC_EXT_CONFIG_
/* MAC frame filter */
#define ETH_MAC_FRAME_FILTER_
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
#define ETH_MAC_MDIO_ADDR_
/* MDIO data */
#define ETH_MAC_MDIO_DATA_
/* MAC address0 high */
#define ETH_MAC_ADDR_HIGH_
/* MAC address0 low */
#define ETH_MAC_ADDR_LOW_

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

/* MTL TxQ0 operation mode */
#define ETH_MTL_TXQ0_OP_MODE_
/* MTL TxQ0 underflow */
#define ETH_MTL_TXQ0_UNDRFLW_
/* MTL TxQ0 debug */
#define ETH_MTL_TXQ0_DBG_
/* MTL TxQ0 ETS status */
#define ETH_MTL_TXQ0_ETS_STAT_
/* Queue 0 quantum or weights */
#define ETH_MTL_TXQ0_QNTM_WGHT_
/* MTL TxQ0 interrupt control status */
#define ETH_MTL_TXQ0_INTCTRL_STAT_
/* MTL RxQ0 operation mode */
#define ETH_MTL_RXQ0_OP_MODE_
/* MTL RxQ0 missed packet overflow counter */
#define ETH_MTL_RXQ0_MISSPKT_OVRFLW_CNT_
/* MTL RxQ0 debug */
#define ETH_MTL_RXQ0_DBG_
/* MTL RxQ0 control */
#define ETH_MTL_RXQ0_CTRL_
/* MTL TxQ1 operation mode */
#define ETH_MTL_TXQ1_OP_MODE_
/* MTL TxQ1 underflow */
#define ETH_MTL_TXQ1_UNDRFLW_
/* MTL TxQ1 debug */
#define ETH_MTL_TXQ1_DBG_
/* MTL TxQ1 ETS control */
#define ETH_MTL_TXQ1_ETS_CTRL_
/* MTL TxQ1 ETS status */
#define ETH_MTL_TXQ1_ETS_STAT_
/* MTL TxQ1 quantum Weight */
#define ETH_MTL_TXQ1_QNTM_WGHT_
/* MTL TxQ1 SendSlopCredit */
#define ETH_MTL_TXQ1_SNDSLP_CRDT_
/* MTL TxQ1 hiCredit */
#define ETH_MTL_TXQ1_HI_CRDT_
/* MTL TxQ1 loCredit */
#define ETH_MTL_TXQ1_LO_CRDT_
/* MTL TxQ1 interrupt control status */
#define ETH_MTL_TXQ1_INTCTRL_STAT_
/* MTL RxQ1 operation mode */
#define ETH_MTL_RXQ1_OP_MODE_
/* MTL RxQ1 missed packet overflow counter */
#define ETH_MTL_RXQ1_MISSPKT_OVRFLW_CNT_
/* MTL RxQ1 debug */
#define ETH_MTL_RXQ1_DBG_
/* MTL RxQ1 control */
#define ETH_MTL_RXQ1_CTRL_

/* DMA mode */
#define ETH_DMA_MODE_
/* DMA system bus mode */
#define ETH_DMA_SYSBUS_MODE_
/* DMA interrupt status */
#define ETH_DMA_INTR_STAT_
/* DMA debug status */
#define ETH_DMA_DBG_STAT_

/* DMA channel 0 control */
#define ETH_DMA_CH0_CTRL_
/* DMA channel 0 transmit control */
#define ETH_CH0_TX_CTRL_
/* DMA channel 0 receive control */
#define ETH_DMA_CH0_RX_CTRL_
/* Channel 0 Tx descriptor list address */
#define ETH_DMA_CH0_TXDESC_LIST_ADDR_
/* Channel 0 Rx descriptor list address */
#define ETH_DMA_CH0_RXDESC_LIST_ADDR_
/* Channel 0 Tx descriptor tail pointer */
#define ETH_DMA_CH0_TXDESC_TAIL_PTR_
/* Channel 0 Rx descriptor tail pointer */
#define ETH_DMA_CH0_RXDESC_TAIL_PTR_
/* Channel 0 Tx descriptor ring length */
#define ETH_DMA_CH0_TXDESC_RING_LENGTH_
/* Channel 0 Rx descriptor ring length */
#define ETH_DMA_CH0_RXDESC_RING_LENGTH_
/* Channel 0 interrupt enable */
#define ETH_DMA_CH0_INT_EN_
/* Receive interrupt watchdog timer */
#define ETH_DMA_CH0_RX_INT_WDTIMER_
/* Slot function control and status */
#define ETH_DMA_CH0_SLOT_FUNC_CTRL_STAT_
/* Channel 0 current host transmit descriptor */
#define ETH_DMA_CH0_CUR_HST_TXDESC_
/* Channel 0 current host receive descriptor */
#define ETH_DMA_CH0_CUR_HST_RXDESC_
/* Channel 0 current host transmit buffer address */
#define ETH_DMA_CH0_CUR_HST_TXBUF_
/* Channel 0 current application receive buffer address */
#define ETH_DMA_CH0_CUR_HST_RXBUF_
/* Channel 0 DMA status */
#define ETH_DMA_CH0_STAT_
/** Channel 0 missed frame count */
#define ETH_DMA_CH0_MISS_FRAME_CNT_
/* DMA channel 1 control */
#define ETH_DMA_CH1_CTRL_
/* DMA channel 1 transmit control */
#define ETH_DMA_CH1_TX_CTRL_
/* The DMA channel 1 receive control */
#define ETH_DMA_CH1_RX_CTRL_
/* The channel 1 Tx descriptor list address */
#define ETH_DMA_CH1_TXDESC_LIST_ADDR_
/* The channel 1 Rx descriptor list address */
#define ETH_DMA_CH1_RXDESC_LIST_ADDR_
/* The channel 1 Tx descriptor tail pointer */
#define ETH_DMA_CH1_TXDESC_TAIL_PTR_
/* The channel 1 Rx descriptor tail pointer */
#define ETH_DMA_CH1_RXDESC_TAIL_PTR_
/* Channel 1 Tx descriptor ring length */
#define ETH_DMA_CH1_TXDESC_RING_LENGTH_
/* The channel 1 Rx descriptor ring length */
#define ETH_DMA_CH1_RXDESC_RING_LENGTH_
/* The channel 1 interrupt enable */
#define ETH_DMA_CH1_INT_EN_
/* The channel 1 receive interrupt watchdog timer */
#define ETH_DMA_CH1_RX_INT_WDTIMER_
/* The channel 1 slot function control and status */
#define ETH_DMA_CH1_SLOT_FUNC_CTRL_STAT_
/* The channel 1 current host transmit descriptor */
#define ETH_DMA_CH1_CUR_HST_TXDESC_
/* The channel 1 current host receive descriptor */
#define ETH_DMA_CH1_CUR_HST_RXDESC_
/* The channel 1 current host transmit buffer address */
#define ETH_DMA_CH1_CUR_HST_TXBUF_
/** The channel 1 current host receive buffer address */
#define ETH_DMA_CH1_CUR_HST_RXBUF_
/* Channel 1 DMA status */
#define ETH_DMA_CH1_STAT_
/* Channel 1 missed frame count */
#define ETH_DMA_CH1_MISS_FRAME_CNT_

#endif /* __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_ETHERNET_H */
