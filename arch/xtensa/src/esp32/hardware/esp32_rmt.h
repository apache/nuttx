/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_rmt.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_RMT_H
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_RMT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RMT Peripheral constants */
#define RMT_NUMBER_OF_CHANNELS      8
#define RMT_DATA_BASE_ADDR          0x3ff56800
#define RMT_DATA_MEMORY_BLOCK_WORDS 64

/* RMT Channel configuration registers */

#define RMT_CHNCONF_REG_BASE (DR_REG_RMT_BASE+0x20)
#define RMT_CHNCONF0_REG(n) (RMT_CHNCONF_REG_BASE + 8*n)
#define RMT_CHNCONF1_REG(n) (RMT_CHNCONF0_REG(n) + 4)

#define RMT_INT_RAW_REG (DR_REG_RMT_BASE+0x00A0)
#define RMT_INT_ST_REG  (DR_REG_RMT_BASE+0x00A4)
#define RMT_INT_ENA_REG (DR_REG_RMT_BASE+0x00A8)
#define RMT_INT_CLR_REG (DR_REG_RMT_BASE+0x00AC)

#define RMT_CHNCARRIER_DUTY_REG(n)  (DR_REG_RMT_BASE + 0x00B0+4*n)
#define RMT_CHN_TX_LIM_REG(n)       (DR_REG_RMT_BASE + 0x00D0+4*n)

#define RMT_APB_CONF_REG            (DR_REG_RMT_BASE + 0x00F0)

/* RMT_CHNCONF0_REG Bits */

/* RMT_MEM_PD: This bit is used to power down the entire RMT RAM block.
 * (It only exists in RMT_CH0CONF0).
 * 1: power down memory; 0: power up memory. (R/W)
 */

#define RMT_MEM_PD BIT(30)
#define RMT_MEM_PD_M (RMT_MEM_PD_V << RMT_MEM_PD_S)
#define RMT_MEM_PD_V 0x00000001
#define RMT_MEM_PD_S 30

/* RMT_CARRIER_OUT_LV_CHN This bit is used for configuration when the
 * carrier wave is being transmitted. Transmit on low output level with 0,
 * and transmit on high output level with 1. (R/W)
 */

#define RMT_CARRIER_OUT_LV_CHN BIT(29)
#define RMT_CARRIER_OUT_LV_CHN_M (RMT_CARRIER_OUT_LV_CHN_V << RMT_CARRIER_OUT_LV_CHN_S)
#define RMT_CARRIER_OUT_LV_CHN_V 0x00000001
#define RMT_CARRIER_OUT_LV_CHN_S 29

/* RMT_CARRIER_EN_CHN This is the carrier modulation enable-control bit
 * for channel n. Carrier modulation is enabled with 1, while carrier
 * modulation is disabled with 0. (R/W)
 */

#define RMT_CARRIER_EN_CHN BIT(28)
#define RMT_CARRIER_EN_CHN_M (RMT_CARRIER_EN_CHN_V << RMT_CARRIER_EN_CHN_S)
#define RMT_CARRIER_EN_CHN_V 0x00000001
#define RMT_CARRIER_EN_CHN_S 28

/* RMT_MEM_SIZE_CHN This register is used to configure the amount of
 * memory blocks allocated to channel n. (R/W)
 */

#define RMT_MEM_SIZE_CHN BIT(24)
#define RMT_MEM_SIZE_CHN_M (RMT_MEM_SIZE_CHN_V << RMT_MEM_SIZE_CHN_S)
#define RMT_MEM_SIZE_CHN_V 0x00000001
#define RMT_MEM_SIZE_CHN_S 24

/* RMT_IDLE_THRES_CHN In receive mode, when no edge is detected on
 * the input signal for longer than REG_IDLE_THRES_CHN channel clock cycles,
 * the receive process is finished. (R/W)
 */

#define RMT_IDLE_THRES_CHN BIT(8)
#define RMT_IDLE_THRES_CHN_M (RMT_IDLE_THRES_CHN_V << RMT_IDLE_THRES_CHN_S)
#define RMT_IDLE_THRES_CHN_V 0x00000001
#define RMT_IDLE_THRES_CHN_S 8

/* RMT_DIV_CNT_CHN This register is used to set the divider for the channel
 * clock of channel n. (R/W)
 */

#define RMT_DIV_CNT_CHN BIT(0)
#define RMT_DIV_CNT_CHN_M (RMT_DIV_CNT_CHN_V << RMT_DIV_CNT_CHN_S)
#define RMT_DIV_CNT_CHN_V 0x00000001
#define RMT_DIV_CNT_CHN_S 0

/* RMT_CHNCONF1_REG Bits */

/* RMT_IDLE_OUT_EN_CHN This is the output enable-control bit for channel n
 * in IDLE state. (R/W)
 */

#define RMT_IDLE_OUT_EN_CHN BIT(19)
#define RMT_IDLE_OUT_EN_CHN_M (RMT_IDLE_OUT_EN_CHN_V << RMT_IDLE_OUT_EN_CHN_S)
#define RMT_IDLE_OUT_EN_CHN_V 0x00000001
#define RMT_IDLE_OUT_EN_CHN_S 19

/* RMT_IDLE_OUT_LV_CHN This bit configures the level of output signals
 * in channel n when the latter is in IDLE state. (R/W)
 */

#define RMT_IDLE_OUT_LV_CHN BIT(18)
#define RMT_IDLE_OUT_LV_CHN_M (RMT_IDLE_OUT_LV_CHN_V << RMT_IDLE_OUT_LV_CHN_S)
#define RMT_IDLE_OUT_LV_CHN_V 0x00000001
#define RMT_IDLE_OUT_LV_CHN_S 18

/* RMT_REF_ALWAYS_ON_CHN This bit is used to select the channel's base
 * clock. 1:clk_apb; 0:clk_ref. (R/W)
 */

#define RMT_REF_ALWAYS_ON_CHN BIT(17)
#define RMT_REF_ALWAYS_ON_CHN_M (RMT_REF_ALWAYS_ON_CHN_V << RMT_REF_ALWAYS_ON_CHN_S)
#define RMT_REF_ALWAYS_ON_CHN_V 0x00000001
#define RMT_REF_ALWAYS_ON_CHN_S 17

/* RMT_REF_CNT_RST_CHN Setting this bit resets the clock divider of channel
 * n. (R/W)
 */

#define RMT_REF_CNT_RST_CHN BIT(16)
#define RMT_REF_CNT_RST_CHN_M (RMT_REF_CNT_RST_CHN_V << RMT_REF_CNT_RST_CHN_S)
#define RMT_REF_CNT_RST_CHN_V 0x00000001
#define RMT_REF_CNT_RST_CHN_S 16

/* RMT_RX_FILTER_THRES_CHN In receive mode, channel n ignores input
 * pulse when the pulse width is smaller than this value in APB clock
 * periods. (R/W)
 */

#define RMT_RX_FILTER_THRES_CHN BIT(8)
#define RMT_RX_FILTER_THRES_CHN_M (RMT_RX_FILTER_THRES_CHN_V << RMT_RX_FILTER_THRES_CHN_S)
#define RMT_RX_FILTER_THRES_CHN_V 0x00000001
#define RMT_RX_FILTER_THRES_CHN_S 8

/* RMT_RX_FILTER_EN_CHN This is the receive filter's enable-bit for channel
 * n. (R/W)
 */

#define RMT_RX_FILTER_EN_CHN BIT(7)
#define RMT_RX_FILTER_EN_CHN_M (RMT_RX_FILTER_EN_CHN_V << RMT_RX_FILTER_EN_CHN_S)
#define RMT_RX_FILTER_EN_CHN_V 0x00000001
#define RMT_RX_FILTER_EN_CHN_S 7

/* RMT_TX_CONTI_MODE_CHN If this bit is set, instead of going to an idle
 * state when transmission ends, the transmitter will restart transmission.
 * This results in a repeating output signal. (R/W)
 */

#define RMT_TX_CONTI_MODE_CHN BIT(6)
#define RMT_TX_CONTI_MODE_CHN_M (RMT_TX_CONTI_MODE_CHN_V << RMT_TX_CONTI_MODE_CHN_S)
#define RMT_TX_CONTI_MODE_CHN_V 0x00000001
#define RMT_TX_CONTI_MODE_CHN_S 6

/* RMT_MEM_OWNER_CHN This bit marks channel n's RAM block ownership.
 * Number 1 indicates that the receiver is using the RAM, while 0 indicates
 * that the transmitter is using the RAM. (R/W)
 */

#define RMT_MEM_OWNER_CHN BIT(5)
#define RMT_MEM_OWNER_CHN_M (RMT_MEM_OWNER_CHN_V << RMT_MEM_OWNER_CHN_S)
#define RMT_MEM_OWNER_CHN_V 0x00000001
#define RMT_MEM_OWNER_CHN_S 5

/* RMT_MEM_RD_RST_CHN Set this bit to reset the read-RAM address for channel
 * n by accessing the transmitter. (R/W)
 */

#define RMT_MEM_RD_RST_CHN BIT(3)
#define RMT_MEM_RD_RST_CHN_M (RMT_MEM_RD_RST_CHN_V << RMT_MEM_RD_RST_CHN_S)
#define RMT_MEM_RD_RST_CHN_V 0x00000001
#define RMT_MEM_RD_RST_CHN_S 3

/* RMT_MEM_WR_RST_CHN Set this bit to reset the write-RAM address for
 * channel n by accessing the receiver. (R/W)
 */

#define RMT_MEM_WR_RST_CHN BIT(2)
#define RMT_MEM_WR_RST_CHN_M (RMT_MEM_WR_RST_CHN_V << RMT_MEM_WR_RST_CHN_S)
#define RMT_MEM_WR_RST_CHN_V 0x00000001
#define RMT_MEM_WR_RST_CHN_S 2

/* RMT_RX_EN_CHN Set this bit to enable receiving data on channel n. (R/W) */

#define RMT_RX_EN_CHN BIT(1)
#define RMT_RX_EN_CHN_M (RMT_RX_EN_CHN_V << RMT_RX_EN_CHN_S)
#define RMT_RX_EN_CHN_V 0x00000001
#define RMT_RX_EN_CHN_S 1

/* RMT_TX_START_CHN Set this bit to start sending data on channel n. (R/W) */

#define RMT_TX_START_CHN(n) BIT(n)
#define RMT_TX_START_CHN_M (RMT_TX_START_CHN_V << RMT_TX_START_CHN_S)
#define RMT_TX_START_CHN_V 0x00000001
#define RMT_TX_START_CHN_S 0

/* RMT_INT_RAW_REG Bits */

/* RMT_CHN_TX_THR_EVENT_INT_RAW The raw interrupt status bit for the
 * RMT_CHN_TX_THR_EVENT_INT interrupt. (RO)
 */

#define RMT_CHN_TX_THR_EVENT_INT_RAW(n) BIT(24+n)

/* RMT_CHN_ERR_INT_RAW The raw interrupt status bit for the RMT_CHN_ERR_INT
 * interrupt. (RO)
 */

#define RMT_CHN_ERR_INT_RAW(n) BIT(3*n+2)

/* RMT_CHN_RX_END_INT_RAW The raw interrupt status bit for
 * the RMT_CHN_RX_END_INT interrupt. (RO)
 */
#define RMT_CHN_RX_END_INT_RAW(n) BIT(3*n+1)

/* RMT_CHN_TX_END_INT_RAW The raw interrupt status bit for the
 * RMT_CHN_TX_END_INT interrupt. (RO)
 */
#define RMT_CHN_TX_END_INT_RAW(n) BIT(3*n)

/* RMT_INT_ST_REG Bits */
#define RMT_CHN_TX_THR_EVENT_INT_ST(n) BIT(24+n)
#define RMT_CHN_ERR_INT_ST(n) BIT(3*n+2)
#define RMT_CHN_RX_END_INT_ST(n) BIT(3*n+1)
#define RMT_CHN_TX_END_INT_ST(n) BIT(3*n)

/* RMT_INT_ENA_REG Bits */
#define RMT_CHN_TX_THR_EVENT_INT_ENA(n) BIT(24+n)
#define RMT_CHN_ERR_INT_ENA(n) BIT(3*n+2)
#define RMT_CHN_RX_END_INT_ENA(n) BIT(3*n+1)
#define RMT_CHN_TX_END_INT_ENA(n) BIT(3*n)

/* RMT_INT_CLR_REG Bits */
#define RMT_CHN_TX_THR_EVENT_INT_CLR(n) BIT(24+n)
#define RMT_CHN_ERR_INT_CLR(n) BIT(3*n+2)
#define RMT_CHN_RX_END_INT_CLR(n) BIT(3*n+1)
#define RMT_CHN_TX_END_INT_CLR(n) BIT(3*n)

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_RMT_H */
