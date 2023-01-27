/****************************************************************************
 * arch/arm/src/phy62xx/global_config.h
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
 *    @file global_config.h
 *
 *    @brief This file contains the definitions of index of

 *           global configuration which
 *           will be configured in APP project.
 *
 *    $Rev:  $
 *
 ****************************************************************************/

#ifndef _GLOBAL_CONFIG_H_
#define _GLOBAL_CONFIG_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "types.h"

#define CONFIG_BASE_ADDR 0x1fff0400

#define   SOFT_PARAMETER_NUM               256
/* parameter index of configuration array */
#define   ADV_CHANNEL_INTERVAL             0                /* interval between adv channel in the same adv event */
#define   SCAN_RSP_DELAY                   1                /* to adjust scan req -> scan rsp delay */
#define   CONN_REQ_TO_SLAVE_DELAY          2                /* to calibrate the delay between conn req & 1st slave conn event */
#define   SLAVE_CONN_DELAY                 3                /* to adjust the delay between 2 slave connection events */
#define   SLAVE_CONN_DELAY_BEFORE_SYNC     4                /* to adjust the delay between 2 slave connection events before 1st anchor is acquired */
#define   MAX_SLEEP_TIME                   5                /* maximum sleep time in us */
#define   MIN_SLEEP_TIME                   6                /* minimum sleep time in us */
#define   WAKEUP_ADVANCE                   7                /* wakeup advance time, to cover HW delay, crystal settle time, sw delay, ... etc. */
#define   WAKEUP_DELAY                     8                /* cycles of SW delay to wait crystal settle */

#define   HDC_DIRECT_ADV_INTERVAL          9
#define   LDC_DIRECT_ADV_INTERVAL          10

#define   LL_SWITCH                        11               /* Link Layer switch, 1 enable, 0 disable */
#define   NON_ADV_CHANNEL_INTERVAL         12               /* interval between non-adv channel in the same adv event */

#define   CLOCK_SETTING                    14               /* HCLK */
#define   LL_HW_BB_DELAY                   15
#define   LL_HW_AFE_DELAY                  16
#define   LL_HW_PLL_DELAY                  17

#define   LL_HW_RTLP_LOOP_TIMEOUT          18
#define   LL_HW_RTLP_1ST_TIMEOUT           19

#define   MIN_TIME_TO_STABLE_32KHZ_XOSC    20

#define   LL_TX_PKTS_PER_CONN_EVT          21
#define   LL_RX_PKTS_PER_CONN_EVT          22

/*  ============= A1 ROM metal change add */

#define   DIR_ADV_DELAY                    23
#define   LL_TX_PWR_TO_REG_BIAS            24
#define   LL_SMART_WINDOW_COEF_ALPHA       25
#define   LL_SMART_WINDOW_TARGET           26
#define   LL_SMART_WINDOW_INCREMENT        27
#define   LL_SMART_WINDOW_LIMIT            28
#define   LL_SMART_WINDOW_ACTIVE_THD       29
#define   LL_SMART_WINDOW_ACTIVE_RANGE     30
#define   LL_SMART_WINDOW_FIRST_WINDOW     31
#define   LL_HW_Tx_TO_RX_INTV              32
#define   LL_HW_Rx_TO_TX_INTV              33
#define   INITIAL_STACK_PTR                34
#define   ALLOW_TO_SLEEP_TICK_RC32K        35
#define   LL_HW_BB_DELAY_ADV               36
#define   LL_HW_AFE_DELAY_ADV              37
#define   LL_HW_PLL_DELAY_ADV              38

/* For scan & master, add 2018-6-15 */
#define   LL_ADV_TO_SCAN_REQ_DELAY         39
#define   LL_ADV_TO_CONN_REQ_DELAY         40
#define   LL_MOVE_TO_MASTER_DELAY          41
#define   LL_HW_TRLP_LOOP_TIMEOUT          42
#define   LL_CONN_REQ_WIN_SIZE             43
#define   LL_CONN_REQ_WIN_OFFSET           44
#define   LL_MASTER_PROCESS_TARGET         45
#define   LL_MASTER_TIRQ_DELAY             46

/* for PHY updated add 2018-11-07 */
#define   LL_HW_BB_DELAY_2MPHY             47
#define   LL_HW_AFE_DELAY_2MPHY            48
#define   LL_HW_PLL_DELAY_2MPHY            49
#define   LL_HW_Tx_TO_RX_INTV_2MPHY        50
#define   LL_HW_Rx_TO_TX_INTV_2MPHY        51
#define   LL_HW_BB_DELAY_500KPHY           52
#define   LL_HW_AFE_DELAY_500KPHY          53
#define   LL_HW_PLL_DELAY_500KPHY          54
#define   LL_HW_Tx_TO_RX_INTV_500KPHY      55
#define   LL_HW_Rx_TO_TX_INTV_500KPHY      56
#define   LL_HW_BB_DELAY_125KPHY           57
#define   LL_HW_AFE_DELAY_125KPHY          58
#define   LL_HW_PLL_DELAY_125KPHY          59
#define   LL_HW_Tx_TO_RX_INTV_125KPHY      60
#define   LL_HW_Rx_TO_TX_INTV_125KPHY      61
#define   LL_HW_TRLP_TO_GAP                62
#define   LL_HW_RTLP_TO_GAP                63
#define   LL_TRX_NUM_ADAPTIVE_CONFIG       64
#define   OSAL_SYS_TICK_WAKEUP_TRIM        65

/* ==== A2 add, for secondary adv/scan */
#define   LL_NOCONN_ADV_EST_TIME           70
#define   LL_NOCONN_ADV_MARGIN             71
#define   LL_SEC_SCAN_MARGIN               72
#define   LL_MIN_SCAN_TIME                 73
/* Bumblebee ROM code */
#define   LL_CONN_ADV_EST_TIME             74
#define   LL_SCANABLE_ADV_EST_TIME         75
#define   MAC_ADDRESS_LOC                  80

/* ==== For Extended Adv & Periodic adv */
#define   LL_EXT_ADV_INTER_PRI_CHN_INT     81
#define   LL_EXT_ADV_INTER_AUX_CHN_INT     82
#define   LL_EXT_ADV_RSC_POOL_PERIOD       83
#define   LL_EXT_ADV_RSC_POOL_UNIT         84

#define   LL_EXT_ADV_TASK_DURATION         86
#define   LL_PRD_ADV_TASK_DURATION         87
#define   LL_CONN_TASK_DURATION            88

#define   TIMER_ISR_ENTRY_TIME             90       /* time from HW timer expiry to ISR entry, unit: us */
#define   LL_MULTICONN_MASTER_PREEMP       91
#define   LL_MULTICONN_SLAVE_PREEMP        92
#define   LL_EXT_ADV_INTER_SEC_CHN_INT     93
#define   LL_EXT_ADV_PRI_2_SEC_CHN_INT     94
#define   LL_EXT_ADV_RSC_PERIOD            95
#define   LL_EXT_ADV_RSC_SLOT_DURATION     96
#define   LL_PRD_ADV_RSC_PERIOD            97
#define   LL_PRD_ADV_RSC_SLOT_DURATION     98
#define   LL_EXT_ADV_PROCESS_TARGET        99
#define   LL_PRD_ADV_PROCESS_TARGET        100

#define   RC32_TRACKINK_ALLOW              0x00000001       /* enable tracking RC 32KHz clock with 16MHz hclk */
#define   SLAVE_LATENCY_ALLOW              0x00000002       /* slave latency allow switch */
#define   LL_DEBUG_ALLOW                   0x00000004       /* enable invoke RAM project debug output function */
#define   LL_WHITELIST_ALLOW               0x00000008       /* enable whitelist filter */
#define   LL_RC32K_SEL                     0x00000010       /* select RC32K RTC, otherwise select crystal 32K RTC */
#define   SIMUL_CONN_ADV_ALLOW             0x00000020       /* allow send adv in connect state */
#define   SIMUL_CONN_SCAN_ALLOW            0x00000040       /* allow scan in connect state */

#define   GAP_DUP_RPT_FILTER_DISALLOW      0x00000100       /* duplicate report filter in GAP layer, allow default */

/* delete 2018-7-17, should use enum  H_SYSCLK_SEL */

/* enum
 * {
 *     CLOCK_16MHZ = 0,
 *     CLOCK_32MHZ = 1,
 *     CLOCK_48MHZ = 2,
 *     CLOCK_64MHZ = 3,
 *     CLOCK_96MHZ = 4,
 *     CLOCK_32MHZ_DBL=5
 * };
 */

/* extern uint32 global_config[SOFT_PARAMETER_NUM]; */

extern uint32 *pGlobal_config;                               /* note: app project needn't this variable */

#endif /* _GLOBAL_CONFIG_H_ */
