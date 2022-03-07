/****************************************************************************
 * arch/xtensa/src/esp32s2/hardware/esp32s2_rtccntl.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_RTCCNTL_H
#define __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_RTCCNTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32s2_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Offset relative to each watchdog timer instance memory base */

#define RWDT_CONFIG0_OFFSET         0x0094

/* RWDT */

#define RWDT_STAGE0_TIMEOUT_OFFSET  0x0098
#define RWDT_STAGE1_TIMEOUT_OFFSET  0x009c
#define RWDT_STAGE2_TIMEOUT_OFFSET  0x00a0
#define RWDT_STAGE3_TIMEOUT_OFFSET  0x00a4
#define RWDT_FEED_OFFSET            0x00a8
#define RWDT_WP_REG                 0x00ac
#define RWDT_INT_ENA_REG_OFFSET     0x0040
#define RWDT_INT_CLR_REG_OFFSET     0x004c

/* The value that needs to be written to RTC_CNTL_WDT_WKEY to
 * write-enable the wdt registers
 */

#define RTC_CNTL_WDT_WKEY_VALUE     0x50d83aa1

/* The value that needs to be written to RTC_CNTL_SWD_WPROTECT_REG
 * to write-enable the wdt registers
 */

#define RTC_CNTL_SWD_WKEY_VALUE     0x8f1d312a

#define DPORT_CPUPERIOD_SEL_80          0
#define DPORT_CPUPERIOD_SEL_160         1
#define DPORT_CPUPERIOD_SEL_240         2

#define RTC_APB_FREQ_REG            RTC_CNTL_STORE5_REG

/* RTC_CNTL_OPTIONS0_REG register
 * set xtal and pll power and  sw reset register
 */

#define RTC_CNTL_OPTIONS0_REG (DR_REG_RTCCNTL_BASE + 0x0)

/* RTC_CNTL_SW_SYS_RST : WO; bitpos: [31]; default: 0;
 * SW system reset
 */

#define RTC_CNTL_SW_SYS_RST    (BIT(31))
#define RTC_CNTL_SW_SYS_RST_M  (RTC_CNTL_SW_SYS_RST_V << RTC_CNTL_SW_SYS_RST_S)
#define RTC_CNTL_SW_SYS_RST_V  0x00000001
#define RTC_CNTL_SW_SYS_RST_S  31

/* RTC_CNTL_DG_WRAP_FORCE_NORST : R/W; bitpos: [30]; default: 0;
 * digital core force no reset in deep sleep
 */

#define RTC_CNTL_DG_WRAP_FORCE_NORST    (BIT(30))
#define RTC_CNTL_DG_WRAP_FORCE_NORST_M  (RTC_CNTL_DG_WRAP_FORCE_NORST_V << RTC_CNTL_DG_WRAP_FORCE_NORST_S)
#define RTC_CNTL_DG_WRAP_FORCE_NORST_V  0x00000001
#define RTC_CNTL_DG_WRAP_FORCE_NORST_S  30

/* RTC_CNTL_DG_WRAP_FORCE_RST : R/W; bitpos: [29]; default: 0;
 * digital wrap force reset in deep sleep
 */

#define RTC_CNTL_DG_WRAP_FORCE_RST    (BIT(29))
#define RTC_CNTL_DG_WRAP_FORCE_RST_M  (RTC_CNTL_DG_WRAP_FORCE_RST_V << RTC_CNTL_DG_WRAP_FORCE_RST_S)
#define RTC_CNTL_DG_WRAP_FORCE_RST_V  0x00000001
#define RTC_CNTL_DG_WRAP_FORCE_RST_S  29

/* RTC_CNTL_ANALOG_FORCE_NOISO : R/W; bitpos: [28]; default: 1; */

#define RTC_CNTL_ANALOG_FORCE_NOISO    (BIT(28))
#define RTC_CNTL_ANALOG_FORCE_NOISO_M  (RTC_CNTL_ANALOG_FORCE_NOISO_V << RTC_CNTL_ANALOG_FORCE_NOISO_S)
#define RTC_CNTL_ANALOG_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_ANALOG_FORCE_NOISO_S  28

/* RTC_CNTL_PLL_FORCE_NOISO : R/W; bitpos: [27]; default: 1; */

#define RTC_CNTL_PLL_FORCE_NOISO    (BIT(27))
#define RTC_CNTL_PLL_FORCE_NOISO_M  (RTC_CNTL_PLL_FORCE_NOISO_V << RTC_CNTL_PLL_FORCE_NOISO_S)
#define RTC_CNTL_PLL_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_PLL_FORCE_NOISO_S  27

/* RTC_CNTL_XTL_FORCE_NOISO : R/W; bitpos: [26]; default: 1; */

#define RTC_CNTL_XTL_FORCE_NOISO    (BIT(26))
#define RTC_CNTL_XTL_FORCE_NOISO_M  (RTC_CNTL_XTL_FORCE_NOISO_V << RTC_CNTL_XTL_FORCE_NOISO_S)
#define RTC_CNTL_XTL_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_XTL_FORCE_NOISO_S  26

/* RTC_CNTL_ANALOG_FORCE_ISO : R/W; bitpos: [25]; default: 0; */

#define RTC_CNTL_ANALOG_FORCE_ISO    (BIT(25))
#define RTC_CNTL_ANALOG_FORCE_ISO_M  (RTC_CNTL_ANALOG_FORCE_ISO_V << RTC_CNTL_ANALOG_FORCE_ISO_S)
#define RTC_CNTL_ANALOG_FORCE_ISO_V  0x00000001
#define RTC_CNTL_ANALOG_FORCE_ISO_S  25

/* RTC_CNTL_PLL_FORCE_ISO : R/W; bitpos: [24]; default: 0; */

#define RTC_CNTL_PLL_FORCE_ISO    (BIT(24))
#define RTC_CNTL_PLL_FORCE_ISO_M  (RTC_CNTL_PLL_FORCE_ISO_V << RTC_CNTL_PLL_FORCE_ISO_S)
#define RTC_CNTL_PLL_FORCE_ISO_V  0x00000001
#define RTC_CNTL_PLL_FORCE_ISO_S  24

/* RTC_CNTL_XTL_FORCE_ISO : R/W; bitpos: [23]; default: 0; */

#define RTC_CNTL_XTL_FORCE_ISO    (BIT(23))
#define RTC_CNTL_XTL_FORCE_ISO_M  (RTC_CNTL_XTL_FORCE_ISO_V << RTC_CNTL_XTL_FORCE_ISO_S)
#define RTC_CNTL_XTL_FORCE_ISO_V  0x00000001
#define RTC_CNTL_XTL_FORCE_ISO_S  23

/* RTC_CNTL_XTL_FORCE_PU : R/W; bitpos: [13]; default: 1;
 * crystall force power up
 */

#define RTC_CNTL_XTL_FORCE_PU    (BIT(13))
#define RTC_CNTL_XTL_FORCE_PU_M  (RTC_CNTL_XTL_FORCE_PU_V << RTC_CNTL_XTL_FORCE_PU_S)
#define RTC_CNTL_XTL_FORCE_PU_V  0x00000001
#define RTC_CNTL_XTL_FORCE_PU_S  13

/* RTC_CNTL_XTL_FORCE_PD : R/W; bitpos: [12]; default: 0;
 * crystall force power down
 */

#define RTC_CNTL_XTL_FORCE_PD    (BIT(12))
#define RTC_CNTL_XTL_FORCE_PD_M  (RTC_CNTL_XTL_FORCE_PD_V << RTC_CNTL_XTL_FORCE_PD_S)
#define RTC_CNTL_XTL_FORCE_PD_V  0x00000001
#define RTC_CNTL_XTL_FORCE_PD_S  12

/* RTC_CNTL_BBPLL_FORCE_PU : R/W; bitpos: [11]; default: 0;
 * BB_PLL force power up
 */

#define RTC_CNTL_BBPLL_FORCE_PU    (BIT(11))
#define RTC_CNTL_BBPLL_FORCE_PU_M  (RTC_CNTL_BBPLL_FORCE_PU_V << RTC_CNTL_BBPLL_FORCE_PU_S)
#define RTC_CNTL_BBPLL_FORCE_PU_V  0x00000001
#define RTC_CNTL_BBPLL_FORCE_PU_S  11

/* RTC_CNTL_BBPLL_FORCE_PD : R/W; bitpos: [10]; default: 0;
 * BB_PLL force power down
 */

#define RTC_CNTL_BBPLL_FORCE_PD    (BIT(10))
#define RTC_CNTL_BBPLL_FORCE_PD_M  (RTC_CNTL_BBPLL_FORCE_PD_V << RTC_CNTL_BBPLL_FORCE_PD_S)
#define RTC_CNTL_BBPLL_FORCE_PD_V  0x00000001
#define RTC_CNTL_BBPLL_FORCE_PD_S  10

/* RTC_CNTL_BBPLL_I2C_FORCE_PU : R/W; bitpos: [9]; default: 0;
 * BB_PLL_I2C force power up
 */

#define RTC_CNTL_BBPLL_I2C_FORCE_PU    (BIT(9))
#define RTC_CNTL_BBPLL_I2C_FORCE_PU_M  (RTC_CNTL_BBPLL_I2C_FORCE_PU_V << RTC_CNTL_BBPLL_I2C_FORCE_PU_S)
#define RTC_CNTL_BBPLL_I2C_FORCE_PU_V  0x00000001
#define RTC_CNTL_BBPLL_I2C_FORCE_PU_S  9

/* RTC_CNTL_BBPLL_I2C_FORCE_PD : R/W; bitpos: [8]; default: 0;
 * BB_PLL _I2C force power down
 */

#define RTC_CNTL_BBPLL_I2C_FORCE_PD    (BIT(8))
#define RTC_CNTL_BBPLL_I2C_FORCE_PD_M  (RTC_CNTL_BBPLL_I2C_FORCE_PD_V << RTC_CNTL_BBPLL_I2C_FORCE_PD_S)
#define RTC_CNTL_BBPLL_I2C_FORCE_PD_V  0x00000001
#define RTC_CNTL_BBPLL_I2C_FORCE_PD_S  8

/* RTC_CNTL_BB_I2C_FORCE_PU : R/W; bitpos: [7]; default: 0;
 * BB_I2C force power up
 */

#define RTC_CNTL_BB_I2C_FORCE_PU    (BIT(7))
#define RTC_CNTL_BB_I2C_FORCE_PU_M  (RTC_CNTL_BB_I2C_FORCE_PU_V << RTC_CNTL_BB_I2C_FORCE_PU_S)
#define RTC_CNTL_BB_I2C_FORCE_PU_V  0x00000001
#define RTC_CNTL_BB_I2C_FORCE_PU_S  7

/* RTC_CNTL_BB_I2C_FORCE_PD : R/W; bitpos: [6]; default: 0;
 * BB_I2C force power down
 */

#define RTC_CNTL_BB_I2C_FORCE_PD    (BIT(6))
#define RTC_CNTL_BB_I2C_FORCE_PD_M  (RTC_CNTL_BB_I2C_FORCE_PD_V << RTC_CNTL_BB_I2C_FORCE_PD_S)
#define RTC_CNTL_BB_I2C_FORCE_PD_V  0x00000001
#define RTC_CNTL_BB_I2C_FORCE_PD_S  6

/* RTC_CNTL_SW_PROCPU_RST : WO; bitpos: [5]; default: 0;
 * PRO CPU SW reset
 */

#define RTC_CNTL_SW_PROCPU_RST    (BIT(5))
#define RTC_CNTL_SW_PROCPU_RST_M  (RTC_CNTL_SW_PROCPU_RST_V << RTC_CNTL_SW_PROCPU_RST_S)
#define RTC_CNTL_SW_PROCPU_RST_V  0x00000001
#define RTC_CNTL_SW_PROCPU_RST_S  5

/* RTC_CNTL_SW_APPCPU_RST : WO; bitpos: [4]; default: 0;
 * APP CPU SW reset
 */

#define RTC_CNTL_SW_APPCPU_RST    (BIT(4))
#define RTC_CNTL_SW_APPCPU_RST_M  (RTC_CNTL_SW_APPCPU_RST_V << RTC_CNTL_SW_APPCPU_RST_S)
#define RTC_CNTL_SW_APPCPU_RST_V  0x00000001
#define RTC_CNTL_SW_APPCPU_RST_S  4

/* RTC_CNTL_SW_STALL_PROCPU_C0 : R/W; bitpos: [3:2]; default: 0;
 * {reg_sw_stall_procpu_c1[5:0] , reg_sw_stall_procpu_c0[1:0]} == 0x86 will
 * stall PRO CPU
 */

#define RTC_CNTL_SW_STALL_PROCPU_C0    0x00000003
#define RTC_CNTL_SW_STALL_PROCPU_C0_M  (RTC_CNTL_SW_STALL_PROCPU_C0_V << RTC_CNTL_SW_STALL_PROCPU_C0_S)
#define RTC_CNTL_SW_STALL_PROCPU_C0_V  0x00000003
#define RTC_CNTL_SW_STALL_PROCPU_C0_S  2

/* RTC_CNTL_SW_STALL_APPCPU_C0 : R/W; bitpos: [1:0]; default: 0;
 * {reg_sw_stall_appcpu_c1[5:0] , reg_sw_stall_appcpu_c0[1:0]} == 0x86 will
 * stall APP CPU
 */

#define RTC_CNTL_SW_STALL_APPCPU_C0    0x00000003
#define RTC_CNTL_SW_STALL_APPCPU_C0_M  (RTC_CNTL_SW_STALL_APPCPU_C0_V << RTC_CNTL_SW_STALL_APPCPU_C0_S)
#define RTC_CNTL_SW_STALL_APPCPU_C0_V  0x00000003
#define RTC_CNTL_SW_STALL_APPCPU_C0_S  0

/* RTC_CNTL_SLP_TIMER0_REG register
 * rtc_sleep_timer0 register
 */

#define RTC_CNTL_SLP_TIMER0_REG (DR_REG_RTCCNTL_BASE + 0x4)

/* RTC_CNTL_SLP_VAL_LO : R/W; bitpos: [31:0]; default: 0;
 * RTC sleep timer low 32 bits
 */

#define RTC_CNTL_SLP_VAL_LO    0xFFFFFFFF
#define RTC_CNTL_SLP_VAL_LO_M  (RTC_CNTL_SLP_VAL_LO_V << RTC_CNTL_SLP_VAL_LO_S)
#define RTC_CNTL_SLP_VAL_LO_V  0xFFFFFFFF
#define RTC_CNTL_SLP_VAL_LO_S  0

/* RTC_CNTL_SLP_TIMER1_REG register
 * rtc_sleep_timer1 register
 */

#define RTC_CNTL_SLP_TIMER1_REG (DR_REG_RTCCNTL_BASE + 0x8)

/* RTC_CNTL_MAIN_TIMER_ALARM_EN : WO; bitpos: [16]; default: 0;
 * timer alarm enable bit
 */

#define RTC_CNTL_MAIN_TIMER_ALARM_EN    (BIT(16))
#define RTC_CNTL_MAIN_TIMER_ALARM_EN_M  (RTC_CNTL_MAIN_TIMER_ALARM_EN_V << RTC_CNTL_MAIN_TIMER_ALARM_EN_S)
#define RTC_CNTL_MAIN_TIMER_ALARM_EN_V  0x00000001
#define RTC_CNTL_MAIN_TIMER_ALARM_EN_S  16

/* RTC_CNTL_SLP_VAL_HI : R/W; bitpos: [15:0]; default: 0;
 * RTC sleep timer high 16 bits
 */

#define RTC_CNTL_SLP_VAL_HI    0x0000FFFF
#define RTC_CNTL_SLP_VAL_HI_M  (RTC_CNTL_SLP_VAL_HI_V << RTC_CNTL_SLP_VAL_HI_S)
#define RTC_CNTL_SLP_VAL_HI_V  0x0000FFFF
#define RTC_CNTL_SLP_VAL_HI_S  0

/* RTC_CNTL_TIME_UPDATE_REG register
 * rtc time update register
 */

#define RTC_CNTL_TIME_UPDATE_REG (DR_REG_RTCCNTL_BASE + 0xc)

/* RTC_CNTL_TIME_UPDATE : WO; bitpos: [31]; default: 0;
 * Set 1: to update register with RTC timer
 */

#define RTC_CNTL_TIME_UPDATE    (BIT(31))
#define RTC_CNTL_TIME_UPDATE_M  (RTC_CNTL_TIME_UPDATE_V << RTC_CNTL_TIME_UPDATE_S)
#define RTC_CNTL_TIME_UPDATE_V  0x00000001
#define RTC_CNTL_TIME_UPDATE_S  31

/* RTC_CNTL_TIMER_SYS_RST : R/W; bitpos: [29]; default: 0;
 * enable to record system reset time
 */

#define RTC_CNTL_TIMER_SYS_RST    (BIT(29))
#define RTC_CNTL_TIMER_SYS_RST_M  (RTC_CNTL_TIMER_SYS_RST_V << RTC_CNTL_TIMER_SYS_RST_S)
#define RTC_CNTL_TIMER_SYS_RST_V  0x00000001
#define RTC_CNTL_TIMER_SYS_RST_S  29

/* RTC_CNTL_TIMER_XTL_OFF : R/W; bitpos: [28]; default: 0;
 * Enable to record 40M XTAL OFF time
 */

#define RTC_CNTL_TIMER_XTL_OFF    (BIT(28))
#define RTC_CNTL_TIMER_XTL_OFF_M  (RTC_CNTL_TIMER_XTL_OFF_V << RTC_CNTL_TIMER_XTL_OFF_S)
#define RTC_CNTL_TIMER_XTL_OFF_V  0x00000001
#define RTC_CNTL_TIMER_XTL_OFF_S  28

/* RTC_CNTL_TIMER_SYS_STALL : R/W; bitpos: [27]; default: 0;
 * Enable to record system stall time
 */

#define RTC_CNTL_TIMER_SYS_STALL    (BIT(27))
#define RTC_CNTL_TIMER_SYS_STALL_M  (RTC_CNTL_TIMER_SYS_STALL_V << RTC_CNTL_TIMER_SYS_STALL_S)
#define RTC_CNTL_TIMER_SYS_STALL_V  0x00000001
#define RTC_CNTL_TIMER_SYS_STALL_S  27

/* RTC_CNTL_TIME_LOW0_REG register
 * RTC timer0 low 32 bits
 */

#define RTC_CNTL_TIME_LOW0_REG (DR_REG_RTCCNTL_BASE + 0x10)

/* RTC_CNTL_TIMER_VALUE0_LOW : RO; bitpos: [31:0]; default: 0;
 * RTC timer low 32 bits
 */

#define RTC_CNTL_TIMER_VALUE0_LOW    0xFFFFFFFF
#define RTC_CNTL_TIMER_VALUE0_LOW_M  (RTC_CNTL_TIMER_VALUE0_LOW_V << RTC_CNTL_TIMER_VALUE0_LOW_S)
#define RTC_CNTL_TIMER_VALUE0_LOW_V  0xFFFFFFFF
#define RTC_CNTL_TIMER_VALUE0_LOW_S  0

/* RTC_CNTL_TIME_HIGH0_REG register
 * RTC timer0 high16 bits
 */

#define RTC_CNTL_TIME_HIGH0_REG (DR_REG_RTCCNTL_BASE + 0x14)

/* RTC_CNTL_TIMER_VALUE0_HIGH : RO; bitpos: [15:0]; default: 0;
 * RTC timer high 16 bits
 */

#define RTC_CNTL_TIMER_VALUE0_HIGH    0x0000FFFF
#define RTC_CNTL_TIMER_VALUE0_HIGH_M  (RTC_CNTL_TIMER_VALUE0_HIGH_V << RTC_CNTL_TIMER_VALUE0_HIGH_S)
#define RTC_CNTL_TIMER_VALUE0_HIGH_V  0x0000FFFF
#define RTC_CNTL_TIMER_VALUE0_HIGH_S  0

/* RTC_CNTL_STATE0_REG register
 * configure sleep/reject/wakeup state
 */

#define RTC_CNTL_STATE0_REG (DR_REG_RTCCNTL_BASE + 0x18)

/* RTC_CNTL_SLEEP_EN : R/W; bitpos: [31]; default: 0;
 * sleep enable bit
 */

#define RTC_CNTL_SLEEP_EN    (BIT(31))
#define RTC_CNTL_SLEEP_EN_M  (RTC_CNTL_SLEEP_EN_V << RTC_CNTL_SLEEP_EN_S)
#define RTC_CNTL_SLEEP_EN_V  0x00000001
#define RTC_CNTL_SLEEP_EN_S  31

/* RTC_CNTL_SLP_REJECT : R/W; bitpos: [30]; default: 0;
 * leep reject bit
 */

#define RTC_CNTL_SLP_REJECT    (BIT(30))
#define RTC_CNTL_SLP_REJECT_M  (RTC_CNTL_SLP_REJECT_V << RTC_CNTL_SLP_REJECT_S)
#define RTC_CNTL_SLP_REJECT_V  0x00000001
#define RTC_CNTL_SLP_REJECT_S  30

/* RTC_CNTL_SLP_WAKEUP : R/W; bitpos: [29]; default: 0;
 * leep wakeup bit
 */

#define RTC_CNTL_SLP_WAKEUP    (BIT(29))
#define RTC_CNTL_SLP_WAKEUP_M  (RTC_CNTL_SLP_WAKEUP_V << RTC_CNTL_SLP_WAKEUP_S)
#define RTC_CNTL_SLP_WAKEUP_V  0x00000001
#define RTC_CNTL_SLP_WAKEUP_S  29

/* RTC_CNTL_SDIO_ACTIVE_IND : RO; bitpos: [28]; default: 0;
 * SDIO active indication
 */

#define RTC_CNTL_SDIO_ACTIVE_IND    (BIT(28))
#define RTC_CNTL_SDIO_ACTIVE_IND_M  (RTC_CNTL_SDIO_ACTIVE_IND_V << RTC_CNTL_SDIO_ACTIVE_IND_S)
#define RTC_CNTL_SDIO_ACTIVE_IND_V  0x00000001
#define RTC_CNTL_SDIO_ACTIVE_IND_S  28

/* RTC_CNTL_APB2RTC_BRIDGE_SEL : R/W; bitpos: [22]; default: 0;
 * 1: APB to RTC using bridge    0: APB to RTC using sync
 */

#define RTC_CNTL_APB2RTC_BRIDGE_SEL    (BIT(22))
#define RTC_CNTL_APB2RTC_BRIDGE_SEL_M  (RTC_CNTL_APB2RTC_BRIDGE_SEL_V << RTC_CNTL_APB2RTC_BRIDGE_SEL_S)
#define RTC_CNTL_APB2RTC_BRIDGE_SEL_V  0x00000001
#define RTC_CNTL_APB2RTC_BRIDGE_SEL_S  22

/* RTC_CNTL_SLP_REJECT_CAUSE_CLR : WO ;bitpos:[1] ;default: 1'b0 ;
 *description: clear rtc sleep reject cause
 */

#define RTC_CNTL_SLP_REJECT_CAUSE_CLR  (BIT(1))
#define RTC_CNTL_SLP_REJECT_CAUSE_CLR_M  (BIT(1))
#define RTC_CNTL_SLP_REJECT_CAUSE_CLR_V  0x1
#define RTC_CNTL_SLP_REJECT_CAUSE_CLR_S  1

/* RTC_CNTL_SW_CPU_INT : WO; bitpos: [0]; default: 0;
 * rtc software interrupt to main cpu
 */

#define RTC_CNTL_SW_CPU_INT    (BIT(0))
#define RTC_CNTL_SW_CPU_INT_M  (RTC_CNTL_SW_CPU_INT_V << RTC_CNTL_SW_CPU_INT_S)
#define RTC_CNTL_SW_CPU_INT_V  0x00000001
#define RTC_CNTL_SW_CPU_INT_S  0

/* RTC_CNTL_TIMER1_REG register
 * configure time that wait analog state stable
 */

#define RTC_CNTL_TIMER1_REG (DR_REG_RTCCNTL_BASE + 0x1c)

/* RTC_CNTL_PLL_BUF_WAIT : R/W; bitpos: [31:24]; default: 40;
 * PLL wait cycles in slow_clk_rtc
 */

#define RTC_CNTL_PLL_BUF_WAIT    0x000000FF
#define RTC_CNTL_PLL_BUF_WAIT_M  (RTC_CNTL_PLL_BUF_WAIT_V << RTC_CNTL_PLL_BUF_WAIT_S)
#define RTC_CNTL_PLL_BUF_WAIT_V  0x000000FF
#define RTC_CNTL_PLL_BUF_WAIT_S  24
#define RTC_CNTL_PLL_BUF_WAIT_DEFAULT  20

/* RTC_CNTL_XTL_BUF_WAIT : R/W; bitpos: [23:14]; default: 80;
 * XTAL wait cycles in slow_clk_rtc
 */

#define RTC_CNTL_XTL_BUF_WAIT    0x000003FF
#define RTC_CNTL_XTL_BUF_WAIT_M  (RTC_CNTL_XTL_BUF_WAIT_V << RTC_CNTL_XTL_BUF_WAIT_S)
#define RTC_CNTL_XTL_BUF_WAIT_V  0x000003FF
#define RTC_CNTL_XTL_BUF_WAIT_S  14
#define RTC_CNTL_XTL_BUF_WAIT_DEFAULT  20

/* RTC_CNTL_CK8M_WAIT : R/W; bitpos: [13:6]; default: 16;
 * CK8M wait cycles in slow_clk_rtc
 */

#define RTC_CNTL_CK8M_WAIT    0x000000FF
#define RTC_CNTL_CK8M_WAIT_M  (RTC_CNTL_CK8M_WAIT_V << RTC_CNTL_CK8M_WAIT_S)
#define RTC_CNTL_CK8M_WAIT_V  0x000000FF
#define RTC_CNTL_CK8M_WAIT_S  6
#define RTC_CNTL_CK8M_WAIT_DEFAULT  20

/* RTC_CNTL_CPU_STALL_WAIT : R/W; bitpos: [5:1]; default: 1;
 * CPU stall wait cycles in fast_clk_rtc
 */

#define RTC_CNTL_CPU_STALL_WAIT    0x0000001F
#define RTC_CNTL_CPU_STALL_WAIT_M  (RTC_CNTL_CPU_STALL_WAIT_V << RTC_CNTL_CPU_STALL_WAIT_S)
#define RTC_CNTL_CPU_STALL_WAIT_V  0x0000001F
#define RTC_CNTL_CPU_STALL_WAIT_S  1

/* RTC_CNTL_CPU_STALL_EN : R/W; bitpos: [0]; default: 1;
 * CPU stall enable bit
 */

#define RTC_CNTL_CPU_STALL_EN    (BIT(0))
#define RTC_CNTL_CPU_STALL_EN_M  (RTC_CNTL_CPU_STALL_EN_V << RTC_CNTL_CPU_STALL_EN_S)
#define RTC_CNTL_CPU_STALL_EN_V  0x00000001
#define RTC_CNTL_CPU_STALL_EN_S  0

/* RTC_CNTL_TIMER2_REG register
 * configure time that wait analog state stable
 */

#define RTC_CNTL_TIMER2_REG (DR_REG_RTCCNTL_BASE + 0x20)

/* RTC_CNTL_MIN_TIME_CK8M_OFF : R/W; bitpos: [31:24]; default: 1;
 * minimal cycles in slow_clk_rtc for CK8M in power down state
 */

#define RTC_CNTL_MIN_TIME_CK8M_OFF    0x000000FF
#define RTC_CNTL_MIN_TIME_CK8M_OFF_M  (RTC_CNTL_MIN_TIME_CK8M_OFF_V << RTC_CNTL_MIN_TIME_CK8M_OFF_S)
#define RTC_CNTL_MIN_TIME_CK8M_OFF_V  0x000000FF
#define RTC_CNTL_MIN_TIME_CK8M_OFF_S  24

/* RTC_CNTL_ULPCP_TOUCH_START_WAIT : R/W; bitpos: [23:15]; default: 16;
 * wait cycles in slow_clk_rtc before ULP-coprocessor / touch controller
 * start to work
 */

#define RTC_CNTL_ULPCP_TOUCH_START_WAIT    0x000001FF
#define RTC_CNTL_ULPCP_TOUCH_START_WAIT_M  (RTC_CNTL_ULPCP_TOUCH_START_WAIT_V << RTC_CNTL_ULPCP_TOUCH_START_WAIT_S)
#define RTC_CNTL_ULPCP_TOUCH_START_WAIT_V  0x000001FF
#define RTC_CNTL_ULPCP_TOUCH_START_WAIT_S  15

/* RTC_CNTL_TIMER3_REG register
 * configure some wait time for power on
 */

#define RTC_CNTL_TIMER3_REG (DR_REG_RTCCNTL_BASE + 0x24)

/* RTC_CNTL_ROM_RAM_POWERUP_TIMER : R/W; bitpos: [31:25]; default: 10; */

#define RTC_CNTL_ROM_RAM_POWERUP_TIMER    0x0000007F
#define RTC_CNTL_ROM_RAM_POWERUP_TIMER_M  (RTC_CNTL_ROM_RAM_POWERUP_TIMER_V << RTC_CNTL_ROM_RAM_POWERUP_TIMER_S)
#define RTC_CNTL_ROM_RAM_POWERUP_TIMER_V  0x0000007F
#define RTC_CNTL_ROM_RAM_POWERUP_TIMER_S  25

/* RTC_CNTL_ROM_RAM_WAIT_TIMER : R/W; bitpos: [24:16]; default: 22; */

#define RTC_CNTL_ROM_RAM_WAIT_TIMER    0x000001FF
#define RTC_CNTL_ROM_RAM_WAIT_TIMER_M  (RTC_CNTL_ROM_RAM_WAIT_TIMER_V << RTC_CNTL_ROM_RAM_WAIT_TIMER_S)
#define RTC_CNTL_ROM_RAM_WAIT_TIMER_V  0x000001FF
#define RTC_CNTL_ROM_RAM_WAIT_TIMER_S  16

/* RTC_CNTL_WIFI_POWERUP_TIMER : R/W; bitpos: [15:9]; default: 5; */

#define RTC_CNTL_WIFI_POWERUP_TIMER    0x0000007F
#define RTC_CNTL_WIFI_POWERUP_TIMER_M  (RTC_CNTL_WIFI_POWERUP_TIMER_V << RTC_CNTL_WIFI_POWERUP_TIMER_S)
#define RTC_CNTL_WIFI_POWERUP_TIMER_V  0x0000007F
#define RTC_CNTL_WIFI_POWERUP_TIMER_S  9

/* RTC_CNTL_WIFI_WAIT_TIMER : R/W; bitpos: [8:0]; default: 8; */

#define RTC_CNTL_WIFI_WAIT_TIMER    0x000001FF
#define RTC_CNTL_WIFI_WAIT_TIMER_M  (RTC_CNTL_WIFI_WAIT_TIMER_V << RTC_CNTL_WIFI_WAIT_TIMER_S)
#define RTC_CNTL_WIFI_WAIT_TIMER_V  0x000001FF
#define RTC_CNTL_WIFI_WAIT_TIMER_S  0

/* RTC_CNTL_TIMER4_REG register
 * configure some wait time for power on
 */

#define RTC_CNTL_TIMER4_REG (DR_REG_RTCCNTL_BASE + 0x28)

/* RTC_CNTL_DG_WRAP_POWERUP_TIMER : R/W; bitpos: [31:25]; default: 8; */

#define RTC_CNTL_DG_WRAP_POWERUP_TIMER    0x0000007F
#define RTC_CNTL_DG_WRAP_POWERUP_TIMER_M  (RTC_CNTL_DG_WRAP_POWERUP_TIMER_V << RTC_CNTL_DG_WRAP_POWERUP_TIMER_S)
#define RTC_CNTL_DG_WRAP_POWERUP_TIMER_V  0x0000007F
#define RTC_CNTL_DG_WRAP_POWERUP_TIMER_S  25

/* RTC_CNTL_DG_WRAP_WAIT_TIMER : R/W; bitpos: [24:16]; default: 32; */

#define RTC_CNTL_DG_WRAP_WAIT_TIMER    0x000001FF
#define RTC_CNTL_DG_WRAP_WAIT_TIMER_M  (RTC_CNTL_DG_WRAP_WAIT_TIMER_V << RTC_CNTL_DG_WRAP_WAIT_TIMER_S)
#define RTC_CNTL_DG_WRAP_WAIT_TIMER_V  0x000001FF
#define RTC_CNTL_DG_WRAP_WAIT_TIMER_S  16

/* RTC_CNTL_POWERUP_TIMER : R/W; bitpos: [15:9]; default: 5; */

#define RTC_CNTL_POWERUP_TIMER    0x0000007F
#define RTC_CNTL_POWERUP_TIMER_M  (RTC_CNTL_POWERUP_TIMER_V << RTC_CNTL_POWERUP_TIMER_S)
#define RTC_CNTL_POWERUP_TIMER_V  0x0000007F
#define RTC_CNTL_POWERUP_TIMER_S  9

/* RTC_CNTL_WAIT_TIMER : R/W; bitpos: [8:0]; default: 8; */

#define RTC_CNTL_WAIT_TIMER    0x000001FF
#define RTC_CNTL_WAIT_TIMER_M  (RTC_CNTL_WAIT_TIMER_V << RTC_CNTL_WAIT_TIMER_S)
#define RTC_CNTL_WAIT_TIMER_V  0x000001FF
#define RTC_CNTL_WAIT_TIMER_S  0

/* RTC_CNTL_TIMER5_REG register
 * Configure minimal sleep cycles register
 */

#define RTC_CNTL_TIMER5_REG (DR_REG_RTCCNTL_BASE + 0x2c)

/* RTC_CNTL_RTCMEM_POWERUP_TIMER : R/W; bitpos: [31:25]; default: 9; */

#define RTC_CNTL_RTCMEM_POWERUP_TIMER    0x0000007F
#define RTC_CNTL_RTCMEM_POWERUP_TIMER_M  (RTC_CNTL_RTCMEM_POWERUP_TIMER_V << RTC_CNTL_RTCMEM_POWERUP_TIMER_S)
#define RTC_CNTL_RTCMEM_POWERUP_TIMER_V  0x0000007F
#define RTC_CNTL_RTCMEM_POWERUP_TIMER_S  25

/* RTC_CNTL_RTCMEM_WAIT_TIMER : R/W; bitpos: [24:16]; default: 20; */

#define RTC_CNTL_RTCMEM_WAIT_TIMER    0x000001FF
#define RTC_CNTL_RTCMEM_WAIT_TIMER_M  (RTC_CNTL_RTCMEM_WAIT_TIMER_V << RTC_CNTL_RTCMEM_WAIT_TIMER_S)
#define RTC_CNTL_RTCMEM_WAIT_TIMER_V  0x000001FF
#define RTC_CNTL_RTCMEM_WAIT_TIMER_S  16

/* RTC_CNTL_MIN_SLP_VAL : R/W; bitpos: [15:8]; default: 128;
 * minimal sleep cycles in slow_clk_rtc
 */

#define RTC_CNTL_MIN_SLP_VAL    0x000000FF
#define RTC_CNTL_MIN_SLP_VAL_M  (RTC_CNTL_MIN_SLP_VAL_V << RTC_CNTL_MIN_SLP_VAL_S)
#define RTC_CNTL_MIN_SLP_VAL_V  0x000000FF
#define RTC_CNTL_MIN_SLP_VAL_S  8
#define RTC_CNTL_MIN_SLP_VAL_MIN 2

/* RTC_CNTL_TIMER6_REG register
 * Configure minimal sleep cycles register
 */

#define RTC_CNTL_TIMER6_REG (DR_REG_RTCCNTL_BASE + 0x30)

/* RTC_CNTL_DG_DCDC_POWERUP_TIMER : R/W; bitpos: [31:25]; default: 8; */

#define RTC_CNTL_DG_DCDC_POWERUP_TIMER    0x0000007F
#define RTC_CNTL_DG_DCDC_POWERUP_TIMER_M  (RTC_CNTL_DG_DCDC_POWERUP_TIMER_V << RTC_CNTL_DG_DCDC_POWERUP_TIMER_S)
#define RTC_CNTL_DG_DCDC_POWERUP_TIMER_V  0x0000007F
#define RTC_CNTL_DG_DCDC_POWERUP_TIMER_S  25

/* RTC_CNTL_DG_DCDC_WAIT_TIMER : R/W; bitpos: [24:16]; default: 32; */

#define RTC_CNTL_DG_DCDC_WAIT_TIMER    0x000001FF
#define RTC_CNTL_DG_DCDC_WAIT_TIMER_M  (RTC_CNTL_DG_DCDC_WAIT_TIMER_V << RTC_CNTL_DG_DCDC_WAIT_TIMER_S)
#define RTC_CNTL_DG_DCDC_WAIT_TIMER_V  0x000001FF
#define RTC_CNTL_DG_DCDC_WAIT_TIMER_S  16

/* RTC_CNTL_ANA_CONF_REG register
 * configure some i2c and plla power
 */

#define RTC_CNTL_ANA_CONF_REG (DR_REG_RTCCNTL_BASE + 0x34)

/* RTC_CNTL_PLL_I2C_PU : R/W; bitpos: [31]; default: 0;
 * 1. PLL_I2C power up ,otherwise power down
 */

#define RTC_CNTL_PLL_I2C_PU    (BIT(31))
#define RTC_CNTL_PLL_I2C_PU_M  (RTC_CNTL_PLL_I2C_PU_V << RTC_CNTL_PLL_I2C_PU_S)
#define RTC_CNTL_PLL_I2C_PU_V  0x00000001
#define RTC_CNTL_PLL_I2C_PU_S  31

/* RTC_CNTL_CKGEN_I2C_PU : R/W; bitpos: [30]; default: 0;
 * 1: CKGEN_I2C power up   , otherwise power down
 */

#define RTC_CNTL_CKGEN_I2C_PU    (BIT(30))
#define RTC_CNTL_CKGEN_I2C_PU_M  (RTC_CNTL_CKGEN_I2C_PU_V << RTC_CNTL_CKGEN_I2C_PU_S)
#define RTC_CNTL_CKGEN_I2C_PU_V  0x00000001
#define RTC_CNTL_CKGEN_I2C_PU_S  30

/* RTC_CNTL_RFRX_PBUS_PU : R/W; bitpos: [28]; default: 0;
 * 1: RFRX_PBUS power up   ,   otherwise power down
 */

#define RTC_CNTL_RFRX_PBUS_PU    (BIT(28))
#define RTC_CNTL_RFRX_PBUS_PU_M  (RTC_CNTL_RFRX_PBUS_PU_V << RTC_CNTL_RFRX_PBUS_PU_S)
#define RTC_CNTL_RFRX_PBUS_PU_V  0x00000001
#define RTC_CNTL_RFRX_PBUS_PU_S  28

/* RTC_CNTL_TXRF_I2C_PU : R/W; bitpos: [27]; default: 0;
 * 1: TXRF_I2C power up    ,    otherwise power down
 */

#define RTC_CNTL_TXRF_I2C_PU    (BIT(27))
#define RTC_CNTL_TXRF_I2C_PU_M  (RTC_CNTL_TXRF_I2C_PU_V << RTC_CNTL_TXRF_I2C_PU_S)
#define RTC_CNTL_TXRF_I2C_PU_V  0x00000001
#define RTC_CNTL_TXRF_I2C_PU_S  27

/* RTC_CNTL_PVTMON_PU : R/W; bitpos: [26]; default: 0;
 * 1: PVTMON power up    ,  otherwise power down
 */

#define RTC_CNTL_PVTMON_PU    (BIT(26))
#define RTC_CNTL_PVTMON_PU_M  (RTC_CNTL_PVTMON_PU_V << RTC_CNTL_PVTMON_PU_S)
#define RTC_CNTL_PVTMON_PU_V  0x00000001
#define RTC_CNTL_PVTMON_PU_S  26

/* RTC_CNTL_BBPLL_CAL_SLP_START : R/W; bitpos: [25]; default: 0;
 * start BBPLL calibration during sleep
 */

#define RTC_CNTL_BBPLL_CAL_SLP_START    (BIT(25))
#define RTC_CNTL_BBPLL_CAL_SLP_START_M  (RTC_CNTL_BBPLL_CAL_SLP_START_V << RTC_CNTL_BBPLL_CAL_SLP_START_S)
#define RTC_CNTL_BBPLL_CAL_SLP_START_V  0x00000001
#define RTC_CNTL_BBPLL_CAL_SLP_START_S  25

/* RTC_CNTL_PLLA_FORCE_PU : R/W; bitpos: [24]; default: 0;
 * PLLA force power up
 */

#define RTC_CNTL_PLLA_FORCE_PU    (BIT(24))
#define RTC_CNTL_PLLA_FORCE_PU_M  (RTC_CNTL_PLLA_FORCE_PU_V << RTC_CNTL_PLLA_FORCE_PU_S)
#define RTC_CNTL_PLLA_FORCE_PU_V  0x00000001
#define RTC_CNTL_PLLA_FORCE_PU_S  24

/* RTC_CNTL_PLLA_FORCE_PD : R/W; bitpos: [23]; default: 1;
 * PLLA force power down
 */

#define RTC_CNTL_PLLA_FORCE_PD    (BIT(23))
#define RTC_CNTL_PLLA_FORCE_PD_M  (RTC_CNTL_PLLA_FORCE_PD_V << RTC_CNTL_PLLA_FORCE_PD_S)
#define RTC_CNTL_PLLA_FORCE_PD_V  0x00000001
#define RTC_CNTL_PLLA_FORCE_PD_S  23

/* RTC_CNTL_SAR_I2C_FORCE_PU : R/W; bitpos: [22]; default: 0;
 * SAR_I2C force power up
 */

#define RTC_CNTL_SAR_I2C_FORCE_PU    (BIT(22))
#define RTC_CNTL_SAR_I2C_FORCE_PU_M  (RTC_CNTL_SAR_I2C_FORCE_PU_V << RTC_CNTL_SAR_I2C_FORCE_PU_S)
#define RTC_CNTL_SAR_I2C_FORCE_PU_V  0x00000001
#define RTC_CNTL_SAR_I2C_FORCE_PU_S  22

/* RTC_CNTL_SAR_I2C_FORCE_PD : R/W; bitpos: [21]; default: 1;
 * SAR_I2C force power down
 */

#define RTC_CNTL_SAR_I2C_FORCE_PD    (BIT(21))
#define RTC_CNTL_SAR_I2C_FORCE_PD_M  (RTC_CNTL_SAR_I2C_FORCE_PD_V << RTC_CNTL_SAR_I2C_FORCE_PD_S)
#define RTC_CNTL_SAR_I2C_FORCE_PD_V  0x00000001
#define RTC_CNTL_SAR_I2C_FORCE_PD_S  21

/* RTC_CNTL_GLITCH_RST_EN : R/W; bitpos: [20]; default: 0;
 * enable glitch reset if system detect glitch
 */

#define RTC_CNTL_GLITCH_RST_EN    (BIT(20))
#define RTC_CNTL_GLITCH_RST_EN_M  (RTC_CNTL_GLITCH_RST_EN_V << RTC_CNTL_GLITCH_RST_EN_S)
#define RTC_CNTL_GLITCH_RST_EN_V  0x00000001
#define RTC_CNTL_GLITCH_RST_EN_S  20

/* RTC_CNTL_I2C_RESET_POR_FORCE_PU : R/W; bitpos: [19]; default: 0;
 * SLEEP_I2CPOR force pu
 */

#define RTC_CNTL_I2C_RESET_POR_FORCE_PU    (BIT(19))
#define RTC_CNTL_I2C_RESET_POR_FORCE_PU_M  (RTC_CNTL_I2C_RESET_POR_FORCE_PU_V << RTC_CNTL_I2C_RESET_POR_FORCE_PU_S)
#define RTC_CNTL_I2C_RESET_POR_FORCE_PU_V  0x00000001
#define RTC_CNTL_I2C_RESET_POR_FORCE_PU_S  19

/* RTC_CNTL_I2C_RESET_POR_FORCE_PD : R/W; bitpos: [18]; default: 1;
 * SLEEP_I2CPOR force pd
 */

#define RTC_CNTL_I2C_RESET_POR_FORCE_PD    (BIT(18))
#define RTC_CNTL_I2C_RESET_POR_FORCE_PD_M  (RTC_CNTL_I2C_RESET_POR_FORCE_PD_V << RTC_CNTL_I2C_RESET_POR_FORCE_PD_S)
#define RTC_CNTL_I2C_RESET_POR_FORCE_PD_V  0x00000001
#define RTC_CNTL_I2C_RESET_POR_FORCE_PD_S  18

/* RTC_CNTL_RESET_STATE_REG register
 * reset cause state register
 */

#define RTC_CNTL_RESET_STATE_REG (DR_REG_RTCCNTL_BASE + 0x38)

/* RTC_CNTL_PROCPU_STAT_VECTOR_SEL : R/W; bitpos: [13]; default: 1;
 * PRO CPU state vector sel
 */

#define RTC_CNTL_PROCPU_STAT_VECTOR_SEL    (BIT(13))
#define RTC_CNTL_PROCPU_STAT_VECTOR_SEL_M  (RTC_CNTL_PROCPU_STAT_VECTOR_SEL_V << RTC_CNTL_PROCPU_STAT_VECTOR_SEL_S)
#define RTC_CNTL_PROCPU_STAT_VECTOR_SEL_V  0x00000001
#define RTC_CNTL_PROCPU_STAT_VECTOR_SEL_S  13

/* RTC_CNTL_APPCPU_STAT_VECTOR_SEL : R/W; bitpos: [12]; default: 1;
 * APP CPU state vector sel
 */

#define RTC_CNTL_APPCPU_STAT_VECTOR_SEL    (BIT(12))
#define RTC_CNTL_APPCPU_STAT_VECTOR_SEL_M  (RTC_CNTL_APPCPU_STAT_VECTOR_SEL_V << RTC_CNTL_APPCPU_STAT_VECTOR_SEL_S)
#define RTC_CNTL_APPCPU_STAT_VECTOR_SEL_V  0x00000001
#define RTC_CNTL_APPCPU_STAT_VECTOR_SEL_S  12

/* RTC_CNTL_RESET_CAUSE_APPCPU : RO; bitpos: [11:6]; default: 0;
 * reset cause of APP CPU
 */

#define RTC_CNTL_RESET_CAUSE_APPCPU    0x0000003F
#define RTC_CNTL_RESET_CAUSE_APPCPU_M  (RTC_CNTL_RESET_CAUSE_APPCPU_V << RTC_CNTL_RESET_CAUSE_APPCPU_S)
#define RTC_CNTL_RESET_CAUSE_APPCPU_V  0x0000003F
#define RTC_CNTL_RESET_CAUSE_APPCPU_S  6

/* RTC_CNTL_RESET_CAUSE_PROCPU : RO; bitpos: [5:0]; default: 0;
 * reset cause of PRO CPU
 */

#define RTC_CNTL_RESET_CAUSE_PROCPU    0x0000003F
#define RTC_CNTL_RESET_CAUSE_PROCPU_M  (RTC_CNTL_RESET_CAUSE_PROCPU_V << RTC_CNTL_RESET_CAUSE_PROCPU_S)
#define RTC_CNTL_RESET_CAUSE_PROCPU_V  0x0000003F
#define RTC_CNTL_RESET_CAUSE_PROCPU_S  0

/* RTC_CNTL_WAKEUP_STATE_REG register
 * wakeup enable register
 */

#define RTC_CNTL_WAKEUP_STATE_REG (DR_REG_RTCCNTL_BASE + 0x3c)

/* RTC_CNTL_WAKEUP_ENA : R/W; bitpos: [31:15]; default: 12;
 * wakeup enable bitmap
 */

#define RTC_CNTL_WAKEUP_ENA    0x0001FFFF
#define RTC_CNTL_WAKEUP_ENA_M  (RTC_CNTL_WAKEUP_ENA_V << RTC_CNTL_WAKEUP_ENA_S)
#define RTC_CNTL_WAKEUP_ENA_V  0x0001FFFF
#define RTC_CNTL_WAKEUP_ENA_S  15

/* RTC_CNTL_INT_ENA_RTC_REG register
 * rtc interrupt enable register
 */

#define RTC_CNTL_INT_ENA_RTC_REG (DR_REG_RTCCNTL_BASE + 0x40)

/* RTC_CNTL_GLITCH_DET_INT_ENA : R/W; bitpos: [19]; default: 0;
 * enbale gitch det interrupt
 */

#define RTC_CNTL_GLITCH_DET_INT_ENA    (BIT(19))
#define RTC_CNTL_GLITCH_DET_INT_ENA_M  (RTC_CNTL_GLITCH_DET_INT_ENA_V << RTC_CNTL_GLITCH_DET_INT_ENA_S)
#define RTC_CNTL_GLITCH_DET_INT_ENA_V  0x00000001
#define RTC_CNTL_GLITCH_DET_INT_ENA_S  19

/* RTC_CNTL_TOUCH_TIMEOUT_INT_ENA : R/W; bitpos: [18]; default: 0;
 * enable touch timeout interrupt
 */

#define RTC_CNTL_TOUCH_TIMEOUT_INT_ENA    (BIT(18))
#define RTC_CNTL_TOUCH_TIMEOUT_INT_ENA_M  (RTC_CNTL_TOUCH_TIMEOUT_INT_ENA_V << RTC_CNTL_TOUCH_TIMEOUT_INT_ENA_S)
#define RTC_CNTL_TOUCH_TIMEOUT_INT_ENA_V  0x00000001
#define RTC_CNTL_TOUCH_TIMEOUT_INT_ENA_S  18

/* RTC_CNTL_COCPU_TRAP_INT_ENA : R/W; bitpos: [17]; default: 0;
 * enable cocpu trap interrupt
 */

#define RTC_CNTL_COCPU_TRAP_INT_ENA    (BIT(17))
#define RTC_CNTL_COCPU_TRAP_INT_ENA_M  (RTC_CNTL_COCPU_TRAP_INT_ENA_V << RTC_CNTL_COCPU_TRAP_INT_ENA_S)
#define RTC_CNTL_COCPU_TRAP_INT_ENA_V  0x00000001
#define RTC_CNTL_COCPU_TRAP_INT_ENA_S  17

/* RTC_CNTL_XTAL32K_DEAD_INT_ENA : R/W; bitpos: [16]; default: 0;
 * enable xtal32k_dead  interrupt
 */

#define RTC_CNTL_XTAL32K_DEAD_INT_ENA    (BIT(16))
#define RTC_CNTL_XTAL32K_DEAD_INT_ENA_M  (RTC_CNTL_XTAL32K_DEAD_INT_ENA_V << RTC_CNTL_XTAL32K_DEAD_INT_ENA_S)
#define RTC_CNTL_XTAL32K_DEAD_INT_ENA_V  0x00000001
#define RTC_CNTL_XTAL32K_DEAD_INT_ENA_S  16

/* RTC_CNTL_SWD_INT_ENA : R/W; bitpos: [15]; default: 0;
 * enable super watch dog interrupt
 */

#define RTC_CNTL_SWD_INT_ENA    (BIT(15))
#define RTC_CNTL_SWD_INT_ENA_M  (RTC_CNTL_SWD_INT_ENA_V << RTC_CNTL_SWD_INT_ENA_S)
#define RTC_CNTL_SWD_INT_ENA_V  0x00000001
#define RTC_CNTL_SWD_INT_ENA_S  15

/* RTC_CNTL_SARADC2_INT_ENA : R/W; bitpos: [14]; default: 0;
 * enable saradc2 interrupt
 */

#define RTC_CNTL_SARADC2_INT_ENA    (BIT(14))
#define RTC_CNTL_SARADC2_INT_ENA_M  (RTC_CNTL_SARADC2_INT_ENA_V << RTC_CNTL_SARADC2_INT_ENA_S)
#define RTC_CNTL_SARADC2_INT_ENA_V  0x00000001
#define RTC_CNTL_SARADC2_INT_ENA_S  14

/* RTC_CNTL_COCPU_INT_ENA : R/W; bitpos: [13]; default: 0;
 * enable riscV cocpu interrupt
 */

#define RTC_CNTL_COCPU_INT_ENA    (BIT(13))
#define RTC_CNTL_COCPU_INT_ENA_M  (RTC_CNTL_COCPU_INT_ENA_V << RTC_CNTL_COCPU_INT_ENA_S)
#define RTC_CNTL_COCPU_INT_ENA_V  0x00000001
#define RTC_CNTL_COCPU_INT_ENA_S  13

/* RTC_CNTL_TSENS_INT_ENA : R/W; bitpos: [12]; default: 0;
 * enable tsens interrupt
 */

#define RTC_CNTL_TSENS_INT_ENA    (BIT(12))
#define RTC_CNTL_TSENS_INT_ENA_M  (RTC_CNTL_TSENS_INT_ENA_V << RTC_CNTL_TSENS_INT_ENA_S)
#define RTC_CNTL_TSENS_INT_ENA_V  0x00000001
#define RTC_CNTL_TSENS_INT_ENA_S  12

/* RTC_CNTL_SARADC1_INT_ENA : R/W; bitpos: [11]; default: 0;
 * enable saradc1 interrupt
 */

#define RTC_CNTL_SARADC1_INT_ENA    (BIT(11))
#define RTC_CNTL_SARADC1_INT_ENA_M  (RTC_CNTL_SARADC1_INT_ENA_V << RTC_CNTL_SARADC1_INT_ENA_S)
#define RTC_CNTL_SARADC1_INT_ENA_V  0x00000001
#define RTC_CNTL_SARADC1_INT_ENA_S  11

/* RTC_CNTL_MAIN_TIMER_INT_ENA : R/W; bitpos: [10]; default: 0;
 * enable RTC main timer interrupt
 */

#define RTC_CNTL_MAIN_TIMER_INT_ENA    (BIT(10))
#define RTC_CNTL_MAIN_TIMER_INT_ENA_M  (RTC_CNTL_MAIN_TIMER_INT_ENA_V << RTC_CNTL_MAIN_TIMER_INT_ENA_S)
#define RTC_CNTL_MAIN_TIMER_INT_ENA_V  0x00000001
#define RTC_CNTL_MAIN_TIMER_INT_ENA_S  10

/* RTC_CNTL_BROWN_OUT_INT_ENA : R/W; bitpos: [9]; default: 0;
 * enable brown out interrupt
 */

#define RTC_CNTL_BROWN_OUT_INT_ENA    (BIT(9))
#define RTC_CNTL_BROWN_OUT_INT_ENA_M  (RTC_CNTL_BROWN_OUT_INT_ENA_V << RTC_CNTL_BROWN_OUT_INT_ENA_S)
#define RTC_CNTL_BROWN_OUT_INT_ENA_V  0x00000001
#define RTC_CNTL_BROWN_OUT_INT_ENA_S  9

/* RTC_CNTL_TOUCH_INACTIVE_INT_ENA : R/W; bitpos: [8]; default: 0;
 * enable touch inactive interrupt
 */

#define RTC_CNTL_TOUCH_INACTIVE_INT_ENA    (BIT(8))
#define RTC_CNTL_TOUCH_INACTIVE_INT_ENA_M  (RTC_CNTL_TOUCH_INACTIVE_INT_ENA_V << RTC_CNTL_TOUCH_INACTIVE_INT_ENA_S)
#define RTC_CNTL_TOUCH_INACTIVE_INT_ENA_V  0x00000001
#define RTC_CNTL_TOUCH_INACTIVE_INT_ENA_S  8

/* RTC_CNTL_TOUCH_ACTIVE_INT_ENA : R/W; bitpos: [7]; default: 0;
 * enable touch active interrupt
 */

#define RTC_CNTL_TOUCH_ACTIVE_INT_ENA    (BIT(7))
#define RTC_CNTL_TOUCH_ACTIVE_INT_ENA_M  (RTC_CNTL_TOUCH_ACTIVE_INT_ENA_V << RTC_CNTL_TOUCH_ACTIVE_INT_ENA_S)
#define RTC_CNTL_TOUCH_ACTIVE_INT_ENA_V  0x00000001
#define RTC_CNTL_TOUCH_ACTIVE_INT_ENA_S  7

/* RTC_CNTL_TOUCH_DONE_INT_ENA : R/W; bitpos: [6]; default: 0;
 * enable touch done interrupt
 */

#define RTC_CNTL_TOUCH_DONE_INT_ENA    (BIT(6))
#define RTC_CNTL_TOUCH_DONE_INT_ENA_M  (RTC_CNTL_TOUCH_DONE_INT_ENA_V << RTC_CNTL_TOUCH_DONE_INT_ENA_S)
#define RTC_CNTL_TOUCH_DONE_INT_ENA_V  0x00000001
#define RTC_CNTL_TOUCH_DONE_INT_ENA_S  6

/* RTC_CNTL_ULP_CP_INT_ENA : R/W; bitpos: [5]; default: 0;
 * enable ULP-coprocessor interrupt
 */

#define RTC_CNTL_ULP_CP_INT_ENA    (BIT(5))
#define RTC_CNTL_ULP_CP_INT_ENA_M  (RTC_CNTL_ULP_CP_INT_ENA_V << RTC_CNTL_ULP_CP_INT_ENA_S)
#define RTC_CNTL_ULP_CP_INT_ENA_V  0x00000001
#define RTC_CNTL_ULP_CP_INT_ENA_S  5

/* RTC_CNTL_TOUCH_SCAN_DONE_INT_ENA : R/W; bitpos: [4]; default: 0;
 * enable touch scan done interrupt
 */

#define RTC_CNTL_TOUCH_SCAN_DONE_INT_ENA    (BIT(4))
#define RTC_CNTL_TOUCH_SCAN_DONE_INT_ENA_M  (RTC_CNTL_TOUCH_SCAN_DONE_INT_ENA_V << RTC_CNTL_TOUCH_SCAN_DONE_INT_ENA_S)
#define RTC_CNTL_TOUCH_SCAN_DONE_INT_ENA_V  0x00000001
#define RTC_CNTL_TOUCH_SCAN_DONE_INT_ENA_S  4

/* RTC_CNTL_WDT_INT_ENA : R/W; bitpos: [3]; default: 0;
 * enable RTC WDT interrupt
 */

#define RTC_CNTL_WDT_INT_ENA    (BIT(3))
#define RTC_CNTL_WDT_INT_ENA_M  (RTC_CNTL_WDT_INT_ENA_V << RTC_CNTL_WDT_INT_ENA_S)
#define RTC_CNTL_WDT_INT_ENA_V  0x00000001
#define RTC_CNTL_WDT_INT_ENA_S  3

/* RTC_CNTL_SDIO_IDLE_INT_ENA : R/W; bitpos: [2]; default: 0;
 * enable SDIO idle interrupt
 */

#define RTC_CNTL_SDIO_IDLE_INT_ENA    (BIT(2))
#define RTC_CNTL_SDIO_IDLE_INT_ENA_M  (RTC_CNTL_SDIO_IDLE_INT_ENA_V << RTC_CNTL_SDIO_IDLE_INT_ENA_S)
#define RTC_CNTL_SDIO_IDLE_INT_ENA_V  0x00000001
#define RTC_CNTL_SDIO_IDLE_INT_ENA_S  2

/* RTC_CNTL_SLP_REJECT_INT_ENA : R/W; bitpos: [1]; default: 0;
 * enable sleep reject interrupt
 */

#define RTC_CNTL_SLP_REJECT_INT_ENA    (BIT(1))
#define RTC_CNTL_SLP_REJECT_INT_ENA_M  (RTC_CNTL_SLP_REJECT_INT_ENA_V << RTC_CNTL_SLP_REJECT_INT_ENA_S)
#define RTC_CNTL_SLP_REJECT_INT_ENA_V  0x00000001
#define RTC_CNTL_SLP_REJECT_INT_ENA_S  1

/* RTC_CNTL_SLP_WAKEUP_INT_ENA : R/W; bitpos: [0]; default: 0;
 * enable sleep wakeup interrupt
 */

#define RTC_CNTL_SLP_WAKEUP_INT_ENA    (BIT(0))
#define RTC_CNTL_SLP_WAKEUP_INT_ENA_M  (RTC_CNTL_SLP_WAKEUP_INT_ENA_V << RTC_CNTL_SLP_WAKEUP_INT_ENA_S)
#define RTC_CNTL_SLP_WAKEUP_INT_ENA_V  0x00000001
#define RTC_CNTL_SLP_WAKEUP_INT_ENA_S  0

/* RTC_CNTL_INT_RAW_RTC_REG register
 * rtc_interrupt raw register
 */

#define RTC_CNTL_INT_RAW_RTC_REG (DR_REG_RTCCNTL_BASE + 0x44)

/* RTC_CNTL_GLITCH_DET_INT_RAW : RO; bitpos: [19]; default: 0;
 * glitch_det_interrupt_raw
 */

#define RTC_CNTL_GLITCH_DET_INT_RAW    (BIT(19))
#define RTC_CNTL_GLITCH_DET_INT_RAW_M  (RTC_CNTL_GLITCH_DET_INT_RAW_V << RTC_CNTL_GLITCH_DET_INT_RAW_S)
#define RTC_CNTL_GLITCH_DET_INT_RAW_V  0x00000001
#define RTC_CNTL_GLITCH_DET_INT_RAW_S  19

/* RTC_CNTL_TOUCH_TIMEOUT_INT_RAW : RO; bitpos: [18]; default: 0;
 * touch timeout interrupt raw
 */

#define RTC_CNTL_TOUCH_TIMEOUT_INT_RAW    (BIT(18))
#define RTC_CNTL_TOUCH_TIMEOUT_INT_RAW_M  (RTC_CNTL_TOUCH_TIMEOUT_INT_RAW_V << RTC_CNTL_TOUCH_TIMEOUT_INT_RAW_S)
#define RTC_CNTL_TOUCH_TIMEOUT_INT_RAW_V  0x00000001
#define RTC_CNTL_TOUCH_TIMEOUT_INT_RAW_S  18

/* RTC_CNTL_COCPU_TRAP_INT_RAW : RO; bitpos: [17]; default: 0;
 * cocpu trap interrupt raw
 */

#define RTC_CNTL_COCPU_TRAP_INT_RAW    (BIT(17))
#define RTC_CNTL_COCPU_TRAP_INT_RAW_M  (RTC_CNTL_COCPU_TRAP_INT_RAW_V << RTC_CNTL_COCPU_TRAP_INT_RAW_S)
#define RTC_CNTL_COCPU_TRAP_INT_RAW_V  0x00000001
#define RTC_CNTL_COCPU_TRAP_INT_RAW_S  17

/* RTC_CNTL_XTAL32K_DEAD_INT_RAW : RO; bitpos: [16]; default: 0;
 * xtal32k dead detection interrupt raw
 */

#define RTC_CNTL_XTAL32K_DEAD_INT_RAW    (BIT(16))
#define RTC_CNTL_XTAL32K_DEAD_INT_RAW_M  (RTC_CNTL_XTAL32K_DEAD_INT_RAW_V << RTC_CNTL_XTAL32K_DEAD_INT_RAW_S)
#define RTC_CNTL_XTAL32K_DEAD_INT_RAW_V  0x00000001
#define RTC_CNTL_XTAL32K_DEAD_INT_RAW_S  16

/* RTC_CNTL_SWD_INT_RAW : RO; bitpos: [15]; default: 0;
 * super watch dog interrupt raw
 */

#define RTC_CNTL_SWD_INT_RAW    (BIT(15))
#define RTC_CNTL_SWD_INT_RAW_M  (RTC_CNTL_SWD_INT_RAW_V << RTC_CNTL_SWD_INT_RAW_S)
#define RTC_CNTL_SWD_INT_RAW_V  0x00000001
#define RTC_CNTL_SWD_INT_RAW_S  15

/* RTC_CNTL_SARADC2_INT_RAW : RO; bitpos: [14]; default: 0;
 * saradc2 interrupt raw
 */

#define RTC_CNTL_SARADC2_INT_RAW    (BIT(14))
#define RTC_CNTL_SARADC2_INT_RAW_M  (RTC_CNTL_SARADC2_INT_RAW_V << RTC_CNTL_SARADC2_INT_RAW_S)
#define RTC_CNTL_SARADC2_INT_RAW_V  0x00000001
#define RTC_CNTL_SARADC2_INT_RAW_S  14

/* RTC_CNTL_COCPU_INT_RAW : RO; bitpos: [13]; default: 0;
 * riscV cocpu interrupt raw
 */

#define RTC_CNTL_COCPU_INT_RAW    (BIT(13))
#define RTC_CNTL_COCPU_INT_RAW_M  (RTC_CNTL_COCPU_INT_RAW_V << RTC_CNTL_COCPU_INT_RAW_S)
#define RTC_CNTL_COCPU_INT_RAW_V  0x00000001
#define RTC_CNTL_COCPU_INT_RAW_S  13

/* RTC_CNTL_TSENS_INT_RAW : RO; bitpos: [12]; default: 0;
 * tsens interrupt raw
 */

#define RTC_CNTL_TSENS_INT_RAW    (BIT(12))
#define RTC_CNTL_TSENS_INT_RAW_M  (RTC_CNTL_TSENS_INT_RAW_V << RTC_CNTL_TSENS_INT_RAW_S)
#define RTC_CNTL_TSENS_INT_RAW_V  0x00000001
#define RTC_CNTL_TSENS_INT_RAW_S  12

/* RTC_CNTL_SARADC1_INT_RAW : RO; bitpos: [11]; default: 0;
 * saradc1 interrupt raw
 */

#define RTC_CNTL_SARADC1_INT_RAW    (BIT(11))
#define RTC_CNTL_SARADC1_INT_RAW_M  (RTC_CNTL_SARADC1_INT_RAW_V << RTC_CNTL_SARADC1_INT_RAW_S)
#define RTC_CNTL_SARADC1_INT_RAW_V  0x00000001
#define RTC_CNTL_SARADC1_INT_RAW_S  11

/* RTC_CNTL_MAIN_TIMER_INT_RAW : RO; bitpos: [10]; default: 0;
 * RTC main timer interrupt raw
 */

#define RTC_CNTL_MAIN_TIMER_INT_RAW    (BIT(10))
#define RTC_CNTL_MAIN_TIMER_INT_RAW_M  (RTC_CNTL_MAIN_TIMER_INT_RAW_V << RTC_CNTL_MAIN_TIMER_INT_RAW_S)
#define RTC_CNTL_MAIN_TIMER_INT_RAW_V  0x00000001
#define RTC_CNTL_MAIN_TIMER_INT_RAW_S  10

/* RTC_CNTL_BROWN_OUT_INT_RAW : RO; bitpos: [9]; default: 0;
 * brown out interrupt raw
 */

#define RTC_CNTL_BROWN_OUT_INT_RAW    (BIT(9))
#define RTC_CNTL_BROWN_OUT_INT_RAW_M  (RTC_CNTL_BROWN_OUT_INT_RAW_V << RTC_CNTL_BROWN_OUT_INT_RAW_S)
#define RTC_CNTL_BROWN_OUT_INT_RAW_V  0x00000001
#define RTC_CNTL_BROWN_OUT_INT_RAW_S  9

/* RTC_CNTL_TOUCH_INACTIVE_INT_RAW : RO; bitpos: [8]; default: 0;
 * touch inactive interrupt raw
 */

#define RTC_CNTL_TOUCH_INACTIVE_INT_RAW    (BIT(8))
#define RTC_CNTL_TOUCH_INACTIVE_INT_RAW_M  (RTC_CNTL_TOUCH_INACTIVE_INT_RAW_V << RTC_CNTL_TOUCH_INACTIVE_INT_RAW_S)
#define RTC_CNTL_TOUCH_INACTIVE_INT_RAW_V  0x00000001
#define RTC_CNTL_TOUCH_INACTIVE_INT_RAW_S  8

/* RTC_CNTL_TOUCH_ACTIVE_INT_RAW : RO; bitpos: [7]; default: 0;
 * touch active interrupt raw
 */

#define RTC_CNTL_TOUCH_ACTIVE_INT_RAW    (BIT(7))
#define RTC_CNTL_TOUCH_ACTIVE_INT_RAW_M  (RTC_CNTL_TOUCH_ACTIVE_INT_RAW_V << RTC_CNTL_TOUCH_ACTIVE_INT_RAW_S)
#define RTC_CNTL_TOUCH_ACTIVE_INT_RAW_V  0x00000001
#define RTC_CNTL_TOUCH_ACTIVE_INT_RAW_S  7

/* RTC_CNTL_TOUCH_DONE_INT_RAW : RO; bitpos: [6]; default: 0;
 * touch interrupt raw
 */

#define RTC_CNTL_TOUCH_DONE_INT_RAW    (BIT(6))
#define RTC_CNTL_TOUCH_DONE_INT_RAW_M  (RTC_CNTL_TOUCH_DONE_INT_RAW_V << RTC_CNTL_TOUCH_DONE_INT_RAW_S)
#define RTC_CNTL_TOUCH_DONE_INT_RAW_V  0x00000001
#define RTC_CNTL_TOUCH_DONE_INT_RAW_S  6

/* RTC_CNTL_ULP_CP_INT_RAW : RO; bitpos: [5]; default: 0;
 * ULP-coprocessor interrupt raw
 */

#define RTC_CNTL_ULP_CP_INT_RAW    (BIT(5))
#define RTC_CNTL_ULP_CP_INT_RAW_M  (RTC_CNTL_ULP_CP_INT_RAW_V << RTC_CNTL_ULP_CP_INT_RAW_S)
#define RTC_CNTL_ULP_CP_INT_RAW_V  0x00000001
#define RTC_CNTL_ULP_CP_INT_RAW_S  5

/* RTC_CNTL_TOUCH_SCAN_DONE_INT_RAW : RO; bitpos: [4]; default: 0;
 * touch complete a loop interrupt raw
 */

#define RTC_CNTL_TOUCH_SCAN_DONE_INT_RAW    (BIT(4))
#define RTC_CNTL_TOUCH_SCAN_DONE_INT_RAW_M  (RTC_CNTL_TOUCH_SCAN_DONE_INT_RAW_V << RTC_CNTL_TOUCH_SCAN_DONE_INT_RAW_S)
#define RTC_CNTL_TOUCH_SCAN_DONE_INT_RAW_V  0x00000001
#define RTC_CNTL_TOUCH_SCAN_DONE_INT_RAW_S  4

/* RTC_CNTL_WDT_INT_RAW : RO; bitpos: [3]; default: 0;
 * RTC WDT interrupt raw
 */

#define RTC_CNTL_WDT_INT_RAW    (BIT(3))
#define RTC_CNTL_WDT_INT_RAW_M  (RTC_CNTL_WDT_INT_RAW_V << RTC_CNTL_WDT_INT_RAW_S)
#define RTC_CNTL_WDT_INT_RAW_V  0x00000001
#define RTC_CNTL_WDT_INT_RAW_S  3

/* RTC_CNTL_SDIO_IDLE_INT_RAW : RO; bitpos: [2]; default: 0;
 * SDIO idle interrupt raw
 */

#define RTC_CNTL_SDIO_IDLE_INT_RAW    (BIT(2))
#define RTC_CNTL_SDIO_IDLE_INT_RAW_M  (RTC_CNTL_SDIO_IDLE_INT_RAW_V << RTC_CNTL_SDIO_IDLE_INT_RAW_S)
#define RTC_CNTL_SDIO_IDLE_INT_RAW_V  0x00000001
#define RTC_CNTL_SDIO_IDLE_INT_RAW_S  2

/* RTC_CNTL_SLP_REJECT_INT_RAW : RO; bitpos: [1]; default: 0;
 * sleep reject interrupt raw
 */

#define RTC_CNTL_SLP_REJECT_INT_RAW    (BIT(1))
#define RTC_CNTL_SLP_REJECT_INT_RAW_M  (RTC_CNTL_SLP_REJECT_INT_RAW_V << RTC_CNTL_SLP_REJECT_INT_RAW_S)
#define RTC_CNTL_SLP_REJECT_INT_RAW_V  0x00000001
#define RTC_CNTL_SLP_REJECT_INT_RAW_S  1

/* RTC_CNTL_SLP_WAKEUP_INT_RAW : RO; bitpos: [0]; default: 0;
 * sleep wakeup interrupt raw
 */

#define RTC_CNTL_SLP_WAKEUP_INT_RAW    (BIT(0))
#define RTC_CNTL_SLP_WAKEUP_INT_RAW_M  (RTC_CNTL_SLP_WAKEUP_INT_RAW_V << RTC_CNTL_SLP_WAKEUP_INT_RAW_S)
#define RTC_CNTL_SLP_WAKEUP_INT_RAW_V  0x00000001
#define RTC_CNTL_SLP_WAKEUP_INT_RAW_S  0

/* RTC_CNTL_INT_ST_RTC_REG register
 * rtc_interrupt state register
 */

#define RTC_CNTL_INT_ST_RTC_REG (DR_REG_RTCCNTL_BASE + 0x48)

/* RTC_CNTL_GLITCH_DET_INT_ST : RO; bitpos: [19]; default: 0;
 * glitch_det_interrupt state
 */

#define RTC_CNTL_GLITCH_DET_INT_ST    (BIT(19))
#define RTC_CNTL_GLITCH_DET_INT_ST_M  (RTC_CNTL_GLITCH_DET_INT_ST_V << RTC_CNTL_GLITCH_DET_INT_ST_S)
#define RTC_CNTL_GLITCH_DET_INT_ST_V  0x00000001
#define RTC_CNTL_GLITCH_DET_INT_ST_S  19

/* RTC_CNTL_TOUCH_TIMEOUT_INT_ST : RO; bitpos: [18]; default: 0;
 * Touch timeout interrupt state
 */

#define RTC_CNTL_TOUCH_TIMEOUT_INT_ST    (BIT(18))
#define RTC_CNTL_TOUCH_TIMEOUT_INT_ST_M  (RTC_CNTL_TOUCH_TIMEOUT_INT_ST_V << RTC_CNTL_TOUCH_TIMEOUT_INT_ST_S)
#define RTC_CNTL_TOUCH_TIMEOUT_INT_ST_V  0x00000001
#define RTC_CNTL_TOUCH_TIMEOUT_INT_ST_S  18

/* RTC_CNTL_COCPU_TRAP_INT_ST : RO; bitpos: [17]; default: 0;
 * cocpu trap interrupt state
 */

#define RTC_CNTL_COCPU_TRAP_INT_ST    (BIT(17))
#define RTC_CNTL_COCPU_TRAP_INT_ST_M  (RTC_CNTL_COCPU_TRAP_INT_ST_V << RTC_CNTL_COCPU_TRAP_INT_ST_S)
#define RTC_CNTL_COCPU_TRAP_INT_ST_V  0x00000001
#define RTC_CNTL_COCPU_TRAP_INT_ST_S  17

/* RTC_CNTL_XTAL32K_DEAD_INT_ST : RO; bitpos: [16]; default: 0;
 * xtal32k dead detection interrupt state
 */

#define RTC_CNTL_XTAL32K_DEAD_INT_ST    (BIT(16))
#define RTC_CNTL_XTAL32K_DEAD_INT_ST_M  (RTC_CNTL_XTAL32K_DEAD_INT_ST_V << RTC_CNTL_XTAL32K_DEAD_INT_ST_S)
#define RTC_CNTL_XTAL32K_DEAD_INT_ST_V  0x00000001
#define RTC_CNTL_XTAL32K_DEAD_INT_ST_S  16

/* RTC_CNTL_SWD_INT_ST : RO; bitpos: [15]; default: 0;
 * super watch dog interrupt state
 */

#define RTC_CNTL_SWD_INT_ST    (BIT(15))
#define RTC_CNTL_SWD_INT_ST_M  (RTC_CNTL_SWD_INT_ST_V << RTC_CNTL_SWD_INT_ST_S)
#define RTC_CNTL_SWD_INT_ST_V  0x00000001
#define RTC_CNTL_SWD_INT_ST_S  15

/* RTC_CNTL_SARADC2_INT_ST : RO; bitpos: [14]; default: 0;
 * saradc2 interrupt state
 */

#define RTC_CNTL_SARADC2_INT_ST    (BIT(14))
#define RTC_CNTL_SARADC2_INT_ST_M  (RTC_CNTL_SARADC2_INT_ST_V << RTC_CNTL_SARADC2_INT_ST_S)
#define RTC_CNTL_SARADC2_INT_ST_V  0x00000001
#define RTC_CNTL_SARADC2_INT_ST_S  14

/* RTC_CNTL_COCPU_INT_ST : RO; bitpos: [13]; default: 0;
 * riscV cocpu interrupt state
 */

#define RTC_CNTL_COCPU_INT_ST    (BIT(13))
#define RTC_CNTL_COCPU_INT_ST_M  (RTC_CNTL_COCPU_INT_ST_V << RTC_CNTL_COCPU_INT_ST_S)
#define RTC_CNTL_COCPU_INT_ST_V  0x00000001
#define RTC_CNTL_COCPU_INT_ST_S  13

/* RTC_CNTL_TSENS_INT_ST : RO; bitpos: [12]; default: 0;
 * tsens interrupt state
 */

#define RTC_CNTL_TSENS_INT_ST    (BIT(12))
#define RTC_CNTL_TSENS_INT_ST_M  (RTC_CNTL_TSENS_INT_ST_V << RTC_CNTL_TSENS_INT_ST_S)
#define RTC_CNTL_TSENS_INT_ST_V  0x00000001
#define RTC_CNTL_TSENS_INT_ST_S  12

/* RTC_CNTL_SARADC1_INT_ST : RO; bitpos: [11]; default: 0;
 * saradc1 interrupt state
 */

#define RTC_CNTL_SARADC1_INT_ST    (BIT(11))
#define RTC_CNTL_SARADC1_INT_ST_M  (RTC_CNTL_SARADC1_INT_ST_V << RTC_CNTL_SARADC1_INT_ST_S)
#define RTC_CNTL_SARADC1_INT_ST_V  0x00000001
#define RTC_CNTL_SARADC1_INT_ST_S  11

/* RTC_CNTL_MAIN_TIMER_INT_ST : RO; bitpos: [10]; default: 0;
 * RTC main timer interrupt state
 */

#define RTC_CNTL_MAIN_TIMER_INT_ST    (BIT(10))
#define RTC_CNTL_MAIN_TIMER_INT_ST_M  (RTC_CNTL_MAIN_TIMER_INT_ST_V << RTC_CNTL_MAIN_TIMER_INT_ST_S)
#define RTC_CNTL_MAIN_TIMER_INT_ST_V  0x00000001
#define RTC_CNTL_MAIN_TIMER_INT_ST_S  10

/* RTC_CNTL_BROWN_OUT_INT_ST : RO; bitpos: [9]; default: 0;
 * brown out interrupt state
 */

#define RTC_CNTL_BROWN_OUT_INT_ST    (BIT(9))
#define RTC_CNTL_BROWN_OUT_INT_ST_M  (RTC_CNTL_BROWN_OUT_INT_ST_V << RTC_CNTL_BROWN_OUT_INT_ST_S)
#define RTC_CNTL_BROWN_OUT_INT_ST_V  0x00000001
#define RTC_CNTL_BROWN_OUT_INT_ST_S  9

/* RTC_CNTL_TOUCH_INACTIVE_INT_ST : RO; bitpos: [8]; default: 0;
 * touch inactive interrupt state
 */

#define RTC_CNTL_TOUCH_INACTIVE_INT_ST    (BIT(8))
#define RTC_CNTL_TOUCH_INACTIVE_INT_ST_M  (RTC_CNTL_TOUCH_INACTIVE_INT_ST_V << RTC_CNTL_TOUCH_INACTIVE_INT_ST_S)
#define RTC_CNTL_TOUCH_INACTIVE_INT_ST_V  0x00000001
#define RTC_CNTL_TOUCH_INACTIVE_INT_ST_S  8

/* RTC_CNTL_TOUCH_ACTIVE_INT_ST : RO; bitpos: [7]; default: 0;
 * touch active interrupt state
 */

#define RTC_CNTL_TOUCH_ACTIVE_INT_ST    (BIT(7))
#define RTC_CNTL_TOUCH_ACTIVE_INT_ST_M  (RTC_CNTL_TOUCH_ACTIVE_INT_ST_V << RTC_CNTL_TOUCH_ACTIVE_INT_ST_S)
#define RTC_CNTL_TOUCH_ACTIVE_INT_ST_V  0x00000001
#define RTC_CNTL_TOUCH_ACTIVE_INT_ST_S  7

/* RTC_CNTL_TOUCH_DONE_INT_ST : RO; bitpos: [6]; default: 0;
 * touch done interrupt state
 */

#define RTC_CNTL_TOUCH_DONE_INT_ST    (BIT(6))
#define RTC_CNTL_TOUCH_DONE_INT_ST_M  (RTC_CNTL_TOUCH_DONE_INT_ST_V << RTC_CNTL_TOUCH_DONE_INT_ST_S)
#define RTC_CNTL_TOUCH_DONE_INT_ST_V  0x00000001
#define RTC_CNTL_TOUCH_DONE_INT_ST_S  6

/* RTC_CNTL_ULP_CP_INT_ST : RO; bitpos: [5]; default: 0;
 * ULP-coprocessor interrupt state
 */

#define RTC_CNTL_ULP_CP_INT_ST    (BIT(5))
#define RTC_CNTL_ULP_CP_INT_ST_M  (RTC_CNTL_ULP_CP_INT_ST_V << RTC_CNTL_ULP_CP_INT_ST_S)
#define RTC_CNTL_ULP_CP_INT_ST_V  0x00000001
#define RTC_CNTL_ULP_CP_INT_ST_S  5

/* RTC_CNTL_TOUCH_SCAN_DONE_INT_ST : RO; bitpos: [4]; default: 0;
 * touch complete a loop interrupt state
 */

#define RTC_CNTL_TOUCH_SCAN_DONE_INT_ST    (BIT(4))
#define RTC_CNTL_TOUCH_SCAN_DONE_INT_ST_M  (RTC_CNTL_TOUCH_SCAN_DONE_INT_ST_V << RTC_CNTL_TOUCH_SCAN_DONE_INT_ST_S)
#define RTC_CNTL_TOUCH_SCAN_DONE_INT_ST_V  0x00000001
#define RTC_CNTL_TOUCH_SCAN_DONE_INT_ST_S  4

/* RTC_CNTL_WDT_INT_ST : RO; bitpos: [3]; default: 0;
 * RTC WDT interrupt state
 */

#define RTC_CNTL_WDT_INT_ST    (BIT(3))
#define RTC_CNTL_WDT_INT_ST_M  (RTC_CNTL_WDT_INT_ST_V << RTC_CNTL_WDT_INT_ST_S)
#define RTC_CNTL_WDT_INT_ST_V  0x00000001
#define RTC_CNTL_WDT_INT_ST_S  3

/* RTC_CNTL_SDIO_IDLE_INT_ST : RO; bitpos: [2]; default: 0;
 * SDIO idle interrupt state
 */

#define RTC_CNTL_SDIO_IDLE_INT_ST    (BIT(2))
#define RTC_CNTL_SDIO_IDLE_INT_ST_M  (RTC_CNTL_SDIO_IDLE_INT_ST_V << RTC_CNTL_SDIO_IDLE_INT_ST_S)
#define RTC_CNTL_SDIO_IDLE_INT_ST_V  0x00000001
#define RTC_CNTL_SDIO_IDLE_INT_ST_S  2

/* RTC_CNTL_SLP_REJECT_INT_ST : RO; bitpos: [1]; default: 0;
 * sleep reject interrupt state
 */

#define RTC_CNTL_SLP_REJECT_INT_ST    (BIT(1))
#define RTC_CNTL_SLP_REJECT_INT_ST_M  (RTC_CNTL_SLP_REJECT_INT_ST_V << RTC_CNTL_SLP_REJECT_INT_ST_S)
#define RTC_CNTL_SLP_REJECT_INT_ST_V  0x00000001
#define RTC_CNTL_SLP_REJECT_INT_ST_S  1

/* RTC_CNTL_SLP_WAKEUP_INT_ST : RO; bitpos: [0]; default: 0;
 * sleep wakeup interrupt state
 */

#define RTC_CNTL_SLP_WAKEUP_INT_ST    (BIT(0))
#define RTC_CNTL_SLP_WAKEUP_INT_ST_M  (RTC_CNTL_SLP_WAKEUP_INT_ST_V << RTC_CNTL_SLP_WAKEUP_INT_ST_S)
#define RTC_CNTL_SLP_WAKEUP_INT_ST_V  0x00000001
#define RTC_CNTL_SLP_WAKEUP_INT_ST_S  0

/* RTC_CNTL_INT_CLR_RTC_REG register
 * Clear rtc_interrupt register
 */

#define RTC_CNTL_INT_CLR_RTC_REG (DR_REG_RTCCNTL_BASE + 0x4c)

/* RTC_CNTL_GLITCH_DET_INT_CLR : WO; bitpos: [19]; default: 0;
 * Clear glitch det interrupt state
 */

#define RTC_CNTL_GLITCH_DET_INT_CLR    (BIT(19))
#define RTC_CNTL_GLITCH_DET_INT_CLR_M  (RTC_CNTL_GLITCH_DET_INT_CLR_V << RTC_CNTL_GLITCH_DET_INT_CLR_S)
#define RTC_CNTL_GLITCH_DET_INT_CLR_V  0x00000001
#define RTC_CNTL_GLITCH_DET_INT_CLR_S  19

/* RTC_CNTL_TOUCH_TIMEOUT_INT_CLR : WO; bitpos: [18]; default: 0;
 * Clear touch timeout interrupt state
 */

#define RTC_CNTL_TOUCH_TIMEOUT_INT_CLR    (BIT(18))
#define RTC_CNTL_TOUCH_TIMEOUT_INT_CLR_M  (RTC_CNTL_TOUCH_TIMEOUT_INT_CLR_V << RTC_CNTL_TOUCH_TIMEOUT_INT_CLR_S)
#define RTC_CNTL_TOUCH_TIMEOUT_INT_CLR_V  0x00000001
#define RTC_CNTL_TOUCH_TIMEOUT_INT_CLR_S  18

/* RTC_CNTL_COCPU_TRAP_INT_CLR : WO; bitpos: [17]; default: 0;
 * Clear cocpu trap interrupt state
 */

#define RTC_CNTL_COCPU_TRAP_INT_CLR    (BIT(17))
#define RTC_CNTL_COCPU_TRAP_INT_CLR_M  (RTC_CNTL_COCPU_TRAP_INT_CLR_V << RTC_CNTL_COCPU_TRAP_INT_CLR_S)
#define RTC_CNTL_COCPU_TRAP_INT_CLR_V  0x00000001
#define RTC_CNTL_COCPU_TRAP_INT_CLR_S  17

/* RTC_CNTL_XTAL32K_DEAD_INT_CLR : WO; bitpos: [16]; default: 0;
 * Clear RTC WDT interrupt state
 */

#define RTC_CNTL_XTAL32K_DEAD_INT_CLR    (BIT(16))
#define RTC_CNTL_XTAL32K_DEAD_INT_CLR_M  (RTC_CNTL_XTAL32K_DEAD_INT_CLR_V << RTC_CNTL_XTAL32K_DEAD_INT_CLR_S)
#define RTC_CNTL_XTAL32K_DEAD_INT_CLR_V  0x00000001
#define RTC_CNTL_XTAL32K_DEAD_INT_CLR_S  16

/* RTC_CNTL_SWD_INT_CLR : WO; bitpos: [15]; default: 0;
 * Clear super watch dog interrupt state
 */

#define RTC_CNTL_SWD_INT_CLR    (BIT(15))
#define RTC_CNTL_SWD_INT_CLR_M  (RTC_CNTL_SWD_INT_CLR_V << RTC_CNTL_SWD_INT_CLR_S)
#define RTC_CNTL_SWD_INT_CLR_V  0x00000001
#define RTC_CNTL_SWD_INT_CLR_S  15

/* RTC_CNTL_SARADC2_INT_CLR : WO; bitpos: [14]; default: 0;
 * Clear saradc2 interrupt state
 */

#define RTC_CNTL_SARADC2_INT_CLR    (BIT(14))
#define RTC_CNTL_SARADC2_INT_CLR_M  (RTC_CNTL_SARADC2_INT_CLR_V << RTC_CNTL_SARADC2_INT_CLR_S)
#define RTC_CNTL_SARADC2_INT_CLR_V  0x00000001
#define RTC_CNTL_SARADC2_INT_CLR_S  14

/* RTC_CNTL_COCPU_INT_CLR : WO; bitpos: [13]; default: 0;
 * Clear riscV cocpu interrupt state
 */

#define RTC_CNTL_COCPU_INT_CLR    (BIT(13))
#define RTC_CNTL_COCPU_INT_CLR_M  (RTC_CNTL_COCPU_INT_CLR_V << RTC_CNTL_COCPU_INT_CLR_S)
#define RTC_CNTL_COCPU_INT_CLR_V  0x00000001
#define RTC_CNTL_COCPU_INT_CLR_S  13

/* RTC_CNTL_TSENS_INT_CLR : WO; bitpos: [12]; default: 0;
 * Clear tsens interrupt state
 */

#define RTC_CNTL_TSENS_INT_CLR    (BIT(12))
#define RTC_CNTL_TSENS_INT_CLR_M  (RTC_CNTL_TSENS_INT_CLR_V << RTC_CNTL_TSENS_INT_CLR_S)
#define RTC_CNTL_TSENS_INT_CLR_V  0x00000001
#define RTC_CNTL_TSENS_INT_CLR_S  12

/* RTC_CNTL_SARADC1_INT_CLR : WO; bitpos: [11]; default: 0;
 * Clear saradc1 interrupt state
 */

#define RTC_CNTL_SARADC1_INT_CLR    (BIT(11))
#define RTC_CNTL_SARADC1_INT_CLR_M  (RTC_CNTL_SARADC1_INT_CLR_V << RTC_CNTL_SARADC1_INT_CLR_S)
#define RTC_CNTL_SARADC1_INT_CLR_V  0x00000001
#define RTC_CNTL_SARADC1_INT_CLR_S  11

/* RTC_CNTL_MAIN_TIMER_INT_CLR : WO; bitpos: [10]; default: 0;
 * Clear RTC main timer interrupt state
 */

#define RTC_CNTL_MAIN_TIMER_INT_CLR    (BIT(10))
#define RTC_CNTL_MAIN_TIMER_INT_CLR_M  (RTC_CNTL_MAIN_TIMER_INT_CLR_V << RTC_CNTL_MAIN_TIMER_INT_CLR_S)
#define RTC_CNTL_MAIN_TIMER_INT_CLR_V  0x00000001
#define RTC_CNTL_MAIN_TIMER_INT_CLR_S  10

/* RTC_CNTL_BROWN_OUT_INT_CLR : WO; bitpos: [9]; default: 0;
 * Clear brown out interrupt state
 */

#define RTC_CNTL_BROWN_OUT_INT_CLR    (BIT(9))
#define RTC_CNTL_BROWN_OUT_INT_CLR_M  (RTC_CNTL_BROWN_OUT_INT_CLR_V << RTC_CNTL_BROWN_OUT_INT_CLR_S)
#define RTC_CNTL_BROWN_OUT_INT_CLR_V  0x00000001
#define RTC_CNTL_BROWN_OUT_INT_CLR_S  9

/* RTC_CNTL_TOUCH_INACTIVE_INT_CLR : WO; bitpos: [8]; default: 0;
 * Clear touch inactive interrupt state
 */

#define RTC_CNTL_TOUCH_INACTIVE_INT_CLR    (BIT(8))
#define RTC_CNTL_TOUCH_INACTIVE_INT_CLR_M  (RTC_CNTL_TOUCH_INACTIVE_INT_CLR_V << RTC_CNTL_TOUCH_INACTIVE_INT_CLR_S)
#define RTC_CNTL_TOUCH_INACTIVE_INT_CLR_V  0x00000001
#define RTC_CNTL_TOUCH_INACTIVE_INT_CLR_S  8

/* RTC_CNTL_TOUCH_ACTIVE_INT_CLR : WO; bitpos: [7]; default: 0;
 * Clear touch active interrupt state
 */

#define RTC_CNTL_TOUCH_ACTIVE_INT_CLR    (BIT(7))
#define RTC_CNTL_TOUCH_ACTIVE_INT_CLR_M  (RTC_CNTL_TOUCH_ACTIVE_INT_CLR_V << RTC_CNTL_TOUCH_ACTIVE_INT_CLR_S)
#define RTC_CNTL_TOUCH_ACTIVE_INT_CLR_V  0x00000001
#define RTC_CNTL_TOUCH_ACTIVE_INT_CLR_S  7

/* RTC_CNTL_TOUCH_DONE_INT_CLR : WO; bitpos: [6]; default: 0;
 * Clear touch done interrupt state
 */

#define RTC_CNTL_TOUCH_DONE_INT_CLR    (BIT(6))
#define RTC_CNTL_TOUCH_DONE_INT_CLR_M  (RTC_CNTL_TOUCH_DONE_INT_CLR_V << RTC_CNTL_TOUCH_DONE_INT_CLR_S)
#define RTC_CNTL_TOUCH_DONE_INT_CLR_V  0x00000001
#define RTC_CNTL_TOUCH_DONE_INT_CLR_S  6

/* RTC_CNTL_ULP_CP_INT_CLR : WO; bitpos: [5]; default: 0;
 * Clear ULP-coprocessor interrupt state
 */

#define RTC_CNTL_ULP_CP_INT_CLR    (BIT(5))
#define RTC_CNTL_ULP_CP_INT_CLR_M  (RTC_CNTL_ULP_CP_INT_CLR_V << RTC_CNTL_ULP_CP_INT_CLR_S)
#define RTC_CNTL_ULP_CP_INT_CLR_V  0x00000001
#define RTC_CNTL_ULP_CP_INT_CLR_S  5

/* RTC_CNTL_TOUCH_SCAN_DONE_INT_CLR : WO; bitpos: [4]; default: 0;
 * Clear touch complete a loop interrupt state
 */

#define RTC_CNTL_TOUCH_SCAN_DONE_INT_CLR    (BIT(4))
#define RTC_CNTL_TOUCH_SCAN_DONE_INT_CLR_M  (RTC_CNTL_TOUCH_SCAN_DONE_INT_CLR_V << RTC_CNTL_TOUCH_SCAN_DONE_INT_CLR_S)
#define RTC_CNTL_TOUCH_SCAN_DONE_INT_CLR_V  0x00000001
#define RTC_CNTL_TOUCH_SCAN_DONE_INT_CLR_S  4

/* RTC_CNTL_WDT_INT_CLR : WO; bitpos: [3]; default: 0;
 * Clear RTC WDT interrupt state
 */

#define RTC_CNTL_WDT_INT_CLR    (BIT(3))
#define RTC_CNTL_WDT_INT_CLR_M  (RTC_CNTL_WDT_INT_CLR_V << RTC_CNTL_WDT_INT_CLR_S)
#define RTC_CNTL_WDT_INT_CLR_V  0x00000001
#define RTC_CNTL_WDT_INT_CLR_S  3

/* RTC_CNTL_SDIO_IDLE_INT_CLR : WO; bitpos: [2]; default: 0;
 * Clear SDIO idle interrupt state
 */

#define RTC_CNTL_SDIO_IDLE_INT_CLR    (BIT(2))
#define RTC_CNTL_SDIO_IDLE_INT_CLR_M  (RTC_CNTL_SDIO_IDLE_INT_CLR_V << RTC_CNTL_SDIO_IDLE_INT_CLR_S)
#define RTC_CNTL_SDIO_IDLE_INT_CLR_V  0x00000001
#define RTC_CNTL_SDIO_IDLE_INT_CLR_S  2

/* RTC_CNTL_SLP_REJECT_INT_CLR : WO; bitpos: [1]; default: 0;
 * Clear sleep reject interrupt state
 */

#define RTC_CNTL_SLP_REJECT_INT_CLR    (BIT(1))
#define RTC_CNTL_SLP_REJECT_INT_CLR_M  (RTC_CNTL_SLP_REJECT_INT_CLR_V << RTC_CNTL_SLP_REJECT_INT_CLR_S)
#define RTC_CNTL_SLP_REJECT_INT_CLR_V  0x00000001
#define RTC_CNTL_SLP_REJECT_INT_CLR_S  1

/* RTC_CNTL_SLP_WAKEUP_INT_CLR : WO; bitpos: [0]; default: 0;
 * Clear sleep wakeup interrupt state
 */

#define RTC_CNTL_SLP_WAKEUP_INT_CLR    (BIT(0))
#define RTC_CNTL_SLP_WAKEUP_INT_CLR_M  (RTC_CNTL_SLP_WAKEUP_INT_CLR_V << RTC_CNTL_SLP_WAKEUP_INT_CLR_S)
#define RTC_CNTL_SLP_WAKEUP_INT_CLR_V  0x00000001
#define RTC_CNTL_SLP_WAKEUP_INT_CLR_S  0

/* RTC_CNTL_STORE0_REG register
 * reservation register0
 */

#define RTC_CNTL_STORE0_REG (DR_REG_RTCCNTL_BASE + 0x50)

/* RTC_CNTL_SCRATCH0 : R/W; bitpos: [31:0]; default: 0;
 * reservation register0
 */

#define RTC_CNTL_SCRATCH0    0xFFFFFFFF
#define RTC_CNTL_SCRATCH0_M  (RTC_CNTL_SCRATCH0_V << RTC_CNTL_SCRATCH0_S)
#define RTC_CNTL_SCRATCH0_V  0xFFFFFFFF
#define RTC_CNTL_SCRATCH0_S  0

/* RTC_CNTL_STORE1_REG register
 * reservation register1
 */

#define RTC_CNTL_STORE1_REG (DR_REG_RTCCNTL_BASE + 0x54)

#define RTC_SLOW_CLK_CAL_REG    RTC_CNTL_STORE1_REG

/* RTC_CNTL_SCRATCH1 : R/W; bitpos: [31:0]; default: 0;
 * reservation register1
 */

#define RTC_CNTL_SCRATCH1    0xFFFFFFFF
#define RTC_CNTL_SCRATCH1_M  (RTC_CNTL_SCRATCH1_V << RTC_CNTL_SCRATCH1_S)
#define RTC_CNTL_SCRATCH1_V  0xFFFFFFFF
#define RTC_CNTL_SCRATCH1_S  0

/* RTC_CNTL_STORE2_REG register
 * reservation register2
 */

#define RTC_CNTL_STORE2_REG (DR_REG_RTCCNTL_BASE + 0x58)

/* RTC_CNTL_SCRATCH2 : R/W; bitpos: [31:0]; default: 0;
 * reservation register2
 */

#define RTC_CNTL_SCRATCH2    0xFFFFFFFF
#define RTC_CNTL_SCRATCH2_M  (RTC_CNTL_SCRATCH2_V << RTC_CNTL_SCRATCH2_S)
#define RTC_CNTL_SCRATCH2_V  0xFFFFFFFF
#define RTC_CNTL_SCRATCH2_S  0

/* RTC_CNTL_STORE3_REG register
 * reservation register3
 */

#define RTC_CNTL_STORE3_REG (DR_REG_RTCCNTL_BASE + 0x5c)

/* RTC_CNTL_SCRATCH3 : R/W; bitpos: [31:0]; default: 0;
 * reservation register3
 */

#define RTC_CNTL_SCRATCH3    0xFFFFFFFF
#define RTC_CNTL_SCRATCH3_M  (RTC_CNTL_SCRATCH3_V << RTC_CNTL_SCRATCH3_S)
#define RTC_CNTL_SCRATCH3_V  0xFFFFFFFF
#define RTC_CNTL_SCRATCH3_S  0

/* RTC_CNTL_EXT_XTL_CONF_REG register
 * configure 32k xtal register
 */

#define RTC_CNTL_EXT_XTL_CONF_REG (DR_REG_RTCCNTL_BASE + 0x60)

/* RTC_CNTL_XTL_EXT_CTR_EN : R/W; bitpos: [31]; default: 0;
 * enable gpio power down XTAL
 */

#define RTC_CNTL_XTL_EXT_CTR_EN    (BIT(31))
#define RTC_CNTL_XTL_EXT_CTR_EN_M  (RTC_CNTL_XTL_EXT_CTR_EN_V << RTC_CNTL_XTL_EXT_CTR_EN_S)
#define RTC_CNTL_XTL_EXT_CTR_EN_V  0x00000001
#define RTC_CNTL_XTL_EXT_CTR_EN_S  31

/* RTC_CNTL_XTL_EXT_CTR_LV : R/W; bitpos: [30]; default: 0;
 * 0: power down XTAL at high level     1: power down XTAL at low level
 */

#define RTC_CNTL_XTL_EXT_CTR_LV    (BIT(30))
#define RTC_CNTL_XTL_EXT_CTR_LV_M  (RTC_CNTL_XTL_EXT_CTR_LV_V << RTC_CNTL_XTL_EXT_CTR_LV_S)
#define RTC_CNTL_XTL_EXT_CTR_LV_V  0x00000001
#define RTC_CNTL_XTL_EXT_CTR_LV_S  30

/* RTC_CNTL_XTAL32K_GPIO_SEL : R/W; bitpos: [23]; default: 0;
 * XTAL_32K sel. 0: external XTAL_32K    1: CLK from RTC pad X32P_C
 */

#define RTC_CNTL_XTAL32K_GPIO_SEL    (BIT(23))
#define RTC_CNTL_XTAL32K_GPIO_SEL_M  (RTC_CNTL_XTAL32K_GPIO_SEL_V << RTC_CNTL_XTAL32K_GPIO_SEL_S)
#define RTC_CNTL_XTAL32K_GPIO_SEL_V  0x00000001
#define RTC_CNTL_XTAL32K_GPIO_SEL_S  23

/* RTC_CNTL_WDT_STATE : RO; bitpos: [22:20]; default: 0;
 * state of 32k_wdt
 */

#define RTC_CNTL_WDT_STATE    0x00000007
#define RTC_CNTL_WDT_STATE_M  (RTC_CNTL_WDT_STATE_V << RTC_CNTL_WDT_STATE_S)
#define RTC_CNTL_WDT_STATE_V  0x00000007
#define RTC_CNTL_WDT_STATE_S  20

/* RTC_CNTL_DAC_XTAL_32K : R/W; bitpos: [19:17]; default: 3;
 * DAC_XTAL_32K
 */

#define RTC_CNTL_DAC_XTAL_32K    0x00000007
#define RTC_CNTL_DAC_XTAL_32K_M  (RTC_CNTL_DAC_XTAL_32K_V << RTC_CNTL_DAC_XTAL_32K_S)
#define RTC_CNTL_DAC_XTAL_32K_V  0x00000007
#define RTC_CNTL_DAC_XTAL_32K_S  17

/* RTC_CNTL_XPD_XTAL_32K : R/W; bitpos: [16]; default: 0;
 * XPD_XTAL_32K
 */

#define RTC_CNTL_XPD_XTAL_32K    (BIT(16))
#define RTC_CNTL_XPD_XTAL_32K_M  (RTC_CNTL_XPD_XTAL_32K_V << RTC_CNTL_XPD_XTAL_32K_S)
#define RTC_CNTL_XPD_XTAL_32K_V  0x00000001
#define RTC_CNTL_XPD_XTAL_32K_S  16

/* RTC_CNTL_DRES_XTAL_32K : R/W; bitpos: [15:13]; default: 3;
 * DRES_XTAL_32K
 */

#define RTC_CNTL_DRES_XTAL_32K    0x00000007
#define RTC_CNTL_DRES_XTAL_32K_M  (RTC_CNTL_DRES_XTAL_32K_V << RTC_CNTL_DRES_XTAL_32K_S)
#define RTC_CNTL_DRES_XTAL_32K_V  0x00000007
#define RTC_CNTL_DRES_XTAL_32K_S  13

/* RTC_CNTL_DGM_XTAL_32K : R/W; bitpos: [12:10]; default: 3;
 * xtal_32k gm control
 */

#define RTC_CNTL_DGM_XTAL_32K    0x00000007
#define RTC_CNTL_DGM_XTAL_32K_M  (RTC_CNTL_DGM_XTAL_32K_V << RTC_CNTL_DGM_XTAL_32K_S)
#define RTC_CNTL_DGM_XTAL_32K_V  0x00000007
#define RTC_CNTL_DGM_XTAL_32K_S  10

/* RTC_CNTL_DBUF_XTAL_32K : R/W; bitpos: [9]; default: 0;
 * 0: single-end buffer 1: differential buffer
 */

#define RTC_CNTL_DBUF_XTAL_32K    (BIT(9))
#define RTC_CNTL_DBUF_XTAL_32K_M  (RTC_CNTL_DBUF_XTAL_32K_V << RTC_CNTL_DBUF_XTAL_32K_S)
#define RTC_CNTL_DBUF_XTAL_32K_V  0x00000001
#define RTC_CNTL_DBUF_XTAL_32K_S  9

/* RTC_CNTL_ENCKINIT_XTAL_32K : R/W; bitpos: [8]; default: 0;
 * apply an internal clock to help xtal 32k to start
 */

#define RTC_CNTL_ENCKINIT_XTAL_32K    (BIT(8))
#define RTC_CNTL_ENCKINIT_XTAL_32K_M  (RTC_CNTL_ENCKINIT_XTAL_32K_V << RTC_CNTL_ENCKINIT_XTAL_32K_S)
#define RTC_CNTL_ENCKINIT_XTAL_32K_V  0x00000001
#define RTC_CNTL_ENCKINIT_XTAL_32K_S  8

/* RTC_CNTL_XTAL32K_XPD_FORCE : R/W; bitpos: [7]; default: 1;
 * Xtal 32k xpd control by sw or fsm
 */

#define RTC_CNTL_XTAL32K_XPD_FORCE    (BIT(7))
#define RTC_CNTL_XTAL32K_XPD_FORCE_M  (RTC_CNTL_XTAL32K_XPD_FORCE_V << RTC_CNTL_XTAL32K_XPD_FORCE_S)
#define RTC_CNTL_XTAL32K_XPD_FORCE_V  0x00000001
#define RTC_CNTL_XTAL32K_XPD_FORCE_S  7

/* RTC_CNTL_XTAL32K_AUTO_RETURN : R/W; bitpos: [6]; default: 0;
 * xtal 32k switch back xtal when xtal is restarted
 */

#define RTC_CNTL_XTAL32K_AUTO_RETURN    (BIT(6))
#define RTC_CNTL_XTAL32K_AUTO_RETURN_M  (RTC_CNTL_XTAL32K_AUTO_RETURN_V << RTC_CNTL_XTAL32K_AUTO_RETURN_S)
#define RTC_CNTL_XTAL32K_AUTO_RETURN_V  0x00000001
#define RTC_CNTL_XTAL32K_AUTO_RETURN_S  6

/* RTC_CNTL_XTAL32K_AUTO_RESTART : R/W; bitpos: [5]; default: 0;
 * xtal 32k restart xtal when xtal is dead
 */

#define RTC_CNTL_XTAL32K_AUTO_RESTART    (BIT(5))
#define RTC_CNTL_XTAL32K_AUTO_RESTART_M  (RTC_CNTL_XTAL32K_AUTO_RESTART_V << RTC_CNTL_XTAL32K_AUTO_RESTART_S)
#define RTC_CNTL_XTAL32K_AUTO_RESTART_V  0x00000001
#define RTC_CNTL_XTAL32K_AUTO_RESTART_S  5

/* RTC_CNTL_XTAL32K_AUTO_BACKUP : R/W; bitpos: [4]; default: 0;
 * xtal 32k switch to back up clock when xtal is dead
 */

#define RTC_CNTL_XTAL32K_AUTO_BACKUP    (BIT(4))
#define RTC_CNTL_XTAL32K_AUTO_BACKUP_M  (RTC_CNTL_XTAL32K_AUTO_BACKUP_V << RTC_CNTL_XTAL32K_AUTO_BACKUP_S)
#define RTC_CNTL_XTAL32K_AUTO_BACKUP_V  0x00000001
#define RTC_CNTL_XTAL32K_AUTO_BACKUP_S  4

/* RTC_CNTL_XTAL32K_EXT_CLK_FO : R/W; bitpos: [3]; default: 0;
 * xtal 32k external xtal clock force on
 */

#define RTC_CNTL_XTAL32K_EXT_CLK_FO    (BIT(3))
#define RTC_CNTL_XTAL32K_EXT_CLK_FO_M  (RTC_CNTL_XTAL32K_EXT_CLK_FO_V << RTC_CNTL_XTAL32K_EXT_CLK_FO_S)
#define RTC_CNTL_XTAL32K_EXT_CLK_FO_V  0x00000001
#define RTC_CNTL_XTAL32K_EXT_CLK_FO_S  3

/* RTC_CNTL_XTAL32K_WDT_RESET : R/W; bitpos: [2]; default: 0;
 * xtal 32k watch dog sw reset
 */

#define RTC_CNTL_XTAL32K_WDT_RESET    (BIT(2))
#define RTC_CNTL_XTAL32K_WDT_RESET_M  (RTC_CNTL_XTAL32K_WDT_RESET_V << RTC_CNTL_XTAL32K_WDT_RESET_S)
#define RTC_CNTL_XTAL32K_WDT_RESET_V  0x00000001
#define RTC_CNTL_XTAL32K_WDT_RESET_S  2

/* RTC_CNTL_XTAL32K_WDT_CLK_FO : R/W; bitpos: [1]; default: 0;
 * xtal 32k watch dog clock force on
 */

#define RTC_CNTL_XTAL32K_WDT_CLK_FO    (BIT(1))
#define RTC_CNTL_XTAL32K_WDT_CLK_FO_M  (RTC_CNTL_XTAL32K_WDT_CLK_FO_V << RTC_CNTL_XTAL32K_WDT_CLK_FO_S)
#define RTC_CNTL_XTAL32K_WDT_CLK_FO_V  0x00000001
#define RTC_CNTL_XTAL32K_WDT_CLK_FO_S  1

/* RTC_CNTL_XTAL32K_WDT_EN : R/W; bitpos: [0]; default: 0;
 * xtal 32k watch dog enable
 */

#define RTC_CNTL_XTAL32K_WDT_EN    (BIT(0))
#define RTC_CNTL_XTAL32K_WDT_EN_M  (RTC_CNTL_XTAL32K_WDT_EN_V << RTC_CNTL_XTAL32K_WDT_EN_S)
#define RTC_CNTL_XTAL32K_WDT_EN_V  0x00000001
#define RTC_CNTL_XTAL32K_WDT_EN_S  0

/* RTC_CNTL_EXT_WAKEUP_CONF_REG register
 * configure gpio wakeup register
 */

#define RTC_CNTL_EXT_WAKEUP_CONF_REG (DR_REG_RTCCNTL_BASE + 0x64)

/* RTC_CNTL_EXT_WAKEUP1_LV : R/W; bitpos: [31]; default: 0;
 * 0: external wakeup at low level     1: external wakeup at high level
 */

#define RTC_CNTL_EXT_WAKEUP1_LV    (BIT(31))
#define RTC_CNTL_EXT_WAKEUP1_LV_M  (RTC_CNTL_EXT_WAKEUP1_LV_V << RTC_CNTL_EXT_WAKEUP1_LV_S)
#define RTC_CNTL_EXT_WAKEUP1_LV_V  0x00000001
#define RTC_CNTL_EXT_WAKEUP1_LV_S  31

/* RTC_CNTL_EXT_WAKEUP0_LV : R/W; bitpos: [30]; default: 0;
 * 0: external wakeup at low level     1: external wakeup at high level
 */

#define RTC_CNTL_EXT_WAKEUP0_LV    (BIT(30))
#define RTC_CNTL_EXT_WAKEUP0_LV_M  (RTC_CNTL_EXT_WAKEUP0_LV_V << RTC_CNTL_EXT_WAKEUP0_LV_S)
#define RTC_CNTL_EXT_WAKEUP0_LV_V  0x00000001
#define RTC_CNTL_EXT_WAKEUP0_LV_S  30

/* RTC_CNTL_GPIO_WAKEUP_FILTER : R/W; bitpos: [29]; default: 0;
 * enable filter for gpio wakeup event
 */

#define RTC_CNTL_GPIO_WAKEUP_FILTER    (BIT(29))
#define RTC_CNTL_GPIO_WAKEUP_FILTER_M  (RTC_CNTL_GPIO_WAKEUP_FILTER_V << RTC_CNTL_GPIO_WAKEUP_FILTER_S)
#define RTC_CNTL_GPIO_WAKEUP_FILTER_V  0x00000001
#define RTC_CNTL_GPIO_WAKEUP_FILTER_S  29

/* RTC_CNTL_SLP_REJECT_CONF_REG register
 * configure sleep reject register
 */

#define RTC_CNTL_SLP_REJECT_CONF_REG (DR_REG_RTCCNTL_BASE + 0x68)

/* RTC_CNTL_DEEP_SLP_REJECT_EN : R/W; bitpos: [31]; default: 0;
 * enable reject for deep sleep
 */

#define RTC_CNTL_DEEP_SLP_REJECT_EN    (BIT(31))
#define RTC_CNTL_DEEP_SLP_REJECT_EN_M  (RTC_CNTL_DEEP_SLP_REJECT_EN_V << RTC_CNTL_DEEP_SLP_REJECT_EN_S)
#define RTC_CNTL_DEEP_SLP_REJECT_EN_V  0x00000001
#define RTC_CNTL_DEEP_SLP_REJECT_EN_S  31

/* RTC_CNTL_LIGHT_SLP_REJECT_EN : R/W; bitpos: [30]; default: 0;
 * enable reject for light sleep
 */

#define RTC_CNTL_LIGHT_SLP_REJECT_EN    (BIT(30))
#define RTC_CNTL_LIGHT_SLP_REJECT_EN_M  (RTC_CNTL_LIGHT_SLP_REJECT_EN_V << RTC_CNTL_LIGHT_SLP_REJECT_EN_S)
#define RTC_CNTL_LIGHT_SLP_REJECT_EN_V  0x00000001
#define RTC_CNTL_LIGHT_SLP_REJECT_EN_S  30

/* RTC_CNTL_SLEEP_REJECT_ENA : R/W; bitpos: [29:13]; default: 0;
 * sleep reject enable
 */

#define RTC_CNTL_SLEEP_REJECT_ENA    0x0001FFFF
#define RTC_CNTL_SLEEP_REJECT_ENA_M  (RTC_CNTL_SLEEP_REJECT_ENA_V << RTC_CNTL_SLEEP_REJECT_ENA_S)
#define RTC_CNTL_SLEEP_REJECT_ENA_V  0x0001FFFF
#define RTC_CNTL_SLEEP_REJECT_ENA_S  13

/* RTC_CNTL_CPU_PERIOD_CONF_REG register
 * CPU sel option
 */

#define RTC_CNTL_CPU_PERIOD_CONF_REG (DR_REG_RTCCNTL_BASE + 0x6c)

/* RTC_CNTL_CPUPERIOD_SEL : R/W; bitpos: [31:30]; default: 0; */

#define RTC_CNTL_CPUPERIOD_SEL    0x00000003
#define RTC_CNTL_CPUPERIOD_SEL_M  (RTC_CNTL_CPUPERIOD_SEL_V << RTC_CNTL_CPUPERIOD_SEL_S)
#define RTC_CNTL_CPUPERIOD_SEL_V  0x00000003
#define RTC_CNTL_CPUPERIOD_SEL_S  30

/* RTC_CNTL_CPUSEL_CONF : R/W; bitpos: [29]; default: 0;
 * CPU sel option
 */

#define RTC_CNTL_CPUSEL_CONF    (BIT(29))
#define RTC_CNTL_CPUSEL_CONF_M  (RTC_CNTL_CPUSEL_CONF_V << RTC_CNTL_CPUSEL_CONF_S)
#define RTC_CNTL_CPUSEL_CONF_V  0x00000001
#define RTC_CNTL_CPUSEL_CONF_S  29

/* RTC_CNTL_SDIO_ACT_CONF_REG register
 * configure sdio active register
 */

#define RTC_CNTL_SDIO_ACT_CONF_REG (DR_REG_RTCCNTL_BASE + 0x70)

/* RTC_CNTL_SDIO_ACT_DNUM : R/W; bitpos: [31:22]; default: 0;
 * configure sdio act dnum
 */

#define RTC_CNTL_SDIO_ACT_DNUM    0x000003FF
#define RTC_CNTL_SDIO_ACT_DNUM_M  (RTC_CNTL_SDIO_ACT_DNUM_V << RTC_CNTL_SDIO_ACT_DNUM_S)
#define RTC_CNTL_SDIO_ACT_DNUM_V  0x000003FF
#define RTC_CNTL_SDIO_ACT_DNUM_S  22

/* RTC_CNTL_CLK_CONF_REG register
 * configure rtc clk register
 */

#define RTC_CNTL_CLK_CONF_REG (DR_REG_RTCCNTL_BASE + 0x74)

/* RTC_CNTL_ANA_CLK_RTC_SEL : R/W; bitpos: [31:30]; default: 0;
 * slow clk sel 0 : 90K rtc_clk   1 : 32k XTAL  2 : 8md256
 */

#define RTC_CNTL_ANA_CLK_RTC_SEL    0x00000003
#define RTC_CNTL_ANA_CLK_RTC_SEL_M  (RTC_CNTL_ANA_CLK_RTC_SEL_V << RTC_CNTL_ANA_CLK_RTC_SEL_S)
#define RTC_CNTL_ANA_CLK_RTC_SEL_V  0x00000003
#define RTC_CNTL_ANA_CLK_RTC_SEL_S  30

/* RTC_CNTL_FAST_CLK_RTC_SEL : R/W; bitpos: [29]; default: 0;
 * fast_clk_rtc sel. 0: XTAL div 4    1: CK8M
 */

#define RTC_CNTL_FAST_CLK_RTC_SEL    (BIT(29))
#define RTC_CNTL_FAST_CLK_RTC_SEL_M  (RTC_CNTL_FAST_CLK_RTC_SEL_V << RTC_CNTL_FAST_CLK_RTC_SEL_S)
#define RTC_CNTL_FAST_CLK_RTC_SEL_V  0x00000001
#define RTC_CNTL_FAST_CLK_RTC_SEL_S  29

/* RTC_CNTL_CK8M_FORCE_PU : R/W; bitpos: [26]; default: 0;
 * CK8M force power up
 */

#define RTC_CNTL_CK8M_FORCE_PU    (BIT(26))
#define RTC_CNTL_CK8M_FORCE_PU_M  (RTC_CNTL_CK8M_FORCE_PU_V << RTC_CNTL_CK8M_FORCE_PU_S)
#define RTC_CNTL_CK8M_FORCE_PU_V  0x00000001
#define RTC_CNTL_CK8M_FORCE_PU_S  26

/* RTC_CNTL_CK8M_FORCE_PD : R/W; bitpos: [25]; default: 0;
 * CK8M force power down
 */

#define RTC_CNTL_CK8M_FORCE_PD    (BIT(25))
#define RTC_CNTL_CK8M_FORCE_PD_M  (RTC_CNTL_CK8M_FORCE_PD_V << RTC_CNTL_CK8M_FORCE_PD_S)
#define RTC_CNTL_CK8M_FORCE_PD_V  0x00000001
#define RTC_CNTL_CK8M_FORCE_PD_S  25

/* RTC_CNTL_CK8M_DFREQ : R/W; bitpos: [24:17]; default: 172;
 * CK8M_DFREQ
 */

#define RTC_CNTL_CK8M_DFREQ    0x000000FF
#define RTC_CNTL_CK8M_DFREQ_M  (RTC_CNTL_CK8M_DFREQ_V << RTC_CNTL_CK8M_DFREQ_S)
#define RTC_CNTL_CK8M_DFREQ_V  0x000000FF
#define RTC_CNTL_CK8M_DFREQ_S  17

/* RTC_CNTL_CK8M_FORCE_NOGATING : R/W; bitpos: [16]; default: 0;
 * CK8M force no gating during sleep
 */

#define RTC_CNTL_CK8M_FORCE_NOGATING    (BIT(16))
#define RTC_CNTL_CK8M_FORCE_NOGATING_M  (RTC_CNTL_CK8M_FORCE_NOGATING_V << RTC_CNTL_CK8M_FORCE_NOGATING_S)
#define RTC_CNTL_CK8M_FORCE_NOGATING_V  0x00000001
#define RTC_CNTL_CK8M_FORCE_NOGATING_S  16

/* RTC_CNTL_XTAL_FORCE_NOGATING : R/W; bitpos: [15]; default: 0;
 * XTAL force no gating during sleep
 */

#define RTC_CNTL_XTAL_FORCE_NOGATING    (BIT(15))
#define RTC_CNTL_XTAL_FORCE_NOGATING_M  (RTC_CNTL_XTAL_FORCE_NOGATING_V << RTC_CNTL_XTAL_FORCE_NOGATING_S)
#define RTC_CNTL_XTAL_FORCE_NOGATING_V  0x00000001
#define RTC_CNTL_XTAL_FORCE_NOGATING_S  15

/* RTC_CNTL_CK8M_DIV_SEL : R/W; bitpos: [14:12]; default: 3;
 * divider = reg_ck8m_div_sel + 1
 */

#define RTC_CNTL_CK8M_DIV_SEL    0x00000007
#define RTC_CNTL_CK8M_DIV_SEL_M  (RTC_CNTL_CK8M_DIV_SEL_V << RTC_CNTL_CK8M_DIV_SEL_S)
#define RTC_CNTL_CK8M_DIV_SEL_V  0x00000007
#define RTC_CNTL_CK8M_DIV_SEL_S  12

/* RTC_CNTL_DIG_CLK8M_EN : R/W; bitpos: [10]; default: 0;
 * enable CK8M for digital core (no relationship with RTC core)
 */

#define RTC_CNTL_DIG_CLK8M_EN    (BIT(10))
#define RTC_CNTL_DIG_CLK8M_EN_M  (RTC_CNTL_DIG_CLK8M_EN_V << RTC_CNTL_DIG_CLK8M_EN_S)
#define RTC_CNTL_DIG_CLK8M_EN_V  0x00000001
#define RTC_CNTL_DIG_CLK8M_EN_S  10

/* RTC_CNTL_DIG_CLK8M_D256_EN : R/W; bitpos: [9]; default: 1;
 * enable CK8M_D256_OUT for digital core (no relationship with RTC core)
 */

#define RTC_CNTL_DIG_CLK8M_D256_EN    (BIT(9))
#define RTC_CNTL_DIG_CLK8M_D256_EN_M  (RTC_CNTL_DIG_CLK8M_D256_EN_V << RTC_CNTL_DIG_CLK8M_D256_EN_S)
#define RTC_CNTL_DIG_CLK8M_D256_EN_V  0x00000001
#define RTC_CNTL_DIG_CLK8M_D256_EN_S  9

/* RTC_CNTL_DIG_XTAL32K_EN : R/W; bitpos: [8]; default: 0;
 * enable CK_XTAL_32K for digital core (no relationship with RTC core)
 */

#define RTC_CNTL_DIG_XTAL32K_EN    (BIT(8))
#define RTC_CNTL_DIG_XTAL32K_EN_M  (RTC_CNTL_DIG_XTAL32K_EN_V << RTC_CNTL_DIG_XTAL32K_EN_S)
#define RTC_CNTL_DIG_XTAL32K_EN_V  0x00000001
#define RTC_CNTL_DIG_XTAL32K_EN_S  8

/* RTC_CNTL_ENB_CK8M_DIV : R/W; bitpos: [7]; default: 0;
 * 1: CK8M_D256_OUT is actually CK8M    0: CK8M_D256_OUT is CK8M divided by
 * 256
 */

#define RTC_CNTL_ENB_CK8M_DIV    (BIT(7))
#define RTC_CNTL_ENB_CK8M_DIV_M  (RTC_CNTL_ENB_CK8M_DIV_V << RTC_CNTL_ENB_CK8M_DIV_S)
#define RTC_CNTL_ENB_CK8M_DIV_V  0x00000001
#define RTC_CNTL_ENB_CK8M_DIV_S  7

/* RTC_CNTL_ENB_CK8M : R/W; bitpos: [6]; default: 0;
 * disable CK8M and CK8M_D256_OUT
 */

#define RTC_CNTL_ENB_CK8M    (BIT(6))
#define RTC_CNTL_ENB_CK8M_M  (RTC_CNTL_ENB_CK8M_V << RTC_CNTL_ENB_CK8M_S)
#define RTC_CNTL_ENB_CK8M_V  0x00000001
#define RTC_CNTL_ENB_CK8M_S  6

/* RTC_CNTL_CK8M_DIV : R/W; bitpos: [5:4]; default: 1;
 * CK8M_D256_OUT divider. 00: div128   01: div256   10: div512    11:
 * div1024.
 */

#define RTC_CNTL_CK8M_DIV    0x00000003
#define RTC_CNTL_CK8M_DIV_M  (RTC_CNTL_CK8M_DIV_V << RTC_CNTL_CK8M_DIV_S)
#define RTC_CNTL_CK8M_DIV_V  0x00000003
#define RTC_CNTL_CK8M_DIV_S  4

/* RTC_CNTL_CK8M_DIV_SEL_VLD : R/W; bitpos: [3]; default: 1;
 * used to sync reg_ck8m_div_sel bus. Clear vld before set reg_ck8m_div_sel
 * then set vld to actually switch the clk
 */

#define RTC_CNTL_CK8M_DIV_SEL_VLD    (BIT(3))
#define RTC_CNTL_CK8M_DIV_SEL_VLD_M  (RTC_CNTL_CK8M_DIV_SEL_VLD_V << RTC_CNTL_CK8M_DIV_SEL_VLD_S)
#define RTC_CNTL_CK8M_DIV_SEL_VLD_V  0x00000001
#define RTC_CNTL_CK8M_DIV_SEL_VLD_S  3

/* RTC_CNTL_SLOW_CLK_CONF_REG register
 * configure rtc slow clk register
 */

#define RTC_CNTL_SLOW_CLK_CONF_REG (DR_REG_RTCCNTL_BASE + 0x78)

/* RTC_CNTL_SLOW_CLK_NEXT_EDGE : R/W; bitpos: [31]; default: 0; */

#define RTC_CNTL_SLOW_CLK_NEXT_EDGE    (BIT(31))
#define RTC_CNTL_SLOW_CLK_NEXT_EDGE_M  (RTC_CNTL_SLOW_CLK_NEXT_EDGE_V << RTC_CNTL_SLOW_CLK_NEXT_EDGE_S)
#define RTC_CNTL_SLOW_CLK_NEXT_EDGE_V  0x00000001
#define RTC_CNTL_SLOW_CLK_NEXT_EDGE_S  31

/* RTC_CNTL_ANA_CLK_DIV : R/W; bitpos: [30:23]; default: 0;
 * rtc_clk divider
 */

#define RTC_CNTL_ANA_CLK_DIV    0x000000FF
#define RTC_CNTL_ANA_CLK_DIV_M  (RTC_CNTL_ANA_CLK_DIV_V << RTC_CNTL_ANA_CLK_DIV_S)
#define RTC_CNTL_ANA_CLK_DIV_V  0x000000FF
#define RTC_CNTL_ANA_CLK_DIV_S  23

/* RTC_CNTL_ANA_CLK_DIV_VLD : R/W; bitpos: [22]; default: 1;
 * used to sync div bus. clear vld before set reg_rtc_ana_clk_div   then set
 * vld to actually switch the clk
 */

#define RTC_CNTL_ANA_CLK_DIV_VLD    (BIT(22))
#define RTC_CNTL_ANA_CLK_DIV_VLD_M  (RTC_CNTL_ANA_CLK_DIV_VLD_V << RTC_CNTL_ANA_CLK_DIV_VLD_S)
#define RTC_CNTL_ANA_CLK_DIV_VLD_V  0x00000001
#define RTC_CNTL_ANA_CLK_DIV_VLD_S  22

/* RTC_CNTL_SDIO_CONF_REG register
 * configure vddsdio register
 */

#define RTC_CNTL_SDIO_CONF_REG (DR_REG_RTCCNTL_BASE + 0x7c)

/* RTC_CNTL_XPD_SDIO_REG : R/W; bitpos: [31]; default: 0;
 * SW option for XPD_VOOSDIO. Only active when reg_sdio_force = 1
 */

#define RTC_CNTL_XPD_SDIO_REG    (BIT(31))
#define RTC_CNTL_XPD_SDIO_REG_M  (RTC_CNTL_XPD_SDIO_REG_V << RTC_CNTL_XPD_SDIO_REG_S)
#define RTC_CNTL_XPD_SDIO_REG_V  0x00000001
#define RTC_CNTL_XPD_SDIO_REG_S  31

/* RTC_CNTL_DREFH_SDIO : R/W; bitpos: [30:29]; default: 0;
 * SW option for DREFH_SDIO. Only active when reg_sdio_force = 1
 */

#define RTC_CNTL_DREFH_SDIO    0x00000003
#define RTC_CNTL_DREFH_SDIO_M  (RTC_CNTL_DREFH_SDIO_V << RTC_CNTL_DREFH_SDIO_S)
#define RTC_CNTL_DREFH_SDIO_V  0x00000003
#define RTC_CNTL_DREFH_SDIO_S  29

/* RTC_CNTL_DREFM_SDIO : R/W; bitpos: [28:27]; default: 0;
 * SW option for DREFM_SDIO. Only active when reg_sdio_force = 1
 */

#define RTC_CNTL_DREFM_SDIO    0x00000003
#define RTC_CNTL_DREFM_SDIO_M  (RTC_CNTL_DREFM_SDIO_V << RTC_CNTL_DREFM_SDIO_S)
#define RTC_CNTL_DREFM_SDIO_V  0x00000003
#define RTC_CNTL_DREFM_SDIO_S  27

/* RTC_CNTL_DREFL_SDIO : R/W; bitpos: [26:25]; default: 1;
 * SW option for DREFL_SDIO. Only active when reg_sdio_force = 1
 */

#define RTC_CNTL_DREFL_SDIO    0x00000003
#define RTC_CNTL_DREFL_SDIO_M  (RTC_CNTL_DREFL_SDIO_V << RTC_CNTL_DREFL_SDIO_S)
#define RTC_CNTL_DREFL_SDIO_V  0x00000003
#define RTC_CNTL_DREFL_SDIO_S  25

/* RTC_CNTL_REG1P8_READY : RO; bitpos: [24]; default: 0;
 * read only register for REG1P8_READY
 */

#define RTC_CNTL_REG1P8_READY    (BIT(24))
#define RTC_CNTL_REG1P8_READY_M  (RTC_CNTL_REG1P8_READY_V << RTC_CNTL_REG1P8_READY_S)
#define RTC_CNTL_REG1P8_READY_V  0x00000001
#define RTC_CNTL_REG1P8_READY_S  24

/* RTC_CNTL_SDIO_TIEH : R/W; bitpos: [23]; default: 1;
 * SW option for SDIO_TIEH. Only active when reg_sdio_force = 1
 */

#define RTC_CNTL_SDIO_TIEH    (BIT(23))
#define RTC_CNTL_SDIO_TIEH_M  (RTC_CNTL_SDIO_TIEH_V << RTC_CNTL_SDIO_TIEH_S)
#define RTC_CNTL_SDIO_TIEH_V  0x00000001
#define RTC_CNTL_SDIO_TIEH_S  23

/* RTC_CNTL_SDIO_FORCE : R/W; bitpos: [22]; default: 0;
 * 1: use SW option to control SDIO_REG   0: use state machine
 */

#define RTC_CNTL_SDIO_FORCE    (BIT(22))
#define RTC_CNTL_SDIO_FORCE_M  (RTC_CNTL_SDIO_FORCE_V << RTC_CNTL_SDIO_FORCE_S)
#define RTC_CNTL_SDIO_FORCE_V  0x00000001
#define RTC_CNTL_SDIO_FORCE_S  22

/* RTC_CNTL_SDIO_REG_PD_EN : R/W; bitpos: [21]; default: 1;
 * power down SDIO_REG in sleep. Only active when reg_sdio_force = 0
 */

#define RTC_CNTL_SDIO_REG_PD_EN    (BIT(21))
#define RTC_CNTL_SDIO_REG_PD_EN_M  (RTC_CNTL_SDIO_REG_PD_EN_V << RTC_CNTL_SDIO_REG_PD_EN_S)
#define RTC_CNTL_SDIO_REG_PD_EN_V  0x00000001
#define RTC_CNTL_SDIO_REG_PD_EN_S  21

/* RTC_CNTL_SDIO_ENCURLIM : R/W; bitpos: [20]; default: 1;
 * enable current limit
 */

#define RTC_CNTL_SDIO_ENCURLIM    (BIT(20))
#define RTC_CNTL_SDIO_ENCURLIM_M  (RTC_CNTL_SDIO_ENCURLIM_V << RTC_CNTL_SDIO_ENCURLIM_S)
#define RTC_CNTL_SDIO_ENCURLIM_V  0x00000001
#define RTC_CNTL_SDIO_ENCURLIM_S  20

/* RTC_CNTL_SDIO_MODECURLIM : R/W; bitpos: [19]; default: 0;
 * select current limit mode
 */

#define RTC_CNTL_SDIO_MODECURLIM    (BIT(19))
#define RTC_CNTL_SDIO_MODECURLIM_M  (RTC_CNTL_SDIO_MODECURLIM_V << RTC_CNTL_SDIO_MODECURLIM_S)
#define RTC_CNTL_SDIO_MODECURLIM_V  0x00000001
#define RTC_CNTL_SDIO_MODECURLIM_S  19

/* RTC_CNTL_SDIO_DCURLIM : R/W; bitpos: [18:16]; default: 0;
 * tune current limit threshold when tieh = 0. About 800mA/(8+d)
 */

#define RTC_CNTL_SDIO_DCURLIM    0x00000007
#define RTC_CNTL_SDIO_DCURLIM_M  (RTC_CNTL_SDIO_DCURLIM_V << RTC_CNTL_SDIO_DCURLIM_S)
#define RTC_CNTL_SDIO_DCURLIM_V  0x00000007
#define RTC_CNTL_SDIO_DCURLIM_S  16

/* RTC_CNTL_SDIO_EN_INITI : R/W; bitpos: [15]; default: 1;
 * 0 to set init[1:0]=0
 */

#define RTC_CNTL_SDIO_EN_INITI    (BIT(15))
#define RTC_CNTL_SDIO_EN_INITI_M  (RTC_CNTL_SDIO_EN_INITI_V << RTC_CNTL_SDIO_EN_INITI_S)
#define RTC_CNTL_SDIO_EN_INITI_V  0x00000001
#define RTC_CNTL_SDIO_EN_INITI_S  15

/* RTC_CNTL_SDIO_INITI : R/W; bitpos: [14:13]; default: 1;
 * add resistor from ldo output to ground. 0: no res  1: 6k  2: 4k   3: 2k
 */

#define RTC_CNTL_SDIO_INITI    0x00000003
#define RTC_CNTL_SDIO_INITI_M  (RTC_CNTL_SDIO_INITI_V << RTC_CNTL_SDIO_INITI_S)
#define RTC_CNTL_SDIO_INITI_V  0x00000003
#define RTC_CNTL_SDIO_INITI_S  13

/* RTC_CNTL_SDIO_DCAP : R/W; bitpos: [12:11]; default: 3;
 * ability to prevent LDO from overshoot
 */

#define RTC_CNTL_SDIO_DCAP    0x00000003
#define RTC_CNTL_SDIO_DCAP_M  (RTC_CNTL_SDIO_DCAP_V << RTC_CNTL_SDIO_DCAP_S)
#define RTC_CNTL_SDIO_DCAP_V  0x00000003
#define RTC_CNTL_SDIO_DCAP_S  11

/* RTC_CNTL_SDIO_DTHDRV : R/W; bitpos: [10:9]; default: 3;
 * Tieh = 1 mode drive ability. Initially set to 0 to limit charge current
 * set to 3 after several us.
 */

#define RTC_CNTL_SDIO_DTHDRV    0x00000003
#define RTC_CNTL_SDIO_DTHDRV_M  (RTC_CNTL_SDIO_DTHDRV_V << RTC_CNTL_SDIO_DTHDRV_S)
#define RTC_CNTL_SDIO_DTHDRV_V  0x00000003
#define RTC_CNTL_SDIO_DTHDRV_S  9

/* RTC_CNTL_SDIO_TIMER_TARGET : R/W; bitpos: [7:0]; default: 10;
 * timer count to apply reg_sdio_dcap after sdio power on
 */

#define RTC_CNTL_SDIO_TIMER_TARGET    0x000000FF
#define RTC_CNTL_SDIO_TIMER_TARGET_M  (RTC_CNTL_SDIO_TIMER_TARGET_V << RTC_CNTL_SDIO_TIMER_TARGET_S)
#define RTC_CNTL_SDIO_TIMER_TARGET_V  0x000000FF
#define RTC_CNTL_SDIO_TIMER_TARGET_S  0

/* RTC_CNTL_BIAS_CONF_REG register
 * configure power register
 */

#define RTC_CNTL_BIAS_CONF_REG (DR_REG_RTCCNTL_BASE + 0x80)

/* RTC_CNTL_RST_BIAS_I2C : R/W; bitpos: [31]; default: 0; */

#define RTC_CNTL_RST_BIAS_I2C    (BIT(31))
#define RTC_CNTL_RST_BIAS_I2C_M  (RTC_CNTL_RST_BIAS_I2C_V << RTC_CNTL_RST_BIAS_I2C_S)
#define RTC_CNTL_RST_BIAS_I2C_V  0x00000001
#define RTC_CNTL_RST_BIAS_I2C_S  31

/* RTC_CNTL_DEC_HEARTBEAT_WIDTH : R/W; bitpos: [30]; default: 0;
 * DEC_HEARTBEAT_WIDTH
 */

#define RTC_CNTL_DEC_HEARTBEAT_WIDTH    (BIT(30))
#define RTC_CNTL_DEC_HEARTBEAT_WIDTH_M  (RTC_CNTL_DEC_HEARTBEAT_WIDTH_V << RTC_CNTL_DEC_HEARTBEAT_WIDTH_S)
#define RTC_CNTL_DEC_HEARTBEAT_WIDTH_V  0x00000001
#define RTC_CNTL_DEC_HEARTBEAT_WIDTH_S  30

/* RTC_CNTL_INC_HEARTBEAT_PERIOD : R/W; bitpos: [29]; default: 0;
 * INC_HEARTBEAT_PERIOD
 */

#define RTC_CNTL_INC_HEARTBEAT_PERIOD    (BIT(29))
#define RTC_CNTL_INC_HEARTBEAT_PERIOD_M  (RTC_CNTL_INC_HEARTBEAT_PERIOD_V << RTC_CNTL_INC_HEARTBEAT_PERIOD_S)
#define RTC_CNTL_INC_HEARTBEAT_PERIOD_V  0x00000001
#define RTC_CNTL_INC_HEARTBEAT_PERIOD_S  29

/* RTC_CNTL_DEC_HEARTBEAT_PERIOD : R/W; bitpos: [28]; default: 0;
 * DEC_HEARTBEAT_PERIOD
 */

#define RTC_CNTL_DEC_HEARTBEAT_PERIOD    (BIT(28))
#define RTC_CNTL_DEC_HEARTBEAT_PERIOD_M  (RTC_CNTL_DEC_HEARTBEAT_PERIOD_V << RTC_CNTL_DEC_HEARTBEAT_PERIOD_S)
#define RTC_CNTL_DEC_HEARTBEAT_PERIOD_V  0x00000001
#define RTC_CNTL_DEC_HEARTBEAT_PERIOD_S  28

/* RTC_CNTL_INC_HEARTBEAT_REFRESH : R/W; bitpos: [27]; default: 0;
 * INC_HEARTBEAT_REFRESH
 */

#define RTC_CNTL_INC_HEARTBEAT_REFRESH    (BIT(27))
#define RTC_CNTL_INC_HEARTBEAT_REFRESH_M  (RTC_CNTL_INC_HEARTBEAT_REFRESH_V << RTC_CNTL_INC_HEARTBEAT_REFRESH_S)
#define RTC_CNTL_INC_HEARTBEAT_REFRESH_V  0x00000001
#define RTC_CNTL_INC_HEARTBEAT_REFRESH_S  27

/* RTC_CNTL_ENB_SCK_XTAL : R/W; bitpos: [26]; default: 0;
 * ENB_SCK_XTAL
 */

#define RTC_CNTL_ENB_SCK_XTAL    (BIT(26))
#define RTC_CNTL_ENB_SCK_XTAL_M  (RTC_CNTL_ENB_SCK_XTAL_V << RTC_CNTL_ENB_SCK_XTAL_S)
#define RTC_CNTL_ENB_SCK_XTAL_V  0x00000001
#define RTC_CNTL_ENB_SCK_XTAL_S  26

/* RTC_CNTL_DBG_ATTEN_MONITOR : R/W; bitpos: [25:22]; default: 0;
 * DBG_ATTEN when rtc in monitor state
 */

#define RTC_CNTL_DBG_ATTEN_MONITOR    0x0000000F
#define RTC_CNTL_DBG_ATTEN_MONITOR_M  (RTC_CNTL_DBG_ATTEN_MONITOR_V << RTC_CNTL_DBG_ATTEN_MONITOR_S)
#define RTC_CNTL_DBG_ATTEN_MONITOR_V  0x0000000F
#define RTC_CNTL_DBG_ATTEN_MONITOR_S  22

/* RTC_CNTL_DBG_ATTEN_DEEP_SLP : R/W; bitpos: [21:18]; default: 0;
 * DBG_ATTEN when rtc in sleep state
 */

#define RTC_CNTL_DBG_ATTEN_DEEP_SLP    0x0000000F
#define RTC_CNTL_DBG_ATTEN_DEEP_SLP_M  (RTC_CNTL_DBG_ATTEN_DEEP_SLP_V << RTC_CNTL_DBG_ATTEN_DEEP_SLP_S)
#define RTC_CNTL_DBG_ATTEN_DEEP_SLP_V  0x0000000F
#define RTC_CNTL_DBG_ATTEN_DEEP_SLP_S  18

/* RTC_CNTL_BIAS_SLEEP_MONITOR : R/W; bitpos: [17]; default: 0;
 * bias_sleep when rtc in monitor state
 */

#define RTC_CNTL_BIAS_SLEEP_MONITOR    (BIT(17))
#define RTC_CNTL_BIAS_SLEEP_MONITOR_M  (RTC_CNTL_BIAS_SLEEP_MONITOR_V << RTC_CNTL_BIAS_SLEEP_MONITOR_S)
#define RTC_CNTL_BIAS_SLEEP_MONITOR_V  0x00000001
#define RTC_CNTL_BIAS_SLEEP_MONITOR_S  17

/* RTC_CNTL_BIAS_SLEEP_DEEP_SLP : R/W; bitpos: [16]; default: 1;
 * bias_sleep when rtc in sleep_state
 */

#define RTC_CNTL_BIAS_SLEEP_DEEP_SLP    (BIT(16))
#define RTC_CNTL_BIAS_SLEEP_DEEP_SLP_M  (RTC_CNTL_BIAS_SLEEP_DEEP_SLP_V << RTC_CNTL_BIAS_SLEEP_DEEP_SLP_S)
#define RTC_CNTL_BIAS_SLEEP_DEEP_SLP_V  0x00000001
#define RTC_CNTL_BIAS_SLEEP_DEEP_SLP_S  16

/* RTC_CNTL_PD_CUR_MONITOR : R/W; bitpos: [15]; default: 0;
 * xpd cur when rtc in monitor state
 */

#define RTC_CNTL_PD_CUR_MONITOR    (BIT(15))
#define RTC_CNTL_PD_CUR_MONITOR_M  (RTC_CNTL_PD_CUR_MONITOR_V << RTC_CNTL_PD_CUR_MONITOR_S)
#define RTC_CNTL_PD_CUR_MONITOR_V  0x00000001
#define RTC_CNTL_PD_CUR_MONITOR_S  15

/* RTC_CNTL_PD_CUR_DEEP_SLP : R/W; bitpos: [14]; default: 0;
 * xpd cur when rtc in sleep_state
 */

#define RTC_CNTL_PD_CUR_DEEP_SLP    (BIT(14))
#define RTC_CNTL_PD_CUR_DEEP_SLP_M  (RTC_CNTL_PD_CUR_DEEP_SLP_V << RTC_CNTL_PD_CUR_DEEP_SLP_S)
#define RTC_CNTL_PD_CUR_DEEP_SLP_V  0x00000001
#define RTC_CNTL_PD_CUR_DEEP_SLP_S  14

/* RTC_CNTL_BIAS_BUF_MONITOR : R/W; bitpos: [13]; default: 0;
 * open bias buf when rtc in monitor state
 */

#define RTC_CNTL_BIAS_BUF_MONITOR    (BIT(13))
#define RTC_CNTL_BIAS_BUF_MONITOR_M  (RTC_CNTL_BIAS_BUF_MONITOR_V << RTC_CNTL_BIAS_BUF_MONITOR_S)
#define RTC_CNTL_BIAS_BUF_MONITOR_V  0x00000001
#define RTC_CNTL_BIAS_BUF_MONITOR_S  13

/* RTC_CNTL_BIAS_BUF_DEEP_SLP : R/W; bitpos: [12]; default: 0;
 * open bias buf when rtc in deep sleep
 */

#define RTC_CNTL_BIAS_BUF_DEEP_SLP    (BIT(12))
#define RTC_CNTL_BIAS_BUF_DEEP_SLP_M  (RTC_CNTL_BIAS_BUF_DEEP_SLP_V << RTC_CNTL_BIAS_BUF_DEEP_SLP_S)
#define RTC_CNTL_BIAS_BUF_DEEP_SLP_V  0x00000001
#define RTC_CNTL_BIAS_BUF_DEEP_SLP_S  12

/* RTC_CNTL_BIAS_BUF_WAKE : R/W; bitpos: [11]; default: 1;
 * open bias buf when rtc in wakeup
 */

#define RTC_CNTL_BIAS_BUF_WAKE    (BIT(11))
#define RTC_CNTL_BIAS_BUF_WAKE_M  (RTC_CNTL_BIAS_BUF_WAKE_V << RTC_CNTL_BIAS_BUF_WAKE_S)
#define RTC_CNTL_BIAS_BUF_WAKE_V  0x00000001
#define RTC_CNTL_BIAS_BUF_WAKE_S  11

/* RTC_CNTL_BIAS_BUF_IDLE : R/W; bitpos: [10]; default: 0;
 * open bias buf when system in active
 */

#define RTC_CNTL_BIAS_BUF_IDLE    (BIT(10))
#define RTC_CNTL_BIAS_BUF_IDLE_M  (RTC_CNTL_BIAS_BUF_IDLE_V << RTC_CNTL_BIAS_BUF_IDLE_S)
#define RTC_CNTL_BIAS_BUF_IDLE_V  0x00000001
#define RTC_CNTL_BIAS_BUF_IDLE_S  10

/* RTC_CNTL_REG register
 * configure rtc/dig regulator register
 */
#define RTC_CNTL_DIG_DBIAS_0V85  0
#define RTC_CNTL_DIG_DBIAS_0V90  1
#define RTC_CNTL_DIG_DBIAS_0V95  2
#define RTC_CNTL_DIG_DBIAS_1V00  3
#define RTC_CNTL_DIG_DBIAS_1V05  4
#define RTC_CNTL_DIG_DBIAS_1V10  5
#define RTC_CNTL_DIG_DBIAS_1V15  6
#define RTC_CNTL_DIG_DBIAS_1V20  7

#define RTC_CNTL_REG (DR_REG_RTCCNTL_BASE + 0x84)

/* RTC_CNTL_REGULATOR_FORCE_PU : R/W; bitpos: [31]; default: 1;
 * RTC_REG force power pu
 */

#define RTC_CNTL_REGULATOR_FORCE_PU    (BIT(31))
#define RTC_CNTL_REGULATOR_FORCE_PU_M  (RTC_CNTL_REGULATOR_FORCE_PU_V << RTC_CNTL_REGULATOR_FORCE_PU_S)
#define RTC_CNTL_REGULATOR_FORCE_PU_V  0x00000001
#define RTC_CNTL_REGULATOR_FORCE_PU_S  31

/* RTC_CNTL_REGULATOR_FORCE_PD : R/W; bitpos: [30]; default: 0;
 * RTC_REG force power down (for RTC_REG power down means decrease the
 * voltage to 0.8v or lower )
 */

#define RTC_CNTL_REGULATOR_FORCE_PD    (BIT(30))
#define RTC_CNTL_REGULATOR_FORCE_PD_M  (RTC_CNTL_REGULATOR_FORCE_PD_V << RTC_CNTL_REGULATOR_FORCE_PD_S)
#define RTC_CNTL_REGULATOR_FORCE_PD_V  0x00000001
#define RTC_CNTL_REGULATOR_FORCE_PD_S  30

/* RTC_CNTL_DBOOST_FORCE_PU : R/W; bitpos: [29]; default: 1;
 * RTC_DBOOST force power up
 */

#define RTC_CNTL_DBOOST_FORCE_PU    (BIT(29))
#define RTC_CNTL_DBOOST_FORCE_PU_M  (RTC_CNTL_DBOOST_FORCE_PU_V << RTC_CNTL_DBOOST_FORCE_PU_S)
#define RTC_CNTL_DBOOST_FORCE_PU_V  0x00000001
#define RTC_CNTL_DBOOST_FORCE_PU_S  29

/* RTC_CNTL_DBOOST_FORCE_PD : R/W; bitpos: [28]; default: 0;
 * RTC_DBOOST force power down
 */

#define RTC_CNTL_DBOOST_FORCE_PD    (BIT(28))
#define RTC_CNTL_DBOOST_FORCE_PD_M  (RTC_CNTL_DBOOST_FORCE_PD_V << RTC_CNTL_DBOOST_FORCE_PD_S)
#define RTC_CNTL_DBOOST_FORCE_PD_V  0x00000001
#define RTC_CNTL_DBOOST_FORCE_PD_S  28

/* RTC_CNTL_DBIAS_WAK : R/W; bitpos: [27:25]; default: 4;
 * RTC_DBIAS during wakeup
 */

#define RTC_CNTL_DBIAS_WAK    0x00000007
#define RTC_CNTL_DBIAS_WAK_M  (RTC_CNTL_DBIAS_WAK_V << RTC_CNTL_DBIAS_WAK_S)
#define RTC_CNTL_DBIAS_WAK_V  0x00000007
#define RTC_CNTL_DBIAS_WAK_S  25

/* RTC_CNTL_DBIAS_SLP : R/W; bitpos: [24:22]; default: 4;
 * RTC_DBIAS during sleep
 */

#define RTC_CNTL_DBIAS_SLP    0x00000007
#define RTC_CNTL_DBIAS_SLP_M  (RTC_CNTL_DBIAS_SLP_V << RTC_CNTL_DBIAS_SLP_S)
#define RTC_CNTL_DBIAS_SLP_V  0x00000007
#define RTC_CNTL_DBIAS_SLP_S  22

/* RTC_CNTL_SCK_DCAP : R/W; bitpos: [21:14]; default: 0;
 * SCK_DCAP
 */

#define RTC_CNTL_SCK_DCAP    0x000000FF
#define RTC_CNTL_SCK_DCAP_M  (RTC_CNTL_SCK_DCAP_V << RTC_CNTL_SCK_DCAP_S)
#define RTC_CNTL_SCK_DCAP_V  0x000000FF
#define RTC_CNTL_SCK_DCAP_S  14

/* RTC_CNTL_DIG_REG_DBIAS_WAK : R/W; bitpos: [13:11]; default: 4;
 * DIG_REG_DBIAS during wakeup
 */

#define RTC_CNTL_DIG_DBIAS_WAK    0x00000007
#define RTC_CNTL_DIG_DBIAS_WAK_M  (RTC_CNTL_DIG_DBIAS_WAK_V << RTC_CNTL_DIG_DBIAS_WAK_S)
#define RTC_CNTL_DIG_DBIAS_WAK_V  0x00000007
#define RTC_CNTL_DIG_DBIAS_WAK_S  11

/* RTC_CNTL_DIG_REG_DBIAS_SLP : R/W; bitpos: [10:8]; default: 4;
 * DIG_REG_DBIAS during sleep
 */

#define RTC_CNTL_DIG_DBIAS_SLP    0x00000007
#define RTC_CNTL_DIG_DBIAS_SLP_M  (RTC_CNTL_DIG_DBIAS_SLP_V << RTC_CNTL_DIG_DBIAS_SLP_S)
#define RTC_CNTL_DIG_DBIAS_SLP_V  0x00000007
#define RTC_CNTL_DIG_DBIAS_SLP_S  8

/* RTC_CNTL_PWC_REG register
 * configure rtc power configure
 */

#define RTC_CNTL_PWC_REG (DR_REG_RTCCNTL_BASE + 0x88)

/* RTC_CNTL_PAD_FORCE_HOLD : R/W; bitpos: [21]; default: 0;
 * rtc pad force hold
 */

#define RTC_CNTL_PAD_FORCE_HOLD    (BIT(21))
#define RTC_CNTL_PAD_FORCE_HOLD_M  (RTC_CNTL_PAD_FORCE_HOLD_V << RTC_CNTL_PAD_FORCE_HOLD_S)
#define RTC_CNTL_PAD_FORCE_HOLD_V  0x00000001
#define RTC_CNTL_PAD_FORCE_HOLD_S  21

/* RTC_CNTL_PD_EN : R/W; bitpos: [20]; default: 0;
 * enable power down rtc_peri in sleep
 */

#define RTC_CNTL_PD_EN    (BIT(20))
#define RTC_CNTL_PD_EN_M  (RTC_CNTL_PD_EN_V << RTC_CNTL_PD_EN_S)
#define RTC_CNTL_PD_EN_V  0x00000001
#define RTC_CNTL_PD_EN_S  20

/* RTC_CNTL_FORCE_PU : R/W; bitpos: [19]; default: 0;
 * rtc_peri force power up
 */

#define RTC_CNTL_FORCE_PU    (BIT(19))
#define RTC_CNTL_FORCE_PU_M  (RTC_CNTL_FORCE_PU_V << RTC_CNTL_FORCE_PU_S)
#define RTC_CNTL_FORCE_PU_V  0x00000001
#define RTC_CNTL_FORCE_PU_S  19

/* RTC_CNTL_FORCE_PD : R/W; bitpos: [18]; default: 0;
 * rtc_peri force power down
 */

#define RTC_CNTL_FORCE_PD    (BIT(18))
#define RTC_CNTL_FORCE_PD_M  (RTC_CNTL_FORCE_PD_V << RTC_CNTL_FORCE_PD_S)
#define RTC_CNTL_FORCE_PD_V  0x00000001
#define RTC_CNTL_FORCE_PD_S  18

/* RTC_CNTL_SLOWMEM_PD_EN : R/W; bitpos: [17]; default: 0;
 * enable power down RTC memory in sleep
 */

#define RTC_CNTL_SLOWMEM_PD_EN    (BIT(17))
#define RTC_CNTL_SLOWMEM_PD_EN_M  (RTC_CNTL_SLOWMEM_PD_EN_V << RTC_CNTL_SLOWMEM_PD_EN_S)
#define RTC_CNTL_SLOWMEM_PD_EN_V  0x00000001
#define RTC_CNTL_SLOWMEM_PD_EN_S  17

/* RTC_CNTL_SLOWMEM_FORCE_PU : R/W; bitpos: [16]; default: 1;
 * RTC memory force power up
 */

#define RTC_CNTL_SLOWMEM_FORCE_PU    (BIT(16))
#define RTC_CNTL_SLOWMEM_FORCE_PU_M  (RTC_CNTL_SLOWMEM_FORCE_PU_V << RTC_CNTL_SLOWMEM_FORCE_PU_S)
#define RTC_CNTL_SLOWMEM_FORCE_PU_V  0x00000001
#define RTC_CNTL_SLOWMEM_FORCE_PU_S  16

/* RTC_CNTL_SLOWMEM_FORCE_PD : R/W; bitpos: [15]; default: 0;
 * RTC memory force power down
 */

#define RTC_CNTL_SLOWMEM_FORCE_PD    (BIT(15))
#define RTC_CNTL_SLOWMEM_FORCE_PD_M  (RTC_CNTL_SLOWMEM_FORCE_PD_V << RTC_CNTL_SLOWMEM_FORCE_PD_S)
#define RTC_CNTL_SLOWMEM_FORCE_PD_V  0x00000001
#define RTC_CNTL_SLOWMEM_FORCE_PD_S  15

/* RTC_CNTL_FASTMEM_PD_EN : R/W; bitpos: [14]; default: 0;
 * enable power down fast RTC memory in sleep
 */

#define RTC_CNTL_FASTMEM_PD_EN    (BIT(14))
#define RTC_CNTL_FASTMEM_PD_EN_M  (RTC_CNTL_FASTMEM_PD_EN_V << RTC_CNTL_FASTMEM_PD_EN_S)
#define RTC_CNTL_FASTMEM_PD_EN_V  0x00000001
#define RTC_CNTL_FASTMEM_PD_EN_S  14

/* RTC_CNTL_FASTMEM_FORCE_PU : R/W; bitpos: [13]; default: 1;
 * Fast RTC memory force power up
 */

#define RTC_CNTL_FASTMEM_FORCE_PU    (BIT(13))
#define RTC_CNTL_FASTMEM_FORCE_PU_M  (RTC_CNTL_FASTMEM_FORCE_PU_V << RTC_CNTL_FASTMEM_FORCE_PU_S)
#define RTC_CNTL_FASTMEM_FORCE_PU_V  0x00000001
#define RTC_CNTL_FASTMEM_FORCE_PU_S  13

/* RTC_CNTL_FASTMEM_FORCE_PD : R/W; bitpos: [12]; default: 0;
 * Fast RTC memory force power down
 */

#define RTC_CNTL_FASTMEM_FORCE_PD    (BIT(12))
#define RTC_CNTL_FASTMEM_FORCE_PD_M  (RTC_CNTL_FASTMEM_FORCE_PD_V << RTC_CNTL_FASTMEM_FORCE_PD_S)
#define RTC_CNTL_FASTMEM_FORCE_PD_V  0x00000001
#define RTC_CNTL_FASTMEM_FORCE_PD_S  12

/* RTC_CNTL_SLOWMEM_FORCE_LPU : R/W; bitpos: [11]; default: 1;
 * RTC memory force no PD
 */

#define RTC_CNTL_SLOWMEM_FORCE_LPU    (BIT(11))
#define RTC_CNTL_SLOWMEM_FORCE_LPU_M  (RTC_CNTL_SLOWMEM_FORCE_LPU_V << RTC_CNTL_SLOWMEM_FORCE_LPU_S)
#define RTC_CNTL_SLOWMEM_FORCE_LPU_V  0x00000001
#define RTC_CNTL_SLOWMEM_FORCE_LPU_S  11

/* RTC_CNTL_SLOWMEM_FORCE_LPD : R/W; bitpos: [10]; default: 0;
 * RTC memory force PD
 */

#define RTC_CNTL_SLOWMEM_FORCE_LPD    (BIT(10))
#define RTC_CNTL_SLOWMEM_FORCE_LPD_M  (RTC_CNTL_SLOWMEM_FORCE_LPD_V << RTC_CNTL_SLOWMEM_FORCE_LPD_S)
#define RTC_CNTL_SLOWMEM_FORCE_LPD_V  0x00000001
#define RTC_CNTL_SLOWMEM_FORCE_LPD_S  10

/* RTC_CNTL_SLOWMEM_FOLW_CPU : R/W; bitpos: [9]; default: 0;
 * 1: RTC memory  PD following CPU  0: RTC memory PD following RTC state
 * machine
 */

#define RTC_CNTL_SLOWMEM_FOLW_CPU    (BIT(9))
#define RTC_CNTL_SLOWMEM_FOLW_CPU_M  (RTC_CNTL_SLOWMEM_FOLW_CPU_V << RTC_CNTL_SLOWMEM_FOLW_CPU_S)
#define RTC_CNTL_SLOWMEM_FOLW_CPU_V  0x00000001
#define RTC_CNTL_SLOWMEM_FOLW_CPU_S  9

/* RTC_CNTL_FASTMEM_FORCE_LPU : R/W; bitpos: [8]; default: 1;
 * Fast RTC memory force no PD
 */

#define RTC_CNTL_FASTMEM_FORCE_LPU    (BIT(8))
#define RTC_CNTL_FASTMEM_FORCE_LPU_M  (RTC_CNTL_FASTMEM_FORCE_LPU_V << RTC_CNTL_FASTMEM_FORCE_LPU_S)
#define RTC_CNTL_FASTMEM_FORCE_LPU_V  0x00000001
#define RTC_CNTL_FASTMEM_FORCE_LPU_S  8

/* RTC_CNTL_FASTMEM_FORCE_LPD : R/W; bitpos: [7]; default: 0;
 * Fast RTC memory force PD
 */

#define RTC_CNTL_FASTMEM_FORCE_LPD    (BIT(7))
#define RTC_CNTL_FASTMEM_FORCE_LPD_M  (RTC_CNTL_FASTMEM_FORCE_LPD_V << RTC_CNTL_FASTMEM_FORCE_LPD_S)
#define RTC_CNTL_FASTMEM_FORCE_LPD_V  0x00000001
#define RTC_CNTL_FASTMEM_FORCE_LPD_S  7

/* RTC_CNTL_FASTMEM_FOLW_CPU : R/W; bitpos: [6]; default: 0;
 * 1: Fast RTC memory PD following CPU   0: fast RTC memory PD following RTC
 * state machine
 */

#define RTC_CNTL_FASTMEM_FOLW_CPU    (BIT(6))
#define RTC_CNTL_FASTMEM_FOLW_CPU_M  (RTC_CNTL_FASTMEM_FOLW_CPU_V << RTC_CNTL_FASTMEM_FOLW_CPU_S)
#define RTC_CNTL_FASTMEM_FOLW_CPU_V  0x00000001
#define RTC_CNTL_FASTMEM_FOLW_CPU_S  6

/* RTC_CNTL_FORCE_NOISO : R/W; bitpos: [5]; default: 1;
 * rtc_peri force no ISO
 */

#define RTC_CNTL_FORCE_NOISO    (BIT(5))
#define RTC_CNTL_FORCE_NOISO_M  (RTC_CNTL_FORCE_NOISO_V << RTC_CNTL_FORCE_NOISO_S)
#define RTC_CNTL_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_FORCE_NOISO_S  5

/* RTC_CNTL_FORCE_ISO : R/W; bitpos: [4]; default: 0;
 * rtc_peri force ISO
 */

#define RTC_CNTL_FORCE_ISO    (BIT(4))
#define RTC_CNTL_FORCE_ISO_M  (RTC_CNTL_FORCE_ISO_V << RTC_CNTL_FORCE_ISO_S)
#define RTC_CNTL_FORCE_ISO_V  0x00000001
#define RTC_CNTL_FORCE_ISO_S  4

/* RTC_CNTL_SLOWMEM_FORCE_ISO : R/W; bitpos: [3]; default: 0;
 * RTC memory force ISO
 */

#define RTC_CNTL_SLOWMEM_FORCE_ISO    (BIT(3))
#define RTC_CNTL_SLOWMEM_FORCE_ISO_M  (RTC_CNTL_SLOWMEM_FORCE_ISO_V << RTC_CNTL_SLOWMEM_FORCE_ISO_S)
#define RTC_CNTL_SLOWMEM_FORCE_ISO_V  0x00000001
#define RTC_CNTL_SLOWMEM_FORCE_ISO_S  3

/* RTC_CNTL_SLOWMEM_FORCE_NOISO : R/W; bitpos: [2]; default: 1;
 * RTC memory force no ISO
 */

#define RTC_CNTL_SLOWMEM_FORCE_NOISO    (BIT(2))
#define RTC_CNTL_SLOWMEM_FORCE_NOISO_M  (RTC_CNTL_SLOWMEM_FORCE_NOISO_V << RTC_CNTL_SLOWMEM_FORCE_NOISO_S)
#define RTC_CNTL_SLOWMEM_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_SLOWMEM_FORCE_NOISO_S  2

/* RTC_CNTL_FASTMEM_FORCE_ISO : R/W; bitpos: [1]; default: 0;
 * Fast RTC memory force ISO
 */

#define RTC_CNTL_FASTMEM_FORCE_ISO    (BIT(1))
#define RTC_CNTL_FASTMEM_FORCE_ISO_M  (RTC_CNTL_FASTMEM_FORCE_ISO_V << RTC_CNTL_FASTMEM_FORCE_ISO_S)
#define RTC_CNTL_FASTMEM_FORCE_ISO_V  0x00000001
#define RTC_CNTL_FASTMEM_FORCE_ISO_S  1

/* RTC_CNTL_FASTMEM_FORCE_NOISO : R/W; bitpos: [0]; default: 1;
 * Fast RTC memory force no ISO
 */

#define RTC_CNTL_FASTMEM_FORCE_NOISO    (BIT(0))
#define RTC_CNTL_FASTMEM_FORCE_NOISO_M  (RTC_CNTL_FASTMEM_FORCE_NOISO_V << RTC_CNTL_FASTMEM_FORCE_NOISO_S)
#define RTC_CNTL_FASTMEM_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_FASTMEM_FORCE_NOISO_S  0

/* RTC_CNTL_DIG_PWC_REG register
 * configure power of digital core
 */

#define RTC_CNTL_DIG_PWC_REG (DR_REG_RTCCNTL_BASE + 0x8c)

/* RTC_CNTL_DG_WRAP_PD_EN : R/W; bitpos: [31]; default: 0;
 * enable power down digital core in sleep
 */

#define RTC_CNTL_DG_WRAP_PD_EN    (BIT(31))
#define RTC_CNTL_DG_WRAP_PD_EN_M  (RTC_CNTL_DG_WRAP_PD_EN_V << RTC_CNTL_DG_WRAP_PD_EN_S)
#define RTC_CNTL_DG_WRAP_PD_EN_V  0x00000001
#define RTC_CNTL_DG_WRAP_PD_EN_S  31

/* RTC_CNTL_WIFI_PD_EN : R/W; bitpos: [30]; default: 0;
 * enable power down wifi in sleep
 */

#define RTC_CNTL_WIFI_PD_EN    (BIT(30))
#define RTC_CNTL_WIFI_PD_EN_M  (RTC_CNTL_WIFI_PD_EN_V << RTC_CNTL_WIFI_PD_EN_S)
#define RTC_CNTL_WIFI_PD_EN_V  0x00000001
#define RTC_CNTL_WIFI_PD_EN_S  30

/* RTC_CNTL_INTER_RAM4_PD_EN : R/W; bitpos: [29]; default: 0;
 * enable power down internal SRAM 4 in sleep
 */

#define RTC_CNTL_INTER_RAM4_PD_EN    (BIT(29))
#define RTC_CNTL_INTER_RAM4_PD_EN_M  (RTC_CNTL_INTER_RAM4_PD_EN_V << RTC_CNTL_INTER_RAM4_PD_EN_S)
#define RTC_CNTL_INTER_RAM4_PD_EN_V  0x00000001
#define RTC_CNTL_INTER_RAM4_PD_EN_S  29

/* RTC_CNTL_INTER_RAM3_PD_EN : R/W; bitpos: [28]; default: 0;
 * enable power down internal SRAM 3 in sleep
 */

#define RTC_CNTL_INTER_RAM3_PD_EN    (BIT(28))
#define RTC_CNTL_INTER_RAM3_PD_EN_M  (RTC_CNTL_INTER_RAM3_PD_EN_V << RTC_CNTL_INTER_RAM3_PD_EN_S)
#define RTC_CNTL_INTER_RAM3_PD_EN_V  0x00000001
#define RTC_CNTL_INTER_RAM3_PD_EN_S  28

/* RTC_CNTL_INTER_RAM2_PD_EN : R/W; bitpos: [27]; default: 0;
 * enable power down internal SRAM 2 in sleep
 */

#define RTC_CNTL_INTER_RAM2_PD_EN    (BIT(27))
#define RTC_CNTL_INTER_RAM2_PD_EN_M  (RTC_CNTL_INTER_RAM2_PD_EN_V << RTC_CNTL_INTER_RAM2_PD_EN_S)
#define RTC_CNTL_INTER_RAM2_PD_EN_V  0x00000001
#define RTC_CNTL_INTER_RAM2_PD_EN_S  27

/* RTC_CNTL_INTER_RAM1_PD_EN : R/W; bitpos: [26]; default: 0;
 * enable power down internal SRAM 1 in sleep
 */

#define RTC_CNTL_INTER_RAM1_PD_EN    (BIT(26))
#define RTC_CNTL_INTER_RAM1_PD_EN_M  (RTC_CNTL_INTER_RAM1_PD_EN_V << RTC_CNTL_INTER_RAM1_PD_EN_S)
#define RTC_CNTL_INTER_RAM1_PD_EN_V  0x00000001
#define RTC_CNTL_INTER_RAM1_PD_EN_S  26

/* RTC_CNTL_INTER_RAM0_PD_EN : R/W; bitpos: [25]; default: 0;
 * enable power down internal SRAM 0 in sleep
 */

#define RTC_CNTL_INTER_RAM0_PD_EN    (BIT(25))
#define RTC_CNTL_INTER_RAM0_PD_EN_M  (RTC_CNTL_INTER_RAM0_PD_EN_V << RTC_CNTL_INTER_RAM0_PD_EN_S)
#define RTC_CNTL_INTER_RAM0_PD_EN_V  0x00000001
#define RTC_CNTL_INTER_RAM0_PD_EN_S  25

/* RTC_CNTL_ROM0_PD_EN : R/W; bitpos: [24]; default: 0;
 * enable power down ROM in sleep
 */

#define RTC_CNTL_ROM0_PD_EN    (BIT(24))
#define RTC_CNTL_ROM0_PD_EN_M  (RTC_CNTL_ROM0_PD_EN_V << RTC_CNTL_ROM0_PD_EN_S)
#define RTC_CNTL_ROM0_PD_EN_V  0x00000001
#define RTC_CNTL_ROM0_PD_EN_S  24

/* RTC_CNTL_DG_DCDC_PD_EN : R/W; bitpos: [23]; default: 0;
 * enable power down digital dcdc in sleep
 */

#define RTC_CNTL_DG_DCDC_PD_EN    (BIT(23))
#define RTC_CNTL_DG_DCDC_PD_EN_M  (RTC_CNTL_DG_DCDC_PD_EN_V << RTC_CNTL_DG_DCDC_PD_EN_S)
#define RTC_CNTL_DG_DCDC_PD_EN_V  0x00000001
#define RTC_CNTL_DG_DCDC_PD_EN_S  23

/* RTC_CNTL_DG_DCDC_FORCE_PU : R/W; bitpos: [22]; default: 1;
 * digital dcdc force power up
 */

#define RTC_CNTL_DG_DCDC_FORCE_PU    (BIT(22))
#define RTC_CNTL_DG_DCDC_FORCE_PU_M  (RTC_CNTL_DG_DCDC_FORCE_PU_V << RTC_CNTL_DG_DCDC_FORCE_PU_S)
#define RTC_CNTL_DG_DCDC_FORCE_PU_V  0x00000001
#define RTC_CNTL_DG_DCDC_FORCE_PU_S  22

/* RTC_CNTL_DG_DCDC_FORCE_PD : R/W; bitpos: [21]; default: 0;
 * digital dcdc force power down
 */

#define RTC_CNTL_DG_DCDC_FORCE_PD    (BIT(21))
#define RTC_CNTL_DG_DCDC_FORCE_PD_M  (RTC_CNTL_DG_DCDC_FORCE_PD_V << RTC_CNTL_DG_DCDC_FORCE_PD_S)
#define RTC_CNTL_DG_DCDC_FORCE_PD_V  0x00000001
#define RTC_CNTL_DG_DCDC_FORCE_PD_S  21

/* RTC_CNTL_DG_WRAP_FORCE_PU : R/W; bitpos: [20]; default: 1;
 * digital core force power up
 */

#define RTC_CNTL_DG_WRAP_FORCE_PU    (BIT(20))
#define RTC_CNTL_DG_WRAP_FORCE_PU_M  (RTC_CNTL_DG_WRAP_FORCE_PU_V << RTC_CNTL_DG_WRAP_FORCE_PU_S)
#define RTC_CNTL_DG_WRAP_FORCE_PU_V  0x00000001
#define RTC_CNTL_DG_WRAP_FORCE_PU_S  20

/* RTC_CNTL_DG_WRAP_FORCE_PD : R/W; bitpos: [19]; default: 0;
 * digital core force power down
 */

#define RTC_CNTL_DG_WRAP_FORCE_PD    (BIT(19))
#define RTC_CNTL_DG_WRAP_FORCE_PD_M  (RTC_CNTL_DG_WRAP_FORCE_PD_V << RTC_CNTL_DG_WRAP_FORCE_PD_S)
#define RTC_CNTL_DG_WRAP_FORCE_PD_V  0x00000001
#define RTC_CNTL_DG_WRAP_FORCE_PD_S  19

/* RTC_CNTL_WIFI_FORCE_PU : R/W; bitpos: [18]; default: 1;
 * wifi force power up
 */

#define RTC_CNTL_WIFI_FORCE_PU    (BIT(18))
#define RTC_CNTL_WIFI_FORCE_PU_M  (RTC_CNTL_WIFI_FORCE_PU_V << RTC_CNTL_WIFI_FORCE_PU_S)
#define RTC_CNTL_WIFI_FORCE_PU_V  0x00000001
#define RTC_CNTL_WIFI_FORCE_PU_S  18

/* RTC_CNTL_WIFI_FORCE_PD : R/W; bitpos: [17]; default: 0;
 * wifi force power down
 */

#define RTC_CNTL_WIFI_FORCE_PD    (BIT(17))
#define RTC_CNTL_WIFI_FORCE_PD_M  (RTC_CNTL_WIFI_FORCE_PD_V << RTC_CNTL_WIFI_FORCE_PD_S)
#define RTC_CNTL_WIFI_FORCE_PD_V  0x00000001
#define RTC_CNTL_WIFI_FORCE_PD_S  17

/* RTC_CNTL_INTER_RAM4_FORCE_PU : R/W; bitpos: [16]; default: 1;
 * internal SRAM 4 force power up
 */

#define RTC_CNTL_INTER_RAM4_FORCE_PU    (BIT(16))
#define RTC_CNTL_INTER_RAM4_FORCE_PU_M  (RTC_CNTL_INTER_RAM4_FORCE_PU_V << RTC_CNTL_INTER_RAM4_FORCE_PU_S)
#define RTC_CNTL_INTER_RAM4_FORCE_PU_V  0x00000001
#define RTC_CNTL_INTER_RAM4_FORCE_PU_S  16

/* RTC_CNTL_INTER_RAM4_FORCE_PD : R/W; bitpos: [15]; default: 0;
 * internal SRAM 4 force power down
 */

#define RTC_CNTL_INTER_RAM4_FORCE_PD    (BIT(15))
#define RTC_CNTL_INTER_RAM4_FORCE_PD_M  (RTC_CNTL_INTER_RAM4_FORCE_PD_V << RTC_CNTL_INTER_RAM4_FORCE_PD_S)
#define RTC_CNTL_INTER_RAM4_FORCE_PD_V  0x00000001
#define RTC_CNTL_INTER_RAM4_FORCE_PD_S  15

/* RTC_CNTL_INTER_RAM3_FORCE_PU : R/W; bitpos: [14]; default: 1;
 * internal SRAM 3 force power up
 */

#define RTC_CNTL_INTER_RAM3_FORCE_PU    (BIT(14))
#define RTC_CNTL_INTER_RAM3_FORCE_PU_M  (RTC_CNTL_INTER_RAM3_FORCE_PU_V << RTC_CNTL_INTER_RAM3_FORCE_PU_S)
#define RTC_CNTL_INTER_RAM3_FORCE_PU_V  0x00000001
#define RTC_CNTL_INTER_RAM3_FORCE_PU_S  14

/* RTC_CNTL_INTER_RAM3_FORCE_PD : R/W; bitpos: [13]; default: 0;
 * internal SRAM 3 force power down
 */

#define RTC_CNTL_INTER_RAM3_FORCE_PD    (BIT(13))
#define RTC_CNTL_INTER_RAM3_FORCE_PD_M  (RTC_CNTL_INTER_RAM3_FORCE_PD_V << RTC_CNTL_INTER_RAM3_FORCE_PD_S)
#define RTC_CNTL_INTER_RAM3_FORCE_PD_V  0x00000001
#define RTC_CNTL_INTER_RAM3_FORCE_PD_S  13

/* RTC_CNTL_INTER_RAM2_FORCE_PU : R/W; bitpos: [12]; default: 1;
 * internal SRAM 2 force power up
 */

#define RTC_CNTL_INTER_RAM2_FORCE_PU    (BIT(12))
#define RTC_CNTL_INTER_RAM2_FORCE_PU_M  (RTC_CNTL_INTER_RAM2_FORCE_PU_V << RTC_CNTL_INTER_RAM2_FORCE_PU_S)
#define RTC_CNTL_INTER_RAM2_FORCE_PU_V  0x00000001
#define RTC_CNTL_INTER_RAM2_FORCE_PU_S  12

/* RTC_CNTL_INTER_RAM2_FORCE_PD : R/W; bitpos: [11]; default: 0;
 * internal SRAM 2 force power down
 */

#define RTC_CNTL_INTER_RAM2_FORCE_PD    (BIT(11))
#define RTC_CNTL_INTER_RAM2_FORCE_PD_M  (RTC_CNTL_INTER_RAM2_FORCE_PD_V << RTC_CNTL_INTER_RAM2_FORCE_PD_S)
#define RTC_CNTL_INTER_RAM2_FORCE_PD_V  0x00000001
#define RTC_CNTL_INTER_RAM2_FORCE_PD_S  11

/* RTC_CNTL_INTER_RAM1_FORCE_PU : R/W; bitpos: [10]; default: 1;
 * internal SRAM 1 force power up
 */

#define RTC_CNTL_INTER_RAM1_FORCE_PU    (BIT(10))
#define RTC_CNTL_INTER_RAM1_FORCE_PU_M  (RTC_CNTL_INTER_RAM1_FORCE_PU_V << RTC_CNTL_INTER_RAM1_FORCE_PU_S)
#define RTC_CNTL_INTER_RAM1_FORCE_PU_V  0x00000001
#define RTC_CNTL_INTER_RAM1_FORCE_PU_S  10

/* RTC_CNTL_INTER_RAM1_FORCE_PD : R/W; bitpos: [9]; default: 0;
 * internal SRAM 1 force power down
 */

#define RTC_CNTL_INTER_RAM1_FORCE_PD    (BIT(9))
#define RTC_CNTL_INTER_RAM1_FORCE_PD_M  (RTC_CNTL_INTER_RAM1_FORCE_PD_V << RTC_CNTL_INTER_RAM1_FORCE_PD_S)
#define RTC_CNTL_INTER_RAM1_FORCE_PD_V  0x00000001
#define RTC_CNTL_INTER_RAM1_FORCE_PD_S  9

/* RTC_CNTL_INTER_RAM0_FORCE_PU : R/W; bitpos: [8]; default: 1;
 * internal SRAM 0 force power up
 */

#define RTC_CNTL_INTER_RAM0_FORCE_PU    (BIT(8))
#define RTC_CNTL_INTER_RAM0_FORCE_PU_M  (RTC_CNTL_INTER_RAM0_FORCE_PU_V << RTC_CNTL_INTER_RAM0_FORCE_PU_S)
#define RTC_CNTL_INTER_RAM0_FORCE_PU_V  0x00000001
#define RTC_CNTL_INTER_RAM0_FORCE_PU_S  8

/* RTC_CNTL_INTER_RAM0_FORCE_PD : R/W; bitpos: [7]; default: 0;
 * internal SRAM 0 force power down
 */

#define RTC_CNTL_INTER_RAM0_FORCE_PD    (BIT(7))
#define RTC_CNTL_INTER_RAM0_FORCE_PD_M  (RTC_CNTL_INTER_RAM0_FORCE_PD_V << RTC_CNTL_INTER_RAM0_FORCE_PD_S)
#define RTC_CNTL_INTER_RAM0_FORCE_PD_V  0x00000001
#define RTC_CNTL_INTER_RAM0_FORCE_PD_S  7

/* RTC_CNTL_ROM0_FORCE_PU : R/W; bitpos: [6]; default: 1;
 * ROM force power up
 */

#define RTC_CNTL_ROM0_FORCE_PU    (BIT(6))
#define RTC_CNTL_ROM0_FORCE_PU_M  (RTC_CNTL_ROM0_FORCE_PU_V << RTC_CNTL_ROM0_FORCE_PU_S)
#define RTC_CNTL_ROM0_FORCE_PU_V  0x00000001
#define RTC_CNTL_ROM0_FORCE_PU_S  6

/* RTC_CNTL_ROM0_FORCE_PD : R/W; bitpos: [5]; default: 0;
 * ROM force power down
 */

#define RTC_CNTL_ROM0_FORCE_PD    (BIT(5))
#define RTC_CNTL_ROM0_FORCE_PD_M  (RTC_CNTL_ROM0_FORCE_PD_V << RTC_CNTL_ROM0_FORCE_PD_S)
#define RTC_CNTL_ROM0_FORCE_PD_V  0x00000001
#define RTC_CNTL_ROM0_FORCE_PD_S  5

/* RTC_CNTL_LSLP_MEM_FORCE_PU : R/W; bitpos: [4]; default: 1;
 * memories in digital core force no PD in sleep
 */

#define RTC_CNTL_LSLP_MEM_FORCE_PU    (BIT(4))
#define RTC_CNTL_LSLP_MEM_FORCE_PU_M  (RTC_CNTL_LSLP_MEM_FORCE_PU_V << RTC_CNTL_LSLP_MEM_FORCE_PU_S)
#define RTC_CNTL_LSLP_MEM_FORCE_PU_V  0x00000001
#define RTC_CNTL_LSLP_MEM_FORCE_PU_S  4

/* RTC_CNTL_LSLP_MEM_FORCE_PD : R/W; bitpos: [3]; default: 0;
 * memories in digital core force PD in sleep
 */

#define RTC_CNTL_LSLP_MEM_FORCE_PD    (BIT(3))
#define RTC_CNTL_LSLP_MEM_FORCE_PD_M  (RTC_CNTL_LSLP_MEM_FORCE_PD_V << RTC_CNTL_LSLP_MEM_FORCE_PD_S)
#define RTC_CNTL_LSLP_MEM_FORCE_PD_V  0x00000001
#define RTC_CNTL_LSLP_MEM_FORCE_PD_S  3

/* RTC_CNTL_DIG_ISO_REG register
 * configure ISO of digital core
 */

#define RTC_CNTL_DIG_ISO_REG (DR_REG_RTCCNTL_BASE + 0x90)

/* RTC_CNTL_DG_WRAP_FORCE_NOISO : R/W; bitpos: [31]; default: 1;
 * digital core force no ISO
 */

#define RTC_CNTL_DG_WRAP_FORCE_NOISO    (BIT(31))
#define RTC_CNTL_DG_WRAP_FORCE_NOISO_M  (RTC_CNTL_DG_WRAP_FORCE_NOISO_V << RTC_CNTL_DG_WRAP_FORCE_NOISO_S)
#define RTC_CNTL_DG_WRAP_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_DG_WRAP_FORCE_NOISO_S  31

/* RTC_CNTL_DG_WRAP_FORCE_ISO : R/W; bitpos: [30]; default: 0;
 * digital core force ISO
 */

#define RTC_CNTL_DG_WRAP_FORCE_ISO    (BIT(30))
#define RTC_CNTL_DG_WRAP_FORCE_ISO_M  (RTC_CNTL_DG_WRAP_FORCE_ISO_V << RTC_CNTL_DG_WRAP_FORCE_ISO_S)
#define RTC_CNTL_DG_WRAP_FORCE_ISO_V  0x00000001
#define RTC_CNTL_DG_WRAP_FORCE_ISO_S  30

/* RTC_CNTL_WIFI_FORCE_NOISO : R/W; bitpos: [29]; default: 1;
 * wifi force no ISO
 */

#define RTC_CNTL_WIFI_FORCE_NOISO    (BIT(29))
#define RTC_CNTL_WIFI_FORCE_NOISO_M  (RTC_CNTL_WIFI_FORCE_NOISO_V << RTC_CNTL_WIFI_FORCE_NOISO_S)
#define RTC_CNTL_WIFI_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_WIFI_FORCE_NOISO_S  29

/* RTC_CNTL_WIFI_FORCE_ISO : R/W; bitpos: [28]; default: 0;
 * wifi force ISO
 */

#define RTC_CNTL_WIFI_FORCE_ISO    (BIT(28))
#define RTC_CNTL_WIFI_FORCE_ISO_M  (RTC_CNTL_WIFI_FORCE_ISO_V << RTC_CNTL_WIFI_FORCE_ISO_S)
#define RTC_CNTL_WIFI_FORCE_ISO_V  0x00000001
#define RTC_CNTL_WIFI_FORCE_ISO_S  28

/* RTC_CNTL_INTER_RAM4_FORCE_NOISO : R/W; bitpos: [27]; default: 1;
 * internal SRAM 4 force no ISO
 */

#define RTC_CNTL_INTER_RAM4_FORCE_NOISO    (BIT(27))
#define RTC_CNTL_INTER_RAM4_FORCE_NOISO_M  (RTC_CNTL_INTER_RAM4_FORCE_NOISO_V << RTC_CNTL_INTER_RAM4_FORCE_NOISO_S)
#define RTC_CNTL_INTER_RAM4_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_INTER_RAM4_FORCE_NOISO_S  27

/* RTC_CNTL_INTER_RAM4_FORCE_ISO : R/W; bitpos: [26]; default: 0;
 * internal SRAM 4 force ISO
 */

#define RTC_CNTL_INTER_RAM4_FORCE_ISO    (BIT(26))
#define RTC_CNTL_INTER_RAM4_FORCE_ISO_M  (RTC_CNTL_INTER_RAM4_FORCE_ISO_V << RTC_CNTL_INTER_RAM4_FORCE_ISO_S)
#define RTC_CNTL_INTER_RAM4_FORCE_ISO_V  0x00000001
#define RTC_CNTL_INTER_RAM4_FORCE_ISO_S  26

/* RTC_CNTL_INTER_RAM3_FORCE_NOISO : R/W; bitpos: [25]; default: 1;
 * internal SRAM 3 force no ISO
 */

#define RTC_CNTL_INTER_RAM3_FORCE_NOISO    (BIT(25))
#define RTC_CNTL_INTER_RAM3_FORCE_NOISO_M  (RTC_CNTL_INTER_RAM3_FORCE_NOISO_V << RTC_CNTL_INTER_RAM3_FORCE_NOISO_S)
#define RTC_CNTL_INTER_RAM3_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_INTER_RAM3_FORCE_NOISO_S  25

/* RTC_CNTL_INTER_RAM3_FORCE_ISO : R/W; bitpos: [24]; default: 0;
 * internal SRAM 3 force ISO
 */

#define RTC_CNTL_INTER_RAM3_FORCE_ISO    (BIT(24))
#define RTC_CNTL_INTER_RAM3_FORCE_ISO_M  (RTC_CNTL_INTER_RAM3_FORCE_ISO_V << RTC_CNTL_INTER_RAM3_FORCE_ISO_S)
#define RTC_CNTL_INTER_RAM3_FORCE_ISO_V  0x00000001
#define RTC_CNTL_INTER_RAM3_FORCE_ISO_S  24

/* RTC_CNTL_INTER_RAM2_FORCE_NOISO : R/W; bitpos: [23]; default: 1;
 * internal SRAM 2 force no ISO
 */

#define RTC_CNTL_INTER_RAM2_FORCE_NOISO    (BIT(23))
#define RTC_CNTL_INTER_RAM2_FORCE_NOISO_M  (RTC_CNTL_INTER_RAM2_FORCE_NOISO_V << RTC_CNTL_INTER_RAM2_FORCE_NOISO_S)
#define RTC_CNTL_INTER_RAM2_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_INTER_RAM2_FORCE_NOISO_S  23

/* RTC_CNTL_INTER_RAM2_FORCE_ISO : R/W; bitpos: [22]; default: 0;
 * internal SRAM 2 force ISO
 */

#define RTC_CNTL_INTER_RAM2_FORCE_ISO    (BIT(22))
#define RTC_CNTL_INTER_RAM2_FORCE_ISO_M  (RTC_CNTL_INTER_RAM2_FORCE_ISO_V << RTC_CNTL_INTER_RAM2_FORCE_ISO_S)
#define RTC_CNTL_INTER_RAM2_FORCE_ISO_V  0x00000001
#define RTC_CNTL_INTER_RAM2_FORCE_ISO_S  22

/* RTC_CNTL_INTER_RAM1_FORCE_NOISO : R/W; bitpos: [21]; default: 1;
 * internal SRAM 1 force no ISO
 */

#define RTC_CNTL_INTER_RAM1_FORCE_NOISO    (BIT(21))
#define RTC_CNTL_INTER_RAM1_FORCE_NOISO_M  (RTC_CNTL_INTER_RAM1_FORCE_NOISO_V << RTC_CNTL_INTER_RAM1_FORCE_NOISO_S)
#define RTC_CNTL_INTER_RAM1_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_INTER_RAM1_FORCE_NOISO_S  21

/* RTC_CNTL_INTER_RAM1_FORCE_ISO : R/W; bitpos: [20]; default: 0;
 * internal SRAM 1 force ISO
 */

#define RTC_CNTL_INTER_RAM1_FORCE_ISO    (BIT(20))
#define RTC_CNTL_INTER_RAM1_FORCE_ISO_M  (RTC_CNTL_INTER_RAM1_FORCE_ISO_V << RTC_CNTL_INTER_RAM1_FORCE_ISO_S)
#define RTC_CNTL_INTER_RAM1_FORCE_ISO_V  0x00000001
#define RTC_CNTL_INTER_RAM1_FORCE_ISO_S  20

/* RTC_CNTL_INTER_RAM0_FORCE_NOISO : R/W; bitpos: [19]; default: 1;
 * internal SRAM 0 force no ISO
 */

#define RTC_CNTL_INTER_RAM0_FORCE_NOISO    (BIT(19))
#define RTC_CNTL_INTER_RAM0_FORCE_NOISO_M  (RTC_CNTL_INTER_RAM0_FORCE_NOISO_V << RTC_CNTL_INTER_RAM0_FORCE_NOISO_S)
#define RTC_CNTL_INTER_RAM0_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_INTER_RAM0_FORCE_NOISO_S  19

/* RTC_CNTL_INTER_RAM0_FORCE_ISO : R/W; bitpos: [18]; default: 0;
 * internal SRAM 0 force ISO
 */

#define RTC_CNTL_INTER_RAM0_FORCE_ISO    (BIT(18))
#define RTC_CNTL_INTER_RAM0_FORCE_ISO_M  (RTC_CNTL_INTER_RAM0_FORCE_ISO_V << RTC_CNTL_INTER_RAM0_FORCE_ISO_S)
#define RTC_CNTL_INTER_RAM0_FORCE_ISO_V  0x00000001
#define RTC_CNTL_INTER_RAM0_FORCE_ISO_S  18

/* RTC_CNTL_ROM0_FORCE_NOISO : R/W; bitpos: [17]; default: 1;
 * ROM force no ISO
 */

#define RTC_CNTL_ROM0_FORCE_NOISO    (BIT(17))
#define RTC_CNTL_ROM0_FORCE_NOISO_M  (RTC_CNTL_ROM0_FORCE_NOISO_V << RTC_CNTL_ROM0_FORCE_NOISO_S)
#define RTC_CNTL_ROM0_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_ROM0_FORCE_NOISO_S  17

/* RTC_CNTL_ROM0_FORCE_ISO : R/W; bitpos: [16]; default: 0;
 * ROM force ISO
 */

#define RTC_CNTL_ROM0_FORCE_ISO    (BIT(16))
#define RTC_CNTL_ROM0_FORCE_ISO_M  (RTC_CNTL_ROM0_FORCE_ISO_V << RTC_CNTL_ROM0_FORCE_ISO_S)
#define RTC_CNTL_ROM0_FORCE_ISO_V  0x00000001
#define RTC_CNTL_ROM0_FORCE_ISO_S  16

/* RTC_CNTL_DG_PAD_FORCE_HOLD : R/W; bitpos: [15]; default: 0;
 * digital pad force hold
 */

#define RTC_CNTL_DG_PAD_FORCE_HOLD    (BIT(15))
#define RTC_CNTL_DG_PAD_FORCE_HOLD_M  (RTC_CNTL_DG_PAD_FORCE_HOLD_V << RTC_CNTL_DG_PAD_FORCE_HOLD_S)
#define RTC_CNTL_DG_PAD_FORCE_HOLD_V  0x00000001
#define RTC_CNTL_DG_PAD_FORCE_HOLD_S  15

/* RTC_CNTL_DG_PAD_FORCE_UNHOLD : R/W; bitpos: [14]; default: 1;
 * digital pad force un-hold
 */

#define RTC_CNTL_DG_PAD_FORCE_UNHOLD    (BIT(14))
#define RTC_CNTL_DG_PAD_FORCE_UNHOLD_M  (RTC_CNTL_DG_PAD_FORCE_UNHOLD_V << RTC_CNTL_DG_PAD_FORCE_UNHOLD_S)
#define RTC_CNTL_DG_PAD_FORCE_UNHOLD_V  0x00000001
#define RTC_CNTL_DG_PAD_FORCE_UNHOLD_S  14

/* RTC_CNTL_DG_PAD_FORCE_ISO : R/W; bitpos: [13]; default: 0;
 * digital pad force ISO
 */

#define RTC_CNTL_DG_PAD_FORCE_ISO    (BIT(13))
#define RTC_CNTL_DG_PAD_FORCE_ISO_M  (RTC_CNTL_DG_PAD_FORCE_ISO_V << RTC_CNTL_DG_PAD_FORCE_ISO_S)
#define RTC_CNTL_DG_PAD_FORCE_ISO_V  0x00000001
#define RTC_CNTL_DG_PAD_FORCE_ISO_S  13

/* RTC_CNTL_DG_PAD_FORCE_NOISO : R/W; bitpos: [12]; default: 1;
 * digital pad force no ISO
 */

#define RTC_CNTL_DG_PAD_FORCE_NOISO    (BIT(12))
#define RTC_CNTL_DG_PAD_FORCE_NOISO_M  (RTC_CNTL_DG_PAD_FORCE_NOISO_V << RTC_CNTL_DG_PAD_FORCE_NOISO_S)
#define RTC_CNTL_DG_PAD_FORCE_NOISO_V  0x00000001
#define RTC_CNTL_DG_PAD_FORCE_NOISO_S  12

/* RTC_CNTL_DG_PAD_AUTOHOLD_EN : R/W; bitpos: [11]; default: 0;
 * digital pad enable auto-hold
 */

#define RTC_CNTL_DG_PAD_AUTOHOLD_EN    (BIT(11))
#define RTC_CNTL_DG_PAD_AUTOHOLD_EN_M  (RTC_CNTL_DG_PAD_AUTOHOLD_EN_V << RTC_CNTL_DG_PAD_AUTOHOLD_EN_S)
#define RTC_CNTL_DG_PAD_AUTOHOLD_EN_V  0x00000001
#define RTC_CNTL_DG_PAD_AUTOHOLD_EN_S  11

/* RTC_CNTL_CLR_DG_PAD_AUTOHOLD : WO; bitpos: [10]; default: 0;
 * wtite only register to clear digital pad auto-hold
 */

#define RTC_CNTL_CLR_DG_PAD_AUTOHOLD    (BIT(10))
#define RTC_CNTL_CLR_DG_PAD_AUTOHOLD_M  (RTC_CNTL_CLR_DG_PAD_AUTOHOLD_V << RTC_CNTL_CLR_DG_PAD_AUTOHOLD_S)
#define RTC_CNTL_CLR_DG_PAD_AUTOHOLD_V  0x00000001
#define RTC_CNTL_CLR_DG_PAD_AUTOHOLD_S  10

/* RTC_CNTL_DG_PAD_AUTOHOLD : RO; bitpos: [9]; default: 0;
 * read only register to indicate digital pad auto-hold status
 */

#define RTC_CNTL_DG_PAD_AUTOHOLD    (BIT(9))
#define RTC_CNTL_DG_PAD_AUTOHOLD_M  (RTC_CNTL_DG_PAD_AUTOHOLD_V << RTC_CNTL_DG_PAD_AUTOHOLD_S)
#define RTC_CNTL_DG_PAD_AUTOHOLD_V  0x00000001
#define RTC_CNTL_DG_PAD_AUTOHOLD_S  9

/* RTC_CNTL_DIG_ISO_FORCE_ON : R/W; bitpos: [8]; default: 0; */

#define RTC_CNTL_DIG_ISO_FORCE_ON    (BIT(8))
#define RTC_CNTL_DIG_ISO_FORCE_ON_M  (RTC_CNTL_DIG_ISO_FORCE_ON_V << RTC_CNTL_DIG_ISO_FORCE_ON_S)
#define RTC_CNTL_DIG_ISO_FORCE_ON_V  0x00000001
#define RTC_CNTL_DIG_ISO_FORCE_ON_S  8

/* RTC_CNTL_DIG_ISO_FORCE_OFF : R/W; bitpos: [7]; default: 0; */

#define RTC_CNTL_DIG_ISO_FORCE_OFF    (BIT(7))
#define RTC_CNTL_DIG_ISO_FORCE_OFF_M  (RTC_CNTL_DIG_ISO_FORCE_OFF_V << RTC_CNTL_DIG_ISO_FORCE_OFF_S)
#define RTC_CNTL_DIG_ISO_FORCE_OFF_V  0x00000001
#define RTC_CNTL_DIG_ISO_FORCE_OFF_S  7

/* RTC_CNTL_WDTCONFIG0_REG register
 * configure rtc watch dog register
 */

#define RTC_CNTL_WDTCONFIG0_REG (DR_REG_RTCCNTL_BASE + 0x94)

/* RTC_CNTL_WDT_EN : R/W; bitpos: [31]; default: 0;
 * enable rtc wdt
 */

#define RTC_CNTL_WDT_EN    (BIT(31))
#define RTC_CNTL_WDT_EN_M  (RTC_CNTL_WDT_EN_V << RTC_CNTL_WDT_EN_S)
#define RTC_CNTL_WDT_EN_V  0x00000001
#define RTC_CNTL_WDT_EN_S  31

/* RTC_CNTL_WDT_STG0 : R/W; bitpos: [30:28]; default: 0;
 * 1: interrupt stage en  2: CPU reset stage en  3: system reset stage en
 * 4: RTC reset stage en
 */

#define RTC_CNTL_WDT_STG0    0x00000007
#define RTC_CNTL_WDT_STG0_M  (RTC_CNTL_WDT_STG0_V << RTC_CNTL_WDT_STG0_S)
#define RTC_CNTL_WDT_STG0_V  0x00000007
#define RTC_CNTL_WDT_STG0_S  28

/* RTC_CNTL_WDT_STG1 : R/W; bitpos: [27:25]; default: 0;
 * 1: interrupt stage en  2: CPU reset stage en  3: system reset stage en
 * 4: RTC reset stage en
 */

#define RTC_CNTL_WDT_STG1    0x00000007
#define RTC_CNTL_WDT_STG1_M  (RTC_CNTL_WDT_STG1_V << RTC_CNTL_WDT_STG1_S)
#define RTC_CNTL_WDT_STG1_V  0x00000007
#define RTC_CNTL_WDT_STG1_S  25

/* RTC_CNTL_WDT_STG2 : R/W; bitpos: [24:22]; default: 0;
 * 1: interrupt stage en  2: CPU reset stage en  3: system reset stage en
 * 4: RTC reset stage en
 */

#define RTC_CNTL_WDT_STG2    0x00000007
#define RTC_CNTL_WDT_STG2_M  (RTC_CNTL_WDT_STG2_V << RTC_CNTL_WDT_STG2_S)
#define RTC_CNTL_WDT_STG2_V  0x00000007
#define RTC_CNTL_WDT_STG2_S  22

/* RTC_CNTL_WDT_STG3 : R/W; bitpos: [21:19]; default: 0;
 * 1: interrupt stage en  2: CPU reset stage en  3: system reset stage en
 * 4: RTC reset stage en
 */

#define RTC_CNTL_WDT_STG3    0x00000007
#define RTC_CNTL_WDT_STG3_M  (RTC_CNTL_WDT_STG3_V << RTC_CNTL_WDT_STG3_S)
#define RTC_CNTL_WDT_STG3_V  0x00000007
#define RTC_CNTL_WDT_STG3_S  19

/* RTC_CNTL_WDT_STGX :
 * description: stage action selection values
 */

#define RTC_WDT_STG_SEL_OFF             0
#define RTC_WDT_STG_SEL_INT             1
#define RTC_WDT_STG_SEL_RESET_CPU       2
#define RTC_WDT_STG_SEL_RESET_SYSTEM    3
#define RTC_WDT_STG_SEL_RESET_RTC       4

/* RTC_CNTL_WDT_CPU_RESET_LENGTH : R/W; bitpos: [18:16]; default: 1;
 * CPU reset counter length
 */

#define RTC_CNTL_WDT_CPU_RESET_LENGTH    0x00000007
#define RTC_CNTL_WDT_CPU_RESET_LENGTH_M  (RTC_CNTL_WDT_CPU_RESET_LENGTH_V << RTC_CNTL_WDT_CPU_RESET_LENGTH_S)
#define RTC_CNTL_WDT_CPU_RESET_LENGTH_V  0x00000007
#define RTC_CNTL_WDT_CPU_RESET_LENGTH_S  16

/* RTC_CNTL_WDT_SYS_RESET_LENGTH : R/W; bitpos: [15:13]; default: 1;
 * system reset counter length
 */

#define RTC_CNTL_WDT_SYS_RESET_LENGTH    0x00000007
#define RTC_CNTL_WDT_SYS_RESET_LENGTH_M  (RTC_CNTL_WDT_SYS_RESET_LENGTH_V << RTC_CNTL_WDT_SYS_RESET_LENGTH_S)
#define RTC_CNTL_WDT_SYS_RESET_LENGTH_V  0x00000007
#define RTC_CNTL_WDT_SYS_RESET_LENGTH_S  13

/* RTC_CNTL_WDT_FLASHBOOT_MOD_EN : R/W; bitpos: [12]; default: 1;
 * enable WDT in flash boot
 */

#define RTC_CNTL_WDT_FLASHBOOT_MOD_EN    (BIT(12))
#define RTC_CNTL_WDT_FLASHBOOT_MOD_EN_M  (RTC_CNTL_WDT_FLASHBOOT_MOD_EN_V << RTC_CNTL_WDT_FLASHBOOT_MOD_EN_S)
#define RTC_CNTL_WDT_FLASHBOOT_MOD_EN_V  0x00000001
#define RTC_CNTL_WDT_FLASHBOOT_MOD_EN_S  12

/* RTC_CNTL_WDT_PROCPU_RESET_EN : R/W; bitpos: [11]; default: 0;
 * enable WDT reset PRO CPU
 */

#define RTC_CNTL_WDT_PROCPU_RESET_EN    (BIT(11))
#define RTC_CNTL_WDT_PROCPU_RESET_EN_M  (RTC_CNTL_WDT_PROCPU_RESET_EN_V << RTC_CNTL_WDT_PROCPU_RESET_EN_S)
#define RTC_CNTL_WDT_PROCPU_RESET_EN_V  0x00000001
#define RTC_CNTL_WDT_PROCPU_RESET_EN_S  11

/* RTC_CNTL_WDT_APPCPU_RESET_EN : R/W; bitpos: [10]; default: 0;
 * enable WDT reset APP CPU
 */

#define RTC_CNTL_WDT_APPCPU_RESET_EN    (BIT(10))
#define RTC_CNTL_WDT_APPCPU_RESET_EN_M  (RTC_CNTL_WDT_APPCPU_RESET_EN_V << RTC_CNTL_WDT_APPCPU_RESET_EN_S)
#define RTC_CNTL_WDT_APPCPU_RESET_EN_V  0x00000001
#define RTC_CNTL_WDT_APPCPU_RESET_EN_S  10

/* RTC_CNTL_WDT_PAUSE_IN_SLP : R/W; bitpos: [9]; default: 1;
 * pause WDT in sleep
 */

#define RTC_CNTL_WDT_PAUSE_IN_SLP    (BIT(9))
#define RTC_CNTL_WDT_PAUSE_IN_SLP_M  (RTC_CNTL_WDT_PAUSE_IN_SLP_V << RTC_CNTL_WDT_PAUSE_IN_SLP_S)
#define RTC_CNTL_WDT_PAUSE_IN_SLP_V  0x00000001
#define RTC_CNTL_WDT_PAUSE_IN_SLP_S  9

/* RTC_CNTL_WDT_CHIP_RESET_EN : R/W; bitpos: [8]; default: 0;
 * wdt reset whole chip enable
 */

#define RTC_CNTL_WDT_CHIP_RESET_EN    (BIT(8))
#define RTC_CNTL_WDT_CHIP_RESET_EN_M  (RTC_CNTL_WDT_CHIP_RESET_EN_V << RTC_CNTL_WDT_CHIP_RESET_EN_S)
#define RTC_CNTL_WDT_CHIP_RESET_EN_V  0x00000001
#define RTC_CNTL_WDT_CHIP_RESET_EN_S  8

/* RTC_CNTL_WDT_CHIP_RESET_WIDTH : R/W; bitpos: [7:0]; default: 20;
 * chip reset siginal pulse width
 */

#define RTC_CNTL_WDT_CHIP_RESET_WIDTH    0x000000FF
#define RTC_CNTL_WDT_CHIP_RESET_WIDTH_M  (RTC_CNTL_WDT_CHIP_RESET_WIDTH_V << RTC_CNTL_WDT_CHIP_RESET_WIDTH_S)
#define RTC_CNTL_WDT_CHIP_RESET_WIDTH_V  0x000000FF
#define RTC_CNTL_WDT_CHIP_RESET_WIDTH_S  0

/* RTC_CNTL_WDTCONFIG1_REG register
 * Configure hold time of rtc wdt  at level1
 */

#define RTC_CNTL_WDTCONFIG1_REG (DR_REG_RTCCNTL_BASE + 0x98)

/* RTC_CNTL_WDT_STG0_HOLD : R/W; bitpos: [31:0]; default: 200000;
 * Configure hold time of rtc wdt  at level1
 */

#define RTC_CNTL_WDT_STG0_HOLD    0xFFFFFFFF
#define RTC_CNTL_WDT_STG0_HOLD_M  (RTC_CNTL_WDT_STG0_HOLD_V << RTC_CNTL_WDT_STG0_HOLD_S)
#define RTC_CNTL_WDT_STG0_HOLD_V  0xFFFFFFFF
#define RTC_CNTL_WDT_STG0_HOLD_S  0

/* RTC_CNTL_WDTCONFIG2_REG register
 * Configure hold time of rtc wdt  at level2
 */

#define RTC_CNTL_WDTCONFIG2_REG (DR_REG_RTCCNTL_BASE + 0x9c)

/* RTC_CNTL_WDT_STG1_HOLD : R/W; bitpos: [31:0]; default: 80000;
 * Configure hold time of rtc wdt  at level2
 */

#define RTC_CNTL_WDT_STG1_HOLD    0xFFFFFFFF
#define RTC_CNTL_WDT_STG1_HOLD_M  (RTC_CNTL_WDT_STG1_HOLD_V << RTC_CNTL_WDT_STG1_HOLD_S)
#define RTC_CNTL_WDT_STG1_HOLD_V  0xFFFFFFFF
#define RTC_CNTL_WDT_STG1_HOLD_S  0

/* RTC_CNTL_WDTCONFIG3_REG register
 * Configure hold time of rtc wdt  at level3
 */

#define RTC_CNTL_WDTCONFIG3_REG (DR_REG_RTCCNTL_BASE + 0xa0)

/* RTC_CNTL_WDT_STG2_HOLD : R/W; bitpos: [31:0]; default: 4095;
 * Configure hold time of rtc wdt  at level3
 */

#define RTC_CNTL_WDT_STG2_HOLD    0xFFFFFFFF
#define RTC_CNTL_WDT_STG2_HOLD_M  (RTC_CNTL_WDT_STG2_HOLD_V << RTC_CNTL_WDT_STG2_HOLD_S)
#define RTC_CNTL_WDT_STG2_HOLD_V  0xFFFFFFFF
#define RTC_CNTL_WDT_STG2_HOLD_S  0

/* RTC_CNTL_WDTCONFIG4_REG register
 * Configure hold time of rtc wdt  at level4
 */

#define RTC_CNTL_WDTCONFIG4_REG (DR_REG_RTCCNTL_BASE + 0xa4)

/* RTC_CNTL_WDT_STG3_HOLD : R/W; bitpos: [31:0]; default: 4095;
 * Configure hold time of rtc wdt  at level4
 */

#define RTC_CNTL_WDT_STG3_HOLD    0xFFFFFFFF
#define RTC_CNTL_WDT_STG3_HOLD_M  (RTC_CNTL_WDT_STG3_HOLD_V << RTC_CNTL_WDT_STG3_HOLD_S)
#define RTC_CNTL_WDT_STG3_HOLD_V  0xFFFFFFFF
#define RTC_CNTL_WDT_STG3_HOLD_S  0

/* RTC_CNTL_WDTFEED_REG register
 * feed rtc wdt by sw
 */

#define RTC_CNTL_WDTFEED_REG (DR_REG_RTCCNTL_BASE + 0xa8)

/* RTC_CNTL_WDT_FEED : WO; bitpos: [31]; default: 0;
 * Set 1 to feed rtc wdt
 */

#define RTC_CNTL_WDT_FEED    (BIT(31))
#define RTC_CNTL_WDT_FEED_M  (RTC_CNTL_WDT_FEED_V << RTC_CNTL_WDT_FEED_S)
#define RTC_CNTL_WDT_FEED_V  0x00000001
#define RTC_CNTL_WDT_FEED_S  31

/* RTC_CNTL_WDTWPROTECT_REG register
 * configure rtc wdt write protect
 */

#define RTC_CNTL_WDTWPROTECT_REG (DR_REG_RTCCNTL_BASE + 0xac)

/* RTC_CNTL_WDT_WKEY : R/W; bitpos: [31:0]; default: 1356348065;
 * wdt_wprotectn
 */

#define RTC_CNTL_WDT_WKEY    0xFFFFFFFF
#define RTC_CNTL_WDT_WKEY_M  (RTC_CNTL_WDT_WKEY_V << RTC_CNTL_WDT_WKEY_S)
#define RTC_CNTL_WDT_WKEY_V  0xFFFFFFFF
#define RTC_CNTL_WDT_WKEY_S  0

/* RTC_CNTL_SWD_CONF_REG register
 * configure super watch dog
 */

#define RTC_CNTL_SWD_CONF_REG (DR_REG_RTCCNTL_BASE + 0xb0)

/* RTC_CNTL_SWD_AUTO_FEED_EN : R/W; bitpos: [31]; default: 0;
 * automatically feed swd when int comes
 */

#define RTC_CNTL_SWD_AUTO_FEED_EN    (BIT(31))
#define RTC_CNTL_SWD_AUTO_FEED_EN_M  (RTC_CNTL_SWD_AUTO_FEED_EN_V << RTC_CNTL_SWD_AUTO_FEED_EN_S)
#define RTC_CNTL_SWD_AUTO_FEED_EN_V  0x00000001
#define RTC_CNTL_SWD_AUTO_FEED_EN_S  31

/* RTC_CNTL_SWD_DISABLE : R/W; bitpos: [30]; default: 0;
 * disabel SWD
 */

#define RTC_CNTL_SWD_DISABLE    (BIT(30))
#define RTC_CNTL_SWD_DISABLE_M  (RTC_CNTL_SWD_DISABLE_V << RTC_CNTL_SWD_DISABLE_S)
#define RTC_CNTL_SWD_DISABLE_V  0x00000001
#define RTC_CNTL_SWD_DISABLE_S  30

/* RTC_CNTL_SWD_FEED : WO; bitpos: [29]; default: 0;
 * Sw feed swd
 */

#define RTC_CNTL_SWD_FEED    (BIT(29))
#define RTC_CNTL_SWD_FEED_M  (RTC_CNTL_SWD_FEED_V << RTC_CNTL_SWD_FEED_S)
#define RTC_CNTL_SWD_FEED_V  0x00000001
#define RTC_CNTL_SWD_FEED_S  29

/* RTC_CNTL_SWD_RST_FLAG_CLR : WO; bitpos: [28]; default: 0;
 * reset swd reset flag
 */

#define RTC_CNTL_SWD_RST_FLAG_CLR    (BIT(28))
#define RTC_CNTL_SWD_RST_FLAG_CLR_M  (RTC_CNTL_SWD_RST_FLAG_CLR_V << RTC_CNTL_SWD_RST_FLAG_CLR_S)
#define RTC_CNTL_SWD_RST_FLAG_CLR_V  0x00000001
#define RTC_CNTL_SWD_RST_FLAG_CLR_S  28

/* RTC_CNTL_SWD_SIGNAL_WIDTH : R/W; bitpos: [27:18]; default: 300;
 * adjust signal width send to swd
 */

#define RTC_CNTL_SWD_SIGNAL_WIDTH    0x000003FF
#define RTC_CNTL_SWD_SIGNAL_WIDTH_M  (RTC_CNTL_SWD_SIGNAL_WIDTH_V << RTC_CNTL_SWD_SIGNAL_WIDTH_S)
#define RTC_CNTL_SWD_SIGNAL_WIDTH_V  0x000003FF
#define RTC_CNTL_SWD_SIGNAL_WIDTH_S  18

/* RTC_CNTL_SWD_FEED_INT : RO; bitpos: [1]; default: 0;
 * swd interrupt for feeding
 */

#define RTC_CNTL_SWD_FEED_INT    (BIT(1))
#define RTC_CNTL_SWD_FEED_INT_M  (RTC_CNTL_SWD_FEED_INT_V << RTC_CNTL_SWD_FEED_INT_S)
#define RTC_CNTL_SWD_FEED_INT_V  0x00000001
#define RTC_CNTL_SWD_FEED_INT_S  1

/* RTC_CNTL_SWD_RESET_FLAG : RO; bitpos: [0]; default: 0;
 * swd reset flag
 */

#define RTC_CNTL_SWD_RESET_FLAG    (BIT(0))
#define RTC_CNTL_SWD_RESET_FLAG_M  (RTC_CNTL_SWD_RESET_FLAG_V << RTC_CNTL_SWD_RESET_FLAG_S)
#define RTC_CNTL_SWD_RESET_FLAG_V  0x00000001
#define RTC_CNTL_SWD_RESET_FLAG_S  0

/* RTC_CNTL_SWD_WPROTECT_REG register
 * configure super watch dog write protect
 */

#define RTC_CNTL_SWD_WPROTECT_REG (DR_REG_RTCCNTL_BASE + 0xb4)

/* RTC_CNTL_SWD_WKEY : R/W; bitpos: [31:0]; default: 2401055018;
 * swd write protect
 */

#define RTC_CNTL_SWD_WKEY    0xFFFFFFFF
#define RTC_CNTL_SWD_WKEY_M  (RTC_CNTL_SWD_WKEY_V << RTC_CNTL_SWD_WKEY_S)
#define RTC_CNTL_SWD_WKEY_V  0xFFFFFFFF
#define RTC_CNTL_SWD_WKEY_S  0

/* RTC_CNTL_SW_CPU_STALL_REG register
 * configure cpu stall register
 */

#define RTC_CNTL_SW_CPU_STALL_REG (DR_REG_RTCCNTL_BASE + 0xb8)

/* RTC_CNTL_SW_STALL_PROCPU_C1 : R/W; bitpos: [31:26]; default: 0;
 * enable cpu enter stall status by sw
 */

#define RTC_CNTL_SW_STALL_PROCPU_C1    0x0000003F
#define RTC_CNTL_SW_STALL_PROCPU_C1_M  (RTC_CNTL_SW_STALL_PROCPU_C1_V << RTC_CNTL_SW_STALL_PROCPU_C1_S)
#define RTC_CNTL_SW_STALL_PROCPU_C1_V  0x0000003F
#define RTC_CNTL_SW_STALL_PROCPU_C1_S  26

/* RTC_CNTL_SW_STALL_APPCPU_C1 : R/W; bitpos: [25:20]; default: 0;
 * {reg_sw_stall_appcpu_c1[5:0]  reg_sw_stall_appcpu_c0[1:0]} == 0x86 will
 * stall APP CPU
 */

#define RTC_CNTL_SW_STALL_APPCPU_C1    0x0000003F
#define RTC_CNTL_SW_STALL_APPCPU_C1_M  (RTC_CNTL_SW_STALL_APPCPU_C1_V << RTC_CNTL_SW_STALL_APPCPU_C1_S)
#define RTC_CNTL_SW_STALL_APPCPU_C1_V  0x0000003F
#define RTC_CNTL_SW_STALL_APPCPU_C1_S  20

/* RTC_CNTL_STORE4_REG register
 * reservation register4
 */

#define RTC_CNTL_STORE4_REG (DR_REG_RTCCNTL_BASE + 0xbc)

/* RTC_CNTL_SCRATCH4 : R/W; bitpos: [31:0]; default: 0;
 * reservation register4
 */

#define RTC_CNTL_SCRATCH4    0xFFFFFFFF
#define RTC_CNTL_SCRATCH4_M  (RTC_CNTL_SCRATCH4_V << RTC_CNTL_SCRATCH4_S)
#define RTC_CNTL_SCRATCH4_V  0xFFFFFFFF
#define RTC_CNTL_SCRATCH4_S  0

/* RTC_CNTL_STORE5_REG register
 * reservation register5
 */

#define RTC_CNTL_STORE5_REG (DR_REG_RTCCNTL_BASE + 0xc0)

/* RTC_CNTL_SCRATCH5 : R/W; bitpos: [31:0]; default: 0;
 * reservation register5
 */

#define RTC_CNTL_SCRATCH5    0xFFFFFFFF
#define RTC_CNTL_SCRATCH5_M  (RTC_CNTL_SCRATCH5_V << RTC_CNTL_SCRATCH5_S)
#define RTC_CNTL_SCRATCH5_V  0xFFFFFFFF
#define RTC_CNTL_SCRATCH5_S  0

/* RTC_CNTL_STORE6_REG register
 * reservation register6
 */

#define RTC_CNTL_STORE6_REG (DR_REG_RTCCNTL_BASE + 0xc4)

/* RTC_CNTL_SCRATCH6 : R/W; bitpos: [31:0]; default: 0;
 * reservation register6
 */

#define RTC_CNTL_SCRATCH6    0xFFFFFFFF
#define RTC_CNTL_SCRATCH6_M  (RTC_CNTL_SCRATCH6_V << RTC_CNTL_SCRATCH6_S)
#define RTC_CNTL_SCRATCH6_V  0xFFFFFFFF
#define RTC_CNTL_SCRATCH6_S  0

/* RTC_CNTL_STORE7_REG register
 * reservation register7
 */

#define RTC_CNTL_STORE7_REG (DR_REG_RTCCNTL_BASE + 0xc8)

/* RTC_CNTL_SCRATCH7 : R/W; bitpos: [31:0]; default: 0;
 * reservation register7
 */

#define RTC_CNTL_SCRATCH7    0xFFFFFFFF
#define RTC_CNTL_SCRATCH7_M  (RTC_CNTL_SCRATCH7_V << RTC_CNTL_SCRATCH7_S)
#define RTC_CNTL_SCRATCH7_V  0xFFFFFFFF
#define RTC_CNTL_SCRATCH7_S  0

/* RTC_CNTL_LOW_POWER_ST_REG register
 * rtc main state machine status
 */

#define RTC_CNTL_LOW_POWER_ST_REG (DR_REG_RTCCNTL_BASE + 0xcc)

/* RTC_CNTL_MAIN_STATE : RO; bitpos: [31:28]; default: 0;
 * rtc main state machine status
 */

#define RTC_CNTL_MAIN_STATE    0x0000000F
#define RTC_CNTL_MAIN_STATE_M  (RTC_CNTL_MAIN_STATE_V << RTC_CNTL_MAIN_STATE_S)
#define RTC_CNTL_MAIN_STATE_V  0x0000000F
#define RTC_CNTL_MAIN_STATE_S  28

/* RTC_CNTL_MAIN_STATE_IN_IDLE : RO; bitpos: [27]; default: 0;
 * rtc main state machine is in idle state
 */

#define RTC_CNTL_MAIN_STATE_IN_IDLE    (BIT(27))
#define RTC_CNTL_MAIN_STATE_IN_IDLE_M  (RTC_CNTL_MAIN_STATE_IN_IDLE_V << RTC_CNTL_MAIN_STATE_IN_IDLE_S)
#define RTC_CNTL_MAIN_STATE_IN_IDLE_V  0x00000001
#define RTC_CNTL_MAIN_STATE_IN_IDLE_S  27

/* RTC_CNTL_MAIN_STATE_IN_SLP : RO; bitpos: [26]; default: 0;
 * rtc main state machine is in sleep state
 */

#define RTC_CNTL_MAIN_STATE_IN_SLP    (BIT(26))
#define RTC_CNTL_MAIN_STATE_IN_SLP_M  (RTC_CNTL_MAIN_STATE_IN_SLP_V << RTC_CNTL_MAIN_STATE_IN_SLP_S)
#define RTC_CNTL_MAIN_STATE_IN_SLP_V  0x00000001
#define RTC_CNTL_MAIN_STATE_IN_SLP_S  26

/* RTC_CNTL_MAIN_STATE_IN_WAIT_XTL : RO; bitpos: [25]; default: 0;
 * rtc main state machine is in wait xtal state
 */

#define RTC_CNTL_MAIN_STATE_IN_WAIT_XTL    (BIT(25))
#define RTC_CNTL_MAIN_STATE_IN_WAIT_XTL_M  (RTC_CNTL_MAIN_STATE_IN_WAIT_XTL_V << RTC_CNTL_MAIN_STATE_IN_WAIT_XTL_S)
#define RTC_CNTL_MAIN_STATE_IN_WAIT_XTL_V  0x00000001
#define RTC_CNTL_MAIN_STATE_IN_WAIT_XTL_S  25

/* RTC_CNTL_MAIN_STATE_IN_WAIT_PLL : RO; bitpos: [24]; default: 0;
 * rtc main state machine is in wait pll state
 */

#define RTC_CNTL_MAIN_STATE_IN_WAIT_PLL    (BIT(24))
#define RTC_CNTL_MAIN_STATE_IN_WAIT_PLL_M  (RTC_CNTL_MAIN_STATE_IN_WAIT_PLL_V << RTC_CNTL_MAIN_STATE_IN_WAIT_PLL_S)
#define RTC_CNTL_MAIN_STATE_IN_WAIT_PLL_V  0x00000001
#define RTC_CNTL_MAIN_STATE_IN_WAIT_PLL_S  24

/* RTC_CNTL_MAIN_STATE_IN_WAIT_8M : RO; bitpos: [23]; default: 0;
 * rtc main state machine is in wait 8m state
 */

#define RTC_CNTL_MAIN_STATE_IN_WAIT_8M    (BIT(23))
#define RTC_CNTL_MAIN_STATE_IN_WAIT_8M_M  (RTC_CNTL_MAIN_STATE_IN_WAIT_8M_V << RTC_CNTL_MAIN_STATE_IN_WAIT_8M_S)
#define RTC_CNTL_MAIN_STATE_IN_WAIT_8M_V  0x00000001
#define RTC_CNTL_MAIN_STATE_IN_WAIT_8M_S  23

/* RTC_CNTL_IN_LOW_POWER_STATE : RO; bitpos: [22]; default: 0;
 * rtc main state machine is in the states of low power
 */

#define RTC_CNTL_IN_LOW_POWER_STATE    (BIT(22))
#define RTC_CNTL_IN_LOW_POWER_STATE_M  (RTC_CNTL_IN_LOW_POWER_STATE_V << RTC_CNTL_IN_LOW_POWER_STATE_S)
#define RTC_CNTL_IN_LOW_POWER_STATE_V  0x00000001
#define RTC_CNTL_IN_LOW_POWER_STATE_S  22

/* RTC_CNTL_IN_WAKEUP_STATE : RO; bitpos: [21]; default: 0;
 * rtc main state machine is in the states of wakeup process
 */

#define RTC_CNTL_IN_WAKEUP_STATE    (BIT(21))
#define RTC_CNTL_IN_WAKEUP_STATE_M  (RTC_CNTL_IN_WAKEUP_STATE_V << RTC_CNTL_IN_WAKEUP_STATE_S)
#define RTC_CNTL_IN_WAKEUP_STATE_V  0x00000001
#define RTC_CNTL_IN_WAKEUP_STATE_S  21

/* RTC_CNTL_MAIN_STATE_WAIT_END : RO; bitpos: [20]; default: 0;
 * rtc main state machine has been waited for some cycles
 */

#define RTC_CNTL_MAIN_STATE_WAIT_END    (BIT(20))
#define RTC_CNTL_MAIN_STATE_WAIT_END_M  (RTC_CNTL_MAIN_STATE_WAIT_END_V << RTC_CNTL_MAIN_STATE_WAIT_END_S)
#define RTC_CNTL_MAIN_STATE_WAIT_END_V  0x00000001
#define RTC_CNTL_MAIN_STATE_WAIT_END_S  20

/* RTC_CNTL_RDY_FOR_WAKEUP : RO; bitpos: [19]; default: 0;
 * rtc is ready to receive wake up trigger from wake up source
 */

#define RTC_CNTL_RDY_FOR_WAKEUP    (BIT(19))
#define RTC_CNTL_RDY_FOR_WAKEUP_M  (RTC_CNTL_RDY_FOR_WAKEUP_V << RTC_CNTL_RDY_FOR_WAKEUP_S)
#define RTC_CNTL_RDY_FOR_WAKEUP_V  0x00000001
#define RTC_CNTL_RDY_FOR_WAKEUP_S  19

/* RTC_CNTL_MAIN_STATE_PLL_ON : RO; bitpos: [18]; default: 0;
 * rtc main state machine is in states that pll should be running
 */

#define RTC_CNTL_MAIN_STATE_PLL_ON    (BIT(18))
#define RTC_CNTL_MAIN_STATE_PLL_ON_M  (RTC_CNTL_MAIN_STATE_PLL_ON_V << RTC_CNTL_MAIN_STATE_PLL_ON_S)
#define RTC_CNTL_MAIN_STATE_PLL_ON_V  0x00000001
#define RTC_CNTL_MAIN_STATE_PLL_ON_S  18

/* RTC_CNTL_MAIN_STATE_XTAL_ISO : RO; bitpos: [17]; default: 0;
 * no use any more
 */

#define RTC_CNTL_MAIN_STATE_XTAL_ISO    (BIT(17))
#define RTC_CNTL_MAIN_STATE_XTAL_ISO_M  (RTC_CNTL_MAIN_STATE_XTAL_ISO_V << RTC_CNTL_MAIN_STATE_XTAL_ISO_S)
#define RTC_CNTL_MAIN_STATE_XTAL_ISO_V  0x00000001
#define RTC_CNTL_MAIN_STATE_XTAL_ISO_S  17

/* RTC_CNTL_COCPU_STATE_DONE : RO; bitpos: [16]; default: 0;
 * ulp/cocpu is done
 */

#define RTC_CNTL_COCPU_STATE_DONE    (BIT(16))
#define RTC_CNTL_COCPU_STATE_DONE_M  (RTC_CNTL_COCPU_STATE_DONE_V << RTC_CNTL_COCPU_STATE_DONE_S)
#define RTC_CNTL_COCPU_STATE_DONE_V  0x00000001
#define RTC_CNTL_COCPU_STATE_DONE_S  16

/* RTC_CNTL_COCPU_STATE_SLP : RO; bitpos: [15]; default: 0;
 * ulp/cocpu is in sleep state
 */

#define RTC_CNTL_COCPU_STATE_SLP    (BIT(15))
#define RTC_CNTL_COCPU_STATE_SLP_M  (RTC_CNTL_COCPU_STATE_SLP_V << RTC_CNTL_COCPU_STATE_SLP_S)
#define RTC_CNTL_COCPU_STATE_SLP_V  0x00000001
#define RTC_CNTL_COCPU_STATE_SLP_S  15

/* RTC_CNTL_COCPU_STATE_SWITCH : RO; bitpos: [14]; default: 0;
 * ulp/cocpu is about to working. Switch rtc main state
 */

#define RTC_CNTL_COCPU_STATE_SWITCH    (BIT(14))
#define RTC_CNTL_COCPU_STATE_SWITCH_M  (RTC_CNTL_COCPU_STATE_SWITCH_V << RTC_CNTL_COCPU_STATE_SWITCH_S)
#define RTC_CNTL_COCPU_STATE_SWITCH_V  0x00000001
#define RTC_CNTL_COCPU_STATE_SWITCH_S  14

/* RTC_CNTL_COCPU_STATE_START : RO; bitpos: [13]; default: 0;
 * ulp/cocpu should start to work
 */

#define RTC_CNTL_COCPU_STATE_START    (BIT(13))
#define RTC_CNTL_COCPU_STATE_START_M  (RTC_CNTL_COCPU_STATE_START_V << RTC_CNTL_COCPU_STATE_START_S)
#define RTC_CNTL_COCPU_STATE_START_V  0x00000001
#define RTC_CNTL_COCPU_STATE_START_S  13

/* RTC_CNTL_TOUCH_STATE_DONE : RO; bitpos: [12]; default: 0;
 * touch is done
 */

#define RTC_CNTL_TOUCH_STATE_DONE    (BIT(12))
#define RTC_CNTL_TOUCH_STATE_DONE_M  (RTC_CNTL_TOUCH_STATE_DONE_V << RTC_CNTL_TOUCH_STATE_DONE_S)
#define RTC_CNTL_TOUCH_STATE_DONE_V  0x00000001
#define RTC_CNTL_TOUCH_STATE_DONE_S  12

/* RTC_CNTL_TOUCH_STATE_SLP : RO; bitpos: [11]; default: 0;
 * touch is in sleep state
 */

#define RTC_CNTL_TOUCH_STATE_SLP    (BIT(11))
#define RTC_CNTL_TOUCH_STATE_SLP_M  (RTC_CNTL_TOUCH_STATE_SLP_V << RTC_CNTL_TOUCH_STATE_SLP_S)
#define RTC_CNTL_TOUCH_STATE_SLP_V  0x00000001
#define RTC_CNTL_TOUCH_STATE_SLP_S  11

/* RTC_CNTL_TOUCH_STATE_SWITCH : RO; bitpos: [10]; default: 0;
 * touch is about to working. Switch rtc main state
 */

#define RTC_CNTL_TOUCH_STATE_SWITCH    (BIT(10))
#define RTC_CNTL_TOUCH_STATE_SWITCH_M  (RTC_CNTL_TOUCH_STATE_SWITCH_V << RTC_CNTL_TOUCH_STATE_SWITCH_S)
#define RTC_CNTL_TOUCH_STATE_SWITCH_V  0x00000001
#define RTC_CNTL_TOUCH_STATE_SWITCH_S  10

/* RTC_CNTL_TOUCH_STATE_START : RO; bitpos: [9]; default: 0;
 * touch should start to work
 */

#define RTC_CNTL_TOUCH_STATE_START    (BIT(9))
#define RTC_CNTL_TOUCH_STATE_START_M  (RTC_CNTL_TOUCH_STATE_START_V << RTC_CNTL_TOUCH_STATE_START_S)
#define RTC_CNTL_TOUCH_STATE_START_V  0x00000001
#define RTC_CNTL_TOUCH_STATE_START_S  9

/* RTC_CNTL_XPD_DIG : RO; bitpos: [8]; default: 0;
 * digital wrap power down
 */

#define RTC_CNTL_XPD_DIG    (BIT(8))
#define RTC_CNTL_XPD_DIG_M  (RTC_CNTL_XPD_DIG_V << RTC_CNTL_XPD_DIG_S)
#define RTC_CNTL_XPD_DIG_V  0x00000001
#define RTC_CNTL_XPD_DIG_S  8

/* RTC_CNTL_DIG_ISO : RO; bitpos: [7]; default: 0;
 * digital wrap iso
 */

#define RTC_CNTL_DIG_ISO    (BIT(7))
#define RTC_CNTL_DIG_ISO_M  (RTC_CNTL_DIG_ISO_V << RTC_CNTL_DIG_ISO_S)
#define RTC_CNTL_DIG_ISO_V  0x00000001
#define RTC_CNTL_DIG_ISO_S  7

/* RTC_CNTL_XPD_WIFI : RO; bitpos: [6]; default: 0;
 * wifi wrap power down
 */

#define RTC_CNTL_XPD_WIFI    (BIT(6))
#define RTC_CNTL_XPD_WIFI_M  (RTC_CNTL_XPD_WIFI_V << RTC_CNTL_XPD_WIFI_S)
#define RTC_CNTL_XPD_WIFI_V  0x00000001
#define RTC_CNTL_XPD_WIFI_S  6

/* RTC_CNTL_WIFI_ISO : RO; bitpos: [5]; default: 0;
 * wifi iso
 */

#define RTC_CNTL_WIFI_ISO    (BIT(5))
#define RTC_CNTL_WIFI_ISO_M  (RTC_CNTL_WIFI_ISO_V << RTC_CNTL_WIFI_ISO_S)
#define RTC_CNTL_WIFI_ISO_V  0x00000001
#define RTC_CNTL_WIFI_ISO_S  5

/* RTC_CNTL_XPD_RTC_PERI : RO; bitpos: [4]; default: 0;
 * rtc peripheral power down
 */

#define RTC_CNTL_XPD_RTC_PERI    (BIT(4))
#define RTC_CNTL_XPD_RTC_PERI_M  (RTC_CNTL_XPD_RTC_PERI_V << RTC_CNTL_XPD_RTC_PERI_S)
#define RTC_CNTL_XPD_RTC_PERI_V  0x00000001
#define RTC_CNTL_XPD_RTC_PERI_S  4

/* RTC_CNTL_PERI_ISO : RO; bitpos: [3]; default: 0;
 * rtc peripheral iso
 */

#define RTC_CNTL_PERI_ISO    (BIT(3))
#define RTC_CNTL_PERI_ISO_M  (RTC_CNTL_PERI_ISO_V << RTC_CNTL_PERI_ISO_S)
#define RTC_CNTL_PERI_ISO_V  0x00000001
#define RTC_CNTL_PERI_ISO_S  3

/* RTC_CNTL_XPD_DIG_DCDC : RO; bitpos: [2]; default: 0;
 * External DCDC power down
 */

#define RTC_CNTL_XPD_DIG_DCDC    (BIT(2))
#define RTC_CNTL_XPD_DIG_DCDC_M  (RTC_CNTL_XPD_DIG_DCDC_V << RTC_CNTL_XPD_DIG_DCDC_S)
#define RTC_CNTL_XPD_DIG_DCDC_V  0x00000001
#define RTC_CNTL_XPD_DIG_DCDC_S  2

/* RTC_CNTL_XPD_ROM0 : RO; bitpos: [0]; default: 0;
 * rom0 power down
 */

#define RTC_CNTL_XPD_ROM0    (BIT(0))
#define RTC_CNTL_XPD_ROM0_M  (RTC_CNTL_XPD_ROM0_V << RTC_CNTL_XPD_ROM0_S)
#define RTC_CNTL_XPD_ROM0_V  0x00000001
#define RTC_CNTL_XPD_ROM0_S  0

/* RTC_CNTL_DIAG0_REG register
 * debug register
 */

#define RTC_CNTL_DIAG0_REG (DR_REG_RTCCNTL_BASE + 0xd0)

/* RTC_CNTL_LOW_POWER_DIAG1 : RO; bitpos: [31:0]; default: 0; */

#define RTC_CNTL_LOW_POWER_DIAG1    0xFFFFFFFF
#define RTC_CNTL_LOW_POWER_DIAG1_M  (RTC_CNTL_LOW_POWER_DIAG1_V << RTC_CNTL_LOW_POWER_DIAG1_S)
#define RTC_CNTL_LOW_POWER_DIAG1_V  0xFFFFFFFF
#define RTC_CNTL_LOW_POWER_DIAG1_S  0

/* RTC_CNTL_PAD_HOLD_REG register
 * configure rtc pad hold register
 */

#define RTC_CNTL_PAD_HOLD_REG (DR_REG_RTCCNTL_BASE + 0xd4)

/* RTC_CNTL_PAD21_HOLD : R/W; bitpos: [21]; default: 0;
 * set rtc_pad21_hold
 */

#define RTC_CNTL_PAD21_HOLD    (BIT(21))
#define RTC_CNTL_PAD21_HOLD_M  (RTC_CNTL_PAD21_HOLD_V << RTC_CNTL_PAD21_HOLD_S)
#define RTC_CNTL_PAD21_HOLD_V  0x00000001
#define RTC_CNTL_PAD21_HOLD_S  21

/* RTC_CNTL_PAD20_HOLD : R/W; bitpos: [20]; default: 0;
 * set rtc_pad20_hold
 */

#define RTC_CNTL_PAD20_HOLD    (BIT(20))
#define RTC_CNTL_PAD20_HOLD_M  (RTC_CNTL_PAD20_HOLD_V << RTC_CNTL_PAD20_HOLD_S)
#define RTC_CNTL_PAD20_HOLD_V  0x00000001
#define RTC_CNTL_PAD20_HOLD_S  20

/* RTC_CNTL_PAD19_HOLD : R/W; bitpos: [19]; default: 0;
 * set rtc_pad19_hold
 */

#define RTC_CNTL_PAD19_HOLD    (BIT(19))
#define RTC_CNTL_PAD19_HOLD_M  (RTC_CNTL_PAD19_HOLD_V << RTC_CNTL_PAD19_HOLD_S)
#define RTC_CNTL_PAD19_HOLD_V  0x00000001
#define RTC_CNTL_PAD19_HOLD_S  19

/* RTC_CNTL_PDAC2_HOLD : R/W; bitpos: [18]; default: 0;
 * set pdac2_hold
 */

#define RTC_CNTL_PDAC2_HOLD    (BIT(18))
#define RTC_CNTL_PDAC2_HOLD_M  (RTC_CNTL_PDAC2_HOLD_V << RTC_CNTL_PDAC2_HOLD_S)
#define RTC_CNTL_PDAC2_HOLD_V  0x00000001
#define RTC_CNTL_PDAC2_HOLD_S  18

/* RTC_CNTL_PDAC1_HOLD : R/W; bitpos: [17]; default: 0;
 * set pdac1_hold
 */

#define RTC_CNTL_PDAC1_HOLD    (BIT(17))
#define RTC_CNTL_PDAC1_HOLD_M  (RTC_CNTL_PDAC1_HOLD_V << RTC_CNTL_PDAC1_HOLD_S)
#define RTC_CNTL_PDAC1_HOLD_V  0x00000001
#define RTC_CNTL_PDAC1_HOLD_S  17

/* RTC_CNTL_X32N_HOLD : R/W; bitpos: [16]; default: 0;
 * set x32n_hold
 */

#define RTC_CNTL_X32N_HOLD    (BIT(16))
#define RTC_CNTL_X32N_HOLD_M  (RTC_CNTL_X32N_HOLD_V << RTC_CNTL_X32N_HOLD_S)
#define RTC_CNTL_X32N_HOLD_V  0x00000001
#define RTC_CNTL_X32N_HOLD_S  16

/* RTC_CNTL_X32P_HOLD : R/W; bitpos: [15]; default: 0;
 * Set x32p_hold
 */

#define RTC_CNTL_X32P_HOLD    (BIT(15))
#define RTC_CNTL_X32P_HOLD_M  (RTC_CNTL_X32P_HOLD_V << RTC_CNTL_X32P_HOLD_S)
#define RTC_CNTL_X32P_HOLD_V  0x00000001
#define RTC_CNTL_X32P_HOLD_S  15

/* RTC_CNTL_TOUCH_PAD14_HOLD : R/W; bitpos: [14]; default: 0;
 * set touch_pad14_hold
 */

#define RTC_CNTL_TOUCH_PAD14_HOLD    (BIT(14))
#define RTC_CNTL_TOUCH_PAD14_HOLD_M  (RTC_CNTL_TOUCH_PAD14_HOLD_V << RTC_CNTL_TOUCH_PAD14_HOLD_S)
#define RTC_CNTL_TOUCH_PAD14_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD14_HOLD_S  14

/* RTC_CNTL_TOUCH_PAD13_HOLD : R/W; bitpos: [13]; default: 0;
 * set touch_pad13_hold
 */

#define RTC_CNTL_TOUCH_PAD13_HOLD    (BIT(13))
#define RTC_CNTL_TOUCH_PAD13_HOLD_M  (RTC_CNTL_TOUCH_PAD13_HOLD_V << RTC_CNTL_TOUCH_PAD13_HOLD_S)
#define RTC_CNTL_TOUCH_PAD13_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD13_HOLD_S  13

/* RTC_CNTL_TOUCH_PAD12_HOLD : R/W; bitpos: [12]; default: 0;
 * set touch_pad12_hold
 */

#define RTC_CNTL_TOUCH_PAD12_HOLD    (BIT(12))
#define RTC_CNTL_TOUCH_PAD12_HOLD_M  (RTC_CNTL_TOUCH_PAD12_HOLD_V << RTC_CNTL_TOUCH_PAD12_HOLD_S)
#define RTC_CNTL_TOUCH_PAD12_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD12_HOLD_S  12

/* RTC_CNTL_TOUCH_PAD11_HOLD : R/W; bitpos: [11]; default: 0;
 * set touch_pad11_hold
 */

#define RTC_CNTL_TOUCH_PAD11_HOLD    (BIT(11))
#define RTC_CNTL_TOUCH_PAD11_HOLD_M  (RTC_CNTL_TOUCH_PAD11_HOLD_V << RTC_CNTL_TOUCH_PAD11_HOLD_S)
#define RTC_CNTL_TOUCH_PAD11_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD11_HOLD_S  11

/* RTC_CNTL_TOUCH_PAD10_HOLD : R/W; bitpos: [10]; default: 0;
 * set touch_pad10_hold
 */

#define RTC_CNTL_TOUCH_PAD10_HOLD    (BIT(10))
#define RTC_CNTL_TOUCH_PAD10_HOLD_M  (RTC_CNTL_TOUCH_PAD10_HOLD_V << RTC_CNTL_TOUCH_PAD10_HOLD_S)
#define RTC_CNTL_TOUCH_PAD10_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD10_HOLD_S  10

/* RTC_CNTL_TOUCH_PAD9_HOLD : R/W; bitpos: [9]; default: 0;
 * set touch_pad9_hold
 */

#define RTC_CNTL_TOUCH_PAD9_HOLD    (BIT(9))
#define RTC_CNTL_TOUCH_PAD9_HOLD_M  (RTC_CNTL_TOUCH_PAD9_HOLD_V << RTC_CNTL_TOUCH_PAD9_HOLD_S)
#define RTC_CNTL_TOUCH_PAD9_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD9_HOLD_S  9

/* RTC_CNTL_TOUCH_PAD8_HOLD : R/W; bitpos: [8]; default: 0;
 * set touch_pad8_hold
 */

#define RTC_CNTL_TOUCH_PAD8_HOLD    (BIT(8))
#define RTC_CNTL_TOUCH_PAD8_HOLD_M  (RTC_CNTL_TOUCH_PAD8_HOLD_V << RTC_CNTL_TOUCH_PAD8_HOLD_S)
#define RTC_CNTL_TOUCH_PAD8_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD8_HOLD_S  8

/* RTC_CNTL_TOUCH_PAD7_HOLD : R/W; bitpos: [7]; default: 0;
 * set touch_pad7_hold
 */

#define RTC_CNTL_TOUCH_PAD7_HOLD    (BIT(7))
#define RTC_CNTL_TOUCH_PAD7_HOLD_M  (RTC_CNTL_TOUCH_PAD7_HOLD_V << RTC_CNTL_TOUCH_PAD7_HOLD_S)
#define RTC_CNTL_TOUCH_PAD7_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD7_HOLD_S  7

/* RTC_CNTL_TOUCH_PAD6_HOLD : R/W; bitpos: [6]; default: 0;
 * set touch_pad6_hold
 */

#define RTC_CNTL_TOUCH_PAD6_HOLD    (BIT(6))
#define RTC_CNTL_TOUCH_PAD6_HOLD_M  (RTC_CNTL_TOUCH_PAD6_HOLD_V << RTC_CNTL_TOUCH_PAD6_HOLD_S)
#define RTC_CNTL_TOUCH_PAD6_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD6_HOLD_S  6

/* RTC_CNTL_TOUCH_PAD5_HOLD : R/W; bitpos: [5]; default: 0;
 * set touch_pad5_hold
 */

#define RTC_CNTL_TOUCH_PAD5_HOLD    (BIT(5))
#define RTC_CNTL_TOUCH_PAD5_HOLD_M  (RTC_CNTL_TOUCH_PAD5_HOLD_V << RTC_CNTL_TOUCH_PAD5_HOLD_S)
#define RTC_CNTL_TOUCH_PAD5_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD5_HOLD_S  5

/* RTC_CNTL_TOUCH_PAD4_HOLD : R/W; bitpos: [4]; default: 0;
 * set touch_pad4_hold
 */

#define RTC_CNTL_TOUCH_PAD4_HOLD    (BIT(4))
#define RTC_CNTL_TOUCH_PAD4_HOLD_M  (RTC_CNTL_TOUCH_PAD4_HOLD_V << RTC_CNTL_TOUCH_PAD4_HOLD_S)
#define RTC_CNTL_TOUCH_PAD4_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD4_HOLD_S  4

/* RTC_CNTL_TOUCH_PAD3_HOLD : R/W; bitpos: [3]; default: 0;
 * set touch_pad3_hold
 */

#define RTC_CNTL_TOUCH_PAD3_HOLD    (BIT(3))
#define RTC_CNTL_TOUCH_PAD3_HOLD_M  (RTC_CNTL_TOUCH_PAD3_HOLD_V << RTC_CNTL_TOUCH_PAD3_HOLD_S)
#define RTC_CNTL_TOUCH_PAD3_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD3_HOLD_S  3

/* RTC_CNTL_TOUCH_PAD2_HOLD : R/W; bitpos: [2]; default: 0;
 * set touch_pad2_hold
 */

#define RTC_CNTL_TOUCH_PAD2_HOLD    (BIT(2))
#define RTC_CNTL_TOUCH_PAD2_HOLD_M  (RTC_CNTL_TOUCH_PAD2_HOLD_V << RTC_CNTL_TOUCH_PAD2_HOLD_S)
#define RTC_CNTL_TOUCH_PAD2_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD2_HOLD_S  2

/* RTC_CNTL_TOUCH_PAD1_HOLD : R/W; bitpos: [1]; default: 0;
 * set touch_pad1_hold
 */

#define RTC_CNTL_TOUCH_PAD1_HOLD    (BIT(1))
#define RTC_CNTL_TOUCH_PAD1_HOLD_M  (RTC_CNTL_TOUCH_PAD1_HOLD_V << RTC_CNTL_TOUCH_PAD1_HOLD_S)
#define RTC_CNTL_TOUCH_PAD1_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD1_HOLD_S  1

/* RTC_CNTL_TOUCH_PAD0_HOLD : R/W; bitpos: [0]; default: 0;
 * set touch_pad0_hold
 */

#define RTC_CNTL_TOUCH_PAD0_HOLD    (BIT(0))
#define RTC_CNTL_TOUCH_PAD0_HOLD_M  (RTC_CNTL_TOUCH_PAD0_HOLD_V << RTC_CNTL_TOUCH_PAD0_HOLD_S)
#define RTC_CNTL_TOUCH_PAD0_HOLD_V  0x00000001
#define RTC_CNTL_TOUCH_PAD0_HOLD_S  0

/* RTC_CNTL_DIG_PAD_HOLD_REG register
 * configure digital pad hold register
 */

#define RTC_CNTL_DIG_PAD_HOLD_REG (DR_REG_RTCCNTL_BASE + 0xd8)

/* RTC_CNTL_DIG_PAD_HOLD : R/W; bitpos: [31:0]; default: 0;
 * Hold GPIO21~GPIO45 base on bitmap
 */

#define RTC_CNTL_DIG_PAD_HOLD    0xFFFFFFFF
#define RTC_CNTL_DIG_PAD_HOLD_M  (RTC_CNTL_DIG_PAD_HOLD_V << RTC_CNTL_DIG_PAD_HOLD_S)
#define RTC_CNTL_DIG_PAD_HOLD_V  0xFFFFFFFF
#define RTC_CNTL_DIG_PAD_HOLD_S  0

/* RTC_CNTL_EXT_WAKEUP1_REG register
 * configure EXT1 wakeup register
 */

#define RTC_CNTL_EXT_WAKEUP1_REG (DR_REG_RTCCNTL_BASE + 0xdc)

/* RTC_CNTL_EXT_WAKEUP1_STATUS_CLR : WO; bitpos: [22]; default: 0;
 * clear ext wakeup1 status
 */

#define RTC_CNTL_EXT_WAKEUP1_STATUS_CLR    (BIT(22))
#define RTC_CNTL_EXT_WAKEUP1_STATUS_CLR_M  (RTC_CNTL_EXT_WAKEUP1_STATUS_CLR_V << RTC_CNTL_EXT_WAKEUP1_STATUS_CLR_S)
#define RTC_CNTL_EXT_WAKEUP1_STATUS_CLR_V  0x00000001
#define RTC_CNTL_EXT_WAKEUP1_STATUS_CLR_S  22

/* RTC_CNTL_EXT_WAKEUP1_SEL : R/W; bitpos: [21:0]; default: 0;
 * Bitmap to select RTC pads for ext wakeup1
 */

#define RTC_CNTL_EXT_WAKEUP1_SEL    0x003FFFFF
#define RTC_CNTL_EXT_WAKEUP1_SEL_M  (RTC_CNTL_EXT_WAKEUP1_SEL_V << RTC_CNTL_EXT_WAKEUP1_SEL_S)
#define RTC_CNTL_EXT_WAKEUP1_SEL_V  0x003FFFFF
#define RTC_CNTL_EXT_WAKEUP1_SEL_S  0

/* RTC_CNTL_EXT_WAKEUP1_STATUS_REG register
 * EXT1 wakeup source register
 */

#define RTC_CNTL_EXT_WAKEUP1_STATUS_REG (DR_REG_RTCCNTL_BASE + 0xe0)

/* RTC_CNTL_EXT_WAKEUP1_STATUS : RO; bitpos: [21:0]; default: 0;
 * ext wakeup1 status
 */

#define RTC_CNTL_EXT_WAKEUP1_STATUS    0x003FFFFF
#define RTC_CNTL_EXT_WAKEUP1_STATUS_M  (RTC_CNTL_EXT_WAKEUP1_STATUS_V << RTC_CNTL_EXT_WAKEUP1_STATUS_S)
#define RTC_CNTL_EXT_WAKEUP1_STATUS_V  0x003FFFFF
#define RTC_CNTL_EXT_WAKEUP1_STATUS_S  0

/* RTC_CNTL_BROWN_OUT_REG register
 * configure brownout register
 */

#define RTC_CNTL_BROWN_OUT_REG (DR_REG_RTCCNTL_BASE + 0xe4)

/* RTC_CNTL_BROWN_OUT_DET : RO; bitpos: [31]; default: 0;
 * status of brown detcet signal
 */

#define RTC_CNTL_BROWN_OUT_DET    (BIT(31))
#define RTC_CNTL_BROWN_OUT_DET_M  (RTC_CNTL_BROWN_OUT_DET_V << RTC_CNTL_BROWN_OUT_DET_S)
#define RTC_CNTL_BROWN_OUT_DET_V  0x00000001
#define RTC_CNTL_BROWN_OUT_DET_S  31

/* RTC_CNTL_BROWN_OUT_ENA : R/W; bitpos: [30]; default: 0;
 * enable brown out
 */

#define RTC_CNTL_BROWN_OUT_ENA    (BIT(30))
#define RTC_CNTL_BROWN_OUT_ENA_M  (RTC_CNTL_BROWN_OUT_ENA_V << RTC_CNTL_BROWN_OUT_ENA_S)
#define RTC_CNTL_BROWN_OUT_ENA_V  0x00000001
#define RTC_CNTL_BROWN_OUT_ENA_S  30

/* RTC_CNTL_BROWN_OUT_CNT_CLR : WO; bitpos: [29]; default: 0;
 * clear brown out counter
 */

#define RTC_CNTL_BROWN_OUT_CNT_CLR    (BIT(29))
#define RTC_CNTL_BROWN_OUT_CNT_CLR_M  (RTC_CNTL_BROWN_OUT_CNT_CLR_V << RTC_CNTL_BROWN_OUT_CNT_CLR_S)
#define RTC_CNTL_BROWN_OUT_CNT_CLR_V  0x00000001
#define RTC_CNTL_BROWN_OUT_CNT_CLR_S  29

/* RTC_CNTL_BROWN_OUT_RST_SEL : R/W; bitpos: [27]; default: 0;
 * 1:  chip reset     0:  sys_reset
 */

#define RTC_CNTL_BROWN_OUT_RST_SEL    (BIT(27))
#define RTC_CNTL_BROWN_OUT_RST_SEL_M  (RTC_CNTL_BROWN_OUT_RST_SEL_V << RTC_CNTL_BROWN_OUT_RST_SEL_S)
#define RTC_CNTL_BROWN_OUT_RST_SEL_V  0x00000001
#define RTC_CNTL_BROWN_OUT_RST_SEL_S  27

/* RTC_CNTL_BROWN_OUT_RST_ENA : R/W; bitpos: [26]; default: 0;
 * enable brown out reset
 */

#define RTC_CNTL_BROWN_OUT_RST_ENA    (BIT(26))
#define RTC_CNTL_BROWN_OUT_RST_ENA_M  (RTC_CNTL_BROWN_OUT_RST_ENA_V << RTC_CNTL_BROWN_OUT_RST_ENA_S)
#define RTC_CNTL_BROWN_OUT_RST_ENA_V  0x00000001
#define RTC_CNTL_BROWN_OUT_RST_ENA_S  26

/* RTC_CNTL_BROWN_OUT_RST_WAIT : R/W; bitpos: [25:16]; default: 1023;
 * brown out reset wait cycles
 */

#define RTC_CNTL_BROWN_OUT_RST_WAIT    0x000003FF
#define RTC_CNTL_BROWN_OUT_RST_WAIT_M  (RTC_CNTL_BROWN_OUT_RST_WAIT_V << RTC_CNTL_BROWN_OUT_RST_WAIT_S)
#define RTC_CNTL_BROWN_OUT_RST_WAIT_V  0x000003FF
#define RTC_CNTL_BROWN_OUT_RST_WAIT_S  16

/* RTC_CNTL_BROWN_OUT_PD_RF_ENA : R/W; bitpos: [15]; default: 0;
 * enable power down RF when brown out happens
 */

#define RTC_CNTL_BROWN_OUT_PD_RF_ENA    (BIT(15))
#define RTC_CNTL_BROWN_OUT_PD_RF_ENA_M  (RTC_CNTL_BROWN_OUT_PD_RF_ENA_V << RTC_CNTL_BROWN_OUT_PD_RF_ENA_S)
#define RTC_CNTL_BROWN_OUT_PD_RF_ENA_V  0x00000001
#define RTC_CNTL_BROWN_OUT_PD_RF_ENA_S  15

/* RTC_CNTL_BROWN_OUT_CLOSE_FLASH_ENA : R/W; bitpos: [14]; default: 0;
 * enable close flash when brown out happens
 */

#define RTC_CNTL_BROWN_OUT_CLOSE_FLASH_ENA    (BIT(14))
#define RTC_CNTL_BROWN_OUT_CLOSE_FLASH_ENA_M  (RTC_CNTL_BROWN_OUT_CLOSE_FLASH_ENA_V << RTC_CNTL_BROWN_OUT_CLOSE_FLASH_ENA_S)
#define RTC_CNTL_BROWN_OUT_CLOSE_FLASH_ENA_V  0x00000001
#define RTC_CNTL_BROWN_OUT_CLOSE_FLASH_ENA_S  14

/* RTC_CNTL_BROWN_OUT_INT_WAIT : R/W; bitpos: [13:4]; default: 767;
 * brown out interrupt wait cycles
 */

#define RTC_CNTL_BROWN_OUT_INT_WAIT    0x000003FF
#define RTC_CNTL_BROWN_OUT_INT_WAIT_M  (RTC_CNTL_BROWN_OUT_INT_WAIT_V << RTC_CNTL_BROWN_OUT_INT_WAIT_S)
#define RTC_CNTL_BROWN_OUT_INT_WAIT_V  0x000003FF
#define RTC_CNTL_BROWN_OUT_INT_WAIT_S  4

/* RTC_CNTL_BROWN_OUT2_ENA : R/W; bitpos: [0]; default: 1;
 * enable brown_out2 to start chip reset
 */

#define RTC_CNTL_BROWN_OUT2_ENA    (BIT(0))
#define RTC_CNTL_BROWN_OUT2_ENA_M  (RTC_CNTL_BROWN_OUT2_ENA_V << RTC_CNTL_BROWN_OUT2_ENA_S)
#define RTC_CNTL_BROWN_OUT2_ENA_V  0x00000001
#define RTC_CNTL_BROWN_OUT2_ENA_S  0

/* RTC_CNTL_TIME_LOW1_REG register
 * RTC timer1 low 32 bits
 */

#define RTC_CNTL_TIME_LOW1_REG (DR_REG_RTCCNTL_BASE + 0xe8)

/* RTC_CNTL_TIMER_VALUE1_LOW : RO; bitpos: [31:0]; default: 0;
 * RTC timer low 32 bits
 */

#define RTC_CNTL_TIMER_VALUE1_LOW    0xFFFFFFFF
#define RTC_CNTL_TIMER_VALUE1_LOW_M  (RTC_CNTL_TIMER_VALUE1_LOW_V << RTC_CNTL_TIMER_VALUE1_LOW_S)
#define RTC_CNTL_TIMER_VALUE1_LOW_V  0xFFFFFFFF
#define RTC_CNTL_TIMER_VALUE1_LOW_S  0

/* RTC_CNTL_TIME_HIGH1_REG register
 * RTC timer1 high 16 bits
 */

#define RTC_CNTL_TIME_HIGH1_REG (DR_REG_RTCCNTL_BASE + 0xec)

/* RTC_CNTL_TIMER_VALUE1_HIGH : RO; bitpos: [15:0]; default: 0;
 * RTC timer high 16 bits
 */

#define RTC_CNTL_TIMER_VALUE1_HIGH    0x0000FFFF
#define RTC_CNTL_TIMER_VALUE1_HIGH_M  (RTC_CNTL_TIMER_VALUE1_HIGH_V << RTC_CNTL_TIMER_VALUE1_HIGH_S)
#define RTC_CNTL_TIMER_VALUE1_HIGH_V  0x0000FFFF
#define RTC_CNTL_TIMER_VALUE1_HIGH_S  0

/* RTC_CNTL_XTAL32K_CLK_FACTOR_REG register
 * configure xtal32k backup fatcor register
 */

#define RTC_CNTL_XTAL32K_CLK_FACTOR_REG (DR_REG_RTCCNTL_BASE + 0xf0)

/* RTC_CNTL_XTAL32K_CLK_FACTOR : R/W; bitpos: [31:0]; default: 0;
 * xtal 32k watch dog backup clock factor
 */

#define RTC_CNTL_XTAL32K_CLK_FACTOR    0xFFFFFFFF
#define RTC_CNTL_XTAL32K_CLK_FACTOR_M  (RTC_CNTL_XTAL32K_CLK_FACTOR_V << RTC_CNTL_XTAL32K_CLK_FACTOR_S)
#define RTC_CNTL_XTAL32K_CLK_FACTOR_V  0xFFFFFFFF
#define RTC_CNTL_XTAL32K_CLK_FACTOR_S  0

/* RTC_CNTL_XTAL32K_CONF_REG register
 * configure xtal32k register
 */

#define RTC_CNTL_XTAL32K_CONF_REG (DR_REG_RTCCNTL_BASE + 0xf4)

/* RTC_CNTL_XTAL32K_STABLE_THRES : R/W; bitpos: [31:28]; default: 0;
 * if restarted xtal32k period is smaller than this  it is regarded as stable
 */

#define RTC_CNTL_XTAL32K_STABLE_THRES    0x0000000F
#define RTC_CNTL_XTAL32K_STABLE_THRES_M  (RTC_CNTL_XTAL32K_STABLE_THRES_V << RTC_CNTL_XTAL32K_STABLE_THRES_S)
#define RTC_CNTL_XTAL32K_STABLE_THRES_V  0x0000000F
#define RTC_CNTL_XTAL32K_STABLE_THRES_S  28

/* RTC_CNTL_XTAL32K_WDT_TIMEOUT : R/W; bitpos: [27:20]; default: 255;
 * If no clock detected for this amount of time  32k is regarded as dead
 */

#define RTC_CNTL_XTAL32K_WDT_TIMEOUT    0x000000FF
#define RTC_CNTL_XTAL32K_WDT_TIMEOUT_M  (RTC_CNTL_XTAL32K_WDT_TIMEOUT_V << RTC_CNTL_XTAL32K_WDT_TIMEOUT_S)
#define RTC_CNTL_XTAL32K_WDT_TIMEOUT_V  0x000000FF
#define RTC_CNTL_XTAL32K_WDT_TIMEOUT_S  20

/* RTC_CNTL_XTAL32K_RESTART_WAIT : R/W; bitpos: [19:4]; default: 0;
 * cycles to wait to repower on xtal 32k
 */

#define RTC_CNTL_XTAL32K_RESTART_WAIT    0x0000FFFF
#define RTC_CNTL_XTAL32K_RESTART_WAIT_M  (RTC_CNTL_XTAL32K_RESTART_WAIT_V << RTC_CNTL_XTAL32K_RESTART_WAIT_S)
#define RTC_CNTL_XTAL32K_RESTART_WAIT_V  0x0000FFFF
#define RTC_CNTL_XTAL32K_RESTART_WAIT_S  4

/* RTC_CNTL_XTAL32K_RETURN_WAIT : R/W; bitpos: [3:0]; default: 0;
 * cycles to wait to return noral xtal 32k
 */

#define RTC_CNTL_XTAL32K_RETURN_WAIT    0x0000000F
#define RTC_CNTL_XTAL32K_RETURN_WAIT_M  (RTC_CNTL_XTAL32K_RETURN_WAIT_V << RTC_CNTL_XTAL32K_RETURN_WAIT_S)
#define RTC_CNTL_XTAL32K_RETURN_WAIT_V  0x0000000F
#define RTC_CNTL_XTAL32K_RETURN_WAIT_S  0

/* RTC_CNTL_USB_CONF_REG register
 * configure usb control register
 */

#define RTC_CNTL_USB_CONF_REG (DR_REG_RTCCNTL_BASE + 0x11c)

/* RTC_CNTL_IO_MUX_RESET_DISABLE : R/W; bitpos: [18]; default: 0; */

#define RTC_CNTL_IO_MUX_RESET_DISABLE    (BIT(18))
#define RTC_CNTL_IO_MUX_RESET_DISABLE_M  (RTC_CNTL_IO_MUX_RESET_DISABLE_V << RTC_CNTL_IO_MUX_RESET_DISABLE_S)
#define RTC_CNTL_IO_MUX_RESET_DISABLE_V  0x00000001
#define RTC_CNTL_IO_MUX_RESET_DISABLE_S  18

/* RTC_CNTL_USB_RESET_DISABLE : R/W; bitpos: [17]; default: 0; */

#define RTC_CNTL_USB_RESET_DISABLE    (BIT(17))
#define RTC_CNTL_USB_RESET_DISABLE_M  (RTC_CNTL_USB_RESET_DISABLE_V << RTC_CNTL_USB_RESET_DISABLE_S)
#define RTC_CNTL_USB_RESET_DISABLE_V  0x00000001
#define RTC_CNTL_USB_RESET_DISABLE_S  17

/* RTC_CNTL_USB_TX_EN_OVERRIDE : R/W; bitpos: [16]; default: 0; */

#define RTC_CNTL_USB_TX_EN_OVERRIDE    (BIT(16))
#define RTC_CNTL_USB_TX_EN_OVERRIDE_M  (RTC_CNTL_USB_TX_EN_OVERRIDE_V << RTC_CNTL_USB_TX_EN_OVERRIDE_S)
#define RTC_CNTL_USB_TX_EN_OVERRIDE_V  0x00000001
#define RTC_CNTL_USB_TX_EN_OVERRIDE_S  16

/* RTC_CNTL_USB_TX_EN : R/W; bitpos: [15]; default: 0; */

#define RTC_CNTL_USB_TX_EN    (BIT(15))
#define RTC_CNTL_USB_TX_EN_M  (RTC_CNTL_USB_TX_EN_V << RTC_CNTL_USB_TX_EN_S)
#define RTC_CNTL_USB_TX_EN_V  0x00000001
#define RTC_CNTL_USB_TX_EN_S  15

/* RTC_CNTL_USB_TXP : R/W; bitpos: [14]; default: 0; */

#define RTC_CNTL_USB_TXP    (BIT(14))
#define RTC_CNTL_USB_TXP_M  (RTC_CNTL_USB_TXP_V << RTC_CNTL_USB_TXP_S)
#define RTC_CNTL_USB_TXP_V  0x00000001
#define RTC_CNTL_USB_TXP_S  14

/* RTC_CNTL_USB_TXM : R/W; bitpos: [13]; default: 0; */

#define RTC_CNTL_USB_TXM    (BIT(13))
#define RTC_CNTL_USB_TXM_M  (RTC_CNTL_USB_TXM_V << RTC_CNTL_USB_TXM_S)
#define RTC_CNTL_USB_TXM_V  0x00000001
#define RTC_CNTL_USB_TXM_S  13

/* RTC_CNTL_USB_PAD_ENABLE : R/W; bitpos: [12]; default: 0; */

#define RTC_CNTL_USB_PAD_ENABLE    (BIT(12))
#define RTC_CNTL_USB_PAD_ENABLE_M  (RTC_CNTL_USB_PAD_ENABLE_V << RTC_CNTL_USB_PAD_ENABLE_S)
#define RTC_CNTL_USB_PAD_ENABLE_V  0x00000001
#define RTC_CNTL_USB_PAD_ENABLE_S  12

/* RTC_CNTL_USB_PAD_ENABLE_OVERRIDE : R/W; bitpos: [11]; default: 0; */

#define RTC_CNTL_USB_PAD_ENABLE_OVERRIDE    (BIT(11))
#define RTC_CNTL_USB_PAD_ENABLE_OVERRIDE_M  (RTC_CNTL_USB_PAD_ENABLE_OVERRIDE_V << RTC_CNTL_USB_PAD_ENABLE_OVERRIDE_S)
#define RTC_CNTL_USB_PAD_ENABLE_OVERRIDE_V  0x00000001
#define RTC_CNTL_USB_PAD_ENABLE_OVERRIDE_S  11

/* RTC_CNTL_USB_PULLUP_VALUE : R/W; bitpos: [10]; default: 0; */

#define RTC_CNTL_USB_PULLUP_VALUE    (BIT(10))
#define RTC_CNTL_USB_PULLUP_VALUE_M  (RTC_CNTL_USB_PULLUP_VALUE_V << RTC_CNTL_USB_PULLUP_VALUE_S)
#define RTC_CNTL_USB_PULLUP_VALUE_V  0x00000001
#define RTC_CNTL_USB_PULLUP_VALUE_S  10

/* RTC_CNTL_USB_DM_PULLDOWN : R/W; bitpos: [9]; default: 0; */

#define RTC_CNTL_USB_DM_PULLDOWN    (BIT(9))
#define RTC_CNTL_USB_DM_PULLDOWN_M  (RTC_CNTL_USB_DM_PULLDOWN_V << RTC_CNTL_USB_DM_PULLDOWN_S)
#define RTC_CNTL_USB_DM_PULLDOWN_V  0x00000001
#define RTC_CNTL_USB_DM_PULLDOWN_S  9

/* RTC_CNTL_USB_DM_PULLUP : R/W; bitpos: [8]; default: 0; */

#define RTC_CNTL_USB_DM_PULLUP    (BIT(8))
#define RTC_CNTL_USB_DM_PULLUP_M  (RTC_CNTL_USB_DM_PULLUP_V << RTC_CNTL_USB_DM_PULLUP_S)
#define RTC_CNTL_USB_DM_PULLUP_V  0x00000001
#define RTC_CNTL_USB_DM_PULLUP_S  8

/* RTC_CNTL_USB_DP_PULLDOWN : R/W; bitpos: [7]; default: 0; */

#define RTC_CNTL_USB_DP_PULLDOWN    (BIT(7))
#define RTC_CNTL_USB_DP_PULLDOWN_M  (RTC_CNTL_USB_DP_PULLDOWN_V << RTC_CNTL_USB_DP_PULLDOWN_S)
#define RTC_CNTL_USB_DP_PULLDOWN_V  0x00000001
#define RTC_CNTL_USB_DP_PULLDOWN_S  7

/* RTC_CNTL_USB_DP_PULLUP : R/W; bitpos: [6]; default: 0; */

#define RTC_CNTL_USB_DP_PULLUP    (BIT(6))
#define RTC_CNTL_USB_DP_PULLUP_M  (RTC_CNTL_USB_DP_PULLUP_V << RTC_CNTL_USB_DP_PULLUP_S)
#define RTC_CNTL_USB_DP_PULLUP_V  0x00000001
#define RTC_CNTL_USB_DP_PULLUP_S  6

/* RTC_CNTL_USB_PAD_PULL_OVERRIDE : R/W; bitpos: [5]; default: 0; */

#define RTC_CNTL_USB_PAD_PULL_OVERRIDE    (BIT(5))
#define RTC_CNTL_USB_PAD_PULL_OVERRIDE_M  (RTC_CNTL_USB_PAD_PULL_OVERRIDE_V << RTC_CNTL_USB_PAD_PULL_OVERRIDE_S)
#define RTC_CNTL_USB_PAD_PULL_OVERRIDE_V  0x00000001
#define RTC_CNTL_USB_PAD_PULL_OVERRIDE_S  5

/* RTC_CNTL_USB_VREF_OVERRIDE : R/W; bitpos: [4]; default: 0; */

#define RTC_CNTL_USB_VREF_OVERRIDE    (BIT(4))
#define RTC_CNTL_USB_VREF_OVERRIDE_M  (RTC_CNTL_USB_VREF_OVERRIDE_V << RTC_CNTL_USB_VREF_OVERRIDE_S)
#define RTC_CNTL_USB_VREF_OVERRIDE_V  0x00000001
#define RTC_CNTL_USB_VREF_OVERRIDE_S  4

/* RTC_CNTL_USB_VREFL : R/W; bitpos: [3:2]; default: 0; */

#define RTC_CNTL_USB_VREFL    0x00000003
#define RTC_CNTL_USB_VREFL_M  (RTC_CNTL_USB_VREFL_V << RTC_CNTL_USB_VREFL_S)
#define RTC_CNTL_USB_VREFL_V  0x00000003
#define RTC_CNTL_USB_VREFL_S  2

/* RTC_CNTL_USB_VREFH : R/W; bitpos: [1:0]; default: 0; */

#define RTC_CNTL_USB_VREFH    0x00000003
#define RTC_CNTL_USB_VREFH_M  (RTC_CNTL_USB_VREFH_V << RTC_CNTL_USB_VREFH_S)
#define RTC_CNTL_USB_VREFH_V  0x00000003
#define RTC_CNTL_USB_VREFH_S  0

/* RTC_CNTL_SLP_REJECT_CAUSE_REG register
 * sleep reject casue register
 */

#define RTC_CNTL_SLP_REJECT_CAUSE_REG (DR_REG_RTCCNTL_BASE + 0x124)

/* RTC_CNTL_REJECT_CAUSE : RO; bitpos: [16:0]; default: 0;
 * sleep reject cause
 */

#define RTC_CNTL_REJECT_CAUSE    0x0001FFFF
#define RTC_CNTL_REJECT_CAUSE_M  (RTC_CNTL_REJECT_CAUSE_V << RTC_CNTL_REJECT_CAUSE_S)
#define RTC_CNTL_REJECT_CAUSE_V  0x0001FFFF
#define RTC_CNTL_REJECT_CAUSE_S  0

/* RTC_CNTL_OPTION1_REG register
 * configure rtc option
 */

#define RTC_CNTL_OPTION1_REG (DR_REG_RTCCNTL_BASE + 0x128)

/* RTC_CNTL_FORCE_DOWNLOAD_BOOT : R/W; bitpos: [0]; default: 0;
 * force chip boot from download mode
 */

#define RTC_CNTL_FORCE_DOWNLOAD_BOOT    (BIT(0))
#define RTC_CNTL_FORCE_DOWNLOAD_BOOT_M  (RTC_CNTL_FORCE_DOWNLOAD_BOOT_V << RTC_CNTL_FORCE_DOWNLOAD_BOOT_S)
#define RTC_CNTL_FORCE_DOWNLOAD_BOOT_V  0x00000001
#define RTC_CNTL_FORCE_DOWNLOAD_BOOT_S  0

/* RTC_CNTL_SLP_WAKEUP_CAUSE_REG register
 * sleep wakeup cause state register
 */

#define RTC_CNTL_SLP_WAKEUP_CAUSE_REG (DR_REG_RTCCNTL_BASE + 0x12c)

/* RTC_CNTL_WAKEUP_CAUSE : RO; bitpos: [16:0]; default: 0;
 * sleep wakeup cause
 */

#define RTC_CNTL_WAKEUP_CAUSE    0x0001FFFF
#define RTC_CNTL_WAKEUP_CAUSE_M  (RTC_CNTL_WAKEUP_CAUSE_V << RTC_CNTL_WAKEUP_CAUSE_S)
#define RTC_CNTL_WAKEUP_CAUSE_V  0x0001FFFF
#define RTC_CNTL_WAKEUP_CAUSE_S  0

/* RTC_CNTL_DATE_REG register */

#define RTC_CNTL_DATE_REG (DR_REG_RTCCNTL_BASE + 0x138)

/* RTC_CNTL_CNTL_DATE : R/W; bitpos: [27:0]; default: 26239377; */

#define RTC_CNTL_CNTL_DATE    0x0FFFFFFF
#define RTC_CNTL_CNTL_DATE_M  (RTC_CNTL_CNTL_DATE_V << RTC_CNTL_CNTL_DATE_S)
#define RTC_CNTL_CNTL_DATE_V  0x0FFFFFFF
#define RTC_CNTL_CNTL_DATE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_RTCCNTL_H */
