/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_soc.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_SOC_H
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_SOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "xtensa_attr.h"
#include "hardware/esp32_efuse.h"

#include <nuttx/bits.h>

#include "soc/soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Bits */

#define BIT31   0x80000000
#define BIT30   0x40000000
#define BIT29   0x20000000
#define BIT28   0x10000000
#define BIT27   0x08000000
#define BIT26   0x04000000
#define BIT25   0x02000000
#define BIT24   0x01000000
#define BIT23   0x00800000
#define BIT22   0x00400000
#define BIT21   0x00200000
#define BIT20   0x00100000
#define BIT19   0x00080000
#define BIT18   0x00040000
#define BIT17   0x00020000
#define BIT16   0x00010000
#define BIT15   0x00008000
#define BIT14   0x00004000
#define BIT13   0x00002000
#define BIT12   0x00001000
#define BIT11   0x00000800
#define BIT10   0x00000400
#define BIT9    0x00000200
#define BIT8    0x00000100
#define BIT7    0x00000080
#define BIT6    0x00000040
#define BIT5    0x00000020
#define BIT4    0x00000010
#define BIT3    0x00000008
#define BIT2    0x00000004
#define BIT1    0x00000002
#define BIT0    0x00000001

#define PRO_CPU_NUM (0)
#define APP_CPU_NUM (1)

/* Registers Operation */

/* Extract the field from the register and shift it to avoid wrong reading */

#define REG_MASK(_reg, _field) (((_reg) & (_field##_M)) >> (_field##_S))

/* Helper to place a value in a field */

#define VALUE_TO_FIELD(_value, _field) (((_value) << (_field##_S)) & (_field##_M))

/* Periheral Clock */

#define CPU_CLK_FREQ_ROM                        APB_CLK_FREQ_ROM
#define CPU_CLK_FREQ                            APB_CLK_FREQ
#define UART_CLK_FREQ                           APB_CLK_FREQ
#define MWDT_CLK_FREQ                           APB_CLK_FREQ
#define SPI_CLK_DIV                             4
#define TICKS_PER_US_ROM                        26              /* CPU is 80MHz */

#define DR_REG_TWAI_BASE                        DR_REG_CAN_BASE

#define PERIPHS_SPI_ENCRYPT_BASEADDR            DR_REG_SPI_ENCRYPT_BASE

/* Some AHB addresses can be used instead of DPORT addresses
 * as a workaround for some HW bugs.
 * This workaround is detailed at
 * https://www.espressif.com/sites/default/files/documentation/
 * eco_and_workarounds_for_bugs_in_esp32_en.pdf
 */

#define AHB_REG_UART_BASE   0x60000000

/* Overall memory map */

#define SOC_RTC_SLOW_LOW        SOC_RTC_DATA_LOW
#define SOC_RTC_SLOW_HIGH       SOC_RTC_DATA_HIGH

/* Virtual address 0 */

#define VADDR0_START_ADDR       SOC_DROM_LOW
#define VADDR0_END_ADDR         (SOC_DROM_HIGH - 1)

#define GPIO_STRAP_REG                          (DR_REG_GPIO_BASE + 0x0038)

/* Interrupt cpu using table */

/****************************************************************************
 *Intr num  Level     Type          PRO CPU usage     APP CPU uasge
 *   0        1   extern level      WMAC                 Reserved
 *   1        1   extern level      BT/BLE Host VHCI     Reserved
 *   2        1   extern level      FROM_CPU             FROM_CPU
 *   3        1   extern level      TG0_WDT              Reserved
 *   4        1   extern level      WBB
 *   5        1   extern level      BT Controller
 *   6        1   timer             RTOS Tick            RTOS Tick
 *   7        1   software          Reserved             Reserved
 *   8        1   extern level      BLE Controller
 *   9        1   extern level
 *   10       1   extern edge       Internal Timer
 *   11       3   profiling
 *   12       1   extern level
 *   13       1   extern level
 *   14       7   nmi               Reserved             Reserved
 *   15       3   timer             Internal Timer
 *   16       5   timer
 *   17       1   extern level
 *   18       1   extern level
 *   19       2   extern level
 *   20       2   extern level
 *   21       2   extern level
 *   22       3   extern edge
 *   23       3   extern level
 *   24       4   extern level
 *   25       4   extern level      Reserved             Reserved
 *   26       5   extern level      Reserved             Reserved
 *   27       3   extern level      Reserved             Reserved
 *   28       4   extern edge
 *   29       3   software          Reserved             Reserved
 *   30       4   extern edge       Reserved             Reserved
 *   31       5   extern level      Reserved             Reserved
 ****************************************************************************/

/* CPU0 Interrupt number reserved, not touch this. */

#define ETS_WMAC_INUM               0
#define ETS_BT_HOST_INUM            1
#define ETS_FROM_CPU_INUM           2
#define ETS_T0_WDT_INUM             3
#define ETS_WBB_INUM                4
#define ETS_TG0_T1_INUM             10 /* Use edge interrupt */

/* CPU0 Interrupt number used in ROM, should be cancelled in SDK */

#define ETS_SLC_INUM                1
#define ETS_UART0_INUM              5
#define ETS_UART1_INUM              5

/* Other interrupt numbers should be managed by the user */

#define DR_REG_APB_CTRL_BASE        0x3ff66000    /* Old name for SYSCON */
                                                  /* to be removed */
#define APB_CTRL_SYSCLK_CONF_REG    (DR_REG_APB_CTRL_BASE + 0x0)
#define APB_CTRL_XTAL_TICK_CONF_REG (DR_REG_APB_CTRL_BASE + 0x4)

#define I2C_BBPLL_IR_CAL_DELAY      0
#define I2C_BBPLL_IR_CAL_EXT_CAP    1
#define I2C_BBPLL_OC_ENB_FCAL       4
#define I2C_BBPLL_OC_ENB_VCON       10
#define I2C_BBPLL_BBADC_CAL_7_0     12

#define I2C_BBPLL_OC_LREF           2
#define I2C_BBPLL_OC_LREF_MSB       7
#define I2C_BBPLL_OC_LREF_LSB       7

#define I2C_BBPLL_OC_DIV_7_0        3
#define I2C_BBPLL_OC_DIV_7_0_MSB    7
#define I2C_BBPLL_OC_DIV_7_0_LSB    0

#define I2C_BBPLL_BBADC_DSMP        9
#define I2C_BBPLL_BBADC_DSMP_MSB    7
#define I2C_BBPLL_BBADC_DSMP_LSB    4

#define I2C_BBPLL_OC_DCUR           5
#define I2C_BBPLL_OC_DCUR_MSB       2
#define I2C_BBPLL_OC_DCUR_LSB       0

#define I2C_BBPLL_ENDIV5            11

#define I2C_BBPLL                   0x66
#define I2C_BBPLL_HOSTID            4

extern int rom_i2c_writereg(int block, int block_id, int reg_add,
                            int indata);

#define I2C_WRITEREG_RTC(block, reg_add, indata) \
      rom_i2c_writereg(block, block##_HOSTID,  reg_add, indata)

#define I2C_READREG_RTC(block, reg_add) \
      rom_i2c_readreg(block, block##_HOSTID,  reg_add)

/* BBPLL configuration values */

#define BBPLL_ENDIV5_VAL_320M       0x43
#define BBPLL_BBADC_DSMP_VAL_320M   0x84
#define BBPLL_ENDIV5_VAL_480M       0xc3
#define BBPLL_BBADC_DSMP_VAL_480M   0x74

#define BBPLL_IR_CAL_DELAY_VAL      0x18
#define BBPLL_IR_CAL_EXT_CAP_VAL    0x20
#define BBPLL_OC_ENB_FCAL_VAL       0x9a
#define BBPLL_OC_ENB_VCON_VAL       0x00
#define BBPLL_BBADC_CAL_7_0_VAL     0x00

#define REG_TIMG_BASE(i)            (DR_REG_TIMERGROUP0_BASE + i*0x1000)
#define TIMG_RTCCALICFG_REG(i)      (REG_TIMG_BASE(i) + 0x0068)

#define RTC_CNTL_OPTIONS0_REG       (DR_REG_RTCCNTL_BASE + 0x0)
#define RTC_CNTL_STORE5_REG         (DR_REG_RTCCNTL_BASE + 0xb4)

#define RTC_APB_FREQ_REG            RTC_CNTL_STORE5_REG
#define RTC_CNTL_REG                (DR_REG_RTCCNTL_BASE + 0x7c)

#define RTC_CNTL_CLK_CONF_REG       (DR_REG_RTCCNTL_BASE + 0x70)

#define RTC_CNTL_ANA_CONF_REG       (DR_REG_RTCCNTL_BASE + 0x30)

#define RTC_CNTL_STORE4_REG         (DR_REG_RTCCNTL_BASE + 0xb0)
#define RTC_XTAL_FREQ_REG           RTC_CNTL_STORE4_REG

/* Approximate mapping of voltages to RTC_CNTL_DBIAS_WAK, RTC_CNTL_DBIAS_SLP,
 * RTC_CNTL_DIG_DBIAS_WAK, RTC_CNTL_DIG_DBIAS_SLP values.
 * Valid if RTC_CNTL_DBG_ATTEN is 0.
 */

#define RTC_CNTL_DBIAS_1V00         2
#define RTC_CNTL_DBIAS_1V10         4
#define RTC_CNTL_DBIAS_1V25         7

/* RTC_CNTL_DIG_DBIAS_WAK : R/W ;bitpos:[13:11] ;default: 3'd4 ; */

#define RTC_CNTL_DIG_DBIAS_WAK      0x00000007
#define RTC_CNTL_DIG_DBIAS_WAK_M    ((RTC_CNTL_DIG_DBIAS_WAK_V) << (RTC_CNTL_DIG_DBIAS_WAK_S))
#define RTC_CNTL_DIG_DBIAS_WAK_V    0x7
#define RTC_CNTL_DIG_DBIAS_WAK_S    11

/* RTC_CNTL_SOC_CLK_SEL : R/W ;bitpos:[28:27] ;default: 2'd0 ;
 * description: SOC clock sel. 0: XTAL  1: PLL  2: CK8M  3: APLL
 */

#define RTC_CNTL_SOC_CLK_SEL        0x00000003
#define RTC_CNTL_SOC_CLK_SEL_M      ((RTC_CNTL_SOC_CLK_SEL_V) << (RTC_CNTL_SOC_CLK_SEL_S))
#define RTC_CNTL_SOC_CLK_SEL_V      0x3
#define RTC_CNTL_SOC_CLK_SEL_S      27
#define RTC_CNTL_SOC_CLK_SEL_XTL    0
#define RTC_CNTL_SOC_CLK_SEL_PLL    1
#define RTC_CNTL_SOC_CLK_SEL_8M     2
#define RTC_CNTL_SOC_CLK_SEL_APLL   3

/* Core voltage needs to be increased in two cases:
 * 1. running at 240 MHz
 * 2. running with 80MHz Flash frequency
 * There is a record in efuse which indicates the
 * proper voltage for these two cases.
 */

#define RTC_CNTL_DBIAS_HP_VOLT      (RTC_CNTL_DBIAS_1V25 - \
                                    (REG_GET_FIELD(EFUSE_BLK0_RDATA5_REG, \
                                     EFUSE_RD_VOL_LEVEL_HP_INV)))

#ifdef CONFIG_ESP32_FLASH_FREQ_80M
#define DIG_DBIAS_80M_160M          RTC_CNTL_DBIAS_HP_VOLT
#else
#define DIG_DBIAS_80M_160M          RTC_CNTL_DBIAS_1V10
#endif
#define DIG_DBIAS_240M              RTC_CNTL_DBIAS_HP_VOLT
#define DIG_DBIAS_XTAL              RTC_CNTL_DBIAS_1V10
#define DIG_DBIAS_2M                RTC_CNTL_DBIAS_1V00

#define DIG_DBIAS_240M              RTC_CNTL_DBIAS_HP_VOLT
#define DIG_DBIAS_XTAL              RTC_CNTL_DBIAS_1V10
#define DIG_DBIAS_2M                RTC_CNTL_DBIAS_1V00

#define DELAY_PLL_DBIAS_RAISE       3
#define DELAY_PLL_ENABLE_WITH_150K  80
#define DELAY_PLL_ENABLE_WITH_32K   160

/* RTC_CNTL_BB_I2C_FORCE_PD : R/W ;bitpos:[6] ;default: 1'b0 ;
 * description: BB_I2C force power down
 */

#define RTC_CNTL_BB_I2C_FORCE_PD    (BIT(6))

/* RTC_CNTL_BBPLL_FORCE_PD : R/W ;bitpos:[10] ;default: 1'b0 ;
 * description: BB_PLL force power down
 */

#define RTC_CNTL_BBPLL_FORCE_PD     (BIT(10))

/* RTC_CNTL_BBPLL_I2C_FORCE_PD : R/W ;bitpos:[8] ;default: 1'b0 ;
 * description: BB_PLL _I2C force power down
 */

#define RTC_CNTL_BBPLL_I2C_FORCE_PD (BIT(8))

/* RTC_CNTL_PLLA_FORCE_PD : R/W ;bitpos:[23] ;default: 1'b1 ;
 * description: PLLA force power down
 */

#define RTC_CNTL_PLLA_FORCE_PD      (BIT(23))
#define RTC_CNTL_PLLA_FORCE_PD_S    23

/* RTC_CNTL_BIAS_I2C_FORCE_PD : R/W ;bitpos:[18] ;default: 1'b0 ;
 * description: BIAS_I2C force power down
 */

#define RTC_CNTL_BIAS_I2C_FORCE_PD  (BIT(18))

#define MHZ (1000000)
#define RTC_PLL_FREQ_320M           320
#define RTC_PLL_FREQ_480M           480

/* TIMG_RTC_CALI_CLK_SEL : R/W ;bitpos:[14:13] ;default: 2'h1 ; */

#define TIMG_RTC_CALI_CLK_SEL       0x00000003
#define TIMG_RTC_CALI_CLK_SEL_M     ((TIMG_RTC_CALI_CLK_SEL_V) << (TIMG_RTC_CALI_CLK_SEL_S))
#define TIMG_RTC_CALI_CLK_SEL_V     0x3
#define TIMG_RTC_CALI_CLK_SEL_S     13

/* TIMG_RTC_CALI_START_CYCLING : R/W ;bitpos:[12] ;default: 1'd1 ; */

#define TIMG_RTC_CALI_START_CYCLING    (BIT(12))
#define TIMG_RTC_CALI_START_CYCLING_M  (BIT(12))
#define TIMG_RTC_CALI_START_CYCLING_V  0x1
#define TIMG_RTC_CALI_START_CYCLING_S  12

/* TIMG_RTC_CALI_START : R/W ;bitpos:[31] ;default: 1'h0 ; */

#define TIMG_RTC_CALI_START         (BIT(31))
#define TIMG_RTC_CALI_START_M       (BIT(31))
#define TIMG_RTC_CALI_START_V       0x1
#define TIMG_RTC_CALI_START_S       31

/* TIMG_RTC_CALI_MAX : R/W ;bitpos:[30:16] ;default: 15'h1 ; */

#define TIMG_RTC_CALI_MAX           0x00007fff
#define TIMG_RTC_CALI_MAX_M         ((TIMG_RTC_CALI_MAX_V) << (TIMG_RTC_CALI_MAX_S))
#define TIMG_RTC_CALI_MAX_V         0x7fff
#define TIMG_RTC_CALI_MAX_S         16

/* TIMG_RTC_CALI_VALUE : RO ;bitpos:[31:7] ;default: 25'h0 ; */

#define TIMG_RTC_CALI_VALUE         0x01ffffff
#define TIMG_RTC_CALI_VALUE_M       ((TIMG_RTC_CALI_VALUE_V) << (TIMG_RTC_CALI_VALUE_S))
#define TIMG_RTC_CALI_VALUE_V       0x1ffffff
#define TIMG_RTC_CALI_VALUE_S       7

/* TIMG_RTC_CALI_RDY : RO ;bitpos:[15] ;default: 1'h0 ; */

#define TIMG_RTC_CALI_RDY           (BIT(15))
#define TIMG_RTC_CALI_RDY_M         (BIT(15))
#define TIMG_RTC_CALI_RDY_V         0x1
#define TIMG_RTC_CALI_RDY_S         15

#define TIMG_RTCCALICFG1_REG(i)     (REG_TIMG_BASE(i) + 0x006c)

/* Some of the baseband control registers.
 * PU/PD fields defined here are used in sleep related functions.
 */

#define BBPD_CTRL                   (DR_REG_BB_BASE + 0x0054)
#define BB_FFT_FORCE_PU             (BIT(3))
#define BB_FFT_FORCE_PU_M           (BIT(3))
#define BB_FFT_FORCE_PU_V           1
#define BB_FFT_FORCE_PU_S           3
#define BB_FFT_FORCE_PD             (BIT(2))
#define BB_FFT_FORCE_PD_M           (BIT(2))
#define BB_FFT_FORCE_PD_V           1
#define BB_FFT_FORCE_PD_S           2
#define BB_DC_EST_FORCE_PU          (BIT(1))
#define BB_DC_EST_FORCE_PU_M        (BIT(1))
#define BB_DC_EST_FORCE_PU_V        1
#define BB_DC_EST_FORCE_PU_S        1
#define BB_DC_EST_FORCE_PD          (BIT(0))
#define BB_DC_EST_FORCE_PD_M        (BIT(0))
#define BB_DC_EST_FORCE_PD_V        1
#define BB_DC_EST_FORCE_PD_S        0

/* Some of the Wi-Fi RX control registers.
 * PU/PD fields defined here are used in sleep related functions.
 */

#define NRXPD_CTRL                  (DR_REG_NRX_BASE + 0x00d4)
#define NRX_RX_ROT_FORCE_PU         (BIT(5))
#define NRX_RX_ROT_FORCE_PU_M       (BIT(5))
#define NRX_RX_ROT_FORCE_PU_V       1
#define NRX_RX_ROT_FORCE_PU_S       5
#define NRX_RX_ROT_FORCE_PD         (BIT(4))
#define NRX_RX_ROT_FORCE_PD_M       (BIT(4))
#define NRX_RX_ROT_FORCE_PD_V       1
#define NRX_RX_ROT_FORCE_PD_S       4
#define NRX_VIT_FORCE_PU            (BIT(3))
#define NRX_VIT_FORCE_PU_M          (BIT(3))
#define NRX_VIT_FORCE_PU_V          1
#define NRX_VIT_FORCE_PU_S          3
#define NRX_VIT_FORCE_PD            (BIT(2))
#define NRX_VIT_FORCE_PD_M          (BIT(2))
#define NRX_VIT_FORCE_PD_V          1
#define NRX_VIT_FORCE_PD_S          2
#define NRX_DEMAP_FORCE_PU          (BIT(1))
#define NRX_DEMAP_FORCE_PU_M        (BIT(1))
#define NRX_DEMAP_FORCE_PU_V        1
#define NRX_DEMAP_FORCE_PU_S        1
#define NRX_DEMAP_FORCE_PD          (BIT(0))
#define NRX_DEMAP_FORCE_PD_M        (BIT(0))
#define NRX_DEMAP_FORCE_PD_V        1
#define NRX_DEMAP_FORCE_PD_S        0

/* Some of the RF frontend control registers.
 * PU/PD fields defined here are used in sleep related functions.
 */

#define FE_GEN_CTRL                 (DR_REG_FE_BASE + 0x0090)
#define FE_IQ_EST_FORCE_PU          (BIT(5))
#define FE_IQ_EST_FORCE_PU_M        (BIT(5))
#define FE_IQ_EST_FORCE_PU_V        1
#define FE_IQ_EST_FORCE_PU_S        5
#define FE_IQ_EST_FORCE_PD          (BIT(4))
#define FE_IQ_EST_FORCE_PD_M        (BIT(4))
#define FE_IQ_EST_FORCE_PD_V        1
#define FE_IQ_EST_FORCE_PD_S        4

#define FE2_TX_INTERP_CTRL          (DR_REG_FE2_BASE + 0x00f0)
#define FE2_TX_INF_FORCE_PU         (BIT(10))
#define FE2_TX_INF_FORCE_PU_M       (BIT(10))
#define FE2_TX_INF_FORCE_PU_V       1
#define FE2_TX_INF_FORCE_PU_S       10
#define FE2_TX_INF_FORCE_PD         (BIT(9))
#define FE2_TX_INF_FORCE_PD_M       (BIT(9))
#define FE2_TX_INF_FORCE_PD_V       1
#define FE2_TX_INF_FORCE_PD_S       9

/* RO data page in MMU index */

#define DROM0_PAGES_START           0
#define DROM0_PAGES_END             64

#define IROM0_PAGES_START           64
#define IROM0_PAGES_END             256

/* MMU invalid value */

#define INVALID_MMU_VAL             0x100

/*  phy registers and memory size */

#define SOC_PHY_DIG_REGS_MEM_SIZE   (21*4)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_sp_dram
 *
 * Description:
 *   Check if the stack pointer is in DRAM.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32_sp_dram(uint32_t sp)
{
  return (sp >= SOC_DRAM_LOW + 0x10 && sp < SOC_DRAM_HIGH - 0x10);
}

/****************************************************************************
 * Name: esp32_ptr_extram
 *
 * Description:
 *   Check if the buffer comes from the external RAM
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32_ptr_extram(const void *p)
{
  return ((intptr_t)p >= SOC_EXTRAM_DATA_LOW &&
          (intptr_t)p < SOC_EXTRAM_DATA_HIGH);
}

/****************************************************************************
 * Name: esp32_ptr_iram
 *
 * Description:
 *   Check if the pointer is in IRAM
 *
 * Parameters:
 *   p - Pointer to the address being checked.
 *
 * Return Value:
 *   True if the address is a member of the internal memory. False if not.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32_ptr_iram(const void *p)
{
  return ((intptr_t)p >= SOC_IRAM_LOW && (intptr_t)p < SOC_IRAM_HIGH);
}

/****************************************************************************
 * Name: esp32_ptr_exec
 *
 * Description:
 *   Check if the pointer is within an executable range.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32_ptr_exec(const void *p)
{
  intptr_t ip = (intptr_t)p;
  return (ip >= SOC_IROM_LOW && ip < SOC_IROM_HIGH)
      || (ip >= SOC_IRAM_LOW && ip < SOC_IRAM_HIGH)
      || (ip >= SOC_IROM_MASK_LOW && ip < SOC_IROM_MASK_HIGH)
#if defined(SOC_CACHE_APP_LOW) && !defined(CONFIG_SMP)
      || (ip >= SOC_CACHE_APP_LOW && ip < SOC_CACHE_APP_HIGH)
#endif
      || (ip >= SOC_RTC_IRAM_LOW && ip < SOC_RTC_IRAM_HIGH);
}

/****************************************************************************
 * Name: esp32_ptr_rtcslow
 *
 * Description:
 *   Check if the buffer comes from the RTC Slow RAM.
 *
 * Parameters:
 *   p          - Pointer to the buffer.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32_ptr_rtcslow(const void *p)
{
  return ((intptr_t)p >= SOC_RTC_SLOW_LOW &&
          (intptr_t)p < SOC_RTC_SLOW_HIGH);
}

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_SOC_H */
