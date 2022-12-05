/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_soc.h
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

#define ETS_UNCACHED_ADDR(addr) (addr)
#define ETS_CACHED_ADDR(addr) (addr)

#define BIT(nr)                 (1UL << (nr))

/* Write value to register */

#define REG_WRITE(_r, _v)    (*(volatile uint32_t *)(_r)) = (_v)

/* Read value from register */

#define REG_READ(_r) (*(volatile uint32_t *)(_r))

/* Get bit or get bits from register */

#define REG_GET_BIT(_r, _b)  (*(volatile uint32_t*)(_r) & (_b))

/* Set bit or set bits to register */

#define REG_SET_BIT(_r, _b)  (*(volatile uint32_t*)(_r) |= (_b))

/* Clear bit or clear bits of register */

#define REG_CLR_BIT(_r, _b)  (*(volatile uint32_t*)(_r) &= ~(_b))

/* Set bits of register controlled by mask */

#define REG_SET_BITS(_r, _b, _m) (*(volatile uint32_t*)(_r) = (*(volatile uint32_t*)(_r) & ~(_m)) | ((_b) & (_m)))

/* Get field from register,
 * used when _f is not left shifted by _f##_S
 */

#define REG_GET_FIELD(addr, field) ((getreg32(addr) >> field##_S) & field##_V)

/* Set field to register,
 * used when _f is not left shifted by _f##_S
 */

#define REG_SET_FIELD(addr, field, val) (modifyreg32(addr, (((~((uint32_t) val)) & field##_V) << field##_S), (((uint32_t) val) << field##_S)))

/* Set field value from a variable,
 * used when _f is not left shifted by _f##_S
 */

#define VALUE_GET_FIELD(_r, _f) (((_r) >> (_f##_S)) & (_f))

/* Get field value from a variable,
 * used when _f is left shifted by _f##_S
 */

#define VALUE_GET_FIELD2(_r, _f) (((_r) & (_f))>> (_f##_S))

/* Set field value to a variable,
 * used when _f is not left shifted by _f##_S
 */

#define VALUE_SET_FIELD(_r, _f, _v) ((_r)=(((_r) & ~((_f) << (_f##_S)))|((_v)<<(_f##_S))))

/* Set field value to a variable,
 * used when _f is left shifted by _f##_S
 */

#define VALUE_SET_FIELD2(_r, _f, _v) ((_r)=(((_r) & ~(_f))|((_v)<<(_f##_S))))

/* Generate a value from a field value,
 * used when _f is not left shifted by _f##_S
 */

#define FIELD_TO_VALUE(_f, _v) (((_v)&(_f))<<_f##_S)

/* Generate a value from a field value,
 * used when _f is left shifted by _f##_S
 */

#define FIELD_TO_VALUE2(_f, _v) (((_v)<<_f##_S) & (_f))

/* Read value from register */

#define READ_PERI_REG(addr) (*((volatile uint32_t *)ETS_UNCACHED_ADDR(addr)))

/* Write value to register */

#define WRITE_PERI_REG(addr, val) (*((volatile uint32_t *)ETS_UNCACHED_ADDR(addr))) = (uint32_t)(val)

/* Clear bits of register controlled by mask */

#define CLEAR_PERI_REG_MASK(reg, mask) WRITE_PERI_REG((reg), (READ_PERI_REG(reg)&(~(mask))))

/* Set bits of register controlled by mask */

#define SET_PERI_REG_MASK(reg, mask)   WRITE_PERI_REG((reg), (READ_PERI_REG(reg)|(mask)))

/* Get bits of register controlled by mask */

#define GET_PERI_REG_MASK(reg, mask)   (READ_PERI_REG(reg) & (mask))

/* Get bits of register controlled by highest bit and lowest bit */

#define GET_PERI_REG_BITS(reg, hipos,lowpos)      ((READ_PERI_REG(reg)>>(lowpos))&((1<<((hipos)-(lowpos)+1))-1))

/* Set bits of register controlled by mask and shift */

#define SET_PERI_REG_BITS(reg,bit_map,value,shift) (WRITE_PERI_REG((reg),(READ_PERI_REG(reg)&(~((bit_map)<<(shift))))|(((value) & bit_map)<<(shift)) ))

/* Get field of register */

#define GET_PERI_REG_BITS2(reg, mask,shift)      ((READ_PERI_REG(reg)>>(shift))&(mask))

/* Extract the field from the register and shift it to avoid wrong reading */

#define REG_MASK(_reg, _field) (((_reg) & (_field##_M)) >> (_field##_S))

/* Helper to place a value in a field */

#define VALUE_TO_FIELD(_value, _field) (((_value) << (_field##_S)) & (_field##_M))

/* Periheral Clock */

#define APB_CLK_FREQ_ROM                        26 * 1000000
#define CPU_CLK_FREQ_ROM                        APB_CLK_FREQ_ROM
#define CPU_CLK_FREQ                            APB_CLK_FREQ
#define APB_CLK_FREQ                            80 * 1000000    /* Unit: Hz */
#define REF_CLK_FREQ                            (1000000)
#define UART_CLK_FREQ                           APB_CLK_FREQ
#define MWDT_CLK_FREQ                           APB_CLK_FREQ
#define TIMER_CLK_FREQ                          (80000000 >> 4) /* 80MHz divided by 16 */
#define SPI_CLK_DIV                             4
#define TICKS_PER_US_ROM                        26              /* CPU is 80MHz */

#define DR_REG_DPORT_BASE                       0x3ff00000
#define DR_REG_UART_BASE                        0x3ff40000
#define DR_REG_SPI1_BASE                        0x3ff42000
#define DR_REG_SPI0_BASE                        0x3ff43000
#define DR_REG_GPIO_BASE                        0x3ff44000
#define DR_REG_GPIO_SD_BASE                     0x3ff44f00
#define DR_REG_FE2_BASE                         0x3ff45000
#define DR_REG_FE_BASE                          0x3ff46000
#define DR_REG_FRC_TIMER_BASE                   0x3ff47000
#define DR_REG_RTCCNTL_BASE                     0x3ff48000
#define DR_REG_RTCIO_BASE                       0x3ff48400
#define DR_REG_SENS_BASE                        0x3ff48800
#define DR_REG_IO_MUX_BASE                      0x3ff49000
#define DR_REG_EFUSE_BASE                       0x3ff5a000
#define DR_REG_RTCMEM0_BASE                     0x3ff61000
#define DR_REG_RTCMEM1_BASE                     0x3ff62000
#define DR_REG_RTCMEM2_BASE                     0x3ff63000
#define DR_REG_HINF_BASE                        0x3ff4b000
#define DR_REG_UHCI1_BASE                       0x3ff4c000
#define DR_REG_I2S_BASE                         0x3ff4f000
#define DR_REG_UART1_BASE                       0x3ff50000
#define DR_REG_BT_BASE                          0x3ff51000
#define DR_REG_I2C_EXT_BASE                     0x3ff53000
#define DR_REG_UHCI0_BASE                       0x3ff54000
#define DR_REG_SLCHOST_BASE                     0x3ff55000
#define DR_REG_RMT_BASE                         0x3ff56000
#define DR_REG_PCNT_BASE                        0x3ff57000
#define DR_REG_SLC_BASE                         0x3ff58000
#define DR_REG_LEDC_BASE                        0x3ff59000
#define DR_REG_EFUSE_BASE                       0x3ff5a000
#define DR_REG_SPI_ENCRYPT_BASE                 0x3ff5b000
#define DR_REG_NRX_BASE                         0x3ff5cc00
#define DR_REG_BB_BASE                          0x3ff5d000
#define DR_REG_PWM_BASE                         0x3ff5e000
#define DR_REG_TIMERGROUP0_BASE                 0x3ff5f000
#define DR_REG_TIMERGROUP1_BASE                 0x3ff60000
#define DR_REG_SPI2_BASE                        0x3ff64000
#define DR_REG_SPI3_BASE                        0x3ff65000
#define DR_REG_I2C1_EXT_BASE                    0x3ff67000
#define DR_REG_SDMMC_BASE                       0x3ff68000
#define DR_REG_EMAC_BASE                        0x3ff69000
#define DR_REG_TWAI_BASE                        0x3ff6b000
#define DR_REG_CAN_BASE                         DR_REG_TWAI_BASE
#define DR_REG_PWM1_BASE                        0x3ff6c000
#define DR_REG_I2S1_BASE                        0x3ff6d000
#define DR_REG_UART2_BASE                       0x3ff6e000
#define DR_REG_PWM2_BASE                        0x3ff6f000
#define DR_REG_PWM3_BASE                        0x3ff70000
#define PERIPHS_SPI_ENCRYPT_BASEADDR            DR_REG_SPI_ENCRYPT_BASE

/* Some AHB addresses can be used instead of DPORT addresses
 * as a workaround for some HW bugs.
 * This workaround is detailed at
 * https://www.espressif.com/sites/default/files/documentation/
 * eco_and_workarounds_for_bugs_in_esp32_en.pdf
 */

#define AHB_REG_UART_BASE   0x60000000

/* Overall memory map */

#define SOC_DROM_LOW            0x3f400000
#define SOC_DROM_HIGH           0x3f800000
#define SOC_DRAM_LOW            0x3ffae000
#define SOC_DRAM_HIGH           0x40000000
#define SOC_IROM_LOW            0x400d0000
#define SOC_IROM_HIGH           0x40400000
#define SOC_IROM_MASK_LOW       0x40000000
#define SOC_IROM_MASK_HIGH      0x40064f00
#define SOC_CACHE_PRO_LOW       0x40070000
#define SOC_CACHE_PRO_HIGH      0x40078000
#define SOC_CACHE_APP_LOW       0x40078000
#define SOC_CACHE_APP_HIGH      0x40080000
#define SOC_IRAM_LOW            0x40080000
#define SOC_IRAM_HIGH           0x400a0000
#define SOC_RTC_IRAM_LOW        0x400c0000
#define SOC_RTC_IRAM_HIGH       0x400c2000
#define SOC_RTC_DRAM_LOW        0x3ff80000
#define SOC_RTC_DRAM_HIGH       0x3ff82000
#define SOC_RTC_SLOW_LOW        0x50000000
#define SOC_RTC_SLOW_HIGH       0x50002000
#define SOC_EXTRAM_DATA_LOW     0x3f800000
#define SOC_EXTRAM_DATA_HIGH    0x3fc00000

/* Virtual address 0 */

#define VADDR0_START_ADDR       SOC_DROM_LOW
#define VADDR0_END_ADDR         (SOC_DROM_HIGH - 1)

/* Interrupt hardware source table
 * This table is decided by hardware, don't touch this.
 */

#define ETS_WIFI_MAC_INTR_SOURCE                0  /* Interrupt of Wi-Fi MAC, level */
#define ETS_WIFI_MAC_NMI_SOURCE                 1  /* Interrupt of Wi-Fi MAC, NMI, use if MAC have bug to fix in NMI */
#define ETS_WIFI_BB_INTR_SOURCE                 2  /* Interrupt of Wi-Fi BB, level, we can do some calibartion */
#define ETS_BT_MAC_INTR_SOURCE                  3  /* will be cancelled */
#define ETS_BT_BB_INTR_SOURCE                   4  /* Interrupt of BT BB, level */
#define ETS_BT_BB_NMI_SOURCE                    5  /* Interrupt of BT BB, NMI, use if BB have bug to fix in NMI */
#define ETS_RWBT_INTR_SOURCE                    6  /* Interrupt of RWBT, level */
#define ETS_RWBLE_INTR_SOURCE                   7  /* Interrupt of RWBLE, level */
#define ETS_RWBT_NMI_SOURCE                     8  /* Interrupt of RWBT, NMI, use if RWBT have bug to fix in NMI */
#define ETS_RWBLE_NMI_SOURCE                    9  /* Interrupt of RWBLE, NMI, use if RWBT have bug to fix in NMI */
#define ETS_SLC0_INTR_SOURCE                    10 /* Interrupt of SLC0, level */
#define ETS_SLC1_INTR_SOURCE                    11 /* Interrupt of SLC1, level */
#define ETS_UHCI0_INTR_SOURCE                   12 /* Interrupt of UHCI0, level */
#define ETS_UHCI1_INTR_SOURCE                   13 /* Interrupt of UHCI1, level */
#define ETS_TG0_T0_LEVEL_INTR_SOURCE            14 /* Interrupt of TIMER_GROUP0, TIMER0, level, we would like use EDGE for timer if permission */
#define ETS_TG0_T1_LEVEL_INTR_SOURCE            15 /* Interrupt of TIMER_GROUP0, TIMER1, level, we would like use EDGE for timer if permission */
#define ETS_TG0_WDT_LEVEL_INTR_SOURCE           16 /* Interrupt of TIMER_GROUP0, WATCHDOG, level */
#define ETS_TG0_LACT_LEVEL_INTR_SOURCE          17 /* Interrupt of TIMER_GROUP0, LACT, level */
#define ETS_TG1_T0_LEVEL_INTR_SOURCE            18 /* Interrupt of TIMER_GROUP1, TIMER0, level, we would like use EDGE for timer if permission */
#define ETS_TG1_T1_LEVEL_INTR_SOURCE            19 /* Interrupt of TIMER_GROUP1, TIMER1, level, we would like use EDGE for timer if permission */
#define ETS_TG1_WDT_LEVEL_INTR_SOURCE           20 /* Interrupt of TIMER_GROUP1, WATCHDOG, level */
#define ETS_TG1_LACT_LEVEL_INTR_SOURCE          21 /* Interrupt of TIMER_GROUP1, LACT, level */
#define ETS_GPIO_INTR_SOURCE                    22 /* Interrupt of GPIO, level */
#define ETS_GPIO_NMI_SOURCE                     23 /* Interrupt of GPIO, NMI */
#define ETS_FROM_CPU_INTR0_SOURCE               24 /* Interrupt0 generated from a CPU, level */
#define ETS_FROM_CPU_INTR1_SOURCE               25 /* Interrupt1 generated from a CPU, level */
#define ETS_FROM_CPU_INTR2_SOURCE               26 /* Interrupt2 generated from a CPU, level */
#define ETS_FROM_CPU_INTR3_SOURCE               27 /* Interrupt3 generated from a CPU, level */
#define ETS_SPI0_INTR_SOURCE                    28 /* Interrupt of SPI0, level, SPI0 is for Cache Access, do not use this */
#define ETS_SPI1_INTR_SOURCE                    29 /* Interrupt of SPI1, level, SPI1 is for flash read/write, do not use this */
#define ETS_SPI2_INTR_SOURCE                    30 /* Interrupt of SPI2, level */
#define ETS_SPI3_INTR_SOURCE                    31 /* Interrupt of SPI3, level */
#define ETS_I2S0_INTR_SOURCE                    32 /* Interrupt of I2S0, level */
#define ETS_I2S1_INTR_SOURCE                    33 /* Interrupt of I2S1, level */
#define ETS_UART0_INTR_SOURCE                   34 /* Interrupt of UART0, level */
#define ETS_UART1_INTR_SOURCE                   35 /* Interrupt of UART1, level */
#define ETS_UART2_INTR_SOURCE                   36 /* Interrupt of UART2, level */
#define ETS_SDIO_HOST_INTR_SOURCE               37 /* Interrupt of SD/SDIO/MMC HOST, level */
#define ETS_ETH_MAC_INTR_SOURCE                 38 /* Interrupt of ethernet mac, level */
#define ETS_PWM0_INTR_SOURCE                    39 /* Interrupt of PWM0, level, Reserved */
#define ETS_PWM1_INTR_SOURCE                    40 /* Interrupt of PWM1, level, Reserved */
#define ETS_PWM2_INTR_SOURCE                    41 /* Interrupt of PWM2, level */
#define ETS_PWM3_INTR_SOURCE                    42 /* Interruot of PWM3, level */
#define ETS_LEDC_INTR_SOURCE                    43 /* Interrupt of LED PWM, level */
#define ETS_EFUSE_INTR_SOURCE                   44 /* Interrupt of efuse, level, not likely to use */
#define ETS_CAN_INTR_SOURCE                     45 /* Interrupt of can, level */
#define ETS_RTC_CORE_INTR_SOURCE                46 /* Interrupt of rtc core, level, include rtc watchdog */
#define ETS_RMT_INTR_SOURCE                     47 /* Interrupt of remote controller, level */
#define ETS_PCNT_INTR_SOURCE                    48 /* Interrupt of pulse count, level */
#define ETS_I2C_EXT0_INTR_SOURCE                49 /* Interrupt of I2C controller1, level */
#define ETS_I2C_EXT1_INTR_SOURCE                50 /* Interrupt of I2C controller0, level */
#define ETS_RSA_INTR_SOURCE                     51 /* Interrupt of RSA accelerator, level */
#define ETS_SPI1_DMA_INTR_SOURCE                52 /* Interrupt of SPI1 DMA, SPI1 is for flash read/write, do not use this */
#define ETS_SPI2_DMA_INTR_SOURCE                53 /* Interrupt of SPI2 DMA, level */
#define ETS_SPI3_DMA_INTR_SOURCE                54 /* Interrupt of SPI3 DMA, level */
#define ETS_WDT_INTR_SOURCE                     55 /* will be cancelled */
#define ETS_TIMER1_INTR_SOURCE                  56 /* will be cancelled */
#define ETS_TIMER2_INTR_SOURCE                  57 /* will be cancelled */
#define ETS_TG0_T0_EDGE_INTR_SOURCE             58 /* Interrupt of TIMER_GROUP0, TIMER0, EDGE */
#define ETS_TG0_T1_EDGE_INTR_SOURCE             59 /* Interrupt of TIMER_GROUP0, TIMER1, EDGE */
#define ETS_TG0_WDT_EDGE_INTR_SOURCE            60 /* Interrupt of TIMER_GROUP0, WATCH DOG, EDGE */
#define ETS_TG0_LACT_EDGE_INTR_SOURCE           61 /* Interrupt of TIMER_GROUP0, LACT, EDGE */
#define ETS_TG1_T0_EDGE_INTR_SOURCE             62 /* Interrupt of TIMER_GROUP1, TIMER0, EDGE */
#define ETS_TG1_T1_EDGE_INTR_SOURCE             63 /* Interrupt of TIMER_GROUP1, TIMER1, EDGE */
#define ETS_TG1_WDT_EDGE_INTR_SOURCE            64 /* Interrupt of TIMER_GROUP1, WATCHDOG, EDGE */
#define ETS_TG1_LACT_EDGE_INTR_SOURCE           65 /* Interrupt of TIMER_GROUP0, LACT, EDGE */
#define ETS_MMU_IA_INTR_SOURCE                  66 /* Interrupt of MMU Invalid Access, LEVEL */
#define ETS_MPU_IA_INTR_SOURCE                  67 /* Interrupt of MPU Invalid Access, LEVEL */
#define ETS_CACHE_IA_INTR_SOURCE                68 /* Interrupt of Cache Invalied Access, LEVEL */

#define EFUSE_BLK0_RDATA4_REG                   (DR_REG_EFUSE_BASE + 0x010)
#define EFUSE_BLK0_RDATA3_REG                   (DR_REG_EFUSE_BASE + 0x00c)
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

/* APB_CTRL_PRE_DIV_CNT : R/W ;bitpos:[9:0] ;default: 10'h0 ; */

#define APB_CTRL_PRE_DIV_CNT        0x000003ff
#define APB_CTRL_PRE_DIV_CNT_M      ((APB_CTRL_PRE_DIV_CNT_V) << \
                                     (APB_CTRL_PRE_DIV_CNT_S))
#define APB_CTRL_PRE_DIV_CNT_V      0x3ff
#define APB_CTRL_PRE_DIV_CNT_S      0

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

#define EFUSE_BLK0_RDATA5_REG       (DR_REG_EFUSE_BASE + 0x014)

/* EFUSE_RD_VOL_LEVEL_HP_INV: RO; bitpos:[23:22] */

/* description: This field stores the voltage level for
 * CPU to run at 240 MHz, or for flash/PSRAM to run at 80 MHz.
 * 0x0: level 7; 0x1: level 6; 0x2: level 5; 0x3: level 4. (RO)
 */

#define EFUSE_RD_VOL_LEVEL_HP_INV   0x03
#define EFUSE_RD_VOL_LEVEL_HP_INV_M ((EFUSE_RD_VOL_LEVEL_HP_INV_V) << (EFUSE_RD_VOL_LEVEL_HP_INV_S))
#define EFUSE_RD_VOL_LEVEL_HP_INV_V 0x03
#define EFUSE_RD_VOL_LEVEL_HP_INV_S 22

/* EFUSE_RD_SDIO_FORCE : RO ;bitpos:[16] ;default: 1'b0 ; */

/* description: read for sdio_force */

#define EFUSE_RD_SDIO_FORCE         (BIT(16))
#define EFUSE_RD_SDIO_FORCE_M       (BIT(16))
#define EFUSE_RD_SDIO_FORCE_V       0x1
#define EFUSE_RD_SDIO_FORCE_S       16

/* EFUSE_RD_XPD_SDIO_REG : RO ;bitpos:[14] ;default: 1'b0 ; */

/* description: read for XPD_SDIO_REG */

#define EFUSE_RD_XPD_SDIO_REG       (BIT(14))
#define EFUSE_RD_XPD_SDIO_REG_M     (BIT(14))
#define EFUSE_RD_XPD_SDIO_REG_V     0x1
#define EFUSE_RD_XPD_SDIO_REG_S     14

/* EFUSE_RD_SDIO_TIEH : RO ;bitpos:[15] ;default: 1'b0 ; */

/* description: read for SDIO_TIEH */

#define EFUSE_RD_SDIO_TIEH         (BIT(15))
#define EFUSE_RD_SDIO_TIEH_M       (BIT(15))
#define EFUSE_RD_SDIO_TIEH_V       0x1
#define EFUSE_RD_SDIO_TIEH_S       15

/* EFUSE_RD_BLK3_PART_RESERVE : R/W ; bitpos:[14] ; default: 1'b0; */

/* description: If set, this bit indicates that
 * BLOCK3[143:96] is reserved for internal use
 */

#define EFUSE_RD_BLK3_PART_RESERVE   (BIT(14))
#define EFUSE_RD_BLK3_PART_RESERVE_M ((EFUSE_RD_BLK3_PART_RESERVE_V) << (EFUSE_RD_BLK3_PART_RESERVE_S))
#define EFUSE_RD_BLK3_PART_RESERVE_V 0x1
#define EFUSE_RD_BLK3_PART_RESERVE_S 14

/* EFUSE_RD_SDIO_DREFH : RO ;bitpos:[9:8] ;default: 2'b0 ; */

#define EFUSE_RD_SDIO_DREFH         0x00000003
#define EFUSE_RD_SDIO_DREFH_M       ((EFUSE_RD_SDIO_DREFH_V) << (EFUSE_RD_SDIO_DREFH_S))
#define EFUSE_RD_SDIO_DREFH_V       0x3
#define EFUSE_RD_SDIO_DREFH_S       8

/* EFUSE_RD_SDIO_DREFM : RO ;bitpos:[11:10] ;default: 2'b0 ; */

#define EFUSE_RD_SDIO_DREFM         0x00000003
#define EFUSE_RD_SDIO_DREFM_M       ((EFUSE_RD_SDIO_DREFM_V) << (EFUSE_RD_SDIO_DREFM_S))
#define EFUSE_RD_SDIO_DREFM_V       0x3
#define EFUSE_RD_SDIO_DREFM_S       10

/* Note: EFUSE_ADC_VREF and SDIO_DREFH/M/L share the same address space.
 * Newer versions of ESP32 come with EFUSE_ADC_VREF already burned,
 * therefore SDIO_DREFH/M/L is only available in older versions of ESP32
 */

/* EFUSE_RD_SDIO_DREFL : RO ;bitpos:[13:12] ;default: 2'b0 ; */

#define EFUSE_RD_SDIO_DREFL         0x00000003
#define EFUSE_RD_SDIO_DREFL_M       ((EFUSE_RD_SDIO_DREFL_V) << (EFUSE_RD_SDIO_DREFL_S))
#define EFUSE_RD_SDIO_DREFL_V       0x3
#define EFUSE_RD_SDIO_DREFL_S       12

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
