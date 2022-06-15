/****************************************************************************
 * arch/xtensa/src/esp32s2/hardware/esp32s2_soc.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32S2_SOC_H
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32S2_SOC_H

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

#define SOC_MAX_CONTIGUOUS_RAM_SIZE (SOC_EXTRAM_DATA_HIGH - SOC_EXTRAM_DATA_LOW) /* Largest span of contiguous memory (DRAM or IRAM) in the address space */

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

#define REG_GET_FIELD(_r, _f) ((REG_READ(_r) >> (_f##_S)) & (_f##_V))

/* Set field to register,
 * used when _f is not left shifted by _f##_S
 */

#define REG_SET_FIELD(_r, _f, _v) (REG_WRITE((_r),((REG_READ(_r) & ~((_f##_V) << (_f##_S)))|(((_v) & (_f##_V))<<(_f##_S)))))

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

#define REG_MASK(_reg, _field) ((_reg & (_field##_M)) >> (_field##_S))

/* Helper to place a value in a field */

#define VALUE_TO_FIELD(_value, _field) (((_value) << (_field##_S)) & (_field##_M))

/* Periheral Clock */

#define APB_CLK_FREQ_ROM                        40 * 1000000
#define CPU_CLK_FREQ_ROM                        APB_CLK_FREQ_ROM
#define UART_CLK_FREQ_ROM                       APB_CLK_FREQ_ROM
#define CPU_CLK_FREQ                            APB_CLK_FREQ
#define APB_CLK_FREQ                            80 * 1000000    /* Unit: Hz */
#define REF_CLK_FREQ                            (1000000)
#define UART_CLK_FREQ                           APB_CLK_FREQ
#define MWDT_CLK_FREQ                           APB_CLK_FREQ
#define TIMER_CLK_FREQ                          (80000000 >> 4) /* 80MHz divided by 16 */
#define SPI_CLK_DIV                             4
#define TICKS_PER_US_ROM                        40              /* CPU is 80MHz */

#define DR_REG_SYSTEM_BASE                      0x3f4c0000
#define DR_REG_SENSITIVE_BASE                   0x3f4c1000
#define DR_REG_INTERRUPT_BASE                   0x3f4c2000
#define DR_REG_DMA_COPY_BASE                    0x3f4c3000
#define DR_REG_EXTMEM_BASE                      0x61800000
#define DR_REG_MMU_TABLE                        0x61801000
#define DR_REG_ITAG_TABLE                       0x61802000
#define DR_REG_DTAG_TABLE                       0x61803000
#define DR_REG_AES_BASE                         0x6003a000
#define DR_REG_SHA_BASE                         0x6003b000
#define DR_REG_RSA_BASE                         0x6003c000
#define DR_REG_HMAC_BASE                        0x6003e000
#define DR_REG_DIGITAL_SIGNATURE_BASE           0x6003d000
#define DR_REG_CRYPTO_DMA_BASE                  0x6003f000
#define DR_REG_ASSIST_DEBUG_BASE                0x3f4ce000
#define DR_REG_DEDICATED_GPIO_BASE              0x3f4cf000
#define DR_REG_INTRUSION_BASE                   0x3f4d0000
#define DR_REG_DPORT_END                        0x3f4d3FFC
#define DR_REG_UART_BASE                        0x3f400000
#define DR_REG_SPI1_BASE                        0x3f402000
#define DR_REG_SPI0_BASE                        0x3f403000
#define DR_REG_GPIO_BASE                        0x3f404000
#define DR_REG_GPIO_SD_BASE                     0x3f404f00
#define DR_REG_FE2_BASE                         0x3f405000
#define DR_REG_FE_BASE                          0x3f406000
#define DR_REG_FRC_TIMER_BASE                   0x3f407000
#define DR_REG_RTCCNTL_BASE                     0x3f408000
#define DR_REG_RTCIO_BASE                       0x3f408400
#define DR_REG_SENS_BASE                        0x3f408800
#define DR_REG_RTC_I2C_BASE                     0x3f408C00
#define DR_REG_IO_MUX_BASE                      0x3f409000
#define DR_REG_HINF_BASE                        0x3f40B000
#define DR_REG_I2S_BASE                         0x3f40F000
#define DR_REG_UART1_BASE                       0x3f410000
#define DR_REG_I2C_EXT_BASE                     0x3f413000
#define DR_REG_UHCI0_BASE                       0x3f414000
#define DR_REG_SLCHOST_BASE                     0x3f415000
#define DR_REG_RMT_BASE                         0x3f416000
#define DR_REG_PCNT_BASE                        0x3f417000
#define DR_REG_SLC_BASE                         0x3f418000
#define DR_REG_LEDC_BASE                        0x3f419000
#define DR_REG_CP_BASE                          0x3f4c3000
#define DR_REG_EFUSE_BASE                       0x3f41A000
#define DR_REG_NRX_BASE                         0x3f41CC00
#define DR_REG_BB_BASE                          0x3f41D000
#define DR_REG_TIMERGROUP0_BASE                 0x3f41F000
#define DR_REG_TIMERGROUP1_BASE                 0x3f420000
#define DR_REG_RTC_SLOWMEM_BASE                 0x3f421000
#define DR_REG_SYSTIMER_BASE                    0x3f423000
#define DR_REG_SPI2_BASE                        0x3f424000
#define DR_REG_SPI3_BASE                        0x3f425000
#define DR_REG_SYSCON_BASE                      0x3f426000
#define DR_REG_APB_CTRL_BASE                    0x3f426000    /* Old name for SYSCON, to be removed */
#define DR_REG_I2C1_EXT_BASE                    0x3f427000
#define DR_REG_SPI4_BASE                        0x3f437000
#define DR_REG_USB_WRAP_BASE                    0x3f439000
#define DR_REG_APB_SARADC_BASE                  0x3f440000
#define DR_REG_USB_BASE                         0x60080000

#define REG_UHCI_BASE(i)         (DR_REG_UHCI0_BASE)
#define REG_UART_BASE(i)         (DR_REG_UART_BASE + (i) * 0x10000 )
#define REG_UART_AHB_BASE(i)     (0x60000000 + (i) * 0x10000 )
#define UART_FIFO_AHB_REG(i)     (REG_UART_AHB_BASE(i) + 0x0)
#define REG_I2S_BASE(i)          (DR_REG_I2S_BASE)
#define REG_TIMG_BASE(i)         (DR_REG_TIMERGROUP0_BASE + (i) * 0x1000)
#define REG_SPI_MEM_BASE(i)      (DR_REG_SPI0_BASE - (i) * 0x1000)
#define REG_I2C_BASE(i)          (DR_REG_I2C_EXT_BASE + (i) * 0x14000 )

/* Registers Operation */

#define REG_UART_BASE( i )  (DR_REG_UART_BASE + (i) * 0x10000 )

/* Overall memory map */

#define SOC_DROM_LOW            0x3f000000    /* drom0 low address for icache  */
#define SOC_DROM_HIGH           0x3ff80000    /* dram0 high address for dcache */
#define SOC_IROM_LOW            0x40080000
#define SOC_IROM_HIGH           0x40800000
#define SOC_IROM_MASK_LOW       0x40000000
#define SOC_IROM_MASK_HIGH      0x40020000
#define SOC_IRAM_LOW            0x40020000
#define SOC_IRAM_HIGH           0x40070000
#define SOC_DRAM_LOW            0x3ffb0000
#define SOC_DRAM_HIGH           0x40000000
#define SOC_RTC_IRAM_LOW        0x40070000
#define SOC_RTC_IRAM_HIGH       0x40072000
#define SOC_RTC_DRAM_LOW        0x3ff9e000
#define SOC_RTC_DRAM_HIGH       0x3ffa0000
#define SOC_RTC_DATA_LOW        0x50000000
#define SOC_RTC_DATA_HIGH       0x50002000
#define SOC_EXTRAM_DATA_LOW     0x3f500000
#define SOC_EXTRAM_DATA_HIGH    0x3ff80000

#define SOC_EXTRAM_DATA_SIZE (SOC_EXTRAM_DATA_HIGH - SOC_EXTRAM_DATA_LOW)

/* Virtual address 0 */

#define VADDR0_START_ADDR       SOC_DROM_LOW
#define VADDR0_END_ADDR         (SOC_DROM_HIGH - 1)

/* Interrupt hardware source table
 * This table is decided by hardware, don't touch this.
 */

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

#define APB_CTRL_SYSCLK_CONF_REG    (DR_REG_APB_CTRL_BASE + 0x0)
#define APB_CTRL_XTAL_TICK_CONF_REG (DR_REG_APB_CTRL_BASE + 0x4)

/* APB_CTRL_PRE_DIV_CNT : R/W ;bitpos:[9:0] ;default: 10'h0 ; */

#define APB_CTRL_PRE_DIV_CNT        0x000003ff
#define APB_CTRL_PRE_DIV_CNT_M      ((APB_CTRL_PRE_DIV_CNT_V) << \
                                     (APB_CTRL_PRE_DIV_CNT_S))
#define APB_CTRL_PRE_DIV_CNT_V      0x3ff
#define APB_CTRL_PRE_DIV_CNT_S      0

/* ROM functions which read/write internal control bus */

extern uint8_t rom_i2c_readreg(uint8_t block, uint8_t host_id,
                               uint8_t reg_add);
extern uint8_t rom_i2c_readreg_mask(uint8_t block, uint8_t host_id,
                        uint8_t reg_add, uint8_t msb, uint8_t lsb);
extern void rom_i2c_writereg(uint8_t block, uint8_t host_id,
                             uint8_t reg_add, uint8_t data);
extern void rom_i2c_writereg_mask(uint8_t block, uint8_t host_id,
                   uint8_t reg_add, uint8_t msb, uint8_t lsb, uint8_t data);

#define I2C_WRITEREG_RTC(block, reg_add, indata) \
      rom_i2c_writereg(block, block##_HOSTID,  reg_add, indata)

#define I2C_READREG_RTC(block, reg_add) \
      rom_i2c_readreg(block, block##_HOSTID,  reg_add)

#define I2C_WRITEREG_MASK_RTC(block, reg_add, indata) \
      rom_i2c_writereg_mask(block, block##_HOSTID,  reg_add,  reg_add##_MSB,  reg_add##_LSB,  indata)

#define I2C_READREG_MASK_RTC(block, reg_add) \
      rom_i2c_readreg_mask(block, block##_HOSTID,  reg_add,  reg_add##_MSB,  reg_add##_LSB)

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

#define TIMG_RTCCALICFG_REG(i)      (REG_TIMG_BASE(i) + 0x0068)

#define RTC_XTAL_FREQ_REG           RTC_CNTL_STORE4_REG

/* Approximate mapping of voltages to RTC_CNTL_DBIAS_WAK, RTC_CNTL_DBIAS_SLP,
 * RTC_CNTL_DIG_DBIAS_WAK, RTC_CNTL_DIG_DBIAS_SLP values.
 * Valid if RTC_CNTL_DBG_ATTEN is 0.
 */

#define RTC_CNTL_DBIAS_1V00         2
#define RTC_CNTL_DBIAS_1V10         4
#define RTC_CNTL_DBIAS_1V25         7

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
 */

#ifdef CONFIG_ESP32S2_FLASH_FREQ_80M
#define DIG_DBIAS_80M_160M          RTC_CNTL_DBIAS_1V25
#else
#define DIG_DBIAS_80M_160M          RTC_CNTL_DBIAS_1V10
#endif
#define DIG_DBIAS_240M              RTC_CNTL_DBIAS_1V25
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

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_sp_dram
 *
 * Description:
 *   Check if the stack pointer is in DRAM.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32s2_sp_dram(uint32_t sp)
{
  return (sp >= SOC_DRAM_LOW + 0x10 && sp < SOC_DRAM_HIGH - 0x10);
}

/****************************************************************************
 * Name: esp32s2_ptr_extram
 *
 * Description:
 *   Check if the buffer comes from the external RAM
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32s2_ptr_extram(const void *p)
{
  return ((intptr_t)p >= SOC_EXTRAM_DATA_LOW &&
          (intptr_t)p < SOC_EXTRAM_DATA_HIGH);
}

/****************************************************************************
 * Name: esp32s2_ptr_exec
 *
 * Description:
 *   Check if the pointer is within an executable range.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32s2_ptr_exec(const void *p)
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

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32S2_SOC_H */
