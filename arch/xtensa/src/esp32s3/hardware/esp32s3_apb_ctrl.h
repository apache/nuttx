/****************************************************************************
 * arch/xtensa/src/esp32s3/hardware/esp32s3_apb_ctrl.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_APB_CTRL_H
#define __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_APB_CTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32s3_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* APB_CTRL_SYSCLK_CONF_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SYSCLK_CONF_REG (DR_REG_APB_CTRL_BASE + 0x0)

/* APB_CTRL_RST_TICK_CNT : R/W; bitpos: [12]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_RST_TICK_CNT    (BIT(12))
#define APB_CTRL_RST_TICK_CNT_M  (APB_CTRL_RST_TICK_CNT_V << APB_CTRL_RST_TICK_CNT_S)
#define APB_CTRL_RST_TICK_CNT_V  0x00000001
#define APB_CTRL_RST_TICK_CNT_S  12

/* APB_CTRL_CLK_EN : R/W; bitpos: [11]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_CLK_EN    (BIT(11))
#define APB_CTRL_CLK_EN_M  (APB_CTRL_CLK_EN_V << APB_CTRL_CLK_EN_S)
#define APB_CTRL_CLK_EN_V  0x00000001
#define APB_CTRL_CLK_EN_S  11

/* APB_CTRL_CLK_320M_EN : R/W; bitpos: [10]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_CLK_320M_EN    (BIT(10))
#define APB_CTRL_CLK_320M_EN_M  (APB_CTRL_CLK_320M_EN_V << APB_CTRL_CLK_320M_EN_S)
#define APB_CTRL_CLK_320M_EN_V  0x00000001
#define APB_CTRL_CLK_320M_EN_S  10

/* APB_CTRL_PRE_DIV_CNT : R/W; bitpos: [9:0]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_PRE_DIV_CNT    0x000003ff
#define APB_CTRL_PRE_DIV_CNT_M  (APB_CTRL_PRE_DIV_CNT_V << APB_CTRL_PRE_DIV_CNT_S)
#define APB_CTRL_PRE_DIV_CNT_V  0x000003ff
#define APB_CTRL_PRE_DIV_CNT_S  0

/* APB_CTRL_TICK_CONF_REG register
 * ******* Description ***********
 */

#define APB_CTRL_TICK_CONF_REG (DR_REG_APB_CTRL_BASE + 0x4)

/* APB_CTRL_TICK_ENABLE : R/W; bitpos: [16]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_TICK_ENABLE    (BIT(16))
#define APB_CTRL_TICK_ENABLE_M  (APB_CTRL_TICK_ENABLE_V << APB_CTRL_TICK_ENABLE_S)
#define APB_CTRL_TICK_ENABLE_V  0x00000001
#define APB_CTRL_TICK_ENABLE_S  16

/* APB_CTRL_CK8M_TICK_NUM : R/W; bitpos: [15:8]; default: 7;
 * ******* Description ***********
 */

#define APB_CTRL_CK8M_TICK_NUM    0x000000ff
#define APB_CTRL_CK8M_TICK_NUM_M  (APB_CTRL_CK8M_TICK_NUM_V << APB_CTRL_CK8M_TICK_NUM_S)
#define APB_CTRL_CK8M_TICK_NUM_V  0x000000ff
#define APB_CTRL_CK8M_TICK_NUM_S  8

/* APB_CTRL_XTAL_TICK_NUM : R/W; bitpos: [7:0]; default: 39;
 * ******* Description ***********
 */

#define APB_CTRL_XTAL_TICK_NUM    0x000000ff
#define APB_CTRL_XTAL_TICK_NUM_M  (APB_CTRL_XTAL_TICK_NUM_V << APB_CTRL_XTAL_TICK_NUM_S)
#define APB_CTRL_XTAL_TICK_NUM_V  0x000000ff
#define APB_CTRL_XTAL_TICK_NUM_S  0

/* APB_CTRL_CLK_OUT_EN_REG register
 * ******* Description ***********
 */

#define APB_CTRL_CLK_OUT_EN_REG (DR_REG_APB_CTRL_BASE + 0x8)

/* APB_CTRL_CLK_XTAL_OEN : R/W; bitpos: [10]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_CLK_XTAL_OEN    (BIT(10))
#define APB_CTRL_CLK_XTAL_OEN_M  (APB_CTRL_CLK_XTAL_OEN_V << APB_CTRL_CLK_XTAL_OEN_S)
#define APB_CTRL_CLK_XTAL_OEN_V  0x00000001
#define APB_CTRL_CLK_XTAL_OEN_S  10

/* APB_CTRL_CLK40X_BB_OEN : R/W; bitpos: [9]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_CLK40X_BB_OEN    (BIT(9))
#define APB_CTRL_CLK40X_BB_OEN_M  (APB_CTRL_CLK40X_BB_OEN_V << APB_CTRL_CLK40X_BB_OEN_S)
#define APB_CTRL_CLK40X_BB_OEN_V  0x00000001
#define APB_CTRL_CLK40X_BB_OEN_S  9

/* APB_CTRL_CLK_DAC_CPU_OEN : R/W; bitpos: [8]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_CLK_DAC_CPU_OEN    (BIT(8))
#define APB_CTRL_CLK_DAC_CPU_OEN_M  (APB_CTRL_CLK_DAC_CPU_OEN_V << APB_CTRL_CLK_DAC_CPU_OEN_S)
#define APB_CTRL_CLK_DAC_CPU_OEN_V  0x00000001
#define APB_CTRL_CLK_DAC_CPU_OEN_S  8

/* APB_CTRL_CLK_ADC_INF_OEN : R/W; bitpos: [7]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_CLK_ADC_INF_OEN    (BIT(7))
#define APB_CTRL_CLK_ADC_INF_OEN_M  (APB_CTRL_CLK_ADC_INF_OEN_V << APB_CTRL_CLK_ADC_INF_OEN_S)
#define APB_CTRL_CLK_ADC_INF_OEN_V  0x00000001
#define APB_CTRL_CLK_ADC_INF_OEN_S  7

/* APB_CTRL_CLK_320M_OEN : R/W; bitpos: [6]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_CLK_320M_OEN    (BIT(6))
#define APB_CTRL_CLK_320M_OEN_M  (APB_CTRL_CLK_320M_OEN_V << APB_CTRL_CLK_320M_OEN_S)
#define APB_CTRL_CLK_320M_OEN_V  0x00000001
#define APB_CTRL_CLK_320M_OEN_S  6

/* APB_CTRL_CLK160_OEN : R/W; bitpos: [5]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_CLK160_OEN    (BIT(5))
#define APB_CTRL_CLK160_OEN_M  (APB_CTRL_CLK160_OEN_V << APB_CTRL_CLK160_OEN_S)
#define APB_CTRL_CLK160_OEN_V  0x00000001
#define APB_CTRL_CLK160_OEN_S  5

/* APB_CTRL_CLK80_OEN : R/W; bitpos: [4]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_CLK80_OEN    (BIT(4))
#define APB_CTRL_CLK80_OEN_M  (APB_CTRL_CLK80_OEN_V << APB_CTRL_CLK80_OEN_S)
#define APB_CTRL_CLK80_OEN_V  0x00000001
#define APB_CTRL_CLK80_OEN_S  4

/* APB_CTRL_CLK_BB_OEN : R/W; bitpos: [3]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_CLK_BB_OEN    (BIT(3))
#define APB_CTRL_CLK_BB_OEN_M  (APB_CTRL_CLK_BB_OEN_V << APB_CTRL_CLK_BB_OEN_S)
#define APB_CTRL_CLK_BB_OEN_V  0x00000001
#define APB_CTRL_CLK_BB_OEN_S  3

/* APB_CTRL_CLK44_OEN : R/W; bitpos: [2]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_CLK44_OEN    (BIT(2))
#define APB_CTRL_CLK44_OEN_M  (APB_CTRL_CLK44_OEN_V << APB_CTRL_CLK44_OEN_S)
#define APB_CTRL_CLK44_OEN_V  0x00000001
#define APB_CTRL_CLK44_OEN_S  2

/* APB_CTRL_CLK22_OEN : R/W; bitpos: [1]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_CLK22_OEN    (BIT(1))
#define APB_CTRL_CLK22_OEN_M  (APB_CTRL_CLK22_OEN_V << APB_CTRL_CLK22_OEN_S)
#define APB_CTRL_CLK22_OEN_V  0x00000001
#define APB_CTRL_CLK22_OEN_S  1

/* APB_CTRL_CLK20_OEN : R/W; bitpos: [0]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_CLK20_OEN    (BIT(0))
#define APB_CTRL_CLK20_OEN_M  (APB_CTRL_CLK20_OEN_V << APB_CTRL_CLK20_OEN_S)
#define APB_CTRL_CLK20_OEN_V  0x00000001
#define APB_CTRL_CLK20_OEN_S  0

/* APB_CTRL_WIFI_BB_CFG_REG register
 * ******* Description ***********
 */

#define APB_CTRL_WIFI_BB_CFG_REG (DR_REG_APB_CTRL_BASE + 0xc)

/* APB_CTRL_WIFI_BB_CFG : R/W; bitpos: [31:0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_WIFI_BB_CFG    0xffffffff
#define APB_CTRL_WIFI_BB_CFG_M  (APB_CTRL_WIFI_BB_CFG_V << APB_CTRL_WIFI_BB_CFG_S)
#define APB_CTRL_WIFI_BB_CFG_V  0xffffffff
#define APB_CTRL_WIFI_BB_CFG_S  0

/* APB_CTRL_WIFI_BB_CFG_2_REG register
 * ******* Description ***********
 */

#define APB_CTRL_WIFI_BB_CFG_2_REG (DR_REG_APB_CTRL_BASE + 0x10)

/* APB_CTRL_WIFI_BB_CFG_2 : R/W; bitpos: [31:0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_WIFI_BB_CFG_2    0xffffffff
#define APB_CTRL_WIFI_BB_CFG_2_M  (APB_CTRL_WIFI_BB_CFG_2_V << APB_CTRL_WIFI_BB_CFG_2_S)
#define APB_CTRL_WIFI_BB_CFG_2_V  0xffffffff
#define APB_CTRL_WIFI_BB_CFG_2_S  0

/* APB_CTRL_WIFI_CLK_EN_REG register
 * ******* Description ***********
 */

#define APB_CTRL_WIFI_CLK_EN_REG (DR_REG_APB_CTRL_BASE + 0x14)

/* APB_CTRL_WIFI_CLK_EN : R/W; bitpos: [31:0]; default: 4294762544;
 * ******* Description ***********
 */

#define APB_CTRL_WIFI_CLK_EN    0xffffffff
#define APB_CTRL_WIFI_CLK_EN_M  (APB_CTRL_WIFI_CLK_EN_V << APB_CTRL_WIFI_CLK_EN_S)
#define APB_CTRL_WIFI_CLK_EN_V  0xffffffff
#define APB_CTRL_WIFI_CLK_EN_S  0

/* APB_CTRL_WIFI_RST_EN_REG register
 * ******* Description ***********
 */

#define APB_CTRL_WIFI_RST_EN_REG (DR_REG_APB_CTRL_BASE + 0x18)

/* APB_CTRL_WIFI_RST : R/W; bitpos: [31:0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_WIFI_RST    0xffffffff
#define APB_CTRL_WIFI_RST_M  (APB_CTRL_WIFI_RST_V << APB_CTRL_WIFI_RST_S)
#define APB_CTRL_WIFI_RST_V  0xffffffff
#define APB_CTRL_WIFI_RST_S  0

/* APB_CTRL_HOST_INF_SEL_REG register
 * ******* Description ***********
 */

#define APB_CTRL_HOST_INF_SEL_REG (DR_REG_APB_CTRL_BASE + 0x1c)

/* APB_CTRL_PERI_IO_SWAP : R/W; bitpos: [7:0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_PERI_IO_SWAP    0x000000ff
#define APB_CTRL_PERI_IO_SWAP_M  (APB_CTRL_PERI_IO_SWAP_V << APB_CTRL_PERI_IO_SWAP_S)
#define APB_CTRL_PERI_IO_SWAP_V  0x000000ff
#define APB_CTRL_PERI_IO_SWAP_S  0

/* APB_CTRL_EXT_MEM_PMS_LOCK_REG register
 * ******* Description ***********
 */

#define APB_CTRL_EXT_MEM_PMS_LOCK_REG (DR_REG_APB_CTRL_BASE + 0x20)

/* APB_CTRL_EXT_MEM_PMS_LOCK : R/W; bitpos: [0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_EXT_MEM_PMS_LOCK    (BIT(0))
#define APB_CTRL_EXT_MEM_PMS_LOCK_M  (APB_CTRL_EXT_MEM_PMS_LOCK_V << APB_CTRL_EXT_MEM_PMS_LOCK_S)
#define APB_CTRL_EXT_MEM_PMS_LOCK_V  0x00000001
#define APB_CTRL_EXT_MEM_PMS_LOCK_S  0

/* APB_CTRL_EXT_MEM_WRITEBACK_BYPASS_REG register
 * ******* Description ***********
 */

#define APB_CTRL_EXT_MEM_WRITEBACK_BYPASS_REG (DR_REG_APB_CTRL_BASE + 0x24)

/* APB_CTRL_WRITEBACK_BYPASS : R/W; bitpos: [0]; default: 0;
 * Set 1 to bypass cache writeback request to external memory so that spi
 * will not check its attribute.
 */

#define APB_CTRL_WRITEBACK_BYPASS    (BIT(0))
#define APB_CTRL_WRITEBACK_BYPASS_M  (APB_CTRL_WRITEBACK_BYPASS_V << APB_CTRL_WRITEBACK_BYPASS_S)
#define APB_CTRL_WRITEBACK_BYPASS_V  0x00000001
#define APB_CTRL_WRITEBACK_BYPASS_S  0

/* APB_CTRL_FLASH_ACE0_ATTR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE0_ATTR_REG (DR_REG_APB_CTRL_BASE + 0x28)

/* APB_CTRL_FLASH_ACE0_ATTR : R/W; bitpos: [8:0]; default: 255;
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE0_ATTR    0x000001ff
#define APB_CTRL_FLASH_ACE0_ATTR_M  (APB_CTRL_FLASH_ACE0_ATTR_V << APB_CTRL_FLASH_ACE0_ATTR_S)
#define APB_CTRL_FLASH_ACE0_ATTR_V  0x000001ff
#define APB_CTRL_FLASH_ACE0_ATTR_S  0

/* APB_CTRL_FLASH_ACE1_ATTR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE1_ATTR_REG (DR_REG_APB_CTRL_BASE + 0x2c)

/* APB_CTRL_FLASH_ACE1_ATTR : R/W; bitpos: [8:0]; default: 255;
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE1_ATTR    0x000001ff
#define APB_CTRL_FLASH_ACE1_ATTR_M  (APB_CTRL_FLASH_ACE1_ATTR_V << APB_CTRL_FLASH_ACE1_ATTR_S)
#define APB_CTRL_FLASH_ACE1_ATTR_V  0x000001ff
#define APB_CTRL_FLASH_ACE1_ATTR_S  0

/* APB_CTRL_FLASH_ACE2_ATTR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE2_ATTR_REG (DR_REG_APB_CTRL_BASE + 0x30)

/* APB_CTRL_FLASH_ACE2_ATTR : R/W; bitpos: [8:0]; default: 255;
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE2_ATTR    0x000001ff
#define APB_CTRL_FLASH_ACE2_ATTR_M  (APB_CTRL_FLASH_ACE2_ATTR_V << APB_CTRL_FLASH_ACE2_ATTR_S)
#define APB_CTRL_FLASH_ACE2_ATTR_V  0x000001ff
#define APB_CTRL_FLASH_ACE2_ATTR_S  0

/* APB_CTRL_FLASH_ACE3_ATTR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE3_ATTR_REG (DR_REG_APB_CTRL_BASE + 0x34)

/* APB_CTRL_FLASH_ACE3_ATTR : R/W; bitpos: [8:0]; default: 255;
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE3_ATTR    0x000001ff
#define APB_CTRL_FLASH_ACE3_ATTR_M  (APB_CTRL_FLASH_ACE3_ATTR_V << APB_CTRL_FLASH_ACE3_ATTR_S)
#define APB_CTRL_FLASH_ACE3_ATTR_V  0x000001ff
#define APB_CTRL_FLASH_ACE3_ATTR_S  0

/* APB_CTRL_FLASH_ACE0_ADDR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE0_ADDR_REG (DR_REG_APB_CTRL_BASE + 0x38)

/* APB_CTRL_FLASH_ACE0_ADDR_S : R/W; bitpos: [31:0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE0_ADDR_S    0xffffffff
#define APB_CTRL_FLASH_ACE0_ADDR_S_M  (APB_CTRL_FLASH_ACE0_ADDR_S_V << APB_CTRL_FLASH_ACE0_ADDR_S_S)
#define APB_CTRL_FLASH_ACE0_ADDR_S_V  0xffffffff
#define APB_CTRL_FLASH_ACE0_ADDR_S_S  0

/* APB_CTRL_FLASH_ACE1_ADDR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE1_ADDR_REG (DR_REG_APB_CTRL_BASE + 0x3c)

/* APB_CTRL_FLASH_ACE1_ADDR_S : R/W; bitpos: [31:0]; default: 268435456;
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE1_ADDR_S    0xffffffff
#define APB_CTRL_FLASH_ACE1_ADDR_S_M  (APB_CTRL_FLASH_ACE1_ADDR_S_V << APB_CTRL_FLASH_ACE1_ADDR_S_S)
#define APB_CTRL_FLASH_ACE1_ADDR_S_V  0xffffffff
#define APB_CTRL_FLASH_ACE1_ADDR_S_S  0

/* APB_CTRL_FLASH_ACE2_ADDR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE2_ADDR_REG (DR_REG_APB_CTRL_BASE + 0x40)

/* APB_CTRL_FLASH_ACE2_ADDR_S : R/W; bitpos: [31:0]; default: 536870912;
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE2_ADDR_S    0xffffffff
#define APB_CTRL_FLASH_ACE2_ADDR_S_M  (APB_CTRL_FLASH_ACE2_ADDR_S_V << APB_CTRL_FLASH_ACE2_ADDR_S_S)
#define APB_CTRL_FLASH_ACE2_ADDR_S_V  0xffffffff
#define APB_CTRL_FLASH_ACE2_ADDR_S_S  0

/* APB_CTRL_FLASH_ACE3_ADDR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE3_ADDR_REG (DR_REG_APB_CTRL_BASE + 0x44)

/* APB_CTRL_FLASH_ACE3_ADDR_S : R/W; bitpos: [31:0]; default: 805306368;
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE3_ADDR_S    0xffffffff
#define APB_CTRL_FLASH_ACE3_ADDR_S_M  (APB_CTRL_FLASH_ACE3_ADDR_S_V << APB_CTRL_FLASH_ACE3_ADDR_S_S)
#define APB_CTRL_FLASH_ACE3_ADDR_S_V  0xffffffff
#define APB_CTRL_FLASH_ACE3_ADDR_S_S  0

/* APB_CTRL_FLASH_ACE0_SIZE_REG register
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE0_SIZE_REG (DR_REG_APB_CTRL_BASE + 0x48)

/* APB_CTRL_FLASH_ACE0_SIZE : R/W; bitpos: [15:0]; default: 4096;
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE0_SIZE    0x0000ffff
#define APB_CTRL_FLASH_ACE0_SIZE_M  (APB_CTRL_FLASH_ACE0_SIZE_V << APB_CTRL_FLASH_ACE0_SIZE_S)
#define APB_CTRL_FLASH_ACE0_SIZE_V  0x0000ffff
#define APB_CTRL_FLASH_ACE0_SIZE_S  0

/* APB_CTRL_FLASH_ACE1_SIZE_REG register
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE1_SIZE_REG (DR_REG_APB_CTRL_BASE + 0x4c)

/* APB_CTRL_FLASH_ACE1_SIZE : R/W; bitpos: [15:0]; default: 4096;
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE1_SIZE    0x0000ffff
#define APB_CTRL_FLASH_ACE1_SIZE_M  (APB_CTRL_FLASH_ACE1_SIZE_V << APB_CTRL_FLASH_ACE1_SIZE_S)
#define APB_CTRL_FLASH_ACE1_SIZE_V  0x0000ffff
#define APB_CTRL_FLASH_ACE1_SIZE_S  0

/* APB_CTRL_FLASH_ACE2_SIZE_REG register
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE2_SIZE_REG (DR_REG_APB_CTRL_BASE + 0x50)

/* APB_CTRL_FLASH_ACE2_SIZE : R/W; bitpos: [15:0]; default: 4096;
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE2_SIZE    0x0000ffff
#define APB_CTRL_FLASH_ACE2_SIZE_M  (APB_CTRL_FLASH_ACE2_SIZE_V << APB_CTRL_FLASH_ACE2_SIZE_S)
#define APB_CTRL_FLASH_ACE2_SIZE_V  0x0000ffff
#define APB_CTRL_FLASH_ACE2_SIZE_S  0

/* APB_CTRL_FLASH_ACE3_SIZE_REG register
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE3_SIZE_REG (DR_REG_APB_CTRL_BASE + 0x54)

/* APB_CTRL_FLASH_ACE3_SIZE : R/W; bitpos: [15:0]; default: 4096;
 * ******* Description ***********
 */

#define APB_CTRL_FLASH_ACE3_SIZE    0x0000ffff
#define APB_CTRL_FLASH_ACE3_SIZE_M  (APB_CTRL_FLASH_ACE3_SIZE_V << APB_CTRL_FLASH_ACE3_SIZE_S)
#define APB_CTRL_FLASH_ACE3_SIZE_V  0x0000ffff
#define APB_CTRL_FLASH_ACE3_SIZE_S  0

/* APB_CTRL_SRAM_ACE0_ATTR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE0_ATTR_REG (DR_REG_APB_CTRL_BASE + 0x58)

/* APB_CTRL_SRAM_ACE0_ATTR : R/W; bitpos: [8:0]; default: 255;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE0_ATTR    0x000001ff
#define APB_CTRL_SRAM_ACE0_ATTR_M  (APB_CTRL_SRAM_ACE0_ATTR_V << APB_CTRL_SRAM_ACE0_ATTR_S)
#define APB_CTRL_SRAM_ACE0_ATTR_V  0x000001ff
#define APB_CTRL_SRAM_ACE0_ATTR_S  0

/* APB_CTRL_SRAM_ACE1_ATTR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE1_ATTR_REG (DR_REG_APB_CTRL_BASE + 0x5c)

/* APB_CTRL_SRAM_ACE1_ATTR : R/W; bitpos: [8:0]; default: 255;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE1_ATTR    0x000001ff
#define APB_CTRL_SRAM_ACE1_ATTR_M  (APB_CTRL_SRAM_ACE1_ATTR_V << APB_CTRL_SRAM_ACE1_ATTR_S)
#define APB_CTRL_SRAM_ACE1_ATTR_V  0x000001ff
#define APB_CTRL_SRAM_ACE1_ATTR_S  0

/* APB_CTRL_SRAM_ACE2_ATTR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE2_ATTR_REG (DR_REG_APB_CTRL_BASE + 0x60)

/* APB_CTRL_SRAM_ACE2_ATTR : R/W; bitpos: [8:0]; default: 255;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE2_ATTR    0x000001ff
#define APB_CTRL_SRAM_ACE2_ATTR_M  (APB_CTRL_SRAM_ACE2_ATTR_V << APB_CTRL_SRAM_ACE2_ATTR_S)
#define APB_CTRL_SRAM_ACE2_ATTR_V  0x000001ff
#define APB_CTRL_SRAM_ACE2_ATTR_S  0

/* APB_CTRL_SRAM_ACE3_ATTR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE3_ATTR_REG (DR_REG_APB_CTRL_BASE + 0x64)

/* APB_CTRL_SRAM_ACE3_ATTR : R/W; bitpos: [8:0]; default: 255;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE3_ATTR    0x000001ff
#define APB_CTRL_SRAM_ACE3_ATTR_M  (APB_CTRL_SRAM_ACE3_ATTR_V << APB_CTRL_SRAM_ACE3_ATTR_S)
#define APB_CTRL_SRAM_ACE3_ATTR_V  0x000001ff
#define APB_CTRL_SRAM_ACE3_ATTR_S  0

/* APB_CTRL_SRAM_ACE0_ADDR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE0_ADDR_REG (DR_REG_APB_CTRL_BASE + 0x68)

/* APB_CTRL_SRAM_ACE0_ADDR_S : R/W; bitpos: [31:0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE0_ADDR_S    0xffffffff
#define APB_CTRL_SRAM_ACE0_ADDR_S_M  (APB_CTRL_SRAM_ACE0_ADDR_S_V << APB_CTRL_SRAM_ACE0_ADDR_S_S)
#define APB_CTRL_SRAM_ACE0_ADDR_S_V  0xffffffff
#define APB_CTRL_SRAM_ACE0_ADDR_S_S  0

/* APB_CTRL_SRAM_ACE1_ADDR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE1_ADDR_REG (DR_REG_APB_CTRL_BASE + 0x6c)

/* APB_CTRL_SRAM_ACE1_ADDR_S : R/W; bitpos: [31:0]; default: 268435456;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE1_ADDR_S    0xffffffff
#define APB_CTRL_SRAM_ACE1_ADDR_S_M  (APB_CTRL_SRAM_ACE1_ADDR_S_V << APB_CTRL_SRAM_ACE1_ADDR_S_S)
#define APB_CTRL_SRAM_ACE1_ADDR_S_V  0xffffffff
#define APB_CTRL_SRAM_ACE1_ADDR_S_S  0

/* APB_CTRL_SRAM_ACE2_ADDR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE2_ADDR_REG (DR_REG_APB_CTRL_BASE + 0x70)

/* APB_CTRL_SRAM_ACE2_ADDR_S : R/W; bitpos: [31:0]; default: 536870912;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE2_ADDR_S    0xffffffff
#define APB_CTRL_SRAM_ACE2_ADDR_S_M  (APB_CTRL_SRAM_ACE2_ADDR_S_V << APB_CTRL_SRAM_ACE2_ADDR_S_S)
#define APB_CTRL_SRAM_ACE2_ADDR_S_V  0xffffffff
#define APB_CTRL_SRAM_ACE2_ADDR_S_S  0

/* APB_CTRL_SRAM_ACE3_ADDR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE3_ADDR_REG (DR_REG_APB_CTRL_BASE + 0x74)

/* APB_CTRL_SRAM_ACE3_ADDR_S : R/W; bitpos: [31:0]; default: 805306368;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE3_ADDR_S    0xffffffff
#define APB_CTRL_SRAM_ACE3_ADDR_S_M  (APB_CTRL_SRAM_ACE3_ADDR_S_V << APB_CTRL_SRAM_ACE3_ADDR_S_S)
#define APB_CTRL_SRAM_ACE3_ADDR_S_V  0xffffffff
#define APB_CTRL_SRAM_ACE3_ADDR_S_S  0

/* APB_CTRL_SRAM_ACE0_SIZE_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE0_SIZE_REG (DR_REG_APB_CTRL_BASE + 0x78)

/* APB_CTRL_SRAM_ACE0_SIZE : R/W; bitpos: [15:0]; default: 4096;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE0_SIZE    0x0000ffff
#define APB_CTRL_SRAM_ACE0_SIZE_M  (APB_CTRL_SRAM_ACE0_SIZE_V << APB_CTRL_SRAM_ACE0_SIZE_S)
#define APB_CTRL_SRAM_ACE0_SIZE_V  0x0000ffff
#define APB_CTRL_SRAM_ACE0_SIZE_S  0

/* APB_CTRL_SRAM_ACE1_SIZE_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE1_SIZE_REG (DR_REG_APB_CTRL_BASE + 0x7c)

/* APB_CTRL_SRAM_ACE1_SIZE : R/W; bitpos: [15:0]; default: 4096;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE1_SIZE    0x0000ffff
#define APB_CTRL_SRAM_ACE1_SIZE_M  (APB_CTRL_SRAM_ACE1_SIZE_V << APB_CTRL_SRAM_ACE1_SIZE_S)
#define APB_CTRL_SRAM_ACE1_SIZE_V  0x0000ffff
#define APB_CTRL_SRAM_ACE1_SIZE_S  0

/* APB_CTRL_SRAM_ACE2_SIZE_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE2_SIZE_REG (DR_REG_APB_CTRL_BASE + 0x80)

/* APB_CTRL_SRAM_ACE2_SIZE : R/W; bitpos: [15:0]; default: 4096;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE2_SIZE    0x0000ffff
#define APB_CTRL_SRAM_ACE2_SIZE_M  (APB_CTRL_SRAM_ACE2_SIZE_V << APB_CTRL_SRAM_ACE2_SIZE_S)
#define APB_CTRL_SRAM_ACE2_SIZE_V  0x0000ffff
#define APB_CTRL_SRAM_ACE2_SIZE_S  0

/* APB_CTRL_SRAM_ACE3_SIZE_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE3_SIZE_REG (DR_REG_APB_CTRL_BASE + 0x84)

/* APB_CTRL_SRAM_ACE3_SIZE : R/W; bitpos: [15:0]; default: 4096;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_ACE3_SIZE    0x0000ffff
#define APB_CTRL_SRAM_ACE3_SIZE_M  (APB_CTRL_SRAM_ACE3_SIZE_V << APB_CTRL_SRAM_ACE3_SIZE_S)
#define APB_CTRL_SRAM_ACE3_SIZE_V  0x0000ffff
#define APB_CTRL_SRAM_ACE3_SIZE_S  0

/* APB_CTRL_SPI_MEM_PMS_CTRL_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SPI_MEM_PMS_CTRL_REG (DR_REG_APB_CTRL_BASE + 0x88)

/* APB_CTRL_SPI_MEM_REJECT_CDE : RO; bitpos: [6:2]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_SPI_MEM_REJECT_CDE    0x0000001f
#define APB_CTRL_SPI_MEM_REJECT_CDE_M  (APB_CTRL_SPI_MEM_REJECT_CDE_V << APB_CTRL_SPI_MEM_REJECT_CDE_S)
#define APB_CTRL_SPI_MEM_REJECT_CDE_V  0x0000001f
#define APB_CTRL_SPI_MEM_REJECT_CDE_S  2

/* APB_CTRL_SPI_MEM_REJECT_CLR : WOD; bitpos: [1]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_SPI_MEM_REJECT_CLR    (BIT(1))
#define APB_CTRL_SPI_MEM_REJECT_CLR_M  (APB_CTRL_SPI_MEM_REJECT_CLR_V << APB_CTRL_SPI_MEM_REJECT_CLR_S)
#define APB_CTRL_SPI_MEM_REJECT_CLR_V  0x00000001
#define APB_CTRL_SPI_MEM_REJECT_CLR_S  1

/* APB_CTRL_SPI_MEM_REJECT_INT : RO; bitpos: [0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_SPI_MEM_REJECT_INT    (BIT(0))
#define APB_CTRL_SPI_MEM_REJECT_INT_M  (APB_CTRL_SPI_MEM_REJECT_INT_V << APB_CTRL_SPI_MEM_REJECT_INT_S)
#define APB_CTRL_SPI_MEM_REJECT_INT_V  0x00000001
#define APB_CTRL_SPI_MEM_REJECT_INT_S  0

/* APB_CTRL_SPI_MEM_REJECT_ADDR_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SPI_MEM_REJECT_ADDR_REG (DR_REG_APB_CTRL_BASE + 0x8c)

/* APB_CTRL_SPI_MEM_REJECT_ADDR : RO; bitpos: [31:0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_SPI_MEM_REJECT_ADDR    0xffffffff
#define APB_CTRL_SPI_MEM_REJECT_ADDR_M  (APB_CTRL_SPI_MEM_REJECT_ADDR_V << APB_CTRL_SPI_MEM_REJECT_ADDR_S)
#define APB_CTRL_SPI_MEM_REJECT_ADDR_V  0xffffffff
#define APB_CTRL_SPI_MEM_REJECT_ADDR_S  0

/* APB_CTRL_SDIO_CTRL_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SDIO_CTRL_REG (DR_REG_APB_CTRL_BASE + 0x90)

/* APB_CTRL_SDIO_WIN_ACCESS_EN : R/W; bitpos: [0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_SDIO_WIN_ACCESS_EN    (BIT(0))
#define APB_CTRL_SDIO_WIN_ACCESS_EN_M  (APB_CTRL_SDIO_WIN_ACCESS_EN_V << APB_CTRL_SDIO_WIN_ACCESS_EN_S)
#define APB_CTRL_SDIO_WIN_ACCESS_EN_V  0x00000001
#define APB_CTRL_SDIO_WIN_ACCESS_EN_S  0

/* APB_CTRL_REDCY_SIG0_REG register
 * ******* Description ***********
 */

#define APB_CTRL_REDCY_SIG0_REG (DR_REG_APB_CTRL_BASE + 0x94)

/* APB_CTRL_REDCY_ANDOR : RO; bitpos: [31]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_REDCY_ANDOR    (BIT(31))
#define APB_CTRL_REDCY_ANDOR_M  (APB_CTRL_REDCY_ANDOR_V << APB_CTRL_REDCY_ANDOR_S)
#define APB_CTRL_REDCY_ANDOR_V  0x00000001
#define APB_CTRL_REDCY_ANDOR_S  31

/* APB_CTRL_REDCY_SIG0 : R/W; bitpos: [30:0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_REDCY_SIG0    0x7fffffff
#define APB_CTRL_REDCY_SIG0_M  (APB_CTRL_REDCY_SIG0_V << APB_CTRL_REDCY_SIG0_S)
#define APB_CTRL_REDCY_SIG0_V  0x7fffffff
#define APB_CTRL_REDCY_SIG0_S  0

/* APB_CTRL_REDCY_SIG1_REG register
 * ******* Description ***********
 */

#define APB_CTRL_REDCY_SIG1_REG (DR_REG_APB_CTRL_BASE + 0x98)

/* APB_CTRL_REDCY_NANDOR : RO; bitpos: [31]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_REDCY_NANDOR    (BIT(31))
#define APB_CTRL_REDCY_NANDOR_M  (APB_CTRL_REDCY_NANDOR_V << APB_CTRL_REDCY_NANDOR_S)
#define APB_CTRL_REDCY_NANDOR_V  0x00000001
#define APB_CTRL_REDCY_NANDOR_S  31

/* APB_CTRL_REDCY_SIG1 : R/W; bitpos: [30:0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_REDCY_SIG1    0x7fffffff
#define APB_CTRL_REDCY_SIG1_M  (APB_CTRL_REDCY_SIG1_V << APB_CTRL_REDCY_SIG1_S)
#define APB_CTRL_REDCY_SIG1_V  0x7fffffff
#define APB_CTRL_REDCY_SIG1_S  0

/* APB_CTRL_FRONT_END_MEM_PD_REG register
 * ******* Description ***********
 */

#define APB_CTRL_FRONT_END_MEM_PD_REG (DR_REG_APB_CTRL_BASE + 0x9c)

/* APB_CTRL_FREQ_MEM_FORCE_PD : R/W; bitpos: [7]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_FREQ_MEM_FORCE_PD    (BIT(7))
#define APB_CTRL_FREQ_MEM_FORCE_PD_M  (APB_CTRL_FREQ_MEM_FORCE_PD_V << APB_CTRL_FREQ_MEM_FORCE_PD_S)
#define APB_CTRL_FREQ_MEM_FORCE_PD_V  0x00000001
#define APB_CTRL_FREQ_MEM_FORCE_PD_S  7

/* APB_CTRL_FREQ_MEM_FORCE_PU : R/W; bitpos: [6]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_FREQ_MEM_FORCE_PU    (BIT(6))
#define APB_CTRL_FREQ_MEM_FORCE_PU_M  (APB_CTRL_FREQ_MEM_FORCE_PU_V << APB_CTRL_FREQ_MEM_FORCE_PU_S)
#define APB_CTRL_FREQ_MEM_FORCE_PU_V  0x00000001
#define APB_CTRL_FREQ_MEM_FORCE_PU_S  6

/* APB_CTRL_DC_MEM_FORCE_PD : R/W; bitpos: [5]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_DC_MEM_FORCE_PD    (BIT(5))
#define APB_CTRL_DC_MEM_FORCE_PD_M  (APB_CTRL_DC_MEM_FORCE_PD_V << APB_CTRL_DC_MEM_FORCE_PD_S)
#define APB_CTRL_DC_MEM_FORCE_PD_V  0x00000001
#define APB_CTRL_DC_MEM_FORCE_PD_S  5

/* APB_CTRL_DC_MEM_FORCE_PU : R/W; bitpos: [4]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_DC_MEM_FORCE_PU    (BIT(4))
#define APB_CTRL_DC_MEM_FORCE_PU_M  (APB_CTRL_DC_MEM_FORCE_PU_V << APB_CTRL_DC_MEM_FORCE_PU_S)
#define APB_CTRL_DC_MEM_FORCE_PU_V  0x00000001
#define APB_CTRL_DC_MEM_FORCE_PU_S  4

/* APB_CTRL_PBUS_MEM_FORCE_PD : R/W; bitpos: [3]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_PBUS_MEM_FORCE_PD    (BIT(3))
#define APB_CTRL_PBUS_MEM_FORCE_PD_M  (APB_CTRL_PBUS_MEM_FORCE_PD_V << APB_CTRL_PBUS_MEM_FORCE_PD_S)
#define APB_CTRL_PBUS_MEM_FORCE_PD_V  0x00000001
#define APB_CTRL_PBUS_MEM_FORCE_PD_S  3

/* APB_CTRL_PBUS_MEM_FORCE_PU : R/W; bitpos: [2]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_PBUS_MEM_FORCE_PU    (BIT(2))
#define APB_CTRL_PBUS_MEM_FORCE_PU_M  (APB_CTRL_PBUS_MEM_FORCE_PU_V << APB_CTRL_PBUS_MEM_FORCE_PU_S)
#define APB_CTRL_PBUS_MEM_FORCE_PU_V  0x00000001
#define APB_CTRL_PBUS_MEM_FORCE_PU_S  2

/* APB_CTRL_AGC_MEM_FORCE_PD : R/W; bitpos: [1]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_AGC_MEM_FORCE_PD    (BIT(1))
#define APB_CTRL_AGC_MEM_FORCE_PD_M  (APB_CTRL_AGC_MEM_FORCE_PD_V << APB_CTRL_AGC_MEM_FORCE_PD_S)
#define APB_CTRL_AGC_MEM_FORCE_PD_V  0x00000001
#define APB_CTRL_AGC_MEM_FORCE_PD_S  1

/* APB_CTRL_AGC_MEM_FORCE_PU : R/W; bitpos: [0]; default: 1;
 * ******* Description ***********
 */

#define APB_CTRL_AGC_MEM_FORCE_PU    (BIT(0))
#define APB_CTRL_AGC_MEM_FORCE_PU_M  (APB_CTRL_AGC_MEM_FORCE_PU_V << APB_CTRL_AGC_MEM_FORCE_PU_S)
#define APB_CTRL_AGC_MEM_FORCE_PU_V  0x00000001
#define APB_CTRL_AGC_MEM_FORCE_PU_S  0

/* APB_CTRL_SPI_MEM_ECC_CTRL_REG register
 * ******* Description ***********
 */

#define APB_CTRL_SPI_MEM_ECC_CTRL_REG (DR_REG_APB_CTRL_BASE + 0xa0)

/* APB_CTRL_SRAM_PAGE_SIZE : R/W; bitpos: [21:20]; default: 2;
 * Set the page size of the used MSPI external RAM. 0: 256 bytes. 1: 512
 * bytes. 2: 1024 bytes. 3: 2048 bytes.
 */

#define APB_CTRL_SRAM_PAGE_SIZE    0x00000003
#define APB_CTRL_SRAM_PAGE_SIZE_M  (APB_CTRL_SRAM_PAGE_SIZE_V << APB_CTRL_SRAM_PAGE_SIZE_S)
#define APB_CTRL_SRAM_PAGE_SIZE_V  0x00000003
#define APB_CTRL_SRAM_PAGE_SIZE_S  20

/* APB_CTRL_FLASH_PAGE_SIZE : R/W; bitpos: [19:18]; default: 0;
 * Set the page size of the used MSPI flash. 0: 256 bytes. 1: 512 bytes. 2:
 * 1024 bytes. 3: 2048 bytes.
 */

#define APB_CTRL_FLASH_PAGE_SIZE    0x00000003
#define APB_CTRL_FLASH_PAGE_SIZE_M  (APB_CTRL_FLASH_PAGE_SIZE_V << APB_CTRL_FLASH_PAGE_SIZE_S)
#define APB_CTRL_FLASH_PAGE_SIZE_V  0x00000003
#define APB_CTRL_FLASH_PAGE_SIZE_S  18

/* APB_CTRL_CLKGATE_FORCE_ON_REG register
 * ******* Description ***********
 */

#define APB_CTRL_CLKGATE_FORCE_ON_REG (DR_REG_APB_CTRL_BASE + 0xa8)

/* APB_CTRL_SRAM_CLKGATE_FORCE_ON : R/W; bitpos: [13:3]; default: 2047;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_CLKGATE_FORCE_ON    0x000007ff
#define APB_CTRL_SRAM_CLKGATE_FORCE_ON_M  (APB_CTRL_SRAM_CLKGATE_FORCE_ON_V << APB_CTRL_SRAM_CLKGATE_FORCE_ON_S)
#define APB_CTRL_SRAM_CLKGATE_FORCE_ON_V  0x000007ff
#define APB_CTRL_SRAM_CLKGATE_FORCE_ON_S  3

/* APB_CTRL_ROM_CLKGATE_FORCE_ON : R/W; bitpos: [2:0]; default: 7;
 * ******* Description ***********
 */

#define APB_CTRL_ROM_CLKGATE_FORCE_ON    0x00000007
#define APB_CTRL_ROM_CLKGATE_FORCE_ON_M  (APB_CTRL_ROM_CLKGATE_FORCE_ON_V << APB_CTRL_ROM_CLKGATE_FORCE_ON_S)
#define APB_CTRL_ROM_CLKGATE_FORCE_ON_V  0x00000007
#define APB_CTRL_ROM_CLKGATE_FORCE_ON_S  0

/* APB_CTRL_MEM_POWER_DOWN_REG register
 * ******* Description ***********
 */

#define APB_CTRL_MEM_POWER_DOWN_REG (DR_REG_APB_CTRL_BASE + 0xac)

/* APB_CTRL_SRAM_POWER_DOWN : R/W; bitpos: [13:3]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_POWER_DOWN    0x000007ff
#define APB_CTRL_SRAM_POWER_DOWN_M  (APB_CTRL_SRAM_POWER_DOWN_V << APB_CTRL_SRAM_POWER_DOWN_S)
#define APB_CTRL_SRAM_POWER_DOWN_V  0x000007ff
#define APB_CTRL_SRAM_POWER_DOWN_S  3

/* APB_CTRL_ROM_POWER_DOWN : R/W; bitpos: [2:0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_ROM_POWER_DOWN    0x00000007
#define APB_CTRL_ROM_POWER_DOWN_M  (APB_CTRL_ROM_POWER_DOWN_V << APB_CTRL_ROM_POWER_DOWN_S)
#define APB_CTRL_ROM_POWER_DOWN_V  0x00000007
#define APB_CTRL_ROM_POWER_DOWN_S  0

/* APB_CTRL_MEM_POWER_UP_REG register
 * ******* Description ***********
 */

#define APB_CTRL_MEM_POWER_UP_REG (DR_REG_APB_CTRL_BASE + 0xb0)

/* APB_CTRL_SRAM_POWER_UP : R/W; bitpos: [13:3]; default: 2047;
 * ******* Description ***********
 */

#define APB_CTRL_SRAM_POWER_UP    0x000007ff
#define APB_CTRL_SRAM_POWER_UP_M  (APB_CTRL_SRAM_POWER_UP_V << APB_CTRL_SRAM_POWER_UP_S)
#define APB_CTRL_SRAM_POWER_UP_V  0x000007ff
#define APB_CTRL_SRAM_POWER_UP_S  3

/* APB_CTRL_ROM_POWER_UP : R/W; bitpos: [2:0]; default: 7;
 * ******* Description ***********
 */

#define APB_CTRL_ROM_POWER_UP    0x00000007
#define APB_CTRL_ROM_POWER_UP_M  (APB_CTRL_ROM_POWER_UP_V << APB_CTRL_ROM_POWER_UP_S)
#define APB_CTRL_ROM_POWER_UP_V  0x00000007
#define APB_CTRL_ROM_POWER_UP_S  0

/* APB_CTRL_RETENTION_CTRL_REG register
 * ******* Description ***********
 */

#define APB_CTRL_RETENTION_CTRL_REG (DR_REG_APB_CTRL_BASE + 0xb4)

/* APB_CTRL_NOBYPASS_CPU_ISO_RST : R/W; bitpos: [27]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_NOBYPASS_CPU_ISO_RST    (BIT(27))
#define APB_CTRL_NOBYPASS_CPU_ISO_RST_M  (APB_CTRL_NOBYPASS_CPU_ISO_RST_V << APB_CTRL_NOBYPASS_CPU_ISO_RST_S)
#define APB_CTRL_NOBYPASS_CPU_ISO_RST_V  0x00000001
#define APB_CTRL_NOBYPASS_CPU_ISO_RST_S  27

/* APB_CTRL_RETENTION_CPU_LINK_ADDR : R/W; bitpos: [26:0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_RETENTION_CPU_LINK_ADDR    0x07ffffff
#define APB_CTRL_RETENTION_CPU_LINK_ADDR_M  (APB_CTRL_RETENTION_CPU_LINK_ADDR_V << APB_CTRL_RETENTION_CPU_LINK_ADDR_S)
#define APB_CTRL_RETENTION_CPU_LINK_ADDR_V  0x07ffffff
#define APB_CTRL_RETENTION_CPU_LINK_ADDR_S  0

/* APB_CTRL_RETENTION_CTRL1_REG register
 * ******* Description ***********
 */

#define APB_CTRL_RETENTION_CTRL1_REG (DR_REG_APB_CTRL_BASE + 0xb8)

/* APB_CTRL_RETENTION_TAG_LINK_ADDR : R/W; bitpos: [26:0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_RETENTION_TAG_LINK_ADDR    0x07ffffff
#define APB_CTRL_RETENTION_TAG_LINK_ADDR_M  (APB_CTRL_RETENTION_TAG_LINK_ADDR_V << APB_CTRL_RETENTION_TAG_LINK_ADDR_S)
#define APB_CTRL_RETENTION_TAG_LINK_ADDR_V  0x07ffffff
#define APB_CTRL_RETENTION_TAG_LINK_ADDR_S  0

/* APB_CTRL_RETENTION_CTRL2_REG register
 * ******* Description ***********
 */

#define APB_CTRL_RETENTION_CTRL2_REG (DR_REG_APB_CTRL_BASE + 0xbc)

/* APB_CTRL_RET_ICACHE_ENABLE : R/W; bitpos: [31]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_RET_ICACHE_ENABLE    (BIT(31))
#define APB_CTRL_RET_ICACHE_ENABLE_M  (APB_CTRL_RET_ICACHE_ENABLE_V << APB_CTRL_RET_ICACHE_ENABLE_S)
#define APB_CTRL_RET_ICACHE_ENABLE_V  0x00000001
#define APB_CTRL_RET_ICACHE_ENABLE_S  31

/* APB_CTRL_RET_ICACHE_START_POINT : R/W; bitpos: [29:22]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_RET_ICACHE_START_POINT    0x000000ff
#define APB_CTRL_RET_ICACHE_START_POINT_M  (APB_CTRL_RET_ICACHE_START_POINT_V << APB_CTRL_RET_ICACHE_START_POINT_S)
#define APB_CTRL_RET_ICACHE_START_POINT_V  0x000000ff
#define APB_CTRL_RET_ICACHE_START_POINT_S  22

/* APB_CTRL_RET_ICACHE_VLD_SIZE : R/W; bitpos: [20:13]; default: 255;
 * ******* Description ***********
 */

#define APB_CTRL_RET_ICACHE_VLD_SIZE    0x000000ff
#define APB_CTRL_RET_ICACHE_VLD_SIZE_M  (APB_CTRL_RET_ICACHE_VLD_SIZE_V << APB_CTRL_RET_ICACHE_VLD_SIZE_S)
#define APB_CTRL_RET_ICACHE_VLD_SIZE_V  0x000000ff
#define APB_CTRL_RET_ICACHE_VLD_SIZE_S  13

/* APB_CTRL_RET_ICACHE_SIZE : R/W; bitpos: [11:4]; default: 255;
 * ******* Description ***********
 */

#define APB_CTRL_RET_ICACHE_SIZE    0x000000ff
#define APB_CTRL_RET_ICACHE_SIZE_M  (APB_CTRL_RET_ICACHE_SIZE_V << APB_CTRL_RET_ICACHE_SIZE_S)
#define APB_CTRL_RET_ICACHE_SIZE_V  0x000000ff
#define APB_CTRL_RET_ICACHE_SIZE_S  4

/* APB_CTRL_RETENTION_CTRL3_REG register
 * ******* Description ***********
 */

#define APB_CTRL_RETENTION_CTRL3_REG (DR_REG_APB_CTRL_BASE + 0xc0)

/* APB_CTRL_RET_DCACHE_ENABLE : R/W; bitpos: [31]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_RET_DCACHE_ENABLE    (BIT(31))
#define APB_CTRL_RET_DCACHE_ENABLE_M  (APB_CTRL_RET_DCACHE_ENABLE_V << APB_CTRL_RET_DCACHE_ENABLE_S)
#define APB_CTRL_RET_DCACHE_ENABLE_V  0x00000001
#define APB_CTRL_RET_DCACHE_ENABLE_S  31

/* APB_CTRL_RET_DCACHE_START_POINT : R/W; bitpos: [30:22]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_RET_DCACHE_START_POINT    0x000001ff
#define APB_CTRL_RET_DCACHE_START_POINT_M  (APB_CTRL_RET_DCACHE_START_POINT_V << APB_CTRL_RET_DCACHE_START_POINT_S)
#define APB_CTRL_RET_DCACHE_START_POINT_V  0x000001ff
#define APB_CTRL_RET_DCACHE_START_POINT_S  22

/* APB_CTRL_RET_DCACHE_VLD_SIZE : R/W; bitpos: [21:13]; default: 511;
 * ******* Description ***********
 */

#define APB_CTRL_RET_DCACHE_VLD_SIZE    0x000001ff
#define APB_CTRL_RET_DCACHE_VLD_SIZE_M  (APB_CTRL_RET_DCACHE_VLD_SIZE_V << APB_CTRL_RET_DCACHE_VLD_SIZE_S)
#define APB_CTRL_RET_DCACHE_VLD_SIZE_V  0x000001ff
#define APB_CTRL_RET_DCACHE_VLD_SIZE_S  13

/* APB_CTRL_RET_DCACHE_SIZE : R/W; bitpos: [12:4]; default: 511;
 * ******* Description ***********
 */

#define APB_CTRL_RET_DCACHE_SIZE    0x000001ff
#define APB_CTRL_RET_DCACHE_SIZE_M  (APB_CTRL_RET_DCACHE_SIZE_V << APB_CTRL_RET_DCACHE_SIZE_S)
#define APB_CTRL_RET_DCACHE_SIZE_V  0x000001ff
#define APB_CTRL_RET_DCACHE_SIZE_S  4

/* APB_CTRL_RETENTION_CTRL4_REG register
 * ******* Description ***********
 */

#define APB_CTRL_RETENTION_CTRL4_REG (DR_REG_APB_CTRL_BASE + 0xc4)

/* APB_CTRL_RETENTION_INV_CFG : R/W; bitpos: [31:0]; default: 4294967295;
 * ******* Description ***********
 */

#define APB_CTRL_RETENTION_INV_CFG    0xffffffff
#define APB_CTRL_RETENTION_INV_CFG_M  (APB_CTRL_RETENTION_INV_CFG_V << APB_CTRL_RETENTION_INV_CFG_S)
#define APB_CTRL_RETENTION_INV_CFG_V  0xffffffff
#define APB_CTRL_RETENTION_INV_CFG_S  0

/* APB_CTRL_RETENTION_CTRL5_REG register
 * ******* Description ***********
 */

#define APB_CTRL_RETENTION_CTRL5_REG (DR_REG_APB_CTRL_BASE + 0xc8)

/* APB_CTRL_RETENTION_DISABLE : R/W; bitpos: [0]; default: 0;
 * ******* Description ***********
 */

#define APB_CTRL_RETENTION_DISABLE    (BIT(0))
#define APB_CTRL_RETENTION_DISABLE_M  (APB_CTRL_RETENTION_DISABLE_V << APB_CTRL_RETENTION_DISABLE_S)
#define APB_CTRL_RETENTION_DISABLE_V  0x00000001
#define APB_CTRL_RETENTION_DISABLE_S  0

/* APB_CTRL_DATE_REG register
 * ******* Description ***********
 */

#define APB_CTRL_DATE_REG (DR_REG_APB_CTRL_BASE + 0x3fc)

/* APB_CTRL_DATE : R/W; bitpos: [31:0]; default: 34607440;
 * Version control
 */

#define APB_CTRL_DATE    0xffffffff
#define APB_CTRL_DATE_M  (APB_CTRL_DATE_V << APB_CTRL_DATE_S)
#define APB_CTRL_DATE_V  0xffffffff
#define APB_CTRL_DATE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_APB_CTRL_H */
