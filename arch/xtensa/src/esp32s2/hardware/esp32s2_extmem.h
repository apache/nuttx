/****************************************************************************
 * arch/xtensa/src/esp32s2/hardware/esp32s2_extmem.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_EXTMEM_H
#define __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_EXTMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32s2_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EXTMEM_PRO_DCACHE_CTRL_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_CTRL_REG (DR_REG_EXTMEM_BASE + 0x0)

/* EXTMEM_PRO_DCACHE_LOCK_DONE : RO; bitpos: [25]; default: 0;
 * The bit is used to indicate lock operation is finished.
 */

#define EXTMEM_PRO_DCACHE_LOCK_DONE    (BIT(25))
#define EXTMEM_PRO_DCACHE_LOCK_DONE_M  (EXTMEM_PRO_DCACHE_LOCK_DONE_V << EXTMEM_PRO_DCACHE_LOCK_DONE_S)
#define EXTMEM_PRO_DCACHE_LOCK_DONE_V  0x00000001
#define EXTMEM_PRO_DCACHE_LOCK_DONE_S  25

/* EXTMEM_PRO_DCACHE_LOCK_ENA : R/W; bitpos: [24]; default: 0;
 * The bit is used to enable lock operation. It will be cleared by hardware
 * after lock operation done.
 */

#define EXTMEM_PRO_DCACHE_LOCK_ENA    (BIT(24))
#define EXTMEM_PRO_DCACHE_LOCK_ENA_M  (EXTMEM_PRO_DCACHE_LOCK_ENA_V << EXTMEM_PRO_DCACHE_LOCK_ENA_S)
#define EXTMEM_PRO_DCACHE_LOCK_ENA_V  0x00000001
#define EXTMEM_PRO_DCACHE_LOCK_ENA_S  24

/* EXTMEM_PRO_DCACHE_UNLOCK_DONE : RO; bitpos: [23]; default: 0;
 * The bit is used to indicate unlock operation is finished.
 */

#define EXTMEM_PRO_DCACHE_UNLOCK_DONE    (BIT(23))
#define EXTMEM_PRO_DCACHE_UNLOCK_DONE_M  (EXTMEM_PRO_DCACHE_UNLOCK_DONE_V << EXTMEM_PRO_DCACHE_UNLOCK_DONE_S)
#define EXTMEM_PRO_DCACHE_UNLOCK_DONE_V  0x00000001
#define EXTMEM_PRO_DCACHE_UNLOCK_DONE_S  23

/* EXTMEM_PRO_DCACHE_UNLOCK_ENA : R/W; bitpos: [22]; default: 0;
 * The bit is used to enable unlock operation. It will be cleared by
 * hardware after unlock operation done.
 */

#define EXTMEM_PRO_DCACHE_UNLOCK_ENA    (BIT(22))
#define EXTMEM_PRO_DCACHE_UNLOCK_ENA_M  (EXTMEM_PRO_DCACHE_UNLOCK_ENA_V << EXTMEM_PRO_DCACHE_UNLOCK_ENA_S)
#define EXTMEM_PRO_DCACHE_UNLOCK_ENA_V  0x00000001
#define EXTMEM_PRO_DCACHE_UNLOCK_ENA_S  22

/* EXTMEM_PRO_DCACHE_PRELOAD_DONE : RO; bitpos: [21]; default: 0;
 * The bit is used to indicate preload operation is finished.
 */

#define EXTMEM_PRO_DCACHE_PRELOAD_DONE    (BIT(21))
#define EXTMEM_PRO_DCACHE_PRELOAD_DONE_M  (EXTMEM_PRO_DCACHE_PRELOAD_DONE_V << EXTMEM_PRO_DCACHE_PRELOAD_DONE_S)
#define EXTMEM_PRO_DCACHE_PRELOAD_DONE_V  0x00000001
#define EXTMEM_PRO_DCACHE_PRELOAD_DONE_S  21

/* EXTMEM_PRO_DCACHE_PRELOAD_ENA : R/W; bitpos: [20]; default: 0;
 * The bit is used to enable preload operation. It will be cleared by
 * hardware after preload operation done.
 */

#define EXTMEM_PRO_DCACHE_PRELOAD_ENA    (BIT(20))
#define EXTMEM_PRO_DCACHE_PRELOAD_ENA_M  (EXTMEM_PRO_DCACHE_PRELOAD_ENA_V << EXTMEM_PRO_DCACHE_PRELOAD_ENA_S)
#define EXTMEM_PRO_DCACHE_PRELOAD_ENA_V  0x00000001
#define EXTMEM_PRO_DCACHE_PRELOAD_ENA_S  20

/* EXTMEM_PRO_DCACHE_AUTOLOAD_DONE : RO; bitpos: [19]; default: 0;
 * The bit is used to indicate conditional-preload operation is finished.
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_DONE    (BIT(19))
#define EXTMEM_PRO_DCACHE_AUTOLOAD_DONE_M  (EXTMEM_PRO_DCACHE_AUTOLOAD_DONE_V << EXTMEM_PRO_DCACHE_AUTOLOAD_DONE_S)
#define EXTMEM_PRO_DCACHE_AUTOLOAD_DONE_V  0x00000001
#define EXTMEM_PRO_DCACHE_AUTOLOAD_DONE_S  19

/* EXTMEM_PRO_DCACHE_AUTOLOAD_ENA : R/W; bitpos: [18]; default: 0;
 * The bit is used to enable and disable conditional-preload operation. It
 * is combined with pre_dcache_autoload_done. 1: enable, 0: disable.
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_ENA    (BIT(18))
#define EXTMEM_PRO_DCACHE_AUTOLOAD_ENA_M  (EXTMEM_PRO_DCACHE_AUTOLOAD_ENA_V << EXTMEM_PRO_DCACHE_AUTOLOAD_ENA_S)
#define EXTMEM_PRO_DCACHE_AUTOLOAD_ENA_V  0x00000001
#define EXTMEM_PRO_DCACHE_AUTOLOAD_ENA_S  18

/* EXTMEM_PRO_DCACHE_LOCK1_EN : R/W; bitpos: [15]; default: 0;
 * The bit is used to enable pre-lock operation which is combined with
 * PRO_DCACHE_LOCK1_ADDR_REG and PRO_DCACHE_LOCK1_SIZE_REG.
 */

#define EXTMEM_PRO_DCACHE_LOCK1_EN    (BIT(15))
#define EXTMEM_PRO_DCACHE_LOCK1_EN_M  (EXTMEM_PRO_DCACHE_LOCK1_EN_V << EXTMEM_PRO_DCACHE_LOCK1_EN_S)
#define EXTMEM_PRO_DCACHE_LOCK1_EN_V  0x00000001
#define EXTMEM_PRO_DCACHE_LOCK1_EN_S  15

/* EXTMEM_PRO_DCACHE_LOCK0_EN : R/W; bitpos: [14]; default: 0;
 * The bit is used to enable pre-lock operation which is combined with
 * PRO_DCACHE_LOCK0_ADDR_REG and PRO_DCACHE_LOCK0_SIZE_REG.
 */

#define EXTMEM_PRO_DCACHE_LOCK0_EN    (BIT(14))
#define EXTMEM_PRO_DCACHE_LOCK0_EN_M  (EXTMEM_PRO_DCACHE_LOCK0_EN_V << EXTMEM_PRO_DCACHE_LOCK0_EN_S)
#define EXTMEM_PRO_DCACHE_LOCK0_EN_V  0x00000001
#define EXTMEM_PRO_DCACHE_LOCK0_EN_S  14

/* EXTMEM_PRO_DCACHE_CLEAN_DONE : RO; bitpos: [13]; default: 0;
 * The bit is used to indicate clean operation is finished.
 */

#define EXTMEM_PRO_DCACHE_CLEAN_DONE    (BIT(13))
#define EXTMEM_PRO_DCACHE_CLEAN_DONE_M  (EXTMEM_PRO_DCACHE_CLEAN_DONE_V << EXTMEM_PRO_DCACHE_CLEAN_DONE_S)
#define EXTMEM_PRO_DCACHE_CLEAN_DONE_V  0x00000001
#define EXTMEM_PRO_DCACHE_CLEAN_DONE_S  13

/* EXTMEM_PRO_DCACHE_CLEAN_ENA : R/W; bitpos: [12]; default: 0;
 * The bit is used to enable clean operation. It will be cleared by hardware
 * after clean operation done.
 */

#define EXTMEM_PRO_DCACHE_CLEAN_ENA    (BIT(12))
#define EXTMEM_PRO_DCACHE_CLEAN_ENA_M  (EXTMEM_PRO_DCACHE_CLEAN_ENA_V << EXTMEM_PRO_DCACHE_CLEAN_ENA_S)
#define EXTMEM_PRO_DCACHE_CLEAN_ENA_V  0x00000001
#define EXTMEM_PRO_DCACHE_CLEAN_ENA_S  12

/* EXTMEM_PRO_DCACHE_FLUSH_DONE : RO; bitpos: [11]; default: 0;
 * The bit is used to indicate flush operation is finished.
 */

#define EXTMEM_PRO_DCACHE_FLUSH_DONE    (BIT(11))
#define EXTMEM_PRO_DCACHE_FLUSH_DONE_M  (EXTMEM_PRO_DCACHE_FLUSH_DONE_V << EXTMEM_PRO_DCACHE_FLUSH_DONE_S)
#define EXTMEM_PRO_DCACHE_FLUSH_DONE_V  0x00000001
#define EXTMEM_PRO_DCACHE_FLUSH_DONE_S  11

/* EXTMEM_PRO_DCACHE_FLUSH_ENA : R/W; bitpos: [10]; default: 0;
 * The bit is used to enable flush operation. It will be cleared by hardware
 * after flush operation done.
 */

#define EXTMEM_PRO_DCACHE_FLUSH_ENA    (BIT(10))
#define EXTMEM_PRO_DCACHE_FLUSH_ENA_M  (EXTMEM_PRO_DCACHE_FLUSH_ENA_V << EXTMEM_PRO_DCACHE_FLUSH_ENA_S)
#define EXTMEM_PRO_DCACHE_FLUSH_ENA_V  0x00000001
#define EXTMEM_PRO_DCACHE_FLUSH_ENA_S  10

/* EXTMEM_PRO_DCACHE_INVALIDATE_DONE : RO; bitpos: [9]; default: 0;
 * The bit is used to indicate invalidate operation is finished.
 */

#define EXTMEM_PRO_DCACHE_INVALIDATE_DONE    (BIT(9))
#define EXTMEM_PRO_DCACHE_INVALIDATE_DONE_M  (EXTMEM_PRO_DCACHE_INVALIDATE_DONE_V << EXTMEM_PRO_DCACHE_INVALIDATE_DONE_S)
#define EXTMEM_PRO_DCACHE_INVALIDATE_DONE_V  0x00000001
#define EXTMEM_PRO_DCACHE_INVALIDATE_DONE_S  9

/* EXTMEM_PRO_DCACHE_INVALIDATE_ENA : R/W; bitpos: [8]; default: 1;
 * The bit is used to enable invalidate operation. It will be cleared by
 * hardware after invalidate operation done.
 */

#define EXTMEM_PRO_DCACHE_INVALIDATE_ENA    (BIT(8))
#define EXTMEM_PRO_DCACHE_INVALIDATE_ENA_M  (EXTMEM_PRO_DCACHE_INVALIDATE_ENA_V << EXTMEM_PRO_DCACHE_INVALIDATE_ENA_S)
#define EXTMEM_PRO_DCACHE_INVALIDATE_ENA_V  0x00000001
#define EXTMEM_PRO_DCACHE_INVALIDATE_ENA_S  8

/* EXTMEM_PRO_DCACHE_BLOCKSIZE_MODE : R/W; bitpos: [3]; default: 0;
 * The bit is used to configure cache block size.0: 16 bytes, 1: 32 bytes
 */

#define EXTMEM_PRO_DCACHE_BLOCKSIZE_MODE    (BIT(3))
#define EXTMEM_PRO_DCACHE_BLOCKSIZE_MODE_M  (EXTMEM_PRO_DCACHE_BLOCKSIZE_MODE_V << EXTMEM_PRO_DCACHE_BLOCKSIZE_MODE_S)
#define EXTMEM_PRO_DCACHE_BLOCKSIZE_MODE_V  0x00000001
#define EXTMEM_PRO_DCACHE_BLOCKSIZE_MODE_S  3

/* EXTMEM_PRO_DCACHE_SETSIZE_MODE : R/W; bitpos: [2]; default: 0;
 * The bit is used to configure cache memory size.0: 8KB, 1: 16KB
 */

#define EXTMEM_PRO_DCACHE_SETSIZE_MODE    (BIT(2))
#define EXTMEM_PRO_DCACHE_SETSIZE_MODE_M  (EXTMEM_PRO_DCACHE_SETSIZE_MODE_V << EXTMEM_PRO_DCACHE_SETSIZE_MODE_S)
#define EXTMEM_PRO_DCACHE_SETSIZE_MODE_V  0x00000001
#define EXTMEM_PRO_DCACHE_SETSIZE_MODE_S  2

/* EXTMEM_PRO_DCACHE_ENABLE : R/W; bitpos: [0]; default: 0;
 * The bit is used to activate the data cache. 0: disable, 1: enable
 */

#define EXTMEM_PRO_DCACHE_ENABLE    (BIT(0))
#define EXTMEM_PRO_DCACHE_ENABLE_M  (EXTMEM_PRO_DCACHE_ENABLE_V << EXTMEM_PRO_DCACHE_ENABLE_S)
#define EXTMEM_PRO_DCACHE_ENABLE_V  0x00000001
#define EXTMEM_PRO_DCACHE_ENABLE_S  0

/* EXTMEM_PRO_DCACHE_CTRL1_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_CTRL1_REG (DR_REG_EXTMEM_BASE + 0x4)

/* EXTMEM_PRO_DCACHE_MASK_BUS2 : R/W; bitpos: [2]; default: 1;
 * The bit is used to disable dbus2, 0: enable, 1: disable
 */

#define EXTMEM_PRO_DCACHE_MASK_BUS2    (BIT(2))
#define EXTMEM_PRO_DCACHE_MASK_BUS2_M  (EXTMEM_PRO_DCACHE_MASK_BUS2_V << EXTMEM_PRO_DCACHE_MASK_BUS2_S)
#define EXTMEM_PRO_DCACHE_MASK_BUS2_V  0x00000001
#define EXTMEM_PRO_DCACHE_MASK_BUS2_S  2

/* EXTMEM_PRO_DCACHE_MASK_BUS1 : R/W; bitpos: [1]; default: 1;
 * The bit is used to disable dbus1, 0: enable, 1: disable
 */

#define EXTMEM_PRO_DCACHE_MASK_BUS1    (BIT(1))
#define EXTMEM_PRO_DCACHE_MASK_BUS1_M  (EXTMEM_PRO_DCACHE_MASK_BUS1_V << EXTMEM_PRO_DCACHE_MASK_BUS1_S)
#define EXTMEM_PRO_DCACHE_MASK_BUS1_V  0x00000001
#define EXTMEM_PRO_DCACHE_MASK_BUS1_S  1

/* EXTMEM_PRO_DCACHE_MASK_BUS0 : R/W; bitpos: [0]; default: 1;
 * The bit is used to disable dbus0, 0: enable, 1: disable
 */

#define EXTMEM_PRO_DCACHE_MASK_BUS0    (BIT(0))
#define EXTMEM_PRO_DCACHE_MASK_BUS0_M  (EXTMEM_PRO_DCACHE_MASK_BUS0_V << EXTMEM_PRO_DCACHE_MASK_BUS0_S)
#define EXTMEM_PRO_DCACHE_MASK_BUS0_V  0x00000001
#define EXTMEM_PRO_DCACHE_MASK_BUS0_S  0

/* EXTMEM_PRO_DCACHE_TAG_POWER_CTRL_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_TAG_POWER_CTRL_REG (DR_REG_EXTMEM_BASE + 0x8)

/* EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PU : R/W; bitpos: [2]; default: 1;
 * The bit is used to power dcache tag memory down, 0: follow  rtc_lslp_pd,
 * 1: power up
 */

#define EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PU    (BIT(2))
#define EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PU_M  (EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PU_V << EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PU_S)
#define EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PU_V  0x00000001
#define EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PU_S  2

/* EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PD : R/W; bitpos: [1]; default: 0;
 * The bit is used to power dcache tag memory down, 0: follow  rtc_lslp_pd,
 * 1: power down
 */

#define EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PD    (BIT(1))
#define EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PD_M  (EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PD_V << EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PD_S)
#define EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PD_V  0x00000001
#define EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_PD_S  1

/* EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_ON : R/W; bitpos: [0]; default: 1;
 * The bit is used to close clock gating of dcache tag memory. 1: close
 * gating, 0: open clock gating.
 */

#define EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_ON    (BIT(0))
#define EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_ON_M  (EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_ON_V << EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_ON_S)
#define EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_ON_V  0x00000001
#define EXTMEM_PRO_DCACHE_TAG_MEM_FORCE_ON_S  0

/* EXTMEM_PRO_DCACHE_LOCK0_ADDR_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_LOCK0_ADDR_REG (DR_REG_EXTMEM_BASE + 0xc)

/* EXTMEM_PRO_DCACHE_LOCK0_ADDR : R/W; bitpos: [31:0]; default: 0;
 * The bits are used to configure the first start virtual address of data
 * locking, which is combined with PRO_DCACHE_LOCK0_SIZE_REG
 */

#define EXTMEM_PRO_DCACHE_LOCK0_ADDR    0xffffffff
#define EXTMEM_PRO_DCACHE_LOCK0_ADDR_M  (EXTMEM_PRO_DCACHE_LOCK0_ADDR_V << EXTMEM_PRO_DCACHE_LOCK0_ADDR_S)
#define EXTMEM_PRO_DCACHE_LOCK0_ADDR_V  0xffffffff
#define EXTMEM_PRO_DCACHE_LOCK0_ADDR_S  0

/* EXTMEM_PRO_DCACHE_LOCK0_SIZE_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_LOCK0_SIZE_REG (DR_REG_EXTMEM_BASE + 0x10)

/* EXTMEM_PRO_DCACHE_LOCK0_SIZE : R/W; bitpos: [15:0]; default: 0;
 * The bits are used to configure the first length of data locking, which is
 * combined with PRO_DCACHE_LOCK0_ADDR_REG
 */

#define EXTMEM_PRO_DCACHE_LOCK0_SIZE    0x0000ffff
#define EXTMEM_PRO_DCACHE_LOCK0_SIZE_M  (EXTMEM_PRO_DCACHE_LOCK0_SIZE_V << EXTMEM_PRO_DCACHE_LOCK0_SIZE_S)
#define EXTMEM_PRO_DCACHE_LOCK0_SIZE_V  0x0000ffff
#define EXTMEM_PRO_DCACHE_LOCK0_SIZE_S  0

/* EXTMEM_PRO_DCACHE_LOCK1_ADDR_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_LOCK1_ADDR_REG (DR_REG_EXTMEM_BASE + 0x14)

/* EXTMEM_PRO_DCACHE_LOCK1_ADDR : R/W; bitpos: [31:0]; default: 0;
 * The bits are used to configure the second start virtual address of data
 * locking, which is combined with PRO_DCACHE_LOCK1_SIZE_REG
 */

#define EXTMEM_PRO_DCACHE_LOCK1_ADDR    0xffffffff
#define EXTMEM_PRO_DCACHE_LOCK1_ADDR_M  (EXTMEM_PRO_DCACHE_LOCK1_ADDR_V << EXTMEM_PRO_DCACHE_LOCK1_ADDR_S)
#define EXTMEM_PRO_DCACHE_LOCK1_ADDR_V  0xffffffff
#define EXTMEM_PRO_DCACHE_LOCK1_ADDR_S  0

/* EXTMEM_PRO_DCACHE_LOCK1_SIZE_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_LOCK1_SIZE_REG (DR_REG_EXTMEM_BASE + 0x18)

/* EXTMEM_PRO_DCACHE_LOCK1_SIZE : R/W; bitpos: [15:0]; default: 0;
 * The bits are used to configure the second length of data locking, which
 * is combined with PRO_DCACHE_LOCK1_ADDR_REG
 */

#define EXTMEM_PRO_DCACHE_LOCK1_SIZE    0x0000ffff
#define EXTMEM_PRO_DCACHE_LOCK1_SIZE_M  (EXTMEM_PRO_DCACHE_LOCK1_SIZE_V << EXTMEM_PRO_DCACHE_LOCK1_SIZE_S)
#define EXTMEM_PRO_DCACHE_LOCK1_SIZE_V  0x0000ffff
#define EXTMEM_PRO_DCACHE_LOCK1_SIZE_S  0

/* EXTMEM_PRO_DCACHE_MEM_SYNC0_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_MEM_SYNC0_REG (DR_REG_EXTMEM_BASE + 0x1c)

/* EXTMEM_PRO_DCACHE_MEMSYNC_ADDR : R/W; bitpos: [31:0]; default: 0;
 * The bits are used to configure the start virtual address for invalidate,
 * flush, clean, lock and unlock operations. The manual operations will be
 * issued if the address is validate. The auto operations will be issued if
 * the address is invalidate. It should be combined with
 * PRO_DCACHE_MEM_SYNC1.
 */

#define EXTMEM_PRO_DCACHE_MEMSYNC_ADDR    0xffffffff
#define EXTMEM_PRO_DCACHE_MEMSYNC_ADDR_M  (EXTMEM_PRO_DCACHE_MEMSYNC_ADDR_V << EXTMEM_PRO_DCACHE_MEMSYNC_ADDR_S)
#define EXTMEM_PRO_DCACHE_MEMSYNC_ADDR_V  0xffffffff
#define EXTMEM_PRO_DCACHE_MEMSYNC_ADDR_S  0

/* EXTMEM_PRO_DCACHE_MEM_SYNC1_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_MEM_SYNC1_REG (DR_REG_EXTMEM_BASE + 0x20)

/* EXTMEM_PRO_DCACHE_MEMSYNC_SIZE : R/W; bitpos: [18:0]; default: 0;
 * The bits are used to configure the length for invalidate, flush, clean,
 * lock and unlock operations. The manual operations will be issued if it is
 * validate. The auto operations will be issued if it is invalidate. It
 * should be combined with PRO_DCACHE_MEM_SYNC0.
 */

#define EXTMEM_PRO_DCACHE_MEMSYNC_SIZE    0x0007ffff
#define EXTMEM_PRO_DCACHE_MEMSYNC_SIZE_M  (EXTMEM_PRO_DCACHE_MEMSYNC_SIZE_V << EXTMEM_PRO_DCACHE_MEMSYNC_SIZE_S)
#define EXTMEM_PRO_DCACHE_MEMSYNC_SIZE_V  0x0007ffff
#define EXTMEM_PRO_DCACHE_MEMSYNC_SIZE_S  0

/* EXTMEM_PRO_DCACHE_PRELOAD_ADDR_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_PRELOAD_ADDR_REG (DR_REG_EXTMEM_BASE + 0x24)

/* EXTMEM_PRO_DCACHE_PRELOAD_ADDR : R/W; bitpos: [31:0]; default: 0;
 * The bits are used to configure the start virtual address for manual
 * pre-load operation. It should be combined with
 * PRO_DCACHE_PRELOAD_SIZE_REG.
 */

#define EXTMEM_PRO_DCACHE_PRELOAD_ADDR    0xffffffff
#define EXTMEM_PRO_DCACHE_PRELOAD_ADDR_M  (EXTMEM_PRO_DCACHE_PRELOAD_ADDR_V << EXTMEM_PRO_DCACHE_PRELOAD_ADDR_S)
#define EXTMEM_PRO_DCACHE_PRELOAD_ADDR_V  0xffffffff
#define EXTMEM_PRO_DCACHE_PRELOAD_ADDR_S  0

/* EXTMEM_PRO_DCACHE_PRELOAD_SIZE_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_PRELOAD_SIZE_REG (DR_REG_EXTMEM_BASE + 0x28)

/* EXTMEM_PRO_DCACHE_PRELOAD_ORDER : R/W; bitpos: [10]; default: 0;
 * The bits are used to configure the direction of manual pre-load
 * operation. 1: descending, 0: ascending.
 */

#define EXTMEM_PRO_DCACHE_PRELOAD_ORDER    (BIT(10))
#define EXTMEM_PRO_DCACHE_PRELOAD_ORDER_M  (EXTMEM_PRO_DCACHE_PRELOAD_ORDER_V << EXTMEM_PRO_DCACHE_PRELOAD_ORDER_S)
#define EXTMEM_PRO_DCACHE_PRELOAD_ORDER_V  0x00000001
#define EXTMEM_PRO_DCACHE_PRELOAD_ORDER_S  10

/* EXTMEM_PRO_DCACHE_PRELOAD_SIZE : R/W; bitpos: [9:0]; default: 512;
 * The bits are used to configure the length for manual pre-load operation.
 * It should be combined with PRO_DCACHE_PRELOAD_ADDR_REG..
 */

#define EXTMEM_PRO_DCACHE_PRELOAD_SIZE    0x000003ff
#define EXTMEM_PRO_DCACHE_PRELOAD_SIZE_M  (EXTMEM_PRO_DCACHE_PRELOAD_SIZE_V << EXTMEM_PRO_DCACHE_PRELOAD_SIZE_S)
#define EXTMEM_PRO_DCACHE_PRELOAD_SIZE_V  0x000003ff
#define EXTMEM_PRO_DCACHE_PRELOAD_SIZE_S  0

/* EXTMEM_PRO_DCACHE_AUTOLOAD_CFG_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_CFG_REG (DR_REG_EXTMEM_BASE + 0x2c)

/* EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ENA : R/W; bitpos: [9]; default: 0;
 * The bits are used to enable the first section for conditional pre-load
 * operation.
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ENA    (BIT(9))
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ENA_M  (EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ENA_V << EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ENA_S)
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ENA_V  0x00000001
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ENA_S  9

/* EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ENA : R/W; bitpos: [8]; default: 0;
 * The bits are used to enable the second section for conditional pre-load
 * operation.
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ENA    (BIT(8))
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ENA_M  (EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ENA_V << EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ENA_S)
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ENA_V  0x00000001
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ENA_S  8

/* EXTMEM_PRO_DCACHE_AUTOLOAD_SIZE : R/W; bitpos: [7:6]; default: 0;
 * The bits are used to configure the numbers of the cache block for the
 * issuing conditional pre-load operation.
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_SIZE    0x00000003
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SIZE_M  (EXTMEM_PRO_DCACHE_AUTOLOAD_SIZE_V << EXTMEM_PRO_DCACHE_AUTOLOAD_SIZE_S)
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SIZE_V  0x00000003
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SIZE_S  6

/* EXTMEM_PRO_DCACHE_AUTOLOAD_RQST : R/W; bitpos: [5:4]; default: 0;
 * The bits are used to configure trigger conditions for conditional
 * pre-load. 0/3: cache miss, 1: cache hit, 2: both cache miss and hit.
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_RQST    0x00000003
#define EXTMEM_PRO_DCACHE_AUTOLOAD_RQST_M  (EXTMEM_PRO_DCACHE_AUTOLOAD_RQST_V << EXTMEM_PRO_DCACHE_AUTOLOAD_RQST_S)
#define EXTMEM_PRO_DCACHE_AUTOLOAD_RQST_V  0x00000003
#define EXTMEM_PRO_DCACHE_AUTOLOAD_RQST_S  4

/* EXTMEM_PRO_DCACHE_AUTOLOAD_ORDER : R/W; bitpos: [3]; default: 0;
 * The bits are used to configure the direction of conditional pre-load
 * operation. 1: descending, 0: ascending.
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_ORDER    (BIT(3))
#define EXTMEM_PRO_DCACHE_AUTOLOAD_ORDER_M  (EXTMEM_PRO_DCACHE_AUTOLOAD_ORDER_V << EXTMEM_PRO_DCACHE_AUTOLOAD_ORDER_S)
#define EXTMEM_PRO_DCACHE_AUTOLOAD_ORDER_V  0x00000001
#define EXTMEM_PRO_DCACHE_AUTOLOAD_ORDER_S  3

/* EXTMEM_PRO_DCACHE_AUTOLOAD_STEP : R/W; bitpos: [2:1]; default: 0;
 * Reserved.
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_STEP    0x00000003
#define EXTMEM_PRO_DCACHE_AUTOLOAD_STEP_M  (EXTMEM_PRO_DCACHE_AUTOLOAD_STEP_V << EXTMEM_PRO_DCACHE_AUTOLOAD_STEP_S)
#define EXTMEM_PRO_DCACHE_AUTOLOAD_STEP_V  0x00000003
#define EXTMEM_PRO_DCACHE_AUTOLOAD_STEP_S  1

/* EXTMEM_PRO_DCACHE_AUTOLOAD_MODE : R/W; bitpos: [0]; default: 0;
 * Reserved.
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_MODE    (BIT(0))
#define EXTMEM_PRO_DCACHE_AUTOLOAD_MODE_M  (EXTMEM_PRO_DCACHE_AUTOLOAD_MODE_V << EXTMEM_PRO_DCACHE_AUTOLOAD_MODE_S)
#define EXTMEM_PRO_DCACHE_AUTOLOAD_MODE_V  0x00000001
#define EXTMEM_PRO_DCACHE_AUTOLOAD_MODE_S  0

/* EXTMEM_PRO_DCACHE_AUTOLOAD_SECTION0_ADDR_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_SECTION0_ADDR_REG (DR_REG_EXTMEM_BASE + 0x30)

/* EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ADDR : R/W; bitpos: [31:0]; default: 0;
 * The bits are used to configure the start virtual address of the first
 * section for conditional pre-load operation. It should be combined with
 * pro_dcache_autoload_sct0_ena.
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ADDR    0xffffffff
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ADDR_M  (EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ADDR_V << EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ADDR_S)
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ADDR_V  0xffffffff
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_ADDR_S  0

/* EXTMEM_PRO_DCACHE_AUTOLOAD_SECTION0_SIZE_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_SECTION0_SIZE_REG (DR_REG_EXTMEM_BASE + 0x34)

/* EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_SIZE : R/W; bitpos: [23:0]; default:
 * 32768;
 * The bits are used to configure the length of the first section for
 * conditional pre-load operation. It should be combined with
 * pro_dcache_autoload_sct0_ena.
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_SIZE    0x00ffffff
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_SIZE_M  (EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_SIZE_V << EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_SIZE_S)
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_SIZE_V  0x00ffffff
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT0_SIZE_S  0

/* EXTMEM_PRO_DCACHE_AUTOLOAD_SECTION1_ADDR_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_SECTION1_ADDR_REG (DR_REG_EXTMEM_BASE + 0x38)

/* EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ADDR : R/W; bitpos: [31:0]; default: 0;
 * The bits are used to configure the start virtual address of the second
 * section for conditional pre-load operation. It should be combined with
 * pro_dcache_autoload_sct1_ena.
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ADDR    0xffffffff
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ADDR_M  (EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ADDR_V << EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ADDR_S)
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ADDR_V  0xffffffff
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_ADDR_S  0

/* EXTMEM_PRO_DCACHE_AUTOLOAD_SECTION1_SIZE_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_SECTION1_SIZE_REG (DR_REG_EXTMEM_BASE + 0x3c)

/* EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_SIZE : R/W; bitpos: [23:0]; default:
 * 32768;
 * The bits are used to configure the length of the second section for
 * conditional pre-load operation. It should be combined with
 * pro_dcache_autoload_sct1_ena.
 */

#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_SIZE    0x00ffffff
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_SIZE_M  (EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_SIZE_V << EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_SIZE_S)
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_SIZE_V  0x00ffffff
#define EXTMEM_PRO_DCACHE_AUTOLOAD_SCT1_SIZE_S  0

/* EXTMEM_PRO_ICACHE_CTRL_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_CTRL_REG (DR_REG_EXTMEM_BASE + 0x40)

/* EXTMEM_PRO_ICACHE_LOCK_DONE : RO; bitpos: [25]; default: 0;
 * The bit is used to indicate lock operation is finished.
 */

#define EXTMEM_PRO_ICACHE_LOCK_DONE    (BIT(25))
#define EXTMEM_PRO_ICACHE_LOCK_DONE_M  (EXTMEM_PRO_ICACHE_LOCK_DONE_V << EXTMEM_PRO_ICACHE_LOCK_DONE_S)
#define EXTMEM_PRO_ICACHE_LOCK_DONE_V  0x00000001
#define EXTMEM_PRO_ICACHE_LOCK_DONE_S  25

/* EXTMEM_PRO_ICACHE_LOCK_ENA : R/W; bitpos: [24]; default: 0;
 * The bit is used to enable lock operation. It will be cleared by hardware
 * after lock operation done.
 */

#define EXTMEM_PRO_ICACHE_LOCK_ENA    (BIT(24))
#define EXTMEM_PRO_ICACHE_LOCK_ENA_M  (EXTMEM_PRO_ICACHE_LOCK_ENA_V << EXTMEM_PRO_ICACHE_LOCK_ENA_S)
#define EXTMEM_PRO_ICACHE_LOCK_ENA_V  0x00000001
#define EXTMEM_PRO_ICACHE_LOCK_ENA_S  24

/* EXTMEM_PRO_ICACHE_UNLOCK_DONE : RO; bitpos: [23]; default: 0;
 * The bit is used to indicate unlock operation is finished.
 */

#define EXTMEM_PRO_ICACHE_UNLOCK_DONE    (BIT(23))
#define EXTMEM_PRO_ICACHE_UNLOCK_DONE_M  (EXTMEM_PRO_ICACHE_UNLOCK_DONE_V << EXTMEM_PRO_ICACHE_UNLOCK_DONE_S)
#define EXTMEM_PRO_ICACHE_UNLOCK_DONE_V  0x00000001
#define EXTMEM_PRO_ICACHE_UNLOCK_DONE_S  23

/* EXTMEM_PRO_ICACHE_UNLOCK_ENA : R/W; bitpos: [22]; default: 0;
 * The bit is used to enable unlock operation. It will be cleared by
 * hardware after unlock operation done.
 */

#define EXTMEM_PRO_ICACHE_UNLOCK_ENA    (BIT(22))
#define EXTMEM_PRO_ICACHE_UNLOCK_ENA_M  (EXTMEM_PRO_ICACHE_UNLOCK_ENA_V << EXTMEM_PRO_ICACHE_UNLOCK_ENA_S)
#define EXTMEM_PRO_ICACHE_UNLOCK_ENA_V  0x00000001
#define EXTMEM_PRO_ICACHE_UNLOCK_ENA_S  22

/* EXTMEM_PRO_ICACHE_PRELOAD_DONE : RO; bitpos: [21]; default: 0;
 * The bit is used to indicate preload operation is finished.
 */

#define EXTMEM_PRO_ICACHE_PRELOAD_DONE    (BIT(21))
#define EXTMEM_PRO_ICACHE_PRELOAD_DONE_M  (EXTMEM_PRO_ICACHE_PRELOAD_DONE_V << EXTMEM_PRO_ICACHE_PRELOAD_DONE_S)
#define EXTMEM_PRO_ICACHE_PRELOAD_DONE_V  0x00000001
#define EXTMEM_PRO_ICACHE_PRELOAD_DONE_S  21

/* EXTMEM_PRO_ICACHE_PRELOAD_ENA : R/W; bitpos: [20]; default: 0;
 * The bit is used to enable preload operation. It will be cleared by
 * hardware after preload operation done.
 */

#define EXTMEM_PRO_ICACHE_PRELOAD_ENA    (BIT(20))
#define EXTMEM_PRO_ICACHE_PRELOAD_ENA_M  (EXTMEM_PRO_ICACHE_PRELOAD_ENA_V << EXTMEM_PRO_ICACHE_PRELOAD_ENA_S)
#define EXTMEM_PRO_ICACHE_PRELOAD_ENA_V  0x00000001
#define EXTMEM_PRO_ICACHE_PRELOAD_ENA_S  20

/* EXTMEM_PRO_ICACHE_AUTOLOAD_DONE : RO; bitpos: [19]; default: 0;
 * The bit is used to indicate conditional-preload operation is finished.
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_DONE    (BIT(19))
#define EXTMEM_PRO_ICACHE_AUTOLOAD_DONE_M  (EXTMEM_PRO_ICACHE_AUTOLOAD_DONE_V << EXTMEM_PRO_ICACHE_AUTOLOAD_DONE_S)
#define EXTMEM_PRO_ICACHE_AUTOLOAD_DONE_V  0x00000001
#define EXTMEM_PRO_ICACHE_AUTOLOAD_DONE_S  19

/* EXTMEM_PRO_ICACHE_AUTOLOAD_ENA : R/W; bitpos: [18]; default: 0;
 * The bit is used to enable and disable conditional-preload operation. It
 * is combined with pre_dcache_autoload_done. 1: enable, 0: disable.
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_ENA    (BIT(18))
#define EXTMEM_PRO_ICACHE_AUTOLOAD_ENA_M  (EXTMEM_PRO_ICACHE_AUTOLOAD_ENA_V << EXTMEM_PRO_ICACHE_AUTOLOAD_ENA_S)
#define EXTMEM_PRO_ICACHE_AUTOLOAD_ENA_V  0x00000001
#define EXTMEM_PRO_ICACHE_AUTOLOAD_ENA_S  18

/* EXTMEM_PRO_ICACHE_LOCK1_EN : R/W; bitpos: [15]; default: 0;
 * The bit is used to enable pre-lock operation which is combined with
 * PRO_ICACHE_LOCK1_ADDR_REG and PRO_ICACHE_LOCK1_SIZE_REG.
 */

#define EXTMEM_PRO_ICACHE_LOCK1_EN    (BIT(15))
#define EXTMEM_PRO_ICACHE_LOCK1_EN_M  (EXTMEM_PRO_ICACHE_LOCK1_EN_V << EXTMEM_PRO_ICACHE_LOCK1_EN_S)
#define EXTMEM_PRO_ICACHE_LOCK1_EN_V  0x00000001
#define EXTMEM_PRO_ICACHE_LOCK1_EN_S  15

/* EXTMEM_PRO_ICACHE_LOCK0_EN : R/W; bitpos: [14]; default: 0;
 * The bit is used to enable pre-lock operation which is combined with
 * PRO_ICACHE_LOCK0_ADDR_REG and PRO_ICACHE_LOCK0_SIZE_REG.
 */

#define EXTMEM_PRO_ICACHE_LOCK0_EN    (BIT(14))
#define EXTMEM_PRO_ICACHE_LOCK0_EN_M  (EXTMEM_PRO_ICACHE_LOCK0_EN_V << EXTMEM_PRO_ICACHE_LOCK0_EN_S)
#define EXTMEM_PRO_ICACHE_LOCK0_EN_V  0x00000001
#define EXTMEM_PRO_ICACHE_LOCK0_EN_S  14

/* EXTMEM_PRO_ICACHE_INVALIDATE_DONE : RO; bitpos: [9]; default: 0;
 * The bit is used to indicate invalidate operation is finished.
 */

#define EXTMEM_PRO_ICACHE_INVALIDATE_DONE    (BIT(9))
#define EXTMEM_PRO_ICACHE_INVALIDATE_DONE_M  (EXTMEM_PRO_ICACHE_INVALIDATE_DONE_V << EXTMEM_PRO_ICACHE_INVALIDATE_DONE_S)
#define EXTMEM_PRO_ICACHE_INVALIDATE_DONE_V  0x00000001
#define EXTMEM_PRO_ICACHE_INVALIDATE_DONE_S  9

/* EXTMEM_PRO_ICACHE_INVALIDATE_ENA : R/W; bitpos: [8]; default: 1;
 * The bit is used to enable invalidate operation. It will be cleared by
 * hardware after invalidate operation done.
 */

#define EXTMEM_PRO_ICACHE_INVALIDATE_ENA    (BIT(8))
#define EXTMEM_PRO_ICACHE_INVALIDATE_ENA_M  (EXTMEM_PRO_ICACHE_INVALIDATE_ENA_V << EXTMEM_PRO_ICACHE_INVALIDATE_ENA_S)
#define EXTMEM_PRO_ICACHE_INVALIDATE_ENA_V  0x00000001
#define EXTMEM_PRO_ICACHE_INVALIDATE_ENA_S  8

/* EXTMEM_PRO_ICACHE_BLOCKSIZE_MODE : R/W; bitpos: [3]; default: 0;
 * The bit is used to configure cache block size.0: 16 bytes, 1: 32 bytes
 */

#define EXTMEM_PRO_ICACHE_BLOCKSIZE_MODE    (BIT(3))
#define EXTMEM_PRO_ICACHE_BLOCKSIZE_MODE_M  (EXTMEM_PRO_ICACHE_BLOCKSIZE_MODE_V << EXTMEM_PRO_ICACHE_BLOCKSIZE_MODE_S)
#define EXTMEM_PRO_ICACHE_BLOCKSIZE_MODE_V  0x00000001
#define EXTMEM_PRO_ICACHE_BLOCKSIZE_MODE_S  3

/* EXTMEM_PRO_ICACHE_SETSIZE_MODE : R/W; bitpos: [2]; default: 0;
 * The bit is used to configure cache memory size.0: 8KB, 1: 16KB
 */

#define EXTMEM_PRO_ICACHE_SETSIZE_MODE    (BIT(2))
#define EXTMEM_PRO_ICACHE_SETSIZE_MODE_M  (EXTMEM_PRO_ICACHE_SETSIZE_MODE_V << EXTMEM_PRO_ICACHE_SETSIZE_MODE_S)
#define EXTMEM_PRO_ICACHE_SETSIZE_MODE_V  0x00000001
#define EXTMEM_PRO_ICACHE_SETSIZE_MODE_S  2

/* EXTMEM_PRO_ICACHE_ENABLE : R/W; bitpos: [0]; default: 0;
 * The bit is used to activate the data cache. 0: disable, 1: enable
 */

#define EXTMEM_PRO_ICACHE_ENABLE    (BIT(0))
#define EXTMEM_PRO_ICACHE_ENABLE_M  (EXTMEM_PRO_ICACHE_ENABLE_V << EXTMEM_PRO_ICACHE_ENABLE_S)
#define EXTMEM_PRO_ICACHE_ENABLE_V  0x00000001
#define EXTMEM_PRO_ICACHE_ENABLE_S  0

/* EXTMEM_PRO_ICACHE_CTRL1_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_CTRL1_REG (DR_REG_EXTMEM_BASE + 0x44)

/* EXTMEM_PRO_ICACHE_MASK_BUS2 : R/W; bitpos: [2]; default: 1;
 * The bit is used to disable ibus2, 0: enable, 1: disable
 */

#define EXTMEM_PRO_ICACHE_MASK_BUS2    (BIT(2))
#define EXTMEM_PRO_ICACHE_MASK_BUS2_M  (EXTMEM_PRO_ICACHE_MASK_BUS2_V << EXTMEM_PRO_ICACHE_MASK_BUS2_S)
#define EXTMEM_PRO_ICACHE_MASK_BUS2_V  0x00000001
#define EXTMEM_PRO_ICACHE_MASK_BUS2_S  2

/* EXTMEM_PRO_ICACHE_MASK_BUS1 : R/W; bitpos: [1]; default: 1;
 * The bit is used to disable ibus1, 0: enable, 1: disable
 */

#define EXTMEM_PRO_ICACHE_MASK_BUS1    (BIT(1))
#define EXTMEM_PRO_ICACHE_MASK_BUS1_M  (EXTMEM_PRO_ICACHE_MASK_BUS1_V << EXTMEM_PRO_ICACHE_MASK_BUS1_S)
#define EXTMEM_PRO_ICACHE_MASK_BUS1_V  0x00000001
#define EXTMEM_PRO_ICACHE_MASK_BUS1_S  1

/* EXTMEM_PRO_ICACHE_MASK_BUS0 : R/W; bitpos: [0]; default: 1;
 * The bit is used to disable ibus0, 0: enable, 1: disable
 */

#define EXTMEM_PRO_ICACHE_MASK_BUS0    (BIT(0))
#define EXTMEM_PRO_ICACHE_MASK_BUS0_M  (EXTMEM_PRO_ICACHE_MASK_BUS0_V << EXTMEM_PRO_ICACHE_MASK_BUS0_S)
#define EXTMEM_PRO_ICACHE_MASK_BUS0_V  0x00000001
#define EXTMEM_PRO_ICACHE_MASK_BUS0_S  0
#define EXTMEM_PRO_ICACHE_MASK_IRAM0   EXTMEM_PRO_ICACHE_MASK_BUS0
#define EXTMEM_PRO_ICACHE_MASK_IRAM1   EXTMEM_PRO_ICACHE_MASK_BUS1
#define EXTMEM_PRO_ICACHE_MASK_DROM0   EXTMEM_PRO_ICACHE_MASK_BUS2

/* EXTMEM_PRO_ICACHE_TAG_POWER_CTRL_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_TAG_POWER_CTRL_REG (DR_REG_EXTMEM_BASE + 0x48)

/* EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PU : R/W; bitpos: [2]; default: 1;
 * The bit is used to power icache tag memory down, 0: follow rtc_lslp, 1:
 * power up
 */

#define EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PU    (BIT(2))
#define EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PU_M  (EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PU_V << EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PU_S)
#define EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PU_V  0x00000001
#define EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PU_S  2

/* EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PD : R/W; bitpos: [1]; default: 0;
 * The bit is used to power icache tag memory down, 0: follow rtc_lslp, 1:
 * power down
 */

#define EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PD    (BIT(1))
#define EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PD_M  (EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PD_V << EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PD_S)
#define EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PD_V  0x00000001
#define EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_PD_S  1

/* EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_ON : R/W; bitpos: [0]; default: 1;
 * The bit is used to close clock gating of icache tag memory. 1: close
 * gating, 0: open clock gating.
 */

#define EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_ON    (BIT(0))
#define EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_ON_M  (EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_ON_V << EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_ON_S)
#define EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_ON_V  0x00000001
#define EXTMEM_PRO_ICACHE_TAG_MEM_FORCE_ON_S  0

/* EXTMEM_PRO_ICACHE_LOCK0_ADDR_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_LOCK0_ADDR_REG (DR_REG_EXTMEM_BASE + 0x4c)

/* EXTMEM_PRO_ICACHE_LOCK0_ADDR : R/W; bitpos: [31:0]; default: 0;
 * The bits are used to configure the first start virtual address of data
 * locking, which is combined with PRO_ICACHE_LOCK0_SIZE_REG
 */

#define EXTMEM_PRO_ICACHE_LOCK0_ADDR    0xffffffff
#define EXTMEM_PRO_ICACHE_LOCK0_ADDR_M  (EXTMEM_PRO_ICACHE_LOCK0_ADDR_V << EXTMEM_PRO_ICACHE_LOCK0_ADDR_S)
#define EXTMEM_PRO_ICACHE_LOCK0_ADDR_V  0xffffffff
#define EXTMEM_PRO_ICACHE_LOCK0_ADDR_S  0

/* EXTMEM_PRO_ICACHE_LOCK0_SIZE_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_LOCK0_SIZE_REG (DR_REG_EXTMEM_BASE + 0x50)

/* EXTMEM_PRO_ICACHE_LOCK0_SIZE : R/W; bitpos: [15:0]; default: 0;
 * The bits are used to configure the first length of data locking, which is
 * combined with PRO_ICACHE_LOCK0_ADDR_REG
 */

#define EXTMEM_PRO_ICACHE_LOCK0_SIZE    0x0000ffff
#define EXTMEM_PRO_ICACHE_LOCK0_SIZE_M  (EXTMEM_PRO_ICACHE_LOCK0_SIZE_V << EXTMEM_PRO_ICACHE_LOCK0_SIZE_S)
#define EXTMEM_PRO_ICACHE_LOCK0_SIZE_V  0x0000ffff
#define EXTMEM_PRO_ICACHE_LOCK0_SIZE_S  0

/* EXTMEM_PRO_ICACHE_LOCK1_ADDR_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_LOCK1_ADDR_REG (DR_REG_EXTMEM_BASE + 0x54)

/* EXTMEM_PRO_ICACHE_LOCK1_ADDR : R/W; bitpos: [31:0]; default: 0;
 * The bits are used to configure the second start virtual address of data
 * locking, which is combined with PRO_ICACHE_LOCK1_SIZE_REG
 */

#define EXTMEM_PRO_ICACHE_LOCK1_ADDR    0xffffffff
#define EXTMEM_PRO_ICACHE_LOCK1_ADDR_M  (EXTMEM_PRO_ICACHE_LOCK1_ADDR_V << EXTMEM_PRO_ICACHE_LOCK1_ADDR_S)
#define EXTMEM_PRO_ICACHE_LOCK1_ADDR_V  0xffffffff
#define EXTMEM_PRO_ICACHE_LOCK1_ADDR_S  0

/* EXTMEM_PRO_ICACHE_LOCK1_SIZE_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_LOCK1_SIZE_REG (DR_REG_EXTMEM_BASE + 0x58)

/* EXTMEM_PRO_ICACHE_LOCK1_SIZE : R/W; bitpos: [15:0]; default: 0;
 * The bits are used to configure the second length of data locking, which
 * is combined with PRO_ICACHE_LOCK1_ADDR_REG
 */

#define EXTMEM_PRO_ICACHE_LOCK1_SIZE    0x0000ffff
#define EXTMEM_PRO_ICACHE_LOCK1_SIZE_M  (EXTMEM_PRO_ICACHE_LOCK1_SIZE_V << EXTMEM_PRO_ICACHE_LOCK1_SIZE_S)
#define EXTMEM_PRO_ICACHE_LOCK1_SIZE_V  0x0000ffff
#define EXTMEM_PRO_ICACHE_LOCK1_SIZE_S  0

/* EXTMEM_PRO_ICACHE_MEM_SYNC0_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_MEM_SYNC0_REG (DR_REG_EXTMEM_BASE + 0x5c)

/* EXTMEM_PRO_ICACHE_MEMSYNC_ADDR : R/W; bitpos: [31:0]; default: 0;
 * The bits are used to configure the start virtual address for invalidate,
 * flush, clean, lock and unlock operations. The manual operations will be
 * issued if the address is validate. The auto operations will be issued if
 * the address is invalidate. It should be combined with
 * PRO_ICACHE_MEM_SYNC1.
 */

#define EXTMEM_PRO_ICACHE_MEMSYNC_ADDR    0xffffffff
#define EXTMEM_PRO_ICACHE_MEMSYNC_ADDR_M  (EXTMEM_PRO_ICACHE_MEMSYNC_ADDR_V << EXTMEM_PRO_ICACHE_MEMSYNC_ADDR_S)
#define EXTMEM_PRO_ICACHE_MEMSYNC_ADDR_V  0xffffffff
#define EXTMEM_PRO_ICACHE_MEMSYNC_ADDR_S  0

/* EXTMEM_PRO_ICACHE_MEM_SYNC1_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_MEM_SYNC1_REG (DR_REG_EXTMEM_BASE + 0x60)

/* EXTMEM_PRO_ICACHE_MEMSYNC_SIZE : R/W; bitpos: [18:0]; default: 0;
 * The bits are used to configure the length for invalidate, flush, clean,
 * lock and unlock operations. The manual operations will be issued if it is
 * validate. The auto operations will be issued if it is invalidate. It
 * should be combined with PRO_ICACHE_MEM_SYNC0.
 */

#define EXTMEM_PRO_ICACHE_MEMSYNC_SIZE    0x0007ffff
#define EXTMEM_PRO_ICACHE_MEMSYNC_SIZE_M  (EXTMEM_PRO_ICACHE_MEMSYNC_SIZE_V << EXTMEM_PRO_ICACHE_MEMSYNC_SIZE_S)
#define EXTMEM_PRO_ICACHE_MEMSYNC_SIZE_V  0x0007ffff
#define EXTMEM_PRO_ICACHE_MEMSYNC_SIZE_S  0

/* EXTMEM_PRO_ICACHE_PRELOAD_ADDR_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_PRELOAD_ADDR_REG (DR_REG_EXTMEM_BASE + 0x64)

/* EXTMEM_PRO_ICACHE_PRELOAD_ADDR : R/W; bitpos: [31:0]; default: 0;
 * The bits are used to configure the start virtual address for manual
 * pre-load operation. It should be combined with
 * PRO_ICACHE_PRELOAD_SIZE_REG.
 */

#define EXTMEM_PRO_ICACHE_PRELOAD_ADDR    0xffffffff
#define EXTMEM_PRO_ICACHE_PRELOAD_ADDR_M  (EXTMEM_PRO_ICACHE_PRELOAD_ADDR_V << EXTMEM_PRO_ICACHE_PRELOAD_ADDR_S)
#define EXTMEM_PRO_ICACHE_PRELOAD_ADDR_V  0xffffffff
#define EXTMEM_PRO_ICACHE_PRELOAD_ADDR_S  0

/* EXTMEM_PRO_ICACHE_PRELOAD_SIZE_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_PRELOAD_SIZE_REG (DR_REG_EXTMEM_BASE + 0x68)

/* EXTMEM_PRO_ICACHE_PRELOAD_ORDER : R/W; bitpos: [10]; default: 0;
 * The bits are used to configure the direction of manual pre-load
 * operation. 1: descending, 0: ascending.
 */

#define EXTMEM_PRO_ICACHE_PRELOAD_ORDER    (BIT(10))
#define EXTMEM_PRO_ICACHE_PRELOAD_ORDER_M  (EXTMEM_PRO_ICACHE_PRELOAD_ORDER_V << EXTMEM_PRO_ICACHE_PRELOAD_ORDER_S)
#define EXTMEM_PRO_ICACHE_PRELOAD_ORDER_V  0x00000001
#define EXTMEM_PRO_ICACHE_PRELOAD_ORDER_S  10

/* EXTMEM_PRO_ICACHE_PRELOAD_SIZE : R/W; bitpos: [9:0]; default: 512;
 * The bits are used to configure the length for manual pre-load operation.
 * It should be combined with PRO_ICACHE_PRELOAD_ADDR_REG..
 */

#define EXTMEM_PRO_ICACHE_PRELOAD_SIZE    0x000003ff
#define EXTMEM_PRO_ICACHE_PRELOAD_SIZE_M  (EXTMEM_PRO_ICACHE_PRELOAD_SIZE_V << EXTMEM_PRO_ICACHE_PRELOAD_SIZE_S)
#define EXTMEM_PRO_ICACHE_PRELOAD_SIZE_V  0x000003ff
#define EXTMEM_PRO_ICACHE_PRELOAD_SIZE_S  0

/* EXTMEM_PRO_ICACHE_AUTOLOAD_CFG_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_CFG_REG (DR_REG_EXTMEM_BASE + 0x6c)

/* EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ENA : R/W; bitpos: [9]; default: 0;
 * The bits are used to enable the first section for conditional pre-load
 * operation.
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ENA    (BIT(9))
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ENA_M  (EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ENA_V << EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ENA_S)
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ENA_V  0x00000001
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ENA_S  9

/* EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ENA : R/W; bitpos: [8]; default: 0;
 * The bits are used to enable the second section for conditional pre-load
 * operation.
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ENA    (BIT(8))
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ENA_M  (EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ENA_V << EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ENA_S)
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ENA_V  0x00000001
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ENA_S  8

/* EXTMEM_PRO_ICACHE_AUTOLOAD_SIZE : R/W; bitpos: [7:6]; default: 0;
 * The bits are used to configure the numbers of the cache block for the
 * issuing conditional pre-load operation.
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_SIZE    0x00000003
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SIZE_M  (EXTMEM_PRO_ICACHE_AUTOLOAD_SIZE_V << EXTMEM_PRO_ICACHE_AUTOLOAD_SIZE_S)
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SIZE_V  0x00000003
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SIZE_S  6

/* EXTMEM_PRO_ICACHE_AUTOLOAD_RQST : R/W; bitpos: [5:4]; default: 0;
 * The bits are used to configure trigger conditions for conditional
 * pre-load. 0/3: cache miss, 1: cache hit, 2: both cache miss and hit.
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_RQST    0x00000003
#define EXTMEM_PRO_ICACHE_AUTOLOAD_RQST_M  (EXTMEM_PRO_ICACHE_AUTOLOAD_RQST_V << EXTMEM_PRO_ICACHE_AUTOLOAD_RQST_S)
#define EXTMEM_PRO_ICACHE_AUTOLOAD_RQST_V  0x00000003
#define EXTMEM_PRO_ICACHE_AUTOLOAD_RQST_S  4

/* EXTMEM_PRO_ICACHE_AUTOLOAD_ORDER : R/W; bitpos: [3]; default: 0;
 * The bits are used to configure the direction of conditional pre-load
 * operation. 1: descending, 0: ascending.
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_ORDER    (BIT(3))
#define EXTMEM_PRO_ICACHE_AUTOLOAD_ORDER_M  (EXTMEM_PRO_ICACHE_AUTOLOAD_ORDER_V << EXTMEM_PRO_ICACHE_AUTOLOAD_ORDER_S)
#define EXTMEM_PRO_ICACHE_AUTOLOAD_ORDER_V  0x00000001
#define EXTMEM_PRO_ICACHE_AUTOLOAD_ORDER_S  3

/* EXTMEM_PRO_ICACHE_AUTOLOAD_STEP : R/W; bitpos: [2:1]; default: 0;
 * Reserved.
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_STEP    0x00000003
#define EXTMEM_PRO_ICACHE_AUTOLOAD_STEP_M  (EXTMEM_PRO_ICACHE_AUTOLOAD_STEP_V << EXTMEM_PRO_ICACHE_AUTOLOAD_STEP_S)
#define EXTMEM_PRO_ICACHE_AUTOLOAD_STEP_V  0x00000003
#define EXTMEM_PRO_ICACHE_AUTOLOAD_STEP_S  1

/* EXTMEM_PRO_ICACHE_AUTOLOAD_MODE : R/W; bitpos: [0]; default: 0;
 * Reserved.
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_MODE    (BIT(0))
#define EXTMEM_PRO_ICACHE_AUTOLOAD_MODE_M  (EXTMEM_PRO_ICACHE_AUTOLOAD_MODE_V << EXTMEM_PRO_ICACHE_AUTOLOAD_MODE_S)
#define EXTMEM_PRO_ICACHE_AUTOLOAD_MODE_V  0x00000001
#define EXTMEM_PRO_ICACHE_AUTOLOAD_MODE_S  0

/* EXTMEM_PRO_ICACHE_AUTOLOAD_SECTION0_ADDR_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_SECTION0_ADDR_REG (DR_REG_EXTMEM_BASE + 0x70)

/* EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ADDR : R/W; bitpos: [31:0]; default: 0;
 * The bits are used to configure the start virtual address of the first
 * section for conditional pre-load operation. It should be combined with
 * pro_icache_autoload_sct0_ena.
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ADDR    0xffffffff
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ADDR_M  (EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ADDR_V << EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ADDR_S)
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ADDR_V  0xffffffff
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_ADDR_S  0

/* EXTMEM_PRO_ICACHE_AUTOLOAD_SECTION0_SIZE_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_SECTION0_SIZE_REG (DR_REG_EXTMEM_BASE + 0x74)

/* EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_SIZE : R/W; bitpos: [23:0]; default:
 * 32768;
 * The bits are used to configure the length of the first section for
 * conditional pre-load operation. It should be combined with
 * pro_icache_autoload_sct0_ena.
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_SIZE    0x00ffffff
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_SIZE_M  (EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_SIZE_V << EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_SIZE_S)
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_SIZE_V  0x00ffffff
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT0_SIZE_S  0

/* EXTMEM_PRO_ICACHE_AUTOLOAD_SECTION1_ADDR_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_SECTION1_ADDR_REG (DR_REG_EXTMEM_BASE + 0x78)

/* EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ADDR : R/W; bitpos: [31:0]; default: 0;
 * The bits are used to configure the start virtual address of the second
 * section for conditional pre-load operation. It should be combined with
 * pro_icache_autoload_sct1_ena.
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ADDR    0xffffffff
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ADDR_M  (EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ADDR_V << EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ADDR_S)
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ADDR_V  0xffffffff
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_ADDR_S  0

/* EXTMEM_PRO_ICACHE_AUTOLOAD_SECTION1_SIZE_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_SECTION1_SIZE_REG (DR_REG_EXTMEM_BASE + 0x7c)

/* EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_SIZE : R/W; bitpos: [23:0]; default:
 * 32768;
 * The bits are used to configure the length of the second section for
 * conditional pre-load operation. It should be combined with
 * pro_icache_autoload_sct1_ena.
 */

#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_SIZE    0x00ffffff
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_SIZE_M  (EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_SIZE_V << EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_SIZE_S)
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_SIZE_V  0x00ffffff
#define EXTMEM_PRO_ICACHE_AUTOLOAD_SCT1_SIZE_S  0

/* EXTMEM_IC_PRELOAD_CNT_REG register
 * register description
 */

#define EXTMEM_IC_PRELOAD_CNT_REG (DR_REG_EXTMEM_BASE + 0x80)

/* EXTMEM_IC_PRELOAD_CNT : RO; bitpos: [15:0]; default: 0;
 * The bits are used to count the number of issued pre-load which include
 * manual pre-load and conditional pre-load.
 */

#define EXTMEM_IC_PRELOAD_CNT    0x0000ffff
#define EXTMEM_IC_PRELOAD_CNT_M  (EXTMEM_IC_PRELOAD_CNT_V << EXTMEM_IC_PRELOAD_CNT_S)
#define EXTMEM_IC_PRELOAD_CNT_V  0x0000ffff
#define EXTMEM_IC_PRELOAD_CNT_S  0

/* EXTMEM_IC_PRELOAD_MISS_CNT_REG register
 * register description
 */

#define EXTMEM_IC_PRELOAD_MISS_CNT_REG (DR_REG_EXTMEM_BASE + 0x84)

/* EXTMEM_IC_PRELOAD_MISS_CNT : RO; bitpos: [15:0]; default: 0;
 * The bits are used to count the number of missed pre-load which include
 * manual pre-load and conditional pre-load.
 */

#define EXTMEM_IC_PRELOAD_MISS_CNT    0x0000ffff
#define EXTMEM_IC_PRELOAD_MISS_CNT_M  (EXTMEM_IC_PRELOAD_MISS_CNT_V << EXTMEM_IC_PRELOAD_MISS_CNT_S)
#define EXTMEM_IC_PRELOAD_MISS_CNT_V  0x0000ffff
#define EXTMEM_IC_PRELOAD_MISS_CNT_S  0

/* EXTMEM_IBUS2_ABANDON_CNT_REG register
 * register description
 */

#define EXTMEM_IBUS2_ABANDON_CNT_REG (DR_REG_EXTMEM_BASE + 0x88)

/* EXTMEM_IBUS2_ABANDON_CNT : RO; bitpos: [15:0]; default: 0;
 * The bits are used to count the number of the abandoned ibus2 access.
 */

#define EXTMEM_IBUS2_ABANDON_CNT    0x0000ffff
#define EXTMEM_IBUS2_ABANDON_CNT_M  (EXTMEM_IBUS2_ABANDON_CNT_V << EXTMEM_IBUS2_ABANDON_CNT_S)
#define EXTMEM_IBUS2_ABANDON_CNT_V  0x0000ffff
#define EXTMEM_IBUS2_ABANDON_CNT_S  0

/* EXTMEM_IBUS1_ABANDON_CNT_REG register
 * register description
 */

#define EXTMEM_IBUS1_ABANDON_CNT_REG (DR_REG_EXTMEM_BASE + 0x8c)

/* EXTMEM_IBUS1_ABANDON_CNT : RO; bitpos: [15:0]; default: 0;
 * The bits are used to count the number of the abandoned ibus1 access.
 */

#define EXTMEM_IBUS1_ABANDON_CNT    0x0000ffff
#define EXTMEM_IBUS1_ABANDON_CNT_M  (EXTMEM_IBUS1_ABANDON_CNT_V << EXTMEM_IBUS1_ABANDON_CNT_S)
#define EXTMEM_IBUS1_ABANDON_CNT_V  0x0000ffff
#define EXTMEM_IBUS1_ABANDON_CNT_S  0

/* EXTMEM_IBUS0_ABANDON_CNT_REG register
 * register description
 */

#define EXTMEM_IBUS0_ABANDON_CNT_REG (DR_REG_EXTMEM_BASE + 0x90)

/* EXTMEM_IBUS0_ABANDON_CNT : RO; bitpos: [15:0]; default: 0;
 * The bits are used to count the number of the abandoned ibus0 access.
 */

#define EXTMEM_IBUS0_ABANDON_CNT    0x0000ffff
#define EXTMEM_IBUS0_ABANDON_CNT_M  (EXTMEM_IBUS0_ABANDON_CNT_V << EXTMEM_IBUS0_ABANDON_CNT_S)
#define EXTMEM_IBUS0_ABANDON_CNT_V  0x0000ffff
#define EXTMEM_IBUS0_ABANDON_CNT_S  0

/* EXTMEM_IBUS2_ACS_MISS_CNT_REG register
 * register description
 */

#define EXTMEM_IBUS2_ACS_MISS_CNT_REG (DR_REG_EXTMEM_BASE + 0x94)

/* EXTMEM_IBUS2_ACS_MISS_CNT : RO; bitpos: [31:0]; default: 0;
 * The bits are used to count the number of the cache miss caused by ibus2
 * access.
 */

#define EXTMEM_IBUS2_ACS_MISS_CNT    0xffffffff
#define EXTMEM_IBUS2_ACS_MISS_CNT_M  (EXTMEM_IBUS2_ACS_MISS_CNT_V << EXTMEM_IBUS2_ACS_MISS_CNT_S)
#define EXTMEM_IBUS2_ACS_MISS_CNT_V  0xffffffff
#define EXTMEM_IBUS2_ACS_MISS_CNT_S  0

/* EXTMEM_IBUS1_ACS_MISS_CNT_REG register
 * register description
 */

#define EXTMEM_IBUS1_ACS_MISS_CNT_REG (DR_REG_EXTMEM_BASE + 0x98)

/* EXTMEM_IBUS1_ACS_MISS_CNT : RO; bitpos: [31:0]; default: 0;
 * The bits are used to count the number of the cache miss caused by ibus1
 * access.
 */

#define EXTMEM_IBUS1_ACS_MISS_CNT    0xffffffff
#define EXTMEM_IBUS1_ACS_MISS_CNT_M  (EXTMEM_IBUS1_ACS_MISS_CNT_V << EXTMEM_IBUS1_ACS_MISS_CNT_S)
#define EXTMEM_IBUS1_ACS_MISS_CNT_V  0xffffffff
#define EXTMEM_IBUS1_ACS_MISS_CNT_S  0

/* EXTMEM_IBUS0_ACS_MISS_CNT_REG register
 * register description
 */

#define EXTMEM_IBUS0_ACS_MISS_CNT_REG (DR_REG_EXTMEM_BASE + 0x9c)

/* EXTMEM_IBUS0_ACS_MISS_CNT : RO; bitpos: [31:0]; default: 0;
 * The bits are used to count the number of the cache miss caused by ibus0
 * access.
 */

#define EXTMEM_IBUS0_ACS_MISS_CNT    0xffffffff
#define EXTMEM_IBUS0_ACS_MISS_CNT_M  (EXTMEM_IBUS0_ACS_MISS_CNT_V << EXTMEM_IBUS0_ACS_MISS_CNT_S)
#define EXTMEM_IBUS0_ACS_MISS_CNT_V  0xffffffff
#define EXTMEM_IBUS0_ACS_MISS_CNT_S  0

/* EXTMEM_IBUS2_ACS_CNT_REG register
 * register description
 */

#define EXTMEM_IBUS2_ACS_CNT_REG (DR_REG_EXTMEM_BASE + 0xa0)

/* EXTMEM_IBUS2_ACS_CNT : RO; bitpos: [31:0]; default: 0;
 * The bits are used to count the number of ibus2 access icache.
 */

#define EXTMEM_IBUS2_ACS_CNT    0xffffffff
#define EXTMEM_IBUS2_ACS_CNT_M  (EXTMEM_IBUS2_ACS_CNT_V << EXTMEM_IBUS2_ACS_CNT_S)
#define EXTMEM_IBUS2_ACS_CNT_V  0xffffffff
#define EXTMEM_IBUS2_ACS_CNT_S  0

/* EXTMEM_IBUS1_ACS_CNT_REG register
 * register description
 */

#define EXTMEM_IBUS1_ACS_CNT_REG (DR_REG_EXTMEM_BASE + 0xa4)

/* EXTMEM_IBUS1_ACS_CNT : RO; bitpos: [31:0]; default: 0;
 * The bits are used to count the number of ibus1 access icache.
 */

#define EXTMEM_IBUS1_ACS_CNT    0xffffffff
#define EXTMEM_IBUS1_ACS_CNT_M  (EXTMEM_IBUS1_ACS_CNT_V << EXTMEM_IBUS1_ACS_CNT_S)
#define EXTMEM_IBUS1_ACS_CNT_V  0xffffffff
#define EXTMEM_IBUS1_ACS_CNT_S  0

/* EXTMEM_IBUS0_ACS_CNT_REG register
 * register description
 */

#define EXTMEM_IBUS0_ACS_CNT_REG (DR_REG_EXTMEM_BASE + 0xa8)

/* EXTMEM_IBUS0_ACS_CNT : RO; bitpos: [31:0]; default: 0;
 * The bits are used to count the number of ibus0 access icache.
 */

#define EXTMEM_IBUS0_ACS_CNT    0xffffffff
#define EXTMEM_IBUS0_ACS_CNT_M  (EXTMEM_IBUS0_ACS_CNT_V << EXTMEM_IBUS0_ACS_CNT_S)
#define EXTMEM_IBUS0_ACS_CNT_V  0xffffffff
#define EXTMEM_IBUS0_ACS_CNT_S  0

/* EXTMEM_DC_PRELOAD_CNT_REG register
 * register description
 */

#define EXTMEM_DC_PRELOAD_CNT_REG (DR_REG_EXTMEM_BASE + 0xac)

/* EXTMEM_DC_PRELOAD_CNT : RO; bitpos: [15:0]; default: 0;
 * The bits are used to count the number of issued pre-load which include
 * manual pre-load and conditional pre-load.
 */

#define EXTMEM_DC_PRELOAD_CNT    0x0000ffff
#define EXTMEM_DC_PRELOAD_CNT_M  (EXTMEM_DC_PRELOAD_CNT_V << EXTMEM_DC_PRELOAD_CNT_S)
#define EXTMEM_DC_PRELOAD_CNT_V  0x0000ffff
#define EXTMEM_DC_PRELOAD_CNT_S  0

/* EXTMEM_DC_PRELOAD_EVICT_CNT_REG register
 * register description
 */

#define EXTMEM_DC_PRELOAD_EVICT_CNT_REG (DR_REG_EXTMEM_BASE + 0xb0)

/* EXTMEM_DC_PRELOAD_EVICT_CNT : RO; bitpos: [15:0]; default: 0;
 * The bits are used to count the number of cache evictions by pre-load
 * which include manual pre-load and conditional pre-load.
 */

#define EXTMEM_DC_PRELOAD_EVICT_CNT    0x0000ffff
#define EXTMEM_DC_PRELOAD_EVICT_CNT_M  (EXTMEM_DC_PRELOAD_EVICT_CNT_V << EXTMEM_DC_PRELOAD_EVICT_CNT_S)
#define EXTMEM_DC_PRELOAD_EVICT_CNT_V  0x0000ffff
#define EXTMEM_DC_PRELOAD_EVICT_CNT_S  0

/* EXTMEM_DC_PRELOAD_MISS_CNT_REG register
 * register description
 */

#define EXTMEM_DC_PRELOAD_MISS_CNT_REG (DR_REG_EXTMEM_BASE + 0xb4)

/* EXTMEM_DC_PRELOAD_MISS_CNT : RO; bitpos: [15:0]; default: 0;
 * The bits are used to count the number of missed pre-load which include
 * manual pre-load and conditional pre-load.
 */

#define EXTMEM_DC_PRELOAD_MISS_CNT    0x0000ffff
#define EXTMEM_DC_PRELOAD_MISS_CNT_M  (EXTMEM_DC_PRELOAD_MISS_CNT_V << EXTMEM_DC_PRELOAD_MISS_CNT_S)
#define EXTMEM_DC_PRELOAD_MISS_CNT_V  0x0000ffff
#define EXTMEM_DC_PRELOAD_MISS_CNT_S  0

/* EXTMEM_DBUS2_ABANDON_CNT_REG register
 * register description
 */

#define EXTMEM_DBUS2_ABANDON_CNT_REG (DR_REG_EXTMEM_BASE + 0xb8)

/* EXTMEM_DBUS2_ABANDON_CNT : RO; bitpos: [15:0]; default: 0;
 * The bits are used to count the number of the abandoned dbus2 access.
 */

#define EXTMEM_DBUS2_ABANDON_CNT    0x0000ffff
#define EXTMEM_DBUS2_ABANDON_CNT_M  (EXTMEM_DBUS2_ABANDON_CNT_V << EXTMEM_DBUS2_ABANDON_CNT_S)
#define EXTMEM_DBUS2_ABANDON_CNT_V  0x0000ffff
#define EXTMEM_DBUS2_ABANDON_CNT_S  0

/* EXTMEM_DBUS1_ABANDON_CNT_REG register
 * register description
 */

#define EXTMEM_DBUS1_ABANDON_CNT_REG (DR_REG_EXTMEM_BASE + 0xbc)

/* EXTMEM_DBUS1_ABANDON_CNT : RO; bitpos: [15:0]; default: 0;
 * The bits are used to count the number of the abandoned dbus1 access.
 */

#define EXTMEM_DBUS1_ABANDON_CNT    0x0000ffff
#define EXTMEM_DBUS1_ABANDON_CNT_M  (EXTMEM_DBUS1_ABANDON_CNT_V << EXTMEM_DBUS1_ABANDON_CNT_S)
#define EXTMEM_DBUS1_ABANDON_CNT_V  0x0000ffff
#define EXTMEM_DBUS1_ABANDON_CNT_S  0

/* EXTMEM_DBUS0_ABANDON_CNT_REG register
 * register description
 */

#define EXTMEM_DBUS0_ABANDON_CNT_REG (DR_REG_EXTMEM_BASE + 0xc0)

/* EXTMEM_DBUS0_ABANDON_CNT : RO; bitpos: [15:0]; default: 0;
 * The bits are used to count the number of the abandoned dbus0 access.
 */

#define EXTMEM_DBUS0_ABANDON_CNT    0x0000ffff
#define EXTMEM_DBUS0_ABANDON_CNT_M  (EXTMEM_DBUS0_ABANDON_CNT_V << EXTMEM_DBUS0_ABANDON_CNT_S)
#define EXTMEM_DBUS0_ABANDON_CNT_V  0x0000ffff
#define EXTMEM_DBUS0_ABANDON_CNT_S  0

/* EXTMEM_DBUS2_ACS_WB_CNT_REG register
 * register description
 */

#define EXTMEM_DBUS2_ACS_WB_CNT_REG (DR_REG_EXTMEM_BASE + 0xc4)

/* EXTMEM_DBUS2_ACS_WB_CNT : RO; bitpos: [19:0]; default: 0;
 * The bits are used to count the number of cache evictions by dbus2 access
 * cache.
 */

#define EXTMEM_DBUS2_ACS_WB_CNT    0x000fffff
#define EXTMEM_DBUS2_ACS_WB_CNT_M  (EXTMEM_DBUS2_ACS_WB_CNT_V << EXTMEM_DBUS2_ACS_WB_CNT_S)
#define EXTMEM_DBUS2_ACS_WB_CNT_V  0x000fffff
#define EXTMEM_DBUS2_ACS_WB_CNT_S  0

/* EXTMEM_DBUS1_ACS_WB_CNT_REG register
 * register description
 */

#define EXTMEM_DBUS1_ACS_WB_CNT_REG (DR_REG_EXTMEM_BASE + 0xc8)

/* EXTMEM_DBUS1_ACS_WB_CNT : RO; bitpos: [19:0]; default: 0;
 * The bits are used to count the number of cache evictions by dbus1 access
 * cache.
 */

#define EXTMEM_DBUS1_ACS_WB_CNT    0x000fffff
#define EXTMEM_DBUS1_ACS_WB_CNT_M  (EXTMEM_DBUS1_ACS_WB_CNT_V << EXTMEM_DBUS1_ACS_WB_CNT_S)
#define EXTMEM_DBUS1_ACS_WB_CNT_V  0x000fffff
#define EXTMEM_DBUS1_ACS_WB_CNT_S  0

/* EXTMEM_DBUS0_ACS_WB_CNT_REG register
 * register description
 */

#define EXTMEM_DBUS0_ACS_WB_CNT_REG (DR_REG_EXTMEM_BASE + 0xcc)

/* EXTMEM_DBUS0_ACS_WB_CNT : RO; bitpos: [19:0]; default: 0;
 * The bits are used to count the number of cache evictions by dbus0 access
 * cache.
 */

#define EXTMEM_DBUS0_ACS_WB_CNT    0x000fffff
#define EXTMEM_DBUS0_ACS_WB_CNT_M  (EXTMEM_DBUS0_ACS_WB_CNT_V << EXTMEM_DBUS0_ACS_WB_CNT_S)
#define EXTMEM_DBUS0_ACS_WB_CNT_V  0x000fffff
#define EXTMEM_DBUS0_ACS_WB_CNT_S  0

/* EXTMEM_DBUS2_ACS_MISS_CNT_REG register
 * register description
 */

#define EXTMEM_DBUS2_ACS_MISS_CNT_REG (DR_REG_EXTMEM_BASE + 0xd0)

/* EXTMEM_DBUS2_ACS_MISS_CNT : RO; bitpos: [31:0]; default: 0;
 * The bits are used to count the number of the cache miss caused by dbus2
 * access.
 */

#define EXTMEM_DBUS2_ACS_MISS_CNT    0xffffffff
#define EXTMEM_DBUS2_ACS_MISS_CNT_M  (EXTMEM_DBUS2_ACS_MISS_CNT_V << EXTMEM_DBUS2_ACS_MISS_CNT_S)
#define EXTMEM_DBUS2_ACS_MISS_CNT_V  0xffffffff
#define EXTMEM_DBUS2_ACS_MISS_CNT_S  0

/* EXTMEM_DBUS1_ACS_MISS_CNT_REG register
 * register description
 */

#define EXTMEM_DBUS1_ACS_MISS_CNT_REG (DR_REG_EXTMEM_BASE + 0xd4)

/* EXTMEM_DBUS1_ACS_MISS_CNT : RO; bitpos: [31:0]; default: 0;
 * The bits are used to count the number of the cache miss caused by dbus1
 * access.
 */

#define EXTMEM_DBUS1_ACS_MISS_CNT    0xffffffff
#define EXTMEM_DBUS1_ACS_MISS_CNT_M  (EXTMEM_DBUS1_ACS_MISS_CNT_V << EXTMEM_DBUS1_ACS_MISS_CNT_S)
#define EXTMEM_DBUS1_ACS_MISS_CNT_V  0xffffffff
#define EXTMEM_DBUS1_ACS_MISS_CNT_S  0

/* EXTMEM_DBUS0_ACS_MISS_CNT_REG register
 * register description
 */

#define EXTMEM_DBUS0_ACS_MISS_CNT_REG (DR_REG_EXTMEM_BASE + 0xd8)

/* EXTMEM_DBUS0_ACS_MISS_CNT : RO; bitpos: [31:0]; default: 0;
 * The bits are used to count the number of the cache miss caused by dbus0
 * access.
 */

#define EXTMEM_DBUS0_ACS_MISS_CNT    0xffffffff
#define EXTMEM_DBUS0_ACS_MISS_CNT_M  (EXTMEM_DBUS0_ACS_MISS_CNT_V << EXTMEM_DBUS0_ACS_MISS_CNT_S)
#define EXTMEM_DBUS0_ACS_MISS_CNT_V  0xffffffff
#define EXTMEM_DBUS0_ACS_MISS_CNT_S  0

/* EXTMEM_DBUS2_ACS_CNT_REG register
 * register description
 */

#define EXTMEM_DBUS2_ACS_CNT_REG (DR_REG_EXTMEM_BASE + 0xdc)

/* EXTMEM_DBUS2_ACS_CNT : RO; bitpos: [31:0]; default: 0;
 * The bits are used to count the number of dbus2 access dcache.
 */

#define EXTMEM_DBUS2_ACS_CNT    0xffffffff
#define EXTMEM_DBUS2_ACS_CNT_M  (EXTMEM_DBUS2_ACS_CNT_V << EXTMEM_DBUS2_ACS_CNT_S)
#define EXTMEM_DBUS2_ACS_CNT_V  0xffffffff
#define EXTMEM_DBUS2_ACS_CNT_S  0

/* EXTMEM_DBUS1_ACS_CNT_REG register
 * register description
 */

#define EXTMEM_DBUS1_ACS_CNT_REG (DR_REG_EXTMEM_BASE + 0xe0)

/* EXTMEM_DBUS1_ACS_CNT : RO; bitpos: [31:0]; default: 0;
 * The bits are used to count the number of dbus1 access dcache.
 */

#define EXTMEM_DBUS1_ACS_CNT    0xffffffff
#define EXTMEM_DBUS1_ACS_CNT_M  (EXTMEM_DBUS1_ACS_CNT_V << EXTMEM_DBUS1_ACS_CNT_S)
#define EXTMEM_DBUS1_ACS_CNT_V  0xffffffff
#define EXTMEM_DBUS1_ACS_CNT_S  0

/* EXTMEM_DBUS0_ACS_CNT_REG register
 * register description
 */

#define EXTMEM_DBUS0_ACS_CNT_REG (DR_REG_EXTMEM_BASE + 0xe4)

/* EXTMEM_DBUS0_ACS_CNT : RO; bitpos: [31:0]; default: 0;
 * The bits are used to count the number of dbus0 access dcache.
 */

#define EXTMEM_DBUS0_ACS_CNT    0xffffffff
#define EXTMEM_DBUS0_ACS_CNT_M  (EXTMEM_DBUS0_ACS_CNT_V << EXTMEM_DBUS0_ACS_CNT_S)
#define EXTMEM_DBUS0_ACS_CNT_V  0xffffffff
#define EXTMEM_DBUS0_ACS_CNT_S  0

/* EXTMEM_CACHE_DBG_INT_ENA_REG register
 * register description
 */

#define EXTMEM_CACHE_DBG_INT_ENA_REG (DR_REG_EXTMEM_BASE + 0xe8)

/* EXTMEM_MMU_ENTRY_FAULT_INT_ENA : R/W; bitpos: [19]; default: 0;
 * The bit is used to enable interrupt by mmu entry fault.
 */

#define EXTMEM_MMU_ENTRY_FAULT_INT_ENA    (BIT(19))
#define EXTMEM_MMU_ENTRY_FAULT_INT_ENA_M  (EXTMEM_MMU_ENTRY_FAULT_INT_ENA_V << EXTMEM_MMU_ENTRY_FAULT_INT_ENA_S)
#define EXTMEM_MMU_ENTRY_FAULT_INT_ENA_V  0x00000001
#define EXTMEM_MMU_ENTRY_FAULT_INT_ENA_S  19

/* EXTMEM_DCACHE_SET_LOCK_ILG_INT_ENA : R/W; bitpos: [18]; default: 0;
 * The bit is used to enable interrupt by illegal writing lock registers of
 * dcache while dcache is busy to issue lock,sync or pre-load operations.
 */

#define EXTMEM_DCACHE_SET_LOCK_ILG_INT_ENA    (BIT(18))
#define EXTMEM_DCACHE_SET_LOCK_ILG_INT_ENA_M  (EXTMEM_DCACHE_SET_LOCK_ILG_INT_ENA_V << EXTMEM_DCACHE_SET_LOCK_ILG_INT_ENA_S)
#define EXTMEM_DCACHE_SET_LOCK_ILG_INT_ENA_V  0x00000001
#define EXTMEM_DCACHE_SET_LOCK_ILG_INT_ENA_S  18

/* EXTMEM_DCACHE_SET_SYNC_ILG_INT_ENA : R/W; bitpos: [17]; default: 0;
 * The bit is used to enable interrupt by illegal writing sync registers of
 * dcache while dcache is busy to issue lock,sync and pre-load operations.
 */

#define EXTMEM_DCACHE_SET_SYNC_ILG_INT_ENA    (BIT(17))
#define EXTMEM_DCACHE_SET_SYNC_ILG_INT_ENA_M  (EXTMEM_DCACHE_SET_SYNC_ILG_INT_ENA_V << EXTMEM_DCACHE_SET_SYNC_ILG_INT_ENA_S)
#define EXTMEM_DCACHE_SET_SYNC_ILG_INT_ENA_V  0x00000001
#define EXTMEM_DCACHE_SET_SYNC_ILG_INT_ENA_S  17

/* EXTMEM_DCACHE_SET_PRELOAD_ILG_INT_ENA : R/W; bitpos: [16]; default: 0;
 * The bit is used to enable interrupt by illegal writing preload registers
 * of dcache while dcache is busy to issue lock,sync and pre-load operations.
 */

#define EXTMEM_DCACHE_SET_PRELOAD_ILG_INT_ENA    (BIT(16))
#define EXTMEM_DCACHE_SET_PRELOAD_ILG_INT_ENA_M  (EXTMEM_DCACHE_SET_PRELOAD_ILG_INT_ENA_V << EXTMEM_DCACHE_SET_PRELOAD_ILG_INT_ENA_S)
#define EXTMEM_DCACHE_SET_PRELOAD_ILG_INT_ENA_V  0x00000001
#define EXTMEM_DCACHE_SET_PRELOAD_ILG_INT_ENA_S  16

/* EXTMEM_DCACHE_REJECT_INT_ENA : R/W; bitpos: [15]; default: 0;
 * The bit is used to enable interrupt by authentication fail.
 */

#define EXTMEM_DCACHE_REJECT_INT_ENA    (BIT(15))
#define EXTMEM_DCACHE_REJECT_INT_ENA_M  (EXTMEM_DCACHE_REJECT_INT_ENA_V << EXTMEM_DCACHE_REJECT_INT_ENA_S)
#define EXTMEM_DCACHE_REJECT_INT_ENA_V  0x00000001
#define EXTMEM_DCACHE_REJECT_INT_ENA_S  15

/* EXTMEM_DCACHE_WRITE_FLASH_INT_ENA : R/W; bitpos: [14]; default: 0;
 * The bit is used to enable interrupt by dcache trying to write flash.
 */

#define EXTMEM_DCACHE_WRITE_FLASH_INT_ENA    (BIT(14))
#define EXTMEM_DCACHE_WRITE_FLASH_INT_ENA_M  (EXTMEM_DCACHE_WRITE_FLASH_INT_ENA_V << EXTMEM_DCACHE_WRITE_FLASH_INT_ENA_S)
#define EXTMEM_DCACHE_WRITE_FLASH_INT_ENA_V  0x00000001
#define EXTMEM_DCACHE_WRITE_FLASH_INT_ENA_S  14

/* EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_ENA : R/W; bitpos: [13]; default: 0;
 * The bit is used to enable interrupt by manual pre-load configurations
 * fault.
 */

#define EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_ENA    (BIT(13))
#define EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_ENA_M  (EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_ENA_V << EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_ENA_S)
#define EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_ENA_V  0x00000001
#define EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_ENA_S  13

/* EXTMEM_DC_SYNC_SIZE_FAULT_INT_ENA : R/W; bitpos: [12]; default: 0;
 * The bit is used to enable interrupt by manual sync configurations fault.
 */

#define EXTMEM_DC_SYNC_SIZE_FAULT_INT_ENA    (BIT(12))
#define EXTMEM_DC_SYNC_SIZE_FAULT_INT_ENA_M  (EXTMEM_DC_SYNC_SIZE_FAULT_INT_ENA_V << EXTMEM_DC_SYNC_SIZE_FAULT_INT_ENA_S)
#define EXTMEM_DC_SYNC_SIZE_FAULT_INT_ENA_V  0x00000001
#define EXTMEM_DC_SYNC_SIZE_FAULT_INT_ENA_S  12

/* EXTMEM_DBUS_CNT_OVF_INT_ENA : R/W; bitpos: [11]; default: 0;
 * The bit is used to enable interrupt by dbus counter overflow.
 */

#define EXTMEM_DBUS_CNT_OVF_INT_ENA    (BIT(11))
#define EXTMEM_DBUS_CNT_OVF_INT_ENA_M  (EXTMEM_DBUS_CNT_OVF_INT_ENA_V << EXTMEM_DBUS_CNT_OVF_INT_ENA_S)
#define EXTMEM_DBUS_CNT_OVF_INT_ENA_V  0x00000001
#define EXTMEM_DBUS_CNT_OVF_INT_ENA_S  11

/* EXTMEM_DBUS_ACS_MSK_DC_INT_ENA : R/W; bitpos: [10]; default: 0;
 * The bit is used to enable interrupt by cpu access dcache while the
 * corresponding dbus is disabled which include speculative access.
 */

#define EXTMEM_DBUS_ACS_MSK_DC_INT_ENA    (BIT(10))
#define EXTMEM_DBUS_ACS_MSK_DC_INT_ENA_M  (EXTMEM_DBUS_ACS_MSK_DC_INT_ENA_V << EXTMEM_DBUS_ACS_MSK_DC_INT_ENA_S)
#define EXTMEM_DBUS_ACS_MSK_DC_INT_ENA_V  0x00000001
#define EXTMEM_DBUS_ACS_MSK_DC_INT_ENA_S  10

/* EXTMEM_ICACHE_SET_LOCK_ILG_INT_ENA : R/W; bitpos: [9]; default: 0;
 * The bit is used to enable interrupt by illegal writing lock registers of
 * icache while icache is busy to issue lock,sync or pre-load operations.
 */

#define EXTMEM_ICACHE_SET_LOCK_ILG_INT_ENA    (BIT(9))
#define EXTMEM_ICACHE_SET_LOCK_ILG_INT_ENA_M  (EXTMEM_ICACHE_SET_LOCK_ILG_INT_ENA_V << EXTMEM_ICACHE_SET_LOCK_ILG_INT_ENA_S)
#define EXTMEM_ICACHE_SET_LOCK_ILG_INT_ENA_V  0x00000001
#define EXTMEM_ICACHE_SET_LOCK_ILG_INT_ENA_S  9

/* EXTMEM_ICACHE_SET_SYNC_ILG_INT_ENA : R/W; bitpos: [8]; default: 0;
 * The bit is used to enable interrupt by illegal writing sync registers of
 * icache while icache is busy to issue lock,sync and pre-load operations.
 */

#define EXTMEM_ICACHE_SET_SYNC_ILG_INT_ENA    (BIT(8))
#define EXTMEM_ICACHE_SET_SYNC_ILG_INT_ENA_M  (EXTMEM_ICACHE_SET_SYNC_ILG_INT_ENA_V << EXTMEM_ICACHE_SET_SYNC_ILG_INT_ENA_S)
#define EXTMEM_ICACHE_SET_SYNC_ILG_INT_ENA_V  0x00000001
#define EXTMEM_ICACHE_SET_SYNC_ILG_INT_ENA_S  8

/* EXTMEM_ICACHE_SET_PRELOAD_ILG_INT_ENA : R/W; bitpos: [7]; default: 0;
 * The bit is used to enable interrupt by illegal writing preload registers
 * of icache while icache is busy to issue lock,sync and pre-load operations.
 */

#define EXTMEM_ICACHE_SET_PRELOAD_ILG_INT_ENA    (BIT(7))
#define EXTMEM_ICACHE_SET_PRELOAD_ILG_INT_ENA_M  (EXTMEM_ICACHE_SET_PRELOAD_ILG_INT_ENA_V << EXTMEM_ICACHE_SET_PRELOAD_ILG_INT_ENA_S)
#define EXTMEM_ICACHE_SET_PRELOAD_ILG_INT_ENA_V  0x00000001
#define EXTMEM_ICACHE_SET_PRELOAD_ILG_INT_ENA_S  7

/* EXTMEM_ICACHE_REJECT_INT_ENA : R/W; bitpos: [6]; default: 0;
 * The bit is used to enable interrupt by authentication fail.
 */

#define EXTMEM_ICACHE_REJECT_INT_ENA    (BIT(6))
#define EXTMEM_ICACHE_REJECT_INT_ENA_M  (EXTMEM_ICACHE_REJECT_INT_ENA_V << EXTMEM_ICACHE_REJECT_INT_ENA_S)
#define EXTMEM_ICACHE_REJECT_INT_ENA_V  0x00000001
#define EXTMEM_ICACHE_REJECT_INT_ENA_S  6

/* EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_ENA : R/W; bitpos: [5]; default: 0;
 * The bit is used to enable interrupt by manual pre-load configurations
 * fault.
 */

#define EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_ENA    (BIT(5))
#define EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_ENA_M  (EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_ENA_V << EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_ENA_S)
#define EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_ENA_V  0x00000001
#define EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_ENA_S  5

/* EXTMEM_IC_SYNC_SIZE_FAULT_INT_ENA : R/W; bitpos: [4]; default: 0;
 * The bit is used to enable interrupt by manual sync configurations fault.
 */

#define EXTMEM_IC_SYNC_SIZE_FAULT_INT_ENA    (BIT(4))
#define EXTMEM_IC_SYNC_SIZE_FAULT_INT_ENA_M  (EXTMEM_IC_SYNC_SIZE_FAULT_INT_ENA_V << EXTMEM_IC_SYNC_SIZE_FAULT_INT_ENA_S)
#define EXTMEM_IC_SYNC_SIZE_FAULT_INT_ENA_V  0x00000001
#define EXTMEM_IC_SYNC_SIZE_FAULT_INT_ENA_S  4

/* EXTMEM_IBUS_CNT_OVF_INT_ENA : R/W; bitpos: [3]; default: 0;
 * The bit is used to enable interrupt by ibus counter overflow.
 */

#define EXTMEM_IBUS_CNT_OVF_INT_ENA    (BIT(3))
#define EXTMEM_IBUS_CNT_OVF_INT_ENA_M  (EXTMEM_IBUS_CNT_OVF_INT_ENA_V << EXTMEM_IBUS_CNT_OVF_INT_ENA_S)
#define EXTMEM_IBUS_CNT_OVF_INT_ENA_V  0x00000001
#define EXTMEM_IBUS_CNT_OVF_INT_ENA_S  3

/* EXTMEM_IBUS_ACS_MSK_IC_INT_ENA : R/W; bitpos: [2]; default: 0;
 * The bit is used to enable interrupt by cpu access icache while the
 * corresponding ibus is disabled which include speculative access.
 */

#define EXTMEM_IBUS_ACS_MSK_IC_INT_ENA    (BIT(2))
#define EXTMEM_IBUS_ACS_MSK_IC_INT_ENA_M  (EXTMEM_IBUS_ACS_MSK_IC_INT_ENA_V << EXTMEM_IBUS_ACS_MSK_IC_INT_ENA_S)
#define EXTMEM_IBUS_ACS_MSK_IC_INT_ENA_V  0x00000001
#define EXTMEM_IBUS_ACS_MSK_IC_INT_ENA_S  2

/* EXTMEM_CACHE_DBG_EN : R/W; bitpos: [0]; default: 1;
 * The bit is used to activate the cache track function. 1: enable, 0:
 * disable.
 */

#define EXTMEM_CACHE_DBG_EN    (BIT(0))
#define EXTMEM_CACHE_DBG_EN_M  (EXTMEM_CACHE_DBG_EN_V << EXTMEM_CACHE_DBG_EN_S)
#define EXTMEM_CACHE_DBG_EN_V  0x00000001
#define EXTMEM_CACHE_DBG_EN_S  0

/* EXTMEM_CACHE_DBG_INT_CLR_REG register
 * register description
 */

#define EXTMEM_CACHE_DBG_INT_CLR_REG (DR_REG_EXTMEM_BASE + 0xec)

/* EXTMEM_MMU_ENTRY_FAULT_INT_CLR : WOD; bitpos: [13]; default: 0;
 * The bit is used to clear interrupt by mmu entry fault.
 */

#define EXTMEM_MMU_ENTRY_FAULT_INT_CLR    (BIT(13))
#define EXTMEM_MMU_ENTRY_FAULT_INT_CLR_M  (EXTMEM_MMU_ENTRY_FAULT_INT_CLR_V << EXTMEM_MMU_ENTRY_FAULT_INT_CLR_S)
#define EXTMEM_MMU_ENTRY_FAULT_INT_CLR_V  0x00000001
#define EXTMEM_MMU_ENTRY_FAULT_INT_CLR_S  13

/* EXTMEM_DCACHE_SET_ILG_INT_CLR : WOD; bitpos: [12]; default: 0;
 * The bit is used to clear interrupt by illegal writing lock registers of
 * dcache while dcache is busy to issue lock,sync or pre-load operations.
 */

#define EXTMEM_DCACHE_SET_ILG_INT_CLR    (BIT(12))
#define EXTMEM_DCACHE_SET_ILG_INT_CLR_M  (EXTMEM_DCACHE_SET_ILG_INT_CLR_V << EXTMEM_DCACHE_SET_ILG_INT_CLR_S)
#define EXTMEM_DCACHE_SET_ILG_INT_CLR_V  0x00000001
#define EXTMEM_DCACHE_SET_ILG_INT_CLR_S  12

/* EXTMEM_DCACHE_REJECT_INT_CLR : WOD; bitpos: [11]; default: 0;
 * The bit is used to clear interrupt by authentication fail.
 */

#define EXTMEM_DCACHE_REJECT_INT_CLR    (BIT(11))
#define EXTMEM_DCACHE_REJECT_INT_CLR_M  (EXTMEM_DCACHE_REJECT_INT_CLR_V << EXTMEM_DCACHE_REJECT_INT_CLR_S)
#define EXTMEM_DCACHE_REJECT_INT_CLR_V  0x00000001
#define EXTMEM_DCACHE_REJECT_INT_CLR_S  11

/* EXTMEM_DCACHE_WRITE_FLASH_INT_CLR : WOD; bitpos: [10]; default: 0;
 * The bit is used to clear interrupt by dcache trying to write flash.
 */

#define EXTMEM_DCACHE_WRITE_FLASH_INT_CLR    (BIT(10))
#define EXTMEM_DCACHE_WRITE_FLASH_INT_CLR_M  (EXTMEM_DCACHE_WRITE_FLASH_INT_CLR_V << EXTMEM_DCACHE_WRITE_FLASH_INT_CLR_S)
#define EXTMEM_DCACHE_WRITE_FLASH_INT_CLR_V  0x00000001
#define EXTMEM_DCACHE_WRITE_FLASH_INT_CLR_S  10

/* EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_CLR : WOD; bitpos: [9]; default: 0;
 * The bit is used to clear interrupt by manual pre-load configurations
 * fault.
 */

#define EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_CLR    (BIT(9))
#define EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_CLR_M  (EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_CLR_V << EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_CLR_S)
#define EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_CLR_V  0x00000001
#define EXTMEM_DC_PRELOAD_SIZE_FAULT_INT_CLR_S  9

/* EXTMEM_DC_SYNC_SIZE_FAULT_INT_CLR : WOD; bitpos: [8]; default: 0;
 * The bit is used to clear interrupt by manual sync configurations fault.
 */

#define EXTMEM_DC_SYNC_SIZE_FAULT_INT_CLR    (BIT(8))
#define EXTMEM_DC_SYNC_SIZE_FAULT_INT_CLR_M  (EXTMEM_DC_SYNC_SIZE_FAULT_INT_CLR_V << EXTMEM_DC_SYNC_SIZE_FAULT_INT_CLR_S)
#define EXTMEM_DC_SYNC_SIZE_FAULT_INT_CLR_V  0x00000001
#define EXTMEM_DC_SYNC_SIZE_FAULT_INT_CLR_S  8

/* EXTMEM_DBUS_CNT_OVF_INT_CLR : WOD; bitpos: [7]; default: 0;
 * The bit is used to clear interrupt by dbus counter overflow.
 */

#define EXTMEM_DBUS_CNT_OVF_INT_CLR    (BIT(7))
#define EXTMEM_DBUS_CNT_OVF_INT_CLR_M  (EXTMEM_DBUS_CNT_OVF_INT_CLR_V << EXTMEM_DBUS_CNT_OVF_INT_CLR_S)
#define EXTMEM_DBUS_CNT_OVF_INT_CLR_V  0x00000001
#define EXTMEM_DBUS_CNT_OVF_INT_CLR_S  7

/* EXTMEM_DBUS_ACS_MSK_DC_INT_CLR : WOD; bitpos: [6]; default: 0;
 * The bit is used to clear interrupt by cpu access dcache while the
 * corresponding dbus is disabled or dcache is disabled which include
 * speculative access.
 */

#define EXTMEM_DBUS_ACS_MSK_DC_INT_CLR    (BIT(6))
#define EXTMEM_DBUS_ACS_MSK_DC_INT_CLR_M  (EXTMEM_DBUS_ACS_MSK_DC_INT_CLR_V << EXTMEM_DBUS_ACS_MSK_DC_INT_CLR_S)
#define EXTMEM_DBUS_ACS_MSK_DC_INT_CLR_V  0x00000001
#define EXTMEM_DBUS_ACS_MSK_DC_INT_CLR_S  6

/* EXTMEM_ICACHE_SET_ILG_INT_CLR : WOD; bitpos: [5]; default: 0;
 * The bit is used to clear interrupt by illegal writing lock registers of
 * icache while icache is busy to issue lock,sync or pre-load operations.
 */

#define EXTMEM_ICACHE_SET_ILG_INT_CLR    (BIT(5))
#define EXTMEM_ICACHE_SET_ILG_INT_CLR_M  (EXTMEM_ICACHE_SET_ILG_INT_CLR_V << EXTMEM_ICACHE_SET_ILG_INT_CLR_S)
#define EXTMEM_ICACHE_SET_ILG_INT_CLR_V  0x00000001
#define EXTMEM_ICACHE_SET_ILG_INT_CLR_S  5

/* EXTMEM_ICACHE_REJECT_INT_CLR : WOD; bitpos: [4]; default: 0;
 * The bit is used to clear interrupt by authentication fail.
 */

#define EXTMEM_ICACHE_REJECT_INT_CLR    (BIT(4))
#define EXTMEM_ICACHE_REJECT_INT_CLR_M  (EXTMEM_ICACHE_REJECT_INT_CLR_V << EXTMEM_ICACHE_REJECT_INT_CLR_S)
#define EXTMEM_ICACHE_REJECT_INT_CLR_V  0x00000001
#define EXTMEM_ICACHE_REJECT_INT_CLR_S  4

/* EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_CLR : WOD; bitpos: [3]; default: 0;
 * The bit is used to clear interrupt by manual pre-load configurations
 * fault.
 */

#define EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_CLR    (BIT(3))
#define EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_CLR_M  (EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_CLR_V << EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_CLR_S)
#define EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_CLR_V  0x00000001
#define EXTMEM_IC_PRELOAD_SIZE_FAULT_INT_CLR_S  3

/* EXTMEM_IC_SYNC_SIZE_FAULT_INT_CLR : WOD; bitpos: [2]; default: 0;
 * The bit is used to clear interrupt by manual sync configurations fault.
 */

#define EXTMEM_IC_SYNC_SIZE_FAULT_INT_CLR    (BIT(2))
#define EXTMEM_IC_SYNC_SIZE_FAULT_INT_CLR_M  (EXTMEM_IC_SYNC_SIZE_FAULT_INT_CLR_V << EXTMEM_IC_SYNC_SIZE_FAULT_INT_CLR_S)
#define EXTMEM_IC_SYNC_SIZE_FAULT_INT_CLR_V  0x00000001
#define EXTMEM_IC_SYNC_SIZE_FAULT_INT_CLR_S  2

/* EXTMEM_IBUS_CNT_OVF_INT_CLR : WOD; bitpos: [1]; default: 0;
 * The bit is used to clear interrupt by ibus counter overflow.
 */

#define EXTMEM_IBUS_CNT_OVF_INT_CLR    (BIT(1))
#define EXTMEM_IBUS_CNT_OVF_INT_CLR_M  (EXTMEM_IBUS_CNT_OVF_INT_CLR_V << EXTMEM_IBUS_CNT_OVF_INT_CLR_S)
#define EXTMEM_IBUS_CNT_OVF_INT_CLR_V  0x00000001
#define EXTMEM_IBUS_CNT_OVF_INT_CLR_S  1

/* EXTMEM_IBUS_ACS_MSK_IC_INT_CLR : WOD; bitpos: [0]; default: 0;
 * The bit is used to clear interrupt by cpu access icache while the
 * corresponding ibus is disabled or icache is disabled which include
 * speculative access.
 */

#define EXTMEM_IBUS_ACS_MSK_IC_INT_CLR    (BIT(0))
#define EXTMEM_IBUS_ACS_MSK_IC_INT_CLR_M  (EXTMEM_IBUS_ACS_MSK_IC_INT_CLR_V << EXTMEM_IBUS_ACS_MSK_IC_INT_CLR_S)
#define EXTMEM_IBUS_ACS_MSK_IC_INT_CLR_V  0x00000001
#define EXTMEM_IBUS_ACS_MSK_IC_INT_CLR_S  0

/* EXTMEM_CACHE_DBG_STATUS0_REG register
 * register description
 */

#define EXTMEM_CACHE_DBG_STATUS0_REG (DR_REG_EXTMEM_BASE + 0xf0)

/* EXTMEM_ICACHE_SET_LOCK_ILG_ST : RO; bitpos: [24]; default: 0;
 * The bit is used to indicate interrupt by illegal writing lock registers
 * of icache while icache is busy to issue lock,sync or pre-load operations.
 */

#define EXTMEM_ICACHE_SET_LOCK_ILG_ST    (BIT(24))
#define EXTMEM_ICACHE_SET_LOCK_ILG_ST_M  (EXTMEM_ICACHE_SET_LOCK_ILG_ST_V << EXTMEM_ICACHE_SET_LOCK_ILG_ST_S)
#define EXTMEM_ICACHE_SET_LOCK_ILG_ST_V  0x00000001
#define EXTMEM_ICACHE_SET_LOCK_ILG_ST_S  24

/* EXTMEM_ICACHE_SET_SYNC_ILG_ST : RO; bitpos: [23]; default: 0;
 * The bit is used to indicate interrupt by illegal writing sync registers
 * of icache while icache is busy to issue lock,sync and pre-load operations.
 */

#define EXTMEM_ICACHE_SET_SYNC_ILG_ST    (BIT(23))
#define EXTMEM_ICACHE_SET_SYNC_ILG_ST_M  (EXTMEM_ICACHE_SET_SYNC_ILG_ST_V << EXTMEM_ICACHE_SET_SYNC_ILG_ST_S)
#define EXTMEM_ICACHE_SET_SYNC_ILG_ST_V  0x00000001
#define EXTMEM_ICACHE_SET_SYNC_ILG_ST_S  23

/* EXTMEM_ICACHE_SET_PRELOAD_ILG_ST : RO; bitpos: [22]; default: 0;
 * The bit is used to indicate interrupt by illegal writing preload
 * registers of icache while icache is busy to issue lock,sync and pre-load
 * operations.
 */

#define EXTMEM_ICACHE_SET_PRELOAD_ILG_ST    (BIT(22))
#define EXTMEM_ICACHE_SET_PRELOAD_ILG_ST_M  (EXTMEM_ICACHE_SET_PRELOAD_ILG_ST_V << EXTMEM_ICACHE_SET_PRELOAD_ILG_ST_S)
#define EXTMEM_ICACHE_SET_PRELOAD_ILG_ST_V  0x00000001
#define EXTMEM_ICACHE_SET_PRELOAD_ILG_ST_S  22

/* EXTMEM_ICACHE_REJECT_ST : RO; bitpos: [21]; default: 0;
 * The bit is used to indicate interrupt by authentication fail.
 */

#define EXTMEM_ICACHE_REJECT_ST    (BIT(21))
#define EXTMEM_ICACHE_REJECT_ST_M  (EXTMEM_ICACHE_REJECT_ST_V << EXTMEM_ICACHE_REJECT_ST_S)
#define EXTMEM_ICACHE_REJECT_ST_V  0x00000001
#define EXTMEM_ICACHE_REJECT_ST_S  21

/* EXTMEM_IC_PRELOAD_SIZE_FAULT_ST : RO; bitpos: [20]; default: 0;
 * The bit is used to indicate interrupt by manual pre-load configurations
 * fault.
 */

#define EXTMEM_IC_PRELOAD_SIZE_FAULT_ST    (BIT(20))
#define EXTMEM_IC_PRELOAD_SIZE_FAULT_ST_M  (EXTMEM_IC_PRELOAD_SIZE_FAULT_ST_V << EXTMEM_IC_PRELOAD_SIZE_FAULT_ST_S)
#define EXTMEM_IC_PRELOAD_SIZE_FAULT_ST_V  0x00000001
#define EXTMEM_IC_PRELOAD_SIZE_FAULT_ST_S  20

/* EXTMEM_IC_SYNC_SIZE_FAULT_ST : RO; bitpos: [19]; default: 0;
 * The bit is used to indicate interrupt by manual sync configurations fault.
 */

#define EXTMEM_IC_SYNC_SIZE_FAULT_ST    (BIT(19))
#define EXTMEM_IC_SYNC_SIZE_FAULT_ST_M  (EXTMEM_IC_SYNC_SIZE_FAULT_ST_V << EXTMEM_IC_SYNC_SIZE_FAULT_ST_S)
#define EXTMEM_IC_SYNC_SIZE_FAULT_ST_V  0x00000001
#define EXTMEM_IC_SYNC_SIZE_FAULT_ST_S  19

/* EXTMEM_IC_PRELOAD_CNT_OVF_ST : RO; bitpos: [18]; default: 0;
 * The bit is used to indicate interrupt by pre-load counter overflow.
 */

#define EXTMEM_IC_PRELOAD_CNT_OVF_ST    (BIT(18))
#define EXTMEM_IC_PRELOAD_CNT_OVF_ST_M  (EXTMEM_IC_PRELOAD_CNT_OVF_ST_V << EXTMEM_IC_PRELOAD_CNT_OVF_ST_S)
#define EXTMEM_IC_PRELOAD_CNT_OVF_ST_V  0x00000001
#define EXTMEM_IC_PRELOAD_CNT_OVF_ST_S  18

/* EXTMEM_IC_PRELOAD_MISS_CNT_OVF_ST : RO; bitpos: [16]; default: 0;
 * The bit is used to indicate interrupt by pre-load miss counter overflow.
 */

#define EXTMEM_IC_PRELOAD_MISS_CNT_OVF_ST    (BIT(16))
#define EXTMEM_IC_PRELOAD_MISS_CNT_OVF_ST_M  (EXTMEM_IC_PRELOAD_MISS_CNT_OVF_ST_V << EXTMEM_IC_PRELOAD_MISS_CNT_OVF_ST_S)
#define EXTMEM_IC_PRELOAD_MISS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_IC_PRELOAD_MISS_CNT_OVF_ST_S  16

/* EXTMEM_IBUS2_ABANDON_CNT_OVF_ST : RO; bitpos: [14]; default: 0;
 * The bit is used to indicate interrupt by ibus2 abandon counter overflow.
 */

#define EXTMEM_IBUS2_ABANDON_CNT_OVF_ST    (BIT(14))
#define EXTMEM_IBUS2_ABANDON_CNT_OVF_ST_M  (EXTMEM_IBUS2_ABANDON_CNT_OVF_ST_V << EXTMEM_IBUS2_ABANDON_CNT_OVF_ST_S)
#define EXTMEM_IBUS2_ABANDON_CNT_OVF_ST_V  0x00000001
#define EXTMEM_IBUS2_ABANDON_CNT_OVF_ST_S  14

/* EXTMEM_IBUS1_ABANDON_CNT_OVF_ST : RO; bitpos: [13]; default: 0;
 * The bit is used to indicate interrupt by ibus1 abandon counter overflow.
 */

#define EXTMEM_IBUS1_ABANDON_CNT_OVF_ST    (BIT(13))
#define EXTMEM_IBUS1_ABANDON_CNT_OVF_ST_M  (EXTMEM_IBUS1_ABANDON_CNT_OVF_ST_V << EXTMEM_IBUS1_ABANDON_CNT_OVF_ST_S)
#define EXTMEM_IBUS1_ABANDON_CNT_OVF_ST_V  0x00000001
#define EXTMEM_IBUS1_ABANDON_CNT_OVF_ST_S  13

/* EXTMEM_IBUS0_ABANDON_CNT_OVF_ST : RO; bitpos: [12]; default: 0;
 * The bit is used to indicate interrupt by ibus0 abandon counter overflow.
 */

#define EXTMEM_IBUS0_ABANDON_CNT_OVF_ST    (BIT(12))
#define EXTMEM_IBUS0_ABANDON_CNT_OVF_ST_M  (EXTMEM_IBUS0_ABANDON_CNT_OVF_ST_V << EXTMEM_IBUS0_ABANDON_CNT_OVF_ST_S)
#define EXTMEM_IBUS0_ABANDON_CNT_OVF_ST_V  0x00000001
#define EXTMEM_IBUS0_ABANDON_CNT_OVF_ST_S  12

/* EXTMEM_IBUS2_ACS_MISS_CNT_OVF_ST : RO; bitpos: [10]; default: 0;
 * The bit is used to indicate interrupt by ibus2 miss counter overflow.
 */

#define EXTMEM_IBUS2_ACS_MISS_CNT_OVF_ST    (BIT(10))
#define EXTMEM_IBUS2_ACS_MISS_CNT_OVF_ST_M  (EXTMEM_IBUS2_ACS_MISS_CNT_OVF_ST_V << EXTMEM_IBUS2_ACS_MISS_CNT_OVF_ST_S)
#define EXTMEM_IBUS2_ACS_MISS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_IBUS2_ACS_MISS_CNT_OVF_ST_S  10

/* EXTMEM_IBUS1_ACS_MISS_CNT_OVF_ST : RO; bitpos: [9]; default: 0;
 * The bit is used to indicate interrupt by ibus1 miss counter overflow.
 */

#define EXTMEM_IBUS1_ACS_MISS_CNT_OVF_ST    (BIT(9))
#define EXTMEM_IBUS1_ACS_MISS_CNT_OVF_ST_M  (EXTMEM_IBUS1_ACS_MISS_CNT_OVF_ST_V << EXTMEM_IBUS1_ACS_MISS_CNT_OVF_ST_S)
#define EXTMEM_IBUS1_ACS_MISS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_IBUS1_ACS_MISS_CNT_OVF_ST_S  9

/* EXTMEM_IBUS0_ACS_MISS_CNT_OVF_ST : RO; bitpos: [8]; default: 0;
 * The bit is used to indicate interrupt by ibus0 miss counter overflow.
 */

#define EXTMEM_IBUS0_ACS_MISS_CNT_OVF_ST    (BIT(8))
#define EXTMEM_IBUS0_ACS_MISS_CNT_OVF_ST_M  (EXTMEM_IBUS0_ACS_MISS_CNT_OVF_ST_V << EXTMEM_IBUS0_ACS_MISS_CNT_OVF_ST_S)
#define EXTMEM_IBUS0_ACS_MISS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_IBUS0_ACS_MISS_CNT_OVF_ST_S  8

/* EXTMEM_IBUS2_ACS_CNT_OVF_ST : RO; bitpos: [6]; default: 0;
 * The bit is used to indicate interrupt by ibus2 counter overflow.
 */

#define EXTMEM_IBUS2_ACS_CNT_OVF_ST    (BIT(6))
#define EXTMEM_IBUS2_ACS_CNT_OVF_ST_M  (EXTMEM_IBUS2_ACS_CNT_OVF_ST_V << EXTMEM_IBUS2_ACS_CNT_OVF_ST_S)
#define EXTMEM_IBUS2_ACS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_IBUS2_ACS_CNT_OVF_ST_S  6

/* EXTMEM_IBUS1_ACS_CNT_OVF_ST : RO; bitpos: [5]; default: 0;
 * The bit is used to indicate interrupt by ibus1 counter overflow.
 */

#define EXTMEM_IBUS1_ACS_CNT_OVF_ST    (BIT(5))
#define EXTMEM_IBUS1_ACS_CNT_OVF_ST_M  (EXTMEM_IBUS1_ACS_CNT_OVF_ST_V << EXTMEM_IBUS1_ACS_CNT_OVF_ST_S)
#define EXTMEM_IBUS1_ACS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_IBUS1_ACS_CNT_OVF_ST_S  5

/* EXTMEM_IBUS0_ACS_CNT_OVF_ST : RO; bitpos: [4]; default: 0;
 * The bit is used to indicate interrupt by ibus0 counter overflow.
 */

#define EXTMEM_IBUS0_ACS_CNT_OVF_ST    (BIT(4))
#define EXTMEM_IBUS0_ACS_CNT_OVF_ST_M  (EXTMEM_IBUS0_ACS_CNT_OVF_ST_V << EXTMEM_IBUS0_ACS_CNT_OVF_ST_S)
#define EXTMEM_IBUS0_ACS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_IBUS0_ACS_CNT_OVF_ST_S  4

/* EXTMEM_IBUS2_ACS_MSK_ICACHE_ST : RO; bitpos: [2]; default: 0;
 * The bit is used to indicate interrupt by cpu access icache while the
 * ibus2 is disabled or icache is disabled which include speculative access.
 */

#define EXTMEM_IBUS2_ACS_MSK_ICACHE_ST    (BIT(2))
#define EXTMEM_IBUS2_ACS_MSK_ICACHE_ST_M  (EXTMEM_IBUS2_ACS_MSK_ICACHE_ST_V << EXTMEM_IBUS2_ACS_MSK_ICACHE_ST_S)
#define EXTMEM_IBUS2_ACS_MSK_ICACHE_ST_V  0x00000001
#define EXTMEM_IBUS2_ACS_MSK_ICACHE_ST_S  2

/* EXTMEM_IBUS1_ACS_MSK_ICACHE_ST : RO; bitpos: [1]; default: 0;
 * The bit is used to indicate interrupt by cpu access icache while the
 * ibus1 is disabled or icache is disabled which include speculative access.
 */

#define EXTMEM_IBUS1_ACS_MSK_ICACHE_ST    (BIT(1))
#define EXTMEM_IBUS1_ACS_MSK_ICACHE_ST_M  (EXTMEM_IBUS1_ACS_MSK_ICACHE_ST_V << EXTMEM_IBUS1_ACS_MSK_ICACHE_ST_S)
#define EXTMEM_IBUS1_ACS_MSK_ICACHE_ST_V  0x00000001
#define EXTMEM_IBUS1_ACS_MSK_ICACHE_ST_S  1

/* EXTMEM_IBUS0_ACS_MSK_ICACHE_ST : RO; bitpos: [0]; default: 0;
 * The bit is used to indicate interrupt by cpu access icache while the
 * ibus0 is disabled or icache is disabled which include speculative access.
 */

#define EXTMEM_IBUS0_ACS_MSK_ICACHE_ST    (BIT(0))
#define EXTMEM_IBUS0_ACS_MSK_ICACHE_ST_M  (EXTMEM_IBUS0_ACS_MSK_ICACHE_ST_V << EXTMEM_IBUS0_ACS_MSK_ICACHE_ST_S)
#define EXTMEM_IBUS0_ACS_MSK_ICACHE_ST_V  0x00000001
#define EXTMEM_IBUS0_ACS_MSK_ICACHE_ST_S  0

/* EXTMEM_CACHE_DBG_STATUS1_REG register
 * register description
 */

#define EXTMEM_CACHE_DBG_STATUS1_REG (DR_REG_EXTMEM_BASE + 0xf4)

/* EXTMEM_MMU_ENTRY_FAULT_ST : RO; bitpos: [30]; default: 0;
 * The bit is used to indicate interrupt by mmu entry fault.
 */

#define EXTMEM_MMU_ENTRY_FAULT_ST    (BIT(30))
#define EXTMEM_MMU_ENTRY_FAULT_ST_M  (EXTMEM_MMU_ENTRY_FAULT_ST_V << EXTMEM_MMU_ENTRY_FAULT_ST_S)
#define EXTMEM_MMU_ENTRY_FAULT_ST_V  0x00000001
#define EXTMEM_MMU_ENTRY_FAULT_ST_S  30

/* EXTMEM_DCACHE_SET_LOCK_ILG_ST : RO; bitpos: [29]; default: 0;
 * The bit is used to indicate interrupt by illegal writing lock registers
 * of icache while icache is busy to issue lock,sync or pre-load operations.
 */

#define EXTMEM_DCACHE_SET_LOCK_ILG_ST    (BIT(29))
#define EXTMEM_DCACHE_SET_LOCK_ILG_ST_M  (EXTMEM_DCACHE_SET_LOCK_ILG_ST_V << EXTMEM_DCACHE_SET_LOCK_ILG_ST_S)
#define EXTMEM_DCACHE_SET_LOCK_ILG_ST_V  0x00000001
#define EXTMEM_DCACHE_SET_LOCK_ILG_ST_S  29

/* EXTMEM_DCACHE_SET_SYNC_ILG_ST : RO; bitpos: [28]; default: 0;
 * The bit is used to indicate interrupt by illegal writing sync registers
 * of icache while icache is busy to issue lock,sync and pre-load operations.
 */

#define EXTMEM_DCACHE_SET_SYNC_ILG_ST    (BIT(28))
#define EXTMEM_DCACHE_SET_SYNC_ILG_ST_M  (EXTMEM_DCACHE_SET_SYNC_ILG_ST_V << EXTMEM_DCACHE_SET_SYNC_ILG_ST_S)
#define EXTMEM_DCACHE_SET_SYNC_ILG_ST_V  0x00000001
#define EXTMEM_DCACHE_SET_SYNC_ILG_ST_S  28

/* EXTMEM_DCACHE_SET_PRELOAD_ILG_ST : RO; bitpos: [27]; default: 0;
 * The bit is used to indicate interrupt by illegal writing preload
 * registers of icache while icache is busy to issue lock,sync and pre-load
 * operations.
 */

#define EXTMEM_DCACHE_SET_PRELOAD_ILG_ST    (BIT(27))
#define EXTMEM_DCACHE_SET_PRELOAD_ILG_ST_M  (EXTMEM_DCACHE_SET_PRELOAD_ILG_ST_V << EXTMEM_DCACHE_SET_PRELOAD_ILG_ST_S)
#define EXTMEM_DCACHE_SET_PRELOAD_ILG_ST_V  0x00000001
#define EXTMEM_DCACHE_SET_PRELOAD_ILG_ST_S  27

/* EXTMEM_DCACHE_REJECT_ST : RO; bitpos: [26]; default: 0;
 * The bit is used to indicate interrupt by authentication fail.
 */

#define EXTMEM_DCACHE_REJECT_ST    (BIT(26))
#define EXTMEM_DCACHE_REJECT_ST_M  (EXTMEM_DCACHE_REJECT_ST_V << EXTMEM_DCACHE_REJECT_ST_S)
#define EXTMEM_DCACHE_REJECT_ST_V  0x00000001
#define EXTMEM_DCACHE_REJECT_ST_S  26

/* EXTMEM_DCACHE_WRITE_FLASH_ST : RO; bitpos: [25]; default: 0;
 * The bit is used to indicate interrupt by dcache trying to write flash.
 */

#define EXTMEM_DCACHE_WRITE_FLASH_ST    (BIT(25))
#define EXTMEM_DCACHE_WRITE_FLASH_ST_M  (EXTMEM_DCACHE_WRITE_FLASH_ST_V << EXTMEM_DCACHE_WRITE_FLASH_ST_S)
#define EXTMEM_DCACHE_WRITE_FLASH_ST_V  0x00000001
#define EXTMEM_DCACHE_WRITE_FLASH_ST_S  25

/* EXTMEM_DC_PRELOAD_SIZE_FAULT_ST : RO; bitpos: [24]; default: 0;
 * The bit is used to indicate interrupt by manual pre-load configurations
 * fault.
 */

#define EXTMEM_DC_PRELOAD_SIZE_FAULT_ST    (BIT(24))
#define EXTMEM_DC_PRELOAD_SIZE_FAULT_ST_M  (EXTMEM_DC_PRELOAD_SIZE_FAULT_ST_V << EXTMEM_DC_PRELOAD_SIZE_FAULT_ST_S)
#define EXTMEM_DC_PRELOAD_SIZE_FAULT_ST_V  0x00000001
#define EXTMEM_DC_PRELOAD_SIZE_FAULT_ST_S  24

/* EXTMEM_DC_SYNC_SIZE_FAULT_ST : RO; bitpos: [23]; default: 0;
 * The bit is used to indicate interrupt by manual sync configurations fault.
 */

#define EXTMEM_DC_SYNC_SIZE_FAULT_ST    (BIT(23))
#define EXTMEM_DC_SYNC_SIZE_FAULT_ST_M  (EXTMEM_DC_SYNC_SIZE_FAULT_ST_V << EXTMEM_DC_SYNC_SIZE_FAULT_ST_S)
#define EXTMEM_DC_SYNC_SIZE_FAULT_ST_V  0x00000001
#define EXTMEM_DC_SYNC_SIZE_FAULT_ST_S  23

/* EXTMEM_DC_PRELOAD_CNT_OVF_ST : RO; bitpos: [22]; default: 0;
 * The bit is used to indicate interrupt by pre-load counter overflow.
 */

#define EXTMEM_DC_PRELOAD_CNT_OVF_ST    (BIT(22))
#define EXTMEM_DC_PRELOAD_CNT_OVF_ST_M  (EXTMEM_DC_PRELOAD_CNT_OVF_ST_V << EXTMEM_DC_PRELOAD_CNT_OVF_ST_S)
#define EXTMEM_DC_PRELOAD_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DC_PRELOAD_CNT_OVF_ST_S  22

/* EXTMEM_DC_PRELOAD_EVICT_CNT_OVF_ST : RO; bitpos: [21]; default: 0;
 * The bit is used to indicate interrupt by pre-load eviction counter
 * overflow.
 */

#define EXTMEM_DC_PRELOAD_EVICT_CNT_OVF_ST    (BIT(21))
#define EXTMEM_DC_PRELOAD_EVICT_CNT_OVF_ST_M  (EXTMEM_DC_PRELOAD_EVICT_CNT_OVF_ST_V << EXTMEM_DC_PRELOAD_EVICT_CNT_OVF_ST_S)
#define EXTMEM_DC_PRELOAD_EVICT_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DC_PRELOAD_EVICT_CNT_OVF_ST_S  21

/* EXTMEM_DC_PRELOAD_MISS_CNT_OVF_ST : RO; bitpos: [20]; default: 0;
 * The bit is used to indicate interrupt by pre-load miss counter overflow.
 */

#define EXTMEM_DC_PRELOAD_MISS_CNT_OVF_ST    (BIT(20))
#define EXTMEM_DC_PRELOAD_MISS_CNT_OVF_ST_M  (EXTMEM_DC_PRELOAD_MISS_CNT_OVF_ST_V << EXTMEM_DC_PRELOAD_MISS_CNT_OVF_ST_S)
#define EXTMEM_DC_PRELOAD_MISS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DC_PRELOAD_MISS_CNT_OVF_ST_S  20

/* EXTMEM_DBUS2_ABANDON_CNT_OVF_ST : RO; bitpos: [18]; default: 0;
 * The bit is used to indicate interrupt by dbus2 abandon counter overflow.
 */

#define EXTMEM_DBUS2_ABANDON_CNT_OVF_ST    (BIT(18))
#define EXTMEM_DBUS2_ABANDON_CNT_OVF_ST_M  (EXTMEM_DBUS2_ABANDON_CNT_OVF_ST_V << EXTMEM_DBUS2_ABANDON_CNT_OVF_ST_S)
#define EXTMEM_DBUS2_ABANDON_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DBUS2_ABANDON_CNT_OVF_ST_S  18

/* EXTMEM_DBUS1_ABANDON_CNT_OVF_ST : RO; bitpos: [17]; default: 0;
 * The bit is used to indicate interrupt by dbus1 abandon counter overflow.
 */

#define EXTMEM_DBUS1_ABANDON_CNT_OVF_ST    (BIT(17))
#define EXTMEM_DBUS1_ABANDON_CNT_OVF_ST_M  (EXTMEM_DBUS1_ABANDON_CNT_OVF_ST_V << EXTMEM_DBUS1_ABANDON_CNT_OVF_ST_S)
#define EXTMEM_DBUS1_ABANDON_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DBUS1_ABANDON_CNT_OVF_ST_S  17

/* EXTMEM_DBUS0_ABANDON_CNT_OVF_ST : RO; bitpos: [16]; default: 0;
 * The bit is used to indicate interrupt by dbus0 abandon counter overflow.
 */

#define EXTMEM_DBUS0_ABANDON_CNT_OVF_ST    (BIT(16))
#define EXTMEM_DBUS0_ABANDON_CNT_OVF_ST_M  (EXTMEM_DBUS0_ABANDON_CNT_OVF_ST_V << EXTMEM_DBUS0_ABANDON_CNT_OVF_ST_S)
#define EXTMEM_DBUS0_ABANDON_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DBUS0_ABANDON_CNT_OVF_ST_S  16

/* EXTMEM_DBUS2_ACS_WB_CNT_OVF_ST : RO; bitpos: [14]; default: 0;
 * The bit is used to indicate interrupt by dbus2 eviction counter overflow.
 */

#define EXTMEM_DBUS2_ACS_WB_CNT_OVF_ST    (BIT(14))
#define EXTMEM_DBUS2_ACS_WB_CNT_OVF_ST_M  (EXTMEM_DBUS2_ACS_WB_CNT_OVF_ST_V << EXTMEM_DBUS2_ACS_WB_CNT_OVF_ST_S)
#define EXTMEM_DBUS2_ACS_WB_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DBUS2_ACS_WB_CNT_OVF_ST_S  14

/* EXTMEM_DBUS1_ACS_WB_CNT_OVF_ST : RO; bitpos: [13]; default: 0;
 * The bit is used to indicate interrupt by dbus1 eviction counter overflow.
 */

#define EXTMEM_DBUS1_ACS_WB_CNT_OVF_ST    (BIT(13))
#define EXTMEM_DBUS1_ACS_WB_CNT_OVF_ST_M  (EXTMEM_DBUS1_ACS_WB_CNT_OVF_ST_V << EXTMEM_DBUS1_ACS_WB_CNT_OVF_ST_S)
#define EXTMEM_DBUS1_ACS_WB_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DBUS1_ACS_WB_CNT_OVF_ST_S  13

/* EXTMEM_DBUS0_ACS_WB_CNT_OVF_ST : RO; bitpos: [12]; default: 0;
 * The bit is used to indicate interrupt by dbus0 eviction counter overflow.
 */

#define EXTMEM_DBUS0_ACS_WB_CNT_OVF_ST    (BIT(12))
#define EXTMEM_DBUS0_ACS_WB_CNT_OVF_ST_M  (EXTMEM_DBUS0_ACS_WB_CNT_OVF_ST_V << EXTMEM_DBUS0_ACS_WB_CNT_OVF_ST_S)
#define EXTMEM_DBUS0_ACS_WB_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DBUS0_ACS_WB_CNT_OVF_ST_S  12

/* EXTMEM_DBUS2_ACS_MISS_CNT_OVF_ST : RO; bitpos: [10]; default: 0;
 * The bit is used to indicate interrupt by dbus2 miss counter overflow.
 */

#define EXTMEM_DBUS2_ACS_MISS_CNT_OVF_ST    (BIT(10))
#define EXTMEM_DBUS2_ACS_MISS_CNT_OVF_ST_M  (EXTMEM_DBUS2_ACS_MISS_CNT_OVF_ST_V << EXTMEM_DBUS2_ACS_MISS_CNT_OVF_ST_S)
#define EXTMEM_DBUS2_ACS_MISS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DBUS2_ACS_MISS_CNT_OVF_ST_S  10

/* EXTMEM_DBUS1_ACS_MISS_CNT_OVF_ST : RO; bitpos: [9]; default: 0;
 * The bit is used to indicate interrupt by dbus1 miss counter overflow.
 */

#define EXTMEM_DBUS1_ACS_MISS_CNT_OVF_ST    (BIT(9))
#define EXTMEM_DBUS1_ACS_MISS_CNT_OVF_ST_M  (EXTMEM_DBUS1_ACS_MISS_CNT_OVF_ST_V << EXTMEM_DBUS1_ACS_MISS_CNT_OVF_ST_S)
#define EXTMEM_DBUS1_ACS_MISS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DBUS1_ACS_MISS_CNT_OVF_ST_S  9

/* EXTMEM_DBUS0_ACS_MISS_CNT_OVF_ST : RO; bitpos: [8]; default: 0;
 * The bit is used to indicate interrupt by dbus0 miss counter overflow.
 */

#define EXTMEM_DBUS0_ACS_MISS_CNT_OVF_ST    (BIT(8))
#define EXTMEM_DBUS0_ACS_MISS_CNT_OVF_ST_M  (EXTMEM_DBUS0_ACS_MISS_CNT_OVF_ST_V << EXTMEM_DBUS0_ACS_MISS_CNT_OVF_ST_S)
#define EXTMEM_DBUS0_ACS_MISS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DBUS0_ACS_MISS_CNT_OVF_ST_S  8

/* EXTMEM_DBUS2_ACS_CNT_OVF_ST : RO; bitpos: [6]; default: 0;
 * The bit is used to indicate interrupt by dbus2 counter overflow.
 */

#define EXTMEM_DBUS2_ACS_CNT_OVF_ST    (BIT(6))
#define EXTMEM_DBUS2_ACS_CNT_OVF_ST_M  (EXTMEM_DBUS2_ACS_CNT_OVF_ST_V << EXTMEM_DBUS2_ACS_CNT_OVF_ST_S)
#define EXTMEM_DBUS2_ACS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DBUS2_ACS_CNT_OVF_ST_S  6

/* EXTMEM_DBUS1_ACS_CNT_OVF_ST : RO; bitpos: [5]; default: 0;
 * The bit is used to indicate interrupt by dbus1 counter overflow.
 */

#define EXTMEM_DBUS1_ACS_CNT_OVF_ST    (BIT(5))
#define EXTMEM_DBUS1_ACS_CNT_OVF_ST_M  (EXTMEM_DBUS1_ACS_CNT_OVF_ST_V << EXTMEM_DBUS1_ACS_CNT_OVF_ST_S)
#define EXTMEM_DBUS1_ACS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DBUS1_ACS_CNT_OVF_ST_S  5

/* EXTMEM_DBUS0_ACS_CNT_OVF_ST : RO; bitpos: [4]; default: 0;
 * The bit is used to indicate interrupt by dbus0 counter overflow.
 */

#define EXTMEM_DBUS0_ACS_CNT_OVF_ST    (BIT(4))
#define EXTMEM_DBUS0_ACS_CNT_OVF_ST_M  (EXTMEM_DBUS0_ACS_CNT_OVF_ST_V << EXTMEM_DBUS0_ACS_CNT_OVF_ST_S)
#define EXTMEM_DBUS0_ACS_CNT_OVF_ST_V  0x00000001
#define EXTMEM_DBUS0_ACS_CNT_OVF_ST_S  4

/* EXTMEM_DBUS2_ACS_MSK_DCACHE_ST : RO; bitpos: [2]; default: 0;
 * The bit is used to indicate interrupt by cpu access dcache while the
 * dbus2 is disabled or dcache is disabled which include speculative access.
 */

#define EXTMEM_DBUS2_ACS_MSK_DCACHE_ST    (BIT(2))
#define EXTMEM_DBUS2_ACS_MSK_DCACHE_ST_M  (EXTMEM_DBUS2_ACS_MSK_DCACHE_ST_V << EXTMEM_DBUS2_ACS_MSK_DCACHE_ST_S)
#define EXTMEM_DBUS2_ACS_MSK_DCACHE_ST_V  0x00000001
#define EXTMEM_DBUS2_ACS_MSK_DCACHE_ST_S  2

/* EXTMEM_DBUS1_ACS_MSK_DCACHE_ST : RO; bitpos: [1]; default: 0;
 * The bit is used to indicate interrupt by cpu access dcache while the
 * dbus1 is disabled or dcache is disabled which include speculative access.
 */

#define EXTMEM_DBUS1_ACS_MSK_DCACHE_ST    (BIT(1))
#define EXTMEM_DBUS1_ACS_MSK_DCACHE_ST_M  (EXTMEM_DBUS1_ACS_MSK_DCACHE_ST_V << EXTMEM_DBUS1_ACS_MSK_DCACHE_ST_S)
#define EXTMEM_DBUS1_ACS_MSK_DCACHE_ST_V  0x00000001
#define EXTMEM_DBUS1_ACS_MSK_DCACHE_ST_S  1

/* EXTMEM_DBUS0_ACS_MSK_DCACHE_ST : RO; bitpos: [0]; default: 0;
 * The bit is used to indicate interrupt by cpu access dcache while the
 * dbus0 is disabled or dcache is disabled which include speculative access.
 */

#define EXTMEM_DBUS0_ACS_MSK_DCACHE_ST    (BIT(0))
#define EXTMEM_DBUS0_ACS_MSK_DCACHE_ST_M  (EXTMEM_DBUS0_ACS_MSK_DCACHE_ST_V << EXTMEM_DBUS0_ACS_MSK_DCACHE_ST_S)
#define EXTMEM_DBUS0_ACS_MSK_DCACHE_ST_V  0x00000001
#define EXTMEM_DBUS0_ACS_MSK_DCACHE_ST_S  0

/* EXTMEM_PRO_CACHE_ACS_CNT_CLR_REG register
 * register description
 */

#define EXTMEM_PRO_CACHE_ACS_CNT_CLR_REG (DR_REG_EXTMEM_BASE + 0xf8)

/* EXTMEM_PRO_ICACHE_ACS_CNT_CLR : WOD; bitpos: [1]; default: 0;
 * The bit is used to clear icache counter which include IC_PRELOAD_CNT_REG,
 * IC_PRELOAD_MISS_CNT_REG, IBUS0-2_ABANDON_CNT_REG,
 * IBUS0-2_ACS_MISS_CNT_REG and IBUS0-2_ACS_CNT_REG.
 */

#define EXTMEM_PRO_ICACHE_ACS_CNT_CLR    (BIT(1))
#define EXTMEM_PRO_ICACHE_ACS_CNT_CLR_M  (EXTMEM_PRO_ICACHE_ACS_CNT_CLR_V << EXTMEM_PRO_ICACHE_ACS_CNT_CLR_S)
#define EXTMEM_PRO_ICACHE_ACS_CNT_CLR_V  0x00000001
#define EXTMEM_PRO_ICACHE_ACS_CNT_CLR_S  1

/* EXTMEM_PRO_DCACHE_ACS_CNT_CLR : WOD; bitpos: [0]; default: 0;
 * The bit is used to clear dcache counter which include DC_PRELOAD_CNT_REG,
 * DC_PRELOAD_EVICT_CNT_REG, DC_PRELOAD_MISS_CNT_REG,
 * DBUS0-2_ABANDON_CNT_REG, DBUS0-2_ACS_WB_CNT_REG, DBUS0-2_ACS_MISS_CNT_REG
 * and DBUS0-2_ACS_CNT_REG.
 */

#define EXTMEM_PRO_DCACHE_ACS_CNT_CLR    (BIT(0))
#define EXTMEM_PRO_DCACHE_ACS_CNT_CLR_M  (EXTMEM_PRO_DCACHE_ACS_CNT_CLR_V << EXTMEM_PRO_DCACHE_ACS_CNT_CLR_S)
#define EXTMEM_PRO_DCACHE_ACS_CNT_CLR_V  0x00000001
#define EXTMEM_PRO_DCACHE_ACS_CNT_CLR_S  0

/* EXTMEM_PRO_DCACHE_REJECT_ST_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_REJECT_ST_REG (DR_REG_EXTMEM_BASE + 0xfc)

/* EXTMEM_PRO_DCACHE_CPU_ATTR : RO; bitpos: [5:3]; default: 0;
 * The bits are used to indicate the attribute of CPU access dcache when
 * authentication fail. 0: invalidate, 1: execute-able, 2: read-able, 4:
 * write-able.
 */

#define EXTMEM_PRO_DCACHE_CPU_ATTR    0x00000007
#define EXTMEM_PRO_DCACHE_CPU_ATTR_M  (EXTMEM_PRO_DCACHE_CPU_ATTR_V << EXTMEM_PRO_DCACHE_CPU_ATTR_S)
#define EXTMEM_PRO_DCACHE_CPU_ATTR_V  0x00000007
#define EXTMEM_PRO_DCACHE_CPU_ATTR_S  3

/* EXTMEM_PRO_DCACHE_TAG_ATTR : RO; bitpos: [2:0]; default: 0;
 * The bits are used to indicate the attribute of data from external memory
 * when authentication fail. 0: invalidate, 1: execute-able, 2: read-able,
 * 4: write-able.
 */

#define EXTMEM_PRO_DCACHE_TAG_ATTR    0x00000007
#define EXTMEM_PRO_DCACHE_TAG_ATTR_M  (EXTMEM_PRO_DCACHE_TAG_ATTR_V << EXTMEM_PRO_DCACHE_TAG_ATTR_S)
#define EXTMEM_PRO_DCACHE_TAG_ATTR_V  0x00000007
#define EXTMEM_PRO_DCACHE_TAG_ATTR_S  0

/* EXTMEM_PRO_DCACHE_REJECT_VADDR_REG register
 * register description
 */

#define EXTMEM_PRO_DCACHE_REJECT_VADDR_REG (DR_REG_EXTMEM_BASE + 0x100)

/* EXTMEM_PRO_DCACHE_CPU_VADDR : RO; bitpos: [31:0]; default: 0;
 * The bits are used to indicate the virtual address of CPU access dcache
 * when authentication fail.
 */

#define EXTMEM_PRO_DCACHE_CPU_VADDR    0xffffffff
#define EXTMEM_PRO_DCACHE_CPU_VADDR_M  (EXTMEM_PRO_DCACHE_CPU_VADDR_V << EXTMEM_PRO_DCACHE_CPU_VADDR_S)
#define EXTMEM_PRO_DCACHE_CPU_VADDR_V  0xffffffff
#define EXTMEM_PRO_DCACHE_CPU_VADDR_S  0

/* EXTMEM_PRO_ICACHE_REJECT_ST_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_REJECT_ST_REG (DR_REG_EXTMEM_BASE + 0x104)

/* EXTMEM_PRO_ICACHE_CPU_ATTR : RO; bitpos: [5:3]; default: 0;
 * The bits are used to indicate the attribute of CPU access icache when
 * authentication fail. 0: invalidate, 1: execute-able, 2: read-able
 */

#define EXTMEM_PRO_ICACHE_CPU_ATTR    0x00000007
#define EXTMEM_PRO_ICACHE_CPU_ATTR_M  (EXTMEM_PRO_ICACHE_CPU_ATTR_V << EXTMEM_PRO_ICACHE_CPU_ATTR_S)
#define EXTMEM_PRO_ICACHE_CPU_ATTR_V  0x00000007
#define EXTMEM_PRO_ICACHE_CPU_ATTR_S  3

/* EXTMEM_PRO_ICACHE_TAG_ATTR : RO; bitpos: [2:0]; default: 0;
 * The bits are used to indicate the attribute of data from external memory
 * when authentication fail. 0: invalidate, 1: execute-able, 2: read-able,
 * 4: write-able.
 */

#define EXTMEM_PRO_ICACHE_TAG_ATTR    0x00000007
#define EXTMEM_PRO_ICACHE_TAG_ATTR_M  (EXTMEM_PRO_ICACHE_TAG_ATTR_V << EXTMEM_PRO_ICACHE_TAG_ATTR_S)
#define EXTMEM_PRO_ICACHE_TAG_ATTR_V  0x00000007
#define EXTMEM_PRO_ICACHE_TAG_ATTR_S  0

/* EXTMEM_PRO_ICACHE_REJECT_VADDR_REG register
 * register description
 */

#define EXTMEM_PRO_ICACHE_REJECT_VADDR_REG (DR_REG_EXTMEM_BASE + 0x108)

/* EXTMEM_PRO_ICACHE_CPU_VADDR : RO; bitpos: [31:0]; default: 0;
 * The bits are used to indicate the virtual address of CPU access icache
 * when authentication fail.
 */

#define EXTMEM_PRO_ICACHE_CPU_VADDR    0xffffffff
#define EXTMEM_PRO_ICACHE_CPU_VADDR_M  (EXTMEM_PRO_ICACHE_CPU_VADDR_V << EXTMEM_PRO_ICACHE_CPU_VADDR_S)
#define EXTMEM_PRO_ICACHE_CPU_VADDR_V  0xffffffff
#define EXTMEM_PRO_ICACHE_CPU_VADDR_S  0

/* EXTMEM_PRO_CACHE_MMU_FAULT_CONTENT_REG register
 * register description
 */

#define EXTMEM_PRO_CACHE_MMU_FAULT_CONTENT_REG (DR_REG_EXTMEM_BASE + 0x10c)

/* EXTMEM_PRO_CACHE_MMU_FAULT_CODE : RO; bitpos: [19:17]; default: 0;
 * The bits are used to indicate the operations which cause mmu fault
 * occurrence. 0: default, 1: cpu miss, 2: preload miss, 3: flush, 4: cpu
 * miss evict recovery address, 5: load miss evict recovery address, 6:
 * external dma tx, 7: external dma rx
 */

#define EXTMEM_PRO_CACHE_MMU_FAULT_CODE    0x00000007
#define EXTMEM_PRO_CACHE_MMU_FAULT_CODE_M  (EXTMEM_PRO_CACHE_MMU_FAULT_CODE_V << EXTMEM_PRO_CACHE_MMU_FAULT_CODE_S)
#define EXTMEM_PRO_CACHE_MMU_FAULT_CODE_V  0x00000007
#define EXTMEM_PRO_CACHE_MMU_FAULT_CODE_S  17

/* EXTMEM_PRO_CACHE_MMU_FAULT_CONTENT : RO; bitpos: [16:0]; default: 0;
 * The bits are used to indicate the content of mmu entry which cause mmu
 * fault..
 */

#define EXTMEM_PRO_CACHE_MMU_FAULT_CONTENT    0x0001ffff
#define EXTMEM_PRO_CACHE_MMU_FAULT_CONTENT_M  (EXTMEM_PRO_CACHE_MMU_FAULT_CONTENT_V << EXTMEM_PRO_CACHE_MMU_FAULT_CONTENT_S)
#define EXTMEM_PRO_CACHE_MMU_FAULT_CONTENT_V  0x0001ffff
#define EXTMEM_PRO_CACHE_MMU_FAULT_CONTENT_S  0

/* EXTMEM_PRO_CACHE_MMU_FAULT_VADDR_REG register
 * register description
 */

#define EXTMEM_PRO_CACHE_MMU_FAULT_VADDR_REG (DR_REG_EXTMEM_BASE + 0x110)

/* EXTMEM_PRO_CACHE_MMU_FAULT_VADDR : RO; bitpos: [31:0]; default: 0;
 * The bits are used to indicate the virtual address which cause mmu fault..
 */

#define EXTMEM_PRO_CACHE_MMU_FAULT_VADDR    0xffffffff
#define EXTMEM_PRO_CACHE_MMU_FAULT_VADDR_M  (EXTMEM_PRO_CACHE_MMU_FAULT_VADDR_V << EXTMEM_PRO_CACHE_MMU_FAULT_VADDR_S)
#define EXTMEM_PRO_CACHE_MMU_FAULT_VADDR_V  0xffffffff
#define EXTMEM_PRO_CACHE_MMU_FAULT_VADDR_S  0

/* EXTMEM_PRO_CACHE_WRAP_AROUND_CTRL_REG register
 * register description
 */

#define EXTMEM_PRO_CACHE_WRAP_AROUND_CTRL_REG (DR_REG_EXTMEM_BASE + 0x114)

/* EXTMEM_PRO_CACHE_SRAM_RD_WRAP_AROUND : R/W; bitpos: [1]; default: 0;
 * The bit is used to enable wrap around mode when read data from spiram.
 */

#define EXTMEM_PRO_CACHE_SRAM_RD_WRAP_AROUND    (BIT(1))
#define EXTMEM_PRO_CACHE_SRAM_RD_WRAP_AROUND_M  (EXTMEM_PRO_CACHE_SRAM_RD_WRAP_AROUND_V << EXTMEM_PRO_CACHE_SRAM_RD_WRAP_AROUND_S)
#define EXTMEM_PRO_CACHE_SRAM_RD_WRAP_AROUND_V  0x00000001
#define EXTMEM_PRO_CACHE_SRAM_RD_WRAP_AROUND_S  1

/* EXTMEM_PRO_CACHE_FLASH_WRAP_AROUND : R/W; bitpos: [0]; default: 0;
 * The bit is used to enable wrap around mode when read data from flash.
 */

#define EXTMEM_PRO_CACHE_FLASH_WRAP_AROUND    (BIT(0))
#define EXTMEM_PRO_CACHE_FLASH_WRAP_AROUND_M  (EXTMEM_PRO_CACHE_FLASH_WRAP_AROUND_V << EXTMEM_PRO_CACHE_FLASH_WRAP_AROUND_S)
#define EXTMEM_PRO_CACHE_FLASH_WRAP_AROUND_V  0x00000001
#define EXTMEM_PRO_CACHE_FLASH_WRAP_AROUND_S  0

/* EXTMEM_PRO_CACHE_MMU_POWER_CTRL_REG register
 * register description
 */

#define EXTMEM_PRO_CACHE_MMU_POWER_CTRL_REG (DR_REG_EXTMEM_BASE + 0x118)

/* EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PU : R/W; bitpos: [2]; default: 1;
 * The bit is used to power mmu memory down, 0: follow_rtc_lslp_pd, 1: power
 * up
 */

#define EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PU    (BIT(2))
#define EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PU_M  (EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PU_V << EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PU_S)
#define EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PU_V  0x00000001
#define EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PU_S  2

/* EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PD : R/W; bitpos: [1]; default: 0;
 * The bit is used to power mmu memory down, 0: follow_rtc_lslp_pd, 1: power
 * down
 */

#define EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PD    (BIT(1))
#define EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PD_M  (EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PD_V << EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PD_S)
#define EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PD_V  0x00000001
#define EXTMEM_PRO_CACHE_MMU_MEM_FORCE_PD_S  1

/* EXTMEM_PRO_CACHE_MMU_MEM_FORCE_ON : R/W; bitpos: [0]; default: 1;
 * The bit is used to enable clock gating to save power when access mmu
 * memory, 0: enable, 1: disable
 */

#define EXTMEM_PRO_CACHE_MMU_MEM_FORCE_ON    (BIT(0))
#define EXTMEM_PRO_CACHE_MMU_MEM_FORCE_ON_M  (EXTMEM_PRO_CACHE_MMU_MEM_FORCE_ON_V << EXTMEM_PRO_CACHE_MMU_MEM_FORCE_ON_S)
#define EXTMEM_PRO_CACHE_MMU_MEM_FORCE_ON_V  0x00000001
#define EXTMEM_PRO_CACHE_MMU_MEM_FORCE_ON_S  0

/* EXTMEM_PRO_CACHE_STATE_REG register
 * register description
 */

#define EXTMEM_PRO_CACHE_STATE_REG (DR_REG_EXTMEM_BASE + 0x11c)

/* EXTMEM_PRO_DCACHE_STATE : RO; bitpos: [23:12]; default: 0;
 * The bit is used to indicate dcache main fsm is in idle state or not. 1:
 * in idle state,  0: not in idle state
 */

#define EXTMEM_PRO_DCACHE_STATE    0x00000fff
#define EXTMEM_PRO_DCACHE_STATE_M  (EXTMEM_PRO_DCACHE_STATE_V << EXTMEM_PRO_DCACHE_STATE_S)
#define EXTMEM_PRO_DCACHE_STATE_V  0x00000fff
#define EXTMEM_PRO_DCACHE_STATE_S  12

/* EXTMEM_PRO_ICACHE_STATE : RO; bitpos: [11:0]; default: 0;
 * The bit is used to indicate icache main fsm is in idle state or not. 1:
 * in idle state,  0: not in idle state
 */

#define EXTMEM_PRO_ICACHE_STATE    0x00000fff
#define EXTMEM_PRO_ICACHE_STATE_M  (EXTMEM_PRO_ICACHE_STATE_V << EXTMEM_PRO_ICACHE_STATE_S)
#define EXTMEM_PRO_ICACHE_STATE_V  0x00000fff
#define EXTMEM_PRO_ICACHE_STATE_S  0

/* EXTMEM_CACHE_ENCRYPT_DECRYPT_RECORD_DISABLE_REG register
 * register description
 */

#define EXTMEM_CACHE_ENCRYPT_DECRYPT_RECORD_DISABLE_REG (DR_REG_EXTMEM_BASE + 0x120)

/* EXTMEM_RECORD_DISABLE_G0CB_DECRYPT : R/W; bitpos: [1]; default: 0;
 * Reserved.
 */

#define EXTMEM_RECORD_DISABLE_G0CB_DECRYPT    (BIT(1))
#define EXTMEM_RECORD_DISABLE_G0CB_DECRYPT_M  (EXTMEM_RECORD_DISABLE_G0CB_DECRYPT_V << EXTMEM_RECORD_DISABLE_G0CB_DECRYPT_S)
#define EXTMEM_RECORD_DISABLE_G0CB_DECRYPT_V  0x00000001
#define EXTMEM_RECORD_DISABLE_G0CB_DECRYPT_S  1

/* EXTMEM_RECORD_DISABLE_DB_ENCRYPT : R/W; bitpos: [0]; default: 0;
 * Reserved.
 */

#define EXTMEM_RECORD_DISABLE_DB_ENCRYPT    (BIT(0))
#define EXTMEM_RECORD_DISABLE_DB_ENCRYPT_M  (EXTMEM_RECORD_DISABLE_DB_ENCRYPT_V << EXTMEM_RECORD_DISABLE_DB_ENCRYPT_S)
#define EXTMEM_RECORD_DISABLE_DB_ENCRYPT_V  0x00000001
#define EXTMEM_RECORD_DISABLE_DB_ENCRYPT_S  0

/* EXTMEM_CACHE_ENCRYPT_DECRYPT_CLK_FORCE_ON_REG register
 * register description
 */

#define EXTMEM_CACHE_ENCRYPT_DECRYPT_CLK_FORCE_ON_REG (DR_REG_EXTMEM_BASE + 0x124)

/* EXTMEM_CLK_FORCE_ON_AUTOMATIC_ENCRYPT_DECRYPT : R/W; bitpos: [2];
 * default: 1;
 * The bit is used to close clock gating of encrypt and decrypt clock. 1:
 * close gating, 0: open clock gating.
 */

#define EXTMEM_CLK_FORCE_ON_AUTOMATIC_ENCRYPT_DECRYPT    (BIT(2))
#define EXTMEM_CLK_FORCE_ON_AUTOMATIC_ENCRYPT_DECRYPT_M  (EXTMEM_CLK_FORCE_ON_AUTOMATIC_ENCRYPT_DECRYPT_V << EXTMEM_CLK_FORCE_ON_AUTOMATIC_ENCRYPT_DECRYPT_S)
#define EXTMEM_CLK_FORCE_ON_AUTOMATIC_ENCRYPT_DECRYPT_V  0x00000001
#define EXTMEM_CLK_FORCE_ON_AUTOMATIC_ENCRYPT_DECRYPT_S  2

/* EXTMEM_CLK_FORCE_ON_G0CB_DECRYPT : R/W; bitpos: [1]; default: 1;
 * The bit is used to close clock gating of decrypt clock. 1: close gating,
 * 0: open clock gating.
 */

#define EXTMEM_CLK_FORCE_ON_G0CB_DECRYPT    (BIT(1))
#define EXTMEM_CLK_FORCE_ON_G0CB_DECRYPT_M  (EXTMEM_CLK_FORCE_ON_G0CB_DECRYPT_V << EXTMEM_CLK_FORCE_ON_G0CB_DECRYPT_S)
#define EXTMEM_CLK_FORCE_ON_G0CB_DECRYPT_V  0x00000001
#define EXTMEM_CLK_FORCE_ON_G0CB_DECRYPT_S  1

/* EXTMEM_CLK_FORCE_ON_DB_ENCRYPT : R/W; bitpos: [0]; default: 1;
 * The bit is used to close clock gating of encrypt clock. 1: close gating,
 * 0: open clock gating.
 */

#define EXTMEM_CLK_FORCE_ON_DB_ENCRYPT    (BIT(0))
#define EXTMEM_CLK_FORCE_ON_DB_ENCRYPT_M  (EXTMEM_CLK_FORCE_ON_DB_ENCRYPT_V << EXTMEM_CLK_FORCE_ON_DB_ENCRYPT_S)
#define EXTMEM_CLK_FORCE_ON_DB_ENCRYPT_V  0x00000001
#define EXTMEM_CLK_FORCE_ON_DB_ENCRYPT_S  0

/* EXTMEM_CACHE_BRIDGE_ARBITER_CTRL_REG register
 * register description
 */

#define EXTMEM_CACHE_BRIDGE_ARBITER_CTRL_REG (DR_REG_EXTMEM_BASE + 0x128)

/* EXTMEM_ALLOC_WB_HOLD_ARBITER : R/W; bitpos: [0]; default: 0;
 * Reserved.
 */

#define EXTMEM_ALLOC_WB_HOLD_ARBITER    (BIT(0))
#define EXTMEM_ALLOC_WB_HOLD_ARBITER_M  (EXTMEM_ALLOC_WB_HOLD_ARBITER_V << EXTMEM_ALLOC_WB_HOLD_ARBITER_S)
#define EXTMEM_ALLOC_WB_HOLD_ARBITER_V  0x00000001
#define EXTMEM_ALLOC_WB_HOLD_ARBITER_S  0

/* EXTMEM_CACHE_PRELOAD_INT_CTRL_REG register
 * register description
 */

#define EXTMEM_CACHE_PRELOAD_INT_CTRL_REG (DR_REG_EXTMEM_BASE + 0x12c)

/* EXTMEM_PRO_DCACHE_PRELOAD_INT_CLR : WOD; bitpos: [5]; default: 0;
 * The bit is used to clear the interrupt by dcache pre-load done.
 */

#define EXTMEM_PRO_DCACHE_PRELOAD_INT_CLR    (BIT(5))
#define EXTMEM_PRO_DCACHE_PRELOAD_INT_CLR_M  (EXTMEM_PRO_DCACHE_PRELOAD_INT_CLR_V << EXTMEM_PRO_DCACHE_PRELOAD_INT_CLR_S)
#define EXTMEM_PRO_DCACHE_PRELOAD_INT_CLR_V  0x00000001
#define EXTMEM_PRO_DCACHE_PRELOAD_INT_CLR_S  5

/* EXTMEM_PRO_DCACHE_PRELOAD_INT_ENA : R/W; bitpos: [4]; default: 0;
 * The bit is used to enable the interrupt by dcache pre-load done.
 */

#define EXTMEM_PRO_DCACHE_PRELOAD_INT_ENA    (BIT(4))
#define EXTMEM_PRO_DCACHE_PRELOAD_INT_ENA_M  (EXTMEM_PRO_DCACHE_PRELOAD_INT_ENA_V << EXTMEM_PRO_DCACHE_PRELOAD_INT_ENA_S)
#define EXTMEM_PRO_DCACHE_PRELOAD_INT_ENA_V  0x00000001
#define EXTMEM_PRO_DCACHE_PRELOAD_INT_ENA_S  4

/* EXTMEM_PRO_DCACHE_PRELOAD_INT_ST : RO; bitpos: [3]; default: 0;
 * The bit is used to indicate the interrupt by dcache pre-load done.
 */

#define EXTMEM_PRO_DCACHE_PRELOAD_INT_ST    (BIT(3))
#define EXTMEM_PRO_DCACHE_PRELOAD_INT_ST_M  (EXTMEM_PRO_DCACHE_PRELOAD_INT_ST_V << EXTMEM_PRO_DCACHE_PRELOAD_INT_ST_S)
#define EXTMEM_PRO_DCACHE_PRELOAD_INT_ST_V  0x00000001
#define EXTMEM_PRO_DCACHE_PRELOAD_INT_ST_S  3

/* EXTMEM_PRO_ICACHE_PRELOAD_INT_CLR : WOD; bitpos: [2]; default: 0;
 * The bit is used to clear the interrupt by icache pre-load done.
 */

#define EXTMEM_PRO_ICACHE_PRELOAD_INT_CLR    (BIT(2))
#define EXTMEM_PRO_ICACHE_PRELOAD_INT_CLR_M  (EXTMEM_PRO_ICACHE_PRELOAD_INT_CLR_V << EXTMEM_PRO_ICACHE_PRELOAD_INT_CLR_S)
#define EXTMEM_PRO_ICACHE_PRELOAD_INT_CLR_V  0x00000001
#define EXTMEM_PRO_ICACHE_PRELOAD_INT_CLR_S  2

/* EXTMEM_PRO_ICACHE_PRELOAD_INT_ENA : R/W; bitpos: [1]; default: 0;
 * The bit is used to enable the interrupt by icache pre-load done.
 */

#define EXTMEM_PRO_ICACHE_PRELOAD_INT_ENA    (BIT(1))
#define EXTMEM_PRO_ICACHE_PRELOAD_INT_ENA_M  (EXTMEM_PRO_ICACHE_PRELOAD_INT_ENA_V << EXTMEM_PRO_ICACHE_PRELOAD_INT_ENA_S)
#define EXTMEM_PRO_ICACHE_PRELOAD_INT_ENA_V  0x00000001
#define EXTMEM_PRO_ICACHE_PRELOAD_INT_ENA_S  1

/* EXTMEM_PRO_ICACHE_PRELOAD_INT_ST : RO; bitpos: [0]; default: 0;
 * The bit is used to indicate the interrupt by icache pre-load done.
 */

#define EXTMEM_PRO_ICACHE_PRELOAD_INT_ST    (BIT(0))
#define EXTMEM_PRO_ICACHE_PRELOAD_INT_ST_M  (EXTMEM_PRO_ICACHE_PRELOAD_INT_ST_V << EXTMEM_PRO_ICACHE_PRELOAD_INT_ST_S)
#define EXTMEM_PRO_ICACHE_PRELOAD_INT_ST_V  0x00000001
#define EXTMEM_PRO_ICACHE_PRELOAD_INT_ST_S  0

/* EXTMEM_CACHE_SYNC_INT_CTRL_REG register
 * register description
 */

#define EXTMEM_CACHE_SYNC_INT_CTRL_REG (DR_REG_EXTMEM_BASE + 0x130)

/* EXTMEM_PRO_DCACHE_SYNC_INT_CLR : WOD; bitpos: [5]; default: 0;
 * The bit is used to clear the interrupt by dcache sync done.
 */

#define EXTMEM_PRO_DCACHE_SYNC_INT_CLR    (BIT(5))
#define EXTMEM_PRO_DCACHE_SYNC_INT_CLR_M  (EXTMEM_PRO_DCACHE_SYNC_INT_CLR_V << EXTMEM_PRO_DCACHE_SYNC_INT_CLR_S)
#define EXTMEM_PRO_DCACHE_SYNC_INT_CLR_V  0x00000001
#define EXTMEM_PRO_DCACHE_SYNC_INT_CLR_S  5

/* EXTMEM_PRO_DCACHE_SYNC_INT_ENA : R/W; bitpos: [4]; default: 0;
 * The bit is used to enable the interrupt by dcache sync done.
 */

#define EXTMEM_PRO_DCACHE_SYNC_INT_ENA    (BIT(4))
#define EXTMEM_PRO_DCACHE_SYNC_INT_ENA_M  (EXTMEM_PRO_DCACHE_SYNC_INT_ENA_V << EXTMEM_PRO_DCACHE_SYNC_INT_ENA_S)
#define EXTMEM_PRO_DCACHE_SYNC_INT_ENA_V  0x00000001
#define EXTMEM_PRO_DCACHE_SYNC_INT_ENA_S  4

/* EXTMEM_PRO_DCACHE_SYNC_INT_ST : RO; bitpos: [3]; default: 0;
 * The bit is used to indicate the interrupt by dcache sync done.
 */

#define EXTMEM_PRO_DCACHE_SYNC_INT_ST    (BIT(3))
#define EXTMEM_PRO_DCACHE_SYNC_INT_ST_M  (EXTMEM_PRO_DCACHE_SYNC_INT_ST_V << EXTMEM_PRO_DCACHE_SYNC_INT_ST_S)
#define EXTMEM_PRO_DCACHE_SYNC_INT_ST_V  0x00000001
#define EXTMEM_PRO_DCACHE_SYNC_INT_ST_S  3

/* EXTMEM_PRO_ICACHE_SYNC_INT_CLR : WOD; bitpos: [2]; default: 0;
 * The bit is used to clear the interrupt by icache sync done.
 */

#define EXTMEM_PRO_ICACHE_SYNC_INT_CLR    (BIT(2))
#define EXTMEM_PRO_ICACHE_SYNC_INT_CLR_M  (EXTMEM_PRO_ICACHE_SYNC_INT_CLR_V << EXTMEM_PRO_ICACHE_SYNC_INT_CLR_S)
#define EXTMEM_PRO_ICACHE_SYNC_INT_CLR_V  0x00000001
#define EXTMEM_PRO_ICACHE_SYNC_INT_CLR_S  2

/* EXTMEM_PRO_ICACHE_SYNC_INT_ENA : R/W; bitpos: [1]; default: 0;
 * The bit is used to enable the interrupt by icache sync done.
 */

#define EXTMEM_PRO_ICACHE_SYNC_INT_ENA    (BIT(1))
#define EXTMEM_PRO_ICACHE_SYNC_INT_ENA_M  (EXTMEM_PRO_ICACHE_SYNC_INT_ENA_V << EXTMEM_PRO_ICACHE_SYNC_INT_ENA_S)
#define EXTMEM_PRO_ICACHE_SYNC_INT_ENA_V  0x00000001
#define EXTMEM_PRO_ICACHE_SYNC_INT_ENA_S  1

/* EXTMEM_PRO_ICACHE_SYNC_INT_ST : RO; bitpos: [0]; default: 0;
 * The bit is used to indicate the interrupt by icache sync done.
 */

#define EXTMEM_PRO_ICACHE_SYNC_INT_ST    (BIT(0))
#define EXTMEM_PRO_ICACHE_SYNC_INT_ST_M  (EXTMEM_PRO_ICACHE_SYNC_INT_ST_V << EXTMEM_PRO_ICACHE_SYNC_INT_ST_S)
#define EXTMEM_PRO_ICACHE_SYNC_INT_ST_V  0x00000001
#define EXTMEM_PRO_ICACHE_SYNC_INT_ST_S  0

/* EXTMEM_CACHE_CONF_MISC_REG register
 * register description
 */

#define EXTMEM_CACHE_CONF_MISC_REG (DR_REG_EXTMEM_BASE + 0x134)

/* EXTMEM_PRO_CACHE_IGNORE_SYNC_MMU_ENTRY_FAULT : R/W; bitpos: [1]; default:
 * 1;
 * The bit is used to disable checking mmu entry fault by sync operation.
 */

#define EXTMEM_PRO_CACHE_IGNORE_SYNC_MMU_ENTRY_FAULT    (BIT(1))
#define EXTMEM_PRO_CACHE_IGNORE_SYNC_MMU_ENTRY_FAULT_M  (EXTMEM_PRO_CACHE_IGNORE_SYNC_MMU_ENTRY_FAULT_V << EXTMEM_PRO_CACHE_IGNORE_SYNC_MMU_ENTRY_FAULT_S)
#define EXTMEM_PRO_CACHE_IGNORE_SYNC_MMU_ENTRY_FAULT_V  0x00000001
#define EXTMEM_PRO_CACHE_IGNORE_SYNC_MMU_ENTRY_FAULT_S  1

/* EXTMEM_PRO_CACHE_IGNORE_PRELOAD_MMU_ENTRY_FAULT : R/W; bitpos: [0];
 * default: 1;
 * The bit is used to disable checking mmu entry fault by preload operation.
 */

#define EXTMEM_PRO_CACHE_IGNORE_PRELOAD_MMU_ENTRY_FAULT    (BIT(0))
#define EXTMEM_PRO_CACHE_IGNORE_PRELOAD_MMU_ENTRY_FAULT_M  (EXTMEM_PRO_CACHE_IGNORE_PRELOAD_MMU_ENTRY_FAULT_V << EXTMEM_PRO_CACHE_IGNORE_PRELOAD_MMU_ENTRY_FAULT_S)
#define EXTMEM_PRO_CACHE_IGNORE_PRELOAD_MMU_ENTRY_FAULT_V  0x00000001
#define EXTMEM_PRO_CACHE_IGNORE_PRELOAD_MMU_ENTRY_FAULT_S  0

/* EXTMEM_CLOCK_GATE_REG register
 * register description
 */

#define EXTMEM_CLOCK_GATE_REG (DR_REG_EXTMEM_BASE + 0x138)

/* EXTMEM_CLK_EN : R/W; bitpos: [0]; default: 1;
 * Reserved.
 */

#define EXTMEM_CLK_EN    (BIT(0))
#define EXTMEM_CLK_EN_M  (EXTMEM_CLK_EN_V << EXTMEM_CLK_EN_S)
#define EXTMEM_CLK_EN_V  0x00000001
#define EXTMEM_CLK_EN_S  0

/* EXTMEM_PRO_EXTMEM_REG_DATE_REG register
 * register description
 */

#define EXTMEM_PRO_EXTMEM_REG_DATE_REG (DR_REG_EXTMEM_BASE + 0x3fc)

/* EXTMEM_PRO_EXTMEM_REG_DATE : R/W; bitpos: [27:0]; default: 26231168;
 * Reserved.
 */

#define EXTMEM_PRO_EXTMEM_REG_DATE    0x0fffffff
#define EXTMEM_PRO_EXTMEM_REG_DATE_M  (EXTMEM_PRO_EXTMEM_REG_DATE_V << EXTMEM_PRO_EXTMEM_REG_DATE_S)
#define EXTMEM_PRO_EXTMEM_REG_DATE_V  0x0fffffff
#define EXTMEM_PRO_EXTMEM_REG_DATE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_EXTMEM_H */
