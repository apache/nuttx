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

#define EXTMEM_PRO_ICACHE_CTRL1_REG         (DR_REG_EXTMEM_BASE + 0x044)

/* EXTMEM_PRO_ICACHE_MASK_BUS2 : R/W ;bitpos:[2] ;default: 1'b1 ;
 * description: The bit is used to disable ibus2
 * 0: enable  1: disable
 */

#define EXTMEM_PRO_ICACHE_MASK_BUS2         (BIT(2))
#define EXTMEM_PRO_ICACHE_MASK_BUS2_M       (BIT(2))
#define EXTMEM_PRO_ICACHE_MASK_BUS2_V       0x1
#define EXTMEM_PRO_ICACHE_MASK_BUS2_S       2

/* EXTMEM_PRO_ICACHE_MASK_BUS1 : R/W ;bitpos:[1] ;default: 1'b1 ;
 * description: The bit is used to disable ibus1
 * 0: enable  1: disable
 */

#define EXTMEM_PRO_ICACHE_MASK_BUS1         (BIT(1))
#define EXTMEM_PRO_ICACHE_MASK_BUS1_M       (BIT(1))
#define EXTMEM_PRO_ICACHE_MASK_BUS1_V       0x1
#define EXTMEM_PRO_ICACHE_MASK_BUS1_S       1

/* EXTMEM_PRO_ICACHE_MASK_BUS0 : R/W ;bitpos:[0] ;default: 1'b1 ;
 * description: The bit is used to disable ibus0
 * 0: enable  1: disable
 */

#define EXTMEM_PRO_ICACHE_MASK_BUS0         (BIT(0))
#define EXTMEM_PRO_ICACHE_MASK_BUS0_M       (BIT(0))
#define EXTMEM_PRO_ICACHE_MASK_BUS0_V       0x1
#define EXTMEM_PRO_ICACHE_MASK_BUS0_S       0
#define EXTMEM_PRO_ICACHE_MASK_IRAM0        EXTMEM_PRO_ICACHE_MASK_BUS0
#define EXTMEM_PRO_ICACHE_MASK_IRAM1        EXTMEM_PRO_ICACHE_MASK_BUS1
#define EXTMEM_PRO_ICACHE_MASK_DROM0        EXTMEM_PRO_ICACHE_MASK_BUS2

#define EXTMEM_PRO_DCACHE_CTRL1_REG          (DR_REG_EXTMEM_BASE + 0x004)

/* EXTMEM_PRO_DCACHE_MASK_BUS2 : R/W ;bitpos:[2] ;default: 1'b1 ; */

/* Description: The bit is used to disable dbus2  0: enable  1: disable */

#define EXTMEM_PRO_DCACHE_MASK_BUS2  (BIT(2))
#define EXTMEM_PRO_DCACHE_MASK_BUS2_M  (BIT(2))
#define EXTMEM_PRO_DCACHE_MASK_BUS2_V  0x1
#define EXTMEM_PRO_DCACHE_MASK_BUS2_S  2

/* EXTMEM_PRO_DCACHE_MASK_BUS1 : R/W ;bitpos:[1] ;default: 1'b1 ; */

/* Description: The bit is used to disable dbus1  0: enable  1: disable */

#define EXTMEM_PRO_DCACHE_MASK_BUS1  (BIT(1))
#define EXTMEM_PRO_DCACHE_MASK_BUS1_M  (BIT(1))
#define EXTMEM_PRO_DCACHE_MASK_BUS1_V  0x1
#define EXTMEM_PRO_DCACHE_MASK_BUS1_S  1

/* EXTMEM_PRO_DCACHE_MASK_BUS0 : R/W ;bitpos:[0] ;default: 1'b1 ; */

/* description: The bit is used to disable dbus0  0: enable  1: disable */

#define EXTMEM_PRO_DCACHE_MASK_BUS0  (BIT(0))
#define EXTMEM_PRO_DCACHE_MASK_BUS0_M  (BIT(0))
#define EXTMEM_PRO_DCACHE_MASK_BUS0_V  0x1
#define EXTMEM_PRO_DCACHE_MASK_BUS0_S  0
#define EXTMEM_PRO_DCACHE_MASK_DRAM0 EXTMEM_PRO_DCACHE_MASK_BUS0
#define EXTMEM_PRO_DCACHE_MASK_DRAM1 EXTMEM_PRO_DCACHE_MASK_BUS1
#define EXTMEM_PRO_DCACHE_MASK_DPORT EXTMEM_PRO_DCACHE_MASK_BUS2

#endif /* __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_EXTMEM_H */
