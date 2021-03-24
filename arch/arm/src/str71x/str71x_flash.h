/****************************************************************************
 * arch/arm/src/str71x/str71x_flash.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_FLASH_H
#define __ARCH_ARM_SRC_STR71X_STR71X_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash registers **********************************************************/

#define STR71X_FLASH_CR0       (STR71X_FLASHREG_BASE + 0x0000) /* 32-bits wide */
#define STR71X_FLASH_CR1       (STR71X_FLASHREG_BASE + 0x0004) /* 32-bits wide */
#define STR71X_FLASH_DR0       (STR71X_FLASHREG_BASE + 0x0008) /* 32-bits wide */
#define STR71X_FLASH_DR1       (STR71X_FLASHREG_BASE + 0x000c) /* 32-bits wide */
#define STR71X_FLASH_AR        (STR71X_FLASHREG_BASE + 0x0010) /* 32-bits wide */
#define STR71X_FLASH_ER        (STR71X_FLASHREG_BASE + 0x0014) /* 32-bits wide */

/* Register bit settings ****************************************************/

#define STR71X_FLASH_B0F0       (0x00000001)
#define STR71X_FLASH_B0F1       (0x00000002)
#define STR71X_FLASH_B0F2       (0x00000004)
#define STR71X_FLASH_B0F3       (0x00000008)
#define STR71X_FLASH_B0F4       (0x00000010)
#define STR71X_FLASH_B0F5       (0x00000020)
#define STR71X_FLASH_B0F6       (0x00000040)
#define STR71X_FLASH_B0F7       (0x00000080)

#define STR71X_FLASH_B1F0       (0x00010000)
#define STR71X_FLASH_B1F1       (0x00020000)

#define STR71X_FLASH_B0         (STR71X_FLASH_B0F0|STR71X_FLASH_B0F1|\
                                  STR71X_FLASH_B0F2|STR71X_FLASH_B0F3|\
                                  STR71X_FLASH_B0F4|STR71X_FLASH_B0F5|\
                                  STR71X_FLASH_B0F6| STR71X_FLASH_B0F7)
#define STR71X_FLASH_B1         (STR71X_FLASH_B1F0|STR71X_FLASH_B1F1)

#define STR71X_FLASH_BANK0      (0x1000000)
#define STR71X_FLASH_BANK1      (0x2000000)

#define STR71X_FLASH_BSYA0      (0x01) /* 000-00001 (0000 0001 (0x01 */ /* STR71X_FLASH_CR0.1 */
#define STR71X_FLASH_BSYA1      (0x02) /* 000-00010 (0000 0010 (0x02 */ /* STR71X_FLASH_CR0.2 */
#define STR71X_FLASH_LOCK       (0x04) /* 000-00100 (0000 0100 (0x04 */ /* STR71X_FLASH_CR0.4 */
#define STR71X_FLASH_INTP       (0x14) /* 000-10100 (0001 0100 (0x14 */ /* STR71X_FLASH_CR0.20 */
#define STR71X_FLASH_B0S        (0x38) /* 001-11000 (0011 1000 (0x38 */ /* STR71X_FLASH_CR1.24 */
#define STR71X_FLASH_B1S        (0x39) /* 001-11001 (0011 1001 (0x39 */ /* STR71X_FLASH_CR1.25 */
#define STR71X_FLASH_ERR        (0xa0) /* 101-00000 (1010 0000 (0xA0 */ /* STR71X_FLASH_ER.0 */
#define STR71X_FLASH_ERER       (0xa1) /* 101-00001 (1010 0001 (0xA1 */ /* STR71X_FLASH_ER.1 */
#define STR71X_FLASH_PGER       (0xa2) /* 101-00010 (1010 0010 (0xA2 */ /* STR71X_FLASH_ER.2 */
#define STR71X_FLASH_10ER       (0xa3) /* 101-00011 (1010 0011 (0xA3 */ /* STR71X_FLASH_ER.3 */
#define STR71X_FLASH_SEQER      (0xa6) /* 101-00110 (1010 0110 (0xA6 */ /* STR71X_FLASH_ER.6 */
#define STR71X_FLASH_RESER      (0xa7) /* 101-00111 (1010 0111 (0xA7 */ /* STR71X_FLASH_ER.7 */
#define STR71X_FLASH_WPF        (0xa8) /* 101-01000 (1010 1000 (0xA8 */ /* STR71X_FLASH_ER.8 */

#define STR71X_FLASH_WMS_MASK   (0x80000000)
#define STR71X_FLASH_SUSP_MASK  (0x40000000)
#define STR71X_FLASH_WPG_MASK   (0x20000000)
#define STR71X_FLASH_DWPG_MASK  (0x10000000)
#define STR71X_FLASH_SER_MASK   (0x08000000)
#define STR71X_FLASH_SPR_MASK   (0x01000000)
#define STR71X_FLASH_DBGP_MASK  (0x00000002)
#define STR71X_FLASH_ACCP_MASK  (0x00000001)

#define STR71X_FLASH_Reg_Mask   (0xe0)
#define STR71X_FLASH_Flag_Mask  (0x1f)

#define STR71X_FLASH_INTM_Mask  (0x00200000)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_FLASH_H */
