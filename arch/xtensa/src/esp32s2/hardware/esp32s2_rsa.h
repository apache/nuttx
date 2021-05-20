/****************************************************************************
 * arch/xtensa/src/esp32s2/hardware/esp32s2_rsa.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_RSA_H
#define __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_RSA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32s2_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RSA_M_PRIME_REG register
 * Register to store M'
 */

#define RSA_M_PRIME_REG (DR_REG_RSA_BASE + 0x800)

/* RSA_M_PRIME : R/W; bitpos: [31:0]; default: 0;
 * Stores M'
 */

#define RSA_M_PRIME    0xFFFFFFFF
#define RSA_M_PRIME_M  (RSA_M_PRIME_V << RSA_M_PRIME_S)
#define RSA_M_PRIME_V  0xFFFFFFFF
#define RSA_M_PRIME_S  0

/* RSA_MODE_REG register
 * RSA length mode
 */

#define RSA_MODE_REG (DR_REG_RSA_BASE + 0x804)

/* RSA_MODE : R/W; bitpos: [6:0]; default: 0;
 * Stores the mode of modular exponentiation.
 */

#define RSA_MODE    0x0000007F
#define RSA_MODE_M  (RSA_MODE_V << RSA_MODE_S)
#define RSA_MODE_V  0x0000007F
#define RSA_MODE_S  0

/* RSA_CLEAN_REG register
 * RSA clean register
 */

#define RSA_CLEAN_REG (DR_REG_RSA_BASE + 0x808)

/* RSA_CLEAN : RO; bitpos: [0]; default: 0;
 * The content of this bit is 1 when memories complete initialization.
 */

#define RSA_CLEAN    (BIT(0))
#define RSA_CLEAN_M  (RSA_CLEAN_V << RSA_CLEAN_S)
#define RSA_CLEAN_V  0x00000001
#define RSA_CLEAN_S  0

/* RSA_MODEXP_START_REG register
 * Modular exponentiation starting bit
 */

#define RSA_MODEXP_START_REG (DR_REG_RSA_BASE + 0x80c)

/* RSA_MODEXP_START : WO; bitpos: [0]; default: 0;
 * Set this bit to 1 to start the modular exponentiation.
 */

#define RSA_MODEXP_START    (BIT(0))
#define RSA_MODEXP_START_M  (RSA_MODEXP_START_V << RSA_MODEXP_START_S)
#define RSA_MODEXP_START_V  0x00000001
#define RSA_MODEXP_START_S  0

/* RSA_MODMULT_START_REG register
 * Modular multiplication starting bit
 */

#define RSA_MODMULT_START_REG (DR_REG_RSA_BASE + 0x810)

/* RSA_MODMULT_START : WO; bitpos: [0]; default: 0;
 * Set this bit to 1 to start the modular multiplication.
 */

#define RSA_MODMULT_START    (BIT(0))
#define RSA_MODMULT_START_M  (RSA_MODMULT_START_V << RSA_MODMULT_START_S)
#define RSA_MODMULT_START_V  0x00000001
#define RSA_MODMULT_START_S  0

/* RSA_MULT_START_REG register
 * Normal multiplicaiton starting bit
 */

#define RSA_MULT_START_REG (DR_REG_RSA_BASE + 0x814)

/* RSA_MULT_START : WO; bitpos: [0]; default: 0;
 * Set this bit to 1 to start the multiplication.
 */

#define RSA_MULT_START    (BIT(0))
#define RSA_MULT_START_M  (RSA_MULT_START_V << RSA_MULT_START_S)
#define RSA_MULT_START_V  0x00000001
#define RSA_MULT_START_S  0

/* RSA_IDLE_REG register
 * RSA idle register
 */

#define RSA_IDLE_REG (DR_REG_RSA_BASE + 0x818)

/* RSA_IDLE : RO; bitpos: [0]; default: 0;
 * The content of this bit is 1 when the RSA accelerator is idle.
 */

#define RSA_IDLE    (BIT(0))
#define RSA_IDLE_M  (RSA_IDLE_V << RSA_IDLE_S)
#define RSA_IDLE_V  0x00000001
#define RSA_IDLE_S  0

/* RSA_CLEAR_INTERRUPT_REG register
 * RSA clear interrupt register
 */

#define RSA_CLEAR_INTERRUPT_REG (DR_REG_RSA_BASE + 0x81c)

/* RSA_CLEAR_INTERRUPT : WO; bitpos: [0]; default: 0;
 * Set this bit to 1 to clear the RSA interrupts.
 */

#define RSA_CLEAR_INTERRUPT    (BIT(0))
#define RSA_CLEAR_INTERRUPT_M  (RSA_CLEAR_INTERRUPT_V << RSA_CLEAR_INTERRUPT_S)
#define RSA_CLEAR_INTERRUPT_V  0x00000001
#define RSA_CLEAR_INTERRUPT_S  0

/* RSA_CONSTANT_TIME_REG register
 * The constant_time option
 */

#define RSA_CONSTANT_TIME_REG (DR_REG_RSA_BASE + 0x820)

/* RSA_CONSTANT_TIME : R/W; bitpos: [0]; default: 1;
 * Set this bit to 0 to enable the acceleration option of constant_time for
 * modular exponentiation. Set to 1 to disable the acceleration (by default).
 */

#define RSA_CONSTANT_TIME    (BIT(0))
#define RSA_CONSTANT_TIME_M  (RSA_CONSTANT_TIME_V << RSA_CONSTANT_TIME_S)
#define RSA_CONSTANT_TIME_V  0x00000001
#define RSA_CONSTANT_TIME_S  0

/* RSA_SEARCH_ENABLE_REG register
 * The search option
 */

#define RSA_SEARCH_ENABLE_REG (DR_REG_RSA_BASE + 0x824)

/* RSA_SEARCH_ENABLE : R/W; bitpos: [0]; default: 0;
 * Set this bit to 1 to enable the acceleration option of search for modular
 * exponentiation. Set to 0 to disable the acceleration (by default).
 */

#define RSA_SEARCH_ENABLE    (BIT(0))
#define RSA_SEARCH_ENABLE_M  (RSA_SEARCH_ENABLE_V << RSA_SEARCH_ENABLE_S)
#define RSA_SEARCH_ENABLE_V  0x00000001
#define RSA_SEARCH_ENABLE_S  0

/* RSA_SEARCH_POS_REG register
 * The search position
 */

#define RSA_SEARCH_POS_REG (DR_REG_RSA_BASE + 0x828)

/* RSA_SEARCH_POS : R/W; bitpos: [11:0]; default: 0;
 * Is used to configure the starting address when the acceleration option of
 * search is used.
 */

#define RSA_SEARCH_POS    0x00000FFF
#define RSA_SEARCH_POS_M  (RSA_SEARCH_POS_V << RSA_SEARCH_POS_S)
#define RSA_SEARCH_POS_V  0x00000FFF
#define RSA_SEARCH_POS_S  0

/* RSA_INTERRUPT_ENA_REG register
 * RSA interrupt enable register
 */

#define RSA_INTERRUPT_ENA_REG (DR_REG_RSA_BASE + 0x82c)

/* RSA_INTERRUPT_ENA : R/W; bitpos: [0]; default: 0;
 * Set this bit to 1 to enable the RSA interrupt. This option is enabled by
 * default.
 */

#define RSA_INTERRUPT_ENA    (BIT(0))
#define RSA_INTERRUPT_ENA_M  (RSA_INTERRUPT_ENA_V << RSA_INTERRUPT_ENA_S)
#define RSA_INTERRUPT_ENA_V  0x00000001
#define RSA_INTERRUPT_ENA_S  0

/* RSA_DATE_REG register
 * Version control register
 */

#define RSA_DATE_REG (DR_REG_RSA_BASE + 0x830)

/* RSA_DATE : R/W; bitpos: [29:0]; default: 538510373;
 * Version control register
 */

#define RSA_DATE    0x3FFFFFFF
#define RSA_DATE_M  (RSA_DATE_V << RSA_DATE_S)
#define RSA_DATE_V  0x3FFFFFFF
#define RSA_DATE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_RSA_H */
