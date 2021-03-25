/****************************************************************************
 * arch/arm/src/str71x/str71x_i2c.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_I2C_H
#define __ARCH_ARM_SRC_STR71X_STR71X_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define STR71X_I2C_CR_OFFSET   (0x0000) /* 8-bits wide */
#define STR71X_I2C_SR1_OFFSET  (0x0004) /* 8-bits wide */
#define STR71X_I2C_SR2_OFFSET  (0x0008) /* 8-bits wide */
#define STR71X_I2C_CCR_OFFSET  (0x000c) /* 8-bits wide */
#define STR71X_I2C_OAR1_OFFSET (0x0010) /* 8-bits wide */
#define STR71X_I2C_OAR2_OFFSET (0x0014) /* 8-bits wide */
#define STR71X_I2C_DR_OFFSET   (0x0018) /* 8-bits wide */
#define STR71X_I2C_ECCR_OFFSET (0x001c) /* 8-bits wide */

/* Registers ****************************************************************/

#define STR71X_I2C_CR(b)       ((b) + STR71X_I2C_SR_OFFSET)
#define STR71X_I2C_SR1(b)      ((b) + STR71X_I2C_SR1_OFFSET)
#define STR71X_I2C_SR2(b)      ((b) + STR71X_I2C_SR2_OFFSET)
#define STR71X_I2C_CCR(b)      ((b) + STR71X_I2C_CCR_OFFSET)
#define STR71X_I2C_OAR1(b)     ((b) + STR71X_I2C_OAR1_OFFSET)
#define STR71X_I2C_OAR2(b)     ((b) + STR71X_I2C_OAR2_OFFSET)
#define STR71X_I2C_DR(b)       ((b) + STR71X_I2C_DR_OFFSET)
#define STR71X_I2C_ECCR(b)     ((b) + STR71X_I2C_ECCR_OFFSET)

#define STR71X_I2C0_CR         (STR71X_I2C0_BASE + STR71X_I2C_SR_OFFSET)
#define STR71X_I2C0_SR1        (STR71X_I2C0_BASE + STR71X_I2C_SR1_OFFSET)
#define STR71X_I2C0_SR2        (STR71X_I2C0_BASE + STR71X_I2C_SR2_OFFSET)
#define STR71X_I2C0_CCR        (STR71X_I2C0_BASE + STR71X_I2C_CCR_OFFSET)
#define STR71X_I2C0_OAR1       (STR71X_I2C0_BASE + STR71X_I2C_OAR1_OFFSET)
#define STR71X_I2C0_OAR2       (STR71X_I2C0_BASE + STR71X_I2C_OAR2_OFFSET)
#define STR71X_I2C0_DR         (STR71X_I2C0_BASE + STR71X_I2C_DR_OFFSET)
#define STR71X_I2C0_ECCR       (STR71X_I2C0_BASE + STR71X_I2C_ECCR_OFFSET)

#define STR71X_I2C1_CR         (STR71X_I2C1_BASE + STR71X_I2C_SR_OFFSET)
#define STR71X_I2C1_SR1        (STR71X_I2C1_BASE + STR71X_I2C_SR1_OFFSET)
#define STR71X_I2C1_SR2        (STR71X_I2C1_BASE + STR71X_I2C_SR2_OFFSET)
#define STR71X_I2C1_CCR        (STR71X_I2C1_BASE + STR71X_I2C_CCR_OFFSET)
#define STR71X_I2C1_OAR1       (STR71X_I2C1_BASE + STR71X_I2C_OAR1_OFFSET)
#define STR71X_I2C1_OAR2       (STR71X_I2C1_BASE + STR71X_I2C_OAR2_OFFSET)
#define STR71X_I2C1_DR         (STR71X_I2C1_BASE + STR71X_I2C_DR_OFFSET)
#define STR71X_I2C1_ECCR       (STR71X_I2C1_BASE + STR71X_I2C_ECCR_OFFSET)

/* Register bit settings ****************************************************/

/* I2C Control Register (CR) */

#define STR71X_I2CCR_ITE        (0x01)    /* Bit 0: Interrupt enable */
#define STR71X_I2CCR_STOP       (0x02)    /* Bit 1: Generation of a stop condition */
#define STR71X_I2CCR_ACK        (0x04)    /* Bit 2: Acknowledge enable */
#define STR71X_I2CCR_START      (0x08)    /* Bit 3: Generation of a start condition */
#define STR71X_I2CCR_ENGC       (0x10)    /* Bit 4: Enable general call */
#define STR71X_I2CCR_PE         (0x20)    /* Bit 5: Peripheral enable */

/* I2C Status Register 1 (SR1) */

#define STR71X_I2CSR1_SB        (0x01)    /* Bit 0: Start bit (master mode) */
#define STR71X_I2CSR1_MSL       (0x02)    /* Bit 1: Master/slave */
#define STR71X_I2CSR1_ADSL      (0x04)    /* Bit 2: Address matched */
#define STR71X_I2CSR1_BTF       (0x08)    /* Bit 3: Byte transfer finished */
#define STR71X_I2CSR1_BUSY      (0x10)    /* Bit 4: Bus busy */
#define STR71X_I2CSR1_TRA       (0x20)    /* Bit 5: Transmitter/receiver */
#define STR71X_I2CSR1_ADD10     (0x40)    /* Bit 6: 10-bit addressing in master mode */
#define STR71X_I2CSR1_EVF       (0x80)    /* Bit 7: Event flag */

/* I2C Status Register 2 (SR2) */

#define STR71X_I2CSR2_GCAL      (0x01)    /* Bit 0: General call (slave mode) */
#define STR71X_I2CSR2_BERR      (0x02)    /* Bit 1: Bus error */
#define STR71X_I2CSR2_ARLO      (0x04)    /* Bit 2: Arbitration lost */
#define STR71X_I2CSR2_STOPF     (0x08)    /* Bit 3: Stop detection (slave mode) */
#define STR71X_I2CSR2_AF        (0x10)    /* Bit 4: Acknowledge failure */
#define STR71X_I2CSR2_ENDAD     (0x20)    /* Bit 5: End of address transmission */

/* I2C Clock Control Register (CCR) */

#define STR71X_I2CCCR_DIVMASK   (0x7f)    /* Bits 0-6: 7 bits of the 12-bit clock divider */
#define STR71X_I2CCCR_FMSM      (0x80)    /* Bit 7: Fast/standard I2C mode */

/* I2C Extended Clock Control Register (ECCR) */

#define STR71X_I2CECCR_DIVMASK  (0x1f)    /* Bits 0-5: 5 bits of the 12-bit clock divider */

/* I2C Own Address Register 2 (OAR2) */

#define STR71X_I2COAR2_ADDRMASK (0x06)    /* Bits 1-2: 2 bits of the 10-bit interface address */
#define STR71X_I2COAR2_FREQMASK (0xe0)    /* Bits 5-7: Frequency */
#define STR71X_I2COAR2_5_10     (0x00)    /*   FPCLK1 = 5 to 10 */
#define STR71X_I2COAR2_10_16    (0x20)    /*   FPCLK1 = 10 to 16.67 */
#define STR71X_I2COAR2_16_26    (0x40)    /*   FPCLK1 = 16.67 to 26.67 */
#define STR71X_I2COAR2_26_40    (0x60)    /*   FPCLK1 = 26.67 to 40 */
#define STR71X_I2COAR2_40_53    (0x80)    /*   FPCLK1 = 40 to 53.33 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_I2C_H */
