/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/hardware/lpc17_40_i2c.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_I2C_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/lpc17_40_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define LPC17_40_I2C_CONSET_OFFSET    0x0000 /* I2C Control Set Register */
#define LPC17_40_I2C_STAT_OFFSET      0x0004 /* I2C Status Register */
#define LPC17_40_I2C_DAT_OFFSET       0x0008 /* I2C Data Register */
#define LPC17_40_I2C_ADR0_OFFSET      0x000c /* I2C Slave Address Register 0 */
#define LPC17_40_I2C_SCLH_OFFSET      0x0010 /* SCH Duty Cycle Register High Half Word */
#define LPC17_40_I2C_SCLL_OFFSET      0x0014 /* SCL Duty Cycle Register Low Half Word */
#define LPC17_40_I2C_CONCLR_OFFSET    0x0018 /* I2C Control Clear Register */
#define LPC17_40_I2C_MMCTRL_OFFSET    0x001c /* Monitor mode control register */
#define LPC17_40_I2C_ADR1_OFFSET      0x0020 /* I2C Slave Address Register 1 */
#define LPC17_40_I2C_ADR2_OFFSET      0x0024 /* I2C Slave Address Register 2 */
#define LPC17_40_I2C_ADR3_OFFSET      0x0028 /* I2C Slave Address Register 3 */
#define LPC17_40_I2C_BUFR_OFFSET      0x002c /* Data buffer register */
#define LPC17_40_I2C_MASK0_OFFSET     0x0030 /* I2C Slave address mask register 0 */
#define LPC17_40_I2C_MASK1_OFFSET     0x0034 /* I2C Slave address mask register 1 */
#define LPC17_40_I2C_MASK2_OFFSET     0x0038 /* I2C Slave address mask register 2 */
#define LPC17_40_I2C_MASK3_OFFSET     0x003c /* I2C Slave address mask register */

/* Register addresses *******************************************************/

#define LPC17_40_I2C0_CONSET          (LPC17_40_I2C0_BASE+LPC17_40_I2C_CONSET_OFFSET)
#define LPC17_40_I2C0_STAT            (LPC17_40_I2C0_BASE+LPC17_40_I2C_STAT_OFFSET)
#define LPC17_40_I2C0_DAT             (LPC17_40_I2C0_BASE+LPC17_40_I2C_DAT_OFFSET)
#define LPC17_40_I2C0_ADR0            (LPC17_40_I2C0_BASE+LPC17_40_I2C_ADR0_OFFSET)
#define LPC17_40_I2C0_SCLH            (LPC17_40_I2C0_BASE+LPC17_40_I2C_SCLH_OFFSET)
#define LPC17_40_I2C0_SCLL            (LPC17_40_I2C0_BASE+LPC17_40_I2C_SCLL_OFFSET)
#define LPC17_40_I2C0_CONCLR          (LPC17_40_I2C0_BASE+LPC17_40_I2C_CONCLR_OFFSET)
#define LPC17_40_I2C0_MMCTRL          (LPC17_40_I2C0_BASE+LPC17_40_I2C_MMCTRL_OFFSET)
#define LPC17_40_I2C0_ADR1            (LPC17_40_I2C0_BASE+LPC17_40_I2C_ADR1_OFFSET)
#define LPC17_40_I2C0_ADR2            (LPC17_40_I2C0_BASE+LPC17_40_I2C_ADR2_OFFSET)
#define LPC17_40_I2C0_ADR3            (LPC17_40_I2C0_BASE+LPC17_40_I2C_ADR3_OFFSET)
#define LPC17_40_I2C0_BUFR            (LPC17_40_I2C0_BASE+LPC17_40_I2C_BUFR_OFFSET)
#define LPC17_40_I2C0_MASK0           (LPC17_40_I2C0_BASE+LPC17_40_I2C_MASK0_OFFSET)
#define LPC17_40_I2C0_MASK1           (LPC17_40_I2C0_BASE+LPC17_40_I2C_MASK1_OFFSET)
#define LPC17_40_I2C0_MASK2           (LPC17_40_I2C0_BASE+LPC17_40_I2C_MASK2_OFFSET)
#define LPC17_40_I2C0_MASK3           (LPC17_40_I2C0_BASE+LPC17_40_I2C_MASK3_OFFSET)

#define LPC17_40_I2C1_CONSET          (LPC17_40_I2C1_BASE+LPC17_40_I2C_CONSET_OFFSET)
#define LPC17_40_I2C1_STAT            (LPC17_40_I2C1_BASE+LPC17_40_I2C_STAT_OFFSET)
#define LPC17_40_I2C1_DAT             (LPC17_40_I2C1_BASE+LPC17_40_I2C_DAT_OFFSET)
#define LPC17_40_I2C1_ADR0            (LPC17_40_I2C1_BASE+LPC17_40_I2C_ADR0_OFFSET)
#define LPC17_40_I2C1_SCLH            (LPC17_40_I2C1_BASE+LPC17_40_I2C_SCLH_OFFSET)
#define LPC17_40_I2C1_SCLL            (LPC17_40_I2C1_BASE+LPC17_40_I2C_SCLL_OFFSET)
#define LPC17_40_I2C1_CONCLR          (LPC17_40_I2C1_BASE+LPC17_40_I2C_CONCLR_OFFSET)
#define LPC17_40_I2C1_MMCTRL          (LPC17_40_I2C1_BASE+LPC17_40_I2C_MMCTRL_OFFSET)
#define LPC17_40_I2C1_ADR1            (LPC17_40_I2C1_BASE+LPC17_40_I2C_ADR1_OFFSET)
#define LPC17_40_I2C1_ADR2            (LPC17_40_I2C1_BASE+LPC17_40_I2C_ADR2_OFFSET)
#define LPC17_40_I2C1_ADR3            (LPC17_40_I2C1_BASE+LPC17_40_I2C_ADR3_OFFSET)
#define LPC17_40_I2C1_BUFR            (LPC17_40_I2C1_BASE+LPC17_40_I2C_BUFR_OFFSET)
#define LPC17_40_I2C1_MASK0           (LPC17_40_I2C1_BASE+LPC17_40_I2C_MASK0_OFFSET)
#define LPC17_40_I2C1_MASK1           (LPC17_40_I2C1_BASE+LPC17_40_I2C_MASK1_OFFSET)
#define LPC17_40_I2C1_MASK2           (LPC17_40_I2C1_BASE+LPC17_40_I2C_MASK2_OFFSET)
#define LPC17_40_I2C1_MASK3           (LPC17_40_I2C1_BASE+LPC17_40_I2C_MASK3_OFFSET)

#define LPC17_40_I2C2_CONSET          (LPC17_40_I2C2_BASE+LPC17_40_I2C_CONSET_OFFSET)
#define LPC17_40_I2C2_STAT            (LPC17_40_I2C2_BASE+LPC17_40_I2C_STAT_OFFSET)
#define LPC17_40_I2C2_DAT             (LPC17_40_I2C2_BASE+LPC17_40_I2C_DAT_OFFSET)
#define LPC17_40_I2C2_ADR0            (LPC17_40_I2C2_BASE+LPC17_40_I2C_ADR0_OFFSET)
#define LPC17_40_I2C2_SCLH            (LPC17_40_I2C2_BASE+LPC17_40_I2C_SCLH_OFFSET)
#define LPC17_40_I2C2_SCLL            (LPC17_40_I2C2_BASE+LPC17_40_I2C_SCLL_OFFSET)
#define LPC17_40_I2C2_CONCLR          (LPC17_40_I2C2_BASE+LPC17_40_I2C_CONCLR_OFFSET)
#define LPC17_40_I2C2_MMCTRL          (LPC17_40_I2C2_BASE+LPC17_40_I2C_MMCTRL_OFFSET)
#define LPC17_40_I2C2_ADR1            (LPC17_40_I2C2_BASE+LPC17_40_I2C_ADR1_OFFSET)
#define LPC17_40_I2C2_ADR2            (LPC17_40_I2C2_BASE+LPC17_40_I2C_ADR2_OFFSET)
#define LPC17_40_I2C2_ADR3            (LPC17_40_I2C2_BASE+LPC17_40_I2C_ADR3_OFFSET)
#define LPC17_40_I2C2_BUFR            (LPC17_40_I2C2_BASE+LPC17_40_I2C_BUFR_OFFSET)
#define LPC17_40_I2C2_MASK0           (LPC17_40_I2C2_BASE+LPC17_40_I2C_MASK0_OFFSET)
#define LPC17_40_I2C2_MASK1           (LPC17_40_I2C2_BASE+LPC17_40_I2C_MASK1_OFFSET)
#define LPC17_40_I2C2_MASK2           (LPC17_40_I2C2_BASE+LPC17_40_I2C_MASK2_OFFSET)
#define LPC17_40_I2C2_MASK3           (LPC17_40_I2C2_BASE+LPC17_40_I2C_MASK3_OFFSET)

/* Register bit definitions *************************************************/

/* I2C Control Set Register */

                                                /* Bits 0-1: Reserved */
#define I2C_CONSET_AA                 (1 << 2)  /* Bit 2:  Assert acknowledge flag */
#define I2C_CONSET_SI                 (1 << 3)  /* Bit 3:  I2C interrupt flag */
#define I2C_CONSET_STO                (1 << 4)  /* Bit 4:  STOP flag */
#define I2C_CONSET_STA                (1 << 5)  /* Bit 5:  START flag */
#define I2C_CONSET_I2EN               (1 << 6)  /* Bit 6:  I2C interface enable */
                                                /* Bits 7-31: Reserved */

/* I2C Control Clear Register */

                                                /* Bits 0-1: Reserved */
#define I2C_CONCLR_AAC                (1 << 2)  /* Bit 2:  Assert acknowledge Clear bit */
#define I2C_CONCLR_SIC                (1 << 3)  /* Bit 3:  I2C interrupt Clear bit */
                                                /* Bit 4:  Reserved */
#define I2C_CONCLR_STAC               (1 << 5)  /* Bit 5:  START flag Clear bit */
#define I2C_CONCLRT_I2ENC             (1 << 6)  /* Bit 6:  I2C interface Disable bit */
                                                /* Bits 7-31: Reserved */

/* I2C Status Register
 *
 *   See tables 399-402 in the "LPC17xx User Manual" (UM10360),
 *   Rev. 01, 4 January 2010, NXP for definitions of status codes.
 */

#define I2C_STAT_MASK                 (0xff)    /* Bits 0-7: I2C interface status
                                                 *           Bits 0-1 always zero */
                                                /* Bits 8-31: Reserved */

/* I2C Data Register */

#define I2C_DAT_MASK                  (0xff)    /* Bits 0-7: I2C data */
                                                /* Bits 8-31: Reserved */

/* Monitor mode control register */

#define I2C_MMCTRL_MMENA              (1 << 0)  /* Bit 0:  Monitor mode enable */
#define I2C_MMCTRL_ENASCL             (1 << 1)  /* Bit 1:  SCL output enable */
#define I2C_MMCTRL_MATCHALL           (1 << 2)  /* Bit 2:  Select interrupt register match */
                                                /* Bits 3-31: Reserved */

/* Data buffer register */

#define I2C_BUFR_MASK                 (0xff)    /* Bits 0-7: 8 MSBs of the I2DAT shift register */
                                                /* Bits 8-31: Reserved */

/* I2C Slave address registers:
 *
 *   I2C Slave Address Register 0
 *   I2C Slave Address Register 1
 *   I2C Slave Address Register 2
 *   I2C Slave Address Register 3
 */

#define I2C_ADR_GC                    (1 << 0) /* Bit 0: GC General Call enable bit */
#define I2C_ADR_ADDR_SHIFT            (1)      /* Bits 1-7: I2C slave address */
#define I2C_ADR_ADDR_MASK             (0x7f << I2C_ADR_ADDR_SHIFT)
                                               /* Bits 8-31: Reserved */

/* I2C Slave address mask registers:
 *
 *   I2C Slave address mask register 0
 *   I2C Slave address mask register 1
 *   I2C Slave address mask register 2
 *   I2C Slave address mask register 3
 */

                                               /* Bit 0: Reserved */
#define I2C_MASK_SHIFT                (1)      /* Bits 1-7: I2C mask bits */
#define I2C_MASK_MASK                 (0x7f << I2C_ADR_ADDR_SHIFT)
                                               /* Bits 8-31: Reserved */

/* SCH Duty Cycle Register High Half Word */

#define I2C_SCLH_MASK                 (0xffff) /* Bit 0-15: Count for SCL HIGH time period selection */
                                               /* Bits 16-31: Reserved */

/* SCL Duty Cycle Register Low Half Word */

#define I2C_SCLL_MASK                 (0xffff) /* Bit 0-15: Count for SCL LOW time period selection */
                                               /* Bits 16-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_I2C_H */
