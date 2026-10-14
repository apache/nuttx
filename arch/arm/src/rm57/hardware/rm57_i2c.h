/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_i2c.h
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

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_I2C_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The offsets below are those of SPNU562A Table 30-3, cross-checked against
 * the field order of HALCoGen's i2cBASE_t; the two agree.
 * Only the low 16 bits of every register are implemented.
 */

/* I2C Register Offsets *****************************************************/

#define RM57_I2C_OAR_OFFSET        0x0000 /* Own Address */
#define RM57_I2C_IMR_OFFSET        0x0004 /* Interrupt Mask */
#define RM57_I2C_STR_OFFSET        0x0008 /* Status */
#define RM57_I2C_CKL_OFFSET        0x000c /* Clock Divider Low */
#define RM57_I2C_CKH_OFFSET        0x0010 /* Clock Divider High */
#define RM57_I2C_CNT_OFFSET        0x0014 /* Data Count */
#define RM57_I2C_DRR_OFFSET        0x0018 /* Data Receive */
#define RM57_I2C_SAR_OFFSET        0x001c /* Slave Address */
#define RM57_I2C_DXR_OFFSET        0x0020 /* Data Transmit */
#define RM57_I2C_MDR_OFFSET        0x0024 /* Mode */
#define RM57_I2C_IVR_OFFSET        0x0028 /* Interrupt Vector */
#define RM57_I2C_EMDR_OFFSET       0x002c /* Extended Mode */
#define RM57_I2C_PSC_OFFSET        0x0030 /* Prescale */
#define RM57_I2C_PID1_OFFSET       0x0034 /* Peripheral ID 1 */
#define RM57_I2C_PID2_OFFSET       0x0038 /* Peripheral ID 2 */
#define RM57_I2C_DMACR_OFFSET      0x003c /* DMA Control */
#define RM57_I2C_PFNC_OFFSET       0x0048 /* Pin Function */
#define RM57_I2C_PDIR_OFFSET       0x004c /* Pin Direction */
#define RM57_I2C_DIN_OFFSET        0x0050 /* Pin Data In */
#define RM57_I2C_DOUT_OFFSET       0x0054 /* Pin Data Out */
#define RM57_I2C_DSET_OFFSET       0x0058 /* Pin Data Set */
#define RM57_I2C_DCLR_OFFSET       0x005c /* Pin Data Clear */
#define RM57_I2C_PDR_OFFSET        0x0060 /* Pin Open Drain */
#define RM57_I2C_PDIS_OFFSET       0x0064 /* Pin Pull Disable */
#define RM57_I2C_PSEL_OFFSET       0x0068 /* Pin Pull Select */
#define RM57_I2C_SRS_OFFSET        0x006c /* Pin Slew Rate Select */

/* I2C Register Bit Definitions *********************************************/

/* Own Address and Slave Address: 7-bit addresses use bits 0-6 only */

#define I2C_ADDR_MASK              0x03ff /* Bits 0-9: OA / SA */

/* Interrupt Mask (IMR) and the matching flags in Status (STR) */

#define I2C_INT_AL                 (1 << 0)  /* Bit 0: Arbitration lost */
#define I2C_INT_NACK               (1 << 1)  /* Bit 1: No acknowledge */
#define I2C_INT_ARDY               (1 << 2)  /* Bit 2: Register access ready */
#define I2C_INT_RXRDY              (1 << 3)  /* Bit 3: Receive data ready */
#define I2C_INT_TXRDY              (1 << 4)  /* Bit 4: Transmit data ready */
#define I2C_INT_SCD                (1 << 5)  /* Bit 5: Stop condition detected */
#define I2C_INT_AAS                (1 << 6)  /* Bit 6: Addressed as slave */

/* Status (STR), beyond the interrupt flags above.  AL, NACK, ARDY, RXRDY,
 * SCD, NACKSNT and SDIR are write-1-to-clear; writing 1 to TXRDY sets it.
 */

#define I2C_STR_AD0                (1 << 8)  /* Bit 8: General call detected */
#define I2C_STR_AAS                (1 << 9)  /* Bit 9: Addressed as slave */
#define I2C_STR_XSMT               (1 << 10) /* Bit 10: Transmit shift empty (0 = underflow) */
#define I2C_STR_RSFULL             (1 << 11) /* Bit 11: Receive overrun */
#define I2C_STR_BB                 (1 << 12) /* Bit 12: Bus busy */
#define I2C_STR_NACKSNT            (1 << 13) /* Bit 13: NACK sent */
#define I2C_STR_SDIR               (1 << 14) /* Bit 14: Slave direction */

#define I2C_STR_W1C                (I2C_INT_AL | I2C_INT_NACK | I2C_INT_ARDY | \
                                    I2C_INT_RXRDY | I2C_INT_SCD | \
                                    I2C_STR_NACKSNT | I2C_STR_SDIR)

/* Data Count: a count of 0 means 65536 bytes */

#define I2C_CNT_MAX                0xffff

/* Mode (MDR) */

#define I2C_MDR_BC_SHIFT           0         /* Bits 0-2: Bit count */
#define I2C_MDR_BC_MASK            (7 << I2C_MDR_BC_SHIFT)
#  define I2C_MDR_BC_8BITS         (0 << I2C_MDR_BC_SHIFT)
#define I2C_MDR_FDF                (1 << 3)  /* Bit 3: Free data format */
#define I2C_MDR_STB                (1 << 4)  /* Bit 4: START byte mode */
#define I2C_MDR_NIRS               (1 << 5)  /* Bit 5: Out of reset */
#define I2C_MDR_DLB                (1 << 6)  /* Bit 6: Digital loopback */
#define I2C_MDR_RM                 (1 << 7)  /* Bit 7: Repeat mode */
#define I2C_MDR_XA                 (1 << 8)  /* Bit 8: 10-bit addressing */
#define I2C_MDR_TRX                (1 << 9)  /* Bit 9: Transmitter */
#define I2C_MDR_MST                (1 << 10) /* Bit 10: Master */
#define I2C_MDR_STP                (1 << 11) /* Bit 11: Generate STOP */
#define I2C_MDR_STT                (1 << 13) /* Bit 13: Generate START */
#define I2C_MDR_FREE               (1 << 14) /* Bit 14: Run free on emulation halt */
#define I2C_MDR_NACKMOD            (1 << 15) /* Bit 15: NACK the next byte */

/* Interrupt Vector (IVR).  Reading it clears the AL, NACK and SCD flags of
 * the code returned, provided that interrupt is enabled.
 */

#define I2C_IVR_INTCODE_SHIFT      0         /* Bits 0-2: Highest pending interrupt */
#define I2C_IVR_INTCODE_MASK       (7 << I2C_IVR_INTCODE_SHIFT)
#  define I2C_IVR_NONE             0
#  define I2C_IVR_AL               1
#  define I2C_IVR_NACK             2
#  define I2C_IVR_ARDY             3
#  define I2C_IVR_RXRDY            4
#  define I2C_IVR_TXRDY            5
#  define I2C_IVR_SCD              6
#  define I2C_IVR_AAS              7

/* Extended Mode (EMDR).  BCM resets to 1. */

#define I2C_EMDR_BCM               (1 << 0)  /* Bit 0: Backwards compatibility */
#define I2C_EMDR_IGNACK            (1 << 1)  /* Bit 1: Ignore NACK (master TX) */

/* Prescale (PSC): module clock = VCLK / (PSC + 1), 6.7-13.3 MHz */

#define I2C_PSC_MASK               0xff

/* DMA Control (DMACR).  Both bits reset to 1.  HALCoGen's enum i2cDMA puts
 * them at 0x20 / 0x10, which does not match SPNU562A Figure 30-29.
 */

#define I2C_DMACR_RXDMAEN          (1 << 0)  /* Bit 0: Receive DMA event */
#define I2C_DMACR_TXDMAEN          (1 << 1)  /* Bit 1: Transmit DMA event */

/* Pin Function (PFNC) */

#define I2C_PFNC_GIO               (1 << 0)  /* Bit 0: SDA and SCL are GIO pins */

/* Per-pin bits shared by PDIR, DIN, DOUT, DSET, DCLR, PDR, PDIS, PSEL and
 * SRS.  PDIS resets to 1 (pulls off).  PDR's polarity is documented both
 * ways in SPNU562A (Section 30.5.6 vs Table 30-32), so the driver never
 * relies on it and emulates open drain with the direction bit instead.
 */

#define I2C_PIN_SCL                (1 << 0)  /* Bit 0: SCL */
#define I2C_PIN_SDA                (1 << 1)  /* Bit 1: SDA */

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_I2C_H */
