/****************************************************************************
 * arch/arm/src/ra8m1/hardware/ra8m1_iic.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_IIC_H
#define __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_IIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra8m1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define R_IIC_ICCR1_OFFSET                  0x0000  /* I2C Bus Control Register 1 (8-bits) */
#define R_IIC_ICCR2_OFFSET                  0x0001  /* I2C Bus Control Register 2 (8-bits) */
#define R_IIC_ICMR1_OFFSET                  0x0002  /* I2C Bus Mode Register 1 (8-bits) */
#define R_IIC_ICMR2_OFFSET                  0x0003  /* I2C Bus Mode Register 2 (8-bits) */
#define R_IIC_ICMR3_OFFSET                  0x0004  /* I2C Bus Mode Register 3 (8-bits) */
#define R_IIC_ICFER_OFFSET                  0x0005  /* I2C Bus Function Enable Register (8-bits) */
#define R_IIC_ICSER_OFFSET                  0x0006  /* I2C Bus Status Enable Register (8-bits) */
#define R_IIC_ICIER_OFFSET                  0x0007  /* I2C Bus Interrupt Enable Register (8-bits) */
#define R_IIC_ICSR1_OFFSET                  0x0008  /* I2C Bus Status Register 1 (8-bits) */
#define R_IIC_ICSR2_OFFSET                  0x0009  /* I2C Bus Status Register 2 (8-bits) */
#define R_IIC_SARL_OFFSET                   0x000a  /* Slave Address Register L%s (8-bits) */
#define R_IIC_SARU_OFFSET                   0x000b  /* Slave Address Register U%s (8-bits) */
#define R_IIC_ICBRL_OFFSET                  0x0010  /* I2C Bus Bit Rate Low-Level Register (8-bits) */
#define R_IIC_ICBRH_OFFSET                  0x0011  /* I2C Bus Bit Rate High-Level Register (8-bits) */
#define R_IIC_ICDRT_OFFSET                  0x0012  /* I2C Bus Transmit Data Register (8-bits) */
#define R_IIC_ICDRR_OFFSET                  0x0013  /* I2C Bus Receive Data Register (8-bits) */

/* Register Addresses *******************************************************/

/* IIC0 Registers */

#define R_IIC0_ICCR1                       (R_IIC0_BASE + R_IIC_ICCR1_OFFSET)
#define R_IIC0_ICCR2                       (R_IIC0_BASE + R_IIC_ICCR2_OFFSET)
#define R_IIC0_ICMR1                       (R_IIC0_BASE + R_IIC_ICMR1_OFFSET)
#define R_IIC0_ICMR2                       (R_IIC0_BASE + R_IIC_ICMR2_OFFSET)
#define R_IIC0_ICMR3                       (R_IIC0_BASE + R_IIC_ICMR3_OFFSET)
#define R_IIC0_ICFER                       (R_IIC0_BASE + R_IIC_ICFER_OFFSET)
#define R_IIC0_ICSER                       (R_IIC0_BASE + R_IIC_ICSER_OFFSET)
#define R_IIC0_ICIER                       (R_IIC0_BASE + R_IIC_ICIER_OFFSET)
#define R_IIC0_ICSR1                       (R_IIC0_BASE + R_IIC_ICSR1_OFFSET)
#define R_IIC0_ICSR2                       (R_IIC0_BASE + R_IIC_ICSR2_OFFSET)
#define R_IIC0_SARL(p)                     (R_IIC0_BASE + R_IIC_SARL_OFFSET + (p)*0x0002)
#define R_IIC0_SARU(p)                     (R_IIC0_BASE + R_IIC_SARU_OFFSET + (p)*0x0002)
#define R_IIC0_ICBRL                       (R_IIC0_BASE + R_IIC_ICBRL_OFFSET)
#define R_IIC0_ICBRH                       (R_IIC0_BASE + R_IIC_ICBRH_OFFSET)
#define R_IIC0_ICDRT                       (R_IIC0_BASE + R_IIC_ICDRT_OFFSET)
#define R_IIC0_ICDRR                       (R_IIC0_BASE + R_IIC_ICDRR_OFFSET)

/* IIC1 Registers */

#define R_IIC1_ICCR1                       (R_IIC1_BASE + R_IIC_ICCR1_OFFSET)
#define R_IIC1_ICCR2                       (R_IIC1_BASE + R_IIC_ICCR2_OFFSET)
#define R_IIC1_ICMR1                       (R_IIC1_BASE + R_IIC_ICMR1_OFFSET)
#define R_IIC1_ICMR2                       (R_IIC1_BASE + R_IIC_ICMR2_OFFSET)
#define R_IIC1_ICMR3                       (R_IIC1_BASE + R_IIC_ICMR3_OFFSET)
#define R_IIC1_ICFER                       (R_IIC1_BASE + R_IIC_ICFER_OFFSET)
#define R_IIC1_ICSER                       (R_IIC1_BASE + R_IIC_ICSER_OFFSET)
#define R_IIC1_ICIER                       (R_IIC1_BASE + R_IIC_ICIER_OFFSET)
#define R_IIC1_ICSR1                       (R_IIC1_BASE + R_IIC_ICSR1_OFFSET)
#define R_IIC1_ICSR2                       (R_IIC1_BASE + R_IIC_ICSR2_OFFSET)
#define R_IIC1_SARL(p)                     (R_IIC1_BASE + R_IIC_SARL_OFFSET + (p)*0x0002)
#define R_IIC1_SARU(p)                     (R_IIC1_BASE + R_IIC_SARU_OFFSET + (p)*0x0002)
#define R_IIC1_ICBRL                       (R_IIC1_BASE + R_IIC_ICBRL_OFFSET)
#define R_IIC1_ICBRH                       (R_IIC1_BASE + R_IIC_ICBRH_OFFSET)
#define R_IIC1_ICDRT                       (R_IIC1_BASE + R_IIC_ICDRT_OFFSET)
#define R_IIC1_ICDRR                       (R_IIC1_BASE + R_IIC_ICDRR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* I2C Bus Control Register 1 (8-bits) **************************************/

#define R_IIC_ICCR1_SDAI (1 <<  0)    /* 01: SDA Line Monitor */
#define R_IIC_ICCR1_SCLI (1 <<  1)    /* 02: SCL Line Monitor */
#define R_IIC_ICCR1_SDAO (1 <<  2)    /* 04: SDA Output Control/Monitor */
#define R_IIC_ICCR1_SCLO (1 <<  3)    /* 08: SCL Output Control/Monitor */
#define R_IIC_ICCR1_SOWP (1 <<  4)    /* 10: SCLO/SDAO Write Protect */
#define R_IIC_ICCR1_CLO (1 <<  5)     /* 20: Extra SCL Clock Cycle Output */
#define R_IIC_ICCR1_IICRST (1 <<  6)  /* 40: I2C Bus Interface Internal ResetNote:If an internal reset is initiated using the IICRST bit for a bus hang-up occurred during communication with the master device in slave mode, the states may become different between the slave device and the master device (due to the difference in the bit counter information). */
#define R_IIC_ICCR1_ICE (1 <<  7)     /* 80: I2C Bus Interface Enable */

/* I2C Bus Control Register 2 (8-bits) **************************************/

#define R_IIC_ICCR2_ST (1 <<  1)    /* 02: Start Condition Issuance RequestSet the ST bit to 1 (start condition issuance request) when the BBSY flag is set to 0 (bus free state). */
#define R_IIC_ICCR2_RS (1 <<  2)    /* 04: Restart Condition Issuance RequestNote: Do not set the RS bit to 1 while issuing a stop condition. */
#define R_IIC_ICCR2_SP (1 <<  3)    /* 08: Stop Condition Issuance RequestNote: Writing to the SP bit is not possible while the setting of the BBSY flag is 0 (bus free state).Note: Do not set the SP bit to 1 while a restart condition is being issued. */
#define R_IIC_ICCR2_TRS (1 <<  5)   /* 20: Transmit/Receive Mode */
#define R_IIC_ICCR2_MST (1 <<  6)   /* 40: Master/Slave Mode */
#define R_IIC_ICCR2_BBSY (1 <<  7)  /* 80: Bus Busy Detection Flag */

/* I2C Bus Mode Register 1 (8-bits) *****************************************/

#define R_IIC_ICMR1_BC_SHIFT (0)
#define R_IIC_ICMR1_BC_MASK (0x7)
#  define R_IIC_ICMR1_BC_V9_BITS (0 << R_IIC_ICMR1_BC_SHIFT)            /* 9 bits */
#  define R_IIC_ICMR1_BC_V2_BITS (1 << R_IIC_ICMR1_BC_SHIFT)            /* 2 bits */
#  define R_IIC_ICMR1_BC_V3_BITS (2 << R_IIC_ICMR1_BC_SHIFT)            /* 3 bits */
#  define R_IIC_ICMR1_BC_V4_BITS (3 << R_IIC_ICMR1_BC_SHIFT)            /* 4 bits */
#  define R_IIC_ICMR1_BC_V5_BITS (4 << R_IIC_ICMR1_BC_SHIFT)            /* 5 bits */
#  define R_IIC_ICMR1_BC_V6_BITS (5 << R_IIC_ICMR1_BC_SHIFT)            /* 6 bits */
#  define R_IIC_ICMR1_BC_V7_BITS (6 << R_IIC_ICMR1_BC_SHIFT)            /* 7 bits */
#  define R_IIC_ICMR1_BC_V8_BITS (7 << R_IIC_ICMR1_BC_SHIFT)            /* 8 bits */
#define R_IIC_ICMR1_BCWP (1 <<  3)                                      /* 08: BC Write Protect(This bit is read as 1.) */
#define R_IIC_ICMR1_CKS_SHIFT (4)
#define R_IIC_ICMR1_CKS_MASK (0x7)
#  define R_IIC_ICMR1_CKS_PCLKB_1_CLOCK (0 << R_IIC_ICMR1_CKS_SHIFT)    /* PCLKB/1 clock */
#  define R_IIC_ICMR1_CKS_PCLKB_2_CLOCK (1 << R_IIC_ICMR1_CKS_SHIFT)    /* PCLKB/2 clock */
#  define R_IIC_ICMR1_CKS_PCLKB_4_CLOCK (2 << R_IIC_ICMR1_CKS_SHIFT)    /* PCLKB/4 clock */
#  define R_IIC_ICMR1_CKS_PCLKB_8_CLOCK (3 << R_IIC_ICMR1_CKS_SHIFT)    /* PCLKB/8 clock */
#  define R_IIC_ICMR1_CKS_PCLKB_16_CLOCK (4 << R_IIC_ICMR1_CKS_SHIFT)   /* PCLKB/16 clock */
#  define R_IIC_ICMR1_CKS_PCLKB_32_CLOCK (5 << R_IIC_ICMR1_CKS_SHIFT)   /* PCLKB/32 clock */
#  define R_IIC_ICMR1_CKS_PCLKB_64_CLOCK (6 << R_IIC_ICMR1_CKS_SHIFT)   /* PCLKB/64 clock */
#  define R_IIC_ICMR1_CKS_PCLKB_128_CLOCK (7 << R_IIC_ICMR1_CKS_SHIFT)  /* PCLKB/128 clock */
#define R_IIC_ICMR1_MTWP (1 <<  7)                                      /* 80: MST/TRS Write Protect */

/* I2C Bus Mode Register 2 (8-bits) *****************************************/

#define R_IIC_ICMR2_TMOS (1 <<  0)                                        /* 01: Timeout Detection Time Selection */
#define R_IIC_ICMR2_TMOL (1 <<  1)                                        /* 02: Timeout L Count Control */
#define R_IIC_ICMR2_TMOH (1 <<  2)                                        /* 04: Timeout H Count Control */
#define R_IIC_ICMR2_SDDL_SHIFT (4)
#define R_IIC_ICMR2_SDDL_MASK (0x7)
#  define R_IIC_ICMR2_SDDL_NO_OUTPUT_DELAY (0 << R_IIC_ICMR2_SDDL_SHIFT)  /* No output delay */
#  define R_IIC_ICMR2_SDDL_V001 (1 << R_IIC_ICMR2_SDDL_SHIFT)             /* 1 fIIC cycle (ICMR2.DLCS=0) / 1 or 2 fIIC cycles (ICMR2.DLCS=1) */
#  define R_IIC_ICMR2_SDDL_V010 (2 << R_IIC_ICMR2_SDDL_SHIFT)             /* 2 fIIC cycles (ICMR2.DLCS=0) / 3 or 4 fIIC cycles (ICMR2.DLCS=1) */
#  define R_IIC_ICMR2_SDDL_V011 (3 << R_IIC_ICMR2_SDDL_SHIFT)             /* 3 fIIC cycles (ICMR2.DLCS=0) / 5 or 6 fIIC cycles (ICMR2.DLCS=1) */
#  define R_IIC_ICMR2_SDDL_V100 (4 << R_IIC_ICMR2_SDDL_SHIFT)             /* 4 fIIC cycles (ICMR2.DLCS=0) / 7 or 8 fIIC cycles (ICMR2.DLCS=1) */
#  define R_IIC_ICMR2_SDDL_V101 (5 << R_IIC_ICMR2_SDDL_SHIFT)             /* 5 fIIC cycles (ICMR2.DLCS=0) / 9 or 10 fIIC cycles (ICMR2.DLCS=1) */
#  define R_IIC_ICMR2_SDDL_V110 (6 << R_IIC_ICMR2_SDDL_SHIFT)             /* 6 fIIC cycles (ICMR2.DLCS=0) / 11 or 12 fIIC cycles (ICMR2.DLCS=1) */
#  define R_IIC_ICMR2_SDDL_V111 (7 << R_IIC_ICMR2_SDDL_SHIFT)             /* 7 fIIC cycles (ICMR2.DLCS=0) / 13 or 14 fIIC cycles (ICMR2.DLCS=1) */
#define R_IIC_ICMR2_DLCS (1 <<  7)                                        /* 80: SDA Output Delay Clock Source Selection */

/* I2C Bus Mode Register 3 (8-bits) *****************************************/

#define R_IIC_ICMR3_NF_SHIFT (0)
#define R_IIC_ICMR3_NF_MASK (0x3)
#  define R_IIC_ICMR3_NF_V00 (0 << R_IIC_ICMR3_NF_SHIFT)  /* Noise of up to one fIIC cycle is filtered out (single-stage filter). */
#  define R_IIC_ICMR3_NF_V01 (1 << R_IIC_ICMR3_NF_SHIFT)  /* Noise of up to two fIIC cycles is filtered out (2-stage filter). */
#  define R_IIC_ICMR3_NF_V10 (2 << R_IIC_ICMR3_NF_SHIFT)  /* Noise of up to three fIIC cycles is filtered out (3-stage filter). */
#  define R_IIC_ICMR3_NF_V11 (3 << R_IIC_ICMR3_NF_SHIFT)  /* Noise of up to four fIIC cycles is filtered out (4-stage filter) */
#define R_IIC_ICMR3_ACKBR (1 <<  2)                       /* 04: Receive Acknowledge */
#define R_IIC_ICMR3_ACKBT (1 <<  3)                       /* 08: Transmit Acknowledge */
#define R_IIC_ICMR3_ACKWP (1 <<  4)                       /* 10: ACKBT Write Protect */
#define R_IIC_ICMR3_RDRFS (1 <<  5)                       /* 20: RDRF Flag Set Timing Selection */
#define R_IIC_ICMR3_WAIT (1 <<  6)                        /* 40: WAITNote: When the value of the WAIT bit is to be read, be sure to read the ICDRR beforehand. */
#define R_IIC_ICMR3_SMBS (1 <<  7)                        /* 80: SMBus/I2C Bus Selection */

/* I2C Bus Function Enable Register (8-bits) ********************************/

#define R_IIC_ICFER_TMOE (1 <<  0)   /* 01: Timeout Function Enable */
#define R_IIC_ICFER_MALE (1 <<  1)   /* 02: Master Arbitration-Lost Detection Enable */
#define R_IIC_ICFER_NALE (1 <<  2)   /* 04: NACK Transmission Arbitration-Lost Detection Enable */
#define R_IIC_ICFER_SALE (1 <<  3)   /* 08: Slave Arbitration-Lost Detection Enable */
#define R_IIC_ICFER_NACKE (1 <<  4)  /* 10: NACK Reception Transfer Suspension Enable */
#define R_IIC_ICFER_NFE (1 <<  5)    /* 20: Digital Noise Filter Circuit Enable */
#define R_IIC_ICFER_SCLE (1 <<  6)   /* 40: SCL Synchronous Circuit Enable */
#define R_IIC_ICFER_FMPE (1 <<  7)   /* 80: Fast-mode Plus Enable */

/* I2C Bus Status Enable Register (8-bits) **********************************/

#define R_IIC_ICSER_SAR0E (1 <<  0)  /* 01: Slave Address Register 0 Enable */
#define R_IIC_ICSER_SAR1E (1 <<  1)  /* 02: Slave Address Register 1 Enable */
#define R_IIC_ICSER_SAR2E (1 <<  2)  /* 04: Slave Address Register 2 Enable */
#define R_IIC_ICSER_GCAE (1 <<  3)   /* 08: General Call Address Enable */
#define R_IIC_ICSER_DIDE (1 <<  5)   /* 20: Device-ID Address Detection Enable */
#define R_IIC_ICSER_HOAE (1 <<  7)   /* 80: Host Address Enable */

/* I2C Bus Interrupt Enable Register (8-bits) *******************************/

#define R_IIC_ICIER_TMOIE (1 <<  0)  /* 01: Timeout Interrupt Request Enable */
#define R_IIC_ICIER_ALIE (1 <<  1)   /* 02: Arbitration-Lost Interrupt Request Enable */
#define R_IIC_ICIER_STIE (1 <<  2)   /* 04: Start Condition Detection Interrupt Request Enable */
#define R_IIC_ICIER_SPIE (1 <<  3)   /* 08: Stop Condition Detection Interrupt Request Enable */
#define R_IIC_ICIER_NAKIE (1 <<  4)  /* 10: NACK Reception Interrupt Request Enable */
#define R_IIC_ICIER_RIE (1 <<  5)    /* 20: Receive Data Full Interrupt Request Enable */
#define R_IIC_ICIER_TEIE (1 <<  6)   /* 40: Transmit End Interrupt Request Enable */
#define R_IIC_ICIER_TIE (1 <<  7)    /* 80: Transmit Data Empty Interrupt Request Enable */

/* I2C Bus Status Register 1 (8-bits) ***************************************/

#define R_IIC_ICSR1_AAS0 (1 <<  0)  /* 01: Slave Address 0 Detection Flag */
#define R_IIC_ICSR1_AAS1 (1 <<  1)  /* 02: Slave Address 1 Detection Flag */
#define R_IIC_ICSR1_AAS2 (1 <<  2)  /* 04: Slave Address 2 Detection Flag */
#define R_IIC_ICSR1_GCA (1 <<  3)   /* 08: General Call Address Detection Flag */
#define R_IIC_ICSR1_DID (1 <<  5)   /* 20: Device-ID Address Detection Flag */
#define R_IIC_ICSR1_HOA (1 <<  7)   /* 80: Host Address Detection Flag */

/* I2C Bus Status Register 2 (8-bits) ***************************************/

#define R_IIC_ICSR2_TMOF (1 <<  0)   /* 01: Timeout Detection Flag */
#define R_IIC_ICSR2_AL (1 <<  1)     /* 02: Arbitration-Lost Flag */
#define R_IIC_ICSR2_START (1 <<  2)  /* 04: Start Condition Detection Flag */
#define R_IIC_ICSR2_STOP (1 <<  3)   /* 08: Stop Condition Detection Flag */
#define R_IIC_ICSR2_NACKF (1 <<  4)  /* 10: NACK Detection Flag */
#define R_IIC_ICSR2_RDRF (1 <<  5)   /* 20: Receive Data Full Flag */
#define R_IIC_ICSR2_TEND (1 <<  6)   /* 40: Transmit End Flag */
#define R_IIC_ICSR2_TDRE (1 <<  7)   /* 80: Transmit Data Empty Flag */

/* Slave Address Register L%s (8-bits) **************************************/

#define R_IIC_SARL_SIZE 3
#define R_IIC_SARL_SVA_SHIFT (0)
#define R_IIC_SARL_SVA_MASK (0xff)

/* Slave Address Register U%s (8-bits) **************************************/

#define R_IIC_SARU_SIZE 3
#define R_IIC_SARU_FS (1 <<  0)    /* 01: 7-Bit/10-Bit Address Format Selection */
#define R_IIC_SARU_SVA8 (1 <<  1)  /* 02: 10-Bit Address(bit8) */
#define R_IIC_SARU_SVA9 (1 <<  2)  /* 04: 10-Bit Address(bit9) */

/* I2C Bus Bit Rate Low-Level Register (8-bits) *****************************/

#define R_IIC_ICBRL_BRL_SHIFT (0)
#define R_IIC_ICBRL_BRL_MASK (0x1f)

/* I2C Bus Bit Rate High-Level Register (8-bits) ****************************/

#define R_IIC_ICBRH_BRH_SHIFT (0)
#define R_IIC_ICBRH_BRH_MASK (0x1f)

/* I2C Bus Transmit Data Register (8-bits) **********************************/

#define R_IIC_ICDRT_ICDRT_SHIFT (0)
#define R_IIC_ICDRT_ICDRT_MASK (0xff)

/* I2C Bus Receive Data Register (8-bits) ***********************************/

#define R_IIC_ICDRR_ICDRR_SHIFT (0)
#define R_IIC_ICDRR_ICDRR_MASK (0xff)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_IIC_H */
