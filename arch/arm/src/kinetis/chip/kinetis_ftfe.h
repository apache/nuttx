/************************************************************************************
 * arch/arm/src/kinetis/chip/kinetis_ftfe.h
 *
 *   Copyright (C) 2011, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_FTFE_H
#define __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_FTFE_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_FTFE_FSTAT_OFFSET   0x0000 /* Flash Status Register */
#define KINETIS_FTFE_FCNFG_OFFSET   0x0001 /* Flash Configuration Register */
#define KINETIS_FTFE_FSEC_OFFSET    0x0002 /* Flash Security Register */
#define KINETIS_FTFE_FOPT_OFFSET    0x0003 /* Flash Option Register */

#define KINETIS_FTFE_FCCOB3_OFFSET  0x0004 /* Flash Common Command Object Registers */
#define KINETIS_FTFE_FCCOB2_OFFSET  0x0005 /* Flash Common Command Object Registers */
#define KINETIS_FTFE_FCCOB1_OFFSET  0x0006 /* Flash Common Command Object Registers */
#define KINETIS_FTFE_FCCOB0_OFFSET  0x0007 /* Flash Common Command Object Registers */
#define KINETIS_FTFE_FCCOB7_OFFSET  0x0008 /* Flash Common Command Object Registers */
#define KINETIS_FTFE_FCCOB6_OFFSET  0x0009 /* Flash Common Command Object Registers */
#define KINETIS_FTFE_FCCOB5_OFFSET  0x000a /* Flash Common Command Object Registers */
#define KINETIS_FTFE_FCCOB4_OFFSET  0x000b /* Flash Common Command Object Registers */
#define KINETIS_FTFE_FCCOBB_OFFSET  0x000c /* Flash Common Command Object Registers */
#define KINETIS_FTFE_FCCOBA_OFFSET  0x000d /* Flash Common Command Object Registers */
#define KINETIS_FTFE_FCCOB9_OFFSET  0x000e /* Flash Common Command Object Registers */
#define KINETIS_FTFE_FCCOB8_OFFSET  0x000f /* Flash Common Command Object Registers */
#define KINETIS_FTFE_FPROT3_OFFSET  0x0010 /* Program Flash Protection Registers */
#define KINETIS_FTFE_FPROT2_OFFSET  0x0011 /* Program Flash Protection Registers */
#define KINETIS_FTFE_FPROT1_OFFSET  0x0012 /* Program Flash Protection Registers */
#define KINETIS_FTFE_FPROT0_OFFSET  0x0013 /* Program Flash Protection Registers */
#define KINETIS_FTFE_FEPROT_OFFSET  0x0016 /* EEPROM Protection Register */
#define KINETIS_FTFE_FDPROT_OFFSET  0x0017 /* Data Flash Protection Register */

/* Register Addresses ***************************************************************/

#define KINETIS_FTFE_FSTAT          (KINETIS_FTFE_BASE+KINETIS_FTFE_FSTAT_OFFSET)
#define KINETIS_FTFE_FCNFG          (KINETIS_FTFE_BASE+KINETIS_FTFE_FCNFG_OFFSET)
#define KINETIS_FTFE_FSEC           (KINETIS_FTFE_BASE+KINETIS_FTFE_FSEC_OFFSET)
#define KINETIS_FTFE_FOPT           (KINETIS_FTFE_BASE+KINETIS_FTFE_FOPT_OFFSET)
#define KINETIS_FTFE_FCCOB3         (KINETIS_FTFE_BASE+KINETIS_FTFE_FCCOB3_OFFSET)
#define KINETIS_FTFE_FCCOB2         (KINETIS_FTFE_BASE+KINETIS_FTFE_FCCOB2_OFFSET)
#define KINETIS_FTFE_FCCOB1         (KINETIS_FTFE_BASE+KINETIS_FTFE_FCCOB1_OFFSET)
#define KINETIS_FTFE_FCCOB0         (KINETIS_FTFE_BASE+KINETIS_FTFE_FCCOB0_OFFSET)
#define KINETIS_FTFE_FCCOB7         (KINETIS_FTFE_BASE+KINETIS_FTFE_FCCOB7_OFFSET)
#define KINETIS_FTFE_FCCOB6         (KINETIS_FTFE_BASE+KINETIS_FTFE_FCCOB6_OFFSET)
#define KINETIS_FTFE_FCCOB5         (KINETIS_FTFE_BASE+KINETIS_FTFE_FCCOB5_OFFSET)
#define KINETIS_FTFE_FCCOB4         (KINETIS_FTFE_BASE+KINETIS_FTFE_FCCOB4_OFFSET)
#define KINETIS_FTFE_FCCOBB         (KINETIS_FTFE_BASE+KINETIS_FTFE_FCCOBB_OFFSET)
#define KINETIS_FTFE_FCCOBA         (KINETIS_FTFE_BASE+KINETIS_FTFE_FCCOBA_OFFSET)
#define KINETIS_FTFE_FCCOB9         (KINETIS_FTFE_BASE+KINETIS_FTFE_FCCOB9_OFFSET)
#define KINETIS_FTFE_FCCOB8         (KINETIS_FTFE_BASE+KINETIS_FTFE_FCCOB8_OFFSET)
#define KINETIS_FTFE_FPROT3         (KINETIS_FTFE_BASE+KINETIS_FTFE_FPROT3_OFFSET)
#define KINETIS_FTFE_FPROT2         (KINETIS_FTFE_BASE+KINETIS_FTFE_FPROT2_OFFSET)
#define KINETIS_FTFE_FPROT1         (KINETIS_FTFE_BASE+KINETIS_FTFE_FPROT1_OFFSET)
#define KINETIS_FTFE_FPROT0         (KINETIS_FTFE_BASE+KINETIS_FTFE_FPROT0_OFFSET)
#define KINETIS_FTFE_FEPROT         (KINETIS_FTFE_BASE+KINETIS_FTFE_FEPROT_OFFSET)
#define KINETIS_FTFE_FDPROT         (KINETIS_FTFE_BASE+KINETIS_FTFE_FDPROT_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Flash Status Register */

#define FTFE_FSTAT_MGSTAT0          (1 << 0)  /* Bit 0:  Memory Controller Command Completion Status Flag */
                                              /* Bits 1-3: Reserved */
#define FTFE_FSTAT_FPVIOL           (1 << 4)  /* Bit 4:  Flash Protection Violation Flag */
#define FTFE_FSTAT_ACCERR           (1 << 5)  /* Bit 5:  Flash Access Error Flag */
#define FTFE_FSTAT_RDCOLERR         (1 << 6)  /* Bit 6:  FTFE Read Collision Error Flag */
#define FTFE_FSTAT_CCIF             (1 << 7)  /* Bit 7:  Command Complete Interrupt Flag */

/* Flash Configuration Register */

#define FTFE_FCNFG_EEERDY           (1 << 0)  /* Bit 0:  FEEPROM backup data copied to FlexRAM */
#define FTFE_FCNFG_RAMRDY           (1 << 1)  /* Bit 1:  RAM Ready */
#define FTFE_FCNFG_PFLSH            (1 << 2)  /* Bit 2:  FTFE configuration */
#define FTFE_FCNFG_SWAP             (1 << 3)  /* Bit 3:  Swap */
#define FTFE_FCNFG_ERSSUSP          (1 << 4)  /* Bit 4:  Erase Suspend */
#define FTFE_FCNFG_ERSAREQ          (1 << 5)  /* Bit 5:  Erase All Request */
#define FTFE_FCNFG_RDCOLLIE         (1 << 6)  /* Bit 6:  Read Collision Error Interrupt Enable */
#define FTFE_FCNFG_CCIE             (1 << 7)  /* Bit 7:  Command Complete Interrupt Enable */

/* Flash Security Register */

#define FTFE_FSEC_SEC_SHIFT         (0)       /* Bits 0-1: Flash Security */
#define FTFE_FSEC_SEC_MASK          (3 << FTFE_FSEC_SEC_SHIFT)
#  define FTFE_FSEC_SEC_SECURE      (0 << FTFE_FSEC_SEC_SHIFT) /* 00,01,11: status is secure */
#  define FTFE_FSEC_SEC_UNSECURE    (2 << FTFE_FSEC_SEC_SHIFT) /* 10: status is insecure */
#define FTFE_FSEC_FSLACC_SHIFT      (2)       /* Bits 2-3: Freescale Failure Analysis Access Code */
#define FTFE_FSEC_FSLACC_MASK       (3 << FTFE_FSEC_FSLACC_SHIFT)
#  define FTFE_FSEC_FSLACC_GRANTED  (0 << FTFE_FSEC_FSLACC_SHIFT) /* 00 or 11: Access granted */
#  define FTFE_FSEC_FSLACC_DENIED   (1 << FTFE_FSEC_FSLACC_SHIFT) /* 01 or 10: Access denied */
#define FTFE_FSEC_MEEN_SHIFT        (4)       /* Bits 4-5: Mass Erase Enable Bits */
#define FTFE_FSEC_MEEN_MASK         (3 << FTFE_FSEC_MEEN_SHIFT)
#  define FTFE_FSEC_MEEN_ENABLED    (0 << FTFE_FSEC_MEEN_SHIFT) /* All values are enabled */
#define FTFE_FSEC_KEYEN_SHIFT       (6)       /* Bits 6-7: Backdoor Key Security Enable */
#define FTFE_FSEC_KEYEN_MASK        (3 << FTFE_FSEC_KEYEN_SHIFT)
#  define FTFE_FSEC_KEYEN_DISABLED  (1 << FTFE_FSEC_KEYEN_SHIFT) /* All values are disabled */

/* Flash Option Register (32-bits, see Chip Configuration details) */
/* Flash Common Command Object Registers (8-bit flash command data) */
/* Program Flash Protection Registers (8-bit flash protection data) */
/* EEPROM Protection Register (8-bit eeprom protection data) */
/* Data Flash Protection Register (8-bit data flash protection data) */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_FTFE_H */
