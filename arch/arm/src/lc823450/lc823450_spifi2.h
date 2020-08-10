/****************************************************************************
 * arch/arm/src/lc823450/lc823450_spifi2.h
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

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_SPIFI2_H
#define __ARCH_ARM_SRC_LC823450_LC823450_SPIFI2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LC823450_SFLASH_REGBASE 0x40001000

#define SF_SIZE     (LC823450_SFLASH_REGBASE + 0x00)
#define   SF_SIZE_T_SHIFT        0
#define   SF_SIZE_UL_SHIFT       24
#define   SF_SIZE_NOREAD        (1 << 28)
#define SF_CTL      (LC823450_SFLASH_REGBASE + 0x04)
#define   SF_CTL_ACT             (1 << 0)
#define SF_MODE     (LC823450_SFLASH_REGBASE + 0x08)
#define   SF_MODE_MODE_SHIFT     0
#define SF_DUMMY    (LC823450_SFLASH_REGBASE + 0x0c)
#define   SF_DUMMY_DUMMY        (1 << 0)
#define SF_FIFO_CLR (LC823450_SFLASH_REGBASE + 0x10)
#define SF_STATUS   (LC823450_SFLASH_REGBASE + 0x14)
#define SF_IRQEN    (LC823450_SFLASH_REGBASE + 0x18)
#define SF_FIFO_NUM (LC823450_SFLASH_REGBASE + 0x1c)
#define   SF_FIFO_NUM_T_SHIFT    0
#define   SF_FIFO_NUM_T_MASK    (0xf << SF_FIFO_NUM_T_SHIFT)
#define   SF_FIFO_NUM_R_SHIFT    8
#define   SF_FIFO_NUM_R_MASK    (0xf << SF_FIFO_NUM_R_SHIFT)
#define SF_SRSTB     (LC823450_SFLASH_REGBASE + 0x20)
#define SF_PHASE_SET (LC823450_SFLASH_REGBASE + 0x24)
#define SF_BUS       (LC823450_SFLASH_REGBASE + 0x28)
#define   SF_BUS_BUSEN        (1 << 0)
#define   SF_BUS_LOOKAHEAD    (1 << 1)
#define   SF_BUS_ACT          (1 << 24)
#define   SF_BUS_BUSMODE_SHIFT    8
#define SF_TIMING    (LC823450_SFLASH_REGBASE + 0x2c)
#define   SF_TIMING_CS_MASK    (0x7 << 0)
#define   SF_TIMING_CS_SHIFT    0
#define SF_T_FIFO    (LC823450_SFLASH_REGBASE + 0x30)
#define SF_R_FIFO    (LC823450_SFLASH_REGBASE + 0x34)

#define SF_CMD_WRITE_STATUS   0x01
#define SF_CMD_PAGE_PROG      0x02
#define SF_CMD_READ_STATUS1   0x05
#define SF_CMD_WRITE_EN       0x06
#define SF_CMD_FAST_READ      0x0b
#define SF_CMD_SEC_ERASE      0x20
#define SF_CMD_WRITE_STATUS2  0x31
#define SF_CMD_READ_STATUS2   0x35
#define SF_CMD_SR_WRITE_EN    0x50
#define SF_CMD_CHIP_ERASE     0x60
#define SF_CMD_FAST_READ_QUAD 0x6b
#define SF_CMD_READ_JID       0x9f

#define SF_STATUS1_BUSY       (1 << 0)
#define SF_STATUS2_QE    	    (1 << 1)

#define SF_JID_MID_MASK       0xff0000
#define SF_JID_MID_WINBOND    0xef0000
#define SF_JID_MID_MACRONIX   0xc20000

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void lc823450_spiflash_earlyinit(void);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_SPIFI2_H */
