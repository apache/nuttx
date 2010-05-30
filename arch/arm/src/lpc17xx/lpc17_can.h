/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_can.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_CAN_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_CAN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "lp17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* CAN acceptance filter registers */

#define LPC17_CANAF_AFMR_OFFSET     0x0000 /* Acceptance Filter Register */
#define LPC17_CANAF_SFFSA_OFFSET    0x0004 /* Standard Frame Individual Start Address Register */
#define LPC17_CANAF_SFFGRPSA_OFFSET 0x0008 /* Standard Frame Group Start Address Register */
#define LPC17_CANAF_EFFSA_OFFSET    0x000c /* Extended Frame Start Address Register */
#define LPC17_CANAF_EFFGRPSA_OFFSET 0x0010 /* Extended Frame Group Start Address Register */
#define LPC17_CANAF_EOT_OFFSET      0x0014 /* End of AF Tables register */
#define LPC17_CANAF_LUTERRAD_OFFSET 0x0018 /* LUT Error Address register */
#define LPC17_CANAF_LUTERR_OFFSET   0x001c /* LUT Error Register */

/* Central CAN registers */

#define LPC17_CAN_TXSR_OFFSET       0x0000 /* CAN Central Transmit Status Register */
#define LPC17_CAN_RXSR_OFFSET       0x0004 /* CAN Central Receive Status Register */
#define LPC17_CAN_MSR_OFFSET        0x0008 /* CAN Central Miscellaneous Register */

/* CAN1/2 registers */

#define LPC17_CAN_BTR_OFFSET        0x0014 /* Bus Timing */
#define LPC17_CAN_EWL_OFFSET        0x0018 /* Error Warning Limit */
#define LPC17_CAN_SR_OFFSET         0x001c /* Status Register */
#define LPC17_CAN_RFS_OFFSET        0x0020 /* Receive frame status */
#define LPC17_CAN_RID_OFFSET        0x0024 /* Received Identifier */
#define LPC17_CAN_RDA_OFFSET        0x0028 /* Received data bytes 1-4 */
#define LPC17_CAN_RDB_OFFSET        0x002c /* Received data bytes 5-8 */
#define LPC17_CAN_TFI1_OFFSET       0x0030 /* Transmit frame info (Tx Buffer 1) */
#define LPC17_CAN_TID1_OFFSET       0x0034 /* Transmit Identifier (Tx Buffer 1) */
#define LPC17_CAN_TDA1_OFFSET       0x0038 /* Transmit data bytes 1-4 (Tx Buffer 1) */
#define LPC17_CAN_TDB1_OFFSET       0x003c /* Transmit data bytes 5-8 (Tx Buffer 1) */
#define LPC17_CAN_TFI2_OFFSET       0x0040 /* Transmit frame info (Tx Buffer 2) */
#define LPC17_CAN_TID2_OFFSET       0x0044 /* Transmit Identifier (Tx Buffer 2) */
#define LPC17_CAN_TDA2_OFFSET       0x0048 /* Transmit data bytes 1-4 (Tx Buffer 2) */
#define LPC17_CAN_TDB2_OFFSET       0x004c /* Transmit data bytes 5-8 (Tx Buffer 2) */
#define LPC17_CAN_TFI3_OFFSET       0x0050 /* Transmit frame info (Tx Buffer 3) */
#define LPC17_CAN_TID3_OFFSET       0x0054 /* Transmit Identifier (Tx Buffer 3) */
#define LPC17_CAN_TDA3_OFFSET       0x0058 /* Transmit data bytes 1-4 (Tx Buffer 3) */
#define LPC17_CAN_TDB3_OFFSET       0x005c /* Transmit data bytes 5-8 (Tx Buffer 3) */

/* Register addresses ***************************************************************/
/* CAN acceptance filter registers */

#define LPC17_CANAF_AFMR            (LPC17_CANAF_BASE+LPC17_CANAF_AFMR_OFFSET)
#define LPC17_CANAF_SFFSA           (LPC17_CANAF_BASE+LPC17_CANAF_SFFSA_OFFSET)
#define LPC17_CANAF_SFFGRPSA        (LPC17_CANAF_BASE+LPC17_CANAF_SFFGRPSA_OFFSET)
#define LPC17_CANAF_EFFSA           (LPC17_CANAF_BASE+LPC17_CANAF_EFFSA_OFFSET)
#define LPC17_CANAF_EFFGRPSA        (LPC17_CANAF_BASE+LPC17_CANAF_EFFGRPSA_OFFSET)
#define LPC17_CANAF_EOT             (LPC17_CANAF_BASE+LPC17_CANAF_EOT_OFFSET)
#define LPC17_CANAF_LUTERRAD        (LPC17_CANAF_BASE+LPC17_CANAF_LUTERRAD_OFFSET)
#define LPC17_CANAF_LUTERR          (LPC17_CANAF_BASE+LPC17_CANAF_LUTERR_OFFSET)

/* Central CAN registers */

#define LPC17_CAN_TXSR              (LPC17_CAN_BASE+LPC17_CAN_TXSR_OFFSET)
#define LPC17_CAN_RXSR              (LPC17_CAN_BASE+LPC17_CAN_RXSR_OFFSET)
#define LPC17_CAN_MSR               (LPC17_CAN_BASE+LPC17_CAN_MSR_OFFSET)

/* CAN1/2 registers */

#define LPC17_CAN1_BTR              (LPC17_CAN1_BASE+LPC17_CAN_BTR_OFFSET)
#define LPC17_CAN1_EWL              (LPC17_CAN1_BASE+LPC17_CAN_EWL_OFFSET)
#define LPC17_CAN1_SR               (LPC17_CAN1_BASE+LPC17_CAN_SR_OFFSET)
#define LPC17_CAN1_RFS              (LPC17_CAN1_BASE+LPC17_CAN_RFS_OFFSET)
#define LPC17_CAN1_RID              (LPC17_CAN1_BASE+LPC17_CAN_RID_OFFSET)
#define LPC17_CAN1_RDA              (LPC17_CAN1_BASE+LPC17_CAN_RDA_OFFSET)
#define LPC17_CAN1_RDB              (LPC17_CAN1_BASE+LPC17_CAN_RDB_OFFSET)
#define LPC17_CAN1_TFI1             (LPC17_CAN1_BASE+LPC17_CAN_TFI1_OFFSET)
#define LPC17_CAN1_TID1             (LPC17_CAN1_BASE+LPC17_CAN_TID1_OFFSET)
#define LPC17_CAN1_TDA1             (LPC17_CAN1_BASE+LPC17_CAN_TDA1_OFFSET)
#define LPC17_CAN1_TDB1             (LPC17_CAN1_BASE+LPC17_CAN_TDB1_OFFSET)
#define LPC17_CAN1_TFI2             (LPC17_CAN1_BASE+LPC17_CAN_TFI2_OFFSET)
#define LPC17_CAN1_TID2             (LPC17_CAN1_BASE+LPC17_CAN_TID2_OFFSET)
#define LPC17_CAN1_TDA2             (LPC17_CAN1_BASE+LPC17_CAN_TDA2_OFFSET)
#define LPC17_CAN1_TDB2             (LPC17_CAN1_BASE+LPC17_CAN_TDB2_OFFSET)
#define LPC17_CAN1_TFI3             (LPC17_CAN1_BASE+LPC17_CAN_TFI3_OFFSET)
#define LPC17_CAN1_TID3             (LPC17_CAN1_BASE+LPC17_CAN_TID3_OFFSET)
#define LPC17_CAN1_TDA3             (LPC17_CAN1_BASE+LPC17_CAN_TDA3_OFFSET)
#define LPC17_CAN1_TDB3             (LPC17_CAN1_BASE+LPC17_CAN_TDB3_OFFSET)

#define LPC17_CAN2_BTR              (LPC17_CAN2_BASE+LPC17_CAN_BTR_OFFSET)
#define LPC17_CAN2_EWL              (LPC17_CAN2_BASE+LPC17_CAN_EWL_OFFSET)
#define LPC17_CAN2_SR               (LPC17_CAN2_BASE+LPC17_CAN_SR_OFFSET)
#define LPC17_CAN2_RFS              (LPC17_CAN2_BASE+LPC17_CAN_RFS_OFFSET)
#define LPC17_CAN2_RID              (LPC17_CAN2_BASE+LPC17_CAN_RID_OFFSET)
#define LPC17_CAN2_RDA              (LPC17_CAN2_BASE+LPC17_CAN_RDA_OFFSET)
#define LPC17_CAN2_RDB              (LPC17_CAN2_BASE+LPC17_CAN_RDB_OFFSET)
#define LPC17_CAN2_TFI1             (LPC17_CAN2_BASE+LPC17_CAN_TFI1_OFFSET)
#define LPC17_CAN2_TID1             (LPC17_CAN2_BASE+LPC17_CAN_TID1_OFFSET)
#define LPC17_CAN2_TDA1             (LPC17_CAN2_BASE+LPC17_CAN_TDA1_OFFSET)
#define LPC17_CAN2_TDB1             (LPC17_CAN2_BASE+LPC17_CAN_TDB1_OFFSET)
#define LPC17_CAN2_TFI2             (LPC17_CAN2_BASE+LPC17_CAN_TFI2_OFFSET)
#define LPC17_CAN2_TID2             (LPC17_CAN2_BASE+LPC17_CAN_TID2_OFFSET)
#define LPC17_CAN2_TDA2             (LPC17_CAN2_BASE+LPC17_CAN_TDA2_OFFSET)
#define LPC17_CAN2_TDB2             (LPC17_CAN2_BASE+LPC17_CAN_TDB2_OFFSET)
#define LPC17_CAN2_TFI3             (LPC17_CAN2_BASE+LPC17_CAN_TFI3_OFFSET)
#define LPC17_CAN2_TID3             (LPC17_CAN2_BASE+LPC17_CAN_TID3_OFFSET)
#define LPC17_CAN2_TDA3             (LPC17_CAN2_BASE+LPC17_CAN_TDA3_OFFSET)
#define LPC17_CAN2_TDB3             (LPC17_CAN2_BASE+LPC17_CAN_TDB3_OFFSET)

/* Register bit definitions *********************************************************/
/* CAN acceptance filter registers */
/* Acceptance Filter Register */
#define CANAF_AFMR_
/* Standard Frame Individual Start Address Register */
#define CANAF_SFFSA_
/* Standard Frame Group Start Address Register */
#define CANAF_SFFGRPSA_
/* Extended Frame Start Address Register */
#define CANAF_EFFSA_
/* Extended Frame Group Start Address Register */
#define CANAF_EFFGRPSA_
/* End of AF Tables register */
#define CANAF_EOT_
/* LUT Error Address register */
#define CANAF_LUTERRAD_
/* LUT Error Register */
#define CANAF_LUTERR_

/* Central CAN registers */
/* CAN Central Transmit Status Register */
#define CAN_TXSR_
/* CAN Central Receive Status Register */
#define CAN_RXSR_
/* CAN Central Miscellaneous Register */
#define CAN_MSR_

/* CAN1/2 registers */
/* Bus Timing */
#define CAN_BTR_
/* Error Warning Limit */
#define CAN_EWL_
/* Status Register */
#define CAN_SR_
/* Receive frame status */
#define CAN_RFS_
/* Received Identifier */
#define CAN_RID_
/* Received data bytes 1-4 */
#define CAN_RDA_
/* Received data bytes 5-8 */
#define CAN_RDB_
/* Transmit frame info (Tx Buffer 1) */
#define CAN_TFI1_
/* Transmit Identifier (Tx Buffer 1) */
#define CAN_TID1_
/* Transmit data bytes 1-4 (Tx Buffer 1) */
#define CAN_TDA1_
/* Transmit data bytes 5-8 (Tx Buffer 1) */
#define CAN_TDB1_
/* Transmit frame info (Tx Buffer 2) */
#define CAN_TFI2_
/* Transmit Identifier (Tx Buffer 2) */
#define CAN_TID2_
/* Transmit data bytes 1-4 (Tx Buffer 2) */
#define CAN_TDA2_
/* Transmit data bytes 5-8 (Tx Buffer 2) */
#define CAN_TDB2_
/* Transmit frame info (Tx Buffer 3) */
#define CAN_TFI3_
/* Transmit Identifier (Tx Buffer 3) */
#define CAN_TID3_
/* Transmit data bytes 1-4 (Tx Buffer 3) */
#define CAN_TDA3_
/* Transmit data bytes 5-8 (Tx Buffer 3) */
#define CAN_TDB3_

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_CAN_H */
