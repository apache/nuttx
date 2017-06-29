/************************************************************************************
 * arch/arm/src/sama5/chip/sam_emaca.h
 * This is the form of the EMAC interface used the SAMA5D3
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_EMACA_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_EMACA_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* EMAC Register Offsets ************************************************************/

#define SAM_EMAC_NCR_OFFSET       0x0000 /* Network Control Register */
#define SAM_EMAC_NCFGR_OFFSET     0x0004 /* Network Configuration Register */
#define SAM_EMAC_NSR_OFFSET       0x0008 /* Network Status Register */
                                         /* 0x000c-0x0010 Reserved */
#define SAM_EMAC_TSR_OFFSET       0x0014 /* Transmit Status Register */
#define SAM_EMAC_RBQP_OFFSET      0x0018 /* Receive Buffer Queue Pointer Register */
#define SAM_EMAC_TBQP_OFFSET      0x001c /* Transmit Buffer Queue Pointer Register */
#define SAM_EMAC_RSR_OFFSET       0x0020 /* Receive Status Register */
#define SAM_EMAC_ISR_OFFSET       0x0024 /* Interrupt Status Register */
#define SAM_EMAC_IER_OFFSET       0x0028 /* Interrupt Enable Register */
#define SAM_EMAC_IDR_OFFSET       0x002c /* Interrupt Disable Register */
#define SAM_EMAC_IMR_OFFSET       0x0030 /* Interrupt Mask Register */
#define SAM_EMAC_MAN_OFFSET       0x0034 /* Phy Maintenance Register */
#define SAM_EMAC_PTR_OFFSET       0x0038 /* Pause Time Register */
#define SAM_EMAC_PFR_OFFSET       0x003c /* Pause Frames Received Register */
#define SAM_EMAC_FTO_OFFSET       0x0040 /* Frames Transmitted Ok Register */
#define SAM_EMAC_SCF_OFFSET       0x0044 /* Single Collision Frames Register */
#define SAM_EMAC_MCF_OFFSET       0x0048 /* Multiple Collision Frames Register */
#define SAM_EMAC_FRO_OFFSET       0x004c /* Frames Received Ok Register */
#define SAM_EMAC_FCSE_OFFSET      0x0050 /* Frame Check Sequence Errors Register */
#define SAM_EMAC_ALE_OFFSET       0x0054 /* Alignment Errors Register */
#define SAM_EMAC_DTF_OFFSET       0x0058 /* Deferred Transmission Frames Register */
#define SAM_EMAC_LCOL_OFFSET      0x005c /* Late Collisions Register */
#define SAM_EMAC_ECOL_OFFSET      0x0060 /* Excessive Collisions Register */
#define SAM_EMAC_TUND_OFFSET      0x0064 /* Transmit Underrun Errors Register */
#define SAM_EMAC_CSE_OFFSET       0x0068 /* Carrier Sense Errors Register */
#define SAM_EMAC_RRE_OFFSET       0x006c /* Receive Resource Errors Register */
#define SAM_EMAC_ROV_OFFSET       0x0070 /* Receive Overrun Errors Register */
#define SAM_EMAC_RSE_OFFSET       0x0074 /* Receive Symbol Errors Register */
#define SAM_EMAC_ELE_OFFSET       0x0078 /* Excessive Length Errors Register */
#define SAM_EMAC_RJA_OFFSET       0x007c /* Receive Jabbers Register */
#define SAM_EMAC_USF_OFFSET       0x0080 /* Undersize Frames Register */
#define SAM_EMAC_STE_OFFSET       0x0084 /* SQE Test Errors Register */
#define SAM_EMAC_RLE_OFFSET       0x0088 /* Received Length Field Mismatch Register */
#define SAM_EMAC_HRB_OFFSET       0x0090 /* Hash Register Bottom [31:0] Register */
#define SAM_EMAC_HRT_OFFSET       0x0094 /* Hash Register Top [63:32] Register */
#define SAM_EMAC_SA1B_OFFSET      0x0098 /* Specific Address 1 Bottom Register */
#define SAM_EMAC_SA1T_OFFSET      0x009c /* Specific Address 1 Top Register */
#define SAM_EMAC_SA2B_OFFSET      0x00a0 /* Specific Address 2 Bottom Register */
#define SAM_EMAC_SA2T_OFFSET      0x00a4 /* Specific Address 2 Top Register */
#define SAM_EMAC_SA3B_OFFSET      0x00a8 /* Specific Address 3 Bottom Register */
#define SAM_EMAC_SA3T_OFFSET      0x00ac /* Specific Address 3 Top Register */
#define SAM_EMAC_SA4B_OFFSET      0x00b0 /* Specific Address 4 Bottom Register */
#define SAM_EMAC_SA4T_OFFSET      0x00b4 /* Specific Address 4 Top Register */
#define SAM_EMAC_TID_OFFSET       0x00b8 /* Type ID Checking Register */
#define SAM_EMAC_USRIO_OFFSET     0x00c0 /* User Input/Output Register */
#define SAM_EMAC_WOL_OFFSET       0x00c4 /* Wake on LAN Register */
                                         /* 0x00c8-0x00fc Reserved */

/* EMAC Register Addresses **********************************************************/

#define SAM_EMAC_NCR              (SAM_EMAC_VBASE+SAM_EMAC_NCR_OFFSET)
#define SAM_EMAC_NCFGR            (SAM_EMAC_VBASE+SAM_EMAC_NCFGR_OFFSET)
#define SAM_EMAC_NSR              (SAM_EMAC_VBASE+SAM_EMAC_NSR_OFFSET)
#define SAM_EMAC_TSR              (SAM_EMAC_VBASE+SAM_EMAC_TSR_OFFSET)
#define SAM_EMAC_RBQP             (SAM_EMAC_VBASE+SAM_EMAC_RBQP_OFFSET)
#define SAM_EMAC_TBQP             (SAM_EMAC_VBASE+SAM_EMAC_TBQP_OFFSET)
#define SAM_EMAC_RSR              (SAM_EMAC_VBASE+SAM_EMAC_RSR_OFFSET)
#define SAM_EMAC_ISR              (SAM_EMAC_VBASE+SAM_EMAC_ISR_OFFSET)
#define SAM_EMAC_IER              (SAM_EMAC_VBASE+SAM_EMAC_IER_OFFSET)
#define SAM_EMAC_IDR              (SAM_EMAC_VBASE+SAM_EMAC_IDR_OFFSET)
#define SAM_EMAC_IMR              (SAM_EMAC_VBASE+SAM_EMAC_IMR_OFFSET)
#define SAM_EMAC_MAN              (SAM_EMAC_VBASE+SAM_EMAC_MAN_OFFSET)
#define SAM_EMAC_PTR              (SAM_EMAC_VBASE+SAM_EMAC_PTR_OFFSET)
#define SAM_EMAC_PFR              (SAM_EMAC_VBASE+SAM_EMAC_PFR_OFFSET)
#define SAM_EMAC_FTO              (SAM_EMAC_VBASE+SAM_EMAC_FTO_OFFSET)
#define SAM_EMAC_SCF              (SAM_EMAC_VBASE+SAM_EMAC_SCF_OFFSET)
#define SAM_EMAC_MCF              (SAM_EMAC_VBASE+SAM_EMAC_MCF_OFFSET)
#define SAM_EMAC_FRO              (SAM_EMAC_VBASE+SAM_EMAC_FRO_OFFSET)
#define SAM_EMAC_FCSE             (SAM_EMAC_VBASE+SAM_EMAC_FCSE_OFFSET)
#define SAM_EMAC_ALE              (SAM_EMAC_VBASE+SAM_EMAC_ALE_OFFSET)
#define SAM_EMAC_DTF              (SAM_EMAC_VBASE+SAM_EMAC_DTF_OFFSET)
#define SAM_EMAC_LCOL             (SAM_EMAC_VBASE+SAM_EMAC_LCOL_OFFSET)
#define SAM_EMAC_ECOL             (SAM_EMAC_VBASE+SAM_EMAC_ECOL_OFFSET)
#define SAM_EMAC_TUND             (SAM_EMAC_VBASE+SAM_EMAC_TUND_OFFSET)
#define SAM_EMAC_CSE              (SAM_EMAC_VBASE+SAM_EMAC_CSE_OFFSET)
#define SAM_EMAC_RRE              (SAM_EMAC_VBASE+SAM_EMAC_RRE_OFFSET)
#define SAM_EMAC_ROV              (SAM_EMAC_VBASE+SAM_EMAC_ROV_OFFSET)
#define SAM_EMAC_RSE              (SAM_EMAC_VBASE+SAM_EMAC_RSE_OFFSET)
#define SAM_EMAC_ELE              (SAM_EMAC_VBASE+SAM_EMAC_ELE_OFFSET)
#define SAM_EMAC_RJA              (SAM_EMAC_VBASE+SAM_EMAC_RJA_OFFSET)
#define SAM_EMAC_USF              (SAM_EMAC_VBASE+SAM_EMAC_USF_OFFSET)
#define SAM_EMAC_STE              (SAM_EMAC_VBASE+SAM_EMAC_STE_OFFSET)
#define SAM_EMAC_RLE              (SAM_EMAC_VBASE+SAM_EMAC_RLE_OFFSET)
#define SAM_EMAC_HRB              (SAM_EMAC_VBASE+SAM_EMAC_HRB_OFFSET)
#define SAM_EMAC_HRT              (SAM_EMAC_VBASE+SAM_EMAC_HRT_OFFSET)
#define SAM_EMAC_SA1B             (SAM_EMAC_VBASE+SAM_EMAC_SA1B_OFFSET)
#define SAM_EMAC_SA1T             (SAM_EMAC_VBASE+SAM_EMAC_SA1T_OFFSET)
#define SAM_EMAC_SA2B             (SAM_EMAC_VBASE+SAM_EMAC_SA2B_OFFSET)
#define SAM_EMAC_SA2T             (SAM_EMAC_VBASE+SAM_EMAC_SA2T_OFFSET)
#define SAM_EMAC_SA3B             (SAM_EMAC_VBASE+SAM_EMAC_SA3B_OFFSET)
#define SAM_EMAC_SA3T             (SAM_EMAC_VBASE+SAM_EMAC_SA3T_OFFSET)
#define SAM_EMAC_SA4B             (SAM_EMAC_VBASE+SAM_EMAC_SA4B_OFFSET)
#define SAM_EMAC_SA4T             (SAM_EMAC_VBASE+SAM_EMAC_SA4T_OFFSET)
#define SAM_EMAC_TID              (SAM_EMAC_VBASE+SAM_EMAC_TID_OFFSET)
#define SAM_EMAC_USRIO            (SAM_EMAC_VBASE+SAM_EMAC_USRIO_OFFSET)
#define SAM_EMAC_WOL              (SAM_EMAC_VBASE+SAM_EMAC_WOL_OFFSET)

/* EMAC Register Bit Definitions ****************************************************/

/* Network Control Register */

#define EMAC_NCR_LB               (1 << 0)  /* Bit 0:  LoopBack */
#define EMAC_NCR_LLB              (1 << 1)  /* Bit 1:  Loopback local */
#define EMAC_NCR_RE               (1 << 2)  /* Bit 2:  Receive enable */
#define EMAC_NCR_TE               (1 << 3)  /* Bit 3:  Transmit enable */
#define EMAC_NCR_MPE              (1 << 4)  /* Bit 4:  Management port enable */
#define EMAC_NCR_CLRSTAT          (1 << 5)  /* Bit 5:  Clear statistics registers */
#define EMAC_NCR_INCSTAT          (1 << 6)  /* Bit 6:  Increment statistics registers */
#define EMAC_NCR_WESTAT           (1 << 7)  /* Bit 7:  Write enable for statistics registers */
#define EMAC_NCR_BP               (1 << 8)  /* Bit 8:  Back pressure */
#define EMAC_NCR_TSTART           (1 << 9)  /* Bit 9:  Start transmission */
#define EMAC_NCR_THALT            (1 << 10) /* Bit 10: Transmit halt */

/* Network Configuration Register */

#define EMAC_NCFGR_SPD            (1 << 0)  /* Bit 0:  Speed */
#define EMAC_NCFGR_FD             (1 << 1)  /* Bit 1:  Full Duplex */
#define EMAC_NCFGR_JFRAME         (1 << 3)  /* Bit 3:  Jumbo Frames */
#define EMAC_NCFGR_CAF            (1 << 4)  /* Bit 4:  Copy All Frames */
#define EMAC_NCFGR_NBC            (1 << 5)  /* Bit 5:  No Broadcast */
#define EMAC_NCFGR_MTI            (1 << 6)  /* Bit 6:  Multicast Hash Enable */
#define EMAC_NCFGR_UNI            (1 << 7)  /* Bit 7:  Unicast Hash Enable */
#define EMAC_NCFGR_BIG            (1 << 8)  /* Bit 8:  Receive 1536 bytes frames */
#define EMAC_NCFGR_CLK_SHIFT      (10)      /* Bits 10-11: MDC clock divider */
#define EMAC_NCFGR_CLK_MASK       (3 << EMAC_NCFGR_CLK_SHIFT)
#  define EMAC_NCFGR_CLK_DIV8     (0 << EMAC_NCFGR_CLK_SHIFT) /* MCK divided by 8 (MCK up to 20 MHz) */
#  define EMAC_NCFGR_CLK_DIV16    (1 << EMAC_NCFGR_CLK_SHIFT) /* MCK divided by 16 (MCK up to 40 MHz) */
#  define EMAC_NCFGR_CLK_DIV32    (2 << EMAC_NCFGR_CLK_SHIFT) /* MCK divided by 32 (MCK up to 80 MHz) */
#  define EMAC_NCFGR_CLK_DIV64    (3 << EMAC_NCFGR_CLK_SHIFT) /* MCK divided by 64 (MCK up to 160 MHz) */
#define EMAC_NCFGR_RTY            (1 << 12) /* Bit 12: Retry test */
#define EMAC_NCFGR_PAE            (1 << 13) /* Bit 13: Pause Enable */
#define EMAC_NCFGR_RBOF_SHIFT     (14)      /* Bits 14-15: Receive Buffer Offset */
#define EMAC_NCFGR_RBOF_MASK      (3 << EMAC_NCFGR_RBOF_SHIFT)
#  define EMAC_NCFGR_RBOF_NONE    (0 << EMAC_NCFGR_RBOF_SHIFT) /* No offset from RX buffer start */
#  define EMAC_NCFGR_RBOF_1       (1 << EMAC_NCFGR_RBOF_SHIFT) /* One-byte offset from RX buffer start */
#  define EMAC_NCFGR_RBOF_2       (2 << EMAC_NCFGR_RBOF_SHIFT) /* Two-byte offset from RX buffer start */
#  define EMAC_NCFGR_RBOF_3       (3 << EMAC_NCFGR_RBOF_SHIFT) /* Three-byte offset fromRX buffer start */
#define EMAC_NCFGR_RLCE           (1 << 16) /* Bit 16: Receive Length field Checking Enable */
#define EMAC_NCFGR_DRFCS          (1 << 17) /* Bit 17: Discard Receive FCS */
#define EMAC_NCFGR_EFRHD          (1 << 18) /* Bit 18: Enable RX frames in HD mode while transmitting */
#define EMAC_NCFGR_IRXFCS         (1 << 19) /* Bit 19: Ignore RX FCS */

/* Network Status Register */

#define EMAC_NSR_MDIO             (1 << 1)  /* Bit 1:  Status of the mdio_in pin */
#define EMAC_NSR_IDLE             (1 << 2)  /* Bit 2:  PHY management logic is idle */

/* Transmit Status Register */

#define EMAC_TSR_UBR              (1 << 0)  /* Bit 0:  Used Bit Read */
#define EMAC_TSR_COL              (1 << 1)  /* Bit 1:  Collision Occurred */
#define EMAC_TSR_RLES             (1 << 2)  /* Bit 2:  Retry Limit exceeded */
#define EMAC_TSR_TGO              (1 << 3)  /* Bit 3:  Transmit Go */
#define EMAC_TSR_BEX              (1 << 4)  /* Bit 4:  Buffers exhausted mid frame */
#define EMAC_TSR_COMP             (1 << 5)  /* Bit 5:  Transmit Complete */
#define EMAC_TSR_UND              (1 << 6)  /* Bit 6:  Transmit Underrun */

/* Receive Buffer Queue Pointer Register */

#define EMAC_RBQP_MASK            (0xfffffffc)  /* Bits 2-31: Receive buffer queue pointer address */

/* Transmit Buffer Queue Pointer Register */

#define EMAC_TBQP_MASK            (0xfffffffc)  /* Bits 2-31: Transmit buffer queue pointer address */

/* Receive Status Register */

#define EMAC_RSR_BNA              (1 << 0)  /* Bit 0:  Buffer Not Available */
#define EMAC_RSR_REC              (1 << 1)  /* Bit 1:  Frame Received */
#define EMAC_RSR_OVR              (1 << 2)  /* Bit 2:  Receive Overrun */

/* Interrupt Status Register (ISR), Interrupt Enable Register (IER), Interrupt Disable Register (IDR) and Interrupt Mask Register (IMR) */

#define EMAC_INT_MFD              (1 << 0)  /* Bit 0:  Management Frame Done */
#define EMAC_INT_RCOMP            (1 << 1)  /* Bit 1:  Receive Complete */
#define EMAC_INT_RXUBR            (1 << 2)  /* Bit 2:  Receive Used Bit Read */
#define EMAC_INT_TXUBR            (1 << 3)  /* Bit 3:  Transmit Used Bit Read */
#define EMAC_INT_TUND             (1 << 4)  /* Bit 4:  Ethernet Transmit Buffer Underrun */
#define EMAC_INT_RLE              (1 << 5)  /* Bit 5:  Retry Limit Exceeded */
#define EMAC_INT_TXERR            (1 << 6)  /* Bit 6:  Transmit Error */
#define EMAC_INT_TCOMP            (1 << 7)  /* Bit 7:  Transmit Complete */
#define EMAC_INT_ROVR             (1 << 10) /* Bit 10: Receive Overrun */
#define EMAC_INT_HRESP            (1 << 11) /* Bit 11: Hresp not OK */
#define EMAC_INT_PFR              (1 << 12) /* Bit 12: Pause Frame Received */
#define EMAC_INT_PTZ              (1 << 13) /* Bit 13: Pause Time Zero */
#define EMAC_INT_WOL              (1 << 14) /* Bit 14: Wake On LAN */

#define EMAC_INT_ALL              (0x00007cff)
#define EMAC_INT_UNUSED           (0xffff8300)

/* Phy Maintenance Register */

#define EMAC_MAN_DATA_SHIFT       (0)       /* Bits 0-15: Read/write data */
#define EMAC_MAN_DATA_MASK        (0x0000ffff << EMAC_MAN_DATA_SHIFT)
#  define EMAC_MAN_DATA(n)        ((uint32_t)(n) << EMAC_MAN_DATA_SHIFT)
#define EMAC_MAN_CODE_SHIFT       (16)      /* Bits 16-17:  Must be written to b10 */
#define EMAC_MAN_CODE_MASK        (3 << EMAC_MAN_CODE_SHIFT)
#  define EMAC_MAN_CODE           (2 << EMAC_MAN_CODE_SHIFT)
#define EMAC_MAN_REGA_SHIFT       (18)      /* Bits 18-22: Register Address */
#define EMAC_MAN_REGA_MASK        (31 << EMAC_MAN_REGA_SHIFT)
#  define EMAC_MAN_REGA(n)        ((uint32_t)(n) << EMAC_MAN_REGA_SHIFT)
#define EMAC_MAN_PHYA_SHIFT       (23)      /* Bits 23-27: PHY Address */
#define EMAC_MAN_PHYA_MASK        (31 << EMAC_MAN_PHYA_SHIFT)
#  define EMAC_MAN_PHYA(n)        ((uint32_t)(n) << EMAC_MAN_PHYA_SHIFT)
#define EMAC_MAN_RW_SHIFT         (28)      /* Bits 28-29: Read-write */
#define EMAC_MAN_RW_MASK          (3 << EMAC_MAN_RW_SHIFT)
#  define EMAC_MAN_READ           (2 << EMAC_MAN_RW_SHIFT)
#  define EMAC_MAN_WRITE          (1 << EMAC_MAN_RW_SHIFT)
#define EMAC_MAN_SOF_SHIFT        (30)      /* Bits 30-31: Start of frame */
#define EMAC_MAN_SOF_MASK         (3 << EMAC_MAN_SOF_SHIFT)
#  define EMAC_MAN_SOF            (1 << EMAC_MAN_SOF_SHIFT) /* Must be written b01 */

/* Pause Time Register */

#define EMAC_PTR_MASK             (0x0000ffff) /* Bits 0-15: Pause Time */

/* Pause Frames Received Register */

#define EMAC_PFR_MASK             (0x0000ffff) /* Bits 0-15: Pause Frames received OK */

/* Frames Transmitted Ok Register */

#define EMAC_FTO_MASK             (0x00ffffff) /* Bits 0-23: Frames Transmitted OK */

/* Single Collision Frames Register */

#define EMAC_SCF_MASK             (0x0000ffff) /* Bits 0-15: Single Collision Frames */

/* Multiple Collision Frames Register */

#define EMAC_MCF_MASK             (0x0000ffff) /* Bits 0-15: Multicollision Frames */

/* Frames Received Ok Register */

#define EMAC_FRO_MASK             (0x00ffffff) /* Bits 0-23: Frames received OK */

/* Frame Check Sequence Errors Register */

#define EMAC_FCSE_MASK            (0x000000ff) /* Bits 0-7:Frame Check Sequence Errors */

/* Alignment Errors Register */

#define EMAC_ALE_MASK             (0x000000ff) /* Bits 0-7:Alignment Errors */

/* Deferred Transmission Frames Register */

#define EMAC_DTF_MASK             (0x0000ffff) /* Bits 0-15: Deferred Transmission Frames */

/* Late Collisions Register */

#define EMAC_LCOL_MASK            (0x000000ff) /* Bits 0-7: Late Collisions */

/* Excessive Collisions Register */

#define EMAC_ECOL_MASK            (0x000000ff) /* Bits 0-7: Excessive Collisions Register */

/* Transmit Underrun Errors Register */

#define EMAC_TUND_MASK            (0x000000ff) /* Bits 0-7: Transmit Underruns */

/* Carrier Sense Errors Register */

#define EMAC_CSE_MASK             (0x000000ff) /* Bits 0-7: Carrier Sense Errors */

/* Receive Resource Errors Register */

#define EMAC_RRE_MASK             (0x0000ffff) /* Bits 0-15: Receive Resource Errors */

/* Receive Overrun Errors Register */

#define EMAC_ROV_MASK             (0x000000ff) /* Bits 0-7: Receive Overrun */

/* Receive Symbol Errors Register */

#define EMAC_RSE_MASK             (0x000000ff) /* Bits 0-7: Receive Symbol Errors */

/* Excessive Length Errors Register */

#define EMAC_ELE_MASK             (0x000000ff) /* Bits 0-7: Excessive Length Errors */

/* Receive Jabbers Register */

#define EMAC_RJA_MASK             (0x000000ff) /* Bits 0-7: Receive Jabbers */

/* Undersize Frames Register */

#define EMAC_USF_MASK             (0x000000ff) /* Bits 0-7: Undersize frames */

/* SQE Test Errors Register */

#define EMAC_STE_MASK             (0x000000ff) /* Bits 0-7: SQE test errors */

/* Received Length Field Mismatch Register */

#define EMAC_RLE_MASK             (0x000000ff) /* Bits 0-7: Receive Length Field Mismatch */

/* Hash Register Bottom [31:0] Register (LS 32-bit hash address) */
/* Hash Register Top [63:32] Register (MS 32-bit hash address) */

/* Specific Address 1 Bottom [31:0] Register (LS 32-bit address) */
/* Specific Address 1 Top [47:32] Register */

#define EMAC_SA1T_MASK            (0x0000ffff) /* Bits 0-15: Bits 32-47 of the destination address */

/* Specific Address 2 Bottom [31:0] Register (LS 32-bit address) */
/* Specific Address 2 Top [47:32] Register */

#define EMAC_SA2T_MASK            (0x0000ffff) /* Bits 0-15: Bits 32-47 of the destination address */

/* Specific Address 3 Bottom [31:0] Register (LS 32-bit address) */
/* Specific Address 3 Top [47:32] Register */

#define EMAC_SA3T_MASK            (0x0000ffff) /* Bits 0-15: Bits 32-47 of the destination address */

/* Specific Address 4 Bottom [31:0] Register (LS 32-bit address) */
/* Specific Address 4 Top [47:32] Register */

#define EMAC_SA4T_MASK            (0x0000ffff) /* Bits 0-15: Bits 32-47 of the destination address */

/* Type ID Checking Register */

#define EMAC_TID_MASK             (0x0000ffff) /* Bits 0-15: For comparisons with received frames TypeID/Length field. */

/* User Input/Output Register */

#define EMAC_USRIO_RMII           (1 << 0)  /* Bit 0:  Reduce MII */
#define EMAC_USRIO_CLKEN          (1 << 1)  /* Bit 1:  Clock Enable */

/* Wake on LAN Register */

#define EMAC_WOL_IP_SHIFT         (0)       /* Bits 0-15: ARP request IP address */
#define EMAC_WOL_IP_MASK          (0x0000ffff << EMAC_WOL_IP_SHIFT)
#define EMAC_WOL_MAG              (1 << 16) /* Bit 16: Magic packet event enable */
#define EMAC_WOL_ARP              (1 << 17) /* Bit 17: ARP request event enable */
#define EMAC_WOL_SA1              (1 << 18) /* Bit 18: Specific address register 1 event enable */
#define EMAC_WOL_MTI              (1 << 19) /* Bit 19: Multicast hash event enable */

/* Descriptors **********************************************************************/

/* Receive buffer descriptor:  Address word */

#define EMACRXD_ADDR_OWNER        (1 << 0)  /* Bit 0:  1=Software owns; 0=EMAC owns */
#define EMACRXD_ADDR_WRAP         (1 << 1)  /* Bit 1:  Last descriptor in list */
#define EMACRXD_ADDR_MASK         (0xfffffffc) /* Bits 2-31: Aligned buffer address */

/* Receive buffer descriptor:  Control word */

#define EMACRXD_STA_FRLEN_SHIFT   (0)       /* Bits 0-11: Length of frame */
#define EMACRXD_STA_FRLEN_MASK    (0x000007ff << EMACRXD_STA_FRLEN_SHIFT)
#define EMACRXD_STA_BOFFS_SHIFT   (12)      /* Bits 12-13: Receive buffer offset */
#define EMACRXD_STA_BOFFS_MASK    (3 << EMACRXD_STA_BOFFS_SHIFT)
#define EMACRXD_STA_SOF           (1 << 14) /* Bit 14: Start of frame */
#define EMACRXD_STA_EOF           (1 << 15) /* Bit 15: End of frame */
#define EMACRXD_STA_CFI           (1 << 16) /* Bit 16: Concatenation format indicator (CFI) bit */
#define EMACRXD_STA_VLPRIO_SHIFT  (17)      /* Bits 17-19: VLAN priority */
#define EMACRXD_STA_VLPRIO_MASK   (7 << EMACRXD_STA_VLANPRIO_SHIFT)
#define EMACRXD_STA_PRIODET       (1 << 20) /* Bit 20: Priority tag detected */
#define EMACRXD_STA_VLANTAG       (1 << 21) /* Bit 21: VLAN tag detected */
#define EMACRXD_STA_TYPEID        (1 << 22) /* Bit 22: Type ID match */
#define EMACRXD_STA_ADDR4         (1 << 23) /* Bit 23: Specific address register 4 match */
#define EMACRXD_STA_ADDR3         (1 << 24) /* Bit 24: Specific address register 3 match */
#define EMACRXD_STA_ADDR2         (1 << 25) /* Bit 25: Specific address register 2 match */
#define EMACRXD_STA_ADDR1         (1 << 26) /* Bit 26: Specific address register 1 match */
                                            /* Bit 27: Reserved */
#define EMACRXD_STA_EXTADDR       (1 << 28) /* Bit 28: External address match */
#define EMACRXD_STA_UCAST         (1 << 29) /* Bit 29: Unicast hash match */
#define EMACRXD_STA_MCAST         (1 << 30) /* Bit 30: Multicast hash match */
#define EMACRXD_STA_BCAST         (1 << 31) /* Bit 31: Global all ones broadcast address detected */

/* Transmit buffer descriptor:  Address word (un-aligned, 32-bit address */

/* Transmit buffer descriptor:  Control word */

#define EMACTXD_STA_BUFLEN_SHIFT  (0)       /* Bits 0-10: Length of buffer */
#define EMACTXD_STA_BUFLEN_MASK   (0x000003ff << EMACTXD_STA_BUFLEN_SHIFT)
                                            /* Bits 11-14: Reserved */
#define EMACTXD_STA_LAST          (1 << 15) /* Bit 15: Last buffer in the current frame */
#define EMACTXD_STA_NOCRC         (1 << 16) /* Bit 16: No CRC */
                                            /* Bits 17-26: Reserved */
#define EMACTXD_STA_NOBUFFER      (1 << 27) /* Bit 27: Buffers exhausted in mid frame */
#define EMACTXD_STA_TXUR          (1 << 28) /* Bit 28: Transmit underrun */
#define EMACTXD_STA_TXERR         (1 << 29) /* Bit 29: Retry limit exceeded, transmit error detected */
#define EMACTXD_STA_WRAP          (1 << 30) /* Bit 30: Last descriptor in descriptor list */
#define EMACTXD_STA_USED          (1 << 31) /* Bit 31: Zero for the EMAC to read from buffer */

/************************************************************************************
 * Public Types
 ************************************************************************************/
/* Receive buffer descriptor */

struct emac_rxdesc_s
{
  uint32_t addr;     /* Buffer address */
  uint32_t status;   /* RX status and controls */
};

/* Transmit buffer descriptor */

struct emac_txdesc_s
{
  uint32_t addr;     /* Buffer address */
  uint32_t status;   /* TX status and controls */
};

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_EMACA_H */
