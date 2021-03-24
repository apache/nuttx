/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_udphs.h
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

/* References:
 *   SAMA5D3 Series Data Sheet
 */

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_UDPHS_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_UDPHS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General Definitions ******************************************************/

/* Number of endpoints and DMA channels */

#define SAM_UDPHS_NENDPOINTS            16     /* EP0-15 */
#define SAM_UDPHS_NDMACHANNELS           7     /* For EP1-7 */

/* Capabilities and characteristics of endpoints */

#define SAM_UDPHS_MAXPACKETSIZE(ep) \
   (((unsigned)(ep) < 1) ? 64 : 1024)
#define SAM_UDPHS_NBANKS(ep) \
   (((unsigned)(ep) < 1) ? 1 : (((unsigned)(ep) < 3) ? 3 : 2))
#define SAM_UDPHS_DMA(ep) \
   (((unsigned)(ep) < 1) ? false : (((unsigned)(ep) < 8) ? true : false))

/* Register offsets *********************************************************/

/* Global Registers */

#define SAM_UDPHS_CTRL_OFFSET           0x0000 /* UDPHS Control Register */
#define SAM_UDPHS_FNUM_OFFSET           0x0004 /* UDPHS Frame Number Register */
                                               /* 0x0008-0x000c Reserved */
#define SAM_UDPHS_IEN_OFFSET            0x0010 /* UDPHS Interrupt Enable Register */
#define SAM_UDPHS_INTSTA_OFFSET         0x0014 /* UDPHS Interrupt Status Register */
#define SAM_UDPHS_CLRINT_OFFSET         0x0018 /* UDPHS Clear Interrupt Register */
#define SAM_UDPHS_EPTRST_OFFSET         0x001c /* UDPHS Endpoints Reset Register */
                                               /* 0x0020-0x00cc Reserved */
#define SAM_UDPHS_TST_OFFSET            0x00e0 /* UDPHS Test Register */
                                               /* 0x00e4-0x00e8 Reserved */

/* Endpoint Offsets */

#define SAM_UDPHS_EP_OFFSET(ep)         (0x0100+((unsigned int)(ep)<<5))
#define SAM_UDPHS_EP0_OFFSET            0x0100
#define SAM_UDPHS_EP1_OFFSET            0x0120
#define SAM_UDPHS_EP2_OFFSET            0x0140
#define SAM_UDPHS_EP3_OFFSET            0x0160
#define SAM_UDPHS_EP4_OFFSET            0x0180
#define SAM_UDPHS_EP5_OFFSET            0x01a0
#define SAM_UDPHS_EP6_OFFSET            0x01c0
#define SAM_UDPHS_EP7_OFFSET            0x01e0
#define SAM_UDPHS_EP8_OFFSET            0x0200
#define SAM_UDPHS_EP9_OFFSET            0x0220
#define SAM_UDPHS_EP10_OFFSET           0x0240
#define SAM_UDPHS_EP11_OFFSET           0x0260
#define SAM_UDPHS_EP12_OFFSET           0x0280
#define SAM_UDPHS_EP13_OFFSET           0x02a0
#define SAM_UDPHS_EP14_OFFSET           0x02c0
#define SAM_UDPHS_EP15_OFFSET           0x02e0

/* Endpoint registers */

#define SAM_UDPHS_EPTCFG_OFFSET         0x0000 /* UDPHS Endpoint Configuration Register */
#define SAM_UDPHS_EPTCTLENB_OFFSET      0x0004 /* UDPHS Endpoint Control Enable Register */
#define SAM_UDPHS_EPTCTLDIS_OFFSET      0x0008 /* UDPHS Endpoint Control Disable Register */
#define SAM_UDPHS_EPTCTL_OFFSET         0x000c /* UDPHS Endpoint Control Register */
                                               /* 0x0010 Reserved */
#define SAM_UDPHS_EPTSETSTA_OFFSET      0x0014 /* UDPHS Endpoint Set Status Register */
#define SAM_UDPHS_EPTCLRSTA_OFFSET      0x0018 /* UDPHS Endpoint Clear Status Register */
#define SAM_UDPHS_EPTSTA_OFFSET         0x001c /* UDPHS Endpoint Status Register */

/* DMA Channel Offsets */

#define SAM_UDPHS_CH_OFFSET(ch)         (0x0300+((unsigned int)(ch)<<4))
#define SAM_UDPHS_CH0_OFFSET            0x0300
#define SAM_UDPHS_CH1_OFFSET            0x0310
#define SAM_UDPHS_CH2_OFFSET            0x0320
#define SAM_UDPHS_CH3_OFFSET            0x0330
#define SAM_UDPHS_CH4_OFFSET            0x0340
#define SAM_UDPHS_CH5_OFFSET            0x0350
#define SAM_UDPHS_CH6_OFFSET            0x0360

/* DMA Channel Registers */

#define SAM_UDPHS_DMANXTDSC_OFFSET      0x0000 /* UDPHS DMA Next Descriptor Address Register */
#define SAM_UDPHS_DMAADDRESS_OFFSET     0x0004 /* UDPHS DMA Channel Address Register */
#define SAM_UDPHS_DMACONTROL_OFFSET     0x0008 /* UDPHS DMA Channel Control Register */
#define SAM_UDPHS_DMASTATUS_OFFSET      0x000c /* UDPHS DMA Channel Status Register */

/* Register addresses *******************************************************/

/* Global Registers */

#define SAM_UDPHS_CTRL                  (SAM_UDPHS_VBASE+SAM_UDPHS_CTRL_OFFSET)
#define SAM_UDPHS_FNUM                  (SAM_UDPHS_VBASE+SAM_UDPHS_FNUM_OFFSET)
#define SAM_UDPHS_IEN                   (SAM_UDPHS_VBASE+SAM_UDPHS_IEN_OFFSET)
#define SAM_UDPHS_INTSTA                (SAM_UDPHS_VBASE+SAM_UDPHS_INTSTA_OFFSET)
#define SAM_UDPHS_CLRINT                (SAM_UDPHS_VBASE+SAM_UDPHS_CLRINT_OFFSET)
#define SAM_UDPHS_EPTRST                (SAM_UDPHS_VBASE+SAM_UDPHS_EPTRST_OFFSET)
#define SAM_UDPHS_TST                   (SAM_UDPHS_VBASE+SAM_UDPHS_TST_OFFSET)

/* Endpoint Base Addresses */

#define SAM_UDPHS_EP_BASE(ep)           (SAM_UDPHS_VBASE+SAM_UDPHS_EP_OFFSET(ep))
#define SAM_UDPHS_EP0_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_EP0_OFFSET)
#define SAM_UDPHS_EP1_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_EP1_OFFSET)
#define SAM_UDPHS_EP2_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_EP2_OFFSET)
#define SAM_UDPHS_EP3_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_EP3_OFFSET)
#define SAM_UDPHS_EP4_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_EP4_OFFSET)
#define SAM_UDPHS_EP5_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_EP5_OFFSET)
#define SAM_UDPHS_EP6_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_EP6_OFFSET)
#define SAM_UDPHS_EP7_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_EP7_OFFSET)
#define SAM_UDPHS_EP8_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_EP8_OFFSET)
#define SAM_UDPHS_EP9_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_EP9_OFFSET)
#define SAM_UDPHS_EP10_BASE             (SAM_UDPHS_VBASE+SAM_UDPHS_EP10_OFFSET)
#define SAM_UDPHS_EP11_BASE             (SAM_UDPHS_VBASE+SAM_UDPHS_EP11_OFFSET)
#define SAM_UDPHS_EP12_BASE             (SAM_UDPHS_VBASE+SAM_UDPHS_EP12_OFFSET)
#define SAM_UDPHS_EP13_BASE             (SAM_UDPHS_VBASE+SAM_UDPHS_EP13_OFFSET)
#define SAM_UDPHS_EP14_BASE             (SAM_UDPHS_VBASE+SAM_UDPHS_EP14_OFFSET)
#define SAM_UDPHS_EP15_BASE             (SAM_UDPHS_VBASE+SAM_UDPHS_EP15_OFFSET)

/* Endpoint registers */

#define SAM_UDPHS_EPTCFG(ep)            (SAM_UDPHS_EP_BASE(ep)+SAM_UDPHS_EPTCFG_OFFSET)
#define SAM_UDPHS_EPTCTLENB(ep)         (SAM_UDPHS_EP_BASE(ep)+SAM_UDPHS_EPTCTLENB_OFFSET)
#define SAM_UDPHS_EPTCTLDIS(ep)         (SAM_UDPHS_EP_BASE(ep)+SAM_UDPHS_EPTCTLDIS_OFFSET)
#define SAM_UDPHS_EPTCTL(ep)            (SAM_UDPHS_EP_BASE(ep)+SAM_UDPHS_EPTCTL_OFFSET)
#define SAM_UDPHS_EPTSETSTA(ep)         (SAM_UDPHS_EP_BASE(ep)+SAM_UDPHS_EPTSETSTA_OFFSET)
#define SAM_UDPHS_EPTCLRSTA(ep)         (SAM_UDPHS_EP_BASE(ep)+SAM_UDPHS_EPTCLRSTA_OFFSET)
#define SAM_UDPHS_EPTSTA(ep)            (SAM_UDPHS_EP_BASE(ep)+SAM_UDPHS_EPTSTA_OFFSET)

/* DMA Channel Base Addresses */

#define SAM_UDPHS_CH_BASE(ch)           (SAM_UDPHS_VBASE+SAM_UDPHS_CH_OFFSET(ch))
#define SAM_UDPHS_CH1_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_CH1_OFFSET)
#define SAM_UDPHS_CH2_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_CH2_OFFSET)
#define SAM_UDPHS_CH3_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_CH3_OFFSET)
#define SAM_UDPHS_CH4_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_CH4_OFFSET)
#define SAM_UDPHS_CH5_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_CH5_OFFSET)
#define SAM_UDPHS_CH6_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_CH6_OFFSET)
#define SAM_UDPHS_CH7_BASE              (SAM_UDPHS_VBASE+SAM_UDPHS_CH7_OFFSET)

/* DMA Channel Registers */

#define SAM_UDPHS_DMANXTDSC(ch)         (SAM_UDPHS_CH_BASE(ch)+SAM_UDPHS_DMANXTDSC_OFFSET)
#define SAM_UDPHS_DMAADDRESS(ch)        (SAM_UDPHS_CH_BASE(ch)+SAM_UDPHS_DMAADDRESS_OFFSET)
#define SAM_UDPHS_DMACONTROL(ch)        (SAM_UDPHS_CH_BASE(ch)+SAM_UDPHS_DMACONTROL_OFFSET)
#define SAM_UDPHS_DMASTATUS(ch)         (SAM_UDPHS_CH_BASE(ch)+SAM_UDPHS_DMASTATUS_OFFSET)

/* Register bit-field definitions *******************************************/

/* Global Registers */

/* UDPHS Control Register */

#define UDPHS_CTRL_DEVADDR_SHIFT        (0)       /* Bits 0-6: UDPHS Address */
#define UDPHS_CTRL_DEVADDR_MASK         (0x7f << UDPHS_CTRL_DEVADDR_SHIFT)
#  define UDPHS_CTRL_DEVADDR(addr)      ((addr) << UDPHS_CTRL_DEVADDR_SHIFT)
#define UDPHS_CTRL_FADDREN              (1 << 7)  /* Bit 7:  Function Address Enable */
#define UDPHS_CTRL_ENUDPHS              (1 << 8)  /* Bit 8:  UDPHS Enable */
#define UDPHS_CTRL_DETACH               (1 << 9)  /* Bit 9:  Detach Command */
#define UDPHS_CTRL_REWAKEUP             (1 << 10) /* Bit 10: Send Remote Wake Up */
#define UDPHS_CTRL_PULLDDIS             (1 << 11) /* Bit 11: Pull-Down Disable */

/* UDPHS Frame Number Register */

#define UDPHS_FNUM_UFRAMENUM_SHIFT      (0)       /* Bits 0-2: Microframe Number */
#define UDPHS_FNUM_UFRAMENUM_MASK       (7 << UDPHS_FNUM_UFRAMENUM_SHIFT)
#define UDPHS_FNUM_FRAMENUM_SHIFT       (3)       /* Bits 3-13: Frame Number */
#define UDPHS_FNUM_FRAMENUM_MASK        (0x7ff << UDPHS_FNUM_FRAMENUM_SHIFT)
#define UDPHS_FNUM_FNUMERR              (1 << 31) /* Bit 31: Frame Number CRC Error */

/* Common interrupt bits */

/* UDPHS Interrupt Status Register (only) */

#define UDPHS_INTSTA_SPEED              (1 << 0)  /* Bit 0:  Speed Status */

/* UDPHS Interrupt Enable Register, UDPHS Interrupt Status Register,
 * and UDPHS Clear Interrupt Register
 */

#define UDPHS_INT_DETSUSPD              (1 << 1)  /* Bit 1:  Suspend Interrupt */
#define UDPHS_INT_MICROSOF              (1 << 2)  /* Bit 2:  Micro Start Of Frame Interrupt */
#define UDPHS_INT_INTSOF                (1 << 3)  /* Bit 3:  Start Of Frame Interrupt */
#define UDPHS_INT_ENDRESET              (1 << 4)  /* Bit 4:  End Of Reset Interrupt */
#define UDPHS_INT_WAKEUP                (1 << 5)  /* Bit 5:  Wake Up CPU Interrupt */
#define UDPHS_INT_ENDOFRSM              (1 << 6)  /* Bit 6:  End Of Resume Interrupt */
#define UDPHS_INT_UPSTRRES              (1 << 7)  /* Bit 7:  Upstream Resume Interrupt */

/* UDPHS Interrupt Enable Register and UDPHS Interrupt Status Register */

#define UDPHS_INT_EPT_SHIFT             (8)       /* Bits 8-23: Endpoint interrupts */
#define UDPHS_INT_EPT_MASK              (0xffff << UDPHS_INT_EPT_SHIFT)
#define UDPHS_INT_EPT(ep)               (1 << +((ep)+8))  /* Endpoint ep Interrupt */

#  define UDPHS_INT_EPT0                (1 << 8)  /* Bit 8: Endpoint 0 Interrupt */
#  define UDPHS_INT_EPT1                (1 << 9)  /* Bit 9: Endpoint 1 Interrupt */
#  define UDPHS_INT_EPT2                (1 << 10) /* Bit 10: Endpoint 2 Interrupt */
#  define UDPHS_INT_EPT3                (1 << 11) /* Bit 11: Endpoint 3 Interrupt */
#  define UDPHS_INT_EPT4                (1 << 12) /* Bit 12: Endpoint 4 Interrupt */
#  define UDPHS_INT_EPT5                (1 << 13) /* Bit 13: Endpoint 5 Interrupt */
#  define UDPHS_INT_EPT6                (1 << 14) /* Bit 14: Endpoint 6 Interrupt */
#  define UDPHS_INT_EPT7                (1 << 15) /* Bit 15: Endpoint 7 Interrupt */
#  define UDPHS_INT_EPT8                (1 << 16) /* Bit 16: Endpoint 8 Interrupt */
#  define UDPHS_INT_EPT9                (1 << 17) /* Bit 17: Endpoint 9 Interrupt */
#  define UDPHS_INT_EPT10               (1 << 18) /* Bit 18: Endpoint 10 Interrupt */
#  define UDPHS_INT_EPT11               (1 << 19) /* Bit 19: Endpoint 11 Interrupt */
#  define UDPHS_INT_EPT12               (1 << 20) /* Bit 20: Endpoint 12 Interrupt */
#  define UDPHS_INT_EPT13               (1 << 21) /* Bit 21: Endpoint 13 Interrupt */
#  define UDPHS_INT_EPT14               (1 << 22) /* Bit 22: Endpoint 14 Interrupt */
#  define UDPHS_INT_EPT15               (1 << 23) /* Bit 23: Endpoint 15 Interrupt */
#define UDPHS_INT_DMA_SHIFT             (25)      /* Bits 25-31: Endpoint interrupts */
#define UDPHS_INT_DMA_MASK              (0x7f << UDPHS_INT_DMA_SHIFT)
#define UDPHS_INT_DMA(ch)               (1 << ((ch)+24)) /* DMA Channel ch Interrupt */

#  define UDPHS_INT_DMA1                (1 << 25) /* Bit 25: DMA Channel 1 Interrupt */
#  define UDPHS_INT_DMA2                (1 << 26) /* Bit 26: DMA Channel 2 Interrupt */
#  define UDPHS_INT_DMA3                (1 << 27) /* Bit 27: DMA Channel 3 Interrupt */
#  define UDPHS_INT_DMA4                (1 << 28) /* Bit 28: DMA Channel 4 Interrupt */
#  define UDPHS_INT_DMA5                (1 << 29) /* Bit 29: DMA Channel 5 Interrupt */
#  define UDPHS_INT_DMA6                (1 << 30) /* Bit 30: DMA Channel 6 Interrupt */
#  define UDPHS_INT_DMA7                (1 << 31) /* Bit 31: DMA Channel 7 Interrupt */

/* UDPHS Endpoints Reset Register */

#define UDPHS_EPTRST(ep)                (1 << (ep))  /* Endpoint ep Reset */

#  define UDPHS_EPT0RST                 (1 << 0)  /* Bit 0:  Endpoint 0 Reset */
#  define UDPHS_EPT1RST                 (1 << 1)  /* Bit 1:  Endpoint 1 Reset */
#  define UDPHS_EPT2RST                 (1 << 2)  /* Bit 2:  Endpoint 2 Reset */
#  define UDPHS_EPT3RST                 (1 << 3)  /* Bit 3:  Endpoint 3 Reset */
#  define UDPHS_EPT4RST                 (1 << 4)  /* Bit 4:  Endpoint 4 Reset */
#  define UDPHS_EPT5RST                 (1 << 5)  /* Bit 5:  Endpoint 5 Reset */
#  define UDPHS_EPT6RST                 (1 << 6)  /* Bit 6:  Endpoint 6 Reset */
#  define UDPHS_EPT7RST                 (1 << 7)  /* Bit 7:  Endpoint 7 Reset */
#  define UDPHS_EPT8RST                 (1 << 8)  /* Bit 8:  Endpoint 8 Reset */
#  define UDPHS_EPT9RST                 (1 << 9)  /* Bit 9:  Endpoint 9 Reset */
#  define UDPHS_EPT10RST                (1 << 10) /* Bit 10: Endpoint 10 Reset */
#  define UDPHS_EPT11RST                (1 << 11) /* Bit 11: Endpoint 11 Reset */
#  define UDPHS_EPT12RST                (1 << 12) /* Bit 12: Endpoint 12 Reset */
#  define UDPHS_EPT13RST                (1 << 13) /* Bit 13: Endpoint 13 Reset */
#  define UDPHS_EPT14RST                (1 << 14) /* Bit 14: Endpoint 14 Reset */
#  define UDPHS_EPT15RST                (1 << 15) /* Bit 15: Endpoint 15 Reset */

/* UDPHS Test Register */

#define UDPHS_TST_SPEED_SHIFT           (0)       /* Bits 0-1: Speed Configuration */
#define UDPHS_TST_SPEED_MASK            (3 << UDPHS_TST_SPEED_SHIFT)
#  define UDPHS_TST_SPEED_NORMAL        (0 << UDPHS_TST_SPEED_SHIFT) /* Normal Mode */
#  define UDPHS_TST_SPEED_HIGH          (2 << UDPHS_TST_SPEED_SHIFT) /* Force High Speed */
#  define UDPHS_TST_SPEED_FULL          (3 << UDPHS_TST_SPEED_SHIFT) /* Force Full Speed */

#define UDPHS_TST_TSTJ                  (1 << 2)  /* Bit 2:  Test J Mode */
#define UDPHS_TST_TSTK                  (1 << 3)  /* Bit 3:  Test K Mode */
#define UDPHS_TST_TSTPKT                (1 << 4)  /* Bit 4:  Test Packet Mode */
#define UDPHS_TST_OPMODE2               (1 << 5)  /* Bit 4: OpMode2 */

/* Endpoint registers */

/* UDPHS Endpoint Configuration Register */

#define UDPHS_EPTCFG_SIZE_SHIFT         (0)        /* Bits 0-2: Endpoint Size */
#define UDPHS_EPTCFG_SIZE_MASK          (7 << UDPHS_EPTCFG_SIZE_SHIFT)
#  define UDPHS_EPTCFG_SIZE_8           (0 << UDPHS_EPTCFG_SIZE_SHIFT) /* 8 bytes */
#  define UDPHS_EPTCFG_SIZE_16          (1 << UDPHS_EPTCFG_SIZE_SHIFT) /* 16 bytes */
#  define UDPHS_EPTCFG_SIZE_32          (2 << UDPHS_EPTCFG_SIZE_SHIFT) /* 32 bytes */
#  define UDPHS_EPTCFG_SIZE_64          (3 << UDPHS_EPTCFG_SIZE_SHIFT) /* 64 bytes  */
#  define UDPHS_EPTCFG_SIZE_128         (4 << UDPHS_EPTCFG_SIZE_SHIFT) /* 128 bytes */
#  define UDPHS_EPTCFG_SIZE_256         (5 << UDPHS_EPTCFG_SIZE_SHIFT) /* 256 bytes */
#  define UDPHS_EPTCFG_SIZE_512         (6 << UDPHS_EPTCFG_SIZE_SHIFT) /* 512 bytes */
#  define UDPHS_EPTCFG_SIZE_1024        (7 << UDPHS_EPTCFG_SIZE_SHIFT) /* 1024 bytes */

#define UDPHS_EPTCFG_DIR                (1 << 3)  /* Bit 3:  Endpoint Direction */
#define UDPHS_EPTCFG_TYPE_SHIFT         (4)       /* Bits 4-5: Endpoint Type */
#define UDPHS_EPTCFG_TYPE_MASK          (3 << UDPHS_EPTCFG_TYPE_SHIFT)
#  define UDPHS_EPTCFG_TYPE_CTRL8       (0 << UDPHS_EPTCFG_TYPE_SHIFT) /* Control endpoint */
#  define UDPHS_EPTCFG_TYPE_ISO         (1 << UDPHS_EPTCFG_TYPE_SHIFT) /* Isochronous endpoint */
#  define UDPHS_EPTCFG_TYPE_BULK        (2 << UDPHS_EPTCFG_TYPE_SHIFT) /* Bulk endpoint */
#  define UDPHS_EPTCFG_TYPE_INT         (3 << UDPHS_EPTCFG_TYPE_SHIFT) /* Interrupt endpoint */

#define UDPHS_EPTCFG_BKNUMBER_SHIFT     (6)       /* Bits 6-7: Number of Banks */
#define UDPHS_EPTCFG_BKNUMBER_MASK      (3 << UDPHS_EPTCFG_BKNUMBER_SHIFT)
#define UDPHS_EPTCFG_NBTRANS_SHIFT      (8)       /* Bits 8-9: Number Transaction per uframe */
#define UDPHS_EPTCFG_NBTRANS_MASK       (3 << UDPHS_EPTCFG_NBTRANS_SHIFT)
#define UDPHS_EPTCFG_MAPD               (1 << 31) /* Bit 31: Endpoint Mapped */

/* UDPHS Endpoint Control Enable Register,
 * UDPHS Endpoint Control Disable Register, and UDPHS
 * Endpoint Control Register
 */

                                  /* Common bits definitions */

#define UDPHS_EPTCTL_EPTENABL           (1 << 0)  /* Bit 0:  Endpoint Enable */
#define UDPHS_EPTCTL_AUTOVALID          (1 << 1)  /* Bit 1:  Packet Auto-Valid Enable */
#define UDPHS_EPTCTL_INTDISDMA          (1 << 3)  /* Bit 3:  Interrupts Disable DMA */

                                  /* Control/Bulk/Interrupt */

#define UDPHS_EPTCTL_NYETDIS            (1 << 4)  /* Bit 4:  NYET Disable (High Speed Bulk OUT) */

                                  /* Isochronous Endpoints Only */

#define UDPHS_EPTCTL_DATAXRX            (1 << 8)  /* Bit 8:  Interrupt Enable (Isochronous OUT) */
#define UDPHS_EPTCTL_MDATARX            (1 << 9)  /* Bit 9:  MDATA Interrupt Enable (Isochronous OUT) */

                                  /* Common Bit Definitions */

#define UDPHS_EPTCTL_ERROVFLW           (1 << 8)  /* Bit 8:  Overflow Error Interrupt Enable */
#define UDPHS_EPTCTL_RXRDYTXKL          (1 << 9)  /* Bit 9:  Received OUT Data Interrupt Enable */
#define UDPHS_EPTCTL_TXCOMPLT           (1 << 10) /* Bit 10: Transmitted IN Data Complete Interrupt Enable */

                                  /* Control/Bulk/Interrupt */

#define UDPHS_EPTCTL_TXRDY              (1 << 11) /* Bit 11: TX Packet Ready Interrupt Enable */
#define UDPHS_EPTCTL_RXSETUP            (1 << 12) /* Bit 12: Received SETUP */
#define UDPHS_EPTCTL_STALLSNT           (1 << 13) /* Bit 13: Stall Sent Interrupt Enable */
#define UDPHS_EPTCTL_NAKIN              (1 << 14) /* Bit 14: NAKIN Interrupt Enable */
#define UDPHS_EPTCTL_NAKOUT             (1 << 15) /* Bit 15: NAKOUT Interrupt Enable */

                                  /* Isochronous Endpoints Only */

#define UDPHS_EPTCTL_TXRDYTRER          (1 << 11) /* Bit 11: TX Packet Ready/Transaction Error Interrupt Enable */
#define UDPHS_EPTCTL_ERRFLISO           (1 << 12) /* Bit 12: Error Flow Interrupt Enable */
#define UDPHS_EPTCTL_ERRCRCNTR          (1 << 13) /* Bit 13: ISO CRC Error/Number of Transaction Error Interrupt Enable */
#define UDPHS_EPTCTL_ERRFLUSH           (1 << 14) /* Bit 14: Bank Flush Error Interrupt Enable */

                                  /* Common Bit Definitions */

#define UDPHS_EPTCTL_BUSYBANK           (1 << 18) /* Bit 28: Busy Bank Interrupt Enable */
#define UDPHS_EPTCTL_SHRTPCKT           (1 << 31) /* Bit 31: Short Packet Send/Short Packet Interrupt Enable */

/* UDPHS Endpoint Set Status Register */

                                                  /* Control/Bulk/Interrupt */
#define UDPHS_EPTSETSTA_FRCESTALL       (1 << 5)  /* Bit 5:  Stall Handshake Request Set */
                                                  /* Common Bit Definitions */
#define UDPHS_EPTSETSTA_RXRDYTXKL       (1 << 9)  /* Bit 9:  KILL Bank Set (IN Endpoint) */
                                                  /* Control/Bulk/Interrupt */
#define UDPHS_EPTSETSTA_TXRDY           (1 << 11) /* Bit 11: TX Packet Ready Set */
                                                  /* Isochronous Endpoints Only */
#define UDPHS_EPTSETSTA_TXRDYTRER       (1 << 11) /* Bit 11: TX Packet Ready Set */

/* UDPHS Endpoint Clear Status Register */

                                                  /* Control/Bulk/Interrupt */
#define UDPHS_EPTCLRSTA_FRCESTALL       (1 << 5)  /* Bit 5:  Stall Handshake Request Clear */
                                                  /* Common Bit Definitions */
#define UDPHS_EPTCLRSTA_TOGGLESQ        (1 << 6)  /* Bit 6:  Data Toggle Clear */
#define UDPHS_EPTCLRSTA_RXRDYTXKL       (1 << 9)  /* Bit 9:  Received OUT Data Clear */
#define UDPHS_EPTCLRSTA_TXCOMPLT        (1 << 10) /* Bit 10: Transmitted IN Data Complete Clear */
                                                  /* Control/Bulk/Interrupt */
#define UDPHS_EPTCLRSTA_RXSETUP         (1 << 12) /* Bit 12: Received SETUP Clear */
#define UDPHS_EPTCLRSTA_STALLSNT        (1 << 13) /* Bit 13: Stall Sent Clear */
#define UDPHS_EPTCLRSTA_NAKIN           (1 << 14) /* Bit 14: NAKIN Clear */
#define UDPHS_EPTCLRSTA_NAKOUT          (1 << 15) /* Bit 15: NAKOUT Clear */
                                                  /* Isochronous Endpoints Only */
#define UDPHS_EPTCLRSTA_ERRFLISO        (1 << 12) /* Bit 12: Error Flow Clear */
#define UDPHS_EPTCLRSTA_ERRCRCNTR       (1 << 13) /* Bit 13: Number of Transaction Error Clear */
#define UDPHS_EPTCLRSTA_ERRFLUSH        (1 << 14) /* Bit 14: Bank Flush Error Clear */

/* UDPHS Endpoint Status Register */

                                                  /* Control/Bulk/Interrupt */
#define UDPHS_EPTSTA_FRCESTALL          (1 << 5)  /* Bit 5:  Stall Handshake Request */
                                                  /* Common Bit Definitions */
#define UDPHS_EPTSTA_TOGGLESQ_SHIFT     (6)       /* Bits 6-7: Toggle Sequencing */
#define UDPHS_EPTSTA_TOGGLESQ_MASK      (3 << UDPHS_EPTSTA_TOGGLESQ_SHIFT)
#  define UDPHS_EPTSTA_TOGGLESQ_DATA0   (0 << UDPHS_EPTSTA_TOGGLESQ_SHIFT) /* DATA0 */
#  define UDPHS_EPTSTA_TOGGLESQ_DATA1   (1 << UDPHS_EPTSTA_TOGGLESQ_SHIFT) /* DATA1 */
#  define UDPHS_EPTSTA_TOGGLESQ_DATA2   (2 << UDPHS_EPTSTA_TOGGLESQ_SHIFT) /* Isochronous Endpoint */
#  define UDPHS_EPTSTA_TOGGLESQ_MDATA   (3 << UDPHS_EPTSTA_TOGGLESQ_SHIFT) /* Isochronous Endpoint */

#define UDPHS_EPTSTA_ERROVFLW           (1 << 8)  /* Bit 8:  Overflow Error */
#define UDPHS_EPTSTA_RXRDYTXKL          (1 << 9)  /* Bit 9:  Received OUT Data/KILL Bank */
#define UDPHS_EPTSTA_TXCOMPLT           (1 << 10) /* Bit 10: Transmitted IN Data Complete */
                                                  /* Control/Bulk/Interrupt */
#define UDPHS_EPTSTA_TXRDY              (1 << 11) /* Bit 11: TX Packet Ready */
#define UDPHS_EPTSTA_RXSETUP            (1 << 12) /* Bit 12: Received SETUP */
#define UDPHS_EPTSTA_STALLSNT           (1 << 13) /* Bit 13: Stall Sent */
#define UDPHS_EPTSTA_NAKIN              (1 << 14) /* Bit 14: NAK IN */
#define UDPHS_EPTSTA_NAKOUT             (1 << 15) /* Bit 15: NAK OUT */
                                                  /* Isochronous Endpoints Only */
#define UDPHS_EPTSTA_TXRDYTRER          (1 << 11) /* Bit 11: TX Packet Ready/Transaction Error */
#define UDPHS_EPTSTA_ERRFLISO           (1 << 12) /* Bit 12: Error Flow */
#define UDPHS_EPTSTA_ERRCRCNTR          (1 << 13) /* Bit 13: CRC ISO Error/Number of Transaction Error */
#define UDPHS_EPTSTA_ERRFLUSH           (1 << 14) /* Bit 14: Bank Flush Error */
                                                  /* Control Only */
#define UDPHS_EPTSTA_CTLDIR_SHIFT       (16)      /* Bits 16-17: Control Direction */
#define UDPHS_EPTSTA_CTLDIR_MASK        (3 << UDPHS_EPTSTA_CTLDIR_SHIFT)
#  define UDPHS_EPTSTA_CTLDIR_WRITE     (0 << UDPHS_EPTSTA_CTLDIR_SHIFT) /* Control Write requested */
#  define UDPHS_EPTSTA_CTLDIR_READ      (1 << UDPHS_EPTSTA_CTLDIR_SHIFT) /* Control Read requested */

                                          /* Bulk/Interrupt/Isochronous */

#define UDPHS_EPTSTA_CURBK_SHIFT        (16)      /* Bits 16-17: Current Bank */
#define UDPHS_EPTSTA_CURBK_MASK         (3 << UDPHS_EPTSTA_CURBK_SHIFT)
#  define UDPHS_EPTSTA_CURBK_BANK0      (0 << UDPHS_EPTSTA_CURBK_SHIFT) /* Bank 0 (or single bank) */
#  define UDPHS_EPTSTA_CURBK_BANK1      (1 << UDPHS_EPTSTA_CURBK_SHIFT) /* Bank 1 */
#  define UDPHS_EPTSTA_CURBK_BANK2      (2 << UDPHS_EPTSTA_CURBK_SHIFT) /* Bank 2 */

                                                  /* Common Bit Definitions */
#define UDPHS_EPTSTA_BUSYBANK_SHIFT     (18)      /* Bits 18-19: Busy Bank Number */
#define UDPHS_EPTSTA_BUSYBANK_MASK      (3 << UDPHS_EPTSTA_BUSYBANK_SHIFT)
#  define UDPHS_EPTSTA_BUSYBANK_1       (0 << UDPHS_EPTSTA_BUSYBANK_SHIFT) /* 1 busy bank */
#  define UDPHS_EPTSTA_BUSYBANK_2       (1 << UDPHS_EPTSTA_BUSYBANK_SHIFT) /* 2 busy banks */
#  define UDPHS_EPTSTA_BUSYBANK_3       (2 << UDPHS_EPTSTA_BUSYBANK_SHIFT) /* 3 busy banks */

#define UDPHS_EPTSTA_BYTECNT_SHIFT      (20)       /* Bits 20-30: UDPHS Byte Count */
#define UDPHS_EPTSTA_BYTECNT_MASK       (0x7ff << UDPHS_EPTSTA_BYTECNT_SHIFT)
#define UDPHS_EPTSTA_SHRTPCKT           (1 << 31)  /* Bit 31: Short Packet */

/* DMA Channel Registers */

/* UDPHS DMA Next Descriptor Address Register (32-bit address) */

/* UDPHS DMA Channel Address Register (32-bit address) */

/* UDPHS DMA Channel Control Register */

#define UDPHS_DMACONTROL_CHANNENB       (1 << 0)  /* Bit 0:  Channel Enable Command */
#define UDPHS_DMACONTROL_LDNXTDSC       (1 << 1)  /* Bit 1:  Load Next Channel Transfer Descriptor Enable (Command) */
#define UDPHS_DMACONTROL_ENDTREN        (1 << 2)  /* Bit 2:  End of Transfer Enable (Control) */
#define UDPHS_DMACONTROL_ENDBEN         (1 << 3)  /* Bit 3:  End of Buffer Enable (Control) */
#define UDPHS_DMACONTROL_ENDTRIT        (1 << 4)  /* Bit 4:  End of Transfer Interrupt Enable */
#define UDPHS_DMACONTROL_ENDBUFFIT      (1 << 5)  /* Bit 5:  End of Buffer Interrupt Enable */
#define UDPHS_DMACONTROL_DESCLDIT       (1 << 6)  /* Bit 6:  Descriptor Loaded Interrupt Enable */
#define UDPHS_DMACONTROL_BURSTLCK       (1 << 7)  /* Bit 7:  Burst Lock Enable */
#define UDPHS_DMACONTROL_BUFLEN_SHIFT   (16)      /* Bits 16-31: Buffer Byte Length (Write-only) */
#define UDPHS_DMACONTROL_BUFLEN_MASK    (0xffff << UDPHS_DMACONTROL_BUFLEN_SHIFT)
#  define UDPHS_DMACONTROL_BUFLEN(n)    ((uint32_t)(n) << UDPHS_DMACONTROL_BUFLEN_SHIFT)

/* UDPHS DMA Channel Status Register */

#define UDPHS_DMASTATUS_CHANNENB        (1 << 0)  /* Bit 0: Channel Enable Status */
#define UDPHS_DMASTATUS_CHANNACT        (1 << 1)  /* Bit 1: Channel Active Status */
#define UDPHS_DMASTATUS_ENDTRST         (1 << 4)  /* Bit 4: End of Channel Transfer Status */
#define UDPHS_DMASTATUS_ENDBFST         (1 << 5)  /* Bit 5: End of Channel Buffer Status */
#define UDPHS_DMASTATUS_DESCLDST        (1 << 6)  /* Bit 6: Descriptor Loaded Status */
#define UDPHS_DMASTATUS_BUFCNT_SHIFT    (16)      /* Bits 16-31: Buffer Byte Counut */
#define UDPHS_DMASTATUS_BUFCNT_MASK     (0xffff << UDPHS_DMASTATUS_BUFCNT_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure defines the UDPHS DMA Transfer Descriptor.
 *  Instances of DMA transfer descriptors must by aligned to 16-byte
 *  address boundaries.
 *
 * Each value contains the next value of each of three UDPHS DMA registers.
 * The first register value (UDPHS_DMANXTDSCx) is a link that can be used
 * to chain sequences of DMA transfers.
 */

struct udphs_dtd_s
{
  uint32_t nxtd;  /* Next Descriptor Address Register: UDPHS_DMANXTDSCx */
  uint32_t addr;  /* DMA Channelx Address Register: UDPHS_DMAADDRESSx */
  uint32_t ctrl;  /* DMA Channelx Control Register: UDPHS_DMACONTROLx */
};
#define SIZEOF_USPHS_DTD_S 12

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_UDPHS_H */
