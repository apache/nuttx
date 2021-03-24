/****************************************************************************
 * arch/arm/src/tiva/hardware/lm/lm3s_ethernet.h
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_LM_LM3S_ETHERNET_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_LM_LM3S_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/mii.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Ethernet Controller Register Offsets *************************************/

/* Ethernet MAC Register Offsets */

#define TIVA_MAC_RIS_OFFSET   0x000 /* Ethernet MAC Raw Interrupt Status */
#define TIVA_MAC_IACK_OFFSET  0x000 /* Ethernet MAC Acknowledge */
#define TIVA_MAC_IM_OFFSET    0x004 /* Ethernet MAC Interrupt Mask */
#define TIVA_MAC_RCTL_OFFSET  0x008 /* Ethernet MAC Receive Control */
#define TIVA_MAC_TCTL_OFFSET  0x00c /* Ethernet MAC Transmit Control */
#define TIVA_MAC_DATA_OFFSET  0x010 /* Ethernet MAC Data */
#define TIVA_MAC_IA0_OFFSET   0x014 /* Ethernet MAC Individual Address 0 */
#define TIVA_MAC_IA1_OFFSET   0x018 /* Ethernet MAC Individual Address 1 */
#define TIVA_MAC_THR_OFFSET   0x01c /* Ethernet MAC Threshold */
#define TIVA_MAC_MCTL_OFFSET  0x020 /* Ethernet MAC Management Control */
#define TIVA_MAC_MDV_OFFSET   0x024 /* Ethernet MAC Management Divider */
#define TIVA_MAC_MTXD_OFFSET  0x02c /* Ethernet MAC Management Transmit Data */
#define TIVA_MAC_MRXD_OFFSET  0x030 /* Ethernet MAC Management Receive Data */
#define TIVA_MAC_NP_OFFSET    0x034 /* Ethernet MAC Number of Packets */
#define TIVA_MAC_TR_OFFSET    0x038 /* Ethernet MAC Transmission Request */
#ifdef TIVA_ETHTS
#  define TIVA_MAC_TS_OFFSET  0x03c /* Ethernet MAC Time Stamp Configuration */
#endif

/* MII Management Register Offsets (see include/nuttx/net/mii.h) */

/* Ethernet Controller Register Addresses ***********************************/

#define TIVA_MAC_RIS          (TIVA_ETHCON_BASE + TIVA_MAC_RIS_OFFSET)
#define TIVA_MAC_IACK         (TIVA_ETHCON_BASE + TIVA_MAC_IACK_OFFSET)
#define TIVA_MAC_IM           (TIVA_ETHCON_BASE + TIVA_MAC_IM_OFFSET)
#define TIVA_MAC_RCTL         (TIVA_ETHCON_BASE + TIVA_MAC_RCTL_OFFSET)
#define TIVA_MAC_TCTL         (TIVA_ETHCON_BASE + TIVA_MAC_TCTL_OFFSET)
#define TIVA_MAC_DATA         (TIVA_ETHCON_BASE + TIVA_MAC_DATA_OFFSET)
#define TIVA_MAC_IA0          (TIVA_ETHCON_BASE + TIVA_MAC_IA0_OFFSET)
#define TIVA_MAC_IA1          (TIVA_ETHCON_BASE + TIVA_MAC_IA1_OFFSET)
#define TIVA_MAC_THR          (TIVA_ETHCON_BASE + TIVA_MAC_THR_OFFSET)
#define TIVA_MAC_MCTL         (TIVA_ETHCON_BASE + TIVA_MAC_MCTL_OFFSET)
#define TIVA_MAC_MDV          (TIVA_ETHCON_BASE + TIVA_MAC_MDV_OFFSET)
#define TIVA_MAC_MTXD         (TIVA_ETHCON_BASE + TIVA_MAC_MTXD_OFFSET)
#define TIVA_MAC_MRXD         (TIVA_ETHCON_BASE + TIVA_MAC_MRXD_OFFSET)
#define TIVA_MAC_NP           (TIVA_ETHCON_BASE + TIVA_MAC_NP_OFFSET)
#define TIVA_MAC_TR           (TIVA_ETHCON_BASE + TIVA_MAC_TR_OFFSET)
#ifdef TIVA_ETHTS
#  define TIVA_MAC_TS         (TIVA_ETHCON_BASE + TIVA_MAC_TS_OFFSET)
#endif

/* Memory Mapped MII Management Registers */

#define MAC_MII_MCR           (TIVA_ETHCON_BASE + MII_MCR)
#define MAC_MII_MSR           (TIVA_ETHCON_BASE + MII_MSR)
#define MAC_MII_PHYID1        (TIVA_ETHCON_BASE + MII_PHYID1)
#define MAC_MII_PHYID2        (TIVA_ETHCON_BASE + MII_PHYID2)
#define MAC_MII_ADVERTISE     (TIVA_ETHCON_BASE + MII_ADVERTISE)
#define MAC_MII_LPA           (TIVA_ETHCON_BASE + MII_LPA)
#define MAC_MII_EXPANSION     (TIVA_ETHCON_BASE + MII_EXPANSION)
#define MAC_MII_VSPECIFIC     (TIVA_ETHCON_BASE + MII_TIVA_VSPECIFIC)
#define MAC_MII_INTCS         (TIVA_ETHCON_BASE + MII_TIVA_INTCS)
#define MAC_MII_DIAGNOSTIC    (TIVA_ETHCON_BASE + MII_TIVA_DIAGNOSTIC)
#define MAC_MII_XCVRCONTROL   (TIVA_ETHCON_BASE + MII_TIVA_XCVRCONTROL)
#define MAC_MII_LEDCONFIG     (TIVA_ETHCON_BASE + MII_TIVA_LEDCONFIG)
#define MAC_MII_MDICONTROL    (TIVA_ETHCON_BASE + MII_TIVA_MDICONTROL)

/* Ethernet Controller Register Bit Definitions *****************************/

/* Ethernet MAC Raw Interrupt Status/Acknowledge
 * (MACRIS/MACIACK), offset 0x000
 */

#define MAC_RIS_RXINT         (1 << 0)  /* Bit 0:  Packet Received */
#define MAC_RIS_TXER          (1 << 1)  /* Bit 1:  Transmit Error */
#define MAC_RIS_TXEMP         (1 << 2)  /* Bit 2:  Transmit FIFO Empty */
#define MAC_RIS_FOV           (1 << 3)  /* Bit 3:  FIFO Overrun */
#define MAC_RIS_RXER          (1 << 4)  /* Bit 4:  Receive Error */
#define MAC_RIS_MDINT         (1 << 5)  /* Bit 5:  MII Transaction Complete */
#define MAC_RIS_PHYINT        (1 << 6)  /* Bit 6:  PHY Interrupt */

#define MAC_IACK_RXINT        (1 << 0)  /* Bit 0:  Clear Packet Received */
#define MAC_IACK_TXER         (1 << 1)  /* Bit 1:  Clear Transmit Error */
#define MAC_IACK_TXEMP        (1 << 2)  /* Bit 2:  Clear Transmit FIFO Empty */
#define MAC_IACK_FOV          (1 << 3)  /* Bit 3:  Clear FIFO Overrun */
#define MAC_IACK_RXER         (1 << 4)  /* Bit 4:  Clear Receive Error */
#define MAC_IACK_MDINT        (1 << 5)  /* Bit 5:  Clear MII Transaction Complete */
#define MAC_IACK_PHYINT       (1 << 6)  /* Bit 6:  Clear PHY Interrupt */

/* Ethernet MAC Interrupt Mask (MACIM), offset 0x004 */

#define MAC_IM_RXINTM         (1 << 0)  /* Bit 0:  Mask Packet Received */
#define MAC_IM_TXERM          (1 << 1)  /* Bit 1:  Mask Transmit Error */
#define MAC_IM_TXEMPM         (1 << 2)  /* Bit 2:  Mask Transmit FIFO Empty */
#define MAC_IM_FOVM           (1 << 3)  /* Bit 3:  Mask FIFO Overrun */
#define MAC_IM_RXERM          (1 << 4)  /* Bit 4:  Mask Receive Error */
#define MAC_IM_MDINTM         (1 << 5)  /* Bit 5:  Mask MII Transaction Complete */
#define MAC_IM_PHYINTM        (1 << 6)  /* Bit 6:  Mask PHY Interrupt */
#define MAC_IM_ALLINTS        0x7f

/* Ethernet MAC Receive Control (MACRCTL), offset 0x008 */

#define MAC_RCTL_RXEN         (1 << 0)  /* Bit 0:  Enable Receiver */
#define MAC_RCTL_AMUL         (1 << 1)  /* Bit 1:  Enable Multicast Frames */
#define MAC_RCTL_PRMS         (1 << 2)  /* Bit 2:  Enable Promiscuous Mode */
#define MAC_RCTL_BADCRC       (1 << 3)  /* Bit 3:  Enable Reject Bad CRC */
#define MAC_RCTL_RSTFIFO      (1 << 4)  /* Bit 4:  Clear Receive FIFO */

/* Ethernet MAC Transmit Control (MACTCTL), offset 0x00c */

#define MAC_TCTL_TXEN         (1 << 0)  /* Bit 0:  Enable Transmitter */
#define MAC_TCTL_PADEN        (1 << 1)  /* Bit 1:  Enable Packet Padding */
#define MAC_TCTL_CRC          (1 << 2)  /* Bit 2:  Enable CRC Generation */
#define MAC_TCTL_DUPLEX       (1 << 4)  /* Bit 4:  Enable Duplex Mode */

/* Ethernet MAC Threshold (MACTHR), offset 0x01c */

#define MAC_THR_MASK          0x3f      /* Bits 5-0: Threshold Value */

/* Ethernet MAC Management Control (MACMCTL), offset 0x020 */

#define MAC_MCTL_START        (1 << 0)  /* Bit 0:  MII Register Transaction Enable */
#define MAC_MCTL_WRITE        (1 << 1)  /* Bit 1:  MII Register Transaction Type */
#define MAC_MCTL_REGADR_SHIFT 3         /* Bits 7-3: MII Register Address */
#define MAC_MCTL_REGADR_MASK  (0x1f << MAC_MCTL_REGADR_SHIFT)

/* Ethernet MAC Management Divider (MACMDV), offset 0x024 */

#define MAC_MDV_MASK          0xff      /* Bits 7-0: Clock Divider */

/* Ethernet MAC Management Transmit Data (MACTXD), offset 0x02c */

#define MAC_MTXD_MASK         0xffff    /* Bits 15-0: MII Register Transmit Data */

/* Ethernet MAC Management Receive Data (MACRXD), offset 0x030 */

#define MAC_MTRD_MASK         0xffff    /* Bits 15-0: MII Register Receive Data */

/* Ethernet MAC Number of Packets (MACNP), offset 0x034 */

#define MAC_NP_MASK           0x3f      /* Bits 5-0: Number of Packets in Receive FIFO */

/* Ethernet MAC Transmission Request (MACTR), offset 0x038 */

#define MAC_TR_NEWTX          (1 << 0)  /* Bit 0:  New Transmission */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_LM_LM3S_ETHERNET_H */
