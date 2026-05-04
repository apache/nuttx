/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32_usbfs.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_USBFS_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_USBFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <chip.h>

#ifdef CONFIG_STM32H5_HAVE_USBFS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Endpoint Registers */

#define STM32_USB_EPR_OFFSET(n)      ((n) << 2) /* USB endpoint n register (32-bits) */

#define STM32_USB_EP0R_OFFSET        0x0000  /* USB endpoint 0 register (32-bits) */
#define STM32_USB_EP1R_OFFSET        0x0004  /* USB endpoint 1 register (32-bits) */
#define STM32_USB_EP2R_OFFSET        0x0008  /* USB endpoint 2 register (32-bits) */
#define STM32_USB_EP3R_OFFSET        0x000c  /* USB endpoint 3 register (32-bits) */
#define STM32_USB_EP4R_OFFSET        0x0010  /* USB endpoint 4 register (32-bits) */
#define STM32_USB_EP5R_OFFSET        0x0014  /* USB endpoint 5 register (32-bits) */
#define STM32_USB_EP6R_OFFSET        0x0018  /* USB endpoint 6 register (32-bits) */
#define STM32_USB_EP7R_OFFSET        0x001c  /* USB endpoint 7 register (32-bits) */

/* Common Registers */

#define STM32_USB_CNTR_OFFSET        0x0040  /* USB control register (32-bits) */
#define STM32_USB_ISTR_OFFSET        0x0044  /* USB interrupt status register (32-bits) */
#define STM32_USB_FNR_OFFSET         0x0048  /* USB frame number register (32-bits) */
#define STM32_USB_DADDR_OFFSET       0x004c  /* USB device address (32-bits) */
#define STM32_USB_LPMCSR_OFFSET      0x0054  /* LPM control and status register (32-bits) */
#define STM32_USB_BCDR_OFFSET        0x0058  /* Battery charging detector (32-bits) */

/* Buffer Descriptor Table (Relatative to BTABLE address) */

#define STM32_USB_TX_WOFFSET    (0)     /* Transmission buffer n (32-bits) */
#define STM32_USB_RX_WOFFSET    (4)     /* Reception buffer address n (32-bits) */

#define STM32_USB_BTABLE_RADDR(ep,o) ((((uint32_t)STM32_USB_FS_RAM_BASE + ((ep) << 3)) + (o)))

/* Register Addresses *******************************************************/

/* Endpoint Registers */

#define STM32_USB_EPR(n)             (STM32_USB_FS_BASE + STM32_USB_EPR_OFFSET(n))
#define STM32_USB_EP0R               (STM32_USB_FS_BASE + STM32_USB_EP0R_OFFSET)
#define STM32_USB_EP1R               (STM32_USB_FS_BASE + STM32_USB_EP1R_OFFSET)
#define STM32_USB_EP2R               (STM32_USB_FS_BASE + STM32_USB_EP2R_OFFSET)
#define STM32_USB_EP3R               (STM32_USB_FS_BASE + STM32_USB_EP3R_OFFSET)
#define STM32_USB_EP4R               (STM32_USB_FS_BASE + STM32_USB_EP4R_OFFSET)
#define STM32_USB_EP5R               (STM32_USB_FS_BASE + STM32_USB_EP5R_OFFSET)
#define STM32_USB_EP6R               (STM32_USB_FS_BASE + STM32_USB_EP6R_OFFSET)
#define STM32_USB_EP7R               (STM32_USB_FS_BASE + STM32_USB_EP7R_OFFSET)

/* Common Registers */

#define STM32_USB_CNTR               (STM32_USB_FS_BASE + STM32_USB_CNTR_OFFSET)
#define STM32_USB_ISTR               (STM32_USB_FS_BASE + STM32_USB_ISTR_OFFSET)
#define STM32_USB_FNR                (STM32_USB_FS_BASE + STM32_USB_FNR_OFFSET)
#define STM32_USB_DADDR              (STM32_USB_FS_BASE + STM32_USB_DADDR_OFFSET)
#define STM32_USB_LPMCSR             (STM32_USB_FS_BASE + STM32_USB_LPMCSR_OFFSET)
#define STM32_USB_BCDR               (STM32_USB_FS_BASE + STM32_USB_BCDR_OFFSET)

/* Buffer Descriptor Table (Relative to BTABLE address) */

#define STM32_USB_BTABLE_ADDR(ep,o)  (STM32_USB_BTABLE_RADDR(ep,o))
#define STM32_USB_TX(ep)              STM32_USB_BTABLE_RADDR(ep,STM32_USB_TX_WOFFSET)
#define STM32_USB_RX(ep)              STM32_USB_BTABLE_RADDR(ep,STM32_USB_RX_WOFFSET)

/* Register Bitfield Definitions ********************************************/

/* USB endpoint register */

#define USB_EPR_EA_SHIFT               (0)       /* Bits 3:0 [3:0]: Endpoint Address */
#define USB_EPR_EA_MASK                (0X0f << USB_EPR_EA_SHIFT)
#define USB_EPR_STATTX_SHIFT           (4)       /* Bits 5-4: Status bits, for transmission transfers */
#define USB_EPR_STATTX_MASK            (3 << USB_EPR_STATTX_SHIFT)
#  define USB_EPR_STATTX_DIS           (0 << USB_EPR_STATTX_SHIFT) /* EndPoint TX DISabled */
#  define USB_EPR_STATTX_STALL         (1 << USB_EPR_STATTX_SHIFT) /* EndPoint TX STALLed */
#  define USB_EPR_STATTX_NAK           (2 << USB_EPR_STATTX_SHIFT) /* EndPoint TX NAKed */
#  define USB_EPR_STATTX_VALID         (3 << USB_EPR_STATTX_SHIFT) /* EndPoint TX VALID */
#  define USB_EPR_STATTX_DTOG1         (1 << USB_EPR_STATTX_SHIFT) /* EndPoint TX Data Toggle bit1 */
#  define USB_EPR_STATTX_DTOG2         (2 << USB_EPR_STATTX_SHIFT) /* EndPoint TX Data Toggle bit2 */

#define USB_EPR_DTOG_TX                (1 << 6)  /* Bit 6: Data Toggle, for transmission transfers */
#define USB_EPR_CTR_TX                 (1 << 7)  /* Bit 7: Correct Transfer for transmission */
#define USB_EPR_EP_KIND                (1 << 8)  /* Bit 8: Endpoint Kind */
#define USB_EPR_EPTYPE_SHIFT           (9)       /* Bits 10-9: Endpoint type */
#define USB_EPR_EPTYPE_MASK            (3 << USB_EPR_EPTYPE_SHIFT)
#  define USB_EPR_EPTYPE_BULK          (0 << USB_EPR_EPTYPE_SHIFT) /* EndPoint BULK */
#  define USB_EPR_EPTYPE_CONTROL       (1 << USB_EPR_EPTYPE_SHIFT) /* EndPoint CONTROL */
#  define USB_EPR_EPTYPE_ISOC          (2 << USB_EPR_EPTYPE_SHIFT) /* EndPoint ISOCHRONOUS */
#  define USB_EPR_EPTYPE_INTERRUPT     (3 << USB_EPR_EPTYPE_SHIFT) /* EndPoint INTERRUPT */

#define USB_EPR_SETUP                  (1 << 11) /* Bit 11: Setup transaction completed */
#define USB_EPR_STATRX_SHIFT           (12)      /* Bits 13-12: Status bits, for reception transfers */
#define USB_EPR_STATRX_MASK            (3 << USB_EPR_STATRX_SHIFT)
#  define USB_EPR_STATRX_DIS           (0 << USB_EPR_STATRX_SHIFT) /* EndPoint RX DISabled */
#  define USB_EPR_STATRX_STALL         (1 << USB_EPR_STATRX_SHIFT) /* EndPoint RX STALLed */
#  define USB_EPR_STATRX_NAK           (2 << USB_EPR_STATRX_SHIFT) /* EndPoint RX NAKed */
#  define USB_EPR_STATRX_VALID         (3 << USB_EPR_STATRX_SHIFT) /* EndPoint RX VALID */
#  define USB_EPR_STATRX_DTOG1         (1 << USB_EPR_STATRX_SHIFT) /* EndPoint RX Data TOGgle bit1 */
#  define USB_EPR_STATRX_DTOG2         (2 << USB_EPR_STATRX_SHIFT) /* EndPoint RX Data TOGgle bit1 */

#define USB_EPR_DTOG_RX                (1 << 14) /* Bit 14: Data Toggle, for reception transfers */
#define USB_EPR_CTR_RX                 (1 << 15) /* Bit 15: Correct Transfer for reception */

/* USB control register */

#define USB_CNTR_FRES                  (1 << 0)  /* Bit 0: Force USB Reset */
#define USB_CNTR_PDWN                  (1 << 1)  /* Bit 1: Power down */
#define USB_CNTR_SUSPRDY               (1 << 2)  /* Bit 2: Suspend Ready */
#define USB_CNTR_FSUSP                 (1 << 3)  /* Bit 3: Force suspend */
#define USB_CNTR_RESUME                (1 << 4)  /* Bit 4: Resume request */
#define USB_CNTR_L1RESUME              (1 << 5)  /* Bit 5: LPM L1 Resume request */
#define USB_CNTR_L1REQ                 (1 << 7)  /* Bit 7: LPM L1 state request interrupt mask */
#define USB_CNTR_ESOFM                 (1 << 8)  /* Bit 8: Expected Start Of Frame Interrupt Mask */
#define USB_CNTR_SOFM                  (1 << 9)  /* Bit 9: Start Of Frame Interrupt Mask */
#define USB_CNTR_RESETM                (1 << 10) /* Bit 10: USB Reset Interrupt Mask */
#define USB_CNTR_SUSPM                 (1 << 11) /* Bit 11: Suspend mode Interrupt Mask */
#define USB_CNTR_WKUPM                 (1 << 12) /* Bit 12: Wakeup Interrupt Mask */
#define USB_CNTR_ERRM                  (1 << 13) /* Bit 13: Error Interrupt Mask */
#define USB_CNTR_PMAOVRN               (1 << 14) /* Bit 14: Packet Memory Area Over / Underrun Interrupt Mask */
#define USB_CNTR_CTRM                  (1 << 15) /* Bit 15: Correct Transfer Interrupt Mask */
#define USB_CNTR_THR512M               (1 << 16) /* Bit 16: 512byte Threshold interrupt mask */
#define USB_CNTR_DDISCM                (1 << 17) /* Bit 17: Device disconnection mask */
#define USB_CNTR_HOST                  (1 << 31) /* Bit 31: Host Mode */

#define USB_CNTR_ALLINTS               (USB_CNTR_L1REQ|USB_CNTR_ESOFM|USB_CNTR_SOFM|USB_CNTR_RESETM|\
                                        USB_CNTR_SUSPM|USB_CNTR_WKUPM|USB_CNTR_ERRM|USB_CNTR_PMAOVRN|\
                                        USB_CNTR_CTRM)

/* USB interrupt status register */

#define USB_ISTR_EPID_SHIFT            (0)       /* Bits 3-0: Endpoint Identifier */
#define USB_ISTR_EPID_MASK             (0x0f << USB_ISTR_EPID_SHIFT)
#define USB_ISTR_DIR                   (1 << 4)  /* Bit 4:  Direction of transaction */
#define USB_ISTR_L1REQ                 (1 << 7)  /* Bit 7:  LPM L1 state request */
#define USB_ISTR_ESOF                  (1 << 8)  /* Bit 8:  Expected Start Of Frame */
#define USB_ISTR_SOF                   (1 << 9)  /* Bit 9:  Start Of Frame */
#define USB_ISTR_RESET                 (1 << 10) /* Bit 10: USB RESET request. Device connection in Host mode */
#define USB_ISTR_SUSP                  (1 << 11) /* Bit 11: Suspend mode request */
#define USB_ISTR_WKUP                  (1 << 12) /* Bit 12: Wake up */
#define USB_ISTR_ERR                   (1 << 13) /* Bit 13: Error */
#define USB_ISTR_PMAOVRN               (1 << 14) /* Bit 14: Packet Memory Area Over / Underrun */
#define USB_ISTR_CTR                   (1 << 15) /* Bit 15: Correct Transfer */
#define USB_ISTR_THR512                (1 << 16) /* Bit 16: 512byte threshold interrupt */
#define USB_ISTR_DDISC                 (1 << 17) /* Bit 17: Device connection */
#define USB_ISTR_DCON_STAT             (1 << 29) /* Bit 29: Device connection status */
#define USB_ISTR_LS_DCONN              (1 << 30) /* Bit 30: Low-speed device connected */

#define USB_ISTR_ALLINTS               (USB_ISTR_L1REQ|USB_ISTR_ESOF|USB_ISTR_SOF|USB_ISTR_RESET|\
                                        USB_ISTR_SUSP|USB_ISTR_WKUP|USB_ISTR_ERR|USB_ISTR_PMAOVRN|\
                                       USB_ISTR_CTR)

/* USB frame number register */

#define USB_FNR_FN_SHIFT               (0)       /* Bits 10-0: Frame Number */
#define USB_FNR_FN_MASK                (0x07ff << USB_FNR_FN_SHIFT)
#define USB_FNR_LSOF_SHIFT             (11)      /* Bits 12-11: Lost SOF */
#define USB_FNR_LSOF_MASK              (3 << USB_FNR_LSOF_SHIFT)
#define USB_FNR_LCK                    (1 << 13) /* Bit 13: Locked */
#define USB_FNR_RXDM                   (1 << 14) /* Bit 14: Receive Data - Line Status */
#define USB_FNR_RXDP                   (1 << 15) /* Bit 15: Receive Data + Line Status */

/* USB device address */

#define USB_DADDR_ADD_SHIFT            (0)       /* Bits 6-0: Device Address */
#define USB_DADDR_ADD_MASK             (0x7f << USB_DADDR_ADD_SHIFT)
#define USB_DADDR_EF                   (1 << 7)  /* Bit 7: Enable Function */

/* LPM control and status register (32-bits) */

#define USB_LPMCSR_LPMEN               (1 << 0)  /* Bit 0:  LPM support enable */
#define USB_LPMCSR_LPMACK              (1 << 1)  /* Bit 1:  LPM Token acknowledge enable */
#define USB_LPMCSR_REMWAKE             (1 << 3)  /* Bit 3:  bRemoteWake value */
#define USB_LPMCSR_BESL_SHIFT          (4)       /* Bits 4-7: BESL value */
#define USB_LPMCSR_BESL_MASK           (15 << USB_LPMCSR_BESL_SHIFT)

/* Battery charging detector (32-bits) */

#define USB_BCDR_BCDEN                 (1 << 0)  /* Bit 0:  Battery charging detector (BCD) enable */
#define USB_BCDR_DCDEN                 (1 << 1)  /* Bit 1:  Data contact detection (DCD) mode enable */
#define USB_BCDR_PDEN                  (1 << 2)  /* Bit 2:  Primary detection (PD) mode enable */
#define USB_BCDR_SDEN                  (1 << 3)  /* Bit 3:  Secondary detection (SD) mode enable */
#define USB_BCDR_DCDET                 (1 << 4)  /* Bit 4:  Data contact detection (DCD) status */
#define USB_BCDR_PDET                  (1 << 5)  /* Bit 5:  Primary detection (PD) status */
#define USB_BCDR_SDET                  (1 << 6)  /* Bit 6:  Secondary detection (SD) status */
#define USB_BCDR_PS2DET                (1 << 7)  /* Bit 7:  DM pull-up detection status */
#define USB_BCDR_DPPU                  (1 << 15) /* Bit 15: DP pull-up control */
#define USB_BCDR_DPPD                  (1 << 15) /* Bit 15: DP pull-down control (host mode) */

/****************************************************************************
 * USB DRD Host Mode Register Definitions
 ****************************************************************************/

/* Channel/Endpoint register */

#define USB_CHEP_ADDR_SHIFT            (0)       /* Bits 3-0: Endpoint address */
#define USB_CHEP_ADDR_MASK             (0xf << USB_CHEP_ADDR_SHIFT)
#define USB_CHEP_TX_STTX_SHIFT         (4)       /* Bits 5-4: Status TX */
#define USB_CHEP_TX_STTX_MASK          (3 << USB_CHEP_TX_STTX_SHIFT)
#  define USB_CHEP_TX_STTX_DIS         (0 << USB_CHEP_TX_STTX_SHIFT) /* Channel TX disabled */
#  define USB_CHEP_TX_STTX_STALL       (1 << USB_CHEP_TX_STTX_SHIFT) /* Channel TX stalled */
#  define USB_CHEP_TX_STTX_NAK         (2 << USB_CHEP_TX_STTX_SHIFT) /* Channel TX NAK */
#  define USB_CHEP_TX_STTX_VALID       (3 << USB_CHEP_TX_STTX_SHIFT) /* Channel TX valid */
#  define USB_CHEP_TX_DTOG1            (1 << USB_CHEP_TX_STTX_SHIFT) /* TX Data Toggle bit 1 */
#  define USB_CHEP_TX_DTOG2            (2 << USB_CHEP_TX_STTX_SHIFT) /* TX Data Toggle bit 2 */

#define USB_CHEP_DTOG_TX               (1 << 6)  /* Bit 6: Data Toggle TX */
#define USB_CHEP_VTTX                  (1 << 7)  /* Bit 7: Valid transaction transmitted */
#define USB_CHEP_KIND                  (1 << 8)  /* Bit 8: Endpoint/Channel KIND */
#define USB_CHEP_UTYPE_SHIFT           (9)       /* Bits 10-9: USB type of transaction */
#define USB_CHEP_UTYPE_MASK            (3 << USB_CHEP_UTYPE_SHIFT)
#  define USB_CHEP_UTYPE_BULK          (0 << USB_CHEP_UTYPE_SHIFT) /* Bulk transfer */
#  define USB_CHEP_UTYPE_CTRL          (1 << USB_CHEP_UTYPE_SHIFT) /* Control transfer */
#  define USB_CHEP_UTYPE_ISOC          (2 << USB_CHEP_UTYPE_SHIFT) /* Isochronous transfer */
#  define USB_CHEP_UTYPE_INTR          (3 << USB_CHEP_UTYPE_SHIFT) /* Interrupt transfer */

#define USB_CHEP_SETUP                 (1 << 11) /* Bit 11: Setup transaction completed */
#define USB_CHEP_RX_STRX_SHIFT         (12)      /* Bits 13-12: Status RX */
#define USB_CHEP_RX_STRX_MASK          (3 << USB_CHEP_RX_STRX_SHIFT)
#  define USB_CHEP_RX_STRX_DIS         (0 << USB_CHEP_RX_STRX_SHIFT) /* Channel RX disabled */
#  define USB_CHEP_RX_STRX_STALL       (1 << USB_CHEP_RX_STRX_SHIFT) /* Channel RX stalled */
#  define USB_CHEP_RX_STRX_NAK         (2 << USB_CHEP_RX_STRX_SHIFT) /* Channel RX NAK */
#  define USB_CHEP_RX_STRX_VALID       (3 << USB_CHEP_RX_STRX_SHIFT) /* Channel RX valid */
#  define USB_CHEP_RX_DTOG1            (1 << USB_CHEP_RX_STRX_SHIFT) /* RX Data Toggle bit 1 */
#  define USB_CHEP_RX_DTOG2            (2 << USB_CHEP_RX_STRX_SHIFT) /* RX Data Toggle bit 2 */

#define USB_CHEP_DTOG_RX               (1 << 14) /* Bit 14: Data Toggle RX */
#define USB_CHEP_VTRX                  (1 << 15) /* Bit 15: Valid transaction received */
#define USB_CHEP_DEVADDR_SHIFT         (16)      /* Bits 22-16: Target device address */
#define USB_CHEP_DEVADDR_MASK          (0x7f << USB_CHEP_DEVADDR_SHIFT)
#define USB_CHEP_NAK                   (1 << 23) /* Bit 23: Previous NAK detected */
#define USB_CHEP_LSEP                  (1 << 24) /* Bit 24: Low Speed Endpoint */
#define USB_CHEP_ERRTX                 (1 << 25) /* Bit 25: Transmit error */
#define USB_CHEP_ERRRX                 (1 << 26) /* Bit 26: Receive error */

/* Register mask for preserving r/w bits when modifying toggle bits */

#define USB_CHEP_REG_MASK              (USB_CHEP_ERRRX | USB_CHEP_ERRTX | \
                                        USB_CHEP_LSEP | USB_CHEP_DEVADDR_MASK | \
                                        USB_CHEP_VTRX | USB_CHEP_SETUP | \
                                        USB_CHEP_UTYPE_MASK | USB_CHEP_KIND | \
                                        USB_CHEP_VTTX | USB_CHEP_ADDR_MASK | \
                                        USB_CHEP_NAK)

#define USB_CHEP_TX_DTOGMASK           (USB_CHEP_TX_STTX_MASK | USB_CHEP_REG_MASK)
#define USB_CHEP_RX_DTOGMASK           (USB_CHEP_RX_STRX_MASK | USB_CHEP_REG_MASK)
#define USB_CH_T_MASK                  ((~USB_CHEP_UTYPE_MASK) & USB_CHEP_REG_MASK)
#define USB_CHEP_DB_MSK                (0xffff0f0f)

/* PMA (Packet Memory Area) definitions for host mode */

#define USB_DRD_PMA_SIZE               (2048)     /* 2KB PMA */
#define USB_DRD_NCHANNELS              (8)        /* 8 host channels */

/* PMA buffer descriptor structure address calculation
 * Each channel has TX and RX buffer descriptors (8 bytes total per channel)
 */

#define USB_PMA_TXBD_OFFSET(ch)        ((ch) * 8)
#define USB_PMA_RXBD_OFFSET(ch)        ((ch) * 8 + 4)

/* PMA TX buffer descriptor bit definitions */

#define USB_PMA_TXBD_ADDR_SHIFT        (2)        /* Bits 15:2: TX buffer address */
#define USB_PMA_TXBD_ADDR_MASK         (0x3fff << USB_PMA_TXBD_ADDR_SHIFT)
#define USB_PMA_TXBD_COUNT_SHIFT       (16)       /* Bits 25:16: TX byte count */
#define USB_PMA_TXBD_COUNT_MASK        (0x3ff << USB_PMA_TXBD_COUNT_SHIFT)
#define USB_PMA_TXBD_ADDMSK            (0xffff0000)
#define USB_PMA_TXBD_COUNTMSK          (0x0000ffff)

/* PMA RX buffer descriptor bit definitions */

#define USB_PMA_RXBD_ADDR_SHIFT        (2)        /* Bits 15-2: RX buffer address */
#define USB_PMA_RXBD_ADDR_MASK         (0x3fff << USB_PMA_RXBD_ADDR_SHIFT)
#define USB_PMA_RXBD_COUNT_SHIFT       (16)       /* Bits 25-16: RX byte count */
#define USB_PMA_RXBD_COUNT_MASK        (0x3ff << USB_PMA_RXBD_COUNT_SHIFT)
#define USB_PMA_RXBD_NUM_BLOCK_SHIFT   (26)      /* Bits 30-26: Number of blocks */
#define USB_PMA_RXBD_NUM_BLOCK_MASK    (0x1f << USB_PMA_RXBD_NUM_BLOCK_SHIFT)
#define USB_PMA_RXBD_BLSIZE            (1 << 31) /* Bit 31: Block size: 0=2bytes, 1=32bytes */
#define USB_PMA_RXBD_ADDMSK            (0xffff0000)

/* PMA start address (after buffer descriptor table)
 * BDT size = 8 channels * 8 bytes = 64 bytes, aligned to 64
 */

#define USB_DRD_PMA_BDT_SIZE           (USB_DRD_NCHANNELS * 8)
#define USB_DRD_PMA_START_ADDR         (USB_DRD_PMA_BDT_SIZE)

/* Reception buffer address */

#define USB_ADDR_RX_SHIFT              (2)       /* Bits 15-2: Reception Buffer Address */
#define USB_ADDR_RX_MASK               (0x3fff << USB_ADDR_RX_SHIFT)

/* Reception byte count */

#define USB_COUNT_RX_BL_SIZE           (1 << 31) /* Bit 31: Block size: 0=2bytes, 1=32bytes */
#define USB_COUNT_RX_NUM_BLOCK_SHIFT   (26)      /* Bits 30-26: Number of blocks */
#define USB_COUNT_RX_NUM_BLOCK_MASK    (0x1f << USB_COUNT_RX_NUM_BLOCK_SHIFT)
#define USB_COUNT_RX_SHIFT             (16)      /* Bits 25-16: Reception Byte Count */
#define USB_COUNT_RX_MASK              (0x3ff << USB_COUNT_RX_SHIFT)

#endif /* CONFIG_STM32H5_HAVE_USBFS */
#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_USBFS_H */
