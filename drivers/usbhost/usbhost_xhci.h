/****************************************************************************
 * drivers/usbhost/usbhost_xhci.h
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

#ifndef __DRIVERS_USBHOST_USBHOST_XHCI_H
#define __DRIVERS_USBHOST_USBHOST_XHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* xHCI constants */

#define XHCI_MAX_DEVS                (255)                 /* Up to 255 USB devices or hubs */

#define XHCI_PAGE_SIZE               (4096)                /* 4K Bytes is minimum Page size supported */
#define XHCI_BUF_ALIGN               (64)                  /* Buffer alignment */
#define XHCI_CTX_ALIGN               (64)                  /* Context  alignment */
#define XHCI_TRB_ALIGN               (16)                  /* TRB alignment */
#define XHCI_TD_LEN_MAX              (65536)               /* Maximum Transfer Descriptor data length is 64K bytes */
#define XHCI_MAX_ENDPOINTS           (31)                  /* EP0 + 1-15 EP OUT 1-15 EP IN */

/* xHCI Extended Capability Codes */

#define XHCI_CAP_USBLEGSUP           (1)                   /* USB Legacy Support */
#define XHCI_CAP_SUPPROTO            (2)                   /* Supported Protocol */
#define XHCI_CAP_EXTPWRMAN           (3)                   /* Extended Power Management */
#define XHCI_CAP_IOVIRT              (4)                   /* I/O Virtualization */
#define XHCI_CAP_MSGINT              (5)                   /* Message Interrupt */
#define XHCI_CAP_LOCALMEM            (6)                   /* Local Memory */
                                                           /* IDs 7-9: Reserved */
#define XHCI_CAP_USBDEBUG            (10)                  /* USB Debug Capability */
                                                           /* IDs 11-16: Reserved */
#define XHCI_CAP_EXTMSGINT           (17)                  /* Extended Message Interrupt */
                                                           /* IDs 18-191: Reserved */
                                                           /* IDs 192-255: Vendor Defined */

/* USB Legacy Support Capability */

#define XHCI_USBLEGSUP               (0x0000)              /* USB Legacy Support Capability Register */
#define XHCI_USBLEGCTLSTS            (0x0004)              /* USB Legacy Support Control and Status Register */

/* XHCI capability ID's */

                                                           /* 0: Reserved */
#define XHCI_ID_USBLEGSUP            (1)                   /* USB Legacy Support */
#define XHCI_ID_SUPPROTO             (2)                   /* Supported Protocol */
#define XHCI_ID_EXTPM                (3)                   /* Extended Power Management */
#define XHCI_ID_IOVIRT               (4)                   /* I/O Virtualization */
#define XHCI_ID_MSGIRQ               (5)                   /* Message Interrupt */
#define XHCI_ID_LOCALMEM             (6)                   /* Local Memory */
                                                           /* 7-9: Reserved */
#define XHCI_ID_DEBUG                (10)                  /* USB Debug Capability */
                                                           /* 11-16: Reserved */
#define XHCI_ID_EXTMSGIRQ            (17)                  /* Extended Message Interrupt */
                                                           /* 18-191: Reserved */
#define XHCI_ID_VENDOR               (192)                 /* Vendor Defined */

/* USB Legacy Support Capability */

#define XHCI_USBLEGSUP_ID(x)         ((x) & 0xff)          /* Get Capability ID */
#define XHCI_USBLEGSUP_NEXT(x)       (((x) >> 8) & 0xff)   /* Get next Capability Pointer */
#define XHCI_USBLEGSUP_BIOS_SEM      (2)                   /* Offset in bytes to BIOS Owned Semaphore */
#define XHCI_USBLEGSUP_OS_SEM        (3)                   /* Offset in bytes to OS Owned Semaphore */

/* Host Controller Capability Registers
 * Address: Base + offset
 */

#define XHCI_CAPLENGTH               (0x0000)              /* Capability Register Length */
                                                           /* 0x0001: Reserved */
#define XHCI_HCIVERSION              (0x0002)              /* Interface Version Number */
#define XHCI_HCSPARAMS1              (0x0004)              /* Structural Parameters 1 */
#define XHCI_HCSPARAMS2              (0x0008)              /* Structural Parameters 2 */
#define XHCI_HCSPARAMS3              (0x000c)              /* Structural Parameters 3 */
#define XHCI_HCCPARAMS1              (0x0010)              /* Capability Parameters 1 */
#define XHCI_DBOFF                   (0x0014)              /* Doorbell Offset */
#define XHCI_RTSOFF                  (0x0018)              /* Runtime Register Space Offset */
#define XHCI_HCCPARAMS2              (0x001c)              /* Capability Parameters 2 */

/* Structural Parameters 1 */

#define XHCI_HCSPARAMS1_MAXSLOTS(x)  ((x) & 0xff)          /* Bits 0-7: Number of Device Slots (MaxSlots) */
#define XHCI_HCSPARAMS1_MAXINTRS(x)  (((x) >> 8) & 0x7ff)  /* Bits 8-18: Number of Interrupters (MaxIntrs) */
                                                           /* Bits 19-23: Reserved */
#define XHCI_HCSPARAMS1_MAXPORTS(x)  (((x) >> 24) & 0xff)  /* Bits 24-31: Number of Ports (MaxPorts) */

/* Structural Parameters 2 */

#define XHCI_HCSPARAMS2_MAXSPBHI(x)  (((x) >> 21) & 0x1f)  /* Bits 21-25: Max Scratchpad Bufs Hi */
#define XHCI_HCSPARAMS2_MAXSPBLO(x)  (((x) >> 27) & 0x1f)  /* Bits 27-31: Max Scratchpad Bufs Lo */
#define XHCI_HCSPARAMS2_MAXSPB(x)    (XHCI_HCSPARAMS2_MAXSPBLO(x) + \
                                      (XHCI_HCSPARAMS2_MAXSPBHI(x) << 5))

#define XHCI_HCSPARAMS2_ERST(x)      (((x) >> 4) & 0x0f)   /* Bits 4-7: Event Ring Segment Table Max */

/* TODO: Structural Parameters 3 */

/* Capability Parameters 1 */

#define XHCI_HCCPARAMS1_CSZ          (1 << 2)              /* Bit 2: Context Size */
#define XHCI_HCCPARAMS1_XECP(x)      ((x) >> 16)           /* Bits 16-31: xHCI Extended Capabilities Pointer */

/* TODO: Capability Parameters 2 */

/* Host Controller Operational Registers
 * Address: Operational Base + offset
 */

#define XHCI_USBCMD                  (0x0000)              /* USB Command (32b) */
#define XHCI_USBSTS                  (0x0004)              /* USB Status (32b) */
#define XHCI_PAGESIZE                (0x0008)              /* Page Size (32b) */
                                                           /* 0x0c - 0x13: Reserved */
#define XHCI_DNCTRL                  (0x0014)              /* Device Notification Control (32b) */
#define XHCI_CRCR                    (0x0018)              /* Command Ring Control (64b) */
                                                           /* 0x20 - 0x2f: Reserved */
#define XHCI_DCBAAP                  (0x0030)              /* Device Context Base Address Array Pointer (64b) */
#define XHCI_CONFIG                  (0x0038)              /* Configure (32b) */
                                                           /* 0x03c - 0x3ff: Reserved */
#define XHCI_PORT(n)                 (0x400 + 0x10 * (n))  /* Port Register Set */
#define XHCI_PORTSC(n)               (XHCI_PORT(n) + 0x00) /* Port Status and Control (32b) */
#define XHCI_PORTPMSC(n)             (XHCI_PORT(n) + 0x04) /* Port Power Management Status and Control (32b) */
#define XHCI_PORTLI(n)               (XHCI_PORT(n) + 0x08) /* Port Link Info (32b) */
#define XHCI_PORTHLPMC(n)            (XHCI_PORT(n) + 0x0c) /* Port Hardware LPM Control (32b) */

/* Host Controller Runtime Registers (Address: Runtime Base + offset) */

#define XHCI_MFINDEX                 (0x0000)              /* Microframe Index */
                                                           /* 0x0004 - 0x001f: Reserved */
#define XHCI_IRQ(n)                  (0x0020 + 32 * (n))   /* Interrupter Register Set */
#define XHCI_IMAN(n)                 (XHCI_IRQ(n) + 0x00)  /* Interrupter Management (32b) */
#define XHCI_IMOD(n)                 (XHCI_IRQ(n) + 0x04)  /* Interrupter Moderation (32b) */
#define XHCI_ERSTSZ(n)               (XHCI_IRQ(n) + 0x08)  /* Event Ring Segment Table Size (32b) */
                                                           /* 0x0c: Reserved */
#define XHCI_ERSTBA(n)               (XHCI_IRQ(n) + 0x10)  /* Event Ring Segment Table Base Address (64b) */
#define XHCI_ERDP(n)                 (XHCI_IRQ(n) + 0x18)  /* Event Ring Dequeue Pointer (64b) */

/* Doorbel Registers */

#define XHCI_DOORBEL(n)              (0x0000 + 4 * (n))
#define XHCI_DOORBEL_TARGET_SHIFT    (0)                   /* Bits 0-7: DB Target */
#define XHCI_DOORBEL_TARGET_MASK     (0xff << XHCI_DOORBEL_TARGET_SHIFT)
#define XHCI_DOORBEL_TARGET(x)       (((x) << XHCI_DOORBEL_TARGET_SHIFT) & XHCI_DOORBEL_TARGET_MASK)
                                                           /* Bits 8-15: Reserved */
#define XHCI_DOORBEL_TASK_SHIFT      (16)                  /* Bits 16-31: DB Task ID */
#define XHCI_DOORBEL_TASK_MASK       (0xffff << XHCI_DOORBEL_TASK_SHIFT)
#define XHCI_DOORBEL_TASK(x)         (((x) << XHCI_DOORBEL_TASK_SHIFT) & XHCI_DOORBEL_TASK_MASK)

/* USB Command */

#define XHCI_USBCMD_RS               (1 << 0)              /* Bit 0: Run/Stop */
#define XHCI_USBCMD_HCRST            (1 << 1)              /* Bit 1: Host Controller Reset */
#define XHCI_USBCMD_INTE             (1 << 2)              /* Bit 2: Interrupter Enable */
#define XHCI_USBCMD_HSEE             (1 << 3)              /* Bit 3: Host System Error Enable */
                                                           /* Bits 4-6: Reserved */
#define XHCI_USBCMD_LHCRST           (1 << 7)              /* Bit 7: Light Host Controller Reset */
#define XHCI_USBCMD_CSS              (1 << 8)              /* Bit 8: Controller Save State */
#define XHCI_USBCMD_CRS              (1 << 9)              /* Bit 9: Controller Restore State */
#define XHCI_USBCMD_EWE              (1 << 10)             /* Bit 10: Enable Wrap Event */
#define XHCI_USBCMD_EU3S             (1 << 11)             /* Bit 11: Enable U3 MFINDEX Stop */
                                                           /* Bit 12: Reserved */
#define XHCI_USBCMD_CME              (1 << 13)             /* Bit 13: CEM Enable */
#define XHCI_USBCMD_ETE              (1 << 14)             /* Bit 14: Extended TBC Enable */
#define XHCI_USBCMD_TSCEN            (1 << 15)             /* Bit 15: Extended TBC TRB Status Enable */
#define XHCI_USBCMD_VTIOE            (1 << 16)             /* Bit 16: VTIO Enable */
                                                           /* Bits 17-31: Reserved */

/* USB Status */

#define XHCI_USBSTS_HCH              (1 << 0)              /* Bit 0: Host Controller Halted */
                                                           /* Bit 1: Reserved */
#define XHCI_USBSTS_HSE              (1 << 2)              /* Bit 2: Host System Error */
#define XHCI_USBSTS_EINT             (1 << 3)              /* Bit 3: Event Interrupt */
#define XHCI_USBSTS_PCD              (1 << 4)              /* Bit 4: Port Change Detect */
                                                           /* Bits 5-7: Reserved */
#define XHCI_USBSTS_SSS              (1 << 8)              /* Bit 8: Save State Status */
#define XHCI_USBSTS_RSS              (1 << 9)              /* Bit 9: Restore State Status */
#define XHCI_USBSTS_SRE              (1 << 10)             /* Bit 10: Save/Restore Error */
#define XHCI_USBSTS_CNR              (1 << 11)             /* Bit 11: Controller Not Ready */
#define XHCI_USBSTS_HCE              (1 << 12)             /* Bit 12: Host Controller Error */
                                                           /* Bits 13-31: Reserved */

/* Page Size */

#define XHCI_PAGESIZE_MASK           (0xffff)              /* Bits 0-15: Page Size */

/* Device Notification Control */

#define XHCI_DNCTRL_MASK             (0xffff)              /* Bits 0-15: Notification Enable */
#define XHCI_DNCTRL_N(n)             ((1 << (n)) & XHCI_DNCTRL_MASK)

/* Command Ring Control */

#define XHCI_CRCR_RCS                (1 << 0)              /* Bit 0: Ring Cycle State */
#define XHCI_CRCR_CS                 (1 << 1)              /* Bit 1: Command Stop */
#define XHCI_CRCR_CA                 (1 << 2)              /* Bit 2: Command Abort */
#define XHCI_CRCR_CRR                (1 << 3)              /* Bit 3: Command Ring Running */
                                                           /* Bits 4-5: Reserved */

#define XHCI_CRCR_CRP_SHIFT          (6)                   /* Bits 6-63: Command Ring Pointer */
#define XHCI_CRCR_CRP_MASK           (0xffffffffffffffc0)

/* Configure */

#define XHCI_CONFIG_MDSE_SHIFT       (0)                   /* Bits 0-7: Max Device Slots Enabled (MaxSlotsEn) */
#define XHCI_CONFIG_MDSE_MASK        (0xff << XHCI_CONFIG_MDSE_SHIFT)
#define XHCI_CONFIG_U3E              (1 << 8)              /* Bit 8: U3 Entry Enable */
#define XHCI_CONFIG_CIE              (1 << 9)              /* Bit 9: Configuration Information Enable */
                                                           /* Bits 10-31: Reserved */

/* Port Status and Control */

#define XHCI_PORTSC_CCS              (1 << 0)              /* Bit 0: Current Connect Status */
#define XHCI_PORTSC_PED              (1 << 1)              /* Bit 1: Port Enabled/Disabled */
                                                           /* Bit 2: Reserved */
#define XHCI_PORTSC_OCA              (1 << 3)              /* Bit 3: Over-current Active */
#define XHCI_PORTSC_PR               (1 << 4)              /* Bit 4: Port Reset */
#define XHCI_PORTSC_PLS_SHIFT        (5)                   /* Bits 5-8: Port Link State */
#define XHCI_PORTSC_PLS_MASK         (0xf << XHCI_PORTSC_PLS_SHIFT)
#define XHCI_PORTSC_PLS(x)           (((x) & XHCI_PORTSC_PLS_MASK) >> XHCI_PORTSC_PLS_SHIFT)
#define XHCI_PORTSC_PP               (1 << 9)              /* Bit 9: Port Power */
#define XHCI_PORTSC_PS_SHIFT         (10)                  /* Bits 10-13: Port Speed */
#define XHCI_PORTSC_PS_MASK          (0xf << XHCI_PORTSC_PS_SHIFT)
#define XHCI_PORTSC_PS(x)            (((x) & XHCI_PORTSC_PS_MASK) >> XHCI_PORTSC_PS_SHIFT)
#  define XHCI_PORTSC_PS_FULL        (1)                   /* Full-speed */
#  define XHCI_PORTSC_PS_LOW         (2)                   /* Low-speed */
#  define XHCI_PORTSC_PS_HIGH        (3)                   /* High-speed */
#  define XHCI_PORTSC_PS_SUPPER11    (4)                   /* SuperSpeedPlus Gen1 x1 */
#  define XHCI_PORTSC_PS_SUPPER21    (5)                   /* SuperSpeedPlus Gen1 x2 */
#  define XHCI_PORTSC_PS_SUPPER12    (6)                   /* SuperSpeedPlus Gen1 x2 */
#  define XHCI_PORTSC_PS_SUPPER22    (7)                   /* SuperSpeedPlus Gen2 x2 */
#define XHCI_PORTSC_PIC_SHIFT        (14)                  /* Bits 14-15: Port Indicator Control */
#define XHCI_PORTSC_LWS              (1 << 16)             /* Bit 16: Port Link State Write Strobe */
#define XHCI_PORTSC_CSC              (1 << 17)             /* Bit 17: Connect Status Change */
#define XHCI_PORTSC_PEC              (1 << 18)             /* Bit 18: Port Enabled/Disabled Change */
#define XHCI_PORTSC_WRC              (1 << 19)             /* Bit 19: Warm Port Reset Change */
#define XHCI_PORTSC_OCC              (1 << 20)             /* Bit 20: Over-current Change */
#define XHCI_PORTSC_PRC              (1 << 21)             /* Bit 21: Port Reset Change */
#define XHCI_PORTSC_PLC              (1 << 22)             /* Bit 22: Port Link State Change */
#define XHCI_PORTSC_CEC              (1 << 23)             /* Bit 23: Port Config Error Change */
#define XHCI_PORTSC_CAS              (1 << 24)             /* Bit 24: Cold Attach Status */
#define XHCI_PORTSC_WCE              (1 << 25)             /* Bit 25: Wake on Connect Enable */
#define XHCI_PORTSC_WDE              (1 << 26)             /* Bit 26: Wake on Disconnect Enable */
#define XHCI_PORTSC_WOE              (1 << 27)             /* Bit 27: Wake on Over-current Enable */
                                                           /* Bits 28-29: Reserved */
#define XHCI_PORTSC_DR               (1 << 30)             /* Bit 30: Device Removable */
#define XHCI_PORTSC_WPR              (1 << 31)             /* Bit 31: Warm Port Reset */

/* Port Power Management Status and Control (USB3) */

#define XHCI_PORTPMSC_U1TO_SHIFT     (0)                   /* Bits 0-7: U1 Timeout */
#define XHCI_PORTPMSC_U2TO_SHIFT     (8)                   /* Bits 8-15: U2 Timeout */
#define XHCI_PORTPMSC_FLA            (1 << 16)             /* Bit 16: Force Link PM Accept */
                                                           /* Bits 17-31: Reserved */

/* Port Power Management Status and Control (USB2) */

#define XHCI_PORTPMSC_L1S_SHIFT      (0)                   /* Bits 0-2: L1 Status */
#define XHCI_PORTPMSC_RWE            (1 << 3)              /* Bit 3: Remote Wake Enable */
#define XHCI_PORTPMSC_BESL_SHIFT     (4)                   /* Bits 4-7: Best Effort Service Latency */
#define XHCI_PORTPMSC_L1DS_SHIFT     (8)                   /* Bits 8-15: L1 Device Slot */
#define XHCI_PORTPMSC_HLE            (1 << 16)             /* Bit 16: Hardware LPM Enable */
                                                           /* Bits 17-27: Reserved */
#define XHCI_PORTPMSC_PTC_SHIFT      (28)                  /* Bits 28-31: Port Test Control */

/* Port Link Info (USB3, reserved for USB2) */

#define XHCI_PORTLI_LER_SHIFT        (0)                   /* Bits 0-15: Link Error Count */
#define XHCI_PORTLI_RLC_SHIFT        (16)                  /* Bits 16-19: Rx Lane Count */
#define XHCI_PORTLI_TCL              (20)                  /* Bits 20-23: Tx Lane Count */
                                                           /* Bits 24-31: Reserved */

/* Interrupter Management */

#define XHCI_IMAN_IP                 (1 << 0)              /* Bit 0: Interrupt Pending */
#define XHCI_IMAN_IE                 (1 << 1)              /* Bit 1: Interrupt Enable */

/* Interrupter Moderation */

#define XHCI_IMOD_IMODI_SHIFT        (0)                   /* Bits 0-15: Interrupt Moderation Interval */
#define XHCI_IMOD_IMODC_SHIFT        (16)                  /* Bits 16-31: Interrupt Moderation Counter */

/* Event Ring Segment Table Size */

#define XHCI_ERSTS_MASK              (0xffff)              /* Bit 0-15: Event Ring Segment Table Size */
                                                           /* Bits 16-31: Reserved */

/* Event Ring Segment Table Base Address */

#define XHCI_ERSTBA_PTR_SHIFT        (6)                   /* Bit 6-63: Event Ring Segment Table Base Address Register */
#define XHCI_ERSTBA_PTR_MASK         (0xffffffffffffffc0)

/* Event Ring Dequeue Pointer */

#define XHCI_ERDP_DESI_SHIFT         (0)                   /* Bits 0-2: Dequeue ERST Segment Index */
#define XHCI_ERDP_DESI_MASK          (0x7 << XHCI_ERDP_DESI_SHIFT)
#define XHCI_ERDP_EHB                (1 << 3)              /* Bit 3: Event Handler Busy */
#define XHCI_ERDP_PTR_SHIFT          (4)                   /* Bit 4-64: Event Ring Dequeue Pointer */
#define XHCI_ERDP_PTR_MASK           (0xfffffffffffffff << XHCI_ERDP_PTR_SHIFT)

/* TRB definitions **********************************************************/

/* TRB common fields */

#define XHCI_TRB_D2_C                (1 << 0)              /* Bit 0: Cycle bit */
#define XHCI_TRB_D2_TC               (1 << 1)              /* Bit 1: Toggle Cycle */
#define XHCI_TRB_D2_CH               (1 << 4)              /* Bit 4: Chain bit */
#define XHCI_TRB_D2_TYPE_SHIFT       (10)                  /* Bits 10-15: TRB Type */
#define XHCI_TRB_D2_TYPE_MASK        (0x3f << XHCI_TRB_D2_TYPE_SHIFT)
#define XHCI_TRB_D2_TYPE_GET(x)      (((x) & XHCI_TRB_D2_TYPE_MASK) >> XHCI_TRB_D2_TYPE_SHIFT)
#define XHCI_TRB_D2_TYPE_SET(x)      (((x) << XHCI_TRB_D2_TYPE_SHIFT) & XHCI_TRB_D2_TYPE_MASK)

/* TRB common fields for events */

#define XHCI_TRB_D1_CC_SHIFT         (24)                  /* Bits 24-41: Completion Code */
#define XHCI_TRB_D1_CC_MASK          (0xff << XHCI_TRB_D1_CC_SHIFT)
#define XHCI_TRB_D1_CC_GET(x)        (((x) & XHCI_TRB_D1_CC_MASK) >> XHCI_TRB_D1_CC_SHIFT)
#define XHCI_TRB_D1_CC_SET(x)        (((x) << XHCI_TRB_D1_CC_SHIFT) & XHCI_TRB_D1_CC_MASK)
#  define XHCI_TRB_CC_INVAL          (0)                   /* Invalid */
#  define XHCI_TRB_CC_SUCCESS        (1)                   /* Success */
#  define XHCI_TRB_CC_DATA_BUF       (2)                   /* Data Buffer Error */
#  define XHCI_TRB_CC_BABBLE         (3)                   /* Babble Detected Error */
#  define XHCI_TRB_CC_USBT           (4)                   /* USB Transaction Error */
#  define XHCI_TRB_CC_TRB            (5)                   /* TRB Error */
#  define XHCI_TRB_CC_STALL          (6)                   /* Stall Error */
#  define XHCI_TRB_CC_RES            (7)                   /* Resource Error */
#  define XHCI_TRB_CC_BW             (8)                   /* Bandwidth Error */
#  define XHCI_TRB_CC_NOSLOT         (9)                   /* No Slots Available Error */
#  define XHCI_TRB_CC_INVALSTR       (10)                  /* Invalid Stream Type Error */
#  define XHCI_TRB_CC_SLOTNE         (11)                  /* Slot Not Enabled Error */
#  define XHCI_TRB_CC_EPNE           (12)                  /* Endpoint Not Enabled Error */
#  define XHCI_TRB_CC_SHORT_PKT      (13)                  /* Short Packet */
#  define XHCI_TRB_CC_RUR            (14)                  /* Ring Underrun */
#  define XHCI_TRB_CC_ROV            (15)                  /* Ring Overrun */
#  define XHCI_TRB_CC_VFFULL         (16)                  /* VF Event Ring Full Error */
#  define XHCI_TRB_CC_PARAM          (17)                  /* Parameter Error */
#  define XHCI_TRB_CC_BWOR           (18)                  /* Bandwidth Overrun Error */
#  define XHCI_TRB_CC_CTXST          (19)                  /* Context State Error */
#  define XHCI_TRB_CC_NOPING         (20)                  /* No Ping Response Error */
#  define XHCI_TRB_CC_ERFULL         (21)                  /* Event Ring Full Error */
#  define XHCI_TRB_CC_INCOMP         (22)                  /* Incompatible Device Error */
#  define XHCI_TRB_CC_MISSRV         (23)                  /* Missed Service Error */
#  define XHCI_TRB_CC_CRSTOPPED      (24)                  /* Command Ring Stopped */
#  define XHCI_TRB_CC_CMDABORT       (25)                  /* Command Aborted  */
#  define XHCI_TRB_CC_STOPPED        (26)                  /* Stopped */
#  define XHCI_TRB_CC_STOPPED_LEN    (27)                  /* Stopped - Length Invalid */
#  define XHCI_TRB_CC_STOPPED_SHORT  (28)                  /* Stopped - Short Packet */
#  define XHCI_TRB_CC_MELTLE         (29)                  /* Max Exit Latency Too Large Error */

/* Reference:
 *   - 6.4 Transfer Request Block (TRB) section
 */

/* TRB common fields for commands and events */

#define XHCI_TRB_D2_SLOTID_SHIFT     (24)                  /* Bits 24-31: Slot ID */
#define XHCI_TRB_D2_SLOTID_MASK      (0xff << XHCI_TRB_D2_SLOTID_SHIFT)
#define XHCI_TRB_D2_SLOTID_GET(x)    (((x) & XHCI_TRB_D2_SLOTID_MASK) >> XHCI_TRB_D2_SLOTID_SHIFT)
#define XHCI_TRB_D2_SLOTID_SET(x)    (((x) << XHCI_TRB_D2_SLOTID_SHIFT) & XHCI_TRB_D2_SLOTID_MASK)

#define XHCI_TRB_D2_EP_SHIFT         (16)                  /* Bits 16-20: Endpoint ID */
#define XHCI_TRB_D2_EP_MASK          (0x1f << XHCI_TRB_D2_EP_SHIFT)
#define XHCI_TRB_D2_EP_GET(x)        (((x) & XHCI_TRB_D2_EP_MASK) >> XHCI_TRB_D2_EP_SHIFT)
#define XHCI_TRB_D2_EP_SET(x)        (((x) << XHCI_TRB_D2_EP_SHIFT) & XHCI_TRB_D2_EP_MASK)

/* Normal TRB */

#define XHCI_TRB_D1_TXLEN_SHIFT      (0)                   /* Bits 9-16: TRB Transfer Length */
#define XHCI_TRB_D1_TXLEN_MASK       (0x1ffff << XHCI_TRB_D1_TXLEN_SHIFT)
#define XHCI_TRB_D1_TXLEN_SET(x)     (((x) << XHCI_TRB_D1_TXLEN_SHIFT) & XHCI_TRB_D1_TXLEN_MASK)
#define XHCI_TRB_D1_TXLEN_GET(x)     (((x) & XHCI_TRB_D1_TXLEN_MASK) >> XHCI_TRB_D1_TXLEN_SHIFT)
#define XHCI_TRB_D1_TDSIZE_SHIFT     (17)                  /* Bits 17-21: TD Size */
#define XHCI_TRB_D1_TDSIZE_MASK      (0x1f << XHCI_TRB_D1_TDSIZE_SHIFT)
#define XHCI_TRB_D1_TDSIZE_SET(x)    (((x) << XHCI_TRB_D1_TDSIZE_SHIFT) & XHCI_TRB_D1_TDSIZE_MASK)
#define XHCI_TRB_D1_IRQ_SHIFT        (22)                  /* Bits 22-31: Interrupter Target */
#define XHCI_TRB_D1_IRQ_MASK         (0x3ff << XHCI_TRB_D1_IRQ_SHIFT)
#define XHCI_TRB_D1_IRQ_SET(x)       (((x) << XHCI_TRB_D1_IRQ_SHIFT) & XHCI_TRB_D1_IRQ_MASK)

#define XHCI_TRB_D2_ISP              (1 << 2)              /* Bit 2: Interrupt-on Short Packet */
#define XHCI_TRB_D2_NS               (1 << 3)              /* Bit 3: No Snoop */
#define XHCI_TRB_D2_CH               (1 << 4)              /* Bit 4: Chain bit */
#define XHCI_TRB_D2_IOC              (1 << 5)              /* Bit 5: Interrupt On Completion */
#define XHCI_TRB_D2_IDT              (1 << 6)              /* Bit 6: Immediate Data */
                                                           /* Bits 7-8: Reserved */
#define XHCI_TRB_D2_BEI              (1 << 9)              /* Bit 9: Block Event Interrupt */

/* Setup Stage TRB */

#define XHCI_TRB_D2_TRT_SHIFT        (16)                  /* Bits 16-17: Transfer Type */
#define XHCI_TRB_D2_TRT_MASK         (0x3 << XHCI_TRB_D2_TRT_SHIFT)
#define XHCI_TRB_D2_TRT_SET(x)       (((x) << XHCI_TRB_D2_TRT_SHIFT) & XHCI_TRB_D2_TRT_MASK)
#  define XHCI_TRB_D2_TRT_NODATA     (0)                   /* 0: No Data Stage */
                                                           /* 1: reserved */
#  define XHCI_TRB_D2_TRT_OUTDATA    (2)                   /* 2: OUT Data Stage */
#  define XHCI_TRB_D2_TRT_INDATA     (3)                   /* 3: IN Data Stage */

/* Data Stage TRB */

#define XHCI_TRB_D2_DIR              (1 << 16)             /* Bit 16: Direction */

/* Isoch TRB */

#define XHCI_TRB_D2_SIA              (1 << 31)             /* Bit 31: Start Isoch ASAP */

/* Address Device Command TRB */

#define XHCI_TRB_D2_BSR              (1 << 9)              /* Bit 9: Block Set Address Request */

/* Configure Endpoint Command TRB */

#define XHCI_TRB_D2_DC               (1 << 9)              /* Bit 9: Deconfigure */

/* Stop Endpoint Command TRB */

#define XHCI_TRB_D2_SP               (1 << 23)              /* Bit 23: Suspend */

/* Command Ring and Transfer Ring Allowed */

#define   XHCI_TRB_TYPE_RESERVED       (0x00)                /* Reserved */
#define   XHCI_TRB_TYPE_NORMAL         (0x01)                /* Normal */
#define   XHCI_TRB_TYPE_SETUP_STAGE    (0x02)                /* Setup Stage */
#define   XHCI_TRB_TYPE_DATA_STAGE     (0x03)                /* Data Stage */
#define   XHCI_TRB_TYPE_STAT_STAGE     (0x04)                /* Status Stage */
#define   XHCI_TRB_TYPE_ISOCH          (0x05)                /* Isoch */
#define   XHCI_TRB_TYPE_LINK           (0x06)                /* Link */
#define   XHCI_TRB_TYPE_EV_DATA        (0x07)                /* Event Data */
#define   XHCI_TRB_TYPE_NOOP           (0x08)                /* No-Op */
#define   XHCI_TRB_TYPE_EN_SLOT        (0x09)                /* Enable Slot Command */
#define   XHCI_TRB_TYPE_DIS_SLOT       (0x0a)                /* Disable Slot Command */
#define   XHCI_TRB_TYPE_ADDR_DEV       (0x0b)                /* Address Device Command */
#define   XHCI_TRB_TYPE_CFG_EP         (0x0c)                /* Configure Endpoint Command */
#define   XHCI_TRB_TYPE_EVAL_CTX       (0x0d)                /* Evaluate Context Command */
#define   XHCI_TRB_TYPE_RST_EP         (0x0e)                /* Reset Endpoint Command */
#define   XHCI_TRB_TYPE_STOP_EP        (0x0f)                /* Stop Endpoint Command */
#define   XHCI_TRB_TYPE_SET_TR_DEQ     (0x10)                /* Set TR Dequeue Pointer Command */
#define   XHCI_TRB_TYPE_RST_DEV        (0x11)                /* Reset Device Command */
#define   XHCI_TRB_TYPE_FORCE_EV       (0x12)                /* Force Event Command */
#define   XHCI_TRB_TYPE_NEG_BW         (0x13)                /* Negotiate Bandwidth Command */
#define   XHCI_TRB_TYPE_SET_LAT_TOL    (0x14)                /* Set Latency Tolerance Value Command */
#define   XHCI_TRB_TYPE_GET_PORT_BW    (0x15)                /* Get Port Bandwidth Command */
#define   XHCI_TRB_TYPE_FORCE_HDR      (0x16)                /* Force Header Command */
#define   XHCI_TRB_TYPE_NOOP_CMD       (0x17)                /* No Op Command */
#define   XHCI_TRB_TYPE_GET_EXTPROP    (0x18)                /* Get Extended Property Command */
#define   XHCI_TRB_TYPE_SET_EXTPROP    (0x19)                /* Set Extended Property Command */

/* Event Ring Allowed */

#define   XHCI_TRB_EVT_TRANSFER        (0x20)                /* Transfer Event */
#define   XHCI_TRB_EVT_CMD_COMP        (0x21)                /* Command Completion Event */
#define   XHCI_TRB_EVT_PSTAT_CHANGE    (0x22)                /* Port Status Change Event */
#define   XHCI_TRB_EVT_BW_REQUEST      (0x23)                /* Bandwidth Request Event */
#define   XHCI_TRB_EVT_DOORBELL        (0x24)                /* Doorbell Event */
#define   XHCI_TRB_EVT_HOST_CTRL       (0x25)                /* Host Controller Event */
#define   XHCI_TRB_EVT_DEV_NOTIFY      (0x26)                /* Device Notification Event */
#define   XHCI_TRB_EVT_MFINDEX_WRP     (0x27)                /* MFINDEX Wrap Event */

/* Slot Context */

#define XHCI_ST_CTX0_RTSTR_SHIFT     (0)                   /* Bits 0:19: Route String */
#define XHCI_ST_CTX0_RTSTR_MASK      (0xfffff << XHCI_ST_CTX0_RTSTR_SHIFT)
#define XHCI_ST_CTX0_SPEED_SHIFT     (20)                  /* Bits 20:23: Speed */
#define XHCI_ST_CTX0_SPEED_MASK      (0xf << XHCI_ST_CTX0_SPEED_SHIFT)
#define XHCI_ST_CTX0_MTT             (1 << 25)             /* Bit 25: Multi-TT */
                                                           /* Bit 24: Reserved */
#define XHCI_ST_CTX0_HUB             (1 << 26)             /* Bit 26: Hub */
#define XHCI_ST_CTX0_CTXENT_SHIFT    (27)                  /* Bit 27-31: Context Entries */
#define XHCI_ST_CTX0_CTXENT_MASK     (0x1f << XHCI_ST_CTX0_CTXENT_SHIFT)
#define XHCI_ST_CTX0_CTXENT_SET(x)   (((x) << XHCI_ST_CTX0_CTXENT_SHIFT) & XHCI_ST_CTX0_CTXENT_MASK)
#define XHCI_ST_CTX1_RHPN_SHIFT      (16)                  /* Bit 16-23: Root Hub Port Number */
#define XHCI_ST_CTX1_RHPN_MASK       (0xff << XHCI_ST_CTX1_RHPN_SHIFT)
#define XHCI_ST_CTX1_RHPN_SET(x)     (((x) << XHCI_ST_CTX1_RHPN_SHIFT) & XHCI_ST_CTX1_RHPN_MASK)
#define XHCI_ST_CTX1_PORTS_SHIFT     (24)                  /* Bit 24-31: Number of Ports */
#define XHCI_ST_CTX1_PORTS_MASK      (0xff << XHCI_ST_CTX1_PORTS_SHIFT)
#define XHCI_ST_CTX1_PORTS_SET(x)    (((x) << XHCI_ST_CTX1_PORTS_SHIFT) & XHCI_ST_CTX1_PORTS_MASK)

#define XHCI_ST_CTX3_ADDR_SHIFT      (0)                  /* Bit 0-7: USB Device Address */
#define XHCI_ST_CTX3_ADDR_MASK       (0xff << XHCI_ST_CTX3_ADDR_SHIFT)
#define XHCI_ST_CTX3_ADDR_SET(x)     (((x) << XHCI_ST_CTX3_ADDR_SHIFT) & XHCI_ST_CTX3_ADDR_MASK)
#define XHCI_ST_CTX3_ADDR_GET(x)     (((x) & XHCI_ST_CTX3_ADDR_MASK) >> XHCI_ST_CTX3_ADDR_SHIFT )

/* Input Control Context */

#define XHCI_IN_CTX0_D(x)            (1 << (x))            /* Drop Context flags */
#define XHCI_IN_CTX1_A(x)            (1 << (x))            /* Add Context flags */
#  define XHCI_SLOT_FLAG             (0)
#  define XHCI_EP0_FLAG              (1)
#  define XHCI_EP_FLAG(ep)           (ep)                  /* IMPORTANT: to be used with Device Context Index (DCI) */

/* Endpoint Context */

#define XHCI_EP_CTX0_EPSTATE_SHIFT   (0)                   /* Bits 0-2: Endpoint State */
#define XHCI_EP_CTX0_EPSTATE_MASK    (7 << XHCI_EP_CTX0_EPSTATE_SHIFT)
#define XHCI_EP_CTX0_EPSTATE_GET(x)  ((x) & XHCI_EP_CTX0_EPSTATE_MASK) >> XHCI_EP_CTX0_EPSTATE_SHIFT)
                                                           /* Bits 3-7: Reserved */
#define XHCI_EP_CTX0_MULT_SHIFT      (8)                   /* Bits 8-9: Mult */
#define XHCI_EP_CTX0_MULT_MASK       (3 << XHCI_EP_CTX0_MULT_SHIFT)
#define XHCI_EP_CTX0_MULT(x)         (((x) << XHCI_EP_CTX0_MULT_SHIFT) & XHCI_EP_CTX0_MULT_MASK)

#define XHCI_EP_CTX0_MAXPSTR_SHIFT   (10)                  /* Bits 10-14: Max Primary Streams (MaxPStreams) */
#define XHCI_EP_CTX0_MAXPSTR_MASK    (0x1f << XHCI_EP_CTX0_MAXPSTR_SHIFT)
#define XHCI_EP_CTX0_MAXPSTR(x)      (((x) << XHCI_EP_CTX0_MAXPSTR_SHIFT) & XHCI_EP_CTX0_MAXPSTR_MASK)

#define XHCI_EP_CTX0_LSA             (1 << 15)             /* Bit 15: Linear Stream Array */
#define XHCI_EP_CTX0_INTERVAL_SHIFT  (16)                  /* Bits 16-23: Interval */
#define XHCI_EP_CTX0_INTERVAL_MASK   (0xff << XHCI_EP_CTX0_INTERVAL_SHIFT)
#define XHCI_EP_CTX0_INTERVAL(x)     (((x) << XHCI_EP_CTX0_INTERVAL_SHIFT) & XHCI_EP_CTX0_INTERVAL_MASK)
#define XHCI_EP_CTX0_MESITHI_SHIFT   (24)                  /* Bits 24-31: Max Endpoint Service Time Interval Payload High */

#define XHCI_EP_CTX1_CERR_SHIFT      (1)                   /* Bits 1-2: Error Count */
#define XHCI_EP_CTX1_CERR_MASK       (3 << XHCI_EP_CTX1_CERR_SHIFT)
#define XHCI_EP_CTX1_CERR(x)         (((x) << XHCI_EP_CTX1_CERR_SHIFT) & XHCI_EP_CTX1_CERR_MASK)

#define XHCI_EP_CTX1_EPTYPE_SHIFT    (3)                   /* Bits 3-5: Endpoint Type */
#define XHCI_EP_CTX1_EPTYPE_MASK     (7 << XHCI_EP_CTX1_EPTYPE_SHIFT)
#define XHCI_EP_CTX1_EPTYPE(x)       (((x) << XHCI_EP_CTX1_EPTYPE_SHIFT) & XHCI_EP_CTX1_EPTYPE_MASK)
#  define XHCI_EPTYPE_INVAL          (0)                   /* Invalid */
#  define XHCI_EPTYPE_ISO_OUT        (1)                   /* Isoch Out */
#  define XHCI_EPTYPE_BULK_OUT       (2)                   /* Bulk Out */
#  define XHCI_EPTYPE_INTR_OUT       (3)                   /* Interrupt Out */
#  define XHCI_EPTYPE_CTRL           (4)                   /* Control Bidirectional */
#  define XHCI_EPTYPE_ISO_IN         (5)                   /* Isoch In */
#  define XHCI_EPTYPE_BULK_IN        (6)                   /* Bulk In */
#  define XHCI_EPTYPE_INTR_IN        (7)                   /* Interrupt In */
#define XHCI_EP_CTX1_HID             (1 << 7)              /* Bit 7: Host Initiate Disable */
#define XHCI_EP_CTX1_MAXBRST_SHIFT   (8)                   /* Bits 8-15: Max Burst Size */
#define XHCI_EP_CTX1_MAXBRST_MASK    (0xff << XHCI_EP_CTX1_MAXBRST_SHIFT)
#define XHCI_EP_CTX1_MAXBRST(x)      (((x) << XHCI_EP_CTX1_MAXBRST_SHIFT) & XHCI_EP_CTX1_MAXBRST_MASK)
#define XHCI_EP_CTX1_MAXPKT_SHIFT    (16)                  /* Bits 16-31: Max Packet Size */
#define XHCI_EP_CTX1_MAXPKT_MASK     (0xffff << XHCI_EP_CTX1_MAXPKT_SHIFT)
#define XHCI_EP_CTX1_MAXPKT(x)       (((x) << XHCI_EP_CTX1_MAXPKT_SHIFT) & XHCI_EP_CTX1_MAXPKT_MASK)

#define XHCI_EP_CTX2_DCS             (1 << 0)              /* Bit 1: Dequeue Cycle State */
                                                           /* Bits 1-3: Reserved */
#define XHCI_EP_CTX2_TRDP_SHIFT      (4)                   /* Bits 4-63: TR Dequeue Pointer */

#define XHCI_EP_CTX3_AVGTRB_SHIFT    (0)                   /* Bits 0-15: Average TRB Length */
#define XHCI_EP_CTX3_MESITLO_SHIFT   (16)                  /* Bits 16-31: Max Endpoint Service Time Interval Payload Low */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Common representation for TRB */

begin_packed_struct struct xhci_trb_s
{
  uint64_t d0;                  /* Parameter */
  uint32_t d1;                  /* Status */
  uint32_t d2;                  /* Control */
} end_packed_struct;

/* Event ring register */

begin_packed_struct struct xhci_event_ring_s
{
  uint64_t base;
  uint32_t size;
  uint32_t res;
} end_packed_struct;

/* Input Control Context.
 *
 * Reference:
 *   - 6.2.5.1: Input Control Context
 */

begin_packed_struct struct xhci_input_ctx_s
{
  uint32_t ctx[8];
} end_packed_struct;

/* Slot Context */

begin_packed_struct struct xhci_slot_ctx_s
{
  uint32_t ctx[8];
} end_packed_struct;

/* EP Context */

begin_packed_struct struct xhci_ep_ctx_s
{
  uint32_t ctx0;
  uint32_t ctx1;
  uint64_t ctx2;                /* Need 64 here for pointers! */
  uint32_t ctx3;
  uint32_t ctx4;
  uint32_t ctx5;
  uint32_t ctx6;
} end_packed_struct;

/* Device Context. */

begin_packed_struct struct xhci_dev_ctx_s
{
  struct xhci_slot_ctx_s  slot;                   /* Slot Context */
  struct xhci_ep_ctx_s    ep[XHCI_MAX_ENDPOINTS]; /* EP Context */
} end_packed_struct;

/* Device Input Context.
 *
 * Reference:
 *   - Figure 6-5: Input Context
 */

begin_packed_struct struct xhci_input_dev_ctx_s
{
  struct xhci_input_ctx_s input;                  /* Input Control Context */
  struct xhci_slot_ctx_s  slot;                   /* Slot Context */
  struct xhci_ep_ctx_s    ep[XHCI_MAX_ENDPOINTS]; /* EP Context */
} end_packed_struct;

#endif /* #define __DRIVERS_USBHOST_USBHOST_XHCI_H */
