/****************************************************************************
 * include/nuttx/usb/ehci.h
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

#ifndef __INCLUDE_NUTTX_USB_EHCI_H
#define __INCLUDE_NUTTX_USB_EHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General definitions ******************************************************/

/* Endpoint speed values as used in endpoint characteristics field.
 * NOTE:  These values are *NOT* the same as the SPEED definitions in usb.h.
 */

#define EHCI_FULL_SPEED               (0) /* Full-Speed (12Mbs) */
#define EHCI_LOW_SPEED                (1) /* Low-Speed (1.5Mbs) */
#define EHCI_HIGH_SPEED               (2) /* High-Speed (480 Mb/s) */

#define EHCI_DIR_IN                   (1) /* Direction IN: Peripheral to host */
#define EHCI_DIR_OUT                  (0) /* Direction OUT: Host to peripheral */

/* PCI Configuration Space Register Offsets *********************************/

/* Paragraph 2.1 */

/* 0x0009-0x000b: Class Code */

#define EHCI_PCI_CLASSC_OFFSET         0x0009

/* 0x0010-0x0013: Base Address to Memory-mapped Host Controller Register
 * Space
 */

#define EHCI_PCIUSBBASE_OFFSET         0x0010

/* 0x0060: Serial Bus Release Number */

#define EHCI_PCI_SBRN_OFFSET           0x0060

/* 0x0061: Frame Length Adjustment Register */

#define EHCI_PCI_FLADJ_OFFSET          0x0061

/* 0x0062-0x0063: Port wake capabilities register (OPTIONAL) */

#define EHCI_PCI_PORTWAKECAP_OFFSET    0x0062

/* EECP+0x0000: USB Legacy Support EHCI Extended Capability Register */

#define EHCI_PCI_USBLEGSUP_OFFSET      0x0000

/* EECP+0x0000: USB Legacy Support Control and Status Register */

#define EHCI_PCI_USBLEGCTLSTS_OFFSET   0x0004

/* Host Controller Capability Register Offsets ******************************/

/* Paragraph 2.2 */

#define EHCI_CAPLENGTH_OFFSET          0x0000     /* Core Capability Register Length */
                                                  /* 0x0001 Reserved  */
#define EHCI_HCIVERSION_OFFSET         0x0002     /* Core Interface Version Number */
#define EHCI_HCSPARAMS_OFFSET          0x0004     /* Core Structural Parameters */
#define EHCI_HCCPARAMS_OFFSET          0x0008     /* Core Capability Parameters */
#define EHCI_HCSP_PORTROUTE_OFFSET     0x000c     /* Core Companion Port Route Description */

/* Host Controller Operational Register Offsets *****************************/

/* Paragraph 2.3 */

#define EHCI_USBCMD_OFFSET             0x0000     /* USB Command */
#define EHCI_USBSTS_OFFSET             0x0004     /* USB Status */
#define EHCI_USBINTR_OFFSET            0x0008     /* USB Interrupt Enable */
#define EHCI_FRINDEX_OFFSET            0x000c     /* USB Frame Index */
#define EHCI_CTRLDSSEGMENT_OFFSET      0x0010     /* 4G Segment Selector */
#define EHCI_PERIODICLISTBASE_OFFSET   0x0014     /* Frame List Base Address */
#define EHCI_ASYNCLISTADDR_OFFSET      0x0018     /* Next Asynchronous List Address */
                                                  /* 0x001c-0x003f: Reserved */
#define EHCI_CONFIGFLAG_OFFSET         0x0040     /* Configured Flag Register */

/* Port Status/Control, Port 1-n */

#define EHCI_PORTSC_OFFSET(n)          (0x0044 + ((n-1) << 2))
#define EHCI_PORTSC1_OFFSET            0x0044     /* Port Status/Control, Port 1 */
#define EHCI_PORTSC2_OFFSET            0x0048     /* Port Status/Control, Port 2 */
#define EHCI_PORTSC3_OFFSET            0x004c     /* Port Status/Control, Port 3 */
#define EHCI_PORTSC4_OFFSET            0x0050     /* Port Status/Control, Port 4 */
#define EHCI_PORTSC5_OFFSET            0x0054     /* Port Status/Control, Port 5 */
#define EHCI_PORTSC6_OFFSET            0x0058     /* Port Status/Control, Port 6 */
#define EHCI_PORTSC7_OFFSET            0x005c     /* Port Status/Control, Port 7 */
#define EHCI_PORTSC8_OFFSET            0x0060     /* Port Status/Control, Port 8 */
#define EHCI_PORTSC9_OFFSET            0x0064     /* Port Status/Control, Port 9 */
#define EHCI_PORTSC10_OFFSET           0x0068     /* Port Status/Control, Port 10 */
#define EHCI_PORTSC11_OFFSET           0x006c     /* Port Status/Control, Port 11 */
#define EHCI_PORTSC12_OFFSET           0x0070     /* Port Status/Control, Port 12 */
#define EHCI_PORTSC13_OFFSET           0x0074     /* Port Status/Control, Port 13 */
#define EHCI_PORTSC14_OFFSET           0x0078     /* Port Status/Control, Port 14 */
#define EHCI_PORTSC15_OFFSET           0x007c     /* Port Status/Control, Port 15 */

/* Debug Register Offsets ***************************************************/

/* Paragraph C.3 */

#define EHCI_DEBUG_PCS_OFFSET          0x0000     /* Debug Port Control/Status Register */
#define EHCI_DEBUG_USBPIDS_OFFSET      0x0004     /* Debug USB PIDs Register */
#define EHCI_DEBUG_DATA0_OFFSET        0x0008     /* Debug Data Buffer 0 Register [31:0]  */
#define EHCI_DEBUG_DATA1_OFFSET        0x000c     /* Debug Data Buffer 1 Register [63:32]  */
#define EHCI_DEBUG_DEVADDR_OFFSET      0x0010     /* Debug Device Address Register */

/* PCI Configuration Space Register Bit Definitions *************************/

/* 0x0009-0x000b: Class Code.  Paragraph 2.1.2 */

#define EHCI_PCI_PI_SHIFT              (0)        /* Bits 0-7: Programming Interface */
#define EHCI_PCI_PI_MASK               (0xff << EHCI_PCI_PI_SHIFT)
#  define EHCI_PCI_PI                  (0x20 << EHCI_PCI_PI_SHIFT)
#define EHCI_PCI_SCC_SHIFT             (8)        /* Bits 8-15: Sub-Class Code */
#define EHCI_PCI_SCC_MASK              (0xff << EHCI_PCI_SCC_SHIFT)
#    define EHCI_PCI_SCC               (0x03 << EHCI_PCI_SCC_SHIFT)
#define EHCI_PCI_BASEC_SHIFT           (16)       /* Base Class Code */
#define EHCI_PCI_BASEC_MASK            (0xff << EHCI_PCI_BASEC_SHIFT)
#    define EHCI_PCI_BASEC             (0x0c << EHCI_PCI_BASEC_SHIFT)
#define EHCI_PCI_CLASSC                0x000c0320 /* Default value (little endian) */

/* 0x0010-0x0013: Base Address to Memory-mapped Host Controller Register
 * Space.  Paragraph 2.1.3
 */

                                                  /* Bit 0: Reserved */

#define EHCI_PCIUSBBASE_TYPE_SHIFT     (0)        /* Bits 1-2: Type */
#define EHCI_PCIUSBBASE_TYPE_MASK      (3 << EHCI_PCIUSBBASE_TYPE_SHIFT)
#  define EHCI_PCIUSBBASE_TYPE_32BIT   (3 << EHCI_PCIUSBBASE_TYPE_SHIFT) /* 32-bit addressing */
#  define EHCI_PCIUSBBASE_TYPE_64BIT   (3 << EHCI_PCIUSBBASE_TYPE_SHIFT) /* 64-bit addressing */

                                                  /* Bits 3-7: Reserved */
#define EHCI_PCIUSBBASE_BASE_SHIFT     (8)        /* Bits 8-31: Base address */
#define EHCI_PCIUSBBASE_BASE_MASK      (0xffffff00)

/* 0x0060: Serial Bus Release Number.  Paragraph 2.1.4 */

#define EHCI_PCI_SBRN_MASK             0xff       /* Bits 0-7: Serial Bus Release Number */

/* 0x0061: Frame Length Adjustment Register.  Paragraph 2.1.5 */

#define EHCI_PCI_FLADJ_SHIFT           (0)        /* Bit 0-5: Frame Length Timing Value */
#define EHCI_PCI_FLADJ_MASK            (0x3f >> EHCI_PCI_FLADJ_SHIFT)
                                                  /* Bits 6-7: Reserved */

/* 0x0062-0x0063: Port wake capabilities register (OPTIONAL).
 *  Paragraph 2.1.6
 */

#define EHCI_PCI_PORTWAKECAP_MASK      (0xffff)

/* EECP+0x0000: USB Legacy Support EHCI Extended Capability Register.
 *  Paragraph 2.1.7
 */

#define EHCI_PCI_USBLEGSUP_CAPID_SHIFT (0)        /* Bits 0-7 Capability ID */
#define EHCI_PCI_USBLEGSUP_CAPID_MASK  (0xff << EHCI_PCI_USBLEGSUP_CAPID_SHIFT)
#define EHCI_PCI_USBLEGSUP_NEECP_SHIFT (8)        /* Bits 8-15: Next EHCI Extended Capability Pointer */
#define EHCI_PCI_USBLEGSUP_NEECP_MASK  (0xff << EHCI_PCI_USBLEGSUP_NEECP_SHIFT)
#define EHCI_PCI_USBLEGSUP_BOWN        (1 << 16)  /* Bit 16: HC BIOS Owned Semaphore */
                                                  /* Bits 17-23: Reserved */
#define EHCI_PCI_USBLEGSUP_OSOWN       (1 << 24)  /* Bit 24: HC OS Owned Semaphore */
                                                  /* Bits 25-31: Reserved */

/* EECP+0x0000: USB Legacy Support Control and Status Register.
 * Paragraph 2.1.8
 */

#define EHCI_PCI_USBLEGCTLSTS_USBCMPEN (1 << 0)   /* Bit 0:  USB SMI Enable */
#define EHCI_PCI_USBLEGCTLSTS_USBERREN (1 << 1)   /* Bit 1:  SMI on USB Error Enable */
#define EHCI_PCI_USBLEGCTLSTS_PCHEN    (1 << 2)   /* Bit 2:  SMI on Port Change Enable */
#define EHCI_PCI_USBLEGCTLSTS_FLREN    (1 << 3)   /* Bit 3:  SMI on Frame List Rollover Enable */
#define EHCI_PCI_USBLEGCTLSTS_HSEEN    (1 << 4)   /* Bit 4:  SMI on Host System Error Enable */
#define EHCI_PCI_USBLEGCTLSTS_AAEN     (1 << 5)   /* Bit 5:  SMI on Async Advance Enable */
                                                  /* Bits 6-12: Reserved */
#define EHCI_PCI_USBLEGCTLSTS_OOEN     (1 << 13)  /* Bit 13: SMI on OS Ownership Enable */
#define EHCI_PCI_USBLEGCTLSTS_PCEN     (1 << 14)  /* Bit 14: SMI on PCI Command Enable */
#define EHCI_PCI_USBLEGCTLSTS_BAREN    (1 << 15)  /* Bit 15: SMI on BAR Enable */
#define EHCI_PCI_USBLEGCTLSTS_USBCMP   (1 << 16)  /* Bit 16: SMI on USB Complete */
#define EHCI_PCI_USBLEGCTLSTS_USBERR   (1 << 17)  /* Bit 17: SMI on USB Error */
#define EHCI_PCI_USBLEGCTLSTS_PCH      (1 << 18)  /* Bit 18: SMI on Port Change Detect */
#define EHCI_PCI_USBLEGCTLSTS_FLR      (1 << 19)  /* Bit 19: SMI on Frame List Rollover */
#define EHCI_PCI_USBLEGCTLSTS_HSE      (1 << 20)  /* Bit 20: SMI on Host System Error */
#define EHCI_PCI_USBLEGCTLSTS_AA       (1 << 21)  /* Bit 21: SMI on Async Advance */
                                                  /* Bits 22-28: Reserved */
#define EHCI_PCI_USBLEGCTLSTS_OO       (1 << 29)  /* Bit 29: SMI on OS Ownership Change */
#define EHCI_PCI_USBLEGCTLSTS_PC       (1 << 30)  /* Bit 30: SMI on PCI Command */
#define EHCI_PCI_USBLEGCTLSTS_BAR      (1 << 31)  /* Bit 31: SMI on BAR */

/* Host Controller Capability Register Bit Definitions **********************/

/* Paragraph 2.2 */

/* Core Capability Register Length. Paragraph 2.2.1. 8-bit length. */

/* Core Interface Version Number. Paragraph 2.2.2.  Two byte BCD encoding */

/* Core Structural Parameters. Paragraph 2.2.3 */

#define EHCI_HCSPARAMS_NPORTS_SHIFT    (0)        /* Bit 0-3: Number of physical downstream ports */
#define EHCI_HCSPARAMS_NPORTS_MASK     (15 << EHCI_HCSPARAMS_NPORTS_SHIFT)
#define EHCI_HCSPARAMS_PPC             (1 << 4)   /* Bit 4: Port Power Control */
                                                  /* Bits 5-6: Reserved */
#define EHCI_HCSPARAMS_PRR             (1 << 7)   /* Bit 7: Port Routing Rules */
#define EHCI_HCSPARAMS_NPCC_SHIFT      (8)        /* Bit 8-11: Number of Ports per Companion Controller */
#define EHCI_HCSPARAMS_NPCC_MASK       (15 << EHCI_HCSPARAMS_NPCC_SHIFT)
#define EHCI_HCSPARAMS_NCC_SHIFT       (12)       /* Bit 12-15: Number of Companion Controllers */
#define EHCI_HCSPARAMS_NCC_MASK        (15 << EHCI_HCSPARAMS_NCC_SHIFT)
#define EHCI_HCSPARAMS_PIND            (1 << 16)  /* Bit 16: Port Indicators */
                                                  /* Bits 17-19: Reserved */
#define EHCI_HCSPARAMS_DBGPORT_SHIFT   (20)       /* Bit 20-23: Debug Port Number */
#define EHCI_HCSPARAMS_DBGPORT_MASK    (15 << EHCI_HCSPARAMS_DBGPORT_SHIFT)
                                                  /* Bits 24-31: Reserved */

/* Core Capability Parameters. Paragraph 2.2.4 */

#define EHCI_HCCPARAMS_64BIT           (1 << 0)   /* Bit 0: 64-bit Addressing Capability */
#define EHCI_HCCPARAMS_PFLF            (1 << 1)   /* Bit 1: Programmable Frame List Flag */
#define EHCI_HCCPARAMS_ASPC            (1 << 2)   /* Bit 2: Asynchronous Schedule Park Capability */
                                                  /* Bit 3: Reserved */
#define EHCI_HCCPARAMS_IST_SHIFT       (4)        /* Bits 4-7: Isochronous Scheduling Threshold */
#define EHCI_HCCPARAMS_IST_MASK        (15 << EHCI_HCCPARAMS_IST_SHIFT)
#define EHCI_HCCPARAMS_EECP_SHIFT      (8)        /* Bits 8-15: EHCI Extended Capabilities Pointer */
#define EHCI_HCCPARAMS_EECP_MASK       (0xff << EHCI_HCCPARAMS_EECP_SHIFT)
                                                  /* Bits 16-31: Reserved */

/* Core Companion Port Route Description.
 * Paragraph 2.2.5. 15 x 4-bit array (60 bits)
 */

/* Host Controller Operational Register Bit Definitions *********************/

/* Paragraph 2.3 */

/* USB Command. Paragraph 2.3.1 */

#define EHCI_USBCMD_RUN                (1 << 0)   /* Bit 0: Run/Stop */
#define EHCI_USBCMD_HCRESET            (1 << 1)   /* Bit 1: Host Controller Reset */
#define EHCI_USBCMD_FLSIZE_SHIFT       (2)        /* Bits 2-3: Frame List Size */
#define EHCI_USBCMD_FLSIZE_MASK        (3 << EHCI_USBCMD_FLSIZE_SHIFT)
#  define EHCI_USBCMD_FLSIZE_1024      (0 << EHCI_USBCMD_FLSIZE_SHIFT) /* 1024 elements (4096 bytes) */
#  define EHCI_USBCMD_FLSIZE_512       (1 << EHCI_USBCMD_FLSIZE_SHIFT) /* 512 elements (2048 bytes) */
#  define EHCI_USBCMD_FLSIZE_256       (2 << EHCI_USBCMD_FLSIZE_SHIFT) /* 256 elements (1024 bytes) */

#define EHCI_USBCMD_PSEN               (1 << 4)   /* Bit 4: Periodic Schedule Enable */
#define EHCI_USBCMD_ASEN               (1 << 5)   /* Bit 5: Asynchronous Schedule Enable */
#define EHCI_USBCMD_IAADB              (1 << 6)   /* Bit 6: Interrupt on Async Advance Doorbell */
#define EHCI_USBCMD_LRESET             (1 << 7)   /* Bit 7: Light Host Controller Reset */
#define EHCI_USBCMD_PARKCNT_SHIFT      (8)        /* Bits 8-9: Asynchronous Schedule Park Mode Count */
#define EHCI_USBCMD_PARKCNT_MASK       (3 << EHCI_USBCMD_PARKCNT_SHIFT)
                                                  /* Bit 10: Reserved */
#define EHCI_USBCMD_PARK               (1 << 11)  /* Bit 11: Asynchronous Schedule Park Mode Enable */
                                                  /* Bits 12-15: Reserved */
#define EHCI_USBCMD_ITHRE_SHIFT        (16)       /* Bits 16-23: Interrupt Threshold Control */
#define EHCI_USBCMD_ITHRE_MASK         (0xff << EHCI_USBCMD_ITHRE_SHIFT)
#  define EHCI_USBCMD_ITHRE_1MF        (0x01 << EHCI_USBCMD_ITHRE_SHIFT) /* 1 micro-frame */
#  define EHCI_USBCMD_ITHRE_2MF        (0x02 << EHCI_USBCMD_ITHRE_SHIFT) /* 2 micro-frames */
#  define EHCI_USBCMD_ITHRE_4MF        (0x04 << EHCI_USBCMD_ITHRE_SHIFT) /* 4 micro-frames */
#  define EHCI_USBCMD_ITHRE_8MF        (0x08 << EHCI_USBCMD_ITHRE_SHIFT) /* 8 micro-frames (default, 1 ms) */
#  define EHCI_USBCMD_ITHRE_16MF       (0x10 << EHCI_USBCMD_ITHRE_SHIFT) /* 16 micro-frames (2 ms) */
#  define EHCI_USBCMD_ITHRE_32MF       (0x20 << EHCI_USBCMD_ITHRE_SHIFT) /* 32 micro-frames (4 ms) */
#  define EHCI_USBCMD_ITHRE_64MF       (0x40 << EHCI_USBCMD_ITHRE_SHIFT) /* 64 micro-frames (8 ms) */

                                                  /* Bits 24-31: Reserved */

/* USB Status. Paragraph 2.3.2 */

/* USB Interrupt Enable. Paragraph 2.3.3 */

#define EHCI_INT_USBINT                (1 << 0)   /* Bit 0:  USB Interrupt */
#define EHCI_INT_USBERRINT             (1 << 1)   /* Bit 1:  USB Error Interrupt */
#define EHCI_INT_PORTSC                (1 << 2)   /* Bit 2:  Port Change Detect */
#define EHCI_INT_FLROLL                (1 << 3)   /* Bit 3:  Frame List Rollover */
#define EHCI_INT_SYSERROR              (1 << 4)   /* Bit 4:  Host System Error */
#define EHCI_INT_AAINT                 (1 << 5)   /* Bit 5:  Interrupt on Async Advance */
#define EHCI_INT_ALLINTS               (0x3f)     /* Bits 0-5:  All interrupts */
                                                  /* Bits 6-11: Reserved */
#define EHCI_USBSTS_HALTED             (1 << 12)  /* Bit 12: HC Halted */
#define EHCI_USBSTS_RECLAM             (1 << 13)  /* Bit 13: Reclamation */
#define EHCI_USBSTS_PSS                (1 << 14)  /* Bit 14: Periodic Schedule Status */
#define EHCI_USBSTS_ASS                (1 << 15)  /* Bit 15: Asynchronous Schedule Status */
                                                  /* Bits 16-31: Reserved */

/* USB Frame Index. Paragraph 2.3.4 */

#define EHCI_FRINDEX_MASK              (0x1fff)   /* Bits 0-13: Frame index */
                                                  /* Bits 14-31: Reserved */

/* 4G Segment Selector.
 * Paragraph 2.3.5,  Bits[64:32] of data structure addresses
 */

/* Frame List Base Address. Paragraph 2.3.6 */

                                                    /* Bits 0-11: Reserved */
#define EHCI_PERIODICLISTBASE_MASK     (0xfffff000) /* Bits 12-31: Base Address (Low) */

/* Next Asynchronous List Address. Paragraph 2.3.7 */

                                                    /* Bits 0-4: Reserved */
#define EHCI_ASYNCLISTADDR_MASK        (0xffffffe0) /* Bits 5-31: Link Pointer Low (LPL) */

/* Configured Flag Register. Paragraph 2.3.8 */

#define EHCI_CONFIGFLAG              (1 << 0)   /* Bit 0: Configure Flag */
                                                /* Bits 1-31: Reserved */

/* Port Status/Control, Port 1-n. Paragraph 2.3.9 */

#define EHCI_PORTSC_CCS                (1 << 0)   /* Bit 0: Current Connect Status */
#define EHCI_PORTSC_CSC                (1 << 1)   /* Bit 1: Connect Status Change */
#define EHCI_PORTSC_PE                 (1 << 2)   /* Bit 2: Port Enable */
#define EHCI_PORTSC_PEC                (1 << 3)   /* Bit 3: Port Enable/Disable Change */
#define EHCI_PORTSC_OCA                (1 << 4)   /* Bit 4: Over-current Active */
#define EHCI_PORTSC_OCC                (1 << 5)   /* Bit 5: Over-current Change */
#define EHCI_PORTSC_RESUME             (1 << 6)   /* Bit 6: Force Port Resume */
#define EHCI_PORTSC_SUSPEND            (1 << 7)   /* Bit 7: Suspend */
#define EHCI_PORTSC_RESET              (1 << 8)   /* Bit 8: Port Reset */
                                                  /* Bit 9: Reserved */
#define EHCI_PORTSC_LSTATUS_SHIFT      (10)       /* Bits 10-11: Line Status */
#define EHCI_PORTSC_LSTATUS_MASK       (3 << EHCI_PORTSC_LSTATUS_SHIFT)
#  define EHCI_PORTSC_LSTATUS_SE0      (0 << EHCI_PORTSC_LSTATUS_SHIFT) /* SE0 Not Low-speed device, perform EHCI reset */
#  define EHCI_PORTSC_LSTATUS_KSTATE   (1 << EHCI_PORTSC_LSTATUS_SHIFT) /* K-state Low-speed device, release ownership of port */
#  define EHCI_PORTSC_LSTATUS_JSTATE   (2 << EHCI_PORTSC_LSTATUS_SHIFT) /* J-state Not Low-speed device, perform EHCI reset */

#define EHCI_PORTSC_PP                 (1 << 12)  /* Bit 12: Port Power */
#define EHCI_PORTSC_OWNER              (1 << 13)  /* Bit 13: Port Owner */
#define EHCI_PORTSC_PIC_SHIFT          (14)       /* Bits 14-15: Port Indicator Control */
#define EHCI_PORTSC_PIC_MASK           (3 << EHCI_PORTSC_PIC_SHIFT)
#  define EHCI_PORTSC_PIC_OFF          (0 << EHCI_PORTSC_PIC_SHIFT) /* Port indicators are off */
#  define EHCI_PORTSC_PIC_AMBER        (1 << EHCI_PORTSC_PIC_SHIFT) /* Amber */
#  define EHCI_PORTSC_PIC_GREEN        (2 << EHCI_PORTSC_PIC_SHIFT) /* Green */

#define EHCI_PORTSC_PTC_SHIFT          (16)       /* Bits 16-19: Port Test Control */
#define EHCI_PORTSC_PTC_MASK           (15 << EHCI_PORTSC_PTC_SHIFT)
#  define EHCI_PORTSC_PTC_DISABLED     (0 << EHCI_PORTSC_PTC_SHIFT) /* Test mode not enabled */
#  define EHCI_PORTSC_PTC_JSTATE       (1 << EHCI_PORTSC_PTC_SHIFT) /* Test J_STATE */
#  define EHCI_PORTSC_PTC_KSTATE       (2 << EHCI_PORTSC_PTC_SHIFT) /* Test K_STATE */
#  define EHCI_PORTSC_PTC_SE0NAK       (3 << EHCI_PORTSC_PTC_SHIFT) /* Test SE0_NAK */
#  define EHCI_PORTSC_PTC_PACKET       (4 << EHCI_PORTSC_PTC_SHIFT) /* Test Packet */
#  define EHCI_PORTSC_PTC_ENABLE       (5 << EHCI_PORTSC_PTC_SHIFT) /* Test FORCE_ENABLE */

#define EHCI_PORTSC_WKCCNTE            (1 << 20)  /* Bit 20: Wake on Connect Enable */
#define EHCI_PORTSC_WKDSCNNTE          (1 << 21)  /* Bit 21: Wake on Disconnect Enable */
#define EHCI_PORTSC_WKOCE              (1 << 22)  /* Bit 22: Wake on Over-current Enable */
                                                  /* Bits 23-31: Reserved */

#define EHCI_PORTSC_ALLINTS            (EHCI_PORTSC_CSC | EHCI_PORTSC_PEC | \
                                        EHCI_PORTSC_OCC | EHCI_PORTSC_RESUME)

/* Debug Register Bit Definitions *******************************************/

/* Debug Port Control/Status Register.  Paragraph C.3.1 */

#define EHCI_DEBUG_PCS_LENGTH_SHIFT    (0)        /* Bits 0-3: Data Length */
#define EHCI_DEBUG_PCS_LENGTH_MASK     (15 << EHCI_DEBUG_PCS_LENGTH_SHIFT)
#define EHCI_DEBUG_PCS_WRITE           (1 << 4)   /* Bit 6:  Write/Read# */
#define EHCI_DEBUG_PCS_GO              (1 << 5)   /* Bit 5:  Go */
#define EHCI_DEBUG_PCS_ERROR           (1 << 6)   /* Bit 6:  Error/Good# */
#define EHCI_DEBUG_PCS_EXCEPTION_SHIFT (17)       /* Bits 7-9: Exception */
#define EHCI_DEBUG_PCS_EXCEPTION_MASK  (7 << EHCI_DEBUG_PCS_EXCEPTION_SHIFT)
#define EHCI_DEBUG_PCS_INUSE           (1 << 10)  /* Bit 10: In Use */
                                                  /* Bits 11-15: Reserved */
#define EHCI_DEBUG_PCS_DONE            (1 << 16)  /* Bit 16: Done */
                                                  /* Bits 17-27: Reserved */
#define EHCI_DEBUG_PCS_ENABLED         (1 << 28)  /* Bit 28: Enabled */
                                                  /* Bit 29: Reserved */
#define EHCI_DEBUG_PCS_OWNER           (1 << 30)  /* Bit 30: Owner */
                                                  /* Bit 31: Reserved */

/* Debug USB PIDs Register.  Paragraph C.3.2 */

#define EHCI_DEBUG_USBPIDS_TKPID_SHIFT (0)        /* Bits 0-7: Token PID */
#define EHCI_DEBUG_USBPIDS_TKPID_MASK  (0xff << EHCI_DEBUG_USBPIDS_TKPID_SHIFT)
#define EHCI_DEBUG_USBPIDS_SPID_SHIFT  (8)        /* Bits 8-15: Sent PID */
#define EHCI_DEBUG_USBPIDS_SPID_MASK   (0xff << EHCI_DEBUG_USBPIDS_SPID_SHIFT)
#define EHCI_DEBUG_USBPIDS_RPID_SHIFT  (16)       /* Bits 16-23: Received PID */
#define EHCI_DEBUG_USBPIDS_RPID_MASK   (0xff << EHCI_DEBUG_USBPIDS_RPID_SHIFT)
                                                  /* Bits 24-31: Reserved */

/* Debug Data Buffer 0/1 Register [64:0].
 * Paragreph C.3.3.  64 bits of data.
 */

/* Debug Device Address Register.  Paragraph C.3.4 */

#define EHCI_DEBUG_DEVADDR_ENDPT_SHIFT (0)        /* Bit 0-3: USB Endpoint */
#define EHCI_DEBUG_DEVADDR_ENDPT_MASK  (15 << EHCI_DEBUG_DEVADDR_ENDPT_SHIFT)
                                                  /* Bits 4-7: Reserved */
#define EHCI_DEBUG_DEVADDR_ADDR_SHIFT  (8)        /* Bits 8-14: USB Address */
#define EHCI_DEBUG_DEVADDR_ADDR_MASK   (0x7f << EHCI_DEBUG_DEVADDR_ADDR_SHIFT)
                                                  /* Bits 15-31: Reserved */

/* Data Structures **********************************************************/

/* Paragraph 3 */

/* Periodic Frame List. Paragraph 3.1 */

#define PFL_T                          (1 << 0)   /* Bit 0: Terminate, Link pointer invalid */
#define PFL_TYP_SHIFT                  (1)        /* Bits 1-2: Type */
#define PFL_TYP_MASK                   (3 << PFL_TYP_SHIFT)
#  define PFL_TYP_ITD                  (0 << PFL_TYP_SHIFT) /* Isochronous Transfer Descriptor */
#  define PFL_TYP_QH                   (1 << PFL_TYP_SHIFT) /* Queue Head */
#  define PFL_TYP_SITD                 (2 << PFL_TYP_SHIFT) /* Split Transaction Isochronous Transfer Descriptor */
#  define PFL_TYP_FSTN                 (3 << PFL_TYP_SHIFT) /* Frame Span Traversal Node */

                                                    /* Bits 3-4: zero */
#define PFL_MASK                       (0xffffffe0) /* Bits 5-31:  Frame List Link Pointer */

/* Aysnchronous List Queue Head Pointer.
 * Paragraph 3.2. Circular list of queue heads
 */

/* Isochronous (High-Speed) Transfer Descriptor (iTD). Paragraph 3.3 */

/* iTD Next Link Pointer. Paragraph 3.3.1 */

#define ITD_NLP_T                      (1 << 0)   /* Bit 0: Terminate, Link pointer invalid */
#define ITD_NLP_TYP_SHIFT              (1)        /* Bits 1-2: Type */
#define ITD_NLP_TYP_MASK               (3 << ITD_NLP_TYP_SHIFT)
#  define ITD_NLP_TYP_ITD              (0 << ITD_NLP_TYP_SHIFT) /* Isochronous Transfer Descriptor */
#  define ITD_NLP_TYP_QH               (1 << ITD_NLP_TYP_SHIFT) /* Queue Head */
#  define ITD_NLP_TYP_SITD             (2 << ITD_NLP_TYP_SHIFT) /* Split Transaction Isochronous Transfer Descriptor */
#  define ITD_NLP_TYP_FSTN             (3 << ITD_NLP_TYP_SHIFT) /* Frame Span Traversal Node */

                                                    /* Bits 3-4: zero */
#define ITD_NLP_MASK                   (0xffffffe0) /* Bits 5-31:  Frame List Link Pointer */

/* iTD Transaction Status and Control List. Paragraph 3.3.2 */

#define ITD_TRAN_XOFFS_SHIFT           (0)        /* Bits 0-11: Transaction X offset */
#define ITD_TRAN_XOFFS_MASK            (0xfff << ITD_TRAN_XOFFS_SHIFT)
#define ITD_TRAN_PG_SHIFT              (12)       /* Bits 12-14: Page select */
#define ITD_TRAN_PG_MASK               (7 << ITD_TRAN_PG_SHIFT)
#define ITD_TRAN_IOC                   (1 << 15)  /* Bit 15:  Interrupt On Comp */
#define ITD_TRAN_LENGTH_SHIFT          (16)       /* Bits 16-27:  Transaction length */
#define ITD_TRAN_LENGTH_MASK           (0xfff << ITD_TRAN_LENGTH_SHIFT)
#define ITD_TRAN_STATUS_SHIFT          (28)       /* Bits 28-31:  Transaction status */
#define ITD_TRAN_STATUS_MASK           (15 << ITD_TRAN_STATUS_SHIFT)
#  define ITD_TRAN_STATUS_XACTERR      (1 << 28)  /* Bit 28: Transaction error */
#  define ITD_TRAN_STATUS_BABBLE       (1 << 29)  /* Bit 29: Babble Detected */
#  define ITD_TRAN_STATUS_DBERROR      (1 << 30)  /* Bit 30: Data Buffer Error */
#  define ITD_TRAN_STATUS_ACTIVE       (1 << 31)  /* Bit 28: Transaction error */

/* iTD Buffer Page Pointer List. Paragraph 3.3.4 */

/* iTD Buffer Pointer Page 0. Table 3-4 */

#define ITD_BUFPTR0_DEVADDR_SHIFT      (0)        /* Bits 0-6: Device Address */
#define ITD_BUFPTR0_DEVADDR_MASK       (0x7f << ITD_BUFPTR0_DEVADDR_SHIFT)
                                                  /* Bit 7: Reserved */
#define ITD_BUFPTR0_ENDPT_SHIFT        (8)        /* Bits 8-11: Endpoint Number */
#define ITD_BUFPTR0_ENDPT_MASK         (15 << ITD_BUFPTR0_ENDPT_SHIFT)

/* iTD Buffer Pointer Page 1. Table 3-5 */

#define ITD_BUFPTR1_MAXPKT_SHIFT       (0)        /* Bits 0-10: Maximum Packet Size */
#define ITD_BUFPTR1_MAXPKT_MASK        (0x7ff << ITD_BUFPTR1_MAXPKT_SHIFT)
#define ITD_BUFPTR1_DIRIN              (1 << 11)  /* Bit 11: Direction 1=IN */
#define ITD_BUFPTR1_DIROUT             (0)        /* Bit 11: Direction 0=OUT */

/* iTD Buffer Pointer Page 2. Table 3-6 */

#define ITD_BUFPTR2_MULTI_SHIFT        (0)        /* Bits 0-1: Multi */
#define ITD_BUFPTR2_MULTI_MASK         (3 << ITD_BUFPTR2_MULTI_SHIFT)
#  define ITD_BUFPTR2_MULTI_1          (1 << ITD_BUFPTR2_MULTI_SHIFT) /* One transaction per micro-frame */
#  define ITD_BUFPTR2_MULTI_2          (2 << ITD_BUFPTR2_MULTI_SHIFT) /* Two transactions per micro-frame */
#  define ITD_BUFPTR2_MULTI_3          (3 << ITD_BUFPTR2_MULTI_SHIFT) /* Three transactions per micro-frame */

                                                  /* Bits 2-11: Reserved */

/* iTD Buffer Pointer Page 3-6. Table 3-7 */

                                                  /* Bits 0-11: Reserved */

/* iTD Buffer Pointer All Pages */

#define ITD_BUFPTR_MASK                (0xfffff000) /* Bits 12-31: Buffer Pointer */

/* Split Transaction Isochronous Transfer Descriptor (siTD).
 * Paragraph 3.4
 */

/* siTD Next Link Pointer. Paragraph 3.4.1 */

#define SITD_NLP_T                     (1 << 0)   /* Bit 0: Terminate, Link pointer invalid */
#define SITD_NLP_TYP_SHIFT             (1)        /* Bits 1-2: Type */
#define SITD_NLP_TYP_MASK              (3 << SITD_NLP_TYP_SHIFT)
#  define SITD_NLP_TYP_ITD             (0 << SITD_NLP_TYP_SHIFT) /* Isochronous Transfer Descriptor */
#  define SITD_NLP_TYP_QH              (1 << SITD_NLP_TYP_SHIFT) /* Queue Head */
#  define SITD_NLP_TYP_SITD            (2 << SITD_NLP_TYP_SHIFT) /* Split Transaction Isochronous Transfer Descriptor */
#  define SITD_NLP_TYP_FSTN            (3 << SITD_NLP_TYP_SHIFT) /* Frame Span Traversal Node */

                                                    /* Bits 3-4: zero */
#define SITD_NLP_MASK                  (0xffffffe0) /* Bits 5-31: Frame List Link Pointer */

/* siTD Endpoint Capabilities/Characteristics.
 * Paragraph 3.4.2
 */

/* Endpoint and Transaction Translator Characteristics. Table 3-9 */

#define SITD_EPCHAR_DEVADDR_SHIFT      (0)        /* Bitx 0-6: Device Address */
#define SITD_EPCHAR_DEVADDR_MASK       (0x7f << SITD_EPCHAR_DEVADDR_SHIFT)
                                                  /* Bits 7: Reserved */
#define SITD_EPCHAR_ENDPT_SHIFT        (8)        /* Bitx 8-11: Endpoint Number */
#define SITD_EPCHAR_ENDPT_MASK         (15 << SITD_EPCHAR_ENDPT_SHIFT)
                                                  /* Bits 12-15: Reserved */
#define SITD_EPCHAR_HUBADDR_SHIFT      (16)       /* Bitx 16-22: Hub Address */
#define SITD_EPCHAR_HUBADDR_MASK       (0x7f << SITD_EPCHAR_HUBADDR_SHIFT)
                                                  /* Bit 23: Reserved */
#define SITD_EPCHAR_DIRIN              (1 << 31)  /* Bit 31: Direction 1=IN */
#define SITD_EPCHAR_DIROUT             (0)        /* Bit 31: Direction 0=OUT */

/* Micro-frame Schedule Control. Table 3-10 */

#define SITD_FMSCHED_SSMASK_SHIFT      (0)        /* Bitx 0-7: Split Start Mask (µFrame S-mask) */
#define SITD_FMSCHED_SSMASK_MASK       (0xff << SITD_FMSCHED_SSMASK_SHIFT)
#  define SITD_FMSCHED_SSMASK(n)       ((n) << SITD_FMSCHED_SSMASK_SHIFT)
#define SITD_FMSCHED_SCMASK_SHIFT      (8)        /* Bitx 8-15: Split Completion Mask (µFrame C-Mask) */
#define SITD_FMSCHED_SCMASK_MASK       (0xff << SITD_FMSCHED_SCMASK_SHIFT)
#  define SITD_FMSCHED_SCMASK(n)       ((n) << SITD_FMSCHED_SCMASK_SHIFT)
                                                  /* Bits 16-31: Reserved */

/* siTD Transfer State. Paragraph 3.4.3 */

#define SITD_XFRSTATE_STATUS_SHIFT     (0)        /* Bits 0-7: Status */
#define SITD_XFRSTATE_STATUS_MASK      (0xff << SITD_XFRSTATE_STATUS_SHIFT)
#define SITD_XFRSTATE_CPROGMASK_SHIFT  (8)        /* Bits 8-15: µFrame Complete-split Progress Mask  */
#define SITD_XFRSTATE_CPROGMASK_MASK   (0xff << SITD_XFRSTATE_CPROGMASK_SHIFT)
#define SITD_XFRSTATE_NBYTES_SHIFT     (16)       /* Bits 16-25: Total Bytes To Transfer */
#define SITD_XFRSTATE_NBYTES_MASK      (0x3ff << SITD_XFRSTATE_NBYTES_SHIFT)
                                                  /* Bits 26-29: Reserved */
#define SITD_XFRSTATE_P                (1 << 30)  /* Bit 30: Page Select */
#define SITD_XFRSTATE_IOC              (1 << 31)  /* Bit 31: Interrupt On Complete */

/* siTD Buffer Pointer List.
 * Paragraph 3.4.4
 */

/* Page 0 */

#define SITD_BUFPTR0_OFFSET_SHIFT      (0)        /* Bits 0-11: Current Offset */
#define SITD_BUFPTR0_OFFSET_MASK       (0xff << SITD_BUFPTR0_OFFSET_SHIFT)

/* Page 1 */

#define SITD_BUFPTR1_TCOUNT_SHIFT      (0)        /* Bits 0-2: Transaction count */
#define SITD_BUFPTR1_TCOUNT_MASK       (7 << SITD_BUFPTR1_TCOUNT_SHIFT)
#define SITD_BUFPTR1_TP_SHIFT          (33)       /* Bits 3-4: Transaction position */
#define SITD_BUFPTR1_TP_MASK           (3 << SITD_BUFPTR1_TP_SHIFT)
#  define SITD_BUFPTR1_TP_ENTIRE       (0 << SITD_BUFPTR1_TP_SHIFT) /* Entire full-speed transaction data payload. */
#  define SITD_BUFPTR1_TP_BEGIN        (1 << SITD_BUFPTR1_TP_SHIFT) /* This is the first data payload */
#  define SITD_BUFPTR1_TP_MID          (2 << SITD_BUFPTR1_TP_SHIFT) /* This the middle payload */
#  define SITD_BUFPTR1_TP_END          (3 << SITD_BUFPTR1_TP_SHIFT) /* This is the last payload */

                                                  /* Bits 5-11: Reserved */

/* All pages */

#define SITD_BUFPTR_MASK               (0xfffff000) /* Bits 12-31: Buffer Pointer List */

/* Queue Element Transfer Descriptor (qTD).
 * Paragraph 3.5
 */

/* Next qTD Pointer.
 * Paragraph 3.5.1
 */

#define QTD_NQP_T                      (1 << 0)   /* Bit 0: Terminate */
                                                  /* Bits 1-4: Reserved */
#define QTD_NQP_NTEP_SHIFT             (5)        /* Bits 5-31: Next Transfer Element Pointer */
#define QTD_NQP_NTEP_MASK              (0xffffffe0)

/* Alternate Next qTD Pointer.
 * Paragraph 3.5.2
 */

#define QTD_AQP_T                      (1 << 0)   /* Bit 0: Terminate */
                                                  /* Bits 1-4: Reserved */
#define QTD_AQP_NTEP_SHIFT             (5)        /* Bits 5-31: Next Transfer Element Pointer */
#define QTD_AQP_NTEP_MASK              (0xffffffe0)

/* qTD Token.
 * Paragraph 3.5.3
 */

#define QTD_TOKEN_STATUS_SHIFT         (0)        /* Bits 0-7: Status */
#define QTD_TOKEN_STATUS_MASK          (0xff << QTD_TOKEN_STATUS_SHIFT)
#  define QTD_TOKEN_P                  (1 << 0)   /* Bit 0 Ping State  */
#  define QTD_TOKEN_ERR                (1 << 0)   /* Bit 0 Error */
#  define QTD_TOKEN_SPLITXSTATE        (1 << 1)   /* Bit 1 Split Transaction State */
#  define QTD_TOKEN_MMF                (1 << 2)   /* Bit 2 Missed Micro-Frame */
#  define QTD_TOKEN_XACTERR            (1 << 3)   /* Bit 3 Transaction Error */
#  define QTD_TOKEN_BABBLE             (1 << 4)   /* Bit 4 Babble Detected */
#  define QTD_TOKEN_DBERR              (1 << 5)   /* Bit 5 Data Buffer Error */
#  define QTD_TOKEN_HALTED             (1 << 6)   /* Bit 6 Halted */
#  define QTD_TOKEN_ACTIVE             (1 << 7)   /* Bit 7 Active */
#  define QTD_TOKEN_ERRORS             (0x78 << QTD_TOKEN_STATUS_SHIFT)
#define QTD_TOKEN_PID_SHIFT            (8)        /* Bits 8-9: PID Code */
#define QTD_TOKEN_PID_MASK             (3 << QTD_TOKEN_PID_SHIFT)
#  define QTD_TOKEN_PID_OUT            (0 << QTD_TOKEN_PID_SHIFT) /* OUT Token generates token (E1H) */
#  define QTD_TOKEN_PID_IN             (1 << QTD_TOKEN_PID_SHIFT) /* IN Token generates token (69H) */
#  define QTD_TOKEN_PID_SETUP          (2 << QTD_TOKEN_PID_SHIFT) /* SETUP Token generates token (2DH) */

#define QTD_TOKEN_CERR_SHIFT           (10)       /* Bits 10-11: Error Counter */
#define QTD_TOKEN_CERR_MASK            (3 << QTD_TOKEN_CERR_SHIFT)
#define QTD_TOKEN_CPAGE_SHIFT          (12)       /* Bits 12-14: Current Page */
#define QTD_TOKEN_CPAGE_MASK           (7 << QTD_TOKEN_CPAGE_SHIFT)
#define QTD_TOKEN_IOC                  (1 << 15)  /* Bit 15: Interrupt On Complete */
#define QTD_TOKEN_NBYTES_SHIFT         (16)       /* Bits 16-30: Total Bytes to Transfer */
#define QTD_TOKEN_NBYTES_MASK          (0x7fff << QTD_TOKEN_NBYTES_SHIFT)
#define QTD_TOKEN_TOGGLE_SHIFT         (31)       /* Bit 31: Data Toggle */
#define QTD_TOKEN_TOGGLE               (1 << 31)  /* Bit 31: Data Toggle */

/* qTD Buffer Page Pointer List.
 * Paragraph 3.5.4
 */

/* Page 0 */

#define QTD_BUFPTR0_OFFFSET_SHIFT      (0)        /* Bits 0-11: Current Offset */
#define QTD_BUFPTR0_OFFFSET_MASK       (0xfff << QTD_BUFPTR0_OFFFSET_SHIFT)

/* Other pages */

                                                  /* Bits 0-11: Reserved */

/* All pages */

#define QTD_BUFPTR_SHIFT               (12)       /* Bits 12-31: Buffer Pointer List */
#define QTD_BUFPTR_MASK                (0xfffff000)

/* Queue Head. Paragraph 3.6 */

/* Queue Head Horizontal Link Pointer.
 * Paragraph 3.6.1
 */

#define QH_HLP_T                       (1 << 0)   /* Bit 0: Terminate, QH HL pointer invalid */
#define QH_HLP_TYP_SHIFT               (1)        /* Bits 1-2: Type */
#define QH_HLP_TYP_MASK                (3 << QH_HLP_TYP_SHIFT)
#  define QH_HLP_TYP_ITD               (0 << QH_HLP_TYP_SHIFT) /* Isochronous Transfer Descriptor */
#  define QH_HLP_TYP_QH                (1 << QH_HLP_TYP_SHIFT) /* Queue Head */
#  define QH_HLP_TYP_SITD              (2 << QH_HLP_TYP_SHIFT) /* Split Transaction Isochronous Transfer Descriptor */
#  define QH_HLP_TYP_FSTN              (3 << QH_HLP_TYP_SHIFT) /* Frame Span Traversal Node */

                                                    /* Bits 3-4: Reserved */
#define QH_HLP_MASK                    (0xffffffe0) /* Bits 5-31: Queue Head Horizontal Link Pointer */

/* Endpoint Capabilities/Characteristics. Paragraph 3.6.2 */

/* Endpoint Characteristics: Queue Head DWord. Table 3-19 */

#define QH_EPCHAR_DEVADDR_SHIFT        (0)        /* Bitx 0-6: Device Address */
#define QH_EPCHAR_DEVADDR_MASK         (0x7f << QH_EPCHAR_DEVADDR_SHIFT)
#define QH_EPCHAR_I                    (1 << 7)   /* Bit 7: Inactivate on Next Transaction */
#define QH_EPCHAR_ENDPT_SHIFT          (8)        /* Bitx 8-11: Endpoint Number */
#define QH_EPCHAR_ENDPT_MASK           (15 << QH_EPCHAR_ENDPT_SHIFT)
#define QH_EPCHAR_EPS_SHIFT            (12)       /* Bitx 12-13: Endpoint Speed */
#define QH_EPCHAR_EPS_MASK             (3 << QH_EPCHAR_EPS_SHIFT)
#  define QH_EPCHAR_EPS_FULL           (0 << QH_EPCHAR_EPS_SHIFT) /* Full-Speed (12Mbs) */
#  define QH_EPCHAR_EPS_LOW            (1 << QH_EPCHAR_EPS_SHIFT) /* Low-Speed (1.5Mbs) */
#  define QH_EPCHAR_EPS_HIGH           (2 << QH_EPCHAR_EPS_SHIFT) /* High-Speed (480 Mb/s) */

#define QH_EPCHAR_DTC                  (1 << 14)  /* Bit 14: Data Toggle Control */
#define QH_EPCHAR_H                    (1 << 15)  /* Bit 15: Head of Reclamation List Flag */
#define QH_EPCHAR_MAXPKT_SHIFT         (16)       /* Bitx 16-26: Maximum Packet Length */
#define QH_EPCHAR_MAXPKT_MASK          (0x7ff << QH_EPCHAR_MAXPKT_SHIFT)
#define QH_EPCHAR_C                    (1 << 27)  /* Bit 27: Control Endpoint Flag */
#define QH_EPCHAR_RL_SHIFT             (28)       /* Bitx 28-31: Nak Count Reload */
#define QH_EPCHAR_RL_MASK              (15 << QH_EPCHAR_RL_SHIFT)

/* Endpoint Capabilities: Queue Head DWord 2. Table 3-20 */

#define QH_EPCAPS_SSMASK_SHIFT         (0)        /* Bitx 0-7: Interrupt Schedule Mask (µFrame S-mask) */
#define QH_EPCAPS_SSMASK_MASK          (0xff << QH_EPCAPS_SSMASK_SHIFT)
#  define QH_EPCAPS_SSMASK(n)          ((n) <<  QH_EPCAPS_SSMASK_SHIFT)
#define QH_EPCAPS_SCMASK_SHIFT         (8)        /* Bitx 8-15: Split Completion Mask (µFrame C-Mask) */
#define QH_EPCAPS_SCMASK_MASK          (0xff << QH_EPCAPS_SCMASK_SHIFT)
#  define QH_EPCAPS_SCMASK(n)          ((n) << QH_EPCAPS_SCMASK_SHIFT)
#define QH_EPCAPS_HUBADDR_SHIFT        (16)       /* Bitx 16-22: Hub Address */
#define QH_EPCAPS_HUBADDR_MASK         (0x7f << QH_EPCAPS_HUBADDR_SHIFT)
#  define QH_EPCAPS_HUBADDR(n)         ((n) << QH_EPCAPS_HUBADDR_SHIFT)
#define QH_EPCAPS_PORT_SHIFT           (23)       /* Bit 23-29: Port Number */
#define QH_EPCAPS_PORT_MASK            (0x7f << QH_EPCAPS_PORT_SHIFT)
#  define QH_EPCAPS_PORT(n)            ((n) << QH_EPCAPS_PORT_SHIFT)
#define QH_EPCAPS_MULT_SHIFT           (30)       /* Bit 30-31: High-Bandwidth Pipe Multiplier */
#define QH_EPCAPS_MULT_MASK            (3 << QH_EPCAPS_MULT_SHIFT)
#  define QH_EPCAPS_MULT(n)            ((n) << QH_EPCAPS_MULT_SHIFT)

/* Current qTD Link Pointer.  Table 3-21 */

#define QH_CQP_NTEP_SHIFT              (5)        /* Bits 5-31: Next Transfer Element Pointer */
#define QH_CQP_NTEP_MASK               (0xffffffe0)

/* Transfer Overlay.  Paragraph 3.6.3
 *
 * NOTES:
 * 1. Same as the field of the same name in struct ehci_qtd_s
 * 2. Similar to the field of the same name in struct ehci_qtd_s, but with
 *    some additional bitfields.
 */

/* Next qTD Pointer (NOTE 1) */

#define QH_NQP_T                       (1 << 0)   /* Bit 0: Terminate */
                                                  /* Bits 1-4: Reserved */
#define QH_NQP_NTEP_SHIFT              (5)        /* Bits 5-31: Next Transfer Element Pointer */
#define QH_NQP_NTEP_MASK               (0xffffffe0)

/* Alternate Next qTD Pointer.  Table 3.7 (NOTE 2) */

#define QH_AQP_T                       (1 << 0)   /* Bit 0: Terminate */
#define QH_AQP_NAKCNT                  (1)        /* Bits 1-4: Nak Counter */
#define QH_AQP_NTEP_SHIFT              (5)        /* Bits 5-31: Next Transfer Element Pointer */
#define QH_AQP_NTEP_MASK               (0xffffffe0)

/* qTD Token (NOTE 1) */

#define QH_TOKEN_STATUS_SHIFT          (0)        /* Bits 0-7: Status */
#define QH_TOKEN_STATUS_MASK           (0xff << QH_TOKEN_STATUS_SHIFT)
#  define QH_TOKEN_P                   (1 << 0)   /* Bit 0 Ping State  */
#  define QH_TOKEN_ERR                 (1 << 0)   /* Bit 0 Error */
#  define QH_TOKEN_SPLITXSTATE         (1 << 1)   /* Bit 1 Split Transaction State */
#  define QH_TOKEN_MMF                 (1 << 2)   /* Bit 2 Missed Micro-Frame */
#  define QH_TOKEN_XACTERR             (1 << 3)   /* Bit 3 Transaction Error */
#  define QH_TOKEN_BABBLE              (1 << 4)   /* Bit 4 Babble Detected */
#  define QH_TOKEN_DBERR               (1 << 5)   /* Bit 5 Data Buffer Error */
#  define QH_TOKEN_HALTED              (1 << 6)   /* Bit 6 Halted */
#  define QH_TOKEN_ACTIVE              (1 << 7)   /* Bit 7 Active */
#  define QH_TOKEN_ERRORS              (0x78 << QH_TOKEN_STATUS_SHIFT)
#define QH_TOKEN_PID_SHIFT             (8)        /* Bits 8-9: PID Code */
#define QH_TOKEN_PID_MASK              (3 << QH_TOKEN_PID_SHIFT)
#  define QH_TOKEN_PID_OUT             (0 << QH_TOKEN_PID_SHIFT) /* OUT Token generates token (E1H) */
#  define QH_TOKEN_PID_IN              (1 << QH_TOKEN_PID_SHIFT) /* IN Token generates token (69H) */
#  define QH_TOKEN_PID_SETUP           (2 << QH_TOKEN_PID_SHIFT) /* SETUP Token generates token (2DH) */

#define QH_TOKEN_CERR_SHIFT            (10)       /* Bits 10-11: Error Counter */
#define QH_TOKEN_CERR_MASK             (3 << QH_TOKEN_CERR_SHIFT)
#define QH_TOKEN_CPAGE_SHIFT           (12)       /* Bits 12-14: Current Page */
#define QH_TOKEN_CPAGE_MASK            (7 << QH_TOKEN_CPAGE_SHIFT)
#define QH_TOKEN_IOC                   (1 << 15)  /* Bit 15: Interrupt On Complete */
#define QH_TOKEN_NBYTES_SHIFT          (16)       /* Bits 16-30: Total Bytes to Transfer */
#define QH_TOKEN_NBYTES_MASK           (0x7fff << QH_TOKEN_NBYTES_SHIFT)
#define QH_TOKEN_TOGGLE_SHIFT          (31)       /* Bit 31: Data Toggle */
#define QH_TOKEN_TOGGLE                (1 << 31)  /* Bit 31: Data Toggle */

/* Buffer Page Pointer List (NOTE 2) */

/* Page 0 */

#define QH_BUFPTR0_OFFFSET_SHIFT      (0)        /* Bits 0-11: Current Offset */
#define QH_BUFPTR0_OFFFSET_MASK       (0xfff << QH_BUFPTR0_OFFFSET_SHIFT)

/* Page 1. Table 3.22 */

#define QH_BUFPTR1_CPROGMASK_SHIFT    (0)        /* Bits 0-7: Split-transaction Complete-split Progress */
#define QH_BUFPTR1_CPROGMASK_MASK     (0xff << QH_BUFPTR1_CPROGMASK_SHIFT)
                                                  /* Bits 8-11: Reserved */

/* Page 2. Table 3.22 */

#define QH_BUFPTR2_FRAMETAG_SHIFT     (0)        /* Bits 0-4: Split-transaction Frame Tag */
#define QH_BUFPTR2_FRAMETAG_MASK      (31 << QH_BUFPTR2_FRAMETAG_SHIFT)
#define QH_BUFPTR2_SBYTES_SHIFT       (5)        /* Bits 5-11: S-bytes */
#define QH_BUFPTR2_SBYTES_MASK        (0x7f << QH_BUFPTR2_SBYTES_SHIFT)

/* Other pages */

                                                  /* Bits 0-11: Reserved */

/* All pages */

#define QH_BUFPTR_SHIFT               (12)       /* Bits 12-31: Buffer Pointer List */
#define QH_BUFPTR_MASK                (0xfffff000)

/* Periodic Frame Span Traversal Node (STN). Paragrap 3.7 */

/* FSTN Normal Path Pointer. Paragraph 3.7.1 */

#define FSTN_NPP_T                     (1 << 0)   /* Bit 0: Terminate. 1=Link Pointer not valid */
#define FSTN_NPP_TYP_SHIFT             (1)        /* Bits 1-2: Type */
#define FSTN_NPP_TYP_MASK              (3 << FSTN_NPP_TYP_SHIFT)
#  define FSTN_NPP_TYP_ITD             (0 << FSTN_NPP_TYP_SHIFT) /* Isochronous Transfer Descriptor */
#  define FSTN_NPP_TYP_QH              (1 << FSTN_NPP_TYP_SHIFT) /* Queue Head */
#  define FSTN_NPP_TYP_SITD            (2 << FSTN_NPP_TYP_SHIFT) /* Split Transaction Isochronous Transfer Descriptor */
#  define FSTN_NPP_TYP_FSTN            (3 << FSTN_NPP_TYP_SHIFT) /* Frame Span Traversal Node */

                                                  /* Bits 3-4: Reserved */
#define FSTN_NPP_NPLP_SHIFT            (5)        /* Bits 5-31: Normal Path Link Pointer */
#define FSTN_NPP_NPLP_MASK             (0xffffffe0)

/* FSTN Back Path Link Pointer. Paragraph 3.7.2 */

#define FSTN_BPP_T                     (1 << 0)   /* Bit 0: Terminate. 1=Link Pointer not valid */
#define FSTN_BPP_TYP_SHIFT             (1)        /* Bits 1-2: Type */
#define FSTN_BPP_TYP_MASK              (3 << FSTN_BPP_TYP_SHIFT)
#  define FSTN_BPP_TYP_QH              (1 << FSTN_BPP_TYP_SHIFT) /* Queue Head */

                                                  /* Bits 3-4: Reserved */
#define FSTN_BPP_BPLP_SHIFT            (5)        /* Bits 5-31: Back Path Link Pointer */
#define FSTN_BPP_BPLP_MASK             (0xffffffe0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Registers ****************************************************************/

/* Since the operational registers are not known a compile time, representing
 * register blocks with structures is more convenient than using individual
 * register offsets.
 */

/* Host Controller Capability Registers.
 * This register block must be positioned at a well known address.
 */

struct ehci_hccr_s
{
  uint8_t  caplength;        /* 0x00: Capability Register Length */
  uint8_t  reserved;
  uint16_t hciversion;       /* 0x02: Interface Version Number */
  uint32_t hcsparams;        /* 0x04: Structural Parameters */
  uint32_t hccparams;        /* 0x08: Capability Parameters */
  uint8_t  hcspportrt[8];    /* 0x0c: Companion Port Route Description */
};

/* Host Controller Operational Registers.
 * This register block is positioned at an offset of 'caplength' from the
 * beginning of the Host Controller Capability Registers.
 */

struct ehci_hcor_s
{
  uint32_t usbcmd;           /* 0x00: USB Command */
  uint32_t usbsts;           /* 0x04: USB Status */
  uint32_t usbintr;          /* 0x08: USB Interrupt Enable */
  uint32_t frindex;          /* 0x0c: USB Frame Index */
  uint32_t ctrldssegment;    /* 0x10: 4G Segment Selector */
  uint32_t periodiclistbase; /* 0x14: Frame List Base Address */
  uint32_t asynclistaddr;    /* 0x18: Next Asynchronous List Address */
  uint32_t reserved[9];
  uint32_t configflag;       /* 0x40: Configured Flag Register */
  uint32_t portsc[15];       /* 0x44: Port Status/Control */
};

/* USB2 Debug Port Register Interface.
 *  This register block is normally found via the PCI capabalities.
 * In non-PCI implementions, you need apriori information about the
 * location of these registers.
 */

struct ehci_debug_s
{
  uint32_t psc;              /* 0x00: Debug Port Control/Status Register */
  uint32_t pids;             /* 0x04: Debug USB PIDs Register */
  uint32_t data[2];          /* 0x08: Debug Data buffer Registers */
  uint32_t addr;             /* 0x10: Device Address Register */
};

/* Data Structures **********************************************************/

/* Paragraph 3 */

/* Periodic Frame List.
 * Paragraph 3.1.  An array of pointers.
 */

/* Aysnchronous List Queue Head Pointer.
 * Paragraph 3.2. Circular list of queue heads
 */

/* Isochronous (High-Speed) Transfer Descriptor (iTD).
 * Paragraph 3.3.  Must be aligned to 32-byte boundaries.
 */

struct ehci_itd_s
{
  uint32_t nlp;                              /* 0x00-0x03: Next link pointer */
  uint32_t trans[8];                         /* 0x04-0x23: Transaction Status and Control List */
  uint32_t bpl[7];                           /* 0x24-0x3c: Buffer Page Pointer List */
};

#define SIZEOF_EHCI_ITD_S (64)               /* 16*sizeof(uint32_t) */

/* Split Transaction Isochronous Transfer Descriptor (siTD). Paragraph 3.4 */

struct ehci_sitd_s
{
  uint32_t nlp;                              /* 0x00-0x03: Next link pointer */
  uint32_t epchar;                           /* 0x04-0x07: Endpoint and Transaction Translator Characteristics */
  uint32_t fmsched;                          /* 0x08-0x0b: Micro-frame Schedule Control */
  uint32_t xfrstate;                         /* 0x0c-0x0f: Transfer Status and Control */
  uint32_t bpl[2];                           /* 0x10-0x17: Buffer Pointer List */
  uint32_t blp;                              /* 0x18-0x1b: Back link pointer */
};

#define SIZEOF_EHCI_SITD_S (28)              /* 7*sizeof(uint32_t) */

/* Queue Element Transfer Descriptor (qTD). Paragraph 3.5 */

/* 32-bit version.  See EHCI Appendix B for the 64-bit version. */

struct ehci_qtd_s
{
  uint32_t nqp;                              /* 0x00-0x03: Next qTD Pointer */
  uint32_t alt;                              /* 0x04-0x07: Alternate Next qTD Pointer */
  uint32_t token;                            /* 0x08-0x0b: qTD Token */
  uint32_t bpl[5];                           /* 0x0c-0x1c: Buffer Page Pointer List */
};

#define SIZEOF_EHCI_QTD_S (32)               /* 8*sizeof(uint32_t) */

/* Queue Head. Paragraph 3.6
 *
 * NOTE:
 * 1. Same as the field of the same name in struct ehci_qtd_s
 * 2. Similar to the field of the same name in struct ehci_qtd_s,
 *    but with some additional bitfields.
 */

struct ehci_overlay_s
{
  uint32_t nqp;                              /* 0x00-0x03: Next qTD Pointer (NOTE 1) */
  uint32_t alt;                              /* 0x04-0x07: Alternate Next qTD Pointer (NOTE 2) */
  uint32_t token;                            /* 0x08-0x0b: qTD Token (NOTE 1) */
  uint32_t bpl[5];                           /* 0x0c-0x1c: Buffer Page Pointer List (NOTE 2) */
};

#define SIZEOF_EHCI_OVERLAY (32)             /* 8*sizeof(uint32_t) */

struct ehci_qh_s
{
  uint32_t hlp;                              /* 0x00-0x03: Queue Head Horizontal Link Pointer */
  uint32_t epchar;                           /* 0x04-0x07: Endpoint Characteristics */
  uint32_t epcaps;                           /* 0x08-0x0b: Endpoint Capabilities */
  uint32_t cqp;                              /* 0x0c-0x0f: Current qTD Pointer */
  struct ehci_overlay_s overlay;             /* 0x10-0x2c: Transfer overlay */
};

#define SIZEOF_EHCI_OVERLAY (48)             /* 4*sizeof(uint32_t) + SIZEOF_EHCI_OVERLAY */

/* Periodic Frame Span Traversal Node (STN). Paragrap 3.7 */

struct ehci_fstn_s
{
  uint32_t npp;                              /* 0x00-0x03: Normal Path Pointer */
  uint32_t bpp;                              /* 0x04-0x07: Back Path Link Pointer */
};

#define SIZEOF_EHCI_FSTN_S (8)               /* 2*sizeof(uint32_t) */

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_USB_EHCI_H */
