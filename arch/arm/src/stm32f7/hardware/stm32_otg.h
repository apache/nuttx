/****************************************************************************************************
 * arch/arm/src/stm32f7/hardware/stm32_otg.h
 *
 *   Copyright (C) 2012, 2014-2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016 Omni Hoverboards Inc. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Paul Alexander Patience <paul-a.patience@polymtl.ca>
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32_OTG_H
#define __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32_OTG_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/
/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* General definitions */

#define OTG_EPTYPE_CTRL               (0) /* Control */
#define OTG_EPTYPE_ISOC               (1) /* Isochronous */
#define OTG_EPTYPE_BULK               (2) /* Bulk */
#define OTG_EPTYPE_INTR               (3) /* Interrupt */

#define OTG_PID_DATA0                 (0)
#define OTG_PID_DATA2                 (1)
#define OTG_PID_DATA1                 (2)
#define OTG_PID_MDATA                 (3) /* Non-control */
#define OTG_PID_SETUP                 (3) /* Control */

/* Register Offsets *********************************************************************************/
/* Core global control and status registers */

#define STM32_OTG_GOTGCTL_OFFSET      0x0000 /* Control and status register */
#define STM32_OTG_GOTGINT_OFFSET      0x0004 /* Interrupt register */
#define STM32_OTG_GAHBCFG_OFFSET      0x0008 /* AHB configuration register */
#define STM32_OTG_GUSBCFG_OFFSET      0x000c /* USB configuration register */
#define STM32_OTG_GRSTCTL_OFFSET      0x0010 /* Reset register */
#define STM32_OTG_GINTSTS_OFFSET      0x0014 /* Core interrupt register */
#define STM32_OTG_GINTMSK_OFFSET      0x0018 /* Interrupt mask register */
#define STM32_OTG_GRXSTSR_OFFSET      0x001c /* Receive status debug read/OTG status read register */
#define STM32_OTG_GRXSTSP_OFFSET      0x0020 /* Receive status debug read/OTG status pop register */
#define STM32_OTG_GRXFSIZ_OFFSET      0x0024 /* Receive FIFO size register */
#define STM32_OTG_HNPTXFSIZ_OFFSET    0x0028 /* Host non-periodic transmit FIFO size register */
#define STM32_OTG_DIEPTXF0_OFFSET     0x0028 /* Endpoint 0 Transmit FIFO size */
#define STM32_OTG_HNPTXSTS_OFFSET     0x002c /* Non-periodic transmit FIFO/queue status register */
#define STM32_OTG_GCCFG_OFFSET        0x0038 /* General core configuration register */
#define STM32_OTG_CID_OFFSET          0x003c /* Core ID register  */
#define STM32_OTG_HPTXFSIZ_OFFSET     0x0100 /* Host periodic transmit FIFO size register */

#define STM32_OTG_DIEPTXF_OFFSET(n)   (0x0104+(((n)-1) << 2))

/* Host-mode control and status registers */

#define STM32_OTG_HCFG_OFFSET         0x0400 /* Host configuration register */
#define STM32_OTG_HFIR_OFFSET         0x0404 /* Host frame interval register */
#define STM32_OTG_HFNUM_OFFSET        0x0408 /* Host frame number/frame time remaining register */
#define STM32_OTG_HPTXSTS_OFFSET      0x0410 /* Host periodic transmit FIFO/queue status register */
#define STM32_OTG_HAINT_OFFSET        0x0414 /* Host all channels interrupt register */
#define STM32_OTG_HAINTMSK_OFFSET     0x0418 /* Host all channels interrupt mask register */
#define STM32_OTG_HPRT_OFFSET         0x0440 /* Host port control and status register */

#define STM32_OTG_CHAN_OFFSET(n)      (0x500 + ((n) << 5)
#define STM32_OTG_HCCHAR_CHOFFSET     0x0000 /* Host channel characteristics register */
#define STM32_OTG_HCINT_CHOFFSET      0x0008 /* Host channel interrupt register */
#define STM32_OTG_HCINTMSK_CHOFFSET   0x000c /* Host channel interrupt mask register */
#define STM32_OTG_HCTSIZ_CHOFFSET     0x0010 /* Host channel interrupt register */

#define STM32_OTG_HCCHAR_OFFSET(n)    (0x500 + ((n) << 5))

#define STM32_OTG_HCINT_OFFSET(n)     (0x508 + ((n) << 5))

#define STM32_OTG_HCINTMSK_OFFSET(n)  (0x50c + ((n) << 5))

#define STM32_OTG_HCTSIZ_OFFSET(n)    (0x510 + ((n) << 5))

/* Device-mode control and status registers */

#define STM32_OTG_DCFG_OFFSET         0x0800 /* Device configuration register */
#define STM32_OTG_DCTL_OFFSET         0x0804 /* Device control register */
#define STM32_OTG_DSTS_OFFSET         0x0808 /* Device status register */
#define STM32_OTG_DIEPMSK_OFFSET      0x0810 /* Device IN endpoint common interrupt mask register */
#define STM32_OTG_DOEPMSK_OFFSET      0x0814 /* Device OUT endpoint common interrupt mask register */
#define STM32_OTG_DAINT_OFFSET        0x0818 /* Device all endpoints interrupt register */
#define STM32_OTG_DAINTMSK_OFFSET     0x081c /* All endpoints interrupt mask register */
#define STM32_OTG_DVBUSDIS_OFFSET     0x0828 /* Device VBUS discharge time register */
#define STM32_OTG_DVBUSPULSE_OFFSET   0x082c /* Device VBUS pulsing time register */
#define STM32_OTG_DIEPEMPMSK_OFFSET   0x0834 /* Device IN endpoint FIFO empty interrupt mask register */

#define STM32_OTG_DIEP_OFFSET(n)      (0x0900 + ((n) << 5))
#define STM32_OTG_DIEPCTL_EPOFFSET    0x0000 /* Device endpoint control register */
#define STM32_OTG_DIEPINT_EPOFFSET    0x0008 /* Device endpoint interrupt register */
#define STM32_OTG_DIEPTSIZ_EPOFFSET   0x0010 /* Device IN endpoint transfer size register */
#define STM32_OTG_DTXFSTS_EPOFFSET    0x0018 /* Device IN endpoint transmit FIFO status register */

#define STM32_OTG_DIEPCTL_OFFSET(n)   (0x0900 + ((n) << 5))

#define STM32_OTG_DIEPINT_OFFSET(n)   (0x0908 + ((n) << 5))

#define STM32_OTG_DIEPTSIZ_OFFSET(n)  (0x910 + ((n) << 5))

#define STM32_OTG_DTXFSTS_OFFSET(n)   (0x0918 + ((n) << 5))

#define STM32_OTG_DOEP_OFFSET(n)      (0x0b00 + ((n) << 5))
#define STM32_OTG_DOEPCTL_EPOFFSET    0x0000 /* Device control OUT endpoint 0 control register */
#define STM32_OTG_DOEPINT_EPOFFSET    0x0008 /* Device endpoint-x interrupt register */

#define STM32_OTG_DOEPCTL_OFFSET(n)   (0x0b00 + ((n) << 5))

#define STM32_OTG_DOEPINT_OFFSET(n)   (0x0b08 + ((n) << 5))

#define STM32_OTG_DOEPTSIZ_OFFSET(n)  (0x0b10 + ((n) << 5))

/* Power and clock gating registers */

#define STM32_OTG_PCGCCTL_OFFSET      0x0e00 /* Power and clock gating control register */

/* Data FIFO (DFIFO) access registers */

#define STM32_OTG_DFIFO_DEP_OFFSET(n) (0x1000 + ((n) << 12))
#define STM32_OTG_DFIFO_HCH_OFFSET(n) (0x1000 + ((n) << 12))

/* USB PHY OFFSET */

#define STM32_USBPHYC_PLL1_OFFSET     0x0000 /* USBPHYC PLL1 control register */
#define STM32_USBPHYC_TUNE_OFFSET     0x000c /* USBPHYC tuning control register */
#define STM32_USBPHYC_LDO_OFFSET      0x0018 /* USBPHYC LDO control and status register */

/* Register Addresses *******************************************************************************/

#define STM32_OTG_GOTGCTL             (STM32_OTG_BASE+STM32_OTG_GOTGCTL_OFFSET)
#define STM32_OTG_GOTGINT             (STM32_OTG_BASE+STM32_OTG_GOTGINT_OFFSET)
#define STM32_OTG_GAHBCFG             (STM32_OTG_BASE+STM32_OTG_GAHBCFG_OFFSET)
#define STM32_OTG_GUSBCFG             (STM32_OTG_BASE+STM32_OTG_GUSBCFG_OFFSET)
#define STM32_OTG_GRSTCTL             (STM32_OTG_BASE+STM32_OTG_GRSTCTL_OFFSET)
#define STM32_OTG_GINTSTS             (STM32_OTG_BASE+STM32_OTG_GINTSTS_OFFSET)
#define STM32_OTG_GINTMSK             (STM32_OTG_BASE+STM32_OTG_GINTMSK_OFFSET)
#define STM32_OTG_GRXSTSR             (STM32_OTG_BASE+STM32_OTG_GRXSTSR_OFFSET)
#define STM32_OTG_GRXSTSP             (STM32_OTG_BASE+STM32_OTG_GRXSTSP_OFFSET)
#define STM32_OTG_GRXFSIZ             (STM32_OTG_BASE+STM32_OTG_GRXFSIZ_OFFSET)
#define STM32_OTG_HNPTXFSIZ           (STM32_OTG_BASE+STM32_OTG_HNPTXFSIZ_OFFSET)
#define STM32_OTG_DIEPTXF0            (STM32_OTG_BASE+STM32_OTG_DIEPTXF0_OFFSET)
#define STM32_OTG_HNPTXSTS            (STM32_OTG_BASE+STM32_OTG_HNPTXSTS_OFFSET)
#define STM32_OTG_GCCFG               (STM32_OTG_BASE+STM32_OTG_GCCFG_OFFSET)
#define STM32_OTG_CID                 (STM32_OTG_BASE+STM32_OTG_CID_OFFSET)
#define STM32_OTG_HPTXFSIZ            (STM32_OTG_BASE+STM32_OTG_HPTXFSIZ_OFFSET)

#define STM32_OTG_DIEPTXF(n)          (STM32_OTG_BASE+STM32_OTG_DIEPTXF_OFFSET(n))

/* Host-mode control and status registers */

#define STM32_OTG_HCFG                (STM32_OTG_BASE+STM32_OTG_HCFG_OFFSET)
#define STM32_OTG_HFIR                (STM32_OTG_BASE+STM32_OTG_HFIR_OFFSET)
#define STM32_OTG_HFNUM               (STM32_OTG_BASE+STM32_OTG_HFNUM_OFFSET)
#define STM32_OTG_HPTXSTS             (STM32_OTG_BASE+STM32_OTG_HPTXSTS_OFFSET)
#define STM32_OTG_HAINT               (STM32_OTG_BASE+STM32_OTG_HAINT_OFFSET)
#define STM32_OTG_HAINTMSK            (STM32_OTG_BASE+STM32_OTG_HAINTMSK_OFFSET)
#define STM32_OTG_HPRT                (STM32_OTG_BASE+STM32_OTG_HPRT_OFFSET)

#define STM32_OTG_CHAN(n)             (STM32_OTG_BASE+STM32_OTG_CHAN_OFFSET(n))

#define STM32_OTG_HCCHAR(n)           (STM32_OTG_BASE+STM32_OTG_HCCHAR_OFFSET(n))

#define STM32_OTG_HCINT(n)            (STM32_OTG_BASE+STM32_OTG_HCINT_OFFSET(n))

#define STM32_OTG_HCINTMSK(n)         (STM32_OTG_BASE+STM32_OTG_HCINTMSK_OFFSET(n))

#define STM32_OTG_HCTSIZ(n)           (STM32_OTG_BASE+STM32_OTG_HCTSIZ_OFFSET(n))

/* Device-mode control and status registers */

#define STM32_OTG_DCFG                (STM32_OTG_BASE+STM32_OTG_DCFG_OFFSET)
#define STM32_OTG_DCTL                (STM32_OTG_BASE+STM32_OTG_DCTL_OFFSET)
#define STM32_OTG_DSTS                (STM32_OTG_BASE+STM32_OTG_DSTS_OFFSET)
#define STM32_OTG_DIEPMSK             (STM32_OTG_BASE+STM32_OTG_DIEPMSK_OFFSET)
#define STM32_OTG_DOEPMSK             (STM32_OTG_BASE+STM32_OTG_DOEPMSK_OFFSET)
#define STM32_OTG_DAINT               (STM32_OTG_BASE+STM32_OTG_DAINT_OFFSET)
#define STM32_OTG_DAINTMSK            (STM32_OTG_BASE+STM32_OTG_DAINTMSK_OFFSET)
#define STM32_OTG_DVBUSDIS            (STM32_OTG_BASE+STM32_OTG_DVBUSDIS_OFFSET)
#define STM32_OTG_DVBUSPULSE          (STM32_OTG_BASE+STM32_OTG_DVBUSPULSE_OFFSET)
#define STM32_OTG_DIEPEMPMSK          (STM32_OTG_BASE+STM32_OTG_DIEPEMPMSK_OFFSET)

#define STM32_OTG_DIEP(n)             (STM32_OTG_BASE+STM32_OTG_DIEP_OFFSET(n))

#define STM32_OTG_DIEPCTL(n)          (STM32_OTG_BASE+STM32_OTG_DIEPCTL_OFFSET(n))

#define STM32_OTG_DIEPINT(n)          (STM32_OTG_BASE+STM32_OTG_DIEPINT_OFFSET(n))

#define STM32_OTG_DIEPTSIZ(n)         (STM32_OTG_BASE+STM32_OTG_DIEPTSIZ_OFFSET(n))

#define STM32_OTG_DTXFSTS(n)          (STM32_OTG_BASE+STM32_OTG_DTXFSTS_OFFSET(n))

#define STM32_OTG_DOEP(n)             (STM32_OTG_BASE+STM32_OTG_DOEP_OFFSET(n))

#define STM32_OTG_DOEPCTL(n)          (STM32_OTG_BASE+STM32_OTG_DOEPCTL_OFFSET(n))

#define STM32_OTG_DOEPINT(n)          (STM32_OTG_BASE+STM32_OTG_DOEPINT_OFFSET(n))

#define STM32_OTG_DOEPTSIZ(n)         (STM32_OTG_BASE+STM32_OTG_DOEPTSIZ_OFFSET(n))

/* Power and clock gating registers */

#define STM32_OTG_PCGCCTL             (STM32_OTG_BASE+STM32_OTG_PCGCCTL_OFFSET)

/* Data FIFO (DFIFO) access registers */

#define STM32_OTG_DFIFO_DEP(n)        (STM32_OTG_BASE+STM32_OTG_DFIFO_DEP_OFFSET(n))
#define STM32_OTG_DFIFO_HCH(n)        (STM32_OTG_BASE+STM32_OTG_DFIFO_HCH_OFFSET(n))

/* USB PHY Registers */

#define STM32_USBPHYC_PLL1            (STM32_USBPHYC_BASE+STM32_USBPHYC_PLL1_OFFSET)
#define STM32_USBPHYC_TUNE            (STM32_USBPHYC_BASE+STM32_USBPHYC_TUNE_OFFSET)
#define STM32_USBPHYC_LDO             (STM32_USBPHYC_BASE+STM32_USBPHYC_LDO_OFFSET)

/* Register Bitfield Definitions ********************************************************************/
/* Core global control and status registers */

/* Control and status register */

#define OTG_GOTGCTL_SRQSCS            (1 << 0)  /* Bit 0:  Session request success */
#define OTG_GOTGCTL_SRQ               (1 << 1)  /* Bit 1:  Session request */
#define OTG_GOTGCTL_VBVALOEN          (1 << 2)  /* Bit 2:  VBUS valid override enable */
#define OTG_GOTGCTL_VBVALOVAL         (1 << 3)  /* Bit 3:  VBUS valid override value */
#define OTG_GOTGCTL_AVALOEN           (1 << 4)  /* Bit 4:  A-peripheral session valid override enable */
#define OTG_GOTGCTL_AVALOVAL          (1 << 5)  /* Bit 5:  A-peripheral session valid override value */
#define OTG_GOTGCTL_BVALOEN           (1 << 6)  /* Bit 6:  B-peripheral session valid override enable */
#define OTG_GOTGCTL_BVALOVAL          (1 << 7)  /* Bit 7:  B-peripheral session valid override value  */
#define OTG_GOTGCTL_HNGSCS            (1 << 8)  /* Bit 8:  Host negotiation success */
#define OTG_GOTGCTL_HNPRQ             (1 << 9)  /* Bit 9:  HNP request */
#define OTG_GOTGCTL_HSHNPEN           (1 << 10) /* Bit 10: host set HNP enable */
#define OTG_GOTGCTL_DHNPEN            (1 << 11) /* Bit 11: Device HNP enabled */
#define OTG_GOTGCTL_EHEN              (1 << 12) /* Bit 12: Embedded host enable */
                                                /* Bits 13-15: Reserved, must be kept at reset value */
#define OTG_GOTGCTL_CIDSTS            (1 << 16) /* Bit 16: Connector ID status */
#define OTG_GOTGCTL_DBCT              (1 << 17) /* Bit 17: Long/short debounce time */
#define OTG_GOTGCTL_ASVLD             (1 << 18) /* Bit 18: A-session valid */
#define OTG_GOTGCTL_BSVLD             (1 << 19) /* Bit 19: B-session valid */
#define OTG_GOTGCTL_OTGVER            (1 << 20) /* Bit 20: OTG version */
                                                /* Bits 21-31: Reserved, must be kept at reset value */
/* Interrupt register */
                                                /* Bits 1:0 Reserved, must be kept at reset value */
#define OTG_GOTGINT_SEDET             (1 << 2)  /* Bit 2: Session end detected */
                                                /* Bits 3-7: Reserved, must be kept at reset value */
#define OTG_GOTGINT_SRSSCHG           (1 << 8)  /* Bit 8: Session request success status change */
#define OTG_GOTGINT_HNSSCHG           (1 << 9)  /* Bit 9: Host negotiation success status change */
                                                /* Bits 16:10 Reserved, must be kept at reset value */
#define OTG_GOTGINT_HNGDET            (1 << 17) /* Bit 17: Host negotiation detected */
#define OTG_GOTGINT_ADTOCHG           (1 << 18) /* Bit 18: A-device timeout change */
#define OTG_GOTGINT_DBCDNE            (1 << 19) /* Bit 19: Debounce done */
#define OTG_GOTGINT_IDCHNG            (1 << 20) /* Bit 20: Change in ID pin input value */
                                                /* Bits 21-31: Reserved, must be kept at reset value */

/* AHB configuration register */

#define OTG_GAHBCFG_GINTMSK           (1 << 0)  /* Bit 0: Global interrupt mask */
                                                /* Bits 1-6: Reserved, must be kept at reset value */
#define OTG_GAHBCFG_TXFELVL           (1 << 7)  /* Bit 7: TxFIFO empty level */
#define OTG_GAHBCFG_PTXFELVL          (1 << 8)  /* Bit 8: Periodic TxFIFO empty level */
                                                /* Bits 20-31: Reserved, must be kept at reset value */
/* USB configuration register */

#define OTG_GUSBCFG_TOCAL_SHIFT       (0)       /* Bits 0-2: FS timeout calibration */
#define OTG_GUSBCFG_TOCAL_MASK        (7 << OTG_GUSBCFG_TOCAL_SHIFT)
                                                /* Bits 3-5: Reserved, must be kept at reset value */
#define OTG_GUSBCFG_ULPISEL           (1 << 4)  /* Bit 4: Select which high speed interface is to be used STM32F7x3 only*/
#define OTG_GUSBCFG_PHYSEL            (1 << 6)  /* Bit 6: Full Speed serial transceiver select */
                                                /* Bit 7: Reserved, must be kept at reset value */
#define OTG_GUSBCFG_SRPCAP            (1 << 8)  /* Bit 8: SRP-capable */
#define OTG_GUSBCFG_HNPCAP            (1 << 9)  /* Bit 9: HNP-capable */
#define OTG_GUSBCFG_TRDT_SHIFT        (10)      /* Bits 10-13: USB turnaround time */
#define OTG_GUSBCFG_TRDT_MASK         (15 << OTG_GUSBCFG_TRDT_SHIFT)
#  define OTG_GUSBCFG_TRDT(n)         ((n) << OTG_GUSBCFG_TRDT_SHIFT)
                                                /* Bit 14: Reserved, must be kept at reset value */
#define OTG_GUSBCFG_PHYLPC            (1 << 15) /* Bit 15: PHY Low-power clock select for USB OTG HS */
                                                /* Bit 16: Reserved, must be kept at reset value */
#define OTG_GUSBCFG_ULPIFSLS          (1 << 17) /* Bit 17: ULPI FS/LS select for USB OTG HS */
#define OTG_GUSBCFG_ULPIAR            (1 << 18) /* Bit 18: ULPI Auto-resume for USB OTG HS */
#define OTG_GUSBCFG_ULPICSM           (1 << 19) /* Bit 19: ULPI clock SuspendM for USB OTG HS */
#define OTG_GUSBCFG_ULPIEVBUSD        (1 << 20) /* Bit 20: ULPI External VBUS Drive for USB OTG HS */
#define OTG_GUSBCFG_ULPIEVBUSI        (1 << 21) /* Bit 21: ULPI external VBUS indicator for USB OTG HS */
#define OTG_GUSBCFG_TSDPS             (1 << 22) /* Bit 22: TermSel DLine pulsing selection for USB OTG HS */
#define OTG_GUSBCFG_PCCI              (1 << 23) /* Bit 23: Indicator complement for USB OTG HS */
#define OTG_GUSBCFG_PTCI              (1 << 24) /* Bit 24: Indicator pass through for USB OTG HS */
#define OTG_GUSBCFG_ULPIIPD           (1 << 25) /* Bit 24: ULPI interface protect disable for USB OTG HS */
                                                /* Bit 26-28: Reserved, must be kept at reset value */
#define OTG_GUSBCFG_FHMOD             (1 << 29) /* Bit 29: Force host mode */
#define OTG_GUSBCFG_FDMOD             (1 << 30) /* Bit 30: Force device mode */
                                                /* Bits 20-31: Reserved, must be kept at reset value */
/* Reset register */

#define OTG_GRSTCTL_CSRST             (1 << 0)  /* Bit 0: Core soft reset */
#define OTG_GRSTCTL_HSRST             (1 << 1)  /* Bit 1: HCLK soft reset */
#define OTG_GRSTCTL_FCRST             (1 << 2)  /* Bit 2: Host frame counter reset */
                                                /* Bit 3 Reserved, must be kept at reset value */
#define OTG_GRSTCTL_RXFFLSH           (1 << 4)  /* Bit 4: RxFIFO flush */
#define OTG_GRSTCTL_TXFFLSH           (1 << 5)  /* Bit 5: TxFIFO flush */
#define OTG_GRSTCTL_TXFNUM_SHIFT      (6)       /* Bits 6-10: TxFIFO number */
#define OTG_GRSTCTL_TXFNUM_MASK       (31 << OTG_GRSTCTL_TXFNUM_SHIFT)
#  define OTG_GRSTCTL_TXFNUM_HNONPER  (0 << OTG_GRSTCTL_TXFNUM_SHIFT)   /* Non-periodic TxFIFO flush in host mode */
#  define OTG_GRSTCTL_TXFNUM_HPER     (1 << OTG_GRSTCTL_TXFNUM_SHIFT)   /* Periodic TxFIFO flush in host mode */
#  define OTG_GRSTCTL_TXFNUM_HALL     (16 << OTG_GRSTCTL_TXFNUM_SHIFT)  /* Flush all the transmit FIFOs in host mode.*/
#  define OTG_GRSTCTL_TXFNUM_D(n)     ((n) << OTG_GRSTCTL_TXFNUM_SHIFT) /* TXFIFO n flush in device mode, n=0-15 */
#  define OTG_GRSTCTL_TXFNUM_DALL     (16 << OTG_GRSTCTL_TXFNUM_SHIFT)  /* Flush all the transmit FIFOs in device mode.*/
                                                  /* Bits 11-31: Reserved, must be kept at reset value */
#define OTG_GRSTCTL_AHBIDL            (1 << 31) /* Bit 31: AHB master idle */

/* Core interrupt and Interrupt mask registers */

#define OTG_GINTSTS_CMOD              (1 << 0)  /* Bit 0:  Current mode of operation */
#  define OTG_GINTSTS_DEVMODE         (0)
#  define OTG_GINTSTS_HOSTMODE        (OTG_GINTSTS_CMOD)
#define OTG_GINT_MMIS                 (1 << 1)  /* Bit 1:  Mode mismatch interrupt */
#define OTG_GINT_OTG                  (1 << 2)  /* Bit 2:  OTG interrupt */
#define OTG_GINT_SOF                  (1 << 3)  /* Bit 3:  Start of frame */
#define OTG_GINT_RXFLVL               (1 << 4)  /* Bit 4:  RxFIFO non-empty */
#define OTG_GINT_NPTXFE               (1 << 5)  /* Bit 5:  Non-periodic TxFIFO empty */
#define OTG_GINT_GINAKEFF             (1 << 6)  /* Bit 6:  Global IN non-periodic NAK effective */
#define OTG_GINT_GONAKEFF             (1 << 7)  /* Bit 7:  Global OUT NAK effective */
#define OTG_GINT_RES89                (3 << 8)  /* Bits 8-9: Reserved, must be kept at reset value */
#define OTG_GINT_ESUSP                (1 << 10) /* Bit 10: Early suspend */
#define OTG_GINT_USBSUSP              (1 << 11) /* Bit 11: USB suspend */
#define OTG_GINT_USBRST               (1 << 12) /* Bit 12: USB reset */
#define OTG_GINT_ENUMDNE              (1 << 13) /* Bit 13: Enumeration done */
#define OTG_GINT_ISOODRP              (1 << 14) /* Bit 14: Isochronous OUT packet dropped interrupt */
#define OTG_GINT_EOPF                 (1 << 15) /* Bit 15: End of periodic frame interrupt */
#define OTG_GINT_RES1617              (3 << 16) /* Bits 16-17 Reserved, must be kept at reset value */
#define OTG_GINT_IEP                  (1 << 18) /* Bit 18: IN endpoint interrupt */
#define OTG_GINT_OEP                  (1 << 19) /* Bit 19: OUT endpoint interrupt */
#define OTG_GINT_IISOIXFR             (1 << 20) /* Bit 20: Incomplete isochronous IN transfer */
#define OTG_GINT_IISOOXFR             (1 << 21) /* Bit 21: Incomplete isochronous OUT transfer (device) */
#define OTG_GINT_IPXFR                (1 << 21) /* Bit 21: Incomplete periodic transfer (host) */
#define OTG_GINT_RES22                (1 << 22) /* Bit 22: Reserved, must be kept at reset value */
#define OTG_GINT_DATAFSUSP            (1 << 22) /* Bit 22: Data fetch suspended for USB OTG HS */
#define OTG_GINT_RES23                (1 << 23) /* Bit 23: Reserved, must be kept at reset value */
#define OTG_GINT_RSTDET               (1 << 23) /* Bit 23: Reset detected interrupt */
#define OTG_GINT_HPRT                 (1 << 24) /* Bit 24: Host port interrupt */
#define OTG_GINT_HC                   (1 << 25) /* Bit 25: Host channels interrupt */
#define OTG_GINT_PTXFE                (1 << 26) /* Bit 26: Periodic TxFIFO empty */
#define OTG_GINT_LPMINT               (1 << 27) /* Bit 27: LPM interrupt */
#define OTG_GINT_RES27                (1 << 27) /* Bit 27: Reserved, must be kept at reset value */
#define OTG_GINT_CIDSCHG              (1 << 28) /* Bit 28: Connector ID status change */
#define OTG_GINT_DISC                 (1 << 29) /* Bit 29: Disconnect detected interrupt */
#define OTG_GINT_SRQ                  (1 << 30) /* Bit 30: Session request/new session detected interrupt */
#define OTG_GINT_WKUP                 (1 << 31) /* Bit 31: Resume/remote wakeup detected interrupt */

/* Receive status debug read/OTG status read and pop registers (host mode) */

#define OTG_GRXSTSH_CHNUM_SHIFT       (0)       /* Bits 0-3: Channel number */
#define OTG_GRXSTSH_CHNUM_MASK        (15 << OTG_GRXSTSH_CHNUM_SHIFT)
#define OTG_GRXSTSH_BCNT_SHIFT        (4)       /* Bits 4-14: Byte count */
#define OTG_GRXSTSH_BCNT_MASK         (0x7ff << OTG_GRXSTSH_BCNT_SHIFT)
#define OTG_GRXSTSH_DPID_SHIFT        (15)      /* Bits 15-16: Data PID */
#define OTG_GRXSTSH_DPID_MASK         (3 << OTG_GRXSTSH_DPID_SHIFT)
#  define OTG_GRXSTSH_DPID_DATA0      (0 << OTG_GRXSTSH_DPID_SHIFT)
#  define OTG_GRXSTSH_DPID_DATA2      (1 << OTG_GRXSTSH_DPID_SHIFT)
#  define OTG_GRXSTSH_DPID_DATA1      (2 << OTG_GRXSTSH_DPID_SHIFT)
#  define OTG_GRXSTSH_DPID_MDATA      (3 << OTG_GRXSTSH_DPID_SHIFT)
#define OTG_GRXSTSH_PKTSTS_SHIFT      (17)      /* Bits 17-20: Packet status */
#define OTG_GRXSTSH_PKTSTS_MASK       (15 << OTG_GRXSTSH_PKTSTS_SHIFT)
#  define OTG_GRXSTSH_PKTSTS_INRECVD  (2 << OTG_GRXSTSH_PKTSTS_SHIFT) /* IN data packet received */
#  define OTG_GRXSTSH_PKTSTS_INDONE   (3 << OTG_GRXSTSH_PKTSTS_SHIFT) /* IN transfer completed */
#  define OTG_GRXSTSH_PKTSTS_DTOGERR  (5 << OTG_GRXSTSH_PKTSTS_SHIFT) /* Data toggle error */
#  define OTG_GRXSTSH_PKTSTS_HALTED   (7 << OTG_GRXSTSH_PKTSTS_SHIFT) /* Channel halted */
                                                  /* Bits 21-31: Reserved, must be kept at reset value */
/* Receive status debug read/OTG status read and pop registers (device mode) */

#define OTG_GRXSTSD_EPNUM_SHIFT       (0)       /* Bits 0-3: Endpoint number */
#define OTG_GRXSTSD_EPNUM_MASK        (15 << OTG_GRXSTSD_EPNUM_SHIFT)
#define OTG_GRXSTSD_BCNT_SHIFT        (4)       /* Bits 4-14: Byte count */
#define OTG_GRXSTSD_BCNT_MASK         (0x7ff << OTG_GRXSTSD_BCNT_SHIFT)
#define OTG_GRXSTSD_DPID_SHIFT        (15)      /* Bits 15-16: Data PID */
#define OTG_GRXSTSD_DPID_MASK         (3 << OTG_GRXSTSD_DPID_SHIFT)
#  define OTG_GRXSTSD_DPID_DATA0      (0 << OTG_GRXSTSD_DPID_SHIFT)
#  define OTG_GRXSTSD_DPID_DATA2      (1 << OTG_GRXSTSD_DPID_SHIFT)
#  define OTG_GRXSTSD_DPID_DATA1      (2 << OTG_GRXSTSD_DPID_SHIFT)
#  define OTG_GRXSTSD_DPID_MDATA      (3 << OTG_GRXSTSD_DPID_SHIFT)
#define OTG_GRXSTSD_PKTSTS_SHIFT      (17)      /* Bits 17-20: Packet status */
#define OTG_GRXSTSD_PKTSTS_MASK       (15 << OTG_GRXSTSD_PKTSTS_SHIFT)
#  define OTG_GRXSTSD_PKTSTS_OUTNAK     (1 << OTG_GRXSTSD_PKTSTS_SHIFT) /* Global OUT NAK */
#  define OTG_GRXSTSD_PKTSTS_OUTRECVD   (2 << OTG_GRXSTSD_PKTSTS_SHIFT) /* OUT data packet received */
#  define OTG_GRXSTSD_PKTSTS_OUTDONE    (3 << OTG_GRXSTSD_PKTSTS_SHIFT) /* OUT transfer completed */
#  define OTG_GRXSTSD_PKTSTS_SETUPDONE  (4 << OTG_GRXSTSD_PKTSTS_SHIFT) /* SETUP transaction completed */
#  define OTG_GRXSTSD_PKTSTS_SETUPRECVD (6 << OTG_GRXSTSD_PKTSTS_SHIFT) /* SETUP data packet received */
#define OTG_GRXSTSD_FRMNUM_SHIFT      (21)      /* Bits 21-24: Frame number */
#define OTG_GRXSTSD_FRMNUM_MASK       (15 << OTG_GRXSTSD_FRMNUM_SHIFT)
                                                /* Bits 25-31: Reserved, must be kept at reset value */
/* Receive FIFO size register */

#define OTG_GRXFSIZ_MASK              (0xffff)

/* Host non-periodic transmit FIFO size register */

#define OTG_HNPTXFSIZ_NPTXFSA_SHIFT   (0)       /* Bits 0-15: Non-periodic transmit RAM start address */
#define OTG_HNPTXFSIZ_NPTXFSA_MASK    (0xffff << OTG_HNPTXFSIZ_NPTXFSA_SHIFT)
#define OTG_HNPTXFSIZ_NPTXFD_SHIFT    (16)      /* Bits 16-31: Non-periodic TxFIFO depth */
#define OTG_HNPTXFSIZ_NPTXFD_MASK     (0xffff << OTG_HNPTXFSIZ_NPTXFD_SHIFT)
#  define OTG_HNPTXFSIZ_NPTXFD_MIN    (16 << OTG_HNPTXFSIZ_NPTXFD_SHIFT)
#  define OTG_HNPTXFSIZ_NPTXFD_MAX    (256 << OTG_HNPTXFSIZ_NPTXFD_SHIFT)

/* Endpoint 0 Transmit FIFO size */

#define OTG_DIEPTXF0_TX0FD_SHIFT      (0)       /* Bits 0-15: Endpoint 0 transmit RAM start address */
#define OTG_DIEPTXF0_TX0FD_MASK       (0xffff << OTG_DIEPTXF0_TX0FD_SHIFT)
#define OTG_DIEPTXF0_TX0FSA_SHIFT     (16)      /* Bits 16-31: Endpoint 0 TxFIFO depth */
#define OTG_DIEPTXF0_TX0FSA_MASK      (0xffff << OTG_DIEPTXF0_TX0FSA_SHIFT)
#  define OTG_DIEPTXF0_TX0FSA_MIN     (16 << OTG_DIEPTXF0_TX0FSA_SHIFT)
#  define OTG_DIEPTXF0_TX0FSA_MAX     (256 << OTG_DIEPTXF0_TX0FSA_SHIFT)

/* Non-periodic transmit FIFO/queue status register */

#define OTG_HNPTXSTS_NPTXFSAV_SHIFT   (0)       /* Bits 0-15: Non-periodic TxFIFO space available */
#define OTG_HNPTXSTS_NPTXFSAV_MASK    (0xffff << OTG_HNPTXSTS_NPTXFSAV_SHIFT)
#  define OTG_HNPTXSTS_NPTXFSAV_FULL  (0 << OTG_HNPTXSTS_NPTXFSAV_SHIFT)
#define OTG_HNPTXSTS_NPTQXSAV_SHIFT   (16)      /* Bits 16-23: Non-periodic transmit request queue space available */
#define OTG_HNPTXSTS_NPTQXSAV_MASK    (0xff << OTG_HNPTXSTS_NPTQXSAV_SHIFT)
#  define OTG_HNPTXSTS_NPTQXSAV_FULL  (0 << OTG_HNPTXSTS_NPTQXSAV_SHIFT)
#define OTG_HNPTXSTS_NPTXQTOP_SHIFT   (24)      /* Bits 24-30: Top of the non-periodic transmit request queue */
#define OTG_HNPTXSTS_NPTXQTOP_MASK    (0x7f << OTG_HNPTXSTS_NPTXQTOP_SHIFT)
#  define OTG_HNPTXSTS_TERMINATE      (1 << 24) /* Bit 24: Terminate (last entry for selected channel/endpoint) */
#  define OTG_HNPTXSTS_TYPE_SHIFT     (25)      /* Bits 25-26: Status */
#  define OTG_HNPTXSTS_TYPE_MASK      (3 << OTG_HNPTXSTS_TYPE_SHIFT)
#    define OTG_HNPTXSTS_TYPE_INOUT   (0 << OTG_HNPTXSTS_TYPE_SHIFT) /* IN/OUT token */
#    define OTG_HNPTXSTS_TYPE_ZLP     (1 << OTG_HNPTXSTS_TYPE_SHIFT) /* Zero-length transmit packet (device IN/host OUT) */
#    define OTG_HNPTXSTS_TYPE_HALT    (3 << OTG_HNPTXSTS_TYPE_SHIFT) /* Channel halt command */
#  define OTG_HNPTXSTS_CHNUM_SHIFT    (27)      /* Bits 27-30: Channel number */
#  define OTG_HNPTXSTS_CHNUM_MASK     (15 << OTG_HNPTXSTS_CHNUM_SHIFT)
#  define OTG_HNPTXSTS_EPNUM_SHIFT    (27)      /* Bits 27-30: Endpoint number */
#  define OTG_HNPTXSTS_EPNUM_MASK     (15 << OTG_HNPTXSTS_EPNUM_SHIFT)
                                                /* Bit 31 Reserved, must be kept at reset value */
/* General core configuration register */
                                                /* Bits 0-15: Reserved, must be kept at reset value */
#define OTG_GCCFG_PWRDWN              (1 << 16) /* Bit 16: Power down */
                                                /* Bit 17 Reserved, must be kept at reset value */
#define OTG_GCCFG_VBDEN               (1 << 21) /* Bit 21: USB VBUS detection enable */
#define OTG_GCCFG_PHYHSEN             (1 << 23) /* Bit 21: NOT In datasheet but in STMCube.. */
                                                /* Bits 22-31: Reserved, must be kept at reset value */
/* Core ID register  (32-bit product ID) */

/* Host periodic transmit FIFO size register */

#define OTG_HPTXFSIZ_PTXSA_SHIFT      (0)       /* Bits 0-15: Host periodic TxFIFO start address */
#define OTG_HPTXFSIZ_PTXSA_MASK       (0xffff << OTG_HPTXFSIZ_PTXSA_SHIFT)
#define OTG_HPTXFSIZ_PTXFD_SHIFT      (16)      /* Bits 16-31: Host periodic TxFIFO depth */
#define OTG_HPTXFSIZ_PTXFD_MASK       (0xffff << OTG_HPTXFSIZ_PTXFD_SHIFT)

/* Device IN endpoint transmit FIFOn size register */

#define OTG_DIEPTXF_INEPTXSA_SHIFT    (0)       /* Bits 0-15: IN endpoint FIFOx transmit RAM start address */
#define OTG_DIEPTXF_INEPTXSA_MASK     (0xffff << OTG_DIEPTXF_INEPTXSA_SHIFT)
#define OTG_DIEPTXF_INEPTXFD_SHIFT    (16)       /* Bits 16-31: IN endpoint TxFIFO depth */
#define OTG_DIEPTXF_INEPTXFD_MASK     (0xffff << OTG_DIEPTXF_INEPTXFD_SHIFT)
#  define OTG_DIEPTXF_INEPTXFD_MIN    (16 << OTG_DIEPTXF_INEPTXFD_MASK)

/* Host-mode control and status registers */

/* Host configuration register */

#define OTG_HCFG_FSLSPCS_SHIFT        (0)       /* Bits 0-1: FS/LS PHY clock select */
#define OTG_HCFG_FSLSPCS_MASK         (3 << OTG_HCFG_FSLSPCS_SHIFT)
#  define OTG_HCFG_FSLSPCS_FS48MHz    (1 << OTG_HCFG_FSLSPCS_SHIFT) /* FS host mode, PHY clock is running at 48 MHz */
#  define OTG_HCFG_FSLSPCS_LS48MHz    (1 << OTG_HCFG_FSLSPCS_SHIFT) /* LS host mode,  Select 48 MHz PHY clock frequency */
#  define OTG_HCFG_FSLSPCS_LS6MHz     (2 << OTG_HCFG_FSLSPCS_SHIFT) /* LS host mode, Select 6 MHz PHY clock frequency */
#define OTG_HCFG_FSLSS                (1 << 2)  /* Bit 2: FS- and LS-only support */
                                                /* Bits 31:3 Reserved, must be kept at reset value */
/* Host frame interval register */

#define OTG_HFIR_MASK                 (0xffff)

/* Host frame number/frame time remaining register */

#define OTG_HFNUM_FRNUM_SHIFT         (0)       /* Bits 0-15: Frame number */
#define OTG_HFNUM_FRNUM_MASK          (0xffff << OTG_HFNUM_FRNUM_SHIFT)
#define OTG_HFNUM_FTREM_SHIFT         (16)      /* Bits 16-31: Frame time remaining */
#define OTG_HFNUM_FTREM_MASK          (0xffff << OTG_HFNUM_FTREM_SHIFT)

/* Host periodic transmit FIFO/queue status register */

#define OTG_HPTXSTS_PTXFSAVL_SHIFT    (0)       /* Bits 0-15: Periodic transmit data FIFO space available */
#define OTG_HPTXSTS_PTXFSAVL_MASK     (0xffff << OTG_HPTXSTS_PTXFSAVL_SHIFT)
#  define OTG_HPTXSTS_PTXFSAVL_FULL   (0 << OTG_HPTXSTS_PTXFSAVL_SHIFT)
#define OTG_HPTXSTS_PTXQSAV_SHIFT     (16)      /* Bits 16-23: Periodic transmit request queue space available */
#define OTG_HPTXSTS_PTXQSAV_MASK      (0xff << OTG_HPTXSTS_PTXQSAV_SHIFT)
#  define OTG_HPTXSTS_PTXQSAV_FULL    (0 << OTG_HPTXSTS_PTXQSAV_SHIFT)
#define OTG_HPTXSTS_PTXQTOP_SHIFT     (24)      /* Bits 24-31: Top of the periodic transmit request queue */
#define OTG_HPTXSTS_PTXQTOP_MASK      (0x7f << OTG_HPTXSTS_PTXQTOP_SHIFT)
#  define OTG_HPTXSTS_TERMINATE       (1 << 24) /* Bit 24: Terminate (last entry for selected channel/endpoint) */
#  define OTG_HPTXSTS_TYPE_SHIFT      (25)      /* Bits 25-26: Type */
#  define OTG_HPTXSTS_TYPE_MASK       (3 << OTG_HPTXSTS_TYPE_SHIFT)
#    define OTG_HPTXSTS_TYPE_INOUT    (0 << OTG_HPTXSTS_TYPE_SHIFT) /* IN/OUT token */
#    define OTG_HPTXSTS_TYPE_ZLP      (1 << OTG_HPTXSTS_TYPE_SHIFT) /* Zero-length transmit packet */
#    define OTG_HPTXSTS_TYPE_HALT     (3 << OTG_HPTXSTS_TYPE_SHIFT) /* Disable channel command */
#  define OTG_HPTXSTS_EPNUM_SHIFT     (27)      /* Bits 27-30: Endpoint number */
#  define OTG_HPTXSTS_EPNUM_MASK      (15 << OTG_HPTXSTS_EPNUM_SHIFT)
#  define OTG_HPTXSTS_CHNUM_SHIFT     (27)      /* Bits 27-30: Channel number */
#  define OTG_HPTXSTS_CHNUM_MASK      (15 << OTG_HPTXSTS_CHNUM_SHIFT)
#  define OTG_HPTXSTS_ODD             (1 << 24) /* Bit 31: Send in odd (vs even) frame */

/* Host all channels interrupt and all channels interrupt mask registers */

#define OTG_HAINT(n)                  (1 << (n)) /* Bits 15:0 HAINTM: Channel interrupt */

/* Host port control and status register */

#define OTG_HPRT_PCSTS                (1 << 0)  /* Bit 0:  Port connect status */
#define OTG_HPRT_PCDET                (1 << 1)  /* Bit 1:  Port connect detected */
#define OTG_HPRT_PENA                 (1 << 2)  /* Bit 2:  Port enable */
#define OTG_HPRT_PENCHNG              (1 << 3)  /* Bit 3:  Port enable/disable change */
#define OTG_HPRT_POCA                 (1 << 4)  /* Bit 4:  Port overcurrent active */
#define OTG_HPRT_POCCHNG              (1 << 5)  /* Bit 5:  Port overcurrent change */
#define OTG_HPRT_PRES                 (1 << 6)  /* Bit 6:  Port resume */
#define OTG_HPRT_PSUSP                (1 << 7)  /* Bit 7:  Port suspend */
#define OTG_HPRT_PRST                 (1 << 8)  /* Bit 8:  Port reset */
                                                /* Bit 9:  Reserved, must be kept at reset value */
#define OTG_HPRT_PLSTS_SHIFT          (10)      /* Bits 10-11: Port line status */
#define OTG_HPRT_PLSTS_MASK           (3 << OTG_HPRT_PLSTS_SHIFT)
#  define OTG_HPRT_PLSTS_DP           (1 << 10) /* Bit 10: Logic level of OTG_FS_FS_DP */
#  define OTG_HPRT_PLSTS_DM           (1 << 11) /* Bit 11: Logic level of OTG_FS_FS_DM */
#define OTG_HPRT_PPWR                 (1 << 12) /* Bit 12: Port power */
#define OTG_HPRT_PTCTL_SHIFT          (13)      /* Bits 13-16: Port test control */
#define OTG_HPRT_PTCTL_MASK           (15 << OTG_HPRT_PTCTL_SHIFT)
#  define OTG_HPRT_PTCTL_DISABLED     (0 << OTG_HPRT_PTCTL_SHIFT) /* Test mode disabled */
#  define OTG_HPRT_PTCTL_J            (1 << OTG_HPRT_PTCTL_SHIFT) /* Test_J mode */
#  define OTG_HPRT_PTCTL_L            (2 << OTG_HPRT_PTCTL_SHIFT) /* Test_K mode */
#  define OTG_HPRT_PTCTL_SE0_NAK      (3 << OTG_HPRT_PTCTL_SHIFT) /* Test_SE0_NAK mode */
#  define OTG_HPRT_PTCTL_PACKET       (4 << OTG_HPRT_PTCTL_SHIFT) /* Test_Packet mode */
#  define OTG_HPRT_PTCTL_FORCE        (5 << OTG_HPRT_PTCTL_SHIFT) /* Test_Force_Enable */
#define OTG_HPRT_PSPD_SHIFT           (17)      /* Bits 17-18: Port speed */
#define OTG_HPRT_PSPD_MASK            (3 << OTG_HPRT_PSPD_SHIFT)
#  define OTG_HPRT_PSPD_FS            (1 << OTG_HPRT_PSPD_SHIFT) /* Full speed */
#  define OTG_HPRT_PSPD_LS            (2 << OTG_HPRT_PSPD_SHIFT) /* Low speed */
                                                /* Bits 19-31: Reserved, must be kept at reset value */

/* Host channel-n characteristics register */

#define OTG_HCCHAR_MPSIZ_SHIFT        (0)       /* Bits 0-10: Maximum packet size */
#define OTG_HCCHAR_MPSIZ_MASK         (0x7ff << OTG_HCCHAR_MPSIZ_SHIFT)
#define OTG_HCCHAR_EPNUM_SHIFT        (11)      /* Bits 11-14: Endpoint number */
#define OTG_HCCHAR_EPNUM_MASK         (15 << OTG_HCCHAR_EPNUM_SHIFT)
#define OTG_HCCHAR_EPDIR              (1 << 15) /* Bit 15: Endpoint direction */
#  define OTG_HCCHAR_EPDIR_OUT        (0)
#  define OTG_HCCHAR_EPDIR_IN         OTG_HCCHAR_EPDIR
                                                /* Bit 16 Reserved, must be kept at reset value */
#define OTG_HCCHAR_LSDEV              (1 << 17) /* Bit 17: Low-speed device */
#define OTG_HCCHAR_EPTYP_SHIFT        (18)      /* Bits 18-19: Endpoint type */
#define OTG_HCCHAR_EPTYP_MASK         (3 << OTG_HCCHAR_EPTYP_SHIFT)
#  define OTG_HCCHAR_EPTYP_CTRL       (0 << OTG_HCCHAR_EPTYP_SHIFT) /* Control */
#  define OTG_HCCHAR_EPTYP_ISOC       (1 << OTG_HCCHAR_EPTYP_SHIFT) /* Isochronous */
#  define OTG_HCCHAR_EPTYP_BULK       (2 << OTG_HCCHAR_EPTYP_SHIFT) /* Bulk */
#  define OTG_HCCHAR_EPTYP_INTR       (3 << OTG_HCCHAR_EPTYP_SHIFT) /* Interrupt */
#define OTG_HCCHAR_MCNT_SHIFT         (20)      /* Bits 20-21: Multicount */
#define OTG_HCCHAR_MCNT_MASK          (3 << OTG_HCCHAR_MCNT_SHIFT)
#define OTG_HCCHAR_DAD_SHIFT          (22)      /* Bits 22-28: Device address */
#define OTG_HCCHAR_DAD_MASK           (0x7f << OTG_HCCHAR_DAD_SHIFT)
#define OTG_HCCHAR_ODDFRM             (1 << 29) /* Bit 29: Odd frame */
#define OTG_HCCHAR_CHDIS              (1 << 30) /* Bit 30: Channel disable */
#define OTG_HCCHAR_CHENA              (1 << 31) /* Bit 31: Channel enable */

/* Host channel-n interrupt and Host channel-0 interrupt mask registers */

#define OTG_HCINT_XFRC                (1 << 0)  /* Bit 0:  Transfer completed */
#define OTG_HCINT_CHH                 (1 << 1)  /* Bit 1:  Channel halted */
                                                /* Bit 2:  Reserved, must be kept at reset value */
#define OTG_HCINT_STALL               (1 << 3)  /* Bit 3:  STALL response received interrupt */
#define OTG_HCINT_NAK                 (1 << 4)  /* Bit 4:  NAK response received interrupt */
#define OTG_HCINT_ACK                 (1 << 5)  /* Bit 5:  ACK response received/transmitted interrupt */
#define OTG_HCINT_NYET                (1 << 6)  /* Bit 6:  Response received interrupt */
#define OTG_HCINT_TXERR               (1 << 7)  /* Bit 7:  Transaction error */
#define OTG_HCINT_BBERR               (1 << 8)  /* Bit 8:  Babble error */
#define OTG_HCINT_FRMOR               (1 << 9)  /* Bit 9:  Frame overrun */
#define OTG_HCINT_DTERR               (1 << 10) /* Bit 10: Data toggle error */
                                                /* Bits 11-31 Reserved, must be kept at reset value */
/* Host channel-n interrupt register */

#define OTG_HCTSIZ_XFRSIZ_SHIFT       (0)       /* Bits 0-18: Transfer size */
#define OTG_HCTSIZ_XFRSIZ_MASK        (0x7ffff << OTG_HCTSIZ_XFRSIZ_SHIFT)
#define OTG_HCTSIZ_PKTCNT_SHIFT       (19)      /* Bits 19-28: Packet count */
#define OTG_HCTSIZ_PKTCNT_MASK        (0x3ff << OTG_HCTSIZ_PKTCNT_SHIFT)
#define OTG_HCTSIZ_DPID_SHIFT         (29)      /* Bits 29-30: Data PID */
#define OTG_HCTSIZ_DPID_MASK          (3 << OTG_HCTSIZ_DPID_SHIFT)
#  define OTG_HCTSIZ_DPID_DATA0       (0 << OTG_HCTSIZ_DPID_SHIFT)
#  define OTG_HCTSIZ_DPID_DATA2       (1 << OTG_HCTSIZ_DPID_SHIFT)
#  define OTG_HCTSIZ_DPID_DATA1       (2 << OTG_HCTSIZ_DPID_SHIFT)
#  define OTG_HCTSIZ_DPID_MDATA       (3 << OTG_HCTSIZ_DPID_SHIFT) /* Non-control */
#  define OTG_HCTSIZ_PID_SETUP        (3 << OTG_HCTSIZ_DPID_SHIFT) /* Control */
                                                /* Bit 31 Reserved, must be kept at reset value */
/* Device-mode control and status registers */

/* Device configuration register */

#define OTG_DCFG_DSPD_SHIFT           (0)       /* Bits 0-1: Device speed */
#define OTG_DCFG_DSPD_MASK            (3 << OTG_DCFG_DSPD_SHIFT)
#  define OTG_DCFG_DSPD_HS            (0 << OTG_DCFG_DSPD_SHIFT) /* High Speed */
#  define OTG_DCFG_DSPD_FS_USING_HS   (1 << OTG_DCFG_DSPD_SHIFT) /* Full speed using High Speed*/
#  define OTG_DCFG_DSPD_FS            (3 << OTG_DCFG_DSPD_SHIFT) /* Full speed */
#define OTG_DCFG_NZLSOHSK             (1 << 2)  /* Bit 2:  Non-zero-length status OUT handshake */
                                                /* Bit 3:  Reserved, must be kept at reset value */
#define OTG_DCFG_DAD_SHIFT            (4)       /* Bits 4-10: Device address */
#define OTG_DCFG_DAD_MASK             (0x7f << OTG_DCFG_DAD_SHIFT)
#define OTG_DCFG_PFIVL_SHIFT          (11)      /* Bits 11-12: Periodic frame interval */
#define OTG_DCFG_PFIVL_MASK           (3 << OTG_DCFG_PFIVL_SHIFT)
#  define OTG_DCFG_PFIVL_80PCT        (0 << OTG_DCFG_PFIVL_SHIFT) /* 80% of the frame interval */
#  define OTG_DCFG_PFIVL_85PCT        (1 << OTG_DCFG_PFIVL_SHIFT) /* 85% of the frame interval */
#  define OTG_DCFG_PFIVL_90PCT        (2 << OTG_DCFG_PFIVL_SHIFT) /* 90% of the frame interval */
#  define OTG_DCFG_PFIVL_95PCT        (3 << OTG_DCFG_PFIVL_SHIFT) /* 95% of the frame interval */
                                                /* Bits 13-31 Reserved, must be kept at reset value */
/* Device control register */

#define OTG_TESTMODE_DISABLED         (0) /* Test mode disabled */
#define OTG_TESTMODE_J                (1) /* Test_J mode */
#define OTG_TESTMODE_K                (2) /* Test_K mode */
#define OTG_TESTMODE_SE0_NAK          (3) /* Test_SE0_NAK mode */
#define OTG_TESTMODE_PACKET           (4) /* Test_Packet mode */
#define OTG_TESTMODE_FORCE            (5) /* Test_Force_Enable */

#define OTG_DCTL_RWUSIG               (1 << 0)  /* Bit 0:  Remote wakeup signaling */
#define OTG_DCTL_SDIS                 (1 << 1)  /* Bit 1:  Soft disconnect */
#define OTG_DCTL_GINSTS               (1 << 2)  /* Bit 2:  Global IN NAK status */
#define OTG_DCTL_GONSTS               (1 << 3)  /* Bit 3:  Global OUT NAK status */
#define OTG_DCTL_TCTL_SHIFT           (4)       /* Bits 4-6: Test control */
#define OTG_DCTL_TCTL_MASK            (7 << OTG_DCTL_TCTL_SHIFT)
#  define OTG_DCTL_TCTL_DISABLED      (0 << OTG_DCTL_TCTL_SHIFT) /* Test mode disabled */
#  define OTG_DCTL_TCTL_J             (1 << OTG_DCTL_TCTL_SHIFT) /* Test_J mode */
#  define OTG_DCTL_TCTL_K             (2 << OTG_DCTL_TCTL_SHIFT) /* Test_K mode */
#  define OTG_DCTL_TCTL_SE0_NAK       (3 << OTG_DCTL_TCTL_SHIFT) /* Test_SE0_NAK mode */
#  define OTG_DCTL_TCTL_PACKET        (4 << OTG_DCTL_TCTL_SHIFT) /* Test_Packet mode */
#  define OTG_DCTL_TCTL_FORCE         (5 << OTG_DCTL_TCTL_SHIFT) /* Test_Force_Enable */
#define OTG_DCTL_SGINAK               (1 << 7)  /* Bit 7:  Set global IN NAK */
#define OTG_DCTL_CGINAK               (1 << 8)  /* Bit 8:  Clear global IN NAK */
#define OTG_DCTL_SGONAK               (1 << 9)  /* Bit 9:  Set global OUT NAK */
#define OTG_DCTL_CGONAK               (1 << 10) /* Bit 10: Clear global OUT NAK */
#define OTG_DCTL_POPRGDNE             (1 << 11) /* Bit 11: Power-on programming done */
                                                /* Bits 12-31: Reserved, must be kept at reset value */
/* Device status register */

#define OTG_DSTS_SUSPSTS              (1 << 0)  /* Bit 0: Suspend status */
#define OTG_DSTS_ENUMSPD_SHIFT        (1)       /* Bits 1-2: Enumerated speed */
#define OTG_DSTS_ENUMSPD_MASK         (3 << OTG_DSTS_ENUMSPD_SHIFT)
#  define OTG_DSTS_ENUMSPD_FS         (3 << OTG_DSTS_ENUMSPD_MASK) /* Full speed */
                                                /* Bits 4-7: Reserved, must be kept at reset value */
#define OTG_DSTS_EERR                 (1 << 3)  /* Bit 3: Erratic error */
#define OTG_DSTS_SOFFN_SHIFT          (8)       /* Bits 8-21: Frame number of the received SOF */
#define OTG_DSTS_SOFFN_MASK           (0x3fff << OTG_DSTS_SOFFN_SHIFT)
#define OTG_DSTS_SOFFN0               (1 << 8)  /* Bits 8: Frame number even/odd bit */
#define OTG_DSTS_SOFFN_EVEN           0
#define OTG_DSTS_SOFFN_ODD            OTG_DSTS_SOFFN0
                                                /* Bits 22-31: Reserved, must be kept at reset value */
/* Device IN endpoint common interrupt mask register */

#define OTG_DIEPMSK_XFRCM             (1 << 0)  /* Bit 0: Transfer completed interrupt mask */
#define OTG_DIEPMSK_EPDM              (1 << 1)  /* Bit 1: Endpoint disabled interrupt mask */
#define OTG_DIEPMSK_AHBERRM           (1 << 2)  /* Bit 2: AHB error mask for USB OTG HS */
#define OTG_DIEPMSK_TOM               (1 << 3)  /* Bit 3: Timeout condition mask (Non-isochronous endpoints) */
#define OTG_DIEPMSK_ITTXFEMSK         (1 << 4)  /* Bit 4: IN token received when TxFIFO empty mask */
#define OTG_DIEPMSK_INEPNMM           (1 << 5)  /* Bit 5: IN token received with EP mismatch mask */
#define OTG_DIEPMSK_INEPNEM           (1 << 6)  /* Bit 6: IN endpoint NAK effective mask */
                                                /* Bit 7: Reserved, must be kept at reset value */
#define OTG_DIEPMSK_TXFURM            (1 << 8)  /* Bit 8: FIFO underrun mask */
                                                /* Bits 9-12: Reserved, must be kept at reset value */
#define OTG_DIEPMSK_NAKM              (1 << 13) /* Bit 13: NAK interrupt mask */
                                                /* Bits 14-31: Reserved, must be kept at reset value */

/* Device OUT endpoint common interrupt mask register */

#define OTG_DOEPMSK_XFRCM             (1 << 0)  /* Bit 0: Transfer completed interrupt mask */
#define OTG_DOEPMSK_EPDM              (1 << 1)  /* Bit 1: Endpoint disabled interrupt mask */
                                                /* Bit 2:  Reserved, must be kept at reset value */
#define OTG_DOEPMSK_STUPM             (1 << 3)  /* Bit 3: SETUP phase done mask */
#define OTG_DOEPMSK_OTEPDM            (1 << 4)  /* Bit 4: OUT token received when endpoint disabled mask */
                                                /* Bits 5-31: Reserved, must be kept at reset value */
/* Device all endpoints interrupt and All endpoints interrupt mask registers */

#define OTG_DAINT_IEP_SHIFT           (0)       /* Bits 0-15: IN endpoint interrupt bits */
#define OTG_DAINT_IEP_MASK            (0xffff << OTG_DAINT_IEP_SHIFT)
#  define OTG_DAINT_IEP(n)            (1 << (n))
#define OTG_DAINT_OEP_SHIFT           (16)      /* Bits 16-31: OUT endpoint interrupt bits */
#define OTG_DAINT_OEP_MASK            (0xffff << OTG_DAINT_OEP_SHIFT)
#  define OTG_DAINT_OEP(n)            (1 << ((n)+16))

/* Device VBUS discharge time register */

#define OTG_DVBUSDIS_MASK             (0xffff)

/* Device VBUS pulsing time register */

#define OTG_DVBUSPULSE_MASK           (0xfff)

/* Device IN endpoint FIFO empty interrupt mask register */

#define OTG_DIEPEMPMSK(n)             (1 << (n))

/* Device control IN endpoint 0 control register */

#define OTG_DIEPCTL0_MPSIZ_SHIFT      (0)       /* Bits 0-1: Maximum packet size */
#define OTG_DIEPCTL0_MPSIZ_MASK       (3 << OTG_DIEPCTL0_MPSIZ_SHIFT)
#  define OTG_DIEPCTL0_MPSIZ_64       (0 << OTG_DIEPCTL0_MPSIZ_SHIFT) /* 64 bytes */
#  define OTG_DIEPCTL0_MPSIZ_32       (1 << OTG_DIEPCTL0_MPSIZ_SHIFT) /* 32 bytes */
#  define OTG_DIEPCTL0_MPSIZ_16       (2 << OTG_DIEPCTL0_MPSIZ_SHIFT) /* 16 bytes */
#  define OTG_DIEPCTL0_MPSIZ_8        (3 << OTG_DIEPCTL0_MPSIZ_SHIFT) /* 8 bytes */
                                                /* Bits 2-14: Reserved, must be kept at reset value */
#define OTG_DIEPCTL0_USBAEP           (1 << 15) /* Bit 15: USB active endpoint */
                                                /* Bit 16: Reserved, must be kept at reset value */
#define OTG_DIEPCTL0_NAKSTS           (1 << 17) /* Bit 17: NAK status */
#define OTG_DIEPCTL0_EPTYP_SHIFT      (18)      /* Bits 18-19: Endpoint type */
#define OTG_DIEPCTL0_EPTYP_MASK       (3 << OTG_DIEPCTL0_EPTYP_SHIFT)
#  define OTG_DIEPCTL0_EPTYP_CTRL     (0 << OTG_DIEPCTL0_EPTYP_SHIFT) /* Control (hard-coded) */
                                                /* Bit 20: Reserved, must be kept at reset value */
#define OTG_DIEPCTL0_STALL            (1 << 21) /* Bit 21: STALL handshake */
#define OTG_DIEPCTL0_TXFNUM_SHIFT     (22)      /* Bits 22-25: TxFIFO number */
#define OTG_DIEPCTL0_TXFNUM_MASK      (15 << OTG_DIEPCTL0_TXFNUM_SHIFT)
#define OTG_DIEPCTL0_CNAK             (1 << 26) /* Bit 26: Clear NAK */
#define OTG_DIEPCTL0_SNAK             (1 << 27) /* Bit 27: Set NAK */
                                                /* Bits 28-29: Reserved, must be kept at reset value */
#define OTG_DIEPCTL0_EPDIS            (1 << 30) /* Bit 30: Endpoint disable */
#define OTG_DIEPCTL0_EPENA            (1 << 31) /* Bit 31: Endpoint enable */

/* Device control IN endpoint n control register */

#define OTG_DIEPCTL_MPSIZ_SHIFT       (0)       /* Bits 0-10: Maximum packet size */
#define OTG_DIEPCTL_MPSIZ_MASK        (0x7ff << OTG_DIEPCTL_MPSIZ_SHIFT)
                                                /* Bits 11-14: Reserved, must be kept at reset value */
#define OTG_DIEPCTL_USBAEP            (1 << 15) /* Bit 15: USB active endpoint */
#define OTG_DIEPCTL_EONUM             (1 << 16) /* Bit 16: Even/odd frame */
#  define OTG_DIEPCTL_EVEN            (0)
#  define OTG_DIEPCTL_ODD             OTG_DIEPCTL_EONUM
#  define OTG_DIEPCTL_DATA0           (0)
#  define OTG_DIEPCTL_DATA1           OTG_DIEPCTL_EONUM
#define OTG_DIEPCTL_NAKSTS            (1 << 17) /* Bit 17: NAK status */
#define OTG_DIEPCTL_EPTYP_SHIFT       (18)      /* Bits 18-19: Endpoint type */
#define OTG_DIEPCTL_EPTYP_MASK        (3 << OTG_DIEPCTL_EPTYP_SHIFT)
#  define OTG_DIEPCTL_EPTYP_CTRL      (0 << OTG_DIEPCTL_EPTYP_SHIFT) /* Control */
#  define OTG_DIEPCTL_EPTYP_ISOC      (1 << OTG_DIEPCTL_EPTYP_SHIFT) /* Isochronous */
#  define OTG_DIEPCTL_EPTYP_BULK      (2 << OTG_DIEPCTL_EPTYP_SHIFT) /* Bulk */
#  define OTG_DIEPCTL_EPTYP_INTR      (3 << OTG_DIEPCTL_EPTYP_SHIFT) /* Interrupt */
                                                /* Bit 20: Reserved, must be kept at reset value */
#define OTG_DIEPCTL_STALL             (1 << 21) /* Bit 21: STALL handshake */
#define OTG_DIEPCTL_TXFNUM_SHIFT      (22)      /* Bits 22-25: TxFIFO number */
#define OTG_DIEPCTL_TXFNUM_MASK       (15 << OTG_DIEPCTL_TXFNUM_SHIFT)
#define OTG_DIEPCTL_CNAK              (1 << 26) /* Bit 26: Clear NAK */
#define OTG_DIEPCTL_SNAK              (1 << 27) /* Bit 27: Set NAK */
#define OTG_DIEPCTL_SD0PID            (1 << 28) /* Bit 28: Set DATA0 PID (interrupt/bulk) */
#define OTG_DIEPCTL_SEVNFRM           (1 << 28) /* Bit 28: Set even frame (isochronous)) */
#define OTG_DIEPCTL_SODDFRM           (1 << 29) /* Bit 29: Set odd frame (isochronous) */
#define OTG_DIEPCTL_EPDIS             (1 << 30) /* Bit 30: Endpoint disable */
#define OTG_DIEPCTL_EPENA             (1 << 31) /* Bit 31: Endpoint enable */

/* Device endpoint-n interrupt register */

#define OTG_DIEPINT_XFRC              (1 << 0)  /* Bit 0:  Transfer completed interrupt */
#define OTG_DIEPINT_EPDISD            (1 << 1)  /* Bit 1:  Endpoint disabled interrupt */
                                                /* Bit 2:  Reserved, must be kept at reset value */
#define OTG_DIEPINT_TOC               (1 << 3)  /* Bit 3:  Timeout condition */
#define OTG_DIEPINT_ITTXFE            (1 << 4)  /* Bit 4:  IN token received when TxFIFO is empty */
                                                /* Bit 5:  Reserved, must be kept at reset value */
#define OTG_DIEPINT_INEPNE            (1 << 6)  /* Bit 6:  IN endpoint NAK effective */
#define OTG_DIEPINT_TXFE              (1 << 7)  /* Bit 7:  Transmit FIFO empty */
                                                /* Bits 8-31: Reserved, must be kept at reset value */
/* Device IN endpoint 0 transfer size register */

#define OTG_DIEPTSIZ0_XFRSIZ_SHIFT    (0)       /* Bits 0-6: Transfer size */
#define OTG_DIEPTSIZ0_XFRSIZ_MASK     (0x7f << OTG_DIEPTSIZ0_XFRSIZ_SHIFT)
                                                /* Bits 7-18: Reserved, must be kept at reset value */
#define OTG_DIEPTSIZ0_PKTCNT_SHIFT    (19)      /* Bits 19-20: Packet count */
#define OTG_DIEPTSIZ0_PKTCNT_MASK     (3 << OTG_DIEPTSIZ0_PKTCNT_SHIFT)
                                                /* Bits 21-31: Reserved, must be kept at reset value */
/* Device IN endpoint n transfer size register */

#define OTG_DIEPTSIZ_XFRSIZ_SHIFT     (0)       /* Bits 0-18: Transfer size */
#define OTG_DIEPTSIZ_XFRSIZ_MASK      (0x7ffff << OTG_DIEPTSIZ_XFRSIZ_SHIFT)
#define OTG_DIEPTSIZ_PKTCNT_SHIFT     (19)      /* Bit 19-28: Packet count */
#define OTG_DIEPTSIZ_PKTCNT_MASK      (0x3ff << OTG_DIEPTSIZ_PKTCNT_SHIFT)
#define OTG_DIEPTSIZ_MCNT_SHIFT       (29)      /* Bits 29-30: Multi count */
#define OTG_DIEPTSIZ_MCNT_MASK        (3 << OTG_DIEPTSIZ_MCNT_SHIFT)
                                                /* Bit 31: Reserved, must be kept at reset value */
/* Device OUT endpoint TxFIFO status register */

#define OTG_DTXFSTS_MASK              (0xffff)

/* Device OUT endpoint 0 control register */

#define OTG_DOEPCTL0_MPSIZ_SHIFT      (0)       /* Bits 0-1: Maximum packet size */
#define OTG_DOEPCTL0_MPSIZ_MASK       (3 << OTG_DOEPCTL0_MPSIZ_SHIFT)
#  define OTG_DOEPCTL0_MPSIZ_64       (0 << OTG_DOEPCTL0_MPSIZ_SHIFT) /* 64 bytes */
#  define OTG_DOEPCTL0_MPSIZ_32       (1 << OTG_DOEPCTL0_MPSIZ_SHIFT) /* 32 bytes */
#  define OTG_DOEPCTL0_MPSIZ_16       (2 << OTG_DOEPCTL0_MPSIZ_SHIFT) /* 16 bytes */
#  define OTG_DOEPCTL0_MPSIZ_8        (3 << OTG_DOEPCTL0_MPSIZ_SHIFT) /* 8 bytes */
                                                /* Bits 2-14: Reserved, must be kept at reset value */
#define OTG_DOEPCTL0_USBAEP           (1 << 15) /* Bit 15: USB active endpoint */
                                                /* Bit 16: Reserved, must be kept at reset value */
#define OTG_DOEPCTL0_NAKSTS           (1 << 17) /* Bit 17: NAK status */
#define OTG_DOEPCTL0_EPTYP_SHIFT      (18)      /* Bits 18-19: Endpoint type */
#define OTG_DOEPCTL0_EPTYP_MASK       (3 << OTG_DOEPCTL0_EPTYP_SHIFT)
#  define OTG_DOEPCTL0_EPTYP_CTRL     (0 << OTG_DOEPCTL0_EPTYP_SHIFT) /* Control (hard-coded) */
#define OTG_DOEPCTL0_SNPM             (1 << 20) /* Bit 20: Snoop mode */
#define OTG_DOEPCTL0_STALL            (1 << 21) /* Bit 21: STALL handshake */
                                                /* Bits 22-25: Reserved, must be kept at reset value */
#define OTG_DOEPCTL0_CNAK             (1 << 26) /* Bit 26: Clear NAK */
#define OTG_DOEPCTL0_SNAK             (1 << 27) /* Bit 27: Set NAK */
                                                /* Bits 28-29: Reserved, must be kept at reset value */
#define OTG_DOEPCTL0_EPDIS            (1 << 30) /* Bit 30: Endpoint disable */
#define OTG_DOEPCTL0_EPENA            (1 << 31) /* Bit 31: Endpoint enable */

/* Device OUT endpoint n control register */

#define OTG_DOEPCTL_MPSIZ_SHIFT       (0)       /* Bits 0-10: Maximum packet size */
#define OTG_DOEPCTL_MPSIZ_MASK        (0x7ff << OTG_DOEPCTL_MPSIZ_SHIFT)
                                                /* Bits 11-14: Reserved, must be kept at reset value */
#define OTG_DOEPCTL_USBAEP            (1 << 15) /* Bit 15: USB active endpoint */
#define OTG_DOEPCTL_DPID              (1 << 16) /* Bit 16: Endpoint data PID (interrupt/bulk) */
#  define OTG_DOEPCTL_DATA0           (0)
#  define OTG_DOEPCTL_DATA1           OTG_DOEPCTL_DPID
#define OTG_DOEPCTL_EONUM             (1 << 16) /* Bit 16: Even/odd frame (isochronous) */
#  define OTG_DOEPCTL_EVEN            (0)
#  define OTG_DOEPCTL_ODD             OTG_DOEPCTL_EONUM
#define OTG_DOEPCTL_NAKSTS            (1 << 17) /* Bit 17: NAK status */
#define OTG_DOEPCTL_EPTYP_SHIFT       (18)      /* Bits 18-19: Endpoint type */
#define OTG_DOEPCTL_EPTYP_MASK        (3 << OTG_DOEPCTL_EPTYP_SHIFT)
#  define OTG_DOEPCTL_EPTYP_CTRL      (0 << OTG_DOEPCTL_EPTYP_SHIFT) /* Control */
#  define OTG_DOEPCTL_EPTYP_ISOC      (1 << OTG_DOEPCTL_EPTYP_SHIFT) /* Isochronous */
#  define OTG_DOEPCTL_EPTYP_BULK      (2 << OTG_DOEPCTL_EPTYP_SHIFT) /* Bulk */
#  define OTG_DOEPCTL_EPTYP_INTR      (3 << OTG_DOEPCTL_EPTYP_SHIFT) /* Interrupt */
#define OTG_DOEPCTL_SNPM              (1 << 20) /* Bit 20: Snoop mode */
#define OTG_DOEPCTL_STALL             (1 << 21) /* Bit 21: STALL handshake */
                                                /* Bits 22-25: Reserved, must be kept at reset value */
#define OTG_DOEPCTL_CNAK              (1 << 26) /* Bit 26: Clear NAK */
#define OTG_DOEPCTL_SNAK              (1 << 27) /* Bit 27: Set NAK */
#define OTG_DOEPCTL_SD0PID            (1 << 28) /* Bit 28: Set DATA0 PID (interrupt/bulk) */
#define OTG_DOEPCTL_SEVNFRM           (1 << 28) /* Bit 28: Set even frame (isochronous) */
#define OTG_DOEPCTL_SD1PID            (1 << 29) /* Bit 29: Set DATA1 PID (interrupt/bulk) */
#define OTG_DOEPCTL_SODDFRM           (1 << 29) /* Bit 29: Set odd frame (isochronous */
#define OTG_DOEPCTL_EPDIS             (1 << 30) /* Bit 30: Endpoint disable */
#define OTG_DOEPCTL_EPENA             (1 << 31) /* Bit 31: Endpoint enable */

/* Device endpoint-n interrupt register */

#define OTG_DOEPINT_XFRC              (1 << 0)  /* Bit 0: Transfer completed interrupt */
#define OTG_DOEPINT_EPDISD            (1 << 1)  /* Bit 1: Endpoint disabled interrupt */
                                                /* Bit 2: Reserved, must be kept at reset value */
#define OTG_DOEPINT_SETUP             (1 << 3)  /* Bit 3: SETUP phase done */
#define OTG_DOEPINT_OTEPDIS           (1 << 4)  /* Bit 4: OUT token received when endpoint disabled */
                                                /* Bit 5: Reserved, must be kept at reset value */
#define OTG_DOEPINT_B2BSTUP           (1 << 6)  /* Bit 6: Back-to-back SETUP packets received */
                                                /* Bits 7-31: Reserved, must be kept at reset value */
/* Device OUT endpoint-0 transfer size register */

#define OTG_DOEPTSIZ0_XFRSIZ_SHIFT    (0)       /* Bits 0-6: Transfer size */
#define OTG_DOEPTSIZ0_XFRSIZ_MASK     (0x7f << OTG_DOEPTSIZ0_XFRSIZ_SHIFT)
                                                /* Bits 7-18: Reserved, must be kept at reset value */
#define OTG_DOEPTSIZ0_PKTCNT          (1 << 19) /* Bit 19 PKTCNT: Packet count */
                                                /* Bits 20-28: Reserved, must be kept at reset value */
#define OTG_DOEPTSIZ0_STUPCNT_SHIFT   (29)      /* Bits 29-30: SETUP packet count */
#define OTG_DOEPTSIZ0_STUPCNT_MASK    (3 << OTG_DOEPTSIZ0_STUPCNT_SHIFT)
                                                /* Bit 31: Reserved, must be kept at reset value */
/* Device OUT endpoint-n transfer size register */

#define OTG_DOEPTSIZ_XFRSIZ_SHIFT     (0)       /* Bits 0-18: Transfer size */
#define OTG_DOEPTSIZ_XFRSIZ_MASK      (0x7ffff << OTG_DOEPTSIZ_XFRSIZ_SHIFT)
#define OTG_DOEPTSIZ_PKTCNT_SHIFT     (19)      /* Bit 19-28: Packet count */
#define OTG_DOEPTSIZ_PKTCNT_MASK      (0x3ff << OTG_DOEPTSIZ_PKTCNT_SHIFT)
#define OTG_DOEPTSIZ_STUPCNT_SHIFT    (29)      /* Bits 29-30: SETUP packet count */
#define OTG_DOEPTSIZ_STUPCNT_MASK     (3 << OTG_DOEPTSIZ_STUPCNT_SHIFT)
#define OTG_DOEPTSIZ_RXDPID_SHIFT     (29)      /* Bits 29-30: Received data PID */
#define OTG_DOEPTSIZ_RXDPID_MASK      (3 << OTG_DOEPTSIZ_RXDPID_SHIFT)
#  define OTG_DOEPTSIZ_RXDPID_DATA0   (0 << OTG_DOEPTSIZ_RXDPID_SHIFT)
#  define OTG_DOEPTSIZ_RXDPID_DATA2   (1 << OTG_DOEPTSIZ_RXDPID_SHIFT)
#  define OTG_DOEPTSIZ_RXDPID_DATA1   (2 << OTG_DOEPTSIZ_RXDPID_SHIFT)
#  define OTG_DOEPTSIZ_RXDPID_MDATA   (3 << OTG_DOEPTSIZ_RXDPID_SHIFT)
                                                  /* Bit 31: Reserved, must be kept at reset value */
/* Power and clock gating control register */

#define OTG_PCGCCTL_STPPCLK           (1 << 0)  /* Bit 0: Stop PHY clock */
#define OTG_PCGCCTL_GATEHCLK          (1 << 1)  /* Bit 1: Gate HCLK */
                                                /* Bits 2-3: Reserved, must be kept at reset value */
#define OTG_PCGCCTL_PHYSUSP           (1 << 4)  /* Bit 4: PHY Suspended */
                                                /* Bits 5-31: Reserved, must be kept at reset value */

/* USB PHYC Registers */
/* USBPHYC PLL1 control register */

#define USBPHYC_PLL1_EN                 (1 << 0)  /* Bit 0: Enable the PLL1 inside PHY */
#define USBPHYC_PLL1_SEL_SHIFT          (1)       /* Bits 1-3: Controls the PHY PLL1 input clock frequency selection */
#define USBPHYC_PLL1_SEL_MASK           (7 << USBPHYC_PLL1SEL_SHIFT)
#  define USBPHYC_PLL1_SEL_12MHz        (0 << USBPHYC_PLL1_SEL_SHIFT)  /* 12 MHz */
#  define USBPHYC_PLL1_SEL_12_5MHz      (1 << USBPHYC_PLL1_SEL_SHIFT)  /* 12.5 MHz */
#  define USBPHYC_PLL1_SEL_12_5bMHz     (2 << USBPHYC_PLL1_SEL_SHIFT)  /* 12.5 MHz */
#  define USBPHYC_PLL1_SEL_16MHz        (3 << USBPHYC_PLL1_SEL_SHIFT)  /* 16 MHz */
#  define USBPHYC_PLL1_SEL_24MHz        (4 << USBPHYC_PLL1_SEL_SHIFT)  /* 24 MHz */
#  define USBPHYC_PLL1_SEL_25MHz        (5 << USBPHYC_PLL1_SEL_SHIFT)  /* 25 MHz */
#  define USBPHYC_PLL1_SEL_25bMHz       (6 << USBPHYC_PLL1_SEL_SHIFT)  /* 25 MHz */

/* USBPHYC tuning control register */

#define USBPHYC_TUNE_INCURREN           (1 << 0)  /* Bit 0: Controls the current boosting function */
#define USBPHYC_TUNE_INCURRINT          (1 << 1)  /* Bit 1: Controls PHY current boosting */
#define USBPHYC_TUNE_LFSCAPEN           (1 << 2)  /* Bit 2: Enables the Low Full Speed feedback capacitor */
#define USBPHYC_TUNE_HSDRVSLEW          (1 << 3)  /* Bit 3: Controls the HS driver slew rate */
#define USBPHYC_TUNE_HSDRVDCLEV         (1 << 4)  /* Bit 4: Decreases the HS driver DC level */
#define USBPHYC_TUNE_HSDRVDCCUR         (1 << 5)  /* Bit 5: Increases the HS Driver DC level. Not applicable
                                                   *        during the HS Test J and Test K data transfer */
#define USBPHYC_TUNE_HSDRVCURINCR       (1 << 6)  /* Bit 6: Enable the HS driver current increase feature */
#define USBPHYC_TUNE_FSDRVRFADJ         (1 << 7)  /* Bit 7: Tuning pin to adjust the full speed rise/fall time */
#define USBPHYC_TUNE_HSDRVRFRED         (1 << 8)  /* Bit 8: High Speed rise-fall reduction enable */
#define USBPHYC_TUNE_HSDRVCHKITRM_SHIFT (9)       /* Bits 9-12 : HS Driver current trimming pins for choke compensation */
#define USBPHYC_TUNE_HSDRVCHKITRM_MASK  (0xF << USBPHYC_TUNE_HSDRVCHKITRM_SHIFT)
#define USBPHYC_TUNE_HSDRVCHKZTRM_SHIFT (13)       /* Bits 13-14 : Controls the PHY bus HS driver impedance tuning for choke  */
#define USBPHYC_TUNE_HSDRVCHKZTRM_MASK  (3 << USBPHYC_TUNE_HSDRVCHKZTRM_SHIFT)
#define USBPHYC_TUNE_SQLCHCTL_SHIFT     (15)       /* Bits 15-16 :  Adjust the squelch DC threshold value */
#define USBPHYC_TUNE_SQLCHCTL_MASK      (3 << USBPHYC_TUNE_SQLCHCTL_SHIFT)
#define USBPHYC_TUNE_HDRXGNEQEN         (1 << 17)  /* Bit 17: Enables the HS Rx Gain Equalizer */
#define USBPHYC_TUNE_STAGSEL            (1 << 18)  /* Bit 18: HS Tx staggering enable */
#define USBPHYC_TUNE_HSFALLPREEM        (1 << 19)  /* Bit 19: HS Fall time control of single ended signals during pre-emphasis */
#define USBPHYC_TUNE_HSRXOFF_SHIFT      (20)       /* Bits 20-21 : HS Receiver Offset adjustment  */
#define USBPHYC_TUNE_HSRXOFF_MASK       (3 << USBPHYC_TUNE_HSRXOFF_SHIFT)
#define USBPHYC_TUNE_SHTCCTCTLPROT      (1 << 22)  /* Bit 22: Enables the short circuit protection circuitry in LS/FS driver */
#define USBPHYC_TUNE_SQLBYP             (1 << 23)  /* Bit 23: This pin is used to bypass the squelch inter-locking circuitry */

/* USBPHYC LDO control and status register */

#define USBPHYC_LDO_USED                (1 << 0)  /* Bit 0: Indicates the presence of the LDO in the chip. */
#define USBPHYC_LDO_STATUS              (1 << 1)  /* Bit 1: Monitors the status of the PHY's LDO. */

/* It looks like they made a mistake, is it ENABLE or DISABLE? */

#define USBPHYC_LDO_ENABLE             (1 << 2)  /* Bit 2: Controls disable of the High Speed PHY's LDO. */

#endif /* __ARCH_ARM_SRC_STM32F7_HARDWARE_STM32_OTG_H */
