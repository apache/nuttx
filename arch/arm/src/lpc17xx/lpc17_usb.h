/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_usb.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_USB_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_USB_H

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
/* USB Host Controller (OHCI) *******************************************************/

#define LPC17_USBHOST_HCIREV_OFFSET      0x0000 /* HcRevision: Version of HCI specification */
#define LPC17_USBHOST_CTRL_OFFSET        0x0004 /* HcControl: HC control */
#define LPC17_USBHOST_CMDST_OFFSET       0x0008 /* HcCommandStatus: HC command status */
#define LPC17_USBHOST_INTST_OFFSET       0x000c /* HcInterruptStatus: HC interrupt status */
#define LPC17_USBHOST_INTEN_OFFSET       0x0010 /* HcInterruptEnable: HC interrupt enable */
#define LPC17_USBHOST_INTDIS_OFFSET      0x0014 /* HcInterruptDisable: HC interrupt disable */
#define LPC17_USBHOST_HCCA_OFFSET        0x0018 /* HcHCCA: HC communication area */
#define LPC17_USBHOST_IIED_OFFSET        0x001c /* HcPeriodCurrentED: Current isoc or int endpoint desc */
#define LPC17_USBHOST_CTRLHEADED_OFFSET  0x0020 /* HcControlHeadED: First EP desc in the control list */
#define LPC17_USBHOST_CTRLED_OFFSET      0x0024 /* HcControlCurrentED: Current EP desc in the control list */
#define LPC17_USBHOST_BULKHEADED_OFFSET  0x0028 /* HcBulkHeadED: First EP desc in the bulk list */
#define LPC17_USBHOST_BULKED_OFFSET      0x002c /* HcBulkCurrentED: Current EP desc in the bulk list */
#define LPC17_USBHOST_DONEHEAD_OFFSET    0x0030 /* HcDoneHead: Last transfer desc added to DONE queue */
#define LPC17_USBHOST_FMINT_OFFSET       0x0034 /* HcFmInterval: Bit time interval that would not cause overrun */
#define LPC17_USBHOST_FMREM_OFFSET       0x0038 /* HcFmRemaining: Bit time remaining in current frame */
#define LPC17_USBHOST_FMNO_OFFSET        0x003c /* HcFmNumber: Frame number counter */
#define LPC17_USBHOST_PERSTART_OFFSET    0x0040 /* HcPeriodicStart: Time to start processing periodic list */
#define LPC17_USBHOST_LSTHRES_OFFSET     0x0044 /* HcLSThreshold: Commit to transfer threshold */
#define LPC17_USBHOST_RHDESCA_OFFSET     0x0048 /* HcRhDescriptorA: Describes root hub (part A) */
#define LPC17_USBHOST_RHDESCB_OFFSET     0x004c /* HcRhDescriptorB: Describes root hub (part B) */
#define LPC17_USBHOST_RHSTATUS_OFFSET    0x0050 /* HcRhStatus: Root hub status */
#define LPC17_USBHOST_RHPORTST1_OFFSET   0x0054 /* HcRhPort1Status: Root hub port status 1 */
#define LPC17_USBHOST_RHPORTST2_OFFSET   0x0058 /* HcRhPort2Status: Root hub port status 2 */
#define LPC17_USBHOST_MODID_OFFSET       0x00fc /* Module ID/Revision ID */

/* USB OTG Controller ***************************************************************/
/* OTG registers */

#define LPC17_USBOTG_INTST_OFFSET        0x0100 /* OTG Interrupt Status */
#define LPC17_USBOTG_INTEN_OFFSET        0x0104 /* OTG Interrupt Enable */
#define LPC17_USBOTG_INTSET_OFFSET       0x0108 /* OTG Interrupt Set */
#define LPC17_USBOTG_INTCLR_OFFSET       0x010c /* OTG Interrupt Clear */
#define LPC17_USBOTG_STCTRL_OFFSET       0x0110 /* OTG Status and Control */
#define LPC17_USBOTG_TMR_OFFSET          0x0114 /* OTG Timer */

/* USB Device Controller ************************************************************/
/* Device interrupt registers.  See also SYSCON_USBINTST in lpc17_syscon.h */

#define LPC17_USBDEV_INTST_OFFSET        0x0200 /* USB Device Interrupt Status */
#define LPC17_USBDEV_INTEN_OFFSET        0x0204 /* USB Device Interrupt Enable */
#define LPC17_USBDEV_INTCLR_OFFSET       0x0208 /* USB Device Interrupt Clear */
#define LPC17_USBDEV_INTSET_OFFSET       0x020c /* USB Device Interrupt Set */

/* SIE Command registers */

#define LPC17_USBDEV_CMDCODE_OFFSET      0x0210 /* USB Command Code */
#define LPC17_USBDEV_CMDDATA_OFFSET      0x0214 /* USB Command Data */

/* USB transfer registers */

#define LPC17_USBDEV_RXDATA_OFFSET       0x0218 /* USB Receive Data */
#define LPC17_USBDEV_RXPLEN_OFFSET       0x0220 /* USB Receive Packet Length */
#define LPC17_USBDEV_TXDATA_OFFSET       0x021c /* USB Transmit Data */
#define LPC17_USBDEV_TXPLEN_OFFSET       0x0224 /* USB Transmit Packet Length */
#define LPC17_USBDEV_CTRL_OFFSET         0x0228 /* USB Control */

/* More Device interrupt registers */

#define LPC17_USBDEV_INTPRI_OFFSET       0x022c /* USB Device Interrupt Priority */

/* Endpoint interrupt registers */

#define LPC17_USBDEV_EPINTST_OFFSET      0x0230 /* USB Endpoint Interrupt Status */
#define LPC17_USBDEV_EPINTEN_OFFSET      0x0234 /* USB Endpoint Interrupt Enable */
#define LPC17_USBDEV_EPINTCLR_OFFSET     0x0238 /* USB Endpoint Interrupt Clear */
#define LPC17_USBDEV_EPINTSET_OFFSET     0x023c /* USB Endpoint Interrupt Set */
#define LPC17_USBDEV_EPINTPRI_OFFSET     0x0240 /* USB Endpoint Priority */

/* Endpoint realization registers */

#define LPC17_USBDEV_REEP_OFFSET         0x0244 /* USB Realize Endpoint */
#define LPC17_USBDEV_EPIND_OFFSET        0x0248 /* USB Endpoint Index */
#define LPC17_USBDEV_MAXPSIZE_OFFSET     0x024c /* USB MaxPacketSize */

/* DMA registers */

#define LPC17_USBDEV_DMARST_OFFSET       0x0250 /* USB DMA Request Status */
#define LPC17_USBDEV_DMARCLR_OFFSET      0x0254 /* USB DMA Request Clear */
#define LPC17_USBDEV_DMARSET_OFFSET      0x0258 /* USB DMA Request Set */
#define LPC17_USBDEV_UDCAH_OFFSET        0x0280 /* USB UDCA Head */
#define LPC17_USBDEV_EPDMAST_OFFSET      0x0284 /* USB Endpoint DMA Status */
#define LPC17_USBDEV_EPDMAEN_OFFSET      0x0288 /* USB Endpoint DMA Enable */
#define LPC17_USBDEV_EPDMADIS_OFFSET     0x028c /* USB Endpoint DMA Disable */
#define LPC17_USBDEV_DMAINTST_OFFSET     0x0290 /* USB DMA Interrupt Status */
#define LPC17_USBDEV_DMAINTEN_OFFSET     0x0294 /* USB DMA Interrupt Enable */
#define LPC17_USBDEV_EOTINTST_OFFSET     0x02a0 /* USB End of Transfer Interrupt Status */
#define LPC17_USBDEV_EOTINTCLR_OFFSET    0x02a4 /* USB End of Transfer Interrupt Clear */
#define LPC17_USBDEV_EOTINTSET_OFFSET    0x02a8 /* USB End of Transfer Interrupt Set */
#define LPC17_USBDEV_NDDRINTST_OFFSET    0x02ac /* USB New DD Request Interrupt Status */
#define LPC17_USBDEV_NDDRINTCLR_OFFSET   0x02b0 /* USB New DD Request Interrupt Clear */
#define LPC17_USBDEV_NDDRINTSET_OFFSET   0x02b4 /* USB New DD Request Interrupt Set */
#define LPC17_USBDEV_SYSERRINTST_OFFSET  0x02b8 /* USB System Error Interrupt Status */
#define LPC17_USBDEV_SYSERRINTCLR_OFFSET 0x02bc /* USB System Error Interrupt Clear */
#define LPC17_USBDEV_SYSERRINTSET_OFFSET 0x02c0 /* USB System Error Interrupt Set */

/* OTG I2C registers ****************************************************************/

#define LPC17_OTGI2C_RX_OFFSET           0x0300 /* I2C Receive */
#define LPC17_OTGI2C_TX_OFFSET           0x0300 /* I2C Transmit */
#define LPC17_OTGI2C_STS_OFFSET          0x0304 /* I2C Status */
#define LPC17_OTGI2C_CTL_OFFSET          0x0308 /* I2C Control */
#define LPC17_OTGI2C_CLKHI_OFFSET        0x030c /* I2C Clock High */
#define LPC17_OTGI2C_CLKLO_OFFSET        0x0310 /* I2C Clock Low */

/* Clock control registers ***********************************************************/

#define LPC17_USBOTG_CLKCTRL_OFFSET      0x0ff4 /* OTG clock controller */
#define LPC17_USBOTG_CLKST_OFFSET        0x0ff8 /* OTG clock status */

#define LPC17_USBDEV_CLKCTRL_OFFSET      0x0ff4 /* USB Clock Control */
#define LPC17_USBDEV_CLKST_OFFSET        0x0ff8 /* USB Clock Status */

/* Register addresses ***************************************************************/
/* USB Host Controller (OHCI) *******************************************************/

#define LPC17_USBHOST_HCIREV             (LPC17_USB_BASE+LPC17_USBHOST_HCIREV_OFFSET)
#define LPC17_USBHOST_CTRL               (LPC17_USB_BASE+LPC17_USBHOST_CTRL_OFFSET)
#define LPC17_USBHOST_CMDST              (LPC17_USB_BASE+LPC17_USBHOST_CMDST_OFFSET)
#define LPC17_USBHOST_INTST              (LPC17_USB_BASE+LPC17_USBHOST_INTST_OFFSET)
#define LPC17_USBHOST_INTEN              (LPC17_USB_BASE+LPC17_USBHOST_INTEN_OFFSET)
#define LPC17_USBHOST_INTDIS             (LPC17_USB_BASE+LPC17_USBHOST_INTDIS_OFFSET)
#define LPC17_USBHOST_HCCA               (LPC17_USB_BASE+LPC17_USBHOST_HCCA_OFFSET)
#define LPC17_USBHOST_IIED               (LPC17_USB_BASE+LPC17_USBHOST_IIED_OFFSET)
#define LPC17_USBHOST_CTRLHEADED         (LPC17_USB_BASE+LPC17_USBHOST_CTRLHEADED_OFFSET)
#define LPC17_USBHOST_CTRLED             (LPC17_USB_BASE+LPC17_USBHOST_CTRLED_OFFSET)
#define LPC17_USBHOST_BULKHEADED         (LPC17_USB_BASE+LPC17_USBHOST_BULKHEADED_OFFSET)
#define LPC17_USBHOST_BULKED             (LPC17_USB_BASE+LPC17_USBHOST_BULKED_OFFSET)
#define LPC17_USBHOST_DONEHEAD           (LPC17_USB_BASE+LPC17_USBHOST_DONEHEAD_OFFSET)
#define LPC17_USBHOST_FMINT              (LPC17_USB_BASE+LPC17_USBHOST_FMINT_OFFSET)
#define LPC17_USBHOST_FMREM              (LPC17_USB_BASE+LPC17_USBHOST_FMREM_OFFSET)
#define LPC17_USBHOST_FMNO               (LPC17_USB_BASE+LPC17_USBHOST_FMNO_OFFSET)
#define LPC17_USBHOST_PERSTART           (LPC17_USB_BASE+LPC17_USBHOST_PERSTART_OFFSET)
#define LPC17_USBHOST_LSTHRES            (LPC17_USB_BASE+LPC17_USBHOST_LSTHRES_OFFSET)
#define LPC17_USBHOST_RHDESCA            (LPC17_USB_BASE+LPC17_USBHOST_RHDESCA_OFFSET)
#define LPC17_USBHOST_RHDESCB            (LPC17_USB_BASE+LPC17_USBHOST_RHDESCB_OFFSET)
#define LPC17_USBHOST_RHSTATUS           (LPC17_USB_BASE+LPC17_USBHOST_RHSTATUS_OFFSET)
#define LPC17_USBHOST_RHPORTST1          (LPC17_USB_BASE+LPC17_USBHOST_RHPORTST1_OFFSET)
#define LPC17_USBHOST_RHPORTST2          (LPC17_USB_BASE+LPC17_USBHOST_RHPORTST2_OFFSET)
#define LPC17_USBHOST_MODID              (LPC17_USB_BASE+LPC17_USBHOST_MODID_OFFSET)

/* USB OTG Controller ***************************************************************/
/* OTG registers */

#define LPC17_USBOTG_INTST               (LPC17_USB_BASE+LPC17_USBOTG_INTST_OFFSET)
#define LPC17_USBOTG_INTEN               (LPC17_USB_BASE+LPC17_USBOTG_INTEN_OFFSET)
#define LPC17_USBOTG_INTSET              (LPC17_USB_BASE+LPC17_USBOTG_INTSET_OFFSET)
#define LPC17_USBOTG_INTCLR              (LPC17_USB_BASE+LPC17_USBOTG_INTCLR_OFFSET)
#define LPC17_USBOTG_STCTRL              (LPC17_USB_BASE+LPC17_USBOTG_STCTRL_OFFSET)
#define LPC17_USBOTG_TMR                 (LPC17_USB_BASE+LPC17_USBOTG_TMR_OFFSET)

/* USB Device Controller ************************************************************/
/* Device interrupt registers.  See also SYSCON_USBINTST in lpc17_syscon.h */

#define LPC17_USBDEV_INTST               (LPC17_USB_BASE+LPC17_USBDEV_INTST_OFFSET)
#define LPC17_USBDEV_INTEN               (LPC17_USB_BASE+LPC17_USBDEV_INTEN_OFFSET)
#define LPC17_USBDEV_INTCLR              (LPC17_USB_BASE+LPC17_USBDEV_INTCLR_OFFSET)
#define LPC17_USBDEV_INTSET              (LPC17_USB_BASE+LPC17_USBDEV_INTSET_OFFSET)

/* SIE Command registers */

#define LPC17_USBDEV_CMDCODE            (LPC17_USB_BASE+LPC17_USBDEV_CMDCODE_OFFSET)
#define LPC17_USBDEV_CMDDATA            (LPC17_USB_BASE+LPC17_USBDEV_CMDDATA_OFFSET)

/* USB transfer registers */

#define LPC17_USBDEV_RXDATA              (LPC17_USB_BASE+LPC17_USBDEV_RXDATA_OFFSET)
#define LPC17_USBDEV_RXPLEN              (LPC17_USB_BASE+LPC17_USBDEV_RXPLEN_OFFSET)
#define LPC17_USBDEV_TXDATA              (LPC17_USB_BASE+LPC17_USBDEV_TXDATA_OFFSET)
#define LPC17_USBDEV_TXPLEN              (LPC17_USB_BASE+LPC17_USBDEV_TXPLEN_OFFSET)
#define LPC17_USBDEV_CTRL                (LPC17_USB_BASE+LPC17_USBDEV_CTRL_OFFSET)

/* More Device interrupt registers */

#define LPC17_USBDEV_INTPRI              (LPC17_USB_BASE+LPC17_USBDEV_INTPRI_OFFSET)

/* Endpoint interrupt registers */

#define LPC17_USBDEV_EPINTST             (LPC17_USB_BASE+LPC17_USBDEV_EPINTST_OFFSET)
#define LPC17_USBDEV_EPINTEN             (LPC17_USB_BASE+LPC17_USBDEV_EPINTEN_OFFSET)
#define LPC17_USBDEV_EPINTCLR            (LPC17_USB_BASE+LPC17_USBDEV_EPINTCLR_OFFSET)
#define LPC17_USBDEV_EPINTSET            (LPC17_USB_BASE+LPC17_USBDEV_EPINTSET_OFFSET)
#define LPC17_USBDEV_EPINTPRI            (LPC17_USB_BASE+LPC17_USBDEV_EPINTPRI_OFFSET)

/* Endpoint realization registers */

#define LPC17_USBDEV_REEP                (LPC17_USB_BASE+LPC17_USBDEV_REEP_OFFSET)
#define LPC17_USBDEV_EPIND               (LPC17_USB_BASE+LPC17_USBDEV_EPIND_OFFSET)
#define LPC17_USBDEV_MAXPSIZE            (LPC17_USB_BASE+LPC17_USBDEV_MAXPSIZE_OFFSET)

/* DMA registers */

#define LPC17_USBDEV_DMARST              (LPC17_USB_BASE+LPC17_USBDEV_DMARST_OFFSET)
#define LPC17_USBDEV_DMARCLR             (LPC17_USB_BASE+LPC17_USBDEV_DMARCLR_OFFSET)
#define LPC17_USBDEV_DMARSET             (LPC17_USB_BASE+LPC17_USBDEV_DMARSET_OFFSET)
#define LPC17_USBDEV_UDCAH               (LPC17_USB_BASE+LPC17_USBDEV_UDCAH_OFFSET)
#define LPC17_USBDEV_EPDMAST             (LPC17_USB_BASE+LPC17_USBDEV_EPDMAST_OFFSET)
#define LPC17_USBDEV_EPDMAEN             (LPC17_USB_BASE+LPC17_USBDEV_EPDMAEN_OFFSET)
#define LPC17_USBDEV_EPDMADIS            (LPC17_USB_BASE+LPC17_USBDEV_EPDMADIS_OFFSET)
#define LPC17_USBDEV_DMAINTST            (LPC17_USB_BASE+LPC17_USBDEV_DMAINTST_OFFSET)
#define LPC17_USBDEV_DMAINTEN            (LPC17_USB_BASE+LPC17_USBDEV_DMAINTEN_OFFSET)
#define LPC17_USBDEV_EOTINTST            (LPC17_USB_BASE+LPC17_USBDEV_EOTINTST_OFFSET)
#define LPC17_USBDEV_EOTINTCLR           (LPC17_USB_BASE+LPC17_USBDEV_EOTINTCLR_OFFSET)
#define LPC17_USBDEV_EOTINTSET           (LPC17_USB_BASE+LPC17_USBDEV_EOTINTSET_OFFSET)
#define LPC17_USBDEV_NDDRINTST           (LPC17_USB_BASE+LPC17_USBDEV_NDDRINTST_OFFSET)
#define LPC17_USBDEV_NDDRINTCLR          (LPC17_USB_BASE+LPC17_USBDEV_NDDRINTCLR_OFFSET)
#define LPC17_USBDEV_NDDRINTSET          (LPC17_USB_BASE+LPC17_USBDEV_NDDRINTSET_OFFSET)
#define LPC17_USBDEV_SYSERRINTST         (LPC17_USB_BASE+LPC17_USBDEV_SYSERRINTST_OFFSET)
#define LPC17_USBDEV_SYSERRINTCLR        (LPC17_USB_BASE+LPC17_USBDEV_SYSERRINTCLR_OFFSET)
#define LPC17_USBDEV_SYSERRINTSET        (LPC17_USB_BASE+LPC17_USBDEV_SYSERRINTSET_OFFSET)

/* OTG I2C registers ****************************************************************/

#define LPC17_OTGI2C_RX                  (LPC17_USB_BASE+LPC17_OTGI2C_RX_OFFSET)
#define LPC17_OTGI2C_TX                  (LPC17_USB_BASE+LPC17_OTGI2C_TX_OFFSET)
#define LPC17_OTGI2C_STS                 (LPC17_USB_BASE+LPC17_OTGI2C_STS_OFFSET)
#define LPC17_OTGI2C_CTL                 (LPC17_USB_BASE+LPC17_OTGI2C_CTL_OFFSET)
#define LPC17_OTGI2C_CLKHI               (LPC17_USB_BASE+LPC17_OTGI2C_CLKHI_OFFSET)
#define LPC17_OTGI2C_CLKLO               (LPC17_USB_BASE+LPC17_OTGI2C_CLKLO_OFFSET)

/* Clock control registers ***********************************************************/

#define LPC17_USBOTG_CLKCTRL             (LPC17_USB_BASE+LPC17_USBOTG_CLKCTRL_OFFSET)
#define LPC17_USBOTG_CLKST               (LPC17_USB_BASE+LPC17_USBOTG_CLKST_OFFSET)

#define LPC17_USBDEV_CLKCTRL             (LPC17_USB_BASE+LPC17_USBDEV_CLKCTRL_OFFSET)
#define LPC17_USBDEV_CLKST               (LPC17_USB_BASE+LPC17_USBDEV_CLKST_OFFSET)

/* Register bit definitions *********************************************************/
/* USB Host Controller (OHCI) *******************************************************/
/* UM10360: "Refer to the OHCI specification document on the Compaq website for
 *           register definitions"
 */

/* USB OTG Controller ***************************************************************/
/* OTG registers:
 *
 * OTG Interrupt Status, OTG Interrupt Enable, OTG Interrupt Set, AND OTG Interrupt
 * Clear
 */

#define USBOTG_INT_TMR                   (1 << 0)  /* Bit 0:  Timer time-out */
#define USBOTG_INT_REMOVE_PU             (1 << 1)  /* Bit 1:  Remove pull-up */
#define USBOTG_INT_HNP_FAILURE           (1 << 2)  /* Bit 2:  HNP failed */
#define USBOTG_INT_HNP_SUCCESS           (1 << 3)  /* Bit 3:  HNP succeeded */
                                                   /* Bits 4-31: Reserved */
/* OTG Status and Control */

#define USBOTG_STCTRL_PORTFUNC_SHIFT     (0)       /* Bits 0-1: Controls port function */
#define USBOTG_STCTRL_PORTFUNC_MASK      (3 << USBOTG_STCTRL_PORTFUNC_SHIFT)
#  define USBOTG_STCTRL_PORTFUNC_HNPOK   (1 << USBOTG_STCTRL_PORTFUNC_SHIFT) /* HNP suceeded */

#define USBOTG_STCTRL_TMRSCALE_SHIFT     (0)       /* Bits 2-3: Timer scale selection */
#define USBOTG_STCTRL_TMRSCALE_MASK      (3 << USBOTG_STCTRL_TMR_SCALE_SHIFT)
#  define USBOTG_STCTRL_TMRSCALE_10US    (0 << USBOTG_STCTRL_TMR_SCALE_SHIFT) /* 10uS (100 KHz) */
#  define USBOTG_STCTRL_TMRSCALE_100US   (1 << USBOTG_STCTRL_TMR_SCALE_SHIFT) /* 100uS (10 KHz) */
#  define USBOTG_STCTRL_TMRSCALE_1000US  (2 << USBOTG_STCTRL_TMR_SCALE_SHIFT) /* 1000uS (1 KHz) */
#define USBOTG_STCTRL_TMRMODE            (1 << 4)  /* Bit 4:  Timer mode selection */
#define USBOTG_STCTRL_TMREN              (1 << 5)  /* Bit 5:  Timer enable */
#define USBOTG_STCTRL_TMRRST             (1 << 6)  /* Bit 6:  TTimer reset */
                                                   /* Bit 7:  Reserved */
#define USBOTG_STCTRL_BHNPTRACK          (1 << 8)  /* Bit 8:  Enable HNP tracking for B-device (peripheral) */
#define USBOTG_STCTRL_AHNPTRACK          (1 << 9)  /* Bit 9:  Enable HNP tracking for A-device (host) */
#define USBOTG_STCTRL_PUREMOVED          (1 << 10) /* Bit 10: Set when D+ pull-up removed */
                                                   /* Bits 11-15: Reserved */
#define USBOTG_STCTRL_TMRCNT_SHIFT       (0)       /* Bits 16-313: Timer scale selection */
#define USBOTG_STCTRL_TMRCNT_MASK        (0ffff << USBOTG_STCTRL_TMR_CNT_SHIFT)

/* OTG Timer */

#define USBOTG_TMR_TIMEOUTCNT_SHIFT      (0)       /* Bits 0-15: Interrupt when CNT matches this */
#define USBOTG_TMR_TIMEOUTCNT_MASK       (0xffff << USBOTG_TMR_TIMEOUTCNT_SHIFT)
                                                   /* Bits 16-31: Reserved */

/* USB Device Controller ************************************************************/
/* Device interrupt registers.  See also SYSCON_USBINTST in lpc17_syscon.h */
/* USB Device Interrupt Status, USB Device Interrupt Enable, USB Device Interrupt
 * Clear, USB Device Interrupt Set, and USB Device Interrupt Priority
 */
 
#define USBDEV_INT_FRAME                 (1 << 0)  /* Bit 0:  frame interrupt (every 1 ms) */
#define USBDEV_INT_EPFAST                (1 << 1)  /* Bit 1:  Fast endpoint interrupt */
#define USBDEV_INT_EPSLOW                (1 << 2)  /* Bit 2:  Slow endpoints interrupt */
#define USBDEV_INT_DEVSTAT               (1 << 3)  /* Bit 3:  Bus reset, suspend change or connect change */
#define USBDEV_INT_CCEMPTY               (1 << 4)  /* Bit 4:  Command code register empty */
#define USBDEV_INT_CDFULL                (1 << 5)  /* Bit 5:  Command data register full */
#define USBDEV_INT_RXENDPKT              (1 << 6)  /* Bit 6:  RX endpoint data transferred */
#define USBDEV_INT_TXENDPKT              (1 << 7)  /* Bit 7:  TX endpoint data tansferred */
#define USBDEV_INT_EPRLZED               (1 << 8)  /* Bit 8:  Endpoints realized */
#define USBDEV_INT_ERRINT                (1 << 9)  /* Bit 9:  Error Interrupt */
                                                   /* Bits 10-31: Reserved */
/* SIE Command registers:
 *
 * USB Command Code
 */
                                                   /* Bits 0-7: Reserved */
#define USBDEV_CMDCODE_PHASE_SHIFT       (8)       /* Bits 8-15: Command phase */
#define USBDEV_CMDCODE_PHASE_MASK        (0xff << USBDEV_CMDCODE_PHASE_SHIFT)
#  define USBDEV_CMDCODE_PHASE_READ      (1 << USBDEV_CMDCODE_PHASE_SHIFT)
#  define USBDEV_CMDCODE_PHASE_WRITE     (2 << USBDEV_CMDCODE_PHASE_SHIFT)
#  define USBDEV_CMDCODE_PHASE_COMMAND   (5 << USBDEV_CMDCODE_PHASE_SHIFT)
#define USBDEV_CMDCODE_CMD_SHIFT         (16)     /* Bits 15-23: Command (READ/COMMAND phases) */
#define USBDEV_CMDCODE_CMD_MASK          (0xff << USBDEV_CMDCODE_CMD_SHIFT)
#define USBDEV_CMDCODE_WDATA_SHIFT       (16)     /* Bits 15-23: Write dagta (WRITE phase) */
#define USBDEV_CMDCODE_WDATA_MASK        (0xff << USBDEV_CMDCODE_CMD_SHIFT)
                                                   /* Bits 24-31: Reserved */
/* USB Command Data */

#define USBDEV_CMDDATA_SHIFT             (0)       /* Bits 0-7: Command read data */
#define USBDEV_CMDDATA_MASK              (0xff << USBDEV_CMDDATA_SHIFT)
                                                   /* Bits 8-31: Reserved */

/* USB transfer registers:
 *
 * USB Receive Data (Bits 0-31: Received data)
 */

/* USB Receive Packet Length */

#define USBDEV_RXPLEN_SHIFT              (0)       /* Bits 0-9: Bytes remaining to be read */
#define USBDEV_RXPLEN_MASK               (0x3ff << USBDEV_RXPLEN_SHIFT)
#define USBDEV_RXPLEN_DV                 (1 << 10) /* Bit 10: DV Data valid*/
#define USBDEV_RXPLEN_PKTRDY             (1 << 11) /* Bit 11: Packet ready for reading */
                                                   /* Bits 12-31: Reserved */
/* USB Transmit Data (Bits 0-31: Transmit data) */

/* USB Transmit Packet Length */

#define USBDEV_TXPLEN_SHIFT              (0)       /* Bits 0-9: Bytes remaining to be written */
#define USBDEV_TXPLEN_MASK               (0x3ff << USBDEV_TXPLEN_SHIFT)
                                                   /* Bits 10-31: Reserved */
/* USB Control */

#define USBDEV_CTRL_RDEN                 (1 << 0)  /* Bit 0:  Read mode control */
#define USBDEV_CTRL_WREN                 (1 << 1)  /* Bit 1:  Write mode control */
#define USBDEV_CTRL_LOGENDPOINT_SHIFT    (2)       /* Bits 2-5: Logical Endpoint number */
#define USBDEV_CTRL_LOGENDPOINT_MASK     (15 << USBDEV_CTRL_LOGENDPOINT_SHIFT)
                                                   /* Bits 6-31: Reserved */
/* Endpoint interrupt registers:
 *
 * USB Endpoint Interrupt Status, USB Endpoint Interrupt Enable, USB Endpoint Interrupt
 * Clear, USB Endpoint Interrupt Set, and USB Endpoint Priority.  Bits correspond
 * to on RX or TX value for any of 15 logical endpoints).
 */

#define USBDEV_LOGEPRX(n)                (1 << ((n) << 1))
#define USBDEV_LOGEPTX(n)                ((1 << ((n) << 1)) + 1)
#define USBDEV_LOGEPRX0                  (1 << 0)
#define USBDEV_LOGEPTX0                  (1 << 1)
#define USBDEV_LOGEPRX1                  (1 << 2)
#define USBDEV_LOGEPTX1                  (1 << 3)
#define USBDEV_LOGEPRX2                  (1 << 4)
#define USBDEV_LOGEPTX2                  (1 << 5)
#define USBDEV_LOGEPRX3                  (1 << 6)
#define USBDEV_LOGEPTX3                  (1 << 7)
#define USBDEV_LOGEPRX4                  (1 << 8)
#define USBDEV_LOGEPTX4                  (1 << 9)
#define USBDEV_LOGEPRX5                  (1 << 10)
#define USBDEV_LOGEPTX5                  (1 << 11)
#define USBDEV_LOGEPRX6                  (1 << 12)
#define USBDEV_LOGEPTX6                  (1 << 13)
#define USBDEV_LOGEPRX7                  (1 << 14)
#define USBDEV_LOGEPTX7                  (1 << 15)
#define USBDEV_LOGEPRX8                  (1 << 16)
#define USBDEV_LOGEPTX8                  (1 << 17)
#define USBDEV_LOGEPRX9                  (1 << 18)
#define USBDEV_LOGEPTX9                  (1 << 19)
#define USBDEV_LOGEPRX10                 (1 << 20)
#define USBDEV_LOGEPTX10                 (1 << 21)
#define USBDEV_LOGEPRX11                 (1 << 22)
#define USBDEV_LOGEPTX11                 (1 << 23)
#define USBDEV_LOGEPRX12                 (1 << 24)
#define USBDEV_LOGEPTX12                 (1 << 25)
#define USBDEV_LOGEPRX13                 (1 << 26)
#define USBDEV_LOGEPTX13                 (1 << 27)
#define USBDEV_LOGEPRX14                 (1 << 28)
#define USBDEV_LOGEPTX14                 (1 << 29)
#define USBDEV_LOGEPRX15                 (1 << 30)
#define USBDEV_LOGEPTX15                 (1 << 31)

/* Endpoint realization registers:
 *
 * USB Realize Endpoint (Bits correspond to 1 of 32 physical endpoints)
 */

#define USBDEV_PHYEP(n)                  (1 << (n))
#define USBDEV_PHYEP0                    (1 << 0)
#define USBDEV_PHYEP0                    (1 << 1)
#define USBDEV_PHYEP0                    (1 << 2)
#define USBDEV_PHYEP0                    (1 << 3)
#define USBDEV_PHYEP0                    (1 << 4)
#define USBDEV_PHYEP0                    (1 << 5)
#define USBDEV_PHYEP0                    (1 << 6)
#define USBDEV_PHYEP0                    (1 << 7)
#define USBDEV_PHYEP0                    (1 << 8)
#define USBDEV_PHYEP0                    (1 << 9)
#define USBDEV_PHYEP10                   (1 << 10)
#define USBDEV_PHYEP11                   (1 << 11)
#define USBDEV_PHYEP12                   (1 << 12)
#define USBDEV_PHYEP13                   (1 << 13)
#define USBDEV_PHYEP14                   (1 << 14)
#define USBDEV_PHYEP15                   (1 << 15)
#define USBDEV_PHYEP16                   (1 << 16)
#define USBDEV_PHYEP17                   (1 << 17)
#define USBDEV_PHYEP18                   (1 << 18)
#define USBDEV_PHYEP19                   (1 << 19)
#define USBDEV_PHYEP20                   (1 << 20)
#define USBDEV_PHYEP21                   (1 << 21)
#define USBDEV_PHYEP22                   (1 << 22)
#define USBDEV_PHYEP23                   (1 << 23)
#define USBDEV_PHYEP24                   (1 << 24)
#define USBDEV_PHYEP25                   (1 << 25)
#define USBDEV_PHYEP26                   (1 << 26)
#define USBDEV_PHYEP27                   (1 << 27)
#define USBDEV_PHYEP28                   (1 << 28)
#define USBDEV_PHYEP29                   (1 << 29)
#define USBDEV_PHYEP30                   (1 << 30)
#define USBDEV_PHYEP31                   (1 << 31)

/* USB Endpoint Index */

#define USBDEV_EPIND_SHIFT               (0)       /* Bits 0-4: Physical endpoint number (0-31) */
#define USBDEV_EPIND_MASK                (31 << USBDEV_EPIND_SHIFT)
                                                   /* Bits 5-31: Reserved */

/* USB MaxPacketSize */

#define USBDEV_MAXPSIZE_SHIFT            (0)       /* Bits 0-9: Maximum packet size value */
#define USBDEV_MAXPSIZE_                 (0x3ff << USBDEV_MAXPSIZE_SHIFT)
                                                   /* Bits 10-31: Reserved */
/* DMA registers:
 *
 * USB DMA Request Status, USB DMA Request Clear, and USB DMA Request Set.  Registers
 * contain bits for each of 32 physical endpoints.  Use the USBDEV_PHYEP* definitions
 * above.  PHYEP0-1 (bits 0-1) must be zero.
 */

/* USB UDCA Head */
                                                  /* Bits 0-6: Reserved */
#define USBDEV_UDCAH_SHIFT               (7)      /* Bits 7-31: UDCA start address */
#define USBDEV_UDCAH_MASK                (0x01ffffff << USBDEV_UDCAH_SHIFT)

/* USB Endpoint DMA Status, USB Endpoint DMA Enable, and USB Endpoint DMA Disable.
 * Registers contain bits for physical endpoints 2-31. Use the USBDEV_PHYEP*
 * definitions above.  PHYEP0-1 (bits 0-1) must be zero.
 */

/* USB DMA Interrupt Status and USB DMA Interrupt Enable */

#define USBDEV_DMAINT_EOT                (1 << 0)  /* Bit 0:  End of Transfer Interrupt */
#define USBDEV_DMAINT_NDDR               (1 << 1)  /* Bit 1:  New DD Request Interrupt */
#define USBDEV_DMAINT_ERR                (1 << 2)  /* Bit 2:  System Error Interrupt */
                                                   /* Bits 3-31: Reserved */
/* USB End of Transfer Interrupt Status, USB End of Transfer Interrupt Clear, and USB
 * End of Transfer Interrupt Set.  Registers contain bits for physical endpoints 2-31.
 * Use the USBDEV_PHYEP* definitions above.  PHYEP0-1 (bits 0-1) must be zero.
 */

/* USB New DD Request Interrupt Status, USB New DD Request Interrupt Clear, and USB
 * New DD Request Interrupt Set.  Registers contain bits for physical endpoints 2-31.
 * Use the USBDEV_PHYEP* definitions above.  PHYEP0-1 (bits 0-1) must be zero.
 */

/* USB System Error Interrupt Status, USB System Error Interrupt Clear, USB System
 * Error Interrupt Set.  Registers contain bits for  physical endpoints 2-31.  Use
 * the USBDEV_PHYEP* definitions above.  PHYEP0-1 (bits 0-1) must be zero.
 */

/* OTG I2C registers ****************************************************************/

/* I2C Receive */

#define OTGI2C_RX_DATA_SHIFT             (0)       /* Bits 0-7: RX data */
#define OTGI2C_RX_DATA_MASK              (0xff << OTGI2C_RX_SHIFT)
                                                   /* Bits 8-31: Reserved */
/* I2C Transmit */

#define OTGI2C_TX_DATA_SHIFT             (0)       /* Bits 0-7: TX data */
#define OTGI2C_TX_DATA_MASK              (0xff << OTGI2C_TX_DATA_SHIFT)
#define OTGI2C_TX_DATA_START             (1 << 8)  /* Bit 8:  Issue START before transmit */
#define OTGI2C_TX_DATA_STOP              (1 << 9)  /* Bit 9:  Issue STOP before transmit */
                                                   /* Bits 3-31: Reserved */
/* I2C Status */

#define OTGI2C_STS_TDI                   (1 << 0)  /* Bit 0:  Transaction Done Interrupt */
#define OTGI2C_STS_AFI                   (1 << 1)  /* Bit 1:  Arbitration Failure Interrupt */
#define OTGI2C_STS_NAI                   (1 << 2)  /* Bit 2:  No Acknowledge Interrupt */
#define OTGI2C_STS_DRMI                  (1 << 3)  /* Bit 3:  Master Data Request Interrupt */
#define OTGI2C_STS_DRSI                  (1 << 4)  /* Bit 4:  Slave Data Request Interrupt */
#define OTGI2C_STS_ACTIVE                (1 << 5)  /* Bit 5:  Indicates whether the bus is busy */
#define OTGI2C_STS_SCL                   (1 << 6)  /* Bit 6:  The current value of the SCL signal */
#define OTGI2C_STS_SDA                   (1 << 7)  /* Bit 7:  The current value of the SDA signal */
#define OTGI2C_STS_RFF                   (1 << 8)  /* Bit 8:  Receive FIFO Full (RFF) */
#define OTGI2C_STS_RFE                   (1 << 9)  /* Bit 9:  Receive FIFO Empty */
#define OTGI2C_STS_TFF                   (1 << 10) /* Bit 10: Transmit FIFO Full */
#define OTGI2C_STS_TFE                   (1 << 11) /* Bit 11: Transmit FIFO Empty */
                                                   /* Bits 12-31: Reserved */
/* I2C Control */

#define OTGI2C_CTL_TDIE                  (1 << 0)  /* Bit 0:  Transmit Done Interrupt Enable */
#define OTGI2C_CTL_AFIE                  (1 << 1)  /* Bit 1:  Transmitter Arbitration Failure Interrupt Enable */
#define OTGI2C_CTL_NAIE                  (1 << 2)  /* Bit 2:  Transmitter No Acknowledge Interrupt Enable */
#define OTGI2C_CTL_DRMIE                 (1 << 3)  /* Bit 3:  Master Transmitter Data Request Interrupt Enable */
#define OTGI2C_CTL_DRSIE                 (1 << 4)  /* Bit 4:  Slave Transmitter Data Request Interrupt Enable */
#define OTGI2C_CTL_REFIE                 (1 << 5)  /* Bit 5:  Receive FIFO Full Interrupt Enable */
#define OTGI2C_CTL_RFDAIE                (1 << 6)  /* Bit 6:  Receive Data Available Interrupt Enable */
#define OTGI2C_CTL_TFFIE                 (1 << 7)  /* Bit 7:  Transmit FIFO Not Full Interrupt Enable */
#define OTGI2C_CTL_SRST                  (1 << 8)  /* Bit 8:  Soft reset */
                                                   /* Bits 9-31: Reserved */
/* I2C Clock High */

#define OTGI2C_CLKHI_SHIFT               (0)       /* Bits 0-7: Clock divisor high */
#define OTGI2C_CLKHI_MASK                (0xff << OTGI2C_CLKHI_SHIFT)
                                                   /* Bits 8-31: Reserved */
/* I2C Clock Low */

#define OTGI2C_CLKLO_SHIFT               (0)       /* Bits 0-7: Clock divisor high */
#define OTGI2C_CLLO_MASK                (0xff << OTGI2C_CLKLO_SHIFT)
                                                   /* Bits 8-31: Reserved */
/* Clock control registers ***********************************************************/

/* USB Clock Control (OTG clock controller) and USB Clock Status (OTG clock status) */

#define USBDEV_CLK_HOSTCLK               (1 << 0)  /* Bit 1:  Host clock (OTG only) */
#define USBDEV_CLK_DEVCLK                (1 << 1)  /* Bit 1:  Device clock */
#define USBDEV_CLK_I2CCLK                (1 << 2)  /* Bit 2:  I2C clock (OTG only) */
#define USBDEV_CLK_PORTSELCLK            (1 << 3)  /* Bit 3:  Port select register clock (device only) */
#define USBDEV_CLK_OTGCLK                (1 << 3)  /* Bit 3:  OTG clock (OTG only) */
#define USBDEV_CLK_AHBCLK                (1 << 4)  /* Bit 4:  AHB clock */
                                                   /* Bits 5-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_USB_H */
