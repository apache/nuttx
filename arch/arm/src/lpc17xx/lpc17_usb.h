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
/* OTG registers */
/* OTG Interrupt Status, OTG Interrupt Enable, OTG Interrupt Set, AND OTG Interrupt
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

/* USB Device Interrupt Status */
#define USBDEV_INTST_
/* USB Device Interrupt Enable */
#define USBDEV_INTEN_
/* USB Device Interrupt Clear */
#define USBDEV_INTCLR_
/* USB Device Interrupt Set */
#define USBDEV_INTSET_

/* SIE Command registers */

/* USB Command Code */
#define USBDEV_CMDCODE_
/* USB Command Data */
#define USBDEV_CMDDATA_

/* USB transfer registers */

/* USB Receive Data */
#define USBDEV_RXDATA_
/* USB Receive Packet Length */
#define USBDEV_RXPLEN_
/* USB Transmit Data */
#define USBDEV_TXDATA_
/* USB Transmit Packet Length */
#define USBDEV_TXPLEN_
/* USB Control */
#define USBDEV_CTRL_

/* More Device interrupt registers */

/* USB Device Interrupt Priority */
#define USBDEV_INTPRI_

/* Endpoint interrupt registers */

/* USB Endpoint Interrupt Status */
#define USBDEV_EPINTST_
/* USB Endpoint Interrupt Enable */
#define USBDEV_EPINTEN_
/* USB Endpoint Interrupt Clear */
#define USBDEV_EPINTCLR_
/* USB Endpoint Interrupt Set */
#define USBDEV_EPINTSET_
/* USB Endpoint Priority */
#define USBDEV_EPINTPRI_

/* Endpoint realization registers */

/* USB Realize Endpoint */
#define USBDEV_REEP_
/* USB Endpoint Index */
#define USBDEV_EPIND_
/* USB MaxPacketSize */
#define USBDEV_MAXPSIZE_

/* DMA registers */

/* USB DMA Request Status */
#define USBDEV_DMARST_
/* USB DMA Request Clear */
#define USBDEV_DMARCLR_
/* USB DMA Request Set */
#define USBDEV_DMARSET_
/* USB UDCA Head */
#define USBDEV_UDCAH_
/* USB Endpoint DMA Status */
#define USBDEV_EPDMAST_
/* USB Endpoint DMA Enable */
#define USBDEV_EPDMAEN_
/* USB Endpoint DMA Disable */
#define USBDEV_EPDMADIS_
/* USB DMA Interrupt Status */
#define USBDEV_DMAINTST_
/* USB DMA Interrupt Enable */
#define USBDEV_DMAINTEN_
/* USB End of Transfer Interrupt Status */
#define USBDEV_EOTINTST_
/* USB End of Transfer Interrupt Clear */
#define USBDEV_EOTINTCLR_
/* USB End of Transfer Interrupt Set */
#define USBDEV_EOTINTSET_
/* USB New DD Request Interrupt Status */
#define USBDEV_NDDRINTST_
/* USB New DD Request Interrupt Clear */
#define USBDEV_NDDRINTCLR_
/* USB New DD Request Interrupt Set */
#define USBDEV_NDDRINTSET_
/* USB System Error Interrupt Status */
#define USBDEV_SYSERRINTST_
/* USB System Error Interrupt Clear */
#define USBDEV_SYSERRINTCLR_
/* USB System Error Interrupt Set */
#define USBDEV_SYSERRINTSET_

/* OTG I2C registers ****************************************************************/

/* I2C Receive */
#define OTGI2C_RX_
/* I2C Transmit */
#define OTGI2C_TX_
/* I2C Status */
#define OTGI2C_STS_
/* I2C Control */
#define OTGI2C_CTL_
/* I2C Clock High */
#define OTGI2C_CLKHI_
/* I2C Clock Low */
#define OTGI2C_CLKLO_

/* Clock control registers ***********************************************************/

/* OTG clock controller */
#define USBOTG_CLKCTRL_
/* OTG clock status */
#define USBOTG_CLKST_

/* USB Clock Control */
#define USBDEV_CLKCTRL_
/* USB Clock Status */
#define USBDEV_CLKST_

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
