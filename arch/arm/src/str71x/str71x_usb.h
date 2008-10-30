/************************************************************************************
 * arch/arm/src/str71x/str71x_usb.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_USB_H
#define __ARCH_ARM_SRC_STR71X_STR71X_USB_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include "str71x_map.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* USB registers ********************************************************************/

#define STR71X_USB_NENDPNTS     (16)
#define STR71X_USB_EPR(ep)      (STR71X_USB_BASE + ((ep) << 4))
#define STR71X_USB_EP0R         (STR71X_USB_BASE + 0x0000)  /* Endpoint 0 */
#define STR71X_USB_EP1R         (STR71X_USB_BASE + 0x0004)  /* Endpoint 1 */
#define STR71X_USB_EP2R         (STR71X_USB_BASE + 0x0008)  /* Endpoint 2 */
#define STR71X_USB_EP3R         (STR71X_USB_BASE + 0x000c)  /* Endpoint 3 */
#define STR71X_USB_EP4R         (STR71X_USB_BASE + 0x0010)  /* Endpoint 4 */
#define STR71X_USB_EP5R         (STR71X_USB_BASE + 0x0014)  /* Endpoint 5 */
#define STR71X_USB_EP6R         (STR71X_USB_BASE + 0x0018)  /* Endpoint 6 */
#define STR71X_USB_EP7R         (STR71X_USB_BASE + 0x001c)  /* Endpoint 7 */
#define STR71X_USB_EP8R         (STR71X_USB_BASE + 0x0020)  /* Endpoint 8 */
#define STR71X_USB_EP9R         (STR71X_USB_BASE + 0x0024)  /* Endpoint 9 */
#define STR71X_USB_EP10R        (STR71X_USB_BASE + 0x0028)  /* Endpoint 10 */
#define STR71X_USB_EP11R        (STR71X_USB_BASE + 0x002c)  /* Endpoint 11 */
#define STR71X_USB_EP12R        (STR71X_USB_BASE + 0x0030)  /* Endpoint 12 */
#define STR71X_USB_EP13R        (STR71X_USB_BASE + 0x0034)  /* Endpoint 13 */
#define STR71X_USB_EP14R        (STR71X_USB_BASE + 0x0038)  /* Endpoint 14 */
#define STR71X_USB_EP15R        (STR71X_USB_BASE + 0x003c)  /* Endpoint 15 */
#define STR71X_USB_CNTR         (STR71X_USB_BASE + 0x0040)  /* Control register */
#define STR71X_USB_ISTR         (STR71X_USB_BASE + 0x0044)  /* Interrupt status register */
#define STR71X_USB_FNR          (STR71X_USB_BASE + 0x0048)  /* Frame number register */
#define STR71X_USB_DADDR        (STR71X_USB_BASE + 0x004C)  /* Device address register */
#define STR71X_USB_BTABLE       (STR71X_USB_BASE + 0x0050)  /* Buffer Table address register */

/* Register bit settings ***********************************************************/

/* Interrupt status register (ISTR) */

#define STR71X_USBISTR_CTR      (0x8000)  /* Correct Transfer */
#define STR71X_USBISTR_DOVR     (0x4000)  /* DMA Over/underrun */
#define STR71X_USBISTR_ERR      (0x2000)  /* Error */
#define STR71X_USBISTR_WKUP     (0x1000)  /* Wakeup */
#define STR71X_USBISTR_SUSP     (0x0800)  /* Suspend */
#define STR71X_USBISTR_RESET    (0x0400)  /* Reset */
#define STR71X_USBISTR_SOF      (0x0200)  /* Start of frame */
#define STR71X_USBISTR_ESOF     (0x0100)  /* Expected start of frame */
#define STR71X_USBISTR_DIR      (0x0010)  /* DIRection of transaction  */
#define STR71X_USBISTR_EPID     (0x000f)  /* Endpoint IDentifier  */

#define STR71X_USBCLR_CTR       (~STR71X_USBISTR_CTR)
#define STR71X_USBCLR_DOVR      (~STR71X_USBISTR_DOVR)
#define STR71X_USBCLR_ERR       (~STR71X_USBISTR_ERR)
#define STR71X_USBCLR_WKUP      (~STR71X_USBISTR_WKUP)
#define STR71X_USBCLR_SUSP      (~STR71X_USBISTR_SUSP)
#define STR71X_USBCLR_RESET     (~STR71X_USBISTR_RESET)
#define STR71X_USBCLR_SOF       (~STR71X_USBISTR_SOF)
#define STR71X_USBCLR_ESOF      (~STR71X_USBISTR_ESOF)

/* Control Register (CNTR) */

#define STR71X_USBCNTR_CTRM     (0x8000)  /* Correct transfer */
#define STR71X_USBCNTR_DOVRM    (0x4000)  /* DMA over/underrun */
#define STR71X_USBCNTR_ERRM     (0x2000)  /* Error */
#define STR71X_USBCNTR_WKUPM    (0x1000)  /* Wake up */
#define STR71X_USBCNTR_SUSPM    (0x0800)  /* Suspend  */
#define STR71X_USBCNTR_RESETM   (0x0400)  /* Reset   */
#define STR71X_USBCNTR_SOFM     (0x0200)  /* Start of frame */
#define STR71X_USBCNTR_ESOFM    (0x0100)  /* Expected start of frame */
#define STR71X_USBCNTR_RESUME   (0x0010)  /* Resume request */
#define STR71X_USBCNTR_FSUSP    (0x0008)  /* Force suspend */
#define STR71X_USBCNTR_LPMODE   (0x0004)  /* Low-power mode  */
#define STR71X_USBCNTR_PDWN     (0x0002)  /* Power down */
#define STR71X_USBCNTR_FRES     (0x0001)  /* Force usb reset */

/* Frame number register (FNR) */

#define STR71X_USBFNR_RXDP      (0x8000)  /* Status of D+ data line */
#define STR71X_USBFNR_RXDM      (0x4000)  /* Status of D- data line */
#define STR71X_USBFNR_LCK       (0x2000)  /* Locked */
#define STR71X_USBFNR_LSOF      (0x1800)  /* Lost SOF */
#define STR71X_USBFNR_FN        (0x07ff)  /* Frame number */

/* Device address register (DADDR) */

#define STR71X_USBDADDR_EF      (0x80)
#define STR71X_USBDADDR_ADD     (0x7f)

/* Endpoint registers (EPR) */

#define STR71X_USBEPR_CTRRX     (0x8000) /* Endpoint correct transfer RX   */
#define STR71X_USBEPR_DTOGRX    (0x4000) /* Endpoint data toggle RX */
#define STR71X_USBEPR_RXSTAT    (0x3000) /* Endpoint RX status bit */
#define STR71X_USBEPR_SETUP     (0x0800) /* Endpoint setup */
#define STR71X_USBEPR_TFIELD    (0x0600) /* Endpoint type */
#define STR71X_USBEPR_KIND      (0x0100) /* Endpoint kind */
#define STR71X_USBEPR_CTRTX     (0x0080) /* Endpoint correct transfer TX */
#define STR71X_USBEPR_DTOGTX    (0x0040) /* Endpoint data toggle TX */
#define STR71X_USBEPR_TXSTAT    (0x0030) /* Endpoint TX status bit */
#define STR71X_USBEPR_ADDRFIELD (0x000f) /* Endpoint address */

/* Endpoint register mask (no toggle fields) */

#define STR71X_USBEPRREG_MASK   (STR71X_USBEPR_CTRRX|STR71X_USBEPR_SETUP|STR71X_USBEPR_TFIELD|\
                                 STR71X_USBEPR_KIND|STR71X_USBEPR_CTRTX|STR71X_USBEPR_ADDRFIELD)

/* EP_TYPE[1:0] Endpoint type */

#define STR71X_USBEPR_TYPEMASK   (0x0600) /* Endpoint type mask */
#define STR71X_USBEPR_BULK       (0x0000) /* Endpoint BULK */
#define STR71X_USBEPR_CONTROL    (0x0200) /* Endpoint CONTROL */
#define STR71X_USBEPR_ISOC       (0x0400) /* Endpoint ISOCHRONOUS */
#define STR71X_USBEPR_INTERRUPT  (0x0600) /* Endpoint INTERRUPT */
#define STR71X_USBEPR_TMASK      (~STR71X_USBEPR_TFIELD & STR71X_USBEPRREG_MASK)

/* EP_KIND Endpoint kind */

#define STR71X_USBEPR_KINDMASK    (~STR71X_USBEPR_KIND & STR71X_USBEPRREG_MASK)

/* STAT_TX[1:0] Status for TX transfer */

#define STR71X_USBEPR_TXDIS      (0x0000) /* Endpoint TX disabled */
#define STR71X_USBEPR_TXSTALL    (0x0010) /* Endpoint TX stalled */
#define STR71X_USBEPR_TXNAK      (0x0020) /* Endpoint TX NAKed */
#define STR71X_USBEPR_TXVALID    (0x0030) /* Endpoint TX valid */
#define STR71X_USBEPRTX_DTOG1    (0x0010) /* Endpoint TX data toggle bit1 */
#define STR71X_USBEPRTX_DTOG2    (0x0020) /* Endpoint TX data toggle bit2 */
#define STR71X_USBEPRTX_DTOGMASK (STR71X_USBEPR_TXSTAT|STR71X_USBEPRREG_MASK)

/* STAT_RX[1:0] Status for RX transfer */

#define STR71X_USBEPR_RXDIS      (0x0000) /* Endpoint RX disabled */
#define STR71X_USBEPR_RXSTALL    (0x1000) /* Endpoint RX stalled */
#define STR71X_USBEPR_RXNAK      (0x2000) /* Endpoint RX NAKed */
#define STR71X_USBEPR_RXVALID    (0x3000) /* Endpoint RX valid */
#define STR71X_USBEPR_RXDTOG1    (0x1000) /* Endpoint RX data toggle bit1 */
#define STR71X_USBEPR_RXDTOG2    (0x2000) /* Endpoint RX data toggle bit1 */
#define STR71X_USBEPR_RXDTOGMASK (EPRX_STAT|STR71X_USBEPRREG_MASK)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_USB_H */
