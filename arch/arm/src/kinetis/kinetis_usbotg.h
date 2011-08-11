/********************************************************************************************
 * arch/arm/src/kinetis/kinetis_usbotg.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_USBOTG_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_USBOTG_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINETIS_USB_PERID_OFFSET    0x0000 /* Peripheral ID Register */
#define KINETIS_USB_IDCOMP_OFFSET   0x0004 /* Peripheral ID Complement Register */
#define KINETIS_USB_REV_OFFSET      0x0008 /* Peripheral Revision Register */
#define KINETIS_USB_ADDINFO_OFFSET  0x000c /* Peripheral Additional Info Register */
#define KINETIS_USB_OTGISTAT_OFFSET 0x0010 /* OTG Interrupt Status Register */
#define KINETIS_USB_OTGICR_OFFSET   0x0014 /* OTG Interrupt Control Register */
#define KINETIS_USB_OTGSTAT_OFFSET  0x0018 /* OTG Status Register */
#define KINETIS_USB_OTGCTL_OFFSET   0x001c /* OTG Control Register */
#define KINETIS_USB_ISTAT_OFFSET    0x0080 /* Interrupt Status Register */
#define KINETIS_USB_INTEN_OFFSET    0x0084 /* Interrupt Enable Register */
#define KINETIS_USB_ERRSTAT_OFFSET  0x0088 /* Error Interrupt Status Register */
#define KINETIS_USB_ERREN_OFFSET    0x008c /* Error Interrupt Enable Register */
#define KINETIS_USB_STAT_OFFSET     0x0090 /* Status Register */
#define KINETIS_USB_CTL_OFFSET      0x0094 /* Control Register */
#define KINETIS_USB_ADDR_OFFSET     0x0098 /* Address Register */
#define KINETIS_USB_BDTPAGE1_OFFSET 0x009c /* BDT Page Register 1 */
#define KINETIS_USB_FRMNUML_OFFSET  0x00a0 /* Frame Number Register Low */
#define KINETIS_USB_FRMNUMH_OFFSET  0x00a4 /* Frame Number Register High */
#define KINETIS_USB_TOKEN_OFFSET    0x00a8 /* Token Register */
#define KINETIS_USB_SOFTHLD_OFFSET  0x00ac /* SOF Threshold Register */
#define KINETIS_USB_BDTPAGE2_OFFSET 0x00b0 /* BDT Page Register 2 */
#define KINETIS_USB_BDTPAGE3_OFFSET 0x00b4 /* BDT Page Register 3 */

#define KINETIS_USB_ENDPT_OFFSET(n) (0x00c0+((n)<<2)) /* Endpoint n Control Register */
#define KINETIS_USB_ENDPT0_OFFSET   0x00c0 /* Endpoint 0 Control Register */
#define KINETIS_USB_ENDPT1_OFFSET   0x00c4 /* Endpoint 1 Control Register */
#define KINETIS_USB_ENDPT2_OFFSET   0x00c8 /* Endpoint 2 Control Register */
#define KINETIS_USB_ENDPT3_OFFSET   0x00cc /* Endpoint 3 Control Register */
#define KINETIS_USB_ENDPT4_OFFSET   0x00d0 /* Endpoint 4 Control Register */
#define KINETIS_USB_ENDPT5_OFFSET   0x00d4 /* Endpoint 5 Control Register */
#define KINETIS_USB_ENDPT6_OFFSET   0x00d8 /* Endpoint 6 Control Register */
#define KINETIS_USB_ENDPT7_OFFSET   0x00dc /* Endpoint 7 Control Register */
#define KINETIS_USB_ENDPT8_OFFSET   0x00e0 /* Endpoint 8 Control Register */
#define KINETIS_USB_ENDPT9_OFFSET   0x00e4 /* Endpoint 9 Control Register */
#define KINETIS_USB_ENDPT10_OFFSET  0x00e8 /* Endpoint 10 Control Register */
#define KINETIS_USB_ENDPT11_OFFSET  0x00ec /* Endpoint 11 Control Register */
#define KINETIS_USB_ENDPT12_OFFSET  0x00f0 /* Endpoint 12 Control Register */
#define KINETIS_USB_ENDPT13_OFFSET  0x00f4 /* Endpoint 13 Control Register */
#define KINETIS_USB_ENDPT14_OFFSET  0x00f8 /* Endpoint 14 Control Register */
#define KINETIS_USB_ENDPT15_OFFSET  0x00fc /* Endpoint 15 Control Register */

#define KINETIS_USB_USBCTRL_OFFSET  0x0100 /* USB Control Register */
#define KINETIS_USB_OBSERVE_OFFSET  0x0104 /* USB OTG Observe Register */
#define KINETIS_USB_CONTROL_OFFSET  0x0108 /* USB OTG Control Register */
#define KINETIS_USB_USBTRC0_OFFSET  0x010c /* USB Transceiver Control Register 0 */

/* Register Addresses ***********************************************************************/

#define KINETIS_USB0_PERID          (KINETIS_USB0_BASE+KINETIS_USB_PERID_OFFSET)
#define KINETIS_USB0_IDCOMP         (KINETIS_USB0_BASE+KINETIS_USB_IDCOMP_OFFSET)
#define KINETIS_USB0_REV            (KINETIS_USB0_BASE+KINETIS_USB_REV_OFFSET)
#define KINETIS_USB0_ADDINFO        (KINETIS_USB0_BASE+KINETIS_USB_ADDINFO_OFFSET)
#define KINETIS_USB0_OTGISTAT       (KINETIS_USB0_BASE+KINETIS_USB_OTGISTAT_OFFSET)
#define KINETIS_USB0_OTGICR         (KINETIS_USB0_BASE+KINETIS_USB_OTGICR_OFFSET)
#define KINETIS_USB0_OTGSTAT        (KINETIS_USB0_BASE+KINETIS_USB_OTGSTAT_OFFSET)
#define KINETIS_USB0_OTGCTL         (KINETIS_USB0_BASE+KINETIS_USB_OTGCTL_OFFSET)
#define KINETIS_USB0_ISTAT          (KINETIS_USB0_BASE+KINETIS_USB_ISTAT_OFFSET)
#define KINETIS_USB0_INTEN          (KINETIS_USB0_BASE+KINETIS_USB_INTEN_OFFSET)
#define KINETIS_USB0_ERRSTAT        (KINETIS_USB0_BASE+KINETIS_USB_ERRSTAT_OFFSET)
#define KINETIS_USB0_ERREN          (KINETIS_USB0_BASE+KINETIS_USB_ERREN_OFFSET)
#define KINETIS_USB0_STAT           (KINETIS_USB0_BASE+KINETIS_USB_STAT_OFFSET)
#define KINETIS_USB0_CTL            (KINETIS_USB0_BASE+KINETIS_USB_CTL_OFFSET)
#define KINETIS_USB0_ADDR           (KINETIS_USB0_BASE+KINETIS_USB_ADDR_OFFSET)
#define KINETIS_USB0_BDTPAGE1       (KINETIS_USB0_BASE+KINETIS_USB_BDTPAGE1_OFFSET)
#define KINETIS_USB0_FRMNUML        (KINETIS_USB0_BASE+KINETIS_USB_FRMNUML_OFFSET)
#define KINETIS_USB0_FRMNUMH        (KINETIS_USB0_BASE+KINETIS_USB_FRMNUMH_OFFSET)
#define KINETIS_USB0_TOKEN          (KINETIS_USB0_BASE+KINETIS_USB_TOKEN_OFFSET)
#define KINETIS_USB0_SOFTHLD        (KINETIS_USB0_BASE+KINETIS_USB_SOFTHLD_OFFSET)
#define KINETIS_USB0_BDTPAGE2       (KINETIS_USB0_BASE+KINETIS_USB_BDTPAGE2_OFFSET)
#define KINETIS_USB0_BDTPAGE3       (KINETIS_USB0_BASE+KINETIS_USB_BDTPAGE3_OFFSET)

#define KINETIS_USB0_ENDPT(n)       (KINETIS_USB0_BASE+KINETIS_USB_ENDPT_OFFSET(n))
#define KINETIS_USB0_ENDPT0         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT0_OFFSET)
#define KINETIS_USB0_ENDPT1         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT1_OFFSET)
#define KINETIS_USB0_ENDPT2         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT2_OFFSET)
#define KINETIS_USB0_ENDPT3         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT3_OFFSET)
#define KINETIS_USB0_ENDPT4         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT4_OFFSET)
#define KINETIS_USB0_ENDPT5         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT5_OFFSET)
#define KINETIS_USB0_ENDPT6         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT6_OFFSET)
#define KINETIS_USB0_ENDPT7         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT7_OFFSET)
#define KINETIS_USB0_ENDPT8         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT8_OFFSET)
#define KINETIS_USB0_ENDPT9         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT9_OFFSET)
#define KINETIS_USB0_ENDPT10        (KINETIS_USB0_BASE+KINETIS_USB_ENDPT10_OFFSET)
#define KINETIS_USB0_ENDPT11        (KINETIS_USB0_BASE+KINETIS_USB_ENDPT11_OFFSET)
#define KINETIS_USB0_ENDPT12        (KINETIS_USB0_BASE+KINETIS_USB_ENDPT12_OFFSET)
#define KINETIS_USB0_ENDPT13        (KINETIS_USB0_BASE+KINETIS_USB_ENDPT13_OFFSET)
#define KINETIS_USB0_ENDPT14        (KINETIS_USB0_BASE+KINETIS_USB_ENDPT14_OFFSET)
#define KINETIS_USB0_ENDPT15        (KINETIS_USB0_BASE+KINETIS_USB_ENDPT15_OFFSET)

#define KINETIS_USB0_USBCTRL        (KINETIS_USB0_BASE+KINETIS_USB_USBCTRL_OFFSET)
#define KINETIS_USB0_OBSERVE        (KINETIS_USB0_BASE+KINETIS_USB_OBSERVE_OFFSET)
#define KINETIS_USB0_CONTROL        (KINETIS_USB0_BASE+KINETIS_USB_CONTROL_OFFSET)
#define KINETIS_USB0_USBTRC0        (KINETIS_USB0_BASE+KINETIS_USB_USBTRC0_OFFSET)

/* Register Bit Definitions *****************************************************************/

/* Peripheral ID Register */
#define KINETIS_USB_PERID_
/* Peripheral ID Complement Register */
#define KINETIS_USB_IDCOMP_
/* Peripheral Revision Register */
#define KINETIS_USB_REV_
/* Peripheral Additional Info Register */
#define KINETIS_USB_ADDINFO_
/* OTG Interrupt Status Register */
#define KINETIS_USB_OTGISTAT_
/* OTG Interrupt Control Register */
#define KINETIS_USB_OTGICR_
/* OTG Status Register */
#define KINETIS_USB_OTGSTAT_
/* OTG Control Register */
#define KINETIS_USB_OTGCTL_
/* Interrupt Status Register */
#define KINETIS_USB_ISTAT_
/* Interrupt Enable Register */
#define KINETIS_USB_INTEN_
/* Error Interrupt Status Register */
#define KINETIS_USB_ERRSTAT_
/* Error Interrupt Enable Register */
#define KINETIS_USB_ERREN_
/* Status Register */
#define KINETIS_USB_STAT_
/* Control Register */
#define KINETIS_USB_CTL_
/* Address Register */
#define KINETIS_USB_ADDR_
/* BDT Page Register 1 */
#define KINETIS_USB_BDTPAGE1_
/* Frame Number Register Low */
#define KINETIS_USB_FRMNUML_
/* Frame Number Register High */
#define KINETIS_USB_FRMNUMH_
/* Token Register */
#define KINETIS_USB_TOKEN_
/* SOF Threshold Register */
#define KINETIS_USB_SOFTHLD_
/* BDT Page Register 2 */
#define KINETIS_USB_BDTPAGE2_
/* BDT Page Register 3 */
#define KINETIS_USB_BDTPAGE3_

/* Endpoint n Control Register */
#define KINETIS_USB_ENDPT_

/* USB Control Register */
#define KINETIS_USB_USBCTRL_
/* USB OTG Observe Register */
#define KINETIS_USB_OBSERVE_
/* USB OTG Control Register */
#define KINETIS_USB_CONTROL_
/* USB Transceiver Control Register 0 */
#define KINETIS_USB_USBTRC0_

                (1 << nn)  /* Bit nn:  
_SHIFT          (nn)       /* Bits nn-nn: 
_MASK           (nn << nn)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_USBOTG_H */
