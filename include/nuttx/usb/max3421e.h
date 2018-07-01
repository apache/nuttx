/****************************************************************************
 * include/nuttx/usb/max3421e.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_USB_MAX3421E_H
#define __INCLUDE_NUTTX_USB_MAX3421E_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_USBHOST_MAX3421E

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Host Mode Register Addresses *********************************************/

#define MAX3421E_USBHOST_EP0FIFO        0
#define MAX3421E_USBHOST_EP1OUTFIFO     1
#define MAX3421E_USBHOST_EP2INFIFO      2
#define MAX3421E_USBHOST_EP3INFIFO      3
#define MAX3421E_USBHOST_SUDFIFO        4
#define MAX3421E_USBHOST_EP0BC          5
#define MAX3421E_USBHOST_EP1OUTBC       6
#define MAX3421E_USBHOST_EP2INBC        7
#define MAX3421E_USBHOST_EP3INBC        8
#define MAX3421E_USBHOST_EPSTALLS       9
#define MAX3421E_USBHOST_CLRTOGS        10
#define MAX3421E_USBHOST_EPIRQ          11
#define MAX3421E_USBHOST_EPIEN          12
#define MAX3421E_USBHOST_USBIRQ         13
#define MAX3421E_USBHOST_USBIEN         14
#define MAX3421E_USBHOST_USBCTL         15
#define MAX3421E_USBHOST_CPUCTL         16
#define MAX3421E_USBHOST_PINCTL         17
#define MAX3421E_USBHOST_REVISION       18
#define MAX3421E_USBHOST_FNADDR         19
#define MAX3421E_USBHOST_IOPINS1        20
#define MAX3421E_USBHOST_IOPINS2        21
#define MAX3421E_USBHOST_GPINIRQ        22
#define MAX3421E_USBHOST_GPINIEN        23
#define MAX3421E_USBHOST_GPINPOL        24
#define MAX3421E_USBHOST_MODE           27

/* Peripheral Mode Register Addresses ***************************************/

#define MAX3421E_USBDEV_RCVFIFO         1
#define MAX3421E_USBDEV_SNDFIFO         2
#define MAX3421E_USBDEV_SUDFIFO         4
#define MAX3421E_USBDEV_RCVBC           6
#define MAX3421E_USBDEV_SNDBC           7
#define MAX3421E_USBDEV_USBIRQ          13
#define MAX3421E_USBDEV_USBIEN          14
#define MAX3421E_USBDEV_USBCTL          15
#define MAX3421E_USBDEV_CPUCTL          16
#define MAX3421E_USBDEV_PINCTL          17
#define MAX3421E_USBDEV_REVISION        18
#define MAX3421E_USBDEV_IOPINS1         20
#define MAX3421E_USBDEV_IOPINS2         21
#define MAX3421E_USBDEV_GPINIRQ         22
#define MAX3421E_USBDEV_GPINIEN         23
#define MAX3421E_USBDEV_GPINPOL         24
#define MAX3421E_USBDEV_HIRQ            25
#define MAX3421E_USBDEV_HIEN            26
#define MAX3421E_USBDEV_MODE            27
#define MAX3421E_USBDEV_PERADDR         28
#define MAX3421E_USBDEV_HCTL            29
#define MAX3421E_USBDEV_HXFR            30
#define MAX3421E_USBDEV_HRSL            31

/* Host Mode Register Bit-Field Definitions *********************************/

#define USBHOST_EP0BC_MASK              0x7f
#define USBHOST_EP1OUTBC_MASK           0x7f
#define USBHOST_EP2INBC_MASK            0x7f
#define USBHOST_EP3INBC_MASK            0x7f

#define USBHOST_EPSTALLS_STLEP0IN       (1 << 0)
#define USBHOST_EPSTALLS_STLEP0OUT      (1 << 1)
#define USBHOST_EPSTALLS_STLEP1OUT      (1 << 2)
#define USBHOST_EPSTALLS_STLEP2IN       (1 << 3)
#define USBHOST_EPSTALLS_STLEP3IN       (1 << 4)
#define USBHOST_EPSTALLS_STLSTAT        (1 << 5)
#define USBHOST_EPSTALLS_ACKSTAT        (1 << 6)

#define USBHOST_CLRTOGS_CTGEP1OUT       (1 << 2)
#define USBHOST_CLRTOGS_CTGEP2IN        (1 << 3)
#define USBHOST_CLRTOGS_CTGEP3IN        (1 << 4)
#define USBHOST_CLRTOGS_EP1DISAB        (1 << 5)
#define USBHOST_CLRTOGS_EP2DISAB        (1 << 6)
#define USBHOST_CLRTOGS_EP3DISAB        (1 << 7)

#define USBHOST_EPIRQ_IN0BAVIRQ         (1 << 0)
#define USBHOST_EPIRQ_OUT0DAVIRQ        (1 << 1)
#define USBHOST_EPIRQ_OUT1DAVIRQ        (1 << 2)
#define USBHOST_EPIRQ_IN2BAVIRQ         (1 << 3)
#define USBHOST_EPIRQ_IN3BAVIRQ         (1 << 4)
#define USBHOST_EPIRQ_SUDAVIRQ          (1 << 5)

#define USBHOST_EPIEN_IN0BAVIE          (1 << 0)
#define USBHOST_EPIEN_OUT0DAVIE         (1 << 1)
#define USBHOST_EPIEN_OUT1DAVIE         (1 << 2)
#define USBHOST_EPIEN_IN2BAVIE          (1 << 3)
#define USBHOST_EPIEN_IN3BAVIE          (1 << 4)
#define USBHOST_EPIEN_SUDAVIE           (1 << 5)

#define USBHOST_USBIRQ_OSCOKIRQ         (1 << 0)
#define USBHOST_USBIRQ_RWUDNIRQ         (1 << 1)
#define USBHOST_USBIRQ_BUSACTIRQ        (1 << 2)
#define USBHOST_USBIRQ_URESIRQ          (1 << 3)
#define USBHOST_USBIRQ_SUSPIRQ          (1 << 4)
#define USBHOST_USBIRQ_NOVBUSIRQ        (1 << 5)
#define USBHOST_USBIRQ_VBUSIRQ          (1 << 6)
#define USBHOST_USBIRQ_URESDNIRQ        (1 << 7)

#define USBHOST_USBIEN_OSCOKIE          (1 << 0)
#define USBHOST_USBIEN_RWUDNIE          (1 << 1)
#define USBHOST_USBIEN_BUSACTIE         (1 << 2)
#define USBHOST_USBIEN_URESIE           (1 << 3)
#define USBHOST_USBIEN_SUSPIE           (1 << 4)
#define USBHOST_USBIEN_NOVBUSIE         (1 << 5)
#define USBHOST_USBIEN_VBUSIEN          (1 << 6)
#define USBHOST_USBIEN_URESDNIE         (1 << 7)

#define USBHOST_USBCTL_SIGRWU           (1 << 2)
#define USBHOST_USBCTL_CONNECT          (1 << 3)
#define USBHOST_USBCTL_PWRDOWN          (1 << 4)
#define USBHOST_USBCTL_CHIPRES          (1 << 5)
#define USBHOST_USBCTL_VBGATE           (1 << 6)
#define USBHOST_USBCTL_HOSCSTEN         (1 << 7)

#define USBHOST_CPUCTL_IE               (1 << 0)
#define USBHOST_CPUCTL_PULSEWID0        (1 << 6)
#define USBHOST_CPUCTL_PULSEWID1        (1 << 7)

#define USBHOST_PINCTL_PXA              (1 << 0)
#define USBHOST_PINCTL_GPXB             (1 << 1)
#define USBHOST_PINCTL_POSINT           (1 << 2)
#define USBHOST_PINCTL_INTLEVEL         (1 << 3)
#define USBHOST_PINCTL_FDUPSPI          (1 << 4)
#define USBHOST_PINCTL_EP0INAK          (1 << 5)
#define USBHOST_PINCTL_EP2INAK          (1 << 6)
#define USBHOST_PINCTL_EP3INAK          (1 << 7)

#define USBHOST_REVISION                0x13
#define USBHOST_FNADDR_MASK             0x7f

#define USBHOST_IOPINS1_GPOUT(n)        (1 << (n))
#define USBHOST_IOPINS1_GPIN(n)         (1 << ((n) + 4))
#define USBHOST_IOPINS2_GPOUT(n)        (1 << ((n) - 4))
#define USBHOST_IOPINS2_GPIN(n)         (1 << (n))
#define USBHOST_GPINIRQ(n)              (1 << (n))
#define USBHOST_GPINIEN(n)              (1 << (n))
#define USBHOST_GPINPOL(n)              (1 << (n))

#define USBHOST_MODE_SEPIRQ             (1 << 4)
#define USBHOST_MODE_HOST               (1 << 0)

/* Peripheral Mode Register Bit-Field Definitions ***************************/

#define USBDEV_RCVBC_MASK               0x7f
#define USBDEV_SNDBC_MASK               0x7f

#define USBDEV_USBIRQ_OSCOKIRQ          (1 << 0)
#define USBDEV_USBIRQ_NOVBUSIRQ         (1 << 5)
#define USBDEV_USBIRQ_VBUSIRQ           (1 << 6)

#define USBDEV_USBIEN_OSCOKIE           (1 << 0)
#define USBDEV_USBIEN_NOVBUSIE          (1 << 5)
#define USBDEV_USBIEN_VBUSIEN           (1 << 6)

#define USBDEV_USBCTL_PWRDOWN           (1 << 4)
#define USBDEV_USBCTL_CHIPRES           (1 << 5)

#define USBDEV_CPUCTL_IE                (1 << 0)
#define USBDEV_CPUCTL_PULSEWID0         (1 << 6)
#define USBDEV_CPUCTL_PULSEWID1         (1 << 7)

#define USBDEV_PINCTL_PXA               (1 << 0)
#define USBDEV_PINCTL_GPXB              (1 << 1)
#define USBDEV_PINCTL_POSINT            (1 << 2)
#define USBDEV_PINCTL_INTLEVEL          (1 << 3)
#define USBDEV_PINCTL_FDUPSPI           (1 << 4)
#define USBDEV_PINCTL_EP0INAK           (1 << 5)
#define USBDEV_PINCTL_EP2INAK           (1 << 6)
#define USBDEV_PINCTL_EP3INAK           (1 << 7)

#define USBDEV_REVISION                 0x13

#define USBDEV_IOPINS1_GPOUT(n)         (1 << (n))
#define USBDEV_IOPINS1_GPIN(n)          (1 << ((n) + 4))
#define USBDEV_IOPINS2_GPOUT(n)         (1 << ((n) - 4))
#define USBDEV_IOPINS2_GPIN(n)          (1 << (n))
#define USBDEV_GPINIRQ(n)               (1 << (n))
#define USBDEV_GPINIEN(n)               (1 << (n))
#define USBDEV_GPINPOL(n)               (1 << (n))

#define USBDEV_HIRQ_BUSEVENTIRQ         (1 << 0)
#define USBDEV_HIRQ_RSMREQIRQ           (1 << 1)
#define USBDEV_HIRQ_RCVDAVIRQ           (1 << 2)
#define USBDEV_HIRQ_SNDBAVIRQ           (1 << 3)
#define USBDEV_HIRQ_SUSDNIRQ            (1 << 4)
#define USBDEV_HIRQ_CONNIRQ             (1 << 5)
#define USBDEV_HIRQ_FRAMEIRQ            (1 << 6)
#define USBDEV_HIRQ_HXFRDNIRQ           (1 << 7)

#define USBDEV_HIEN_BUSEVENTIE          (1 << 0)
#define USBDEV_HIEN_RSMREQIE            (1 << 1)
#define USBDEV_HIEN_RCVDAVIE            (1 << 2)
#define USBDEV_HIEN_SNDBAVIE            (1 << 3)
#define USBDEV_HIEN_SUSDNIE             (1 << 4)
#define USBDEV_HIEN_CONNIE              (1 << 5)
#define USBDEV_HIEN_FRAMEIE             (1 << 6)
#define USBDEV_HIEN_HXFRDNIE            (1 << 7)

#define USBDEV_MODE_HOST                (1 << 0)
#define USBDEV_MODE_SPEED               (1 << 1)
#define USBDEV_MODE_HUBPRE              (1 << 2)
#define USBDEV_MODE_SOFKAENAB           (1 << 3)
#define USBDEV_MODE_SEPIRQ              (1 << 4)
#define USBDEV_MODE_NDELAYISO           (1 << 5)
#define USBDEV_MODE_DMPULLD             (1 << 6)
#define USBDEV_MODE_DPPULLDN            (1 << 7)

#define USBDEV_PERADDR_MASK             0x7f

#define USBDEV_HCTL_BUSRST              (1 << 0)
#define USBDEV_HCTL_FRMRST              (1 << 1)
#define USBDEV_HCTL_BUSSAMPLE           (1 << 2)
#define USBDEV_HCTL_SIGRSM              (1 << 3)
#define USBDEV_HCTL_RCVTOG0             (1 << 4)
#define USBDEV_HCTL_RCVTOG1             (1 << 5)
#define USBDEV_HCTL_SNDTOG0             (1 << 6)
#define USBDEV_HCTL_SNDTOG1             (1 << 7)

#define USBDEV_HXFR_EP0                 (1 << 0)
#define USBDEV_HXFR_EP1                 (1 << 1)
#define USBDEV_HXFR_EP2                 (1 << 2)
#define USBDEV_HXFR_EP3                 (1 << 3)
#define USBDEV_HXFR_SETUP               (1 << 4)
#define USBDEV_HXFR_OUTNIN              (1 << 5)
#define USBDEV_HXFR_ISO                 (1 << 6)
#define USBDEV_HXFR_HS                  (1 << 7)

#define USBDEV_HRSL_HRSLT0              (1 << 0)
#define USBDEV_HRSL_HRSLT1              (1 << 1)
#define USBDEV_HRSL_HRSLT2              (1 << 2)
#define USBDEV_HRSL_HRSLT3              (1 << 3)
#define USBDEV_HRSL_RCVTOGRD            (1 << 4)
#define USBDEV_HRSL_SNDTOGRD            (1 << 5)
#define USBDEV_HRSL_KSTATUS             (1 << 6)
#define USBDEV_HRSL_JSTATUS             (1 << 7)

/* Misc. Definitions ********************************************************/

/* Sizes and numbers of things */

#define MAX3421E_NENDPOINTS             4       /* EP0..EP3 */
#define MAX3421E_DBLBUF_SET             0x06    /* EP2, EP3 double buffered */

#define MAX3421E_SNDFIFO_SIZE           64
#define MAX3421E_RCVFIFO_SIZE           64
#define MAX3421E_SETUPFIFO_SIZE         8

/* Value of the MODE register HOST bit */

#define MAX3421E_MODE_PERIPH            0
#define MAX3421E_MODE_HOST              1

/* The MAX3421E SPI interface operates without adjustment in either SPI mode
 * (CPOL = 0, CPHA = 0) or (CPOL = 1, CPHA = 1).
 */

#define MAX3421E_SPIMODE                SPIDEV_MODE0

/* The SPI clock frequency can be between DC and 26MHz. */

#define MAX3421E_SPIFREQ_MAX            (26*1000*1000)

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

/****************************************************************************
 * Name:
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_USBHOST_MAX3421E */
#endif /* __INCLUDE_NUTTX_USB_MAX3421E_H */
