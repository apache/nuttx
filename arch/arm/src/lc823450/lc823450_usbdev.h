/****************************************************************************
 * arch/arm/src/lc823450/lc823450_usbdev.h
 *
 *   Copyright 2014,2015,2016,2017 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
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

#ifndef __ARCH_ARM_SRC_LC823450_USBDEV_H
#define __ARCH_ARM_SRC_LC823450_USBDEV_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_USBDEV_MAXENDPOINTS
#  define LC823450_NLOGENDPOINTS        CONFIG_USBDEV_MAXENDPOINTS
#else /* CONFIG_USBDEV_MAXENDPOINTS */
#  define LC823450_NLOGENDPOINTS        (16)          /* ep0-15 */
#endif /* CONFIG_USBDEV_MAXENDPOINTS */
#define LC823450_NPHYSENDPOINTS       (LC823450_NLOGENDPOINTS * 2)

/* Register define */

#define USBDEV_BASE             0x40010000

#define USB_CONF                (USBDEV_BASE + 0x0000)
#define USB_CONF_SOFT_RESET     (1 << 2)

#define USB_MODE                (USBDEV_BASE + 0x0004)
#define USB_MODE_ADDR_LDMOD     (1 << 3)
#define USB_MODE_DEV_INTMOD     (1 << 2)
#define USB_MODE_DEV_EN         (1 << 1)

#define USB_INTEN               (USBDEV_BASE + 0x0008)

#define USB_INTS                (USBDEV_BASE + 0x000c)
#define USB_INT_EP0_SHIFT       16
#define USB_INT_EP0             (1 << 16)
#define USB_INT_EP15_SHIFT      31
#define USB_INT_DMA2            (1 << 9)
#define USB_INT_DMA1            (1 << 8)
#define USB_INT_CMD             (1 << 4)
#define USB_INT_PHYERR          (1 << 3)
#define USB_INT_DEV             (1 << 1)

#define USB_CUSTOMC             (USBDEV_BASE + 0x0018)
#define USB_CUSTOMC_HSDSEL_5    0x1
#define USB_CUSTOMC_HSDSEL_10   0x2
#define USB_CUSTOMC_HSDSEL_MASK 0xf

#define USB_EPCMD(n)            (USBDEV_BASE + 0x0040 + (n) * 0x4)
#define USB_EPCMD_BUSY          (1 << 31)
#define USB_EPCMD_ET_SHIFT      23
#define USB_EPCMD_DIR_SHIFT     25
#define USB_EPCMD_DIR           (1 << 25)
#define USB_EPCMD_NACK_CLR      (1 << 22)
#define USB_EPCMD_READYO_CLR    (1 << 19)
#define USB_EPCMD_READYI_CLR    (1 << 18)
#define USB_EPCMD_EMPTY_CLR     (1 << 19)
#define USB_EPCMD_READY_CLR     (1 << 18)
#define USB_EPCMD_READYO_EN     (1 << 13)
#define USB_EPCMD_READYI_EN     (1 << 12)
#define USB_EPCMD_EMPTY_EN      (1 << 13)
#define USB_EPCMD_READY_EN      (1 << 12)
#define USB_EPCMD_WRITE_EN      (1 << 11)
#define USB_EPCMD_NAK           (1 << 10)
#define USB_EPCMD_NULL          (1 << 9)
#define USB_EPCMD_TGL_CLR       (1 << 8)
#define USB_EPCMD_TGL_SET       (1 << 7)
#define USB_EPCMD_STALL_CLR     (1 << 6)
#define USB_EPCMD_STALL_SET     (1 << 5)
#define USB_EPCMD_BUFRD         (1 << 4)
#define USB_EPCMD_BUFWR         (1 << 3)
#define USB_EPCMD_INIT          (1 << 2)
#define USB_EPCMD_STOP          (1 << 1)
#define USB_EPCMD_START         (1 << 0)

#define USB_DEVC                (USBDEV_BASE + 0x0200)
#define USB_DEVC_USBRSTB        (1 << 29)
#define USB_DEVC_USBRSTE        (1 << 28)
#define USB_DEVC_SETUP          (1 << 27)
#define USB_DEVC_SUSPENDB       (1 << 25)
#define USB_DEVC_SUSPENDE       (1 << 24)
#define USB_DEVC_DISCON         (1 << 5)
#define USB_DEVC_RWUP_EN        (1 << 3)
#define USB_DEVC_RESUME         (1 << 2)

#define USB_DEVS                (USBDEV_BASE + 0x0204)
#define USB_DEVS_SOF            (1 << 26)
#define USB_DEVS_CRTSPEED       (1 << 17)
#define USB_DEVS_SUSPEND        (1 << 0)

#define USB_FADDR               (USBDEV_BASE + 0x0208)
#define USB_FADDR_CONFD         (1 << 8)
#define USB_FADDR_ADDR_MASK     (0x7f)
#define USB_FADDR_ADDR_SHIFT    0

#define USB_TSTAMP              (USBDEV_BASE + 0x020c)

#define USB_OTGC                (USBDEV_BASE + 0x0300)

#define USB_OTGSTS              (USBDEV_BASE + 0x0310)
#define USB_OTGSTS_VBUS_VLD     (1 << 10)

#define USB_OTGSTSC             (USBDEV_BASE + 0x0314)

#define USB_OTGSTSFALL          (USBDEV_BASE + 0x0318)

#define USB_OTGSTSRISE          (USBDEV_BASE + 0x031c)

#define USB_OTGTC               (USBDEV_BASE + 0x0320)

#define USB_OTGT                (USBDEV_BASE + 0x0324)

#define USB_DMAC1               (USBDEV_BASE + 0x0400)
#define USB_DMAC_BSIZE_SHIFT    16
#define USB_DMAC_DMAEP_SHIFT    8
#define USB_DMAC_INTEMP         (1 << 4)
#define USB_DMAC_START          (1 << 0)

#define USB_DMAS1               (USBDEV_BASE + 0x0404)

#define USB_DMATCI1             (USBDEV_BASE + 0x0408)

#define USB_DMATC1              (USBDEV_BASE + 0x040c)

#define USB_DMAC2               (USBDEV_BASE + 0x0420)

#define USB_DMAS2               (USBDEV_BASE + 0x0424)

#define USB_DMATCI2             (USBDEV_BASE + 0x0428)

#define USB_DMATC2              (USBDEV_BASE + 0x042c)

#define USB_TESTC               (USBDEV_BASE + 0x0500)
#define USB_TESTC_FORCE_HS      (1 << 24)

#define USB_EPBUF               (USBDEV_BASE + 0x8000)

#define USB_EPCTRL(n)           (USBDEV_BASE + 0x8000 + (n) * 0x4)
#define USB_EPCTRL_EMPTI        (1 << 27)
#define USB_EPCTRL_READI        (1 << 26)
#define USB_EPCTRL_READOI       (1 << 27)
#define USB_EPCTRL_READII       (1 << 26)
#define USB_EPCTRL_STALL        (1 << 12)
#define USB_EPCTRL_FULL         (1 << 11)
#define USB_EPCTRL_EMPTY        (1 << 10)
#define USB_EPCTRL_EMPTYO       (1 << 10)
#define USB_EPCTRL_EMPTYI       (1 << 8)
#define USB_EPCTRL_DIR_SHIFT    3
#define USB_EPCTRL_ET_SHIFT     1
#define USB_EPCTRL_EPEN         (1 << 0)

#define USB_EPCONF(n)           (USBDEV_BASE + 0x8040 + (n) * 0x4)
#define USB_EPCONF_CIDX_SHIFT   24
#define USB_EPCONF_SIZE_SHIFT   13
#define USB_EPCONF_BASE_SHIFT   0

#define USB_EPCOUNT(n)          (USBDEV_BASE + 0x8080 + (n) * 0x4)
#define USB_EPCOUNT_PHYCNT_SHIFT 16
#define USB_EPCOUNT_PHYCNT_MASK 0x07ff0000
#define USB_EPCOUNT_APPCNT_SHIFT 0
#define USB_EPCOUNT_APPCNT_MASK 0x000007ff

#endif /* __ARCH_ARM_SRC_LC823450_USBDEV_H */
