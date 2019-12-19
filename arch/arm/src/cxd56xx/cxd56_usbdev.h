/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_usbdev.h
 *
 *   Copyright (C) 2008-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_USBDEV_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_USBDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets */

/* Common Register Offsets */

#define CXD56_USB_IN_EP_CONTROL(x)    (CXD56_USBDEV_BASE + 0x0000 + ((x) * 0x20))
#define CXD56_USB_IN_EP_STATUS(x)     (CXD56_USBDEV_BASE + 0x0004 + ((x) * 0x20))
#define CXD56_USB_IN_EP_BUFSIZE(x)    (CXD56_USBDEV_BASE + 0x0008 + ((x) * 0x20))
#define CXD56_USB_IN_EP_MAXPKTSIZE(x) (CXD56_USBDEV_BASE + 0x000c + ((x) * 0x20))
#define CXD56_USB_IN_EP_DATADESC(x)   (CXD56_USBDEV_BASE + 0x0014 + ((x) * 0x20))

#define CXD56_USB_OUT_EP_CONTROL(x)   (CXD56_USBDEV_BASE + 0x0200 + ((x) * 0x20))
#define CXD56_USB_OUT_EP_STATUS(x)    (CXD56_USBDEV_BASE + 0x0204 + ((x) * 0x20))
#define CXD56_USB_OUT_EP_BUFSIZE(x)   (CXD56_USBDEV_BASE + 0x020c + ((x) * 0x20))
#define CXD56_USB_OUT_EP_SETUP(x)     (CXD56_USBDEV_BASE + 0x0210 + ((x) * 0x20))
#define CXD56_USB_OUT_EP_DATADESC(x)  (CXD56_USBDEV_BASE + 0x0214 + ((x) * 0x20))

#define CXD56_USB_DEV_CONFIG          (CXD56_USBDEV_BASE + 0x0400)
#define CXD56_USB_DEV_CONTROL         (CXD56_USBDEV_BASE + 0x0404)
#define CXD56_USB_DEV_STATUS          (CXD56_USBDEV_BASE + 0x0408)
#define CXD56_USB_DEV_INTR            (CXD56_USBDEV_BASE + 0x040c)
#define CXD56_USB_DEV_INTR_MASK       (CXD56_USBDEV_BASE + 0x0410)
#define CXD56_USB_DEV_EP_INTR         (CXD56_USBDEV_BASE + 0x0414)
#define CXD56_USB_DEV_EP_INTR_MASK    (CXD56_USBDEV_BASE + 0x0418)

#define CXD56_USB_DEV_UDC_EP0         (CXD56_USBDEV_BASE + 0x0504)
#define CXD56_USB_DEV_UDC_EP1         (CXD56_USBDEV_BASE + 0x0508)
#define CXD56_USB_DEV_UDC_EP2         (CXD56_USBDEV_BASE + 0x050c)
#define CXD56_USB_DEV_UDC_EP3         (CXD56_USBDEV_BASE + 0x0510)
#define CXD56_USB_DEV_UDC_EP4         (CXD56_USBDEV_BASE + 0x0514)
#define CXD56_USB_DEV_UDC_EP5         (CXD56_USBDEV_BASE + 0x0518)
#define CXD56_USB_DEV_UDC_EP6         (CXD56_USBDEV_BASE + 0x051c)
#define CXD56_USB_DEV_UDC_EP(x)       (CXD56_USB_DEV_UDC_EP0 + ((x) * 4))

#define CXD56_USB_SYS_INTR            (CXD56_USBDEV_BASE + 0x0800)
#define CXD56_USB_SYS_INTR_MASK       (CXD56_USBDEV_BASE + 0x0804)
#define CXD56_USB_BUSY                (CXD56_USBDEV_BASE + 0x0808)
#define CXD56_USB_VBUS_CTRL           (CXD56_USBDEV_BASE + 0x080c)
#define CXD56_USB_RESET               (CXD56_USBDEV_BASE + 0x0810)

#define CXD56_USB_SUSPEND_MASK        (CXD56_USBDEV_BASE + 0x081c)

#define CXD56_USB_PJ_DEMAND           (CXD56_USBDEV_BASE + 0x0830)

#define CXD56_USB_PHY_CONFIG0         (CXD56_USBDEV_BASE + 0x083c)
#define CXD56_USB_PHY_CONFIG1         (CXD56_USBDEV_BASE + 0x0840)
#define CXD56_USB_PHY_CONFIG2         (CXD56_USBDEV_BASE + 0x0844)

/* EP types */

#define USB_EP_CONTROL     0
#define USB_EP_ISOCHRONOUS 1
#define USB_EP_BULK        2
#define USB_EP_INTERRUPT   3

/* EP control bits */

#define USB_MRXFLUSH    (1<<12)
#define USB_CLOSEDESC   (1<<11)
#define USB_SENDNULL    (1<<10)
#define USB_RRDY        (1<<9)
#define USB_CNAK        (1<<8)
#define USB_SNAK        (1<<7)
#define USB_NAK         (1<<6)
#define USB_ET(x)       ((x)<<4)
#define USB_P           (1<<3)
#define USB_SN          (1<<2)
#define USB_F           (1<<1)
#define USB_STALL       (1<<0)

/* USB device configuration */

#define USB_CONFIG_DDR                 (1<<19)
#define USB_CONFIG_SET_DESC            (1<<18)
#define USB_CONFIG_CSR_PRG             (1<<17)
#define USB_CONFIG_HALT_STATUS         (1<<16)
#define USB_CONFIG_HS_TIMEOUT_CALIB(x) (((x)&7)<<13)
#define USB_CONFIG_FS_TIMEOUT_CALIB(x) (((x)&7)<<10)
#define USB_CONFIG_PHY_ERROR_DETECT    (1<<9)
#define USB_CONFIG_STATUS_1            (1<<8)
#define USB_CONFIG_STATUS              (1<<7)
#define USB_CONFIG_DIR                 (1<<6)
#define USB_CONFIG_PI                  (1<<5)
#define USB_CONFIG_SS                  (1<<4)
#define USB_CONFIG_SP                  (1<<3)
#define USB_CONFIG_RWKP                (1<<2)
#define USB_CONFIG_SPD(x)              (((x)&3))
#define USB_CONFIG_HS                  0
#define USB_CONFIG_FS                  1
#define USB_CONFIG_SPD_MASK            3

/* USB device control */

#define USB_CTRL_THLEN(x)              (((x)&0xff)<<24)
#define USB_CTRL_BRLEN(x)              (((x)&0xff)<<16)
#define USB_CTRL_SRXFLUSH              (1<<14)
#define USB_CTRL_CSR_DONE              (1<<13)
#define USB_CTRL_DEVNAK                (1<<12)
#define USB_CTRL_SCALE                 (1<<11)
#define USB_CTRL_SD                    (1<<10)
#define USB_CTRL_MODE                  (1<<9)
#define USB_CTRL_BREN                  (1<<8)
#define USB_CTRL_THE                   (1<<7)
#define USB_CTRL_BF                    (1<<6)
#define USB_CTRL_BE                    (1<<5)
#define USB_CTRL_DU                    (1<<4)
#define USB_CTRL_TDE                   (1<<3)
#define USB_CTRL_RDE                   (1<<2)
#define USB_CTRL_RES                   (1<<0)

/* USB device status bits */

#define USB_STATUS_RMTWKP_STATE        (1<<17)
#define USB_STATUS_PHYERROR            (1<<16)
#define USB_STATUS_RXFIFOEMPTY         (1<<15)
#define USB_STATUS_SPD_SHIFT           13
#define USB_STATUS_SPD_MASK            (3<<USB_STATUS_SPD_SHIFT)
#define USB_STATUS_SPD(x)              (((x) & USB_STATUS_SPD_MASK) >> USB_STATUS_SPD_SHIFT)
#define USB_STATUS_SUSP                (1<<12)
#define USB_STATUS_ALT(x)              (((x)>>8)&0xf)
#define USB_STATUS_INTF(x)             (((x)>>4)&0xf)
#define USB_STATUS_CFG(x)              (((x)>>0)&0xf)

/* USB device interrupt bits */

#define USB_INT_RMTWKP_STATE  (1<<7)
#define USB_INT_ENUM          (1<<6)
#define USB_INT_SOF           (1<<5)
#define USB_INT_US            (1<<4)
#define USB_INT_UR            (1<<3)
#define USB_INT_ES            (1<<2)
#define USB_INT_SI            (1<<1)
#define USB_INT_SC            (1<<0)

/* USB system interrupt bits */

#define USB_INT_VBUS_DISC   (1<<3)
#define USB_INT_VBUS_CONN   (1<<2)
#define USB_INT_RESUME      (1<<1)
#define USB_INT_READY       (1<<0)

/* USB endpoint interrupt bits */

#define USB_INT_CDC_CLEAR   (1<<28)              /*  */
#define USB_INT_XFERDONE    (1<<27)              /* Tfansfer Done/Transmit FIFO Empty */
#define USB_INT_RSS         (1<<26)              /* Received Set Stall Indication */
#define USB_INT_RCS         (1<<25)              /* Received Clear Stall Indication */
#define USB_INT_TXEMPTY     (1<<24)              /* Transmit FIFO Empty */
#define USB_INT_ISO_IN_DONE (1<<23)              /* Isochronous IN transaction for the current microframe is complete */
#define USB_INT_RX_PKT_SIZE(x) (((x)>>11)&0xfff) /* Receive Packet Size */
#define USB_INT_TDC         (1<<10)              /* Transmit DMA Completion */
#define USB_INT_HE          (1<<9)               /* Error response on the host bus */
#define USB_INT_MRXFIFOEMPTY (1<<8)              /* Receive Address FIFO Empty Status */
#define USB_INT_BNA         (1<<7)               /* Buffer Not Available */
#define USB_INT_IN          (1<<6)               /* IN token has been received */
#define USB_INT_OUT(x)      ((x) & (3 << 4))     /* OUT packet has been received. */
#define   USB_INT_OUT_DATA  (1 << 4)
#define   USB_INT_OUT_SETUP (2 << 4)

/* PHY Configuration 0 bits */

#define PHY_STAGSELECT      (1u<<31)
#define PHY_SHORTCKT_PROT   (1u<<30)
#define PHY_HSFALLCNTRL     (1u<<29)
#define PHY_HSRXOFFSET(x)   (((x)&3)<<27)
#define PHY_HSRXEQUALIZE    (1u<<26)
#define PHY_INSQUETUNE(x)   (((x)&3)<<24)
#define PHY_ZHSDRV(x)       (((x)&3)<<22)
#define PHY_IHSTX(x)        (((x)&0xf)<<18)
#define PHY_INHSRFRED       (1u<<17)
#define PHY_INFSRFCNTL      (1u<<16)
#define PHY_INHSIPLUSENABLE (1u<<15)
#define PHY_INHSIPLUS       (1u<<14)
#define PHY_INHSIMINUS      (1u<<13)
#define PHY_INHSDRVSLEW     (1u<<12)
#define PHY_INLFSFBCAP      (1u<<11)
#define PHY_INTRCC1MA       (1u<<10)
#define PHY_INCURRENTENABLE (1u<<9)
#define PHY_INAFETRIM(x)    ((x)&0x1ff)

/* PHY Configuration 1 bits */

#define PHY_STRB_BYPASS     (1u<<31)
#define PHY_STRB            (1u<<30)
#define PHY_DITHER_DISABLE_RECT (1u<<29)
#define PHY_DITHER_DISABLE_TRI  (1u<<28)
#define PHY_FRAC_CONTROL    (1u<<27)
#define PHY_PLLENABLE       (1u<<26)
#define PHY_FRAC_INPUT(x)   (((x)&0xffff)<<10)
#define PHY_ODF(x)          (((x)&0x7)<<7)
#define PHY_NDIV(x)         ((x)&0x3f)

/* DMA descriptor status bits */

#define DESC_BS_SHIFT       30
#define DESC_BS_MASK        (3 << DESC_BS_SHIFT)
#define DESC_BS_HOST_READY  (0 << DESC_BS_SHIFT)
#define DESC_BS_DMA_BUSY    (1 << DESC_BS_SHIFT)
#define DESC_BS_DMA_DONE    (2 << DESC_BS_SHIFT)
#define DESC_BS_HOST_BUSY   (3 << DESC_BS_SHIFT)
#define IS_BS_HOST_READY(desc) (((desc)->status & DESC_BS_MASK) == DESC_BS_HOST_READY)
#define IS_BS_DMA_BUSY(desc)   (((desc)->status & DESC_BS_MASK) == DESC_BS_DMA_BUSY)
#define IS_BS_DMA_DONE(desc)   (((desc)->status & DESC_BS_MASK) == DESC_BS_DMA_DONE)
#define IS_BS_HOST_BUSY(desc)  (((desc)->status & DESC_BS_MASK) == DESC_BS_HOST_BUSY)

/* There is same definitions for TX/RX */

#define DESC_STS_SHIFT      28
#define DESC_STS_MASK       (3 << DESC_STS_SHIFT)
#define DESC_STS_SUCCESS    (0 << DESC_STS_SHIFT)
#define DESC_STS_DESERR     (1 << DESC_STS_SHIFT)
#define DESC_STS_BUFERR     (3 << DESC_STS_SHIFT)
#define IS_STS_SUCCESS(desc)  (((desc)->status & DESC_STS_MASK) == DESC_STS_SUCCESS)
#define IS_STS_DESERR(desc)   (((desc)->status & DESC_STS_MASK) == DESC_STS_DESERR)
#define IS_STS_BUFERR(desc)   (((desc)->status & DESC_STS_MASK) == DESC_STS_BUFERR)

#define DESC_LAST           (1 << 27)

#define DESC_SIZE_MASK      (0xffff)

/****************************************************************************
 * Name: cxd56_usbdev_setsigno
 *
 * Description:
 *   cxd56xx usb device driver attach / detach event signal
 *
 ****************************************************************************/

int cxd56_usbdev_setsigno(int signo);

/****************************************************************************
 * Name: cxd56_usbdev_procfs_register
 *
 * Description:
 *   Register the usbdev procfs file system entry
 *
 ****************************************************************************/

#ifdef CONFIG_FS_PROCFS_REGISTER
int cxd56_usbdev_procfs_register(void);
#endif

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_USBDEV_H */
