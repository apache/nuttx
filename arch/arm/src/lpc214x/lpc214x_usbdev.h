/*******************************************************************************
 * arch/arm/src/lpc214x/lpc214x_usbdev.h
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
 *******************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC214X_LPC214X_USBDEV_H
#define __ARCH_ARM_SRC_LPC214X_LPC214X_USBDEV_H

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/* USB Register definitions */

#define LPC214X_USBDEV_PLLCON        (0xe01fc0a0)
#define LPC214X_USBDEV_PLLCFG        (0xe01fc0a4)
#define LPC214X_USBDEV_PLLSTAT       (0xe01fc0a8)
#define LPC214X_USBDEV_PLLFEED       (0xe01fc0ac)

#define LPC214X_USBDEV_INTST         (0xe01fc1c0)

#define LPC214X_USBDEV_DEVINTST      (0xe0090000)
#define LPC214X_USBDEV_DEVINTEN      (0xe0090004)
#define LPC214X_USBDEV_DEVINTCLR     (0xe0090008)
#define LPC214X_USBDEV_DEVINTSET     (0xe009000c)
#define LPC214X_USBDEV_DEVINTPRI     (0xe009002c)
#define LPC214X_USBDEV_EPINTST       (0xe0090030)
#define LPC214X_USBDEV_EPINTEN       (0xe0090034)
#define LPC214X_USBDEV_EPINTCLR      (0xe0090038)
#define LPC214X_USBDEV_EPINTSET      (0xe009003c)
#define LPC214X_USBDEV_EPINTPRI      (0xe0090040)
#define LPC214X_USBDEV_REEP          (0xe0090044)
#define LPC214X_USBDEV_EPIND         (0xe0090048)
#define LPC214X_USBDEV_MAXPSIZE      (0xe009004c)
#define LPC214X_USBDEV_RXDATA        (0xe0090018)
#define LPC214X_USBDEV_RXPLEN        (0xe0090020)
#define LPC214X_USBDEV_TXDATA        (0xe009001c)
#define LPC214X_USBDEV_TXPLEN        (0xe0090024)
#define LPC214X_USBDEV_CTRL          (0xe0090028)
#define LPC214X_USBDEV_CMDCODE       (0xe0090010)
#define LPC214X_USBDEV_CMDDATA       (0xe0090014)
#define LPC214X_USBDEV_DMARST        (0xe0090050)
#define LPC214X_USBDEV_DMARCLR       (0xe0090054)
#define LPC214X_USBDEV_DMARSET       (0xe0090058)
#define LPC214X_USBDEV_UDCAH         (0xe0090080)
#define LPC214X_USBDEV_EpDMASt       (0xe0090084)
#define LPC214X_USBDEV_EPDMAEN       (0xe0090088)
#define LPC214X_USBDEV_EPDMADIS      (0xe009008c)
#define LPC214X_USBDEV_DMAINTST      (0xe0090090)
#define LPC214X_USBDEV_DMAINTEN      (0xe0090094)
#define LPC214X_USBDEV_EOTINTST      (0xe00900a0)
#define LPC214X_USBDEV_EOTINTCLR     (0xe00900a4)
#define LPC214X_USBDEV_EOTINTSET     (0xe00900a8)
#define LPC214X_USBDEV_NDDRINTST     (0xe00900ac)
#define LPC214X_USBDEV_NDDRINTCLR    (0xe00900b0)
#define LPC214X_USBDEV_NDDRINTSET    (0xe00900b4)
#define LPC214X_USBDEV_SYSERRINTST   (0xe00900b8)
#define LPC214X_USBDEV_SYSERRINTCLR  (0xe00900bc)
#define LPC214X_USBDEV_SYSERRINTSET  (0xe00900c0)

/* INTST bit definitions */

#define USBDEV_INTST_REQLP           (0x00000001)
#define USBDEV_INTST_REQHP           (0x00000002)
#define USBDEV_INTST_REQDMA          (0x00000004)
#define USBDEV_INTST_NEEDCLOCK       (0x00000100)
#define USBDEV_INTST_ENUSBINTS       (0x80000000)
#define USBDEV_INTST_MASK            (0x80000107)

/* DEVINTST/DEVINTEN/DEVINTCLR/DEVINTSET bit definitions */

#define USBDEV_DEVINT_FRAME          (0x00000001)
#define USBDEV_DEVINT_EPFAST         (0x00000002)
#define USBDEV_DEVINT_EPSLOW         (0x00000004)
#define USBDEV_DEVINT_DEVSTAT        (0x00000008)
#define USBDEV_DEVINT_CCEMTY         (0x00000010)
#define USBDEV_DEVINT_CDFULL         (0x00000020)
#define USBDEV_DEVINT_RXENDPKT       (0x00000040)
#define USBDEV_DEVINT_TXENDPKT       (0x00000080)
#define USBDEV_DEVINT_EPRLZED        (0x00000100)
#define USBDEV_DEVINT_EPRINT         (0x00000200)
#define USBDEV_DEVINT_MASK           (0x000003ff)

/* DEVINTPRI bit definitions */

#define USBDEV_DEVINTPRI_FRAME       (0x00000001)
#define USBDEV_DEVINTPRI_EPFAST      (0x00000002)
#define USBDEV_DEVINTPRI_MASK        (0x00000003)

/* RXPLEN bit definitions */

#define USBDEV_RXPLEN_PKTLENGTH      (0x000003ff)
#define USBDEV_RXPLEN_PKTLENGTH_MASK (0x000003ff)
#define USBDEV_RXPLEN_DV             (0x00000400)
#define USBDEV_RXPLEN_PKTRDY         (0x00000800)
#define USBDEV_RXPLEN_MASK           (0x00000fff)

/* TXPLEN bit definitions */

#define USBDEV_TXPLEN_PKTLENGTH      (0x000003ff)
#define USBDEV_TXPLEN_MASK           (0x000003ff)

/* USBCTRL bit definitions */

#define USBDEV_CTRL_RDEN             (0x00000001)
#define USBDEV_CTRL_WREN             (0x00000002)
#define USBDEV_CTRL_LOGENDPOINT      (0x0000003c)
#define USBDEV_CTRL_MASK             (0x0000003f)

/* CMDCODE bit definitions */

#define USBDEV_CMDCODE_CMDPHASE      (0x0000ff00)
#define USBDEV_CMDCODE_CMDCODE       (0x00ff0000)
#define USBDEV_CMDCODE_MASK          (0x00ffff00)

/* DMAINSTST/DMAINSTEN bit defintions */

#define USBDEV_DMAINST_EOT           (0x00000001)
#define USBDEV_DMAINST_NDDR          (0x00000002)
#define USBDEV_DMAINST_SE            (0x00000004)
#define USBDEV_DMAINST_MASK          (0x00000007)

/* Endpoints */

#define LPC214X_EP0_OUT                0
#define LPC214X_EP0_IN                 1
#define LPC214X_CTRLEP_OUT            LPC214X_EP0_OUT
#define LPC214X_CTRLEP_IN             LPC214X_EP0_IN
#define LPC214X_EP1_OUT                2
#define LPC214X_EP1_IN                 3
#define LPC214X_EP2_OUT                4
#define LPC214X_EP2_IN                 5
#define LPC214X_EP3_OUT                6
#define LPC214X_EP3_IN                 7
#define LPC214X_EP4_OUT                8
#define LPC214X_EP4_IN                 9
#define LPC214X_EP5_OUT               10
#define LPC214X_EP5_IN                11
#define LPC214X_EP6_OUT               12
#define LPC214X_EP6_IN                13
#define LPC214X_EP7_OUT               14
#define LPC214X_EP7_IN                15
#define LPC214X_EP8_OUT               16
#define LPC214X_EP8_IN                17
#define LPC214X_EP9_OUT               18
#define LPC214X_EP9_IN                19
#define LPC214X_EP10_OUT              20
#define LPC214X_EP10_IN               21
#define LPC214X_EP11_OUT              22
#define LPC214X_EP11_IN               23
#define LPC214X_EP12_OUT              24
#define LPC214X_EP12_IN               25
#define LPC214X_EP13_OUT              26
#define LPC214X_EP13_IN               27
#define LPC214X_EP14_OUT              28
#define LPC214X_EP14_IN               29
#define LPC214X_EP15_OUT              30
#define LPC214X_EP15_IN               31
#define LPC214X_NUMEPS                32

/* Mapping to more traditional endpoint numbers */

#define LPC214X_EP_LOG2PHYOUT(ep)    ((ep)&0x0f)<<1))
#define LPC214X_EP_LOG2PHYIN(ep)     (LPC214X_EP_OUT(ep) + 1)
#define LPC214X_EP_LOG2PHY(ep)       ((((ep)&0x0f)<<1)|(((ep)&0x80)>>7))

/* USB Command Code Register -- Command phase values */

#define CMD_USB_CMDWR                (0x00000500)
#define CMD_USB_DATAWR               (0x00000100)
#define CMD_USB_DATARD               (0x00000200)

/* Device Commands */

#define CMD_USB_DEV_SETADDRESS       (0x00d0)
#define CMD_USB_DEV_CONFIG           (0x00d8)
#define CMD_USB_DEV_SETMODE          (0x00f3)
#define CMD_USB_DEV_READFRAMENO      (0x00f5)
#define CMD_USB_DEV_READTESTREG      (0x00fd)
#define CMD_USB_DEV_SETSTATUS        (0x01fe)
#define CMD_USB_DEV_GETSTATUS        (0x00fe)
#define CMD_USB_DEV_GETERRORCODE     (0x00ff)
#define CMD_USB_DEV_READERRORSTATUS  (0x00fb)

/* Endpoint Commands */

#define CMD_USB_EP_SELECT            (0x0000)
#define CMD_USB_EP_SELECTCLEAR       (0x0040)
#define CMD_USB_EP_SETSTATUS         (0x0140)
#define CMD_USB_EP_CLRBUFFER         (0x00f2)
#define CMD_USB_EP_VALIDATEBUFFER    (0x00fa)

/* Device Status Bits */

#define USBDEV_DEVSTATUS_CONNECT     (0x00000001) /* Bit 0: Connected */
#define USBDEV_DEVSTATUS_CONNCHG     (0x00000002) /* Bit 1: Connect change */
#define USBDEV_DEVSTATUS_SUSPEND     (0x00000004) /* Bit 2: Suspend */
#define USBDEV_DEVSTATUS_SUSPCHG     (0x00000008) /* Bit 3: Suspend change */
#define USBDEV_DEVSTATUS_RESET       (0x00000002) /* Bit 4: Bus reset bit */

#define USBDEV_EPSTALL               (0x00000001)
#define USBDEV_EPSTALLSTATUS         (0x00000002)
#define USBDEV_EPSETUPPACKET         (0x00000004)
#define USBDEV_EPPOSSTATUS           (0x00000010)
#define USBDEV_EPCONDSTALL           (0x00000080)

/* USB Control register bit definitions */

#define LPC214X_USBCTRL_RDEN         (0x00000001)
#define LPC214X_USBCTRL_WREN         (0x00000002)

#define USBDEV_PACKETOVERWRITTEN      (0x00000001)

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC214X_LPC214X_USBDEV_H */
