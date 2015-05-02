/********************************************************************************************
 * arch/arm/src/sama5/sam_usbhost.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <nuttx/usb/usbhost_trace.h>

#include "sam_usbhost.h"

#ifdef HAVE_USBHOST_TRACE

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define TR_OHCI false
#define TR_EHCI true

#define TR_FMT1 false
#define TR_FMT2 true

#define TRENTRY(id,ehci,fmt1,string) {string}

#ifndef NULL
#  define NULL ((FAR void *)0)
#endif

/********************************************************************************************
 * Private Types
 ********************************************************************************************/

struct sam_usbhost_trace_s
{
#if 0
  uint16_t id;
  bool ehci;
  bool fmt2;
#endif
  FAR const char *string;
};

/********************************************************************************************
 * Private Data
 ********************************************************************************************/

static const struct sam_usbhost_trace_s g_trace1[TRACE1_NSTRINGS] =
{
#ifdef CONFIG_SAMA5_OHCI
  TRENTRY(OHCI_TRACE1_DEVDISCONN,          TR_OHCI, TR_FMT1, "OHCI ERROR: RHport%d Device disconnected\n"),
  TRENTRY(OHCI_TRACE1_INTRUNRECOVERABLE,   TR_OHCI, TR_FMT1, "OHCI ERROR: Unrecoverable error. pending: %06x\n"),
  TRENTRY(OHCI_TRACE1_INTRUNHANDLED,       TR_OHCI, TR_FMT1, "OHCI ERROR: Unhandled interrupts pending: %06x\n"),
  TRENTRY(OHCI_TRACE1_EPLISTALLOC_FAILED,  TR_OHCI, TR_FMT1, "OHCI ERROR: Failed to allocate EP list\n"),
  TRENTRY(OHCI_TRACE1_EDALLOC_FAILED,      TR_OHCI, TR_FMT1, "OHCI ERROR: Failed to allocate ED\n"),
  TRENTRY(OHCI_TRACE1_TDALLOC_FAILED,      TR_OHCI, TR_FMT1, "OHCI ERROR: Failed to allocate TD\n"),
  TRENTRY(OHCI_TRACE1_IRQATTACH,           TR_OHCI, TR_FMT1, "OHCI ERROR: Failed to attach IRQ%d\n"),
#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(OHCI_TRACE1_BADTDSTATUS,         TR_OHCI, TR_FMT1, "OHCI ERROR: Bad asynch TD completion status: %d\n"),
#endif

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  TRENTRY(OHCI_VTRACE1_PHYSED,             TR_OHCI, TR_FMT1, "OHCI physed: %06x\n"),
  TRENTRY(OHCI_VTRACE1_VIRTED,             TR_OHCI, TR_FMT1, "OHCI ed: %06x\n"),
  TRENTRY(OHCI_VTRACE1_CSC,                TR_OHCI, TR_FMT1, "OHCI Connect Status Change, RHSTATUS: %06x\n"),
  TRENTRY(OHCI_VTRACE1_DRWE,               TR_OHCI, TR_FMT1, "OHCI DRWE: Remote wake-up, RHSTATUS: %06x\n"),
  TRENTRY(OHCI_VTRACE1_ALREADYCONN,        TR_OHCI, TR_FMT1, "OHCI Already connected, RHPORTST: %06x\n"),
  TRENTRY(OHCI_VTRACE1_SPEED,              TR_OHCI, TR_FMT1, "OHCI Port speed: %d\n"),
  TRENTRY(OHCI_VTRACE1_ALREADYDISCONN,     TR_OHCI, TR_FMT1, "OHCI Already disconnected, RHPORTST: %06x\n"),
  TRENTRY(OHCI_VTRACE1_RHSC,               TR_OHCI, TR_FMT1, "OHCI Root Hub Status Change. Pending: %06x\n"),
  TRENTRY(OHCI_VTRACE1_WDHINTR,            TR_OHCI, TR_FMT1, "OHCI Writeback Done Head interrupt. Pending: %06x\n"),
  TRENTRY(OHCI_VTRACE1_CLASSENUM,          TR_OHCI, TR_FMT1, "OHCI Hub port %d: Enumerate device\n"),
  TRENTRY(OHCI_VTRACE1_ENUMDISCONN,        TR_OHCI, TR_FMT1, "OHCI RHport%dNot connected\n"),
  TRENTRY(OHCI_VTRACE1_INITIALIZING,       TR_OHCI, TR_FMT1, "OHCI Initializing Stack\n"),
  TRENTRY(OHCI_VTRACE1_INITIALIZED,        TR_OHCI, TR_FMT1, "OHCI Initialized\n"),
  TRENTRY(OHCI_VTRACE1_INTRPENDING,        TR_OHCI, TR_FMT1, "OHCI Interrupts pending: %06x\n"),
#endif
#endif

#ifdef CONFIG_SAMA5_EHCI
  TRENTRY(EHCI_TRACE1_SYSTEMERROR,         TR_EHCI, TR_FMT1, "EHCI ERROR: System error: %06x\n"),
  TRENTRY(EHCI_TRACE1_QTDFOREACH_FAILED,   TR_EHCI, TR_FMT1, "EHCI ERROR: sam_qtd_foreach failed: %d\n"),
  TRENTRY(EHCI_TRACE1_QHALLOC_FAILED,      TR_EHCI, TR_FMT1, "EHCI ERROR: Failed to allocate a QH\n"),
  TRENTRY(EHCI_TRACE1_BUFTOOBIG,           TR_EHCI, TR_FMT1, "EHCI ERROR: Buffer too big. Remaining %d\n"),
  TRENTRY(EHCI_TRACE1_REQQTDALLOC_FAILED,  TR_EHCI, TR_FMT1, "EHCI ERROR: Failed to allocate request qTD"),
  TRENTRY(EHCI_TRACE1_ADDBPL_FAILED,       TR_EHCI, TR_FMT1, "EHCI ERROR: sam_qtd_addbpl failed: %d\n"),
  TRENTRY(EHCI_TRACE1_DATAQTDALLOC_FAILED, TR_EHCI, TR_FMT1, "EHCI ERROR: Failed to allocate data buffer qTD, 0"),
  TRENTRY(EHCI_TRACE1_DEVDISCONNECTED,     TR_EHCI, TR_FMT1, "EHCI ERROR: Device disconnected %d\n"),
  TRENTRY(EHCI_TRACE1_QHCREATE_FAILED,     TR_EHCI, TR_FMT1, "EHCI ERROR: sam_qh_create failed\n"),
  TRENTRY(EHCI_TRACE1_QTDSETUP_FAILED,     TR_EHCI, TR_FMT1, "EHCI ERROR: sam_qtd_setupphase failed\n"),
  TRENTRY(EHCI_TRACE1_QTDDATA_FAILED,      TR_EHCI, TR_FMT1, "EHCI ERROR: sam_qtd_dataphase failed\n"),
  TRENTRY(EHCI_TRACE1_QTDSTATUS_FAILED,    TR_EHCI, TR_FMT1, "EHCI ERROR: sam_qtd_statusphase failed\n"),
  TRENTRY(EHCI_TRACE1_TRANSFER_FAILED,     TR_EHCI, TR_FMT1, "EHCI ERROR: Transfer failed %d\n"),
  TRENTRY(EHCI_TRACE1_QHFOREACH_FAILED,    TR_EHCI, TR_FMT1, "EHCI ERROR: sam_qh_foreach failed: %d\n"),
  TRENTRY(EHCI_TRACE1_SYSERR_INTR,         TR_EHCI, TR_FMT1, "EHCI: Host System Error Interrupt\n"),
  TRENTRY(EHCI_TRACE1_USBERR_INTR,         TR_EHCI, TR_FMT1, "EHCI: USB Error Interrupt (USBERRINT) Interrupt: %06x\n"),
  TRENTRY(EHCI_TRACE1_EPALLOC_FAILED,      TR_EHCI, TR_FMT1, "EHCI ERROR: Failed to allocate EP info structure\n"),
  TRENTRY(EHCI_TRACE1_BADXFRTYPE,          TR_EHCI, TR_FMT1, "EHCI ERROR: Support for transfer type %d not implemented\n"),
  TRENTRY(EHCI_TRACE1_HCHALTED_TIMEOUT,    TR_EHCI, TR_FMT1, "EHCI ERROR: Timed out waiting for HCHalted. USBSTS: %06x\n"),
  TRENTRY(EHCI_TRACE1_QHPOOLALLOC_FAILED,  TR_EHCI, TR_FMT1, "EHCI ERROR: Failed to allocate the QH pool\n"),
  TRENTRY(EHCI_TRACE1_QTDPOOLALLOC_FAILED, TR_EHCI, TR_FMT1, "EHCI ERROR: Failed to allocate the qTD pool\n"),
  TRENTRY(EHCI_TRACE1_PERFLALLOC_FAILED,   TR_EHCI, TR_FMT1, "EHCI ERROR: Failed to allocate the periodic frame list\n"),
  TRENTRY(EHCI_TRACE1_RESET_FAILED,        TR_EHCI, TR_FMT1, "EHCI ERROR: sam_reset failed: %d\n"),
  TRENTRY(EHCI_TRACE1_RUN_FAILED,          TR_EHCI, TR_FMT1, "EHCI ERROR: EHCI Failed to run: USBSTS=%06x\n"),
  TRENTRY(EHCI_TRACE1_IRQATTACH_FAILED,    TR_EHCI, TR_FMT1, "EHCI ERROR: Failed to attach IRQ%d\n"),

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  TRENTRY(EHCI_VTRACE1_PORTSC_CSC,         TR_EHCI, TR_FMT1, "EHCI Connect Status Change: %06x\n"),
  TRENTRY(EHCI_VTRACE1_PORTSC_CONNALREADY, TR_EHCI, TR_FMT1, "EHCI Already connected: %06x\n"),
  TRENTRY(EHCI_VTRACE1_PORTSC_DISCALREADY, TR_EHCI, TR_FMT1, "EHCI Already disconnected: %06x\n"),
  TRENTRY(EHCI_VTRACE1_TOPHALF,            TR_EHCI, TR_FMT1, "EHCI Interrupt: %06x\n"),
  TRENTRY(EHCI_VTRACE1_AAINTR,             TR_EHCI, TR_FMT1, "EHCI Async Advance Interrupt\n"),
  TRENTRY(EHCI_VTRACE1_USBINTR,            TR_EHCI, TR_FMT1, "EHCI USB Interrupt (USBINT) Interrupt: %06x\n"),
  TRENTRY(EHCI_VTRACE1_CLASSENUM,          TR_EHCI, TR_FMT1, "EHCI Hub port %d: Enumerate device\n"),
  TRENTRY(EHCI_VTRACE1_ENUM_DISCONN,       TR_EHCI, TR_FMT1, "EHCI Enumeration not connected\n"),
  TRENTRY(EHCI_VTRACE1_INITIALIZING,       TR_EHCI, TR_FMT1, "EHCI Initializing EHCI Stack\n"),
  TRENTRY(EHCI_VTRACE1_HCCPARAMS,          TR_EHCI, TR_FMT1, "EHCI HCCPARAMS=%06x\n"),
  TRENTRY(EHCI_VTRACE1_INIITIALIZED,       TR_EHCI, TR_FMT1, "EHCI USB EHCI Initialized\n"),
#endif
#endif
};

static const struct sam_usbhost_trace_s g_trace2[TRACE2_NSTRINGS] =
{
#ifdef CONFIG_SAMA5_OHCI
  TRENTRY(OHCI_TRACE2_BADTDSTATUS,         TR_OHCI, TR_FMT2, "OHCI ERROR: RHport%d Bad TD completion status: %d\n"),
  TRENTRY(OHCI_TRACE2_WHDTDSTATUS,         TR_OHCI, TR_FMT2, "OHCI ERROR: WHD Bad TD completion status: %d xfrtype: %d\n"),
  TRENTRY(OHCI_TRACE2_EP0ENQUEUE_FAILED,   TR_OHCI, TR_FMT2, "OHCI ERROR: RHport%d Failed to enqueue EP0: %d\n"),
  TRENTRY(OHCI_TRACE2_EDENQUEUE_FAILED,    TR_OHCI, TR_FMT2, "OHCI ERROR: Failed to queue ED for transfer type %d: %d\n"),
  TRENTRY(OHCI_TRACE2_CLASSENUM_FAILED,    TR_OHCI, TR_FMT2, "OHCI Hub port %d usbhost_enumerate() failed: %d\n"),

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  TRENTRY(OHCI_VTRACE2_EP0CONFIG,          TR_OHCI, TR_FMT2, "OHCI EP0 configure speed=%d funcaddr=%d\n"),
  TRENTRY(OHCI_VTRACE2_INTERVAL,           TR_OHCI, TR_FMT2, "OHCI interval: %d->%d\n"),
  TRENTRY(OHCI_VTRACE2_MININTERVAL,        TR_OHCI, TR_FMT2, "OHCI MIN interval: %d offset: %d\n"),
  TRENTRY(OHCI_VTRACE2_RHPORTST,           TR_OHCI, TR_FMT2, "OHCI RHPORTST%d: %04x\n"),
  TRENTRY(OHCI_VTRACE2_CONNECTED,          TR_OHCI, TR_FMT2, "OHCI RHPort%d connected, rhswait: %d\n"),
  TRENTRY(OHCI_VTRACE2_DISCONNECTED,       TR_OHCI, TR_FMT2, "OHCI RHPort%d disconnected, rhswait: %d\n"),
  TRENTRY(OHCI_VTRACE2_WAKEUP,             TR_OHCI, TR_FMT2, "OHCI RHPort%d connected: %d\n"),
  TRENTRY(OHCI_VTRACE2_EP0CTRLED,          TR_OHCI, TR_FMT2, "OHCI RHPort%d EP0 CTRL: %04x\n"),
  TRENTRY(OHCI_VTRACE2_EPALLOC,            TR_OHCI, TR_FMT2, "OHCI EP%d CTRL: %04x\n"),
  TRENTRY(OHCI_VTRACE2_CTRLIN,             TR_OHCI, TR_FMT2, "OHCI CTRLIN RHPort%d req: %02x\n"),
  TRENTRY(OHCI_VTRACE2_CTRLOUT,            TR_OHCI, TR_FMT2, "OHCI CTRLOUT RHPort%d req: %02x\n"),
  TRENTRY(OHCI_VTRACE2_TRANSFER,           TR_OHCI, TR_FMT2, "OHCI EP%d buflen: %d\n"),
  TRENTRY(OHCI_VTRACE2_INITCONNECTED,      TR_OHCI, TR_FMT2, "OHCI RHPort%d Device connected: %d\n"),
#ifdef CONFIG_USBHOST_HUB
  TRENTRY(OHCI_VTRACE2_HUBWAKEUP,          TR_OHCI, TR_FMT2, "OHCI Hub Port%d connected: %d\n"),
#endif
#endif
#endif

#ifdef CONFIG_SAMA5_EHCI
  TRENTRY(EHCI_TRACE2_EPSTALLED,           TR_EHCI, TR_FMT2, "EHCI EP%d Stalled: TOKEN=%04x\n"),
  TRENTRY(EHCI_TRACE2_EPIOERROR,           TR_EHCI, TR_FMT2, "EHCI ERROR: EP%d TOKEN=%04x\n"),
  TRENTRY(EHCI_TRACE2_CLASSENUM_FAILED,    TR_EHCI, TR_FMT2, "EHCI Hub port %d usbhost_enumerate() failed: %d\n"),

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  TRENTRY(EHCI_VTRACE2_EP0CONFIG,          TR_EHCI, TR_FMT2, "EHCI EP0 configure speed=%d funcaddr=%d\n"),
  TRENTRY(EHCI_VTRACE2_ASYNCXFR,           TR_EHCI, TR_FMT2, "EHCI Async transfer EP%d buflen=%d\n"),
  TRENTRY(EHCI_VTRACE2_INTRXFR,            TR_EHCI, TR_FMT2, "EHCI Intr Transfer EP%d buflen=%d\n"),
  TRENTRY(EHCI_VTRACE2_IOCCHECK,           TR_EHCI, TR_FMT2, "EHCI IOC EP%d TOKEN=%04x\n"),
  TRENTRY(EHCI_VTRACE2_PORTSC,             TR_EHCI, TR_FMT2, "EHCI PORTSC%d: %04x\n"),
  TRENTRY(EHCI_VTRACE2_PORTSC_CONNECTED,   TR_EHCI, TR_FMT2, "EHCI RHPort%d connected, pscwait: %d\n"),
  TRENTRY(EHCI_VTRACE2_PORTSC_DISCONND,    TR_EHCI, TR_FMT2, "EHCI RHport%d disconnected, pscwait: %d\n"),
  TRENTRY(EHCI_VTRACE2_MONWAKEUP,          TR_EHCI, TR_FMT2, "EHCI Hub port%d connected: %d\n"),
  TRENTRY(EHCI_VTRACE2_EPALLOC,            TR_EHCI, TR_FMT2, "EHCI EPALLOC: EP%d TYPE=%d\n"),
  TRENTRY(EHCI_VTRACE2_CTRLINOUT,          TR_EHCI, TR_FMT2, "EHCI CTRLIN/OUT: RHPort%d req: %02x\n"),
  TRENTRY(EHCI_VTRACE2_HCIVERSION,         TR_EHCI, TR_FMT2, "EHCI HCIVERSION %x.%02x\n"),
  TRENTRY(EHCI_VTRACE2_HCSPARAMS,          TR_EHCI, TR_FMT2, "EHCI nports=%d, HCSPARAMS=%04x\n"),
#endif
#endif
};

/********************************************************************************************
 * Private Function Prototypes
 ********************************************************************************************/

/********************************************************************************************
 * Global Functions
 ********************************************************************************************/

/********************************************************************************************
 * Name: usbhost_trformat1 and usbhost_trformat2
 *
 * Description:
 *   This interface must be provided by platform specific logic that knows
 *   the HCDs encoding of USB trace data.
 *
 *   Given an 9-bit index, return a format string suitable for use with, say,
 *   printf.  The returned format is expected to handle two unsigned integer
 *   values.
 *
 ********************************************************************************************/

FAR const char *usbhost_trformat1(uint16_t id)
{
  int ndx = TRACE1_INDEX(id);

  if (ndx < TRACE1_NSTRINGS)
    {
      return g_trace1[ndx].string;
    }

  return NULL;
}

FAR const char *usbhost_trformat2(uint16_t id)
{
  int ndx = TRACE2_INDEX(id);

  if (ndx < TRACE2_NSTRINGS)
    {
      return g_trace2[ndx].string;
    }

  return NULL;
}

#endif /* HAVE_USBHOST_TRACE */

