/********************************************************************************************
 * arch/arm/src/sama5/sam_host.c
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

#if defined(CONFIG_USBHOST_TRACE) || \
   (defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_USB))

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
  TRENTRY(EHCI_TRACE1_USBERR_INTR,         TR_EHCI, TR_FMT1, "EHCI: USB Error Interrupt (USBERRINT) Interrupt\n"),
  TRENTRY(EHCI_TRACE1_EPALLOC_FAILED,      TR_EHCI, TR_FMT1, "EHCI ERROR: Failed to allocate EP info structure\n"),
  TRENTRY(EHCI_TRACE1_BADXFRTYPE,          TR_EHCI, TR_FMT1, "EHCI ERROR: Support for transfer type %d not implemented\n"),
  TRENTRY(EHCI_TRACE1_HCHALTED_TIMEOUT,    TR_EHCI, TR_FMT1, "EHCI ERROR: Timed out waiting for HCHalted. USBSTS: %06x\n"),
  TRENTRY(EHCI_TRACE1_QHPOOLALLOC_FAILED,  TR_EHCI, TR_FMT1, "EHCI ERROR: Failed to allocate the QH pool\n"),
  TRENTRY(EHCI_TRACE1_QTDPOOLALLOC_FAILED, TR_EHCI, TR_FMT1, "EHCI ERROR: Failed to allocate the qTD pool\n"),
  TRENTRY(EHCI_TRACE1_PERFLALLOC_FAILED,   TR_EHCI, TR_FMT1, "EHCI ERROR: Failed to allocate the periodic frame list\n"),
  TRENTRY(EHCI_TRACE1_RESET_FAILED,        TR_EHCI, TR_FMT1, "EHCI ERROR: sam_reset failed: %d\n"),
  TRENTRY(EHCI_TRACE1_RUN_FAILED,          TR_EHCI, TR_FMT1, "EHCI ERROR: EHCI Failed to run: USBSTS=%08x\n"),
  TRENTRY(EHCI_TRACE1_IRQATTACH_FAILED,    TR_EHCI, TR_FMT1, "EHCI ERROR: Failed to attach IRQ%d\n"),
};

static const struct sam_usbhost_trace_s g_trace2[TRACE2_NSTRINGS] =
{
  TRENTRY(EHCI_TRACE2_EPSTALLED,           TR_EHCI, TR_FMT2, "EHCI EP%d Stalled: TOKEN=%08x\n"),
  TRENTRY(EHCI_TRACE2_EPIOERROR,           TR_EHCI, TR_FMT2, "EHCI ERROR: EP%d TOKEN=%08x\n"),
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

#endif /* CONFIG_USBHOST_TRACE || CONFIG_DEBUG && CONFIG_DEBUG_USB */

