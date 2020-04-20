/********************************************************************************************
 * arch/arm/src/stm32l4/stm32l4_usbhost_trace.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           dev@ziggurat29.com
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

#include "stm32l4_usbhost.h"

#ifdef HAVE_USBHOST_TRACE

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define TR_FMT1 false
#define TR_FMT2 true

#define TRENTRY(id,fmt1,string) {string}

#ifndef NULL
#  define NULL ((FAR void *)0)
#endif

/********************************************************************************************
 * Private Types
 ********************************************************************************************/

struct stm32l4_usbhost_trace_s
{
#if 0
  uint16_t id;
  bool fmt2;
#endif
  FAR const char *string;
};

/********************************************************************************************
 * Private Data
 ********************************************************************************************/

static const struct stm32l4_usbhost_trace_s g_trace1[TRACE1_NSTRINGS] =
{
  TRENTRY(OTGFS_TRACE1_DEVDISCONN,         TR_FMT1,
          "OTGFS ERROR: Host Port %d. Device disconnected\n"),
  TRENTRY(OTGFS_TRACE1_IRQATTACH,          TR_FMT1,
          "OTGFS ERROR: Failed to attach IRQ\n"),
  TRENTRY(OTGFS_TRACE1_TRNSFRFAILED,       TR_FMT1,
          "OTGFS  ERROR: Transfer Failed. ret=%d\n"),
  TRENTRY(OTGFS_TRACE1_SENDSETUP,          TR_FMT1,
          "OTGFS  ERROR: ctrl_sendsetup() failed with: %d\n"),
  TRENTRY(OTGFS_TRACE1_SENDDATA,           TR_FMT1,
          "OTGFS  ERROR: ctrl_senddata() failed with: %d\n"),
  TRENTRY(OTGFS_TRACE1_RECVDATA,           TR_FMT1,
          "OTGFS  ERROR: ctrl_recvdata() failed with: %d\n"),

#ifdef HAVE_USBHOST_TRACE_VERBOSE

  TRENTRY(OTGFS_VTRACE1_CONNECTED,         TR_FMT1,
          "OTGFS Host Port %d connected.\n"),
  TRENTRY(OTGFS_VTRACE1_DISCONNECTED,      TR_FMT1,
          "OTGFS Host Port %d disconnected.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT,              TR_FMT1,
          "OTGFS Handling Interrupt. Entry Point.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_SOF,          TR_FMT1,
          "OTGFS Handle the start of frame interrupt.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_RXFLVL,       TR_FMT1,
          "OTGFS Handle the RxFIFO non-empty interrupt.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_NPTXFE,       TR_FMT1,
          "OTGFS Handle the non-periodic TxFIFO empty interrupt.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_PTXFE,        TR_FMT1,
          "OTGFS Handle the periodic TxFIFO empty interrupt.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_HC,           TR_FMT1,
          "OTGFS Handle the host channels interrupt.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_HPRT,         TR_FMT1,
          "OTGFS Handle the host port interrupt.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_HPRT_POCCHNG, TR_FMT1,
          "OTGFS  HPRT: Port Over-Current Change.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_HPRT_PCDET,   TR_FMT1,
          "OTGFS  HPRT: Port Connect Detect.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_HPRT_PENCHNG, TR_FMT1,
          "OTGFS  HPRT: Port Enable Changed.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_HPRT_LSDEV,   TR_FMT1,
          "OTGFS  HPRT: Low Speed Device Connected.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_HPRT_FSDEV,   TR_FMT1,
          "OTGFS  HPRT: Full Speed Device Connected.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_HPRT_LSFSSW,  TR_FMT1,
          "OTGFS  HPRT: Host Switch: LS -> FS.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_HPRT_FSLSSW,  TR_FMT1,
          "OTGFS  HPRT: Host Switch: FS -> LS.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_DISC,         TR_FMT1,
          "OTGFS Handle the disconnect detected interrupt.\n"),
  TRENTRY(OTGFS_VTRACE1_GINT_IPXFR,        TR_FMT1,
          "OTGFS Handle the incomplete periodic transfer.\n"),

#endif
};

static const struct stm32l4_usbhost_trace_s g_trace2[TRACE2_NSTRINGS] =
{
  TRENTRY(OTGFS_TRACE2_CLIP,                TR_FMT2,
          "OTGFS CLIP: chidx: %d buflen: %d\n"),

#ifdef HAVE_USBHOST_TRACE_VERBOSE

  TRENTRY(OTGFS_VTRACE2_CHANWAKEUP_IN,      TR_FMT2,
          "OTGFS  EP%d(IN)  wake up with result: %d\n"),
  TRENTRY(OTGFS_VTRACE2_CHANWAKEUP_OUT,     TR_FMT2,
          "OTGFS  EP%d(OUT) wake up with result: %d\n"),
  TRENTRY(OTGFS_VTRACE2_CTRLIN,             TR_FMT2,
          "OTGFS CTRL_IN  type: %02x req: %02x\n"),
  TRENTRY(OTGFS_VTRACE2_CTRLOUT,            TR_FMT2,
          "OTGFS CTRL_OUT type: %02x req: %02x\n"),
  TRENTRY(OTGFS_VTRACE2_INTRIN,             TR_FMT2,
          "OTGFS INTR_IN  chidx: %02x len: %02x\n"),
  TRENTRY(OTGFS_VTRACE2_INTROUT,            TR_FMT2,
          "OTGFS INTR_OUT chidx: %02x len: %02x\n"),
  TRENTRY(OTGFS_VTRACE2_BULKIN,             TR_FMT2,
          "OTGFS BULK_IN  chidx: %02x len: %02x\n"),
  TRENTRY(OTGFS_VTRACE2_BULKOUT,            TR_FMT2,
          "OTGFS BULK_OUT chidx: %02x len: %02x\n"),
  TRENTRY(OTGFS_VTRACE2_ISOCIN,             TR_FMT2,
          "OTGFS ISOC_IN  chidx: %02x len: %04d\n"),
  TRENTRY(OTGFS_VTRACE2_ISOCOUT,            TR_FMT2,
          "OTGFS ISOC_OUT chidx: %02x req: %02x\n"),
  TRENTRY(OTGFS_VTRACE2_STARTTRANSFER,      TR_FMT2,
          "OTGFS  Transfer chidx: %d buflen: %d\n"),
  TRENTRY(OTGFS_VTRACE2_CHANCONF_CTRL_IN,   TR_FMT2,
          "OTGFS Channel configured. chidx: %d: (EP%d,IN ,CTRL)\n"),
  TRENTRY(OTGFS_VTRACE2_CHANCONF_CTRL_OUT,  TR_FMT2,
          "OTGFS Channel configured. chidx: %d: (EP%d,OUT,CTRL)\n"),
  TRENTRY(OTGFS_VTRACE2_CHANCONF_INTR_IN,   TR_FMT2,
          "OTGFS Channel configured. chidx: %d: (EP%d,IN ,INTR)\n"),
  TRENTRY(OTGFS_VTRACE2_CHANCONF_INTR_OUT,  TR_FMT2,
          "OTGFS Channel configured. chidx: %d: (EP%d,OUT,INTR)\n"),
  TRENTRY(OTGFS_VTRACE2_CHANCONF_BULK_IN,   TR_FMT2,
          "OTGFS Channel configured. chidx: %d: (EP%d,IN ,BULK)\n"),
  TRENTRY(OTGFS_VTRACE2_CHANCONF_BULK_OUT,  TR_FMT2,
          "OTGFS Channel configured. chidx: %d: (EP%d,OUT,BULK)\n"),
  TRENTRY(OTGFS_VTRACE2_CHANCONF_ISOC_IN,   TR_FMT2,
          "OTGFS Channel configured. chidx: %d: (EP%d,IN ,ISOC)\n"),
  TRENTRY(OTGFS_VTRACE2_CHANCONF_ISOC_OUT,  TR_FMT2,
          "OTGFS Channel configured. chidx: %d: (EP%d,OUT,ISOC)\n"),
  TRENTRY(OTGFS_VTRACE2_CHANHALT, TR_FMT2,
          "OTGFS Channel halted. chidx: %d, reason: %d\n"),

#endif
};

/********************************************************************************************
 * Private Function Prototypes
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
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
