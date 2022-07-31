/****************************************************************************
 * arch/arm/src/samd5e5/sam_usbhost.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include <nuttx/usb/usbhost_trace.h>

#include "sam_usbhost.h"

#ifdef HAVE_USBHOST_TRACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define TR_FMT1 false
#define TR_FMT2 true

#define TRENTRY(id,fmt1,string) {string}

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_usbhost_trace_s
{
  const char *string;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sam_usbhost_trace_s g_trace1[TRACE1_NSTRINGS] =
{
  TRENTRY(SAM_TRACE1_ALLOC_FAIL, TR_FMT1,
        "INIT: Failed to allocate state structure: %u\n"),

#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(SAM_TRACE1_ASYNCHSETUP_FAIL1, TR_FMT1,
          "OUT: Asynch setup failed: %u\n"),
  TRENTRY(SAM_TRACE1_ASYNCHSETUP_FAIL2, TR_FMT1,
          "IN: Asynch setup failed: %u\n"),
#endif

  TRENTRY(SAM_TRACE1_BAD_JKSTATE,       TR_FMT1,
          "CONNECT: Bad JK state: %02x\n"),
  TRENTRY(SAM_TRACE1_BADREVISION,       TR_FMT1,
          "INIT: Bad revision number:  %02x\n"),
  TRENTRY(SAM_TRACE1_PIPEALLOC_FAIL,    TR_FMT1,
          "EPALLOC: Pipe allocation failed: %u\n"),
  TRENTRY(SAM_TRACE1_PIPEWAIT_FAIL,     TR_FMT1,
          "OUT: Pipe wait failure: %u\n"),
  TRENTRY(SAM_TRACE1_DEVDISCONN1,       TR_FMT1,
          "OUT: Disconnected during wait: %u\n"),
  TRENTRY(SAM_TRACE1_DEVDISCONN2,       TR_FMT1,
          "CTRL: Disconnected during SETUP phase: %u\n"),
  TRENTRY(SAM_TRACE1_DEVDISCONN3,       TR_FMT1,
          "CTRL OUT: Disconnected during DATA phase: %u\n"),
  TRENTRY(SAM_TRACE1_DEVDISCONN4,       TR_FMT1,
          "CTRL IN: Disconnected during DATA phase: %u"),
  TRENTRY(SAM_TRACE1_DEVDISCONN5,       TR_FMT1,
          "IN: Disconnected during wait: %u\n"),
  TRENTRY(SAM_TRACE1_DEVDISCONN6,       TR_FMT1,
          "CONNECT: Device disconnect #1: %u\n"),
  TRENTRY(SAM_TRACE1_DEVDISCONN7,       TR_FMT1,
          "CONNECT: Device disconnect #2: %u\n"),
  TRENTRY(SAM_TRACE1_DEVDISCONN8,       TR_FMT1,
          "CONNECT: Device disconnect #3: %u\n"),
  TRENTRY(SAM_TRACE1_ENUMERATE_FAIL,    TR_FMT1,
          "CONNECT: Enumeration failed: %u\n"),
  TRENTRY(SAM_TRACE1_INSETUP_FAIL1,     TR_FMT1,
          "CTRL IN: SETUP phase failure: %u\n"),

#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(SAM_TRACE1_INSETUP_FAIL2,     TR_FMT1,
          "CTRL IN: Asynch SETUP phase failure #1: %u\n"),
  TRENTRY(SAM_TRACE1_INSETUP_FAIL3,     TR_FMT1,
          "CTRL IN: Asynch SETUP phase failure #2: %u\n"),
#endif

  TRENTRY(SAM_TRACE1_IRQATTACH_FAIL,    TR_FMT1,
          "INIT: Failed to attach interrupt: %u\n"),
  TRENTRY(SAM_TRACE1_OUTSETUP_FAIL1,    TR_FMT1,
          "CTRL OUT: SETUP phase failure: %u\n"),

#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(SAM_TRACE1_OUTSETUP_FAIL2,    TR_FMT1,
          "CTRL OUT: Asynch SETUP phase failure #1: %u\n"),
  TRENTRY(SAM_TRACE1_OUTSETUP_FAIL3,    TR_FMT1,
          "CTRL OUT: Asynch SETUP phase failure #2: %u\n"),
#endif

  TRENTRY(SAM_TRACE1_RECVDATA_FAIL,     TR_FMT1,
          "CTRL IN: Data phase failure: %u\n"),
  TRENTRY(SAM_TRACE1_RECVSTATUS_FAIL,   TR_FMT1,
          "CTRL OUT: Status phase failure: %u\n"),
  TRENTRY(SAM_TRACE1_SENDDATA_FAIL,     TR_FMT1,
          "CTRL OUT: Data phase failure: %u\n"),
  TRENTRY(SAM_TRACE1_SENDSETUP_FAIL1,   TR_FMT1,
          "CTRL OUT: SETUP phase failure: %u\n"),
  TRENTRY(SAM_TRACE1_SENDSETUP_FAIL2,   TR_FMT1,
          "CTRL IN: SETUP phase failure: %u\n"),
  TRENTRY(SAM_TRACE1_SENDSTATUS_FAIL,   TR_FMT1,
          "CTRL IN: Status phase failure: %u\n"),
  TRENTRY(SAM_TRACE1_TRANSFER_FAILED1,  TR_FMT1,
          "OUT: Transfer wait returned failure: %u\n"),
  TRENTRY(SAM_TRACE1_TRANSFER_FAILED2,  TR_FMT1,
          "CTRL: SETUP wait returned failure: %u\n"),
  TRENTRY(SAM_TRACE1_TRANSFER_FAILED3,  TR_FMT1,
          "IN: Transfer wait returned failure: %u\n"),

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  TRENTRY(SAM_VTRACE1_CANCEL,           TR_FMT1,
          "Transfer canceled: EP%u\n"),
  TRENTRY(SAM_VTRACE1_CONNECTED1,       TR_FMT1,
          "CONNECT: Connection event: %u\n"),
  TRENTRY(SAM_VTRACE1_CONNECTED2,       TR_FMT1,
          "CONNECT: Connection change detected: %u\n"),
  TRENTRY(SAM_VTRACE1_CONNECTED3,       TR_FMT1,
          "CONNECT: Connected: %u\n"),
  TRENTRY(SAM_VTRACE1_DISCONNECTED1,    TR_FMT1,
          "CONNECT: Disconnected: %u\n"),
  TRENTRY(SAM_VTRACE1_DISCONNECTED2,    TR_FMT1,
          "CONNECT: Disconnect detected: %u\n"),
  TRENTRY(SAM_VTRACE1_ENUMERATE,        TR_FMT1,
          "ENUMERATE: Start: %u\n"),

#ifdef CONFIG_USBHOST_HUB
  TRENTRY(SAM_VTRACE1_HUB_CONNECTED,    TR_FMT1,
          "CONNECT: Hub connected: %u\n"),
#endif
  TRENTRY(SAM_VTRACE1_INITIALIZED,      TR_FMT1,
          "INIT: Hardware initialized: %u\n"),
#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(SAM_VTRACE1_TRANSFER_COMPLETE, TR_FMT1,
          "OUT: Asynch transfer complete: %u\n"),
#endif
#endif

  TRENTRY(TRACE1_DEVDISCONN,          TR_FMT1,
          "ERROR: RHport%d Device disconnected\n"),
  TRENTRY(TRACE1_INTRUNRECOVERABLE,   TR_FMT1,
          "ERROR: Unrecoverable error. pending: %06x\n"),
  TRENTRY(TRACE1_INTRUNHANDLED,       TR_FMT1,
          "ERROR: Unhandled interrupts pending: %06x\n"),
  TRENTRY(TRACE1_EPLISTALLOC_FAILED,  TR_FMT1,
          "ERROR: Failed to allocate EP list\n"),
  TRENTRY(TRACE1_EDALLOC_FAILED,      TR_FMT1,
          "ERROR: Failed to allocate ED\n"),
  TRENTRY(TRACE1_TDALLOC_FAILED,      TR_FMT1,
          "ERROR: Failed to allocate TD\n"),
  TRENTRY(TRACE1_IRQATTACH,           TR_FMT1,
          "ERROR: Failed to attach IRQ%d\n"),
#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(TRACE1_BADTDSTATUS,         TR_FMT1,
        "ERROR: Bad asynch TD completion status: %d\n"),
#endif

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  TRENTRY(VTRACE1_PHYSED,             TR_FMT1,
        "physed: %06x\n"),
  TRENTRY(VTRACE1_VIRTED,             TR_FMT1,
        "ed: %06x\n"),
  TRENTRY(VTRACE1_CSC,                TR_FMT1,
        "Connect Status Change, RHSTATUS: %06x\n"),
  TRENTRY(VTRACE1_DRWE,               TR_FMT1,
        "DRWE: Remote wake-up, RHSTATUS: %06x\n"),
  TRENTRY(VTRACE1_ALREADYCONN,        TR_FMT1,
        "Already connected, RHPORTST: %06x\n"),
  TRENTRY(VTRACE1_SPEED,              TR_FMT1,
        "Port speed: %d\n"),
  TRENTRY(VTRACE1_ALREADYDISCONN,     TR_FMT1,
        "Already disconnected, RHPORTST: %06x\n"),
  TRENTRY(VTRACE1_RHSC,               TR_FMT1,
        "Root Hub Status Change. Pending: %06x\n"),
  TRENTRY(VTRACE1_WDHINTR,            TR_FMT1,
        "Writeback Done Head interrupt. Pending: %06x\n"),
  TRENTRY(VTRACE1_CLASSENUM,          TR_FMT1,
        "Hub port %d: Enumerate device\n"),
  TRENTRY(VTRACE1_ENUMDISCONN,        TR_FMT1,
        "RHport%dNot connected\n"),
  TRENTRY(VTRACE1_INITIALIZING,       TR_FMT1,
        "Initializing Stack\n"),
  TRENTRY(VTRACE1_INITIALIZED,        TR_FMT1,
        "Initialized\n"),
  TRENTRY(VTRACE1_INTRPENDING,        TR_FMT1,
        "Interrupts pending: %06x\n"),
#endif
};

static const struct sam_usbhost_trace_s g_trace2[TRACE2_NSTRINGS] =
{
#ifdef HAVE_USBHOST_TRACE_VERBOSE
#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(SAM_VTRACE2_ASYNCH,           TR_FMT2,
      "ASYNCH: Transfer started: EP%u len=%u\n"),
#endif
  TRENTRY(SAM_VTRACE2_BULKIN,           TR_FMT2,
      "BULK IN: SETUP: pipe%u len=%u\n"),
  TRENTRY(SAM_VTRACE2_BULKOUT,          TR_FMT2,
      "BULK OUT: SETUP: pipe%u len=%u\n"),
  TRENTRY(SAM_VTRACE2_PIPEWAKEUP_IN,    TR_FMT2,
      "IN: Pipe Wakeup: pipe%u, result=%u\n"),
  TRENTRY(SAM_VTRACE2_PIPEWAKEUP_OUT,   TR_FMT2,
      "OUT: Pipe Wakeup: pipe%u result=%u\n"),
  TRENTRY(SAM_VTRACE2_CTRLIN,           TR_FMT2,
      "CTRL IN: Start: type=%u req=%u\n"),
  TRENTRY(SAM_VTRACE2_CTRLOUT,          TR_FMT2,
      "CTRL OUT: Start: type=%u req=%u\n"),
#ifdef CONFIG_USBHOST_HUB
  TRENTRY(SAM_VTRACE2_HUB_CONNECTED,    TR_FMT2,
      "CONNECT: Hub connected: port=%u, connected=%u\n"),
#endif
  TRENTRY(SAM_VTRACE2_INTRIN,           TR_FMT2,
      "INTR IN: SETUP: pipe%u len=%u\n"),
  TRENTRY(SAM_VTRACE2_INTROUT,          TR_FMT2,
      "INTR OUT: SETUP: pipe%u len=%u\n"),
  TRENTRY(SAM_VTRACE2_ISOCIN,           TR_FMT2,
      "ISOC IN: SETUP: pipe%u len=%u\n"),
  TRENTRY(SAM_VTRACE2_ISOCOUT,          TR_FMT2,
      "ISOC OUT: SETUP: pipe%u len=%u\n"),
  TRENTRY(SAM_VTRACE2_RECVSTATUS,       TR_FMT2,
      "CTRL OUT: Receive status: pipe%u len=%u\n"),
  TRENTRY(SAM_VTRACE2_SENDSTATUS,       TR_FMT2,
      "CTRL IN: Send status: pipe%u len=%u\n"),
  TRENTRY(SAM_VTRACE2_STARTTRANSFER1,   TR_FMT2,
      "OUT: Send start: pipe%u len=%u\n"),
  TRENTRY(SAM_VTRACE2_STARTTRANSFER2,   TR_FMT2,
      "IN: Receive start: pipe%u len=%u\n"),
  TRENTRY(SAM_VTRACE2_TRANSFER,         TR_FMT2,
      "Transfer start: EP%u len=%u\n"),
  TRENTRY(SAM_VTRACE2_PIPECONF_CTRL_IN, TR_FMT2,
      "Pipe%d configured (EP%d,IN ,CTRL)\n"),
  TRENTRY(SAM_VTRACE2_PIPECONF_CTRL_OUT, TR_FMT2,
      "Pipe%d configured (EP%d,OUT,CTRL)\n"),
  TRENTRY(SAM_VTRACE2_PIPECONF_INTR_IN, TR_FMT2,
      "Pipe%d configured (EP%d,IN ,INTR)\n"),
  TRENTRY(SAM_VTRACE2_PIPECONF_INTR_OUT, TR_FMT2,
      "Pipe%d configured (EP%d,OUT,INTR)\n"),
  TRENTRY(SAM_VTRACE2_PIPECONF_BULK_IN, TR_FMT2,
      "Pipe%d configured (EP%d,IN ,BULK)\n"),
  TRENTRY(SAM_VTRACE2_PIPECONF_BULK_OUT, TR_FMT2,
      "Pipe%d configured (EP%d,OUT,BULK)\n"),
  TRENTRY(SAM_VTRACE2_PIPECONF_ISOC_IN, TR_FMT2,
      "Pipe%d configured (EP%d,IN ,ISOC)\n"),
  TRENTRY(SAM_VTRACE2_PIPECONF_ISOC_OUT, TR_FMT2,
      "Pipe%d configured (EP%d,OUT,ISOC)\n"),

#ifdef CONFIG_USBHOST_ASYNCH
  TRENTRY(SAM_VTRACE2_XFRCOMPLETE,      TR_FMT2,
    "ASYNCH: Transfer complete: EP%u len=%u\n"),
#endif
#endif

  TRENTRY(TRACE2_BADTDSTATUS,         TR_FMT2,
    "ERROR: RHport%d Bad TD completion status: %d\n"),
  TRENTRY(TRACE2_WHDTDSTATUS,         TR_FMT2,
    "ERROR: WHD Bad TD completion status: %d xfrtype: %d\n"),
  TRENTRY(TRACE2_EP0ENQUEUE_FAILED,   TR_FMT2,
    "ERROR: RHport%d Failed to enqueue EP0: %d\n"),
  TRENTRY(TRACE2_EDENQUEUE_FAILED,    TR_FMT2,
    "ERROR: Failed to queue ED for transfer type %d: %d\n"),
  TRENTRY(TRACE2_CLASSENUM_FAILED,    TR_FMT2,
    "Hub port %d usbhost_enumerate() failed: %d\n"),

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  TRENTRY(VTRACE2_EP0CONFIG,          TR_FMT2,
    "EP0 configure speed=%d funcaddr=%d\n"),
  TRENTRY(VTRACE2_INTERVAL,           TR_FMT2,
    "interval: %d->%d\n"),
  TRENTRY(VTRACE2_MININTERVAL,        TR_FMT2,
    "MIN interval: %d offset: %d\n"),
  TRENTRY(VTRACE2_RHPORTST,           TR_FMT2,
    "RHPORTST%d: %04x\n"),
  TRENTRY(VTRACE2_CONNECTED,          TR_FMT2,
    "RHPort%d connected, rhswait: %d\n"),
  TRENTRY(VTRACE2_DISCONNECTED,       TR_FMT2,
    "RHPort%d disconnected, rhswait: %d\n"),
  TRENTRY(VTRACE2_WAKEUP,             TR_FMT2,
    "RHPort%d connected: %d\n"),
  TRENTRY(VTRACE2_EP0CTRLED,          TR_FMT2,
    "RHPort%d EP0 CTRL: %04x\n"),
  TRENTRY(VTRACE2_EPALLOC,            TR_FMT2,
    "EP%d CTRL: %04x\n"),
  TRENTRY(VTRACE2_CTRLIN,             TR_FMT2,
    "CTRLIN RHPort%d req: %02x\n"),
  TRENTRY(VTRACE2_CTRLOUT,            TR_FMT2,
    "CTRLOUT RHPort%d req: %02x\n"),
  TRENTRY(VTRACE2_TRANSFER,           TR_FMT2,
    "EP%d buflen: %d\n"),
  TRENTRY(VTRACE2_INITCONNECTED,      TR_FMT2,
    "RHPort%d Device connected: %d\n"),
#ifdef CONFIG_USBHOST_HUB
  TRENTRY(VTRACE2_HUBWAKEUP,          TR_FMT2,
    "Hub Port%d connected: %d\n"),
#endif
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
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
 ****************************************************************************/

const char *usbhost_trformat1(uint16_t id)
{
  int ndx = TRACE1_INDEX(id);

  if (ndx < TRACE1_NSTRINGS)
    {
      return g_trace1[ndx].string;
    }

  return NULL;
}

const char *usbhost_trformat2(uint16_t id)
{
  int ndx = TRACE2_INDEX(id);

  if (ndx < TRACE2_NSTRINGS)
    {
      return g_trace2[ndx].string;
    }

  return NULL;
}

#endif /* HAVE_USBHOST_TRACE */
