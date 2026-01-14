/****************************************************************************
 * drivers/usbhost/usbhost_xhci_trace.c
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
#include <stddef.h>

#include "usbhost_xhci_trace.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRENTRY(id,string) {string}

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct xhci_usbhost_trace_s
{
  const char *string;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct xhci_usbhost_trace_s g_trace1[TRACE1_NSTRINGS] =
{
  TRENTRY(XHCI_TRACE1_RESET_FAILED,
          "XHCI ERROR: xhci_reset failed %d\n"),
  TRENTRY(XHCI_TRACE1_START_FAILED,
          "XHCI ERROR: XHCI Failed to start\n"),
  TRENTRY(XHCI_TRACE1_SLOTEN_FAILED,
          "XHCI ERROR: XHCI Slot Enable failed %d\n"),
  TRENTRY(XHCI_TRACE1_TRANSFER_FAILED,
          "XHCI ERROR: XHCI transfer failed %d\n"),
  TRENTRY(XHCI_TRACE1_BADXFRTYPE,
          "XHCI ERROR: XHCI bad transfer type %d\n"),

  /* Verbose */

  TRENTRY(XHCI_VTRACE1_INITIALIZING,
          "XHCI Initializing XHCI Stack\n"),
  TRENTRY(XHCI_VTRACE1_INIITIALIZED,
          "XHCI USB XHCI Initialized\n"),
  TRENTRY(XHCI_VTRACE1_PORTSC_CSC,
          "XHCI Connect Status Change: %06x\n"),
  TRENTRY(XHCI_VTRACE1_PORTSC_CONNALREADY,
          "XHCI Already connected: %06x\n"),
  TRENTRY(XHCI_VTRACE1_PORTSC_DISCALREADY,
          "XHCI Already disconnected: %06x\n"),
};

static const struct xhci_usbhost_trace_s g_trace2[TRACE2_NSTRINGS] =
{
  TRENTRY(XHCI_TRACE2_EPSTALLED,
          "XHCI EP%d Stalled: TOKEN=%04x\n"),

  /* Verbose */

  TRENTRY(XHCI_VTRACE2_PORTSC,
          "XHCI PORTSC%d: %08x\n"),
  TRENTRY(XHCI_VTRACE2_PORTSC_CONNECTED,
          "XHCI RHPort%d connected, pscwait: %d\n"),
  TRENTRY(XHCI_VTRACE2_PORTSC_DISCONND,
          "XHCI RHport%d disconnected, pscwait: %d\n"),
  TRENTRY(XHCI_VTRACE2_MONWAKEUP,
          "XHCI Hub port%d connected: %d\n"),
  TRENTRY(XHCI_VTRACE2_CTRLINOUT,
          "HXCI CTRLIN/OUT: RHPort%d req: %02x\n"),
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
