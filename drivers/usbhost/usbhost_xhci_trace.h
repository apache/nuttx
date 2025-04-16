/****************************************************************************
 * drivers/usbhost/usbhost_xhci_trace.h
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

#ifndef __DRIVERS_USBHOST_USBHOST_XHCI_TRACE_H
#define __DRIVERS_USBHOST_USBHOST_XHCI_TRACE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum usbhost_trace1codes_e
{
  __TRACE1_BASEVALUE = 0,           /* This will force the first value to be 1 */

  XHCI_TRACE1_RESET_FAILED,         /* XHCI ERROR: sam_reset failed */
  XHCI_TRACE1_START_FAILED,         /* XHCI ERROR: XHCI Failed to start */
  XHCI_TRACE1_SLOTEN_FAILED,        /* XHCI ERROR: XHCI Slot Enable failed */
  XHCI_TRACE1_TRANSFER_FAILED,      /* XHCI ERROR: XHCI transfer failed */
  XHCI_TRACE1_BADXFRTYPE,           /* XHCI ERROR: XHCI bad transfer type */

  /* Verbose */

  XHCI_VTRACE1_INITIALIZING,        /* XHCI Initializing XHCI Stack */
  XHCI_VTRACE1_INIITIALIZED,        /* XHCI USB XHCI Initialized */
  XHCI_VTRACE1_PORTSC_CSC,          /* XHCI Connect Status Change */
  XHCI_VTRACE1_PORTSC_CONNALREADY,  /* XHCI Already connected */
  XHCI_VTRACE1_PORTSC_DISCALREADY,  /* XHCI Already disconnected */

  __TRACE1_NSTRINGS,                /* Separates the format 1 from the format 2 strings */

  XHCI_TRACE2_EPSTALLED,            /* XHCI EP Stalled */

  /* Verbose */

  XHCI_VTRACE2_PORTSC,              /* XHCI PORTSC */
  XHCI_VTRACE2_PORTSC_CONNECTED,    /* XHCI RHPort connected */
  XHCI_VTRACE2_PORTSC_DISCONND,     /* XHCI RHport disconnected */
  XHCI_VTRACE2_MONWAKEUP,           /* XHCI RHPort connected wakeup */
  XHCI_VTRACE2_CTRLINOUT,           /* XHCI CTRLIN/OUT */
  __TRACE2_NSTRINGS                 /* Total number of enumeration values */
};

#define TRACE1_FIRST     ((int)__TRACE1_BASEVALUE + 1)
#define TRACE1_INDEX(id) ((int)(id) - TRACE1_FIRST)
#define TRACE1_NSTRINGS  TRACE1_INDEX(__TRACE1_NSTRINGS)

#define TRACE2_FIRST     ((int)__TRACE1_NSTRINGS + 1)
#define TRACE2_INDEX(id) ((int)(id) - TRACE2_FIRST)
#define TRACE2_NSTRINGS  TRACE2_INDEX(__TRACE2_NSTRINGS)

#endif /* #define __DRIVERS_USBHOST_USBHOST_XHCI_TRACE_H */
