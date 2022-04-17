/****************************************************************************
 * arch/arm/src/samd5e5/sam_usbhost.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_SAM_USBHOST_H
#define __ARCH_ARM_SRC_SAMD5E5_SAM_USBHOST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usbhost_trace.h>

#ifdef CONFIG_USBHOST

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is the interface argument for call outs to board-specific
 * functions which need to know which USB host interface is being used.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef HAVE_USBHOST_TRACE
enum usbhost_trace1codes_e
{
  __TRACE1_BASEVALUE = 0,           /* This will force the first value to be 1 */

  SAM_TRACE1_ALLOC_FAIL,
#ifdef CONFIG_USBHOST_ASYNCH
  SAM_TRACE1_ASYNCHSETUP_FAIL1,
  SAM_TRACE1_ASYNCHSETUP_FAIL2,
#endif
  SAM_TRACE1_BAD_JKSTATE,
  SAM_TRACE1_BADREVISION,
  SAM_TRACE1_PIPEALLOC_FAIL,
  SAM_TRACE1_PIPEWAIT_FAIL,
  SAM_TRACE1_DEVDISCONN1,
  SAM_TRACE1_DEVDISCONN2,
  SAM_TRACE1_DEVDISCONN3,
  SAM_TRACE1_DEVDISCONN4,
  SAM_TRACE1_DEVDISCONN5,
  SAM_TRACE1_DEVDISCONN6,
  SAM_TRACE1_DEVDISCONN7,
  SAM_TRACE1_DEVDISCONN8,
  SAM_TRACE1_ENUMERATE_FAIL,
  SAM_TRACE1_INSETUP_FAIL1,
#ifdef CONFIG_USBHOST_ASYNCH
  SAM_TRACE1_INSETUP_FAIL2,
  SAM_TRACE1_INSETUP_FAIL3,
#endif
  SAM_TRACE1_IRQATTACH_FAIL,
  SAM_TRACE1_OUTSETUP_FAIL1,
#ifdef CONFIG_USBHOST_ASYNCH
  SAM_TRACE1_OUTSETUP_FAIL2,
  SAM_TRACE1_OUTSETUP_FAIL3,
#endif
  SAM_TRACE1_RECVDATA_FAIL,
  SAM_TRACE1_RECVSTATUS_FAIL,
  SAM_TRACE1_SENDDATA_FAIL,
  SAM_TRACE1_SENDSETUP_FAIL1,
  SAM_TRACE1_SENDSETUP_FAIL2,
  SAM_TRACE1_SENDSTATUS_FAIL,
  SAM_TRACE1_TRANSFER_FAILED1,
  SAM_TRACE1_TRANSFER_FAILED2,
  SAM_TRACE1_TRANSFER_FAILED3,

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  SAM_VTRACE1_CANCEL,
  SAM_VTRACE1_CONNECTED1,
  SAM_VTRACE1_CONNECTED2,
  SAM_VTRACE1_CONNECTED3,
  SAM_VTRACE1_DISCONNECTED1,
  SAM_VTRACE1_DISCONNECTED2,
  SAM_VTRACE1_ENUMERATE,
#ifdef CONFIG_USBHOST_HUB
  SAM_VTRACE1_HUB_CONNECTED,
#endif
  SAM_VTRACE1_INITIALIZED,
#ifdef CONFIG_USBHOST_ASYNCH
  SAM_VTRACE1_TRANSFER_COMPLETE,
#endif
#endif

  TRACE1_DEVDISCONN,                /* ERROR: RHport Device disconnected */
  TRACE1_INTRUNRECOVERABLE,         /* ERROR: Unrecoverable error */
  TRACE1_INTRUNHANDLED,             /* ERROR: Unhandled interrupts */
  TRACE1_EPLISTALLOC_FAILED,        /* ERROR: Failed to allocate EP list */
  TRACE1_EDALLOC_FAILED,            /* ERROR: Failed to allocate ED */
  TRACE1_TDALLOC_FAILED,            /* ERROR: Failed to allocate TD */
  TRACE1_IRQATTACH,                 /* ERROR: Failed to attach IRQ */
#ifdef CONFIG_USBHOST_ASYNCH
  TRACE1_BADTDSTATUS,               /* ERROR: Bad asynch TD completion status */
#endif

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  VTRACE1_PHYSED,                   /* physed */
  VTRACE1_VIRTED,                   /* ed */
  VTRACE1_CSC,                      /* Connect Status Change */
  VTRACE1_DRWE,                     /* DRWE: Remote wake-up */
  VTRACE1_ALREADYCONN,              /* Already connected */
  VTRACE1_SPEED,                    /* Low speed */
  VTRACE1_ALREADYDISCONN,           /* Already disconnected */
  VTRACE1_RHSC,                     /* Root Hub Status Change */
  VTRACE1_WDHINTR,                  /* Writeback Done Head interrupt */
  VTRACE1_CLASSENUM,                /* Enumerate the device */
  VTRACE1_ENUMDISCONN,              /* RHport Not connected */
  VTRACE1_INITIALIZING,             /* Initializing Stack */
  VTRACE1_INITIALIZED,              /* Initialized */
  VTRACE1_INTRPENDING,              /* Interrupts pending */
#endif

  __TRACE1_NSTRINGS,                /* Separates the format 1 from the format 2 strings */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
#ifdef CONFIG_USBHOST_ASYNCH
  SAM_VTRACE2_ASYNCH,
#endif
  SAM_VTRACE2_BULKIN,
  SAM_VTRACE2_BULKOUT,
  SAM_VTRACE2_PIPEWAKEUP_IN,
  SAM_VTRACE2_PIPEWAKEUP_OUT,
  SAM_VTRACE2_CTRLIN,
  SAM_VTRACE2_CTRLOUT,
#ifdef CONFIG_USBHOST_HUB
  SAM_VTRACE2_HUB_CONNECTED,
#endif
  SAM_VTRACE2_INTRIN,
  SAM_VTRACE2_INTROUT,
  SAM_VTRACE2_ISOCIN,
  SAM_VTRACE2_ISOCOUT,
  SAM_VTRACE2_RECVSTATUS,
  SAM_VTRACE2_SENDSTATUS,
  SAM_VTRACE2_STARTTRANSFER1,
  SAM_VTRACE2_STARTTRANSFER2,
  SAM_VTRACE2_TRANSFER,
  SAM_VTRACE2_PIPECONF_CTRL_IN,
  SAM_VTRACE2_PIPECONF_CTRL_OUT,
  SAM_VTRACE2_PIPECONF_INTR_IN,
  SAM_VTRACE2_PIPECONF_INTR_OUT,
  SAM_VTRACE2_PIPECONF_BULK_IN,
  SAM_VTRACE2_PIPECONF_BULK_OUT,
  SAM_VTRACE2_PIPECONF_ISOC_IN,
  SAM_VTRACE2_PIPECONF_ISOC_OUT,
#ifdef CONFIG_USBHOST_ASYNCH
  SAM_VTRACE2_XFRCOMPLETE,
#endif
#endif

  TRACE2_BADTDSTATUS,               /* ERROR: RHport Bad TD completion status */
  TRACE2_WHDTDSTATUS,               /* ERROR: WDH Bad TD completion status */
  TRACE2_EP0ENQUEUE_FAILED,         /* ERROR: RHport Failed to enqueue EP0 */
  TRACE2_EDENQUEUE_FAILED,          /* ERROR: Failed to queue ED for transfer type */
  TRACE2_CLASSENUM_FAILED,          /* usbhost_enumerate() failed */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  VTRACE2_EP0CONFIG,                /* EP0 configuration */
  VTRACE2_INTERVAL,                 /* interval */
  VTRACE2_MININTERVAL,              /* MIN interval/offset */
  VTRACE2_RHPORTST,                 /* RHPORTST */
  VTRACE2_CONNECTED,                /* RHPort connected */
  VTRACE2_DISCONNECTED,             /* RHPort disconnected */
  VTRACE2_WAKEUP,                   /* RHPort connected wakeup */
  VTRACE2_EP0CTRLED,                /* RHPort EP0 CTRL */
  VTRACE2_EPALLOC,                  /* EP CTRL */
  VTRACE2_CTRLIN,                   /* CTRLIN */
  VTRACE2_CTRLOUT,                  /* CTRLOUT */
  VTRACE2_TRANSFER,                 /* EP buflen */
  VTRACE2_INITCONNECTED,            /* RHPort Device connected */
#ifdef CONFIG_USBHOST_HUB
  VTRACE2_HUBWAKEUP,                /* Hub Port connected wakeup */
#endif
#endif

  __TRACE2_NSTRINGS                 /* Total number of enumeration values */
};

#  define TRACE1_FIRST     ((int)__TRACE1_BASEVALUE + 1)
#  define TRACE1_INDEX(id) ((int)(id) - TRACE1_FIRST)
#  define TRACE1_NSTRINGS  TRACE1_INDEX(__TRACE1_NSTRINGS)

#  define TRACE2_FIRST     ((int)__TRACE1_NSTRINGS + 1)
#  define TRACE2_INDEX(id) ((int)(id) - TRACE2_FIRST)
#  define TRACE2_NSTRINGS  TRACE2_INDEX(__TRACE2_NSTRINGS)

#endif /* HAVE_USBHOST_TRACE */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int samd_usbhost_initialize(void);

/****************************************************************************
 * Name: sam_usbhost_initialize
 *
 * Description:
 *   Initialize USB host controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *   this identifies which controller is being initialized.  Normally, this
 *   is just zero.
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 ****************************************************************************/

struct usbhost_connection_s;
struct usbhost_connection_s *sam_usbhost_initialize(int controller);

/****************************************************************************
 * Name: sam_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.
 *   This function must be provided by each platform that
 *   implements the OHCI or EHCI host interface
 *
 * Input Parameters:
 *   rhport - Selects root hub port to be powered host interface.
 *            See SAM_RHPORT_* definitions above.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_usbhost_vbusdrive(int rhport, bool enable);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_USBHOST */
#endif /* __ARCH_ARM_SRC_SAMD5E5_SAM_USBHOST_H */
