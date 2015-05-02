/************************************************************************************
 * arch/arm/src/sama5/sam_usbhost.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_USBHOST_H
#define __ARCH_ARM_SRC_SAMA5_SAM_USBHOST_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usbhost_trace.h>

#ifdef CONFIG_USBHOST

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* This is the interface argument for call outs to board-specific functions which
 * need to know which USB host interface is being used.
 */

#define SAM_EHCI_IFACE 0
#define SAM_OHCI_IFACE 1

/* This is the interface argument for call outs to board-specific functions which
 * need to know which root hub port is being used.
 */

#define SAM_RHPORT1    0
#define SAM_RHPORT2    1
#define SAM_RHPORT3    2

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifdef HAVE_USBHOST_TRACE
enum usbhost_trace1codes_e
{
  __TRACE1_BASEVALUE = 0,           /* This will force the first value to be 1 */

#ifdef CONFIG_SAMA5_OHCI
  OHCI_TRACE1_DEVDISCONN,           /* OHCI ERROR: RHport Device disconnected */
  OHCI_TRACE1_INTRUNRECOVERABLE,    /* OHCI ERROR: Unrecoverable error */
  OHCI_TRACE1_INTRUNHANDLED,        /* OHCI ERROR: Unhandled interrupts */
  OHCI_TRACE1_EPLISTALLOC_FAILED,   /* OHCI ERROR: Failed to allocate EP list */
  OHCI_TRACE1_EDALLOC_FAILED,       /* OHCI ERROR: Failed to allocate ED */
  OHCI_TRACE1_TDALLOC_FAILED,       /* OHCI ERROR: Failed to allocate TD */
  OHCI_TRACE1_IRQATTACH,            /* OHCI ERROR: Failed to attach IRQ */
#ifdef CONFIG_USBHOST_ASYNCH
  OHCI_TRACE1_BADTDSTATUS,          /* OHCI ERROR: Bad asynch TD completion status */
#endif

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  OHCI_VTRACE1_PHYSED,              /* OHCI physed */
  OHCI_VTRACE1_VIRTED,              /* OHCI ed */
  OHCI_VTRACE1_CSC,                 /* OHCI Connect Status Change */
  OHCI_VTRACE1_DRWE,                /* OHCI DRWE: Remote wake-up */
  OHCI_VTRACE1_ALREADYCONN,         /* OHCI Already connected */
  OHCI_VTRACE1_SPEED,               /* OHCI Low speed */
  OHCI_VTRACE1_ALREADYDISCONN,      /* OHCI Already disconnected */
  OHCI_VTRACE1_RHSC,                /* OHCI Root Hub Status Change */
  OHCI_VTRACE1_WDHINTR,             /* OHCI Writeback Done Head interrupt */
  OHCI_VTRACE1_CLASSENUM,           /* OHCI Enumerate the device */
  OHCI_VTRACE1_ENUMDISCONN,         /* OHCI RHport Not connected */
  OHCI_VTRACE1_INITIALIZING,        /* OHCI Initializing Stack */
  OHCI_VTRACE1_INITIALIZED,         /* OHCI Initialized */
  OHCI_VTRACE1_INTRPENDING,         /* OHCI Interrupts pending */
#endif
#endif

#ifdef CONFIG_SAMA5_EHCI
  EHCI_TRACE1_SYSTEMERROR,          /* EHCI ERROR: System error */
  EHCI_TRACE1_QTDFOREACH_FAILED,    /* EHCI ERROR: sam_qtd_foreach failed */
  EHCI_TRACE1_QHALLOC_FAILED,       /* EHCI ERROR: Failed to allocate a QH */
  EHCI_TRACE1_BUFTOOBIG,            /* EHCI ERROR: Buffer too big */
  EHCI_TRACE1_REQQTDALLOC_FAILED,   /* EHCI ERROR: Failed to allocate request qTD */
  EHCI_TRACE1_ADDBPL_FAILED,        /* EHCI ERROR: sam_qtd_addbpl failed */
  EHCI_TRACE1_DATAQTDALLOC_FAILED,  /* EHCI ERROR: Failed to allocate data buffer qTD */
  EHCI_TRACE1_DEVDISCONNECTED,      /* EHCI ERROR: Device disconnected */
  EHCI_TRACE1_QHCREATE_FAILED,      /* EHCI ERROR: sam_qh_create failed */
  EHCI_TRACE1_QTDSETUP_FAILED,      /* EHCI ERROR: sam_qtd_setupphase failed */
  EHCI_TRACE1_QTDDATA_FAILED,       /* EHCI ERROR: sam_qtd_dataphase failed */
  EHCI_TRACE1_QTDSTATUS_FAILED,     /* EHCI ERROR: sam_qtd_statusphase failed */
  EHCI_TRACE1_TRANSFER_FAILED,      /* EHCI ERROR: Transfer failed */
  EHCI_TRACE1_QHFOREACH_FAILED,     /* EHCI ERROR: sam_qh_foreach failed: */
  EHCI_TRACE1_SYSERR_INTR,          /* EHCI: Host System Error Interrup */
  EHCI_TRACE1_USBERR_INTR,          /* EHCI: USB Error Interrupt (USBERRINT) Interrupt */
  EHCI_TRACE1_EPALLOC_FAILED,       /* EHCI ERROR: Failed to allocate EP info structure */
  EHCI_TRACE1_BADXFRTYPE,           /* EHCI ERROR: Support for transfer type not implemented */
  EHCI_TRACE1_HCHALTED_TIMEOUT,     /* EHCI ERROR: Timed out waiting for HCHalted */
  EHCI_TRACE1_QHPOOLALLOC_FAILED,   /* EHCI ERROR: Failed to allocate the QH pool */
  EHCI_TRACE1_QTDPOOLALLOC_FAILED,  /* EHCI ERROR: Failed to allocate the qTD pool */
  EHCI_TRACE1_PERFLALLOC_FAILED,    /* EHCI ERROR: Failed to allocate the periodic frame list */
  EHCI_TRACE1_RESET_FAILED,         /* EHCI ERROR: sam_reset failed */
  EHCI_TRACE1_RUN_FAILED,           /* EHCI ERROR: EHCI Failed to run */
  EHCI_TRACE1_IRQATTACH_FAILED,     /* EHCI ERROR: Failed to attach IRQ */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  EHCI_VTRACE1_PORTSC_CSC,          /* EHCI Connect Status Change */
  EHCI_VTRACE1_PORTSC_CONNALREADY,  /* EHCI Already connected */
  EHCI_VTRACE1_PORTSC_DISCALREADY,  /* EHCI Already disconnected */
  EHCI_VTRACE1_TOPHALF,             /* EHCI Interrupt top half */
  EHCI_VTRACE1_AAINTR,              /* EHCI Async Advance Interrupt */
  EHCI_VTRACE1_USBINTR,             /* EHCI USB Interrupt (USBINT) Interrupt */
  EHCI_VTRACE1_CLASSENUM,           /* EHCI Enumerate the device */
  EHCI_VTRACE1_ENUM_DISCONN,        /* EHCI Enumeration not connected */
  EHCI_VTRACE1_INITIALIZING,        /* EHCI Initializing EHCI Stack */
  EHCI_VTRACE1_HCCPARAMS,           /* EHCI HCCPARAMS */
  EHCI_VTRACE1_INIITIALIZED,        /* EHCI USB EHCI Initialized */
#endif
#endif

  __TRACE1_NSTRINGS,                /* Separates the format 1 from the format 2 strings */

#ifdef CONFIG_SAMA5_OHCI
  OHCI_TRACE2_BADTDSTATUS,          /* OHCI ERROR: RHport Bad TD completion status */
  OHCI_TRACE2_WHDTDSTATUS,          /* OHCI ERROR: WDH Bad TD completion status */
  OHCI_TRACE2_EP0ENQUEUE_FAILED,    /* OHCI ERROR: RHport Failed to enqueue EP0 */
  OHCI_TRACE2_EDENQUEUE_FAILED,     /* OHCI ERROR: Failed to queue ED for transfer type */
  OHCI_TRACE2_CLASSENUM_FAILED,     /* OHCI usbhost_enumerate() failed */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  OHCI_VTRACE2_EP0CONFIG,           /* OHCI EP0 configuration */
  OHCI_VTRACE2_INTERVAL,            /* OHCI interval */
  OHCI_VTRACE2_MININTERVAL,         /* OHCI MIN interval/offset */
  OHCI_VTRACE2_RHPORTST,            /* OHCI RHPORTST */
  OHCI_VTRACE2_CONNECTED,           /* OHCI RHPort connected */
  OHCI_VTRACE2_DISCONNECTED,        /* OHCI RHPort disconnected */
  OHCI_VTRACE2_WAKEUP,              /* OHCI RHPort connected wakeup */
  OHCI_VTRACE2_EP0CTRLED,           /* OHCI RHPort EP0 CTRL */
  OHCI_VTRACE2_EPALLOC,             /* OHCI EP CTRL */
  OHCI_VTRACE2_CTRLIN,              /* OHCI CTRLIN */
  OHCI_VTRACE2_CTRLOUT,             /* OHCI CTRLOUT */
  OHCI_VTRACE2_TRANSFER,            /* OHCI EP buflen */
  OHCI_VTRACE2_INITCONNECTED,       /* OHCI RHPort Device connected */
#ifdef CONFIG_USBHOST_HUB
  OHCI_VTRACE2_HUBWAKEUP,           /* EHCI Hub Port connected wakeup */
#endif
#endif
#endif

#ifdef CONFIG_SAMA5_EHCI
  EHCI_TRACE2_EPSTALLED,            /* EHCI EP Stalled */
  EHCI_TRACE2_EPIOERROR,            /* EHCI ERROR: EP TOKEN */
  EHCI_TRACE2_CLASSENUM_FAILED,     /* EHCI usbhost_enumerate() failed */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  EHCI_VTRACE2_EP0CONFIG,           /* EHCI EP0 configuration */
  EHCI_VTRACE2_ASYNCXFR,            /* EHCI Async transfer */
  EHCI_VTRACE2_INTRXFR,             /* EHCI Interrupt Transfer */
  EHCI_VTRACE2_IOCCHECK,            /* EHCI IOC */
  EHCI_VTRACE2_PORTSC,              /* EHCI PORTSC */
  EHCI_VTRACE2_PORTSC_CONNECTED,    /* EHCI RHPort connected */
  EHCI_VTRACE2_PORTSC_DISCONND,     /* EHCI RHport disconnected */
  EHCI_VTRACE2_MONWAKEUP,           /* EHCI RHPort connected wakeup */
  EHCI_VTRACE2_EPALLOC,             /* EHCI EPALLOC */
  EHCI_VTRACE2_CTRLINOUT,           /* EHCI CTRLIN/OUT */
  EHCI_VTRACE2_HCIVERSION,          /* EHCI HCIVERSION */
  EHCI_VTRACE2_HCSPARAMS,           /* EHCI HCSPARAMS */
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

#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/*******************************************************************************
 * Name: sam_ohci_initialize
 *
 * Description:
 *   Initialize USB OHCI host controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than one USB OHCI interface, then
 *     this identifies which controller is being initializeed.  Normally, this
 *     is just zero.
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
 *******************************************************************************/

#ifdef CONFIG_SAMA5_OHCI
struct usbhost_connection_s;
FAR struct usbhost_connection_s *sam_ohci_initialize(int controller);
#endif

/*******************************************************************************
 * Name: sam_ohci_tophalf
 *
 * Description:
 *   OHCI "Top Half" interrupt handler.  If both EHCI and OHCI are enabled, then
 *   EHCI will manage the common UHPHS interrupt and will forward the interrupt
 *   event to this function.
 *
 *******************************************************************************/

#ifdef CONFIG_SAMA5_OHCI
int sam_ohci_tophalf(int irq, FAR void *context);
#endif

/*******************************************************************************
 * Name: sam_ehci_initialize
 *
 * Description:
 *   Initialize USB EHCI host controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than one EHCI interface, then
 *     this identifies which controller is being initializeed.  Normally, this
 *     is just zero.
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
 *******************************************************************************/

#ifdef CONFIG_SAMA5_EHCI
struct usbhost_connection_s;
FAR struct usbhost_connection_s *sam_ehci_initialize(int controller);
#endif

/***********************************************************************************
 * Name: sam_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be provided by
 *   each platform that implements the OHCI or EHCI host interface
 *
 * Input Parameters:
 *   rhport - Selects root hub port to be powered host interface.  See SAM_RHPORT_*
 *            definitions above.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ***********************************************************************************/

void sam_usbhost_vbusdrive(int rhport, bool enable);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_USBHOST */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_USBHOST_H */
