/************************************************************************************
 * arch/arm/src/efm32/efm32_usb.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_OTGFS_H
#define __ARCH_ARM_SRC_EFM32_EFM32_OTGFS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/efm32_usb.h"

#if defined(CONFIG_EFM32_OTGFS)

/************************************************************************************
 * Public Types
 ************************************************************************************/

#if defined(HAVE_USBHOST_TRACE) && defined(CONFIG_EFM32_OTGFS)
enum usbhost_trace1codes_e
{
  __TRACE1_BASEVALUE = 0,              /* This will force the first value to be 1 */

  USBHOST_TRACE1_DEVDISCONN,           /* OTGFS ERROR: Host Port Device disconnected */
  USBHOST_TRACE1_IRQATTACH,            /* OTGFS ERROR: Failed to attach IRQ */
  USBHOST_TRACE1_TRNSFRFAILED,         /* OTGFS ERROR: Host Port Transfer Failed */
  USBHOST_TRACE1_SENDSETUP,            /* OTGFS ERROR: sendsetup() failed with: */
  USBHOST_TRACE1_SENDDATA,             /* OTGFS ERROR: senddata() failed with: */
  USBHOST_TRACE1_RECVDATA,             /* OTGFS ERROR: recvdata() failed with: */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  USBHOST_VTRACE1_CONNECTED,           /* OTGFS Host Port connected */
  USBHOST_VTRACE1_DISCONNECTED,        /* OTGFS Host Port disconnected */
  USBHOST_VTRACE1_GINT,                /* OTGFS Handling Interrupt. Entry Point */
  USBHOST_VTRACE1_GINT_SOF,            /* OTGFS Handle the start of frame interrupt */
  USBHOST_VTRACE1_GINT_RXFLVL,         /* OTGFS Handle the RxFIFO non-empty interrupt */
  USBHOST_VTRACE1_GINT_NPTXFE,         /* OTGFS Handle the non-periodic TxFIFO empty interrupt */
  USBHOST_VTRACE1_GINT_PTXFE,          /* OTGFS Handle the periodic TxFIFO empty interrupt */
  USBHOST_VTRACE1_GINT_HC,             /* OTGFS Handle the host channels interrupt */
  USBHOST_VTRACE1_GINT_HPRT,           /* OTGFS Handle the host port interrupt */
  USBHOST_VTRACE1_GINT_HPRT_POCCHNG,   /* OTGFS  HPRT: Port Over-Current Change*/
  USBHOST_VTRACE1_GINT_HPRT_PCDET,     /* OTGFS  HPRT: Port Connect Detect */
  USBHOST_VTRACE1_GINT_HPRT_PENCHNG,   /* OTGFS  HPRT: Port Enable Changed */
  USBHOST_VTRACE1_GINT_HPRT_LSDEV,     /* OTGFS  HPRT: Low Speed Device Connected */
  USBHOST_VTRACE1_GINT_HPRT_FSDEV,     /* OTGFS  HPRT: Full Speed Device Connected */
  USBHOST_VTRACE1_GINT_HPRT_LSFSSW,    /* OTGFS  HPRT: Host Switch: LS -> FS */
  USBHOST_VTRACE1_GINT_HPRT_FSLSSW,    /* OTGFS  HPRT: Host Switch: FS -> LS */
  USBHOST_VTRACE1_GINT_DISC,           /* OTGFS Handle the disconnect detected interrupt */
  USBHOST_VTRACE1_GINT_IPXFR,          /* OTGFS Handle the incomplete periodic transfer */
#endif

  __TRACE1_NSTRINGS,                   /* Separates the format 1 from the format 2 strings */

  USBHOST_TRACE2_CLIP,                 /* OTGFS CLIP: chidx:  buflen: */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  USBHOST_VTRACE2_CHANWAKEUP_IN,       /* OTGFS IN Channel wake up with result */
  USBHOST_VTRACE2_CHANWAKEUP_OUT,      /* OTGFS OUT Channel wake up with result */
  USBHOST_VTRACE2_CTRLIN,              /* OTGFS CTRLIN */
  USBHOST_VTRACE2_CTRLOUT,             /* OTGFS CTRLOUT */
  USBHOST_VTRACE2_INTRIN,              /* OTGFS INTRIN */
  USBHOST_VTRACE2_INTROUT,             /* OTGFS INTROUT */
  USBHOST_VTRACE2_BULKIN,              /* OTGFS BULKIN */
  USBHOST_VTRACE2_BULKOUT,             /* OTGFS BULKOUT */
  USBHOST_VTRACE2_ISOCIN,              /* OTGFS ISOCIN */
  USBHOST_VTRACE2_ISOCOUT,             /* OTGFS ISOCOUT */
  USBHOST_VTRACE2_STARTTRANSFER,       /* OTGFS EP buflen */
  USBHOST_VTRACE2_CHANCONF_CTRL_IN,
  USBHOST_VTRACE2_CHANCONF_CTRL_OUT,
  USBHOST_VTRACE2_CHANCONF_INTR_IN,
  USBHOST_VTRACE2_CHANCONF_INTR_OUT,
  USBHOST_VTRACE2_CHANCONF_BULK_IN,
  USBHOST_VTRACE2_CHANCONF_BULK_OUT,
  USBHOST_VTRACE2_CHANCONF_ISOC_IN,
  USBHOST_VTRACE2_CHANCONF_ISOC_OUT,
  USBHOST_VTRACE2_CHANHALT,            /* Channel halted. chidx: , reason:  */
#endif

  __TRACE2_NSTRINGS                    /* Total number of enumeration values */
};

#  define TRACE1_FIRST     ((int)__TRACE1_BASEVALUE + 1)
#  define TRACE1_INDEX(id) ((int)(id) - TRACE1_FIRST)
#  define TRACE1_NSTRINGS  TRACE1_INDEX(__TRACE1_NSTRINGS)

#  define TRACE2_FIRST     ((int)__TRACE1_NSTRINGS + 1)
#  define TRACE2_INDEX(id) ((int)(id) - TRACE2_FIRST)
#  define TRACE2_NSTRINGS  TRACE2_INDEX(__TRACE2_NSTRINGS)

#endif /* HAVE_USBHOST_TRACE && CONFIG_EFM32_OTGFS */

/************************************************************************************
 * Public Functions
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

/****************************************************************************
 * Name: efm32_usbhost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
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
 ****************************************************************************/

#ifdef CONFIG_USBHOST
struct usbhost_connection_s;
FAR struct usbhost_connection_s *efm32_usbhost_initialize(int controller);
#endif

/***********************************************************************************
 * Name: efm32_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be provided be
 *   each platform that implements the EFM32 OTG FS host interface
 *
 * Input Parameters:
 *   iface - For future growth to handle multiple USB host interface.  Should be zero.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ***********************************************************************************/

#ifdef CONFIG_USBHOST
void efm32_usbhost_vbusdrive(int iface, bool enable);
#endif

/************************************************************************************
 * Name:  efm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the efm32_usbsuspend logic if the OTG FS device driver
 *   is used.  This function is called whenever the USB enters or leaves suspend
 *   mode. This is an opportunity for the board logic to shutdown clocks, power,
 *   etc. while the USB is suspended.
 *
 ************************************************************************************/

#ifdef CONFIG_USBDEV
struct usbdev_s;
void efm32_usbsuspend(FAR struct usbdev_s *dev, bool resume);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_EFM32_OTGFS */
#endif /* __ARCH_ARM_SRC_EFM32_EFM32_OTGFS_H */
