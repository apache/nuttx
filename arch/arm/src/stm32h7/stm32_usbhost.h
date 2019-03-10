/************************************************************************************
 * arch/arm/src/stm32h7/stm32_usbhost.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_USBHOST_H
#define __ARCH_ARM_SRC_STM32H7_STM32_USBHOST_H

/* STM32 USB OTG Host Driver Support
 *
 * Pre-requisites
 *
 *  CONFIG_USBHOST        - Enable general USB host support
 *  CONFIG_STM32H7_OTGFS  - Enable the STM32 USB OTG FS block
 *     or
 *  CONFIG_STM32H7_OTGHS  - Enable the STM32 USB OTG HS block
 *  CONFIG_STM32H7_SYSCFG - Needed
 *
 * Options:
 *
 *  CONFIG_STM32H7_OTG_RXFIFO_SIZE - Size of the RX FIFO in 32-bit words.
 *    Default 128 (512 bytes)
 *  CONFIG_STM32H7_OTG_NPTXFIFO_SIZE - Size of the non-periodic Tx FIFO
 *    in 32-bit words.  Default 96 (384 bytes)
 *  CONFIG_STM32H7_OTG_PTXFIFO_SIZE - Size of the periodic Tx FIFO in 32-bit
 *    words.  Default 96 (384 bytes)
 *  CONFIG_STM32H7_OTG_SOFINTR - Enable SOF interrupts.  Why would you ever
 *    want to do that?
 *
 *  CONFIG_STM32H7_USBHOST_REGDEBUG - Enable very low-level register access
 *    debug.  Depends on CONFIG_DEBUG_FEATURES.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#if (defined(CONFIG_STM32H7_OTGFS) || defined(CONFIG_STM32H7_OTGHS)) && \
    defined(CONFIG_USBHOST)


#ifdef HAVE_USBHOST_TRACE
enum usbhost_trace1codes_e
{
  __TRACE1_BASEVALUE = 0,           /* This will force the first value to be 1 */

  OTG_TRACE1_DEVDISCONN,           /* OTG ERROR: Host Port Device disconnected */
  OTG_TRACE1_IRQATTACH,            /* OTG ERROR: Failed to attach IRQ */
  OTG_TRACE1_TRNSFRFAILED,         /* OTG ERROR: Host Port Transfer Failed */
  OTG_TRACE1_SENDSETUP,            /* OTG ERROR: sendsetup() failed with: */
  OTG_TRACE1_SENDDATA,             /* OTG ERROR: senddata() failed with: */
  OTG_TRACE1_RECVDATA,             /* OTG ERROR: recvdata() failed with: */

#ifdef HAVE_USBHOST_TRACE_VERBOSE

  OTG_VTRACE1_CONNECTED,           /* OTG Host Port connected */
  OTG_VTRACE1_DISCONNECTED,        /* OTG Host Port disconnected */
  OTG_VTRACE1_GINT,                /* OTG Handling Interrupt. Entry Point */
  OTG_VTRACE1_GINT_SOF,            /* OTG Handle the start of frame interrupt */
  OTG_VTRACE1_GINT_RXFLVL,         /* OTG Handle the RxFIFO non-empty interrupt */
  OTG_VTRACE1_GINT_NPTXFE,         /* OTG Handle the non-periodic TxFIFO empty interrupt */
  OTG_VTRACE1_GINT_PTXFE,          /* OTG Handle the periodic TxFIFO empty interrupt */
  OTG_VTRACE1_GINT_HC,             /* OTG Handle the host channels interrupt */
  OTG_VTRACE1_GINT_HPRT,           /* OTG Handle the host port interrupt */
  OTG_VTRACE1_GINT_HPRT_POCCHNG,   /* OTG  HPRT: Port Over-Current Change*/
  OTG_VTRACE1_GINT_HPRT_PCDET,     /* OTG  HPRT: Port Connect Detect */
  OTG_VTRACE1_GINT_HPRT_PENCHNG,   /* OTG  HPRT: Port Enable Changed */
  OTG_VTRACE1_GINT_HPRT_LSDEV,     /* OTG  HPRT: Low Speed Device Connected */
  OTG_VTRACE1_GINT_HPRT_FSDEV,     /* OTG  HPRT: Full Speed Device Connected */
  OTG_VTRACE1_GINT_HPRT_LSFSSW,    /* OTG  HPRT: Host Switch: LS -> FS */
  OTG_VTRACE1_GINT_HPRT_FSLSSW,    /* OTG  HPRT: Host Switch: FS -> LS */
  OTG_VTRACE1_GINT_DISC,           /* OTG Handle the disconnect detected interrupt */
  OTG_VTRACE1_GINT_IPXFR,          /* OTG Handle the incomplete periodic transfer */

#endif

  __TRACE1_NSTRINGS,                 /* Separates the format 1 from the format 2 strings */

  OTG_TRACE2_CLIP,                 /* OTG CLIP: chidx:  buflen: */

#ifdef HAVE_USBHOST_TRACE_VERBOSE

  OTG_VTRACE2_CHANWAKEUP_IN,       /* OTG IN Channel wake up with result */
  OTG_VTRACE2_CHANWAKEUP_OUT,      /* OTG OUT Channel wake up with result */
  OTG_VTRACE2_CTRLIN,              /* OTG CTRLIN */
  OTG_VTRACE2_CTRLOUT,             /* OTG CTRLOUT */
  OTG_VTRACE2_INTRIN,              /* OTG INTRIN */
  OTG_VTRACE2_INTROUT,             /* OTG INTROUT */
  OTG_VTRACE2_BULKIN,              /* OTG BULKIN */
  OTG_VTRACE2_BULKOUT,             /* OTG BULKOUT */
  OTG_VTRACE2_ISOCIN,              /* OTG ISOCIN */
  OTG_VTRACE2_ISOCOUT,             /* OTG ISOCOUT */
  OTG_VTRACE2_STARTTRANSFER,       /* OTG EP buflen */
  OTG_VTRACE2_CHANCONF_CTRL_IN,
  OTG_VTRACE2_CHANCONF_CTRL_OUT,
  OTG_VTRACE2_CHANCONF_INTR_IN,
  OTG_VTRACE2_CHANCONF_INTR_OUT,
  OTG_VTRACE2_CHANCONF_BULK_IN,
  OTG_VTRACE2_CHANCONF_BULK_OUT,
  OTG_VTRACE2_CHANCONF_ISOC_IN,
  OTG_VTRACE2_CHANCONF_ISOC_OUT,
  OTG_VTRACE2_CHANHALT,            /* Channel halted. chidx: , reason:  */

#endif

  __TRACE2_NSTRINGS                  /* Total number of enumeration values */
};

#  define TRACE1_FIRST     ((int)__TRACE1_BASEVALUE + 1)
#  define TRACE1_INDEX(id) ((int)(id) - TRACE1_FIRST)
#  define TRACE1_NSTRINGS  TRACE1_INDEX(__TRACE1_NSTRINGS)

#  define TRACE2_FIRST     ((int)__TRACE1_NSTRINGS + 1)
#  define TRACE2_INDEX(id) ((int)(id) - TRACE2_FIRST)
#  define TRACE2_NSTRINGS  TRACE2_INDEX(__TRACE2_NSTRINGS)

#endif

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

/***********************************************************************************
 * Name: stm32_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be provided be
 *   each platform that implements the STM32 OTG FS host interface
 *
 *   "On-chip 5 V VBUS generation is not supported. For this reason, a charge pump
 *    or, if 5 V are available on the application board, a basic power switch, must
 *    be added externally to drive the 5 V VBUS line. The external charge pump can
 *    be driven by any GPIO output. When the application decides to power on VBUS
 *    using the chosen GPIO, it must also set the port power bit in the host port
 *    control and status register (PPWR bit in OTG_FS_HPRT).
 *
 *   "The application uses this field to control power to this port, and the core
 *    clears this bit on an overcurrent condition."
 *
 * Input Parameters:
 *   iface - For future growth to handle multiple USB host interface.  Should be zero.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ***********************************************************************************/

void stm32_usbhost_vbusdrive(int iface, bool enable);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* (CONFIG_STM32H7_OTGFS || CONFIG_STM32H7_OTGHS) && CONFIG_USBHOST */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_USBHOST_H */
