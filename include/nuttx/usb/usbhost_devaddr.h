/*******************************************************************************
 * include/nuttx/usb/usbhost_devaddr.h
 * Manage USB device addresses
 *
 *   Copyright (C) 2013, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * NOTE:  This interface was inspired by the Linux gadget interface by
 * David Brownell. That work was very helpful in determining a usable
 * partitioning of functionality between standard class drivers and various
 * implementations of USB controller drivers.  This work, however, does
 * not derive directly from that work and is licensed differently.
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
 *******************************************************************************/

#ifndef _INCLUDE_NUTTX_USB_USBHOST_DEVADDR_H
#define _INCLUDE_NUTTX_USB_USBHOST_DEVADDR_H

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <semaphore.h>

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************/
/* Configuration ***************************************************************/

#define USBHOST_DEVADDR_HASHSIZE 8
#define USBHOST_DEVADDR_HASHMASK (USBHOST_DEVADDR_HASHSIZE-1)

/*******************************************************************************
 * Public Types
 *******************************************************************************/

struct usbhost_devaddr_s
{
  uint8_t   next;           /* Next device address */
  sem_t     exclsem;        /* Enforces mutually exclusive access */
  uint32_t  alloctab[4];    /* Bit allocation table */
};

/*******************************************************************************
 * Public Data
 *******************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

struct usbhost_hubport_s;     /* Forward reference */
struct usbhost_roothubport_s; /* Forward reference */

/*******************************************************************************
 * Name: usbhost_devaddr_initialize
 *
 * Description:
 *   Initialize the caller provided struct usbhost_devaddr_s instance in
 *   preparation for the management of device addresses on behalf of an root
 *   hub port.
 *
 * Input Parameters:
 *   rhport - A reference to a roothubport structure.
 *
 * Returned Value:
 *   None
 *
 *******************************************************************************/

void usbhost_devaddr_initialize(FAR struct usbhost_roothubport_s *rhport);

/*******************************************************************************
 * Name: usbhost_devaddr_create
 *
 * Description:
 *   Create a new unique device address for this hub port.
 *
 * Input Parameters:
 *   hport - A reference to a hub port structure to which a device has been
 *     newly connected and so is in need of a function address.
 *
 * Returned Value:
 *   On success, a new device function address in the the range 0x01 to 0x7f
 *   is returned.  On failure, a negated errno value is returned.
 *
 *******************************************************************************/

int usbhost_devaddr_create(FAR struct usbhost_hubport_s *hport);

/*******************************************************************************
 * Name: usbhost_devaddr_destroy
 *
 * Description:
 *   Release a device address previously assigned by usbhost_devaddr_create().
 *
 * Input Parameters:
 *   hport - A reference to a hub port structure from which a device has been
 *     disconnected and so no longer needs the function address.
 *   devaddr - The address to be released.
 *
 * Returned Value:
 *   None
 *
 *******************************************************************************/

void usbhost_devaddr_destroy(FAR struct usbhost_hubport_s *hport,
                             uint8_t devaddr);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* _INCLUDE_NUTTX_USB_USBHOST_DEVADDR_H */
