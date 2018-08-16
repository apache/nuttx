/*******************************************************************************
 * include/nuttx/usb/cdcecm.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Michael Jung <mijung@gmx.net>
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
 ******************************************************************************/

#ifndef __INCLUDE_NUTTX_USB_CDCECM_H
#define __INCLUDE_NUTTX_USB_CDCECM_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_CDCECM_COMPOSITE
# include <nuttx/usb/composite.h>
#endif

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

#define CDCECM_EP_INTIN_IDX      (0)
#define CDCECM_EP_BULKIN_IDX     (1)
#define CDCECM_EP_BULKOUT_IDX    (2)

/******************************************************************************
 * Public Data
 ******************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/****************************************************************************
 * Name: cdcecm_initialize
 *
 * Description:
 *   Register CDC/ECM USB device interface. Register the corresponding network
 *   driver to NuttX and bring up the network.
 *
 * Input Parameters:
 *   minor - Device minor number.
 *   handle - An optional opaque reference to the CDC/ECM class object that
 *     may subsequently be used with cdcecm_uninitialize().
 *
 * Returned Value:
 *   Zero (OK) means that the driver was successfully registered.  On any
 *   failure, a negated errno value is returned.
 *
 ****************************************************************************/

#if !defined(CONFIG_CDCECM_COMPOSITE)
int cdcecm_initialize(int minor, FAR void **handle);
#endif

/****************************************************************************
 * Name: cdcecm_get_composite_devdesc
 *
 * Description:
 *   Helper function to fill in some constants into the composite
 *   configuration struct.
 *
 * Input Parameters:
 *     dev - Pointer to the configuration struct we should fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CDCECM_COMPOSITE
void cdcecm_get_composite_devdesc(struct composite_devdesc_s *dev);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_CDCECM_H */
