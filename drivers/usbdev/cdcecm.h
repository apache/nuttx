/****************************************************************************
 * drivers/usbdev/cdcecm.h
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
 ****************************************************************************/

#ifndef __DRIVERS_USBDEV_CDCECM_H
#define __DRIVERS_USBDEV_CDCECM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>

#include <nuttx/usb/cdcecm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CDCECM_VERSIONNO         (0x0100)
#define CDCECM_MXDESCLEN         (80)
#define CDCECM_MAXSTRLEN         (CDCECM_MXDESCLEN - 2)
#define CDCECM_NCONFIGS          (1)
#define CDCECM_NINTERFACES       (2)
#define CDCECM_NUM_EPS           (3)

#define CDCECM_MANUFACTURERSTRID (1)
#define CDCECM_PRODUCTSTRID      (2)
#define CDCECM_SERIALSTRID       (3)
#define CDCECM_CONFIGSTRID       (4)
#define CDCECM_MACSTRID          (5)
#define CDCECM_NSTRIDS           (5)

#define CDCECM_STR_LANGUAGE      (0x0409) /* en-us */

#define CDCECM_CONFIGID_NONE     (0)
#define CDCECM_CONFIGID          (1)

#define CDCECM_SELFPOWERED       (0)
#define CDCECM_REMOTEWAKEUP      (0)

#ifndef MIN
#  define MIN(a,b) ((a)<(b)?(a):(b))
#endif

#endif /* __DRIVERS_USBDEV_CDCECM_H */
