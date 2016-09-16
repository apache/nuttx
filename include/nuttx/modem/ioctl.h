/****************************************************************************
 * include/nuttx/modem/ioctl.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_MODEM_IOCTL_H
#define __INCLUDE_NUTTX_MODEM_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All modem-related IOCTL commands are defined here to assure that they are
 * globally unique.
 */

/* U-Blox Modem IOCTL commands */

#define MODEM_IOC_POWERON    _MODEMIOC(1)
#define MODEM_IOC_POWEROFF   _MODEMIOC(2)
#define MODEM_IOC_RESET      _MODEMIOC(3)
#define MODEM_IOC_GETSTATUS  _MODEMIOC(4)

/* Unrecognized IOCTL commands are forwarded to the lower half driver.  These
 * may include the modem commands from include/nuttx/serial/ioctl.h such as
 * the following:
 *
 *   TIOCMGET: Get modem status bits: FAR int
 *   TIOCMSET: Set modem status bits: FAR const int
 *   TIOCMBIC: Clear modem bits: FAR const int
 *   TIOCMBIS: Set modem bits: FAR const int
 */

#endif /* __INCLUDE_NUTTX_MODEM_IOCTL_H */
