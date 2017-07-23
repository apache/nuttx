/****************************************************************************
 * include/nuttx/power/power_ioctl.h
 * NuttX Power-Related IOCTLs definitions
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_POWER_POWER_IOCTL_H
#define __INCLUDE_NUTTX_POWER_POWER_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All power-related IOCTL commands must be defined in this header file
 * in order to assure that every IOCTL command is unique and will not be
 * aliased.
 */

#define PWRIOC_START          _PWRIOC(1)
#define PWRIOC_STOP           _PWRIOC(2)
#define PWRIOC_SET_MODE       _PWRIOC(3)
#define PWRIOC_SET_LIMITS     _PWRIOC(4)
#define PWRIOC_GET_STATE      _PWRIOC(5)
#define PWRIOC_GET_FAULT      _PWRIOC(6)
#define PWRIOC_SET_FAULT      _PWRIOC(7)
#define PWRIOC_CLEAN_FAULT    _PWRIOC(8)
#define PWRIOC_SET_PARAMS     _PWRIOC(9)

#endif /* __INCLUDE_NUTTX_POWER_POWER_IOCTL_H */
