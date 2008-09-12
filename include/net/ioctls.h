/****************************************************************************
 * include/net/ioctls.h
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#ifndef __NET_IOCTLS_H
#define __NET_IOCTLS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/ioctl.h> /* _SIOCBASE, etc. */

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* These are ioctl commands to use with a socket FD.  At present, commnads
 * are accepted only to set/get IP addresses, broadcast address, network
 * masks, and hardware address, and a few others
 */

#define _SIOCVALID(c)   (_IOC_TYPE(c)==_SIOCBASE)
#define _SIOC(nr)       _IOC(_SIOCBASE,nr)

#define SIOCGIFADDR     _SIOC(0x0001)  /* Get IP address */
#define SIOCSIFADDR     _SIOC(0x0002)  /* Set IP address */
#define SIOCGIFDSTADDR  _SIOC(0x0003)  /* Get P-to-P address */
#define SIOCSIFDSTADDR  _SIOC(0x0004)  /* Set P-to-P address */
#define SIOCGIFBRDADDR  _SIOC(0x0005)  /* Get broadcast IP address */
#define SIOCSIFBRDADDR  _SIOC(0x0006)  /* Set broadcast IP address */
#define SIOCGIFNETMASK  _SIOC(0x0007)  /* Get network mask */
#define SIOCSIFNETMASK  _SIOC(0x0008)  /* Set network mask */
#define SIOCGIFMTU      _SIOC(0x0009)  /* Get MTU size */
#define SIOCGIFHWADDR   _SIOC(0x000a)  /* Get hardware address */
#define SIOCSIFHWADDR   _SIOC(0x000b)  /* Set hardware address */
#define SIOCDIFADDR     _SIOC(0x000c)  /* Delete IP address */
#define SIOCGIFCOUNT    _SIOC(0x000d)  /* Get number of devices */

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __NET_IOCTLS_H */
