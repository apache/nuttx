/****************************************************************************
 * include/nuttx/net/netdb.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Includes some definitions that a compatible with the LGPL GNU C Library
 * header file of the same name.
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

#ifndef __INCLUDE_NUTTX_NET_NETDB_H
#define __INCLUDE_NUTTX_NET_NETDB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/net/netconfig.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: netdb_gethostname
 *
 * Description:
 *   The netdb_gethostname() function will return the standard host name for
 *   the current machine. The namelen argument will specify the size of the
 *   array pointed to by the name argument. The returned name will be null-
 *   terminated, except that if namelen is an insufficient length to hold the
 *   host name, then the returned name will be truncated and it is
 *   unspecified whether the returned name is null-terminated.
 *
 *   Host names are limited to {HOST_NAME_MAX} bytes.
 *
 * Input Parameters:
 *   name - The user-provided buffer to receive the host name.
 *   namelen - The size of the user provided buffer in bytes.
 *
 * Returned Value:
 *   Upon successful completion, 0 will be returned; otherwise, -1 will be
 *   returned.  No errors are defined;  errno variable is not set.
 *
 ****************************************************************************/

int netdb_gethostname(FAR char *name, size_t namelen);

/****************************************************************************
 * Name: netdb_sethostname
 *
 * Description:
 *
 *   netdb_sethostname() sets the hostname to the value given in the
 *   character array name. The len argument specifies the number of bytes in
 *   name.  (Thus, name does not require a terminating null byte.)
 *
 * Input Parameters:
 *   name - The user buffer to providing the new host name.
 *   namelen - The size of the user buffer in bytes.
 *
 * Returned Value:
 *   Upon successful completion, 0 will be returned; otherwise, -1 will be
 *   returned.  No errors are defined;  errno variable is not set.
 *
 ****************************************************************************/

int netdb_sethostname(FAR const char *name, size_t size);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_NETDB_H */
