/****************************************************************************
 * net/tcp/tcp.h
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
 ****************************************************************************/

#ifndef _NET_TCP_TCP_H
#define _NET_TCP_TCP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NET_TCP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Function: tcp_wrbuffer_initialize
 *
 * Description:
 *   Initialize the list of free write buffers
 *
 * Assumptions:
 *   Called once early initialization.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
void tcp_wrbuffer_initialize(void);
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

/****************************************************************************
 * Function: tcp_wrbuffer_alloc
 *
 * Description:
 *   Allocate a TCP write buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from TCP logic when a buffer
 *   of TCP data is about to sent
 *
 * Assumptions:
 *   Called from user logic with interrupts enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
struct tcp_wrbuffer_s;

FAR struct tcp_wrbuffer_s *tcp_wrbuffer_alloc(void);
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

/****************************************************************************
 * Function: tcp_wrbuffer_release
 *
 * Description:
 *   Release a TCP write buffer by returning the buffer to the free list.
 *   This function is called from user logic after it is consumed the
 *   buffered data.
 *
 * Assumptions:
 *   Called from interrupt level with interrupts disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
void tcp_wrbuffer_release(FAR struct tcp_wrbuffer_s *wrb);
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

/****************************************************************************
 * Function: tcp_wrbuffer_dump
 *
 * Description:
 *   Dump the contents of a write buffer.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
#ifdef CONFIG_DEBUG
void tcp_wrbuffer_dump(FAR const char *msg, FAR struct tcp_wrbuffer_s *wrb,
                       unsigned int len, unsigned int offset);
#else
#  define tcp_wrbuffer_dump(msg,wrb)
#endif
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_TCP */
#endif /* _NET_TCP_TCP_H */
