/****************************************************************************
 * include/net/uip/webclient.h
 * Header file for the HTTP client
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Based remotely on the uIP webclient which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2002, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __NET_UIP_WEBCLIENT_H
#define __NET_UIP_WEBCLIENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef CONFIG_NETUTILS_WEBCLIENT_HOST
#  include <nuttx/config.h>
#endif
#include <sys/types.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#ifndef CONFIG_NETUTILS_WEBCLIENT_MAXHTTPLINE
#  define CONFIG_NETUTILS_WEBCLIENT_MAXHTTPLINE 200
#endif

#ifndef CONFIG_NETUTILS_WEBCLIENT_MAXMIMESIZE
#  define CONFIG_NETUTILS_WEBCLIENT_MAXMIMESIZE 32
#endif

#ifndef CONFIG_NETUTILS_WEBCLIENT_MAXHOSTNAME
#  define CONFIG_NETUTILS_WEBCLIENT_MAXHOSTNAME 40
#endif

#ifndef CONFIG_NETUTILS_WEBCLIENT_MAXFILENAME
#  define CONFIG_NETUTILS_WEBCLIENT_MAXFILENAME 100
#endif

/****************************************************************************
 * Public types
 ****************************************************************************/

/* wget calls a user provided function of the follwoing type to process
 * each received chuck of the incoming file data.  If the system has a file
 * system, then it may just write the data to a file.  Or it may buffer the
 * file in memory.  To facilitate this latter case, the caller may modify
 * the buffer address in this callback by writing to buffer and buflen. This
 * may be used, for example, to implement double buffering.
 *
 * Input Parameters:
 *   buffer - A pointer to a pointer to a buffer.  If the callee wishes to
 *       change the buffer address, it may do so in the callback by writing
 *       to buffer.
 *   offset - Offset to the beginning of valid data in the buffer.  Offset
 *       is used to skip over any HTTP header info that may be at the
 *       beginning of the buffer.
 *   datend - The end+1 offset of valid data in the buffer.  The total number
 *       of valid bytes is datend - offset.
 *   buflen - A pointer to the length of the buffer.  If the callee wishes
 *       to change the size of the buffer, it may write to buflen.
 */

typedef void (*wget_callback_t)(FAR char **buffer, int offset,
                                int datend, FAR int *buflen);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: wget
 *
 * Description:
 *   Obtain the requested file from an HTTP server using the GET method.
 *
 *   Note: If the function is passed a host name, it must already be in
 *   the resolver cache in order for the function to connect to the web
 *   server. It is therefore up to the calling module to implement the
 *   resolver calls and the signal handler used for reporting a resolv
 *   query answer.
 *
 * Input Parameters
 *   port     - The port number to which to connect, in host byte order.
 *   hostname - A pointer to a string containing either a host name or
 *              a numerical IP address in dotted decimal notation (e.g., 192.168.23.1).
 *   filename - A pointer to the name of the file to get.
 *   buffer   - A user provided buffer to receive the file data (also
 *              used for the outgoing GET request
 *   buflen   - The size of the user provided buffer
 *   callback - As data is obtained from the host, this function is
 *              to dispose of each block of file data as it is received.
 *
 * Returned Value:
 *   0: if the GET operation completed successfully;
 *  -1: On a failure with errno set appropriately 
 *
 ****************************************************************************/

EXTERN int wget(uint16 port,
                FAR const char *hostname, FAR const char *filename,
                FAR char *buffer, int buflen, wget_callback_t callback);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __NET_UIP_WEBCLIENT_H */
