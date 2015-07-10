/****************************************************************************
 * libc/netdb/lib_gethostbyaddrr.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <errno.h>

#include <arpa/inet.h>

#include "lib_internal.h"
#include "netdb/lib_netdb.h"

#ifdef CONFIG_NETDB_HOSTFILE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gethostbyaddr_r
 *
 * Description:
 *   The gethostbyaddr_r() function returns a structure of type hostent for
 *   the given host address addr of length len and address type type. Valid
 *   address types are AF_INET and AF_INET6. The host address argument is a
 *   pointer to a struct of a type depending on the address type, for example
 *   a struct in_addr *  for address type AF_INET.
 * 
 *   gethostbyaddr_r() is *not* POSIX but is similar to a Glibc extension and is
 *   used internally by NuttX to implement the POSIX gethostbyaddr().
 *
 * Input Parameters:
 *   addr - The address of the host to find.
 *   len - The length of the address
 *   type - The type of the address
 *   host - Caller provided location to return the host data.
 *   buf - Caller provided buffer to hold string data associated with the
 *     host data.
 *   buflen - The size of the caller-provided buffer
 *   h_errnop - There h_errno value returned in the event of a failure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, -1 (ERROR) is returned on a failure
 *   with the returned h_errno value provided the reason for the failure.
 *
 ****************************************************************************/

int gethostbyaddr_r(FAR const void *addr, socklen_t len, int type,
                    FAR struct hostent *host, FAR char *buf,
                    size_t buflen, int *h_errnop)
{
  FAR FILE *stream;
  int herrnocode;
  int nread;

  DEBUGASSERT(addr != NULL && host != NULL && buf != NULL);
  DEBUGASSERT(type == AF_INET || type == AF_INET6);

  /* Make sure that the h_errno has a non-error code */

  if (h_errnop)
    {
      *h_errnop = 0;
    }

  /* Search the hosts file for a match */

  stream = fopen(CONFIG_NETDB_HOSTCONF_PATH, "r");
  if (stream == NULL)
    {
      int errcode = errno;

      ndbg("ERROR:  Failed to open the hosts file %s: %d\n",
           CONFIG_NETDB_HOSTCONF_PATH, errcode);
      UNUSED(errcode);

      herrnocode = NO_RECOVERY;
      goto errorout_with_herrnocode;
    }

  /* Loop reading entries from the hosts file until a match is found or
   * until we hit the end-of-file.
   */

  do
    {
      /* Read the next entry from the hosts file */

      nread = lib_parse_hostfile(stream, host, buf, buflen);
      if (nread < 0)
        {
          /* Possible errors:
           *     ERANGE - Buffer not big enough
           *     ESPIPE - End of file (or possibly a read error).
           *     EAGAIN - Error parsing the line (E.g., missing hostname)
           */

          if (nread == -ESPIPE)
            {
              nread = 0;
            }
          else if (nread != -EAGAIN)
            {
              herrnocode = NO_RECOVERY;
              goto errorout_with_stream;
            }
        }
      else if (nread > 0 && len == host->h_length && type == host->h_addrtype)
        {
          /* We successfully read the entry and the type and size of the
           * address is good.  Now compare the addresses:
           */

          FAR char *hostaddr = host->h_addr;
          if (hostaddr != NULL)
            {
              nvdbg("Comparing addresses...\n");
              if (memcmp(addr, hostaddr, len) == 0)
                {
                   /* We have a match */

                  fclose(stream);
                  return OK;
                }
            }
        }
    }
  while (nread != 0);

  /* We get here when the end of the hosts file is encountered without
   * finding the hostname.
   */

  herrnocode = HOST_NOT_FOUND;

errorout_with_stream:
  fclose(stream);

errorout_with_herrnocode:
  if (h_errnop)
    {
      *h_errnop = herrnocode;
    }

 return ERROR;
}

#endif /* CONFIG_NETDB_HOSTFILE */
