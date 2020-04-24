/****************************************************************************
 * libs/libc/netdb/lib_parsehostfile.c
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
#include <sys/socket.h>

#include <stdio.h>
#include <netdb.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include <arpa/inet.h>

#include "lib_netdb.h"

#ifdef CONFIG_NETDB_HOSTFILE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if character is any kind of white space (except for newline) */

#define lib_isspace(c) \
  ((c) == ' '  || (c) == '\t' || (c) == '\r' || (c) == '\f' || c == '\v')

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This is the layout of the caller provided memory area */

struct hostent_info_s
{
  FAR char *hi_aliases[CONFIG_NETDB_MAX_ALTNAMES + 1];
  int       hi_addrtypes[1];
  int       hi_lengths[1];
  FAR char *hi_addrlist[2];
  char      hi_data[1];
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_skipspaces
 *
 * Description:
 *   Read from the 'stream' until a non-whitespace character is read or the
 *   end-of-line or end-of-file is encountered.
 *
 * Input Parameters:
 *   stream - The stream to read from
 *   nread  - A count to the pointer of characters read.  Will be
 *     incremented after each successful character read.
 *
 * Returned Value:
 *   The first non-whitespace character read.  This will be the newline
 *   character of EROF if the end-of-line or end-of-file is encountered.
 *
 ****************************************************************************/

static int lib_skipspaces(FAR FILE *stream, FAR size_t *nread)
{
  int ch;

  /* Skip over most white space (but not newline) */

  do
    {
      ch = fgetc(stream);
      if (ch != EOF)
        {
          (*nread)++;
        }
    }
  while (lib_isspace(ch));

  return ch;
}

/****************************************************************************
 * Name: lib_skipline
 *
 * Description:
 *   Read from the 'stream' until the end-of-line or end-of-file is
 *   encountered.
 *
 * Input Parameters:
 *   stream - The stream to read from
 *   nread  - A count to the pointer of characters read.  Will be
 *     incremented after each successful character read.
 *
 * Returned Value:
 *   The character that terminated the line.  This may be either the newline
 *   character or EOF.
 *
 ****************************************************************************/

static int lib_skipline(FAR FILE *stream, FAR size_t *nread)
{
  int ch;

  /* Skip over all characters until we encounter a newline or end-of-file */

  do
    {
      ch = fgetc(stream);
      if (ch != EOF)
        {
          (*nread)++;
        }
    }
  while (ch != EOF && ch != '\n');

  return ch;
}

/****************************************************************************
 * Name: lib_copystring
 *
 * Description:
 *   Read from the 'stream' And copy each byte to the buffer at 'ptr' until
 *   either a whitespace delimiter, the end-of-line, or the end-of-file is
 *   encountered.
 *
 * Input Parameters:
 *   stream - The stream to read from
 *   ptr - The pointer to the buffer to receive the string
 *   nread  - A count to the pointer of characters read.  Will be
 *     incremented after each successful character read.
 *   buflen - The size of the buffer in bytes
 *   terminator - The actual character the terminated the copy is returned
 *     to this location.
 *
 * Returned Value:
 *  Number of bytes written to the buffer on success.  0 if the end of
 *  file is encountered (or a read error occurs).  A negated errno value on
 *  any failure:
 *
 *    -ERANGE - Insufficient buffer space to hold the string.
 *
 ****************************************************************************/

static ssize_t lib_copystring(FAR FILE *stream, FAR char *ptr,
                              FAR size_t *nread, size_t buflen,
                              FAR int *terminator)
{
  size_t nwritten = 0;
  int ch;

  /* Copy the string from the file until any whitepace delimiter is
   * encountered
   */

  for (; ; )
    {
      /* There there space to buffer one more character? */

      if (nwritten >= buflen)
        {
          return -ERANGE;
        }

      /* Read the next character from the file */

      ch = fgetc(stream);
      if (ch != EOF)
        {
          (*nread)++;
        }

      /* Check for whitepace (including \n') or EOF terminating the string */

      if (isspace(ch) || ch == EOF)
        {
          /* Remember what terminated the string */

          *terminator = ch;

          /* Add NUL termination */

          *ptr++ = '\0';

          /* Return EOF if nothing has written */

          return nwritten + 1;
        }

      /* Write the next string to the buffer */

      *ptr++ = ch;
      nwritten++;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: parse_hostfile
 *
 * Description:
 *   Parse the next line from the hosts file.
 *
 * Input Parameters:
 *   stream - File stream of the opened hosts file with the file pointer
 *     positioned at the beginning of the next host entry.
 *   host - Caller provided location to return the host data.
 *   buf - Caller provided buffer to hold string data associated with the
 *     host data.
 *   buflen - The size of the caller-provided buffer
 *
 * Returned Value:
 *   The non-zero number of bytes read from the hosts file is returned if
 *   the host entry was successfully read.  Zero is returned if the end
 *   of the host file has been reached.  A negated errno value is return
 *   in the event a failure:
 *
 *     ERANGE - Buffer not big enough
 *     ESPIPE - End of file (or possibly a read error).
 *     EAGAIN - Error parsing the line (E.g., missing hostname)
 *
 ****************************************************************************/

ssize_t parse_hostfile(FAR FILE *stream, FAR struct hostent_s *host,
                       FAR char *buf, size_t buflen)
{
  FAR struct hostent_info_s *info;
  FAR char addrstring[48];
  FAR char *ptr;
  FAR char *start;
  socklen_t addrlen;
  size_t nread = 0;
  ssize_t nwritten;
  int ret;
  int ch;
  int i;

  /* Verify that we have a buffer big enough to get started (it still may not
   * be big enough).
   */

  if (buflen <= sizeof(struct hostent_info_s))
    {
      return -ERANGE;
    }

  info    = (FAR struct hostent_info_s *)buf;
  ptr     = info->hi_data;
  buflen -= (sizeof(struct hostent_info_s) - 1);

  memset(host, 0, sizeof(struct hostent_s));
  memset(info, 0, sizeof(struct hostent_info_s));

  host->h_addrtypes = info->hi_addrtypes;
  host->h_lengths   = info->hi_lengths;
  host->h_addr_list = info->hi_addrlist;

  /* Skip over any leading spaces */

  do
    {
      ch = lib_skipspaces(stream, &nread);
      if (ch == EOF)
        {
          return -EPIPE;
        }

      /* Skip comment lines beginning with '#' */

      if (ch == '#')
        {
          /* Skip to the end of line. */

          ch = lib_skipline(stream, &nread);
          if (ch == EOF)
            {
              return -EPIPE;
            }
        }
    }
  while (ch == '\n');

  /* Parse the IP address */

  addrstring[0] = ch;

  nwritten = lib_copystring(stream, &addrstring[1], &nread, 47, &ch);
  if (nwritten < 0)
    {
      return nwritten;
    }

  if (!lib_isspace(ch))
    {
      /* The string was terminated with a newline of EOF */

      return ch == EOF ? -EPIPE : -EAGAIN;
    }

  /* If the address contains a colon, say it is IPv6 */

  if (strchr(addrstring, ':') != NULL)
    {
      /* Make sure that space remains to hold the IPv6 address */

      addrlen = sizeof(struct in6_addr);
      if (buflen < addrlen)
        {
          return -ERANGE;
        }

      ret = inet_pton(AF_INET6, addrstring, ptr);
      if (ret <= 0)
        {
          /* Conversion failed.  Entry is corrupted */

          lib_skipline(stream, &nread);
          return -EAGAIN;
        }

      host->h_addrtypes[0] = AF_INET6;
    }
  else
    {
      /* Make sure that space remains to hold the IPv4 address */

      addrlen = sizeof(struct in_addr);
      if (buflen < addrlen)
        {
          return -ERANGE;
        }

      ret = inet_pton(AF_INET, addrstring, ptr);
      if (ret <= 0)
        {
          /* Conversion failed.  Entry is corrupted */

          lib_skipline(stream, &nread);
          return -EAGAIN;
        }

      host->h_addrtypes[0] = AF_INET;
    }

  host->h_addr_list[0] = ptr;
  host->h_lengths[0]   = addrlen;

  ptr    += addrlen;
  buflen -= addrlen;

  /* Skip over any additional whitespace */

  ch = lib_skipspaces(stream, &nread);
  if (ch == EOF)
    {
      return -EPIPE;
    }
  else if (ch == '\n')
    {
      return -EAGAIN;
    }
  else if (buflen == 0)
    {
      return -ERANGE;
    }

  /* Parse the host name */

  start = ptr;
  *ptr++ = ch;
  buflen--;

  nwritten = lib_copystring(stream, ptr, &nread, buflen, &ch);
  if (nwritten < 0)
    {
      return nwritten;
    }

  host->h_name = start;

  if (!lib_isspace(ch))
    {
      /* The string was terminated with a newline or EOF */

      return nread;
    }

  ptr += nwritten;
  buflen -= nwritten;

  /* Parse any host name aliases */

  for (i = 0; ; i++)
    {
      /* Skip over any leading whitespace */

      ch = lib_skipspaces(stream, &nread);
      if (ch == EOF || ch == '\n')
        {
          /* No further aliases on the line */

          return nread;
        }
      else if (buflen == 0 || i >= CONFIG_NETDB_MAX_ALTNAMES)
        {
          return -ERANGE;
        }

      /* Parse the next alias */

      start = ptr;
      *ptr++ = ch;
      buflen--;

      nwritten = lib_copystring(stream, ptr, &nread, buflen, &ch);
      if (nwritten < 0)
        {
          return nwritten;
        }

      /* Save the pointer to the beginning of the next alias */

      info->hi_aliases[i] = start;
      if (host->h_aliases == NULL)
        {
          host->h_aliases = info->hi_aliases;
        }

      if (!lib_isspace(ch))
        {
          /* The string was terminated with a newline of EOF */

          return nread;
        }

      ptr += nwritten;
      buflen -= nwritten;
    }
}

#endif /* CONFIG_NETDB_HOSTFILE */
