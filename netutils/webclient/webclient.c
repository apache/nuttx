/****************************************************************************
 * netutils/webclient/webclient.c
 * Implementation of the HTTP client.
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Based on uIP which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2002, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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

/* This example shows a HTTP client that is able to download web pages
 * and files from web servers. It requires a number of callback
 * functions to be implemented by the module that utilizes the code:
 * webclient_datahandler().
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>

#include <net/uip/uip.h>
#include <net/uip/uip-lib.h>
#include <net/uip/resolv.h>
#include <net/uip/webclient.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define WEBCLIENT_TIMEOUT 100

#define WEBCLIENT_STATE_STATUSLINE 0
#define WEBCLIENT_STATE_HEADERS    1
#define WEBCLIENT_STATE_DATA       2
#define WEBCLIENT_STATE_CLOSE      3

#define HTTPFLAG_NONE   0
#define HTTPFLAG_OK     1
#define HTTPFLAG_MOVED  2
#define HTTPFLAG_ERROR  3

#define ISO_nl       0x0a
#define ISO_cr       0x0d
#define ISO_space    0x20

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct wget_s
{
  ubyte state;
  ubyte httpflag;

  /* These describe the just-received buffer of data */

  FAR char *buffer;  /* user-provided buffer */
  int buflen;        /* Length of the user provided buffer */
  int offset;        /* Offset to the beginning of interesting data */
  int datend;        /* Offset+1 to the last valid byte of data in the buffer */  

  /* Buffer HTTP header data and parse line at a time */

  char line[CONFIG_NETUTILS_WEBCLIENT_MAXHTTPLINE];
  int  ndx;

#ifdef CONFIG_WEBCLIENT_GETMIMETYPE
  char mimetype[CONFIG_NETUTILS_WEBCLIENT_MAXMIMESIZE];
#endif
#ifdef CONFIG_WEBCLIENT_GETHOST
  char host[CONFIG_NETUTILS_WEBCLIENT_MAXHOSTNAME];
#endif
};


/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_http10[]          = "HTTP/1.0";
static const char g_http11[]          = "HTTP/1.1";
#ifdef CONFIG_WEBCLIENT_GETMIMETYPE
static const char g_httpcontenttype[] = "content-type: ";
#endif
static const char g_httphost[]        = "host: ";
#ifdef CONFIG_WEBCLIENT_GETHOST
static const char g_httplocation[]    = "location: ";
#endif

static const char g_httpget[]         = "GET ";
#ifdef CONFIG_WEBCLIENT_GETHOST
static const char g_httphttp[]        = "http://";
#endif

static const char g_httpuseragentfields[] =
  "Connection: close\r\n"
  "User-Agent: uIP/1.0 (; http://www.sics.se/~adam/uip/)\r\n\r\n";

static const char g_http200[]         = "200 ";
static const char g_http301[]         = "301 ";
static const char g_http302[]         = "302 ";

static const char g_httpcrnl[]        = "\r\n";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wget_strcpy
 ****************************************************************************/

static char *wget_strcpy(char *dest, const char *src)
{
  int len = strlen(src);
  memcpy(dest, src, len);
  return dest + len;
}

/****************************************************************************
 * Name: wget_parsestatus
 ****************************************************************************/

static inline int wget_parsestatus(struct wget_s *ws)
{
  int offset;
  int ndx;
  char *dest;

  offset = ws->offset;
  ndx    = ws->ndx;

  while (offset < ws->datend)
    {
      ws->line[ndx] = ws->buffer[offset];
      if (ws->line[ndx] == ISO_nl)
        {
          ws->line[ndx] = '\0';
          if ((strncmp(ws->line, g_http10, strlen(g_http10)) == 0) ||
              (strncmp(ws->line, g_http11, strlen(g_http11)) == 0))
            {
              dest = &(ws->line[9]);
              ws->httpflag = HTTPFLAG_NONE;

              /* Check for 200 OK */

              if (strncmp(dest, g_http200, strlen(g_http200)) == 0)
                {
                  ws->httpflag = HTTPFLAG_OK;
                }

              /* Check for 301 Moved permanently or 302 Found. Location: header line
               * will contain the new location.
               */

              else if (strncmp(dest, g_http301, strlen(g_http301)) == 0 ||
                       strncmp(dest, g_http302, strlen(g_http302)) == 0)
                {

                  ws->httpflag = HTTPFLAG_MOVED;
                }
            }
          else
            {
              return - ECONNABORTED;
            }

          /* We're done parsing the status line, so start parsing the HTTP headers. */

          ws->state = WEBCLIENT_STATE_HEADERS;
          break;
        }
      else
        {
          offset++;
          ndx++;
        }
    }

  ws->offset = offset;
  ws->ndx    = ndx;
  return OK;
}

/****************************************************************************
 * Name: wget_parsestatus
 ****************************************************************************/

static inline int wget_parseheaders(struct wget_s *ws)
{
  int offset;
  int ndx;
  char *dest;

  offset = ws->offset;
  ndx    = ws->ndx;

  while (offset < ws->datend)
    {
      ws->line[ndx] = ws->buffer[offset];
      if (ws->line[ndx] == ISO_nl)
        {
          /* We have an entire HTTP header line in s.line, so
           * we parse it.
           */

          if (ws->line[0] == ISO_cr)
            {
              /* This was the last header line (i.e., and empty "\r\n"), so
               * we are done with the headers and proceed with the actual
               * data.
               */

              ws->state = WEBCLIENT_STATE_DATA;
              return OK;
            }

          ws->line[ndx] = '\0';

          /* Check for specific HTTP header fields. */

#ifdef CONFIG_WEBCLIENT_GETMIMETYPE
          if (strncasecmp(ws->line, g_httpcontenttype, strlen(g_httpcontenttype)) == 0)
            {
              /* Found Content-type field. */
              dest = strchr(ws->line, ';');
              if (dest != NULL)
                {
                  *dest = 0;
                }
              strncpy(ws->mimetype, ws->line + strlen(g_httpcontenttype) - 1, sizeof(ws->mimetype));
            }
#  ifdef CONFIG_WEBCLIENT_GETHOST
          else
#  endif
#endif
#ifdef CONFIG_WEBCLIENT_GETHOST
          if (strncasecmp(ws->line, g_httplocation, strlen(g_httplocation)) == 0)
            {
              dest = ws->line + strlen(g_httplocation) - 1;

              if (strncmp(dest, g_httphttp, strlen(g_httphttp)) == 0)
                {
                  dest += 7;
                  for(i = 0; i < ws->ndx - 7; ++i)
                    {
                      if (*dest == 0 || *dest == '/' || *dest == ' ' || *dest == ':')
                        {
                          ws->host[i] = 0;
                          break;
                        }
                      ws->host[i] = *dest;
                      ++dest;
                    }
                }
              strncpy(ws->file, dest, sizeof(ws->file));
            }
#endif

          /* We're done parsing, so we reset the pointer and start the
           * next line.
           */

          ndx = 0;
        }
      else
        {
          ndx++;
          offset++;
        }
    }

  ws->offset = offset;
  ws->ndx    = ndx;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wget
 ****************************************************************************/
 
int wget(FAR const char *host, uint16 port, FAR const char *file,
         FAR char *buffer, int buflen, wget_callback_t callback)
{
  static uip_ipaddr_t addr;
  struct sockaddr_in server;
  struct wget_s ws;
  char *dest;
  int sockfd;
  int len;
  int ret = OK;

  /* First check if the host is an IP address. */

  if (!uiplib_ipaddrconv(host, (unsigned char *)addr))
    {
      /* 'host' does not point to a avalid address string.  Try to resolve
       *  the host name to an IP address.
       */
 
      if (resolv_query(host, &server) < 0)
        {
          return ERROR;
        }

      /* Save the host address */

      addr = server.sin_addr.s_addr;
  }

  /* Create a socket */

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    {
      /* socket failed.  It will set the errno appropriately */

      ndbg("socket failed: %d\n", errno);
      return ERROR;
    }

  /* Connect to server.  First we have to set some fields in the
   * 'server' address structure.  The system will assign me an arbitrary
   * local port that is not in use.
   */

  server.sin_family = AF_INET;
  memcpy(&server.sin_addr.s_addr, &host, sizeof(in_addr_t));
  server.sin_port   = htons(port);

  ret = connect(sockfd, (struct sockaddr *)&server, sizeof(struct sockaddr_in));
  if (ret < 0)
    {
      ndbg("connect failed: %d\n", errno);
      goto errout;
  }

  /* Send the GET request */

  dest   = (char *)buffer;
  dest   = wget_strcpy(dest, g_httpget);
  dest   = wget_strcpy(dest, file);
 *dest++ = ISO_space;
  dest   = wget_strcpy(dest, g_http10);
  dest   = wget_strcpy(dest, g_httpcrnl);
  dest   = wget_strcpy(dest, g_httphost);
  dest   = wget_strcpy(dest, host);
  dest   = wget_strcpy(dest, g_httpcrnl);
  dest   = wget_strcpy(dest, g_httpuseragentfields);
  len    = dest - buffer;

  ret    = send(sockfd, buffer, len, 0);
  if (ret < 0)
    {
      ndbg("send failed: %d\n", errno);
      goto errout;
    }

  /* Now get the response */

  memset(&ws, 0, sizeof(struct wget_s));
  ws.state  = WEBCLIENT_STATE_STATUSLINE;
  ws.buffer = buffer;
  ws.buflen = buflen;
  
  for (;;)
    {
      ws.datend = recv(sockfd, ws.buffer, ws.buflen, 0);
      if (ws.datend < 0)
        {
          ndbg("recv failed: %d\n", errno);
          ret = ws.datend;
          goto errout;
        }
      else if (ret == 0)
        {
          break;
        }
  
      ws.offset = 0;
      if (ws.state == WEBCLIENT_STATE_STATUSLINE)
        {
          ret = wget_parsestatus(&ws);
          if (ret < 0)
            {
              goto errout_with_errno;
            }
        }

      if (ws.state == WEBCLIENT_STATE_HEADERS)
        {
          ret = wget_parseheaders(&ws);
          if (ret < 0)
            {
              goto errout_with_errno;
            }
        }

      /* Let the client decide what to do with the received file */

      if (ws.state == WEBCLIENT_STATE_DATA && ws.httpflag != HTTPFLAG_MOVED)
        {
          callback(&ws.buffer, ws.offset, ws.datend, &buflen);
        }
    }

okout:
  close(sockfd);
  return OK;

errout_with_errno:
  errno = -ret;
errout:
  close(sockfd);
  return ERROR;
}
