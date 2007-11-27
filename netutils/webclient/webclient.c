/****************************************************************************
 * netutils/webclient/webclient.c
 * Implementation of the HTTP client.
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * webclient_datahandler(), webclient_connected(),
 * webclient_timedout(), webclient_aborted(), webclient_closed().
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <string.h>
#include <sys/socket.h>

#include <net/uip/uip.h>
#include <net/uip/resolv.h>

#include <net/uip/uip-arch.h>
#include <net/uip/uip-lib.h>

#include "webclient.h"

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
 * Private Data
 ****************************************************************************/

static uint8 g_return; /* Kludge for now */
static struct webclient_state s;

static const char g_http10[]          = "HTTP/1.0";
static const char g_http11[]          = "HTTP/1.1";
static const char g_httpcontenttype[] = "content-type: ";
static const char g_httphost[]        = "host: ";
static const char g_httplocation[]    = "location: ";

static const char g_httpget[]         = "GET ";
static const char g_httphttp[]        = "http://";

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

static void init_connection(void)
{
  s.state = WEBCLIENT_STATE_STATUSLINE;

  s.getrequestleft = strlen(g_httpget) - 1 + 1 +
    strlen(g_http10) - 1 +
    strlen(g_httpcrnl) - 1 +
    strlen(g_httphost) - 1 +
    strlen(g_httpcrnl) - 1 +
    strlen(g_httpuseragentfields) +
    strlen(s.file) + strlen(s.host);
  s.getrequestptr = 0;

  s.httpheaderlineptr = 0;
}

static char *copy_string(char *dest, const char *src, int len)
{
  strncpy(dest, src, len);
  return dest + len;
}

static void senddata(struct uip_driver_s *dev, struct uip_conn *conn)
{
  uint16 len;
  char *getrequest;
  char *cptr;

  if (s.getrequestleft > 0) {
    cptr = getrequest = (char *)dev->d_appdata;

    cptr = copy_string(cptr, g_httpget, strlen(g_httpget) - 1);
    cptr = copy_string(cptr, s.file, strlen(s.file));
    *cptr++ = ISO_space;
    cptr = copy_string(cptr, g_http10, strlen(g_http10) - 1);

    cptr = copy_string(cptr, g_httpcrnl, strlen(g_httpcrnl) - 1);

    cptr = copy_string(cptr, g_httphost, strlen(g_httphost) - 1);
    cptr = copy_string(cptr, s.host, strlen(s.host));
    cptr = copy_string(cptr, g_httpcrnl, strlen(g_httpcrnl) - 1);

    cptr = copy_string(cptr, g_httpuseragentfields,
		       strlen(g_httpuseragentfields));

    len = s.getrequestleft > uip_mss(conn)?
      uip_mss(conn):
      s.getrequestleft;
    uip_send(dev, &(getrequest[s.getrequestptr]), len);
  }
}

static void acked(struct uip_conn *conn)
{
  uint16 len;

  if (s.getrequestleft > 0) {
    len = s.getrequestleft > uip_mss(conn)?
      uip_mss(conn):
      s.getrequestleft;
    s.getrequestleft -= len;
    s.getrequestptr += len;
  }
}

static uint16 parse_statusline(struct uip_driver_s *dev, uint16 len)
{
  char *cptr;

  while(len > 0 && s.httpheaderlineptr < sizeof(s.httpheaderline))
    {
      char *pappdata = (char*)dev->d_appdata;
      s.httpheaderline[s.httpheaderlineptr] = *pappdata++;
      dev->d_appdata = (void*)pappdata;
      len--;

      if (s.httpheaderline[s.httpheaderlineptr] == ISO_nl)
        {
          if ((strncmp(s.httpheaderline, g_http10, strlen(g_http10) - 1) == 0) ||
              (strncmp(s.httpheaderline, g_http11, strlen(g_http11) - 1) == 0))
            {
              cptr = &(s.httpheaderline[9]);
              s.httpflag = HTTPFLAG_NONE;
              if (strncmp(cptr, g_http200, strlen(g_http200) - 1) == 0)
                {
                  /* 200 OK */
                  s.httpflag = HTTPFLAG_OK;
                }
              else if (strncmp(cptr, g_http301, strlen(g_http301) - 1) == 0 ||
                       strncmp(cptr, g_http302, strlen(g_http302) - 1) == 0)
                {
                  /* 301 Moved permanently or 302 Found. Location: header line
                   * will contain thw new location.
                   */

                  s.httpflag = HTTPFLAG_MOVED;
                }
              else
                {
                  s.httpheaderline[s.httpheaderlineptr - 1] = 0;
                }
            }
          else
            {
              g_return |= UIP_ABORT;
              webclient_aborted();
              return 0;
            }

          /* We're done parsing the status line, so we reset the pointer
           * and start parsing the HTTP headers.
           */

          s.httpheaderlineptr = 0;
          s.state = WEBCLIENT_STATE_HEADERS;
          break;
        }
      else
        {
          ++s.httpheaderlineptr;
        }
    }
  return len;
}

static char casecmp(char *str1, const char *str2, char len)
{
  static char c;

  while(len > 0) {
    c = *str1;
    /* Force lower-case characters. */
    if (c & 0x40) {
      c |= 0x20;
    }
    if (*str2 != c) {
      return 1;
    }
    ++str1;
    ++str2;
    --len;
  }
  return 0;
}

static uint16 parse_headers(struct uip_driver_s *dev, uint16 len)
{
  char *cptr;
  static unsigned char i;

  while(len > 0 && s.httpheaderlineptr < sizeof(s.httpheaderline))
    {
      char *pappdata = (char*)dev->d_appdata;
      s.httpheaderline[s.httpheaderlineptr] = *pappdata++;
      dev->d_appdata = (void*)pappdata;
      len--;

      if (s.httpheaderline[s.httpheaderlineptr] == ISO_nl)
        {
          /* We have an entire HTTP header line in s.httpheaderline, so
           * we parse it.
           */

          if (s.httpheaderline[0] == ISO_cr)
            {
              /* This was the last header line (i.e., and empty "\r\n"), so
               * we are done with the headers and proceed with the actual
               * data.
               */

              s.state = WEBCLIENT_STATE_DATA;
              return len;
            }

          s.httpheaderline[s.httpheaderlineptr - 1] = 0;

          /* Check for specific HTTP header fields. */
          if (casecmp(s.httpheaderline, g_httpcontenttype, strlen(g_httpcontenttype) - 1) == 0)
            {
              /* Found Content-type field. */
              cptr = strchr(s.httpheaderline, ';');
              if (cptr != NULL)
                {
                  *cptr = 0;
                }
              strncpy(s.mimetype, s.httpheaderline + strlen(g_httpcontenttype) - 1, sizeof(s.mimetype));
            }
          else if (casecmp(s.httpheaderline, g_httplocation, strlen(g_httplocation) - 1) == 0)
            {
              cptr = s.httpheaderline + strlen(g_httplocation) - 1;

              if (strncmp(cptr, g_httphttp, strlen(g_httphttp)) == 0)
                {
                  cptr += 7;
                  for(i = 0; i < s.httpheaderlineptr - 7; ++i)
                    {
                      if (*cptr == 0 || *cptr == '/' || *cptr == ' ' || *cptr == ':')
                        {
                          s.host[i] = 0;
                          break;
                        }
                      s.host[i] = *cptr;
                      ++cptr;
                    }
                }
              strncpy(s.file, cptr, sizeof(s.file));
            }

          /* We're done parsing, so we reset the pointer and start the
           * next line.
           */

          s.httpheaderlineptr = 0;
        }
      else
        {
          ++s.httpheaderlineptr;
        }
    }
  return len;
}

static void newdata(struct uip_driver_s *dev)
{
  uint16 len;

  len = dev->d_len;

  if (s.state == WEBCLIENT_STATE_STATUSLINE) {
    len = parse_statusline(dev, len);
  }

  if (s.state == WEBCLIENT_STATE_HEADERS && len > 0) {
    len = parse_headers(dev, len);
  }

  if (len > 0 && s.state == WEBCLIENT_STATE_DATA &&
     s.httpflag != HTTPFLAG_MOVED) {
    webclient_datahandler((char *)dev->d_appdata, len);
  }
}

/* This function is called by the UIP interrupt handling logic whenevent an
 * event of interest occurs.
 */

uint8 uip_interrupt_event(struct uip_driver_s *dev, struct uip_conn *conn, uint8 flags)
{
#warning OBSOLETE -- needs to be redesigned
  g_return = flags;

  if ((flags & UIP_CONNECTED) != 0)
    {
      s.timer = 0;
      s.state = WEBCLIENT_STATE_STATUSLINE;
      senddata(dev, conn);
      webclient_connected();
      return g_return;
    }

  if (s.state == WEBCLIENT_STATE_CLOSE)
    {
      webclient_closed();
      return UIP_ABORT;
    }

  if ((flags & UIP_ABORT) != 0)
    {
      webclient_aborted();
    }

  if ((flags & UIP_TIMEDOUT) != 0)
    {
      webclient_timedout();
    }

  if ((flags & UIP_ACKDATA) != 0)
    {
      s.timer = 0;
      acked(conn);
    }

  if ((flags & UIP_NEWDATA) != 0)
    {
      s.timer = 0;
      newdata(dev);
    }

  if ((flags & UIP_REXMIT) != 0 || (flags & UIP_NEWDATA) != 0 || (flags & UIP_ACKDATA) != 0)
    {
      senddata(dev, conn);
    }
  else if ((flags & UIP_POLL) != 0)
    {
      ++s.timer;
      if (s.timer == WEBCLIENT_TIMEOUT)
        {
          webclient_timedout();
          return UIP_ABORT;
        }
    }

  if ((flags & UIP_CLOSE) != 0)
    {
      if (s.httpflag != HTTPFLAG_MOVED)
        {
          /* Send NULL data to signal EOF. */
          webclient_datahandler(NULL, 0);
        }
      else
        {
#ifdef CONFIG_NET_IPv6
          struct sockaddr_in6 addr;
#else
          struct sockaddr_in addr;
#endif
          if (resolv_query(s.host, &addr) < 0)
            {
              return g_return;
            }
          webclient_get(s.host, s.port, s.file);
        }
    }
  return g_return;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void webclient_init(void)
{
}

unsigned char webclient_get(const char *host, uint16 port, char *file)
{
  uip_ipaddr_t *ipaddr;
  static uip_ipaddr_t addr;
  struct sockaddr_in server;
  int sockfd;

  /* First check if the host is an IP address. */

  ipaddr = &addr;
  if (uiplib_ipaddrconv(host, (unsigned char *)addr) == 0)
    {
      if (resolv_query(host, &ipaddr) < 0)
        {
          return ERROR;
        }
  }

  /* Create a socket */

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    {
      return ERROR;
    }

  /* Connect to server.  First we have to set some fields in the
   * 'server' structure.  The system will assign me an arbitrary
   * local port that is not in use.
   */

  server.sin_family = AF_INET;
  memcpy(&server.sin_addr.s_addr, &host, sizeof(in_addr_t));
  server.sin_port = htons(port);

  if (connect(sockfd, (struct sockaddr *)&server, sizeof(struct sockaddr_in)) < 0)
    {
      return ERROR;
  }

  s.port = port;
  strncpy(s.file, file, sizeof(s.file));
  strncpy(s.host, host, sizeof(s.host));

  init_connection();
  return OK;
}

void webclient_close(void)
{
  s.state = WEBCLIENT_STATE_CLOSE;
}

char *webclient_mimetype(void)
{
  return s.mimetype;
}

char *webclient_filename(void)
{
  return s.file;
}

char *webclient_hostname(void)
{
  return s.host;
}

unsigned shortwebclient_port(void)
{
  return s.port;
}

