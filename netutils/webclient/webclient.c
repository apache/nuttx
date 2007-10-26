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


static struct webclient_state s;

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

void webclient_init(void)
{
}

static void init_connection(void)
{
  s.state = WEBCLIENT_STATE_STATUSLINE;

  s.getrequestleft = sizeof(http_get) - 1 + 1 +
    sizeof(http_10) - 1 +
    sizeof(http_crnl) - 1 +
    sizeof(http_host) - 1 +
    sizeof(http_crnl) - 1 +
    strlen(http_user_agent_fields) +
    strlen(s.file) + strlen(s.host);
  s.getrequestptr = 0;

  s.httpheaderlineptr = 0;
}

void webclient_close(void)
{
  s.state = WEBCLIENT_STATE_CLOSE;
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

static char *copy_string(char *dest, const char *src, int len)
{
  strncpy(dest, src, len);
  return dest + len;
}

static void senddata(struct uip_driver_s *dev)
{
  uint16 len;
  char *getrequest;
  char *cptr;

  if (s.getrequestleft > 0) {
    cptr = getrequest = (char *)dev->d_appdata;

    cptr = copy_string(cptr, http_get, sizeof(http_get) - 1);
    cptr = copy_string(cptr, s.file, strlen(s.file));
    *cptr++ = ISO_space;
    cptr = copy_string(cptr, http_10, sizeof(http_10) - 1);

    cptr = copy_string(cptr, http_crnl, sizeof(http_crnl) - 1);

    cptr = copy_string(cptr, http_host, sizeof(http_host) - 1);
    cptr = copy_string(cptr, s.host, strlen(s.host));
    cptr = copy_string(cptr, http_crnl, sizeof(http_crnl) - 1);

    cptr = copy_string(cptr, http_user_agent_fields,
		       strlen(http_user_agent_fields));

    len = s.getrequestleft > uip_mss()?
      uip_mss():
      s.getrequestleft;
    uip_send(dev, &(getrequest[s.getrequestptr]), len);
  }
}

static void acked(void)
{
  uint16 len;

  if (s.getrequestleft > 0) {
    len = s.getrequestleft > uip_mss()?
      uip_mss():
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
          if ((strncmp(s.httpheaderline, http_10, sizeof(http_10) - 1) == 0) ||
              (strncmp(s.httpheaderline, http_11, sizeof(http_11) - 1) == 0))
            {
              cptr = &(s.httpheaderline[9]);
              s.httpflag = HTTPFLAG_NONE;
              if (strncmp(cptr, http_200, sizeof(http_200) - 1) == 0)
                {
                  /* 200 OK */
                  s.httpflag = HTTPFLAG_OK;
                }
              else if (strncmp(cptr, http_301, sizeof(http_301) - 1) == 0 ||
                       strncmp(cptr, http_302, sizeof(http_302) - 1) == 0)
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
              uip_abort();
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
          if (casecmp(s.httpheaderline, http_content_type, sizeof(http_content_type) - 1) == 0)
            {
              /* Found Content-type field. */
              cptr = strchr(s.httpheaderline, ';');
              if (cptr != NULL)
                {
                  *cptr = 0;
                }
              strncpy(s.mimetype, s.httpheaderline + sizeof(http_content_type) - 1, sizeof(s.mimetype));
            }
          else if (casecmp(s.httpheaderline, http_location, sizeof(http_location) - 1) == 0)
            {
              cptr = s.httpheaderline + sizeof(http_location) - 1;

              if (strncmp(cptr, http_http, 7) == 0)
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

  len = uip_datalen(dev);

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

void uip_interrupt_event(struct uip_driver_s *dev)
{
#warning OBSOLETE -- needs to be redesigned
  if (uip_connected())
    {
      s.timer = 0;
      s.state = WEBCLIENT_STATE_STATUSLINE;
      senddata(dev);
      webclient_connected();
      return;
    }

  if (s.state == WEBCLIENT_STATE_CLOSE)
    {
      webclient_closed();
      uip_abort();
      return;
    }

  if (uip_aborted())
    {
      webclient_aborted();
    }

  if (uip_timedout())
    {
      webclient_timedout();
    }

  if (uip_acked())
    {
      s.timer = 0;
      acked();
    }

  if (uip_newdata())
    {
      s.timer = 0;
      newdata(dev);
    }

  if (uip_rexmit() || uip_newdata() || uip_acked())
    {
      senddata(dev);
    }
  else if (uip_poll())
    {
      ++s.timer;
      if (s.timer == WEBCLIENT_TIMEOUT)
        {
          webclient_timedout();
          uip_abort();
          return;
        }
    }

  if (uip_closed())
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
              return;
            }
          webclient_get(s.host, s.port, s.file);
        }
    }
}
