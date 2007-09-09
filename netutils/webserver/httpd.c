/* httpd
 * httpd Web server
 * Author: Adam Dunkels <adam@sics.se>
 *
 * The uIP web server is a very simplistic implementation of an HTTP
 * server. It can serve web pages and files from a read-only ROM
 * filesystem, and provides a very small scripting language.
 *
 * Copyright (c) 2004, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <stdlib.h>
#include <sys/socket.h>

#include <net/uip/uip.h>
#include <net/uip/httpd.h>

#include "httpd.h"
#include "httpd-cgi.h"
#include "http-strings.h"

#include <string.h>

#define STATE_WAITING 0
#define STATE_OUTPUT  1

#define ISO_nl      0x0a
#define ISO_space   0x20
#define ISO_bang    0x21
#define ISO_percent 0x25
#define ISO_period  0x2e
#define ISO_slash   0x2f
#define ISO_colon   0x3a

#define SEND_STR(psock, str) psock_send(psock, str, strlen(str))

static inline int send_file(struct httpd_state *pstate)
{
  return send(pstate->sockout, pstate->file.data, pstate->file.len, 0);
}

static inline int send_part_of_file(struct httpd_state *pstate)
{
  return send(pstate->sockout, pstate->file.data, pstate->len, 0);
}

static void next_scriptstate(struct httpd_state *pstate)
{
  char *p;
  p = strchr(pstate->scriptptr, ISO_nl) + 1;
  pstate->scriptlen -= (unsigned short)(p - pstate->scriptptr);
  pstate->scriptptr = p;
}

static void handle_script(struct httpd_state *pstate)
{
  char *ptr;

  while(pstate->file.len > 0) {

    /* Check if we should start executing a script. */
    if (*pstate->file.data == ISO_percent &&
       *(pstate->file.data + 1) == ISO_bang) {
      pstate->scriptptr = pstate->file.data + 3;
      pstate->scriptlen = pstate->file.len - 3;
      if (*(pstate->scriptptr - 1) == ISO_colon) {
	httpd_fs_open(pstate->scriptptr + 1, &pstate->file);
	send_file(pstate);
      } else {
        httpd_cgi(pstate->scriptptr)(pstate, pstate->scriptptr);
      }
      next_scriptstate(pstate);

      /* The script is over, so we reset the pointers and continue
	 sending the rest of the file. */
      pstate->file.data = pstate->scriptptr;
      pstate->file.len = pstate->scriptlen;
    } else {
      /* See if we find the start of script marker in the block of HTML
	 to be sent. */

      if (pstate->file.len > uip_mss()) {
	pstate->len = uip_mss();
      } else {
	pstate->len = pstate->file.len;
      }

      if (*pstate->file.data == ISO_percent) {
	ptr = strchr(pstate->file.data + 1, ISO_percent);
      } else {
	ptr = strchr(pstate->file.data, ISO_percent);
      }
      if (ptr != NULL &&
	 ptr != pstate->file.data) {
	pstate->len = (int)(ptr - pstate->file.data);
	if (pstate->len >= uip_mss()) {
	  pstate->len = uip_mss();
	}
      }
      send_part_of_file(pstate);
      pstate->file.data += pstate->len;
      pstate->file.len -= pstate->len;
    }
  }
}

static int send_headers(struct httpd_state *pstate, const char *statushdr)
{
  char *ptr;
  int ret;

  ret = send(pstate->sockout, statushdr, strlen(statushdr), 0);

  ptr = strrchr(pstate->filename, ISO_period);
  if (ptr == NULL)
    {
      ret = send(pstate->sockout, http_content_type_binary, strlen(http_content_type_binary), 0);
    }
  else if (strncmp(http_html, ptr, 5) == 0 || strncmp(http_shtml, ptr, 6) == 0)
    {
      ret = send(pstate->sockout, http_content_type_html, strlen(http_content_type_html), 0);
    }
  else if (strncmp(http_css, ptr, 4) == 0)
    {
      ret = send(pstate->sockout, http_content_type_css, strlen(http_content_type_css), 0);
    }
  else if (strncmp(http_png, ptr, 4) == 0)
    {
      ret = send(pstate->sockout, http_content_type_png, strlen(http_content_type_png), 0);
    }
  else if (strncmp(http_gif, ptr, 4) == 0)
    {
      ret = send(pstate->sockout, http_content_type_gif, strlen(http_content_type_gif), 0);
    }
  else if (strncmp(http_jpg, ptr, 4) == 0)
    {
      ret = send(pstate->sockout, http_content_type_jpg, strlen(http_content_type_jpg), 0);
    }
  else
    {
      ret = send(pstate->sockout, http_content_type_plain, strlen(http_content_type_plain), 0);
    }
  return ret;
}

static void handle_output(struct httpd_state *pstate)
{
  char *ptr;

  if (!httpd_fs_open(pstate->filename, &pstate->file))
    {
      httpd_fs_open(http_404_html, &pstate->file);
      strcpy(pstate->filename, http_404_html);
      send_headers(pstate, http_header_404);
      send_file(pstate);
    }
  else
    {
      send_headers(pstate, http_header_200);
      ptr = strchr(pstate->filename, ISO_period);
      if (ptr != NULL && strncmp(ptr, http_shtml, 6) == 0)
        {
          handle_script(pstate);
        }
      else
        {
        send_file(pstate);
        }
    }
}

static int handle_input(struct httpd_state *pstate)
{
  ssize_t recvlen;

  if (recv(pstate->sockin, pstate->inputbuf, HTTPD_INBUFFER_SIZE, 0) < 0)
    {
      return ERROR;
    }

  if (strncmp(pstate->inputbuf, http_get, 4) != 0)
    {
      return ERROR;
  }

  recvlen = recv(pstate->sockin, pstate->inputbuf, HTTPD_INBUFFER_SIZE, 0);
  if (recvlen < 0)
    {
      return ERROR;
    }

  if (pstate->inputbuf[0] != ISO_slash)
  {
    return ERROR;
  }

  if (pstate->inputbuf[1] == ISO_space)
    {
      strncpy(pstate->filename, http_index_html, sizeof(pstate->filename));
    }
  else
    {
      pstate->inputbuf[recvlen - 1] = 0;
      strncpy(pstate->filename, &pstate->inputbuf[0], sizeof(pstate->filename));
    }

  pstate->state = STATE_OUTPUT;

  while(1)
    {
      recvlen = recv(pstate->sockin, pstate->inputbuf, HTTPD_INBUFFER_SIZE, 0);
      if (recvlen < 0)
        {
          return ERROR;
        }

      if (strncmp(pstate->inputbuf, http_referer, 8) == 0)
        {
          pstate->inputbuf[recvlen - 2] = 0;
        }
    }

  return OK;
}

static void handle_connection(struct httpd_state *pstate)
{
  handle_input(pstate);
  if (pstate->state == STATE_OUTPUT) {
    handle_output(pstate);
  }
}

void httpd_listen(void)
{
#warning "this is all very broken at the moment"
}

/* Initialize the web server
 *
 * This function initializes the web server and should be
 * called at system boot-up.
 */

void httpd_init(void)
{
  uip_listen(HTONS(80));
}
