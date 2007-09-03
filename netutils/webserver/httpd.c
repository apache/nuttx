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
#include <net/uip/uip.h>
#include <net/uip/httpd.h>

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

static unsigned short generate_part_of_file(void *state)
{
  struct httpd_state *s = (struct httpd_state *)state;

  if (s->file.len > uip_mss()) {
    s->len = uip_mss();
  } else {
    s->len = s->file.len;
  }
  memcpy(uip_appdata, s->file.data, s->len);

  return s->len;
}

static void send_file(struct httpd_state *s)
{
  do {
    psock_generator_send(&s->sout, generate_part_of_file, s);
    s->file.len -= s->len;
    s->file.data += s->len;
  } while(s->file.len > 0);
#warning REVISIT must not return until file sent
}

static void send_part_of_file(struct httpd_state *s)
{
  psock_send(&s->sout, s->file.data, s->len);
#warning REVISIT must not return until file sent
}

static void next_scriptstate(struct httpd_state *s)
{
  char *p;
  p = strchr(s->scriptptr, ISO_nl) + 1;
  s->scriptlen -= (unsigned short)(p - s->scriptptr);
  s->scriptptr = p;
}

static void handle_script(struct httpd_state *s)
{
  char *ptr;
  
  while(s->file.len > 0) {

    /* Check if we should start executing a script. */
    if (*s->file.data == ISO_percent &&
       *(s->file.data + 1) == ISO_bang) {
      s->scriptptr = s->file.data + 3;
      s->scriptlen = s->file.len - 3;
      if (*(s->scriptptr - 1) == ISO_colon) {
	httpd_fs_open(s->scriptptr + 1, &s->file);
	send_file(s);
      } else {
        httpd_cgi(s->scriptptr)(s, s->scriptptr);
      }
      next_scriptstate(s);

      /* The script is over, so we reset the pointers and continue
	 sending the rest of the file. */
      s->file.data = s->scriptptr;
      s->file.len = s->scriptlen;
    } else {
      /* See if we find the start of script marker in the block of HTML
	 to be sent. */

      if (s->file.len > uip_mss()) {
	s->len = uip_mss();
      } else {
	s->len = s->file.len;
      }

      if (*s->file.data == ISO_percent) {
	ptr = strchr(s->file.data + 1, ISO_percent);
      } else {
	ptr = strchr(s->file.data, ISO_percent);
      }
      if (ptr != NULL &&
	 ptr != s->file.data) {
	s->len = (int)(ptr - s->file.data);
	if (s->len >= uip_mss()) {
	  s->len = uip_mss();
	}
      }
      send_part_of_file(s);
      s->file.data += s->len;
      s->file.len -= s->len;
    }
  }
#warning REVISIT must not return until sent
}

static void send_headers(struct httpd_state *s, const char *statushdr)
{
  char *ptr;

  PSOCK_SEND_STR(&s->sout, statushdr);

  ptr = strrchr(s->filename, ISO_period);
  if (ptr == NULL) {
    PSOCK_SEND_STR(&s->sout, http_content_type_binary);
  } else if (strncmp(http_html, ptr, 5) == 0 ||
	    strncmp(http_shtml, ptr, 6) == 0) {
    PSOCK_SEND_STR(&s->sout, http_content_type_html);
  } else if (strncmp(http_css, ptr, 4) == 0) {
    PSOCK_SEND_STR(&s->sout, http_content_type_css);
  } else if (strncmp(http_png, ptr, 4) == 0) {
    PSOCK_SEND_STR(&s->sout, http_content_type_png);
  } else if (strncmp(http_gif, ptr, 4) == 0) {
    PSOCK_SEND_STR(&s->sout, http_content_type_gif);
  } else if (strncmp(http_jpg, ptr, 4) == 0) {
    PSOCK_SEND_STR(&s->sout, http_content_type_jpg);
  } else {
    PSOCK_SEND_STR(&s->sout, http_content_type_plain);
  }
#warning REVISIT must not return until sent
}

static void handle_output(struct httpd_state *s)
{
  char *ptr;

  if (!httpd_fs_open(s->filename, &s->file))
    {
      httpd_fs_open(http_404_html, &s->file);
      strcpy(s->filename, http_404_html);
      send_headers(s, http_header_404);
      send_file(s);
    }
  else
    {
      send_headers(s, http_header_200);
      ptr = strchr(s->filename, ISO_period);
      if (ptr != NULL && strncmp(ptr, http_shtml, 6) == 0)
        {
          handle_script(s);
        }
      else
        {
        send_file(s);
        }
    }
  PSOCK_CLOSE(&s->sout);
}

static void handle_input(struct httpd_state *s)
{
  psock_readto(&s->sin, ISO_space);

  if (strncmp(s->inputbuf, http_get, 4) != 0)
    {
      PSOCK_CLOSE(&s->sin);
      return;
  }

  psock_readto(&s->sin, ISO_space);

  if (s->inputbuf[0] != ISO_slash)
  {
    PSOCK_CLOSE(&s->sin);
    return;
  }

  if (s->inputbuf[1] == ISO_space)
    {
      strncpy(s->filename, http_index_html, sizeof(s->filename));
    }
  else
    {
      s->inputbuf[psock_datalen(&s->sin) - 1] = 0;
      strncpy(s->filename, &s->inputbuf[0], sizeof(s->filename));
    }

  s->state = STATE_OUTPUT;

  while(1)
    {
      psock_readto(&s->sin, ISO_nl);

      if (strncmp(s->inputbuf, http_referer, 8) == 0)
        {
          s->inputbuf[psock_datalen(&s->sin) - 2] = 0;
        }
    }
}

static void handle_connection(struct httpd_state *s)
{
  handle_input(s);
  if (s->state == STATE_OUTPUT) {
    handle_output(s);
  }
}

/* This function is called by the UIP interrupt handling logic whenevent an
 * event of interest occurs.
 */

void uip_interrupt_event(void)
{
#warning OBSOLETE -- needs to be redesigned
  /* Get the private application specific data */
  struct httpd_state *s = (struct httpd_state *)(uip_conn->private);

  /* Has application specific data been allocate yet? */

  if (!s)
    {
       /* No.. allocate it now */
       s = (struct httpd_state *)malloc(sizeof(struct httpd_state));
       if (!s)
         {
           return;
         }

       /* And assign the private instance to the connection */
       uip_conn->private = s;
    }

  if (uip_closed() || uip_aborted() || uip_timedout()) {
  } else if (uip_connected()) {
    psock_init(&s->sin, s->inputbuf, sizeof(s->inputbuf) - 1);
    psock_init(&s->sout, s->inputbuf, sizeof(s->inputbuf) - 1);
    s->state = STATE_WAITING;
    s->timer = 0;
    handle_connection(s);
  } else if (s != NULL) {
    if (uip_poll()) {
      ++s->timer;
      if (s->timer >= 20) {
	uip_abort();
      }
    } else {
      s->timer = 0;
    }
    handle_connection(s);
  } else {
    uip_abort();
  }
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
