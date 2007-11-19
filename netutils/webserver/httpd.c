/****************************************************************************
 * httpd
 * httpd Web server
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * This is a leverage of similar logic from uIP:
 *
 *   Author: Adam Dunkels <adam@sics.se>
 *   Copyright (c) 2004, Adam Dunkels.
 *   All rights reserved.
 *
 *   The uIP web server is a very simplistic implementation of an HTTP
 *   server. It can serve web pages and files from a read-only ROM
 *   filesystem, and provides a very small scripting language.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/socket.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <debug.h>

#include <net/uip/uip.h>
#include <net/uip/uip-lib.h>
#include <net/uip/httpd.h>

#include "httpd.h"
#include "httpd-cgi.h"
#include "netutil-strings.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define ISO_nl      0x0a
#define ISO_space   0x20
#define ISO_bang    0x21
#define ISO_percent 0x25
#define ISO_period  0x2e
#define ISO_slash   0x2f
#define ISO_colon   0x3a

#define CONFIG_NETUTILS_HTTPD_DUMPBUFFER 1

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_HTTPD_DUMPBUFFER
static void httpd_dumpbuffer(struct httpd_state *pstate, ssize_t nbytes)
{
#ifdef CONFIG_DEBUG
  char line[128];
  int ch;
  int i;
  int j;

  for (i = 0; i < nbytes; i += 16)
    {
      sprintf(line, "%04x: ", i);
      for ( j = 0; j < 16; j++)
        {
          if (i + j < nbytes)
            {
              sprintf(&line[strlen(line)], "%02x ", pstate->ht_buffer[i+j] );
            }
          else
            {
              strcpy(&line[strlen(line)], "   ");
            }
        }
      for ( j = 0; j < 16; j++)
        {
          if (i + j < nbytes)
            {
              ch = pstate->ht_buffer[i+j];
              sprintf(&line[strlen(line)], "%c", ch >= 0x20 && ch <= 0x7e ? ch : '.');
            }
        }
      dbg("%s", line);
    }
#endif
}
#else
# define httpd_dumpbuffer(pstate,nbytes)
#endif

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

    /* Check if we should start executing a script */

    if (*pstate->file.data == ISO_percent &&
       *(pstate->file.data + 1) == ISO_bang) {
      pstate->scriptptr = pstate->file.data + 3;
      pstate->scriptlen = pstate->file.len - 3;
      if (*(pstate->scriptptr - 1) == ISO_colon)
        {
          httpd_fs_open(pstate->scriptptr + 1, &pstate->file);
          send(pstate->sockfd, pstate->file.data, pstate->file.len, 0);
        }
      else
        {
          httpd_cgi(pstate->scriptptr)(pstate, pstate->scriptptr);
        }
      next_scriptstate(pstate);

      /* The script is over, so we reset the pointers and continue
	 sending the rest of the file */
      pstate->file.data = pstate->scriptptr;
      pstate->file.len = pstate->scriptlen;

    } else {
      /* See if we find the start of script marker in the block of HTML
	 to be sent */

      if (pstate->file.len > HTTPD_IOBUFFER_SIZE) {
	pstate->len = HTTPD_IOBUFFER_SIZE;
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
	if (pstate->len >= HTTPD_IOBUFFER_SIZE) {
	  pstate->len = HTTPD_IOBUFFER_SIZE;
	}
      }
      send(pstate->sockfd, pstate->file.data, pstate->len, 0);
      pstate->file.data += pstate->len;
      pstate->file.len -= pstate->len;
    }
  }
}

static int send_headers(struct httpd_state *pstate, const char *statushdr)
{
  char *ptr;
  int ret;

  ret = send(pstate->sockfd, statushdr, strlen(statushdr), 0);

  ptr = strrchr(pstate->filename, ISO_period);
  if (ptr == NULL)
    {
      ret = send(pstate->sockfd, http_content_type_binary, strlen(http_content_type_binary), 0);
    }
  else if (strncmp(http_html, ptr, 5) == 0 || strncmp(http_shtml, ptr, 6) == 0)
    {
      ret = send(pstate->sockfd, http_content_type_html, strlen(http_content_type_html), 0);
    }
  else if (strncmp(http_css, ptr, 4) == 0)
    {
      ret = send(pstate->sockfd, http_content_type_css, strlen(http_content_type_css), 0);
    }
  else if (strncmp(http_png, ptr, 4) == 0)
    {
      ret = send(pstate->sockfd, http_content_type_png, strlen(http_content_type_png), 0);
    }
  else if (strncmp(http_gif, ptr, 4) == 0)
    {
      ret = send(pstate->sockfd, http_content_type_gif, strlen(http_content_type_gif), 0);
    }
  else if (strncmp(http_jpg, ptr, 4) == 0)
    {
      ret = send(pstate->sockfd, http_content_type_jpg, strlen(http_content_type_jpg), 0);
    }
  else
    {
      ret = send(pstate->sockfd, http_content_type_plain, strlen(http_content_type_plain), 0);
    }
  return ret;
}

static void httpd_sendfile(struct httpd_state *pstate)
{
  char *ptr;

  if (!httpd_fs_open(pstate->filename, &pstate->file))
    {
      httpd_fs_open(http_404_html, &pstate->file);
      strcpy(pstate->filename, http_404_html);
      send_headers(pstate, http_header_404);
      send(pstate->sockfd, pstate->file.data, pstate->file.len, 0);
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
          send(pstate->sockfd, pstate->file.data, pstate->file.len, 0);
        }
    }
}

static inline int httpd_cmd(struct httpd_state *pstate)
{
  ssize_t recvlen;
  int i;

  /* Get the next HTTP command.  We will handle only GET */

  recvlen = recv(pstate->sockfd, pstate->ht_buffer, HTTPD_IOBUFFER_SIZE, 0);
  if (recvlen < 0)
    {
      return ERROR;
    }
  httpd_dumpbuffer(pstate, recvlen);

  /*  We will handle only GET */

  if (strncmp(pstate->ht_buffer, http_get, 4) != 0)
    {
      return ERROR;
    }

  /* Get the name of the file to provide */

  if (pstate->ht_buffer[4] != ISO_slash)
    {
      return ERROR;
    }
  else if (pstate->ht_buffer[5] == ISO_space)
    {
      strncpy(pstate->filename, http_index_html, sizeof(pstate->filename));
    }
  else
    {
      for (i = 5; i < sizeof(pstate->filename) + 5 && pstate->ht_buffer[5] != ISO_space; i++)
        {
          pstate->filename[i] = pstate->ht_buffer[i+5];
        }
    }

  /* Then send the file */

  httpd_sendfile(pstate);
  return OK;
}

/****************************************************************************
 * Name: httpd_handler
 *
 * Description:
 *   Each time a new connection to port 80 is made, a new thread is created
 *   that begins at this entry point.  There should be exactly one argument
 *   and it should be the socket descriptor (+1).
 *
 ****************************************************************************/

static int httpd_handler(int argc, char *argv[])
{
  struct httpd_state *pstate = (struct httpd_state *)malloc(sizeof(struct httpd_state));
  int                 sockfd = (int)argv[1] - 1;
  int                 ret    = ERROR;

  /* Verify that the state structure was successfully allocated */

  if (pstate)
    {
      /* Loop processing each HTTP command */
      do
        {
          /* Re-initialize the thread state structure */

          memset(pstate, 0, sizeof(struct httpd_state));
          pstate->sockfd = sockfd;

          /* Then handle the next httpd command */

          ret = httpd_cmd(pstate);
        }
      while (ret == OK);

    /* End of command processing -- Clean up and exit */

    free(pstate);
  }

  /* Exit the task */

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: httpd_listen
 *
 * Description:
 *   This is the main processing thread for the webserver.  It never returns
 *   unless an error occurs
 *
 ****************************************************************************/

int httpd_listen(void)
{
  /* Execute httpd_handler on each connection to port 80 */

  uip_server(HTONS(80), httpd_handler, CONFIG_EXAMPLES_UIP_HTTPDSTACKSIZE);

  /* uip_server only returns on errors */

  return ERROR;
}

/****************************************************************************
 * Name: httpd_init
 *
 * Description:
 *   This function initializes the web server and should be called at system
 *   boot-up.
 *
 ****************************************************************************/

void httpd_init(void)
{
}
