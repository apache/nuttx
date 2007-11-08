/* netutils/telnetd/telnetd.c
 *
 * Copyright (c) 2003, Adam Dunkels.
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
 */

#include <sys/types.h>
#include <stdlib.h>
#include <string.h>

#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>

#include "telnetd.h"
#include "shell.h"

#define ISO_nl       0x0a
#define ISO_cr       0x0d

#define STATE_NORMAL 0
#define STATE_IAC    1
#define STATE_WILL   2
#define STATE_WONT   3
#define STATE_DO     4
#define STATE_DONT   5
#define STATE_CLOSE  6

static struct telnetd_state s;

#define TELNET_IAC   255
#define TELNET_WILL  251
#define TELNET_WONT  252
#define TELNET_DO    253
#define TELNET_DONT  254

void shell_quit(char *str)
{
  s.state = STATE_CLOSE;
}

static void sendline(char *line)
{
  unsigned int i;

  for (i = 0; i < TELNETD_CONF_NUMLINES; ++i)
    {
      if (s.lines[i] == NULL)
        {
          s.lines[i] = line;
          break;
        }
    }
  if (i == TELNETD_CONF_NUMLINES)
    {
      free(line);
    }
}

void shell_prompt(char *str)
{
  char *line;
  line = (char*)malloc(TELNETD_CONF_LINELEN);
  if (line != NULL)
    {
      strncpy(line, str, TELNETD_CONF_LINELEN);
      sendline(line);
    }
}

void shell_output(char *str1, char *str2)
{
  unsigned len;
  char *line;

  line = (char*)malloc(TELNETD_CONF_LINELEN);
  if (line != NULL)
    {
      len = strlen(str1);
      strncpy(line, str1, TELNETD_CONF_LINELEN);
      if (len < TELNETD_CONF_LINELEN)
        {
          strncpy(line + len, str2, TELNETD_CONF_LINELEN - len);
        }
      len = strlen(line);
      if (len < TELNETD_CONF_LINELEN - 2)
        {
          line[len] = ISO_cr;
          line[len+1] = ISO_nl;
          line[len+2] = 0;
        }
      sendline(line);
    }
}

void telnetd_init(void)
{
  uip_listen(HTONS(23));
  shell_init();
}

static void acked(void)
{
  unsigned int i;

  while(s.numsent > 0)
    {
      free(s.lines[0]);
      for (i = 1; i < TELNETD_CONF_NUMLINES; ++i)
        {
          s.lines[i - 1] = s.lines[i];
        }
      s.lines[TELNETD_CONF_NUMLINES - 1] = NULL;
      --s.numsent;
    }
}

static void senddata(struct uip_driver_s *dev, struct uip_conn *conn)
{
  char *bufptr, *lineptr;
  int buflen, linelen;

  bufptr = (char*)dev->d_appdata;
  buflen = 0;
  for (s.numsent = 0;
        s.numsent < TELNETD_CONF_NUMLINES && s.lines[s.numsent] != NULL;
        ++s.numsent)
    {
      lineptr = s.lines[s.numsent];
      linelen = strlen(lineptr);
      if (linelen > TELNETD_CONF_LINELEN)
        {
          linelen = TELNETD_CONF_LINELEN;
        }
      if (buflen + linelen < uip_mss(conn))
        {
          memcpy(bufptr, lineptr, linelen);
          bufptr += linelen;
          buflen += linelen;
        }
      else
        {
          break;
        }
    }
  uip_send(dev, dev->d_appdata, buflen);
}

static void closed(void)
{
  unsigned int i;

  for (i = 0; i < TELNETD_CONF_NUMLINES; ++i)
    {
      if (s.lines[i] != NULL)
        {
          free(s.lines[i]);
        }
    }
}

static void get_char(uint8 c)
{
  if (c == ISO_cr)
  {
    return;
  }

  s.buf[(int)s.bufptr] = c;
  if (s.buf[(int)s.bufptr] == ISO_nl || s.bufptr == sizeof(s.buf) - 1)
    {
      if (s.bufptr > 0)
        {
          s.buf[(int)s.bufptr] = 0;
        }
      shell_input(s.buf);
      s.bufptr = 0;
    }
  else
    {
      ++s.bufptr;
    }
}

static void sendopt(uint8 option, uint8 value)
{
  char *line;
  line = (char*)malloc(TELNETD_CONF_LINELEN);
  if (line != NULL)
    {
      line[0] = TELNET_IAC;
      line[1] = option;
      line[2] = value;
      line[3] = 0;
      sendline(line);
    }
}

static void newdata(struct uip_driver_s *dev)
{
  uint16 len;
  uint8 c;
  char *dataptr;

  len = uip_datalen(dev);
  dataptr = (char *)dev->d_appdata;

  while(len > 0 && s.bufptr < sizeof(s.buf))\
    {
      c = *dataptr;
      ++dataptr;
      --len;
      switch(s.state)
        {
          case STATE_IAC:
            if (c == TELNET_IAC)
              {
                get_char(c);
                s.state = STATE_NORMAL;
             }
            else
              {
                switch (c)
                  {
                    case TELNET_WILL:
                      s.state = STATE_WILL;
                      break;
                    case TELNET_WONT:
                      s.state = STATE_WONT;
                      break;
                    case TELNET_DO:
                      s.state = STATE_DO;
                      break;
                    case TELNET_DONT:
                      s.state = STATE_DONT;
                      break;
                    default:
                      s.state = STATE_NORMAL;
                      break;
                  }
              }
            break;
          case STATE_WILL:
            /* Reply with a DONT */
            sendopt(TELNET_DONT, c);
            s.state = STATE_NORMAL;
            break;

          case STATE_WONT:
            /* Reply with a DONT */
            sendopt(TELNET_DONT, c);
            s.state = STATE_NORMAL;
            break;
          case STATE_DO:
            /* Reply with a WONT */
            sendopt(TELNET_WONT, c);
            s.state = STATE_NORMAL;
            break;
          case STATE_DONT:
            /* Reply with a WONT */
            sendopt(TELNET_WONT, c);
            s.state = STATE_NORMAL;
            break;
          case STATE_NORMAL:
            if (c == TELNET_IAC)
              {
                s.state = STATE_IAC;
              }
            else
              {
                get_char(c);
              }
            break;
        }
    }
}

/* This function is called by the UIP interrupt handling logic whenevent an
 * event of interest occurs.
 */

uint8 uip_interrupt_event(struct uip_driver_s *dev, struct uip_conn *conn, uint8 flags)
{
#warning OBSOLETE -- needs to be redesigned
  unsigned int i;

  if (uip_connected_event(flags))
    {
      for (i = 0; i < TELNETD_CONF_NUMLINES; ++i)
      {
        s.lines[i] = NULL;
      }
      s.bufptr = 0;
      s.state = STATE_NORMAL;

      shell_start();
  }

  if (s.state == STATE_CLOSE)
  {
    s.state = STATE_NORMAL;
    return UIP_CLOSE;
  }

  if (uip_close_event(flags) || uip_abort_event(flags) || uip_timeout_event(flags))
  {
    closed();
  }

  if (uip_ack_event(flags))
  {
    acked();
  }

  if (uip_newdata_event(flags))
  {
    newdata(dev);
  }

  if (uip_rexmit_event(flags) || uip_newdata_event(flags) || uip_ack_event(flags) ||
      uip_connected_event(flags) || uip_poll_event(flags))
  {
    senddata(dev, conn);
  }
  return 0;
}
