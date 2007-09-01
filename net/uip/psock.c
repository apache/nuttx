/*
 * Copyright (c) 2004, Swedish Institute of Computer Science.
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
 *
 * This file is part of the uIP TCP/IP stack
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 * $Id: psock.c,v 1.2 2007-09-01 18:06:13 patacongo Exp $
 */

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <debug.h>

#include <net/uip/uip.h>
#include <net/uip/uipopt.h>
#include <net/uip/psock.h>

#define STATE_NONE 0
#define STATE_ACKED 1
#define STATE_READ 2
#define STATE_BLOCKED_NEWDATA 3
#define STATE_BLOCKED_CLOSE 4
#define STATE_BLOCKED_SEND 5
#define STATE_DATA_SENT 6

/*
 * Return value of the buffering functions that indicates that a
 * buffer was not filled by incoming data.
 *
 */
#define BUF_NOT_FULL 0
#define BUF_NOT_FOUND 0

/*
 * Return value of the buffering functions that indicates that a
 * buffer was completely filled by incoming data.
 *
 */
#define BUF_FULL 1

/*
 * Return value of the buffering functions that indicates that an
 * end-marker byte was found.
 *
 */
#define BUF_FOUND 2

static void buf_setup(struct psock_buf *buf, uint8 *bufptr, uint16 bufsize)
{
  buf->ptr = bufptr;
  buf->left = bufsize;
}

static uint8 buf_bufdata(struct psock_buf *buf, uint16 len, uint8 **dataptr, uint16 *datalen)
{
  if (*datalen < buf->left)
    {
      memcpy(buf->ptr, *dataptr, *datalen);
      buf->ptr += *datalen;
      buf->left -= *datalen;
      *dataptr += *datalen;
      *datalen = 0;
      return BUF_NOT_FULL;
    }
  else if (*datalen == buf->left)
    {
      memcpy(buf->ptr, *dataptr, *datalen);
      buf->ptr += *datalen;
      buf->left = 0;
      *dataptr += *datalen;
      *datalen = 0;
      return BUF_FULL;
    }
  else
    {
      memcpy(buf->ptr, *dataptr, buf->left);
      buf->ptr += buf->left;
      *datalen -= buf->left;
      *dataptr += buf->left;
      buf->left = 0;
      return BUF_FULL;
    }
}

static uint8 buf_bufto(struct psock_buf *buf, uint8 endmarker, uint8 **dataptr, uint16 *datalen)
{
  uint8 c;
  while(buf->left > 0 && *datalen > 0)
    {
      c = *buf->ptr = **dataptr;
      ++*dataptr;
      ++buf->ptr;
      --*datalen;
      --buf->left;

      if (c == endmarker)
        {
          return BUF_FOUND;
        }
    }

  if (*datalen == 0)
    {
      return BUF_NOT_FOUND;
    }

  while(*datalen > 0)
    {
      c = **dataptr;
      --*datalen;
      ++*dataptr;

      if (c == endmarker)
        {
          return BUF_FOUND | BUF_FULL;
        }
    }

  return BUF_FULL;
}

static boolean send_data(register struct psock *s)
{
  /* Inidicate that we are blocked waiting for the send to complete */

  s->state = STATE_BLOCKED_SEND;

  /* Loop until we successfully send the data */

  for (;;)
    {
      /* If the data has not been sent OR if it needs to be retransmitted,
       * then send it now.
       */

      if (s->state != STATE_DATA_SENT || uip_rexmit())
        {
          if (s->sendlen > uip_mss())
            {
              uip_send(s->sendptr, uip_mss());
            }
          else
            {
              uip_send(s->sendptr, s->sendlen);
            }

          s->state = STATE_DATA_SENT;
        }

    /* Check if all data has been sent and acknowledged */

    if (s->state == STATE_DATA_SENT && uip_acked())
      {
        /* Yes.. the data has been sent AND acknowledge */

        if (s->sendlen > uip_mss())
          {
            s->sendlen -= uip_mss();
            s->sendptr += uip_mss();
          }
        else
          {
            s->sendptr += s->sendlen;
            s->sendlen = 0;
          }

        s->state = STATE_ACKED;
        return TRUE;
      }

    /* No.. then wait on the retransmit or acked events */

    (void)uip_event_wait(UIP_ACKDATA|UIP_REXMIT);
  }

  return FALSE; /* We never get here */
}

void psock_send(struct psock *s, const char *buf, unsigned int len)
{
  /* If there is no data to send, we exit immediately. */

  if (len > 0)
    {
      /* Save the length of and a pointer to the data that is to be sent. */

      s->sendptr = (const uint8*)buf;
      s->sendlen = len;
      s->state   = STATE_NONE;

      /* Loop here until all data is sent. The s->sendlen variable is updated
       * by the data_sent() function.
       */

      while(s->sendlen > 0) {

        /* Wait until the data has been sent and acknowledged */

        send_data(s);
      }

      /* Done */

      s->state = STATE_NONE;
    }
}

void psock_generator_send(register struct psock *s, unsigned short (*generate)(void *), void *arg)
{
  /* Ensure that there is a generator function to call. */

  if (generate != NULL)
    {
      /* Call the generator function to generate the data in the uip_appdata
       * buffer.
       */

      s->sendlen = generate(arg);
      s->sendptr = uip_appdata;
      s->state   = STATE_NONE;

      do
        {
          /* Call the generator function again if we are called to perform a
          * retransmission.
           */

          if (uip_rexmit())
            {
              generate(arg);
            }

          /* Wait until all data is sent and acknowledged. */

          send_data(s);
        }
      while(s->sendlen > 0);

      /* Done */

      s->state = STATE_NONE;
    }
}

uint16 psock_datalen(struct psock *psock)
{
  return psock->bufsize - psock->buf.left;
}

boolean psock_checknewdata(struct psock *s)
{
  if (s->readlen > 0)
    {
      /* There is data in the uip_appdata buffer that has not yet been read
       * with the PSOCK_READ functions.
       */
      return TRUE;
    }
  else if (s->state == STATE_READ)
    {
      /* All data in uip_appdata buffer already consumed. */

      s->state = STATE_BLOCKED_NEWDATA;
      return FALSE;
    }
  else if (uip_newdata())
    {
      /* There is new data that has not been consumed. */

      return TRUE;
    }
  else
    {
      /* There is no new data. */

      return FALSE;
    }
}

void psock_waitnewdata(struct psock *s)
{
  while (!psock_checknewdata(s))
    {
      uip_event_wait(UIP_NEWDATA);
    }
}

void psock_readto(register struct psock *psock, unsigned char c)
{
restart:
  buf_setup(&psock->buf, psock->bufptr, psock->bufsize);

  /* XXX: Should add buf_checkmarker() before do{} loop, if
    incoming data has been handled while waiting for a write. */

  do
    {
      if (psock->readlen == 0)
        {
          psock_waitnewdata(psock);
          psock->state   = STATE_READ;
          psock->readptr = (uint8 *)uip_appdata;
          psock->readlen = uip_datalen();
        }
    }
  while((buf_bufto(&psock->buf, c, &psock->readptr, &psock->readlen) & BUF_FOUND) == 0);

  if (psock_datalen(psock) == 0)
    {
      psock->state = STATE_NONE;
      goto restart;
    }
}

void psock_readbuf(register struct psock *psock)
{
restart:
  buf_setup(&psock->buf, psock->bufptr, psock->bufsize);

  /* XXX: Should add buf_checkmarker() before do{} loop, if
     incoming data has been handled while waiting for a write. */

  do
    {
      if (psock->readlen == 0)
        {
          psock_waitnewdata(psock);
          dbg("Waited for newdata\n");
          psock->state   = STATE_READ;
          psock->readptr = (uint8 *)uip_appdata;
          psock->readlen = uip_datalen();
        }
    }
  while(buf_bufdata(&psock->buf, psock->bufsize, &psock->readptr, &psock->readlen) != BUF_FULL);

  if (psock_datalen(psock) == 0)
    {
      psock->state = STATE_NONE;
      goto restart;
    }
}

void psock_init(register struct psock *psock, char *buffer, unsigned int buffersize)
{
  psock->state   = STATE_NONE;
  psock->readlen = 0;
  psock->bufptr  = (uint8*)buffer;
  psock->bufsize = buffersize;
  buf_setup(&psock->buf, (uint8*)buffer, buffersize);
}

