/****************************************************************************
 * smtp.c
 * smtp SMTP E-mail sender
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Heavily leveraged from uIP 1.0 which also has a BSD-like license:
 *
 * The Simple Mail Transfer Protocol (SMTP) as defined by RFC821 is
 * the standard way of sending and transfering e-mail on the
 * Internet. This simple example implementation is intended as an
 * example of how to implement protocols in uIP, and is able to send
 * out e-mail but has not been extensively tested.
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2004, Adam Dunkels.
 *   All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <semaphore.h>
#include <sys/socket.h>

#include <net/uip/uip.h>
#include <net/uip/psock.h>
#include <net/uip/smtp.h>

#include "smtp-strings.h"

#define SMTP_INPUT_BUFFER_SIZE 512

#define ISO_nl 0x0a
#define ISO_cr 0x0d

#define ISO_period 0x2e

#define ISO_2  0x32
#define ISO_3  0x33
#define ISO_4  0x34
#define ISO_5  0x35

/* This structure represents the state of a single SMTP transaction */

struct smtp_state
{
  uint8   state;
  boolean connected;
  sem_t   sem;
  struct psock psock;
  uip_ipaddr_t smtpserver;
  char   *localhostname;
  char   *to;
  char   *cc;
  char   *from;
  char   *subject;
  char   *msg;
  int     msglen;
  int     sentlen;
  int     textlen;
  int     sendptr;
  int     result;
  char    buffer[SMTP_INPUT_BUFFER_SIZE];
};

static volatile struct smtp_state *gpsmtp = 0;

static void smtp_send_message(struct smtp_state *psmtp)
{
  psock_readto(&psmtp->psock, ISO_nl);

  if (strncmp(psmtp->buffer, smtp_220, 3) != 0)
    {
      PSOCK_CLOSE(&psmtp->psock);
      psmtp->result = 2;
      return;
    }

  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_helo);
  PSOCK_SEND_STR(&psmtp->psock, psmtp->localhostname);
  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_crnl);

  psock_readto(&psmtp->psock, ISO_nl);

  if (psmtp->buffer[0] != ISO_2)
    {
      PSOCK_CLOSE(&psmtp->psock);
      psmtp->result = 3;
      return;
    }

  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_mail_from);
  PSOCK_SEND_STR(&psmtp->psock, psmtp->from);
  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_crnl);

  psock_readto(&psmtp->psock, ISO_nl);

  if (psmtp->buffer[0] != ISO_2)
    {
      PSOCK_CLOSE(&psmtp->psock);
      psmtp->result = 3;
      return;
    }

  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_rcpt_to);
  PSOCK_SEND_STR(&psmtp->psock, psmtp->to);
  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_crnl);

  psock_readto(&psmtp->psock, ISO_nl);

  if (psmtp->buffer[0] != ISO_2)
    {
      PSOCK_CLOSE(&psmtp->psock);
      psmtp->result = 5;
      return;
    }

  if (psmtp->cc != 0)
    {
      PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_rcpt_to);
      PSOCK_SEND_STR(&psmtp->psock, psmtp->cc);
      PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_crnl);

      psock_readto(&psmtp->psock, ISO_nl);

      if (psmtp->buffer[0] != ISO_2)
        {
          PSOCK_CLOSE(&psmtp->psock);
          psmtp->result = 6;
          return;
        }
    }

  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_data);

  psock_readto(&psmtp->psock, ISO_nl);

  if (psmtp->buffer[0] != ISO_3)
    {
      PSOCK_CLOSE(&psmtp->psock);
      psmtp->result = 7;
      return;
    }

  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_to);
  PSOCK_SEND_STR(&psmtp->psock, psmtp->to);
  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_crnl);

  if (psmtp->cc != 0)
    {
      PSOCK_SEND_STR(&psmtp->psock, (char *)psmtp->cc);
      PSOCK_SEND_STR(&psmtp->psock, psmtp->cc);
      PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_crnl);
    }

  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_from);
  PSOCK_SEND_STR(&psmtp->psock, psmtp->from);
  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_crnl);

  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_subject);
  PSOCK_SEND_STR(&psmtp->psock, psmtp->subject);
  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_crnl);

  psock_send(&psmtp->psock, psmtp->msg, psmtp->msglen);

  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_crnlperiodcrnl);

  psock_readto(&psmtp->psock, ISO_nl);
  if (psmtp->buffer[0] != ISO_2)
    {
      PSOCK_CLOSE(&psmtp->psock);
      psmtp->result = 8;
      return;
    }

  PSOCK_SEND_STR(&psmtp->psock, (char *)smtp_quit);
  psmtp->result = 0;
}

/* This function is called by the UIP interrupt handling logic whenevent an
 * event of interest occurs.
 */

void uip_interrupt_event(void)
{
  if (gpsmtp)
    {
      if (uip_closed())
        {
          gpsmtp->connected = FALSE;
          return;
        }

      if (uip_aborted() || uip_timedout())
        {
          gpsmtp->connected = FALSE;
        }

      sem_post((sem_t*)&gpsmtp->sem);
    }
}

/* Specificy an SMTP server and hostname.
 *
 * This function is used to configure the SMTP module with an SMTP
 * server and the hostname of the host.
 *
 * lhostname The hostname of the uIP host.
 *
 * server A pointer to a 4-byte array representing the IP
 * address of the SMTP server to be configured.
 */

void smtp_configure(void *handle, char *lhostname, void *server)
{
  struct smtp_state *psmtp = (struct smtp_state *)handle;
  psmtp->localhostname = lhostname;
  uip_ipaddr_copy(psmtp->smtpserver, server);
}

/* Send an e-mail.
 *
 * to The e-mail address of the receiver of the e-mail.
 * cc The e-mail address of the CC: receivers of the e-mail.
 * from The e-mail address of the sender of the e-mail.
 * subject The subject of the e-mail.
 * msg The actual e-mail message.
 * msglen The length of the e-mail message.
 */

int smtp_send(void *handle, char *to, char *cc, char *from, char *subject, char *msg, int msglen)
{
  struct smtp_state *psmtp = (struct smtp_state *)handle;
  struct sockaddr_in server;
  int sockfd;

  /* Setup */

  psmtp->connected = TRUE;
  psmtp->to        = to;
  psmtp->cc        = cc;
  psmtp->from      = from;
  psmtp->subject   = subject;
  psmtp->msg       = msg;
  psmtp->msglen    = msglen;
  psmtp->result    = OK;

  /* Create a socket */

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    {
      return ERROR;
    }

  /* Make this instance globally visible (we will get interrupts as
   * soon as we connect
   */

  gpsmtp = psmtp;

  /* Connect to server.  First we have to set some fields in the
   * 'server' structure.  The system will assign me an arbitrary
   * local port that is not in use.
   */

  server.sin_family = AF_INET;
  memcpy(&server.sin_addr.s_addr, &psmtp->smtpserver, sizeof(in_addr_t));
  server.sin_port = HTONS(25);

  if (connect(sockfd, (struct sockaddr *)&server, sizeof(struct sockaddr_in)) < 0)
    {
      return ERROR;
  }

  /* Initialize the psock structure inside the smtp state structure */

  psock_init(&psmtp->psock, psmtp->buffer, SMTP_INPUT_BUFFER_SIZE);

  /* And wait for the the socket to be connected */

  sem_wait(&psmtp->sem);
  gpsmtp           = 0;

  /* Was an error reported by interrupt handler? */

  if (psmtp->result == OK )
    {
      /* No... Send the message */
      smtp_send_message(psmtp);
    }

  return psmtp->result;
}

void *smtp_open(void)
{
  /* Allocate the handle */

  struct smtp_state *psmtp = (struct smtp_state *)malloc(sizeof(struct smtp_state));
  if (psmtp)
    {
      /* Initialize the handle */

      memset(psmtp, 0, sizeof(struct smtp_state));
     (void)sem_init(&psmtp->sem, 0, 0);
    }
  return (void*)psmtp;
}

void smtp_close(void *handle)
{
  struct smtp_state *psmtp = (struct smtp_state *)handle;
  if (psmtp)
    {
      sem_destroy(&psmtp->sem);
      free(psmtp);
    }
}