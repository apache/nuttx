/****************************************************************************
 * examples/nettest/nettestn-client.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "nettest.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void send_client(void)
{
  struct sockaddr_in myaddr;
  char outbuf[SENDSIZE];
#ifndef CONFIG_NETTEST_PERFORMANCE
  char inbuf[SENDSIZE];
#endif
  int sockfd;
  int nbytessent;
#ifndef CONFIG_NETTEST_PERFORMANCE
  int nbytesrecvd;
#endif
  int ch;
  int i;

  /* Create a new TCP socket */

  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    {
      message("client socket failure %d\n", errno);
      exit(1);
    }

  /* Connect the socket to the server */

  myaddr.sin_family      = AF_INET;
  myaddr.sin_port        = HTONS(PORTNO);
#if 0
  myaddr.sin_addr.s_addr = HTONL(INADDR_LOOPBACK);
#else
  myaddr.sin_addr.s_addr = HTONL(CONFIG_EXAMPLE_NETTEST_CLIENTIP);
#endif

  message("client: Connecting...\n");
  if (connect( sockfd, (struct sockaddr*)&myaddr, sizeof(struct sockaddr_in)) < 0)
    {
      message("client: connect failure: %d\n", errno);
      exit(1);
    }
  message("client: Connected\n");

  /* Initialize the buffer */

  ch = 0x20;
  for (i = 0; i < SENDSIZE; i++ )
    {
      outbuf[i] = ch;
      if (++ch > 0x7e)
        {
          ch = 0x20;
        }
    }

#ifdef CONFIG_NETTEST_PERFORMANCE
  /* Then receive messages forever */

  for (;;)
    {
      nbytessent = send(sockfd, outbuf, 512, 0);
      if (nbytessent < 0)
        {
          message("client: send failed: %d\n", errno);
          close(sockfd);
          exit(-1);
        }
      else if (nbytessent != 512)
        {
          message("client: Bad send length=%d: %d\n", nbytessent);
          close(sockfd);
          exit(-1);
        }
    }
#else
  /* Then send and receive one message */

  message("client: Sending %d bytes\n", SENDSIZE);
  nbytessent = send(sockfd, outbuf, SENDSIZE, 0);
  message("client: Sent %d bytes\n", nbytessent);

  if (nbytessent < 0)
    {
      message("client: send failed: %d\n", errno);
      close(sockfd);
      exit(-1);
    }
  else if (nbytessent != SENDSIZE)
    {
      message("client: Bad send length=%d: %d\n", nbytessent);
      close(sockfd);
      exit(-1);
    }

  message("client: Receiving...\n");
  nbytesrecvd = recv(sockfd, inbuf, SENDSIZE, 0);
  message("client: Received %d bytes\n", nbytesrecvd);

  if (nbytesrecvd < 0)
    {
      message("client: recv failed: %d\n", errno);
      close(sockfd);
      exit(-1);
    }
  else if (nbytesrecvd != SENDSIZE)
    {
      message("client: Bad recv length=%d: %d\n", nbytesrecvd);
      close(sockfd);
      exit(-1);
    }
  else if (memcmp(inbuf, outbuf, SENDSIZE) != 0)
    {
      message("client: Received buffer does not match sent buffer\n");
      close(sockfd);
      exit(-1);
    }

  close(sockfd);
#endif
}
