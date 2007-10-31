/****************************************************************************
 * examples/nettest/nettest.c
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
#include <errno.h>

#include "nettest.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void recv_server(void)
{
  struct sockaddr_in myaddr;
  char buffer[1024];
  int listensd;
  int acceptsd;
  socklen_t addrlen;
  int nbytesread;
#ifndef CONFIG_NETTEST_PERFORMANCE
  int nbytessent;
#endif
  int optval;

  /* Create a new TCP socket */

  listensd = socket(PF_INET, SOCK_STREAM, 0);
  if (listensd < 0)
    {
      printf("server: socket failure: %d\n", errno);
      exit(1);
    }

  /* Set socket to reuse address */

  optval = 1;
  if (setsockopt(listensd, SOL_SOCKET, SO_REUSEADDR, (void*)&optval, sizeof(int)) < 0)
    {
      printf("server: setsockopt failure: %d\n", errno);
      exit(1);
    }

  /* Bind the TCP socket to a local address */

  myaddr.sin_family      = AF_INET;
  myaddr.sin_port        = HTONS(PORTNO);
  myaddr.sin_addr.s_addr = INADDR_ANY;

  if (bind(listensd, (struct sockaddr*)&myaddr, sizeof(struct sockaddr_in)) < 0)
    {
      printf("server: bind failure: %d\n", errno);
      exit(1);
    }

  /* Listen for connections on the bound TCP socket */

  if (listen(listensd, 5) < 0)
    {
      printf("server: listen failure %d\n", errno);
      exit(1);
    }

  /* Accept only one connection */

  printf("server: Accepting connections on port %d\n", PORTNO);
  addrlen = sizeof(struct sockaddr_in);
  acceptsd = accept(listensd, (struct sockaddr*)&myaddr, &addrlen);
  if (acceptsd < 0)
    {
      printf("server: accept failure: %d\n", errno);
      exit(1);
    }
  printf("server: Connection accepted -- receiving\n");

#ifdef CONFIG_NETTEST_PERFORMANCE
  /* Then receive data forever */

  for (;;)
    {
      nbytesread = recv(acceptsd, buffer, 1024, 0);
      if (nbytesread <= 0)
        {
          printf("server: recv failed: %d\n", errno);
          close(listensd);
          close(acceptsd);
          exit(-1);
        }
    }
#else
  /* Receive and echo own message */

  nbytesread = recv(acceptsd, buffer, 1024, 0);
  if (nbytesread <= 0)
    {
      printf("server: recv failed: %d\n", errno);
      close(listensd);
      close(acceptsd);
      exit(-1);
    }
  printf("server: Received %d bytes\n", nbytesread);

  nbytessent = send(acceptsd, buffer, nbytesread, 0);
  if (nbytessent <= 0)
    {
      printf("server: send failed: %d\n", errno);
      close(listensd);
      close(acceptsd);
      exit(-1);
    }
  printf("server: Sent %d bytes\n", nbytessent);

  close(listensd);
  close(acceptsd);
#endif
}
