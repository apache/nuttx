/****************************************************************************
 * netutils/uiplib/uip-server.c
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

#include <unistd.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <net/uip/uip-lib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#define errno *get_errno_ptr()

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_server
 *
 * Description:
 *   Implement basic server logic
 *
 * Parameters:
 *   portno    The port to listen on (in network byte order)
 *   handler   The entrypoint of the task to spawn when a new connection is
 *             accepted.
 *   stacksize The stack size needed by the spawned task
 *
 * Return:
 *   Does not return unless an error occurs.
 *
 ****************************************************************************/

void uip_server(uint16 portno, main_t handler, int stacksize)
{
  struct sockaddr_in myaddr;
#ifdef CONFIG_NET_HAVE_SOLINGER
  struct linger ling;
#endif
  struct sched_param param;
  socklen_t addrlen;
  const char *argv[2];
  int listensd;
  int acceptsd;
#ifdef CONFIG_NET_HAVE_REUSEADDR
  int optval;
#endif

  /* Create a new TCP socket to use to listen for connections */

  listensd = socket(PF_INET, SOCK_STREAM, 0);
  if (listensd < 0)
    {
      dbg("socket failure: %d\n", errno);
      return;
    }

  /* Set socket to reuse address */

#ifdef CONFIG_NET_HAVE_REUSEADDR
  optval = 1;
  if (setsockopt(listensd, SOL_SOCKET, SO_REUSEADDR, (void*)&optval, sizeof(int)) < 0)
    {
      dbg("setsockopt SO_REUSEADDR failure: %d\n", errno);
      goto errout_with_socket;
    }
#endif

  /* Bind the socket to a local address */

  myaddr.sin_family      = AF_INET;
  myaddr.sin_port        = portno;
  myaddr.sin_addr.s_addr = INADDR_ANY;

  if (bind(listensd, (struct sockaddr*)&myaddr, sizeof(struct sockaddr_in)) < 0)
    {
      dbg("bind failure: %d\n", errno);
      goto errout_with_socket;
    }

  /* Listen for connections on the bound TCP socket */

  if (listen(listensd, 5) < 0)
    {
      dbg("listen failure %d\n", errno);
      goto errout_with_socket;
    }

  /* Begin accepting connections */

  dbg("Accepting connections on port %d\n", ntohs(portno));
  for (;;)
    {
      addrlen = sizeof(struct sockaddr_in);
      acceptsd = accept(listensd, (struct sockaddr*)&myaddr, &addrlen);
      if (acceptsd < 0)
        {
          dbg("accept failure: %d\n", errno);
          break;;
        }
      dbg("Connection accepted -- spawning\n");

      /* Configure to "linger" until all data is sent when the socket is closed */

#ifdef CONFIG_NET_HAVE_SOLINGER
      ling.l_onoff  = 1;
      ling.l_linger = 30;     /* timeout is seconds */
      if (setsockopt(acceptsd, SOL_SOCKET, SO_LINGER, &ling, sizeof(struct linger)) < 0)
        {
          close(acceptsd);
          dbg("setsockopt SO_LINGER failure: %d\n", errno);
          break;;
        }
#endif

      /* Spawn a thread to handle the connection.  The socket descriptor +1 is
       * provided in as the single argument to the new thread. (The +1 is intended
       * to handle the valid, zero file descriptor).
       */

      if (sched_getparam(0, &param) < 0)
        {
          close(acceptsd);
          dbg("sched_getparam failed: %d\n", errno);
          break;;
        }

      argv[0] = (char*)(acceptsd + 1);
      argv[1] = NULL;

      if (task_create("", param.sched_priority, stacksize, handler, argv) < 0)
        {
          close(acceptsd);
          dbg("task_create failed: %d\n", errno);
          break;;
        }

      /* We can close our copy of acceptsd now.  This file descriptor was dup'ed
       * by task_create and we no longer need to retain the reference.
       */

      close(acceptsd);
    }

errout_with_socket:
  close(listensd);
}
