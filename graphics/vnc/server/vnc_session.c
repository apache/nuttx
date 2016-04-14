/****************************************************************************
 * graphics/vnc/vnc_session.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>

#include "vnc_server.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_initialize_session
 *
 * Description:
 *  [Re-]initialize a VNC session
 *
 * Input Parameters:
 *   session - the VNC session to be initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void vnc_initialize_session(FAR struct vnc_session_s *session)
{
  /* Initialize the session.  Set all values to 0 == NULL == false. */

  memset(session, 0, sizeof(struct vnc_session_s));

  /* Then initialize only non-zero values */
  /* Initialized, not connected */

  session->state = VNCSERVER_INITIALIZED;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_connect
 *
 * Description:
 *  Wait for a connection from the VNC client
 *
 * Input Parameters:
 *   session - An instance of the session structure allocated by
 *     vnc_create_session().
 *   port    - The listen port to use
 *
 * Returned Value:
 *   Returns zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int vnc_connect(FAR struct vnc_session_s *session, int port)
{
  struct sockaddr_in addr;
  int ret;

  /* Create a listening socket */

  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = INADDR_ANY;

  ret = psock_socket(AF_INET, SOCK_STREAM, 0, &session->listen);
  if (ret < 0)
    {
      ret = -get_errno();
      return ret;
    }

  /* Bind the listening socket to a local address */

  ret = psock_bind(&session->listen, (struct sockaddr *)&addr,
                   sizeof(struct sockaddr_in));
  if (ret < 0)
    {
      ret = -get_errno();
      goto errout_with_listener;
    }

  /* Listen for a connection */

  ret = psock_listen(&session->listen, 5);
  if (ret < 0)
    {
      ret = -get_errno();
      goto errout_with_listener;
    }

  /* Connect to the client */

  ret = psock_accept(&session->listen, NULL, NULL, &session->connect);
  if (ret < 0)
    {
      ret = -get_errno();
      goto errout_with_listener;
    }

  session->state = VNCSERVER_CONNECTED;
  return OK;

errout_with_listener:
  psock_close(&session->listen);
  return ret;
}

/****************************************************************************
 * Name: vnc_create_session
 *
 * Description:
 *  Create a new, unconnected session
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, this function returns the allocated and initialize session
 *   structure.  NULL is returned on failure.
 *
 ****************************************************************************/

FAR struct vnc_session_s *vnc_create_session(void)
{
  FAR struct vnc_session_s *session;

  /* Allocate the session */

  session = (FAR struct vnc_session_s *)
    kmm_zalloc(sizeof(struct vnc_session_s));

  /* Initialize the session */

  if (session != NULL)
    {
      vnc_initialize_session(session);
    }

  return session;
}

/****************************************************************************
 * Name: vnc_release_session
 *
 * Description:
 *  Conclude the current VNC session and free most resources.  This function
 *  re-initializes the session structure; it does not free it so that it
 *  can be re-used.
 *
 * Input Parameters:
 *   session - An instance of the session structure allocated by
 *     vnc_create_session().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void vnc_release_session(FAR struct vnc_session_s *session)
{
  /* Close any open sockets */

  if (session->state >= VNCSERVER_CONNECTED)
    {
      psock_close(&session->connect);
      psock_close(&session->listen);
    }

  /* Free the allocated framebuffer */

  if (session->fb)
    {
      kmm_free(session->fb);
    }

  vnc_initialize_session(session);
}
