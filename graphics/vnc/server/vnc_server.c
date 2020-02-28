/****************************************************************************
 * graphics/vnc/vnc_server.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
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

#include "nuttx/config.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <queue.h>
#include <assert.h>
#include <errno.h>

#if defined(CONFIG_VNCSERVER_DEBUG) && !defined(CONFIG_DEBUG_GRAPHICS)
#  undef  CONFIG_DEBUG_ERROR
#  undef  CONFIG_DEBUG_WARN
#  undef  CONFIG_DEBUG_INFO
#  undef  CONFIG_DEBUG_GRAPHICS_ERROR
#  undef  CONFIG_DEBUG_GRAPHICS_WARN
#  undef  CONFIG_DEBUG_GRAPHICS_INFO
#  define CONFIG_DEBUG_ERROR          1
#  define CONFIG_DEBUG_WARN           1
#  define CONFIG_DEBUG_INFO           1
#  define CONFIG_DEBUG_GRAPHICS       1
#  define CONFIG_DEBUG_GRAPHICS_ERROR 1
#  define CONFIG_DEBUG_GRAPHICS_WARN  1
#  define CONFIG_DEBUG_GRAPHICS_INFO  1
#endif
#include <debug.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>

#include "vnc_server.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Given a display number as an index, the following array can be used to
 * look-up the session structure for that display.
 */

FAR struct vnc_session_s *g_vnc_sessions[RFB_MAX_DISPLAYS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_reset_session
 *
 * Description:
 *  Conclude the current VNC session.  This function re-initializes the
 *  session structure; it does not free either the session structure nor
 *  the framebuffer so that they may be re-used.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void vnc_reset_session(FAR struct vnc_session_s *session,
                              FAR uint8_t *fb, int display)
{
  int i;

  /* Close any open sockets */

  if (session->state >= VNCSERVER_CONNECTED)
    {
      psock_close(&session->connect);
      psock_close(&session->listen);
    }

  /* [Re-]initialize the session. */

  memset(&session->connect, 0, sizeof(struct socket));
  memset(&session->listen, 0, sizeof(struct socket));

  /* Put all of the pre-allocated update structures into the freelist */

  sq_init(&session->updqueue);
  sq_init(&session->updfree);

  for (i = 0; i < CONFIG_VNCSERVER_NUPDATES; i++)
    {
      sq_addlast((FAR sq_entry_t *)&session->updpool[i], &session->updfree);
    }

  /* Set the INITIALIZED state */

  nxsem_reset(&session->freesem, CONFIG_VNCSERVER_NUPDATES);
  nxsem_reset(&session->queuesem, 0);

  session->fb      = fb;
  session->display = display;
  session->state   = VNCSERVER_INITIALIZED;
  session->nwhupd  = 0;
  session->change  = true;

  /* Careful not to disturb the keyboard/mouse callouts set by
   * vnc_fbinitialize().  Client related data left in garbage state.
   */
}

/****************************************************************************
 * Name: vnc_connect
 *
 * Description:
 *  Wait for a connection from the VNC client
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *   port    - The listen port to use
 *
 * Returned Value:
 *   Returns zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int vnc_connect(FAR struct vnc_session_s *session, int port)
{
  struct sockaddr_in addr;
  int ret;

  ginfo("Connecting display %d\n", session->display);

  /* Create a listening socket */

  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = INADDR_ANY;

  ret = psock_socket(AF_INET, SOCK_STREAM, 0, &session->listen);
  if (ret < 0)
    {
      return ret;
    }

  /* Bind the listening socket to a local address */

  ret = psock_bind(&session->listen, (struct sockaddr *)&addr,
                   sizeof(struct sockaddr_in));
  if (ret < 0)
    {
      goto errout_with_listener;
    }

  /* Listen for a connection */

  ret = psock_listen(&session->listen, 5);
  if (ret < 0)
    {
      goto errout_with_listener;
    }

  /* Connect to the client */

  ginfo("Accepting connection for Display %d\n", session->display);

  ret = psock_accept(&session->listen, NULL, NULL, &session->connect);
  if (ret < 0)
    {
      goto errout_with_listener;
    }

  ginfo("Display %d connected\n", session->display);
  session->state = VNCSERVER_CONNECTED;
  return OK;

errout_with_listener:
  psock_close(&session->listen);
  return ret;
}

/****************************************************************************
 * Pubic Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_server
 *
 * Description:
 *  The VNC server daemon.  This daemon is implemented as a kernel thread.
 *
 * Input Parameters:
 *   Standard kernel thread arguments (all ignored)
 *
 * Returned Value:
 *   This function does not return.
 *
 ****************************************************************************/

int vnc_server(int argc, FAR char *argv[])
{
  FAR struct vnc_session_s *session;
  FAR uint8_t *fb;
  int display;
  int ret;

  /* A single argument is expected:  A display port number in ASCII form */

  if (argc != 2)
    {
      /* In this case the start-up logic will probably hang, waiting for the
       * display-related semaphore to be set.
       */

      gerr("ERROR: Unexpected number of arguments: %d\n", argc);
      ret = -EINVAL;
      goto errout_with_hang;
    }

  display = atoi(argv[1]);
  if (display < 0 || display >= RFB_MAX_DISPLAYS)
    {
      /* In this case the start-up logic will probably hang, waiting for the
       * display-related semaphore to be set.
       */

      gerr("ERROR: Invalid display number: %d\n", display);
      ret = -EINVAL;
      goto errout_with_hang;
    }

  ginfo("Server started for Display %d\n", display);

  /* Allocate the framebuffer memory.  We rely on the fact that
   * the KMM allocator will align memory to 32-bits or better.
   */

  fb = (FAR uint8_t *)kmm_zalloc(RFB_SIZE);
  if (fb == NULL)
    {
      gerr("ERROR: Failed to allocate framebuffer memory: %lu KB\n",
           (unsigned long)(RFB_SIZE / 1024));
      ret = -ENOMEM;
      goto errout_with_post;
    }

  /* Allocate a session structure for this display */

  session = kmm_zalloc(sizeof(struct vnc_session_s));
  if (session == NULL)
    {
      gerr("ERROR: Failed to allocate session\n");
      ret = -ENOMEM;
      goto errout_with_fb;
    }

  g_vnc_sessions[display] = session;
  nxsem_init(&session->freesem, 0, CONFIG_VNCSERVER_NUPDATES);
  nxsem_init(&session->queuesem, 0, 0);

  /* Inform any waiter that we have started */

  vnc_reset_session(session, fb, display);
  nxsem_post(&g_fbstartup[display].fbinit);

  /* Loop... handling each each VNC client connection to this display.  Only
   * a single client is allowed for each display.
   */

  for (; ; )
    {
      /* Release the last session and [Re-]initialize the session structure
       * for the next connection.
       */

      vnc_reset_session(session, fb, display);
      g_fbstartup[display].result = -EBUSY;
      nxsem_reset(&g_fbstartup[display].fbconnect, 0);

      /* Establish a connection with the VNC client */

      ret = vnc_connect(session, RFB_DISPLAY_PORT(display));
      if (ret >= 0)
        {
          ginfo("New VNC connection\n");

          /* Perform the VNC initialization sequence after the client has
           * successfully connected to the server.  Negotiate security,
           * framebuffer and color properties.
           */

          ret = vnc_negotiate(session);
          if (ret < 0)
            {
              gerr("ERROR: Failed to negotiate security/framebuffer: %d\n",
                   ret);
              continue;
            }

          /* Start the VNC updater thread that sends all Server-to-Client
           * messages.
           */

          ret = vnc_start_updater(session);
          if (ret < 0)
            {
              gerr("ERROR: Failed to start updater thread: %d\n", ret);
              continue;
            }

          /* Let the framebuffer driver know that we are ready to perform
           * updates.
           */

          g_fbstartup[display].result = OK;
          nxsem_post(&g_fbstartup[display].fbconnect);

          /* Run the VNC receiver on this trhead.  The VNC receiver handles
           * all Client-to-Server messages.  The VNC receiver function does
           * not return until the session has been terminated (or an error
           * occurs).
           */

          ret = vnc_receiver(session);
          ginfo("Session terminated with %d\n", ret);
          UNUSED(ret);

          /* Stop the VNC updater thread. */

          ret = vnc_stop_updater(session);
          if (ret < 0)
            {
              gerr("ERROR: Failed to stop updater thread: %d\n", ret);
            }
        }
    }

errout_with_fb:
  kmm_free(fb);

errout_with_post:
  g_fbstartup[display].result = ret;
  nxsem_post(&g_fbstartup[display].fbconnect);

errout_with_hang:
  return EXIT_FAILURE;
}
