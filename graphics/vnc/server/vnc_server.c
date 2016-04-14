/****************************************************************************
 * graphics/vnc/vnc_server.c
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

#include "nuttx/config.h"

#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include "vnc_server.h"

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
  int display;
  int ret;

  DEBUGASSERT(session != NULL);

  /* A single argument is expected:  A diplay port number in ASCII form */

  if (argc != 2)
    {
      gdbg("ERROR: Unexpected number of arguments: %d\n", argc);
      return EXIT_FAILURE;
    }

  display = atoi(argv[1]);
  if (display < 0 || display >= RFB_MAX_DISPLAYS)
    {
      gdbg("ERROR: Invalid display number: %d\n", display);
      return EXIT_FAILURE;
    }

  /* Allocate a session structure for this display */

  session = vnc_create_session();
  if (session == NULL)
    {
      gdbg("ERROR: Failed to allocate session\n");
      return EXIT_FAILURE;
    }

  /* Loop... handling each each VNC client connection to this display.  Only
   * a single client is allowed for each display.
   */

  for (; ; )
    {
      /* Establish a connection with the VNC client */

      ret = vnc_connect(session, RFB_DISPLAY_PORT(display));
      if (ret >= 0)
        {
          gvdbg("New VNC connection\n");

          /* Start the VNC session */

          (void)vnc_session(session);

          /* Re-initialize the session structure for re-use */

          vnc_release_session(session);
        }
    }

  return EXIT_FAILURE; /* We won't get here */
}
