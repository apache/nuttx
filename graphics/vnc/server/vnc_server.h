/****************************************************************************
 * graphics/vnc/server/vnc_server.h
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

#ifndef __GRAPHICS_VNC_SERVER_VNC_SERVER_H
#define __GRAPHICS_VNC_SERVER_VNC_SERVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/video/fb.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>
#include <nuttx/net/net.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RFB Port Number */

#define RFB_PORT_BASE       5900
#define RFB_MAX_DISPLAYS    100
#define RFB_DISPLAY_PORT(d) (RFB_PORT_BASE + (d))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This enumeration indicates the state of the VNC server */

enum vnc_server_e
{
  VNCSERVER_UNINITIALIZED = 0, /* Initial state */
  VNCSERVER_INITIALIZED,       /* State structured initialized, but not connected */
  VNCSERVER_CONNECTED,         /* Connect to a client, but not yet configured */
  VNCSERVER_CONFIGURED,        /* Configured and ready to transfer graphics */
  VNCSERVER_SCANNING,          /* Running and activly transferring graphics */
  VNCSERVER_STOPPING           /* The server has been asked to stop */
};

struct vnc_session_s
{
  /* NX graphics system */

  NXHANDLE handle;             /* NX graphics handle */

  /* Connection data */

  struct socket listen;        /* Listen socket */
  struct socket connect;       /* Connected socket */
  volatile uint8_t state;      /* See enum vnc_server_e */

  /* Display geometry and color characteristics */

  uint8_t colorfmt;            /* See include/nuttx/fb.h */
  uint8_t bpp;                 /* Bits per pixel */
  struct nxgl_size_s screen;   /* Size of the screen in pixels x rows */
  FAR uint8_t *fb;             /* Allocated local frame buffer */
};

/****************************************************************************
 * Public Function Prototypes
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

int vnc_connect(FAR struct vnc_session_s *session, int port);

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

FAR struct vnc_session_s *vnc_create_session(void);

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

void vnc_release_session(FAR struct vnc_session_s *session);

/****************************************************************************
 * Name: vnc_session
 *
 * Description:
 *  This function encapsulates the entire VNC session.
 *
 * Input Parameters:
 *   session - An instance of the session structure allocated by
 *     vnc_create_session().
 *
 * Returned Value:
 *   At present, always returns OK
 *
 ****************************************************************************/

int vnc_session(FAR struct vnc_session_s *session);

#endif /* __GRAPHICS_VNC_SERVER_VNC_SERVER_H */
