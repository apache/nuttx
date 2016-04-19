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
#include <semaphore.h>
#include <pthread.h>
#include <queue.h>

#include <nuttx/video/fb.h>
#include <nuttx/video/rfb.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>
#include <nuttx/net/net.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

#ifndef CONFIG_NET_TCP_READAHEAD
#  error CONFIG_NET_TCP_READAHEAD must be set to use VNC
#endif

#ifndef CONFIG_NX_UPDATE
#  error CONFIG_NX_UPDATE must be set to use VNC
#endif

#if !defined(CONFIG_VNCSERVER_PROTO3p3) && !defined(CONFIG_VNCSERVER_PROTO3p8)
#  error No VNC protocol selected
#endif

#if defined(CONFIG_VNCSERVER_PROTO3p3) && defined(CONFIG_VNCSERVER_PROTO3p8)
#  error Too many VNC protocols selected
#endif

#ifndef CONFIG_VNCSERVER_NDISPLAYS
#  define CONFIG_VNCSERVER_NDISPLAYS 1
#endif

#if defined(CONFIG_VNCSERVER_COLORFMT_RGB16)
#  define RFB_COLORFMT     FB_FMT_RGB16_565
#  define RFB_BITSPERPIXEL 16
#  define RFB_PIXELDEPTH   16
#  define RFB_TRUECOLOR    1
#  define RFB_RMAX         0x1f
#  define RFB_GMAX         0x3f
#  define RFB_BMAX         0x1f
#  define RFB_RSHIFT       11
#  define RFB_GSHIFT       5
#  define RFB_BSHIFT       0
#elif defined(CONFIG_VNCSERVER_COLORFMT_RGB32)
#  define RFB_COLORFMT     FB_FMT_RGB32
#  define RFB_BITSPERPIXEL 32
#  define RFB_PIXELDEPTH   24
#  define RFB_TRUECOLOR    1
#  define RFB_RMAX         0xff
#  define RFB_GMAX         0xff
#  define RFB_BMAX         0xff
#  define RFB_RSHIFT       16
#  define RFB_GSHIFT       8
#  define RFB_BSHIFT       0
#else
#  error Unspecified/unsupported color format
#endif

#ifndef CONFIG_VNCSERVER_SCREENWIDTH
#  define CONFIG_VNCSERVER_SCREENWIDTH 320
#endif

#ifndef CONFIG_VNCSERVER_SCREENHEIGHT
#  define CONFIG_VNCSERVER_SCREENHEIGHT 240
#endif

#ifndef CONFIG_VNCSERVER_PRIO
#  define CONFIG_VNCSERVER_PRIO 100
#endif

#ifndef CONFIG_VNCSERVER_STACKSIZE
#  define CONFIG_VNCSERVER_STACKSIZE 2048
#endif

#ifndef CONFIG_VNCSERVER_UPDATER_PRIO
#  define CONFIG_VNCSERVER_UPDATER_PRIO 100
#endif

#ifndef CONFIG_VNCSERVER_UPDATER_STACKSIZE
#  define CONFIG_VNCSERVER_UPDATER_STACKSIZE 2048
#endif

#ifndef CONFIG_VNCSERVER_INBUFFER_SIZE
#  define CONFIG_VNCSERVER_INBUFFER_SIZE 80
#endif

#ifndef CONFIG_VNCSERVER_NUPDATES
#  define CONFIG_VNCSERVER_NUPDATES 48
#endif

#ifndef CONFIG_VNCSERVER_UPDATE_BUFSIZE
#  define CONFIG_VNCSERVER_UPDATE_BUFSIZE 4096
#endif

#define VNCSERVER_UPDATE_BUFSIZE \
  (CONFIG_VNCSERVER_UPDATE_BUFSIZE + SIZEOF_RFB_FRAMEBUFFERUPDATE_S(0))

/* Local framebuffer characteristics in bytes */

#define RFB_BYTESPERPIXEL   ((RFB_BITSPERPIXEL + 7) >> 3)
#define RFB_STRIDE          (RFB_BYTESPERPIXEL * CONFIG_VNCSERVER_SCREENWIDTH)
#define RFB_SIZE            (RFB_STRIDE * CONFIG_VNCSERVER_SCREENHEIGHT)

/* RFB Port Number */

#define RFB_PORT_BASE       5900
#define RFB_MAX_DISPLAYS    CONFIG_VNCSERVER_NDISPLAYS
#define RFB_DISPLAY_PORT(d) (RFB_PORT_BASE + (d))

/* Miscellaneous */

#ifndef MIN
#  define MIN(a,b)          (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b)          (((a) > (b)) ? (a) : (b))
#endif

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
  VNCSERVER_RUNNING,           /* Running and activly transferring graphics */
  VNCSERVER_STOPPING,          /* The updater has been asked to stop */
  VNCSERVER_STOPPED            /* The updater has stopped */
};

/* This structure is used to queue FrameBufferUpdate event.  It includes a
 * pointer to support singly linked list.
 */

struct vnc_fbupdate_s
{
  FAR struct vnc_fbupdate_s *flink;
  struct nxgl_rect_s rect;     /* The enqueued update rectangle */
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

  uint8_t display;             /* Display number (for debug) */
  uint8_t colorfmt;            /* Remote color format (See include/nuttx/fb.h) */
  uint8_t bpp;                 /* Remote bits per pixel */
  FAR uint8_t *fb;             /* Allocated local frame buffer */

  /* Updater information */

  pthread_t updater;           /* Updater thread ID */

  /* Update list information */

  struct vnc_fbupdate_s updpool[CONFIG_VNCSERVER_NUPDATES];
  sq_queue_t updfree;
  sq_queue_t updqueue;
  sem_t freesem;
  sem_t queuesem;

  /* I/O buffers for misc network send/receive */

  uint8_t inbuf[CONFIG_VNCSERVER_INBUFFER_SIZE];
  uint8_t outbuf[VNCSERVER_UPDATE_BUFSIZE];
};

/* This structure is used to communicate start-up status between the server
 * the framebuffer driver.
 */

struct fb_startup_s
{
  sem_t fbsem;                  /* Framebuffer driver will wait on this */
  int16_t result;               /* OK: successfully initialized */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Given a display number as an index, the following array can be used to
 * look-up the session structure for that display.
 */

EXTERN FAR struct vnc_session_s *g_vnc_sessions[RFB_MAX_DISPLAYS];

/* Used to synchronize the server thread with the framebuffer driver. */

EXTERN struct fb_startup_s g_fbstartup[RFB_MAX_DISPLAYS];

/****************************************************************************
 * Public Function Prototypes
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

int vnc_server(int argc, FAR char *argv[]);

/****************************************************************************
 * Name: vnc_negotiate
 *
 * Description:
 *  Perform the VNC initialization sequence after the client has sucessfully
 *  connected to the server.  Negotiate security, framebuffer and color
 *  properties.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *
 * Returned Value:
 *   Returns zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int vnc_negotiate(FAR struct vnc_session_s *session);

/****************************************************************************
 * Name: vnc_start_updater
 *
 * Description:
 *  Start the updater thread
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int vnc_start_updater(FAR struct vnc_session_s *session);

/****************************************************************************
 * Name: vnc_stop_updater
 *
 * Description:
 *  Stop the updater thread
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int vnc_stop_updater(FAR struct vnc_session_s *session);

/****************************************************************************
 * Name: vnc_update_rectangle
 *
 * Description:
 *  Queue an update of the specified rectangular region on the display.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *   rect    - The rectanglular region to be updated.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int vnc_update_rectangle(FAR struct vnc_session_s *session,
                         FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: vnc_receiver
 *
 * Description:
 *  This function handles all Client-to-Server messages.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *
 * Returned Value:
 *   At present, always returns OK
 *
 ****************************************************************************/

int vnc_receiver(FAR struct vnc_session_s *session);

/****************************************************************************
 * Name: vnc_key_map
 *
 * Description:
 *   Map the receive X11 keysym into something understood by NuttX and route
 *   that through NX to the appropriate window.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *   keysym  - The X11 keysym value (see include/nuttx/inputx11_keysymdef)
 *   keydown - True: Key pressed; False: Key released
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
void vnc_key_map(FAR struct vnc_session_s *session, uint16_t keysym,
                 bool keydown);
#endif

/****************************************************************************
 * Name: vnc_find_session
 *
 * Description:
 *  Return the session structure associated with this display.
 *
 * Input Parameters:
 *   display - The display number of interest.
 *
 * Returned Value:
 *   Returns the instance of the session structure for this display.  NULL
 *   will be returned if the server has not yet been started or if the
 *   display number is out of range.
 *
 ****************************************************************************/

FAR struct vnc_session_s *vnc_find_session(int display);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __GRAPHICS_VNC_SERVER_VNC_SERVER_H */
