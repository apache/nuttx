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
#include <nuttx/video/vnc.h>
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

#if defined(CONFIG_VNCSERVER_COLORFMT_RGB8)
#  define RFB_COLORFMT     FB_FMT_RGB8_332
#  define RFB_BITSPERPIXEL 8
#  define RFB_PIXELDEPTH   8
#  define RFB_TRUECOLOR    1
#  define RFB_RMAX         0x07
#  define RFB_GMAX         0x07
#  define RFB_BMAX         0x03
#  define RFB_RSHIFT       5
#  define RFB_GSHIFT       2
#  define RFB_BSHIFT       0
#elif defined(CONFIG_VNCSERVER_COLORFMT_RGB16)
#  define RFB_COLORFMT     FB_FMT_RGB16_565
#  define RFB_BITSPERPIXEL 16
#  define RFB_PIXELDEPTH   16
#  define RFB_TRUECOLOR    1
#  define RFB_RMAX         0x001f
#  define RFB_GMAX         0x003f
#  define RFB_BMAX         0x001f
#  define RFB_RSHIFT       11
#  define RFB_GSHIFT       5
#  define RFB_BSHIFT       0
#elif defined(CONFIG_VNCSERVER_COLORFMT_RGB32)
#  define RFB_COLORFMT     FB_FMT_RGB32
#  define RFB_BITSPERPIXEL 32
#  define RFB_PIXELDEPTH   24
#  define RFB_TRUECOLOR    1
#  define RFB_RMAX         0x000000ff
#  define RFB_GMAX         0x000000ff
#  define RFB_BMAX         0x000000ff
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

#ifndef CONFIG_VNCSERVER_NAME
#  define CONFIG_VNCSERVER_NAME "NuttX"
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

/* Debug */

#ifdef CONFIG_VNCSERVER_UPDATE_DEBUG
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define upderr(format, ...)    _err(format, ##__VA_ARGS__)
#    define updinfo(format, ...)   _info(format, ##__VA_ARGS__)
#    define updinfo(format, ...)   _info(format, ##__VA_ARGS__)
#  else
#   define upderr                  _err
#   define updwarn                 _warn
#   define updinfo                 _info
#  endif
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define upderr(x...)
#    define updwarn(x...)
#    define updinfo(x...)
#  else
#    define upderr                 (void)
#    define updwarn                (void)
#    define updinfo                (void)
#  endif
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
  bool whupd;                  /* True: whole screen update */
  struct nxgl_rect_s rect;     /* The enqueued update rectangle */
};

struct vnc_session_s
{
  /* Connection data */

  struct socket listen;        /* Listen socket */
  struct socket connect;       /* Connected socket */
  volatile uint8_t state;      /* See enum vnc_server_e */
  volatile uint8_t nwhupd;     /* Number of whole screen updates queued */
  volatile bool change;        /* True: Frambebuffer data change since last whole screen update */

  /* Display geometry and color characteristics */

  uint8_t display;             /* Display number (for debug) */
  volatile uint8_t colorfmt;   /* Remote color format (See include/nuttx/fb.h) */
  volatile uint8_t bpp;        /* Remote bits per pixel */
  volatile bool bigendian;     /* True: Remote expect data in big-endian format */
  volatile bool rre;           /* True: Remote supports RRE encoding */
  FAR uint8_t *fb;             /* Allocated local frame buffer */

  /* VNC client input support */

  vnc_kbdout_t kbdout;         /* Callout when keyboard input is received */
  vnc_mouseout_t mouseout;     /* Callout when keyboard input is received */
  FAR void *arg;               /* Argument that accompanies the callouts */

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
  sem_t fbinit;                 /* Wait for session creation */
  sem_t fbconnect;              /* Wait for client connection */
  int16_t result;               /* OK: successfully initialized */
};

/* The size of the color type in the local framebuffer */

#if defined(CONFIG_VNCSERVER_COLORFMT_RGB8)
typedef uint8_t lfb_color_t;
#elif defined(CONFIG_VNCSERVER_COLORFMT_RGB16)
typedef uint16_t lfb_color_t;
#elif defined(CONFIG_VNCSERVER_COLORFMT_RGB32)
typedef uint32_t lfb_color_t;
#else
#  error Unspecified/unsupported color format
#endif

/* Color conversion function pointer types */

typedef CODE uint8_t  (*vnc_convert8_t) (lfb_color_t rgb);
typedef CODE uint16_t (*vnc_convert16_t)(lfb_color_t rgb);
typedef CODE uint32_t (*vnc_convert32_t)(lfb_color_t rgb);

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
 * Name: vnc_client_pixelformat
 *
 * Description:
 *  A Client-to-Sever SetPixelFormat message has been received.  We need to
 *  immediately switch the output color format that we generate.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *   pixelfmt - The pixel from from the received SetPixelFormat message
 *
 * Returned Value:
 *   Returns zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int vnc_client_pixelformat(FAR struct vnc_session_s *session,
                           FAR struct rfb_pixelfmt_s *pixelfmt);

/****************************************************************************
 * Name: vnc_client_encodings
 *
 * Description:
 *   Pick out any mutually supported encodings from the Client-to-Server
 *   SetEncodings message
 *
 * Input Parameters:
 *   session   - An instance of the session structure.
 *   encodings - The received SetEncodings message
 *
 * Returned Value:
 *   At present, always returns OK
 *
 ****************************************************************************/

int vnc_client_encodings(FAR struct vnc_session_s *session,
                         FAR struct rfb_setencodings_s *encodings);

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
 *   change  - True: Frame buffer data has changed
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int vnc_update_rectangle(FAR struct vnc_session_s *session,
                         FAR const struct nxgl_rect_s *rect,
                         bool change);

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
 * Name: vnc_rre
 *
 * Description:
 *  This function does not really do RRE encoding.  It just checks if the
 *  update region is one color then uses the RRE encoding format to send
 *  the constant color rectangle.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *   rect  - Describes the rectangle in the local framebuffer.
 *
 * Returned Value:
 *   Zero is returned if RRE coding was not performed (but not error was)
 *   encountered.  Otherwise, the size of the framebuffer update message
 *   is returned on success or a negated errno value is returned on failure
 *   that indicates the nature of the failure.  A failure is only
 *   returned in cases of a network failure and unexpected internal failures.
 *
 ****************************************************************************/

int vnc_rre(FAR struct vnc_session_s *session, FAR struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: vnc_raw
 *
 * Description:
 *  As a fallback, send the framebuffer update using the RAW encoding which
 *  must be supported by all VNC clients.
 *
 * Input Parameters:
 *   session - An instance of the session structure.
 *   rect  - Describes the rectangle in the local framebuffer.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on failure that
 *   indicates the nature of the failure.  A failure is only returned
 *   in cases of a network failure and unexpected internal failures.
 *
 ****************************************************************************/

int vnc_raw(FAR struct vnc_session_s *session, FAR struct nxgl_rect_s *rect);

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
 * Name: vnc_convert_rgbNN
 *
 * Description:
 *  Convert the native framebuffer color format (either RGB8 3:3:2,
 *  RGB16 5:6:5, or RGB32 8:8:8) to the remote framebuffer color format
 *  (either RGB8 2:2:2, RGB8 3:3:2, RGB16 5:5:5, RGB16 5:6:5, or RGB32
 *  8:8:8)
 *
 * Input Parameters:
 *   pixel - The src color in local framebuffer format.
 *
 * Returned Value:
 *   The pixel in the remote framebuffer color format.
 *
 ****************************************************************************/

uint8_t  vnc_convert_rgb8_222(lfb_color_t rgb);
uint8_t  vnc_convert_rgb8_332(lfb_color_t rgb);
uint16_t vnc_convert_rgb16_555(lfb_color_t rgb);
uint16_t vnc_convert_rgb16_565(lfb_color_t rgb);
uint32_t vnc_convert_rgb32_888(lfb_color_t rgb);

/****************************************************************************
 * Name: vnc_colors
 *
 * Description:
 *  Test the update rectangle to see if it contains complex colors.  If it
 *  contains only a few colors, then it may be a candidate for some type
 *  run-length encoding.
 *
 * Input Parameters:
 *   session   - An instance of the session structure.
 *   rect      - The update region in the local frame buffer.
 *   maxcolors - The maximum number of colors that should be returned.  This
 *               currently cannot exceed eight.
 *   colors    - The top 'maxcolors' most frequency colors are returned.
 *
 * Returned Value:
 *   The number of valid colors in the colors[] array are returned, the
 *   first entry being the most frequent.  A negated errno value is returned
 *   if the colors cannot be determined.  This would be the case if the color
 *   depth is > 8 and there are more than 'maxcolors' colors in the update
 *   rectangle.
 *
 ****************************************************************************/

int vnc_colors(FAR struct vnc_session_s *session, FAR struct nxgl_rect_s *rect,
               unsigned int maxcolors, FAR lfb_color_t *colors);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __GRAPHICS_VNC_SERVER_VNC_SERVER_H */
