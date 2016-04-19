/****************************************************************************
 * graphics/vnc/server/vnc_fbdev.c
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/video/fb.h>

#include "vnc_server.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the frame buffer interface and also incapulates
 * information about the frame buffer instances for each display.
 */

struct vnc_fbinfo_s
{
  /* The publically visible frame buffer interface.  This must appear first
   * so that struct vnc_fbinfo_s is cast compatible with struct fb_vtable_s.
   */

  struct fb_vtable_s vtable;

  /* Our private per-display information */

  bool initialized;           /* True: This instance has been initialized */
  uint8_t display;            /* Display number */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Get information about the video controller configuration and the
 * configuration of each color plane.
 */

static int up_getvideoinfo(FAR struct fb_vtable_s *vtable,
                           FAR struct fb_videoinfo_s *vinfo);
static int up_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                           FAR struct fb_planeinfo_s *pinfo);

/* The following are provided only if the video hardware supports RGB color
 * mapping.
 */

#ifdef CONFIG_FB_CMAP
static int up_getcmap(FAR struct fb_vtable_s *vtable,
                      FAR struct fb_cmap_s *cmap);
static int up_putcmap(FAR struct fb_vtable_s *vtable,
                      FAR const struct fb_cmap_s *cmap);
#endif

/* The following are provided only if the video hardware supports a hardware
 * cursor.
 */

#ifdef CONFIG_FB_HWCURSOR
static int up_getcursor(FAR struct fb_vtable_s *vtable,
                        FAR struct fb_cursorattrib_s *attrib);
static int up_setcursor(FAR struct fb_vtable_s *vtable,
                        FAR struct fb_setcursor_s *setttings);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Current cursor position */

#ifdef CONFIG_FB_HWCURSOR
static struct fb_cursorpos_s g_cpos;

/* Current cursor size */

#ifdef CONFIG_FB_HWCURSORSIZE
static struct fb_cursorsize_s g_csize;
#endif
#endif

/* The framebuffer objects, one for each configured display. */

static struct vnc_fbinfo_s g_fbinfo[RFB_MAX_DISPLAYS];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Used to synchronize the server thread with the framebuffer driver.
 * NOTE:  This depends on the fact that all zero is correct initial state
 * for the semaphores.
 */

struct fb_startup_s g_fbstartup[RFB_MAX_DISPLAYS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getvideoinfo
 ****************************************************************************/

static int up_getvideoinfo(FAR struct fb_vtable_s *vtable,
                           FAR struct fb_videoinfo_s *vinfo)
{
  FAR struct vnc_fbinfo_s *fbinfo = (FAR struct vnc_fbinfo_s *)vtable;
  FAR struct vnc_session_s *session;

  gvdbg("vtable=%p vinfo=%p\n", vtable, vinfo);

  DEBUGASSERT(fbinfo != NULL && vinfo != NULL);
  if (fbinfo != NULL && vinfo != NULL)
    {
      session = vnc_find_session(fbinfo->display);
      if (session == NULL || session->state != VNCSERVER_RUNNING)
        {
          gdbg("ERROR: session is not connected\n");
          return -ENOTCONN;
        }

      /* Return the requested video info.  We are committed to using the
       * configured color format in the framebuffer, but performing color
       * conversions on the fly for the remote framebuffer as necessary.
       */

      vinfo->fmt     = RFB_COLORFMT;
      vinfo->xres    = CONFIG_VNCSERVER_SCREENWIDTH;
      vinfo->yres    = CONFIG_VNCSERVER_SCREENHEIGHT;
      vinfo->nplanes = 1;

      return OK;
    }

  gdbg("ERROR: Invalid arguments\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: up_getplaneinfo
 ****************************************************************************/

static int up_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                           FAR struct fb_planeinfo_s *pinfo)
{
  FAR struct vnc_fbinfo_s *fbinfo = (FAR struct vnc_fbinfo_s *)vtable;
  FAR struct vnc_session_s *session;

  gvdbg("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);

  DEBUGASSERT(fbinfo != NULL && pinfo != NULL && planeno == 0);
  if (fbinfo != NULL && pinfo != NULL && planeno == 0)
    {
      session = vnc_find_session(fbinfo->display);
      if (session == NULL || session->state != VNCSERVER_RUNNING)
        {
          gdbg("ERROR: session is not connected\n");
          return -ENOTCONN;
        }

      DEBUGASSERT(session->fb != NULL);

      /* Return the requested plane info.  We are committed to using the
       * configured bits-per-pixels in the framebuffer, but performing color
       * conversions on the fly for the remote framebuffer as necessary.
       */

      pinfo->fbmem    = (FAR void *)session->fb;
      pinfo->fblen    = RFB_SIZE;
      pinfo->stride   = RFB_STRIDE;
      pinfo->display  = fbinfo->display;
      pinfo->bpp      = RFB_BITSPERPIXEL;

      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: up_getcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int up_getcmap(FAR struct fb_vtable_s *vtable,
                      FAR struct fb_cmap_s *cmap)
{
  FAR struct vnc_fbinfo_s *fbinfo = (FAR struct vnc_fbinfo_s *)vtable;
  FAR struct vnc_session_s *session;
  int i;

  gvdbg("vtable=%p cmap=%p\n", vtable, cmap);

  DEBUGASSERT(fbinfo != NULL && cmap != NULL);

  if (fbinfo != NULL && cmap != NULL)
    {
      session = vnc_find_session(fbinfo->display);
      if (session == NULL || session->state != VNCSERVER_RUNNING)
        {
          gdbg("ERROR: session is not connected\n");
          return -ENOTCONN;
        }

      gvdbg("first=%d len=%d\n", vcmap->first, cmap->len);
#warning Missing logic

      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: up_putcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int up_putcmap(FAR struct fb_vtable_s *vtable, FAR const struct fb_cmap_s *cmap)
{
  FAR struct vnc_fbinfo_s *fbinfo = (FAR struct vnc_fbinfo_s *)vtable;
  FAR struct vnc_session_s *session;
  int i;

  gvdbg("vtable=%p cmap=%p\n", vtable, cmap);

  DEBUGASSERT(fbinfo != NULL && cmap != NULL);

  if (fbinfo != NULL && cmap != NULL)
    {
      session = vnc_find_session(fbinfo->display);
      if (session == NULL || session->state != VNCSERVER_RUNNING)
        {
          gdbg("ERROR: session is not connected\n");
          return -ENOTCONN;
        }

      gvdbg("first=%d len=%d\n", vcmap->first, cmap->len);
#warning Missing logic

      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: up_getcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int up_getcursor(FAR struct fb_vtable_s *vtable,
                        FAR struct fb_cursorattrib_s *attrib)
{
  FAR struct vnc_fbinfo_s *fbinfo = (FAR struct vnc_fbinfo_s *)vtable;
  FAR struct vnc_session_s *session;
  int i;

  gvdbg("vtable=%p attrib=%p\n", vtable, attrib);

  DEBUGASSERT(fbinfo != NULL && attrib != NULL);

  if (fbinfo != NULL && attrib != NULL)
    {
      session = vnc_find_session(fbinfo->display);
      if (session == NULL || session->state != VNCSERVER_RUNNING)
        {
          gdbg("ERROR: session is not connected\n");
          return -ENOTCONN;
        }

#warning Missing logic

      return OK;
    }
  gdbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: up_setcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int up_setcursor(FAR struct fb_vtable_s *vtable,
                       FAR struct fb_setcursor_s *settings)
{
  FAR struct vnc_fbinfo_s *fbinfo = (FAR struct vnc_fbinfo_s *)vtable;
  FAR struct vnc_session_s *session;
  int i;

  gvdbg("vtable=%p settings=%p\n", vtable, settings);

  DEBUGASSERT(fbinfo != NULL && settings != NULL);

  if (fbinfo != NULL && settings != NULL)
    {
      session = vnc_find_session(fbinfo->display);
      if (session == NULL || session->state != VNCSERVER_RUNNING)
        {
          gdbg("ERROR: session is not connected\n");
          return -ENOTCONN;
        }

      gvdbg("flags:   %02x\n", settings->flags);
      if ((settings->flags & FB_CUR_SETPOSITION) != 0)
        {
#warning Missing logic
        }

#ifdef CONFIG_FB_HWCURSORSIZE
      if ((settings->flags & FB_CUR_SETSIZE) != 0)
        {
#warning Missing logic
        }
#endif
#ifdef CONFIG_FB_HWCURSORIMAGE
      if ((settings->flags & FB_CUR_SETIMAGE) != 0)
        {
#warning Missing logic
        }
#endif
      return OK;
    }

  gdbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: vnc_wait_server
 *
 * Description:
 *   Wait for the server to be connected to the VNC client.  We can do
 *   nothing until that connection is established.
 *
 * Input parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

static inline int vnc_wait_server(int display)
{
  int errcode;
  int result;

  /* Check if there has been a session allocated yet.  This is one of the
   * first things that the VNC server will do with the kernel thread is
   * started.  But we might be here before the thread has gotten that far.
   *
   * If it has been allocated, then wait until it is in the RUNNING state.
   * The RUNNING state indicates that the server has started, it has
   * established a connection with the VNC client, it is negotiated
   * encodings and framebuffer characteristics, and it has started the
   * updater thread.  The server is now ready to recieve Client-to-Server
   * messages and to perform remote framebuffer updates.
   */

 while (g_vnc_sessions[display] == NULL ||
        g_vnc_sessions[display]->state != VNCSERVER_RUNNING)
    {
      /* The server is not yet running.  Wait for the server to post the FB
       * semaphore.  In certain error situations, the server may post the
       * semaphore, then reset it to zero.  There are are certainly race
       * conditions here, but I think none that are fatal.
       */

      while (sem_wait(&g_fbstartup[display].fbsem) < 0)
        {
          errcode = get_errno();

          /* sem_wait() should fail only if it is interrupt by a signal. */

          DEBUGASSERT(errcode == EINTR);
          if (errcode != EINTR)
            {
              DEBUGASSERT(errcode > 0);
              return -errcode;
            }
        }

      /* We were awakened.  A result of -EBUSY means that the negotiation
       * is not complete.  Why would we be awakened in that case?  Some
       * counting semaphore screw-up?
       */

      result = g_fbstartup[display].result;
      if (result != -EBUSY)
        {
#ifdef CONFIG_DEBUG
          if (result < 0)
            {
              DEBUGASSERT(g_vnc_sessions[display] == NULL);
              gdbg("ERROR: VNC server startup failed: %d\n", result);
            }
          else
            {
              DEBUGASSERT(g_vnc_sessions[display] != NULL &&
                          g_vnc_sessions[display]->state == VNCSERVER_RUNNING);
            }
#endif
          return result;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer video hardware associated with the display.
 *
 * Input parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  FAR char *argv[2];
  char str[8];
  pid_t pid;

  /* Start the VNC server kernel thread.
   * REVISIT: There is no protection for the case where this function is
   * called more that once.
   */

  gvdbg("Starting the VNC server for display %d\n", display);
  DEBUGASSERT(display >= 0 && display < RFB_MAX_DISPLAYS);

  /* Check if the server is already running */

  g_fbstartup[display].result = -EBUSY;
  sem_reset(&g_fbstartup[display].fbsem, 0);

  if (g_vnc_sessions[display] != NULL)
    {
      DEBUGASSERT(g_vnc_sessions[display]->state >= VNCSERVER_INITIALIZED);
    }
  else
    {
      /* Format the kernel thread arguments (ASCII.. yech) */

      (void)itoa(display, str, 10);
      argv[0] = str;
      argv[1] = NULL;

      pid = kernel_thread("vnc_server", CONFIG_VNCSERVER_PRIO,
                           CONFIG_VNCSERVER_STACKSIZE,
                           (main_t)vnc_server, argv);
      if (pid < 0)
        {
          gdbg("ERROR: Failed to start the VNC server: %d\n", (int)pid);
          return (int)pid;
        }
    }

  /* Wait for the VNC client to connect and for the RFB to be ready */

  return vnc_wait_server(display);
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video
 *   plane of the specified plane.  Many OSDs support multiple planes of video.
 *
 * Input parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *   vplane - Identifies the plane being queried.
 *
 * Returned Value:
 *   A non-NULL pointer to the frame buffer access structure is returned on
 *   success; NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  FAR struct vnc_session_s *session = vnc_find_session(display);
  FAR struct vnc_fbinfo_s *fbinfo;

  /* Verify that the session is still valid */

  if (session->state != VNCSERVER_RUNNING)
    {
      return NULL;
    }

  if (vplane == 0)
    {
      /* Has the framebuffer information been initialized for this display? */

      fbinfo = &g_fbinfo[display];
      if (!fbinfo->initialized)
        {
          fbinfo->vtable.getvideoinfo = up_getvideoinfo,
          fbinfo->vtable.getplaneinfo = up_getplaneinfo,
#ifdef CONFIG_FB_CMAP
          fbinfo->vtable.getcmap      = up_getcmap,
          fbinfo->vtable.putcmap      = up_putcmap,
#endif
#ifdef CONFIG_FB_HWCURSOR
          fbinfo->vtable.getcursor    = up_getcursor,
          fbinfo->vtable.setcursor    = up_setcursor,
#endif
          fbinfo->display             = display;
          fbinfo->initialized         = true;
        }

      return &fbinfo->vtable;
    }
  else
    {
      return NULL;
    }
}

/****************************************************************************
 * Name: up_fbuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer support for the specified display.
 *
 * Input Parameters:
 *   display - In the case of hardware with multiple displays, this
 *     specifies the display.  Normally this is zero.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_fbuninitialize(int display)
{
#if 0 /* Do nothing */
  FAR struct vnc_session_s *session = vnc_find_session(display);
  FAR struct vnc_fbinfo_s *fbinfo;

  DEBUGASSERT(session != NULL);
  fbinfo = &g_fbinfo[display];
#warning Missing logic
  UNUSED(session);
  UNUSED(fbinfo);
#endif
}

/****************************************************************************
 * Name: nx_notify_rectangle
 *
 * Description:
 *   When CONFIG_NX_UPDATE=y, then the graphics system will callout to
 *   inform some external module that the display has been updated.  This
 *   would be useful in a couple for cases.
 *
 *   - When a serial LCD is used, but a framebuffer is used to access the
 *     LCD.  In this case, the update callout can be used to refresh the
 *     affected region of the display.
 *
 *   - When VNC is enabled.  This is case, this callout is necessary to
 *     update the remote frame buffer to match the local framebuffer.
 *
 * When this feature is enabled, some external logic must provide this
 * interface.  This is the function that will handle the notification.  It
 * receives the rectangular region that was updated on the provided plane.
 *
 ****************************************************************************/

#ifdef CONFIG_NX_UPDATE
void nx_notify_rectangle(FAR NX_PLANEINFOTYPE *pinfo,
                         FAR const struct nxgl_rect_s *rect)
{
  FAR struct vnc_session_s *session;
  int ret;

  DEBUGASSERT(pinfo != NULL && rect != NULL);

  /* Recover the session informatin from the display number in the planeinfo
   * structure.
   */

  DEBUGASSERT(pinfo->display >= 0 && pinfo->display < RFB_MAX_DISPLAYS);
  session = vnc_find_session(pinfo->display);

  /* Verify that the session is still valid */

  if (session != NULL && session->state == VNCSERVER_RUNNING)
    {
      /* Queue the rectangular update */

      ret = vnc_update_rectangle(session, rect);
      if (ret < 0)
        {
          gdbg("ERROR: vnc_update_rectangle failed: %d\n", ret);
        }
    }
}
#endif
