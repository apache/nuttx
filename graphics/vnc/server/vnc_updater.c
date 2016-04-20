/****************************************************************************
 * graphics/vnc/vnc_updater.c
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

#include <semaphore.h>
#include <sched.h>
#include <string.h>
#include <pthread.h>
#include <queue.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/video/rgbcolors.h>

#include "vnc_server.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef VNCSERVER_SEM_DEBUG

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Color conversion functions */

#if defined(CONFIG_VNCSERVER_COLORFMT_RGB16)
typedef CODE uint8_t(*vnc_convert8_t)(uint16_t rgb);
typedef CODE uint16_t(*vnc_convert16_t)(uint16_t rgb);
typedef CODE uint32_t(*vnc_convert32_t)(uint16_t rgb);
#elif defined(CONFIG_VNCSERVER_COLORFMT_RGB32)
typedef CODE uint8_t(*vnc_convert8_t)(uint32_t rgb);
typedef CODE uint16_t(*vnc_convert16_t)(uint32_t rgb);
typedef CODE uint32_t(*vnc_convert32_t)(uint32_t rgb);
#else
#  error Unspecified/unsupported color format
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef VNCSERVER_SEM_DEBUG
static sem_t g_dbgsem = SEM_INITIALIZER(1);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_sem_debug
 *
 * Description:
 *   Dump information about the freesem to verify that it is sync.
 *
 * Input Parameters:
 *   session - A reference to the VNC session structure.
 *
 * Returned Value:
 *   A non-NULL structure pointer should always be returned.  This function
 *   will wait if no structure is available.
 *
 ****************************************************************************/

#ifdef VNCSERVER_SEM_DEBUG
static void vnc_sem_debug(FAR struct vnc_session_s *session,
                          FAR const char *msg, unsigned int unattached)
{
  FAR struct vnc_fbupdate_s *update;
  unsigned int nqueued;
  unsigned int nfree;

  while (sem_wait(&g_dbgsem) < 0)
    {
      DEBUGASSERT(get_errno() == EINTR);
    }

  /* Count structures in the list */

  for (nqueued = 0, update = (FAR struct vnc_fbupdate_s *)session->updqueue.head;
       update != NULL;
       nqueued++, update = update->flink);

  for (nfree = 0, update = (FAR struct vnc_fbupdate_s *)session->updfree.head;
       update != NULL;
       nfree++, update = update->flink);

  syslog(LOG_INFO, "FREESEM DEBUG: %s\n", msg);
  syslog(LOG_INFO, "  freesem:    %d\n", session->freesem.semcount);
  syslog(LOG_INFO, "  queued:     %u\n", nqueued);
  syslog(LOG_INFO, "  free:       %u\n", nfree);
  syslog(LOG_INFO, "  unattached: %u\n", unattached);

  sem_post(&g_dbgsem);
}
#else
#  define vnc_sem_debug(s,m,u)
#endif

/****************************************************************************
 * Name: vnc_alloc_update
 *
 * Description:
 *   Allocate one update structure by taking it from the freelist.
 *
 * Input Parameters:
 *   session - A reference to the VNC session structure.
 *
 * Returned Value:
 *   A non-NULL structure pointer should always be returned.  This function
 *   will wait if no structure is available.
 *
 ****************************************************************************/

static FAR struct vnc_fbupdate_s *
vnc_alloc_update(FAR struct vnc_session_s *session)
{
  FAR struct vnc_fbupdate_s *update;

  /* Reserve one element from the free list.  Lock the scheduler to assure
   * that the sq_remfirst() and the successful return from sem_wait are
   * atomic.  Of course, the scheduler will be unlocked while we wait.
   */

  sched_lock();
  vnc_sem_debug(session, "Before alloc", 0);

  while (sem_wait(&session->freesem) < 0)
    {
      DEBUGASSERT(get_errno() == EINTR);
    }

  /* It is reserved.. go get it */

  update = (FAR struct vnc_fbupdate_s *)sq_remfirst(&session->updfree);

  vnc_sem_debug(session, "After alloc", 1);
  sched_unlock();

  DEBUGASSERT(update != NULL);
  return update;
}

/****************************************************************************
 * Name: vnc_free_update
 *
 * Description:
 *  Free one update structure by returning it from the freelist.
 *
 * Input Parameters:
 *   session - A reference to the VNC session structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void vnc_free_update(FAR struct vnc_session_s *session,
                            FAR struct vnc_fbupdate_s *update)
{
  /* Reserve one element from the free list.  Lock the scheduler to assure
   * that the sq_addlast() and the sem_post() are atomic.
   */

  sched_lock();
  vnc_sem_debug(session, "Before free", 1);

  /* Put the entry into the free list */

  sq_addlast((FAR sq_entry_t *)update, &session->updfree);

  /* Post the semaphore to indicate the availability of one more update */

  sem_post(&session->freesem);

  vnc_sem_debug(session, "After free", 0);
  DEBUGASSERT(session->freesem.semcount <= CONFIG_VNCSERVER_NUPDATES);

  sched_unlock();
}

/****************************************************************************
 * Name: vnc_remove_queue
 *
 * Description:
 *  Remove one entry from the list of queued rectangles, waiting if
 *  necessary if the queue is empty.
 *
 * Input Parameters:
 *   session - A reference to the VNC session structure.
 *
 * Returned Value:
 *   A non-NULL structure pointer should always be returned.  This function
 *   will wait if no structure is available.
 *
 ****************************************************************************/

static FAR struct vnc_fbupdate_s *
vnc_remove_queue(FAR struct vnc_session_s *session)
{
  FAR struct vnc_fbupdate_s *rect;

  /* Reserve one element from the list of queued rectangle.  Lock the
   * scheduler to assure that the sq_remfirst() and the successful return
   * from sem_wait are atomic.  Of course, the scheduler will be unlocked
   * while we wait.
   */

  sched_lock();
  vnc_sem_debug(session, "Before remove", 0);

  while (sem_wait(&session->queuesem) < 0)
    {
      DEBUGASSERT(get_errno() == EINTR);
    }

  /* It is reserved.. go get it */

  rect = (FAR struct vnc_fbupdate_s *)sq_remfirst(&session->updqueue);

  vnc_sem_debug(session, "After remove", 0);
  sched_unlock();

  DEBUGASSERT(rect != NULL);
  return rect;
}

/****************************************************************************
 * Name: vnc_add_queue
 *
 * Description:
 *   Add one rectangle entry to the list of queued rectangles to be updated.
 *
 * Input Parameters:
 *   session - A reference to the VNC session structure.
 *   rect    - The rectangle to be added to the queue.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void vnc_add_queue(FAR struct vnc_session_s *session,
                          FAR struct vnc_fbupdate_s *rect)
{
  /* Lock the scheduler to assure that the sq_addlast() and the sem_post()
   * are atomic.
   */

  sched_lock();
  vnc_sem_debug(session, "Before add", 1);

  /* Put the entry into the list of queued rectangles. */

  sq_addlast((FAR sq_entry_t *)rect, &session->updqueue);

  /* Post the semaphore to indicate the availability of one more rectangle
   * in the queue.  This may wakeup the updater.
   */

  sem_post(&session->queuesem);

  vnc_sem_debug(session, "After add", 0);
  DEBUGASSERT(session->queuesem.semcount <= CONFIG_VNCSERVER_NUPDATES);

  sched_unlock();
}

/****************************************************************************
 * Name: vnc_convert_rgbNN
 *
 * Description:
 *  Convert the native framebuffer color format (either RGB16 5:6:5 or RGB32
 *  8:8:8) to the remote framebuffer color format (either RGB16 5:6:5,
 *  RGB16 5:5:5, or RGB32 8:8:)
 *
 * Input Parameters:
 *   pixel - The src color in local framebuffer format.
 *
 * Returned Value:
 *   The pixel in the remote framebuffer color format.
 *
 ****************************************************************************/

#if defined(CONFIG_VNCSERVER_COLORFMT_RGB16)

uint8_t vnc_convert_rgb8_222(uint16_t rgb)
{
  /* 111111
   * 54321098 76543210
   * -----------------
   * RRRRRGGG GGGBBBBB
   *          ..RRGGBB
   */

  return (uint8_t)(((rgb >> 10) & 0x0030)  |
                   ((rgb >> 7)  & 0x000c)  |
                   ((rgb >> 3)  & 0x0003));
}

uint8_t vnc_convert_rgb8_332(uint16_t rgb)
{
  /* 111111
   * 54321098 76543210
   * -----------------
   * RRRRRGGG GGGBBBBB
   *          RRRGGGBB
   */

  return (uint8_t)(((rgb >> 8) & 0x0070)  |
                   ((rgb >> 6) & 0x001c)  |
                   ((rgb >> 3) & 0x0003));
}

uint16_t vnc_convert_rgb16_555(uint16_t rgb)
{
  /* 111111
   * 54321098 76543210
   * -----------------
   * RRRRRGGG GGGBBBBB
   * .RRRRRGG GGGBBBBB
   */

  return (((rgb >> 1) & ~0x1f) | (rgb & 0x1f));
}

uint16_t vnc_convert_rgb16_565(uint16_t rgb)
{
  /* Identity mapping */

  return rgb;
}

uint32_t vnc_convert_rgb32_888(uint16_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * ----------------------------------
   *                   RRRRRGGG GGGBBBBB
   *          RRRRR... GGGGGG.. BBBBB...
   */

  return (((uint32_t)rgb << 8) & 0x00f80000) |
         (((uint32_t)rgb << 6) & 0x0000fc00) |
         (((uint32_t)rgb << 3) & 0x000000f8);
}

#elif defined(CONFIG_VNCSERVER_COLORFMT_RGB32)
uint8_t vnc_convert_rgb8_222(uint16_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * -----------------------------------
   *          RRRRRRRR GGGGGGGG BBBBBBBB
   *                            ..RRGGBB
   */

  return (uint8_t)(((rgb >> 18) & 0x0030)  |
                   ((rgb >> 12) & 0x000c)  |
                    (rgb >> 6)  & 0x0003));
}

uint8_t vnc_convert_rgb8_332(uint16_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * -----------------------------------
   *          RRRRRRRR GGGGGGGG BBBBBBBB
   *                            RRRGGGBB
   */

  return (uint8_t)(((rgb >> 16) & 0x0070)  |
                   ((rgb >> 11) & 0x001c)  |
                    (rgb >> 6)  & 0x0003));
}

uint16_t vnc_convert_rgb16_555(uint32_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * -----------------------------------
   *          RRRRRRRR GGGGGGGG BBBBBBBB
   *                   .RRRRRGG GGGBBBBB
   */

  return (uint16_t)
    (((rgb >> 9) & 0x00007c00) |
     ((rgb >> 6) & 0x000003e0) |
     ((rgb >> 3) & 0x0000001f));
}

uint16_t vnc_convert_rgb16_565(uint32_t rgb)
{
  /* 33222222 22221111 111111
   * 10987654 32109876 54321098 76543210
   * -----------------------------------
   *          RRRRRRRR GGGGGGGG BBBBBBBB
   *                   RRRRRGGG GGGBBBBB
   */

  return (uint16_t)
    (((rgb >> 8) & 0x0000f800) |
     ((rgb >> 5) & 0x000007e0) |
     ((rgb >> 3) & 0x0000001f));
}

uint32_t vnc_convert_rgb32_888(uint32_t rgb)
{
  /* Identity mapping */

  return rgb;
}
#else
#  error Unspecified/unsupported color format
#endif

/****************************************************************************
 * Name: vnc_copy8
 *
 * Description:
 *   Copy a 16/32-bit pixels from the source rectangle to a 8-bit pixel
 *   destination rectangle.
 *
 * Input Parameters:
 *   session      - A reference to the VNC session structure.
 *   row,col      - The upper left X/Y (pixel/row) position of the rectangle
 *   width,height - The width (pixels) and height (rows of the rectangle)
 *   convert      - The function to use to convert from the local framebuffer
 *                  color format to the remote framebuffer color format.
 *
 * Returned Value:
 *   The size of the transfer in bytes.
 *
 ****************************************************************************/

static size_t vnc_copy8(FAR struct vnc_session_s *session,
                         nxgl_coord_t row, nxgl_coord_t col,
                         nxgl_coord_t height, nxgl_coord_t width,
                         vnc_convert8_t convert)
{
#if defined(CONFIG_VNCSERVER_COLORFMT_RGB16)
  FAR struct rfb_framebufferupdate_s *update;
  FAR const uint16_t *srcleft;
  FAR const uint16_t *src;
  FAR uint8_t *dest;
  nxgl_coord_t x;
  nxgl_coord_t y;

  /* Destination rectangle start address */

  update = (FAR struct rfb_framebufferupdate_s *)session->outbuf;
  dest   = (FAR uint8_t *)update->rect[0].data;

  /* Source rectangle start address (left/top)*/

  srcleft = (FAR uint16_t *)(session->fb + RFB_STRIDE * row + RFB_BYTESPERPIXEL * col);

  /* Transfer each row from the source buffer into the update buffer */

  for (y = 0; y < height; y++)
    {
      src = srcleft;
      for (x = 0; x < width; x++)
        {
          *dest++ = convert(*src);
          src++;
        }

      srcleft = (FAR uint16_t *)((uintptr_t)srcleft + RFB_STRIDE);
    }

  return (size_t)((uintptr_t)dest - (uintptr_t)update->rect[0].data);

#elif defined(CONFIG_VNCSERVER_COLORFMT_RGB32)
  FAR struct rfb_framebufferupdate_s *update;
  FAR const uint32_t *srcleft;
  FAR const uint32_t *src;
  FAR uint8_t *dest;
  nxgl_coord_t x;
  nxgl_coord_t y;

  /* Destination rectangle start address */

  update = (FAR struct rfb_framebufferupdate_s *)session->outbuf;
  dest   = (FAR uint8_t *)update->rect[0].data;

  /* Source rectangle start address */

  srcleft = (FAR uint32_t *)(session->fb + RFB_STRIDE * row + RFB_BYTESPERPIXEL * col);

  for (y = 0; y < height; y++)
    {
      src = srcleft;
      for (x = 0; x < width; x++)
        {
          *dest++ = convert(*src);
          src++;
        }

      srcleft = (FAR uint32_t *)((uintptr_t)srcleft + RFB_STRIDE);
    }

  return (size_t)((uintptr_t)dest - (uintptr_t)update->rect[0].data);
#endif
}

/****************************************************************************
 * Name: vnc_copy16
 *
 * Description:
 *   Copy a 16/32-bit pixels from the source rectangle to a 16-bit pixel
 *   destination rectangle.
 *
 * Input Parameters:
 *   session      - A reference to the VNC session structure.
 *   row,col      - The upper left X/Y (pixel/row) position of the rectangle
 *   width,height - The width (pixels) and height (rows of the rectangle)
 *   convert      - The function to use to convert from the local framebuffer
 *                  color format to the remote framebuffer color format.
 *
 * Returned Value:
 *   The size of the transfer in bytes.
 *
 ****************************************************************************/

static size_t vnc_copy16(FAR struct vnc_session_s *session,
                         nxgl_coord_t row, nxgl_coord_t col,
                         nxgl_coord_t height, nxgl_coord_t width,
                         vnc_convert16_t convert)
{
#if defined(CONFIG_VNCSERVER_COLORFMT_RGB16)
  FAR struct rfb_framebufferupdate_s *update;
  FAR const uint16_t *srcleft;
  FAR const uint16_t *src;
  FAR uint16_t *dest;
  nxgl_coord_t x;
  nxgl_coord_t y;

  /* Destination rectangle start address */

  update = (FAR struct rfb_framebufferupdate_s *)session->outbuf;
  dest   = (FAR uint16_t *)update->rect[0].data;

  /* Source rectangle start address (left/top)*/

  srcleft = (FAR uint16_t *)(session->fb + RFB_STRIDE * row + RFB_BYTESPERPIXEL * col);

  /* Transfer each row from the source buffer into the update buffer */

  for (y = 0; y < height; y++)
    {
      src = srcleft;
      for (x = 0; x < width; x++)
        {
          *dest++ = convert(*src);
          src++;
        }

      srcleft = (FAR uint16_t *)((uintptr_t)srcleft + RFB_STRIDE);
    }

  return (size_t)((uintptr_t)dest - (uintptr_t)update->rect[0].data);

#elif defined(CONFIG_VNCSERVER_COLORFMT_RGB32)
  FAR struct rfb_framebufferupdate_s *update;
  FAR const uint32_t *srcleft;
  FAR const uint32_t *src;
  FAR uint16_t *dest;
  nxgl_coord_t x;
  nxgl_coord_t y;

  /* Destination rectangle start address */

  update = (FAR struct rfb_framebufferupdate_s *)session->outbuf;
  dest   = (FAR uint16_t *)update->rect[0].data;

  /* Source rectangle start address */

  srcleft = (FAR uint32_t *)(session->fb + RFB_STRIDE * row + RFB_BYTESPERPIXEL * col);

  for (y = 0; y < height; y++)
    {
      src = srcleft;
      for (x = 0; x < width; x++)
        {
          *dest++ = convert(*src);
          src++;
        }

      srcleft = (FAR uint32_t *)((uintptr_t)srcleft + RFB_STRIDE);
    }

  return (size_t)((uintptr_t)dest - (uintptr_t)update->rect[0].data);
#endif
}

/****************************************************************************
 * Name: vnc_copy32
 *
 * Description:
 *   Copy a 16/32-bit pixels from the source rectangle to a 32-bit pixel
 *   destination rectangle.
 *
 * Input Parameters:
 *   session      - A reference to the VNC session structure.
 *   row,col      - The upper left X/Y (pixel/row) position of the rectangle
 *   width,height - The width (pixels) and height (rows of the rectangle)
 *   convert      - The function to use to convert from the local framebuffer
 *                  color format to the remote framebuffer color format.
 *
 * Returned Value:
 *   The size of the transfer in bytes.
 *
 ****************************************************************************/

static size_t vnc_copy32(FAR struct vnc_session_s *session,
                         nxgl_coord_t row, nxgl_coord_t col,
                         nxgl_coord_t height, nxgl_coord_t width,
                         vnc_convert32_t convert)
{
#if defined(CONFIG_VNCSERVER_COLORFMT_RGB16)
  FAR struct rfb_framebufferupdate_s *update;
  FAR const uint16_t *srcleft;
  FAR const uint16_t *src;
  FAR uint32_t *dest;
  nxgl_coord_t x;
  nxgl_coord_t y;

  /* Destination rectangle start address */

  update = (FAR struct rfb_framebufferupdate_s *)session->outbuf;
  dest   = (FAR uint32_t *)update->rect[0].data;

  /* Source rectangle start address (left/top)*/

  srcleft = (FAR uint16_t *)(session->fb + RFB_STRIDE * row + RFB_BYTESPERPIXEL * col);

  /* Transfer each row from the source buffer into the update buffer */

  for (y = 0; y < height; y++)
    {
      src = srcleft;
      for (x = 0; x < width; x++)
        {
          *dest++ = convert(*src);
          src++;
        }

      srcleft = (FAR uint16_t *)((uintptr_t)srcleft + RFB_STRIDE);
    }

  return (size_t)((uintptr_t)srcleft - (uintptr_t)update->rect[0].data);

#elif defined(CONFIG_VNCSERVER_COLORFMT_RGB32)
  FAR struct rfb_framebufferupdate_s *update;
  FAR const uint32_t *srcleft;
  FAR const uint32_t *src;
  FAR uint32_t *dest;
  nxgl_coord_t x;
  nxgl_coord_t y;

  /* Destination rectangle start address */

  update = (FAR struct rfb_framebufferupdate_s *)session->outbuf;
  dest   = (FAR uint32_t *)update->rect[0].data;

  /* Source rectangle start address */

  srcleft = (FAR uint32_t *)(session->fb + RFB_STRIDE * row + RFB_BYTESPERPIXEL * col);

  for (y = 0; y < height; y++)
    {
      src = srcleft;
      for (x = 0; x < width; x++)
        {
          *dest++ = convert(*src);
          src++;
        }

      srcleft = (FAR uint32_t *)((uintptr_t)srcleft + RFB_STRIDE);
    }

  return (size_t)((uintptr_t)srcleft - (uintptr_t)update->rect[0].data);
#endif
}

/****************************************************************************
 * Name: vnc_updater
 *
 * Description:
 *  This is the "updater" thread.  It is the sender of all Server-to-Client
 *  messages
 *
 * Input Parameters:
 *   Standard pthread arguments.
 *
 * Returned Value:
 *   NULL is always returned.
 *
 ****************************************************************************/

static FAR void *vnc_updater(FAR void *arg)
{
  FAR struct vnc_session_s *session = (FAR struct vnc_session_s *)arg;
  FAR struct rfb_framebufferupdate_s *update;
  FAR struct vnc_fbupdate_s *srcrect;
  FAR const uint8_t *src;
  nxgl_coord_t srcwidth;
  nxgl_coord_t srcheight;
  nxgl_coord_t destwidth;
  nxgl_coord_t destheight;
  nxgl_coord_t deststride;
  nxgl_coord_t updwidth;
  nxgl_coord_t updheight;
  nxgl_coord_t width;
  nxgl_coord_t x;
  nxgl_coord_t y;
  unsigned int bytesperpixel;
  unsigned int maxwidth;
  size_t size;
  ssize_t nsent;
  uint8_t colorfmt;

  union
  {
    vnc_convert8_t bpp8;
    vnc_convert16_t bpp16;
    vnc_convert32_t bpp32;
  } convert;

  DEBUGASSERT(session != NULL);
  gvdbg("Updater running for Display %d\n", session->display);

  /* Loop, processing updates until we are asked to stop.
   * REVISIT: Probably need some kind of signal mechanism to wake up
   * vnc_remove_queue() in order to stop.  Or perhaps a special STOP
   * message in the queue?
   */

  while (session->state == VNCSERVER_RUNNING)
    {
      /* Get the next queued rectangle update.  This call will block until an
       * upate is available for the case where the update queue is empty.
       */

      srcrect = vnc_remove_queue(session);
      DEBUGASSERT(srcrect != NULL);

      /* Set up characteristics of the client pixel format to use on this
       * update.  These can change at any time if a SetPixelFormat is
       * received asynchronously.
       */

      bytesperpixel = (session->bpp + 7) >> 3;
      maxwidth      = CONFIG_VNCSERVER_UPDATE_BUFSIZE / bytesperpixel;

      /* Set up the color conversion */

      colorfmt = session->colorfmt;
      switch (colorfmt)
        {
          case FB_FMT_RGB8_222:
            convert.bpp8 = vnc_convert_rgb8_222;
            break;

          case FB_FMT_RGB8_332:
            convert.bpp8 = vnc_convert_rgb8_332;
            break;

          case FB_FMT_RGB16_555:
            convert.bpp16 = vnc_convert_rgb16_555;
            break;

          case FB_FMT_RGB16_565:
            convert.bpp16 = vnc_convert_rgb16_565;
            break;

          case FB_FMT_RGB32:
            convert.bpp32 = vnc_convert_rgb32_888;
            break;

          default:
            gdbg("ERROR: Unrecognized color format: %d\n", session->colorfmt);
            goto errout;
        }

      /* Get with width and height of the source and destination rectangles.
       * The source rectangle many be larger than the destination rectangle.
       * In that case, we will have to emit multiple rectangles.
       */

      DEBUGASSERT(srcrect->rect.pt1.x <= srcrect->rect.pt2.x);
      srcwidth = srcrect->rect.pt2.x - srcrect->rect.pt1.x + 1;

      DEBUGASSERT(srcrect->rect.pt1.y <= srcrect->rect.pt2.y);
      srcheight = srcrect->rect.pt2.y - srcrect->rect.pt1.y + 1;

      deststride = srcwidth * bytesperpixel;
      if (deststride > maxwidth)
        {
          deststride = maxwidth;
        }

      destwidth  = deststride / bytesperpixel;
      destheight = CONFIG_VNCSERVER_UPDATE_BUFSIZE / deststride;

      if (destheight > srcheight)
        {
          destheight = srcheight;
        }

      /* Format the rectangle header.  We may have to send several update
       * messages if the pre-allocated outbuf is smaller than the rectangle.
       * Each update contains a small "sub-rectangle" of the origin update.
       *
       * Loop until all sub-rectangles have been output.  Start with the
       * top row and transfer rectangles horizontally across each swath.
       * The height of the swath is destwidth (the last may be shorter).
       *
       * NOTE that the loop also terminates of the color format changes
       * asynchronously.
       */

      for (y = srcrect->rect.pt1.y;
           srcheight > 0 && colorfmt == session->colorfmt;
           srcheight -= updheight, y += updheight)
        {
          /* updheight = Height to update on this pass through the loop.
           * This will be destheight unless fewer than that number of rows
           * remain.
           */

          updheight = destheight;
          if (updheight > srcheight)
            {
              updheight = srcheight;
            }

          /* Loop until this horizontal swath has sent to the VNC client.
           * Start with the leftmost pixel and transfer rectangles
           * horizontally with width of destwidth until all srcwidth
           * columns have been transferred (the last rectangle may be
           * narrower).
           *
           * NOTE that the loop also terminates of the color format
           * changes asynchronously.
           */

          for (width = srcwidth, x = srcrect->rect.pt1.x;
               width > 0 && colorfmt == session->colorfmt;
               width -= updwidth, x += updwidth)
            {
              /* updwidth = Width to update on this pass through the loop.
               * This will be destwidth unless fewer than that number of
               * columns remain.
               */

              updwidth = destwidth;
              if (updwidth > width)
                {
                  updwidth = width;
                }

              /* Transfer the frame buffer data into the rectangle,
               * performing the necessary color conversions.
               */

              if (bytesperpixel == 1)
                {
                  size = vnc_copy8(session, y, x, updheight, updwidth,
                                   convert.bpp8);
                }
              else if (bytesperpixel == 2)
                {
                  size = vnc_copy16(session, y, x, updheight, updwidth,
                                    convert.bpp16);
                }
              else /* bytesperpixel == 4 */
                {
                  size = vnc_copy32(session, y, x, updheight, updwidth,
                                    convert.bpp32);
                }

              /* Format the FramebufferUpdate message */

              update          = (FAR struct rfb_framebufferupdate_s *)session->outbuf;
              update->msgtype = RFB_FBUPDATE_MSG;
              update->padding = 0;
              rfb_putbe16(update->nrect, 1);

              rfb_putbe16(update->rect[0].xpos, x);
              rfb_putbe16(update->rect[0].ypos, y);
              rfb_putbe16(update->rect[0].width, updwidth);
              rfb_putbe16(update->rect[0].height, updheight);
              rfb_putbe32(update->rect[0].encoding, RFB_ENCODING_RAW);

              DEBUGASSERT(size <= CONFIG_VNCSERVER_UPDATE_BUFSIZE);

              /* We are ready to send the update packet to the VNC client */

              size += SIZEOF_RFB_FRAMEBUFFERUPDATE_S(SIZEOF_RFB_RECTANGE_S(0));
              src   = session->outbuf;

              /* At the very last most, make certain that the color format
               * has not changed asynchronously.
               */

              if (colorfmt == session->colorfmt)
                {
                  /* Okay send until all of the bytes are out.  This may
                   * loop for the case where TCP write buffering is enabled
                   * and there are a limited number of IOBs available.
                   */

                  do
                    {
                      nsent = psock_send(&session->connect, src, size, 0);
                      if (nsent < 0)
                        {
                           gdbg("ERROR: Send FrameBufferUpdate failed: %d\n",
                                get_errno());
                          goto errout;
                        }

                      DEBUGASSERT(nsent <= size);
                      src  += nsent;
                      size -= nsent;
                    }
                  while (size > 0);
                }
            }
        }

      vnc_free_update(session, srcrect);
    }

errout:
  session->state = VNCSERVER_STOPPED;
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

int vnc_start_updater(FAR struct vnc_session_s *session)
{
  pthread_attr_t attr;
  struct sched_param param;
  int status;

  gvdbg("Starting updater for Display %d\n", session->display);

  /* Create thread that is gonna send rectangles to the client */

  session->state = VNCSERVER_RUNNING;

  DEBUGVERIFY(pthread_attr_init(&attr));
  DEBUGVERIFY(pthread_attr_setstacksize(&attr, CONFIG_VNCSERVER_UPDATER_STACKSIZE));

  param.sched_priority = CONFIG_VNCSERVER_UPDATER_PRIO;
  DEBUGVERIFY(pthread_attr_setschedparam(&attr, &param));

  status = pthread_create(&session->updater, &attr, vnc_updater,
                          (pthread_addr_t)session);
  if (status != 0)
    {
      return -status;
    }

  return OK;
}

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

int vnc_stop_updater(FAR struct vnc_session_s *session)
{
  pthread_addr_t result;
  int status;

  /* Is the update thread running? */

  if (session->state == VNCSERVER_RUNNING)
    {
      /* Yes.. ask it to please stop */

      session->state = VNCSERVER_STOPPING;

      /* Wait for the thread to comply with our request */

      status = pthread_join(session->updater, &result);
      if (status != 0)
        {
          return -status;
        }

      /* Return what the thread returned */

      return (int)((intptr_t)result);
    }

  /* Not running?  Just say we stopped the thread successfully. */

  return OK;
}

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
                         FAR const struct nxgl_rect_s *rect)
{
  FAR struct vnc_fbupdate_s *update;

  /* Make sure that the rectangle has a area */

  if (!nxgl_nullrect(rect))
    {
      /* Allocate an update structure... waiting if necessary */

      update = vnc_alloc_update(session);
      DEBUGASSERT(update != NULL);

      /* Copy the rectangle into the update structure */

      memcpy(&update->rect, rect, sizeof(struct nxgl_rect_s));

      /* Add the upate to the end of the update queue. */

      vnc_add_queue(session, update);
    }

  /* Since we ignore bad rectangles and wait for updata structures, there is
   * really no way a failure can occur.
   */

  return OK;
}
