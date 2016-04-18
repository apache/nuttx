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

#include "vnc_server.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_alloc_update
 *
 * Description:
 *  Allocate one update structure by taking it from the freelist.
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
  while (sem_wait(&session->freesem) < 0)
    {
      DEBUGASSERT(get_errno() == EINTR);
    }

  /* It is reserved.. go get it */

  update = (FAR struct vnc_fbupdate_s *)sq_remfirst(&session->updfree);
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

  /* Put the entry into the free list */

  sq_addlast((FAR sq_entry_t *)update, &session->updfree);

  /* Post the semaphore to indicate the availability of one more update */

  sem_post(&session->freesem);
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
  while (sem_wait(&session->queuesem) < 0)
    {
      DEBUGASSERT(get_errno() == EINTR);
    }

  /* It is reserved.. go get it */

  rect = (FAR struct vnc_fbupdate_s *)sq_remfirst(&session->updqueue);
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
 *   Standard pthread arguments.
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

  /* Put the entry into the list of queued rectangles. */

  sq_addlast((FAR sq_entry_t *)rect, &session->updqueue);

  /* Post the semaphore to indicate the availability of one more rectangle
   * in the queue.  This may wakeup the updater.
   */

  sem_post(&session->queuesem);
  DEBUGASSERT(session->queuesem.semcount <= CONFIG_VNCSERVER_NUPDATES);

  sched_unlock();
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
  FAR struct rfb_rectangle_s *destrect;
  FAR struct vnc_fbupdate_s *srcrect;
  FAR uint8_t *destdata;
  nxgl_coord_t width;
  nxgl_coord_t height;
  nxgl_coord_t stride;
  unsigned int bytesperpixel;
  unsigned int maxwidth;
  unsigned int maxheight;

  DEBUGASSERT(session != NULL);
  update       = (FAR struct rfb_framebufferupdate_s *)session->outbuf;
  destrect     = update->rect;
  destdata     = destrect->data;

  bytesperpixel = (session->bpp + 7) >> 3;
  maxwidth     = CONFIG_VNCSERVER_UPDATE_BUFSIZE / bytesperpixel;

  while (session->state == VNCSERVER_RUNNING)
    {
      /* Get the next queued rectangle update.  This call will block until an
       * upate is available for the case where the update queue is empty.
       */

      srcrect = vnc_remove_queue(session);
      DEBUGASSERT(srcrect != NULL);

      /* Format the rectangle header.  We may have to send several update
       * messages if the pre-allocated outbuf is smaller than the rectangle.
       */

      DEBUGASSERT(srcrect->rect.pt1.x <= srcrect->rect.pt2.x);
      width     = srcrect->rect.pt2.x - srcrect->rect.pt1.x + 1;
      stride    = width * bytesperpixel;

      DEBUGASSERT(srcrect->rect.pt1.y <= srcrect->rect.pt2.y);
      height    = srcrect->rect.pt2.y - srcrect->rect.pt1.y + 1;

      maxheight = CONFIG_VNCSERVER_UPDATE_BUFSIZE / stride;

      while (height > 0)
        {
          /* Determine the part of the rectangle that we can send on this
           * loop.
           */
#warning Missing logic

          /* Format the FramebufferUpdate message */

          update->msgtype = RFB_FBUPDATE_MSG;
          update->padding = 0;
          rfb_putbe16(update->nrect, 1);

          rfb_putbe16(destrect->xpos, srcrect->rect.pt1.x);
          rfb_putbe16(destrect->ypos, srcrect->rect.pt1.y);
          rfb_putbe16(destrect->width, width);
          rfb_putbe16(destrect->height, height);
          rfb_putbe16(destrect->encoding, RFB_ENCODING_RAW);

          /* Transfer the frame buffer data into the rectangle, performing
           * the necessary color conversions.
           */
#warning Missing logic
        }

      vnc_free_update(session, srcrect);
    }

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

  return -ENOSYS;
}
