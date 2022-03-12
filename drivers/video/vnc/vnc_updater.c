/****************************************************************************
 * drivers/video/vnc/vnc_updater.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <sched.h>
#include <pthread.h>
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

#include "vnc_server.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef VNCSERVER_SEM_DEBUG          /* Define to dump queue/semaphore state */
#undef VNCSERVER_SEM_DEBUG_SILENT   /* Define to dump only suspicious conditions */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef VNCSERVER_SEM_DEBUG
static sem_t g_errsem = SEM_INITIALIZER(1);
#endif

/* A rectangle represent the entire local framebuffer */

static const struct fb_area_s g_wholescreen =
{
  0, 0, CONFIG_VNCSERVER_SCREENWIDTH, CONFIG_VNCSERVER_SCREENHEIGHT
};

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
  int nqueued;
  int nfree;
  int freesem;
  int queuesem;
  int freecount;
  int queuecount;
  int freewaiting;
  int queuewaiting;
  int ret;

  ret = nxsem_wait_uninterruptible(&g_errsem);
  if (ret < 0)
    {
      /* Should happen only on task cancellation */

      DEBUGASSERT(ret == -ECANCELED);
      return;
    }

  /* Count structures in the list */

  nqueued      = sq_count(&session->updqueue);
  nfree        = sq_count(&session->updfree);

  freesem      = session->freesem.semcount;
  queuesem     = session->queuesem.semcount;

  freecount    = freesem  > 0 ? freesem   : 0;
  queuecount   = queuesem > 0 ? queuesem  : 0;

  freewaiting  = freesem  < 0 ? -freesem  : 0;
  queuewaiting = queuesem < 0 ? -queuesem : 0;

#ifdef VNCSERVER_SEM_DEBUG_SILENT
  /* This dumps most false alarms in the case where:
   *
   * - Updater was waiting on a semaphore (count is -1)
   * - New update added to the queue (queue count is 1)
   * - queuesem posted.  Wakes up Updater and the count is 0.
   */

  if ((nqueued + nfree) != (freecount + queuecount))
#endif
    {
      syslog(LOG_INFO, "FREESEM DEBUG:    %s\n", msg);
      syslog(LOG_INFO, "  Free list:\n");
      syslog(LOG_INFO, "    semcount:     %d\n", freecount);
      syslog(LOG_INFO, "    queued nodes: %u\n", nfree);
      syslog(LOG_INFO, "    waiting:      %u\n", freewaiting);
      syslog(LOG_INFO, "  Qeued Updates:\n");
      syslog(LOG_INFO, "    semcount:     %d\n", queuecount);
      syslog(LOG_INFO, "    queued nodes: %u\n", nqueued);
      syslog(LOG_INFO, "    waiting:      %u\n", queuewaiting);
      syslog(LOG_INFO, "  Unqueued:       %u\n", unattached);
    }

  nxsem_post(&g_errsem);
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
   * that the sq_remfirst() and the successful return from nxsem_wait are
   * atomic.  Of course, the scheduler will be unlocked while we wait.
   */

  sched_lock();
  vnc_sem_debug(session, "Before alloc", 0);

  nxsem_wait_uninterruptible(&session->freesem);

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
   * that the sq_addlast() and the nxsem_post() are atomic.
   */

  sched_lock();
  vnc_sem_debug(session, "Before free", 1);

  /* Put the entry into the free list */

  sq_addlast((FAR sq_entry_t *)update, &session->updfree);

  /* Post the semaphore to indicate the availability of one more update */

  nxsem_post(&session->freesem);

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
 *   A structure pointer should always be returned. If the return value is
 *   NULL, that means connection lost, no frame update request anymore.
 *
 ****************************************************************************/

static FAR struct vnc_fbupdate_s *
vnc_remove_queue(FAR struct vnc_session_s *session)
{
  FAR struct vnc_fbupdate_s *rect;

  /* Reserve one element from the list of queued rectangle.  Lock the
   * scheduler to assure that the sq_remfirst() and the successful return
   * from nxsem_wait are atomic.  Of course, the scheduler will be unlocked
   * while we wait.
   */

  sched_lock();
  vnc_sem_debug(session, "Before remove", 0);

  nxsem_wait_uninterruptible(&session->queuesem);

  /* It is reserved.. go get it */

  rect = (FAR struct vnc_fbupdate_s *)sq_remfirst(&session->updqueue);

  vnc_sem_debug(session, "After remove", 0);

  if (NULL == rect)
    {
      goto errout;
    }

  /* Check if we just removed the whole screen update from the queue */

  if (session->nwhupd > 0 && rect->whupd)
    {
      session->nwhupd--;
      updinfo("Whole screen update: nwhupd=%d\n", session->nwhupd);
    }

errout:
  sched_unlock();
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
  /* Lock the scheduler to assure that the sq_addlast() and the nxsem_post()
   * are atomic.
   */

  sched_lock();
  vnc_sem_debug(session, "Before add", 1);

  /* Put the entry into the list of queued rectangles. */

  sq_addlast((FAR sq_entry_t *)rect, &session->updqueue);

  /* Post the semaphore to indicate the availability of one more rectangle
   * in the queue.  This may wakeup the updater.
   */

  nxsem_post(&session->queuesem);

  vnc_sem_debug(session, "After add", 0);
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
  FAR struct vnc_fbupdate_s *srcrect;
  int ret;
#ifdef CONFIG_FB_SYNC
  int val;
#endif

  DEBUGASSERT(session != NULL);
  ginfo("Updater running for Display %d\n", session->display);

  /* Loop, processing updates until we are asked to stop.
   * REVISIT: Probably need some kind of signal mechanism to wake up
   * vnc_remove_queue() in order to stop.  Or perhaps a special STOP
   * message in the queue?
   */

  while (session->state == VNCSERVER_RUNNING)
    {
      /* Get the next queued rectangle update.  This call will block until an
       * update is available for the case where the update queue is empty.
       */

      srcrect = vnc_remove_queue(session);

      /* If connect lost, exit this updater loop */

      if (session->state == VNCSERVER_STOPPING)
        {
          break;
        }

      DEBUGASSERT(srcrect != NULL);

      updinfo("Dequeued {(%d, %d),(%d, %d)}\n",
              srcrect->rect.x, srcrect->rect.y,
              srcrect->rect.w, srcrect->rect.h);

      /* Attempt to use RRE encoding */

      ret = vnc_rre(session, &srcrect->rect);
      if (ret == 0)
        {
          /* Perform the framebuffer update using the default RAW encoding */

          ret = vnc_raw(session, &srcrect->rect);
        }

      /* Release the update structure */

      vnc_free_update(session, srcrect);

#ifdef CONFIG_FB_SYNC
      ret = nxsem_get_value(&session->vsyncsem, &val);

      if (ret < 0)
        {
          gerr("ERROR: Get vsync sem failed: %d\n", ret);
          break;
        }

      if (sq_count(&session->updqueue) == 0 && val <= 0)
        {
          nxsem_post(&session->vsyncsem);
        }
#endif

      /* Break out and terminate the server if the encoding failed */

      if (ret < 0)
        {
          gerr("ERROR: Encoding failed: %d\n", ret);
          break;
        }
    }

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

  ginfo("Starting updater for Display %d\n", session->display);

  /* Create thread that is gonna send rectangles to the client */

  session->state = VNCSERVER_RUNNING;

  DEBUGVERIFY(pthread_attr_init(&attr));
  DEBUGVERIFY(pthread_attr_setstacksize(&attr,
                                        CONFIG_VNCSERVER_UPDATER_STACKSIZE));

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

      /* Notify updater thread we stopped */

      nxsem_post(&session->queuesem);

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
 *   change  - True: Frame buffer data has changed
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int vnc_update_rectangle(FAR struct vnc_session_s *session,
                         FAR const struct fb_area_s *rect, bool change)
{
  FAR struct vnc_fbupdate_s *update;
  struct fb_area_s intersection;
  bool whupd;

  intersection.x = rect->x;
  intersection.y = rect->y;

  /* Clip rectangle to the screen dimensions */

  if (rect->w < g_wholescreen.w)
    {
      intersection.w = rect->w;
    }
  else
    {
      intersection.w = g_wholescreen.w;
    }

  if (rect->h < g_wholescreen.h)
    {
      intersection.h = rect->h;
    }
  else
    {
      intersection.h = g_wholescreen.h;
    }

  /* Make sure that the clipped rectangle has an area */

  if (intersection.w && intersection.h)
    {
      /* Check for a whole screen update.  The RealVNC client sends a lot
       * of these (especially when it is confused)
       */

      whupd = (memcmp(&intersection, &g_wholescreen,
                      sizeof(struct fb_area_s)) == 0);

      /* Ignore any client update requests if there have been no changes to
       * the framebuffer since the last whole screen update.
       */

      sched_lock();
      if (!change && !session->change)
        {
          /* No.. ignore the client update.  We have nothing new to report. */

          sched_unlock();
          return OK;
        }

      /* Ignore all updates if there is a queued whole screen update */

      if (session->nwhupd == 0)
        {
          /* No whole screen updates in the queue.  Is this a new whole
           * screen update?
           */

          if (whupd)
            {
              /* Yes.. Discard all of the previously queued updates */

              FAR struct vnc_fbupdate_s *curr;
              FAR struct vnc_fbupdate_s *next;

              updinfo("New whole screen update...\n");

              curr = (FAR struct vnc_fbupdate_s *)session->updqueue.head;
              sq_init(&session->updqueue);
              nxsem_reset(&session->queuesem, 0);

              for (; curr != NULL; curr = next)
                {
                  next = curr->flink;
                  vnc_free_update(session, curr);
                }

              /* One whole screen update will be queued.  There have been
               * no frame buffer data changes since this update was queued.
               */

              session->nwhupd = 1;
              session->change = false;
            }
          else
            {
              /* We are not updating the whole screen.  Remember if this
               * update (OR a preceding update) was due to a data change.
               */

              session->change |= change;
            }

          /* Allocate an update structure... waiting if necessary */

          update = vnc_alloc_update(session);
          DEBUGASSERT(update != NULL);

          /* Copy the clipped rectangle into the update structure */

          update->whupd = whupd;
          memcpy(&update->rect, &intersection, sizeof(intersection));

          /* Add the update to the end of the update queue. */

          vnc_add_queue(session, update);

          updinfo("Queued {(%d, %d),(%d, %d)}\n",
                  intersection.x, intersection.y,
                  intersection.w, intersection.h);
        }

      sched_unlock();
    }

  /* Since we ignore bad rectangles and wait for update structures, there is
   * really no way a failure can occur.
   */

  return OK;
}
