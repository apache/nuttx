/****************************************************************************
 * net/devif/devif_callback.c
 *
 *   Copyright (C) 2008-2009, 2015 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET)

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"
#include "devif/devif.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct devif_callback_s g_cbprealloc[CONFIG_NET_NACTIVESOCKETS];
static FAR struct devif_callback_s *g_cbfreelist = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devif_callback_free
 *
 * Description:
 *   Return a callback container to the free list.
 *
 * Assumptions:
 *   This function is called with the network locked.
 *
 ****************************************************************************/

static void devif_callback_free(FAR struct net_driver_s *dev,
                                FAR struct devif_callback_s *cb,
                                FAR struct devif_callback_s **list)
{
  FAR struct devif_callback_s *prev;
  FAR struct devif_callback_s *curr;

  if (cb)
    {
      net_lock();

#ifdef CONFIG_DEBUG_FEATURES
      /* Check for double freed callbacks */

      curr = g_cbfreelist;

      while (curr != NULL)
        {
          DEBUGASSERT(cb != curr);
          curr = curr->nxtconn;
        }
#endif

      /* Remove the callback structure from the device notification list if
       * it is supposed to be in the device notification list.
       */

      if (dev != NULL)
        {
          /* Find the callback structure in the device event list */

          for (prev = NULL, curr = dev->d_devcb;
               curr != NULL && curr != cb;
               prev = curr, curr = curr->nxtdev);

          /* Remove the structure from the device event list */

          DEBUGASSERT(curr);
          if (curr != NULL)
            {
              if (prev)
                {
                  prev->nxtdev = cb->nxtdev;
                }
              else
                {
                  dev->d_devcb = cb->nxtdev;
                }
            }
        }

      /* Remove the callback structure from the data notification list if
       * it is supposed to be in the data notification list.
       */

      if (list)
        {
          /* Find the callback structure in the connection event list */

          for (prev = NULL, curr = *list;
               curr && curr != cb;
               prev = curr, curr = curr->nxtconn);

          /* Remove the structure from the connection event list */

          DEBUGASSERT(curr);
          if (curr)
            {
              if (prev)
                {
                  prev->nxtconn = cb->nxtconn;
                }
              else
                {
                  *list = cb->nxtconn;
                }
            }
        }

      /* Put the structure into the free list */

      cb->nxtconn  = g_cbfreelist;
      cb->nxtdev   = NULL;
      g_cbfreelist = cb;
      net_unlock();
    }
}

/****************************************************************************
 * Name: devif_event_trigger
 *
 * Description:
 *   Return true if the current set of events should trigger a callback to
 *   occur.
 *
 * Input Parameters:
 *   events   - The set of events that has occurred.
 *   triggers - The set of events that will trigger a callback.
 *
 ****************************************************************************/

static bool devif_event_trigger(uint16_t events, uint16_t triggers)
{
  /* The events are divided into a set of individual bits that may be ORed
   * together PLUS a field that encodes a single poll event.
   *
   * First check if any of the individual event bits will trigger the
   * callback.
   */

  if ((events & triggers & ~DEVPOLL_MASK) != 0)
    {
      return true;
    }

  /* No... check the encoded device event. */

  if ((events & DEVPOLL_MASK) == (triggers & DEVPOLL_MASK))
    {
      return true;
    }

  /* No.. this event set will not generate the callback */

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devif_callback_init
 *
 * Description:
 *   Configure the pre-allocated callback structures into a free list.
 *
 * Assumptions:
 *   Called early in the initialization sequence so that no special
 *   protection is required.
 *
 ****************************************************************************/

void devif_callback_init(void)
{
  int i;

  for (i = 0; i < CONFIG_NET_NACTIVESOCKETS; i++)
    {
      g_cbprealloc[i].nxtconn = g_cbfreelist;
      g_cbfreelist = &g_cbprealloc[i];
    }
}

/****************************************************************************
 * Name: devif_callback_alloc
 *
 * Description:
 *   Allocate a callback container from the free list.
 *
 *   If dev is non-NULL, then this function verifies that the device
 *   reference is still  valid and that the device is still UP status.  If
 *   those conditions are not true, this function will fail to allocate the
 *   callback.
 *
 * Assumptions:
 *   This function is called with the network locked.
 *
 ****************************************************************************/

FAR struct devif_callback_s *
  devif_callback_alloc(FAR struct net_driver_s *dev,
                       FAR struct devif_callback_s **list)
{
  FAR struct devif_callback_s *ret;

  /* Check  the head of the free list */

  net_lock();
  ret  = g_cbfreelist;
  if (ret)
    {
      /* Remove the next instance from the head of the free list */

      g_cbfreelist = ret->nxtconn;
      memset(ret, 0, sizeof(struct devif_callback_s));

      /* Add the newly allocated instance to the head of the device event
       * list.
       */

      if (dev)
        {
          /* Verify that the device pointer is valid, i.e., that it still
           * points to a registered network device and also that the network
           * device in in the UP state.
           *
           * And if it does, should that device also not be in the UP state?
           */

          if (!netdev_verify(dev) && (dev->d_flags & IFF_UP) != 0)
            {
              /* No.. release the callback structure and fail */

              devif_callback_free(NULL, NULL, list);
              net_unlock();
              return NULL;
            }

          ret->nxtdev  = dev->d_devcb;
          dev->d_devcb = ret;
        }

      /* Add the newly allocated instance to the head of the specified list */

      if (list)
        {
           ret->nxtconn = *list;
           *list = ret;
        }
    }
#ifdef CONFIG_DEBUG_FEATURES
  else
    {
      nerr("ERROR: Failed to allocate callback\n");
    }
#endif

  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: devif_conn_callback_free
 *
 * Description:
 *   Return a connection/port callback container to the free list.
 *
 *   This function is just a front-end for devif_callback_free().  If the
 *   dev argument is non-NULL, it will verify that the device reference is
 *   still valid before attempting to free the callback structure.  A
 *   non-NULL list pointer is assumed to be valid in any case.
 *
 *   The callback structure will be freed in any event.
 *
 * Assumptions:
 *   This function is called with the network locked.
 *
 ****************************************************************************/

void devif_conn_callback_free(FAR struct net_driver_s *dev,
                              FAR struct devif_callback_s *cb,
                              FAR struct devif_callback_s **list)
{
  /* Check if the device pointer is still valid.  It could be invalid if, for
   * example, the device were unregistered between the time when the callback
   * was allocated and the time when the callback was freed.
   */

  if (dev && !netdev_verify(dev))
    {
      /* The device reference is longer valid */

      dev = NULL;
    }

  /* Then free the callback */

  devif_callback_free(dev, cb, list);
}

/****************************************************************************
 * Name: devif_dev_callback_free
 *
 * Description:
 *   Return a device callback container to the free list.
 *
 *   This function is just a front-end for devif_callback_free().  If the
 *   de argument is non-NULL, it will verify that the device reference is
 *   still valid before attempting to free the callback structure.  It
 *   differs from devif_conn_callback_free in that connection/port-related
 *   connections are also associated with the device and, hence, also will
 *   not be valid if the device pointer is not valid.
 *
 *   The callback structure will be freed in any event.
 *
 * Assumptions:
 *   This function is called with the network locked.
 *
 ****************************************************************************/

void devif_dev_callback_free(FAR struct net_driver_s *dev,
                             FAR struct devif_callback_s *cb)
{
  FAR struct devif_callback_s **list;

  /* Check if the device pointer is still valid.  It could be invalid if, for
   * example, the device were unregistered between the time when the callback
   * was allocated and the time when the callback was freed.
   */

  if (dev != NULL && netdev_verify(dev))
    {
      /* The device reference is valid.. the use the list pointer in the
       * device structure as well.
       */

      list = &dev->d_conncb;
    }
  else
    {
      /* The device reference is longer valid */

      dev  = NULL;
      list = NULL;
    }

  /* Then free the callback */

  devif_callback_free(dev, cb, list);
}

/****************************************************************************
 * Name: devif_conn_event
 *
 * Description:
 *   Execute a list of callbacks using the packet event chain.
 *
 * Input Parameters:
 *   dev - The network device state structure associated with the network
 *     device that initiated the callback event.
 *   pvconn - Holds a reference to the TCP connection structure or the UDP
 *     port structure.  May be NULL if the even is not related to a TCP
 *     connection or UDP port.
 *   flags - The bit set of events to be notified.
 *   list - The list to traverse in performing the notifications
 *
 * Returned Value:
 *   The updated flags as modified by the callback functions.
 *
 * Assumptions:
 *   This function is called with the network locked.
 *
 ****************************************************************************/

uint16_t devif_conn_event(FAR struct net_driver_s *dev, void *pvconn,
                          uint16_t flags, FAR struct devif_callback_s *list)
{
  FAR struct devif_callback_s *next;

  /* Loop for each callback in the list and while there are still events
   * set in the flags set.
   */

  net_lock();
  while (list && flags)
    {
      /* Save the pointer to the next callback in the lists.  This is done
       * because the callback action might delete the entry pointed to by
       * list.
       */

      next = list->nxtconn;

      /* Check if this callback handles any of the events in the flag set */

      if (list->event != NULL && devif_event_trigger(flags, list->flags))
        {
          /* Yes.. perform the callback.  Actions perform by the callback
           * may delete the current list entry or add a new list entry to
           * beginning of the list (which will be ignored on this pass)
           */

          flags = list->event(dev, pvconn, list->priv, flags);
        }

      /* Set up for the next time through the loop */

      list = next;
    }

  net_unlock();
  return flags;
}

/****************************************************************************
 * Name: devif_dev_event
 *
 * Description:
 *   Execute a list of callbacks using the device event chain.
 *
 * Input Parameters:
 *   dev - The network device state structure associated with the network
 *     device that initiated the callback event.
 *   pvconn - Holds a reference to the TCP connection structure or the UDP
 *     port structure.  May be NULL if the even is not related to a TCP
 *     connection or UDP port.
 *   flags - The bit set of events to be notified.
 *
 * Returned Value:
 *   The updated flags as modified by the callback functions.
 *
 * Assumptions:
 *   This function is called with the network locked.
 *
 ****************************************************************************/

uint16_t devif_dev_event(FAR struct net_driver_s *dev, void *pvconn,
                         uint16_t flags)
{
  FAR struct devif_callback_s *cb;
  FAR struct devif_callback_s *next;

  /* Loop for each callback in the list and while there are still events
   * set in the flags set.
   */

  net_lock();
  for (cb = dev->d_devcb; cb != NULL && flags != 0; cb = next)
    {
      /* Save the pointer to the next callback in the lists.  This is done
       * because the callback action might delete the entry pointed to by
       * list.
       */

      next = cb->nxtdev;

      /* Check if this callback handles any of the events in the flag set */

      if (cb->event != NULL && devif_event_trigger(flags, cb->flags))
        {
          /* Yes.. perform the callback.  Actions perform by the callback
           * may delete the current list entry or add a new list entry to
           * beginning of the list (which will be ignored on this pass)
           */

          flags = cb->event(dev, pvconn, cb->priv, flags);
        }

      /* Set up for the next time through the loop */

      cb = next;
    }

  net_unlock();
  return flags;
}

#endif /* CONFIG_NET */
