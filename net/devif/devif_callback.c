/****************************************************************************
 * net/devif/devif_callback.c
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
#if defined(CONFIG_NET)

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"
#include "devif/devif.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_NET_ALLOC_CONNS
static struct devif_callback_s g_cbprealloc[CONFIG_NET_NACTIVESOCKETS];
#endif
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
                                FAR struct devif_callback_s **list_head,
                                FAR struct devif_callback_s **list_tail)
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
               prev = curr, curr = curr->nxtdev)
            {
            }

          /* Remove the structure from the device event list */

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

      if (list_head)
        {
          prev = cb->prevconn;

          /* Remove the structure from the connection event list */

          if (prev)
            {
              /* The item to be removed is not in the head. */

              prev->nxtconn = cb->nxtconn;

              if (cb->nxtconn)
                {
                  /* The item to be removed is not in the tail. */

                  cb->nxtconn->prevconn = prev;
                }
            }
          else
            {
              /* The item to be removed is in the head. */

              *list_head = cb->nxtconn;

              if (cb->nxtconn)
                {
                  /* There are more items besides the head item. */

                  cb->nxtconn->prevconn = NULL;
                }
            }

          if (!cb->nxtconn)
            {
              /* If the tail item is being removed,
               * update the tail pointer.
               */

              DEBUGASSERT(list_tail);
              *list_tail = prev;
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
      return (triggers & DEVPOLL_MASK) != 0;
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
#ifndef CONFIG_NET_ALLOC_CONNS
  int i;

  for (i = 0; i < CONFIG_NET_NACTIVESOCKETS; i++)
    {
      g_cbprealloc[i].nxtconn = g_cbfreelist;
      g_cbfreelist = &g_cbprealloc[i];
    }
#endif
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
                       FAR struct devif_callback_s **list_head,
                       FAR struct devif_callback_s **list_tail)
{
  FAR struct devif_callback_s *ret;
#ifdef CONFIG_NET_ALLOC_CONNS
  int i;
#endif

  net_lock();

  /* Verify that the device pointer is valid, i.e., that it still
   * points to a registered network device and also that the network
   * device in the UP state.
   */

  /* Note: dev->d_flags may be asynchronously changed by netdev_ifdown()
   * (in net/netdev/netdev_ioctl.c). Nevertheless, net_lock() / net_unlock()
   * are not required in netdev_ifdown() to prevent dev->d_flags from
   * asynchronous change here. There is not an issue because net_lock() and
   * net_unlock() present inside of devif_dev_event(). That should be enough
   * to de-allocate connection callbacks reliably on NETDEV_DOWN event.
   */

  if (dev && !netdev_verify(dev) && (dev->d_flags & IFF_UP) != 0)
    {
      net_unlock();
      return NULL;
    }

  /* Allocate the callback entry from heap */

#ifdef CONFIG_NET_ALLOC_CONNS
  if (g_cbfreelist == NULL)
    {
      ret = kmm_zalloc(sizeof(struct devif_callback_s) *
                       CONFIG_NET_NACTIVESOCKETS);
      if (ret != NULL)
        {
          for (i = 0; i < CONFIG_NET_NACTIVESOCKETS; i++)
            {
              ret[i].nxtconn = g_cbfreelist;
              g_cbfreelist = &ret[i];
            }
        }
    }
#endif

  /* Check the head of the free list */

  ret = g_cbfreelist;
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
          ret->nxtdev  = dev->d_devcb;
          dev->d_devcb = ret;
        }

      /* Add the newly allocated instance to the tail of the specified list */

      if (list_head && list_tail)
        {
          ret->nxtconn = NULL;
          ret->prevconn = *list_tail;

          if (*list_tail)
            {
              /* If the list is not empty, add the item to the tail. */

              (*list_tail)->nxtconn = ret;
            }
          else
            {
              /* If the list is empty, add the first item to the list. */

              *list_head = ret;
            }

          /* Update the tail pointer */

          *list_tail = ret;
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
                              FAR struct devif_callback_s **list_head,
                              FAR struct devif_callback_s **list_tail)
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

  devif_callback_free(dev, cb, list_head, list_tail);
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
  FAR struct devif_callback_s **list_head;
  FAR struct devif_callback_s **list_tail;

  /* Check if the device pointer is still valid.  It could be invalid if, for
   * example, the device were unregistered between the time when the callback
   * was allocated and the time when the callback was freed.
   */

  if (dev != NULL && netdev_verify(dev))
    {
      /* The device reference is valid. Then use the list pointer in the
       * device structure as well.
       */

      list_head = &dev->d_conncb;
      list_tail = &dev->d_conncb_tail;
    }
  else
    {
      /* The device reference is longer valid */

      dev  = NULL;
      list_head = NULL;
      list_tail = NULL;
    }

  /* Then free the callback */

  devif_callback_free(dev, cb, list_head, list_tail);
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
 *     port structure. It can be NULL if the event is not related to a TCP
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
 *     port structure. It can be NULL if the event is not related to a TCP
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
    }

  net_unlock();
  return flags;
}

#endif /* CONFIG_NET */
