/****************************************************************************
 * drivers/wireless/ieee802154/xbee/xbee_notif.c
 *
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author: Anthony Merlino <anthony@vergeaero.com>
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

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include "xbee.h"
#include "xbee_mac.h"
#include "xbee_notif.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/xbee.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xbee_notif_free
 *
 * Description:
 *   When the XBee driver calls the registered callback, it passes a reference
 *   to a ieee802154_notif_s structure.  This structure needs to be freed
 *   after the callback handler is done using it.
 *
 ****************************************************************************/

void xbee_notif_free(XBEEHANDLE xbee, FAR struct ieee802154_notif_s *notif)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)xbee;

  /* Lock the MAC */

  xbee_lock(priv, false);

  /* Call the internal helper function to free the notification */

  xbee_notif_free_locked(priv, notif);

  /* Unlock the MAC */

  xbee_unlock(priv)
}

/****************************************************************************
 * Internal MAC Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xbee_notifpool_init
 *
 * Description:
 *   This function initializes the notification structure pool. It allows the
 *   XBee driver to pass notifications and for the callee to free them when they
 *   are done using them, saving copying the data when passing.
 *
 ****************************************************************************/

void xbee_notifpool_init(FAR struct xbee_priv_s *priv)
{
  FAR struct xbee_notif_s *pool = priv->notif_pool;
  int remaining = CONFIG_XBEE_NNOTIF;

  priv->notif_free = NULL;
  while (remaining > 0)
    {
      FAR struct xbee_notif_s *notif = pool;

      /* Add the next meta data structure from the pool to the list of
       * general structures.
       */

      notif->flink = priv->notif_free;
      priv->notif_free  = notif;

      /* Set up for the next structure from the pool */

      pool++;
      remaining--;
    }

  sem_init(&priv->notif_sem, 0, CONFIG_XBEE_NNOTIF);
}

/****************************************************************************
 * Name: xbee_notif_alloc
 *
 * Description:
 *   This function allocates a free notification structure from the free list
 *   to be used for passing to the registered notify callback. The callee software
 *   is responsible for freeing the notification structure after it is done using
 *   it via xbee_notif_free.
 *
 * Assumptions:
 *   priv XBee struct is locked when calling.
 *
 * Notes:
 *   If any of the semaphore waits inside this function get interrupted, the
 *   function will release the MAC layer.  If this function returns -EINTR, the
 *   calling code should NOT release the MAC semaphore.
 *
 ****************************************************************************/

int xbee_notif_alloc(FAR struct xbee_priv_s *priv,
                     FAR struct ieee802154_notif_s **notif,
                     bool allow_interrupt)
{
  int ret;
  FAR struct xbee_notif_s *privnotif;

  /* Try and take a count from the semaphore.  If this succeeds, we have
   * "reserved" the structure, but still need to unlink it from the free list.
   * The MAC is already locked, so there shouldn't be any other conflicting calls
   */

  ret = sem_trywait(&priv->notif_sem);

  if (ret == OK)
    {
      privnotif         = priv->notif_free;
      priv->notif_free  = privnotif->flink;
      privnotif->nclients = 0;
    }
  else
    {
      /* Unlock XBee driver so that other work can be done to free a notification */

      xbee_unlock(priv)

      /* Take a count from the notification semaphore, waiting if necessary. We
       * only return from here with an error if we are allowing interruptions
       * and we received a signal */

      ret = xbee_takesem(&priv->notif_sem, allow_interrupt);
      if (ret < 0)
        {
          /* MAC sem is already released */

          return -EINTR;
        }

      /* If we've taken a count from the semaphore, we have "reserved" the struct
       * but now we need to pop it off of the free list. We need to re-lock the
       * MAC in order to ensure this happens correctly.
       */

      ret = xbee_lock(priv, allow_interrupt);
      if (ret < 0)
        {
          xbee_givesem(&priv->notif_sem);
          return -EINTR;
        }

      /* We can now safely unlink the next free structure from the free list */

      privnotif           = priv->notif_free;
      priv->notif_free    = privnotif->flink;
      privnotif->nclients = 0;
    }

  *notif = (FAR struct ieee802154_notif_s *)privnotif;

  return OK;
}

/****************************************************************************
 * Name: xbee_notif_free_locked
 *
 * Description:
 *   When the XBee driver calls the registered callback, it passes a reference
 *   to a ieee802154_notif_s structure.  This structure needs to be freed
 *   after the callback handler is done using it.
 *
 *   Internal version that already has XBee driver locked
 *
 ****************************************************************************/

void xbee_notif_free_locked(FAR struct xbee_priv_s * priv,
                            FAR struct ieee802154_notif_s *notif)
{
  FAR struct xbee_notif_s *privnotif =
    (FAR struct xbee_notif_s *)notif;

  /* We know how many clients have registered for notifications.  Each must
   * call xbee_notif_free() before we can release the notification
   * resource.
   */

  if (privnotif->nclients < 2)
    {
      /* This is the free from the last notification */

      privnotif->flink    = priv->notif_free;
      priv->notif_free    = privnotif;
      privnotif->nclients = 0;

      xbee_givesem(&priv->notif_sem);
    }
  else
    {
      /* More calls are expected.  Decrement the count of expected calls
       * and preserve the notification resources.
       */

      privnotif->nclients--;
    }
}

/****************************************************************************
 * Name: xbee_notify
 *
 * Description:
 *   Notify every register XBee MAC client.
 *
 ****************************************************************************/

void xbee_notify(FAR struct xbee_priv_s *priv,
                 FAR struct ieee802154_notif_s *notif)
{
  FAR struct xbee_maccb_s *cb;
  FAR struct xbee_notif_s *privnotif = (FAR struct xbee_notif_s *)notif;

  /* Set the notification count so that the notification resources will be
   * preserved until the final notification.
   */

  privnotif->nclients = priv->nclients;

  /* Try to notify every registered XBee MAC client */

  for (cb = priv->cb; cb != NULL; cb = cb->flink)
    {
      /* Does this client want notifications? */

      if (cb->notify != NULL)
        {
          /* Yes.. Notify */

          cb->notify(cb, notif);
        }
    }
}
