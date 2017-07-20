/****************************************************************************
 * wireless/ieee802154/mac802154_notif.c
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include "mac802154.h"
#include "mac802154_internal.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Public MAC Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_notif_free
 *
 * Description:
 *   When the MAC calls the registered callback, it passes a reference
 *   to a mac802154_notify_s structure.  This structure needs to be freed
 *   after the callback handler is done using it.
 *
 ****************************************************************************/

void mac802154_notif_free(MACHANDLE mac, FAR struct ieee802154_notif_s *notif)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;

  /* Lock the MAC */

  mac802154_lock(priv, false);

  /* Call the internal helper function to free the notification */

  mac802154_notif_free_locked(priv, notif);

  /* Unlock the MAC */

  mac802154_unlock(priv)
}

/****************************************************************************
 * Internal MAC Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_notifpool_init
 *
 * Description:
 *   This function initializes the notification structure pool. It allows the
 *   MAC to pass notifications and for the callee to free them when they are
 *   done using them, saving copying the data when passing.
 *
 ****************************************************************************/

void mac802154_notifpool_init(FAR struct ieee802154_privmac_s *priv)
{
  FAR struct mac802154_notif_s *pool = priv->notif_pool;
  int remaining = CONFIG_MAC802154_NNOTIF;

  priv->notif_free = NULL;
  while (remaining > 0)
    {
      FAR struct mac802154_notif_s *notif = pool;

      /* Add the next meta data structure from the pool to the list of
       * general structures.
       */

      notif->flink = priv->notif_free;
      priv->notif_free  = notif;

      /* Set up for the next structure from the pool */

      pool++;
      remaining--;
    }

  sem_init(&priv->notif_sem, 0, CONFIG_MAC802154_NNOTIF);
}

/****************************************************************************
 * Name: mac802154_notif_alloc
 *
 * Description:
 *   This function allocates a free notification structure from the free list
 *   to be used for passing to the registered notify callback. The callee software
 *   is responsible for freeing the notification structure after it is done using
 *   it via mac802154_notif_free.
 *
 * Assumptions:
 *   priv MAC struct is locked when calling.
 *
 * Notes:
 *   If any of the semaphore waits inside this function get interrupted, the
 *   function will release the MAC layer.  If this function returns -EINTR, the
 *   calling code should NOT release the MAC semaphore.
 *
 ****************************************************************************/

int mac802154_notif_alloc(FAR struct ieee802154_privmac_s *priv,
                          FAR struct ieee802154_notif_s **notif,
                          bool allow_interrupt)
{
  int ret;
  FAR struct mac802154_notif_s *privnotif;

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
      /* Unlock MAC so that other work can be done to free a notification */

      mac802154_unlock(priv)

      /* Take a count from the notification semaphore, waiting if necessary. We
       * only return from here with an error if we are allowing interruptions
       * and we received a signal */

      ret = mac802154_takesem(&priv->notif_sem, allow_interrupt);
      if (ret < 0)
        {
          /* MAC sem is already released */

          return -EINTR;
        }

      /* If we've taken a count from the semaphore, we have "reserved" the struct
       * but now we need to pop it off of the free list. We need to re-lock the
       * MAC in order to ensure this happens correctly.
       */

      ret = mac802154_lock(priv, allow_interrupt);
      if (ret < 0)
        {
          mac802154_givesem(&priv->notif_sem);
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
 * Name: mac802154_notif_free_locked
 *
 * Description:
 *   When the MAC calls the registered callback, it passes a reference
 *   to a mac802154_notify_s structure.  This structure needs to be freed
 *   after the callback handler is done using it.
 *
 *   Internal version that already has MAC locked
 *
 ****************************************************************************/

void mac802154_notif_free_locked(FAR struct ieee802154_privmac_s * priv,
                                 FAR struct ieee802154_notif_s *notif)
{
  FAR struct mac802154_notif_s *privnotif =
    (FAR struct mac802154_notif_s *)notif;

  /* We know how many clients have registered for notifications.  Each must
   * call mac802154_notif_free() before we can release the notification
   * resource.
   */

  if (privnotif->nclients < 2)
    {
      /* This is the free from the last notification */

      privnotif->flink    = priv->notif_free;
      priv->notif_free    = privnotif;
      privnotif->nclients = 0;

      mac802154_givesem(&priv->notif_sem);
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
 * Name: mac802154_notify
 *
 * Description:
 *   Notify every register MAC client.
 *
 ****************************************************************************/

void mac802154_notify(FAR struct ieee802154_privmac_s *priv,
                      FAR struct ieee802154_notif_s *notif)
{
  FAR struct mac802154_maccb_s *cb;
  FAR struct mac802154_notif_s *privnotif = (FAR struct mac802154_notif_s *)notif;

  /* Set the notification count so that the notification resources will be
   * preserved until the final notification.
   */

  privnotif->nclients = priv->nclients;

  /* Try to notify every registered MAC client */

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
