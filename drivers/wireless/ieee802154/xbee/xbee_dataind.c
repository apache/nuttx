/****************************************************************************
 * drivers/wireless/ieee802154/xbee/xbee_dataind.c
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

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/xbee.h>

#include "xbee.h"
#include "xbee_mac.h"
#include "xbee_notif.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xbee_dataind_free
 *
 * Description:
 *   When the XBee driver calls the registered rxframe function, it passes a
 *   reference to a ieee802154_data_ind_s structure.  This structure needs to be
 *   freed after the handler is done using it.
 *
 ****************************************************************************/

void xbee_dataind_free(XBEEHANDLE xbee, FAR struct ieee802154_data_ind_s *dataind)
{
  FAR struct xbee_priv_s *priv = (FAR struct xbee_priv_s *)xbee;
  FAR struct xbee_dataind_s *privind = (FAR struct xbee_dataind_s *)dataind;

  xbee_lock(priv, false);

  privind->flink      = priv->dataind_free;
  priv->dataind_free  = privind;

  xbee_givesem(&priv->dataind_sem);

  xbee_unlock(priv);
}

/****************************************************************************
 * Name: xbee_datatindpool_init
 *
 * Description:
 *   This function initializes the data indication structure pool. It allows the
 *   XBee driver to pass received frames with meta data to the callee, where they
 *   can free them when the calle is done using them, saving copying the data
 *   when passing.
 *
 ****************************************************************************/

void xbee_dataindpool_init(FAR struct xbee_priv_s *priv)
{
  FAR struct xbee_dataind_s *pool = priv->dataind_pool;
  int remaining = CONFIG_XBEE_NDATAIND;

  priv->dataind_free = NULL;
  while (remaining > 0)
    {
      FAR struct xbee_dataind_s *dataind = pool;

      /* Add the next meta data structure from the pool to the list of
       * general structures.
       */

      dataind->flink = priv->dataind_free;
      priv->dataind_free  = dataind;

      /* Set up for the next structure from the pool */

      pool++;
      remaining--;
    }

  sem_init(&priv->dataind_sem, 0, CONFIG_XBEE_NDATAIND);
}

/****************************************************************************
 * Name: xbee_dataind_alloc
 *
 * Description:
 *   This function allocates a free data indication structure from the free list
 *   to be used for passing to the registered rxframe callback. The callee software
 *   is responsible for freeing the data indication structure after it is done using
 *   it via xbee_data_ind_sfree.
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

int xbee_dataind_alloc(FAR struct xbee_priv_s *priv,
                       FAR struct ieee802154_data_ind_s **dataind,
                       bool allow_interrupt)
{
  int ret;
  FAR struct xbee_dataind_s *privind;

  /* Try and take a count from the semaphore.  If this succeeds, we have
   * "reserved" the structure, but still need to unlink it from the free list.
   * The MAC is already locked, so there shouldn't be any other conflicting calls
   */

  ret = sem_trywait(&priv->dataind_sem);

  if (ret == OK)
    {
      privind            = priv->dataind_free;
      priv->dataind_free = privind->flink;
    }
  else
    {
      wlinfo("waiting for dataind to be free\n");

      /* Unlock XBee driver so that other work can be done to free a data indication */

      xbee_unlock(priv);

      /* Take a count from the indication semaphore, waiting if necessary. We
       * only return from here with an error if we are allowing interruptions
       * and we received a signal */

      ret = xbee_takesem(&priv->dataind_sem, allow_interrupt);
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
          xbee_givesem(&priv->dataind_sem);
          return -EINTR;
        }

      /* We can now safely unlink the next free structure from the free list */

      privind             = priv->dataind_free;
      priv->dataind_free  = privind->flink;

      wlinfo("dataind allocated\n");
    }

  *dataind = (FAR struct ieee802154_data_ind_s *)privind;

  return OK;
}
