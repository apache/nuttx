/****************************************************************************
 * wireless/ieee802154/mac802154_rxenable.c
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

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include "mac802154.h"
#include "mac802154_internal.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mac802154_rxenabletimeout(FAR void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_rxenabletimeout
 *
 * Description:
 *   Function registered with MAC timer that gets called via the work queue
 *   to handle a timeout for extracting the Association Response from the
 *   Coordinator.
 *
 ****************************************************************************/

static void mac802154_rxenabletimeout(FAR void *arg)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)arg;

  while (nxmutex_lock(&priv->lock) != 0);

  if (priv->curr_op != MAC802154_OP_RXENABLE)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  mac802154_rxdisable(priv);

  priv->curr_op = MAC802154_OP_NONE;
  nxsem_post(&priv->opsem);

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_req_rxenable
 *
 * Description:
 *   The MLME-RX-ENABLE.request primitive allows the next higher layer to
 *   request that the receiver is enable for a finite period of time.
 *   Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_rxenable callback.
 *
 ****************************************************************************/

int mac802154_req_rxenable(MACHANDLE mac,
                           FAR struct ieee802154_rxenable_req_s *req)
{
  FAR struct ieee802154_privmac_s * priv =
    (FAR struct ieee802154_privmac_s *)mac;
  int ret;

  /* If this is a Beacon-enabled network */

  if (priv->sfspec.sforder < 15)
    {
      return -EINVAL;
      goto errout_with_lock;
    }

  /* Non-beacon enabled network */

  else
    {
      if (req->rxon_dur > 0)
        {
          /* Get exclusive access to the operation semaphore.  This must
           * happen before getting exclusive access to the MAC struct or
           * else there could be a lockup condition. This would occur if
           * another thread is using the cmdtrans but needs access to the
           * MAC in order to unlock it.
           */

          ret = nxsem_wait_uninterruptible(&priv->opsem);
          if (ret < 0)
            {
              return ret;
            }

          priv->curr_op = MAC802154_OP_RXENABLE;

          /* Get exclusive access to the MAC */

          ret = nxmutex_lock(&priv->lock);
          if (ret < 0)
            {
              /* Should only fail if interrupted by a signal */

              wlwarn("WARNING: nxmutex_lock failed: %d\n", ret);

              nxsem_post(&priv->opsem);
              return ret;
            }

          mac802154_rxenable(priv);

          if (req->rxon_dur != 0xffffffff)
            {
              mac802154_timerstart(priv, req->rxon_dur,
                                   mac802154_rxenabletimeout);
            }
        }
      else
        {
          ret = nxmutex_lock(&priv->lock);
          if (ret < 0)
            {
              /* Should only fail if interrupted by a signal */

              wlwarn("WARNING: nxmutex_lock failed: %d\n", ret);
              return ret;
            }

          if (priv->curr_op != MAC802154_OP_RXENABLE)
            {
              ret = -EINVAL;
              goto errout_with_lock;
            }

          mac802154_timercancel(priv);
          mac802154_rxdisable(priv);

          priv->curr_op = MAC802154_OP_NONE;
          nxsem_post(&priv->opsem);
        }
    }

  nxmutex_unlock(&priv->lock);
  return OK;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}
