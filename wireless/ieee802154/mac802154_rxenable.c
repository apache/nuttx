/****************************************************************************
 * wireless/ieee802154/mac802154_rxenable.c
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

  while (mac802154_lock(priv, true) != 0);

  if (priv->curr_op != MAC802154_OP_RXENABLE)
    {
      mac802154_unlock(priv);
      return;
    }

  mac802154_rxdisable(priv);

  priv->curr_op = MAC802154_OP_NONE;
  mac802154_givesem(&priv->opsem);

  mac802154_unlock(priv)
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
      goto errout_with_sem;
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

          ret = mac802154_takesem(&priv->opsem, true);
          if (ret < 0)
            {
              return ret;
            }

          priv->curr_op = MAC802154_OP_RXENABLE;

          /* Get exclusive access to the MAC */

          ret = mac802154_lock(priv, true);
          if (ret < 0)
            {
              /* Should only fail if interrupted by a signal */

              wlwarn("WARNING: mac802154_takesem failed: %d\n", ret);

              mac802154_givesem(&priv->opsem);
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
          ret = mac802154_lock(priv, true);
          if (ret < 0)
            {
              /* Should only fail if interrupted by a signal */

              wlwarn("WARNING: mac802154_takesem failed: %d\n", ret);
              return ret;
            }

          if (priv->curr_op != MAC802154_OP_RXENABLE)
            {
              ret = -EINVAL;
              goto errout_with_sem;
            }

          mac802154_timercancel(priv);
          mac802154_rxdisable(priv);

          priv->curr_op = MAC802154_OP_NONE;
          mac802154_givesem(&priv->opsem);
        }
    }

  mac802154_unlock(priv)
  return OK;

errout_with_sem:
  mac802154_unlock(priv)
  return ret;
}
