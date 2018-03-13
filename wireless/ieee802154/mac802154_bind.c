/****************************************************************************
 * wireless/ieee802154/mac802154_bind.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *
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
#include <errno.h>

#include "mac802154.h"
#include "mac802154_internal.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_bind
 *
 * Description:
 *   Bind the MAC callback table to the MAC state.
 *
 * Input Parameters:
 *   mac - Reference to the MAC driver state structure
 *   cb  - MAC callback operations
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int mac802154_bind(MACHANDLE mac, FAR struct mac802154_maccb_s *cb)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  FAR struct mac802154_maccb_s *next;
  FAR struct mac802154_maccb_s *prev;

  /* Add the MAC client callback structure to the list of MAC callbacks in
   * priority order.
   *
   * Search the list to find the location to insert the new instance.
   * The list is maintained in descending priority order.
   */

  for (prev = NULL, next = priv->cb;
      (next != NULL && cb->prio <= next->prio);
       prev = next, next = next->flink);

  /* Add the instance to the spot found in the list.  Check if the instance
   * goes at the head of the list.
   */

  if (prev == NULL)
    {
      cb->flink = priv->cb; /* May be NULL */
      priv->cb  = cb;
    }

  /* No.. the instance goes between prev and next */

  else
    {
      cb->flink   = next; /* May be NULL */
      prev->flink = cb;
    }

  /* Keep track of the number of clients requesting notification */

  if (cb->notify != NULL)
    {
      priv->nclients++;
    }

  return OK;
}
