/****************************************************************************
 * wireless/ieee802154/mac802154_bind.c
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
