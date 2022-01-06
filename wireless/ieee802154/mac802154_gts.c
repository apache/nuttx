/****************************************************************************
 * wireless/ieee802154/mac802154_gts.c
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

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_req_gts
 *
 * Description:
 *   The MLME-GTS.request primitive allows a device to send a request to the
 *   PAN coordinator to allocate a new GTS or to deallocate an existing GTS.
 *   Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_gts callback.
 *
 ****************************************************************************/

int mac802154_req_gts(MACHANDLE mac, FAR struct ieee802154_gts_req_s *req)
{
#if 0
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
#endif
  return -ENOTTY;
}
