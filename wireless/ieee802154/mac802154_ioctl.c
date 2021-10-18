/****************************************************************************
 * wireless/ieee802154/mac802154_ioctl.c
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
 * Name: mac802154_ioctl
 *
 * Description:
 *   Handle MAC and radio IOCTL commands directed to the MAC.
 *
 * Input Parameters:
 *   mac - Reference to the MAC driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int mac802154_ioctl(MACHANDLE mac, int cmd, unsigned long arg)
{
  int ret = -EINVAL;

  FAR union ieee802154_macarg_u *macarg =
    (FAR union ieee802154_macarg_u *)((uintptr_t)arg);

  DEBUGASSERT(mac != NULL);

  /* Check for IOCTLs aimed at the IEEE802.15.4 MAC layer */

  if (_MAC802154IOCVALID(cmd))
    {
      /* Handle the MAC IOCTL command */

      switch (cmd)
        {
          case MAC802154IOC_MLME_ASSOC_REQUEST:
            {
              ret = mac802154_req_associate(mac, &macarg->assocreq);
            }
            break;
          case MAC802154IOC_MLME_ASSOC_RESPONSE:
            {
              ret = mac802154_resp_associate(mac, &macarg->assocresp);
            }
            break;
          case MAC802154IOC_MLME_DISASSOC_REQUEST:
            {
              ret = mac802154_req_disassociate(mac, &macarg->disassocreq);
            }
            break;
          case MAC802154IOC_MLME_GET_REQUEST:
            {
              ret = mac802154_req_get(mac, macarg->getreq.attr,
                                      &macarg->getreq.attrval);
            }
            break;
          case MAC802154IOC_MLME_GTS_REQUEST:
            {
              ret = mac802154_req_gts(mac, &macarg->gtsreq);
            }
            break;
          case MAC802154IOC_MLME_ORPHAN_RESPONSE:
            {
              ret = mac802154_resp_orphan(mac, &macarg->orphanresp);
            }
            break;
          case MAC802154IOC_MLME_RESET_REQUEST:
            {
              ret = mac802154_req_reset(mac, macarg->resetreq.resetattr);
            }
            break;
          case MAC802154IOC_MLME_RXENABLE_REQUEST:
            {
              ret = mac802154_req_rxenable(mac, &macarg->rxenabreq);
            }
            break;
          case MAC802154IOC_MLME_SCAN_REQUEST:
            {
              ret = mac802154_req_scan(mac, &macarg->scanreq);
            }
            break;
          case MAC802154IOC_MLME_SET_REQUEST:
            {
              ret = mac802154_req_set(mac, macarg->setreq.attr,
                                      &macarg->setreq.attrval);
            }
            break;
          case MAC802154IOC_MLME_START_REQUEST:
            {
              ret = mac802154_req_start(mac, &macarg->startreq);
            }
            break;
          case MAC802154IOC_MLME_SYNC_REQUEST:
            {
              ret = mac802154_req_sync(mac, &macarg->syncreq);
            }
            break;
          case MAC802154IOC_MLME_POLL_REQUEST:
            {
              ret = mac802154_req_poll(mac, &macarg->pollreq);
            }
            break;
          default:
              wlerr("ERROR: Unrecognized cmd: %d\n", cmd);
              ret = -ENOTTY;
              break;
        }
    }

  return ret;
}
