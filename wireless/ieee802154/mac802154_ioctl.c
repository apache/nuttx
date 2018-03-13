/****************************************************************************
 * wireless/ieee802154/mac802154_ioctl.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include "mac802154.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Public MAC Functions
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

