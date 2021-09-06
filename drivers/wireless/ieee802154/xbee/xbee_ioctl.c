/****************************************************************************
 * drivers/wireless/ieee802154/xbee/xbee_ioctl.c
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/fs/ioctl.h>

#include <xbee.h>
#include <xbee_mac.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xbee_ioctl
 *
 * Description:
 *   Handle MAC and radio IOCTL commands directed to the MAC.
 *
 * Input Parameters:
 *   mac - Reference to the XBee driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int xbee_ioctl(XBEEHANDLE xbee, int cmd, unsigned long arg)
{
  int ret = -EINVAL;

  FAR union ieee802154_macarg_u *macarg =
    (FAR union ieee802154_macarg_u *)((uintptr_t)arg);

  DEBUGASSERT(xbee != NULL);

  /* Check for IOCTLs aimed at the IEEE802.15.4 MAC layer */

  if (_MAC802154IOCVALID(cmd))
    {
      /* Handle the MAC IOCTL command */

      switch (cmd)
        {
          case MAC802154IOC_MLME_GET_REQUEST:
            {
              ret = xbee_req_get(xbee, macarg->getreq.attr,
                                 &macarg->getreq.attrval);
            }
            break;

          case MAC802154IOC_MLME_SET_REQUEST:
            {
              ret = xbee_req_set(xbee, macarg->setreq.attr,
                                 &macarg->setreq.attrval);
            }
            break;

          case MAC802154IOC_MLME_START_REQUEST:
            {
              ret = xbee_req_start(xbee, &macarg->startreq);
            }
            break;

          case MAC802154IOC_MLME_ASSOC_REQUEST:
            {
              ret = xbee_req_associate(xbee, &macarg->assocreq);
            }
            break;

          case MAC802154IOC_MLME_RESET_REQUEST:
            {
              ret = xbee_req_reset(xbee, macarg->resetreq.resetattr);
            }
            break;

#if 0
          case MAC802154IOC_MLME_ASSOC_RESPONSE:
            {
              ret = xbee_resp_associate(xbee, &macarg->assocresp);
            }
            break;

          case MAC802154IOC_MLME_DISASSOC_REQUEST:
            {
              ret = xbee_req_disassociate(xbee, &macarg->disassocreq);
            }
            break;

          case MAC802154IOC_MLME_RXENABLE_REQUEST:
            {
              ret = xbee_req_rxenable(xbee, &macarg->rxenabreq);
            }
            break;

          case MAC802154IOC_MLME_SCAN_REQUEST:
            {
              ret = xbee_req_scan(xbee, &macarg->scanreq);
            }
            break;

          case MAC802154IOC_MLME_POLL_REQUEST:
            {
              ret = xbee_req_poll(xbee, &macarg->pollreq);
            }
            break;
#endif

          default:
              wlerr("ERROR: Unrecognized cmd: %d\n", cmd);
              ret = -ENOTTY;
              break;
        }
    }

  return ret;
}
