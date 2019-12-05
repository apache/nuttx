/****************************************************************************
 * drivers/wireless/xbee/drivers/xbee_ioctl.c
 *
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author:  Anthony Merlino <anthony@vergeaero.com>
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
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
