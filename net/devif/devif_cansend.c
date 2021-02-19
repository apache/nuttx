/****************************************************************************
 * net/devif/devif_cansend.c
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

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/net/netdev.h>

#if defined(CONFIG_NET_CAN)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Constant Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devif_can_send
 *
 * Description:
 *   Called from socket logic in order to send a can packet in response to
 *   an xmit or poll request from the network interface driver.
 *
 *   This is almost identical to calling devif_send() except that the data to
 *   be sent is copied into dev->d_buf (vs. dev->d_appdata), since there is
 *   no header on the data.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

void devif_can_send(FAR struct net_driver_s *dev, FAR const void *buf,
                    unsigned int len)
{
  DEBUGASSERT(dev && len > 0 && len < NETDEV_PKTSIZE(dev));

  /* Copy the data into the device packet buffer */

  memcpy(dev->d_buf, buf, len);

  /* Set the number of bytes to send */

  dev->d_len    = len;
  dev->d_sndlen = len;
}

#endif /* CONFIG_NET_CAN */
