/****************************************************************************
 * arch/risc-v/src/bl602/bl602_netdev.h
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

#ifndef _BL602_NETDEV_H__
#define _BL602_NETDEV_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PRESERVE_80211_HEADER_LEN 128

#define BL602_NET_EVT_TX_DONE (0x1 << 0x0)
#define BL602_NET_EVT_RX      (0x1 << 0x1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_netdev_alloc_txbuf
 *
 * Description:
 *   Allocate wifi transmit buffer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Non-zero: Buffer address, otherwise error.
 *
 ****************************************************************************/

uint8_t *bl602_netdev_alloc_txbuf(void);

/****************************************************************************
 * Name: bl602_netdev_free_txbuf
 *
 * Description:
 *   Free wifi transmit buffer
 *
 * Input Parameters:
 *   buf: The address of the buffer to be released.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_netdev_free_txbuf(uint8_t *buf);

/****************************************************************************
 * Name: bl602_net_notify
 *
 * Description:
 *   BL602 WiFi notify handler, similar interrupt function
 *
 * Input Parameters:
 *   event: notify type, tx done or received new data
 *   data: The data of the event, may be NULL
 *   len: data length
 *   opaque: customer data
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int  bl602_net_notify(uint32_t event, uint8_t *data, int len, void *opaque);

/****************************************************************************
 * Name: bl602_net_event
 *
 * Description:
 *   BL602 WiFi event handler, called by BL602 wifi manager private library
 *
 * Input Parameters:
 *   event: event number
 *   val: the value of the event
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

void bl602_net_event(int evt, int val);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_ASSERT_H */
#endif /* _BL602_NETDEV_H__ */
