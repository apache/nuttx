/****************************************************************************
 * drivers/usrsock/usrsock_rpmsg.h
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

#ifndef __DRIVERS_USRSOCK_USRSOCK_RPMSG_H
#define __DRIVERS_USRSOCK_USRSOCK_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/usrsock.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define USRSOCK_RPMSG_EPT_NAME      "rpmsg-usrsock"

#define USRSOCK_RPMSG_DNS_REQUEST    USRSOCK_REQUEST__MAX
#define USRSOCK_RPMSG_DNS_EVENT      127

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DNS request message */

begin_packed_struct struct usrsock_rpmsg_dns_request_s
{
  struct usrsock_request_common_s head;

  uint16_t addrlen;
} end_packed_struct;

/* DNS event message */

begin_packed_struct struct usrsock_rpmsg_dns_event_s
{
  struct usrsock_message_common_s head;

  uint16_t addrlen;
} end_packed_struct;

#endif /* __DRIVERS_USRSOCK_USRSOCK_RPMSG_H */
