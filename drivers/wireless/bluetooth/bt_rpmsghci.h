/****************************************************************************
 * drivers/wireless/bluetooth/bt_rpmsghci.h
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

#ifndef __DRIVERS_WIRELESS_BLUETOOTH_BT_RPMSGHCI_H
#define __DRIVERS_WIRELESS_BLUETOOTH_BT_RPMSGHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/compiler.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define RPMSGHCI_NAME_PREFIX     "rpmsghci-"
#define RPMSGHCI_NAME_PREFIX_LEN 9

#define RPMSGHCI_OPEN            0 /* Device open */
#define RPMSGHCI_CLOSE           1 /* Device close */
#define RPMSGHCI_SEND            2 /* Data to a HCI controller */
#define RPMSGHCI_IOCTL           3 /* Device ioctl */
#define RPMSGHCI_RECV            4 /* Data from a HCI controller */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Rpmsg device cookie used to handle the response from the remote cpu */

struct rpmsghci_cookie_s
{
  sem_t sem;
};

begin_packed_struct struct rpmsghci_header_s
{
  uint32_t                 command;
  int32_t                  result;
  uint64_t                 cookie;
} end_packed_struct;

begin_packed_struct struct rpmsghci_open_s
{
  struct rpmsghci_header_s header;
} end_packed_struct;

#define rpmsghci_close_s rpmsghci_open_s

begin_packed_struct struct rpmsghci_data_s
{
  struct rpmsghci_header_s header;
  uint8_t                  type;
  char                     data[1];
} end_packed_struct;

begin_packed_struct struct rpmsghci_ioctl_s
{
  struct rpmsghci_header_s header;
  uint64_t                 filep;
  uint64_t                 arg;
  uint32_t                 arglen;
  int32_t                  request;
  char                     buf[1];
} end_packed_struct;

/****************************************************************************
 * Internal function prototypes
 ****************************************************************************/

/****************************************************************************
 * Internal data
 ****************************************************************************/

#endif /* __DRIVERS_WIRELESS_BLUETOOTH_BT_RPMSGHCI_H */
