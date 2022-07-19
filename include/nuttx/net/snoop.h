/****************************************************************************
 * include/nuttx/net/snoop.h
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

#ifndef __INCLUDE_NUTTX_NET_SNOOP_H
#define __INCLUDE_NUTTX_NET_SNOOP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Datalink Type:
 *
 * A 32-bit (4 octet) field identifying the type of
 * datalink header used in the packet records that follow.
 * The datalink type codes are listed in the table below:
 *
 *  Datalink Type            Code
 *  -------------            ----
 *  IEEE 802.3               0
 *  IEEE 802.4 Token Bus     1
 *  IEEE 802.5 Token Ring    2
 *  IEEE 802.6 Metro Net     3
 *  Ethernet                 4
 *  HDLC                     5
 *  Character Synchronous    6
 *  IBM Channel-to-Channel   7
 *  FDDI                     8
 *  Other                    9
 *  Unassigned               10 - 4294967295
 *
 *  Un-encapsulated HCI (H1) 1001
 *  HCI UART (H4)            1002
 *  HCI BSCP                 1003
 *  HCI Serial (H5)          1004
 *  Unassigned               1005 - 4294967295
 *
 * More info about snoop datalink type, please refer to
 * https://www.rfc-editor.org/rfc/rfc1761.txt and
 * https://fte.com/webhelpii/hsu/Content/Technical_Information/
 * BT_Snoop_File_Format.htm
 */

#define SNOOP_DATALINK_TYPE_TOKENBUS    1
#define SNOOP_DATALINK_TYPE_TOKERING    2
#define SNOOP_DATALINK_TYPE_METRONET    3
#define SNOOP_DATALINK_TYPE_ETHERNET    4
#define SNOOP_DATALINK_TYPE_HDLC        5
#define SNOOP_DATALINK_TYPE_CHARSYNC    6
#define SNOOP_DATALINK_TYPE_IBMC2C      7
#define SNOOP_DATALINK_TYPE_FDDI        8
#define SNOOP_DATALINK_TYPE_OTHER       9

#define SNOOP_DATALINK_HCI_UNENCAP      1001
#define SNOOP_DATALINK_HCI_UART         1002
#define SNOOP_DATALINK_HCI_BSCP         1003
#define SNOOP_DATALINK_HCI_SERIAL       1004

#define SNOOP_DIRECTION_FLAG_SENT 0 /* Direction flag 0 = Sent */
#define SNOOP_DIRECTION_FLAG_RECV 1 /* Direction flag 1 = Received */

struct snoop_s
{
  bool        autosync;
  uint32_t    datalink;
  struct file filep;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: snoop_open
 *
 * Description:
 *   This function open snoop file by datalink.
 *
 ****************************************************************************/

int snoop_open(FAR struct snoop_s *snoop, FAR const char *filename,
               uint32_t datalink, bool autosync);

/****************************************************************************
 * Name: snoop_dump
 *
 * Description:
 *   This function dump nbytes buf data into snoop file.
 *
 ****************************************************************************/

int snoop_dump(FAR struct snoop_s *snoop, FAR const void *buf,
               uint32_t nbytes, uint32_t drops, uint32_t flags);

/****************************************************************************
 * Name: snoop_sync
 *
 * Description:
 *   This function sync snoop buffer.
 *
 ****************************************************************************/

int snoop_sync(FAR struct snoop_s *snoop);

/****************************************************************************
 * Name: snoop_close
 *
 * Description:
 *   This function close snoop file.
 *
 ****************************************************************************/

int snoop_close(FAR struct snoop_s *snoop);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_SNOOP_H */
