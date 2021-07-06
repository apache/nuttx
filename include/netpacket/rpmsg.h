/****************************************************************************
 * include/netpacket/rpmsg.h
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

#ifndef __INCLUDE_NETPACKET_RPMSG_H
#define __INCLUDE_NETPACKET_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_SOCKET_CPU_SIZE           16
#define RPMSG_SOCKET_NAME_SIZE          32

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct sockaddr_rpmsg
{
  sa_family_t rp_family;
  char        rp_cpu[RPMSG_SOCKET_CPU_SIZE];
  char        rp_name[RPMSG_SOCKET_NAME_SIZE];
};

#endif /* __INCLUDE_NETPACKET_RPMSG_H */
