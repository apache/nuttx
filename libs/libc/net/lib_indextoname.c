/****************************************************************************
 * libs/libc/net/lib_indextoname.c
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

#include <net/if.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/net/netconfig.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: if_indextoname
 *
 * Description:
 *   The if_indextoname() function maps an interface index to its
 *   corresponding name.
 *
 * Input Parameters:
 *   ifname  - Points to a buffer of at least IF_NAMESIZE bytes.
 *             if_indextoname() will place in this buffer the name of the
 *             interface with index ifindex.
 *
 * Returned Value:
 *   If ifindex is an interface index, then the function will return the
 *   value supplied by ifname. Otherwise, the function returns a NULL pointer
 *   and sets errno to indicate the error.
 *
 ****************************************************************************/

FAR char *if_indextoname(unsigned int ifindex, FAR char *ifname)
{
  int sockfd = socket(NET_SOCK_FAMILY, NET_SOCK_TYPE, NET_SOCK_PROTOCOL);
  if (sockfd >= 0)
    {
      struct ifreq req;
      req.ifr_ifindex = ifindex;
      if (ioctl(sockfd, SIOCGIFNAME, (unsigned long)&req) >= 0)
        {
          strlcpy(ifname, req.ifr_name, IF_NAMESIZE);
          close(sockfd);
          return ifname;
        }

      close(sockfd);
    }

  return NULL;
}
