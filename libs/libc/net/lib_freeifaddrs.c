/****************************************************************************
 * libs/libc/net/lib_freeifaddrs.c
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

#include <ifaddrs.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: freeifaddrs
 *
 * Description:
 *   The freeifaddrs() function frees the data structure returned by
 *   getifaddrs().
 *
 * Input Parameters:
 *   addrs - The data structure to free
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void freeifaddrs(FAR struct ifaddrs *addrs)
{
  while (addrs != NULL)
    {
      FAR struct ifaddrs *tmp = addrs;
      addrs = addrs->ifa_next;
      lib_free(tmp);
    }
}
