/****************************************************************************
 * libs/libc/netdb/lib_resinit.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <resolv.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: res_init
 *
 * Description:
 *   Initialize the resolver.  The DNS client gets the name servers at each
 *   query: from CONFIG_NETDB_RESOLVCONF_PATH with CONFIG_NETDB_RESOLVCONF,
 *   otherwise from the list that dns_add_nameserver() changes.  Thus there
 *   is no state to initialize or to read again.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Always zero.
 *
 ****************************************************************************/

int res_init(void)
{
  return 0;
}
