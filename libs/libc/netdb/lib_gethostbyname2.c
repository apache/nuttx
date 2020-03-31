/****************************************************************************
 * libs/libc/netdb/lib_gethostbyname2.c
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

#include <netdb.h>

#include "netdb/lib_netdb.h"

#ifdef CONFIG_LIBC_NETDB

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gethostbyname2
 *
 * Description:
 *   The gethostbyname2() function returns a structure of type hostent for
 *   the given host name. Here name is either a hostname, or an IPv4 address
 *   in standard dot notation (as for inet_addr(3)), or an IPv6 address in
 *   colon (and possibly dot) notation.
 *
 *   If name is an IPv4 or IPv6 address, no lookup is performed and
 *   gethostbyname2_r() simply copies name into the h_name field
 *   and its struct in_addr equivalent into the h_addr_list[0] field of the
 *   returned hostent structure.
 *
 * Input Parameters:
 *   name - The name of the host to find.
 *   type - The type of the address to find.
 *
 * Returned Value:
 *   Upon successful completion, this function will return a pointer to a
 *   hostent structure if the requested entry was found, and a null pointer
 *   if the end of the database was reached or the requested entry was not
 *   found.
 *
 *   Upon unsuccessful completion, gethostbyname2() will set h_errno to
 *   indicate the error
 *
 ****************************************************************************/

FAR struct hostent *gethostbyname2(FAR const char *name, int type)
{
  FAR struct hostent *res;
  int ret;

  ret = gethostbyname2_r(name, type, &g_hostent, g_hostbuffer,
                         CONFIG_NETDB_BUFSIZE, &res, &h_errno);
  return ret == 0 ? res : NULL;
}

#endif /* CONFIG_LIBC_NETDB */
