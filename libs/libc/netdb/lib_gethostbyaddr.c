/****************************************************************************
 * libs/libc/netdb/lib_gethostbyaddr.c
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
 * Name: gethostbyaddr
 *
 * Description:
 *   The gethostbyaddr() function returns a structure of type hostent for
 *   the given host address addr of length len and address type type. Valid
 *   address types are AF_INET and AF_INET6. The host address argument is a
 *   pointer to a struct of a type depending on the address type, for example
 *   a struct in_addr *  for address type AF_INET.
 *
 * Input Parameters:
 *   addr - The address of the host to find.
 *   len - The length of the address
 *   type - The type of the address
 *
 * Returned Value:
 *   Upon successful completion, this function will return a pointer to a
 *   hostent structure if the requested entry was found, and a null pointer
 *   if the end of the database was reached or the requested entry was not
 *   found.
 *
 *   Upon unsuccessful completion, gethostbyaddr() will set h_errno to
 *   indicate the error
 *
 ****************************************************************************/

FAR struct hostent *gethostbyaddr(FAR const void *addr,
                                  socklen_t len, int type)
{
  FAR struct hostent *res;
  int ret;

  ret = gethostbyaddr_r(addr, len, type, &g_hostent, g_hostbuffer,
                        CONFIG_NETDB_BUFSIZE, &res, &h_errno);
  return ret == 0 ? res : NULL;
}

#endif /* CONFIG_LIBC_NETDB */
