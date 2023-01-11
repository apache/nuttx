/****************************************************************************
 * libs/libc/netdb/lib_netdb.h
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

#ifndef __LIBS_LIBC_NETDB_LIB_NETDB_H
#define __LIBS_LIBC_NETDB_LIB_NETDB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <netdb.h>
#include <stdio.h>

#ifdef CONFIG_LIBC_NETDB

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is the maximum number of alternate host names supported by this
 * implementation:
 */

#ifndef CONFIG_NETDB_MAX_ALTNAMES
#  define CONFIG_NETDB_MAX_ALTNAMES 4
#endif

/* This is the path to the system hosts file */

#ifndef CONFIG_NETDB_HOSTCONF_PATH
#  define CONFIG_NETDB_HOSTCONF_PATH "/etc/hosts"
#endif

/* Size of the buffer available for host data */

#ifndef CONFIG_NETDB_BUFSIZE
#  define CONFIG_NETDB_BUFSIZE 128
#endif

#ifndef CONFIG_NETDB_MAX_IPADDR
#  define CONFIG_NETDB_MAX_IPADDR 1
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct hostent_s
{
  FAR char  *h_name;       /* Official name of the host. */
  FAR char **h_aliases;    /* A pointer to an array of pointers to the
                            * alternative host names, terminated by a
                            * null pointer. */
  FAR int   *h_addrtypes;  /* A pointer to an array of address type. */
  FAR int   *h_lengths;    /* A pointer to an array of the length, in bytes,
                            * of the address. */
  FAR char **h_addr_list;  /* A pointer to an array of pointers to network
                            * addresses (in network byte order) for the host,
                            * terminated by a null pointer. */
};

struct services_db_s
{
  FAR const char *s_name;
  int s_port;
  int s_protocol;
};

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

EXTERN struct hostent g_hostent;
EXTERN char g_hostbuffer[CONFIG_NETDB_BUFSIZE];
EXTERN const struct services_db_s g_services_db[];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

bool convert_hostent(FAR const struct hostent_s *in,
                     int type, FAR struct hostent *out);

ssize_t parse_hostfile(FAR FILE *stream, FAR struct hostent_s *host,
                       FAR char *buf, size_t buflen);

int gethostentbyname_r(FAR const char *name,
                       FAR struct hostent_s *host, FAR char *buf,
                       size_t buflen, FAR int *h_errnop, int flags);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LIBC_NETDB */
#endif /* __LIBS_LIBC_NETDB_LIB_NETDB_H */
