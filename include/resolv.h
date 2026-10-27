/****************************************************************************
 * include/resolv.h
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

#ifndef __INCLUDE_RESOLV_H
#define __INCLUDE_RESOLV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>

#include <arpa/nameser.h>
#include <netinet/in.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The limits and the defaults of the resolver state (BIND, glibc, musl) */

#define MAXNS              3     /* Name servers in the state */
#define MAXDFLSRCH         3     /* Default domain levels to try */
#define MAXDNSRCH          6     /* Domains in the search path */
#define LOCALDOMAINPARTS   2     /* Least levels in a local domain */
#define MAXRESOLVSORT      10    /* Entries of sort_list */
#define RES_TIMEOUT        5     /* Seconds between retries */
#define RES_MAXNDOTS       15
#define RES_MAXRETRANS     30
#define RES_MAXRETRY       5
#define RES_DFLRETRY       2
#define RES_MAXTIME        65535

/* The options of the resolver state (the values of BIND and glibc) */

#define RES_INIT           0x00000001  /* res_init() was called */
#define RES_DEBUG          0x00000002  /* Print debug messages */
#define RES_AAONLY         0x00000004  /* Authoritative answers only */
#define RES_USEVC          0x00000008  /* Use a virtual circuit (TCP) */
#define RES_PRIMARY        0x00000010  /* Query the primary server only */
#define RES_IGNTC          0x00000020  /* Ignore truncation errors */
#define RES_RECURSE        0x00000040  /* Recursion desired */
#define RES_DEFNAMES       0x00000080  /* Use the default domain name */
#define RES_STAYOPEN       0x00000100  /* Keep the TCP socket open */
#define RES_DNSRCH         0x00000200  /* Search up the local domain */
#define RES_INSECURE1      0x00000400  /* Type 1 security disabled */
#define RES_INSECURE2      0x00000800  /* Type 2 security disabled */
#define RES_NOALIASES      0x00001000  /* No HOSTALIASES */
#define RES_USE_INET6      0x00002000  /* IPv6 by gethostbyname() */
#define RES_ROTATE         0x00004000  /* Rotate the name servers */
#define RES_NOCHECKNAME    0x00008000  /* Do not check names */
#define RES_KEEPTSIG       0x00010000  /* Do not strip TSIG records */
#define RES_BLAST          0x00020000  /* Query all servers at once */
#define RES_USE_EDNS0      0x00100000  /* Use EDNS0 */
#define RES_SNGLKUP        0x00200000  /* One request for A and AAAA */
#define RES_SNGLKUPREOP    0x00400000  /* The same, and reopen */
#define RES_USE_DNSSEC     0x00800000  /* Ask for DNSSEC (the DO bit) */
#define RES_NOTLDQUERY     0x01000000  /* No unqualified top level name */

#define RES_DEFAULT        (RES_RECURSE | RES_DEFNAMES | RES_DNSRCH)

/* The resolver state of the process, as in glibc and musl */

#define _res               (*__res_state())

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* An entry of the sort list of the resolver state */

struct __res_sort_s
{
  struct in_addr     addr;
  uint32_t           mask;
};

/* The resolver state (BIND).  NuttX's resolver does not use it: the name
 * servers and the options come from its own configuration.
 */

struct __res_state
{
  int                 retrans;                 /* Retransmission time */
  int                 retry;                   /* Number of tries */
  unsigned long       options;                 /* RES_* options */
  int                 nscount;                 /* Number of name servers */
  struct sockaddr_in  nsaddr_list[MAXNS];      /* The name servers */
  unsigned short      id;                      /* The current message id */
  FAR char           *dnsrch[MAXDNSRCH + 1];   /* The search path */
  char                defdname[256];           /* The default domain */
  unsigned long       pfcode;                  /* RES_PRF_* flags */
  unsigned            ndots:4;                 /* Dots for an absolute name */
  unsigned            nsort:4;                 /* Entries of sort_list */
  struct __res_sort_s sort_list[MAXRESOLVSORT];
};

typedef FAR struct __res_state *res_state;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

int b64_ntop(FAR const unsigned char *src, size_t srclen,
             FAR char *target, size_t targsize);

int b64_pton(FAR const char *src,
             FAR unsigned char *target, size_t targsize);

/* dn resolution */

int dn_comp(FAR const char *src, FAR unsigned char *dst, int space,
            FAR unsigned char **dnptrs, FAR unsigned char **lastdnptr);

int dn_expand(FAR const unsigned char *base, FAR const unsigned char *end,
              FAR const unsigned char *src, FAR char *dest, int space);

int dn_skipname(FAR const unsigned char *src,
                FAR const unsigned char *end);

/* DNS queries */

FAR struct __res_state *__res_state(void);
int res_init(void);

int res_mkquery(int op, FAR const char *dname, int rclass, int type,
                FAR const unsigned char *data, int datalen,
                FAR const unsigned char *newrr,
                FAR unsigned char *buf, int buflen);

int res_send(FAR const unsigned char *msg, int msglen,
             FAR unsigned char *answer, int anslen);

int res_query(FAR const char *dname, int rclass, int type,
              FAR unsigned char *answer, int anslen);

int res_search(FAR const char *dname, int rclass, int type,
               FAR unsigned char *answer, int anslen);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_RESOLV_H */
