/****************************************************************************
 * include/arpa/nameser.h
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

#ifndef __INCLUDE_ARPA_NAMESER_H
#define __INCLUDE_ARPA_NAMESER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <endian.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sizes of DNS message parts (RFC 1035) */

#define NS_PACKETSZ     512    /* Maximum size of a UDP message */
#define NS_MAXDNAME     1025   /* Maximum size of a domain name string */
#define NS_MAXCDNAME    255    /* Maximum size of a compressed name */
#define NS_MAXLABEL     63     /* Maximum size of a label */
#define NS_HFIXEDSZ     12     /* Size of the message header */
#define NS_QFIXEDSZ     4      /* Size of the fixed part of a question */
#define NS_RRFIXEDSZ    10     /* Size of the fixed part of a record */
#define NS_INT32SZ      4
#define NS_INT16SZ      2
#define NS_INT8SZ       1
#define NS_INADDRSZ     4
#define NS_IN6ADDRSZ    16
#define NS_CMPRSFLGS    0xc0   /* Flag bits of a compressed name pointer */
#define NS_DEFAULTPORT  53

/* Get and put 16-bit and 32-bit values in network byte order.  cp is
 * an unsigned char pointer and moves past the value.
 */

#define NS_GET16(s, cp) \
  do \
    { \
      (s) = ((uint16_t)(cp)[0] << 8) | (cp)[1]; \
      (cp) += NS_INT16SZ; \
    } \
  while (0)

#define NS_GET32(l, cp) \
  do \
    { \
      (l) = ((uint32_t)(cp)[0] << 24) | ((uint32_t)(cp)[1] << 16) | \
            ((uint32_t)(cp)[2] << 8) | (cp)[3]; \
      (cp) += NS_INT32SZ; \
    } \
  while (0)

#define NS_PUT16(s, cp) \
  do \
    { \
      (cp)[0] = (uint16_t)(s) >> 8; \
      (cp)[1] = (uint16_t)(s); \
      (cp) += NS_INT16SZ; \
    } \
  while (0)

#define NS_PUT32(l, cp) \
  do \
    { \
      (cp)[0] = (uint32_t)(l) >> 24; \
      (cp)[1] = (uint32_t)(l) >> 16; \
      (cp)[2] = (uint32_t)(l) >> 8; \
      (cp)[3] = (uint32_t)(l); \
      (cp) += NS_INT32SZ; \
    } \
  while (0)

/* Older (BIND 4) names of the definitions above and below */

#define PACKETSZ        NS_PACKETSZ
#define MAXDNAME        NS_MAXDNAME
#define MAXCDNAME       NS_MAXCDNAME
#define MAXLABEL        NS_MAXLABEL
#define HFIXEDSZ        NS_HFIXEDSZ
#define QFIXEDSZ        NS_QFIXEDSZ
#define RRFIXEDSZ       NS_RRFIXEDSZ
#define INT32SZ         NS_INT32SZ
#define INT16SZ         NS_INT16SZ
#define INT8SZ          NS_INT8SZ
#define INADDRSZ        NS_INADDRSZ
#define IN6ADDRSZ       NS_IN6ADDRSZ
#define INDIR_MASK      NS_CMPRSFLGS
#define NAMESERVER_PORT NS_DEFAULTPORT

#define GETSHORT        NS_GET16
#define GETLONG         NS_GET32
#define PUTSHORT        NS_PUT16
#define PUTLONG         NS_PUT32

#define QUERY           ns_o_query
#define IQUERY          ns_o_iquery
#define STATUS          ns_o_status
#define NS_NOTIFY_OP    ns_o_notify
#define NS_UPDATE_OP    ns_o_update

#define NOERROR         ns_r_noerror
#define FORMERR         ns_r_formerr
#define SERVFAIL        ns_r_servfail
#define NXDOMAIN        ns_r_nxdomain
#define NOTIMP          ns_r_notimpl
#define REFUSED         ns_r_refused

#define C_IN            ns_c_in
#define C_CHAOS         ns_c_chaos
#define C_HS            ns_c_hs
#define C_NONE          ns_c_none
#define C_ANY           ns_c_any

#define T_A             ns_t_a
#define T_NS            ns_t_ns
#define T_CNAME         ns_t_cname
#define T_SOA           ns_t_soa
#define T_PTR           ns_t_ptr
#define T_HINFO         ns_t_hinfo
#define T_MX            ns_t_mx
#define T_TXT           ns_t_txt
#define T_AAAA          ns_t_aaaa
#define T_SRV           ns_t_srv
#define T_NAPTR         ns_t_naptr
#define T_OPT           ns_t_opt
#define T_IXFR          ns_t_ixfr
#define T_AXFR          ns_t_axfr
#define T_ANY           ns_t_any

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Operation codes */

typedef enum __ns_opcode
{
  ns_o_query  = 0,
  ns_o_iquery = 1,
  ns_o_status = 2,
  ns_o_notify = 4,
  ns_o_update = 5,
  ns_o_max    = 6
} ns_opcode;

/* Response codes */

typedef enum __ns_rcode
{
  ns_r_noerror  = 0,
  ns_r_formerr  = 1,
  ns_r_servfail = 2,
  ns_r_nxdomain = 3,
  ns_r_notimpl  = 4,
  ns_r_refused  = 5,
  ns_r_yxdomain = 6,
  ns_r_yxrrset  = 7,
  ns_r_nxrrset  = 8,
  ns_r_notauth  = 9,
  ns_r_notzone  = 10,
  ns_r_max      = 11
} ns_rcode;

/* Classes */

typedef enum __ns_class
{
  ns_c_invalid = 0,
  ns_c_in      = 1,
  ns_c_2       = 2,
  ns_c_chaos   = 3,
  ns_c_hs      = 4,
  ns_c_none    = 254,
  ns_c_any     = 255,
  ns_c_max     = 65536
} ns_class;

/* Record types */

typedef enum __ns_type
{
  ns_t_invalid = 0,
  ns_t_a       = 1,
  ns_t_ns      = 2,
  ns_t_md      = 3,
  ns_t_mf      = 4,
  ns_t_cname   = 5,
  ns_t_soa     = 6,
  ns_t_mb      = 7,
  ns_t_mg      = 8,
  ns_t_mr      = 9,
  ns_t_null    = 10,
  ns_t_wks     = 11,
  ns_t_ptr     = 12,
  ns_t_hinfo   = 13,
  ns_t_minfo   = 14,
  ns_t_mx      = 15,
  ns_t_txt     = 16,
  ns_t_rp      = 17,
  ns_t_afsdb   = 18,
  ns_t_sig     = 24,
  ns_t_key     = 25,
  ns_t_aaaa    = 28,
  ns_t_loc     = 29,
  ns_t_srv     = 33,
  ns_t_naptr   = 35,
  ns_t_cert    = 37,
  ns_t_dname   = 39,
  ns_t_opt     = 41,
  ns_t_ds      = 43,
  ns_t_sshfp   = 44,
  ns_t_rrsig   = 46,
  ns_t_nsec    = 47,
  ns_t_dnskey  = 48,
  ns_t_tlsa    = 52,
  ns_t_tkey    = 249,
  ns_t_tsig    = 250,
  ns_t_ixfr    = 251,
  ns_t_axfr    = 252,
  ns_t_any     = 255,
  ns_t_uri     = 256,
  ns_t_caa     = 257,
  ns_t_max     = 65536
} ns_type;

/* The fixed DNS message header (RFC 1035, 4.1.1) */

typedef struct
{
  unsigned id      : 16;      /* Query identification number */
#if BYTE_ORDER == BIG_ENDIAN
  unsigned qr      : 1;       /* Response flag */
  unsigned opcode  : 4;       /* Purpose of message */
  unsigned aa      : 1;       /* Authoritative answer */
  unsigned tc      : 1;       /* Truncated message */
  unsigned rd      : 1;       /* Recursion desired */
  unsigned ra      : 1;       /* Recursion available */
  unsigned unused  : 1;
  unsigned ad      : 1;       /* Authentic data */
  unsigned cd      : 1;       /* Checking disabled */
  unsigned rcode   : 4;       /* Response code */
#else
  unsigned rd      : 1;       /* Recursion desired */
  unsigned tc      : 1;       /* Truncated message */
  unsigned aa      : 1;       /* Authoritative answer */
  unsigned opcode  : 4;       /* Purpose of message */
  unsigned qr      : 1;       /* Response flag */
  unsigned rcode   : 4;       /* Response code */
  unsigned cd      : 1;       /* Checking disabled */
  unsigned ad      : 1;       /* Authentic data */
  unsigned unused  : 1;
  unsigned ra      : 1;       /* Recursion available */
#endif
  unsigned qdcount : 16;      /* Number of question entries */
  unsigned ancount : 16;      /* Number of answer entries */
  unsigned nscount : 16;      /* Number of authority entries */
  unsigned arcount : 16;      /* Number of resource entries */
} HEADER;

#endif /* __INCLUDE_ARPA_NAMESER_H */
