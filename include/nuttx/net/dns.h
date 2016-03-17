/****************************************************************************
 * include/nuttx/net/dns.h
 * DNS resolver code header file.
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2014-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Inspired by/based on uIP logic by Adam Dunkels:
 *
 *   Copyright (c) 2002-2003, Adam Dunkels. All rights reserved.
 *   Author Adam Dunkels <adam@dunkels.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_NET_DNS_H
#define __INCLUDE_NUTTX_NET_DNS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/socket.h>
#include <netinet/in.h>

#include <nuttx/net/netconfig.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* DNS classes */

#define DNS_CLASS_IN                1 /* RFC 1035 Internet */
#define DNS_CLASS CH                3 /* N/A      Chaos */
#define DNS_CLASS_HS                4 /* N/A      Hesiod */
#define DNS_CLASS_QNONE           254 /* RFC 2136 QCLASS NONE */
#define DNS_CLASS_QANY            255 /* RFC 1035 QCLASS ANY */

/* DNS resource record types */

#define DNS_RECTYPE_A                1 /* RFC 1035 IPv4 ddress record */
#define DNS_RECTYPE_AAAA            28 /* RFC 3596 IPv6 address record */
#define DNS_RECTYPE_AFSDB           18 /* RFC 1183 AFS database record */
#define DNS_RECTYPE_APL             42 /* RFC 3123 Address Prefix List */
#define DNS_RECTYPE_CAA            257 /* RFC 6844 Certification Authority Authorization */
#define DNS_RECTYPE_CDNSKEY         60 /* RFC 7344 Child DNSKEY */
#define DNS_RECTYPE_CDS             59 /* RFC 7344 Child DS */
#define DNS_RECTYPE_CERT            37 /* RFC 4398 Certificate record */
#define DNS_RECTYPE_CNAME            5 /* RFC 1035 Canonical name record */
#define DNS_RECTYPE_DHCID           49 /* RFC 4701 DHCP identifier */
#define DNS_RECTYPE_DLV          32769 /* RFC 4431 DNSSEC Lookaside Validation record */
#define DNS_RECTYPE_DNAME           39 /* RFC 2672 Delegation Name */
#define DNS_RECTYPE_DNSKEY          48 /* RFC 4034 DNS Key record */
#define DNS_RECTYPE_DS              43 /* RFC 4034 Delegation signer */
#define DNS_RECTYPE_HIP             55 /* RFC 5205 Host Identity Protocol */
#define DNS_RECTYPE_IPSECKEY        45 /* RFC 4025 IPsec Key */
#define DNS_RECTYPE_KEY             25 /* RFC 2535 and RFC 2930 Key record */
#define DNS_RECTYPE_KX              36 /* RFC 2230 Key eXchanger record */
#define DNS_RECTYPE_LOC             29 /* RFC 1876 Location record */
#define DNS_RECTYPE_MX              15 /* RFC 1035 Mail exchange record */
#define DNS_RECTYPE_NAPTR           35 /* RFC 3403 Naming Authority Pointer */
#define DNS_RECTYPE_NS               2 /* RFC 1035 Name server record */
#define DNS_RECTYPE_NSEC            47 /* RFC 4034 Next-Secure record */
#define DNS_RECTYPE_NSEC3           50 /* RFC 5155 NSEC record version 3 */
#define DNS_RECTYPE_NSEC3PARAM      51 /* RFC 5155 NSEC3 parameters */
#define DNS_RECTYPE_PTR             12 /* RFC 1035 Pointer record */
#define DNS_RECTYPE_RRSIG           46 /* RFC 4034 DNSSEC signature */
#define DNS_RECTYPE_RP              17 /* RFC 1183 Responsible person */
#define DNS_RECTYPE_SIG             24 /* RFC 2535 Signature */
#define DNS_RECTYPE_SOA              6 /* RFC 1035 and RFC 2308 Start of [a zone of] authority record */
#define DNS_RECTYPE_SRV             33 /* RFC 2782 Service locator */
#define DNS_RECTYPE_SSHFP           44 /* RFC 4255 SSH Public Key Fingerprint */
#define DNS_RECTYPE_TA           32768 /* N/A DNSSEC Trust Authorities */
#define DNS_RECTYPE_TKEY           249 /* RFC 2930 Secret key record */
#define DNS_RECTYPE_TLSA            52 /* RFC 6698 TLSA certificate association */
#define DNS_RECTYPE_TSIG           250 /* RFC 2845 Transaction Signature */
#define DNS_RECTYPE_TXT             16 /* RFC 1035[1] Text record */

#define DNS_RECTYPE_ALL            255 /* RFC 1035 All cached records */
#define DNS_RECTYPE_AXFR           252 /* RFC 1035 Authoritative Zone Transfer */
#define DNS_RECTYPE_IXFR           251 /* RFC 1996 Incremental Zone Transfer */
#define DNS_RECTYPE_OPT             41 /* RFC 6891 Option  */

/* Flag1 bit definitions */

#define DNS_FLAG1_RESPONSE        0x80
#define DNS_FLAG1_OPCODE_STATUS   0x10
#define DNS_FLAG1_OPCODE_INVERSE  0x08
#define DNS_FLAG1_OPCODE_STANDARD 0x00
#define DNS_FLAG1_AUTHORATIVE     0x04
#define DNS_FLAG1_TRUNC           0x02
#define DNS_FLAG1_RD              0x01

/* Flag2 bit definitions */

#define DNS_FLAG2_RA              0x80
#define DNS_FLAG2_ERR_MASK        0x0f
#define DNS_FLAG2_ERR_NONE        0x00
#define DNS_FLAG2_ERR_NAME        0x03

/* Default DNS server port number */

#define DNS_DEFAULT_PORT          53

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

 /* The DNS message header */

struct dns_header_s
{
  uint16_t id;
  uint8_t  flags1;
  uint8_t  flags2;
  uint16_t numquestions;
  uint16_t numanswers;
  uint16_t numauthrr;
  uint16_t numextrarr;
};

/* The DNS answer message structure */

struct dns_answer_s
{
  uint16_t type;
  uint16_t class;
  uint16_t ttl[2];
  uint16_t len;

  union
  {
#ifdef CONFIG_NET_IPv4
    struct in_addr ipv4;
#endif
#ifdef CONFIG_NET_IPv6
    struct in6_addr ipv6;
#endif
  } u;
};

/* The type of the callback from dns_foreach_nameserver() */

typedef CODE int (*dns_callback_t)(FAR void *arg,
                                   FAR struct sockaddr *addr,
                                   FAR socklen_t addrlen);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: dns_add_nameserver
 *
 * Description:
 *   Configure a DNS server to use for queries.  Set the port number to zero
 *   to use the default DNS server port.
 *
 ****************************************************************************/

int dns_add_nameserver(FAR const struct sockaddr *addr, socklen_t addrlen);

/****************************************************************************
 * Name: dns_del_nameserver
 *
 * Description:
 *   Remove a DNS server so it is no longer available for further use.
 *
 ****************************************************************************/
/* REVISIT: Not implemented */

/****************************************************************************
 * Name: dns_foreach_nameserver
 *
 * Description:
 *   Traverse each nameserver entry in the resolv.conf file and perform the
 *   the provided callback.
 *
 ****************************************************************************/

int dns_foreach_nameserver(dns_callback_t callback, FAR void *arg);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NET_DNS_H */
