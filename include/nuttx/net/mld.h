/****************************************************************************
 * include/nuttx/net/mld.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *   RFC2710
 *   RFC3810 (version 2)
 *
 * Includes some definitions that a compatible with the LGPL GNU C Library
 * header file of the same name.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_NET_MLD_H
#define __INCLUDE_NUTTX_NET_MLD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/icmpv6.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum response code bit definitions.
 *
 *  If MRC < 32768
 *   MRD = MRC
 *  If  >= 32768
 *   MRD = (mantissa | 0x1000) << (exponent + 3)
 */

#define MLD_MRC_ONE            (1 << 15) /* Bit 15: Always one */
#define MLD_MRC_EXP_SHIFT      (12)      /* Bits 12-14: exponent */
#define MLD_MRC_EXP_MASK       (7 << MLD_MRC_EXP_SHIFT)
#  define MLD_MRC_EXP(n)       ((uint16_t)(n - 3) << MLD_MRC_EXP_SHIFT)
#define MLD_MRC_MANT_SHIFT     (0)       /* Bits 0-11: mantissa */
#define MLD_MRC_MANT_MASK      (0xfff << MLD_MRC_MANT_SHIFT)
#  define MLD_MRC_MANT(n)      ((uint16_t)(n) << MLD_MRC_MANT_SHIFT)

/* MRD conversion (for the case of MRC >= 32768) */

#define MLD_MRC_GETEXP(mrc)    (((mrc) & MLD_MRC_EXP_MASK) >> MLD_MRC_EXP_SHIFT)
#define MLD_MRC_GETMANT(mrc)   (((mrc) & MLD_MRC_MANT_MASK) >> MLD_MRC_MANT_SHIFT)
#define MLD_MRD_VALUE(mrc) \
  ((MLD_MRC_GETMANT(mrc) | 0x1000) << (MLD_MRC_GETEXP(mrc) + 3))

/* Querier's Query Interval Code
 *
 * If If QQIC < 128
 *   QQI = QQIC
 * If QQIC >= 128
 *   QQI = (mantissa | 0x10) << (exponent + 3)
 */

#define MLD_QQIC_ONE           (1 << 15) /* Bit 15: Always one */
#define MLD_QQIC_EXP_SHIFT     (4)       /* Bits 4-6: exponent */
#define MLD_QQIC_EXP_MASK      (7 << MLD_QQIC_EXP_SHIFT)
#  define MLD_QQIC_EXP(n)      ((uint16_t)(n - 3) << MLD_QQIC_EXP_SHIFT)
#define MLD_QQIC_MANT_SHIFT    (0)       /* Bits 0-4: mantissa */
#define MLD_QQIC_MANT_MASK     (15 << MLD_QQIC_MANT_SHIFT)
#  define MLD_QQIC_MANT(n)     ((uint8_t)(n) << MLD_QQIC_MANT_SHIFT)

/* QQI conversion (for the case of QQIC >= 128) */

#define MLD_QQIC_GETEXP(qqic)  (((qqic) & MLD_QQIC_EXP_MASK) >> MLD_QQIC_EXP_SHIFT)
#define MLD_QQIC_GETMANT(qqic) (((qqic) & MLD_QQIC_MANT_MASK) >> MLD_QQIC_MANT_SHIFT)
#define MLD_QQI_VALUE(qqic) \
  ((MLD_QQIC_GETMANT(qqic) | 0x1000) << (MLD_QQIC_GETEXP(qqic) + 3))

/* struct mld_multicast_listener_query_s flag definitions */

#define MLD_FLAG_S             (1 << 3)  /* Bit 3:  Suppress Router-Side Processing */
#define MLD_FLAG_QRV_SHIFT     (0)       /* Bits 0-2: Querier's Robustness Variable */
#define MLD_FLAG_QRV_MASK      (7 << MLD_FLAG_QRV_SHIFT)
#  define MLD_FLAG_QRV(n)      ((uint8_t)(n) << MLD_FLAG_QRV_SHIFT)

/* struct mld_mcast_addrec_v2_s record type definitions (RFC 3810).
 *
 * A "Current State Record" is sent by a node in response to a Query
 * received on an interface.  It reports the current listening state of that
 * interface, with respect to a single multicast address.  The Record Type
 * of a Current State Record may be one of the following two values:
 */

#define MODE_IS_INCLUDE        (1)       /* Indicates that the interface has a
                                          * filter mode of INCLUDE . */
#define MODE_IS_EXCLUDE        (2)       /* Indicates that the interface has a
                                          * filter mode of EXCLUDE. */

/* A "Filter Mode Change Record" is sent by a node there is a change of the
 * filter mode (from INCLUDE to EXCLUDE, or from EXCLUDE to INCLUDE).  The
 * Record Type of a Filter Mode Change Record may be one of the following
 * two values:
 */

#define CHANGE_TO_INCLUDE_MODE (3)       /* Iindicates that the interface has
                                          * changed to INCLUDE filter mode. */
#define CHANGE_TO_EXCLUDE_MODE (4)       /* Indicates that the interface has
                                          * changed to EXCLUDE filter mode. */

/* A "Source List Change Record" is sent by a node whenever a change of
 * source list that is *not* coincident with a change of filter mode.  The
 * Record Type of a Source List Change Record may be one of the following
 * two values:
 */

#define ALLOW_NEW_SOURCES      (5)       /* Indicates that the Source Addresses
                                          * contain a list of additional sources
                                          * that the node wishes to listen to. */
#define BLOCK_OLD_SOURCES      (6)       /* Indicates that the Source Addresses
                                          * contain a list of the sources that
                                          * the node no longer wishes to listen
                                          * to. */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Multicast Listener Queries are sent by multicast routers in Querier State
 * to query the multicast listening state of neighboring interfaces (RFC 3810).
 *
 * There are three variants of the Query message:
 *
 * 1. A "General Query" is sent by the Querier to learn which multicast
 *    addresses have listeners on an attached link.  In a General Query,
 *    both the Multicast Address field and the Number of Sources (N)
 *    field are zero.
 * 2. A "Multicast Address Specific Query" is sent by the Querier to
 *    learn if a particular multicast address has any listeners on an
 *    attached link.  In a Multicast Address Specific Query, the
 *    Multicast Address field contains the multicast address of
 *    interest, while the Number of Sources (N) field is set to zero.
 * 3. "Multicast Address and Source Specific Query" is sent by the
 *    Querier to learn if any of the sources from the specified list for
 *    the particular multicast address has any listeners on an attached
 *    link or not.  In a Multicast Address and Source Specific Query the
 *    Multicast Address field contains the multicast address of
 *    interest, while the Source Address [i] field(s) contain(s) the
 *    source address(es) of interest.
 *
 * All queries have the following format:
 */

struct mld_mcast_listen_query_s
{
  uint8_t  type;             /* Message Type: ICMPV6_MCAST_LISTEN_QUERY */
  uint8_t  reserved1;        /* Reserved, must be zero on transmission */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint16_t mrc;              /* Maximum response code */
  uint16_t reserved2;        /* Reserved, must be zero on transmission */
  net_ipv6addr_t mcast;      /* 128-bit IPv6 multicast address */
  uint8_t  flags;            /* See S and QRV flag definitions */
  uint8_t  qqic;             /* Querier's Query Interval Cod */
  uint16_t nsources;         /* Number of sources that follow */
  net_ipv6addr_t srcaddr[1]; /* Array of source IPv6 address (actual size is
                              * nsources) */
};

/* The actual size of the query structure depends on the number of sources */

#define SIZEOF_MLD_MCAST_LISTEN_QUERY_S(nsources) \
  (sizeof(struct mld_multicast_listener_query_s) + \
   sizeof(net_ipv6addr_t) * ((nsources) - 1)

/* Multicast Listener Reports are sent by IP nodes to report (to neighboring
 * routers) the current multicast listening state, or changes in the
 * multicast listening state, of their interfaces.
 *
 * Version 1 Multicast Listener Reports (RFC 2710)
 */

struct mld_mcast_listen_report_v1_s
{
  uint8_t  type;             /* Message Type: ICMPV6_MCAST_LISTEN_REPORT */
  uint8_t  reserved1;        /* Reserved, must be zero on transmission */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint16_t reserved2;        /* Reserved, must be zero on transmission */
  uint16_t reserved3;        /* Reserved, must be zero on transmission */
  net_ipv6addr_t mcastaddr;  /* Multicast address */
};

/* Version 2 Multicast Listener Reports (RFC 3810). */
/* This is the form of the address record used in the listener report */

struct mld_mcast_addrec_v2_s
{
  uint8_t rectype;           /* Record type.  See definitions above. */
  uint8_t auxdatlen;         /* Auxiliary data length */
  uin16_t nsources;          /* Number of sources */
  net_ipv6addr_t mcast;      /* 128-bit IPv6 multicast address */
  net_ipv6addr_t srcaddr[1]; /* Array of source IPv6 address (actual size is
                              * nsources) */

  /* Auxiliary data may follow the list of address records. */
};

/* The actual size of the query structure depends on the number of sources
 * as well as the size of any auxiliary data.
 */

#define SIZEOF_MLD_MCAST_ADDREC_V2_S(nsources, auxdatlen) \
  (sizeof(struct mld_mcast_addrec_v2_s) + \
   sizeof(net_ipv6addr_t) * ((nsources) - 1) + \
   (auxdatlen)

struct mld_mcast_listen_report_v2_s
{
  uint8_t  type;             /* Message Type: ICMPV6_MCAST_LISTEN_REPORT_V2 */
  uint8_t  reserved1;        /* Reserved, must be zero on transmission */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint16_t reserved2;        /* Reserved, must be zero on transmission */
  uint16_t naddrec;          /* Number of multicast address records */

  /* List of multicast address records (actual size is naddrec) */

  struct mld_mcast_addrec_v2_s addrec[1];
};

/* The actual size of the listener report depends on the sum of the
 * size of each variable length address record (addreclen).
 */

#define SIZEOF_MLD_MCAST_ADDREC_V2_S(addreclen) \
  (sizeof(struct mld_mcast_addrec_v2_s) - \
   sizeof(net_ipv6addr_t) + \
   (addreclen)

/* Version 1 Multicast Listener Done (RFC 2710) */

struct mld_mcast_listen_done_v1_s
{
  uint8_t  type;             /* Message Type: ICMPV6_MCAST_LISTEN_DONE */
  uint8_t  reserved1;        /* Reserved, must be zero on transmission */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint16_t reserved2;        /* Reserved, must be zero on transmission */
  uint16_t reserved3;        /* Reserved, must be zero on transmission */
  net_ipv6addr_t mcastaddr;  /* Multicast address */
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_MLD_H */
