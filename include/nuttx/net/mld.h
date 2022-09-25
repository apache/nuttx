/****************************************************************************
 * include/nuttx/net/mld.h
 * Multicast Listener Discovery (MLD) Definitions
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

#ifndef __INCLUDE_NUTTX_NET_MLD_H
#define __INCLUDE_NUTTX_NET_MLD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <nuttx/queue.h>
#include <nuttx/wdog.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/icmpv6.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MLD is a sub-protocol of ICMPv6, that is, MLD message types are a subset
 * of ICMPv6 messages, and MLD messages are identified in IPv6 packets by a
 * preceding Next Header value of IP_PROTO_ICMP6 (58).  All MLD messages
 * MUST be sent with a link-local IPv6 Source Address, an IPv6 Hop Limit of
 * 1, and an IPv6 Router Alert option (RFC2711) in a Hop-by-Hop Options
 * header.  (The Router Alert option is necessary to cause routers to
 * examine MLD messages sent to IPv6 multicast addresses in which the
 * routers themselves have no interest.)  MLD Reports can be sent with the
 * source address set to the unspecified address (RFC3513), if a valid
 * link-local IPv6 source address has not been acquired yet for the sending
 * interface.
 */

#define MLD_TTL                (1)

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

/* Conversion of MRC to MRD in milliseconds (for the case of MRC >= 32768) */

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

/* Conversion of QQIC to QQI in seconds (for the case of MRC >= 128) */

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

/* MLD default values (RFC 3810) of tunable parameters.
 *
 * MLD_ROBUSTNESS         Robustness Variable.  The Robustness Variable
 *                        allows tuning for the expected packet loss on
 *                        a link.
 * MLD_QUERY_MSEC         Query Interval. The Query Interval variable
 *                        denotes the interval between General Queries
 *                        sent by the Querier.
 * MLD_QRESP_MSEC         Query Response Interval.  The Maximum Response
 *                        Delay used to calculate the Maximum Response
 *                        Code inserted into the periodic General Queries.
 * MLD_MCASTLISTEN_MSEC   Multicast Address Listening Interval.  The
 *                        amount of time that must pass before a multicast
 *                        router decides there are no more listeners of a
 *                        multicast address or a particular source on a
 *                        link.
 * MLD_OQUERY_MSEC        Other Querier Present Timeout.  The length of
 *                        time that must pass before a multicast router
 *                        decides that there is no longer another multicast
 *                        router which should be the Querier.
 * MLD_STARTUP_MSEC       Startup Query Interval.  The interval between
 *                        General Queries sent by a Querier on startup.
 * MLD_STARTUP_COUNT      Startup Query Count.  The number of Queries sent
 *                        out on startup separated by the Startup Query
 *                        Interval.
 * MLD_LASTLISTEN_MSEC    Last Listener Query Interval.  The Maximum
 *                        Response Delay used to calculate the Maximum
 *                        Response Code inserted into Multicast Address
 *                        Specific Queries sent in response to Version 1
 *                        Multicast Listener Done messages.  It is also
 *                        the Maximum Response Delay used to calculate
 *                        the Maximum Response Code inserted into Multicast
 *                        Address and Source Specific Query messages.
 * MLD_LASTLISTEN_COUNT   Last Listener Query Count.  The number of
 *                        Multicast Address Specific Queries sent before
 *                        the router assumes there are no local listeners.
 *                        The Last Listener Query Count is also the number
 *                        of Multicast Address and Source Specific Queries
 *                        sent before the router assumes there are no
 *                        listeners for a particular source.
 * MLD_LLQUERY_MSEC       Last Listener Query Time.  The time value
 *                        represented by the Last Listener Query Interval
 *                        multiplied by Last Listener Query Count.
 * MLD_UNSOLREPORT_MSEC   Unsolicited Report Interval.  The time between
 *                        repetitions of a node's initial report of
 *                        interest in a multicast address.
 * MLD_V1PRESENT_MSEC(m)  Older Version Querier Present Timeout. The time-
 *                        out for transitioning a host back to MLDv2 Host
 *                        Compatibility Mode.  'm' is the Query Interval
 *                        in the last Query received in units of msec.
 * MLD_V1HOST_MSEC        Older Version Host Present Timeout.  The time-out
 *                        for transitioning a router back to MLDv2 Multicast
 *                        Address Compatibility Mode for a specific multicast
 *                        address.  When an MLDv1 report is received for that
 *                        multicast address, routers set their Older Version
 *                        Host Present Timer to the Older Version Host
 *                        Present Timeout.
 */

#define MLD_ROBUSTNESS         (2)
#define MLD_QUERY_MSEC         (125 * 1000)
#define MLD_QRESP_SEC          (10)
#define MLD_QRESP_MSEC         (MLD_QRESP_SEC * 1000)
#define MLD_MCASTLISTEN_MSEC   (MLD_ROBUSTNESS * MLD_QUERY_MSEC + MLD_QRESP_MSEC)
#define MLD_OQUERY_MSEC        (MLD_ROBUSTNESS * MLD_QUERY_MSEC + (MLD_QRESP_MSEC / 2))
#define MLD_STARTUP_MSEC       (MLD_QUERY_MSEC / 4)
#define MLD_STARTUP_COUNT      MLD_ROBUSTNESS
#define MLD_LASTLISTEN_MSEC    (1000)
#define MLD_LASTLISTEN_COUNT   MLD_ROBUSTNESS
#define MLD_LLQUERY_MSEC       (MLD_LASTLISTEN_MSEC * MLD_LASTLISTEN_COUNT)
#define MLD_UNSOLREPORT_MSEC   (1000)
#define MLD_UNSOLREPORT_COUNT  MLD_ROBUSTNESS
#define MLD_V1PRESENT_MSEC(m)  (MLD_ROBUSTNESS * (m) + MLD_QRESP_MSEC)
#define MLD_V1HOST_MSEC        (MLD_ROBUSTNESS * MLD_QUERY_MSEC + MLD_QRESP_MSEC)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Multicast Listener Queries are sent by multicast routers in Querier State
 * to query the multicast listening state of neighboring interfaces
 * (RFC 3810).
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
  /* The initial fields are common for MLDv1 and MLDv2 (24-bytes) */

  uint8_t  type;             /* Message Type: ICMPV6_MCAST_LISTEN_QUERY */
  uint8_t  reserved1;        /* Reserved, must be zero on transmission */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint16_t mrc;              /* Maximum response code */
  uint16_t reserved2;        /* Reserved, must be zero on transmission */
  net_ipv6addr_t grpaddr;    /* 128-bit IPv6 multicast group address */

  /* These fields apply only to the MLDv2 query */

  uint8_t  flags;            /* See S and QRV flag definitions */
  uint8_t  qqic;             /* Querier's Query Interval Code */
  uint16_t nsources;         /* Number of sources that follow */
  net_ipv6addr_t srcaddr[1]; /* Array of source IPv6 address (actual size is
                              * nsources) */
};

/* The actual size of the query structure depends on the number of sources */

#define SIZEOF_MLD_MCAST_LISTEN_QUERY_S(nsources) \
  (sizeof(struct mld_mcast_listen_query_s) + \
   sizeof(net_ipv6addr_t) * ((nsources) - 1))

/* Multicast Listener Reports are sent by IP nodes to report (to neighboring
 * routers) the current multicast listening state, or changes in the
 * multicast listening state, of their interfaces.
 *
 * Version 1 Multicast Listener Report (RFC 2710)
 */

struct mld_mcast_listen_report_v1_s
{
  uint8_t  type;             /* Message Type: ICMPV6_MCAST_LISTEN_REPORT_V1 */
  uint8_t  reserved1;        /* Reserved, must be zero on transmission */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint16_t reserved2;        /* Reserved, must be zero on transmission */
  uint16_t reserved3;        /* Reserved, must be zero on transmission */
  net_ipv6addr_t mcastaddr;  /* Multicast address */
};

/* Version 2 Multicast Listener Report (RFC 3810). */

/* This is the form of the address record used in the listener report */

struct mld_mcast_addrec_v2_s
{
  uint8_t rectype;           /* Record type.  See definitions above. */
  uint8_t auxdatlen;         /* Auxiliary data length */
  uint16_t nsources;         /* Number of sources */
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
   (auxdatlen))

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

#define SIZEOF_MLD_MCAST_LISTEN_REPORT_V2_S(addreclen) \
  (sizeof(struct mld_mcast_listen_report_v2_s) - \
   sizeof(struct mld_mcast_addrec_v2_s) + \
   (addreclen))

/* Multicast Listener Done (RFC 2710) */

struct mld_mcast_listen_done_s
{
  uint8_t  type;             /* Message Type: ICMPV6_MCAST_LISTEN_DONE */
  uint8_t  reserved1;        /* Reserved, must be zero on transmission */
  uint16_t chksum;           /* Checksum of ICMP header and data */
  uint16_t reserved2;        /* Reserved, must be zero on transmission */
  uint16_t reserved3;        /* Reserved, must be zero on transmission */
  net_ipv6addr_t mcastaddr;  /* Multicast address */
};

/* This structure represents the overall MLD state for a single network.
 * This structure in included within the net_driver_s structure.
 *
 * There will be a group for the all systems group address but this
 * will not run the state machine as it is used to kick off reports
 * from all the other groups
 */

struct mld_netdev_s
{
  sq_queue_t grplist;                /* MLD group list */
  struct wdog_s gendog;              /* General query timer */
  struct wdog_s v1dog;               /* MLDv1 compatibility timer */
  uint8_t flags;                     /* See MLD_ flags definitions */
};

#ifdef CONFIG_NET_STATISTICS
/* MLD statistic counters */

struct mld_stats_s
{
  net_stats_t njoins;                /* Requests to join a group */
  net_stats_t nleaves;               /* Requests to leave a group */
  net_stats_t query_sched;           /* General QUERY packets scheduled */
  net_stats_t report_sched;          /* Unsolicited REPORT packets scheduled */
  net_stats_t done_sched;            /* DONE packets scheduled */
  net_stats_t query_sent;            /* General QUERY packets sent */
  net_stats_t v1report_sent;         /* Version 1 REPORT packets sent */
  net_stats_t v2report_sent;         /* Version 2 REPORT packets sent */
  net_stats_t done_sent;             /* DONE packets sent */
  net_stats_t gm_query_received;     /* General multicast QUERY received */
  net_stats_t mas_query_received;    /* Multicast Address Specific QUERY received */
  net_stats_t mass_query_received;   /* Multicast Address and Source Specific QUERY received */
  net_stats_t ucast_query_received;  /* Unicast query received */
  net_stats_t bad_query_received;    /* Unhandled query received */
  net_stats_t v1report_received;     /* Version 1 REPORT packets received */
  net_stats_t v2report_received;     /* Version 2 REPORT packets received */
  net_stats_t done_received;         /* DONE packets received */
};

# define MLD_STATINCR(p) ((p)++)
#else
# define MLD_STATINCR(p)
#endif

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
