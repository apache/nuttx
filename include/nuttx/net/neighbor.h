/****************************************************************************
 * include/nuttx/net/neighbor.h
 * Definitions for use with IPv6 Neighor Table
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_NET_NEIGHBOR_H
#define __INCLUDE_NUTTX_NET_NEIGHBOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <net/ethernet.h>

#include <nuttx/clock.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_IPv6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_IPv6_NCONF_ENTRIES
#  define CONFIG_NET_IPv6_NCONF_ENTRIES 8
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Describes the link layer address */

struct neighbor_addr_s
{
  uint8_t                    na_lltype;  /* See enum net_lltype_e */
  uint8_t                    na_llsize;  /* Link layer header size */

  union
  {
#ifdef CONFIG_NET_ETHERNET
    struct ether_addr        na_ethernet;
#endif
#ifdef CONFIG_NET_6LOWPAN
    struct netdev_maxaddr_s  na_sixlowpan;
#endif
    uint8_t                  na_addr[1];
  } u;
};

/* This structure describes on entry in the neighbor table.  This is intended
 * for internal use within the Neighbor implementation.
 *
 * An unused entry can be detected by all zero entries, particularly the
 * ne_ipaddr (the IPv6 unspecified address) which should never be zero for a
 * real neighbor.
 */

struct neighbor_entry_s
{
  net_ipv6addr_t         ne_ipaddr;  /* IPv6 address of the Neighbor */
  struct neighbor_addr_s ne_addr;    /* Link layer address of the Neighbor */
  clock_t                ne_time;    /* For aging, units of tick */
};

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

#endif /* CONFIG_NET_LOOPBACK */
#endif /* __INCLUDE_NUTTX_NET_NEIGHBOR_H */
