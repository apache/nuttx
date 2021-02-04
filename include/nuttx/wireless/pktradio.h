/****************************************************************************
 * include/nuttx/wireless/pktradio.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_PKTRADIO_H
#define __INCLUDE_NUTTX_WIRELESS_PKTRADIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/netdev.h>
#include <nuttx/wireless/wireless.h>

#ifdef CONFIG_WIRELESS_PKTRADIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Packet radio network device IOCTL commands. */

#ifndef WL_NPKTRADIOCMDS != 3
#  error Incorrect setting for number of PktRadio IOCTL commands
#endif

/* SIOCPKTRADIOGGPROPS
 *   Description:   Get the radio properties
 *   Input:         Pointer to read-write instance of struct pktradio_ifreq_s
 *   Output:        Properties returned in struct pktradio_ifreq_s instance
 */

#define SIOCPKTRADIOGGPROPS  _WLIOC(WL_PKTRADIOFIRST)

/* SIOCPKTRADIOGSNODE
 *   Description:   Set the radio node address
 *   Input:         Pointer to read-only instance of struct pktradio_ifreq_s
 *   Output:        None
 */

#define SIOCPKTRADIOSNODE    _WLIOC(WL_PKTRADIOFIRST + 1)

/* SIOCPKTRADIOGGNODE
 *   Description:   Get the radio node address
 *   Input:         Pointer to read-write instance of struct pktradio_ifreq_s
 *   Output:        Node address return in struct pktradio_ifreq_s instance
 */

#define SIOCPKTRADIOGNODE    _WLIOC(WL_PKTRADIOFIRST + 2)

/* Memory Pools */

#define PKTRADIO_POOL_PREALLOCATED  0
#define PKTRADIO_POOL_DYNAMIC       1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes an address used by the packet radio.  There is no standard
 * size for such an address.  Hence, it is represented simply as a array of
 * bytes.
 *
 * NOTE: This MUST be the same as the struct netdev_varaddr_s as defined in
 * netdev.h.  It is duplicated here for no particularly good reason other
 * than to maintain a clean PktRadio namespace.
 */

struct pktradio_addr_s
{
  uint8_t pa_addr[RADIO_MAX_ADDRLEN];
  uint8_t pa_addrlen;                   /* Length of the following address */
};

/* Different packet radios may have different properties.  If there are
 * multiple packet radios, then those properties have to be queried at
 * run time.  This information is provided to the 6LoWPAN network via the
 * following structure.
 *
 * NOTE: This MUST be the same as the struct radiodev_properties_s as
 * defined in radiodev.h.  It is duplicated here with a different name in
 * order to avoid circular header file inclusion.
 */

struct pktradio_properties_s
{
  uint8_t pp_addrlen;                 /* Length of an address */
  uint8_t pp_pktlen;                  /* Fixed packet/frame size (up to 255) */
  struct pktradio_addr_s pp_mcast;    /* Multicast address */
  struct pktradio_addr_s pp_bcast;    /* Broadcast address */
#ifdef CONFIG_NET_STARPOINT
  struct pktradio_addr_s pp_hubnode;  /* Address of the hub node in a star */
#endif
};

/* This is the structure passed with all packet radio IOCTL commands.
 * NOTE: This is merely a placeholder for now.
 */

struct pktradio_ifreq_s
{
  char                           pifr_name[IFNAMSIZ];  /* Network device name (e.g. "wpan0") */
  union
  {
    struct pktradio_addr_s       pifru_hwaddr;         /* Radio node address */
    struct pktradio_properties_s pifru_props;          /* Radio properties */
  } pifr_u;
};

#define pifr_hwaddr              pifr_u.pifru_hwaddr   /* Radio node address */
#define pifr_props               pifr_u.pifru_props    /* Radio properties */

/* This is the form of the meta data that provides the radio-specific
 * information necessary to send and receive packets to and from the radio.
 */

struct iob_s;  /* Forward reference */

struct pktradio_metadata_s
{
  struct pktradio_metadata_s *pm_flink; /* Supports a singly linked list */
  struct pktradio_addr_s pm_src;        /* Source of the packet */
  struct pktradio_addr_s pm_dest;       /* Destination of the packet */
  FAR struct iob_s *pm_iob;             /* Contained IOB */
  uint8_t pm_pool;                      /* See PKTRADIO_POOL_* definitions */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pktradio_metadata_initialize
 *
 * Description:
 *   This function initializes the meta-data allocator.  This function must
 *   be called early in the initialization sequence before any radios
 *   begin operation.
 *
 *   This function is idempotent:  It may be called numerous times.  The
 *   initialization will be performed only on the first time that it is
 *   called.  Therefore, it may be called during packet radio driver
 *   initialization, even if there are multiple packet radio drivers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pktradio_metadata_initialize(void);

/****************************************************************************
 * Name: pktradio_metadata_allocate
 *
 * Description:
 *   The pktradio_metadata_allocate function will get a free meta-data
 *   structure for use by the packet radio.
 *
 *   This function will first attempt to allocate from the g_free_metadata
 *   list.  If that the list is empty, then the meta-data structure will be
 *   allocated from the dynamic memory pool.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A reference to the allocated metadata structure.  All user fields in
 *   this structure have been zeroed.  On a failure to allocate, NULL is
 *   returned.
 *
 ****************************************************************************/

FAR struct pktradio_metadata_s *pktradio_metadata_allocate(void);

/****************************************************************************
 * Name: pktradio_metadata_free
 *
 * Description:
 *   The pktradio_metadata_free function will return a metadata structure
 *   to the free list of  messages if it was a pre-allocated metadata
 *   structure. If the metadata structure was allocated dynamically it will
 *   be deallocated.
 *
 * Input Parameters:
 *   metadata - metadata structure to free
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pktradio_metadata_free(FAR struct pktradio_metadata_s *metadata);

/****************************************************************************
 * Name: pktradio_loopback
 *
 * Description:
 *   Initialize and register the Ieee802.15.4 MAC loopback network driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_PKTRADIO_LOOPBACK
int pktradio_loopback(void);
#endif

#endif /* CONFIG_WIRELESS_PKTRADIO */
#endif /* __INCLUDE_NUTTX_WIRELESS_PKTRADIO_H */
