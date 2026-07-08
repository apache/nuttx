/****************************************************************************
 * arch/arm/src/common/ameba/wifi/include/lwip_netconf.h
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
 * NuttX port shim for the Realtek WHC host WiFi stack.
 *
 * The precompiled Realtek wifi host libraries (lib_wifi_whc_ap, ...) and a
 * few SDK wifi sources we compile here (rtw_task_size.c, ...) include
 * <lwip_netconf.h> and embed lwIP's pbuf structs BY VALUE, e.g.
 *
 *     struct rtw_pbuf { struct pbuf_custom p; struct sk_buff *skb;
 *                       u8 busy; };
 *
 * NuttX provides its own native BSD socket stack, so lwIP is NOT linked.
 * This shim therefore supplies ONLY:
 *   1. the lwIP struct/enum layouts that cross the precompiled-lib ABI
 *      boundary (struct pbuf / pbuf_custom and the pbuf_layer/pbuf_type
 *      enums passed to pbuf_alloc), copied VERBATIM from the pinned SDK's
 *      component/lwip/lwip_v2.1.2 -- keep in sync if the SDK lwIP version is
 *      bumped; and
 *   2. prototypes of the lwIP-named glue functions that NuttX implements in
 *      ameba_wifi_depend.c (pbuf_alloc / netif_adapter_wifi_recv_whc /
 *      lwip_netif_set_up).
 *
 * It deliberately does NOT include lwip/opt.h or the full lwipconf.h header
 * tree -- only the ABI is reproduced, not the lwIP API.
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_WIFI_INCLUDE_LWIP_NETCONF_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_WIFI_INCLUDE_LWIP_NETCONF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* lwIP arch integer types (lwip/arch.h) used by the pbuf layout. */

#ifndef LWIP_ARCH_TYPES_DEFINED
#define LWIP_ARCH_TYPES_DEFINED
typedef uint8_t  u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t   s8_t;
typedef int16_t  s16_t;
typedef int32_t  s32_t;
#endif

/* lwIP default reference-count width (lwip/opt.h: LWIP_PBUF_REF_T = u8_t). */

#define LWIP_PBUF_REF_T u8_t

/* pbuf_layer header-length constituents.  lwIP defaults with ETH_PAD_SIZE 0
 * and IPv6 enabled on this SoC (wifi_api exposes lwip_ipv6_enabled).  These
 * feed the pbuf_layer enum below; they affect TX headroom only and are not
 * consulted on the control-plane path.
 */

#define PBUF_LINK_ENCAPSULATION_HLEN 0
#define PBUF_LINK_HLEN               14
#define PBUF_IP_HLEN                 40
#define PBUF_TRANSPORT_HLEN          20

/* pbuf_type bit flags (lwip/pbuf.h). */

#define PBUF_TYPE_FLAG_STRUCT_DATA_CONTIGUOUS       0x80
#define PBUF_TYPE_FLAG_DATA_VOLATILE                0x40
#define PBUF_TYPE_ALLOC_SRC_MASK                    0x0F
#define PBUF_ALLOC_FLAG_RX                          0x0100
#define PBUF_ALLOC_FLAG_DATA_CONTIGUOUS             0x0200
#define PBUF_TYPE_ALLOC_SRC_MASK_STD_HEAP           0x00
#define PBUF_TYPE_ALLOC_SRC_MASK_STD_MEMP_PBUF      0x01
#define PBUF_TYPE_ALLOC_SRC_MASK_STD_MEMP_PBUF_POOL 0x02

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Enumeration of pbuf layers (lwip/pbuf.h) -- exact values preserved so the
 * layer hint passed by the precompiled libs is interpreted identically.
 */

typedef enum
{
  PBUF_TRANSPORT = PBUF_LINK_ENCAPSULATION_HLEN + PBUF_LINK_HLEN +
                   PBUF_IP_HLEN + PBUF_TRANSPORT_HLEN,
  PBUF_IP        = PBUF_LINK_ENCAPSULATION_HLEN + PBUF_LINK_HLEN +
                   PBUF_IP_HLEN,
  PBUF_LINK      = PBUF_LINK_ENCAPSULATION_HLEN + PBUF_LINK_HLEN,
  PBUF_RAW_TX    = PBUF_LINK_ENCAPSULATION_HLEN,
  PBUF_RAW       = 0
} pbuf_layer;

/* Enumeration of pbuf types (lwip/pbuf.h) -- exact values preserved. */

typedef enum
{
  PBUF_RAM  = (PBUF_ALLOC_FLAG_DATA_CONTIGUOUS |
               PBUF_TYPE_FLAG_STRUCT_DATA_CONTIGUOUS |
               PBUF_TYPE_ALLOC_SRC_MASK_STD_HEAP),
  PBUF_ROM  = PBUF_TYPE_ALLOC_SRC_MASK_STD_MEMP_PBUF,
  PBUF_REF  = (PBUF_TYPE_FLAG_DATA_VOLATILE |
               PBUF_TYPE_ALLOC_SRC_MASK_STD_MEMP_PBUF),
  PBUF_POOL = (PBUF_ALLOC_FLAG_RX |
               PBUF_TYPE_FLAG_STRUCT_DATA_CONTIGUOUS |
               PBUF_TYPE_ALLOC_SRC_MASK_STD_MEMP_PBUF_POOL)
} pbuf_type;

/* struct pbuf / pbuf_custom -- layout copied verbatim from lwip/pbuf.h of
 * the pinned SDK.  Embedded by value in struct rtw_pbuf, so the layout MUST
 * match.
 */

struct pbuf
{
  struct pbuf    *next;          /* next pbuf in singly linked chain         */
  void           *payload;       /* pointer to the actual data in the buffer */
  u16_t           tot_len;       /* total length of this + all next buffers  */
  u16_t           len;           /* length of this buffer                    */
  u8_t            type_internal; /* pbuf type and allocation source flags    */
  u8_t            flags;         /* misc flags                               */
  LWIP_PBUF_REF_T ref;           /* reference count                          */
  u8_t            if_idx;        /* input netif's index (incoming packets)   */
};

typedef void (*pbuf_free_custom_fn)(struct pbuf *p);

struct pbuf_custom
{
  struct pbuf         pbuf;                 /* the actual pbuf     */
  pbuf_free_custom_fn custom_free_function; /* called by pbuf_free */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* lwIP-named glue implemented by NuttX in ameba_wifi_depend.c. */

struct pbuf *pbuf_alloc(pbuf_layer layer, u16_t length, pbuf_type type);
void         pbuf_free(struct pbuf *p);

/* STA netif index -- only passed to the no-op lwIP link/dhcp glue below, so
 * the value is immaterial (the SDK enum has it = STA_WLAN_INDEX = 0).
 */

#ifndef NETIF_WLAN_STA_INDEX
#define NETIF_WLAN_STA_INDEX 0
#endif

/* lwIP netif link hooks referenced by the precompiled lib_wifi_whc_ap.a
 * (wifi_start_ap/wifi_stop_ap, built with CONFIG_LWIP_LAYER on).  NuttX's
 * netdev (ameba_wlan.c) owns carrier via ifup, so these are no-ops
 * (ameba_wifi_depend.c) -- they exist only to satisfy the link.
 */

void lwip_netif_set_link_up(unsigned char idx);
void lwip_netif_set_link_down(unsigned char idx);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_WIFI_INCLUDE_LWIP_NETCONF_H */
