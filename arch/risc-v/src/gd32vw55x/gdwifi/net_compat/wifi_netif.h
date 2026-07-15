/* wifi_netif.h - NuttX port shim.
 *
 * Shadows the header of the SDK lwIP port.  Here the IP stack is the NuttX
 * one: the binary seam with the MAC firmware lives in gdwifi_netdev.c.
 */

#ifndef _GDWIFI_COMPAT_WIFI_NETIF_H
#define _GDWIFI_COMPAT_WIFI_NETIF_H

#include <stdint.h>
#include <stdbool.h>

#include "gdwifi_netdev.h"

/* The SDK declares "struct netif net_if" inside the VIF and passes
 * &wvif->net_if around as an opaque handle -- the MAC firmware only stores
 * the pointer.  With the NuttX network stack there is no lwIP to define the
 * type, so we define it here.  It has to be a NAMED type (and not an
 * anonymous placeholder): the SDK sources declare "struct netif *" in
 * several places, and GCC 14 rejects the conversion from an anonymous
 * struct.
 */

struct netif
{
  void *nuttx_dev;      /* wlan0 net_driver_s */
};

/* Byte order: the SDK expects the lwIP macros */

#ifndef htons
#  define htons(x) ((uint16_t)((((x) & 0x00ffU) << 8) | (((x) & 0xff00U) >> 8)))
#  define ntohs(x) htons(x)
#endif

#ifndef htonl
#  define htonl(x) ((uint32_t)((((x) & 0x000000ffUL) << 24) | \
                               (((x) & 0x0000ff00UL) <<  8) | \
                               (((x) & 0x00ff0000UL) >>  8) | \
                               (((x) & 0xff000000UL) >> 24)))
#  define ntohl(x) htonl(x)
#endif

int  net_init(void);
void net_deinit(void);
int  net_if_get_name(void *net_if, char *name, int len);

/* lwIP macros used by the SDK sources (byte order / IP composition) */

#ifndef PP_HTONL
#  define PP_HTONL(x) ((uint32_t)((((x) & 0x000000ffUL) << 24) | \
                                  (((x) & 0x0000ff00UL) <<  8) | \
                                  (((x) & 0x00ff0000UL) >>  8) | \
                                  (((x) & 0xff000000UL) >> 24)))
#endif

#ifndef LWIP_MAKEU32
#  define LWIP_MAKEU32(a, b, c, d) (((uint32_t)((a) & 0xff) << 24) | \
                                    ((uint32_t)((b) & 0xff) << 16) | \
                                    ((uint32_t)((c) & 0xff) <<  8) | \
                                     (uint32_t)((d) & 0xff))
#endif

/* Interface state (the SDK used to call the lwIP netif_is_up) */

int netif_is_up(void *net_if);

/* Interface created/removed: with the NuttX netdev this already happened at
 * boot, so these become no-ops.
 */

int  net_if_add(void *net_if, const uint8_t *mac_addr, const uint32_t *ipaddr,
                const uint32_t *netmask, const uint32_t *gw, void *vif);
void net_if_remove(void *net_if);

/* DHCP server (softAP) -- out of scope for the STA port */

int  net_dhcpd_start(void *net_if);
void net_dhcpd_stop(void *net_if);

/* Used by the wifi_manager (SDK source) */

void  net_if_up(void *net_if);
void  net_if_down(void *net_if);
void *net_if_find_from_name(const char *name);
int   net_if_is_static_ip(void);
void  net_if_use_static_ip(bool static_ip);
const uint8_t *net_if_get_mac_addr(void *net_if);
int   net_init(void);

/* EAPOL: TX blocks until the MAC confirm */

int   net_l2_send(void *net_if, const uint8_t *data, int data_len,
                  uint16_t ethertype, const uint8_t *dst_addr, bool *ack);

/* IP configuration: with CONFIG_NET, NuttX is in charge */

void  net_if_set_ip(void *net_if, uint32_t ip, uint32_t mask, uint32_t gw);
int   net_if_get_ip(void *net_if, uint32_t *ip, uint32_t *mask, uint32_t *gw);
void  net_if_set_default(void *net_if);
int   net_dhcp_start(void *net_if);
void  net_dhcp_stop(void *net_if);
int   net_dhcp_release(void *net_if);
bool  net_dhcp_address_obtained(void *net_if);
void  net_if_send_gratuitous_arp(void *net_if);
uint16_t net_ip_chksum(const void *dataptr, int len);
int   net_set_dns(uint32_t dns_server);
int   net_get_dns(uint32_t *dns_server);

#endif /* _GDWIFI_COMPAT_WIFI_NETIF_H */
