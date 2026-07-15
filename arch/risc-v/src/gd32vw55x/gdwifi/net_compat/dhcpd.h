/* dhcpd.h - NuttX port shim (the SDK DHCP server belongs to lwIP and drops
 * out of the build; only the softAP would use it).  Stubs so that the
 * wifi_netlink compiles.
 */

#ifndef _GDWIFI_COMPAT_DHCPD_H
#define _GDWIFI_COMPAT_DHCPD_H

#include <stdint.h>

/* lwIP macros used by wifi_netlink.c (softAP IP configuration) */

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

void     dhcpd_delete_ipaddr_by_macaddr(uint8_t *mac_addr);
uint32_t dhcpd_find_ipaddr_by_macaddr(uint8_t *mac_addr);
void     dhcpd_set_dns_server(uint32_t dns);

#endif /* _GDWIFI_COMPAT_DHCPD_H */
