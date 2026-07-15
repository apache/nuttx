/* Empty shim: the SDK lwIP drops out of the build when CONFIG_NET=y.
 * The few functions still referenced live in wifi_netif.h/dhcpd.h.
 */
#ifndef _GDWIFI_COMPAT_LWIP_STUB_H
#define _GDWIFI_COMPAT_LWIP_STUB_H
#endif
