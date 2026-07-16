/* build_config.h - NuttX shim over the SDK wpa supplicant build config.
 * Chains to wifi_manager/wpas/build_config.h and supplies the byte-order
 * macros that src/utils/common.h expects (RISC-V rv32 is little-endian).
 */

#ifndef _GDWIFI_BUILD_CONFIG_SHIM_H_
#define _GDWIFI_BUILD_CONFIG_SHIM_H_

#include_next "build_config.h"

/* The port is WPA2-only: the SAE (WPA3) handshake faults inside the
 * prebuilt supplicant (load access fault in the elliptic-curve math).
 * Undefining CONFIG_WPA3_SAE activates the vendor's own fallback in
 * wifi_netlink.c: the SAE/OWE AKMs are stripped from the connection
 * config and the SAE auth algorithm is never selected, so a
 * WPA3-transition AP associates through WPA2-PSK and an SAE-only AP is
 * rejected by the AP instead of crashing the supplicant.  (The glue also
 * refuses SAE-only networks up front with a clear error -- see
 * gdwifi_netdev.c.)
 */

#undef CONFIG_WPA3_SAE

#ifndef __LITTLE_ENDIAN
#define __LITTLE_ENDIAN 1234
#endif
#ifndef __BIG_ENDIAN
#define __BIG_ENDIAN 4321
#endif
#ifndef __BYTE_ORDER
#define __BYTE_ORDER __LITTLE_ENDIAN
#endif

#endif /* _GDWIFI_BUILD_CONFIG_SHIM_H_ */
