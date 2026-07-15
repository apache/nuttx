/* build_config.h - NuttX shim over the SDK wpa supplicant build config.
 * Chains to wifi_manager/wpas/build_config.h and supplies the byte-order
 * macros that src/utils/common.h expects (RISC-V rv32 is little-endian).
 */

#ifndef _GDWIFI_BUILD_CONFIG_SHIM_H_
#define _GDWIFI_BUILD_CONFIG_SHIM_H_

#include_next "build_config.h"

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
