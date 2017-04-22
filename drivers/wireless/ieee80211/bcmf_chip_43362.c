#include "bcmf_chip_43362.h"
#include "bcm43362_constants.h"

#include <stdint.h>
#include <debug.h>
#include "bcmf_sdio_core.h"

#define WRAPPER_REGISTER_OFFSET  (0x100000)

uint32_t bcmf_43362_get_core_base_address(unsigned int core)
{
  switch (core)
    {
      case CHIPCOMMON_CORE_ID:
        return CHIPCOMMON_BASE_ADDRESS;
      case DOT11MAC_CORE_ID:
        return DOT11MAC_BASE_ADDRESS;
      case SDIOD_CORE_ID:
        return SDIO_BASE_ADDRESS;
      case WLAN_ARMCM3_CORE_ID:
        return WLAN_ARMCM3_BASE_ADDRESS + WRAPPER_REGISTER_OFFSET;
      case SOCSRAM_CORE_ID:
        return SOCSRAM_BASE_ADDRESS + WRAPPER_REGISTER_OFFSET;
      default:
        _err("Invalid core id %d\n", core);
    }
  return 0;
}
