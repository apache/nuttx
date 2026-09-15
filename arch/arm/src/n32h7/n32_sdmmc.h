/****************************************************************************
 * arch/arm/src/n32h7/n32_sdmmc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * N32H7 SDMMC Driver Public Interface
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_N32H7_N32H7_SDMMC_H
#define __ARCH_ARM_SRC_N32H7_N32H7_SDMMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdbool.h>
#include <nuttx/sdio.h>

#include "chip.h"
#include "hardware/n32h7_sdmmc.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - 0 for SDMMC1, 1 for SDMMC2 (depending on config).
 *
 * Returned Values:
 *   A reference to an SDIO interface structure. NULL is returned on failure.
 *
 ****************************************************************************/

struct sdio_dev_s *sdio_initialize(int slotno);

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic to signal card insertion/removal.
 *
 ****************************************************************************/

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot);

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report write protect status.
 *
 ****************************************************************************/

void sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect);

#if defined(CONFIG_SDMMC1_SDIO_MODE) || defined(CONFIG_SDMMC2_SDIO_MODE)
void sdio_set_sdio_card_isr(struct sdio_dev_s *dev,
                            int (*func)(void *), void *arg);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_N32H7_N32_SDMMC_H */
