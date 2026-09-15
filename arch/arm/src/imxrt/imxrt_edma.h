/****************************************************************************
 * arch/arm/src/imxrt/imxrt_edma.h
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_EDMA_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_EDMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_IMXRT_EDMA_VER2
#  include "imxrt_edma_ver2.h"
#else
#  include "imxrt_edma_ver1.h"
#endif

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_EDMA_H */
