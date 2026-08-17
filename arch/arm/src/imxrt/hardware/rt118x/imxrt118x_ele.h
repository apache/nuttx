/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_ele.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_ELE_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_ELE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RT EdgeLock Enclave messaging unit */

#define IMXRT_S3MUA_TSR                 (IMXRT_MU_RT_S3MUA_BASE + 0x124u)
#define IMXRT_S3MUA_RSR                 (IMXRT_MU_RT_S3MUA_BASE + 0x12cu)
#define IMXRT_S3MUA_TR(n)               (IMXRT_MU_RT_S3MUA_BASE + \
                                         0x200u + ((n) << 2))
#define IMXRT_S3MUA_RR(n)               (IMXRT_MU_RT_S3MUA_BASE + \
                                         0x280u + ((n) << 2))

#define IMXRT_ELE_RESPONSE_SUCCESS       0xd6u
#define IMXRT_ELE_GET_FW_STATUS          0x17c50106u
#define IMXRT_ELE_GET_FW_STATUS_RESPONSE 0xe1c50306u
#define IMXRT_ELE_RELEASE_RDC            0x17c40206u
#define IMXRT_ELE_RELEASE_RDC_RESPONSE   0xe1c40206u
#define IMXRT_ELE_CLOCK_CHANGE_START     0x17100106u
#define IMXRT_ELE_CLOCK_START_RESPONSE   0xe1100206u
#define IMXRT_ELE_CLOCK_CHANGE_FINISH    0x17110206u
#define IMXRT_ELE_CLOCK_FINISH_RESPONSE  0xe1110206u

#define IMXRT_ELE_TRDC_AON_ID            0x74u
#define IMXRT_ELE_TRDC_WAKEUP_ID         0x78u
#define IMXRT_ELE_TRDC_MEGA_ID           0x82u
#define IMXRT_ELE_CORE_CM33_ID           0x01u

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_ELE_H */
