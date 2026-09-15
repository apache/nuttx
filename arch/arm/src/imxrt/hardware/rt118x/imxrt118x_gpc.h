/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_gpc.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_GPC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_GPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CPU control contains two authentication banks, 0x800 bytes apart. */

#define IMXRT_GPC_CPU_BANK_STRIDE                  0x800u
#define IMXRT_GPC_CPU_REG(bank, offset) \
  (IMXRT_GPC_CPU_CTRL_BASE + (bank) * IMXRT_GPC_CPU_BANK_STRIDE + (offset))

#define IMXRT_GPC_CM_AUTHEN_CTRL(bank) \
  IMXRT_GPC_CPU_REG(bank, 0x004u)
#define IMXRT_GPC_CM_MISC(bank) \
  IMXRT_GPC_CPU_REG(bank, 0x00cu)
#define IMXRT_GPC_CM_MODE_CTRL(bank) \
  IMXRT_GPC_CPU_REG(bank, 0x010u)
#define IMXRT_GPC_CM_MODE_STAT(bank) \
  IMXRT_GPC_CPU_REG(bank, 0x014u)
#define IMXRT_GPC_CM_IRQ_WAKEUP_MASK(bank, n) \
  IMXRT_GPC_CPU_REG(bank, 0x100u + ((n) << 2))
#define IMXRT_GPC_CM_NON_IRQ_WAKEUP_MASK(bank) \
  IMXRT_GPC_CPU_REG(bank, 0x140u)
#define IMXRT_GPC_CM_SLEEP_SSAR_CTRL(bank) \
  IMXRT_GPC_CPU_REG(bank, 0x200u)
#define IMXRT_GPC_CM_SLEEP_LPCG_CTRL(bank) \
  IMXRT_GPC_CPU_REG(bank, 0x208u)
#define IMXRT_GPC_CM_SLEEP_PLL_CTRL(bank) \
  IMXRT_GPC_CPU_REG(bank, 0x210u)
#define IMXRT_GPC_CM_WAKEUP_PLL_CTRL(bank) \
  IMXRT_GPC_CPU_REG(bank, 0x2a8u)
#define IMXRT_GPC_CM_WAKEUP_LPCG_CTRL(bank) \
  IMXRT_GPC_CPU_REG(bank, 0x2b0u)
#define IMXRT_GPC_CM_SYS_SLEEP_CTRL(bank) \
  IMXRT_GPC_CPU_REG(bank, 0x380u)

#define GPC_AUTHEN_LOCK_CFG                         (1u << 7)
#define GPC_AUTHEN_USER                             (1u << 8)
#define GPC_AUTHEN_NONSECURE                        (1u << 9)
#define GPC_AUTHEN_LOCK_SETTING                     (1u << 11)
#define GPC_AUTHEN_LOCK_LIST                        (1u << 15)
#define GPC_AUTHEN_WHITE_LIST_SHIFT                 16
#define GPC_AUTHEN_WHITE_LIST_MASK                  (0xffffu << 16)

#define GPC_CM_MODE_TARGET_SHIFT                    0
#define GPC_CM_MODE_TARGET_MASK                     3u
#define GPC_CM_MODE_RUN                             0u
#define GPC_CM_MODE_WAIT                            1u
#define GPC_CM_MODE_STOP                            2u
#define GPC_CM_MODE_SUSPEND                         3u
#define GPC_CM_MODE_WFE_ENABLE                      (1u << 4)

/* Global GPC */

#define IMXRT_GPC_GLOBAL_AUTHEN_CTRL \
  (IMXRT_GPC_GLOBAL_BASE + 0x004u)
#define IMXRT_GPC_GLOBAL_CPU0_DOMAIN \
  (IMXRT_GPC_GLOBAL_BASE + 0x010u)
#define IMXRT_GPC_GLOBAL_CPU1_DOMAIN \
  (IMXRT_GPC_GLOBAL_BASE + 0x014u)
#define IMXRT_GPC_GLOBAL_MASTER \
  (IMXRT_GPC_GLOBAL_BASE + 0x024u)
#define IMXRT_GPC_GLOBAL_ROSC_CTRL \
  (IMXRT_GPC_GLOBAL_BASE + 0x200u)

#define GPC_GLOBAL_CPU0_MASTER                      (1u << 0)
#define GPC_GLOBAL_CPU1_MASTER                      (1u << 1)
#define GPC_GLOBAL_ROSC_OFF_ENABLE                  (1u << 0)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_GPC_H */
