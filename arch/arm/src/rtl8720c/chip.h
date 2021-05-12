
#ifndef __ARCH_ARM_SRC_AMEBA_CHIP_H
#define __ARCH_ARM_SRC_AMEBA_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <sys/types.h>
#endif

#include <arch/chip/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)               (sizeof(x) / sizeof((x)[0]))
#endif

/* If the common ARMv7-M vector handling logic is used, then it expects the following
 * definition in this file that provides the number of supported external interrupts.
 */

#define ARMV8M_PERIPHERAL_INTERRUPTS  (CONFIG_AMEBA_NR_IRQS - 16)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_AMEBA_CHIP_H */
