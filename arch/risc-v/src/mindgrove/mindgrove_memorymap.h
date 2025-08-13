#ifndef __ARCH_RISCV_SRC_MINDGROVE_MEMORYMAP_H
#define __ARCH_RISCV_SRC_MINDGROVE_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/mindgrove_uart.h"
// #include "secure_iot_reg.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Idle thread stack starts from _ebss */

#ifndef __ASSEMBLY__
#define MINDGROVE_IDLESTACK_BASE  (uintptr_t)&_ebss
#else
#define MINDGROVE_IDLESTACK_BASE  _ebss
#endif

#define MINDGROVE_IDLESTACK_TOP  (MINDGROVE_IDLESTACK_BASE + CONFIG_IDLETHREAD_STACKSIZE)
// #define MINDGROVE_UART1_BASE UART_REG(0)


#endif /* __ARCH_RISCV_SRC_MINDGROVE_MEMORYMAP_H */
