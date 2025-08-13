

#ifndef __ARCH_RISCV_SRC_MINDGROVE_CLOCKCONFIG_H
#define __ARCH_RISCV_SRC_MINDGROVE_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "mindgrove_memorymap.h"


#define MSIP       0x02000000
#define MINDGROVE_CLINT_MTIME      0x0200BFF8
#define MINDGROVE_CLINT_MTIMECMP   0x02004000
/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN uint64_t mindgrove_get_hfclk(void);
EXTERN void mindgrove_clockconfig(void);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_MINDGROVE_CLOCKCONFIG_H */
