#ifndef __ARCH_RISCV_SRC_MINDGROVE_LOWPUTC_H
#define __ARCH_RISCV_SRC_MINDGROVE_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include"chip.h"
#include "secure_iot_reg.h"

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
 * Name: secure_iot_lowsetup
 ****************************************************************************/

EXTERN void mindgrove_lowsetup(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_MINDGROVE_LOWPUTC_H */
