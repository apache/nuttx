/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_ddp.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_DDP_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_DDP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define PIC32MX_DDP_CON_OFFSET 0x0000 /* Control Register for the Diagnostic Module */

/* Register Addresses *******************************************************/

#define PIC32MX_DDP_CON        (PIC32MX_DDP_K1BASE+PIC32MX_DDP_CON_OFFSET)

/* See also the ICESEL, DEBUG, and DEBUG0 in the DEVCFG0 register */

/* Register Bit-Field Definitions *******************************************/

/* Control Register for the Diagnostic Module */

#define DDP_CON_TROEN          (1 << 2) /* Bit 2: Trace output enable */
#define DDP_CON_JTAGEN         (1 << 3) /* Bit 3: JTAG port enable */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_DDP_H */
