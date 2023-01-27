/****************************************************************************
 * arch/z80/include/z180/arch.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/arch.h (via arch/arch.h)
 */

#ifndef __ARCH_Z80_INCLUDE_Z180_ARCH_H
#define __ARCH_Z80_INCLUDE_Z180_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The z180 address environment is represented in hardware as the 8-bit
 * Common Base Register (CBR).  CBR specifies the base address (on 4KB
 * boundaries) used to generate a 20-bit physical address for Common Area 1
 * accesses.  CBR is the upper 8-bits of the 20-bit address; the lower
 * 14-bits of the base address are implicitly zero (hence the 4KB boundary
 * alignment).
 */

#ifdef CONFIG_ARCH_ADDRENV

/* At the task-level, the z180 address environment is represented as struct
 * z180_cbr_s which is defined in irq.h.
 */

struct z180_cbr_s;
typedef FAR struct z180_cbr_s *arch_addrenv_t;
#endif

/****************************************************************************
 * Public Data
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

#endif /* __ARCH_Z80_INCLUDE_Z180_ARCH_H */
