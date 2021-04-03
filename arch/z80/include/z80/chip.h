/************************************************************************************
 * arch/z80/include/z80/chip.h
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
 ************************************************************************************/

#ifndef __ARCH_Z80_INCLUDE_Z80_CHIP_H
#define __ARCH_Z80_INCLUDE_Z80_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Bits in the Z80 FLAGS register ***************************************************/

#define Z80_C_FLAG      0x01       /* Bit 0: Carry flag */
#define Z80_N_FLAG      0x02       /* Bit 1: Add/Subtract flag  */
#define Z80_PV_FLAG     0x04       /* Bit 2: Parity/Overflow flag */
#define Z80_H_FLAG      0x10       /* Bit 4: Half carry flag */
#define Z80_Z_FLAG      0x40       /* Bit 5: Zero flag */
#define Z80_S_FLAG      0x80       /* Bit 7: Sign flag */

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
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
#endif

#endif /* __ARCH_Z80_INCLUDE_Z80_CHIP_H */
