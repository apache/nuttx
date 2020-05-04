/****************************************************************************
 * arch/arm/src/common/arm_arch.h
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

#ifndef ___ARCH_ARM_SRC_COMMON_ARM_ARCH_H
#define ___ARCH_ARM_SRC_COMMON_ARM_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

# define getreg8(a)           (*(volatile uint8_t *)(a))
# define putreg8(v,a)         (*(volatile uint8_t *)(a) = (v))
# define getreg16(a)          (*(volatile uint16_t *)(a))
# define putreg16(v,a)        (*(volatile uint16_t *)(a) = (v))
# define getreg32(a)          (*(volatile uint32_t *)(a))
# define putreg32(v,a)        (*(volatile uint32_t *)(a) = (v))

/* Non-atomic, but more effective modification of registers */

# define modreg8(v,m,a)       putreg8((getreg8(a) & ~(m)) | ((v) & (m)), a)
# define modreg16(v,m,a)      putreg16((getreg16(a) & ~(m)) | ((v) & (m)), a)
# define modreg32(v,m,a)      putreg32((getreg32(a) & ~(m)) | ((v) & (m)), a)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Atomic modification of registers */

void modifyreg8(unsigned int addr, uint8_t clearbits, uint8_t setbits);
void modifyreg16(unsigned int addr, uint16_t clearbits, uint16_t setbits);
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* ___ARCH_ARM_SRC_COMMON_ARM_ARCH_H */
