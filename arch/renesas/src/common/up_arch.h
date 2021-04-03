/****************************************************************************
 * arch/renesas/src/common/up_arch.h
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

#ifndef ___ARCH_RENESAS_SRC_COMMON_UP_ARCH_H
#define ___ARCH_RENESAS_SRC_COMMON_UP_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include <arch/board/board.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

#endif

#endif /* ___ARCH_RENESAS_SRC_COMMON_UP_ARCH_H */
