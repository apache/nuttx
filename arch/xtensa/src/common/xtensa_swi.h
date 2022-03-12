/****************************************************************************
 * arch/xtensa/src/common/xtensa_swi.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_XTENSA_SWI_H
#define __ARCH_XTENSA_SRC_COMMON_XTENSA_SWI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/xtensa/core.h>
#include <arch/xtensa/xtensa_corebits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select software interrupt number for context-switch.
 * The SW interrupt level must be greater than XCHAL_SYSCALL_LEVEL
 * and less than XCHAL_EXCM_LEVEL.
 * So that we can generate an interrupt when up_irq_save is called.
 * and not generate interrupt when up_irq_disable is called.
 * Return an error if no suitable software interrupt was found.
 */

#ifndef XTENSA_SWINT
#  ifdef XCHAL_SOFTWARE2_INTERRUPT
#    if XCHAL_INT_LEVEL(XCHAL_SOFTWARE2_INTERRUPT) > XCHAL_SYSCALL_LEVEL && \
        XCHAL_INT_LEVEL(XCHAL_SOFTWARE2_INTERRUPT) <= XCHAL_EXCM_LEVEL
#      undef  XTENSA_SWINT
#      define XTENSA_SWINT XCHAL_SOFTWARE2_INTERRUPT
#    endif
#  endif
#  ifdef XCHAL_SOFTWARE1_INTERRUPT
#    if XCHAL_INT_LEVEL(XCHAL_SOFTWARE1_INTERRUPT) > XCHAL_SYSCALL_LEVEL && \
        XCHAL_INT_LEVEL(XCHAL_SOFTWARE1_INTERRUPT) <= XCHAL_EXCM_LEVEL
#      undef  XTENSA_SWINT
#      define XTENSA_SWINT XCHAL_SOFTWARE1_INTERRUPT
#    endif
#  endif
#  ifdef XCHAL_SOFTWARE0_INTERRUPT
#    if XCHAL_INT_LEVEL(XCHAL_SOFTWARE0_INTERRUPT) > XCHAL_SYSCALL_LEVEL && \
        XCHAL_INT_LEVEL(XCHAL_SOFTWARE0_INTERRUPT) <= XCHAL_EXCM_LEVEL
#      undef  XTENSA_SWINT
#      define XTENSA_SWINT XCHAL_SOFTWARE0_INTERRUPT
#    endif
#  endif
#endif
#ifndef XTENSA_SWINT
#  error "There is no suitable sw interrupt in this Xtensa configuration."
#endif

#define XCHAL_SWINT_CALL        (1 << XTENSA_SWINT)

#endif /* __ARCH_XTENSA_SRC_COMMON_XTENSA_SWI_H */
