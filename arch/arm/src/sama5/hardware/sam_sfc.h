/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_sfc.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SFC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SFC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(ATSAMA5D2) || defined(ATSAMA5D4)

/* SFC Register Offsets *****************************************************/

#define SAM_SFC_KR_OFFSET           (0x0000) /* Key register                */
#define SAM_SFC_MR_OFFSET           (0x0004) /* Mode register               */
                                             /* 0x008-0x00c reserved        */
#define SAM_SFC_IER_OFFSET          (0x0010) /* Interrupt enable register   */
#define SAM_SFC_IDR_OFFSET          (0x0014) /* Interrupr disable register  */
#define SAM_SFC_IMR_OFFSET          (0x0018) /* Interrupt mask register     */
#define SAM_SFC_SR_OFFSET           (0x001c) /* Status register             */
#define SAM_SFC_DR_OFFSET(n)        ((0x0020)+((n) << (2)))
#define SAM_SFC_SR_PGMC_SHIFT       (0)
#define SAM_SFC_SR_PGMC             ((1) << SAM_SFC_SR_PGMC_SHIFT)
                                             /* Programming completed       */
#define SAM_SFC_SR_PGMF_SHIFT       (1)
#define SAM_SFC_SR_PGMF             ((1) << SAM_SFC_SR_PGMF_SHIFT)
                                             /* Programming failed          */

#define SAM_SFC_KR                  (SAM_SFC_VBASE + SAM_SFC_KR_OFFSET)
#define SAM_SFC_MR                  (SAM_SFC_VBASE + SAM_SFC_MR_OFFSET)
#define SAM_SFC_MR_MASK             (1)
#define SAM_SFC_IER                 (SAM_SFC_VBASE + SAM_SFC_IER_OFFSET)
#define SAM_SFC_IDR                 (SAM_SFC_VBASE + SAM_SFC_IDR_OFFSET)
#define SAM_SFC_IMR                 (SAM_SFC_VBASE + SAM_SFC_IMR_OFFSET)
#define SAM_SFC_SR                  (SAM_SFC_VBASE + SAM_SFC_SR_OFFSET)

#define SAM_SFC_DR(n)               (SAM_SFC_VBASE + SAM_SFC_DR_OFFSET(n))

#define SAM_SFC_KEYCODE             (0x00fb) /* Keycode to allow write      */

#endif /* if defined(ATSAMA5D2) || defined(ATSAMA5D4) */
#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SFC_H */
