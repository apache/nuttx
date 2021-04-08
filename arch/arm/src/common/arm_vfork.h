/****************************************************************************
 * arch/arm/src/common/arm_vfork.h
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

#ifndef __ARCH_ARM_SRC_COMMON_ARM_VFORK_H
#define __ARCH_ARM_SRC_COMMON_ARM_VFORK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VFORK_R4_OFFSET  (0*4)   /* Volatile register r4 */
#define VFORK_R5_OFFSET  (1*4)   /* Volatile register r5 */
#define VFORK_R6_OFFSET  (2*4)   /* Volatile register r6 */
#define VFORK_R7_OFFSET  (3*4)   /* Volatile register r7 */
#define VFORK_R8_OFFSET  (4*4)   /* Volatile register r8 */
#define VFORK_R9_OFFSET  (5*4)   /* Volatile register r9 */
#define VFORK_R10_OFFSET (6*4)   /* Volatile register r10 */

#define VFORK_FP_OFFSET  (7*4)   /* Frame pointer */
#define VFORK_SP_OFFSET  (8*4)   /* Stack pointer*/
#define VFORK_LR_OFFSET  (9*4)   /* Return address*/

#define VFORK_SIZEOF     (10*4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
struct vfork_s
{
  /* CPU registers */

  uint32_t r4;   /* Volatile register r4 */
  uint32_t r5;   /* Volatile register r5 */
  uint32_t r6;   /* Volatile register r6 */
  uint32_t r7;   /* Volatile register r7 */
  uint32_t r8;   /* Volatile register r8 */
  uint32_t r9;   /* Volatile register r9 */
  uint32_t r10;  /* Volatile register r10 */

  uint32_t fp;   /* Frame pointer */
  uint32_t sp;   /* Stack pointer */
  uint32_t lr;   /* Return address */

  /* Floating point registers (not yet) */
};
#endif

#endif /* __ARCH_ARM_SRC_COMMON_ARM_VFORK_H */
