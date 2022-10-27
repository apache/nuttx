/****************************************************************************
 * arch/sparc/src/s698pm/s698pm-memorymap.h
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

#ifndef __ARCH_SPARC_SRC_S698PM_S698PM_MEMORYMAP_H
#define __ARCH_SPARC_SRC_S698PM_S698PM_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Addresses **************************************************/

/* UART 1-4 Register Base Addresses */

#  define S698PM_UART1_BASE      (0x80000100)
#  define S698PM_UART2_BASE      (0x80000900)
#  define S698PM_UART3_BASE      (0x80100100)
#  define S698PM_UART4_BASE      (0x80100200)

/* GPIO metux Register Base Addresses */

#  define S698PM_GPREG_BASE      (0x80100500)

#define S698PM_DSU_BASE          (0x90000000)
#define S698PM_DSU_CPU0_BASE     (0x90000000)
#define S698PM_DSU_CPU1_BASE     (0x91000000)
#define S698PM_DSU_CPU2_BASE     (0x92000000)
#define S698PM_DSU_CPU3_BASE     (0x93000000)

#define S698PM_DSU_PC_OFFSET     0x400010  /* PC register */
#define S698PM_DSU_NPC_OFFSET    0x400014  /* nPC register */

#endif /* __ARCH_SPARC_SRC_S698PM_S698PM_MEMORYMAP_H */

