/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_cpufifo.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_CPUFIFO_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_CPUFIFO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD56_FIF_PUSH_FULL    (CXD56_CPUFIFO_BASE + 0x00)
#define CXD56_FIF_PUSH_WRD0    (CXD56_CPUFIFO_BASE + 0x04)
#define CXD56_FIF_PUSH_WRD1    (CXD56_CPUFIFO_BASE + 0x08)
#define CXD56_FIF_PUSH_CMP     (CXD56_CPUFIFO_BASE + 0x0c)
#define CXD56_FIF_PULL_EMP     (CXD56_CPUFIFO_BASE + 0x10)
#define CXD56_FIF_PULL_WRD0    (CXD56_CPUFIFO_BASE + 0x14)
#define CXD56_FIF_PULL_WRD1    (CXD56_CPUFIFO_BASE + 0x18)
#define CXD56_FIF_PULL_CMP     (CXD56_CPUFIFO_BASE + 0x1c)

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_CPUFIFO_H */
