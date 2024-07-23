/****************************************************************************
 * arch/risc-v/src/hpm6000/hardware/hpm_sysctl.h
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

#ifndef __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_SYSCTL_H
#define __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_SYSCTL_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#define CONFIG_ARCH_FAMILY_HPM6300

#if defined(CONFIG_ARCH_FAMILY_HPM6300)
#  include "hpm6300/hpm6300_sysctl.h"
#else
#  error Unrecognized HPM architecture
#endif

#endif /* __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_SYSCTL_H */