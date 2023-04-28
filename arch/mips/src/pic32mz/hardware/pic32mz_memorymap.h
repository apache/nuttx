/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mz_memorymap.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_MEMORYMAP_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_PIC32MZEC)
#  include "hardware/pic32mzec_memorymap.h"
#elif defined(CONFIG_ARCH_CHIP_PIC32MZEF)
#  include "hardware/pic32mzef_memorymap.h"
#else
#  error "Unknown PIC32MZ family"
#endif

#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_MEMORYMAP_H */
