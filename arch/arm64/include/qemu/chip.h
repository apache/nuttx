/****************************************************************************
 * arch/arm64/include/qemu/chip.h
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

#ifndef __ARCH_ARM64_INCLUDE_QEMU_CHIP_H
#define __ARCH_ARM64_INCLUDE_QEMU_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Number of bytes in @p x kibibytes/mebibytes/gibibytes */

#define KB(x)           ((x) << 10)
#define MB(x)           (KB(x) << 10)
#define GB(x)           (MB(UINT64_C(x)) << 10)

#if defined(CONFIG_ARCH_CHIP_QEMU)

#define CONFIG_GICD_BASE          0x8000000
#define CONFIG_GICR_BASE          0x80a0000

#define CONFIG_RAMBANK1_ADDR      0x40000000
#define CONFIG_RAMBANK1_SIZE      MB(128)

#define CONFIG_DEVICEIO_BASEADDR  0x7000000
#define CONFIG_DEVICEIO_SIZE      MB(512)

#define CONFIG_LOAD_BASE          0x40280000

#endif

#endif /* __ARCH_ARM64_INCLUDE_QEMU_CHIP_H */
