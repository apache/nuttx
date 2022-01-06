/****************************************************************************
 * arch/xtensa/src/common/xtensa_attr.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_XTENSA_ATTR_H
#define __ARCH_XTENSA_SRC_COMMON_XTENSA_ATTR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ROMFN_ATTR

/* Normally, the linker script will put all code and rodata in flash,
 * and all variables in shared RAM. These macros can be used to redirect
 * particular functions/variables to other memory regions.
 */

/* Forces code into IRAM instead of flash */

#define IRAM_ATTR locate_data(".iram1")

/* Forces data into DRAM instead of flash */

#define DRAM_ATTR locate_data(".dram1")

/* Forces code into RTC fast memory */

#define RTC_IRAM_ATTR locate_data(".rtc.text")

/* Forces data into RTC slow memory
 * Any variable marked with this attribute will keep its value
 * during a deep sleep / wake cycle.
 */

#define RTC_DATA_ATTR locate_data(".rtc.data")

/* Forces read-only data into RTC slow memory
 * Makes constant data available to RTC wake stubs.
 */

#define RTC_RODATA_ATTR locate_data(".rtc.rodata")

/* Allow bss variables into external memory. */

#ifdef CONFIG_XTENSA_EXTMEM_BSS
#  define EXT_RAM_ATTR locate_data(".extmem.bss")
#else
#  define EXT_RAM_ATTR
#endif

#endif /* __ARCH_XTENSA_SRC_COMMON_XTENSA_ATTR_H */
