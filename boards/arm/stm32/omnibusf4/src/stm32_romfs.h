/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_romfs.h
 *
 *   Copyright (C) 2017 Tomasz Wozniak. All rights reserved.
 *   Author: Tomasz Wozniak <t.wozniak@samsung.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32_OMNIBUSF4_SRC_STM32_ROMFS_H
#define __BOARDS_ARM_STM32_OMNIBUSF4_SRC_STM32_ROMFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_STM32_ROMFS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ROMFS_SECTOR_SIZE 64

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_romfs_initialize
 *
 * Description:
 *   Registers built-in ROMFS image as block device and mounts it.
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno value on error.
 *
 * Assumptions/Limitations:
 *   Memory addresses [romfs_data_begin .. romfs_data_end) should contain
 *   ROMFS volume data, as included in the assembly snippet above (l. 84).
 *
 ****************************************************************************/

int stm32_romfs_initialize(void);

#endif /* CONFIG_STM32_ROMFS */

#endif /* __BOARDS_ARM_STM32_OMNIBUSF4_SRC_STM32_ROMFS_H */
