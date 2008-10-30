/************************************************************************************
 * arch/arm/src/str71x/str71x_emi.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_EMI_H
#define __ARCH_ARM_SRC_STR71X_STR71X_EMI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include "str71x_map.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* External Memory Interfac (EMI) registers *****************************************/

#define STR71X_EMI_BCON0            (STR71X_EMI_BASE + 0x0000) /* 16-bits wide */
#define STR71X_EMI_BCON1            (STR71X_EMI_BASE + 0x0004) /* 16-bits wide */
#define STR71X_EMI_BCON2            (STR71X_EMI_BASE + 0x0008) /* 16-bits wide */
#define STR71X_EMI_BCON3            (STR71X_EMI_BASE + 0x000c) /* 16-bits wide */

/* Register bit settings ***********************************************************/

/* EMI enable */

#define STR71X_EMI_ENABLE           (0x8000)

/* Banks */

#define STR71X_EMI_BANK0            (0x00)
#define STR71X_EMI_BANK1            (0x01)
#define STR71X_EMI_BANK2            (0x02)
#define STR71X_EMI_BANK3            (0x03)

/* EMI data bus length */

#define STR71X_EMI_SIZE8            (0x0000)
#define STR71X_EMI_SIZE16           (0x0001)

/* Number of wait states */

#define STR71X_EMI_0WaitState       (0x00)
#define STR71X_EMI_1WaitState       (0x01)
#define STR71X_EMI_2WaitStates      (0x02)
#define STR71X_EMI_3WaitStates      (0x03)
#define STR71X_EMI_4WaitStates      (0x04)
#define STR71X_EMI_5WaitStates      (0x05)
#define STR71X_EMI_6WaitStates      (0x06)
#define STR71X_EMI_7WaitStates      (0x07)
#define STR71X_EMI_8WaitStates      (0x08)
#define STR71X_EMI_9WaitStates      (0x09)
#define STR71X_EMI_10WaitStates     (0x0a)
#define STR71X_EMI_11WaitStates     (0x0b)
#define STR71X_EMI_12WaitStates     (0x0c)
#define STR71X_EMI_13WaitStates     (0x0d)
#define STR71X_EMI_14WaitStates     (0x0e)
#define STR71X_EMI_15WaitStates     (0x0f)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_EMI_H */
