/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_corepwm.h
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

#ifndef __ARCH_RISCV_SRCMPFS_MPFS_MPFS_COREPWM_H
#define __ARCH_RISCV_SRCMPFS_MPFS_MPFS_COREPWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Check if PWM support for any channel is enabled. */

#ifdef CONFIG_MPFS_HAVE_COREPWM

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/board/board.h>
#include "hardware/mpfs_corepwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_corepwm_init
 *
 * Description:
 *   Initialize a CorePWM block.
 *
 * Input Parameters:
 *   pwmid - A number identifying the pwm block. The number of valid
 *           IDs varies depending on the configuration of the FPGA.
 *
 * Returned Value:
 *   On success, a pointer to the MPFS CorePWM lower half PWM driver is
 *   returned. NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *mpfs_corepwm_init(int pwmid);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_MPFS_HAVE_COREPWM */
#endif /* __ARCH_RISCV_SRCMPFS_MPFS_MPFS_COREPWM_H */
