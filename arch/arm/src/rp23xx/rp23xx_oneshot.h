/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_oneshot.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_ONESHOT_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_ONESHOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/oneshot.h>

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
 * Name: rp23xx_oneshot_initialize
 *
 * Description:
 *   Initialise the RP2350 system timer TIMER0 as a oneshot lower-half and
 *   return it, ready to be handed to up_alarm_set_lowerhalf() to drive the
 *   tickless scheduler.  TIMER0 is a free-running 64-bit microsecond counter
 *   whose ALARM0 provides the compare event.
 *
 * Returned Value:
 *   The oneshot lower-half instance on success; NULL never happens (the
 *   instance is statically allocated).
 *
 ****************************************************************************/

FAR struct oneshot_lowerhalf_s *rp23xx_oneshot_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_ONESHOT_H */
