/****************************************************************************
 * arch/arm/include/cxd56xx/timer.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_TIMER_H
#define __ARCH_ARM_INCLUDE_CXD56XX_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/timers/timer.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Set callback handler
 *
 * param A pointer to struct timer_sethandler_s
 * return ioctl return value provides success/failure indication
 */

#define TCIOC_SETHANDLER _TCIOC(0x0020)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the type of the argument passed to the TCIOC_SETHANDLER ioctl */

struct timer_sethandler_s
{
  void   *arg;            /* An argument */
  tccb_t handler;         /* The timer interrupt handler */
};

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_TIMER_H */
