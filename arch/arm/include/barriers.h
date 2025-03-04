/****************************************************************************
 * arch/arm/include/barriers.h
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

#ifndef __ARCH_ARM_INCLUDE_BARRIERS_H
#define __ARCH_ARM_INCLUDE_BARRIERS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#if defined(CONFIG_ARCH_ARMV7A)
#  include <arch/armv7-a/barriers.h>
#elif defined(CONFIG_ARCH_ARMV7R)
#  include <arch/armv7-r/barriers.h>
#elif defined(CONFIG_ARCH_ARMV8R)
#  include <arch/armv8-r/barriers.h>
#elif defined(CONFIG_ARCH_ARMV7M)
#  include <arch/armv7-m/barriers.h>
#elif defined(CONFIG_ARCH_ARMV8M)
#  include <arch/armv8-m/barriers.h>
#elif defined(CONFIG_ARCH_ARMV6M)
#  include <arch/armv6-m/barriers.h>
#else
#  include <arch/arm/barriers.h>
#endif

#define UP_MB() \
  do            \
    {           \
      UP_DSB(); \
      UP_ISB(); \
    }           \
  while (0)

#endif /* __ARCH_ARM_INCLUDE_BARRIERS_H */
