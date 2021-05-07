/****************************************************************************
 * libs/libc/machine/arm/armv7-m/arch_fabsf.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <math.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if (__ARM_ARCH >= 7) && (__ARM_FP >= 4)

float fabsf(float x)
{
  float result;
  asm volatile ("vabs.f32\t%0, %1" : "=t" (result) : "t" (x));
  return result;
}

#else
#  warning fabsf() not built
#endif
