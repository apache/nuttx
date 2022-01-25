/****************************************************************************
 * arch/ceva/include/xc5/spinlock.h
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

#ifndef __ARCH_CEVA_INCLUDE_XC5_SPINLOCK_H
#define __ARCH_CEVA_INCLUDE_XC5_SPINLOCK_H

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Memory barrier doesn't need on tl4 */

static inline void up_dsb(void)
{
}

static inline void up_dmb(void)
{
}

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_CEVA_INCLUDE_XC5_SPINLOCK_H */
