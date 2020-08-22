/****************************************************************************
 * include/nuttx/drivers/addrenv.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_ADDRENV_H
#define __INCLUDE_NUTTX_DRIVERS_ADDRENV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sys/types.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct simple_addrenv_s
{
  uintptr_t va;
  uintptr_t pa;
  size_t    size;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

void simple_addrenv_initialize(const struct simple_addrenv_s *addrenv);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_DRIVERS_ADDRENV_H */
