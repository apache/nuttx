/****************************************************************************
 * include/nuttx/nuttx.h
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

#ifndef __INCLUDE_NUTTX_NUTTX_H
#define __INCLUDE_NUTTX_NUTTX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Name: container_of
 *
 * Description:
 *   Cast a member of a structure out to get the address of the containing
 *   structure
 *
 * Arguments:
 *   ptr    - The pointer to the member.
 *   type   - The type of the container struct this is embedded in.
 *   member - The name of the member within the struct.
 */

#define container_of(ptr, type, member) \
  ((type *)((uintptr_t)(ptr) - offsetof(type, member)))

#endif /* __INCLUDE_NUTTX_NUTTX_H */
