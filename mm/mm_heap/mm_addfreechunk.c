/****************************************************************************
 * mm/mm_heap/mm_addfreechunk.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Emit the out-of-line definition of mm_addfreechunk().  Must precede
 * the first inclusion of mm_heap/mm.h, or its include guard suppresses
 * the definition.
 */
#define MM_ADDFREECHUNK_IMPL
#include "mm_heap/mm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* mm_addfreechunk() is defined in mm_heap/mm.h. */
