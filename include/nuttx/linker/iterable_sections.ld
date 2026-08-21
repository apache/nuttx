/****************************************************************************
 * include/nuttx/linker/iterable_sections.ld
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

/* CPP macro emitting the linker statements that collect one iterable
 * section (see include/nuttx/iterable_sections.h).  This file is meant to
 * be included from linker scripts that are preprocessed with CPP (the
 * ARCHSCRIPT .tmp rule).
 *
 * The macro must be expanded INSIDE an output section (e.g. .text or
 * .data).  KEEP() protects the entries from --gc-sections and
 * SORT_BY_NAME() defines the iteration order.
 */

#ifndef __INCLUDE_NUTTX_LINKER_ITERABLE_SECTIONS_LD
#define __INCLUDE_NUTTX_LINKER_ITERABLE_SECTIONS_LD

#define ITERABLE_SECTION(name) \
  . = ALIGN(4); \
  _##name##_list_start = .; \
  KEEP(*(SORT_BY_NAME(._##name.static.*))); \
  _##name##_list_end = .; \
  . = ALIGN(4);

#endif /* __INCLUDE_NUTTX_LINKER_ITERABLE_SECTIONS_LD */
