/****************************************************************************
 * include/nuttx/iterable_sections.h
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

/* Iterable sections: link-time registration of struct instances.
 *
 * A struct instance defined with STRUCT_SECTION_ITERABLE() in any
 * compilation unit is placed in a dedicated input section named
 * "._<struct_type>.static.<varname>".  The board linker script collects
 * these input sections (sorted by name) into a contiguous array delimited
 * by the _<struct_type>_list_start/_<struct_type>_list_end symbols by
 * including <nuttx/linker/common-rom.ld> (const data, inside the .text or
 * .rodata output section) and <nuttx/linker/common-ram.ld> (mutable
 * initialized data, inside the .data output section, so that the startup
 * FLASH-to-RAM copy initializes it).
 *
 * The collection is only available on architectures whose linker scripts
 * are preprocessed with CPP (arm, arm64, risc-v, xtensa, x86_64, tricore)
 * and on boards whose scripts include the common-*.ld fragments.
 */

#ifndef __INCLUDE_NUTTX_ITERABLE_SECTIONS_H
#define __INCLUDE_NUTTX_ITERABLE_SECTIONS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define a struct instance inside an iterable section.  A "const"
 * qualifier may be prepended at the point of use to place the instance in
 * ROM.  Each instance is aligned to the natural alignment of its type so
 * that the collected section can be indexed as a plain C array.  The
 * variable name is part of the input section name so that the linker's
 * SORT_BY_NAME() defines the iteration order (instances may encode
 * ordering in their names).
 */

#define STRUCT_SECTION_ITERABLE(struct_type, varname) \
  struct struct_type varname \
  used_data \
  aligned_data(__alignof__(struct struct_type)) \
  locate_data("._" #struct_type ".static." #varname)

/* Start/end symbols provided by the linker script fragments */

#define STRUCT_SECTION_START(struct_type) _##struct_type##_list_start
#define STRUCT_SECTION_END(struct_type)   _##struct_type##_list_end

#define STRUCT_SECTION_START_EXTERN(struct_type) \
  extern struct struct_type STRUCT_SECTION_START(struct_type)[]
#define STRUCT_SECTION_END_EXTERN(struct_type) \
  extern struct struct_type STRUCT_SECTION_END(struct_type)[]

/* Declare both boundary symbols of an iterable section.  Place it at file
 * scope (followed by a semicolon) in every file that iterates with
 * STRUCT_SECTION_FOREACH.
 */

#define STRUCT_SECTION_DECLARE(struct_type) \
  STRUCT_SECTION_START_EXTERN(struct_type); \
  STRUCT_SECTION_END_EXTERN(struct_type)

/* Iterate over every instance of an iterable section.  "iterator" is a
 * pointer variable (FAR struct struct_type *) declared by the caller, as
 * with list_for_every_entry(); the boundary symbols must be in scope
 * (STRUCT_SECTION_DECLARE).
 */

#define STRUCT_SECTION_FOREACH(struct_type, iterator) \
  for ((iterator) = STRUCT_SECTION_START(struct_type); \
       (iterator) < STRUCT_SECTION_END(struct_type); \
       (iterator)++)

/* Get the i-th element of an iterable section (no bounds checking) */

#define STRUCT_SECTION_GET(struct_type, i, dst) \
  do \
    { \
      STRUCT_SECTION_START_EXTERN(struct_type); \
      *(dst) = &STRUCT_SECTION_START(struct_type)[i]; \
    } \
  while (0)

/* Number of elements in an iterable section */

#define STRUCT_SECTION_COUNT(struct_type, dst) \
  do \
    { \
      STRUCT_SECTION_START_EXTERN(struct_type); \
      STRUCT_SECTION_END_EXTERN(struct_type); \
      *(dst) = STRUCT_SECTION_END(struct_type) - \
               STRUCT_SECTION_START(struct_type); \
    } \
  while (0)

#endif /* __INCLUDE_NUTTX_ITERABLE_SECTIONS_H */
