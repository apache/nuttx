/****************************************************************************
 * mm/ubsan/ubsan.h
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

#ifndef __MM_UBSAN_UBSAN_H
#define __MM_UBSAN_UBSAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum type_kind_e
{
  TYPE_KIND_INT     = 0,
  TYPE_KIND_FLOAT   = 1,
  TYPE_KIND_UNKNOWN = 0xffff
};

struct type_descriptor
{
  uint16_t type_kind;
  uint16_t type_info;
  char type_name[1];
};

struct source_location
{
  FAR const char *file_name;
  union
    {
      unsigned long reported;
      struct
        {
          uint32_t line;
          uint32_t column;
        };
    };
};

struct overflow_data
{
  struct source_location location;
  FAR struct type_descriptor *type;
};

struct type_mismatch_data
{
  struct source_location location;
  FAR struct type_descriptor *type;
  unsigned long alignment;
  unsigned char type_check_kind;
};

struct type_mismatch_data_v1
{
  struct source_location location;
  FAR struct type_descriptor *type;
  unsigned char log_alignment;
  unsigned char type_check_kind;
};

struct type_mismatch_data_common
{
  FAR struct source_location *location;
  FAR struct type_descriptor *type;
  unsigned long alignment;
  unsigned char type_check_kind;
};

struct nonnull_arg_data
{
  struct source_location location;
  struct source_location attr_location;
  int arg_index;
};

struct out_of_bounds_data
{
  struct source_location location;
  FAR struct type_descriptor *array_type;
  FAR struct type_descriptor *index_type;
};

struct shift_out_of_bounds_data
{
  struct source_location location;
  FAR struct type_descriptor *lhs_type;
  FAR struct type_descriptor *rhs_type;
};

struct invalid_value_data
{
  struct source_location location;
  FAR struct type_descriptor *type;
};

struct alignment_assumption_data
{
  struct source_location location;
  struct source_location assumption_location;
  FAR struct type_descriptor *type;
};

#endif /* __MM_UBSAN_UBSAN_H */
