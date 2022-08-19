/****************************************************************************
 * mm/ubsan/ubsan.c
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

#include <nuttx/compiler.h>

#include <debug.h>
#include <stdio.h>

#include "ubsan.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IS_ALIGNED(x, a) (((x) & ((a) - 1)) == 0)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR const char *const g_type_check_kinds[] =
{
  "load of",
  "store to",
  "reference binding to",
  "member access within",
  "member call on",
  "constructor call on",
  "downcast of",
  "downcast of"
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ubsan_prologue(FAR struct source_location *loc,
                           FAR const char *reason)
{
  _alert("========================================"
         "========================================\n");
  _alert("UBSAN: %s in %s:%" PRIu32 ":%" PRIu32"\n",
         reason, loc->file_name, loc->line, loc->column);
}

static void ubsan_epilogue(void)
{
  _alert("========================================"
         "========================================\n");
}

static void ubsan_prologue_epilogue(FAR struct source_location *loc,
                                    FAR const char *reason)
{
  ubsan_prologue(loc, reason);
  ubsan_epilogue();
}

static void handle_null_ptr_deref(FAR struct type_mismatch_data_common *data)
{
  ubsan_prologue(data->location, "null-pointer-dereference");

  _alert("%s null pointer of type %s\n",
         g_type_check_kinds[data->type_check_kind], data->type->type_name);

  ubsan_epilogue();
}

static void handle_misaligned_access(
    FAR struct type_mismatch_data_common *data, uintptr_t ptr)
{
  ubsan_prologue(data->location, "misaligned-access");

  _alert("%s misaligned address %p for type %s\n",
         g_type_check_kinds[data->type_check_kind], (FAR void *)ptr,
         data->type->type_name);
  _alert("which requires %ld byte alignment\n", data->alignment);

  ubsan_epilogue();
}

static void handle_object_size_mismatch(
    FAR struct type_mismatch_data_common *data, uintptr_t ptr)
{
  ubsan_prologue(data->location, "object-size-mismatch");

  _alert("%s address %p with insufficient space\n",
         g_type_check_kinds[data->type_check_kind], (FAR void *)ptr);
  _alert("for an object of type %s\n", data->type->type_name);

  ubsan_epilogue();
}

static void ubsan_type_mismatch_common(
    FAR struct type_mismatch_data_common *data, uintptr_t ptr)
{
  if (!ptr)
    {
      handle_null_ptr_deref(data);
    }
  else if (data->alignment && !IS_ALIGNED(ptr, data->alignment))
    {
      handle_misaligned_access(data, ptr);
    }
  else
    {
      handle_object_size_mismatch(data, ptr);
    }
}

static bool type_is_int(FAR struct type_descriptor *type)
{
  return type->type_kind == TYPE_KIND_INT;
}

static bool type_is_signed(FAR struct type_descriptor *type)
{
  return type->type_info & 1;
}

static unsigned type_bit_width(FAR struct type_descriptor *type)
{
  return 1 << (type->type_info >> 1);
}

static bool is_inline_int(FAR struct type_descriptor *type)
{
  unsigned inline_bits = sizeof(uintptr_t) * 8;
  unsigned bits = type_bit_width(type);

  return bits <= inline_bits;
}

static int64_t get_signed_val(FAR struct type_descriptor *type,
                              FAR void *val)
{
  if (is_inline_int(type))
    {
      unsigned extra_bits = sizeof(int64_t) * 8 - type_bit_width(type);
      uintptr_t ulong_val = (uintptr_t)val;

      return ((int64_t)ulong_val) << extra_bits >> extra_bits;
    }

  return *(FAR int64_t *)val;
}

static bool val_is_negative(FAR struct type_descriptor *type, FAR void *val)
{
  return type_is_signed(type) && get_signed_val(type, val) < 0;
}

static uint64_t get_unsigned_val(FAR struct type_descriptor *type,
                                 FAR void *val)
{
  if (is_inline_int(type))
    {
      return (uintptr_t)val;
    }

  return *(uint64_t *)val;
}

static void val_to_string(FAR char *str, size_t size,
                          FAR struct type_descriptor *type,
                          FAR void *value)
{
  if (type_is_int(type))
    {
      if (type_is_signed(type))
        {
          snprintf(str, size, "%" PRId64, get_signed_val(type, value));
        }
      else
        {
          snprintf(str, size, "%" PRIu64, get_unsigned_val(type, value));
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void __ubsan_handle_out_of_bounds(FAR void *data, FAR void *index)
{
  FAR struct out_of_bounds_data *info = data;
  char index_str[40];

  ubsan_prologue(&info->location, "array-index-out-of-bounds");

  val_to_string(index_str, sizeof(index_str), info->index_type, index);
  _alert("index %s is out of range for type %s\n",
         index_str, info->array_type->type_name);
  ubsan_epilogue();
}

void __ubsan_handle_shift_out_of_bounds(FAR void *data,
                                        FAR void *lhs, FAR void *rhs)
{
  FAR struct shift_out_of_bounds_data *info = data;
  FAR struct type_descriptor *rhs_type = info->rhs_type;
  FAR struct type_descriptor *lhs_type = info->lhs_type;
  char rhs_str[40];
  char lhs_str[40];

  ubsan_prologue(&info->location, "shift-out-of-bounds");

  val_to_string(rhs_str, sizeof(rhs_str), rhs_type, rhs);
  val_to_string(lhs_str, sizeof(lhs_str), lhs_type, lhs);

  if (val_is_negative(rhs_type, rhs))
    {
      _alert("shift exponent %s is negative\n", rhs_str);
    }
  else if (get_unsigned_val(rhs_type, rhs) >= type_bit_width(lhs_type))
    {
      _alert("shift exponent %s is too large for %u-bit type %s\n",
             rhs_str, type_bit_width(lhs_type), lhs_type->type_name);
    }
  else if (val_is_negative(lhs_type, lhs))
    {
      _alert("left shift of negative value %s\n", lhs_str);
    }
  else
    {
      _alert("left shift of %s by %s places cannot be"
             " represented in type %s\n",
             lhs_str, rhs_str, lhs_type->type_name);
    }

  ubsan_epilogue();
}

void __ubsan_handle_divrem_overflow(FAR void *data,
                                    FAR void *lhs, FAR void *rhs)
{
  FAR struct overflow_data *info = data;
  char rhs_val_str[40];

  ubsan_prologue(&info->location, "division-overflow");

  val_to_string(rhs_val_str, sizeof(rhs_val_str), info->type, rhs);

  if (type_is_signed(info->type) && get_signed_val(info->type, rhs) == -1)
    {
      _alert("division of %s by -1 cannot be represented in type %s\n",
               rhs_val_str, info->type->type_name);
    }
  else
    {
      _alert("division by zero\n");
    }
}

void __ubsan_handle_alignment_assumption(FAR void *data, uintptr_t ptr,
                                         uintptr_t align, uintptr_t offset)
{
  FAR struct alignment_assumption_data *info = data;
  uintptr_t real_ptr;

  ubsan_prologue(&info->location, "alignment-assumption");

  if (offset)
    {
      _alert("assumption of %u byte alignment (with offset of %u byte) for "
             "pointer of type %s failed",
             align, offset, info->type->type_name);
    }
  else
    {
      _alert("assumption of %u byte alignment for pointer of type %s failed",
             align, info->type->type_name);
    }

  real_ptr = ptr - offset;
  _alert("%saddress is %lu aligned, misalignment offset is %u bytes",
         offset ? "offset " : "",
         1ul << (real_ptr ? ffsl(real_ptr) : 0),
         real_ptr & (align - 1));

  ubsan_epilogue();
}

void __ubsan_handle_type_mismatch(FAR struct type_mismatch_data *data,
                                  FAR void *ptr)
{
  struct type_mismatch_data_common common_data =
  {
    .location        = &data->location,
    .type            = data->type,
    .alignment       = data->alignment,
    .type_check_kind = data->type_check_kind
  };

  ubsan_type_mismatch_common(&common_data, (uintptr_t)ptr);
}

void __ubsan_handle_type_mismatch_v1(FAR void *_data, FAR void *ptr)
{
  FAR struct type_mismatch_data_v1 *data = _data;
  struct type_mismatch_data_common common_data =
  {
      .location        = &data->location,
      .type            = data->type,
      .alignment       = 1ul << data->log_alignment,
      .type_check_kind = data->type_check_kind
  };

  ubsan_type_mismatch_common(&common_data, (uintptr_t)ptr);
}

void __ubsan_handle_builtin_unreachable(FAR void *data)
{
  ubsan_prologue_epilogue(data, "unreachable");
  PANIC();
}

void __ubsan_handle_nonnull_arg(FAR void *data)
{
  ubsan_prologue_epilogue(data, "nonnull-arg");
}

void __ubsan_handle_add_overflow(FAR void *data,
                                 FAR void *lhs, FAR void *rhs)
{
  ubsan_prologue_epilogue(data, "add-overflow");
}

void __ubsan_handle_sub_overflow(FAR void *data,
                                 FAR void *lhs, FAR void *rhs)
{
  ubsan_prologue_epilogue(data, "sub-overflow");
}

void __ubsan_handle_mul_overflow(FAR void *data,
                                 FAR void *lhs, FAR void *rhs)
{
  ubsan_prologue_epilogue(data, "mul-overflow");
}

void __ubsan_handle_load_invalid_value(FAR void *data, FAR void *ptr)
{
  ubsan_prologue_epilogue(data, "load-invalid-value");
}

void __ubsan_handle_negate_overflow(FAR void *data, FAR void *ptr)
{
  ubsan_prologue_epilogue(data, "negate-overflow");
}

void __ubsan_handle_pointer_overflow(FAR void *data,
                                     FAR void *ptr, FAR void *result)
{
  ubsan_prologue_epilogue(data, "pointer-overflow");
}
