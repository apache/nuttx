/****************************************************************************
 * libs/libxx/libcxxmini/libxx_dynamic_cast.cxx
 *
 * Copyright 2010-2011 PathScale, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "libxx_typeinfo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

using namespace __cxxabiv1;

#define ADD_TO_PTR(x, off) \
        reinterpret_cast<__typeof__(x)>(reinterpret_cast<char*>(x) + off)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vtable_header
{
  /* Offset of the leaf object. */

  ptrdiff_t leaf_offset;

  /* Type of the object. */

  const __class_type_info *type;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool std::type_info::__do_catch(std::type_info const *ex_type,
                                void **exception_object,
                                unsigned int outer) const
{
  const type_info *type = this;

  if (type == ex_type)
  {
    return true;
  }

#ifdef CONFIG_CXX_RTTI
  if (const __class_type_info *cti = dynamic_cast<const __class_type_info *>(type))
  {
    return ex_type->__do_upcast(cti, exception_object);
  }
#endif

  return false;
}

void *__class_type_info::cast_to(
                void *obj, const struct __class_type_info *other) const
{
  if (this == other)
  {
    return obj;
  }
  return 0;
}

void *__si_class_type_info::cast_to(
                void *obj, const struct __class_type_info *other) const
{
  if (this == other)
  {
    return obj;
  }
  return __base_type->cast_to(obj, other);
}

bool __si_class_type_info::__do_upcast(const __class_type_info *target,
                                     void **thrown_object) const
{
  if (this == target)
  {
    return true;
  }
  return __base_type->__do_upcast(target, thrown_object);
}

void *__vmi_class_type_info::cast_to(
                void *obj, const struct __class_type_info *other) const
{
  if (__do_upcast(other, &obj))
  {
    return obj;
  }
  return 0;
}

bool __vmi_class_type_info::__do_upcast(const __class_type_info *target,
                          void **thrown_object) const
{
  if (this == target)
  {
    return true;
  }
  for (unsigned int i=0 ; i<__base_count ; i++)
  {
    const __base_class_type_info *info = &__base_info[i];
    ptrdiff_t offset = info->offset();

    /* If this is a virtual superclass, the offset is stored in the
     * object's vtable at the offset requested; 2.9.5.6.c:
     *
     * 'For a non-virtual base, this is the offset in the object of the
     * base subobject. For a virtual base, this is the offset in the
     * virtual table of the virtual base offset for the virtual base
     * referenced (negative).'
     */

    void *obj = *thrown_object;
    if (info->is_virtual())
    {
      // Object's vtable
      ptrdiff_t *off = *static_cast<ptrdiff_t**>(obj);
      // Offset location in vtable
      off = ADD_TO_PTR(off, offset);
      offset = *off;
    }
    void *cast = ADD_TO_PTR(obj, offset);

    if (info->__base_type == target ||
        (info->__base_type->__do_upcast(target, &cast)))
    {
      *thrown_object = cast;
      return true;
    }
  }
  return 0;
}

#ifdef CONFIG_CXX_RTTI
extern "C" void* __dynamic_cast(const void *sub,
                                const __class_type_info *src,
                                const __class_type_info *dst,
                                ptrdiff_t src2dst_offset)
{
  const char *vtable_location = *static_cast<const char * const *>(sub);
  const vtable_header *header =
    reinterpret_cast<const vtable_header*>(vtable_location - sizeof(vtable_header));
  void *leaf = ADD_TO_PTR(const_cast<void *>(sub), header->leaf_offset);
  return header->type->cast_to(leaf, dst);
}
#endif
