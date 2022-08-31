/****************************************************************************
 * libs/libc/obstack/lib_obstack_copy.c
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

#include <obstack.h>
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: obstack_copy
 *
 * Description:
 *   Allocate an object of given size with contents copied from address.
 *   The same remarks regarding the allocation apply here as for
 *   obstack_alloc.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocate an object in
 *   address: pointer to the bytes to be used to initialize new object
 *   size: number of bytes to allocate
 *
 ****************************************************************************/

FAR void *obstack_copy(FAR struct obstack *h,
                       FAR const void *address, size_t size)
{
  FAR void *res = obstack_alloc(h, size);
  memcpy(res, address, size);
  return res;
}

/****************************************************************************
 * Name: obstack_copy0
 *
 * Description:
 *   Allocate an object of given size+1 with contents copied from address and
 *   append null byte at the end.
 *   The same remarks regarding the allocation apply here as for
 *   obstack_alloc.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocate an object in
 *   address: pointer to the bytes to be used to initialize new object
 *   size: number of bytes to allocate (excluding the null byte)
 *
 ****************************************************************************/

FAR void *obstack_copy0(FAR struct obstack *h,
                        FAR const void *address, size_t size)
{
  FAR char *res = obstack_alloc(h, size + 1);
  memcpy(res, address, size);
  res[size] = '\0';
  return res;
}
