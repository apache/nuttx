/****************************************************************************
 * libs/libc/obstack/lib_obstack_blank.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: obstack_blank
 *
 * Description:
 *   Grow object by given size. The bytes are uninitialized.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocated object to
 *   size: number of bytes to grow object
 *
 ****************************************************************************/

void obstack_blank(FAR struct obstack *h, size_t size)
{
  obstack_make_room(h, size);
  obstack_blank_fast(h, size);
}
