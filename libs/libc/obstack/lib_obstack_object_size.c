/****************************************************************************
 * libs/libc/obstack/lib_obstack_object_size.c
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
 * Name: obstack_object_size
 *
 * Description:
 *   Calculate the size of the currently growing object.
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *
 * Returned Value:
 *   Size of the object.
 *
 ****************************************************************************/

size_t obstack_object_size(FAR struct obstack *h)
{
  return h->next_free - h->object_base;
}
