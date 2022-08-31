/****************************************************************************
 * libs/libc/obstack/lib_obstack_room.c
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
#include <assert.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: obstack_room
 *
 * Description:
 *   Calculate the number of bytes available for growth before reallocation
 *   is required.
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *
 * Returned Value:
 *   Number of free bytes.
 *
 ****************************************************************************/

size_t obstack_room(FAR struct obstack *h)
{
  DEBUGASSERT(h);
  return h->chunk != NULL ? h->chunk->limit - h->next_free : 0;
}
