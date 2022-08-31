/****************************************************************************
 * libs/libc/obstack/lib_obstack_grow.c
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
 * Name: obstack_grow
 *
 * Description:
 *   Grow object by given size and allocated it with bytes from address.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocated object to
 *   address: pointer to the bytes to be used to initialize the new object
 *   size: number of bytes to grow object
 *
 ****************************************************************************/

void obstack_grow(FAR struct obstack *h,
                  FAR const void *address, size_t size)
{
  obstack_make_room(h, size);
  memcpy(h->next_free, address, size);
  h->next_free += size;
}

/****************************************************************************
 * Name: obstack_grow0
 *
 * Description:
 *   Grow object by given size+1 and allocated it with bytes from address
 *   plus null byte at the end.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocated object to
 *   address: pointer to the bytes to be used to initialize the new object
 *   size: number of bytes to grow object (excluding the null byte)
 *
 ****************************************************************************/

void obstack_grow0(FAR struct obstack *h,
                   FAR const void *address, size_t size)
{
  obstack_make_room(h, size + 1);
  memcpy(h->next_free, address, size);
  h->next_free += size;
  *h->next_free = '\0';
  h->next_free++;
}

/****************************************************************************
 * Name: obstack_1grow
 *
 * Description:
 *   Grow object by single data byte.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocated object to
 *   data: byte to be added to the growing object
 *
 ****************************************************************************/

void obstack_1grow(FAR struct obstack *h, char data)
{
  obstack_make_room(h, 1);
  obstack_1grow_fast(h, data);
}
