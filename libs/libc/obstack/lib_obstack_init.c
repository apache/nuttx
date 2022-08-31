/****************************************************************************
 * libs/libc/obstack/lib_obstack_init.c
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
#include <stdio.h>
#include <assert.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: obstack_init
 *
 * Description:
 *   Initialize obstack for allocation of objects.
 *   Compared to the GlibC version this won't initialize a first chunk.
 *
 * Input Parameters:
 *   h: pointer to the handle to initialize
 *
 ****************************************************************************/

void obstack_init(FAR struct obstack *h)
{
  DEBUGASSERT(h != NULL);

  h->chunk_size = BUFSIZ;
  h->chunk = NULL;
  h->object_base = NULL;
  h->next_free = NULL;
}
