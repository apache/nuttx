/****************************************************************************
 * fs/fs_heap.h
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

#ifndef __FS_FS_HEAP_H
#define __FS_FS_HEAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_FS_HEAPSIZE) && CONFIG_FS_HEAPSIZE > 0
void      fs_heap_initialize(void);
FAR void *fs_heap_zalloc(size_t size);
size_t    fs_heap_malloc_size(FAR void *mem);
FAR void *fs_heap_realloc(FAR void *oldmem, size_t size);
void      fs_heap_free(FAR void *mem);
#else
#  define fs_heap_initialize()
#  define fs_heap_zalloc       kmm_zalloc
#  define fs_heap_malloc_size  kmm_malloc_size
#  define fs_heap_realloc      kmm_realloc
#  define fs_heap_free         kmm_free
#endif

#endif
