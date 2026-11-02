/****************************************************************************
 * libs/libbuiltin/libgcc/gcov_write.c
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

#include "gcov_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void dump_counter(FAR void *buffer, FAR size_t *off, uint64_t v)
{
  if (buffer)
    {
      *(FAR uint64_t *)((FAR uint8_t *)buffer + *off) = v;
    }

  *off += sizeof(uint64_t);
}

void dump_unsigned(FAR void *buffer, FAR size_t *off, uint32_t v)
{
  if (buffer)
    {
      *(FAR uint32_t *)((FAR uint8_t *)buffer + *off) = v;
    }

  *off += sizeof(uint32_t);
}
