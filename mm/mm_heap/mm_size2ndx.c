/****************************************************************************
 * mm/mm_heap/mm_size2ndx.c
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

#include <nuttx/config.h>

#include <nuttx/mm/mm.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_size2ndx
 *
 * Description:
 *    Convert the size to a nodelist index.
 *
 ****************************************************************************/

int mm_size2ndx(size_t size)
{
  int ndx = 0;

  if (size >= MM_MAX_CHUNK)
    {
      return MM_NNODES - 1;
    }

  size >>= MM_MIN_SHIFT;
  while (size > 1)
    {
      ndx++;
      size >>= 1;
    }

  return ndx;
}
