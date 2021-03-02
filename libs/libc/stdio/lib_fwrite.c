/****************************************************************************
 * libs/libc/stdio/lib_fwrite.c
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

#include <sys/types.h>
#include <stdio.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fwrite
 ****************************************************************************/

size_t fwrite(FAR const void *ptr, size_t size, size_t n_items,
              FAR FILE *stream)
{
  size_t  full_size = n_items * (size_t)size;
  ssize_t bytes_written;
  size_t  items_written = 0;

  /* Write the data into the stream buffer */

  bytes_written = lib_fwrite(ptr, full_size, stream);
  if (bytes_written > 0)
    {
      /* Return the number of full items written */

      items_written = bytes_written / size;
    }

  return items_written;
}
