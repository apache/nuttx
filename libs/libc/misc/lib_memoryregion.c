/****************************************************************************
 * libs/libc/misc/lib_memoryregion.c
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

#include <nuttx/memoryregion.h>
#include <nuttx/lib/lib.h>
#include <stdlib.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: parse_memory_region
 *
 * Input Parameters:
 *   format - The format string to parse. <start>,<end>,<flags>,...
 *            start - The start address of the memory region
 *            end   - The end address of the memory region
 *            flags - Readable 0x1, writable 0x2, executable 0x4
 *  region - The memory region to populate
 *  num    - The number of memory regions to parse
 *
 *  example: 0x1000,0x2000,0x1,0x2000,0x3000,0x3,0x3000,0x4000,0x7
 *
 ****************************************************************************/

ssize_t parse_memory_region(FAR const char *format,
                            FAR struct memory_region_s *region,
                            size_t num)
{
  FAR char *endptr;
  size_t i = 0;

  if (format == NULL)
    {
      return -EINVAL;
    }

  if (num == 0 || region == NULL)
    {
      num = 0;
      while (format[i] != '\0')
        {
          if (format[i++] == ',')
            {
              num++;
            }
        }
    }

  if (region == NULL)
    {
      return num / 3 + 1;
    }

  while (*format != '\0' && i < num * 3)
    {
      if (i % 3 == 0)
        {
          region[i / 3].start = strtoul(format, &endptr, 0);
        }
      else if (i % 3 == 1)
        {
          region[i / 3].end = strtoul(format, &endptr, 0);
        }
      else if (i % 3 == 2)
        {
          region[i / 3].flags = strtoul(format, &endptr, 0);
        }

      format = endptr + 1;
      i++;
    }

  return i / 3;
}

/****************************************************************************
 * Name: alloc_memory_region
 *
 * Input Parameters:
 *   format - The format string to parse. <start>,<end>,<flags>,...
 *            start - The start address of the memory region
 *            end   - The end address of the memory region
 *            flags - Readable 0x1, writable 0x2, executable 0x4
 *  example: 0x1000,0x2000,0x1,0x2000,0x3000,0x3,0x3000,0x4000,0x7
 *
 * Return:
 *   The parsed memory region list on success; NULL on failure.
 *   The boundary value of the memory region is zero.
 *   The return value need free by caller.
 *
 ****************************************************************************/

FAR struct memory_region_s *
alloc_memory_region(FAR const char *format)
{
  FAR struct memory_region_s *region;
  ssize_t num = parse_memory_region(format, NULL, 0);

  if (num < 0)
    {
      return NULL;
    }

  region = lib_zalloc(sizeof(struct memory_region_s) * (num + 1));
  if (region == NULL)
    {
      return NULL;
    }

  parse_memory_region(format, region, num);
  return region;
}

/****************************************************************************
 * Name: free_memory_region
 *
 * Input Parameters:
 *   region - The memory region list to free.
 *
 ****************************************************************************/

void free_memory_region(FAR struct memory_region_s *region)
{
  if (region != NULL)
    {
      lib_free(region);
    }
}
