/****************************************************************************
 * binfmt/binfmt_coredump.c
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

#include <nuttx/binfmt/binfmt.h>
#include <errno.h>

#include "binfmt.h"

#ifndef CONFIG_BINFMT_DISABLE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: core_dump
 *
 * Description:
 *   This function for generating core dump stream.
 *
 ****************************************************************************/

int core_dump(FAR struct memory_region_s *regions,
              FAR struct lib_outstream_s *stream,
              pid_t pid)
{
  FAR struct binfmt_s *binfmt;
  int ret = -ENOENT;

  for (binfmt = g_binfmts; binfmt; binfmt = binfmt->next)
    {
      /* Use this handler to try to load the format */

      if (binfmt->coredump)
        {
          ret = binfmt->coredump(regions, stream, pid);
          if (ret == OK)
            {
              break;
            }
        }
    }

  return ret;
}

#endif /* CONFIG_BINFMT_DISABLE */
