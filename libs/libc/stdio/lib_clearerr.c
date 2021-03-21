/****************************************************************************
 * libs/libc/stdio/lib_clearerr.c
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

#include <stdio.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_FILE_STREAM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: Functions
 *
 * Description:
 *   Clear any end-of-file or error conditions.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void clearerr(FAR FILE *stream)
{
  stream->fs_flags &= (__FS_FLAG_LBF | __FS_FLAG_UBF);
}
#endif /* CONFIG_FILE_STREAM */
