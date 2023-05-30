/****************************************************************************
 * libs/libc/stdio/lib_libgetstreams.c
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
#include <assert.h>

#include <nuttx/tls.h>
#include <nuttx/lib/lib.h>

#ifdef CONFIG_FILE_STREAM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_get_streams
 *
 * Description:
 *   Return a pointer to the streams list for this thread
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A pointer to the errno.
 *
 * Assumptions:
 *
 ****************************************************************************/

FAR struct streamlist *lib_get_streams(void)
{
  FAR struct task_info_s *info;

  info = task_get_info();
  return &info->ta_streamlist;
}

/****************************************************************************
 * Name: lib_get_stream
 *
 * Description:
 *   Return a pointer to the file stream for this thread and given fd。
 *   Note: only reserved fd number 0/1/2 is valid.
 *
 ****************************************************************************/

FAR struct file_struct *lib_get_stream(int fd)
{
  return &lib_get_streams()->sl_std[fd];
}

#endif /* CONFIG_FILE_STREAM */
