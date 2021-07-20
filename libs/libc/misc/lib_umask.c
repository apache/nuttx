/****************************************************************************
 * libs/libc/misc/lib_umask.c
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

#include <sys/stat.h>
#include <nuttx/tls.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: umask
 *
 * Description:
 *    Set and get the file mode creation mask
 *
 * Input Parameters:
 *   mask - The process' file mode creation mask
 *
 * Returned Value:
 *   The previous value of the mask.
 *
 ****************************************************************************/

mode_t umask(mode_t mask)
{
  FAR struct task_info_s *info;
  mode_t prev;

  info = task_get_info();
  prev = info->ta_umask;
  info->ta_umask = mask;

  return prev;
}

/****************************************************************************
 * Name: getumask
 ****************************************************************************/

mode_t getumask(void)
{
  FAR struct task_info_s *info;

  info = task_get_info();
  return info->ta_umask;
}
