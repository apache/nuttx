/****************************************************************************
 * libs/libc/dirent/lib_versionsort.c
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

#include <string.h>
#include <dirent.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: versionsort
 *
 * Description:
 *   The versionsort() function can be used as the comparison function
 *   compar() for scandir().  It sorts directory entries using strverscmp
 *   on the strings (*a)->d_name and (*b)->d_name.
 *
 * Input Parameters:
 *   a - The first direntry to compare
 *   b - The second direntry to compare
 *
 * Returned Value:
 *   An integer less than, equal to, or greater than zero if the first
 *   argument is considered to be respectively less than, equal to, or
 *   greater than the second.
 *
 ****************************************************************************/

int versionsort(FAR const struct dirent **a, FAR const struct dirent **b)
{
  return strverscmp((*a)->d_name, (*b)->d_name);
}
