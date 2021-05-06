/****************************************************************************
 * libs/libc/stdlib/lib_ptsname.c
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

#include <stdlib.h>

#ifdef CONFIG_PSEUDOTERM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ptsname
 *
 * Description:
 *   The ptsname() function returns the name of the slave pseudoterminal
 *   device corresponding to the master referred to by fd.
 *
 * Returned Value:
 *   On success, ptsname() returns a pointer to a string in static storage
 *   which will be overwritten by subsequent calls.  This pointer must not
 *   be freed.  On failure, NULL is returned.
 *
 *     ENOTTY fd does not refer to a pseudoterminal master device.
 *
 ****************************************************************************/

FAR char *ptsname(int fd)
{
  static char devname[16];
  int ret = ptsname_r(fd, devname, 16);
  return ret < 0 ? NULL : devname;
}

#endif /* CONFIG_PSEUDOTERM */
