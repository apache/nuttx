/****************************************************************************
 * sched/module/mod_modsym.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/module.h>
#include <nuttx/lib/modlib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modsym
 *
 * Description:
 *   modsym() returns the address of a symbol defined within the object that
 *   was previously made accessible through a insmod() call.  handle is the
 *   value returned from a call to insmod() (and which has not since been
 *   released via a call to rmmod()), name is the symbol's name as a
 *   character string.
 *
 *   The returned symbol address will remain valid until rmmod() is called.
 *
 * Input Parameters:
 *   handle - The opaque, non-NULL value returned by a previous successful
 *            call to insmod().
 *   name   - A pointer to the symbol name string.
 *
 * Returned Value:
 *   The address associated with the symbol is returned on success.
 *   If handle does not refer to a valid module opened by insmod(), or if
 *   the named symbol cannot be found within any of the objects associated
 *   with handle, modsym() will return NULL and the errno variable will be
 *   set appropriately.
 *
 *   NOTE: This means that the address zero can never be a valid return
 *   value.
 *
 ****************************************************************************/

FAR const void *modsym(FAR void *handle, FAR const char *name)
{
  return modlib_getsymbol(handle, name);
}
