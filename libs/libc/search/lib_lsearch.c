/****************************************************************************
 * libs/libc/search/lib_lsearch.c
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

#include <search.h>
#include <string.h>

#ifdef CONFIG_FDPIC
#  include <nuttx/fdpic.h>
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lfind
 *
 * Description:
 *   Search the table base of *nelp elements of width bytes for key, with
 *   compar() (0 for a match).
 *
 * Returned Value:
 *   A pointer to the first matching element, or NULL if there is none.
 *
 ****************************************************************************/

FAR void *lfind(FAR const void *key, FAR const void *base,
                FAR size_t *nelp, size_t width,
                CODE int (*compar)(FAR const void *, FAR const void *))
{
  FAR const char *p = base;
  size_t i;

#ifdef CONFIG_FDPIC
  /* See qsort(): an FDPIC caller passes a descriptor, not a code address.
   * lsearch() hands its compar to this function, which resolves it once.
   */

  compar = (CODE int (*)(FAR const void *, FAR const void *))
           fdpic_callback((FAR void *)compar);
#endif

  for (i = 0; i < *nelp; i++, p += width)
    {
      if (compar(key, p) == 0)
        {
          return (FAR void *)p;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: lsearch
 *
 * Description:
 *   As lfind(), but if key is not in the table, copy it to the end of the
 *   table and increment *nelp.  The table must have room for it.
 *
 * Returned Value:
 *   A pointer to the matching or the new element.
 *
 ****************************************************************************/

FAR void *lsearch(FAR const void *key, FAR void *base, FAR size_t *nelp,
                  size_t width,
                  CODE int (*compar)(FAR const void *, FAR const void *))
{
  FAR char *p = lfind(key, base, nelp, width, compar);

  if (p == NULL)
    {
      p = base;
      p += *nelp * width;
      memcpy(p, key, width);
      (*nelp)++;
    }

  return p;
}
