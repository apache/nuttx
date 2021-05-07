/****************************************************************************
 * libs/libc/dirent/lib_scandir.c
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

#include <string.h>
#include <dirent.h>
#include <errno.h>
#include <stdlib.h>

#include "libc.h"

/* The scandir() function is not appropriate for use within the kernel in its
 * current form because it uses user space memory allocators and modifies
 * the errno value.
 */

#ifndef __KERNEL__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: scandir
 *
 * Description:
 *   The scandir() function scans the directory dirp, calling filter() on
 *   each directory entry.  Entries for which filter() returns nonzero are
 *   stored in strings allocated via malloc(), sorted using qsort() with
 *   comparison function compar(), and collected in array namelist which is
 *   allocated via malloc().  If filter is NULL, all entries are selected.
 *
 * Input Parameters:
 *   path     - Pathname of the directory to scan
 *   namelist - An array of pointers to directory entries, which is allocated
 *              by scandir via malloc.  Each directory entry is allocated via
 *              malloc as well.  The caller is responsible to free said
 *              objects.
 *   filter   - Directory entries for which filter returns zero are not
 *              included in the namelist.  If filter is NULL, all entries are
 *              included.
 *   compar   - Comparison function used with qsort() to sort the namelist.
 *
 * Returned Value:
 *   If successful, the scandir() function returns the number of entries in
 *   the namelist.  Otherwise, it returns -1 and errno is set to indicate the
 *   error.
 *
 ****************************************************************************/

int scandir(FAR const char *path, FAR struct dirent ***namelist,
            CODE int (*filter)(FAR const struct dirent *),
            CODE int (*compar)(FAR const struct dirent **,
                               FAR const struct dirent **))
{
  FAR struct dirent *d;
  FAR struct dirent *dnew;
  FAR struct dirent **list = NULL;
  size_t listsize = 0;
  size_t cnt = 0;
  int errsv;
  int result;
  FAR DIR *dirp;

  /* This scandir implementation relies on errno being set by other service
   * functions that it is calling to figure if it was successful.  We save
   * the original errno value to be able to restore it in case of success.
   */

  errsv = get_errno();

  dirp = opendir(path);

  if (!dirp)
    {
      return -1;
    }

  /* opendir might have set errno.  Reset to zero. */

  set_errno(0);

  for (d = readdir(dirp); d != NULL; d = readdir(dirp))
    {
      size_t dsize;

      /* If the caller provided a filter function which tells scandir to skip
       * the current directory entry, do so.
       */

      if (filter && !filter(d))
        {
          continue;
        }

      /* The caller provided filter function might have set errno.  Reset to
       * zero.
       */

      set_errno(0);

      /* Grow the directory entry list, if required. */

      if (cnt == listsize)
        {
          struct dirent **newlist;

          if (!listsize)
            {
              listsize = 4;
            }
          else
            {
              listsize *= 2;
            }

          newlist = lib_realloc(list, listsize * sizeof(*list));

          if (!newlist)
            {
              /* realloc failed and set errno.  This will tell follow up code
               * that we failed.
               */

              break;
            }

          list = newlist;
        }

      /* Allocate a new directory entry, but restrict its heap size to what
       * is really required given the directories' path name.
       */

      dsize = (size_t)(&d->d_name[strlen(d->d_name) + 1] - (char *)d);
      dnew = lib_malloc(dsize);
      if (!dnew)
        {
          /* malloc failed and set errno.  This will tell follow up code that
           * we failed.
           */

          break;
        }

      /* Copy directory entry to newly allocated one and update the list
       * accordingly.
       */

      memcpy(dnew, d, dsize);
      list[cnt] = dnew;
      cnt++;

      /* Some service function might have set errno as a side effect.  Reset
       * to zero.
       */

      set_errno(0);
    }

  if (get_errno() == 0)
    {
      /* If the caller provided a comparison function, use it to sort the
       * list of directory entries.
       */

      if (compar)
        {
          typedef int (*compar_fn_t)(FAR const void *, FAR const void *);
          qsort(list, cnt, sizeof(*list), (compar_fn_t)compar);
        }

      /* Set the output parameters. */

      *namelist = list;
      result = (int)cnt;
    }
  else
    {
      size_t i;

      /* Something failed along the way.  Clean up. */

      for (i = 0; i < cnt; i++)
        {
          lib_free(list[i]);
        }

      lib_free(list);

      result = -1;
    }

  closedir(dirp);

  if (result >= 0)
    {
      /* Restore original errno value in case of success. */

      set_errno(errsv);
    }

  return result;
}

#endif /* __KERNEL__ */
