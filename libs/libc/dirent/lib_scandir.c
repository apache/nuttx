/****************************************************************************
 * libs/libc/dirent/lib_scandir.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Michael Jung <mijung@gmx.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
