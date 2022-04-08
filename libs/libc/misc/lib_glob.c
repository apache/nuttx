/****************************************************************************
 * libs/libc/misc/lib_glob.c
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

#include <dirent.h>
#include <errno.h>
#include <fnmatch.h>
#include <glob.h>
#include <limits.h>
#include <pwd.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "libc.h"

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

struct match_s
{
  FAR struct match_s *next;
  char name[1];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int append(FAR struct match_s **tail, FAR const char *name,
                  size_t len, int mark);
static int do_glob(FAR char *buf, size_t pos, int type, FAR char *pat,
                   int flags,
                   CODE int (*errfunc)(FAR const char *path, int err),
                   FAR struct match_s **tail);
static int ignore_err(FAR const char *path, int err);
static void freelist(FAR struct match_s *head);
static int sort(FAR const void *a, FAR const void *b);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: append
 ****************************************************************************/

static int append(FAR struct match_s **tail, FAR const char *name,
                  size_t len, int mark)
{
  FAR struct match_s *new = lib_malloc(sizeof(struct match_s) + len + 1);
  if (new == NULL)
    {
      return -1;
    }

  (*tail)->next = new;
  new->next = NULL;
  memcpy(new->name, name, len + 1);
  if (mark && len && name[len - 1] != '/')
    {
      new->name[len] = '/';
      new->name[len + 1] = '\0';
    }

  *tail = new;
  return 0;
}

/****************************************************************************
 * Name: do_glob
 ****************************************************************************/

static int do_glob(FAR char *buf, size_t pos, int type, FAR char *pat,
                   int flags,
                   CODE int (*errfunc)(FAR const char *path, int err),
                   FAR struct match_s **tail)
{
  ptrdiff_t i = 0;
  ptrdiff_t j = 0;
  int in_bracket = 0;
  int overflow = 0;
  FAR char *p2;
  char saved_sep = '/';
  FAR DIR *dir;
  int old_errno;
  FAR struct dirent *de;
  int readerr;

  /* If GLOB_MARK is unused, we don't care about type. */

  if (!type && !(flags & GLOB_MARK))
    {
      type = DT_REG;
    }

  /* Special-case the remaining pattern being all slashes, in
   * which case we can use caller-passed type if it's a dir.
   */

  if (*pat && type != DT_DIR)
    {
      type = 0;
    }

  while (pos + 1 < PATH_MAX && *pat == '/')
    {
      buf[pos++] = *pat++;
    }

  /* Consume maximal [escaped-]literal prefix of pattern, copying
   * and un-escaping it to the running buffer as we go.
   */

  for (; pat[i] != '*' && pat[i] != '?'
       && (!in_bracket || pat[i] != ']'); i++)
    {
      if (!pat[i])
        {
          if (overflow)
            {
              return 0;
            }

          pat += i;
          pos += j;
          i = j = 0;
          break;
        }
      else if (pat[i] == '[')
        {
          in_bracket = 1;
        }
      else if (pat[i] == '\\' && !(flags & GLOB_NOESCAPE))
        {
          /* Backslashes inside a bracket are (at least by
           * our interpretation) non-special, so if next
           * char is ']' we have a complete expression.
           */

          if (in_bracket && pat[i + 1] == ']')
            {
              break;
            }

          /* Unpaired final backslash never matches. */

          if (!pat[i + 1])
            {
              return 0;
            }

          i++;
        }

      if (pat[i] == '/')
        {
          if (overflow)
            {
              return 0;
            }

          in_bracket = 0;
          pat += i + 1;
          i = -1;
          pos += j + 1;
          j = -1;
        }

      /* Only store a character if it fits in the buffer, but if
       * a potential bracket expression is open, the overflow
       * must be remembered and handled later only if the bracket
       * is unterminated (and thereby a literal), so as not to
       * disallow long bracket expressions with short matches.
       */

      if (pos + (j + 1) < PATH_MAX)
        {
          buf[pos + j++] = pat[i];
        }
      else if (in_bracket)
        {
          overflow = 1;
        }
      else
        {
          return 0;
        }

      /* If we consume any new components, the caller-passed type
       * or dummy type from above is no longer valid.
       */

      type = 0;
    }

  buf[pos] = 0;
  if (!*pat)
    {
      /* If we consumed any components above, or if GLOB_MARK is
       * requested and we don't yet know if the match is a dir,
       * we must confirm the file exists and/or determine its type.
       *
       * If marking dirs, symlink type is inconclusive; we need the
       * type for the symlink target, and therefore must try stat
       * first unless type is known not to be a symlink. Otherwise,
       * or if that fails, use lstat for determining existence to
       * avoid false negatives in the case of broken symlinks.
       */

      struct stat st;
      if ((flags & GLOB_MARK) && (!type || type == DT_LNK)
           && !stat(buf, &st))
        {
          if (S_ISDIR(st.st_mode))
            {
              type = DT_DIR;
            }
          else
            {
              type = DT_REG;
            }
        }

      if (!type && lstat(buf, &st))
        {
          if (errno != ENOENT && (errfunc(buf, errno) || (flags & GLOB_ERR)))
            {
              return GLOB_ABORTED;
            }

          return 0;
        }

      if (append(tail, buf, pos, (flags & GLOB_MARK) && type == DT_DIR))
        {
          return GLOB_NOSPACE;
        }

      return 0;
    }

  p2 = strchr(pat, '/');

  /* Check if the '/' was escaped and, if so, remove the escape char
   * so that it will not be unpaired when passed to fnmatch.
   */

  if (p2 && !(flags & GLOB_NOESCAPE))
    {
      FAR char *p;
      const int prev_index = -1;
      for (p = p2; p > pat && p[prev_index] == '\\'; p--)
        {
        }

      if ((p2 - p) % 2)
        {
          p2--;
          saved_sep = '\\';
        }
    }

  dir = opendir(pos ? buf : ".");
  if (!dir)
    {
      if (errfunc(buf, errno) || (flags & GLOB_ERR))
        {
          return GLOB_ABORTED;
        }

      return 0;
    }

  old_errno = errno;
  while (errno = 0, de = readdir(dir))
    {
      size_t l;
      int fnm_flags;
      int r;

      /* Quickly skip non-directories when there's pattern left. */

      if (p2 && de->d_type && de->d_type != DT_DIR && de->d_type != DT_LNK)
        {
          continue;
        }

      l = strlen(de->d_name);
      if (l >= PATH_MAX - pos)
        {
          continue;
        }

      if (p2)
        {
          *p2 = 0;
        }

      fnm_flags = ((flags & GLOB_NOESCAPE) ? FNM_NOESCAPE : 0)
                  | FNM_PERIOD;

      if (fnmatch(pat, de->d_name, fnm_flags))
        {
          continue;
        }

      strlcpy(buf + pos, de->d_name, l + 1);

      if (p2)
        {
          *p2 = saved_sep;
        }

      r = do_glob(buf, pos + l, de->d_type, p2 ? p2 : "",
                      flags, errfunc, tail);
      if (r)
        {
          closedir(dir);
          return r;
        }
    }

  readerr = errno;
  if (p2)
    {
      *p2 = saved_sep;
    }

  closedir(dir);
  if (readerr && (errfunc(buf, errno) || (flags & GLOB_ERR)))
    {
      return GLOB_ABORTED;
    }

  errno = old_errno;
  return 0;
}

/****************************************************************************
 * Name: ignore_err
 ****************************************************************************/

static int ignore_err(FAR const char *path, int err)
{
  return 0;
}

/****************************************************************************
 * Name: freelist
 ****************************************************************************/

static void freelist(FAR struct match_s *head)
{
  FAR struct match_s *match;
  FAR struct match_s *next;
  for (match = head->next; match; match = next)
    {
      next = match->next;
      lib_free(match);
    }
}

/****************************************************************************
 * Name: sort
 ****************************************************************************/

static int sort(FAR const void *a, FAR const void *b)
{
  return strcmp(*(FAR const char**)a, *(FAR const char**)b);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: glob
 ****************************************************************************/

int glob(FAR const char *pat, int flags,
         CODE int (*errfunc)(FAR const char *path, int err),
         FAR glob_t *g)
{
  struct match_s head;
  FAR struct match_s *tail = &head;
  size_t cnt;
  size_t i;
  size_t offs = (flags & GLOB_DOOFFS) ? g->gl_offs : 0;
  int error = 0;
  char buf[PATH_MAX];

  head.next = NULL;
  head.name[0] = '\0';

  if (!errfunc)
    {
      errfunc = ignore_err;
    }

  if (!(flags & GLOB_APPEND))
    {
      g->gl_offs = offs;
      g->gl_pathc = 0;
      g->gl_pathv = NULL;
    }

  if (*pat)
    {
      FAR char *p = strdup(pat);
      size_t pos = 0;
      FAR char *s;

      if (!p)
        {
          return GLOB_NOSPACE;
        }

      buf[0] = 0;
      s = p;

      error = do_glob(buf, pos, 0, s, flags, errfunc, &tail);

      lib_free(p);
    }

  if (error == GLOB_NOSPACE)
    {
      freelist(&head);
      return error;
    }

  for (cnt = 0, tail = head.next; tail; tail = tail->next, cnt++)
    {
    }

  if (!cnt)
    {
      if (flags & GLOB_NOCHECK)
        {
          tail = &head;
          if (append(&tail, pat, strlen(pat), 0))
            {
              return GLOB_NOSPACE;
            }

          cnt++;
        }
      else
        {
          return GLOB_NOMATCH;
        }
    }

  if (flags & GLOB_APPEND)
    {
      FAR char **pathv = lib_realloc(g->gl_pathv,
                        (offs + g->gl_pathc + cnt + 1) * sizeof(FAR char *));
      if (!pathv)
        {
          freelist(&head);
          return GLOB_NOSPACE;
        }

      g->gl_pathv = pathv;
      offs += g->gl_pathc;
    }
  else
    {
      g->gl_pathv = lib_malloc((offs + cnt + 1) * sizeof(FAR char *));
      if (!g->gl_pathv)
        {
          freelist(&head);
          return GLOB_NOSPACE;
        }

      for (i = 0; i < offs; i++)
        {
          g->gl_pathv[i] = NULL;
        }
    }

  for (i = 0, tail = head.next; i < cnt; tail = tail->next, i++)
    {
      g->gl_pathv[offs + i] = tail->name;
    }

  g->gl_pathv[offs + i] = NULL;
  g->gl_pathc += cnt;

  if (!(flags & GLOB_NOSORT))
    {
      qsort(g->gl_pathv + offs, cnt, sizeof(FAR char *), sort);
    }

  return error;
}

/****************************************************************************
 * Name: globfree
 ****************************************************************************/

void globfree(FAR glob_t *g)
{
  size_t i;
  for (i = 0; i < g->gl_pathc; i++)
    {
      lib_free(g->gl_pathv[g->gl_offs + i] - offsetof(struct match_s, name));
    }

  lib_free(g->gl_pathv);
  g->gl_pathc = 0;
  g->gl_pathv = NULL;
}
