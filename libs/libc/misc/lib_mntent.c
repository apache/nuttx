/****************************************************************************
 * libs/libc/misc/lib_mntent.c
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

#include <limits.h>
#include <mntent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Size of the line buffer that getmntent() uses */

#define MNTENT_BUFSIZE (PATH_MAX + 64)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mntent g_mntent;
static char g_mntbuf[MNTENT_BUFSIZE];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setmntent
 *
 * Description:
 *   Open a file system description file, such as MOUNTED.
 *
 ****************************************************************************/

FAR FILE *setmntent(FAR const char *filename, FAR const char *type)
{
  return fopen(filename, type);
}

/****************************************************************************
 * Name: getmntent_r
 *
 * Description:
 *   Read the next entry of a file system description file into mntbuf,
 *   with its strings in buf.  Two line formats are accepted: the one of
 *   MOUNTED ("<mount point> type <type>") and the one of fstab and mtab
 *   ("<fsname> <dir> <type> [<options> [<freq> [<passno>]]]").  Blank
 *   lines, comments and lines longer than buflen are skipped.
 *
 * Returned Value:
 *   mntbuf, or NULL at the end of the file.
 *
 ****************************************************************************/

FAR struct mntent *getmntent_r(FAR FILE *stream, FAR struct mntent *mntbuf,
                               FAR char *buf, int buflen)
{
  FAR char *field[6];
  FAR char *save;
  FAR char *p;
  size_t len;
  int n;
  int c;

  while (fgets(buf, buflen, stream) != NULL)
    {
      len = strlen(buf);
      if (len > 0 && buf[len - 1] != '\n' && !feof(stream))
        {
          /* The line does not fit: skip the rest of it */

          while ((c = fgetc(stream)) != EOF && c != '\n')
            {
            }

          continue;
        }

      n = 0;
      for (p = strtok_r(buf, " \t\n", &save); p != NULL && n < 6;
           p = strtok_r(NULL, " \t\n", &save))
        {
          field[n++] = p;
        }

      if (n < 3 || field[0][0] == '#')
        {
          continue;
        }

      if (n == 3 && strcmp(field[1], "type") == 0)
        {
          /* MOUNTED: "<mount point> type <type>".  The file system name is
           * not known; use the type, as for a virtual file system.
           */

          mntbuf->mnt_fsname = field[2];
          mntbuf->mnt_dir    = field[0];
          mntbuf->mnt_type   = field[2];
          mntbuf->mnt_opts   = (FAR char *)MNTOPT_RW;
          mntbuf->mnt_freq   = 0;
          mntbuf->mnt_passno = 0;
        }
      else
        {
          mntbuf->mnt_fsname = field[0];
          mntbuf->mnt_dir    = field[1];
          mntbuf->mnt_type   = field[2];
          mntbuf->mnt_opts   = n > 3 ? field[3] :
                               (FAR char *)MNTOPT_DEFAULTS;
          mntbuf->mnt_freq   = n > 4 ? atoi(field[4]) : 0;
          mntbuf->mnt_passno = n > 5 ? atoi(field[5]) : 0;
        }

      return mntbuf;
    }

  return NULL;
}

/****************************************************************************
 * Name: getmntent
 *
 * Description:
 *   As getmntent_r(), with a static entry and buffer.
 *
 ****************************************************************************/

FAR struct mntent *getmntent(FAR FILE *stream)
{
  return getmntent_r(stream, &g_mntent, g_mntbuf, sizeof(g_mntbuf));
}

/****************************************************************************
 * Name: addmntent
 *
 * Description:
 *   Append an entry in the fstab and mtab format to the file.
 *
 * Returned Value:
 *   0 on success, 1 on failure.
 *
 ****************************************************************************/

int addmntent(FAR FILE *stream, FAR const struct mntent *mnt)
{
  if (fseek(stream, 0, SEEK_END) < 0)
    {
      return 1;
    }

  return fprintf(stream, "%s %s %s %s %d %d\n", mnt->mnt_fsname,
                 mnt->mnt_dir, mnt->mnt_type, mnt->mnt_opts,
                 mnt->mnt_freq, mnt->mnt_passno) < 0 ? 1 : 0;
}

/****************************************************************************
 * Name: endmntent
 *
 * Description:
 *   Close a file opened with setmntent().
 *
 * Returned Value:
 *   Always 1.
 *
 ****************************************************************************/

int endmntent(FAR FILE *stream)
{
  if (stream != NULL)
    {
      fclose(stream);
    }

  return 1;
}

/****************************************************************************
 * Name: hasmntopt
 *
 * Description:
 *   Find the option opt in the comma-separated options of an entry.
 *
 * Returned Value:
 *   A pointer to the option in mnt->mnt_opts, or NULL.
 *
 ****************************************************************************/

FAR char *hasmntopt(FAR const struct mntent *mnt, FAR const char *opt)
{
  size_t len = strlen(opt);
  FAR char *p = mnt->mnt_opts;

  while (p != NULL && *p != '\0')
    {
      if (strncmp(p, opt, len) == 0 &&
          (p[len] == '\0' || p[len] == ',' || p[len] == '='))
        {
          return p;
        }

      p = strchr(p, ',');
      if (p != NULL)
        {
          p++;
        }
    }

  return NULL;
}
