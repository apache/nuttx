/****************************************************************************
 * libs/libc/grp/lib_find_grpfile.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <grp.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/lib/lib.h>

#include "grp/lib_grp.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef CODE int (grp_foreach_match_t)(FAR const struct group *entry,
                                       uintptr_t arg);

/* Context used by grp_match_user() to accumulate the group IDs of all the
 * groups that a given user is a member of.
 */

struct grp_user_s
{
  FAR const char *user;       /* User name to search for */
  gid_t           group;      /* Primary group to skip (avoid duplicate) */
  FAR gid_t      *grouplist;  /* Output array of group IDs */
  int             maxgroups;  /* Number of slots in grouplist */
  int             count;      /* Number of group IDs found so far */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: grp_match_name
 *
 * Description:
 *   Called for each record in the group file.  Returns "1" if the record
 *   matches the group name (passed as arg)
 *
 * Input Parameters:
 *   entry  - The parsed group file record
 *   arg    - A pointer to the group name to match
 *
 * Returned Value:
 *   < 0 :  An error has occurred.
 *   = 0 :  No entry name does not match.
 *   = 1 :  The entry name matches
 *
 ****************************************************************************/

static int grp_match_name(FAR const struct group *entry, uintptr_t arg)
{
  FAR const char *gname = (FAR const char *)arg;
  return strcmp(entry->gr_name, gname) == 0 ? 1 : 0;
}

/****************************************************************************
 * Name: grp_match_gid
 *
 * Description:
 *   Called for each record in the group file.  Returns "1" if the record
 *   matches the group ID (passed as arg)
 *
 * Input Parameters:
 *   entry  - The parsed group file record
 *   arg    - The group ID to match
 *
 * Returned Value:
 *   < 0 :  An error has occurred.
 *   = 0 :  No entry name does not match.
 *   = 1 :  The entry name matches
 *
 ****************************************************************************/

static int grp_match_gid(FAR const struct group *entry, uintptr_t arg)
{
  int match_gid = (int)arg;
  return match_gid == entry->gr_gid ? 1 : 0;
}

/****************************************************************************
 * Name: grp_match_user
 *
 * Description:
 *   Called for each record in the group file.  If the user (passed via the
 *   context in arg) is a member of the group, its group ID is appended to
 *   the caller's output array.  Always returns 0 so that every record in
 *   the file is visited and the total number of matching groups is counted.
 *
 * Input Parameters:
 *   entry  - The parsed group file record
 *   arg    - A pointer to the grp_user_s search context
 *
 * Returned Value:
 *   = 0 :  Always, so that iteration continues to the end of the file.
 *
 ****************************************************************************/

static int grp_match_user(FAR const struct group *entry, uintptr_t arg)
{
  FAR struct grp_user_s *ctx = (FAR struct grp_user_s *)arg;
  FAR char **member;

  /* The primary group has already been accounted for by the caller.  Skip
   * it here to avoid reporting it twice.
   */

  if (entry->gr_gid == ctx->group)
    {
      return 0;
    }

  /* Check whether the user is listed as a member of this group. */

  for (member = entry->gr_mem; *member != NULL; member++)
    {
      if (strcmp(*member, ctx->user) == 0)
        {
          /* Append the group ID if there is room in the output array.  Keep
           * counting regardless so the caller learns the required size.
           */

          if (ctx->count < ctx->maxgroups)
            {
              ctx->grouplist[ctx->count] = entry->gr_gid;
            }

          ctx->count++;
          break;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: grp_foreach
 *
 * Description:
 *   Visit each record in group file.
 *
 * Input Parameters:
 *   match  - The match function to call on each record
 *   arg    - Argument passed to the match function
 *   entry  - Location to return the parsed group file entry
 *   buffer - I/O buffer used to access the group file
 *   buflen - The size of the I/O buffer in bytes
 *
 * Returned Value:
 *   < 0 :  An error has occurred.
 *   = 0 :  No entry with this name was found.
 *   = 1 :  The entry with this name was found.
 *
 ****************************************************************************/

static int grp_foreach(grp_foreach_match_t match, uintptr_t arg,
                       FAR struct group *entry, FAR char *buffer,
                       size_t buflen)
{
  FAR FILE *stream;
  FAR char *ptr;
  FAR char *line;
  FAR char *save;
  size_t linelen;
  unsigned int nmembers;
  int ret;

  stream = fopen(CONFIG_LIBC_GROUP_FILEPATH, "r");
  if (stream == NULL)
    {
      int errcode = get_errno();
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

  /* Read the password file line by line until the record with the matching
   * username is found, or until the end of the file is reached.
   *
   * The format of the password file is:
   *
   *   user:x:uid:gid:home
   *
   * Where:
   *   user:  User name
   *   x:     Encrypted password
   *   uid:   User ID
   *   gid:   Group ID
   *   home:  Login directory
   */

  DEBUGASSERT(buflen > MEMBER_SIZE);  /* Buffer must also be aligned */
  line    = buffer + MEMBER_SIZE;
  linelen = buflen - MEMBER_SIZE;

  while (fgets(line, linelen, stream) != NULL)
    {
      ptr            = line;
      entry->gr_name = ptr;

      /* Skip to the end of the name and properly terminate it.  The name
       * must be terminated with the field delimiter ':'.
       */

      for (; *ptr != '\n' && *ptr != '\0' && *ptr != ':'; ptr++)
        {
        }

      if (*ptr == '\n' || *ptr == '\0')
        {
          /* Bad line format? */

          continue;
        }

      *ptr++           = '\0';
      entry->gr_passwd = ptr;

      /* Skip to the end of the password and properly terminate it.  The
       * password must be terminated with the field delimiter ':'.
       */

      for (; *ptr != '\n' && *ptr != '\0' && *ptr != ':'; ptr++)
        {
        }

      if (*ptr == '\n' || *ptr == '\0')
        {
          /* Bad line format? */

          continue;
        }

      *ptr++  = '\0';
      save    = ptr;

      /* Skip to the end of the group ID and properly terminate it.  The
       * group ID must be terminated with the field delimiter ':'.
       */

      for (; *ptr != '\n' && *ptr != '\0' && *ptr != ':'; ptr++)
        {
        }

      if (*ptr == '\n' || *ptr == '\0')
        {
          /* Bad line format? */

          continue;
        }

      *ptr++ = '\0';
      entry->gr_gid = (gid_t)atoi(save);

      /* This is followed by a variable number of user names.  The ':'
       * delimited will be followed by '\n' or '\0' if there are no users
       * in the group
       */

      nmembers      = 0;
      entry->gr_mem = (FAR char **)buffer;

      if (*ptr != '\n' && *ptr != '\0')
        {
          for (; ; )
            {
              /* Add the next user name */

              entry->gr_mem[nmembers] = ptr;
              nmembers++;
              if (nmembers >= CONFIG_LIBC_GROUP_NUSERS)
                {
                  break;
                }

              /* Skip to the end of the user name and properly terminate it.
               * The user name must be terminated with either (1) ','
               * meaning that another user name is present, or (2) '\n' or
               * '\0' meaning that we have reached the end of the line and
               * there are no further user names.
               */

              for (; *ptr != '\n' && *ptr != '\0' && *ptr != ','; ptr++)
                {
                }

              if (*ptr == '\n' || *ptr == '\0')
                {
                  *ptr = '\0';
                  break;
                }

              *ptr++ = '\0';
            }
        }

      /* The list terminates with a NULL pointer */

      entry->gr_mem[nmembers] = NULL;

      /* Check for a match */

      ret = match(entry, arg);
      if (ret != 0)
        {
          /* We either have the match or an error occurred. */

          fclose(stream);
          return ret;
        }
    }

  fclose(stream);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: grp_findby_name
 *
 * Description:
 *   Find group file entry using the group name.
 *
 * Input Parameters:
 *   gname  - The group name
 *   entry  - Location to return the parsed group file entry
 *   buffer - I/O buffer used to access the group file
 *   buflen - The size of the I/O buffer in bytes
 *
 * Returned Value:
 *   < 0 :  An error has occurred.
 *   = 0 :  No entry with this name was found.
 *   = 1 :  The entry with this name was found.
 *
 ****************************************************************************/

int grp_findby_name(FAR const char *gname, FAR struct group *entry,
                    FAR char *buffer, size_t buflen)
{
  return grp_foreach(grp_match_name, (uintptr_t)gname,
                     entry, buffer, buflen);
}

/****************************************************************************
 * Name: grp_findby_gid
 *
 * Description:
 *   Find group file entry using the group ID.
 *
 * Input Parameters:
 *   gid    - The group ID
 *   entry  - Location to return the parsed group file entry
 *   buffer - I/O buffer used to access the group file
 *   buflen - The size of the I/O buffer in bytes
 *
 * Returned Value:
 *   < 0 :  An error has occurred.
 *   = 0 :  No entry with this name was found.
 *   = 1 :  The entry with this name was found.
 *
 ****************************************************************************/

int grp_findby_gid(gid_t gid, FAR struct group *entry, FAR char *buffer,
                   size_t buflen)
{
  /* Verify that the GID is in the valid range of 0 through INT16_MAX.
   * OpenGroup.org does not specify a GID_MAX or GID_MIN.  Instead we use a
   * priori knowledge that gid_t is type int16_t.
   */

  if ((uint16_t)gid > INT16_MAX)
    {
      return -EINVAL;
    }

  return grp_foreach(grp_match_gid, (uintptr_t)gid, entry, buffer, buflen);
}

/****************************************************************************
 * Name: grp_findby_user
 *
 * Description:
 *   Scan the group file and collect the group IDs of all groups that have
 *   'user' as a member, excluding the primary group 'group'.
 *
 * Input Parameters:
 *   user      - The user name to search for
 *   group     - The primary group ID to exclude from the search
 *   grouplist - Location to return the matching group IDs
 *   maxgroups - The number of slots available in grouplist
 *   count     - On input, the number of group IDs already stored in
 *               grouplist.  On output, the total number of group IDs found
 *               (which may exceed maxgroups).
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int grp_findby_user(FAR const char *user, gid_t group,
                    FAR gid_t *grouplist, int maxgroups, FAR int *count)
{
  struct grp_user_s ctx;
  struct group entry;
  FAR char *buffer;
  int ret;

  buffer = lib_get_tempbuffer(GRPBUF_RESERVE_SIZE);
  if (buffer == NULL)
    {
      return -ENOMEM;
    }

  ctx.user      = user;
  ctx.group     = group;
  ctx.grouplist = grouplist;
  ctx.maxgroups = maxgroups;
  ctx.count     = *count;

  ret = grp_foreach(grp_match_user, (uintptr_t)&ctx, &entry, buffer,
                    GRPBUF_RESERVE_SIZE);

  *count = ctx.count;
  lib_put_tempbuffer(buffer);

  return ret < 0 ? ret : 0;
}
