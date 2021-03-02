/****************************************************************************
 * libs/libc/grp/lib_find_grpfile.c
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

#include "grp/lib_grp.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef CODE int (grp_foreach_match_t)(FAR const struct group *entry,
                                       uintptr_t arg);

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
