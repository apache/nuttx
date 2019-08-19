/****************************************************************************
 * libs/libc/grp/lib_find_grpfile.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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
      int errcode = errno;
      DEBUGASSERT(errno > 0);
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
  return grp_foreach(grp_match_name, (uintptr_t)gname, entry, buffer, buflen);
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
