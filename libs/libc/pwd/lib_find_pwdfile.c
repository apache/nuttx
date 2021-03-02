/****************************************************************************
 * libs/libc/pwd/lib_find_pwdfile.c
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
#include <pwd.h>
#include <assert.h>

#include "pwd/lib_pwd.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef CODE int (pwd_foreach_match_t)(FAR const struct passwd *entry,
                                       uintptr_t arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwd_match_name
 *
 * Description:
 *   Called for each record in the passwd file.  Returns "1" if the record
 *   matches the user name (passed as arg)
 *
 * Input Parameters:
 *   entry  - The parsed passwd file record
 *   arg    - A pointer to the user name to match
 *
 * Returned Value:
 *   < 0 :  An error has occurred.
 *   = 0 :  No entry name does not match.
 *   = 1 :  The entry name matches
 *
 ****************************************************************************/

static int pwd_match_name(FAR const struct passwd *entry, uintptr_t arg)
{
  FAR const char *uname = (FAR const char *)arg;
  return strcmp(entry->pw_name, uname) == 0 ? 1 : 0;
}

/****************************************************************************
 * Name: pwd_match_uid
 *
 * Description:
 *   Called for each record in the passwd file.  Returns "1" if the record
 *   matches the user ID (passed as arg)
 *
 * Input Parameters:
 *   entry  - The parsed passwd file record
 *   arg    - The user ID to match
 *
 * Returned Value:
 *   < 0 :  An error has occurred.
 *   = 0 :  No entry name does not match.
 *   = 1 :  The entry name matches
 *
 ****************************************************************************/

static int pwd_match_uid(FAR const struct passwd *entry, uintptr_t arg)
{
  int match_uid = (int)arg;
  return match_uid == entry->pw_uid ? 1 : 0;
}

/****************************************************************************
 * Name: pwd_foreach
 *
 * Description:
 *   Visit each record in passwd file.
 *
 * Input Parameters:
 *   match  - The match function to call on each record
 *   arg    - Argument passed to the match function
 *   entry  - Location to return the parsed passwd file entry
 *   buffer - I/O buffer used to access the passwd file
 *   buflen - The size of the I/O buffer in bytes
 *
 * Returned Value:
 *   < 0 :  An error has occurred.
 *   = 0 :  No entry with this name was found.
 *   = 1 :  The entry with this name was found.
 *
 ****************************************************************************/

static int pwd_foreach(pwd_foreach_match_t match, uintptr_t arg,
                       FAR struct passwd *entry, FAR char *buffer,
                       size_t buflen)
{
  FAR FILE *stream;
  FAR char *ptr;
  FAR char *save;
  int ret;

  stream = fopen(CONFIG_LIBC_PASSWD_FILEPATH, "r");
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
   *   user:x:uid:uid:home
   *
   * Where:
   *   user:  User name
   *   x:     Encrypted password
   *   uid:   User ID
   *   uid:   Group ID
   *   home:  Login directory
   */

  while (fgets(buffer, buflen, stream) != NULL)
    {
      ptr            = buffer;
      entry->pw_name = ptr;

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

      *ptr++ = '\0';

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

      *ptr++ = '\0';
      save   = ptr;

      /* Skip to the end of the user ID and properly terminate it.  The
       * user ID must be terminated with the field delimiter ':'.
       */

      for (; *ptr != '\n' && *ptr != '\0' && *ptr != ':'; ptr++)
        {
        }

      if (*ptr == '\n' || *ptr == '\0')
        {
          /* Bad line format? */

          continue;
        }

      *ptr++        = '\0';
      entry->pw_uid = (uid_t)atoi(save);
      save          = ptr;

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

      *ptr++        = '\0';
      entry->pw_gid = (gid_t)atoi(save);
      entry->pw_dir = ptr;

      /* Skip to the end of the home directory and properly terminate it.
       * The home directory must be the last thing on the line.
       */

      for (; *ptr != '\n' && *ptr != '\0' /* && *ptr != ':' */ ; ptr++)
        {
        }

      *ptr++ = '\0';
      entry->pw_shell = ROOT_SHELL;

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
 * Name: pwd_findby_name
 *
 * Description:
 *   Find passwd file entry using the user name.
 *
 * Input Parameters:
 *   uname  - The user name
 *   entry  - Location to return the parsed passwd file entry
 *   buffer - I/O buffer used to access the passwd file
 *   buflen - The size of the I/O buffer in bytes
 *
 * Returned Value:
 *   < 0 :  An error has occurred.
 *   = 0 :  No entry with this name was found.
 *   = 1 :  The entry with this name was found.
 *
 ****************************************************************************/

int pwd_findby_name(FAR const char *uname, FAR struct passwd *entry,
                    FAR char *buffer, size_t buflen)
{
  return pwd_foreach(pwd_match_name, (uintptr_t)uname,
                     entry, buffer, buflen);
}

/****************************************************************************
 * Name: pwd_findby_uid
 *
 * Description:
 *   Find passwd file entry using the user ID.
 *
 * Input Parameters:
 *   uid    - The user ID
 *   entry  - Location to return the parsed passwd file entry
 *   buffer - I/O buffer used to access the passwd file
 *   buflen - The size of the I/O buffer in bytes
 *
 * Returned Value:
 *   < 0 :  An error has occurred.
 *   = 0 :  No entry with this name was found.
 *   = 1 :  The entry with this name was found.
 *
 ****************************************************************************/

int pwd_findby_uid(uid_t uid, FAR struct passwd *entry, FAR char *buffer,
                   size_t buflen)
{
  /* Verify that the UID is in the valid range of 0 through INT16_MAX.
   * OpenGroup.org does not specify a UID_MAX or UID_MIN.  Instead we use a
   * priori knowledge that uid_t is type int16_t.
   */

  if ((uint16_t)uid > INT16_MAX)
    {
      return -EINVAL;
    }

  return pwd_foreach(pwd_match_uid, (uintptr_t)uid, entry, buffer, buflen);
}
