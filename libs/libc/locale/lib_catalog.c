/****************************************************************************
 * libs/libc/locale/lib_catalog.c
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

#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <nl_types.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#ifdef CONFIG_LIBC_LOCALE_CATALOG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAT_MAGIC 0xff88ff89

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

begin_packed_struct
struct cathdr_s
{
  uint32_t magic;
  uint32_t nsets;
  uint32_t size;
  uint32_t msg_offset;
  uint32_t txt_offset;
} end_packed_struct;

begin_packed_struct
struct catset_s
{
  uint32_t setno;
  uint32_t nmsgs;
  uint32_t index;
} end_packed_struct;

begin_packed_struct
struct catmsg_s
{
  uint32_t msgno;
  uint32_t msglen;
  uint32_t offset;
} end_packed_struct;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static nl_catd catmap(FAR const char *path)
{
  FAR const struct cathdr_s *hdr;
  nl_catd catd = MAP_FAILED;
  struct stat st;
  int fd;

  fd = open(path, O_RDONLY | O_CLOEXEC);
  if (fd < 0)
    {
      return catd;
    }

  if (fstat(fd, &st) >= 0)
    {
      catd = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
      if (catd != MAP_FAILED)
        {
          hdr = (FAR const struct cathdr_s *)catd;
          if (CAT_MAGIC != be32toh(hdr->magic) ||
              st.st_size != sizeof(*hdr) + be32toh(hdr->size))
            {
              munmap(catd, st.st_size);
              catd = MAP_FAILED;
              set_errno(ENOENT);
            }
        }
    }

  close(fd);
  return catd;
}

static int setcmp(FAR const void *a, FAR const void *b)
{
  FAR const int *set_id = a;
  FAR const struct catset_s *set = b;

  return *set_id - be32toh(set->setno);
}

static int msgcmp(FAR const void *a, FAR const void *b)
{
  FAR const int *msg_id = a;
  FAR const struct catmsg_s *msg = b;

  return *msg_id - be32toh(msg->msgno);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: catopen
 *
 * Description:
 *   The catopen() function shall open a message catalog and return a
 *   message catalog descriptor. The name argument specifies the name of
 *   the message catalog to be opened. If name contains a '/', then name
 *   specifies a pathname for the message catalog. Otherwise, the environment
 *   variable NLSPATH is used with name substituted for the %N conversion
 *   specification (see XBD Environment Variables); if NLSPATH exists in the
 *   environment when the process starts, then if the process has appropriate
 *   privileges, the behavior of catopen() is undefined. If NLSPATH does not
 *   exist in the environment, or if a message catalog cannot be found in any
 *   of the components specified by NLSPATH, then an implementation-defined
 *   default path shall be used. This default may be affected by the setting
 *   of LC_MESSAGES if the value of oflag is NL_CAT_LOCALE, or the LANG
 *   environment variable if oflag is 0.
 *
 *   A message catalog descriptor shall remain valid in a process until that
 *   process closes it, or a successful call to one of the exec functions.
 *   A change in the setting of the LC_MESSAGES category may invalidate
 *   existing open catalogs.
 *
 *   If a file descriptor is used to implement message catalog descriptors,
 *   the FD_CLOEXEC flag shall be set; see <fcntl.h>.
 *
 *   If the value of the oflag argument is 0, the LANG environment variable
 *   is used to locate the catalog without regard to the LC_MESSAGES
 *   category. If the oflag argument is NL_CAT_LOCALE, the LC_MESSAGES
 *   category is used to locate the message catalog (see XBD
 *   Internationalization Variables ).
 *
 * Returned Value:
 *   Upon successful completion, catopen() shall return a message catalog
 *   descriptor for use on subsequent calls to catgets() and catclose().
 *   Otherwise, catopen() shall return (nl_catd) -1 and set errno to
 *   indicate the error.
 *
 ****************************************************************************/

nl_catd catopen(FAR const char *name, int oflag)
{
  FAR const char *path;
  FAR const char *lang;
  FAR const char *p;
  FAR const char *z;

  if (strchr(name, '/'))
    {
      return catmap(name);
    }

  path = getenv("NLSPATH");
  if (path == NULL)
    {
      path = CONFIG_LIBC_LOCALE_PATH;
    }

  lang = oflag ? NULL : getenv("LANG");
  if (lang == NULL)
    {
      lang = "";
    }

  for (p = path; *p; p = z)
    {
      char buf[PATH_MAX];
      nl_catd catd;
      size_t i;

      z = strchr(p, ':');
      if (z == NULL)
        {
          z = p + strlen(p);
        }

      for (i = 0; p < z; p++)
        {
          FAR const char *v;
          size_t l;

          if (*p == '%')
            {
              switch (*++p)
                {
                  case 'N':
                    v = name;
                    l = strlen(v);
                    break;

                  case 'L':
                    v = lang;
                    l = strlen(v);
                    break;

                  case 'l':
                    v = lang;
                    l = strcspn(v, "_.@");
                    break;

                  case 't':
                    v = strchr(lang, '_');
                    if (v == NULL)
                      {
                        v = "\0";
                      }

                    l = strcspn(++v, ".@");
                    break;

                  case 'c':
                    v = "UTF-8";
                    l = 5;
                    break;

                  case '%':
                    v = "%";
                    l = 1;
                    break;

                  default:
                    v = NULL;
                }
            }
          else
            {
              v = p;
              l = 1;
            }

          if (v == NULL || i + l >= sizeof(buf))
            {
              break;
            }

          memcpy(buf + i, v, l);
          i += l;
        }

        if (*z)
          {
            z++;
          }

        if (*p != ':' && *p != '\0')
          {
            continue;
          }

        /* Leading : or :: in NLSPATH is same as %N */

        buf[i] = 0;
        catd = catmap(i ? buf : name);
        if (catd != MAP_FAILED)
          {
            return catd;
          }
      }

  set_errno(ENOENT);
  return MAP_FAILED;
}

/****************************************************************************
 * Name: catgets
 *
 * Description:
 *   The catgets() function shall attempt to read message msg_id, in set
 *   set_id, from the message catalog identified by catd. The catd argument
 *   is a message catalog descriptor returned from an earlier call to
 *   catopen(). The results are undefined if catd is not a value returned
 *   by catopen() for a message catalog still open in the process. The s
 *   argument points to a default message string which shall be returned by
 *   catgets() if it cannot retrieve the identified message.
 *
 *   The catgets() function need not be thread-safe.
 *
 * Returned Value:
 *   If the identified message is retrieved successfully, catgets() shall
 *   return a pointer to an internal buffer area containing the null-
 *   terminated message string. If the call is unsuccessful for any reason,
 *   s shall be returned and errno shall be set to indicate the error.
 *
 ****************************************************************************/

FAR char *catgets(nl_catd catd, int set_id, int msg_id, FAR const char *s)
{
  FAR const struct cathdr_s *hdr = (FAR const struct cathdr_s *)catd;
  FAR const struct catset_s *set = (FAR const struct catset_s *)(hdr + 1);
  FAR const struct catmsg_s *msg = (FAR const struct catmsg_s *)
                 ((FAR const char *)(hdr + 1) + be32toh(hdr->msg_offset));
  FAR const char *string =
                 ((FAR const char *)(hdr + 1) + be32toh(hdr->txt_offset));

  set = bsearch(&set_id, set, be32toh(hdr->nsets), sizeof(*set), setcmp);
  if (set == NULL)
    {
      set_errno(ENOMSG);
      return (FAR char *)s;
    }

  msg += be32toh(set->index);
  msg = bsearch(&msg_id, msg, be32toh(set->nmsgs), sizeof(*msg), msgcmp);
  if (msg == NULL)
    {
      set_errno(ENOMSG);
      return (FAR char *)s;
    }

  return (FAR char *)(string + be32toh(msg->offset));
}

/****************************************************************************
 * Name: catclose
 *
 * Description:
 *   The catclose() function shall close the message catalog identified by
 *   catd. If a file descriptor is used to implement the type nl_catd, that
 *   file descriptor shall be closed.
 *
 * Returned Value:
 *   Upon successful completion, catclose() shall return 0; otherwise,
 *   -1 shall be returned, and errno set to indicate the error.
 *
 ****************************************************************************/

int catclose(nl_catd catd)
{
  FAR const struct cathdr_s *hdr = (FAR const struct cathdr_s *)catd;

  return munmap(catd, sizeof(*hdr) + be32toh(hdr->size));
}

#endif
