/****************************************************************************
 * libs/libc/locale/lib_gettext.c
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
#include <libintl.h>
#include <locale.h>
#include <nuttx/tls.h>
#include <strings.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "libc.h"

#ifdef CONFIG_LIBC_LOCALE_GETTEXT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MO_MAGIC 0x950412de

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mofile_s
{
  FAR struct mofile_s *next;
  char path[PATH_MAX];
  FAR void *map;
  size_t size;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR const char *g_catname[] =
{
  "LC_CTYPE",
  "LC_NUMERIC",
  "LC_TIME",
  "LC_COLLATE",
  "LC_MONETARY",
  "LC_MESSAGES",
  "LC_ALL",
};

static sem_t g_sem = SEM_INITIALIZER(1);
static FAR struct mofile_s *g_mofile;

#ifdef CONFIG_BUILD_KERNEL
static FAR char g_domain[NAME_MAX];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR char *gettextdomain(void)
{
  FAR char *domain;
#ifdef CONFIG_BUILD_KERNEL
  domain = g_domain;
#else
  FAR struct task_info_s *info;

  info = task_get_info();
  domain = info->ta_domain;
#endif

  return domain;
}

/* MO file format is documented here:
 * https://www.gnu.org/software/gettext/manual/gettext.html#MO-Files
 */

static FAR void *momap(FAR const char *path, FAR size_t *size)
{
  FAR uint32_t *map = MAP_FAILED;
  struct stat st;
  int fd;

  fd = open(path, O_RDONLY | O_CLOEXEC);
  if (fd < 0)
    {
      return map;
    }

  if (fstat(fd, &st) >= 0)
    {
      *size = st.st_size;
      map = mmap(NULL, *size, PROT_READ, MAP_SHARED, fd, 0);
      if (map[0] != MO_MAGIC && map[0] != __swap_uint32(MO_MAGIC))
        {
          munmap(map, *size);
          map = MAP_FAILED;
        }
    }

  close(fd);
  return map;
}

static inline uint32_t moswap(uint32_t x, int sw)
{
  return sw ? __swap_uint32(x) : x;
}

static FAR char *molookup(FAR char *p, size_t size, FAR const char *s)
{
  FAR uint32_t *mo = (FAR uint32_t *)p;
  int sw = mo[0] - MO_MAGIC;
  uint32_t n = moswap(mo[2], sw);
  uint32_t o = moswap(mo[3], sw);
  uint32_t t = moswap(mo[4], sw);
  uint32_t b = 0;

  if (n > size / 8 || o > size -  8 * n ||
      t > size - 8 * n || (o | t) % 4 != 0)
    {
      return NULL;
    }

  o /= 4;
  t /= 4;

  for (; ; )
    {
      uint32_t ol = moswap(mo[o + 2 * (b + n / 2)], sw);
      uint32_t os = moswap(mo[o + 2 * (b + n / 2) + 1], sw);
      int sign;

      if (ol >= size || os >= size - ol || p[os + ol])
        {
          return NULL;
        }

      sign = strcmp(s, p + os);
      if (sign == 0)
        {
          uint32_t tl = moswap(mo[t + 2 * (b + n / 2)], sw);
          uint32_t ts = moswap(mo[t + 2 * (b + n / 2) + 1], sw);

          if (tl >= size || ts >= size - tl || p[ts + tl])
            {
              return NULL;
            }

          return p + ts;
        }
      else if (n == 1)
        {
          return NULL;
        }
      else if (sign < 0)
        {
          n /= 2;
        }
      else
        {
          b += n / 2;
          n -= n / 2;
        }
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dcngettext
 *
 * Description:
 *   The dcngettext function searches the message catalogs identified by
 *   domainname and category for a string which is equal to msgid1.
 *
 *   The msgid1 parameter must contain the singular form of the string to
 *   be converted. It is also used as the key for the search in the catalog.
 *   The msgid2 parameter is the plural form. The parameter n is used to
 *   determine the plural form.
 *
 * Returned Value:
 *   If there is such a string available it is returned. Otherwise msgid1 is
 *   returned if n == 1, otherwise msgid2.
 *
 *   Please note that although the return value is char * the returned string
 *   must not be changed. This broken type results from the history of the
 *   function and does not reflect the way the function should be used.
 *
 ****************************************************************************/

FAR char *gettext(FAR const char *msgid)
{
  return dgettext(NULL, msgid);
}

FAR char *dgettext(FAR const char *domainname,
                   FAR const char *msgid)
{
  return dcgettext(domainname, msgid, LC_MESSAGES);
}

FAR char *dcgettext(FAR const char *domainname,
                    FAR const char *msgid,
                    int category)
{
  return dcngettext(domainname, msgid, NULL, 1, category);
}

FAR char *ngettext(FAR const char *msgid1,
                   FAR const char *msgid2,
                   unsigned long int n)
{
  return dngettext(NULL, msgid1, msgid2, n);
}

FAR char *dngettext(FAR const char *domainname,
                    FAR const char *msgid1,
                    FAR const char *msgid2,
                    unsigned long int n)
{
  return dcngettext(domainname, msgid1, msgid2, n, LC_MESSAGES);
}

FAR char *dcngettext(FAR const char *domainname,
                     FAR const char *msgid1,
                     FAR const char *msgid2,
                     unsigned long int n,
                     int category)
{
  FAR struct mofile_s *mofile;
  FAR const char *lang;
  char path[PATH_MAX];
  FAR char *notrans;
  FAR char *trans;

  notrans = (FAR char *)(n == 1 ? msgid1 : msgid2);

  if (msgid1 == NULL)
    {
      return notrans;
    }

  if (category < 0 || category >= LC_ALL)
    {
      return notrans;
    }

  if (domainname == NULL)
    {
      domainname = textdomain(NULL);
    }

  lang = getenv("LANG");
  if (lang == NULL)
    {
      lang = "C";
    }

  snprintf(path, PATH_MAX,
           CONFIG_LIBC_LOCALE_PATH"/%s/%s/%s.mo",
           lang, g_catname[category], domainname);

  while (_SEM_WAIT(&g_sem) < 0);

  for (mofile = g_mofile; mofile; mofile = mofile->next)
    {
      if (strcmp(mofile->path, path) == 0)
        {
          break;
        }
    }

  if (mofile == NULL)
    {
      mofile = lib_malloc(sizeof(*mofile));
      if (mofile == NULL)
        {
          _SEM_POST(&g_sem);
          return notrans;
        }

      strlcpy(mofile->path, path, PATH_MAX);
      mofile->map = momap(path, &mofile->size);
      if (mofile->map == MAP_FAILED)
        {
          _SEM_POST(&g_sem);
          lib_free(mofile);
          return notrans;
        }

      mofile->next = g_mofile;
      g_mofile = mofile;
    }

  _SEM_POST(&g_sem); /* Leave look before search */

  trans = molookup(mofile->map, mofile->size, msgid1);
  return trans ? trans : notrans;
}

/****************************************************************************
 * Name: textdomain
 *
 * Description:
 *   The textdomain function sets the default domain, which is used in all
 *   future gettext calls, to domainname. Please note that dgettext and
 *   dcgettext calls are not influenced if the domainname parameter of these
 *   functions is not the null pointer.
 *
 *   Before the first call to textdomain the default domain is messages. This
 *   is the name specified in the specification of the gettext API. This name
 *   is as good as any other name. No program should ever really use a domain
 *   with this name since this can only lead to problems.
 *
 *   If the domainname parameter is the empty string the default domain is
 *   reset to its initial value, the domain with the name messages. This
 *   possibility is questionable to use since the domain messages really
 *   never should be used.
 *
 * Returned Value:
 *   The function returns the value which is from now on taken as the default
 *   domain. If the system went out of memory the returned value is NULL and
 *   the global variable errno is set to ENOMEM. Despite the return value
 *   type being char * the return string must not be changed. It is allocated
 *   internally by the textdomain function.
 *
 *   If the domainname parameter is the null pointer no new default domain is
 *   set. Instead the currently selected default domain is returned.
 *
 ****************************************************************************/

FAR char *textdomain(FAR const char *domainname)
{
  FAR char *domain;
  size_t domainlen;

  domain = gettextdomain();
  if (domainname == NULL)
    {
      return *domain ? domain : "messages";
    }

  domainlen = strlen(domainname);
  if (domainlen > NAME_MAX)
    {
      set_errno(EINVAL);
      return NULL;
    }

  memcpy(domain, domainname, domainlen + 1);
  return domain;
}

/****************************************************************************
 * Name: bindtextdomain
 *
 * Description:
 *   The bindtextdomain function can be used to specify the directory which
 *   contains the message catalogs for domain domainname for the different
 *   languages. To be correct, this is the directory where the hierarchy of
 *   directories is expected. Details are explained below.
 *
 *   For the programmer it is important to note that the translations which
 *   come with the program have to be placed in a directory hierarchy
 *   starting at, say, /foo/bar. Then the program should make a
 *   bindtextdomain call to bind the domain for the current program to this
 *   directory. So it is made sure the catalogs are found. A correctly
 *   running program does not depend on the user setting an environment
 *   variable.
 *
 *   The bindtextdomain function can be used several times and if the
 *   domainname argument is different the previously bound domains will not
 *   be overwritten.
 *
 *   If the program which wish to use bindtextdomain at some point of time
 *   use the chdir function to change the current working directory it is
 *   important that the dirname strings ought to be an absolute pathname.
 *   Otherwise the addressed directory might vary with the time.
 *
 * Returned Value:
 *   If the dirname parameter is the null pointer bindtextdomain returns the
 *   currently selected directory for the domain with the name domainname.
 *
 *   The bindtextdomain function returns a pointer to a string containing the
 *   name of the selected directory name. The string is allocated internally
 *   in the function and must not be changed by the user. If the system went
 *   out of core during the execution of bindtextdomain the return value is
 *   NULL and the global variable errno is set accordingly.
 *
 ****************************************************************************/

FAR char *bindtextdomain(FAR const char *domainname,
                         FAR const char *dirname)
{
  if (domainname == NULL || dirname)
    {
      set_errno(EINVAL); /* Only support the default path */
    }

  return NULL;
}

/****************************************************************************
 * Name: bind_textdomain_codeset
 *
 * Description:
 *   The bind_textdomain_codeset function can be used to specify the output
 *   character set for message catalogs for domain domainname. The codeset
 *   argument must be a valid codeset name which can be used for the
 *   iconv_open function, or a null pointer.
 *
 *   The bind_textdomain_codeset function can be used several times. If used
 *   multiple times with the same domainname argument, the later call
 *   overrides the settings made by the earlier one.
 *
 * Returned Value:
 *   If the codeset parameter is the null pointer, bind_textdomain_codeset
 *   returns the currently selected codeset for the domain with the name
 *   domainname. It returns NULL if no codeset has yet been selected.
 *
 *   The bind_textdomain_codeset function returns a pointer to a string
 *   containing the name of the selected codeset. The string is allocated
 *   internally in the function and must not be changed by the user. If the
 *   system went out of core during the execution of bind_textdomain_codeset,
 *   the return value is NULL and the global variable errno is set
 *   accordingly.
 *
 ****************************************************************************/

FAR char *bind_textdomain_codeset(FAR const char *domainname,
                                  FAR const char *codeset)
{
  if (codeset && strcasecmp(codeset, "UTF-8"))
    {
      set_errno(EINVAL); /* Only support utf8 */
    }

  return NULL;
}

#endif
