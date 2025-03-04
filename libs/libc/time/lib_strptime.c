/****************************************************************************
 * libs/libc/time/lib_strptime.c
 *
 * musl as a whole is licensed under the following standard MIT license:
 *
 * Copyright © 2005-2020 Rich Felker, et al.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <langinfo.h>
#include <time.h>
#include <ctype.h>
#include <stddef.h>
#include <string.h>
#include <strings.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  strptime
 *
 * Description:
 *    Parses a string representing date and time according to a specified
 *    format and stores the result in a struct tm structure.
 *
 * Input Parameters:
 *    s    - Pointer to the input string to be parsed.
 *    f    - Pointer to the format string specifying how to parse the input
 *           string.
 *    tm   - Pointer to a struct tm structure where the parsed date and time
 *           will be stored.
 *
 * Returned Value:
 *    Returns a pointer to the character in the input string s immediately
 *    following the last character processed.
 *
 ****************************************************************************/

FAR char *strptime(FAR const char *restrict s, FAR const char *restrict f,
                   FAR struct tm *restrict tm)
{
  int i;
  int w;
  int neg;
  int adj;
  int min;
  int range;
  FAR int *dest;
  int dummy;
  int want_century = 0;
  int century = 0;
  int relyear = 0;
  #ifdef CONFIG_LIBC_LOCALE
  FAR const char *ex;
#endif
#if defined(CONFIG_LIBC_LOCALE) || defined(CONFIG_LIBC_LOCALTIME)
  size_t len;
  #endif

  if (!s || !f || !tm)
    {
      return NULL;
    }

  while (*f)
    {
      if (*f != '%')
        {
          /* Handle literal characters in the format string */

          if (isspace(*f))
            {
              /* Skip whitespace in the input string */

              for (; *s && isspace(*s); s++);
            }
          else if (*s != *f)
            {
              /* Mismatch between format string and input string */

              return 0;
            }
          else
            {
              s++;
            }

          f++;
          continue;
        }

      f++;

      /* Handle optional width specifier */

      if (*f == '+')
        {
            f++;
        }

      if (isdigit(*f))
        {
          char *new_f;
          w = strtoul(f, &new_f, 10);
          f = new_f;
        }
      else
        {
          w = -1;
        }

      adj = 0;

      /* Handle format specifiers */

      switch (*f++)
        {
          case 'C':
            dest = &century;
            if (w < 0) w = 2;
            want_century |= 2;
            goto numeric_digits;

          case 'd': case 'e':
            dest = &tm->tm_mday;
            min = 1;
            range = 31;
            if (!isdigit(*s))
              {
                return NULL;
              }

            goto numeric_range;

          case 'D':

            /* Date in mm/dd/yy format */

            s = strptime(s, "%m/%d/%y", tm);
            if (!s)
              {
                return NULL;
              }
            break;

          case 'F':

            /* Date in yyyy-mm-dd format with width-limitation handling */

            i = 0;
            char tmp[20];
            if (*s == '-' || *s == '+')
              {
                tmp[i++] = *s++;
              }

            while (*s == '0' && isdigit(s[1]))
              {
                s++;
              }

            for (; *s && i < (size_t)w && i + 1 < sizeof tmp; i++)
              {
                tmp[i] = *s++;
              }

            tmp[i] = 0;
            char *p = strptime(tmp, "%12Y-%m-%d", tm);
            if (!p)
              {
                return NULL;
              }

            s -= tmp + i - p;
            break;

          case 'H':
            dest = &tm->tm_hour;
            min = 0;
            range = 24;
            goto numeric_range;

          case 'I':
            dest = &tm->tm_hour;
            min = 1;
            range = 12;
            goto numeric_range;

          case 'j':
            dest = &tm->tm_yday;
            min = 1;
            range = 366;
            adj = 1;
            goto numeric_range;

          case 'm':
            dest = &tm->tm_mon;
            min = 1;
            range = 12;
            adj = 1;
            goto numeric_range;

          case 'M':
            dest = &tm->tm_min;
            min = 0;
            range = 60;
            goto numeric_range;

          case 'n': case 't':

            /* Skip whitespace */

            for (; *s && isspace(*s); s++);
            break;

          case 'R':

            /* Time in HH:MM format */

            s = strptime(s, "%H:%M", tm);
            if (!s)
              {
                return NULL;
              }

            break;

          case 's':

            /* Seconds since epoch (effect on tm is unspecified) */

            if (*s == '-')
              {
                s++;
              }

            if (!isdigit(*s))
              {
                return NULL;
              }

            while (isdigit(*s))
              {
                s++;
              }

            break;

          case 'S':
            dest = &tm->tm_sec;
            min = 0;
            range = 61;
            goto numeric_range;

          case 'T':

            /* Time in HH:MM:SS format */

            s = strptime(s, "%H:%M:%S", tm);
            if (!s)
              {
                return NULL;
              }

            break;

          case 'U': case 'W':

            /* These specifiers are ignored */

            dest = &dummy;
            min = 0;
            range = 54;
            goto numeric_range;

          case 'V':
            dest = &dummy;
            min = 1;
            range = 53;
            goto numeric_range;

          case 'g':
            dest = &dummy;
            w = 2;
           goto numeric_digits;

          case 'G':
            dest = &dummy;
            if (w < 0)
              {
                w = 4;
              }

            goto numeric_digits;

          case 'u':
            dest = &tm->tm_wday;
            min = 1;
            range = 7;
            goto numeric_range;

          case 'w':
            dest = &tm->tm_wday;
            min = 0;
            range = 7;
            goto numeric_range;

          case 'y':
            dest = &relyear;
            w = 2;
            want_century |= 1;
            goto numeric_digits;

          case 'Y':
            dest = &tm->tm_year;
            if (w < 0)
              {
                w = 4;
              }

            adj = 1900;
            want_century = 0;
            goto numeric_digits;

          case 'z':

            /* Timezone offset from UTC */

            if (*s == '+' || *s == '-')
              {
                neg = (*s == '-');
                s++;
              }

            else
              {
                return NULL;
              }

            for (i = 0; i < 4; i++)
              {
                if (!isdigit(s[1 + i]))
                  {
                    return 0;
                  }
              }

            tm->tm_gmtoff = (s[1] - '0') * 36000 + (s[2] - '0') * 3600
                        + (s[3] - '0') * 600 + (s[4] - '0') * 60;
            if (neg)
              {
                tm->tm_gmtoff = -tm->tm_gmtoff;
              }

            s += 5;
            break;

#ifdef CONFIG_LIBC_LOCALTIME
          case 'Z':

            /* Timezone abbreviation */

            if (!strncmp(s, tzname[0], len = strlen(tzname[0])))
              {
                tm->tm_isdst = 0;
                s += len;
              }
            else if (!strncmp(s, tzname[1], len = strlen(tzname[1])))
              {
                tm->tm_isdst = 1;
                s += len;
              }
            else
              {
                /* Skip unknown timezone abbreviations */

                while ((*s | 32) - 'a' <= 'z' - 'a')
                  {
                    s++;
                  }
              }

            break;
#endif

          case '%':

            /* Literal '%' character */

            if (*s++ != '%')
              {
                return NULL;
              }
            break;

          default:
            return NULL;

          numeric_range:

            /* Process numeric values with a range check */

            if (!isdigit(*s))
              {
                return NULL;
              }

            *dest = 0;
            for (i = 1; i <= min + range && isdigit(*s); i *= 10)
              {
                *dest = *dest * 10 + *s++ - '0';
              }

            if (*dest < min || *dest > min + range)
              {
                return NULL;
              }

            *dest -= adj;
            goto update;

          numeric_digits:

            /* Process numeric values with a fixed width */

            neg = 0;
            if (*s == '+')
              {
                s++;
              }
            else if (*s == '-')
              {
                neg = 1;
                s++;
              }

            if (!isdigit(*s))
              {
                return NULL;
              }

            for (*dest = i = 0; i < w && isdigit(*s); i++)
              {
                *dest = *dest * 10 + *s++ - '0';
              }

            if (neg)
              {
                *dest = -*dest;
              }

            *dest -= adj;
            if (*dest < 0 || (*dest > 99 && w == 2))
              {
                return NULL;
              }

            goto update;

#ifdef CONFIG_LIBC_LOCALE
          case 'a': case 'A':
            dest = &tm->tm_wday;
            min = ABDAY_1;
            range = 7;
            goto symbolic_range;

          case 'b': case 'B': case 'h':
            dest = &tm->tm_mon;
            min = ABMON_1;
            range = 12;
            goto symbolic_range;

          case 'c':

            /* Locale-specific date and time format */

            s = strptime(s, nl_langinfo(D_T_FMT), tm);
            if (!s)
              {
                return NULL;
              }

            break;

          case 'p':

            /* AM/PM */

            ex = nl_langinfo(AM_STR);
            len = strlen(ex);
            if (!strncasecmp(s, ex, len))
              {
                tm->tm_hour %= 12;
                s += len;
                break;
              }

            ex = nl_langinfo(PM_STR);
            len = strlen(ex);
            if (!strncasecmp(s, ex, len))
              {
                tm->tm_hour %= 12;
                tm->tm_hour += 12;
                s += len;
                break;
              }

            return 0;

          case 'r':

            /* Time in 12-hour format with AM/PM */

            s = strptime(s, nl_langinfo(T_FMT_AMPM), tm);
            if (!s)
              {
                return NULL;
              }

            break;

          case 'x':

            /* Date in locale-specific format */

            s = strptime(s, nl_langinfo(D_FMT), tm);
            if (!s)
              {
                return NULL;
              }

            break;

          case 'X':

           /* Time in locale-specific format */

            s = strptime(s, nl_langinfo(T_FMT), tm);
            if (!s)
              {
                return NULL;
              }

            break;

          symbolic_range:

            /* Process symbolic names for days of the week and months */

            for (i = 2 * range - 1; i >= 0; i--)
              {
                ex = nl_langinfo(min + i);
                len = strlen(ex);
                if (strncasecmp(s, ex, len))
                  {
                    continue;
                  }

                s += len;
                *dest = i % range;
                break;
              }

            if (i < 0)
              {
                return NULL;
              }

            goto update;
#endif

          update:

            /* Placeholder for additional updates (if needed) */

            ;
        }
    }

  /* Finalize year calculations */

  if (want_century)
    {
      tm->tm_year = relyear;
      if (want_century & 2)
        {
          tm->tm_year += century * 100 - 1900;
        }
      else if (tm->tm_year <= 68)
        {
          tm->tm_year += 100;
        }
    }

  return (FAR char *)s;
}
