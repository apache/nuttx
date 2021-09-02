/****************************************************************************
 * libs/libc/time/lib_strptime.c
 *
 *  $OpenBSD: strptime.c,v 1.11 2005/08/08 08:05:38 espie Exp $
 *  $NetBSD: strptime.c,v 1.12 1998/01/20 21:39:40 mycroft Exp $
 *
 * Copyright (c) 1997, 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code was contributed to The NetBSD Foundation by Klaus Klein.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *        This product includes software developed by the NetBSD
 *        Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <ctype.h>
#include <locale.h>
#include <strings.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TM_YEAR_BASE        1900
#define _ctloc(x)           (g_defaulttimelocale.x)

/* We do not implement alternate representations. However, we always
 * check whether a given modifier is allowed for a certain conversion.
 */

#define _ALT_E              0x01
#define _ALT_O              0x02
#define _LEGAL_ALT(x)       { if (alt_format & ~(x)) return (0); }

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct
{
  const char *abday[7];
  const char *day[7];
  const char *abmon[12];
  const char *mon[12];
  const char *am_pm[2];
  const char *d_t_fmt;
  const char *d_fmt;
  const char *t_fmt;
  const char *t_fmt_ampm;
} g_defaulttimelocale =
{
  {
    "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat",
  },
  {
    "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday",
    "Friday", "Saturday"
  },
  {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
  },
  {
    "January", "February", "March", "April", "May", "June", "July",
    "August", "September", "October", "November", "December"
  },
  {
    "AM", "PM"
  },
  "%a %b %d %H:%M:%S %Y",
  "%m/%d/%y",
  "%H:%M:%S",
  "%I:%M:%S %p"
};

struct century_relyear
{
  int century;
  int relyear;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int _conv_num(FAR const unsigned char **buf, FAR int *dest,
                     int llim, int ulim)
{
  int result = 0;
  int rulim = ulim;

  while (isspace(**buf))
    {
      (*buf)++;
    }

  if (**buf < '0' || **buf > '9')
    {
      return 0;
    }

  /* we use rulim to break out of the loop when we run out of digits */

  do
    {
      result *= 10;
      result += *(*buf)++ - '0';
      rulim /= 10;
    }
  while ((result * 10 <= ulim) && rulim && **buf >= '0' && **buf <= '9');

  if (result < llim || result > ulim)
    {
      return 0;
    }

  *dest = result;
  return 1;
}

static FAR const unsigned char *_strptime(FAR const unsigned char *buf,
                                          FAR const char *fmt,
                                          FAR struct tm *tm,
                                          FAR struct century_relyear *cr)
{
  unsigned char c;
  FAR const unsigned char *bp;
  size_t len = 0;
  int alt_format;
  int i;

  bp = buf;
  while ((c = *fmt) != '\0')
    {
      /* Clear `alternate' modifier prior to new conversion. */

      alt_format = 0;

      /* Eat up white-space. */

      if (isspace(c))
        {
          while (isspace(*bp))
            bp++;

          fmt++;
          continue;
        }

      if ((c = *fmt++) != '%')
        {
          goto literal;
        }

    again:
      switch (c = *fmt++)
        {
          case '%': /* "%%" is converted to "%". */
          literal:
            if (c != *bp++)
              {
                return NULL;
              }

            break;

            /* "Alternative" modifiers. Just set the appropriate flag
            * and start over again.
            */

          case 'E': /* "%E?" alternative conversion modifier. */
            _LEGAL_ALT(0);
            alt_format |= _ALT_E;
            goto again;

          case 'O': /* "%O?" alternative conversion modifier. */
            _LEGAL_ALT(0);
            alt_format |= _ALT_O;
            goto again;

            /* "Complex" conversion rules, implemented through recursion. */

          case 'c': /* Date and time, using the locale's format. */
            _LEGAL_ALT(_ALT_E);
            if (!(bp = _strptime(bp, _ctloc(d_t_fmt), tm, cr)))
              {
                return NULL;
              }

            break;

          case 'D': /* The date as "%m/%d/%y". */
            _LEGAL_ALT(0);
            if (!(bp = _strptime(bp, "%m/%d/%y", tm, cr)))
              {
                return NULL;
              }

            break;

          case 'R': /* The time as "%H:%M". */
            _LEGAL_ALT(0);
            if (!(bp = _strptime(bp, "%H:%M", tm, cr)))
              {
                return NULL;
              }

            break;

          case 'r': /* The time as "%I:%M:%S %p". */
            _LEGAL_ALT(0);
            if (!(bp = _strptime(bp, "%I:%M:%S %p", tm, cr)))
              {
                return NULL;
              }

            break;

          case 'T': /* The time as "%H:%M:%S". */
            _LEGAL_ALT(0);
            if (!(bp = _strptime(bp, "%H:%M:%S", tm, cr)))
              {
                return NULL;
              }

            break;

          case 'X': /* The time, using the locale's format. */
            _LEGAL_ALT(_ALT_E);
            if (!(bp = _strptime(bp, _ctloc(t_fmt), tm, cr)))
              {
                return NULL;
              }

            break;

          case 'x': /* The date, using the locale's format. */
            _LEGAL_ALT(_ALT_E);
            if (!(bp = _strptime(bp, _ctloc(d_fmt), tm, cr)))
              {
                return NULL;
              }

            break;

            /* "Elementary" conversion rules. */

          case 'A': /* The day of week, using the locale's form. */
          case 'a':
            _LEGAL_ALT(0);
            for (i = 0; i < 7; i++)
              {
                /* Full name. */

                len = strlen(_ctloc(day[i]));
                if (strncasecmp(_ctloc(day[i]),
                                (const char *)bp, len) == 0)
                  break;

                /* Abbreviated name. */

                len = strlen(_ctloc(abday[i]));
                if (strncasecmp(_ctloc(abday[i]),
                                (const char *)bp, len) == 0)
                  break;
              }

            /* Nothing matched. */

            if (i == 7)
              {
                return NULL;
              }

            tm->tm_wday = i;
            bp += len;
            break;

          case 'B': /* The month, using the locale's form. */
          case 'b':
          case 'h':
            _LEGAL_ALT(0);
            for (i = 0; i < 12; i++)
              {
                /* Full name. */

                len = strlen(_ctloc(mon[i]));
                if (strncasecmp(_ctloc(mon[i]),
                                (const char *)bp, len) == 0)
                  {
                    break;
                  }

                /* Abbreviated name. */

                len = strlen(_ctloc(abmon[i]));
                if (strncasecmp(_ctloc(abmon[i]),
                                (const char *)bp, len) == 0)
                  {
                    break;
                  }
              }

            /* Nothing matched. */

            if (i == 12)
              {
                return NULL;
              }

            tm->tm_mon = i;
            bp += len;
            break;

          case 'C': /* The century number. */
            _LEGAL_ALT(_ALT_E);
            if (!(_conv_num(&bp, &i, 0, 99)))
              {
                return (NULL);
              }

            cr->century = i * 100;
            break;

          case 'd': /* The day of month. */
          case 'e':
            _LEGAL_ALT(_ALT_O);
            if (!(_conv_num(&bp, &tm->tm_mday, 1, 31)))
              {
                return (NULL);
              }

            break;

          case 'k': /* The hour (24-hour clock representation). */
            _LEGAL_ALT(0);

            /* FALLTHROUGH */

          case 'H':
            _LEGAL_ALT(_ALT_O);
            if (!(_conv_num(&bp, &tm->tm_hour, 0, 23)))
              {
                return NULL;
              }

            break;

          case 'l': /* The hour (12-hour clock representation). */
            _LEGAL_ALT(0);

            /* FALLTHROUGH */

          case 'I':
            _LEGAL_ALT(_ALT_O);
            if (!(_conv_num(&bp, &tm->tm_hour, 1, 12)))
              {
                return (NULL);
              }

            break;

          case 'j': /* The day of year. */
            _LEGAL_ALT(0);
            if (!(_conv_num(&bp, &tm->tm_yday, 1, 366)))
              {
                return NULL;
              }

            tm->tm_yday--;
            break;

          case 'M': /* The minute. */
            _LEGAL_ALT(_ALT_O);
            if (!(_conv_num(&bp, &tm->tm_min, 0, 59)))
              {
                return NULL;
              }

            break;

          case 'm': /* The month. */
            _LEGAL_ALT(_ALT_O);
            if (!(_conv_num(&bp, &tm->tm_mon, 1, 12)))
              {
                return NULL;
              }

            tm->tm_mon--;
            break;

          case 'p': /* The locale's equivalent of AM/PM. */
            _LEGAL_ALT(0);

            /* AM? */

            len = strlen(_ctloc(am_pm[0]));
            if (strncasecmp(_ctloc(am_pm[0]), (const char *)bp, len) == 0)
              {
                if (tm->tm_hour > 12) /* i.e., 13:00 AM ?! */
                  {
                    return NULL;
                  }
                else if (tm->tm_hour == 12)
                  {
                    tm->tm_hour = 0;
                  }

                bp += len;
                break;
              }

            /* PM? */

            len = strlen(_ctloc(am_pm[1]));
            if (strncasecmp(_ctloc(am_pm[1]), (const char *)bp, len) == 0)
              {
                if (tm->tm_hour > 12) /* i.e., 13:00 PM ?! */
                  {
                    return NULL;
                  }
                else if (tm->tm_hour < 12)
                  {
                    tm->tm_hour += 12;
                  }

                bp += len;
                break;
              }

            /* Nothing matched. */

            return NULL;

          case 'S': /* The seconds. */
            _LEGAL_ALT(_ALT_O);
            if (!(_conv_num(&bp, &tm->tm_sec, 0, 61)))
              {
                return NULL;
              }

            break;

          case 'U': /* The week of year, beginning on sunday. */
          case 'W': /* The week of year, beginning on monday. */
            _LEGAL_ALT(_ALT_O);
            /* XXX This is bogus, as we can not assume any valid
            * information present in the tm structure at this
            * point to calculate a real value, so just check the
            * range for now.
            */

            if (!(_conv_num(&bp, &i, 0, 53)))
              {
                return NULL;
              }

            break;

          case 'w': /* The day of week, beginning on sunday. */
            _LEGAL_ALT(_ALT_O);
            if (!(_conv_num(&bp, &tm->tm_wday, 0, 6)))
              {
                return NULL;
              }

            break;

          case 'Y': /* The year. */
            _LEGAL_ALT(_ALT_E);
            if (!(_conv_num(&bp, &i, 0, 9999)))
              {
                return NULL;
              }

            cr->relyear = -1;
            tm->tm_year = i - TM_YEAR_BASE;
            break;

          case 'y': /* The year within the century (2 digits). */
            _LEGAL_ALT(_ALT_E | _ALT_O);
            if (!(_conv_num(&bp, &cr->relyear, 0, 99)))
              {
                return NULL;
              }

            break;

            /* Miscellaneous conversions. */

          case 'n': /* Any kind of white-space. */
          case 't':
            _LEGAL_ALT(0);
            while (isspace(*bp))
              {
                bp++;
              }

            break;

          default: /* Unknown/unsupported conversion. */
            return NULL;
        }
    }

  /* We need to evaluate the two digit year spec (%y)
   * last as we can get a century spec (%C) at any time.
   */

  if (cr->relyear != -1)
  {
    if (cr->century == TM_YEAR_BASE)
    {
      if (cr->relyear <= 68)
        {
          tm->tm_year = cr->relyear + 2000 - TM_YEAR_BASE;
        }
      else
        {
          tm->tm_year = cr->relyear + 1900 - TM_YEAR_BASE;
        }
    }
    else
    {
      tm->tm_year = cr->relyear + cr->century - TM_YEAR_BASE;
    }
  }

  return bp;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strptime
 *
 * Description:
 *   Convert a string to a time type according to a specific time format
 *   The strptime() function processes the input string from left to
 *   right.  Each of the three possible input elements (whitespace,
 *   literal, or format) are handled one after the other.  If the
 *   input cannot be matched to the format string, the function stops.
 *   The remainder of the format and input strings are not processed.
 *   The supported input field descriptors are listed below.  In case
 *   a text string (such as the name of a day of the week or a month
 *   name) is to be matched, the comparison is case insensitive.  In
 *   case a number is to be matched, leading zeros are permitted but
 *   not required.
 *   %a     The abbreviated weekday name according to the current locale.
 *   %A     The full weekday name according to the current locale.
 *   %b     The abbreviated month name according to the current locale.
 *   %B     The full month name according to the current locale.
 *   %C     The century number (year/100) as a 2-digit integer. (SU)
 *   %d     The day of the month as a decimal number (range 01 to 31).
 *   %e     Like %d, the day of the month as a decimal number, but a leading
 *          zero is replaced by a space.
 *   %h     Equivalent to %b.  (SU)
 *   %H     The hour as a decimal number using a 24-hour clock
 *          (range 00 to 23).
 *   %I     The  hour as a decimal number using a 12-hour clock
 *          (range 01 to 12).
 *   %j     The day of the year as a decimal number (range 001 to 366).
 *   %k     The hour (24-hour clock) as a decimal number (range  0  to  23);
 *          single digits are preceded by a blank.  (See also %H.)  (TZ)
 *   %l     The  hour  (12-hour  clock) as a decimal number (range 1 to 12);
 *          single digits are preceded by a blank.  (See also %I.)  (TZ)
 *   %m     The month as a decimal number (range 01 to 12).
 *   %M     The minute as a decimal number (range 00 to 59).
 *   %n     A newline character. (SU)
 *   %p     Either "AM" or "PM" according to the given time  value, or the
 *          corresponding  strings  for the current locale.  Noon is treated
 *          as "PM" and midnight as "AM".
 *   %P     Like %p but in lowercase: "am" or "pm" or a corresponding string
 *          for the current locale. (GNU)
 *   %S     The second as a decimal number (range 00 to 60).  (The range is
 *          up to 60 to allow for occasional leap seconds.)
 *   %t     A tab character. (SU)
 *   %y     The year as a decimal number without a century (range 00 to 99).
 *   %Y     The year as a decimal number including the century.
 *   %%     A literal '%' character.
 *
 * Returned Value:
 *   The return value of the function is a pointer to the first
 *   character not processed in this function call.  In case the input
 *   string contains more characters than required by the format
 *   string, the return value points right after the last consumed
 *   input character.  In case the whole input string is consumed, the
 *   return value points to the null byte at the end of the string.
 *   If strptime() fails to match all of the format string and
 *   therefore an error occurred, the function returns NULL.
 *
 ****************************************************************************/

FAR char *strptime(FAR const char *buf, FAR const char *fmt,
                   FAR struct tm *tm)
{
  struct century_relyear cr;
  cr.century = TM_YEAR_BASE;
  cr.relyear = -1;
  return (FAR char *)(_strptime((FAR const unsigned char *)buf,
                                fmt, tm, &cr));
}
