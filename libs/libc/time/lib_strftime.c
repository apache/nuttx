/****************************************************************************
 * libs/libc/time/lib_strftime.c
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
#include <sys/types.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>

#include <nuttx/clock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Constant Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char * const g_abbrev_wdayname[7] =
{
  "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};

static const char * const g_wdayname[7] =
{
  "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday",
  "Saturday"
};

static const char * const g_abbrev_monthname[12] =
{
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

static const char * const g_monthname[12] =
{
  "January", "February", "March",     "April",   "May",      "June",
  "July",    "August",   "September", "October", "November", "December"
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: is_leap
 *
 * Description:
 *  determine if the given year is a leap year or not
 *
 * Input Parameters:
 *  year - a year value
 *
 * Returnd value:
 *  true if current is leap year, false is not a leap year
 */

static bool is_leap(int year)
{
  return year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
}

/****************************************************************************
 * Name: get_week_num
 *
 * Description:
 *  get the week number in a year based on iso8601 standard
 *
 * Input Parameters:
 *  time - the specified time
 *
 * Returnd value:
 *  the week numer in a year
 */

static int get_week_num(FAR const struct tm *time)
{
  /* calculate the total week number in a year */

  int week = (time->tm_yday + DAYSPERWEEK -
              (time->tm_wday + 6) % DAYSPERWEEK) / DAYSPERWEEK;

  /* if xxxx-1-1 is just passed 1-3 days after Monday
   * then the previous week will calculated into this year
   */

  if ((time->tm_wday + 371 - time->tm_yday - 2) % DAYSPERWEEK <= 2)
    {
      week++;
    }

  if (week == 0)
    {
      week = 52;

      /* if xxxx-12-31 is Thursday or Friday, and the previous year is
       * leap year, then the previous year has 53 weeks
       */

      int dec31 = (time->tm_wday + DAYSPERWEEK - time->tm_yday - 1)
                      % DAYSPERWEEK;
      if (dec31 == TM_THURSDAY ||
          (dec31 == TM_FRIDAY && is_leap(time->tm_year % 400 - 1)))
        {
          week++;
        }
    }
  else if (week == 53)
    {
      /* If xxxx-1.1 is not a Thursday, and not a Wednesday of a leap year,
       * then this year has only 52 weeks
       */

      int jan1 = (time->tm_wday + 371 - time->tm_yday) % DAYSPERWEEK;
      if (jan1 != TM_THURSDAY &&
            (jan1 != TM_WEDNESDAY || !is_leap(time->tm_year)))
        {
          week = 1;
        }
    }

  return week;
}

/****************************************************************************
 * Name: get_week_year
 *
 * Description:
 *  get the week year based on iso8601 standard
 *
 * Input Parameters:
 *  time - the specified time
 *
 * Returnd value:
 *  the year that calculated based on week number
 */

static int get_week_year(FAR const struct tm *time)
{
  int week_num = get_week_num(time);
  int week_year = time->tm_year + TM_YEAR_BASE;
  if (time->tm_yday < 3 && week_num != 1)
    {
      week_year--;
    }
  else if (time->tm_yday > 360 && week_num == 1)
    {
      week_year++;
    }

  return week_year;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  strftime
 *
 * Description:
 *   The  strftime()  function  formats the broken-down time tm according to
 *   the format specification format and places the result in the  character
 *   array s of size max.
 *
 *   Ordinary characters placed in the format string are copied to s without
 *   conversion.  Conversion specifications are introduced by a '%'  charac-
 *   ter,  and  terminated  by  a  conversion  specifier  character, and are
 *   replaced in s as follows:
 *
 *   %a     The abbreviated weekday name according to the current locale.
 *   %A     The full weekday name according to the current locale.
 *   %b     The abbreviated month name according to the current locale.
 *   %B     The full month name according to the current locale.
 *   %C     The century number (year/100) as a 2-digit integer. (SU)
 *   %d     The day of the month as a decimal number (range 01 to 31).
 *   %e     Like %d, the day of the month as a decimal number, but a leading
 *          zero is replaced by a space.
 *   %F     The full date format but with no time fields
 *   %g     The last 2 digits of the week-based year as a decimal number
 *          [00,99]
 *   %G     The full version of %g, display the full year value
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
 *   %r     The time in a.m. and p.m. notation
 *   %R     The time in 24-hour notation ( %H : %M ).
 *   %s     The number of seconds since the Epoch, that is, since 1970-01-01
 *          00:00:00 UTC. (TZ)
 *   %S     The second as a decimal number (range 00 to 60).  (The range is
 *          up to 60 to allow for occasional leap seconds.)
 *   %t     A tab character. (SU)
 *   %u     The weekday as a decimal number [1,7], with 1 representing
 *          Monday.
 *   %U     The week number of the year as a decimal number [00,53].
 *   %V     The week number of the year
 *   %w     The weekday as a decimal number (range 0 to 6).
 *   %W     The week number of the year as a decimal number [00,53].
 *   %x     The locale's appropriate date representation, but without time.
 *   %X     The locale's appropriate time representation, but without date.
 *   %y     The year as a decimal number without a century (range 00 to 99).
 *   %Y     The year as a decimal number including the century.
 *   %z     The timezone name or abbreviation, or by no bytes if no timezone
 *          information exists.
 *   %%     A literal '%' character.
 *
 * Returned Value:
 *   The strftime() function returns the number of characters placed in  the
 *   array s, not including the terminating null byte, provided the string,
 *   including the terminating null byte, fits.  Otherwise,  it returns 0,
 *   and the contents of the array is undefined.
 *
 ****************************************************************************/

size_t strftime(FAR char *s, size_t max, FAR const char *format,
                FAR const struct tm *tm)
{
  FAR const char *str;
  FAR char       *dest   = s;
  int             chleft = max;
  int             value;
  int             len;

  while (*format && chleft > 0)
    {
      /* Just copy regular characters */

      if (*format != '%')
        {
           *dest++ = *format++;
           chleft--;
           continue;
        }

      /* Handle the format character */

       format++;
       len   = 0;

process_next:
       switch (*format++)
         {
           /* %a: A three-letter abbreviation for the day of the week. */

           case 'a':
             {
               if (tm->tm_wday < 7)
                 {
                   str = g_abbrev_wdayname[tm->tm_wday];
                   len = snprintf(dest, chleft, "%s", str);
                 }
             }
             break;

           /* %A: The full name for the day of the week. */

           case 'A':
             {
               if (tm->tm_wday < 7)
                 {
                   str = g_wdayname[tm->tm_wday];
                   len = snprintf(dest, chleft, "%s", str);
                 }
             }
             break;

           /* %h: Equivalent to %b */

           case 'h':

           /* %b: The abbreviated month name according to the current
            * locale.
            */

           case 'b':
             {
               if (tm->tm_mon < 12)
                 {
                   str = g_abbrev_monthname[tm->tm_mon];
                   len = snprintf(dest, chleft, "%s", str);
                 }
             }
             break;

           /* %B: The full month name according to the current locale. */

           case 'B':
             {
               if (tm->tm_mon < 12)
                 {
                   str = g_monthname[tm->tm_mon];
                   len = snprintf(dest, chleft, "%s", str);
                 }
             }
             break;

           /* %C: The century number (year/100) as a 2-digit integer. */

           case 'C':
             {
               len = snprintf(dest, chleft, "%02d", tm->tm_year / 100);
             }
             break;

           /* %d: The day of the month as a decimal number
            * (range 01 to 31).
            */

           case 'd':
             {
               len = snprintf(dest, chleft, "%02d", tm->tm_mday);
             }
             break;

           /* The 'E' or 'O' are modifier characters to indicate that an
            * alternative format or specification should be used rather than
            * the one normally used by unmodified conversion specifier.
            * the following are the supported format:
            * %Ec %EC %Ex %EX %Ey %EY
            * %Od %oe %OH %OI %Om %OM
            * %OS %Ou %OU %OV %Ow %OW %Oy
            * If the alternative format or specification does not exist for
            * current locale, then the behavior shall be same as the
            * unmodified conversion specification, i.e the %Ec is same as %c
            */

           case 'E':
           case 'O':
             {
               goto process_next;
             }

           /* %e: Like %d, the day of the month as a decimal number, but
            * a leading zero is replaced by a space.
            */

           case 'e':
             {
               len = snprintf(dest, chleft, "%2d", tm->tm_mday);
             }
             break;

            /* %F: ISO 8601 date format: "%Y-%m-%d" */

            case 'F':
              {
                len = snprintf(dest, chleft, "%04d-%02d-%02d",
                              tm->tm_year + TM_YEAR_BASE, tm->tm_mon,
                              tm->tm_mday);
              }
              break;

            /* %g: 2-digit year version of %G, (00-99) */

            case 'g':
              {
                value = get_week_year(tm) % 100;
                len = snprintf(dest, chleft, "%02d", value);
              }
              break;

            /* %G: ISO 8601 week based year */

            case 'G':
              {
                len = snprintf(dest, chleft, "%04d", get_week_year(tm));
              }
              break;

           /* %H: The hour as a decimal number using a 24-hour clock
            * (range 00  to 23).
            */

           case 'H':
             {
               len = snprintf(dest, chleft, "%02d", tm->tm_hour);
             }
             break;

           /* %I: The  hour as a decimal number using a 12-hour clock
            * (range 01 to 12).
            */

           case 'I':
             {
               len = snprintf(dest, chleft, "%02d", (tm->tm_hour % 12) != 0 ?
                                                    (tm->tm_hour % 12) : 12);
             }
             break;

           /* %j: The day of the year as a decimal number
            * (range 001 to 366).
            */

           case 'j':
             {
               if (tm->tm_mon < 12)
                 {
                   value = clock_daysbeforemonth(tm->tm_mon,
                           clock_isleapyear(tm->tm_year)) + tm->tm_mday;
                   len   = snprintf(dest, chleft, "%03d", value);
                 }
             }
             break;

           /* %k: The hour (24-hour clock) as a decimal number
            * (range  0  to  23);
            * single digits are preceded by a blank.
            */

           case 'k':
             {
               len = snprintf(dest, chleft, "%2d", tm->tm_hour);
             }
             break;

           /* %l: The  hour  (12-hour  clock) as a decimal number
            * (range 1 to 12);
            * single digits are preceded by a blank.
            */

           case 'l':
             {
               len = snprintf(dest, chleft, "%2d", (tm->tm_hour % 12) != 0 ?
                                                   (tm->tm_hour % 12) : 12);
             }
             break;

           /* %m: The month as a decimal number (range 01 to 12). */

           case 'm':
             {
               len = snprintf(dest, chleft, "%02d", tm->tm_mon + 1);
             }
             break;

           /* %M: The minute as a decimal number (range 00 to 59). */

           case 'M':
             {
               len = snprintf(dest, chleft, "%02d", tm->tm_min);
             }
             break;

           /* %n: A newline character. */

           case 'n':
             {
               *dest = '\n';
               len   = 1;
             }
             break;

           /* %p: Either "AM" or "PM" according to the given time  value. */

           case 'p':
             {
               if (tm->tm_hour >= 12)
                 {
                   str = "PM";
                 }
               else
                 {
                   str = "AM";
                 }

               len = snprintf(dest, chleft, "%s", str);
             }
             break;

           /* %P: Like %p but in lowercase: "am" or "pm" */

           case 'P':
             {
               if (tm->tm_hour >= 12)
                 {
                   str = "pm";
                 }
               else
                 {
                   str = "am";
                 }

               len = snprintf(dest, chleft, "%s", str);
             }
             break;

           /* %r: 12-hour clock time */

           case 'r':
             {
               if (tm->tm_hour >= 12)
                 {
                   str = "pm";
                 }
               else
                 {
                   str = "am";
                 }

               value = tm->tm_hour == 12 ?
                          tm->tm_hour == 12 :
                          tm->tm_hour % (HOURSPERDAY / 2);

               len = snprintf(dest, chleft, "%02d:%02d:%02d %s",
                              value, tm->tm_min, tm->tm_sec, str);
             }
             break;

            /* %R: Shortcut for %H:%M. */

           case 'R':
             {
               len = snprintf(dest, chleft, "%02d:%02d",
                              tm->tm_hour, tm->tm_min);
             }
             break;

           /* %s: The number of seconds since the Epoch, that is,
            * since 1970-01-01 00:00:00 UTC.
            * Hmmm... mktime argume is not 'const'.
            */

           case 's':
             {
               struct tm tmp = *tm;
               len = snprintf(dest, chleft, "%ju", (uintmax_t)mktime(&tmp));
             }
             break;

           /* %S: The second as a decimal number (range 00 to 60).
            * (The range is up to 60 to allow for occasional leap seconds.)
            */

           case 'S':
             {
               len = snprintf(dest, chleft, "%02d", tm->tm_sec);
             }
             break;

           /* %t: A tab character. */

           case 't':
             {
               *dest = '\t';
               len   = 1;
             }
             break;

           /* %T: Shortcut for %H:%M:%S. */

           case 'T':
             {
               len = snprintf(dest, chleft, "%02d:%02d:%02d",
                              tm->tm_hour, tm->tm_min, tm->tm_sec);
             }
             break;

           /* %u: The day of the week as a decimal, (1-7). Monday being 1,
            * Sunday being 0.
            */

           case 'u':
             {
               value = tm->tm_wday == 0 ? 7 : tm->tm_wday;
               len = snprintf(dest, chleft, "%d", value);
             }
             break;

           /* %U: week number of the current year as a decimal number,
            * (00-53). Starting with the first Sunday as the first day
            * of week 01.
            */

           case 'U':
             {
               value = (tm->tm_yday + DAYSPERWEEK - tm->tm_wday)
                                  / DAYSPERWEEK;
               len = snprintf(dest, chleft, "%02d", value);
             }
             break;

           /* %V: ISO 8601 week number */

           case 'V':
             {
               value = get_week_num(tm);
               len = snprintf(dest, chleft, "%02d", value);
             }
             break;

           /* %w: The weekday as a decimal number (range 0 to 6). */

           case 'w':
             {
               len = snprintf(dest, chleft, "%d", tm->tm_wday);
             }
             break;

           /* %W: Week number of the current year as a decimal number,
            * (00-53). Starting with the first Monday as the first day
            * of week 01.
            */

           case 'W':
             {
               value = (tm->tm_yday + DAYSPERWEEK -
                                 (tm->tm_wday + 6) % DAYSPERWEEK)
                                  / DAYSPERWEEK;
               len = snprintf(dest, chleft, "%02d", value);
             }
             break;

           /* %x Locale date without time */

           case 'x':
             {
                len = snprintf(dest, chleft, "%02d/%02d/%04d",
                              tm->tm_mon, tm->tm_mday,
                              tm->tm_year + TM_YEAR_BASE);
             }
             break;

           /* %X: Locale time without date */

           case 'X':
             {
               len = snprintf(dest, chleft, "%02d:%02d:%02d",
                              tm->tm_hour, tm->tm_min, tm->tm_sec);
             }
             break;

           /* %y: The year as a decimal number without a century
            * (range 00 to 99).
            */

           case 'y':
             {
               len = snprintf(dest, chleft, "%02d", tm->tm_year % 100);
             }
             break;

           /* %Y: The year as a decimal number including the century. */

           case 'Y':
             {
               len = snprintf(dest, chleft, "%04d",
                              tm->tm_year + TM_YEAR_BASE);
             }
             break;

            /* %z: Numeric timezone as hour and minute offset from UTC
             * "+hhmm" or "-hhmm"
             */

            case 'z':
              {
                int hour = tm->tm_gmtoff / 3600;
                int min = tm->tm_gmtoff % 3600 / 60;
                int utc_val = hour  * 100 + min;
                len = snprintf(dest, chleft, "+%04d", utc_val);
              }
              break;

           /* %%:  A literal '%' character. */

           case '%':
             {
               *dest = '%';
               len   = 1;
             }
             break;
        }

      /* Update counts and pointers */

      dest   += len;
      chleft -= len;
    }

  /* We get here because either we have reached the end of the format string
   * or because there is no more space in the user-provided buffer and the
   * resulting string has been truncated.
   *
   * Is there space remaining in the user-provided buffer for the NUL
   * terminator?
   */

  if (chleft > 0)
    {
      /* Yes, append terminating NUL byte */

      *dest = '\0';

      /* And return the number of bytes in the resulting string (excluding
       * the NUL terminator).
       */

      return max - chleft;
    }

  /* The string was truncated and/or not properly terminated.  Return
   * zero.
   */

  return 0;
}
