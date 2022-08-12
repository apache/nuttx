/****************************************************************************
 * libs/libc/time/lib_localtime.c
 *
 * Re-released as part of NuttX under the 3-clause BSD license:
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Ported to NuttX by Max Neklyudov
 *   Style updates by Gregory Nutt
 *
 * With these notes:
 *
 *   This file is in the public domain, so clarified as of
 *   1996-06-05 by Arthur David Olson.
 *
 *   Leap second handling from Bradley White.
 *   POSIX-style TZ environment variable handling from Guy Harris.
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
#include <time.h>
#include <string.h>
#include <limits.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/init.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Time zone object file directory */

#ifdef CONFIG_LIBC_TZDIR
#  define TZDIR CONFIG_LIBC_TZDIR
#else
#  define TZDIR "/etc/zoneinfo"
#endif

/* Time definitions *********************************************************/

/* Time zone files */

#define TZ_MAGIC            "TZif"
#define TZDEFAULT           "localtime"
#define TZDEFRULES          "posixrules"

/* In the current implementation, "tzset()" refuses to deal with files that
 * exceed any of the limits below.
 */

#define TZ_MAX_CHARS        50  /* Maximum number of abbreviation characters */
#define TZ_MAX_LEAPS        50  /* Maximum number of leap second corrections */

#define SECSPERMIN          60
#define MINSPERHOUR         60
#define HOURSPERDAY         24
#define DAYSPERWEEK         7
#define DAYSPERNYEAR        365
#define DAYSPERLYEAR        366
#define SECSPERHOUR         (SECSPERMIN * MINSPERHOUR)
#define SECSPERDAY          ((int_fast32_t)SECSPERHOUR * HOURSPERDAY)
#define MONSPERYEAR         12

#define TM_SUNDAY           0
#define TM_MONDAY           1
#define TM_TUESDAY          2
#define TM_WEDNESDAY        3
#define TM_THURSDAY         4
#define TM_FRIDAY           5
#define TM_SATURDAY         6

#define TM_JANUARY          0
#define TM_FEBRUARY         1
#define TM_MARCH            2
#define TM_APRIL            3
#define TM_MAY              4
#define TM_JUNE             5
#define TM_JULY             6
#define TM_AUGUST           7
#define TM_SEPTEMBER        8
#define TM_OCTOBER          9
#define TM_NOVEMBER         10
#define TM_DECEMBER         11

#define TM_YEAR_BASE        1900

#define EPOCH_YEAR          1970
#define EPOCH_WDAY          TM_THURSDAY

#define isleap(y)           (((y) % 4) == 0 && (((y) % 100) != 0 || ((y) % 400) == 0))

/* Since everything in isleap is modulo 400 (or a factor of 400), we know
 * that
 *    isleap(y) == isleap(y % 400)
 * and so
 *    isleap(a + b) == isleap((a + b) % 400)
 * or
 *    isleap(a + b) == isleap(a % 400 + b % 400)
 *
 * This is true even if % means modulo rather than Fortran remainder (which
 * is allowed by C89 but not C99).  We use this to avoid addition overflow
 * problems.
 */

#define isleap_sum(a, b)    isleap((a) % 400 + (b) % 400)

#define GRANDPARENTED       "Local time zone must be set--see zic manual page"
#define TYPE_BIT(type)      (sizeof(type) * CHAR_BIT)
#define TYPE_SIGNED(type)   (((type)-1) < 0)

#define YEARSPERREPEAT      400    /* years before a Gregorian repeat */

/* The Gregorian year averages 365.2425 days, which is 31556952 seconds. */

#define AVGSECSPERYEAR      31556952L
#define SECSPERREPEAT       ((int_fast64_t)YEARSPERREPEAT * (int_fast64_t)AVGSECSPERYEAR)
#define SECSPERREPEAT_BITS  34    /* ceil(log2(SECSPERREPEAT)) */

#define TZ_ABBR_MAX_LEN     16
#define TZ_ABBR_CHAR_SET \
  "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 :+-._"
#define TZ_ABBR_ERR_CHAR    '_'

/* Unlike <ctype.h>'s isdigit, this also works if c < 0 | c > UCHAR_MAX. */

#define is_digit(c)         ((unsigned)(c) - '0' <= 9)
#define BIGGEST(a, b)       (((a) > (b)) ? (a) : (b))
#define MY_TZNAME_MAX       255
#define GMT                 "GMT"
#define GMTLEN              3

#define JULIAN_DAY            0  /* Jn = Julian day */
#define DAY_OF_YEAR           1  /* n = day of year */
#define MONTH_NTH_DAY_OF_WEEK 2  /* Mm.n.d = month, week, day of week */

/* Someone might make incorrect use of a time zone abbreviation:
 *    1.    They might reference tzname[0] before calling tzset (explicitly
 *        or implicitly).
 *    2.    They might reference tzname[1] before calling tzset (explicitly
 *        or implicitly).
 *    3.    They might reference tzname[1] after setting to a time zone
 *        in which Daylight Saving Time is never observed.
 *    4.    They might reference tzname[0] after setting to a time zone
 *        in which Standard Time is never observed.
 *
 * What's best to do in the above cases is open to debate;
 * for now, we just set things up so that in any of the five cases
 * WILDABBR is used. Another possibility: initialize tzname[0] to the
 * string "tzname[0] used before set", and similarly for the other cases.
 * And another: initialize tzname[0] to "ERA", with an explanation in the
 * manual page of what this "time zone abbreviation" means (doing this so
 * that tzname[0] has the "normal" length of three characters).
 */

#define WILDABBR            "   "

/* The DST rules to use if TZ has no rules and we can't load TZDEFRULES.
 * We default to US rules as of 1999-08-17.
 * POSIX 1003.1 section 8.1.1 says that the default DST rules are
 * implementation dependent; for historical reasons, US rules are a
 * common default.
 */

#define TZDEFRULESTRING     ",M3.2.0,M11.1.0"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Time file file header.
 * Each time zone file begins with a time zone header followed by:
 *
 *    tzh_timecnt (char [4])s        coded transition times a la time(2)
 *    tzh_timecnt (unsigned char)s    types of local time starting at above
 *    tzh_typecnt repetitions of
 *        one (char [4])        coded UT offset in seconds
 *        one (unsigned char)    used to set tm_isdst
 *        one (unsigned char)    that's an abbreviation list index
 *    tzh_charcnt (char)s        '\0'-terminated zone abbreviations
 *    tzh_leapcnt repetitions of
 *        one (char [4])        coded leap second transition times
 *        one (char [4])        total correction after above
 *    tzh_ttisstdcnt (char)s        indexed by type; if TRUE, transition
 *                    time is standard time, if FALSE,
 *                    transition time is wall clock time
 *                    if absent, transition times are
 *                    assumed to be wall clock time
 *    tzh_ttisgmtcnt (char)s        indexed by type; if TRUE, transition
 *                    time is UT, if FALSE,
 *                    transition time is local time
 *                    if absent, transition times are
 *                    assumed to be local time
 *
 * If tzh_version is '2' or greater, the above is followed by a second
 * instance of tzhead_s and a second instance of the data in which each
 * coded transition time uses 8 rather than 4 chars, then a
 * POSIX-TZ-environment-variable-style string for use in handling instants
 * after the last transition time stored in the file (with nothing between
 * the newlines if there is no POSIX representation for such instants).
 *
 * If tz_version is '3' or greater, the above is extended as follows.
 * First, the POSIX TZ string's hour offset may range from -167 through
 * 167 as compared to the POSIX-required 0 through 24.  Second, its DST
 * start time may be January 1 at 00:00 and its stop time December 31 at
 * 24:00 plus the difference between DST and standard time, indicating DST
 * all year.
 */

struct tzhead_s
{
  char tzh_magic[4];          /* TZ_MAGIC */
  char tzh_version[1];        /* '\0' or '2' or '3' as of 2013 */
  char tzh_reserved[15];      /* reserved; must be zero */
  char tzh_ttisgmtcnt[4];     /* coded number of trans. time flags */
  char tzh_ttisstdcnt[4];     /* coded number of trans. time flags */
  char tzh_leapcnt[4];        /* coded number of leap seconds */
  char tzh_timecnt[4];        /* coded number of transition times */
  char tzh_typecnt[4];        /* coded number of local time types */
  char tzh_charcnt[4];        /* coded number of abbr. chars */
};

struct ttinfo_s
{                             /* Time type information */
  int_fast32_t tt_gmtoff;     /* UT offset in seconds */
  int tt_isdst;               /* Used to set tm_isdst */
  int tt_abbrind;             /* Abbreviation list index */
  int tt_ttisstd;             /* True if transition is std time */
  int tt_ttisgmt;             /* True if transition is UT */
};

struct lsinfo_s
{                             /* Leap second information */
  time_t ls_trans;            /* Transition time */
  int_fast32_t ls_corr;       /* Correction to apply */
};

struct state_s
{
  int leapcnt;
  int timecnt;
  int typecnt;
  int charcnt;
  int goback;
  int goahead;
  time_t ats[TZ_MAX_TIMES];
  unsigned char types[TZ_MAX_TIMES];
  struct ttinfo_s ttis[TZ_MAX_TYPES];
  char chars[BIGGEST(BIGGEST(TZ_MAX_CHARS + 1, GMTLEN),
                    (2 * (MY_TZNAME_MAX + 1)))];
  struct lsinfo_s lsis[TZ_MAX_LEAPS];
  int defaulttype;            /* For early times or if no transitions */
};

struct rule_s
{
  int r_type;                 /* type of rule; see below */
  int r_day;                  /* day number of rule */
  int r_week;                 /* week number of rule */
  int r_mon;                  /* month number of rule */
  int_fast32_t r_time;        /* transition time of rule */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The minimum and maximum finite time values.  */

static const time_t g_min_timet =
  (TYPE_SIGNED(time_t)
    ? (time_t)-1 << (CHAR_BIT * sizeof(time_t) - 1)
    : 0);

static const time_t g_max_timet =
  (TYPE_SIGNED(time_t)
    ? - (~ 0 < 0) - ((time_t)-1 << (CHAR_BIT * sizeof(time_t) - 1))
    : -1);

static const char g_wildabbr[] = WILDABBR;

static char g_lcl_tzname[MY_TZNAME_MAX + 1];
static int g_lcl_isset;
static int g_gmt_isset;
static FAR struct state_s *g_lcl_ptr;
static FAR struct state_s *g_gmt_ptr;
static rmutex_t g_lcl_lock = NXRMUTEX_INITIALIZER;
static rmutex_t g_gmt_lock = NXRMUTEX_INITIALIZER;

/* Section 4.12.3 of X3.159-1989 requires that
 *    Except for the strftime function, these functions [asctime,
 *    ctime, gmtime, localtime] return values in one of two static
 *    objects: a broken-down time structure and an array of char.
 * Thanks to Paul Eggert for noting this.
 */

static struct tm g_tm;

static const int g_mon_lengths[2][MONSPERYEAR] =
{
  {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
  {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
};

static const int g_year_lengths[2] =
{
  DAYSPERNYEAR, DAYSPERLYEAR
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Setup by tzset() */

FAR char *tzname[2] =
{
  (FAR char *)g_wildabbr,
  (FAR char *)g_wildabbr
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int_fast32_t detzcode(FAR const char *codep);
static int_fast64_t detzcode64(FAR const char *codep);
static int differ_by_repeat(time_t t1, time_t t0);
static FAR const char *getzname(FAR const char *strp);
static FAR const char *getqzname(FAR const char *strp, int delim);
static FAR const char *getnum(FAR const char *strp, FAR int *nump,
              int min, int max);
static FAR const char *getsecs(FAR const char *strp,
              FAR int_fast32_t *secsp);
static FAR const char *getoffset(FAR const char *strp,
              FAR int_fast32_t *offsetp);
static FAR const char *getrule(FAR const char *strp,
              FAR struct rule_s *rulep);
static void gmtload(FAR struct state_s *sp);
static FAR struct tm *gmtsub(FAR const time_t *timep, int_fast32_t offset,
              FAR struct tm *tmp);
static FAR struct tm *localsub(FAR const time_t *timep, int_fast32_t offset,
              FAR struct tm *tmp);
static int  increment_overflow(FAR int *number, int delta);
static int  leaps_thru_end_of(int y);
static int  increment_overflow32(FAR int_fast32_t *number, int delta);
static int  increment_overflow_time(FAR time_t *t, int_fast32_t delta);
static int  normalize_overflow32(FAR int_fast32_t *tensptr,
              FAR int *unitsptr, int base);
static int  normalize_overflow(FAR int *tensptr, FAR int *unitsptr,
              int base);
static void settzname(void);
static time_t time1(FAR struct tm *tmp,
              FAR struct tm *(*funcp)(FAR const time_t *, int_fast32_t,
                                      FAR struct tm *),
              int_fast32_t offset);
static time_t time2(FAR struct tm *tmp,
              FAR struct tm *(*funcp)(FAR const time_t *,
                                     int_fast32_t, FAR struct tm *),
              int_fast32_t offset, FAR int *okayp);
static time_t time2sub(FAR struct tm *tmp,
              FAR struct tm *(*funcp)(FAR const time_t *,
                                      int_fast32_t, FAR struct tm *),
              int_fast32_t offset, FAR int *okayp, int do_norm_secs);
static FAR struct tm *timesub(FAR const time_t *timep, int_fast32_t offset,
              FAR const struct state_s *sp, FAR struct tm *tmp);
static int  tmcomp(FAR const struct tm *atmp, FAR const struct tm *btmp);
static int_fast32_t transtime(int year, FAR const struct rule_s *rulep,
              int_fast32_t offset);
static int  typesequiv(FAR const struct state_s *sp, int a, int b);
static int  tzload(FAR const char *name, FAR struct state_s *sp,
              int doextend);
static int  tzparse(FAR const char *name, FAR struct state_s *sp,
              int lastditch);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int_fast32_t detzcode(FAR const char *codep)
{
  int_fast32_t result;
  int i;

  result = (codep[0] & 0x80) ? -1 : 0;
  for (i = 0; i < 4; ++i)
    {
      result = (result << 8) | (codep[i] & 0xff);
    }

  return result;
}

static int_fast64_t detzcode64(FAR const char *codep)
{
  int_fast64_t result;
  int i;

  result = (codep[0] & 0x80) ? -1 : 0;
  for (i = 0; i < 8; ++i)
    {
      result = (result * 256) | (codep[i] & 0xff);
    }

  return result;
}

static void settzname(void)
{
  FAR struct state_s *const sp = g_lcl_ptr;
  int i;

  tzname[0] = tzname[1] = (FAR char *)g_wildabbr;
  if (sp == NULL)
    {
      tzname[0] = tzname[1] = (FAR char *)GMT;
      return;
    }

  /* And to get the latest zone names into tzname */

  for (i = 0; i < sp->typecnt; ++i)
    {
      FAR const struct ttinfo_s *const ttisp = &sp->ttis[i];

      tzname[ttisp->tt_isdst] = &sp->chars[ttisp->tt_abbrind];
    }

  for (i = 0; i < sp->timecnt; ++i)
    {
      FAR const struct ttinfo_s *const ttisp = &sp->ttis[sp->types[i]];

      tzname[ttisp->tt_isdst] = &sp->chars[ttisp->tt_abbrind];
    }

  /* Finally, scrub the abbreviations.  First, replace bogus characters. */

  for (i = 0; i < sp->charcnt; ++i)
    {
      if (strchr(TZ_ABBR_CHAR_SET, sp->chars[i]) == NULL)
        {
          sp->chars[i] = TZ_ABBR_ERR_CHAR;
        }
    }

  /* Second, truncate long abbreviations. */

  for (i = 0; i < sp->typecnt; ++i)
    {
      FAR const struct ttinfo_s *const ttisp = &sp->ttis[i];
      FAR char *cp = &sp->chars[ttisp->tt_abbrind];

      if (strlen(cp) > TZ_ABBR_MAX_LEN && strcmp(cp, GRANDPARENTED) != 0)
        {
          *(cp + TZ_ABBR_MAX_LEN) = '\0';
        }
    }
}

static int differ_by_repeat(time_t t1, time_t t0)
{
  if (TYPE_BIT(time_t) - TYPE_SIGNED(time_t) < SECSPERREPEAT_BITS)
    {
      return 0;
    }

  return t1 - t0 == SECSPERREPEAT;
}

static int tzload(FAR const char *name,
                  FAR struct state_s *sp, int doextend)
{
  FAR const char *p;
  int i;
  int fid;
  int stored;
  int nread;

  typedef union
  {
    struct tzhead_s tzhead;
    char buf[2 * sizeof(struct tzhead_s) +
             2 * sizeof *sp +
             4 * TZ_MAX_TIMES];
  } u_t;

  /* Section 4.9.1 of the C standard says that
   * "FILENAME_MAX expands to an integral constant expression
   * that is the size needed for an array of char large enough
   * to hold the longest file name string that the implementation
   * guarantees can be opened."
   */

  union local_storage
  {
    char fullname[FILENAME_MAX + 1];

    /* The main part of the storage for this function.  */

    struct
      {
        u_t u;
        struct state_s st;
      } u;
  };

  FAR char *fullname;
  FAR u_t *up;
  int doaccess;
  FAR union local_storage *lsp;

  lsp = lib_malloc(sizeof *lsp);
  if (!lsp)
    {
      return -1;
    }

  fullname = lsp->fullname;
  up = &lsp->u.u;

  sp->goback = sp->goahead = FALSE;

  if (name == NULL)
    {
      name = TZDEFAULT;
      if (name == NULL)
        {
          goto oops;
        }
    }

  if (name[0] == ':')
    {
      ++name;
    }

  doaccess = name[0] == '/';
  if (!doaccess)
    {
      p = TZDIR;
      if (p == NULL ||
          FILENAME_MAX <= strlen(p) + strlen(name))
        {
          goto oops;
        }

      strcpy(fullname, p);
      strcat(fullname, "/");
      strcat(fullname, name);

      /* Set doaccess if '.' (as in "../") shows up in name.  */

      if (strchr(name, '.'))
        {
          doaccess = TRUE;
        }

      name = fullname;
    }

  if (doaccess && access(name, R_OK) != 0)
    {
      goto oops;
    }

  fid = _NX_OPEN(name, O_RDONLY);
  if (fid < 0)
    {
      goto oops;
    }

  nread = _NX_READ(fid, up->buf, sizeof up->buf);
  if (_NX_CLOSE(fid) < 0 || nread <= 0)
    {
      goto oops;
    }

  for (stored = 4; stored <= 8; stored *= 2)
    {
      int ttisstdcnt;
      int ttisgmtcnt;
      int timecnt;

      ttisstdcnt  = (int)detzcode(up->tzhead.tzh_ttisstdcnt);
      ttisgmtcnt  = (int)detzcode(up->tzhead.tzh_ttisgmtcnt);
      sp->leapcnt = (int)detzcode(up->tzhead.tzh_leapcnt);
      sp->timecnt = (int)detzcode(up->tzhead.tzh_timecnt);
      sp->typecnt = (int)detzcode(up->tzhead.tzh_typecnt);
      sp->charcnt = (int)detzcode(up->tzhead.tzh_charcnt);

      p = up->tzhead.tzh_charcnt + sizeof up->tzhead.tzh_charcnt;
      if (sp->leapcnt < 0 || sp->leapcnt > TZ_MAX_LEAPS ||
          sp->typecnt <= 0 || sp->typecnt > TZ_MAX_TYPES ||
          sp->timecnt < 0 || sp->timecnt > TZ_MAX_TIMES ||
          sp->charcnt < 0 || sp->charcnt > TZ_MAX_CHARS ||
          (ttisstdcnt != sp->typecnt && ttisstdcnt != 0) ||
          (ttisgmtcnt != sp->typecnt && ttisgmtcnt != 0))
        {
          goto oops;
        }

      if (nread - (p - up->buf) < sp->timecnt * stored + /* ats */
          sp->timecnt +                                  /* types */
          sp->typecnt * 6 +                              /* ttinfos */
          sp->charcnt +                                  /* chars */
          sp->leapcnt * (stored + 4) +                   /* lsinfos */
          ttisstdcnt +                                   /* ttisstds */
          ttisgmtcnt)                                    /* ttisgmts */
        {
          goto oops;
        }

      timecnt = 0;
      for (i = 0; i < sp->timecnt; ++i)
        {
          int_fast64_t at = stored == 4 ? detzcode(p) : detzcode64(p);
          sp->types[i] = ((TYPE_SIGNED(time_t)
                           ? g_min_timet <= at : 0 <= at) &&
                           at <= g_max_timet);
          if (sp->types[i])
            {
              if (i && !timecnt && at != g_min_timet)
                {
                  /* Keep the earlier record, but tweak
                   * it so that it starts with the
                   * minimum time_t value.
                   */

                  sp->types[i - 1] = 1;
                  sp->ats[timecnt++] = g_min_timet;
                }

              sp->ats[timecnt++] = at;
            }

          p += stored;
        }

      timecnt = 0;
      for (i = 0; i < sp->timecnt; ++i)
        {
          unsigned char typ = *p++;
          if (sp->typecnt <= typ)
            {
              goto oops;
            }

          if (sp->types[i])
            {
              sp->types[timecnt++] = typ;
            }
        }

      sp->timecnt = timecnt;
      for (i = 0; i < sp->typecnt; ++i)
        {
          FAR struct ttinfo_s *ttisp;

          ttisp = &sp->ttis[i];
          ttisp->tt_gmtoff = detzcode(p);
          p += 4;
          ttisp->tt_isdst = (unsigned char)*p++;
          if (ttisp->tt_isdst != 0 && ttisp->tt_isdst != 1)
            {
              goto oops;
            }

          ttisp->tt_abbrind = (unsigned char)*p++;
          if (ttisp->tt_abbrind < 0 || ttisp->tt_abbrind > sp->charcnt)
            {
              goto oops;
            }
        }

      for (i = 0; i < sp->charcnt; ++i)
        {
          sp->chars[i] = *p++;
        }

      sp->chars[i] = '\0';      /* ensure '\0' at end */
      for (i = 0; i < sp->leapcnt; ++i)
        {
          FAR struct lsinfo_s *lsisp;

          lsisp = &sp->lsis[i];
          lsisp->ls_trans = (stored == 4) ? detzcode(p) : detzcode64(p);
          p += stored;
          lsisp->ls_corr = detzcode(p);
          p += 4;
        }

      for (i = 0; i < sp->typecnt; ++i)
        {
          FAR struct ttinfo_s *ttisp;

          ttisp = &sp->ttis[i];
          if (ttisstdcnt == 0)
            {
              ttisp->tt_ttisstd = FALSE;
            }
          else
            {
              ttisp->tt_ttisstd = *p++;
              if (ttisp->tt_ttisstd != TRUE && ttisp->tt_ttisstd != FALSE)
                {
                  goto oops;
                }
            }
        }

      for (i = 0; i < sp->typecnt; ++i)
        {
          FAR struct ttinfo_s *ttisp;

          ttisp = &sp->ttis[i];
          if (ttisgmtcnt == 0)
            {
              ttisp->tt_ttisgmt = FALSE;
            }
          else
            {
              ttisp->tt_ttisgmt = *p++;
              if (ttisp->tt_ttisgmt != TRUE && ttisp->tt_ttisgmt != FALSE)
                {
                  goto oops;
                }
            }
        }

      /* If this is an old file, we're done. */

      if (up->tzhead.tzh_version[0] == '\0')
        {
          break;
        }

      nread -= p - up->buf;
      for (i = 0; i < nread; ++i)
        {
          up->buf[i] = p[i];
        }

      /* If this is a signed narrow time_t system, we're done. */

      if (TYPE_SIGNED(time_t) && stored >= (int)sizeof(time_t))
        {
          break;
        }
    }

  if (doextend && nread > 2 &&
      up->buf[0] == '\n' && up->buf[nread - 1] == '\n' &&
      sp->typecnt + 2 <= TZ_MAX_TYPES)
    {
      FAR struct state_s *ts = &lsp->u.st;
      int result;

      up->buf[nread - 1] = '\0';
      result = tzparse(&up->buf[1], ts, FALSE);
      if (result == 0 && ts->typecnt == 2 &&
          sp->charcnt + ts->charcnt <= TZ_MAX_CHARS)
        {
          for (i = 0; i < 2; ++i)
            {
              ts->ttis[i].tt_abbrind += sp->charcnt;
            }

          for (i = 0; i < ts->charcnt; ++i)
            {
              sp->chars[sp->charcnt++] = ts->chars[i];
            }

          i = 0;
          while (i < ts->timecnt && ts->ats[i] <= sp->ats[sp->timecnt - 1])
            {
              ++i;
            }

          while (i < ts->timecnt && sp->timecnt < TZ_MAX_TIMES)
            {
              sp->ats[sp->timecnt] = ts->ats[i];
              sp->types[sp->timecnt] = sp->typecnt + ts->types[i];
              ++sp->timecnt;
              ++i;
            }

          sp->ttis[sp->typecnt++] = ts->ttis[0];
          sp->ttis[sp->typecnt++] = ts->ttis[1];
        }
    }

  if (sp->timecnt > 1)
    {
      for (i = 1; i < sp->timecnt; ++i)
        {
          if (typesequiv(sp, sp->types[i], sp->types[0]) &&
              differ_by_repeat(sp->ats[i], sp->ats[0]))
            {
              sp->goback = TRUE;
              break;
            }
        }

      for (i = sp->timecnt - 2; i >= 0; --i)
        {
          if (typesequiv(sp, sp->types[sp->timecnt - 1],
                         sp->types[i]) &&
              differ_by_repeat(sp->ats[sp->timecnt - 1], sp->ats[i]))
            {
              sp->goahead = TRUE;
              break;
            }
        }
    }

  /* If type 0 is is unused in transitions, it's the type to use for early
   * times.
   */

  for (i = 0; i < sp->typecnt; ++i)
    {
      if (sp->types[i] == 0)
        {
          break;
        }
    }

  i = (i >= sp->typecnt) ? 0 : -1;

  /* Absent the above, if there are transition times and the first
   * transition is to a daylight time find the standard type less than and
   * closest to the type of the first transition.
   */

  if (i < 0 && sp->timecnt > 0 && sp->ttis[sp->types[0]].tt_isdst)
    {
      i = sp->types[0];
      while (--i >= 0)
        {
          if (!sp->ttis[i].tt_isdst)
            {
              break;
            }
        }
    }

  /* If no result yet, find the first standard type. If there is none, punt
   * to type zero.
   */

  if (i < 0)
    {
      i = 0;
      while (sp->ttis[i].tt_isdst)
        {
          if (++i >= sp->typecnt)
            {
              i = 0;
              break;
            }
        }
    }

  sp->defaulttype = i;
  lib_free(lsp);
  return 0;

oops:
  lib_free(lsp);
  return -1;
}

static int typesequiv(FAR const struct state_s *sp, int a, int b)
{
  int result;

  if (sp == NULL || a < 0 || a >= sp->typecnt || b < 0 || b >= sp->typecnt)
    {
      result = FALSE;
    }
  else
    {
      FAR const struct ttinfo_s *ap = &sp->ttis[a];
      FAR const struct ttinfo_s *bp = &sp->ttis[b];

      result = ap->tt_gmtoff == bp->tt_gmtoff &&
        ap->tt_isdst == bp->tt_isdst &&
        ap->tt_ttisstd == bp->tt_ttisstd &&
        ap->tt_ttisgmt == bp->tt_ttisgmt &&
        strcmp(&sp->chars[ap->tt_abbrind], &sp->chars[bp->tt_abbrind]) == 0;
    }

  return result;
}

/* Given a pointer into a time zone string, scan until a character that is
 * not a valid character in a zone name is found. Return a pointer to that
 * character.
 */

static FAR const char *getzname(FAR const char *strp)
{
  char c;

  while ((c = *strp) != '\0' && !is_digit(c) && c != ',' &&
         c != '-' && c != '+')
    {
      ++strp;
    }

  return strp;
}

/* Given a pointer into an extended time zone string, scan until the ending
 * delimiter of the zone name is located. Return a pointer to the delimiter.
 *
 * As with getzname above, the legal character set is actually quite
 * restricted, with other characters producing undefined results.
 * We don't do any checking here; checking is done later in common-case code.
 */

static FAR const char *getqzname(FAR const char *strp, int delim)
{
  int c;

  while ((c = *strp) != '\0' && c != delim)
    {
      ++strp;
    }

  return strp;
}

/* Given a pointer into a time zone string, extract a number from that
 * string.  Check that the number is within a specified range; if it is not,
 * return NULL.  Otherwise, return a pointer to the first character not part
 * of the number.
 */

static FAR const char *getnum(FAR const char *strp, FAR int *nump,
                              int min, int max)
{
  char c;
  int num;

  if (strp == NULL || !is_digit(c = *strp))
    {
      return NULL;
    }

  num = 0;
  do
    {
      num = num * 10 + (c - '0');
      if (num > max)
        {
          return NULL; /* illegal value */
        }

      c = *++strp;
    }
  while (is_digit(c));

  if (num < min)
    {
      return NULL; /* illegal value */
    }

  *nump = num;
  return strp;
}

/* Given a pointer into a time zone string, extract a number of seconds,
 * in hh[:mm[:ss]] form, from the string.
 * If any error occurs, return NULL.
 * Otherwise, return a pointer to the first character not part of the number
 * of seconds.
 */

static FAR const char *getsecs(FAR const char *strp,
                               FAR int_fast32_t *secsp)
{
  int num;

  /* 'HOURSPERDAY * DAYSPERWEEK - 1' allows quasi-Posix rules like
   * "M10.4.6/26", which does not conform to Posix,
   * but which specifies the equivalent of
   * "02:00 on the first Sunday on or after 23 Oct".
   */

  strp = getnum(strp, &num, 0, HOURSPERDAY * DAYSPERWEEK - 1);
  if (strp == NULL)
    {
      return NULL;
    }

  *secsp = num * (int_fast32_t)SECSPERHOUR;
  if (*strp == ':')
    {
      ++strp;
      strp = getnum(strp, &num, 0, MINSPERHOUR - 1);
      if (strp == NULL)
        {
          return NULL;
        }

      *secsp += num * SECSPERMIN;
      if (*strp == ':')
        {
          ++strp;

          /* 'SECSPERMIN' allows for leap seconds.  */

          strp = getnum(strp, &num, 0, SECSPERMIN);
          if (strp == NULL)
            {
              return NULL;
            }

          *secsp += num;
        }
    }

  return strp;
}

/* Given a pointer into a time zone string, extract an offset, in
 * [+-]hh[:mm[:ss]] form, from the string.
 * If any error occurs, return NULL.
 * Otherwise, return a pointer to the first character not part of the time.
 */

static FAR const char *getoffset(FAR const char *strp,
                                 FAR int_fast32_t *offsetp)
{
  int neg = 0;

  if (*strp == '-')
    {
      neg = 1;
      ++strp;
    }
  else if (*strp == '+')
    {
      ++strp;
    }

  strp = getsecs(strp, offsetp);
  if (strp == NULL)
    {
      return NULL; /* illegal time */
    }

  if (neg)
    {
      *offsetp = -*offsetp;
    }

  return strp;
}

/* Given a pointer into a time zone string, extract a rule in the form
 * date[/time]. See POSIX section 8 for the format of "date" and "time".
 * If a valid rule is not found, return NULL.
 * Otherwise, return a pointer to the first character not part of the rule.
 */

static FAR const char *getrule(FAR const char *strp,
                               FAR struct rule_s *rulep)
{
  if (*strp == 'J')
    {
      /* Julian day */

      rulep->r_type = JULIAN_DAY;
      ++strp;
      strp = getnum(strp, &rulep->r_day, 1, DAYSPERNYEAR);
    }
  else if (*strp == 'M')
    {
      /* Month, week, day. */

      rulep->r_type = MONTH_NTH_DAY_OF_WEEK;
      ++strp;
      strp = getnum(strp, &rulep->r_mon, 1, MONSPERYEAR);
      if (strp == NULL)
        {
          return NULL;
        }

      if (*strp++ != '.')
        {
          return NULL;
        }

      strp = getnum(strp, &rulep->r_week, 1, 5);
      if (strp == NULL)
        {
          return NULL;
        }

      if (*strp++ != '.')
        {
          return NULL;
        }

      strp = getnum(strp, &rulep->r_day, 0, DAYSPERWEEK - 1);
    }
  else if (is_digit(*strp))
    {
      /* Day of year */

      rulep->r_type = DAY_OF_YEAR;
      strp = getnum(strp, &rulep->r_day, 0, DAYSPERLYEAR - 1);
    }
  else
    {
      return NULL; /* invalid format */
    }

  if (strp == NULL)
    {
      return NULL;
    }

  if (*strp == '/')
    {
      /* Time specified */

      ++strp;
      strp = getoffset(strp, &rulep->r_time);
    }
  else
    {
      rulep->r_time = 2 * SECSPERHOUR; /* default = 2:00:00 */
    }

  return strp;
}

/* Given a year, a rule, and the offset from UT at the time that rule takes
 * effect, calculate the year-relative time that rule takes effect.
 */

static int_fast32_t transtime(int year,
                              FAR const struct rule_s *rulep,
                              int_fast32_t offset)
{
  int leapyear;
  int_fast32_t value;
  int i;
  int d;
  int m1;
  int yy0;
  int yy1;
  int yy2;
  int dow;

  value = 0;
  leapyear = isleap(year);
  switch (rulep->r_type)
    {
    case JULIAN_DAY:

      /* Jn - Julian day, 1 == January 1, 60 == March 1 even in leap
       * years.
       * In non-leap years, or if the day number is 59 or less, just
       * add SECSPERDAY times the day number-1 to the time of
       * January 1, midnight, to get the day.
       */

      value = (rulep->r_day - 1) * SECSPERDAY;
      if (leapyear && rulep->r_day >= 60)
        {
          value += SECSPERDAY;
        }

      break;

    case DAY_OF_YEAR:

      /* n - day of year.
       * Just add SECSPERDAY times the day number to the time of
       * January 1, midnight, to get the day.
       */

      value = rulep->r_day * SECSPERDAY;
      break;

    case MONTH_NTH_DAY_OF_WEEK:

      /* Mm.n.d - nth "dth day" of month m */

      /* Use Zeller's Congruence to get day-of-week of first day of
       * month.
       */

      m1  = (rulep->r_mon + 9) % 12 + 1;
      yy0 = (rulep->r_mon <= 2) ? (year - 1) : year;
      yy1 = yy0 / 100;
      yy2 = yy0 % 100;
      dow = ((26 * m1 - 2) / 10 + 1 + yy2 + yy2 / 4 + yy1 / 4 - 2 * yy1) % 7;
      if (dow < 0)
        {
          dow += DAYSPERWEEK;
        }

      /* "dow" is the day-of-week of the first day of the month. Get
       * the day-of-month (zero-origin) of the first "dow" day of the
       * month.
       */

      d = rulep->r_day - dow;
      if (d < 0)
        {
          d += DAYSPERWEEK;
        }

      for (i = 1; i < rulep->r_week; ++i)
        {
          if (d + DAYSPERWEEK >= g_mon_lengths[leapyear][rulep->r_mon - 1])
            {
              break;
            }

          d += DAYSPERWEEK;
        }

      /* "d" is the day-of-month (zero-origin) of the day we want */

      value = d * SECSPERDAY;
      for (i = 0; i < rulep->r_mon - 1; ++i)
        {
          value += g_mon_lengths[leapyear][i] * SECSPERDAY;
        }
      break;
    }

  /* "value" is the year-relative time of 00:00:00 UT on the day in
   * question. To get the year-relative time of the specified local
   * time on that day, add the transition time and the current offset
   * from UT.
   */

  return value + rulep->r_time + offset;
}

/* Given a POSIX section 8-style TZ string, fill in the rule tables as
 * appropriate.
 */

static int tzparse(FAR const char *name, FAR struct state_s *sp,
                   int lastditch)
{
  FAR const char *stdname;
  FAR const char *dstname;
  size_t stdlen;
  size_t dstlen;
  int_fast32_t stdoffset;
  int_fast32_t dstoffset;
  FAR char *cp;
  int load_result;
  static struct ttinfo_s zttinfo;

  stdname = name;
  if (lastditch)
    {
      stdlen = strlen(name);    /* length of standard zone name */
      name += stdlen;
      if (stdlen >= sizeof sp->chars)
        {
          stdlen = (sizeof sp->chars) - 1;
        }

      stdoffset = 0;
    }
  else
    {
      if (*name == '<')
        {
          name++;
          stdname = name;
          name = getqzname(name, '>');
          if (*name != '>')
            {
              return -1;
            }

          stdlen = name - stdname;
          name++;
        }
      else
        {
          name = getzname(name);
          stdlen = name - stdname;
        }

      if (*name == '\0')
        {
          return -1;
        }

      name = getoffset(name, &stdoffset);
      if (name == NULL)
        {
          return -1;
        }
    }

  load_result = tzload(TZDEFRULES, sp, FALSE);
  if (load_result != 0)
    {
      sp->leapcnt = 0; /* so, we're off a little */
    }

  if (*name != '\0')
    {
      if (*name == '<')
        {
          dstname = ++name;
          name = getqzname(name, '>');
          if (*name != '>')
            {
              return -1;
            }

          dstlen = name - dstname;
          name++;
        }
      else
        {
          dstname = name;
          name = getzname(name);
          dstlen = name - dstname;      /* length of DST zone name */
        }

      if (*name != '\0' && *name != ',' && *name != ';')
        {
          name = getoffset(name, &dstoffset);
          if (name == NULL)
            {
              return -1;
            }
        }
      else
        {
          dstoffset = stdoffset - SECSPERHOUR;
        }

      if (*name == '\0' && load_result != 0)
        {
          name = TZDEFRULESTRING;
        }

      if (*name == ',' || *name == ';')
        {
          struct rule_s start;
          struct rule_s end;
          int year;
          int yearlim;
          int timecnt;
          time_t janfirst;

          ++name;
          if ((name = getrule(name, &start)) == NULL)
            {
              return -1;
            }

          if (*name++ != ',')
            {
              return -1;
            }

          if ((name = getrule(name, &end)) == NULL)
            {
              return -1;
            }

          if (*name != '\0')
            {
              return -1;
            }

          sp->typecnt = 2;      /* standard time and DST */

          /* Two transitions per year, from EPOCH_YEAR forward */

          sp->ttis[0] = sp->ttis[1] = zttinfo;
          sp->ttis[0].tt_gmtoff = -dstoffset;
          sp->ttis[0].tt_isdst = 1;
          sp->ttis[0].tt_abbrind = stdlen + 1;
          sp->ttis[1].tt_gmtoff = -stdoffset;
          sp->ttis[1].tt_isdst = 0;
          sp->ttis[1].tt_abbrind = 0;
          sp->defaulttype = 0;
          timecnt = 0;
          janfirst = 0;
          yearlim = EPOCH_YEAR + YEARSPERREPEAT;
          for (year = EPOCH_YEAR; year < yearlim; year++)
            {
              int_fast32_t
                starttime  = transtime(year, &start, stdoffset);
              int_fast32_t
                endtime    = transtime(year, &end, dstoffset);
              int_fast32_t
                yearsecs   = (g_year_lengths[isleap(year)] * SECSPERDAY);

              int reversed = endtime < starttime;
              if (reversed)
                {
                  int_fast32_t swap = starttime;
                  starttime = endtime;
                  endtime = swap;
                }

              if (reversed ||
                   (starttime < endtime &&
                    (endtime - starttime <
                     (yearsecs + (stdoffset - dstoffset)))))
                {
                  if (TZ_MAX_TIMES - 2 < timecnt)
                    {
                      break;
                    }

                  yearlim = year + YEARSPERREPEAT + 1;
                  sp->ats[timecnt] = janfirst;
                  if (increment_overflow_time(&sp->ats[timecnt], starttime))
                    {
                      break;
                    }

                  sp->types[timecnt++] = reversed;
                  sp->ats[timecnt] = janfirst;
                  if (increment_overflow_time(&sp->ats[timecnt], endtime))
                    {
                      break;
                    }

                  sp->types[timecnt++] = !reversed;
                }

              if (increment_overflow_time(&janfirst, yearsecs))
                {
                  break;
                }
            }

          sp->timecnt = timecnt;
          if (!timecnt)
            {
              sp->typecnt = 1; /* Perpetual DST.  */
            }
        }
      else
        {
          int_fast32_t theirstdoffset;
          int_fast32_t theiroffset;
          int i;
          int j;

          if (*name != '\0')
            {
              return -1;
            }

          /* Initial value of theirstdoffset */

          theirstdoffset = 0;
          for (i = 0; i < sp->timecnt; ++i)
            {
              j = sp->types[i];
              if (!sp->ttis[j].tt_isdst)
                {
                  theirstdoffset = -sp->ttis[j].tt_gmtoff;
                  break;
                }
            }

          /* Initially we're assumed to be in standard time */

          theiroffset = theirstdoffset;

          /* Now juggle transition times and types
           * tracking offsets as you do.
           */

          for (i = 0; i < sp->timecnt; ++i)
            {
              j = sp->types[i];
              sp->types[i] = sp->ttis[j].tt_isdst;
              if (sp->ttis[j].tt_ttisgmt)
                {
                  /* No adjustment to transition time */
                }
              else
                {
                  /* If summer time is in effect, and the
                   * transition time was not specified as
                   * standard time, add the summer time
                   * offset to the transition time;
                   * otherwise, add the standard time
                   * offset to the transition time.
                   */

                  sp->ats[i] += stdoffset - theirstdoffset;
                }

              theiroffset = -sp->ttis[j].tt_gmtoff;
              if (!sp->ttis[j].tt_isdst)
                {
                  theirstdoffset = theiroffset;
                }
            }

          /* Finally, fill in ttis */

          sp->ttis[0] = sp->ttis[1] = zttinfo;
          sp->ttis[0].tt_gmtoff = -stdoffset;
          sp->ttis[0].tt_isdst = FALSE;
          sp->ttis[0].tt_abbrind = 0;
          sp->ttis[1].tt_gmtoff = -dstoffset;
          sp->ttis[1].tt_isdst = TRUE;
          sp->ttis[1].tt_abbrind = stdlen + 1;
          sp->typecnt = 2;
          sp->defaulttype = 0;
        }
    }
  else
    {
      dstlen = 0;
      sp->typecnt = 1;          /* only standard time */
      sp->timecnt = 0;
      sp->ttis[0] = zttinfo;
      sp->ttis[0].tt_gmtoff = -stdoffset;
      sp->ttis[0].tt_isdst = 0;
      sp->ttis[0].tt_abbrind = 0;
      sp->defaulttype = 0;
    }

  sp->charcnt = stdlen + 1;
  if (dstlen != 0)
    {
      sp->charcnt += dstlen + 1;
    }

  if ((size_t)sp->charcnt > sizeof sp->chars)
    {
      return -1;
    }

  cp = sp->chars;
  stdlen += 1;
  strlcpy(cp, stdname, stdlen);
  cp += stdlen;
  if (dstlen != 0)
    {
      strlcpy(cp, dstname, dstlen + 1);
    }

  return 0;
}

static void gmtload(FAR struct state_s *sp)
{
  if (tzload(GMT, sp, TRUE) != 0)
    {
      tzparse(GMT, sp, TRUE);
    }
}

/* A non-static declaration of tzsetwall in a system header file
 * may cause a warning about this upcoming static declaration...
 */

static void tzsetwall(void)
{
  if (g_lcl_isset < 0)
    {
      return;
    }

  if (g_lcl_ptr == NULL)
    {
      g_lcl_ptr = lib_malloc(sizeof *g_lcl_ptr);
    }

  if (g_lcl_ptr != NULL && tzload(NULL, g_lcl_ptr, TRUE) != 0)
    {
      gmtload(g_lcl_ptr);
    }

  settzname();

  g_lcl_isset = -1;
}

/* The easy way to behave "as if no library function calls" localtime
 * is to not call it, so we drop its guts into "localsub", which can be
 * freely called. (And no, the PANS doesn't require the above behavior,
 * but it *is* desirable.)
 *
 * The unused offset argument is for the benefit of mktime variants.
 */

static FAR struct tm *localsub(FAR const time_t *timep,
                               int_fast32_t offset, FAR struct tm *tmp)
{
  FAR struct state_s *sp;
  FAR const struct ttinfo_s *ttisp;
  int i;
  FAR struct tm *result;
  const time_t t = *timep;

  sp = g_lcl_ptr;
  if (sp == NULL)
    {
      return gmtsub(timep, offset, tmp);
    }

  if ((sp->goback && t < sp->ats[0]) ||
      (sp->goahead && t > sp->ats[sp->timecnt - 1]))
    {
      time_t newt = t;
      time_t seconds;
      time_t years;

      if (t < sp->ats[0])
        {
          seconds = sp->ats[0] - t;
        }
      else
        {
          seconds = t - sp->ats[sp->timecnt - 1];
        }

      --seconds;
      years = (seconds / SECSPERREPEAT + 1) * YEARSPERREPEAT;
      seconds = years * AVGSECSPERYEAR;
      if (t < sp->ats[0])
        {
          newt += seconds;
        }
      else
        {
          newt -= seconds;
        }

      if (newt < sp->ats[0] || newt > sp->ats[sp->timecnt - 1])
        {
          return NULL; /* "cannot happen" */
        }

      result = localsub(&newt, offset, tmp);
      if (result == tmp)
        {
          time_t newy;

          newy = tmp->tm_year;
          if (t < sp->ats[0])
            {
              newy -= years;
            }
          else
            {
              newy += years;
            }

          tmp->tm_year = newy;
          if (tmp->tm_year != newy)
            {
              return NULL;
            }
        }

      return result;
    }

  if (sp->timecnt == 0 || t < sp->ats[0])
    {
      i = sp->defaulttype;
    }
  else
    {
      int lo = 1;
      int hi = sp->timecnt;

      while (lo < hi)
        {
          int mid = (lo + hi) >> 1;

          if (t < sp->ats[mid])
            {
              hi = mid;
            }
          else
            {
              lo = mid + 1;
            }
        }

      i = (int)sp->types[lo - 1];
    }

  ttisp = &sp->ttis[i];

  /* To get (wrong) behavior that's compatible with System V Release 2.0
   * you'd replace the statement below with
   *    t += ttisp->tt_gmtoff;
   *    timesub(&t, 0L, sp, tmp);
   */

  result = timesub(&t, ttisp->tt_gmtoff, sp, tmp);
  tmp->tm_isdst = ttisp->tt_isdst;
  tzname[tmp->tm_isdst] = &sp->chars[ttisp->tt_abbrind];
  tmp->tm_zone = tzname[tmp->tm_isdst];

  return result;
}

/* gmtsub is to gmtime as localsub is to localtime */

static FAR struct tm *gmtsub(FAR const time_t *timep,
                             int_fast32_t offset, FAR struct tm *tmp)
{
  if (!g_gmt_isset)
    {
#ifndef __KERNEL__
      if (up_interrupt_context() || (sched_idletask() && OSINIT_IDLELOOP()))
        {
          return NULL;
        }
#endif

      nxrmutex_lock(&g_gmt_lock);

      if (!g_gmt_isset)
        {
          g_gmt_ptr = lib_malloc(sizeof *g_gmt_ptr);
          if (g_gmt_ptr != NULL)
            {
              gmtload(g_gmt_ptr);
              g_gmt_isset = 1;
            }
        }

      nxrmutex_unlock(&g_gmt_lock);
    }

  tmp->tm_zone = GMT;
  return timesub(timep, offset, g_gmt_ptr, tmp);
}

/* Return the number of leap years through the end of the given year
 * where, to make the math easy, the answer for year zero is defined as zero.
 */

static int leaps_thru_end_of(int y)
{
  return (y >= 0) ? (y / 4 - y / 100 + y / 400) :
    -(leaps_thru_end_of(-(y + 1)) + 1);
}

static FAR struct tm *timesub(FAR const time_t *timep,
                              int_fast32_t offset,
                              FAR const struct state_s *sp,
                              FAR struct tm *tmp)
{
  FAR const struct lsinfo_s *lp;
  time_t tdays;
  int idays;           /* unsigned would be so 2003 */
  int_fast32_t rem;
  int y;
  FAR const int *ip;
  int_fast32_t corr;
  int hit;
  int i;

  corr = 0;
  hit = 0;
  i = (sp == NULL) ? 0 : sp->leapcnt;
  while (--i >= 0)
    {
      lp = &sp->lsis[i];
      if (*timep >= lp->ls_trans)
        {
          if (*timep == lp->ls_trans)
            {
              hit = ((i == 0 && lp->ls_corr > 0) ||
                     lp->ls_corr > sp->lsis[i - 1].ls_corr);
              if (hit)
                {
                  while (i > 0 &&
                         sp->lsis[i].ls_trans ==
                         sp->lsis[i - 1].ls_trans + 1 &&
                         sp->lsis[i].ls_corr == sp->lsis[i - 1].ls_corr + 1)
                    {
                      ++hit;
                      --i;
                    }
                }
            }

          corr = lp->ls_corr;
          break;
        }
    }

  y = EPOCH_YEAR;
  tdays = *timep / SECSPERDAY;
  rem = *timep - tdays * SECSPERDAY;
  while (tdays >= g_year_lengths[isleap(y)])
    {
      int newy;
      time_t tdelta;
      int idelta;
      int leapdays;

      tdelta = tdays / DAYSPERLYEAR;
      if (!((!TYPE_SIGNED(time_t) || INT_MIN <= tdelta) &&
          tdelta <= INT_MAX))
        {
          return NULL;
        }

      idelta = tdelta;
      if (idelta == 0)
        {
          idelta = (tdays < 0) ? -1 : 1;
        }

      newy = y;
      if (increment_overflow(&newy, idelta))
        {
          return NULL;
        }

      leapdays = leaps_thru_end_of(newy - 1) - leaps_thru_end_of(y - 1);
      tdays -= ((time_t)newy - y) * DAYSPERNYEAR;
      tdays -= leapdays;
      y = newy;
    }

    {
      int_fast32_t seconds;

      seconds = tdays * SECSPERDAY;
      tdays = seconds / SECSPERDAY;
      rem += seconds - tdays * SECSPERDAY;
    }

  /* Given the range, we can now fearlessly cast */

  idays = tdays;
  rem += offset - corr;
  while (rem < 0)
    {
      rem += SECSPERDAY;
      --idays;
    }

  while (rem >= SECSPERDAY)
    {
      rem -= SECSPERDAY;
      ++idays;
    }

  while (idays < 0)
    {
      if (increment_overflow(&y, -1))
        {
          return NULL;
        }

      idays += g_year_lengths[isleap(y)];
    }

  while (idays >= g_year_lengths[isleap(y)])
    {
      idays -= g_year_lengths[isleap(y)];
      if (increment_overflow(&y, 1))
        {
          return NULL;
        }
    }

  tmp->tm_year = y;
  if (increment_overflow(&tmp->tm_year, -TM_YEAR_BASE))
    {
      return NULL;
    }

  tmp->tm_yday = idays;

  /* The "extra" mods below avoid overflow problems */

  tmp->tm_wday = EPOCH_WDAY +
    ((y - EPOCH_YEAR) % DAYSPERWEEK) *
    (DAYSPERNYEAR % DAYSPERWEEK) +
    leaps_thru_end_of(y - 1) - leaps_thru_end_of(EPOCH_YEAR - 1) + idays;
  tmp->tm_wday %= DAYSPERWEEK;
  if (tmp->tm_wday < 0)
    {
      tmp->tm_wday += DAYSPERWEEK;
    }

  tmp->tm_hour = (int)(rem / SECSPERHOUR);
  rem %= SECSPERHOUR;
  tmp->tm_min = (int)(rem / SECSPERMIN);

  /* A positive leap second requires a special
   * representation. This uses "... ??:59:60" et seq.
   */

  tmp->tm_sec = (int)(rem % SECSPERMIN) + hit;
  ip = g_mon_lengths[isleap(y)];
  for (tmp->tm_mon = 0; idays >= ip[tmp->tm_mon]; ++(tmp->tm_mon))
    {
      idays -= ip[tmp->tm_mon];
    }

  tmp->tm_mday = (int)(idays + 1);
  tmp->tm_isdst = 0;
  tmp->tm_gmtoff = offset;

  return tmp;
}

/* Adapted from code provided by Robert Elz, who writes:
 *    The "best" way to do mktime I think is based on an idea of Bob
 *    Kridle's (so its said...) from a long time ago.
 *    It does a binary search of the time_t space. Since time_t's are
 *    just 32 bits, its a max of 32 iterations (even at 64 bits it
 *    would still be very reasonable).
 */

/* Normalize logic courtesy Paul Eggert */

static int increment_overflow(FAR int *ip, int j)
{
  const int i = *ip;

  /* If i >= 0 there can only be overflow if i + j > INT_MAX
   * or if j > INT_MAX - i; given i >= 0, INT_MAX - i cannot overflow.
   * If i < 0 there can only be overflow if i + j < INT_MIN
   * or if j < INT_MIN - i; given i < 0, INT_MIN - i cannot overflow.
   */

  if ((i >= 0) ? (j > INT_MAX - i) : (j < INT_MIN - i))
    {
      return TRUE;
    }

  *ip += j;
  return FALSE;
}

static int increment_overflow32(FAR int_fast32_t *lp, int m)
{
  const int_fast32_t l = *lp;

  if ((l >= 0) ? (m > INT_FAST32_MAX - l) : (m < INT_FAST32_MIN - l))
    {
      return TRUE;
    }

  *lp += m;
  return FALSE;
}

static int increment_overflow_time(FAR time_t *tp, int_fast32_t j)
{
  /* This is like
   * 'if (! (g_min_timet <= *tp + j && *tp + j <= g_max_timet)) ...',
   * except that it does the right thing even if *tp + j would overflow.
   */

  if (!(j < 0
        ? (TYPE_SIGNED(time_t) ? g_min_timet - j <= *tp : -1 - j < *tp)
        : *tp <= g_max_timet - j))
    {
      return TRUE;
    }

  *tp += j;
  return FALSE;
}

static int normalize_overflow(FAR int *tensptr,
                              FAR int *unitsptr, int base)
{
  int tensdelta;

  tensdelta = (*unitsptr >= 0) ?
    (*unitsptr / base) : (-1 - (-1 - *unitsptr) / base);
  *unitsptr -= tensdelta * base;
  return increment_overflow(tensptr, tensdelta);
}

static int normalize_overflow32(FAR int_fast32_t *tensptr,
                                FAR int *unitsptr, int base)
{
  int tensdelta;

  tensdelta = (*unitsptr >= 0) ?
    (*unitsptr / base) : (-1 - (-1 - *unitsptr) / base);
  *unitsptr -= tensdelta * base;
  return increment_overflow32(tensptr, tensdelta);
}

static int tmcomp(FAR const struct tm *atmp, FAR const struct tm *btmp)
{
  int result;

  if (atmp->tm_year != btmp->tm_year)
    {
      return atmp->tm_year < btmp->tm_year ? -1 : 1;
    }

  if ((result = (atmp->tm_mon - btmp->tm_mon)) == 0 &&
      (result = (atmp->tm_mday - btmp->tm_mday)) == 0 &&
      (result = (atmp->tm_hour - btmp->tm_hour)) == 0 &&
      (result = (atmp->tm_min - btmp->tm_min)) == 0)
    {
      result = atmp->tm_sec - btmp->tm_sec;
    }

  return result;
}

static time_t time2sub(FAR struct tm *tmp,
                       FAR struct tm *(*funcp)(FAR const time_t *,
                       int_fast32_t, FAR struct tm *),
                       int_fast32_t offset, FAR int *okayp,
                       int do_norm_secs)
{
  FAR const struct state_s *sp;
  int dir;
  int i;
  int j;
  int saved_seconds;
  int_fast32_t li;
  time_t lo;
  time_t hi;
  int_fast32_t y;
  time_t newt;
  time_t t;
  struct tm yourtm;
  struct tm mytm;

  *okayp = FALSE;
  yourtm = *tmp;
  if (do_norm_secs)
    {
      if (normalize_overflow(&yourtm.tm_min, &yourtm.tm_sec, SECSPERMIN))
        {
          return -1;
        }
    }

  if (normalize_overflow(&yourtm.tm_hour, &yourtm.tm_min, MINSPERHOUR))
    {
      return -1;
    }

  if (normalize_overflow(&yourtm.tm_mday, &yourtm.tm_hour, HOURSPERDAY))
    {
      return -1;
    }

  y = yourtm.tm_year;
  if (normalize_overflow32(&y, &yourtm.tm_mon, MONSPERYEAR))
    {
      return -1;
    }

  /* Turn y into an actual year number for now.
   * It is converted back to an offset from TM_YEAR_BASE later.
   */

  if (increment_overflow32(&y, TM_YEAR_BASE))
    {
      return -1;
    }

  while (yourtm.tm_mday <= 0)
    {
      if (increment_overflow32(&y, -1))
        {
          return -1;
        }

      li = y + (1 < yourtm.tm_mon);
      yourtm.tm_mday += g_year_lengths[isleap(li)];
    }

  while (yourtm.tm_mday > DAYSPERLYEAR)
    {
      li = y + (1 < yourtm.tm_mon);
      yourtm.tm_mday -= g_year_lengths[isleap(li)];
      if (increment_overflow32(&y, 1))
        {
          return -1;
        }
    }

  for (; ; )
    {
      i = g_mon_lengths[isleap(y)][yourtm.tm_mon];
      if (yourtm.tm_mday <= i)
        {
          break;
        }

      yourtm.tm_mday -= i;
      if (++yourtm.tm_mon >= MONSPERYEAR)
        {
          yourtm.tm_mon = 0;
          if (increment_overflow32(&y, 1))
            {
              return -1;
            }
        }
    }

  if (increment_overflow32(&y, -TM_YEAR_BASE))
    {
      return -1;
    }

  yourtm.tm_year = y;
  if (yourtm.tm_year != y)
    {
      return -1;
    }

  if (yourtm.tm_sec >= 0 && yourtm.tm_sec < SECSPERMIN)
    {
      saved_seconds = 0;
    }
  else if (y + TM_YEAR_BASE < EPOCH_YEAR)
    {
      /* We can't set tm_sec to 0, because that might push the
       * time below the minimum representable time.
       * Set tm_sec to 59 instead.
       * This assumes that the minimum representable time is
       * not in the same minute that a leap second was deleted from,
       * which is a safer assumption than using 58 would be.
       */

      if (increment_overflow(&yourtm.tm_sec, 1 - SECSPERMIN))
        {
          return -1;
        }

      saved_seconds = yourtm.tm_sec;
      yourtm.tm_sec = SECSPERMIN - 1;
    }
  else
    {
      saved_seconds = yourtm.tm_sec;
      yourtm.tm_sec = 0;
    }

  /* Do a binary search (this works whatever time_t's type is) */

  if (!TYPE_SIGNED(time_t))
    {
      lo = 0;
      hi = lo - 1;
    }
  else
    {
      lo = 1;
      for (i = 0; i < (int)TYPE_BIT(time_t) - 1; ++i)
        {
          lo *= 2;
        }

      hi = -(lo + 1);
    }

  for (; ; )
    {
      t = lo / 2 + hi / 2;
      if (t < lo)
        {
          t = lo;
        }
      else if (t > hi)
        {
          t = hi;
        }

      if ((*funcp) (&t, offset, &mytm) == NULL)
        {
          /* Assume that t is too extreme to be represented in
           * a struct tm; arrange things so that it is less
           * extreme on the next pass.
           */

          dir = (t > 0) ? 1 : -1;
        }
      else
        {
          dir = tmcomp(&mytm, &yourtm);
        }

      if (dir != 0)
        {
          if (t == lo)
            {
              if (t == g_max_timet)
                {
                  return -1;
                }

              ++t;
              ++lo;
            }
          else if (t == hi)
            {
              if (t == g_min_timet)
                {
                  return -1;
                }

              --t;
              --hi;
            }

          if (lo > hi)
            {
              return -1;
            }

          if (dir > 0)
            {
              hi = t;
            }
          else
            {
              lo = t;
            }

          continue;
        }

      if (yourtm.tm_isdst < 0 || mytm.tm_isdst == yourtm.tm_isdst)
        {
          break;
        }

      /* Right time, wrong type.
       * Hunt for right time, right type.
       * It's okay to guess wrong since the guess
       * gets checked.
       */

      sp = (FAR const struct state_s *)
        ((funcp == localsub) ? g_lcl_ptr : g_gmt_ptr);
      if (sp == NULL)
        {
          return -1;
        }

      for (i = sp->typecnt - 1; i >= 0; --i)
        {
          if (sp->ttis[i].tt_isdst != yourtm.tm_isdst)
            {
              continue;
            }

          for (j = sp->typecnt - 1; j >= 0; --j)
            {
              if (sp->ttis[j].tt_isdst == yourtm.tm_isdst)
                {
                  continue;
                }

              newt = t + sp->ttis[j].tt_gmtoff - sp->ttis[i].tt_gmtoff;
              if ((*funcp) (&newt, offset, &mytm) == NULL)
                {
                  continue;
                }

              if (tmcomp(&mytm, &yourtm) != 0)
                {
                  continue;
                }

              if (mytm.tm_isdst != yourtm.tm_isdst)
                {
                  continue;
                }

              /* We have a match */

              t = newt;
              goto label;
            }
        }

      return -1;
    }

label:
  newt = t + saved_seconds;
  if ((newt < t) != (saved_seconds < 0))
    {
      return -1;
    }

  t = newt;
  if ((*funcp) (&t, offset, tmp))
    {
      *okayp = TRUE;
    }

  return t;
}

static time_t time2(FAR struct tm *tmp,
                    FAR struct tm *(*funcp)(FAR const time_t *,
                                            int_fast32_t, FAR struct tm *),
                    int_fast32_t offset, FAR int *okayp)
{
  time_t t;

  /* First try without normalization of seconds
   * (in case tm_sec contains a value associated with a leap second).
   * If that fails, try with normalization of seconds.
   */

  t = time2sub(tmp, funcp, offset, okayp, FALSE);
  return *okayp ? t : time2sub(tmp, funcp, offset, okayp, TRUE);
}

static time_t time1(FAR struct tm *tmp,
                    FAR struct tm *(*funcp)(FAR const time_t *,
                                    int_fast32_t, FAR struct tm *),
                    int_fast32_t offset)
{
  time_t t;
  FAR const struct state_s *sp;
  int samei;
  int otheri;
  int sameind;
  int otherind;
  int i;
  int nseen;
  char seen[TZ_MAX_TYPES];
  unsigned char types[TZ_MAX_TYPES];
  int okay;

  if (tmp == NULL)
    {
      set_errno(EINVAL);
      return -1;
    }

  if (tmp->tm_isdst > 1)
    {
      tmp->tm_isdst = 1;
    }

  t = time2(tmp, funcp, offset, &okay);
  if (okay)
    {
      return t;
    }

  if (tmp->tm_isdst < 0)
    {
      return t;
    }

  /* We're supposed to assume that somebody took a time of one type
   * and did some math on it that yielded a "struct tm" that's bad.
   * We try to divine the type they started from and adjust to the
   * type they need.
   */

  sp = ((funcp == localsub) ? g_lcl_ptr : g_gmt_ptr);
  if (sp == NULL)
    {
      return -1;
    }

  for (i = 0; i < sp->typecnt; ++i)
    {
      seen[i] = FALSE;
    }

  nseen = 0;
  for (i = sp->timecnt - 1; i >= 0; --i)
    {
      if (!seen[sp->types[i]])
        {
          seen[sp->types[i]] = TRUE;
          types[nseen++] = sp->types[i];
        }
    }

  for (sameind = 0; sameind < nseen; ++sameind)
    {
      samei = types[sameind];
      if (sp->ttis[samei].tt_isdst != tmp->tm_isdst)
        {
          continue;
        }

      for (otherind = 0; otherind < nseen; ++otherind)
        {
          otheri = types[otherind];
          if (sp->ttis[otheri].tt_isdst == tmp->tm_isdst)
            {
              continue;
            }

          tmp->tm_sec  += sp->ttis[otheri].tt_gmtoff -
                          sp->ttis[samei].tt_gmtoff;
          tmp->tm_isdst = !tmp->tm_isdst;
          t = time2(tmp, funcp, offset, &okay);
          if (okay)
            {
              return t;
            }

          tmp->tm_sec  -= sp->ttis[otheri].tt_gmtoff -
                          sp->ttis[samei].tt_gmtoff;
          tmp->tm_isdst = !tmp->tm_isdst;
        }
    }

  return -1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void tzset(void)
{
  FAR const char *name;

#ifndef __KERNEL__
  if (up_interrupt_context() || (sched_idletask() && OSINIT_IDLELOOP()))
    {
      return;
    }
#endif

  nxrmutex_lock(&g_lcl_lock);

  name = getenv("TZ");
  if (name == NULL)
    {
      tzsetwall();
      goto out;
    }

  if (g_lcl_isset > 0 && strcmp(g_lcl_tzname, name) == 0)
    {
      goto out;
    }

  if (g_lcl_ptr == NULL)
    {
      g_lcl_ptr = lib_malloc(sizeof *g_lcl_ptr);
      if (g_lcl_ptr == NULL)
        {
          goto tzname;
        }
    }

  if (*name == '\0')
    {
      /* User wants it fast rather than right */

      g_lcl_ptr->leapcnt = 0; /* so, we're off a little */
      g_lcl_ptr->timecnt = 0;
      g_lcl_ptr->typecnt = 0;
      g_lcl_ptr->ttis[0].tt_isdst = 0;
      g_lcl_ptr->ttis[0].tt_gmtoff = 0;
      g_lcl_ptr->ttis[0].tt_abbrind = 0;
      strcpy(g_lcl_ptr->chars, GMT);
    }
  else if (tzload(name, g_lcl_ptr, TRUE) != 0)
    {
      if (name[0] == ':' || tzparse(name, g_lcl_ptr, FALSE) != 0)
        {
          gmtload(g_lcl_ptr);
          goto tzname;
        }
    }

  g_lcl_isset = strlen(name) < sizeof g_lcl_tzname;
  if (g_lcl_isset)
    {
      strcpy(g_lcl_tzname, name);
    }

tzname:
  settzname();
out:
  nxrmutex_unlock(&g_lcl_lock);
}

FAR struct tm *localtime(FAR const time_t *timep)
{
  tzset();
  return localsub(timep, 0L, &g_tm);
}

/* Re-entrant version of localtime */

FAR struct tm *localtime_r(FAR const time_t *timep, FAR struct tm *tmp)
{
  tzset();
  return localsub(timep, 0L, tmp);
}

FAR struct tm *gmtime(FAR const time_t *timep)
{
  return gmtsub(timep, 0L, &g_tm);
}

/* Re-entrant version of gmtime */

FAR struct tm *gmtime_r(FAR const time_t *timep, FAR struct tm *tmp)
{
  return gmtsub(timep, 0L, tmp);
}

time_t mktime(FAR struct tm *tmp)
{
  tzset();
  return time1(tmp, localsub, 0L);
}

time_t timegm(FAR struct tm *tmp)
{
  return time1(tmp, gmtsub, 0L);
}
