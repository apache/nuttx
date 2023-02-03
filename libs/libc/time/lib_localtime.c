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
#include <string.h>
#include <limits.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>

#include <sys/param.h>

#include <nuttx/time.h>
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

#define SECSPERHOUR         (SECSPERMIN * MINSPERHOUR)
#define SECSPERDAY          ((int_fast32_t)SECSPERHOUR * HOURSPERDAY)

#define isleap(y)           (((y) % 4) == 0 && (((y) % 100) != 0 || ((y) % 400) == 0))

#define GRANDPARENTED       "Local time zone must be set--see zic manual page"
#define TYPE_BIT(type)      (sizeof(type) * CHAR_BIT)
#define TYPE_SIGNED(type)   (((type)-1) < 0)
#define TWOS_COMPLEMENT(t)  ((t) ~ (t) 0 < 0)

#define YEARSPERREPEAT      400    /* years before a Gregorian repeat */
#define DAYSPERREPEAT       ((int_fast32_t) 400 * 365 + 100 - 4 + 1)

/* The Gregorian year averages 365.2425 days, which is 31556952 seconds. */

#define AVGSECSPERYEAR      31556952L
#define SECSPERREPEAT       ((int_fast64_t)YEARSPERREPEAT * (int_fast64_t)AVGSECSPERYEAR)

#define TZ_ABBR_MAX_LEN     16
#define TZ_ABBR_CHAR_SET \
  "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 :+-._"
#define TZ_ABBR_ERR_CHAR    '_'

/* Unlike <ctype.h>'s isdigit, this also works if c < 0 | c > UCHAR_MAX. */

#define is_digit(c)         ((unsigned)(c) - '0' <= 9)
#define MY_TZNAME_MAX       255

/* Max and min values of the integer type T, of which only the bottom
 * B bits are used, and where the highest-order used bit is considered
 * to be a sign bit if T is signed.
 */

#define MAXVAL(t, b) \
  ((t) (((t) 1 << ((b) - 1 - TYPE_SIGNED(t))) - \
        1 + ((t) 1 << ((b) - 1 - TYPE_SIGNED(t)))))

#define MINVAL(t, b) \
  ((t) (TYPE_SIGNED(t) ? - TWOS_COMPLEMENT(t) - MAXVAL(t, b) : 0))

/* The extreme time values, assuming no padding. */

#define TIME_T_MIN MINVAL(time_t, TYPE_BIT(time_t))
#define TIME_T_MAX MAXVAL(time_t, TYPE_BIT(time_t))

/* This abbreviation means local time is unspecified. */

#define UNSPEC              "-00"

/* How many extra bytes are needed at the end of struct state's chars array.
 * This needs to be at least 1 for null termination in case the input
 * data isn't properly terminated, and it also needs to be big enough
 * for ttunspecified to work without crashing.
 */

#define CHARS_EXTRA         (MAX(sizeof(UNSPEC), 2) - 1)

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
 * We default to US rules as of 2017-05-07.
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
 *    tzh_ttisutcnt  (char)s        indexed by type; if TRUE, transition
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
  char tzh_ttisutcnt[4];      /* coded number of trans. time flags */
  char tzh_ttisstdcnt[4];     /* coded number of trans. time flags */
  char tzh_leapcnt[4];        /* coded number of leap seconds */
  char tzh_timecnt[4];        /* coded number of transition times */
  char tzh_typecnt[4];        /* coded number of local time types */
  char tzh_charcnt[4];        /* coded number of abbr. chars */
};

struct ttinfo_s
{                             /* Time type information */
  int_fast32_t tt_utoff;      /* UT offset in seconds */
  int tt_isdst;               /* Used to set tm_isdst */
  int tt_desigidx;            /* Abbreviation list index */
  int tt_ttisstd;             /* True if transition is std time */
  int tt_ttisut;              /* True if transition is UT */
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
  char chars[MAX(MAX(TZ_MAX_CHARS + CHARS_EXTRA, sizeof("UTC")),
                    (2 * (MY_TZNAME_MAX + 1)))];
  struct lsinfo_s lsis[TZ_MAX_LEAPS];

  /* The time type to use for early times or if no transitions.
   * It is always zero for recent tzdb releases.
   * It might be nonzero for data from tzdb 2018e or earlier.
   */

  int defaulttype;
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

static const char g_wildabbr[] = WILDABBR;

static const char g_etc_utc[] = "Etc/UTC";
static FAR const char *g_utc = g_etc_utc + sizeof("Etc/") - 1;

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
static time_t leaps_thru_end_of(time_t y);
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
              FAR struct state_s *basep);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Initialize *S to a value based on UTOFF, ISDST, and DESIGIDX. */

static void init_ttinfo(FAR struct ttinfo_s *s, int_fast32_t utoff,
                        bool isdst, int desigidx)
{
  s->tt_utoff = utoff;
  s->tt_isdst = isdst;
  s->tt_desigidx = desigidx;
  s->tt_ttisstd = false;
  s->tt_ttisut = false;
}

/* Return true if SP's time type I does not specify local time. */

static int ttunspecified(FAR const struct state_s *sp, int i)
{
  FAR char const *abbr = &sp->chars[sp->ttis[i].tt_desigidx];

  /* memcmp is likely faster than strcmp, and is safe due to CHARS_EXTRA. */

  return memcmp(abbr, UNSPEC, sizeof(UNSPEC)) == 0;
}

static int_fast32_t detzcode(FAR const char *codep)
{
  int_fast32_t result;
  int_fast32_t one = 1;
  int_fast32_t halfmaxval = one << (32 - 2);
  int_fast32_t maxval = halfmaxval - 1 + halfmaxval;
  int_fast32_t minval = -1 - maxval;
  int i;

  result = codep[0] & 0x7f;
  for (i = 1; i < 4; ++i)
    {
      result = (result << 8) | (codep[i] & 0xff);
    }

  if (codep[0] & 0x80)
    {
      /* Do two's-complement negation even on non-two's-complement machines.
       * If the result would be minval - 1, return minval.
       */

      result -= !TWOS_COMPLEMENT(int_fast32_t) && result != 0;
      result += minval;
    }

  return result;
}

static int_fast64_t detzcode64(FAR const char *codep)
{
  int_fast64_t result;
  int_fast64_t one = 1;
  int_fast64_t halfmaxval = one << (64 - 2);
  int_fast64_t maxval = halfmaxval - 1 + halfmaxval;
  int_fast64_t minval = -TWOS_COMPLEMENT(int_fast64_t) - maxval;
  int i;

  result = codep[0] & 0x7f;
  for (i = 1; i < 8; ++i)
    {
      result = (result << 8) | (codep[i] & 0xff);
    }

  if (codep[0] & 0x80)
    {
      /* Do two's-complement negation even on non-two's-complement machines.
       * If the result would be minval - 1, return minval.
       */

      result -= !TWOS_COMPLEMENT(int_fast64_t) && result != 0;
      result += minval;
    }

  return result;
}

static void scrub_abbrs(struct state_s *sp)
{
  int i;

  /* First, replace bogus characters. */

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
      FAR const struct ttinfo_s * const ttisp = &sp->ttis[i];
      FAR char *cp = &sp->chars[ttisp->tt_desigidx];

      if (strlen(cp) > TZ_ABBR_MAX_LEN && strcmp(cp, GRANDPARENTED) != 0)
        {
          *(cp + TZ_ABBR_MAX_LEN) = '\0';
        }
    }
}

static void settzname(void)
{
  FAR struct state_s * const sp = g_lcl_ptr;
  int i;

  tzname[0] = tzname[1] = (FAR char *)g_wildabbr;
  if (sp == NULL)
    {
      tzname[0] = tzname[1] = (FAR char *)g_utc;
      return;
    }

  /* And to get the latest zone names into tzname */

  for (i = 0; i < sp->typecnt; ++i)
    {
      FAR const struct ttinfo_s * const ttisp = &sp->ttis[i];

      tzname[ttisp->tt_isdst] = &sp->chars[ttisp->tt_desigidx];
    }

  for (i = 0; i < sp->timecnt; ++i)
    {
      FAR const struct ttinfo_s * const ttisp = &sp->ttis[sp->types[i]];

      tzname[ttisp->tt_isdst] = &sp->chars[ttisp->tt_desigidx];
    }
}

static int_fast32_t leapcorr(FAR const struct state_s *sp, time_t t)
{
  FAR const struct lsinfo_s *lp;
  int i;

  i = sp->leapcnt;
  while (--i >= 0)
    {
      lp = &sp->lsis[i];
      if (t >= lp->ls_trans)
        {
          return lp->ls_corr;
        }
    }

  return 0;
}

static int tzload(FAR const char *name,
                  FAR struct state_s *sp, int doextend)
{
  int i;
  int fid;
  int stored;
  ssize_t nread;

  typedef union
  {
    struct tzhead_s tzhead;
    char buf[2 * sizeof(struct tzhead_s) +
             2 * sizeof(struct state_s) +
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
  int tzheadsize = sizeof(struct tzhead_s);

  lsp = lib_malloc(sizeof(*lsp));
  if (lsp == NULL)
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
      FAR const char *dot;
      size_t namelen = strlen(name);
      const char tzdirslash[sizeof(TZDIR)] = TZDIR "/";

      if (sizeof(fullname) - sizeof(tzdirslash) <= namelen)
        {
          goto oops;
        }

      /* Create a string "TZDIR/NAME".  Using sprintf here
       * would pull in stdio (and would fail if the
       * resulting string length exceeded INT_MAX!).
       */

      memcpy(fullname, tzdirslash, sizeof(tzdirslash));
      strcpy(fullname + sizeof(tzdirslash), name);

      /* Set doaccess if NAME contains a ".." file name
       * component, as such a name could read a file outside
       * the TZDIR virtual subtree.
       */

      for (dot = name; (dot = strchr(dot, '.')); dot++)
        {
          if ((dot == name || dot[0 - 1] == '/') && dot[1] == '.' &&
              (dot[2] == '/' || !dot[2]))
            {
              doaccess = TRUE;
              break;
            }
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

  nread = _NX_READ(fid, up->buf, sizeof(up->buf));
  if (_NX_CLOSE(fid) < 0 || nread < tzheadsize)
    {
      goto oops;
    }

  for (stored = 4; stored <= 8; stored *= 2)
    {
      char version = up->tzhead.tzh_version[0];
      int skip_datablock = stored == 4 && version;
      int_fast32_t datablock_size;
      int_fast32_t ttisstdcnt;
      int_fast32_t ttisutcnt;
      int_fast32_t leapcnt;
      int_fast32_t timecnt;
      int_fast32_t typecnt;
      int_fast32_t charcnt;
      int_fast64_t prevtr = -1;
      int_fast32_t prevcorr;
      FAR const char *p;

      ttisstdcnt  = detzcode(up->tzhead.tzh_ttisstdcnt);
      ttisutcnt  = detzcode(up->tzhead.tzh_ttisutcnt);
      leapcnt = detzcode(up->tzhead.tzh_leapcnt);
      timecnt = detzcode(up->tzhead.tzh_timecnt);
      typecnt = detzcode(up->tzhead.tzh_typecnt);
      charcnt = detzcode(up->tzhead.tzh_charcnt);
      p = up->buf + tzheadsize;
      if (leapcnt < 0 || leapcnt > TZ_MAX_LEAPS ||
          typecnt < 0 || typecnt > TZ_MAX_TYPES ||
          timecnt < 0 || timecnt > TZ_MAX_TIMES ||
          charcnt < 0 || charcnt > TZ_MAX_CHARS ||
          ttisstdcnt < 0 || ttisstdcnt > TZ_MAX_TYPES ||
          ttisutcnt < 0 || ttisutcnt > TZ_MAX_TYPES)
        {
          goto oops;
        }

      datablock_size = (timecnt * stored          /* ats */
                     + timecnt                    /* types */
                     + typecnt * 6                /* ttinfos */
                     + charcnt                    /* chars */
                     + leapcnt * (stored + 4)     /* lsinfos */
                     + ttisstdcnt                 /* ttisstds */
                     + ttisutcnt);                /* ttisuts */
      if (nread - tzheadsize < datablock_size)
        {
          goto oops;
        }

      if (skip_datablock)
        {
          p += datablock_size;
        }
      else
        {
          if (!((ttisstdcnt == typecnt || ttisstdcnt == 0) &&
              (ttisutcnt == typecnt || ttisutcnt == 0)))
            {
              goto oops;
            }

          sp->leapcnt = leapcnt;
          sp->timecnt = timecnt;
          sp->typecnt = typecnt;
          sp->charcnt = charcnt;

          timecnt = 0;
          for (i = 0; i < sp->timecnt; ++i)
            {
              int_fast64_t at = stored == 4 ? detzcode(p) : detzcode64(p);
              sp->types[i] = at <= TIME_T_MAX;
              if (sp->types[i])
                {
                  time_t attime = ((TYPE_SIGNED(time_t) ?
                                  at < TIME_T_MIN : at < 0) ?
                                  TIME_T_MIN : at);
                  if (timecnt && attime <= sp->ats[timecnt - 1])
                    {
                      if (attime < sp->ats[timecnt - 1])
                        {
                          goto oops;
                        }

                      sp->types[i - 1] = 0;
                      timecnt--;
                    }

                  sp->ats[timecnt++] = attime;
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
              unsigned char isdst;
              unsigned char desigidx;

              ttisp = &sp->ttis[i];
              ttisp->tt_utoff = detzcode(p);
              p += 4;
              isdst = *p++;
              if (isdst >= 2)
                {
                  goto oops;
                }

              ttisp->tt_isdst = isdst;
              desigidx = *p++;
              if (desigidx >= sp->charcnt)
                {
                  goto oops;
                }

              ttisp->tt_desigidx = desigidx;
            }

          for (i = 0; i < sp->charcnt; ++i)
            {
              sp->chars[i] = *p++;
            }

          /* Ensure '\0'-terminated, and make it safe to call
           * ttunspecified later.
           */

          memset(&sp->chars[i], 0, CHARS_EXTRA);
          for (i = 0; i < sp->leapcnt; ++i)
            {
              int_fast64_t tr = stored == 4 ? detzcode(p) : detzcode64(p);
              int_fast32_t corr = detzcode(p + stored);
              p += stored + 4;

              /* Leap seconds cannot occur before the Epoch,
               * or out of order.
               */

              if (tr <= prevtr)
                {
                  goto oops;
                }

              /* To avoid other botches in this code, each leap second's
               * correction must differ from the previous one's by 1
               * second or less, except that the first correction can be
               * any value; these requirements are more generous than
               * RFC 8536, to allow future RFC extensions.
               */

              if (!(i == 0 || (prevcorr < corr ? corr == prevcorr + 1 :
                                                 (corr == prevcorr ||
                                                 corr == prevcorr - 1))))
                {
                  goto oops;
                }

              prevtr = tr;
              prevcorr = corr;
              if (tr <= TIME_T_MAX)
                {
                  sp->lsis[leapcnt].ls_trans = tr;
                  sp->lsis[leapcnt].ls_corr = corr;
                  leapcnt++;
                }
            }

          sp->leapcnt = leapcnt;
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
                  if (*p != TRUE && *p != FALSE)
                    {
                      goto oops;
                    }

                  ttisp->tt_ttisstd = *p++;
                }
            }

          for (i = 0; i < sp->typecnt; ++i)
            {
              FAR struct ttinfo_s *ttisp;

              ttisp = &sp->ttis[i];
              if (ttisutcnt == 0)
                {
                  ttisp->tt_ttisut = FALSE;
                }
              else
                {
                  if (*p != TRUE && *p != FALSE)
                    {
                      goto oops;
                    }

                  ttisp->tt_ttisut = *p++;
                }
            }
        }

      nread -= p - up->buf;
      for (i = 0; i < nread; ++i)
        {
          up->buf[i] = p[i];
        }

      /* If this is an old file, we're done. */

      if (version == '\0')
        {
          break;
        }
    }

  if (doextend && nread > 2 &&
      up->buf[0] == '\n' && up->buf[nread - 1] == '\n' &&
      sp->typecnt + 2 <= TZ_MAX_TYPES)
    {
      FAR struct state_s *ts = &lsp->u.st;

      up->buf[nread - 1] = '\0';
      if (tzparse(&up->buf[1], ts, sp) == 0)
        {
          /* Attempt to reuse existing abbreviations.
           * Without this, America/Anchorage would be right on
           * the edge after 2037 when TZ_MAX_CHARS is 50, as
           * sp->charcnt equals 40 (for LMT AST AWT APT AHST
           * AHDT YST AKDT AKST) and ts->charcnt equals 10
           * (for AKST AKDT).  Reusing means sp->charcnt can
           * stay 40 in this example.
           */

          int gotabbr = 0;
          int charcnt = sp->charcnt;
          for (i = 0; i < ts->typecnt; i++)
            {
              FAR char *tsabbr = ts->chars + ts->ttis[i].tt_desigidx;
              int j;

              for (j = 0; j < charcnt; j++)
                {
                  if (strcmp(sp->chars + j, tsabbr) == 0)
                    {
                      ts->ttis[i].tt_desigidx = j;
                      gotabbr++;
                      break;
                    }
                }

              if (j >= charcnt)
                {
                  int tsabbrlen = strlen(tsabbr);

                  if (j + tsabbrlen < TZ_MAX_CHARS)
                    {
                      strcpy(sp->chars + j, tsabbr);
                      charcnt = j + tsabbrlen + 1;
                      ts->ttis[i].tt_desigidx = j;
                      gotabbr++;
                    }
                }
            }

          if (gotabbr == ts->typecnt)
            {
              sp->charcnt = charcnt;

              /* Ignore any trailing, no-op transitions generated
               * by zic as they don't help here and can run afoul
               * of bugs in zic 2016j or earlier.
               */

              while (sp->timecnt > 1 && (sp->types[sp->timecnt - 1] ==
                                         sp->types[sp->timecnt - 2]))
                {
                  sp->timecnt--;
                }

              for (i = 0; i < ts->timecnt && sp->timecnt < TZ_MAX_TIMES; i++)
                {
                  time_t t = ts->ats[i];
                  if (increment_overflow_time(&t, leapcorr(sp, t))
                      || (0 < sp->timecnt && t <= sp->ats[sp->timecnt - 1]))
                    {
                      continue;
                    }

                  sp->ats[sp->timecnt] = t;
                  sp->types[sp->timecnt] = (sp->typecnt + ts->types[i]);
                  sp->timecnt++;
                }

              for (i = 0; i < ts->typecnt; i++)
                {
                  sp->ttis[sp->typecnt++] = ts->ttis[i];
                }
            }
        }
    }

  if (sp->typecnt == 0)
    {
      goto oops;
    }

  if (sp->timecnt > 1)
    {
      if (TIME_T_MAX > SECSPERREPEAT &&
          sp->ats[0] <= TIME_T_MAX - SECSPERREPEAT)
        {
          time_t repeatat = sp->ats[0] + SECSPERREPEAT;
          int repeattype = sp->types[0];
          for (i = 1; i < sp->timecnt; ++i)
            {
              if (sp->ats[i] == repeatat &&
                  typesequiv(sp, sp->types[i], repeattype))
                {
                  sp->goback = TRUE;
                  break;
                }
            }
        }

      if (TIME_T_MAX > SECSPERREPEAT &&
          TIME_T_MIN + SECSPERREPEAT <= sp->ats[sp->timecnt - 1])
        {
          time_t repeatat = sp->ats[sp->timecnt - 1] - SECSPERREPEAT;
          int repeattype = sp->types[sp->timecnt - 1];
          for (i = sp->timecnt - 2; i >= 0; --i)
            {
              if (sp->ats[i] == repeatat &&
                  typesequiv(sp, sp->types[i], repeattype))
                {
                  sp->goahead = TRUE;
                  break;
                }
            }
        }
    }

  /* If type 0 is is unused in transitions, it's the type to use for early
   * times.
   */

  for (i = 0; i < sp->timecnt; ++i)
    {
      if (sp->types[i] == 0)
        {
          break;
        }
    }

  i = i < sp->timecnt  && ! ttunspecified(sp, 0) ? -1 : 0;

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

      result = ap->tt_utoff == bp->tt_utoff &&
        ap->tt_isdst == bp->tt_isdst &&
        ap->tt_ttisstd == bp->tt_ttisstd &&
        ap->tt_ttisut == bp->tt_ttisut &&
        strcmp(&sp->chars[ap->tt_desigidx],
               &sp->chars[bp->tt_desigidx]) == 0;
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
  int neg = FALSE;

  if (*strp == '-')
    {
      neg = TRUE;
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
                   FAR struct state_s *basep)
{
  FAR const char *stdname;
  FAR const char *dstname;
  size_t stdlen;
  size_t dstlen;
  size_t charcnt;
  int_fast32_t stdoffset;
  int_fast32_t dstoffset;
  FAR char *cp;
  int load_ok;
  time_t atlo = TIME_T_MIN;
  time_t leaplo = TIME_T_MIN;

  stdname = name;
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

  if (stdlen == 0)
    {
      return -1;
    }

  name = getoffset(name, &stdoffset);
  if (name == NULL)
    {
      return -1;
    }

  charcnt = stdlen + 1;
  if (sizeof(sp->chars) < charcnt)
    {
      return -1;
    }

  if (basep)
    {
      if (0 < basep->timecnt)
        {
          atlo = basep->ats[basep->timecnt - 1];
        }

      load_ok = -1;
      sp->leapcnt = basep->leapcnt;
      memcpy(sp->lsis, basep->lsis, sp->leapcnt * sizeof(*sp->lsis));
    }
  else
    {
      load_ok = tzload(TZDEFRULES, sp, FALSE);
      if (load_ok != 0)
        {
          sp->leapcnt = 0; /* so, we're off a little */
        }
    }

  if (sp->leapcnt > 0)
    {
      leaplo = sp->lsis[sp->leapcnt - 1].ls_trans;
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

      if (dstlen == 0)
        {
          return -1;
        }

      charcnt += dstlen + 1;
      if (sizeof(sp->chars) < charcnt)
        {
          return -1;
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

      if (*name == '\0' && load_ok != 0)
        {
          name = TZDEFRULESTRING;
        }

      if (*name == ',' || *name == ';')
        {
          struct rule_s start;
          struct rule_s end;
          int year;
          int yearlim;
          int yearbeg;
          int timecnt;
          time_t janfirst;
          int_fast32_t janoffset = 0;

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

          init_ttinfo(&sp->ttis[0], -stdoffset, FALSE, 0);
          init_ttinfo(&sp->ttis[1], -dstoffset, TRUE, stdlen + 1);
          sp->defaulttype = 0;
          timecnt = 0;
          janfirst = 0;
          yearbeg = EPOCH_YEAR;

          do
            {
              int_fast32_t yearsecs;

              yearsecs = g_year_lengths[isleap(yearbeg - 1)] * SECSPERDAY;
              yearbeg--;
              if (increment_overflow_time(&janfirst, -yearsecs))
                {
                  janoffset = -yearsecs;
                  break;
                }
            }
          while (atlo < janfirst && (EPOCH_YEAR -
                                     YEARSPERREPEAT / 2 < yearbeg));

          while (true)
            {
              int_fast32_t yearsecs;
              int yearbeg1 = yearbeg;
              time_t janfirst1 = janfirst;

              yearsecs = g_year_lengths[isleap(yearbeg)] * SECSPERDAY;
              if (increment_overflow_time(&janfirst1, yearsecs) ||
                  increment_overflow(&yearbeg1, 1) || atlo <= janfirst1)
                {
                  break;
                }

              yearbeg = yearbeg1;
              janfirst = janfirst1;
            }

          yearlim = yearbeg;
          if (increment_overflow(&yearlim, YEARSPERREPEAT + 1))
            {
              yearlim = INT_MAX;
            }

          for (year = yearbeg; year < yearlim; year++)
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
                    (endtime - starttime < yearsecs)))
                {
                  if (TZ_MAX_TIMES - 2 < timecnt)
                    {
                      break;
                    }

                  sp->ats[timecnt] = janfirst;
                  if (!increment_overflow_time(&sp->ats[timecnt],
                                               janoffset + starttime) &&
                      atlo <= sp->ats[timecnt])
                    {
                      sp->types[timecnt++] = !reversed;
                    }

                  sp->ats[timecnt] = janfirst;
                  if (!increment_overflow_time(&sp->ats[timecnt],
                                               janoffset + endtime) &&
                      atlo <= sp->ats[timecnt])
                    {
                      sp->types[timecnt++] = !reversed;
                    }
                }

              if (endtime < leaplo)
                {
                  yearlim = year;
                  if (increment_overflow(&yearlim, YEARSPERREPEAT + 1))
                    {
                      yearlim = INT_MAX;
                    }
                }

              if (increment_overflow_time(&janfirst, janoffset + yearsecs))
                {
                  break;
                }

              janoffset = 0;
            }

          sp->timecnt = timecnt;
          if (!timecnt)
            {
              sp->ttis[0] = sp->ttis[1];
              sp->typecnt = 1; /* Perpetual DST.  */
            }
          else if (YEARSPERREPEAT < year - yearbeg)
            {
              sp->goback = sp->goahead = TRUE;
            }
        }
      else
        {
          int_fast32_t theirstdoffset;
          int_fast32_t theirdstoffset;
          int_fast32_t theiroffset;
          int isdst;
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
                  theirstdoffset = -sp->ttis[j].tt_utoff;
                  break;
                }
            }

          theirdstoffset = 0;
          for (i = 0; i < sp->timecnt; ++i)
            {
              j = sp->types[i];
              if (!sp->ttis[j].tt_isdst)
                {
                  theirdstoffset = -sp->ttis[j].tt_utoff;
                  break;
                }
            }

          /* Initially we're assumed to be in standard time */

          isdst = -1;

          /* Now juggle transition times and types
           * tracking offsets as you do.
           */

          for (i = 0; i < sp->timecnt; ++i)
            {
              j = sp->types[i];
              sp->types[i] = sp->ttis[j].tt_isdst;
              if (sp->ttis[j].tt_ttisut)
                {
                  /* No adjustment to transition time */
                }
              else
                {
                  /* If daylight saving time is in
                   * effect, and the transition time was
                   * not specified as standard time, add
                   * the daylight saving time offset to
                   * the transition time; otherwise, add
                   * the standard time offset to the
                   * transition time.
                   */

                  /* Transitions from DST to DDST
                   * will effectively disappear since
                   * POSIX provides for only one DST
                   * offset.
                   */

                  if (isdst && !sp->ttis[j].tt_ttisstd)
                    {
                      sp->ats[i] += dstoffset - theirdstoffset;
                    }
                  else
                    {
                      sp->ats[i] += stdoffset - theirstdoffset;
                    }
                }

              theiroffset = -sp->ttis[j].tt_utoff;
              if (sp->ttis[j].tt_isdst)
                {
                  theirstdoffset = theiroffset;
                }
              else
                {
                  theirdstoffset = theiroffset;
                }
            }

          /* Finally, fill in ttis */

          init_ttinfo(&sp->ttis[0], -stdoffset, FALSE, 0);
          init_ttinfo(&sp->ttis[1], -dstoffset, TRUE, stdlen + 1);
          sp->typecnt = 2;
          sp->defaulttype = 0;
        }
    }
  else
    {
      dstlen = 0;
      sp->typecnt = 1;          /* only standard time */
      sp->timecnt = 0;
      init_ttinfo(&sp->ttis[0], -stdoffset, FALSE, 0);
      sp->defaulttype = 0;
    }

  sp->charcnt = charcnt;
  cp = sp->chars;
  memcpy(cp, stdname, stdlen);
  cp += stdlen;
  *cp++ = '\0';
  if (dstlen != 0)
    {
      memcpy(cp, dstname, dstlen);
      *(cp + dstlen) = '\0';
    }

  return 0;
}

static void gmtload(FAR struct state_s *sp)
{
  if (tzload(g_etc_utc, sp, TRUE) != 0)
    {
      tzparse("UTC0", sp, NULL);
    }
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

  if (nxrmutex_is_hold(&g_lcl_lock))
    {
      return NULL;
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

      /* Beware integer overflow, as SECONDS might be close
       * to the maximum time_t.
       */

      years = seconds / SECSPERREPEAT * YEARSPERREPEAT;
      seconds = years * AVGSECSPERYEAR;
      years += YEARSPERREPEAT;
      if (t < sp->ats[0])
        {
          newt += seconds + SECSPERREPEAT;
        }
      else
        {
          newt -= seconds + SECSPERREPEAT;
        }

      if (newt < sp->ats[0] || newt > sp->ats[sp->timecnt - 1])
        {
          return NULL; /* "cannot happen" */
        }

      result = localsub(&newt, offset, tmp);
      if (result != NULL)
        {
          int_fast64_t newy;

          newy = result->tm_year;
          if (t < sp->ats[0])
            {
              newy -= years;
            }
          else
            {
              newy += years;
            }

          if (newy < INT_MIN || newy > INT_MAX)
            {
              return NULL;
            }

          result->tm_year = newy;
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

      i = sp->types[lo - 1];
    }

  ttisp = &sp->ttis[i];

  /* To get (wrong) behavior that's compatible with System V Release 2.0
   * you'd replace the statement below with
   *    t += ttisp->tt_utoff;
   *    timesub(&t, 0L, sp, tmp);
   */

  result = timesub(&t, ttisp->tt_utoff, sp, tmp);
  if (result != NULL)
    {
      result->tm_isdst = ttisp->tt_isdst;
      tzname[result->tm_isdst] = &sp->chars[ttisp->tt_desigidx];
      result->tm_zone = tzname[result->tm_isdst];
    }

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
          g_gmt_ptr = lib_malloc(sizeof(*g_gmt_ptr));
          if (g_gmt_ptr != NULL)
            {
              gmtload(g_gmt_ptr);
              g_gmt_isset = 1;
            }
        }

      nxrmutex_unlock(&g_gmt_lock);
    }

  tmp->tm_zone = ((FAR char *)(offset ? g_wildabbr :
                               g_gmt_ptr ? g_gmt_ptr->chars : g_utc));
  return timesub(timep, offset, g_gmt_ptr, tmp);
}

/* Return the number of leap years through the end of the given year
 * where, to make the math easy, the answer for year zero is defined as zero.
 */

static time_t leaps_thru_end_of_nonneg(time_t y)
{
  return y / 4 - y / 100 + y / 400;
}

static time_t leaps_thru_end_of(time_t y)
{
  return (y < 0 ? -1 - leaps_thru_end_of_nonneg(-1 - y)
                : leaps_thru_end_of_nonneg(y));
}

static FAR struct tm *timesub(FAR const time_t *timep,
                              int_fast32_t offset,
                              FAR const struct state_s *sp,
                              FAR struct tm *tmp)
{
  FAR const struct lsinfo_s *lp;
  FAR const int *ip;
  int_fast32_t rem;
  int_fast32_t idays;
  int_fast32_t dayoff;
  int_fast32_t dayrem;
  int_fast32_t corr;
  time_t tdays;
  time_t y;
  int i;

  /* If less than SECSPERMIN, the number of seconds since the
   * most recent positive leap second; otherwise, do not add 1
   * to localtime tm_sec because of leap seconds.
   */

  time_t secs_since_posleap = SECSPERMIN;

  corr = 0;
  i = (sp == NULL) ? 0 : sp->leapcnt;
  while (--i >= 0)
    {
      lp = &sp->lsis[i];
      if (*timep >= lp->ls_trans)
        {
          corr = lp->ls_corr;
          if ((i == 0 && corr > 0) || corr > lp[0 - 1].ls_corr)
            {
              secs_since_posleap = *timep - lp->ls_trans;
            }

          break;
        }
    }

  /* Calculate the year, avoiding integer overflow even if
   * time_t is unsigned.
   */

  tdays = *timep / SECSPERDAY;
  rem = *timep % SECSPERDAY;
  rem += offset % SECSPERDAY - corr % SECSPERDAY + 3 * SECSPERDAY;
  dayoff = offset / SECSPERDAY - corr / SECSPERDAY + rem / SECSPERDAY - 3;
  rem %= SECSPERDAY;

  /* y = (EPOCH_YEAR
   *      + floor((tdays + dayoff) / DAYSPERREPEAT) * YEARSPERREPEAT),
   * sans overflow.  But calculate against 1570 (EPOCH_YEAR -
   * YEARSPERREPEAT) instead of against 1970 so that things work
   * for localtime values before 1970 when time_t is unsigned.
   */

  dayrem = tdays % DAYSPERREPEAT;
  dayrem += dayoff % DAYSPERREPEAT;
  y = (EPOCH_YEAR - YEARSPERREPEAT +
       ((1 + dayoff / DAYSPERREPEAT + dayrem / DAYSPERREPEAT -
        ((dayrem % DAYSPERREPEAT) < 0) + tdays / DAYSPERREPEAT)
        * YEARSPERREPEAT));

  /* idays = (tdays + dayoff) mod DAYSPERREPEAT, sans overflow. */

  idays = tdays % DAYSPERREPEAT;
  idays += dayoff % DAYSPERREPEAT + 2 * DAYSPERREPEAT;
  idays %= DAYSPERREPEAT;

  /* Increase Y and decrease IDAYS until IDAYS is in range for Y.  */

  while (idays >= g_year_lengths[isleap(y)])
    {
      time_t newy;
      int tdelta;
      int_fast32_t ydelta;
      int leapdays;

      tdelta = idays / DAYSPERLYEAR;
      ydelta = tdelta + !tdelta;
      newy = y + ydelta;
      leapdays = leaps_thru_end_of(newy - 1) - leaps_thru_end_of(y - 1);
      idays -= ydelta * DAYSPERNYEAR;
      idays -= leapdays;
      y = newy;
    }

  if (!TYPE_SIGNED(time_t) && y < TM_YEAR_BASE)
    {
      int signed_y = y;
      tmp->tm_year = signed_y - TM_YEAR_BASE;
    }
  else if ((!TYPE_SIGNED(time_t) || INT_MIN + TM_YEAR_BASE <= y)
            && y - TM_YEAR_BASE <= INT_MAX)
    {
      tmp->tm_year = y - TM_YEAR_BASE;
    }
  else
    {
      errno = EOVERFLOW;
      return NULL;
    }

  tmp->tm_yday = idays;

  /* The "extra" mods below avoid overflow problems */

  tmp->tm_wday = TM_WDAY_BASE +
    ((tmp->tm_year % DAYSPERWEEK) *
    (DAYSPERNYEAR % DAYSPERWEEK)) +
    leaps_thru_end_of(y - 1) - leaps_thru_end_of(TM_YEAR_BASE - 1) + idays;
  tmp->tm_wday %= DAYSPERWEEK;
  if (tmp->tm_wday < 0)
    {
      tmp->tm_wday += DAYSPERWEEK;
    }

  tmp->tm_hour = (int)(rem / SECSPERHOUR);
  rem %= SECSPERHOUR;
  tmp->tm_min = (int)(rem / SECSPERMIN);
  tmp->tm_sec = (int)(rem % SECSPERMIN);

  /* Use "... ??:??:60" at the end of the localtime minute containing
   * the second just before the positive leap second.
   */

  tmp->tm_sec += secs_since_posleap <= tmp->tm_sec;

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
   * 'if (! (TIME_T_MIN <= *tp + j && *tp + j <= TIME_T_MAX)) ...',
   * except that it does the right thing even if *tp + j would overflow.
   */

  if (!(j < 0
        ? (TYPE_SIGNED(time_t) ? TIME_T_MIN - j <= *tp : -1 - j < *tp)
        : *tp <= TIME_T_MAX - j))
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

  if (y < INT_MIN || y > INT_MAX)
    {
      return -1;
    }

  yourtm.tm_year = y;
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

  lo = TIME_T_MIN;
  hi = TIME_T_MAX;
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

      if ((*funcp)(&t, offset, &mytm) == NULL)
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
              if (t == TIME_T_MAX)
                {
                  return -1;
                }

              ++t;
              ++lo;
            }
          else if (t == hi)
            {
              if (t == TIME_T_MIN)
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

              newt = t + sp->ttis[j].tt_utoff - sp->ttis[i].tt_utoff;
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
      if (!seen[sp->types[i]] && !ttunspecified(sp, sp->types[i]))
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

          tmp->tm_sec  += sp->ttis[otheri].tt_utoff -
                          sp->ttis[samei].tt_utoff;
          tmp->tm_isdst = !tmp->tm_isdst;
          t = time2(tmp, funcp, offset, &okay);
          if (okay)
            {
              return t;
            }

          tmp->tm_sec  -= sp->ttis[otheri].tt_utoff -
                          sp->ttis[samei].tt_utoff;
          tmp->tm_isdst = !tmp->tm_isdst;
        }
    }

  return -1;
}

/* Initialize *SP to a value appropriate for the TZ setting NAME.
 * Return 0 on success, an errno value on failure.
 */

static int zoneinit(FAR const char *name)
{
  if (name != NULL && name[0] == '\0')
    {
      /* User wants it fast rather than right */

      g_lcl_ptr->leapcnt = 0; /* so, we're off a little */
      g_lcl_ptr->timecnt = 0;
      g_lcl_ptr->typecnt = 0;
      g_lcl_ptr->charcnt = 0;
      g_lcl_ptr->goback  = 0;
      g_lcl_ptr->goahead = 0;
      init_ttinfo(&g_lcl_ptr->ttis[0], 0, FALSE, 0);
      strcpy(g_lcl_ptr->chars, g_utc);
      g_lcl_ptr->defaulttype = 0;
      return 0;
    }
  else
    {
      int err;

      err = tzload(name, g_lcl_ptr, TRUE);
      if (err != 0 && name != NULL && name[0] == ':' &&
          tzparse(name, g_lcl_ptr, NULL) != 0)
        {
          err = 0;
        }

      if (err == 0)
        {
          scrub_abbrs(g_lcl_ptr);
        }

      return err;
    }
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

  name = getenv("TZ");
  if (name == NULL)
    {
      return;
    }

  if (g_lcl_isset > 0 && strcmp(g_lcl_tzname, name) == 0)
    {
      return;
    }

  if (nxrmutex_is_hold(&g_lcl_lock))
    {
      return;
    }

  nxrmutex_lock(&g_lcl_lock);

  if (g_lcl_ptr == NULL)
    {
      g_lcl_ptr = lib_malloc(sizeof(*g_lcl_ptr));
      if (g_lcl_ptr == NULL)
        {
          goto tzname;
        }
    }

  if (zoneinit(name) != 0)
    {
      zoneinit("");
    }

  strcpy(g_lcl_tzname, name);

tzname:
  settzname();
  g_lcl_isset = 1;
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
  if (tmp != NULL)
    {
      tmp->tm_isdst = 0;
    }

  return time1(tmp, gmtsub, 0L);
}
