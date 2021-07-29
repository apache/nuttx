/****************************************************************************
 * include/locale.h
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

#ifndef __INCLUDE_LOCALE_H
#define __INCLUDE_LOCALE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LC_CTYPE         0
#define LC_NUMERIC       1
#define LC_TIME          2
#define LC_COLLATE       3
#define LC_MONETARY      4
#define LC_MESSAGES      5
#define LC_ALL           6

#define LC_COLLATE_MASK  (1 << LC_COLLATE)
#define LC_CTYPE_MASK    (1 << LC_CTYPE)
#define LC_MONETARY_MASK (1 << LC_MONETARY)
#define LC_NUMERIC_MASK  (1 << LC_NUMERIC)
#define LC_TIME_MASK     (1 << LC_TIME)
#define LC_MESSAGES_MASK (1 << LC_MESSAGES)

#define LC_ALL_MASK      (LC_COLLATE_MASK  | LC_CTYPE_MASK   | \
                          LC_MONETARY_MASK | LC_NUMERIC_MASK | \
                          LC_TIME_MASK     | LC_MESSAGES_MASK)

#define LC_GLOBAL_LOCALE ((locale_t)-1)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* OpenGroup.org: "The locale.h header shall define the lconv structure,
 *  which shall include at least the following members. ..."
 */

struct lconv
{
  FAR char *decimal_point;
  FAR char *thousands_sep;
  FAR char *grouping;
  FAR char *int_curr_symbol;
  FAR char *currency_symbol;
  FAR char *mon_decimal_point;
  FAR char *mon_thousands_sep;
  FAR char *mon_grouping;
  FAR char *positive_sign;
  FAR char *negative_sign;
  FAR char int_frac_digits;
  FAR char frac_digits;
  FAR char p_cs_precedes;
  FAR char p_sep_by_space;
  FAR char n_cs_precedes;
  FAR char n_sep_by_space;
  FAR char p_sign_posn;
  FAR char n_sign_posn;
  FAR char int_n_cs_precedes;
  FAR char int_n_sep_by_space;
  FAR char int_n_sign_posn;
  FAR char int_p_cs_precedes;
  FAR char int_p_sep_by_space;
  FAR char int_p_sign_posn;
};

/* OpenGroup.org:  The locale.h header shall define the locale_t type,
 * representing a locale object
 */

typedef FAR void *locale_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

FAR char *setlocale(int category, FAR const char *locale);
FAR struct lconv *localeconv(void);

locale_t newlocale(int category_mask, FAR const char *locale, locale_t base);
locale_t duplocale(locale_t locobj);
void freelocale(locale_t locobj);

locale_t uselocale(locale_t newloc);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_LOCALE_H */
