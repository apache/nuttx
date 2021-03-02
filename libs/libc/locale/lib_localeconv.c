/****************************************************************************
 * libs/libc/locale/lib_localeconv.c
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
#include <locale.h>

#ifdef CONFIG_LIBC_LOCALE

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lconv g_c_lconv =
{
  .decimal_point      = ".",
  .thousands_sep      = "",
  .grouping           = "",
  .int_curr_symbol    = "",
  .currency_symbol    = "",
  .mon_decimal_point  = "",
  .mon_thousands_sep  = "",
  .mon_grouping       = "",
  .positive_sign      = "",
  .negative_sign      = "",
  .int_frac_digits    = CHAR_MAX,
  .frac_digits        = CHAR_MAX,
  .p_cs_precedes      = CHAR_MAX,
  .p_sep_by_space     = CHAR_MAX,
  .n_cs_precedes      = CHAR_MAX,
  .n_sep_by_space     = CHAR_MAX,
  .p_sign_posn        = CHAR_MAX,
  .n_sign_posn        = CHAR_MAX,
  .int_p_cs_precedes  = CHAR_MAX,
  .int_p_sep_by_space = CHAR_MAX,
  .int_n_cs_precedes  = CHAR_MAX,
  .int_n_sep_by_space = CHAR_MAX,
  .int_p_sign_posn    = CHAR_MAX,
  .int_n_sign_posn    = CHAR_MAX,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: localeconv
 *
 * Description:
 *   locales are not supported by NuttX
 *
 * Input Parameters:
 *   category and locale - Select the appropriate piece of the program's
 *     locale.
 *
 ****************************************************************************/

FAR struct lconv *localeconv(void)
{
  return &g_c_lconv;
}
#endif
