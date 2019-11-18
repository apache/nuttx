/****************************************************************************
 * libs/libc/locale/lib_localeconv.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
