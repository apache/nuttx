/****************************************************************************
 * libs/libc/wchar/lib_wcsxfrm.c
 *
 *   Copyright (c)1999 Citrus Project,
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <string.h>
#include <wchar.h>

#ifdef CONFIG_LIBC_WCHAR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wcsxfrm
 *
 * Description:
 *   The wcsxfrm() transforms the wide-character string pointed to by b to the
 *   wide-character string pointed to by a, comparing two transformed wide
 *   strings with wcscmp() should return the same result as comparing the
 *   original strings with wcscoll().
 *   No more than n wide characters are transformed, including the trailing
 *   null character. The current implementation of wcsxfrm() simply uses
 *   wcslcpy() and does not support any language-specific transformations.
 *
 ****************************************************************************/

size_t wcsxfrm(FAR wchar_t *a, FAR const wchar_t *b, size_t n)
{
  return wcslcpy(a, b, n);
}
#endif
