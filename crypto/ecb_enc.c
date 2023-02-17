/****************************************************************************
 * crypto/ecb_enc.c
 * $OpenBSD: ecb_enc.c,v 1.6 2015/12/10 21:00:51 naddy Exp $
 * lib/des/ecb_enc.c
 * Copyright (C) 1995 Eric Young (eay@mincom.oz.au)
 * All rights reserved.
 *
 * This file is part of an SSL implementation written
 * by Eric Young (eay@mincom.oz.au).
 * The implementation was written so as to conform with Netscapes SSL
 * specification.  This library and applications are
 * FREE FOR COMMERCIAL AND NON-COMMERCIAL USE
 * as long as the following conditions are adhered to.
 *
 * Copyright remains Eric Young's, and as such any Copyright notices in
 * the code are not to be removed.  If this code is used in a product,
 * Eric Young should be given attribution as the author of the parts used.
 * This can be in the form of a textual message at program startup or
 * in documentation (online or textual) provided with the package.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by
 *    Eric Young (eay@mincom.oz.au)
 *
 * THIS SOFTWARE IS PROVIDED BY ERIC YOUNG ``AS IS'' AND
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
 * The licence and distribution terms for any publically available version or
 * derivative of this code cannot be changed.
 * i.e. this code cannot simply be
 * copied and put under another distribution licence
 * [including the GNU Public Licence.]
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "des_locl.h"
#include "spr.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void des_encrypt2(FAR uint32_t *data, caddr_t ks, int encrypt)
{
  register uint32_t l;
  register uint32_t r;
  register uint32_t t;
  register uint32_t u;
#ifdef DES_USE_PTR
  FAR register unsigned char *des_sp = (FAR unsigned char *)des_sptrans;
#endif
  register int i;
  FAR register uint32_t *s;

  u = data[0];
  r = data[1];

  /* Things have been modified so that the initial rotate is
   * done outside the loop.  This required the
   * des_SPtrans values in sp.h to be rotated 1 bit to the right.
   * One perl script later and things have a 5% speed up on a sparc2.
   * Thanks to Richard Outerbridge <71755.204@CompuServe.COM>
   * for pointing this out.
   */

  l = (r << 1) | (r >> 31);
  r = (u << 1) | (u >> 31);

  /* clear the top bits on machines with 8byte longs */

  l &= 0xffffffffl;
  r &= 0xffffffffl;

  s = (FAR uint32_t *)ks;

  /* I don't know if it is worth the effort of loop unrolling the
   * inner loop
   */

  if (encrypt)
    {
      for (i = 0; i < 32; i += 4)
        {
          D_ENCRYPT(l, r, i + 0); /*  1 */
          D_ENCRYPT(r, l, i + 2); /*  2 */
        }
    }
  else
    {
      for (i = 30; i > 0; i -= 4)
        {
          D_ENCRYPT(l, r, i - 0); /* 16 */
          D_ENCRYPT(r, l, i - 2); /* 15 */
        }
    }

  l = (l >> 1) | (l << 31);
  r = (r >> 1) | (r << 31);

  /* clear the top bits on machines with 8byte longs */

  l &= 0xffffffffl;
  r &= 0xffffffffl;

  data[0] = l;
  data[1] = r;
  l = r = t = u = 0;
}
