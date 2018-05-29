/****************************************************************************
 * libs/libc/misc/lib_tea_encrypt.c
 * Tiny Encryption Algorithm - Encryption support
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * https://en.wikipedia.org/wiki/Tiny_Encryption_Algorithm:  "Following is
 * an adaptation of the reference encryption and decryption routines in C,
 * released into the public domain by David Wheeler and Roger Needham."
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

#include <stdint.h>
#include <nuttx/crypto/tea.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TEA_KEY_SCHEDULE_CONSTANT  0x9e3779b9
#define TEA_SUM_SETUP_CONSTANT     0xc6ef3720

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tea_encrypt
 *
 * Input Parameters:
 *   value = 2 x 32-bit value (input/output)
 *   key   = 4 x 32-bit Cache key (input)
 *
 ****************************************************************************/

void tea_encrypt(FAR uint32_t *value, FAR const uint32_t *key)
{
  uint32_t v0  = value[0];
  uint32_t v1  = value[1];
  uint32_t sum = 0;
  int i;

  /* Basic cycle start */

  for (i = 0; i < 32; i++)
    {
      sum += TEA_KEY_SCHEDULE_CONSTANT;
      v0  += ((v1 << 4) + key[0]) ^ (v1 + sum) ^ ((v1 >> 5) + key[1]);
      v1  += ((v0 << 4) + key[2]) ^ (v0 + sum) ^ ((v0 >> 5) + key[3]);
    }

  /* End cycle */

  value[0] = v0;
  value[1] = v1;
}
