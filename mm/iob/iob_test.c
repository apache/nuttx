/****************************************************************************
 * mm/iob/iob_test.c
 * Unit test driver.  This is of historical interest only since it requires
 * and custom build setup and modifications to the iob source and header
 * files.
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "iob.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t buffer1[16384];
static uint8_t buffer2[16384];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void dump_chain(struct iob_s *iob)
{
  struct iob_s *head = iob;
  unsigned int pktlen;
  int n;

  printf("=========================================================\n");
  printf("pktlen: %d\n", iob->io_pktlen);

  n = 0;
  pktlen = 0;

  while (iob)
    {
      printf("%d. len=%d, offset=%d\n", n, iob->io_len, iob->io_offset);

      pktlen += iob->io_len;
      iob = iob->io_flink;
      n++;
    }

  if (pktlen != head->io_pktlen)
    {
      printf("ERROR: Bad packet length=%u, actual=%u\n",
             head->io_pktlen, pktlen);
    }

  printf("=========================================================\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   A simple unit test for the I/O buffer logic
 *
 ****************************************************************************/

int main(int argc, char **argv)
{
  struct iob_s *iob;
  int nbytes;
  int i;

  iob_initialize();
  iob = iob_alloc(false);

  for (i = 0; i < 4096; i++)
    {
      buffer1[i] = (uint8_t)(i & 0xff);
    }
  memset(buffer2, 0xff, 4096);

  iob_copyin(iob, buffer2, 47, 0, false);
  printf("Copy IN: 47, offset 0\n");
  dump_chain(iob);

  iob_copyin(iob, buffer1, 4096, 47, false);
  printf("Copy IN: 4096, offset 47\n");
  dump_chain(iob);

  nbytes = iob_copyout(buffer2, iob, 4096, 47);
  printf("Copy OUT: %d, offset 47\n", nbytes);

  if (memcmp(buffer1, buffer2, nbytes) != 0)
    {
      fprintf(stderr, "Buffer1 does not match buffer2\n");
    }

  iob = iob_trimhead(iob, 47);
  printf("Trim: 47 from the beginning of the list\n");
  dump_chain(iob);

  iob = iob_trimtail(iob, 493);
  printf("Trim: 493 from the end of the list\n");
  dump_chain(iob);

  nbytes = iob_copyout(buffer2, iob, 4096, 0);
  printf("Copy OUT: %d, offset 0\n", nbytes);

  if (memcmp(buffer1, buffer2, nbytes) != 0)
    {
      fprintf(stderr, "Buffer1 does not match buffer2\n");
    }

  iob = iob_trimhead(iob, 1362);
  printf("Trim: 1362 from the beginning of the list\n");
  dump_chain(iob);

  nbytes = iob_copyout(buffer2, iob, 4096, 0);
  printf("Copy OUT: %d, offset 0\n", nbytes);

  if (memcmp(&buffer1[1362], buffer2, nbytes) != 0)
    {
      fprintf(stderr, "Buffer1 does not match buffer2\n");
    }

  iob = iob_pack(iob);
  printf("Packed\n");
  dump_chain(iob);

  nbytes = iob_copyout(buffer2, iob, 4096, 0);
  printf("Copy OUT: %d, offset 0\n", nbytes);

  if (memcmp(&buffer1[1362], buffer2, nbytes) != 0)
    {
      fprintf(stderr, "Buffer1 does not match buffer2\n");
    }

  while (iob) iob = iob_free(iob);
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Name: my_assert
 *
 * Description:
 *   A stand-in for the NuttX assertion routine.
 *
 ****************************************************************************/

void my_assert(bool value)
{
  if (!value)
    {
      fprintf(stderr, "Assertion failed\n");

      abort();
    }
}
