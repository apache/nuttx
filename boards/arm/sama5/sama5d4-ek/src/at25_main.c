/****************************************************************************
 * boards/arm/sama5/sama5d4-ek/src/at25_main.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <hex2bin.h>

#include <nuttx/streams.h>
#include <arch/irq.h>

#include "sama5d4-ek.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef HAVE_AT25
#  error The AT25 Serial FLASH is not available
#endif

#ifndef CONFIG_SAMA5D4EK_AT25_BLOCKMOUNT
#  error CONFIG_SAMA5D4EK_AT25_BLOCKMOUNT must be selected
#endif

#ifndef CONFIG_SAMA5D4EK_AT25_CHARDEV
#  error CONFIG_SAMA5D4EK_AT25_CHARDEV must be selected
#endif

#ifdef CONFIG_BOOT_SDRAM_DATA
#  error CONFIG_BOOT_SDRAM_DATA must NOT be selected
#endif

#if !defined(CONFIG_SAMA5D4EK_AT25_PROGSIZE) || CONFIG_SAMA5D4EK_AT25_PROGSIZE < 128
#  error Large CONFIG_SAMA5D4EK_AT25_PROGSIZE must be selected
#endif

#define DEVNAME_MAXSIZE 12
#define IOBUFFER_SIZE   512

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_iobuffer[IOBUFFER_SIZE];
static char g_at25dev[DEVNAME_MAXSIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at25_main
 *
 * Description:
 *   at25_main is a tiny program that runs in ISRAM.  at25_main will
 *   configure DRAM and the AT25 serial FLASH, present a prompt, and load
 *   an Intel HEX file into the AT25 serial FLASH (after first buffering
 *   the binary data into memory).  On re-boot, the program loaded into
 *   the AT25 FLASH should then execute.
 *
 *   On entry:
 *   - SDRAM has already been initialized (we are using it for .bss!)
 *   - SPI0 chip select has already been configured.
 *
 ****************************************************************************/

int at25_main(int argc, char *argv)
{
  struct lib_rawinstream_s rawinstream;
  struct lib_memsostream_s memoutstream;
  const uint8_t *src;
  ssize_t nwritten;
  ssize_t nread;
  ssize_t remaining;
  size_t rdsize;
  int fd;
  int ret;

  /* Configure the AT25 as the a character device.  */

  ret = sam_at25_automount(AT25_MINOR);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to mount AT25 FLASH: %d\n", ret);
      return EXIT_FAILURE;
    }

  /* Open the AT25 device for writing */

  snprintf(g_at25dev, DEVNAME_MAXSIZE, "/dev/mtd%d", AT25_MINOR);
  fd = open(g_at25dev, O_WRONLY);
  if (fd < 0)
    {
      int errcode = get_errno();
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", g_at25dev, errcode);
      return EXIT_FAILURE;
    }

  /* Wrap stdin as an IN stream that can the HEX data over the serial port */

  lib_rawinstream(&rawinstream, 0);

  /* Define a memory buffer of size CONFIG_SAMA5D4EK_AT25_PROGSIZE at the
   * beginning of SDRAM. Wrap the memory buffer as a seek-able OUT stream in
   * which we can buffer the binary data.
   */

  lib_memsostream(&memoutstream, (FAR char *)SAM_DDRCS_VSECTION,
                  CONFIG_SAMA5D4EK_AT25_PROGSIZE);

  /* We are ready to load the Intel HEX stream into DRAM.
   *
   * Hmm.. With no hardware handshake, there is a possibility of data loss
   * to overrunning incoming data buffer.  So far I have not seen this at
   * 115200 8N1, but still it is a possibility.
   */

  printf("Send Intel HEX file now\n");
  fflush(stdout);

  ret = hex2bin(&rawinstream.public, &memoutstream.public,
                (uint32_t)SAM_ISRAM_VSECTION,
                (uint32_t)(SAM_ISRAM_VSECTION + CONFIG_SAMA5D4EK_AT25_PROGSIZE),
                0);
  if (ret < 0)
    {
      /* We failed the load */

      fprintf(stderr, "ERROR: Intel HEX file load failed: %d\n", ret);
      close(fd);
      return EXIT_FAILURE;
    }

  /* Replace the vector at offset 0x14 with the size of the image to load
   * into SRAM.  The RomBOOT loader expects to find this size at that
   * location.
   */

  *(uint32_t *)(SAM_DDRCS_VSECTION + 0x14) = memoutstream.public.nput;

  /* The HEX file load was successful, write the data to FLASH */

  printf("Successfully loaded the Intel HEX file into memory...\n");
  printf("  Writing %d bytes to the AT25 Serial FLASH\n",
         memoutstream.public.nput);

  remaining = memoutstream.public.nput;
  src = (uint8_t *)SAM_DDRCS_VSECTION;

  do
    {
      nwritten = write(fd, src, memoutstream.public.nput);
      if (nwritten <= 0)
        {
          int errcode = get_errno();
          if (errno != EINTR)
            {
              fprintf(stderr, "ERROR: Write failed: %d\n", errcode);
              close(fd);
              return EXIT_FAILURE;
            }
        }
      else
        {
          remaining -= nwritten;
          src += nwritten;
        }
    }
  while (remaining > 0);

  close(fd);

  /* Now verify that the image in memory and the image in FLASH are truly
   * the same.
   */

  printf("  Verifying %d bytes in the AT25 Serial FLASH\n",
         memoutstream.public.nput);

  /* Open the AT25 device for writing */

  fd = open(g_at25dev, O_RDONLY);
  if (fd < 0)
    {
      int errcode = get_errno();
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", g_at25dev, errcode);
      return EXIT_FAILURE;
    }

  remaining = memoutstream.public.nput;
  src = (const uint8_t *)SAM_DDRCS_VSECTION;

  do
    {
      rdsize = remaining;
      if (rdsize > IOBUFFER_SIZE)
        {
          rdsize = IOBUFFER_SIZE;
        }

      nread = read(fd, g_iobuffer, rdsize);
      if (nread <= 0)
        {
          int errcode = get_errno();
          if (errno != EINTR)
            {
              fprintf(stderr, "ERROR: Read failed: %d\n", errcode);
              close(fd);
              return EXIT_FAILURE;
            }
        }
      else
        {
          if (memcmp(g_iobuffer, src, nread) != 0)
            {
              fprintf(stderr, "ERROR: Verify failed at offset %d\n",
                      memoutstream.public.nput - remaining);
              close(fd);
              return EXIT_FAILURE;
            }

          remaining -= nwritten;
          src += nwritten;
        }
    }
  while (remaining > 0);

  printf("  Successfully verified %d bytes in the AT25 Serial FLASH\n",
         memoutstream.public.nput);

  close(fd);
  return EXIT_SUCCESS;
}
