/****************************************************************************
 * boards/z80/ez80/z20x/src/w25_main.c
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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <debug.h>
#include <hex2bin.h>

#include <nuttx/streams.h>
#include <arch/irq.h>

#include "z20x.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef HAVE_SPIFLASH
#  error The W25 Serial FLASH is not available
#endif

#ifndef CONFIG_Z20X_W25_CHARDEV
#  error CONFIG_Z20X_W25_CHARDEV must be selected
#endif

#if !defined(CONFIG_Z20X_W25_PROGSIZE) || CONFIG_Z20X_W25_PROGSIZE < 128
#  error Large CONFIG_Z20X_W25_PROGSIZE must be selected
#endif

#define IOBUFFER_SIZE   512
#define PROG_MAGIC      0xfedbad

#define SRAM_ENTRY      ((sram_entry_t)PROGSTART)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef CODE void (*sram_entry_t)(void);

/* This is the header places at the beginning of the binary data in FLASH */

struct prog_header_s
{
  uint24_t magic;  /* Valid if PROG_MAGIC */
  uint24_t crc;    /* Last 24-bits of CRC32 */
  uint24_t len;    /* Binary length */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: w25_read_hex
 *
 * Description:
 *   Read and parse a HEX data stream into RAM.  HEX data will be read from
 *   stdin and written to the program area reserved in RAM, that is, the
 *   address range from PROGSTART to PROGREND.
 *
 ****************************************************************************/

static int w25_read_hex(FAR uint24_t *len)
{
  struct lib_rawinstream_s rawinstream;
  struct lib_memsostream_s memoutstream;
  int fd;
  int ret;

  /* Open the W25 device for writing */

  fd = open(W25_CHARDEV, O_WRONLY);
  if (fd < 0)
    {
      ret = -errno;
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", W25_CHARDEV, ret);
      return ret;
    }

  /* Wrap stdin as an IN stream that can get the HEX data over the serial
   * port.
   */

  lib_rawinstream(&rawinstream, 0);

  /* Wrap the memory area used to hold the program as a seek-able OUT stream
   * in which we can buffer the binary data.
   */

  lib_memsostream(&memoutstream, (FAR char *)PROGSTART, PROGSIZE);

  /* We are ready to load the Intel HEX stream into external SRAM.
   *
   * Hmm.. With no hardware handshake, there is a possibility of data loss
   * to overrunning incoming data buffer.  So far I have not seen this at
   * 115200 8N1, but still it is a possibility.
   */

  printf("Send Intel HEX file now\n");
  fflush(stdout);

  ret = hex2bin(&rawinstream.public, &memoutstream.public,
                (uint32_t)PROGSTART, (uint32_t)(PROGSTART + PROGSIZE),
                0);

  close(fd);
  if (ret < 0)
    {
      /* We failed the load */

      fprintf(stderr, "ERROR: Intel HEX file load failed: %d\n", ret);
      return ret;
    }

  printf("Intel HEX file into memory loaded into RAM...\n");
  fflush(stdout);

  *len = memoutstream.public.nput;
  return OK;
}

/****************************************************************************
 * Name: w25_write
 *
 * Description:
 *   Write to the open file descriptor, handling failures and repeated
 *   write operations as necessary.
 *
 ****************************************************************************/

static int w25_write(int fd, FAR const void *src, size_t len)
{
  ssize_t remaining = len;
  ssize_t nwritten;

  do
    {
      nwritten = write(fd, src, remaining);
      if (nwritten <= 0)
        {
          int errcode = errno;
          if (errno != EINTR)
            {
              fprintf(stderr, "ERROR: Write failed: %d\n", errcode);
              return -errcode;
            }
        }
      else
        {
          remaining -= nwritten;
          src += nwritten;
        }
    }
  while (remaining > 0);

  return OK;
}

/****************************************************************************
 * Name: w25_write_binary
 *
 * Description:
 *   Write a program header followed by the binary data beginning at address
 *   PROGSTART into the first partition of the w25 FLASH memory.
 *
 ****************************************************************************/

static int w25_write_binary(FAR const struct prog_header_s *hdr)
{
  int fd;
  int ret;

  /* Open the W25 device for writing */

  fd = open(W25_CHARDEV, O_WRONLY);
  if (fd < 0)
    {
      ret = -errno;
      fprintf(stderr, "ERROR: Failed to open %s: %d\n", W25_CHARDEV, ret);
      return ret;
    }

  /* Write the header to the W25 */

  ret = w25_write(fd, hdr, sizeof(struct prog_header_s));
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed write program header: %d\n", ret);
      goto errout;
    }

  printf("Writing %lu bytes to the W25 Serial FLASH\n",
         (unsigned long)hdr->len);
  fflush(stdout);

  ret = w25_write(fd, (FAR const void *)PROGSTART, hdr->len);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed write program header: %d\n", ret);
    }

errout:
  close(fd);
  return ret;
}

/****************************************************************************
 * Name: w25_read
 *
 * Description:
 *   Read from the open file descriptor, handling errors as retries as
 *   necessary.
 *
 ****************************************************************************/

static int w25_read(int fd, FAR void *dest, size_t len)
{
  ssize_t remaining = len;
  ssize_t nread;

  do
    {
      nread = read(fd, dest, remaining);
      if (nread <= 0)
        {
          int errcode = errno;
          if (errno != EINTR)
            {
              fprintf(stderr, "ERROR: Read failed: %d\n", errcode);
              close(fd);
              return -errcode;
            }
        }

      remaining -= nread;
      dest += nread;
    }
  while (remaining > 0);

  return OK;
}

/****************************************************************************
 * Name: w25_read_binary
 *
 * Description:
 *   Read the program in the first partition of the W25 part into memory
 *   at PROGSTART.
 *
 ****************************************************************************/

static int w25_read_binary(FAR struct prog_header_s *hdr)
{
  int fd;
  int ret;

  /* Open the W25 device for reading */

  fd = open(W25_CHARDEV, O_RDONLY);
  if (fd < 0)
    {
      ret = -errno;
      return ret;
    }

  /* Read the header at the beginning of the partition */

  ret = w25_read(fd, hdr, sizeof(struct prog_header_s));
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed read program header: %d\n", ret);
      goto errout;
    }

  /* Check for a valid program header */

  /* A valid program should have a MAGIC number */

  if (hdr->magic != PROG_MAGIC)
    {
      ret = -ENOENT;
      goto errout;
    }

  /* A valid program should fit in RAM */

  if (hdr->len >= PROGSIZE)
    {
      fprintf(stderr, "ERROR: Program too big\n");
      ret = -E2BIG;
      goto errout;
    }

  /* Read the program binary.  A valid program should also have a matching
   * CRC after loaded to memory.
   */

  printf("Reading %lu bytes from the W25 Serial FLASH\n",
         (unsigned long)hdr->len);
  fflush(stdout);

  ret = w25_read(fd, (FAR void *)PROGSTART, hdr->len);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed read program header: %d\n", ret);
    }

errout:
  close(fd);
  return ret;
}

/****************************************************************************
 * Name: w25_crc24
 *
 * Description:
 *   Calculate a CRC24 checksum on the data loaded into memory starting at
 *   PROGSTART.
 *
 ****************************************************************************/

static uint24_t w25_crc24(uint24_t len)
{
#if 0 /* Very slow */
  FAR const uint8_t *src = (FAR const uint8_t *)PROGSTART;
  uint32_t crc = 0;
  uint24_t i;
  int j;

  /* Loop for each byte in the binary image */

  for (i = 0; i < len; i++)
    {
      uint8_t val = *src++;
      crc ^= (uint32_t)val << 16;

      /* Loop for each bit in each byte */

      for (j = 0; j < 8; j++)
        {
          crc <<= 1;
          if ((crc & 0x1000000) != 0)
            {
              crc ^= 0x1864cfb;
            }
        }
    }

  return (uint24_t)crc;
#else
  FAR const uint24_t *src = (FAR const uint24_t *)PROGSTART;
  uint24_t chksum = 0;
  uint24_t remaining;

  /* Loop for each uint24_t in the binary image */

  for (remaining  = len;
       remaining >= sizeof(uint24_t);
       remaining -= sizeof(uint24_t))
    {
      uint24_t val = *src++;

      /* Simple checksum */

      chksum += val;
    }

  /* Handle trailing partial uint24_t's (assumes little endian) */

  if (remaining > 0)
    {
      uint24_t val = *src;

      switch (remaining)
        {
          case 1:
            val &= 0x0000ff;
            break;

          case 2:
            val &= 0x00ffff;
            break;

          default:  /* Shouldn't happen */
            val = 0;
            break;
        }

      /* Simple checksum */

      chksum += val;
    }

  return chksum;
#endif
}

/****************************************************************************
 * Name: w25_read_verify
 *
 * Description:
 *   Verify that the program in the first partition of the W25 part matches
 *   the program in memory at address PROGSTART.
 *
 ****************************************************************************/

static int w25_read_verify(void)
{
  struct prog_header_s hdr;
  uint24_t crc;
  int fd;
  int ret;

  /* Read the program image into memory */

  ret = w25_read_binary(&hdr);
  if (ret < 0)
    {
      return ret;
    }

  printf("Verifying %lu bytes in RAM\n", (unsigned long)hdr.len);
  fflush(stdout);

  /* Compare CRCs */

  crc = w25_crc24(hdr.len);
  if (crc == hdr.crc)
    {
      printf("Successfully verified %lu bytes\n", (unsigned long)hdr.len);
    }
  else
    {
      fprintf(stderr, "ERROR: CRC check failed\n");
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: w25_write_program
 *
 * Description:
 *   Read the HEX program from serial and write to FLASH.
 *
 ****************************************************************************/

static int w25_write_program(void)
{
  struct prog_header_s hdr;
  int ret;

  /* Read the HEX data into RAM */

  memset(&hdr, 0, sizeof(struct prog_header_s));
  hdr.magic = PROG_MAGIC;

  ret = w25_read_hex(&hdr.len);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to load HEX: %d\n", ret);
      return ret;
    }

  /* Calculate a CRC24 checksum */

  hdr.crc = w25_crc24(hdr.len);

  /* The HEX file load was successful, write the data to FLASH */

  ret = w25_write_binary(&hdr);
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to write to W25: %d\n", ret);
      return ret;
    }

  /* Now verify that the image in memory and the image in FLASH are
   * truly the same.
   */

  ret = w25_read_verify();
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to verify program: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: w25_boot_program
 *
 * Description:
 *   Load the program binary from FLASH and execute it
 *
 ****************************************************************************/

static int w25_boot_program(void)
{
  int ret;

  printf("Booting...\n");
  fflush(stdout);

  /* Load the program into memory and verify it. */

  ret = w25_read_verify();
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed to verify program: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_SERIAL_TERMIOS
  /* Drain all pending Tx output in stdout. "Booting..." message will be
   * lost if the outgoing Tx bytes are not drained.
   */

  ret = tcdrain(1);
  if (ret < 0)
    {
      ret = -errno;
      fprintf(stderr, "ERROR: tcdrain() failed: %d\n", ret);
      return ret;
    }
#endif

  /* Start the successfully loaded program */

  SRAM_ENTRY();
  return ret;    /* Should not get here */
}

/****************************************************************************
 * Name: w25_wait_keypress
 *
 * Description:
 *   Wait the specified number of seconds on or until one of the specified
 *   keys is pressed.
 *
 ****************************************************************************/

static int w25_wait_keypress(FAR char *keyset, int nseconds)
{
  char ch = '\0';
  ssize_t nread;
  int count = 0;
  int oflags;
  int ret;
  int fd;
  int i;
  int j;

  /* Duplicate the file descriptor associated with stdin. */

  fd = dup(0);
  if (fd < 0)
    {
      ret = -errno;
      fprintf(stderr, "ERROR: Failed to dup stdin: %d\n", ret);
      return ret;
    }

  /* Make the duplicated file descriptor non-blocking. */

  ret = fcntl(fd, F_GETFL, 0);
  if (ret >= 0)
    {
      ret = fcntl(fd, F_SETFL, ret | O_NONBLOCK);
    }

  if (ret < 0)
    {
      ret = -errno;
      fprintf(stderr, "ERROR: fcnt() failed: %d\n", ret);
      close(fd) ;
      return ret;
    }

  /* Loop for the requested number of seconds */

  for (i = 0; i < nseconds; i++)
    {
      /* Check for input every 50 milliseconds */

      for (j = 0; j < 20; j++)
        {
          char tmpch;

          /* Read handling retries.
           * We get out of this loop if a key is press.
           */

          for (; ; )
            {
              /* Read one character */

              nread = read(fd, &tmpch, 1);

              /* Check for errors */

              if (nread < 0)
                {
                  int errcode = errno;

                  /* If is not an error if a signal occurred or if there is
                   * no key pressed.
                   */

                  if (errcode == EAGAIN)
                    {
                      /* If no key is pressed, then break out of this inner
                       * loop, delay, and read again.
                       */

                      break;
                    }

                  /* If we were awakened by a signal, then loop and read
                   * again immediately.
                   */

                  if (errcode != EINTR)
                    {
                      /* Punt on all other errors */

                      fprintf(stderr, "ERROR: Read from stdin failed: %d\n",
                              errcode);
                      return -errcode;
                    }
                }
              else if (nread != 1)
                {
                  /* This should never happen */

                  fprintf(stderr, "ERROR: Bad read size: %d\n", (int)nread);
                  return -EIO;
                }

              /* A key was pressed.  Is it one we care about? */

              else if (strchr(keyset, tmpch) != NULL)
                {
                  /* Yes, return the key */

                  ch = tmpch;
                  goto errout;
                }
              else
                {
                  /* No... delay and try again */

                  break;
                }
            }

          /* Delay 50 Milliseconds  */

          nxsig_usleep(50 * 1000);

          /* Output a dot to stdout every 10 * 50 = 500 milliseconds */

          if (++count == 10)
            {
              putchar('.');
              fflush(stdout);
              count = 0;
            }
        }
    }

errout:
  close(fd);
  putchar('\n');
  return ch;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: w25_main
 *
 * Description:
 *   w25_main is a tiny program that runs in ISRAM.  w25_main will
 *   configure DRAM and the W25 serial FLASH, present a prompt, and load
 *   an Intel HEX file into the W25 serial FLASH (after first buffering
 *   the binary data into memory).  On re-boot, the program loaded into
 *   the W25 FLASH should then execute.
 *
 *   On entry:
 *   - SDRAM has already been initialized (we are using it for .bss!)
 *   - SPI0 chip select has already been configured.
 *
 ****************************************************************************/

int w25_main(int argc, char *argv)
{
  bool disable = false;
  int ret;

#ifndef CONFIG_BOARD_LATE_INITIALIZE
  /* Initialize the board.  We can do this with a direct call because we are
   * a kernel thread.
   */

  ret = ez80_bringup();
  if (ret < 0)
    {
      fprintf(stderr, "ERROR: Failed initialize system: %d\n", ret);
      return EXIT_FAILURE;
    }
#endif

  /* Verify the program in FLASH. */

  ret = w25_read_verify();
  if (ret < 0)
    {
      printf("No valid program in FLASH: %d\n", ret);
      disable = true;
    }

  /* Now loop, providing the user with options to load a new program into
   * FLASH or to boot from an existing program in FLASH.
   */

  for (; ; )
    {
      /* Disable booting if there is nothing valid in the FLASH */

      if (disable)
        {
          ret = w25_write_program();
          if (ret >= 0)
            {
              /* There is now a valid program in FLASH */

              disable = false;
            }
        }

      /* Both booting from FLASH and loading to flash are possible */

      else
        {
          /* Wait up to 5 seconds for (L)oad or (B) keys. */

          printf("[L]oad [B]oot\n");
          fflush(stdout);

          ret = w25_wait_keypress("LlBb", 5);
          if (ret < 0)
            {
              return EXIT_FAILURE;
            }

          /* Load HEX command */

          else if (ret == 'L' || ret == 'l')
            {
              ret = w25_write_program();
              if (ret < 0)
                {
                  /* Assume that the program in FLASH has been corrupted. */

                  disable = true;
                }
            }

          /* Boot from FLASH or timeout */

          else /* if (ret == 'B' || ret == 'b' || ret == 0) */
            {
              /* REVISIT:  The program is probably already in RAM.  We may
               * not have to reload and verify it.
               */

              ret = w25_boot_program();

              /* Shouldn't get here unless the FLASH content is bad */

              UNUSED(ret);
              disable = true;
            }
        }

      /* Check for a failure */

      if (ret < 0)
        {
          fprintf(stderr, "ERROR: Operation failed: %d\n", ret);
        }
    }

  return EXIT_SUCCESS;
}
