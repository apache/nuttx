/****************************************************************************
 * boards/risc-v/ab32vg1/ab32vg1-generic/tools/ab32vg1_dl.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* ab32vg1_dl - load and flash NuttX on the Bluetrum AB32VG1 over the mask
 * ROM UART loader.
 *
 * The AB32VG1 mask ROM listens on PA7 (single wire, 115200 8N1) for a
 * short time after reset.  Its protocol was documented by Andrey Grigoryev
 * in https://github.com/kagaimiq/bluetrum-tools; this is an independent
 * implementation.  Flash programming uses our own stub (target/stub.c),
 * which this tool loads into RAM through the ROM loader.  Flash images
 * start with a Bluetrum boot header (format documented in the same
 * project) carrying our stage-1 loader (target/boot.c).
 *
 * See Documentation/platforms/risc-v/ab32vg1/boards/ab32vg1-generic.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "target/stub.h"
#include "ab32vg1_stub.h"
#include "ab32vg1_boot.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEFAULT_PORT      "/dev/ttyUSB0"
#define ROM_BAUD          115200
#define STUB_BAUD         921600
#define CONSOLE_BAUD      115200

/* Mask ROM UART loader protocol */

#define SYNC_TOKEN        "\xa5\x96\x87\x5a"
#define SYNC_RESP         "\x5a\x69\x78\xa5"
#define RESET_TOKEN       "\xf5\xa0"

#define RESP_ACK          0x1e
#define RESP_NAK          0x2d
#define RESP_NYET         0x3c
#define TOK_DATA          0x4b
#define TOK_DATA_REQ      0xb4
#define TOK_PING          0xc3

#define CMD_MEM_READ      0x52
#define CMD_AUTHORIZE     0x55
#define CMD_MEM_WRITE     0x57
#define CMD_SET_HANDLER   0x58
#define CMD_GET_INFO      0x5a

#define ROM_BLOCK         512

/* Flash image layout */

#define BOOT_LOAD         0x00010800  /* Stage-1 load/entry address */
#define BOOT_CODE_OFF     0x400       /* Boot code offset in the header */
#define BOOT_SECTOR       0x1000      /* Boot header + code */
#define IMG_TABLE         0x1000      /* Must match target/target.h */
#define IMG_DATA          0x2000      /* First segment */
#define IMG_MAGIC         0x42415858
#define IMG_MAXSEG        4

#define MAGICKEY_LVMG     0x474d564c  /* Boot header/code scrambling key */

#define FLASH_SIZE        0x100000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct segment_s
{
  uint32_t addr;
  uint32_t size;
  uint8_t *data;
};

struct image_s
{
  uint32_t entry;
  int nseg;
  struct segment_s seg[IMG_MAXSEG];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_fd = -1;
static int g_tries = 12;
static uint8_t g_counter;
static bool g_ping_first;
static bool g_verbose;
static bool g_noboot;
static bool g_force;
static int g_watch;
static int g_connect_timeout = 120;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void die(const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  fprintf(stderr, "ab32vg1_dl: ");
  vfprintf(stderr, fmt, ap);
  fprintf(stderr, "\n");
  va_end(ap);
  exit(EXIT_FAILURE);
}

static double now(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec / 1e9;
}

/* CRCs *********************************************************************/

static uint16_t crc16(uint16_t crc, const uint8_t *buf, size_t len)
{
  while (len-- > 0)
    {
      crc ^= (uint16_t)*buf++ << 8;
      for (int i = 0; i < 8; i++)
        {
          crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }

  return crc;
}

static uint32_t crc32(uint32_t crc, const uint8_t *buf, size_t len)
{
  crc = ~crc;
  while (len-- > 0)
    {
      crc ^= *buf++;
      for (int i = 0; i < 8; i++)
        {
          crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1));
        }
    }

  return ~crc;
}

/* Bluetrum LFSR scrambler (x^32 + x^30 + x^26 + x^25) */

static void lfsr_xor(uint8_t *buf, size_t len, uint32_t key)
{
  static uint32_t table[256];
  static bool init;

  if (!init)
    {
      for (uint32_t i = 0; i < 256; i++)
        {
          uint32_t reg = i;

          for (int j = 0; j < 8; j++)
            {
              reg = (reg >> 1) ^ ((reg & 1) ? 0xa3000000 : 0);
            }

          table[i] = reg;
        }

      init = true;
    }

  while (len-- > 0)
    {
      *buf++ ^= key & 0xff;
      key = (key >> 8) ^ table[key & 0xff];
    }
}

static void put16(uint8_t *p, uint32_t v)
{
  p[0] = v;
  p[1] = v >> 8;
}

static void put32(uint8_t *p, uint32_t v)
{
  p[0] = v;
  p[1] = v >> 8;
  p[2] = v >> 16;
  p[3] = v >> 24;
}

static uint32_t get32(const uint8_t *p)
{
  return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint32_t get32be(const uint8_t *p)
{
  return ((uint32_t)p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
}

/* Serial port **************************************************************/

static speed_t baud_constant(unsigned int baud)
{
  switch (baud)
    {
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      case 460800:
        return B460800;
      case 921600:
        return B921600;
      case 1500000:
        return B1500000;
      default:
        die("unsupported baud rate %u", baud);
        return B0;
    }
}

static void serial_baud(unsigned int baud)
{
  struct termios tio;

  if (tcgetattr(g_fd, &tio) < 0)
    {
      die("tcgetattr: %s", strerror(errno));
    }

  cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~(CSTOPB | CRTSCTS);
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, baud_constant(baud));
  cfsetospeed(&tio, baud_constant(baud));
  if (tcsetattr(g_fd, TCSANOW, &tio) < 0)
    {
      die("tcsetattr: %s", strerror(errno));
    }
}

static void serial_open(const char *port, unsigned int baud)
{
  g_fd = open(port, O_RDWR | O_NOCTTY);
  if (g_fd < 0)
    {
      die("cannot open %s: %s", port, strerror(errno));
    }

  serial_baud(baud);
  tcflush(g_fd, TCIOFLUSH);
}

static void serial_write(const void *buf, size_t len)
{
  const uint8_t *p = buf;

  while (len > 0)
    {
      ssize_t n = write(g_fd, p, len);

      if (n < 0)
        {
          if (errno == EINTR)
            {
              continue;
            }

          die("write: %s", strerror(errno));
        }

      p   += n;
      len -= n;
    }

  tcdrain(g_fd);
}

/* Read up to len bytes, waiting at most timeout_ms for each byte */

static size_t serial_read(void *buf, size_t len, int timeout_ms)
{
  uint8_t *p = buf;
  size_t got = 0;

  while (got < len)
    {
      struct pollfd pfd =
        {
          .fd = g_fd, .events = POLLIN
        };

      int ret = poll(&pfd, 1, timeout_ms);
      ssize_t n;

      if (ret <= 0)
        {
          break;
        }

      n = read(g_fd, p + got, len - got);
      if (n <= 0)
        {
          break;
        }

      got += n;
    }

  return got;
}

static void serial_flushin(void)
{
  tcflush(g_fd, TCIFLUSH);
}

/* The CH340 TX and RX are joined into one wire: everything sent comes back
 * and must be dropped.
 */

static void wire_write(const void *buf, size_t len)
{
  uint8_t echo[4096 + 16];

  serial_write(buf, len);
  while (len > 0)
    {
      size_t n = len > sizeof(echo) ? sizeof(echo) : len;

      if (serial_read(echo, n, 100) < n)
        {
          break;
        }

      len -= n;
    }
}

static int stub_request(uint8_t cmd, uint32_t addr, const uint8_t *data,
                        size_t len, uint8_t *rsp, size_t rspmax,
                        size_t *rsplen, int timeout_ms);

/* Mask ROM UART loader *****************************************************/

static void rom_token(uint8_t *pkt, uint8_t tok)
{
  pkt[0] = tok;
  pkt[1] = ++g_counter;
}

static int rom_recv_token(void)
{
  uint8_t r[2];

  if (serial_read(r, 2, 100) < 2)
    {
      return -1;
    }

  return r[0];
}

static int rom_send_packet(const uint8_t *data, size_t len)
{
  uint8_t pkt[2 + 2 + ROM_BLOCK + 2];
  bool ping = g_ping_first;

  pkt[0] = TOK_DATA;
  put16(pkt + 2, len);
  memcpy(pkt + 4, data, len);
  put16(pkt + 4 + len, crc16(0xffff, data, len));

  for (; ; )
    {
      uint8_t ppkt[2];
      int resp = -1;

      if (ping)
        {
          rom_token(ppkt, TOK_PING);
        }
      else
        {
          pkt[1] = ++g_counter;
        }

      for (int tries = 0; tries < g_tries && resp < 0; tries++)
        {
          if (ping)
            {
              wire_write(ppkt, 2);
            }
          else
            {
              wire_write(pkt, 4 + len + 2);
            }

          resp = rom_recv_token();
        }

      if (resp < 0)
        {
          return -ETIMEDOUT;
        }

      if (ping)
        {
          if (resp == RESP_ACK)
            {
              ping = false;
            }
          else if (resp != RESP_NAK)
            {
              return -EPROTO;
            }
        }
      else if (resp == RESP_ACK || resp == RESP_NYET)
        {
          g_ping_first = resp == RESP_NYET;
          return 0;
        }
      else if (resp == RESP_NAK)
        {
          ping = true;
        }
      else
        {
          return -EPROTO;
        }
    }
}

static int rom_recv_packet(uint8_t *buf, size_t max)
{
  uint8_t req[2];

  rom_token(req, TOK_DATA_REQ);
  for (int tries = 0; tries < 50; tries++)
    {
      uint8_t hdr[2];
      uint8_t crc[2];
      size_t size;
      int resp;

      wire_write(req, 2);
      resp = rom_recv_token();
      if (resp < 0 || resp == RESP_NAK)
        {
          continue;
        }

      if (resp != TOK_DATA)
        {
          return -EPROTO;
        }

      if (serial_read(hdr, 2, 100) < 2)
        {
          continue;
        }

      size = hdr[0] | (hdr[1] << 8);
      if (size > max || serial_read(buf, size, 100) < size ||
          serial_read(crc, 2, 100) < 2)
        {
          continue;
        }

      if (crc16(0xffff, buf, size) == (crc[0] | (crc[1] << 8)))
        {
          return size;
        }
    }

  return -ETIMEDOUT;
}

static void rom_cmdblock(uint8_t *cb, uint8_t cmd, uint32_t arg1,
                         uint8_t arg2, uint16_t arg3)
{
  cb[0] = cmd;
  cb[1] = arg1 >> 24;
  cb[2] = arg1 >> 16;
  cb[3] = arg1 >> 8;
  cb[4] = arg1;
  cb[5] = arg2;
  cb[6] = arg3 >> 8;
  cb[7] = arg3;
}

static int rom_execute(const uint8_t *cb, const uint8_t *send,
                       size_t sendlen, uint8_t *recv, size_t recvlen)
{
  int ret = rom_send_packet(cb, 8);

  for (size_t off = 0; ret >= 0 && off < sendlen; off += ROM_BLOCK)
    {
      size_t n = sendlen - off > ROM_BLOCK ? ROM_BLOCK : sendlen - off;

      ret = rom_send_packet(send + off, n);
    }

  for (size_t off = 0; ret >= 0 && off < recvlen; )
    {
      ret = rom_recv_packet(recv + off, recvlen - off);
      if (ret > 0)
        {
          off += ret;
        }
      else if (ret == 0)
        {
          break;
        }
    }

  return ret < 0 ? ret : 0;
}

static uint32_t rom_calckey(uint32_t key)
{
  uint8_t b[4];
  uint16_t c1;
  uint16_t c2;

  key ^= 0x5555aaaa;
  put32(b, key);
  c1 = crc16(0xffff, b, 4);
  put32(b, key ^ 0xffffffff);
  c2 = crc16(0xffff, b, 4);
  return ((uint32_t)c1 << 16) | c2;
}

/* GET_INFO and AUTHORIZE.  Returns the load address or 0 on failure. */

static uint32_t rom_hello(bool print)
{
  uint8_t cb[8];
  uint8_t r[24];

  rom_cmdblock(cb, CMD_GET_INFO, 0x5259414e, 0, 0x67ca);
  if (rom_execute(cb, NULL, 0, r, 24) < 0)
    {
      return 0;
    }

  rom_cmdblock(cb, CMD_AUTHORIZE, rom_calckey(get32be(r + 16)), 0, 0);
  if (rom_execute(cb, NULL, 0, r + 20, 4) < 0)
    {
      return 0;
    }

  if (print)
    {
      printf("chip id      : %.8s %u.%u.%u.%u\n", r, r[8], r[9], r[10],
             r[11]);
      printf("load address : 0x%08x\n", get32be(r + 12));
    }

  return get32be(r + 12);
}

/* The reset token followed by the sync token resets the chip when the ROM
 * loader is active (sent back to back), and resets a running NuttX with
 * CONFIG_AB32VG1_UART0_ROMKEY (paced, so that its console echo does not
 * collide with it on the single wire).
 */

static void rom_send_key(bool paced)
{
  static const uint8_t key[] = RESET_TOKEN SYNC_TOKEN;

  if (!paced)
    {
      serial_write(key, 6);
      return;
    }

  for (int i = 0; i < 6; i++)
    {
      serial_write(&key[i], 1);
      usleep(3000);
    }
}

static uint32_t rom_connect(int timeout, bool print)
{
  uint8_t buf0[16];
  uint32_t load;
  double t0;
  double tkey = 0;
  int nkey = 0;

  /* A session left open by an earlier run only accepts packets with the
   * next sequence number; the reset token alone restarts the sequence.
   */

  serial_write(RESET_TOKEN, 2);
  usleep(20000);
  serial_flushin();
  g_counter = 0;
  g_ping_first = false;
  g_tries = 1;
  load = rom_hello(print);
  g_tries = 12;
  if (load != 0)
    {
      return load;
    }

  /* A flash stub left running by an earlier command: reset the chip
   * through it and catch the ROM loader right after the reset.
   */

  serial_baud(STUB_BAUD);
  if (stub_request(STUB_CMD_INFO, 0, NULL, 0, buf0, sizeof(buf0), NULL,
                   100) == 0)
    {
      stub_request(STUB_CMD_RESET, 0, NULL, 0, NULL, 0, NULL, 100);
    }

  serial_baud(ROM_BAUD);

  fprintf(stderr, "waiting for the ROM loader, press RESET now...\n");

  for (t0 = now(); now() - t0 < timeout; )
    {
      uint8_t buf[16];
      size_t n;

      if (now() - tkey > 0.5)
        {
          rom_send_key(nkey++ % 4 == 0);
          tkey = now();
        }

      serial_flushin();
      serial_write(SYNC_TOKEN, 4);
      n = serial_read(buf, sizeof(buf), 10);
      if (n >= 4 && memmem(buf, n, SYNC_RESP, 4) != NULL)
        {
          g_counter = 0;
          g_ping_first = false;
          load = rom_hello(print);
          if (load != 0)
            {
              return load;
            }
        }
    }

  die("no answer from the ROM loader");
  return 0;
}

static void rom_write_mem(uint32_t addr, const uint8_t *data, size_t len,
                          bool verify)
{
  size_t padded = (len + ROM_BLOCK - 1) & ~(size_t)(ROM_BLOCK - 1);
  uint8_t *buf = calloc(1, padded);
  uint8_t *back = calloc(1, padded);
  uint8_t cb[8];

  if (buf == NULL || back == NULL)
    {
      die("out of memory");
    }

  memcpy(buf, data, len);
  printf("loading %zu bytes at 0x%08x\n", len, addr);
  rom_cmdblock(cb, CMD_MEM_WRITE, addr, 0, padded / ROM_BLOCK);
  if (rom_execute(cb, buf, padded, NULL, 0) < 0)
    {
      die("MEM_WRITE at 0x%08x failed", addr);
    }

  if (verify)
    {
      rom_cmdblock(cb, CMD_MEM_READ, addr, 0, padded / ROM_BLOCK);
      if (rom_execute(cb, NULL, 0, back, padded) < 0 ||
          memcmp(buf, back, padded) != 0)
        {
          die("verification of 0x%08x failed", addr);
        }
    }

  free(buf);
  free(back);
}

/* Install the entry point as command handler and send one command: the
 * ROM calls the image instead of answering, so do not wait for an ACK.
 */

static void rom_start(uint32_t entry)
{
  uint8_t cb[8];
  uint8_t pkt[2 + 2 + 8 + 2];

  rom_cmdblock(cb, CMD_SET_HANDLER, entry, 0, 0);
  if (rom_execute(cb, NULL, 0, NULL, 0) < 0)
    {
      die("SET_CMD_HANDLER failed");
    }

  rom_cmdblock(cb, 0, 0, 0, 0);
  rom_token(pkt, TOK_DATA);
  put16(pkt + 2, 8);
  memcpy(pkt + 4, cb, 8);
  put16(pkt + 12, crc16(0xffff, cb, 8));
  wire_write(pkt, sizeof(pkt));
}

/* ELF images ***************************************************************/

static void load_elf(const char *path, struct image_s *img)
{
  FILE *f = fopen(path, "rb");
  uint8_t eh[52];
  uint32_t phoff;
  int phnum;
  int phentsize;

  if (f == NULL)
    {
      die("cannot open %s: %s", path, strerror(errno));
    }

  if (fread(eh, 1, sizeof(eh), f) != sizeof(eh) ||
      memcmp(eh, "\177ELF", 4) != 0 || eh[4] != 1 || eh[5] != 1 ||
      (eh[18] | (eh[19] << 8)) != 243)
    {
      die("%s is not a 32-bit little endian RISC-V ELF file", path);
    }

  memset(img, 0, sizeof(*img));
  img->entry = get32(eh + 24);
  phoff      = get32(eh + 28);
  phentsize  = eh[42] | (eh[43] << 8);
  phnum      = eh[44] | (eh[45] << 8);

  for (int i = 0; i < phnum; i++)
    {
      uint8_t ph[32];
      struct segment_s *seg;
      uint32_t off;

      if (fseek(f, phoff + i * phentsize, SEEK_SET) != 0 ||
          fread(ph, 1, sizeof(ph), f) != sizeof(ph))
        {
          die("%s: bad program header", path);
        }

      /* PT_LOAD segments with data in the file */

      if (get32(ph) != 1 || get32(ph + 16) == 0)
        {
          continue;
        }

      if (img->nseg == IMG_MAXSEG)
        {
          die("%s: too many segments", path);
        }

      seg       = &img->seg[img->nseg++];
      off       = get32(ph + 4);
      seg->addr = get32(ph + 12);              /* p_paddr */
      seg->size = get32(ph + 16);              /* p_filesz */
      seg->data = malloc(seg->size);
      if (seg->data == NULL || fseek(f, off, SEEK_SET) != 0 ||
          fread(seg->data, 1, seg->size, f) != seg->size)
        {
          die("%s: cannot read segment %d", path, i);
        }
    }

  fclose(f);
  if (img->nseg == 0)
    {
      die("%s: no loadable segment", path);
    }
}

/* Flash programming stub ***************************************************/

static int stub_request(uint8_t cmd, uint32_t addr, const uint8_t *data,
                        size_t len, uint8_t *rsp, size_t rspmax,
                        size_t *rsplen, int timeout_ms)
{
  uint8_t req[1 + 7 + STUB_MAXDATA + 2];
  uint8_t hdr[3];
  uint8_t crc[2];
  size_t size;

  req[0] = STUB_REQ_SYNC;
  req[1] = cmd;
  put16(req + 2, len);
  put32(req + 4, addr);
  if (len > 0)
    {
      memcpy(req + 8, data, len);
    }

  put16(req + 8 + len, crc16(0xffff, req + 1, 7 + len));
  serial_flushin();
  wire_write(req, 8 + len + 2);

  /* Wait for the sync byte of the response */

  for (; ; )
    {
      uint8_t b;

      if (serial_read(&b, 1, timeout_ms) < 1)
        {
          return -ETIMEDOUT;
        }

      if (b == STUB_RSP_SYNC)
        {
          break;
        }
    }

  if (serial_read(hdr, 3, 200) < 3)
    {
      return -ETIMEDOUT;
    }

  size = hdr[1] | (hdr[2] << 8);
  if (size > rspmax || serial_read(rsp, size, 200) < size ||
      serial_read(crc, 2, 200) < 2)
    {
      return -EPROTO;
    }

  if (crc16(crc16(0xffff, hdr, 3), rsp, size) != (crc[0] | (crc[1] << 8)))
    {
      return -EPROTO;
    }

  if (rsplen != NULL)
    {
      *rsplen = size;
    }

  return hdr[0] == STUB_OK ? 0 : -EIO;
}

static void stub_call(uint8_t cmd, uint32_t addr, const uint8_t *data,
                      size_t len, uint8_t *rsp, size_t rspmax,
                      size_t *rsplen, int timeout_ms)
{
  int ret = -1;

  for (int tries = 0; tries < 3 && ret < 0; tries++)
    {
      ret = stub_request(cmd, addr, data, len, rsp, rspmax, rsplen,
                         timeout_ms);
      if (ret == -EIO)
        {
          break;
        }
    }

  if (ret < 0)
    {
      die("stub command 0x%02x at 0x%06x failed (%d)", cmd, addr, ret);
    }
}

/* Load and start the stub, switch to its baud rate and check the flash */

static void stub_connect(void)
{
  uint8_t stub[sizeof(g_ab32vg1_stub)];
  uint8_t rsp[16];
  size_t n;
  int ret = -1;

  memcpy(stub, g_ab32vg1_stub, sizeof(stub));
  if (get32(stub + 4) != 0x54534241)
    {
      die("bad stub image");
    }

  put32(stub + 8, STUB_BAUD);

  /* Reuse a stub left running by an earlier command */

  serial_baud(STUB_BAUD);
  ret = stub_request(STUB_CMD_INFO, 0, NULL, 0, rsp, sizeof(rsp), &n, 100);
  if (ret < 0)
    {
      serial_baud(ROM_BAUD);
      rom_connect(g_connect_timeout, false);
      rom_write_mem(0x00012000, stub, sizeof(stub), true);
      rom_start(0x00012000);
      usleep(50000);
      serial_baud(STUB_BAUD);
    }

  for (int tries = 0; tries < 20 && ret < 0; tries++)
    {
      ret = stub_request(STUB_CMD_INFO, 0, NULL, 0, rsp, sizeof(rsp), &n,
                         200);
    }

  if (ret < 0 || n < 6)
    {
      die("no answer from the flash stub");
    }

  printf("flash        : JEDEC id %06x, status %02x (stub v%u)\n",
         get32(rsp) & 0xffffff, rsp[4], rsp[5]);
  if ((get32(rsp) & 0xffffff) == 0 || (get32(rsp) & 0xffffff) == 0xffffff)
    {
      die("no flash detected");
    }

  if (g_verbose)
    {
      static const uint16_t regs[] =
        {
          0x064, 0x074, 0x3a8, 0x3bc, 0x398, 0x38c, 0x3b8, 0x54c, 0x540,
          0x0b0, 0x0a8, 0x3b0, 0x3fc
        };

      for (unsigned int r = 0; r < sizeof(regs) / sizeof(regs[0]); r++)
        {
          stub_call(STUB_CMD_PEEK, regs[r], NULL, 0, rsp, sizeof(rsp), NULL,
                    200);
          printf("%s%03x=%08x", r % 4 ? " " : "\n  ", regs[r], get32(rsp));
        }

      printf("\n");
    }
}

static uint32_t flash_crc(uint32_t addr, uint32_t len)
{
  uint8_t sz[4];
  uint8_t rsp[4];

  put32(sz, len);
  stub_call(STUB_CMD_CRC32, addr, sz, 4, rsp, sizeof(rsp), NULL, 5000);
  return get32(rsp);
}

static void flash_erase(uint32_t addr, uint32_t len)
{
  uint32_t end = (addr + len + 0xfff) & ~0xfffu;

  addr &= ~0xfffu;
  while (addr < end)
    {
      if ((addr & 0xffff) == 0 && end - addr >= 0x10000)
        {
          stub_call(STUB_CMD_ERASE64K, addr, NULL, 0, NULL, 0, NULL, 3000);
          addr += 0x10000;
        }
      else
        {
          stub_call(STUB_CMD_ERASE4K, addr, NULL, 0, NULL, 0, NULL, 1000);
          addr += 0x1000;
        }
    }
}

static void flash_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
  for (uint32_t off = 0; off < len; off += STUB_MAXDATA)
    {
      uint32_t n = len - off > STUB_MAXDATA ? STUB_MAXDATA : len - off;

      stub_call(STUB_CMD_WRITE, addr + off, data + off, n, NULL, 0, NULL,
                2000);
      if (!g_verbose)
        {
          printf("\rwriting      : %u%%", (unsigned)((off + n) * 100 / len));
          fflush(stdout);
        }
    }

  printf("\n");
}

static void flash_program(uint32_t addr, const uint8_t *data, uint32_t len)
{
  flash_erase(addr, len);
  flash_write(addr, data, len);
  if (flash_crc(addr, len) != crc32(0, data, len))
    {
      die("verification at 0x%06x failed", addr);
    }
}

static void flash_read(uint32_t addr, uint8_t *data, uint32_t len)
{
  for (uint32_t off = 0; off < len; off += STUB_MAXDATA)
    {
      uint32_t n = len - off > STUB_MAXDATA ? STUB_MAXDATA : len - off;
      uint8_t sz[4];
      size_t got;

      put32(sz, n);
      stub_call(STUB_CMD_READ, addr + off, sz, 4, data + off, n, &got,
                2000);
      if (got != n)
        {
          die("short read at 0x%06x", addr + off);
        }

      printf("\rreading      : %u%%", (unsigned)((off + n) * 100 / len));
      fflush(stdout);
    }

  printf("\n");
}

static void stub_reset(void)
{
  stub_request(STUB_CMD_RESET, 0, NULL, 0, NULL, 0, NULL, 500);
}

/* Flash image **************************************************************/

/* Boot sector: boot header + stage-1 loader, in the format the mask ROM
 * expects (see kagaimiq/bluetrum-tools mkheader.py --bootable).
 */

static void make_boot_sector(uint8_t *sec)
{
  static const uint8_t chipid[8] =
    {
      'C', 'R', 'W', 'N', 1, 0, 0, 0
    };

  uint8_t *code = sec + BOOT_CODE_OFF;
  uint32_t codelen = BOOT_SECTOR - BOOT_CODE_OFF;
  uint16_t codecrc;
  uint16_t flags = 0x0001;               /* Initialize the clock system */

  if (sizeof(g_ab32vg1_boot) > codelen)
    {
      die("stage-1 loader too large");
    }

  memset(sec, 0, BOOT_SECTOR);
  memcpy(code, g_ab32vg1_boot, sizeof(g_ab32vg1_boot));
  codecrc = crc16(0xffff, code, codelen);

  sec[0] = 0x5a;
  sec[1] = flags;
  sec[2] = flags >> 8;
  sec[3] = -(sec[0] + sec[1] + sec[2]);
  memcpy(sec + 4, chipid, 8);
  put32(sec + 12, BOOT_LOAD);            /* Load address */
  put32(sec + 16, BOOT_LOAD);            /* Entry point */
  put32(sec + 20, BOOT_CODE_OFF);        /* Code offset */
  put32(sec + 24, codelen);              /* Code size */
  put16(sec + 0x1c, codecrc);
  put16(sec + 0x3e, crc16(0xffff, sec, 0x3e));

  lfsr_xor(sec, 64, MAGICKEY_LVMG);
  for (uint32_t off = BOOT_CODE_OFF; off < BOOT_SECTOR; off += 512)
    {
      lfsr_xor(sec + off, 512, ((off / 512) - 1) ^ MAGICKEY_LVMG ^
               (codecrc * 0x00010001u));
    }
}

/* Image table + segment data, starting at IMG_TABLE */

static uint8_t *make_image(const struct image_s *img, uint32_t *size)
{
  uint32_t off = IMG_DATA;
  uint8_t *buf;
  uint8_t *tbl;

  for (int i = 0; i < img->nseg; i++)
    {
      off = (off + img->seg[i].size + 15) & ~15u;
    }

  if (off > FLASH_SIZE)
    {
      die("image too large for the flash");
    }

  /* The stage-1 loader runs at 0x10800..0x11400 with its stack in the last
   * KB of AECRAM: the image must not be loaded there.
   */

  for (int i = 0; i < img->nseg; i++)
    {
      uint32_t s0 = img->seg[i].addr;
      uint32_t s1 = s0 + img->seg[i].size;

      if ((s0 < 0x00011400 && s1 > 0x00010800) ||
          (s0 < 0x00057a00 && s1 > 0x00057600))
        {
          die("segment at 0x%08x overlaps the stage-1 loader", s0);
        }
    }

  buf = calloc(1, off - IMG_TABLE);
  if (buf == NULL)
    {
      die("out of memory");
    }

  tbl = buf;
  memset(tbl, 0xff, IMG_DATA - IMG_TABLE);
  put32(tbl, IMG_MAGIC);
  put32(tbl + 4, img->entry);
  put32(tbl + 8, img->nseg);
  memset(tbl + 12, 0, 16 * IMG_MAXSEG);

  off = IMG_DATA;
  for (int i = 0; i < img->nseg; i++)
    {
      const struct segment_s *seg = &img->seg[i];

      put32(tbl + 12 + 16 * i, off);
      put32(tbl + 16 + 16 * i, seg->addr);
      put32(tbl + 20 + 16 * i, seg->size);
      put32(tbl + 24 + 16 * i, crc32(0, seg->data, seg->size));
      memcpy(buf + off - IMG_TABLE, seg->data, seg->size);
      off = (off + seg->size + 15) & ~15u;
    }

  put32(tbl + 12 + 16 * IMG_MAXSEG,
        crc32(0, tbl, 12 + 16 * IMG_MAXSEG));

  *size = off - IMG_TABLE;
  return buf;
}

/* Terminal *****************************************************************/

static void terminal(void)
{
  struct termios old;
  struct termios raw;
  bool tty = tcgetattr(STDIN_FILENO, &old) == 0;
  uint8_t pending[256];
  size_t npending = 0;

  fprintf(stderr, "--- terminal, exit with Ctrl-] ---\n");
  if (tty)
    {
      raw = old;
      cfmakeraw(&raw);
      tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

  for (; ; )
    {
      struct pollfd pfd[2] =
        {
          {
            .fd = STDIN_FILENO, .events = POLLIN
          },
          {
            .fd = g_fd, .events = POLLIN
          }
        };

      uint8_t buf[256];
      ssize_t n;

      if (poll(pfd, 2, -1) < 0 && errno != EINTR)
        {
          break;
        }

      if (pfd[0].revents & POLLIN)
        {
          n = read(STDIN_FILENO, buf, sizeof(buf));
          if (n <= 0 || memchr(buf, 0x1d, n) != NULL)
            {
              break;
            }

          serial_write(buf, n);
          if (npending + n <= sizeof(pending))
            {
              memcpy(pending + npending, buf, n);
              npending += n;
            }
        }

      if (pfd[1].revents & POLLIN)
        {
          uint8_t *p = buf;

          n = read(g_fd, buf, sizeof(buf));
          if (n <= 0)
            {
              break;
            }

          /* Drop the single-wire echo of what was typed */

          while (n > 0 && npending > 0 && *p == pending[0])
            {
              p++;
              n--;
              memmove(pending, pending + 1, --npending);
            }

          if (n > 0 && write(STDOUT_FILENO, p, n) < 0)
            {
              break;
            }
        }
    }

  if (tty)
    {
      tcsetattr(STDIN_FILENO, TCSANOW, &old);
    }

  fprintf(stderr, "\n");
}

/* Print the console output for g_watch seconds */

static void watch(void)
{
  double t0;

  for (t0 = now(); now() - t0 < g_watch; )
    {
      uint8_t buf[256];
      size_t n = serial_read(buf, sizeof(buf), 100);

      if (n > 0)
        {
          fwrite(buf, 1, n, stdout);
          fflush(stdout);
        }
    }
}

/* Commands *****************************************************************/

static void cmd_run(const char *path, bool term)
{
  struct image_s img;

  load_elf(path, &img);
  rom_connect(g_connect_timeout, true);
  for (int i = 0; i < img.nseg; i++)
    {
      rom_write_mem(img.seg[i].addr, img.seg[i].data, img.seg[i].size,
                    true);
    }

  printf("starting at 0x%08x\n", img.entry);
  rom_start(img.entry);
  serial_baud(CONSOLE_BAUD);
  watch();
  if (term)
    {
      terminal();
    }
}

static void cmd_flash(const char *path, bool term)
{
  struct image_s img;
  uint8_t sector[BOOT_SECTOR];
  uint8_t *image;
  uint32_t size;

  if (!g_noboot && !g_force)
    {
      die("refusing to write the boot sector without -F.\n"
          "  Once a boot sector is written, the mask ROM UART loader\n"
          "  is only reachable through the listening window of the\n"
          "  stage-1 loader.  An earlier stage-1 without it locked a\n"
          "  board out of the loader; the current one has not been\n"
          "  verified on hardware yet.  Use -n to write the image\n"
          "  only, or run from RAM.");
    }

  load_elf(path, &img);
  image = make_image(&img, &size);
  make_boot_sector(sector);

  stub_connect();
  printf("image        : %u bytes, %d segments, entry 0x%08x\n", size,
         img.nseg, img.entry);

  /* Erase the boot sector first and write it last: an interrupted update
   * leaves no boot header (the mask ROM then stays in its loader), never
   * a boot header pointing to a partial image.
   */

  if (g_noboot)
    {
      flash_program(IMG_TABLE, image, size);
      printf("verified (boot sector not changed)\n");
      free(image);
      return;
    }

  flash_erase(0, BOOT_SECTOR);
  flash_program(IMG_TABLE, image, size);
  flash_program(0, sector, BOOT_SECTOR);
  printf("verified, rebooting\n");
  stub_reset();
  free(image);

  serial_baud(CONSOLE_BAUD);
  watch();
  if (term)
    {
      terminal();
    }
}

/* Write the complete flash image to a file */

static void cmd_mkimage(const char *elf, const char *path)
{
  struct image_s img;
  uint8_t sector[BOOT_SECTOR];
  uint8_t *image;
  uint32_t size;
  FILE *f;

  load_elf(elf, &img);
  image = make_image(&img, &size);
  make_boot_sector(sector);

  f = fopen(path, "wb");
  if (f == NULL || fwrite(sector, 1, BOOT_SECTOR, f) != BOOT_SECTOR ||
      fwrite(image, 1, size, f) != size || fclose(f) != 0)
    {
      die("cannot write %s", path);
    }

  printf("%s: %u bytes (flash offset 0)\n", path, BOOT_SECTOR + size);
  free(image);
}

/* Run the stage-1 loader from RAM: it loads the image in flash exactly as
 * it does at boot, without touching the boot sector.
 */

static void cmd_boottest(bool term)
{
  rom_connect(g_connect_timeout, true);
  rom_write_mem(BOOT_LOAD, g_ab32vg1_boot, sizeof(g_ab32vg1_boot), true);
  printf("starting the stage-1 loader at 0x%08x\n", BOOT_LOAD);
  rom_start(BOOT_LOAD);
  serial_baud(CONSOLE_BAUD);
  watch();
  if (term)
    {
      terminal();
    }
}

/* Save a memory area (RAM, ROM, registers) read by the stub */

static void cmd_memdump(uint32_t addr, uint32_t len, const char *path)
{
  uint8_t *buf = malloc(len);
  FILE *f;

  if (buf == NULL)
    {
      die("out of memory");
    }

  stub_connect();
  for (uint32_t off = 0; off < len; off += STUB_MAXDATA)
    {
      uint32_t n = len - off > STUB_MAXDATA ? STUB_MAXDATA : len - off;
      uint8_t sz[4];
      size_t got;

      put32(sz, n);
      stub_call(STUB_CMD_MEMREAD, addr + off, sz, 4, buf + off, n, &got,
                2000);
    }

  f = fopen(path, "wb");
  if (f == NULL || fwrite(buf, 1, len, f) != len || fclose(f) != 0)
    {
      die("cannot write %s", path);
    }

  printf("saved %u bytes of memory at 0x%08x to %s\n", len, addr, path);
  free(buf);
}

static void cmd_read(uint32_t addr, uint32_t len, const char *path)
{
  uint8_t *buf = malloc(len);
  FILE *f;

  if (buf == NULL)
    {
      die("out of memory");
    }

  stub_connect();
  flash_read(addr, buf, len);
  if (flash_crc(addr, len) != crc32(0, buf, len))
    {
      die("read back CRC mismatch");
    }

  f = fopen(path, "wb");
  if (f == NULL || fwrite(buf, 1, len, f) != len || fclose(f) != 0)
    {
      die("cannot write %s", path);
    }

  printf("saved %u bytes to %s\n", len, path);
  free(buf);
}

static void cmd_write(uint32_t addr, const char *path)
{
  FILE *f = fopen(path, "rb");
  uint8_t *buf;
  long len;

  if (f == NULL || fseek(f, 0, SEEK_END) != 0 || (len = ftell(f)) <= 0)
    {
      die("cannot read %s", path);
    }

  buf = malloc(len);
  rewind(f);
  if (buf == NULL || fread(buf, 1, len, f) != (size_t)len)
    {
      die("cannot read %s", path);
    }

  fclose(f);
  if (addr + len > FLASH_SIZE)
    {
      die("file does not fit in the flash");
    }

  stub_connect();
  flash_program(addr, buf, len);
  printf("wrote and verified %ld bytes at 0x%06x\n", len, addr);
  free(buf);
}

static void cmd_erase(uint32_t addr, uint32_t len)
{
  stub_connect();
  flash_erase(addr, len);
  printf("erased 0x%06x..0x%06x\n", addr & ~0xfffu,
         ((addr + len + 0xfff) & ~0xfffu) - 1);
}

static void usage(void)
{
  fprintf(stderr,
    "Usage: ab32vg1_dl [-p port] [-t] [-v] <command> [args]\n"
    "\n"
    "  info                 show chip and flash information\n"
    "  run <nuttx>          load the ELF file into RAM and start it\n"
    "  flash <nuttx>        write the ELF file to flash and boot it\n"
    "  read <addr> <size> <file>\n"
    "                       save a flash area (e.g. 0 0x100000 backup.bin)\n"
    "  write <addr> <file>  write a raw file to flash (restore a backup)\n"
    "  memdump <addr> <size> <file>\n"
    "                       save a memory area (RAM/ROM) for debugging\n"
    "  erase <addr> <size>  erase a flash area\n"
    "  reset                reset the board (boot from flash)\n"
    "  mkimage <nuttx> <file>\n"
    "                       write the flash image to a file\n"
    "  boottest             run the stage-1 loader from RAM (loads the\n"
    "                       image in flash without using the boot sector)\n"
    "  term                 open a terminal on the console\n"
    "\n"
    "  -p port              serial port (default " DEFAULT_PORT ")\n"
    "  -F                   flash: allow writing the boot sector (see the\n"
    "                       board documentation first)\n"
    "  -n                   flash: keep the boot sector, write the image\n"
    "                       only\n"
    "  -t                   open a terminal after run/flash\n"
    "  -v                   verbose\n"
    "  -T seconds           how long to wait for the ROM loader (120)\n"
    "  -w seconds           print the console output for some seconds\n"
    "                       after run/flash/boottest/reset\n"
    "\n"
    "The board enters the ROM loader when reset while this tool waits; a\n"
    "running NuttX (CONFIG_AB32VG1_UART0_ROMKEY) is reset automatically.\n"
    "info, read, write and erase leave the flash stub running so that the\n"
    "next command does not need a reset.\n");
  exit(EXIT_FAILURE);
}

static uint32_t parse_num(const char *s)
{
  char *end;
  unsigned long v = strtoul(s, &end, 0);

  if (*s == '\0' || *end != '\0')
    {
      die("bad number '%s'", s);
    }

  return v;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  const char *port = DEFAULT_PORT;
  const char *cmd;
  bool term = false;
  int opt;

  while ((opt = getopt(argc, argv, "Fp:nT:tvw:h")) != -1)
    {
      switch (opt)
        {
          case 'p':
            port = optarg;
            break;

          case 'F':
            g_force = true;
            break;

          case 'n':
            g_noboot = true;
            break;

          case 't':
            term = true;
            break;

          case 'v':
            g_verbose = true;
            break;

          case 'T':
            g_connect_timeout = parse_num(optarg);
            break;

          case 'w':
            g_watch = parse_num(optarg);
            break;

          default:
            usage();
        }
    }

  if (optind >= argc)
    {
      usage();
    }

  cmd = argv[optind++];
  argc -= optind;
  argv += optind;

  if (strcmp(cmd, "mkimage") == 0 && argc == 2)
    {
      cmd_mkimage(argv[0], argv[1]);
      return EXIT_SUCCESS;
    }

  serial_open(port, ROM_BAUD);

  if (strcmp(cmd, "info") == 0 && argc == 0)
    {
      stub_connect();
    }
  else if (strcmp(cmd, "reset") == 0 && argc == 0)
    {
      stub_connect();
      stub_reset();
      serial_baud(CONSOLE_BAUD);
      watch();
    }
  else if (strcmp(cmd, "run") == 0 && argc == 1)
    {
      cmd_run(argv[0], term);
    }
  else if (strcmp(cmd, "flash") == 0 && argc == 1)
    {
      cmd_flash(argv[0], term);
    }
  else if (strcmp(cmd, "read") == 0 && argc == 3)
    {
      cmd_read(parse_num(argv[0]), parse_num(argv[1]), argv[2]);
    }
  else if (strcmp(cmd, "memdump") == 0 && argc == 3)
    {
      cmd_memdump(parse_num(argv[0]), parse_num(argv[1]), argv[2]);
    }
  else if (strcmp(cmd, "write") == 0 && argc == 2)
    {
      cmd_write(parse_num(argv[0]), argv[1]);
    }
  else if (strcmp(cmd, "erase") == 0 && argc == 2)
    {
      cmd_erase(parse_num(argv[0]), parse_num(argv[1]));
    }
  else if (strcmp(cmd, "boottest") == 0 && argc == 0)
    {
      cmd_boottest(term);
    }
  else if (strcmp(cmd, "term") == 0 && argc == 0)
    {
      serial_baud(CONSOLE_BAUD);
      terminal();
    }
  else
    {
      usage();
    }

  close(g_fd);
  return EXIT_SUCCESS;
}
