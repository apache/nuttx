/****************************************************************************
 * tools/imxrt1180/mkahab.c
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

/* mkahab - assemble the MIMXRT1180-EVK bootable FlexSPI NOR image
 *          (flash.bin) from the freshly built Cortex-M33 NuttX image, by
 *          writing a single, unsigned NXP AHAB container directly.
 *
 * This is a native, dependency-free replacement for the "nxpimage ahab
 * export" step of tools/imxrt1180/build_flash_image.sh, covering exactly
 * the case that board configuration actually needs today: a single
 * unencrypted, unsigned application container (srk_set: none) holding
 * the M33 image, with no bundled NXP EdgeLock Enclave firmware
 * container.
 *
 * If a build needs to bundle the proprietary ELE firmware AHAB container
 * as well (CONFIG_IMXRT_ELE_FW=y with CONFIG_IMXRT_ELE_LOAD_FW unset),
 * tools/imxrt1180/build_flash_image.sh (which drives NXP's SPSDK
 * "nxpimage") is used instead; see boards/arm/imxrt/imxrt1180-evk/
 * scripts/Make.defs.
 *
 * The on-disk container layout (tag, field offsets, sizes) mirrors the
 * NXP AHAB "container header" / "image array entry" structures used
 * across the i.MX8/i.MX9/RT117x/RT118x families, as documented in the
 * RT1180 Reference Manual (ch. 12) and cross-checked byte-for-byte
 * against the output of "nxpimage ahab export" for this board's
 * configuration.  This file is an original implementation of that
 * (hardware-mandated, non-copyrightable) layout - it is not derived
 * from any NXP or SPSDK source code.
 *
 * Usage:
 *
 *   mkahab --m33 nuttx.bin --out flash.bin
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/stat.h>

#include "sha256.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash layout constants (must match flash-m33.ld). */

#define M33_LOAD_ADDR      0x2800b000u
#define FLASH_ORIGIN       0x28000000u
#define M33_MAX_SIZE       (512u * 1024u)

#define FCB_SIZE           512u
#define FCB_FLASH_OFFSET   0x400u
#define AHAB_FLASH_OFFSET  0x1000u

/* Where the code starts inside the raw M33 input binary: the linker
 * places the FCB at VMA 0x28000400 and the XIP code at VMA 0x2800b000,
 * and arm-none-eabi-objcopy lays these out as a flat binary starting
 * from the lowest section VMA, so the FCB sits at file offset 0 and the
 * code sits at file offset (0xb000 - 0x400).
 */

#define CODE_FILE_OFFSET   (0xb000u - 0x400u)

/* AHAB container header ("flash_header_v3") field layout. */

#define AHAB_TAG           0x87u
#define AHAB_HDR_SIZE      16u

/* Signature block header tag: present (with a valid length/tag) even
 * for an unsigned container - only the SRK table/cert/blob/signature
 * offsets and payload are empty.
 */

#define AHAB_SIGBLK_TAG    0x90u

/* Image array entry ("boot_img") field layout. */

#define AHAB_IMG_HASH_LEN  64u
#define AHAB_IMG_IV_LEN    32u
#define AHAB_IMG_SIZE      (4u + 4u + 8u + 8u + 4u + 4u + \
                             AHAB_IMG_HASH_LEN + AHAB_IMG_IV_LEN)

/* Signature block header, present (but empty/zero) even for an unsigned
 * container.
 */

#define AHAB_SIGBLK_HDR_SIZE 16u

#define AHAB_CONTAINER_SIZE (AHAB_HDR_SIZE + AHAB_IMG_SIZE + \
                              AHAB_SIGBLK_HDR_SIZE)

/* Image array entry flags: image type EXECUTABLE, core id CORTEX-M33
 * (RT118x AHAB core id 1 - distinct from the i.MX8/9 core id space),
 * hash type SHA-256, not encrypted.
 */

#define AHAB_IMG_TYPE_EXEC   0x03u
#define AHAB_CORE_ID_M33     0x01u
#define AHAB_HASH_TYPE_SHA256 0x00u

#define AHAB_IMG_FLAGS \
  (AHAB_IMG_TYPE_EXEC | (AHAB_CORE_ID_M33 << 4) | \
   (AHAB_HASH_TYPE_SHA256 << 8))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void err(const char *msg)
{
  fprintf(stderr, "error: %s\n", msg);
  exit(1);
}

static void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v);
  p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v);
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static void put_u64(uint8_t *p, uint64_t v)
{
  put_u32(p, (uint32_t)v);
  put_u32(p + 4, (uint32_t)(v >> 32));
}

static uint8_t *read_file(const char *path, size_t *size)
{
  FILE *f;
  uint8_t *buf;
  long len;

  f = fopen(path, "rb");
  if (f == NULL)
    {
      fprintf(stderr, "error: cannot open '%s'\n", path);
      exit(1);
    }

  if (fseek(f, 0, SEEK_END) != 0)
    {
      err("fseek failed");
    }

  len = ftell(f);
  if (len < 0)
    {
      err("ftell failed");
    }

  rewind(f);

  buf = malloc((size_t)len);
  if (buf == NULL && len > 0)
    {
      err("out of memory");
    }

  if (len > 0 && fread(buf, 1, (size_t)len, f) != (size_t)len)
    {
      err("short read");
    }

  fclose(f);

  *size = (size_t)len;
  return buf;
}

/* Build the 160-byte AHAB container (header + one image array entry +
 * empty signature block header) covering the M33 XIP code image.
 */

static void build_ahab_container(uint8_t *out, const uint8_t *code,
                                  size_t code_size, uint32_t image_offset)
{
  uint8_t digest[SHA256_DIGEST_SIZE];
  uint8_t *hdr = out;
  uint8_t *img = out + AHAB_HDR_SIZE;
  uint8_t *sig = img + AHAB_IMG_SIZE;

  memset(out, 0, AHAB_CONTAINER_SIZE);

  /* Container header: version(u8) length(u16) tag(u8) flags(u32)
   * sw_version(u16) fuse_version(u8) num_images(u8)
   * sig_blk_offset(u16) reserved(u16)
   */

  hdr[0] = 0;
  put_u16(hdr + 1, (uint16_t)AHAB_CONTAINER_SIZE);
  hdr[3] = AHAB_TAG;
  put_u32(hdr + 4, 0);
  put_u16(hdr + 8, 0);
  hdr[10] = 0;
  hdr[11] = 1;
  put_u16(hdr + 12, (uint16_t)(AHAB_HDR_SIZE + AHAB_IMG_SIZE));
  put_u16(hdr + 14, 0);

  /* Image array entry: offset(u32) size(u32) dst(u64) entry(u64)
   * flags(u32) meta(u32) hash[64] iv[32]
   */

  put_u32(img, image_offset);
  put_u32(img + 4, (uint32_t)code_size);
  put_u64(img + 8, M33_LOAD_ADDR);
  put_u64(img + 16, M33_LOAD_ADDR);
  put_u32(img + 24, AHAB_IMG_FLAGS);
  put_u32(img + 28, 0);

  sha256_buffer(code, code_size, digest);
  memcpy(img + 32, digest, SHA256_DIGEST_SIZE);

  /* The remaining hash bytes and the iv[] field stay zero */

  /* Signature block header: version(u8) length(u16) tag(u8)
   * srk_table_offset(u16) cert_offset(u16) blob_offset(u16)
   * signature_offset(u16) reserved(u32).  Only the length and tag are
   * non-zero: there is no SRK table, cert, blob or signature (matches
   * srk_set: none - unsigned container).
   */

  sig[0] = 0;
  put_u16(sig + 1, (uint16_t)AHAB_SIGBLK_HDR_SIZE);
  sig[3] = AHAB_SIGBLK_TAG;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[])
{
  const char *m33_path = NULL;
  const char *out_path = "flash.bin";
  uint8_t *src;
  size_t src_size;
  uint32_t code_flash_offset;
  uint32_t image_offset;
  uint8_t *code;
  size_t code_size;
  uint8_t ahab[AHAB_CONTAINER_SIZE];
  uint8_t *out;
  size_t total;
  FILE *f;
  int i;

  for (i = 1; i < argc; i++)
    {
      if (strcmp(argv[i], "--m33") == 0 && i + 1 < argc)
        {
          m33_path = argv[++i];
        }
      else if (strcmp(argv[i], "--out") == 0 && i + 1 < argc)
        {
          out_path = argv[++i];
        }
      else
        {
          fprintf(stderr, "usage: %s --m33 <nuttx.bin> --out <flash.bin>\n",
                  argv[0]);
          return 1;
        }
    }

  if (m33_path == NULL)
    {
      err("--m33 <nuttx.bin> is required");
    }

  src = read_file(m33_path, &src_size);

  if (src_size <= CODE_FILE_OFFSET)
    {
      err("input binary shorter than expected");
    }

  code = src + CODE_FILE_OFFSET;
  code_size = src_size - CODE_FILE_OFFSET;

  code_flash_offset = M33_LOAD_ADDR - FLASH_ORIGIN;
  image_offset = code_flash_offset - AHAB_FLASH_OFFSET;

  build_ahab_container(ahab, code, code_size, image_offset);

  total = AHAB_FLASH_OFFSET + image_offset + code_size;
  if (total > M33_MAX_SIZE)
    {
      fprintf(stderr,
              "error: M33 image (%zu B) exceeds the %u B reserve; it "
              "would overlap the Cortex-M7 image at flash offset "
              "0x80000.\n", total, M33_MAX_SIZE);
      return 1;
    }

  out = calloc(1, total);
  if (out == NULL)
    {
      err("out of memory");
    }

  memcpy(out + FCB_FLASH_OFFSET, src, FCB_SIZE);
  memcpy(out + AHAB_FLASH_OFFSET, ahab, AHAB_CONTAINER_SIZE);
  memcpy(out + AHAB_FLASH_OFFSET + image_offset, code, code_size);

  f = fopen(out_path, "wb");
  if (f == NULL)
    {
      fprintf(stderr, "error: cannot create '%s'\n", out_path);
      return 1;
    }

  if (fwrite(out, 1, total, f) != total)
    {
      err("short write");
    }

  fclose(f);

  printf("Wrote %s (FCB@0x%x, M33 AHAB@0x%x, %zu B)\n",
         out_path, FCB_FLASH_OFFSET, AHAB_FLASH_OFFSET, total);

  free(src);
  free(out);
  return 0;
}
