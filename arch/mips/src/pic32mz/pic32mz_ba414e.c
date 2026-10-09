/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_ba414e.c
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

/* PIC32MZ-W1 asymmetric crypto engine (BA414E): modular arithmetic and
 * elliptic curve primitives.  DS70005425 (chapter 26) documents only the
 * memories; the programming model below comes from Microchip's MPLAB
 * Harmony driver (crypto/drivers/ba414e) and is marked [EX].  Register
 * addresses and fields are from the PIC32MZ-W_DFP [DFP].
 *
 * The engine runs a microcode program that must be loaded before the
 * first operation.  It is under Microchip's license and not part of
 * NuttX: either the build downloads Microchip's driver source and links
 * the microcode into the image (CONFIG_PIC32MZ_W1_BA414E_UCODE_BUILTIN),
 * or it is read from a file on first use
 * (CONFIG_PIC32MZ_W1_BA414E_UCODE_FILE).
 *
 * Microcode file format: the 810 32-bit values of the init_ucode_array[810]
 * initializer in drivers/ba414e/src/drv_ba414e.c of
 * https://github.com/Microchip-MPLAB-Harmony/crypto, in array order, each
 * stored little endian; no header, exactly 3240 bytes.  For example, the
 * first values 0x10032004, 0x48013e00 give the bytes
 * 04 20 03 10 00 3e 01 48.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <inttypes.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <arch/irq.h>

#include "mips_internal.h"
#include <arch/board/board.h>

#include "hardware/pic32mzw1_features.h"
#include "pic32mz_ba414e.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* [DFP] Registers */

#define BA414E_PKCONFIG       0xbf920000
#define BA414E_PKCOMMAND      0xbf920004
#define BA414E_PKCONTROL      0xbf920008
#define BA414E_PKSTATUS       0xbf92000c

/* [DFP] PKCONFIG: operand slot pointers */

#define PKCONFIG_OPPTRA(n)    ((uint32_t)(n) << 0)
#define PKCONFIG_OPPTRB(n)    ((uint32_t)(n) << 8)
#define PKCONFIG_OPPTRC(n)    ((uint32_t)(n) << 16)

/* [DFP] PKCOMMAND fields */

#define PKCOMMAND_OPERATION(n) ((uint32_t)(n) << 0)
#define PKCOMMAND_OPSIZE(n)   ((uint32_t)(n) << 8)
#define PKCOMMAND_CALCR2      (1u << 31)

/* [DFP] PKCONTROL.START */

#define PKCONTROL_START       (1u << 0)

/* [DFP] PKSTATUS fields */

#define PKSTATUS_PXNOC        (1u << 4)   /* Point not on curve */
#define PKSTATUS_PXINF        (1u << 5)   /* Point at infinity */

/* [DFP] Shared crypto memory (operand slots) and microcode memory (KSEG1) */

#define BA414E_SCM_BASE       0xbf900000
#define BA414E_UCM_BASE       0xbf902000

/* [EX] Shared memory size, 64-byte operand slots */

#define BA414E_SCM_SIZE       2432
#define BA414E_SLOT_SIZE      64
#define BA414E_SLOT(n)        (BA414E_SCM_BASE + (n) * BA414E_SLOT_SIZE)

/* [EX] Microcode: 90 groups of nine 32-bit words, each packing sixteen
 * 18-bit microcode words, most significant bits first.
 */

#define BA414E_UCODE_GROUPS   90
#define BA414E_UCODE_WORDS    (BA414E_UCODE_GROUPS * 9)

/* [DFP] PMD1.BA414MD: module disable */

#define PMD1_BA414MD          (1u << 23)

/* [EX] Operation codes */

#define OPC_MOD_ADD           0x01
#define OPC_MOD_SUB           0x02
#define OPC_MOD_MULT          0x03
#define OPC_MOD_EXP           0x10
#define OPC_ECC_DOUBLE        0x20
#define OPC_ECC_ADD           0x21
#define OPC_ECC_MULT          0x22
#define OPC_ECC_ONCURVE       0x26

/* [EX] Slots of the modular primitives (P, A, B -> C) */

#define MOD_SLOT_P            0
#define MOD_SLOT_A            1
#define MOD_SLOT_B            2
#define MOD_SLOT_C            3

/* [EX] Slots of the modular exponentiation (C = M^e mod n) */

#define EXP_SLOT_N            0
#define EXP_SLOT_E            1
#define EXP_SLOT_M            2
#define EXP_SLOT_C            3

/* [EX] Slots of the ECC primitives */

#define ECC_SLOT_P            0
#define ECC_SLOT_N            1
#define ECC_SLOT_GX           2
#define ECC_SLOT_GY           3
#define ECC_SLOT_A            4
#define ECC_SLOT_B            5
#define ECC_SLOT_P1X          6
#define ECC_SLOT_P1Y          7
#define ECC_SLOT_P2X          8
#define ECC_SLOT_P2Y          9
#define ECC_SLOT_P3X          10
#define ECC_SLOT_P3Y          11
#define ECC_SLOT_K            12

#define BA414E_TIMEOUT        SEC2TICK(1)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_p256_p[32] =
{
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff
};

static const uint8_t g_p256_n[32] =
{
  0x51, 0x25, 0x63, 0xfc, 0xc2, 0xca, 0xb9, 0xf3,
  0x84, 0x9e, 0x17, 0xa7, 0xad, 0xfa, 0xe6, 0xbc,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff
};

static const uint8_t g_p256_gx[32] =
{
  0x96, 0xc2, 0x98, 0xd8, 0x45, 0x39, 0xa1, 0xf4,
  0xa0, 0x33, 0xeb, 0x2d, 0x81, 0x7d, 0x03, 0x77,
  0xf2, 0x40, 0xa4, 0x63, 0xe5, 0xe6, 0xbc, 0xf8,
  0x47, 0x42, 0x2c, 0xe1, 0xf2, 0xd1, 0x17, 0x6b
};

static const uint8_t g_p256_gy[32] =
{
  0xf5, 0x51, 0xbf, 0x37, 0x68, 0x40, 0xb6, 0xcb,
  0xce, 0x5e, 0x31, 0x6b, 0x57, 0x33, 0xce, 0x2b,
  0x16, 0x9e, 0x0f, 0x7c, 0x4a, 0xeb, 0xe7, 0x8e,
  0x9b, 0x7f, 0x1a, 0xfe, 0xe2, 0x42, 0xe3, 0x4f
};

static const uint8_t g_p256_a[32] =
{
  0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff
};

static const uint8_t g_p256_b[32] =
{
  0x4b, 0x60, 0xd2, 0x27, 0x3e, 0x3c, 0xce, 0x3b,
  0xf6, 0xb0, 0x53, 0xcc, 0xb0, 0x06, 0x1d, 0x65,
  0xbc, 0x86, 0x98, 0x76, 0x55, 0xbd, 0xeb, 0xb3,
  0xe7, 0x93, 0x3a, 0xaa, 0xd8, 0x35, 0xc6, 0x5a
};

#ifdef CONFIG_PIC32MZ_W1_BA414E_SELFTEST
/* Self-test vectors (P-256, little endian): 2G, kG for
 * k = 0xdeadbeef12345678, x of 4G, and 3^(p-2) mod p (the inverse of 3).
 */

static const uint8_t g_st_2gx[32] =
{
  0x78, 0x99, 0x66, 0x47, 0xfc, 0x48, 0x0b, 0xa6,
  0x35, 0x1b, 0xf2, 0x77, 0xe2, 0x69, 0x89, 0xc0,
  0xc3, 0x1a, 0xb5, 0x04, 0x03, 0x38, 0x52, 0x8a,
  0x7e, 0x4f, 0x03, 0x8d, 0x18, 0x7b, 0xf2, 0x7c
};

static const uint8_t g_st_2gy[32] =
{
  0xd1, 0x73, 0x78, 0x22, 0x9d, 0xb7, 0x04, 0x9e,
  0x29, 0x82, 0xe9, 0x3c, 0xe6, 0xad, 0x7d, 0xba,
  0xdb, 0x30, 0x74, 0x9f, 0xc6, 0x9a, 0x3d, 0x29,
  0x40, 0xd0, 0x8e, 0xdb, 0x10, 0x55, 0x77, 0x07
};

static const uint8_t g_st_kgx[32] =
{
  0x60, 0x70, 0xe3, 0x37, 0x8c, 0x9a, 0xbd, 0xfd,
  0x9f, 0xfc, 0x20, 0xe4, 0xae, 0x29, 0x00, 0x8d,
  0x2f, 0xf1, 0x51, 0xb2, 0xdb, 0x1e, 0x9f, 0x17,
  0x13, 0x68, 0xf1, 0x7c, 0x4b, 0xc2, 0x9d, 0xfc
};

static const uint8_t g_st_kgy[32] =
{
  0xbf, 0x2a, 0x5c, 0xe1, 0x3e, 0x15, 0xbb, 0x9f,
  0x23, 0x35, 0xae, 0xed, 0xb7, 0xf7, 0xcc, 0x55,
  0x98, 0xde, 0x03, 0x33, 0x39, 0x7b, 0xd8, 0xdc,
  0x4c, 0xbc, 0xc2, 0x52, 0x4c, 0x4d, 0xbc, 0x62
};

static const uint8_t g_st_4gx[32] =
{
  0x52, 0x08, 0x03, 0x6b, 0x44, 0x02, 0x93, 0x50,
  0xef, 0x96, 0x55, 0x78, 0xdb, 0xe2, 0x1f, 0x03,
  0xd0, 0x2b, 0xe6, 0x9e, 0x65, 0xde, 0x2d, 0xa0,
  0xbb, 0x8f, 0xd0, 0x32, 0x35, 0x4a, 0x53, 0xe2
};

static const uint8_t g_st_inv3[32] =
{
  0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
  0x55, 0x55, 0x55, 0x55, 0xab, 0xaa, 0xaa, 0xaa,
  0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
  0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa, 0xaa, 0xaa
};

#endif

static rmutex_t g_ba414e_lock = NXRMUTEX_INITIALIZER;
static bool g_ba414e_loaded;
static sem_t g_ba414e_done = SEM_INITIALIZER(0);
static volatile uint32_t g_ba414e_status;
static volatile bool g_ba414e_fault;

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct ba414e_curve_s g_ba414e_p256 =
{
  32, g_p256_p, g_p256_n, g_p256_gx, g_p256_gy, g_p256_a, g_p256_b
};

#ifdef CONFIG_PIC32MZ_W1_BA414E_UCODE_BUILTIN
/* Microcode, generated at build time (see Make.defs) */

extern const uint32_t g_ba414e_ucode[BA414E_UCODE_WORDS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* [EX] Operand size code: number of 64-bit words, 2 (128 bits) to 8 */

static int ba414e_opsize(int len)
{
  int size = (len + 7) / 8;

  return size < 2 ? 2 : size;
}

static void ba414e_write_slot(int slot, FAR const uint8_t *data, int len,
                              int slotlen)
{
  volatile uint32_t *dst = (volatile uint32_t *)BA414E_SLOT(slot);
  uint32_t word;
  int i;

  /* Little endian, zero padded to the operand size; 32-bit accesses */

  for (i = 0; i < slotlen; i += 4)
    {
      word = 0;
      if (i < len)
        {
          word = data[i];
        }

      if (i + 1 < len)
        {
          word |= (uint32_t)data[i + 1] << 8;
        }

      if (i + 2 < len)
        {
          word |= (uint32_t)data[i + 2] << 16;
        }

      if (i + 3 < len)
        {
          word |= (uint32_t)data[i + 3] << 24;
        }

      *dst++ = word;
    }
}

static void ba414e_read_slot(int slot, FAR uint8_t *data, int len)
{
  volatile uint32_t *src = (volatile uint32_t *)BA414E_SLOT(slot);
  uint32_t word = 0;
  int i;

  for (i = 0; i < len; i++)
    {
      if ((i & 3) == 0)
        {
          word = *src++;
        }

      data[i] = word & 0xff;
      word >>= 8;
    }
}

static void ba414e_clear_scm(void)
{
  volatile uint32_t *p = (volatile uint32_t *)BA414E_SCM_BASE;
  int i;

  for (i = 0; i < BA414E_SCM_SIZE / 4; i++)
    {
      p[i] = 0;
    }
}

static void ba414e_load_ucode(FAR const uint32_t *in)
{
  volatile uint32_t *ucm = (volatile uint32_t *)BA414E_UCM_BASE;
  uint64_t acc;
  int bits;
  int g;
  int i;

  for (g = 0; g < BA414E_UCODE_GROUPS; g++)
    {
      acc  = 0;
      bits = 0;

      for (i = 0; i < 9; i++)
        {
          /* Shift in 32 bits, emit every complete 18-bit word */

          acc   = (acc << 32) | *in++;
          bits += 32;

          while (bits >= 18)
            {
              bits  -= 18;
              *ucm++ = (uint32_t)(acc >> bits) & 0x3ffff;
            }
        }
    }
}

#ifdef CONFIG_PIC32MZ_W1_BA414E_UCODE_FILE
/* Read the microcode file (format in the header of this file) */

static FAR uint32_t *ba414e_read_ucode(void)
{
  FAR const char *path = CONFIG_PIC32MZ_W1_BA414E_UCODE_PATH;
  size_t size = BA414E_UCODE_WORDS * sizeof(uint32_t);
  FAR uint32_t *ucode;
  struct file file;
  ssize_t nread;
  size_t total = 0;
  char extra;
  int ret;
  static bool warned;

  ret = file_open(&file, path, O_RDONLY | O_CLOEXEC);
  if (ret < 0)
    {
      /* Callers retry; report the missing file once */

      if (!warned)
        {
          syslog(LOG_ERR, "ba414e: cannot open %s: %d\n", path, ret);
          warned = true;
        }

      return NULL;
    }

  ucode = kmm_malloc(size);
  if (ucode != NULL)
    {
      while (total < size)
        {
          nread = file_read(&file, (FAR uint8_t *)ucode + total,
                            size - total);
          if (nread <= 0)
            {
              break;
            }

          total += nread;
        }

      /* The size must match exactly */

      if (total != size || file_read(&file, &extra, 1) != 0)
        {
          syslog(LOG_ERR, "ba414e: %s is not %zu bytes\n", path, size);
          kmm_free(ucode);
          ucode = NULL;
        }
    }

  file_close(&file);
  return ucode;
}
#endif

static int ba414e_isr(int irq, FAR void *context, FAR void *arg)
{
  /* [EX] Latch the status, stop the engine, mask the interrupt */

  g_ba414e_status = getreg32(BA414E_PKSTATUS);
  g_ba414e_fault  = (irq == PIC32MZ_IRQ_CRYPTO1F);
  putreg32(0, BA414E_PKCONTROL);

  up_disable_irq(PIC32MZ_IRQ_CRYPTO1);
  up_disable_irq(PIC32MZ_IRQ_CRYPTO1F);
  mips_clrpend_irq(irq);

  nxsem_post(&g_ba414e_done);
  return OK;
}

/* Program the operation; the operands are written afterwards, since the
 * slot size follows from the operand size.
 */

static void ba414e_setup(int opcode, int opsize, uint32_t config)
{
  putreg32(0, BA414E_PKCONTROL);
  up_disable_irq(PIC32MZ_IRQ_CRYPTO1);
  up_disable_irq(PIC32MZ_IRQ_CRYPTO1F);
  mips_clrpend_irq(PIC32MZ_IRQ_CRYPTO1);
  mips_clrpend_irq(PIC32MZ_IRQ_CRYPTO1F);

  ba414e_clear_scm();

  putreg32(config, BA414E_PKCONFIG);
  putreg32(PKCOMMAND_OPERATION(opcode) | PKCOMMAND_OPSIZE(opsize) |
           PKCOMMAND_CALCR2, BA414E_PKCOMMAND);
}

static int ba414e_run(void)
{
  int ret;

  /* Drain a stale completion */

  while (nxsem_trywait(&g_ba414e_done) == OK);

  g_ba414e_fault = false;
  up_enable_irq(PIC32MZ_IRQ_CRYPTO1F);
  up_enable_irq(PIC32MZ_IRQ_CRYPTO1);
  putreg32(PKCONTROL_START, BA414E_PKCONTROL);

  ret = nxsem_tickwait_uninterruptible(&g_ba414e_done, BA414E_TIMEOUT);
  if (ret < 0)
    {
      putreg32(0, BA414E_PKCONTROL);
      up_disable_irq(PIC32MZ_IRQ_CRYPTO1);
      up_disable_irq(PIC32MZ_IRQ_CRYPTO1F);
      syslog(LOG_ERR, "ba414e: timeout, status %08" PRIx32 "\n",
             getreg32(BA414E_PKSTATUS));
      return -ETIMEDOUT;
    }

  return OK;
}

#ifdef CONFIG_PIC32MZ_W1_BA414E_SELFTEST
static void ba414e_selftest(void);
#endif

/* Take the engine; load the microcode on first use */

static int ba414e_lock(void)
{
  FAR const uint32_t *ucode;

  nxrmutex_lock(&g_ba414e_lock);
  if (g_ba414e_loaded)
    {
      return OK;
    }

#ifdef CONFIG_PIC32MZ_W1_BA414E_UCODE_FILE
  ucode = ba414e_read_ucode();
  if (ucode == NULL)
    {
      nxrmutex_unlock(&g_ba414e_lock);
      return -ENOENT;
    }
#else
  ucode = g_ba414e_ucode;
#endif

  putreg32(0, BA414E_PKCONTROL);
  ba414e_load_ucode(ucode);
  ba414e_clear_scm();
  g_ba414e_loaded = true;

#ifdef CONFIG_PIC32MZ_W1_BA414E_UCODE_FILE
  kmm_free((FAR void *)ucode);
#endif

  syslog(LOG_INFO, "ba414e: microcode loaded\n");

#ifdef CONFIG_PIC32MZ_W1_BA414E_SELFTEST
  ba414e_selftest();
#endif

  return OK;
}

static int ba414e_modop(int opcode, FAR uint8_t *c, FAR const uint8_t *a,
                        FAR const uint8_t *b, FAR const uint8_t *p, int len)
{
  int opsize = ba414e_opsize(len);
  int ret;

  if (len <= 0 || len > BA414E_MAX_BYTES)
    {
      return -EINVAL;
    }

  ret = ba414e_lock();
  if (ret < 0)
    {
      return ret;
    }

  ba414e_setup(opcode, opsize, PKCONFIG_OPPTRA(MOD_SLOT_A) |
               PKCONFIG_OPPTRB(MOD_SLOT_B) | PKCONFIG_OPPTRC(MOD_SLOT_C));
  ba414e_write_slot(MOD_SLOT_P, p, len, opsize * 8);
  ba414e_write_slot(MOD_SLOT_A, a, len, opsize * 8);
  ba414e_write_slot(MOD_SLOT_B, b, len, opsize * 8);

  ret = ba414e_run();
  if (ret == OK)
    {
      if (g_ba414e_fault)
        {
          ret = -EIO;
        }
      else
        {
          ba414e_read_slot(MOD_SLOT_C, c, len);
        }
    }

  nxrmutex_unlock(&g_ba414e_lock);
  return ret;
}

/* Common part of the ECC primitives: load the curve and P1 */

static void ba414e_ecc_setup(FAR const struct ba414e_curve_s *curve,
                             int opcode, uint32_t config,
                             FAR const uint8_t *px, FAR const uint8_t *py)
{
  int opsize = ba414e_opsize(curve->len);
  int slotlen = opsize * 8;

  ba414e_setup(opcode, opsize, config);
  ba414e_write_slot(ECC_SLOT_P,   curve->p,  curve->len, slotlen);
  ba414e_write_slot(ECC_SLOT_N,   curve->n,  curve->len, slotlen);
  ba414e_write_slot(ECC_SLOT_GX,  curve->gx, curve->len, slotlen);
  ba414e_write_slot(ECC_SLOT_GY,  curve->gy, curve->len, slotlen);
  ba414e_write_slot(ECC_SLOT_A,   curve->a,  curve->len, slotlen);
  ba414e_write_slot(ECC_SLOT_B,   curve->b,  curve->len, slotlen);
  ba414e_write_slot(ECC_SLOT_P1X, px,        curve->len, slotlen);
  ba414e_write_slot(ECC_SLOT_P1Y, py,        curve->len, slotlen);
}

/* Run an ECC operation with result P3 */

static int ba414e_ecc_finish(FAR const struct ba414e_curve_s *curve,
                             FAR uint8_t *rx, FAR uint8_t *ry)
{
  int ret = ba414e_run();

  if (ret < 0)
    {
      return ret;
    }

  /* [EX] PXINF flags the point at infinity, also on the fault path */

  if (g_ba414e_status & PKSTATUS_PXINF)
    {
      return BA414E_POINT_AT_INF;
    }

  if (g_ba414e_fault)
    {
      return -EIO;
    }

  ba414e_read_slot(ECC_SLOT_P3X, rx, curve->len);
  ba414e_read_slot(ECC_SLOT_P3Y, ry, curve->len);
  return BA414E_OK;
}

#ifdef CONFIG_PIC32MZ_W1_BA414E_SELFTEST
static inline uint32_t ba414e_count(void)
{
  uint32_t count;

  __asm__ __volatile__("mfc0 %0, $9" : "=r"(count));
  return count;
}

static void ba414e_report(FAR const char *name, bool ok, uint32_t start)
{
  /* CP0 Count runs at SYSCLK / 2 */

  syslog(LOG_INFO, "ba414e: %-8s %s %6" PRIu32 " us\n", name,
         ok ? "ok  " : "FAIL",
         (ba414e_count() - start) / (BOARD_CPU_CLOCK / 2000000));
}

static void ba414e_selftest(void)
{
  FAR const struct ba414e_curve_s *c = &g_ba414e_p256;
  static const uint8_t k[32] =
  {
    0x78, 0x56, 0x34, 0x12, 0xef, 0xbe, 0xad, 0xde
  };

  uint8_t one[32];
  uint8_t x[32];
  uint8_t y[32];
  uint8_t e[32];
  uint32_t t;
  int ret;

  t = ba414e_count();
  ret = pic32mz_ba414e_ecc_check(c, c->gx, c->gy);
  ba414e_report("oncurve", ret == BA414E_OK, t);

  t = ba414e_count();
  ret = pic32mz_ba414e_ecc_check(c, c->gx, c->gx);
  ba414e_report("offcurve", ret == BA414E_NOT_ON_CURVE, t);

  t = ba414e_count();
  ret = pic32mz_ba414e_ecc_add(c, x, y, c->gx, c->gy, c->gx, c->gy);
  ba414e_report("double", ret == BA414E_OK &&
                memcmp(x, g_st_2gx, 32) == 0 &&
                memcmp(y, g_st_2gy, 32) == 0, t);

  /* 2G + G + G = 4G */

  t = ba414e_count();
  ret = pic32mz_ba414e_ecc_add(c, x, y, g_st_2gx, g_st_2gy, c->gx, c->gy);
  if (ret == BA414E_OK)
    {
      ret = pic32mz_ba414e_ecc_add(c, x, y, x, y, c->gx, c->gy);
    }

  ba414e_report("add", ret == BA414E_OK &&
                memcmp(x, g_st_4gx, 32) == 0, t);

  t = ba414e_count();
  ret = pic32mz_ba414e_ecc_mul(c, x, y, c->gx, c->gy, k);
  ba414e_report("mul", ret == BA414E_OK &&
                memcmp(x, g_st_kgx, 32) == 0 &&
                memcmp(y, g_st_kgy, 32) == 0, t);

  t = ba414e_count();
  ret = pic32mz_ba414e_ecc_mul(c, x, y, c->gx, c->gy, c->n);
  ba414e_report("mul n", ret == BA414E_POINT_AT_INF, t);

  /* 3^(p-2) mod p, then 3 * 3^-1 mod p = 1 */

  memset(x, 0, sizeof(x));
  x[0] = 3;
  memcpy(e, c->p, 32);
  e[0] -= 2;
  t = ba414e_count();
  ret = pic32mz_ba414e_modexp(y, x, e, c->p, 32);
  ba414e_report("modexp", ret == BA414E_OK &&
                memcmp(y, g_st_inv3, 32) == 0, t);

  memset(one, 0, sizeof(one));
  one[0] = 1;
  t = ba414e_count();
  ret = pic32mz_ba414e_modmul(e, x, g_st_inv3, c->p, 32);
  ba414e_report("modmul", ret == BA414E_OK &&
                memcmp(e, one, 32) == 0, t);

  /* (p - 1) + 1 = 0, 0 - 1 = p - 1 */

  memcpy(e, c->p, 32);
  e[0] -= 1;
  t = ba414e_count();
  ret = pic32mz_ba414e_modadd(x, e, one, c->p, 32);
  memset(y, 0, sizeof(y));
  ba414e_report("modadd", ret == BA414E_OK && memcmp(x, y, 32) == 0, t);

  t = ba414e_count();
  ret = pic32mz_ba414e_modsub(x, y, one, c->p, 32);
  ba414e_report("modsub", ret == BA414E_OK && memcmp(x, e, 32) == 0, t);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pic32mz_ba414e_initialize(void)
{
  if (getreg32(PIC32MZ_PMD1) & PMD1_BA414MD)
    {
      syslog(LOG_ERR, "ba414e: disabled in PMD1\n");
      return -ENODEV;
    }

  putreg32(0, BA414E_PKCONTROL);
  irq_attach(PIC32MZ_IRQ_CRYPTO1, ba414e_isr, NULL);
  irq_attach(PIC32MZ_IRQ_CRYPTO1F, ba414e_isr, NULL);

  /* The microcode is loaded on first use, when its file system may be
   * mounted.
   */

  return OK;
}

int pic32mz_ba414e_modadd(FAR uint8_t *c, FAR const uint8_t *a,
                          FAR const uint8_t *b, FAR const uint8_t *p,
                          int len)
{
  return ba414e_modop(OPC_MOD_ADD, c, a, b, p, len);
}

int pic32mz_ba414e_modsub(FAR uint8_t *c, FAR const uint8_t *a,
                          FAR const uint8_t *b, FAR const uint8_t *p,
                          int len)
{
  return ba414e_modop(OPC_MOD_SUB, c, a, b, p, len);
}

int pic32mz_ba414e_modmul(FAR uint8_t *c, FAR const uint8_t *a,
                          FAR const uint8_t *b, FAR const uint8_t *p,
                          int len)
{
  return ba414e_modop(OPC_MOD_MULT, c, a, b, p, len);
}

int pic32mz_ba414e_modexp(FAR uint8_t *c, FAR const uint8_t *m,
                          FAR const uint8_t *e, FAR const uint8_t *n,
                          int len)
{
  int opsize = ba414e_opsize(len);
  int ret;

  if (len <= 0 || len > BA414E_MAX_BYTES)
    {
      return -EINVAL;
    }

  ret = ba414e_lock();
  if (ret < 0)
    {
      return ret;
    }

  ba414e_setup(OPC_MOD_EXP, opsize, PKCONFIG_OPPTRA(EXP_SLOT_M) |
               PKCONFIG_OPPTRB(EXP_SLOT_E) | PKCONFIG_OPPTRC(EXP_SLOT_C));
  ba414e_write_slot(EXP_SLOT_N, n, len, opsize * 8);
  ba414e_write_slot(EXP_SLOT_M, m, len, opsize * 8);
  ba414e_write_slot(EXP_SLOT_E, e, len, opsize * 8);

  ret = ba414e_run();
  if (ret == OK)
    {
      if (g_ba414e_fault)
        {
          ret = -EIO;
        }
      else
        {
          ba414e_read_slot(EXP_SLOT_C, c, len);
        }
    }

  nxrmutex_unlock(&g_ba414e_lock);
  return ret;
}

int pic32mz_ba414e_ecc_add(FAR const struct ba414e_curve_s *curve,
                           FAR uint8_t *rx, FAR uint8_t *ry,
                           FAR const uint8_t *px, FAR const uint8_t *py,
                           FAR const uint8_t *qx, FAR const uint8_t *qy)
{
  int slotlen = ba414e_opsize(curve->len) * 8;
  int ret;

  ret = ba414e_lock();
  if (ret < 0)
    {
      return ret;
    }

  /* [EX] The addition primitive needs distinct points: use the doubling
   * primitive for P + P.
   */

  if (memcmp(px, qx, curve->len) == 0 && memcmp(py, qy, curve->len) == 0)
    {
      ba414e_ecc_setup(curve, OPC_ECC_DOUBLE,
                       PKCONFIG_OPPTRA(ECC_SLOT_P1X) |
                       PKCONFIG_OPPTRC(ECC_SLOT_P3X), px, py);
    }
  else
    {
      ba414e_ecc_setup(curve, OPC_ECC_ADD,
                       PKCONFIG_OPPTRA(ECC_SLOT_P1X) |
                       PKCONFIG_OPPTRB(ECC_SLOT_P2X) |
                       PKCONFIG_OPPTRC(ECC_SLOT_P3X), px, py);
      ba414e_write_slot(ECC_SLOT_P2X, qx, curve->len, slotlen);
      ba414e_write_slot(ECC_SLOT_P2Y, qy, curve->len, slotlen);
    }

  ret = ba414e_ecc_finish(curve, rx, ry);
  nxrmutex_unlock(&g_ba414e_lock);
  return ret;
}

int pic32mz_ba414e_ecc_mul(FAR const struct ba414e_curve_s *curve,
                           FAR uint8_t *rx, FAR uint8_t *ry,
                           FAR const uint8_t *px, FAR const uint8_t *py,
                           FAR const uint8_t *k)
{
  int slotlen = ba414e_opsize(curve->len) * 8;
  int ret;

  ret = ba414e_lock();
  if (ret < 0)
    {
      return ret;
    }

  ba414e_ecc_setup(curve, OPC_ECC_MULT,
                   PKCONFIG_OPPTRA(ECC_SLOT_P1X) |
                   PKCONFIG_OPPTRB(ECC_SLOT_K) |
                   PKCONFIG_OPPTRC(ECC_SLOT_P3X), px, py);
  ba414e_write_slot(ECC_SLOT_K, k, curve->len, slotlen);

  ret = ba414e_ecc_finish(curve, rx, ry);
  nxrmutex_unlock(&g_ba414e_lock);
  return ret;
}

int pic32mz_ba414e_ecc_check(FAR const struct ba414e_curve_s *curve,
                             FAR const uint8_t *px, FAR const uint8_t *py)
{
  int ret;

  ret = ba414e_lock();
  if (ret < 0)
    {
      return ret;
    }

  ba414e_ecc_setup(curve, OPC_ECC_ONCURVE,
                   PKCONFIG_OPPTRA(ECC_SLOT_P1X), px, py);

  ret = ba414e_run();
  if (ret == OK)
    {
      /* [EX] PXNOC takes precedence over the fault indication */

      if (g_ba414e_status & PKSTATUS_PXNOC)
        {
          ret = BA414E_NOT_ON_CURVE;
        }
      else if (g_ba414e_fault)
        {
          ret = -EIO;
        }
    }

  nxrmutex_unlock(&g_ba414e_lock);
  return ret;
}
