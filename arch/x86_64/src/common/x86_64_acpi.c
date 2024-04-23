/****************************************************************************
 * arch/x86_64/src/common/x86_64_acpi.c
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

#include <nuttx/arch.h>
#include <debug.h>
#include <stdint.h>

#include <arch/acpi.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACPI_BIOS_MEM_START (0x000e0000)
#define ACPI_BIOS_MEM_END   (0x000fffff)
#define ACPI_BIOS_MEM_SIZE  (ACPI_BIOS_MEM_END - ACPI_BIOS_MEM_START)
#define ACPI_PAGE_FLAGS     (X86_PAGE_PRESENT | X86_PAGE_WR | X86_PAGE_NOCACHE)

/* Debug macros */

#define acpi_info  _info
#define acpi_err   _err
#define acpi_warn  _warn

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct acpi_s
{
  struct acpi_rsdp_s *rsdp;
  struct acpi_rsdt_s *rsdt;
  struct acpi_xsdt_s *xsdt;
  struct acpi_madt_s *madt;
  struct acpi_mcfg_s *mcfg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct acpi_s g_acpi;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: acpi_map_region
 *
 * Description:
 *   Map ACPI region.
 *
 ****************************************************************************/

static void acpi_map_region(uintptr_t addr, size_t size)
{
  /* Map region */

  up_map_region((void *)addr, size, ACPI_PAGE_FLAGS);
}

/****************************************************************************
 * Name: acpi_map_rsdt
 *
 * Description:
 *   Map all ACPI tables.
 *
 ****************************************************************************/

static void acpi_map_rsdt(void)
{
  void     *tps   = NULL;
  uint32_t *tp32  = NULL;
  uint32_t *end32 = NULL;

  tps = &g_acpi.rsdt->table_ptrs;
  tp32  = (uint32_t *)tps;
  end32 = (uint32_t *)((uintptr_t)g_acpi.rsdt + g_acpi.rsdt->sdt.length);

  while (tp32 < end32)
    {
      acpi_map_region((uintptr_t)*tp32,
                      sizeof(struct acpi_sdt_s));

      /* Next table */

      tp32 += 1;
    }
}

/****************************************************************************
 * Name: acpi_map_xsdt
 *
 * Description:
 *   Map all ACPI tables.
 *
 ****************************************************************************/

static void acpi_map_xsdt(void)
{
  void     *tps   = NULL;
  uint64_t *tp64  = NULL;
  uint64_t *end64 = NULL;

  tps   = &g_acpi.rsdt->table_ptrs;
  tp64  = (uint64_t *)tps;
  end64 = (uint64_t *)((uintptr_t)g_acpi.xsdt + g_acpi.xsdt->sdt.length);

  while (tp64 < end64)
    {
      acpi_map_region((uintptr_t)*tp64,
                      sizeof(struct acpi_sdt_s));

      /* Next table */

      tp64 += 1;
    }
}

/****************************************************************************
 * Name: acpi_sdt_checksum
 *
 * Description:
 *   Verify checksum.
 *
 ****************************************************************************/

static bool acpi_sdt_checksum(struct acpi_sdt_s *sdt)
{
  uint8_t *ptr = (uint8_t *)sdt;
  uint8_t  sum = 0;
  uint32_t i   = 0;

  for (i = 0; i < sdt->length; i++)
    {
      sum += ptr[i];
    }

  return sum == 0;
}

/****************************************************************************
 * Name: acpi_rsdp_parse
 *
 * Description:
 *   Parse RSDP.
 *
 ****************************************************************************/

static int acpi_rsdp_parse(struct acpi_rsdp_s *rsdp)
{
  uint8_t  *ptr = (uint8_t *)rsdp;
  uint8_t   sum = 0;
  uint32_t  len = 0;
  uint32_t  i   = 0;

  /* Checksum */

  len = rsdp->revision < 2 ? 20 : rsdp->length;
  for (i = 0; i < len; i++)
    {
      sum += ptr[i];
    }

  if (sum != 0)
    {
      return -EINVAL;
    }

  /* Parse RSDT table */

  if (rsdp->rsdt_addr != 0)
    {
      struct acpi_rsdt_s *rsdt = NULL;

      acpi_map_region((uintptr_t)rsdp->rsdt_addr,
                      sizeof(struct acpi_rsdt_s));

      rsdt = (struct acpi_rsdt_s *)(uintptr_t)rsdp->rsdt_addr;

      /* Verify checksum */

      if (!acpi_sdt_checksum(&rsdt->sdt))
        {
          return -EINVAL;
        }

      /* Store pointer */

      g_acpi.rsdt = rsdt;

      /* Map all tables */

      acpi_map_rsdt();
    }

  /* Parse XSDT table if RSDT not available */

  else if (rsdp->xsdt_addr != 0)
    {
      struct acpi_xsdt_s *xsdt = NULL;

      acpi_map_region((uintptr_t)rsdp->xsdt_addr,
                      sizeof(struct acpi_xsdt_s));

      xsdt = (struct acpi_xsdt_s *)rsdp->xsdt_addr;

      /* Verify checksum */

      if (!acpi_sdt_checksum(&xsdt->sdt))
        {
          return -EINVAL;
        }

      /* Store pointer */

      g_acpi.xsdt = xsdt;

      /* Map all talbes */

      acpi_map_xsdt();
    }

  return OK;
}

#ifdef CONFIG_ARCH_X86_64_ACPI_BIOS
/****************************************************************************
 * Name: acpi_rsdp_find_bios
 *
 * Description:
 *   Find RSDP in BIOS region.
 *
 ****************************************************************************/

static bool acpi_rsdp_find_bios(struct acpi_s *acpi)
{
  char *now = (char *)ACPI_BIOS_MEM_START;
  char *end = (char *)ACPI_BIOS_MEM_END;

  /* Map BIOS region */

  acpi_map_region((uintptr_t)ACPI_BIOS_MEM_START,
                  ACPI_BIOS_MEM_SIZE);

  acpi->rsdp = NULL;

  while (now < end)
    {
      if (strncmp(now, ACPI_SIG_RSDP, sizeof(ACPI_SIG_RSDP) - 1) == 0)
        {
          acpi->rsdp = (struct acpi_rsdp_s *)now;
          break;
        }

      now += 16;
    }

  return acpi->rsdp != NULL;
}
#endif

/****************************************************************************
 * Name: acpi_rsdp_find
 *
 * Description:
 *   Find RSDP.
 *
 ****************************************************************************/

static bool acpi_rsdp_find(struct acpi_s *acpi)
{
  /* For now ony ACPI from BIOS region us supported */

#ifdef CONFIG_ARCH_X86_64_ACPI_BIOS
  return acpi_rsdp_find_bios(acpi);
#else
#  error For now ony ACPI from BIOS region is supported
#endif
}

/****************************************************************************
 * Name: acpi_table64_find
 ****************************************************************************/

static int acpi_table64_find(const char *sig, struct acpi_sdt_s **sdt)
{
  static struct acpi_s *acpi = &g_acpi;
  struct acpi_sdt_s    *tmp  = NULL;
  void                 *tps  = NULL;
  uint64_t             *tp   = NULL;
  uint64_t             *end  = NULL;

  tps = &acpi->xsdt->table_ptrs;
  tp  = (uint64_t *)((uintptr_t)tps);
  end = (uint64_t *)((uintptr_t)acpi->xsdt + acpi->xsdt->sdt.length);

  while (tp < end)
    {
      /* Compare signature */

      tmp = (struct acpi_sdt_s *)(uintptr_t)*tp;
      if (strncmp(tmp->signature, sig, 4) == 0)
        {
          *sdt = tmp;
          return OK;
        }

      /* Next table */

      tp += 1;
    }

  /* Not found */

  return -ENOENT;
}

/****************************************************************************
 * Name: acpi_table32_find
 ****************************************************************************/

static int acpi_table32_find(const char *sig, struct acpi_sdt_s **sdt)
{
  struct acpi_s     *acpi = &g_acpi;
  struct acpi_sdt_s *tmp  = NULL;
  void              *tps  = NULL;
  uint32_t          *tp   = NULL;
  uint32_t          *end  = NULL;

  tps = &acpi->rsdt->table_ptrs;
  tp  = (uint32_t *)((uintptr_t)tps);
  end = (uint32_t *)((uintptr_t)acpi->rsdt + acpi->rsdt->sdt.length);

  while (tp < end)
    {
      /* Compare signature */

      tmp = (struct acpi_sdt_s *)(uintptr_t)*tp;
      if (strncmp(tmp->signature, sig, 4) == 0)
        {
          *sdt = tmp;
          return OK;
        }

      /* Next table */

      tp += 1;
    }

  /* Not found */

  return -ENOENT;
}

/****************************************************************************
 * Name: acpi_table_find
 *
 * Description:
 *   Find and return table with a given signature
 *
 ****************************************************************************/

static int acpi_table_find(const char *sig, struct acpi_sdt_s **sdt)
{
  struct acpi_s *acpi = &g_acpi;

  /* 64 bit pointers or 32 bit pointers */

  if (acpi->xsdt != 0)
    {
      return acpi_table64_find(sig, sdt);
    }
  else
    {
      return acpi_table32_find(sig, sdt);
    }
}

#ifdef CONFIG_ARCH_X86_64_ACPI_DUMP
/****************************************************************************
 * Name: acpi_sdt_dump
 *
 * Description:
 *   Dump SDT entry
 *
 ****************************************************************************/

static void acpi_sdt_dump(struct acpi_sdt_s *sdt)
{
  acpi_info("ptr = %p sig = %.4s", sdt, sdt->signature);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: acpi_init
 *
 * Description:
 *   Initialize ACPI parser.
 *
 ****************************************************************************/

int acpi_init(uintptr_t rsdp)
{
  struct acpi_s *acpi = &g_acpi;
  int            ret  = OK;

  if (rsdp == 0)
    {
      /* Find RSP */

      if (!acpi_rsdp_find(acpi))
        {
          return -EINVAL;
        }
    }
  else
    {
      acpi->rsdp = (struct acpi_rsdp_s *)rsdp;
    }

  /* Make sure that RSDP is mapped */

  acpi_map_region((uintptr_t)acpi->rsdp, sizeof(struct acpi_rsdp_s));

  /* Parse RSDP */

  ret = acpi_rsdp_parse(acpi->rsdp);
  if (ret < 0)
    {
      return ret;
    }

  /* Cache some useful tables */

  /* Get MADT table */

  acpi_table_find(ACPI_SIG_APIC, (struct acpi_sdt_s **)&acpi->madt);

  /* Get MCFG */

  acpi_table_find(ACPI_SIG_MCFG, (struct acpi_sdt_s **)&acpi->mcfg);

  return OK;
}

/****************************************************************************
 * Name: acpi_madt_get
 *
 * Description:
 *   Find the n'th occurence of a MADT entry with a given type.
 *
 ****************************************************************************/

int acpi_madt_get(int type, int n, struct acpi_entry_s **entry)
{
  struct acpi_s       *acpi = &g_acpi;
  struct acpi_entry_s *tmp  = NULL;
  uint8_t             *ptr  = NULL;
  uint8_t             *end  = NULL;

  if (acpi->madt == NULL)
    {
      return -EINVAL;
    }

  ptr = (uint8_t *)&g_acpi.madt->entries;
  end = (uint8_t *)((uintptr_t)g_acpi.madt + g_acpi.madt->sdt.length);

  while (ptr < end)
    {
      tmp = (struct acpi_entry_s *)ptr;

      if (tmp->type == type)
        {
          if (n-- == 0)
            {
              *entry = tmp;
              return OK;
            }
        }

      ptr += tmp->length;
    }

  /* Not found */

  return -ENOENT;
}

/****************************************************************************
 * Name: acpi_lapi_get
 *
 * Description:
 *   Get Local APIC entry for a given CPU.
 *
 ****************************************************************************/

int acpi_lapic_get(int cpu, struct acpi_lapic_s **lapic)
{
  return acpi_madt_get(ACPI_MADT_TYPE_LOCAL_APIC, cpu,
                       (struct acpi_entry_s **)lapic);
}

#ifdef CONFIG_ARCH_X86_64_ACPI_DUMP
/****************************************************************************
 * Name: acpi_dump
 *
 * Description:
 *   Dump ACPI tables.
 *
 ****************************************************************************/

void acpi_dump(void)
{
  struct acpi_entry_s *entry = NULL;
  struct acpi_lapic_s *lapic = NULL;
  uint8_t             *ptr8  = NULL;
  uint8_t             *end8  = NULL;
  struct acpi_entry_s *tmp   = NULL;
  void                *tps   = NULL;
  uint64_t            *tp64  = NULL;
  uint64_t            *end64 = NULL;
  uint32_t            *tp32  = NULL;
  uint32_t            *end32 = NULL;
  int                  i     = 0;
  int                  ret   = 0;

  /* Dump entires */

  if (g_acpi.xsdt != 0)
    {
      tps   = &g_acpi.rsdt->table_ptrs;
      tp64  = (uint64_t *)tps;
      end64 = (uint64_t *)((uintptr_t)g_acpi.xsdt + g_acpi.xsdt->sdt.length);

      acpi_info("XSDT = %p", g_acpi.xsdt);
      acpi_info("  tp64 = %p", tp64);
      acpi_info("  end64 = %p", end64);

      while (tp64 < end64)
        {
          acpi_sdt_dump((struct acpi_sdt_s *)(uintptr_t)*tp64);

          /* Next table */

          tp64 += 1;
        }
    }
  else
    {
      tps   = &g_acpi.rsdt->table_ptrs;
      tp32  = (uint32_t *)tps;
      end32 = (uint32_t *)((uintptr_t)g_acpi.rsdt + g_acpi.rsdt->sdt.length);

      acpi_info("RSDT = %p", g_acpi.rsdt);
      acpi_info("  tp32 = %p", tp32);
      acpi_info("  end32 = %p", end32);

      while (tp32 < end32)
        {
          acpi_sdt_dump((struct acpi_sdt_s *)(uintptr_t)*tp32);

          /* Next table */

          tp32 += 1;
        }
    }

  /* Dump MADT */

  ptr8 = (uint8_t *)&g_acpi.madt->entries;
  end8 = (uint8_t *)((uint64_t)g_acpi.madt + g_acpi.madt->sdt.length);

  while (ptr8 < end8)
    {
      tmp = (struct acpi_entry_s *)ptr8;
      acpi_info("Found MADT type %d %p", tmp->type, tmp);
      ptr8 += tmp->length;
    }

  /* Print all CPU */

  while (true)
    {
      ret = acpi_lapic_get(i, &lapic);
      if (ret < 0)
        {
          break;
        }

      acpi_info("Found LAPIC for CPU %d %p", i, lapic);
      acpi_info("    ACPI ID %d", lapic->acpi_id);
      acpi_info("    APIC ID %d", lapic->apic_id);
      acpi_info("    flags %d", lapic->flags);

      /* IO_APIC */

      ret = acpi_madt_get(ACPI_MADT_TYPE_IO_APIC, i, &entry);
      if (ret == OK)
        {
          acpi_info("Found IOAPIC %p", entry);
        }

      /* APIC64 */

      ret = acpi_madt_get(ACPI_MADT_TYPE_LOCAL_APIC64, i, &entry);
      if (ret == OK)
        {
          acpi_info("Found 64 bit APIC %p", entry);
        }

      /* X2APIC */

      ret = acpi_madt_get(ACPI_MADT_TYPE_LOCAL_X2APIC, i, &entry);
      if (ret == OK)
        {
          acpi_info("Found X2APIC %p", entry);
        }

      /* Try next core */

      i += 1;
    }
}
#endif
