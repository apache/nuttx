/****************************************************************************
 * arch/risc-v/src/common/riscv_pmp.c
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
#include <stdbool.h>

#include <nuttx/compiler.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PMP register length in bits */

#ifdef CONFIG_ARCH_RV32
#define PMP_XLEN                (32)
#else
#define PMP_XLEN                (64)
#endif

/* Minimum supported block size */

#if !defined CONFIG_ARCH_MPU_MIN_BLOCK_SIZE
#define MIN_BLOCK_SIZE          (PMP_XLEN / 8)
#else
#define MIN_BLOCK_SIZE          CONFIG_ARCH_MPU_MIN_BLOCK_SIZE
#endif

/* Address and block size alignment mask */

#define BLOCK_ALIGN_MASK        (MIN_BLOCK_SIZE - 1)

#define PMP_CFG_BITS_CNT        (8)
#define PMP_CFG_FLAG_MASK       ((uintptr_t)0xFF)

#define PMP_CFG_CNT_IN_REG      (PMP_XLEN / PMP_CFG_BITS_CNT)

#define PMP_MASK_SET_ONE_REGION(region, attr, reg) \
  do { \
      uintptr_t offset = region % PMP_CFG_CNT_IN_REG; \
      reg &= ~(PMP_CFG_FLAG_MASK << (offset * PMP_CFG_BITS_CNT)); \
      reg |= attr << (offset * PMP_CFG_BITS_CNT); \
    } while(0);

#define PMP_READ_REGION_FROM_REG(region, reg) \
  ({ \
    uintptr_t region##_val = READ_CSR(reg); \
    region##_val >>= ((region % PMP_CFG_CNT_IN_REG) * PMP_CFG_BITS_CNT); \
    region##_val &= PMP_CFG_FLAG_MASK; \
    region##_val; \
  })

#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Helper structure for handling a PMP entry */

struct pmp_entry_s
{
  uintptr_t base;   /* Base address of region */
  uintptr_t end;    /* End address of region */
  uintptr_t size;   /* Region size */
  uint8_t   mode;   /* Address matching mode */
  uint8_t   rwx;    /* Access rights */
};

typedef struct pmp_entry_s pmp_entry_t;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pmp_check_addrmatch_type
 *
 * Description:
 *   Test if an address matching type is supported by the architecture.
 *
 * Input Parameters:
 *   type - The type to test.
 *
 * Returned Value:
 *   true if it is, false otherwise.
 *
 ****************************************************************************/

static bool pmp_check_addrmatch_type(uintptr_t type)
{
  /* Parameter is potentially unused */

  UNUSED(type);
#ifdef CONFIG_ARCH_MPU_HAS_TOR
  if (type == PMPCFG_A_TOR)
    {
      return true;
    }

#endif
#ifdef CONFIG_ARCH_MPU_HAS_NO4
  if (type == PMPCFG_A_NA4)
    {
      return true;
    }

#endif
#ifdef CONFIG_ARCH_MPU_HAS_NAPOT
  if (type == PMPCFG_A_NAPOT)
    {
      return true;
    }
#endif

  /* None of the supported types match */

  return false;
}

/****************************************************************************
 * Name: pmp_check_region_attrs
 *
 * Description:
 *   Test if the base address and size of region meet alignment requirements.
 *
 * Input Parameters:
 *   base - The base address of the region.
 *   size - The memory length of the region.
 *
 * Returned Value:
 *   true if it is, false otherwise.
 *
 ****************************************************************************/

static bool pmp_check_region_attrs(uintptr_t base, uintptr_t size)
{
  /* Check that the size is not too small */

  if (size < MIN_BLOCK_SIZE)
    {
      return false;
    }

  /* Check that the base address is aligned properly */

  if ((base & BLOCK_ALIGN_MASK) != 0)
    {
      return false;
    }

  /* Check that the size is aligned properly */

  if ((size & BLOCK_ALIGN_MASK) != 0)
    {
      return false;
    }

  return OK;
}

/****************************************************************************
 * Name: pmp_read_region_cfg
 *
 * Description:
 *   Read PMP configuration for region
 *
 * Input Parameters:
 *   region - Region number.
 *
 * Returned Value:
 *   Configuration value from pmpcfg+region
 *
 ****************************************************************************/

static uintptr_t pmp_read_region_cfg(uintptr_t region)
{
# if (PMP_XLEN == 32)
  switch (region)
    {
      case 0 ... 3:
        return PMP_READ_REGION_FROM_REG(region, pmpcfg0);

      case 4 ... 7:
        return PMP_READ_REGION_FROM_REG(region, pmpcfg1);

      case 8 ... 11:
        return PMP_READ_REGION_FROM_REG(region, pmpcfg2);

      case 12 ... 15:
        return PMP_READ_REGION_FROM_REG(region, pmpcfg3);

      default:
        break;
    }
# elif (PMP_XLEN == 64)
  switch (region)
    {
      case 0 ... 7:
        return PMP_READ_REGION_FROM_REG(region, pmpcfg0);

      case 8 ... 15:
        return PMP_READ_REGION_FROM_REG(region, pmpcfg2);

      default:
        break;
    }
#endif

  /* Never executed */

  return 0;
}

/****************************************************************************
 * Name: pmp_read_addr
 *
 * Description:
 *   Read address for region
 *
 * Input Parameters:
 *   region - Region number.
 *
 * Returned Value:
 *   Address value from pmpcfg+region
 *
 ****************************************************************************/

static uintptr_t pmp_read_addr(uintptr_t region)
{
  switch (region)
      {
        case 0:
          return READ_CSR(pmpaddr0);

        case 1:
          return READ_CSR(pmpaddr1);

        case 2:
          return READ_CSR(pmpaddr2);

        case 3:
          return READ_CSR(pmpaddr3);

        case 4:
          return READ_CSR(pmpaddr4);

        case 5:
          return READ_CSR(pmpaddr5);

        case 6:
          return READ_CSR(pmpaddr6);

        case 7:
          return READ_CSR(pmpaddr7);

        case 8:
          return READ_CSR(pmpaddr8);

        case 9:
          return READ_CSR(pmpaddr9);

        case 10:
          return READ_CSR(pmpaddr10);

        case 11:
          return READ_CSR(pmpaddr11);

        case 12:
          return READ_CSR(pmpaddr12);

        case 13:
          return READ_CSR(pmpaddr13);

        case 14:
          return READ_CSR(pmpaddr14);

        case 15:
          return READ_CSR(pmpaddr15);

        default:
          break;
      }

  /* Never executed */

  return 0;
}

/****************************************************************************
 * Name: pmp_napot_decode
 *
 * Description:
 *   Decode base and size from NAPOT value
 *
 * Input Parameters:
 *   val  - Value to decode.
 *   size - Size out.
 *
 * Returned Value:
 *   Base address
 *
 ****************************************************************************/

static uintptr_t pmp_napot_decode(uintptr_t val, uintptr_t * size)
{
  uintptr_t mask = (uintptr_t)(-1) >> 1;
  uintptr_t pot  = PMP_XLEN + 2;

  while (mask)
    {
      if ((val & mask) == mask)
        {
          break;
        }

      pot--;
      mask >>= 1;
    }

  val &= ~mask;
  *size = UINT64_C(1) << pot;
  return (val << 2);
}

/****************************************************************************
 * Name: pmp_read
 *
 * Description:
 *   Read PMP region into PMP entry
 *
 * Input Parameters:
 *   region - Region number.
 *   entry  - Entry out
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pmp_read(uintptr_t region, pmp_entry_t * entry)
{
  uintptr_t addr = 0;
  uintptr_t size = 0;
  uintptr_t mode = 0;
  uintptr_t rwx  = 0;
  uintptr_t cfg  = 0;

  addr = pmp_read_addr(region);
  cfg  = pmp_read_region_cfg(region);
  mode = cfg & PMPCFG_A_MASK;
  rwx  = cfg & PMPCFG_RWX_MASK;

  switch (mode)
  {
    case PMPCFG_A_TOR:
      addr <<= 2;

      /* TOR region, must peek into prior region for size */

      if (region == 0)
        {
          size = addr;
        }
      else
        {
          size = addr - pmp_read_addr(region - 1);
        }

      break;

    case PMPCFG_A_NA4:
      addr <<= 2;
      size = 4;
      break;

    case PMPCFG_A_NAPOT:
      addr = pmp_napot_decode(addr, &size);
      break;

    default:
      break;
  }

  entry->base = addr;
  entry->end  = addr + size;
  entry->size = size;
  entry->rwx  = rwx;
  entry->mode = mode;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_config_pmp_region
 *
 * Description:
 *   This function will set the specific PMP region with the desired cfg.
 *
 * Input Parameters:
 *   region - The region index number.
 *   attr - The region configurations.
 *   base - The base address of the region.
 *   size - The memory length of the region.
 *   For the NAPOT mode, the base address must aligned to the size boundary,
 *   and the size must be power-of-two according to the the PMP spec.
 *
 * Returned Value:
 *   0 on succeess; negated error on failure
 *
 ****************************************************************************/

int riscv_config_pmp_region(uintptr_t region, uintptr_t attr,
                            uintptr_t base, uintptr_t size)
{
  uintptr_t addr    = 0;
  uintptr_t cfg     = 0;
  uintptr_t type    = (attr & PMPCFG_A_MASK);

  /* Check that the architecture supports address matching type */

  if (pmp_check_addrmatch_type(type) == false)
    {
      return -EINVAL;
    }

  /* Check the region attributes */

  if (pmp_check_region_attrs(base, size))
    {
      return -EINVAL;
    }

  /* Calculate new base address from type */

  addr = base >> 2;
  if (PMPCFG_A_NAPOT == (attr & PMPCFG_A_MASK))
    {
      addr |= (size - 1) >> 3;
    }

  /* Set the address value */

  switch (region)
    {
      case 0:
        WRITE_CSR(pmpaddr0, addr);
        break;

      case 1:
        WRITE_CSR(pmpaddr1, addr);
        break;

      case 2:
        WRITE_CSR(pmpaddr2, addr);
        break;

      case 3:
        WRITE_CSR(pmpaddr3, addr);
        break;

      case 4:
        WRITE_CSR(pmpaddr4, addr);
        break;

      case 5:
        WRITE_CSR(pmpaddr5, addr);
        break;

      case 6:
        WRITE_CSR(pmpaddr6, addr);
        break;

      case 7:
        WRITE_CSR(pmpaddr7, addr);
        break;

      case 8:
        WRITE_CSR(pmpaddr8, addr);
        break;

      case 9:
        WRITE_CSR(pmpaddr9, addr);
        break;

      case 10:
        WRITE_CSR(pmpaddr10, addr);
        break;

      case 11:
        WRITE_CSR(pmpaddr11, addr);
        break;

      case 12:
        WRITE_CSR(pmpaddr12, addr);
        break;

      case 13:
        WRITE_CSR(pmpaddr13, addr);
        break;

      case 14:
        WRITE_CSR(pmpaddr14, addr);
        break;

      case 15:
        WRITE_CSR(pmpaddr15, addr);
        break;

      default:
        break;
    }

  /* Set the configuration register value */

# if (PMP_XLEN == 32)
  switch (region)
    {
      case 0 ... 3:
        cfg = READ_CSR(pmpcfg0);
        PMP_MASK_SET_ONE_REGION(region, attr, cfg);
        WRITE_CSR(pmpcfg0, cfg);
        break;

      case 4 ... 7:
        cfg = READ_CSR(pmpcfg1);
        PMP_MASK_SET_ONE_REGION(region, attr, cfg);
        WRITE_CSR(pmpcfg1, cfg);
        break;

      case 8 ... 11:
        cfg = READ_CSR(pmpcfg2);
        PMP_MASK_SET_ONE_REGION(region, attr, cfg);
        WRITE_CSR(pmpcfg2, cfg);
        break;

      case 12 ... 15:
        cfg = READ_CSR(pmpcfg3);
        PMP_MASK_SET_ONE_REGION(region, attr, cfg);
        WRITE_CSR(pmpcfg3, cfg);
        break;

      default:
        break;
    }
# elif (PMP_XLEN == 64)
  switch (region)
    {
      case 0 ... 7:
        cfg = READ_CSR(pmpcfg0);
        PMP_MASK_SET_ONE_REGION(region, attr, cfg);
        WRITE_CSR(pmpcfg0, cfg);
        break;

      case 8 ... 15:
        cfg = READ_CSR(pmpcfg2);
        PMP_MASK_SET_ONE_REGION(region, attr, cfg);
        WRITE_CSR(pmpcfg2, cfg);
        break;

      default:
        break;
    }
# else
#   error "XLEN of risc-v not supported"
# endif

#ifdef CONFIG_ARCH_USE_S_MODE
  /* Fence is needed when page-based virtual memory is implemented.
   * If page-based virtual memory is not implemented, memory accesses check
   * the PMP settings synchronously, so no SFENCE.VMA is needed.
   */

  __asm volatile("sfence.vma x0, x0" : : : "memory");
#endif

  return OK;
}

/****************************************************************************
 * Name: riscv_check_pmp_access
 *
 * Description:
 *   This function will set the specific PMP region with the desired cfg.
 *
 * Input Parameters:
 *   attr - The region configurations.
 *   base - The base address of the region.
 *   size - The memory length of the region.
 *   For the NAPOT mode, the base address must aligned to the size boundary,
 *   and the size must be power-of-two according to the the PMP spec.
 *
 * Returned Value:
 *   0 if access rights are not set at all
 *   < 0 if access rights are set and match match partially
 *   > 0 if access rights are set and match fully
 *
 ****************************************************************************/

int riscv_check_pmp_access(uintptr_t attr, uintptr_t base, uintptr_t size)
{
  pmp_entry_t   entry;
  uintptr_t     end;
  uintptr_t     orgsize;
  unsigned int  region;

  /* Go through every single configured region and test the attributes */

  attr    = (attr & PMPCFG_RWX_MASK);
  end     = base + size;
  orgsize = size;

  for (region = 0; region < 16 && size > 0; region++)
    {
      /* Find matching configuration first */

      pmp_read(region, &entry);

      /* Check if any configuration at all */

      if (entry.mode == PMPCFG_A_OFF)
        {
          continue;
        }

      /* Does this address range match ? Take partial matches into account.
       *
       * NOTE: The PMP end address itself is not part of the mapping
       *
       * There are four possibilities:
       * 1: Full match; region inside mapped area
       * 2: Partial match; mapped area inside region
       * 3: Partial match; base inside mapped region, end outside
       * 4: Partial match; base outside mapped region, end inside
       */

      if ((base >= entry.base && end  <= entry.end) ||
          (base <  entry.base && end  >  entry.end) ||
          (base >= entry.base && base <  entry.end) ||
          (end  >  entry.base && end  <= entry.end))
        {
          /* Found a matching splice, check rights */

          if ((entry.rwx & attr) == attr)
            {
              /* Found matching region that allows access */

              size -= min(end, entry.end) - max(base, entry.base);
            }
          else
            {
              /* Found matching region that does not allow access */

              return PMP_ACCESS_DENIED;
            }
        }
    }

  /* Check if nothing configured at all ? */

  if (size == orgsize)
    {
      return PMP_ACCESS_OFF;
    }

  /* If size is non-positive, the requested range is accessible */

  if (size <= 0)
    {
      return PMP_ACCESS_FULL;
    }

  /* The requested range is either fully or partially inaccessible */

  return PMP_ACCESS_DENIED;
}

/****************************************************************************
 * Name: riscv_configured_pmp_regions
 *
 * Description:
 *   Count amount of configured PMP regions, note: is not atomic
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Amount of configured PMP regions
 *
 ****************************************************************************/

int riscv_configured_pmp_regions(void)
{
  pmp_entry_t   entry;
  unsigned int  region;
  int           ret = 0;

  for (region = 0; region < 16; region++)
    {
      pmp_read(region, &entry);

      if (entry.mode != PMPCFG_A_OFF)
        {
          ret++;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: riscv_next_free_pmp_region
 *
 * Description:
 *   Returns next free PMP region, note: is not atomic
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Next free PMP region, or -1 if none found
 *
 ****************************************************************************/

int riscv_next_free_pmp_region(void)
{
  pmp_entry_t   entry;
  unsigned int  region;

  for (region = 0; region < 16; region++)
    {
      pmp_read(region, &entry);

      if (entry.mode == PMPCFG_A_OFF)
        {
          return region;
        }
    }

  return -1;
}
