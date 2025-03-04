/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_opensbi.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/mpfs_plic.h>
#include <hardware/mpfs_memorymap.h>
#include <hardware/mpfs_clint.h>
#include <hardware/mpfs_sysreg.h>
#ifdef CONFIG_MPFS_IHC_SBI
#include <hardware/mpfs_ihc_sbi.h>
#include <mpfs_ihc.h>
#endif
#include "mpfs_entrypoints.h"

/* Make sure that anything that intefraces with the SBI uses the same data
 * types as the SBI code (e.g. same "bool")
 */

#ifdef bool
#undef bool
#endif

#ifdef true
#undef true
#endif

#ifdef false
#undef false
#endif

#ifdef NULL
#undef NULL
#endif

#include <sbi/sbi_types.h>
#include <sbi/riscv_io.h>
#include <sbi/riscv_encoding.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_init.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_trap.h>
#include <sbi_utils/irqchip/plic.h>
#include <sbi_utils/ipi/aclint_mswi.h>
#include <sbi_utils/timer/aclint_mtimer.h>

#ifdef CONFIG_MPFS_IHC_SBI
#include <mpfs_ihc.h>
#endif

#include <sys/param.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_SYS_CLK               1000000000
#define MPFS_MAX_NUM_HARTS         5
#define MPFS_HART_COUNT            5

#define MPFS_ACLINT_MSWI_ADDR      MPFS_CLINT_MSIP0
#define MPFS_ACLINT_MTIMER_ADDR    MPFS_CLINT_MTIMECMP0

#define MPFS_SYSREG_SOFT_RESET_CR     (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SOFT_RESET_CR_OFFSET)
#define MPFS_SYSREG_SUBBLK_CLOCK_CR   (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET)

#define MICROCHIP_TECHNOLOGY_MVENDOR_ID  0x029
#define SBI_EXT_MICROCHIP_TECHNOLOGY     (SBI_EXT_VENDOR_START | \
                                          MICROCHIP_TECHNOLOGY_MVENDOR_ID)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sbi_scratch_holder_s
{
  union
    {
      struct sbi_scratch scratch;
      unsigned long buffer[SBI_SCRATCH_SIZE / sizeof(uintptr_t)];
    };
};

typedef struct sbi_scratch_holder_s sbi_scratch_holder_t;

/* Linker provided region start / end addresses */

extern const uint8_t __mpfs_nuttx_start[];
extern const uint8_t __mpfs_nuttx_end[];
extern const uint8_t _ssbi_ram[];
extern const uint8_t _esbi_ram[];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mpfs_console_putc(char ch);
static int  mpfs_early_init(bool cold_boot);
static int  mpfs_opensbi_console_init(void);
static int  mpfs_irqchip_init(bool cold_boot);
static int  mpfs_ipi_init(bool cold_boot);
static int  mpfs_timer_init(bool cold_boot);
#ifdef CONFIG_MPFS_IHC_SBI
static bool mpfs_opensbi_vendor_ext_check(void);
static int  mpfs_opensbi_ecall_handler(long funcid,
                                       const struct sbi_trap_regs *regs,
                                       unsigned long *out_val,
                                       struct sbi_trap_info *out_trap);
#endif

/****************************************************************************
 * Extern Function Declarations
 ****************************************************************************/

/* riscv_internal.h cannot be included due to a number of redefinition
 * conflicts.  Thus, define the riscv_lowputc() with the extern definition.
 */

extern void riscv_lowputc(char ch);
extern uintptr_t mpfs_get_entrypt(uint64_t hartid);

/* domains init implemented in board specific file */

#ifdef CONFIG_OPENSBI_DOMAINS
extern int board_domains_init(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool mpfs_console_ready    = false;

static struct plic_data mpfs_plic =
{
  .addr           = MPFS_PLIC_BASE,
  .num_src        = MPFS_HART_COUNT,
};

static struct sbi_console_device mpfs_console =
{
  .name           = "mpfs_uart",
  .console_putc   = mpfs_console_putc,
  .console_getc   = NULL,
};

static struct aclint_mtimer_data mpfs_mtimer =
{
  .mtime_freq     = MPFS_SYS_CLK,
  .mtime_addr     = MPFS_ACLINT_MTIMER_ADDR + ACLINT_DEFAULT_MTIME_OFFSET,
  .mtime_size     = ACLINT_DEFAULT_MTIME_SIZE,
  .mtimecmp_addr  = MPFS_ACLINT_MTIMER_ADDR + ACLINT_DEFAULT_MTIMECMP_OFFSET,
  .mtimecmp_size  = ACLINT_DEFAULT_MTIMECMP_SIZE,
  .first_hartid   = 0,
  .hart_count     = MPFS_HART_COUNT,
  .has_64bit_mmio = true,
};

static const struct sbi_platform_operations platform_ops =
{
#ifdef CONFIG_OPENSBI_DOMAINS
  .domains_init        = board_domains_init,
#endif
  .console_init        = mpfs_opensbi_console_init,
  .early_init          = mpfs_early_init,
  .irqchip_init        = mpfs_irqchip_init,
  .irqchip_exit        = NULL,
  .ipi_init            = mpfs_ipi_init,
  .ipi_exit            = NULL,
  .timer_init          = mpfs_timer_init,
  .timer_exit          = NULL,
#ifdef CONFIG_MPFS_IHC_SBI
  .vendor_ext_check    = mpfs_opensbi_vendor_ext_check,
  .vendor_ext_provider = mpfs_opensbi_ecall_handler,
#endif
};

static struct aclint_mswi_data mpfs_mswi =
{
  .addr           = MPFS_ACLINT_MSWI_ADDR,
  .size           = ACLINT_MSWI_SIZE,
  .first_hartid   = 0,
  .hart_count     = MPFS_HART_COUNT,
};

/* OpenSBI picks the used and unused harts via the hart_index2id table.
 * Unused hart is marked with -1.  Mpfs will always have the hart0 unused.
 */

static u32 mpfs_hart_index2id[MPFS_HART_COUNT];

static const struct sbi_platform platform =
{
  .opensbi_version   = OPENSBI_VERSION,
  .platform_version  = SBI_PLATFORM_VERSION(0x0, 0x01),
  .name              = "Microchip PolarFire(R) SoC",
  .features          = SBI_PLATFORM_DEFAULT_FEATURES,
  .hart_count        = MPFS_HART_COUNT,
  .hart_index2id     = mpfs_hart_index2id,
  .hart_stack_size   = SBI_PLATFORM_DEFAULT_HART_STACK_SIZE,
  .platform_ops_addr = (unsigned long)&platform_ops,
  .firmware_context  = 0
};

/* This must go into l2_scratchpad region, starting at 0x0a000000. */

sbi_scratch_holder_t g_scratches[MPFS_MAX_NUM_HARTS] \
               __attribute__((section(".l2_scratchpad")));

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_hart_to_scratch
 *
 * Description:
 *   Returns the scratch area start for the given hart.
 *
 * Input Parameters:
 *   hartid (0,1..5)
 *
 * Returned Value:
 *   The scratch area in l2_scratchpad.
 *
 ****************************************************************************/

static unsigned long mpfs_hart_to_scratch(int hartid)
{
  return (unsigned long)(&g_scratches[hartid].scratch);
}

/****************************************************************************
 * Name: mpfs_irqchip_init
 *
 * Description:
 *   Sets the interrupt priorities via the plic_cold_irqchip_init() call.
 *   Also this provides the proper PLIC base address for further irq
 *   property handling such as threshold levels.
 *
 * Input Parameters:
 *   cold_boot   - True, if this is the hart doing the real boot
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int mpfs_irqchip_init(bool cold_boot)
{
  int rc;
  uint32_t hartid = current_hartid();

  if (cold_boot)
    {
      rc = plic_cold_irqchip_init(&mpfs_plic);

      if (rc)
        {
          return rc;
        }
    }

  return plic_warm_irqchip_init(&mpfs_plic, (hartid) ? (2 * hartid - 1) : 0,
                                (hartid) ? (2 * hartid) : -1);
}

/****************************************************************************
 * Name: mpfs_console_putc
 *
 * Description:
 *   Sets the interrupt priorities via the plic_cold_irqchip_init() call.
 *   Also this provides the proper PLIC base address for further irq
 *   property handling such as threshold levels.
 *
 * Input Parameters:
 *   ch   - Character to be printed out
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_console_putc(char ch)
{
#ifdef CONFIG_DEBUG_FEATURES
  if (mpfs_console_ready)
    {
      riscv_lowputc(ch);
    }
#endif
}

/****************************************************************************
 * Name: mpfs_opensbi_console_init
 *
 * Description:
 *   Initializes the console for OpenSBI usage.  OpenSBI expects this
 *   function to be present.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Always zero indicating a success.
 *
 ****************************************************************************/

static int mpfs_opensbi_console_init(void)
{
  mpfs_console_ready = true;

  return 0;
}

/****************************************************************************
 * Name: mpfs_ipi_init
 *
 * Description:
 *   Initializes the IPI for OpenSBI usage.  Also adds the regions into
 *   OpenSBI domains.
 *
 * Input Parameters:
 *    cold_boot   - Indicates the primary boot hart
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int mpfs_ipi_init(bool cold_boot)
{
  int rc;

  if (cold_boot)
    {
      rc = aclint_mswi_cold_init(&mpfs_mswi);
      if (rc)
        {
          return rc;
        }
    }

  return aclint_mswi_warm_init();
}

/****************************************************************************
 * Name: mpfs_timer_init
 *
 * Description:
 *   Initializes the clint timer interface.  Commands such as "csrr a0, time"
 *   (reading the CSR time register) will cause an illegal instruction
 *   exception, because the hardware has no support for it.  That command is
 *   emulated via the CLINT timer in the OpenSBI trap handler.
 *
 * Input Parameters:
 *    cold_boot   - If set, indicates the primary boot hart
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int mpfs_timer_init(bool cold_boot)
{
  int rc;

  if (cold_boot)
    {
      rc = aclint_mtimer_cold_init(&mpfs_mtimer, NULL);
      if (rc)
        {
          return rc;
        }
    }

  return aclint_mtimer_warm_init();
}

/****************************************************************************
 * Name: mpfs_early_init
 *
 * Description:
 *   Initializes the clint timer interface.  Commands such as "csrr a0, time"
 *   (reading the CSR time register) will cause an illegal instruction
 *   exception, because the hardware has no support for it.  That command is
 *   emulated via the CLINT timer in the OpenSBI trap handler.
 *
 * Input Parameters:
 *    cold_boot   - If set, indicates the primary boot hart
 *
 * Returned Value:
 *   Zero (OK) is returned on success.
 *
 ****************************************************************************/

static int mpfs_early_init(bool cold_boot)
{
  uint32_t val;

  /* We expect that e51 has terminated the following irqs with
   * up_disable_irq():
   *   1. MPFS_IRQ_MMC_MAIN
   *   2. MPFS_IRQ_MTIMER
   *
   * U-boot will reuse eMMC and loads the kernel from there. OpenSBI will
   * use CLINT timer.  Upstream u-boot doesn't turn the clocks on itsef.
   */

  if (!cold_boot)
    {
      return 0;
    }

  /* Explicitly reset eMMC */

  val = readl((void *)MPFS_SYSREG_SOFT_RESET_CR);
  writel(val | SYSREG_SOFT_RESET_CR_MMC, (void *)MPFS_SYSREG_SOFT_RESET_CR);
  writel(val & ~SYSREG_SOFT_RESET_CR_MMC, (void *)MPFS_SYSREG_SOFT_RESET_CR);

  /* There are other clocks that need to be enabled for the Linux kernel to
   * run. For now, turn on all the clocks.
   */

  writel(0x0, (void *)MPFS_SYSREG_SOFT_RESET_CR);
  writel(0x7fffffff, (void *)MPFS_SYSREG_SUBBLK_CLOCK_CR);

  return 0;
}

/****************************************************************************
 * Name: mpfs_opensbi_scratch_setup
 *
 * Description:
 *   Initializes the scratch area per hart.  The scratch area is used to save
 *   and restore registers (see mpfs_exception_opensbi), and to send and
 *   reveice messages to other harts via the IPI mechanism.
 *
 * Input Parameters:
 *   hartid       - hart number to be prepared
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void mpfs_opensbi_scratch_setup(uint32_t hartid)
{
  g_scratches[hartid].scratch.options = SBI_SCRATCH_DEBUG_PRINTS;
  g_scratches[hartid].scratch.hartid_to_scratch =
      (unsigned long)mpfs_hart_to_scratch;
  g_scratches[hartid].scratch.platform_addr = (unsigned long)&platform;

  /* Our FW area in l2lim section.  OpenSBI needs to be aware of it in order
   * to protect the area.  However, we set the PMP values already and lock
   * them so that OpenSBI has no chance override then.
   */

  g_scratches[hartid].scratch.fw_start = (unsigned long)_ssbi_ram;
  g_scratches[hartid].scratch.fw_size  = (unsigned long)_esbi_ram -
                                         (unsigned long)_ssbi_ram;

  g_scratches[hartid].scratch.fw_rw_offset =
      (unsigned long)g_scratches[hartid].scratch.fw_size;

  /* fw_rw_offset needs to be an aligned address */

  g_scratches[hartid].scratch.fw_rw_offset += 1024 * 2;
  g_scratches[hartid].scratch.fw_rw_offset &= 0xffffff800;
  g_scratches[hartid].scratch.fw_size =
      g_scratches[hartid].scratch.fw_rw_offset;

  g_scratches[hartid].scratch.fw_heap_offset =
      (unsigned long)g_scratches[hartid].scratch.fw_size;

  /* Heap minimum is 16k.  Otherwise sbi_heap.c fails:
   * hpctrl.hksize = hpctrl.size / HEAP_HOUSEKEEPING_FACTOR;
   * hpctrl.hksize &= ~((unsigned long)HEAP_BASE_ALIGN - 1);
   * eg. 8k:  (0x2000 / 16) & ~(1024 - 1) = 0      (Fail!)
   *     16k: (0x4000 / 16) & ~(1024 - 1) = 0x400  (Ok)
   * hpctrl.hksize gets to be zero making the OpenSBI crash.
   */

  g_scratches[hartid].scratch.fw_heap_size = 1024 * 16;
  g_scratches[hartid].scratch.fw_size      =
      g_scratches[hartid].scratch.fw_heap_offset +
      g_scratches[hartid].scratch.fw_heap_size;
}

/****************************************************************************
 * Name: mpfs_opensbi_vendor_ext_check
 *
 * Description:
 *   Used by the OpenSBI to check if vendor extension is enabled.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   1 on match, zero in case of no match
 *
 ****************************************************************************/

#ifdef CONFIG_MPFS_IHC_SBI
static bool mpfs_opensbi_vendor_ext_check(void)
{
  return true;
}

/****************************************************************************
 * Name: mpfs_opensbi_ecall_handler
 *
 * Description:
 *   Used by the S-mode kernel such as Linux to perform M-mode ecall actions
 *   related to Inter-Hart Communication (IHC).
 *
 * Input Parameters:
 *   funcid         - One of the valid functions
 *   sbi_trap_regs  - SBI trap registers
 *   out_val        - Error code location
 *   out_trap       - Trap registers such as epc, unused
 *
 * Returned Value:
 *   0 always
 *
 ****************************************************************************/

static int mpfs_opensbi_ecall_handler(long funcid,
                                      const struct sbi_trap_regs *regs,
                                      unsigned long *out_val,
                                      struct sbi_trap_info *out_trap)
{
  uint32_t remote_channel = (uint32_t)regs->a0;
  uint32_t *message_ptr   = (uint32_t *)regs->a1;
  int result = 0;

  switch (funcid)
    {
      case SBI_EXT_IHC_CTX_INIT:
      case SBI_EXT_IHC_SEND:
      case SBI_EXT_IHC_RECEIVE:
        result = mpfs_ihc_sbi_ecall_handler(funcid, remote_channel,
                                            message_ptr);
        break;

      default:
        result = SBI_ENOTSUPP;
    }

  *out_val = result;

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_opensbi_setup
 *
 * Description:
 *   Calls the necessary OpenSBI init functions:
 *     - Sets up the PMP registers (to avoid OpenSBI overriding them)
 *     - Sets up the OpenSBI console
 *     - Sets up the mscratch registers
 *     - Sets up the firmware to be run (should be already at .next_addr)
 *     - Calls the sbi_init() that will not return
 *
 * Input Parameters:
 *   a0 - hartid
 *   a1 - next_addr
 *
 * Returned Value:
 *   None - this will never return
 *
 ****************************************************************************/

void __attribute__((noreturn)) mpfs_opensbi_setup(uintptr_t hartid,
                                                  uintptr_t next_addr)
{
  size_t i;

  for (i = 0; i < nitems(mpfs_hart_index2id); i++)
    {
      mpfs_hart_index2id[i] = mpfs_get_use_sbi(i) ? i : -1;
    }

  sbi_console_set_device(&mpfs_console);
  mpfs_opensbi_scratch_setup(hartid);

  csr_write(mscratch, &g_scratches[hartid].scratch);
  g_scratches[hartid].scratch.next_mode = PRV_S;
  g_scratches[hartid].scratch.next_addr = next_addr;
  g_scratches[hartid].scratch.next_arg1 = 0;

  sbi_init(&g_scratches[hartid].scratch);

  /* Will never get here */

  sbi_panic(__func__);
}
