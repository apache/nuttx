/***************************************************************************
 * arch/arm64/src/common/arm64_gicv3.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <sched/sched.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_gic.h"
#include "arm64_fatal.h"

#if CONFIG_ARM_GIC_VERSION == 3 || CONFIG_ARM_GIC_VERSION == 4

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#define GICR_TYPER_NR_PPIS(r)                   \
  ({                                            \
    unsigned int __ppinum = ((r) >> 27) & 0x1f; \
    unsigned int __nr_ppis = 16;                \
    if (__ppinum == 1 || __ppinum == 2)         \
        {  __nr_ppis +=  __ppinum * 32;  }      \
    __nr_ppis;                                  \
  })

/* selects redistributor SGI_base for current core for PPI and SGI
 * selects distributor base for SPI
 * The macro translates to distributor base for GICv2 and GICv1
 */
#define GET_DIST_BASE(intid)  ((intid < GIC_SPI_INT_BASE) ?          \
                               (gic_get_rdist() + GICR_SGI_BASE_OFF) \
                               : GIC_DIST_BASE)

#define IGROUPR_VAL  0xFFFFFFFFU

/***************************************************************************
 * Private Data
 ***************************************************************************/

/* Redistributor base addresses for each core */

static unsigned long gic_rdists[CONFIG_SMP_NCPUS];

/***************************************************************************
 * Private Functions
 ***************************************************************************/

static inline void sys_set_bit(unsigned long addr, unsigned int bit)
{
  uint32_t temp;

  temp = getreg32(addr);
  temp = temp | (BIT(bit));
  putreg32(temp, addr);
}

static inline void sys_clear_bit(unsigned long addr, unsigned int bit)
{
  uint32_t temp;

  temp = getreg32(addr);
  temp = temp & ~(BIT(bit));
  putreg32(temp, addr);
}

static inline int sys_test_bit(unsigned long addr, unsigned int bit)
{
  uint32_t temp;

  temp = getreg32(addr);
  return (temp & BIT(bit));
}

static inline unsigned long gic_get_rdist(void)
{
  return gic_rdists[this_cpu()];
}

static inline uint32_t read_gicd_wait_rwp(void)
{
  uint32_t value;

  value = getreg32(GICD_CTLR);

  while (value & BIT(GICD_CTLR_RWP))
    {
      value = getreg32(GICD_CTLR);
    }

  return value;
}

/* Wait for register write pending
 * TODO: add timed wait
 */

static int gic_wait_rwp(uint32_t intid)
{
  uint32_t      rwp_mask;
  unsigned long base;

  if (intid < GIC_SPI_INT_BASE)
    {
      base        = (gic_get_rdist() + GICR_CTLR);
      rwp_mask    = BIT(GICR_CTLR_RWP);
    }
  else
    {
      base        = GICD_CTLR;
      rwp_mask    = BIT(GICD_CTLR_RWP);
    }

  while (getreg32(base) & rwp_mask)
    {
    }

  return 0;
}

void arm64_gic_irq_set_priority(unsigned int intid, unsigned int prio,
                                uint32_t flags)
{
  uint32_t      mask    = BIT(intid & (GIC_NUM_INTR_PER_REG - 1));
  uint32_t      idx     = intid / GIC_NUM_INTR_PER_REG;
  uint32_t      shift;
  uint32_t      val;
  unsigned long base = GET_DIST_BASE(intid);

  /* Disable the interrupt */

  putreg32(mask, ICENABLER(base, idx));
  gic_wait_rwp(intid);

  /* PRIORITYR registers provide byte access */

  putreg8(prio & GIC_PRI_MASK, IPRIORITYR(base, intid));

  /* Interrupt type config */

  if (!GIC_IS_SGI(intid))
    {
      idx     = intid / GIC_NUM_CFG_PER_REG;
      shift   = (intid & (GIC_NUM_CFG_PER_REG - 1)) * 2;

      val = getreg32(ICFGR(base, idx));
      val &= ~(GICD_ICFGR_MASK << shift);
      if (flags & IRQ_TYPE_EDGE)
        {
          val |= (GICD_ICFGR_TYPE << shift);
        }

      putreg32(val, ICFGR(base, idx));
    }
}

void arm64_gic_irq_enable(unsigned int intid)
{
  uint32_t  mask    = BIT(intid & (GIC_NUM_INTR_PER_REG - 1));
  uint32_t  idx     = intid / GIC_NUM_INTR_PER_REG;

  putreg32(mask, ISENABLER(GET_DIST_BASE(intid), idx));

  /* Affinity routing is enabled for Non-secure state (GICD_CTLR.ARE_NS
   * is set to '1' when GIC distributor is initialized) ,so need to set
   * SPI's affinity, now set it to be the PE on which it is enabled.
   */

  if (GIC_IS_SPI(intid))
    {
      putreg64(MPIDR_TO_CORE(GET_MPIDR()),
              IROUTER(GET_DIST_BASE(intid), intid));
    }
}

void arm64_gic_irq_disable(unsigned int intid)
{
  uint32_t  mask    = BIT(intid & (GIC_NUM_INTR_PER_REG - 1));
  uint32_t  idx     = intid / GIC_NUM_INTR_PER_REG;

  putreg32(mask, ICENABLER(GET_DIST_BASE(intid), idx));

  /* poll to ensure write is complete */

  gic_wait_rwp(intid);
}

bool arm64_gic_irq_is_enabled(unsigned int intid)
{
  uint32_t  mask    = BIT(intid & (GIC_NUM_INTR_PER_REG - 1));
  uint32_t  idx     = intid / GIC_NUM_INTR_PER_REG;
  uint32_t  val;

  val = getreg32(ISENABLER(GET_DIST_BASE(intid), idx));

  return (val & mask) != 0;
}

unsigned int arm64_gic_get_active(void)
{
  int intid;

  /* (Pending -> Active / AP) or (AP -> AP) */

  intid = read_sysreg(ICC_IAR1_EL1);

  return intid;
}

void arm64_gic_eoi(unsigned int intid)
{
  /* Interrupt request deassertion from peripheral to GIC happens
   * by clearing interrupt condition by a write to the peripheral
   * register. It is desired that the write transfer is complete
   * before the core tries to change GIC state from 'AP/Active' to
   * a new state on seeing 'EOI write'.
   * Since ICC interface writes are not ordered against Device
   * memory writes, a barrier is required to ensure the ordering.
   * The dsb will also ensure *completion* of previous writes with
   * DEVICE nGnRnE attribute.
   */

  ARM64_DSB();

  /* (AP -> Pending) Or (Active -> Inactive) or (AP to AP) nested case */

  write_sysreg(intid, ICC_EOIR1_EL1);
}

int arm64_gic_raise_sgi(unsigned int sgi_id, uint64_t target_aff,
                        uint16_t target_list)
{
  uint32_t  aff3;
  uint32_t  aff2;
  uint32_t  aff1;
  uint64_t  sgi_val;

  assert(GIC_IS_SGI(sgi_id));

  /* Extract affinity fields from target */

  aff1  = MPIDR_AFFLVL(target_aff, 1);
  aff2  = MPIDR_AFFLVL(target_aff, 2);
  aff3  = MPIDR_AFFLVL(target_aff, 3);

  sgi_val = GICV3_SGIR_VALUE(aff3, aff2, aff1, sgi_id, SGIR_IRM_TO_AFF,
                             target_list);

  ARM64_DSB();
  write_sysreg(sgi_val, ICC_SGI1R);
  ARM64_ISB();

  return 0;
}

/* Wake up GIC redistributor.
 * clear ProcessorSleep and wait till ChildAsleep is cleared.
 * ProcessSleep to be cleared only when ChildAsleep is set
 * Check if redistributor is not powered already.
 */

static void gicv3_rdist_enable(unsigned long rdist)
{
  if (!(getreg32(rdist + GICR_WAKER) & BIT(GICR_WAKER_CA)))
    {
      return;
    }

  sys_clear_bit(rdist + GICR_WAKER, GICR_WAKER_PS);

  while (getreg32(rdist + GICR_WAKER) & BIT(GICR_WAKER_CA))
    {
    }
}

/* Initialize the cpu interface. This should be called by each core. */

static void gicv3_cpuif_init(void)
{
  uint32_t      icc_sre;
  uint32_t      intid;

  unsigned long base = gic_get_rdist() + GICR_SGI_BASE_OFF;

  /* Disable all sgi ppi */

  putreg32(BIT64_MASK(GIC_NUM_INTR_PER_REG), ICENABLER(base, 0));

  /* Any sgi/ppi intid ie. 0-31 will select GICR_CTRL */

  gic_wait_rwp(0);

  /* Clear pending */

  putreg32(BIT64_MASK(GIC_NUM_INTR_PER_REG), ICPENDR(base, 0));

  /* Configure all SGIs/PPIs as G1S or G1NS depending on Zephyr
   * is run in EL1S or EL1NS respectively.
   * All interrupts will be delivered as irq
   */

  putreg32(IGROUPR_VAL, IGROUPR(base, 0));
  putreg32(BIT64_MASK(GIC_NUM_INTR_PER_REG), IGROUPMODR(base, 0));

  /* Configure default priorities for SGI 0:15 and PPI 0:15. */

  for (intid = 0; intid < GIC_SPI_INT_BASE;
       intid += GIC_NUM_PRI_PER_REG)
    {
      putreg32(GIC_INT_DEF_PRI_X4, IPRIORITYR(base, intid));
    }

  /* Configure PPIs as level triggered */

  putreg32(0, ICFGR(base, 1));

  /* Check if system interface can be enabled.
   * 'icc_sre_el3' needs to be configured at 'EL3'
   * to allow access to 'icc_sre_el1' at 'EL1'
   * eg: z_arch_el3_plat_init can be used by platform.
   */

  icc_sre = read_sysreg(ICC_SRE_EL1);

  if (!(icc_sre & ICC_SRE_ELX_SRE_BIT))
    {
      icc_sre =
        (icc_sre | ICC_SRE_ELX_SRE_BIT | ICC_SRE_ELX_DIB_BIT |
         ICC_SRE_ELX_DFB_BIT);
      write_sysreg(icc_sre, ICC_SRE_EL1);
      icc_sre = read_sysreg(ICC_SRE_EL1);

      assert(icc_sre & ICC_SRE_ELX_SRE_BIT);
    }

  write_sysreg(GIC_IDLE_PRIO, ICC_PMR_EL1);

  /* Allow group1 interrupts */

  write_sysreg(1, ICC_IGRPEN1_EL1);
}

static void gicv3_dist_init(void)
{
  unsigned int  num_ints;
  unsigned int  intid;
  unsigned int  idx;
  unsigned long base = GIC_DIST_BASE;

  num_ints  = getreg32(GICD_TYPER);
  num_ints  &= GICD_TYPER_ITLINESNUM_MASK;
  num_ints  = (num_ints + 1) << 5;

  /* Disable the distributor */

  putreg32(0, GICD_CTLR);
  gic_wait_rwp(GIC_SPI_INT_BASE);

#ifdef CONFIG_ARCH_SINGLE_SECURITY_STATE

  /* Before configuration, we need to check whether
   * the GIC single security state mode is supported.
   * Make sure GICD_CTRL_NS is 1.
   */

  sys_set_bit(GICD_CTLR, GICD_CTRL_DS);
  if (!sys_test_bit(GICD_CTLR, GICD_CTRL_DS))
    {
      sinfo("Current GIC does not support single security state\n");
      PANIC();
    }
#endif

  /* Default configuration of all SPIs */

  for (intid = GIC_SPI_INT_BASE; intid < num_ints;
       intid += GIC_NUM_INTR_PER_REG)
    {
      idx = intid / GIC_NUM_INTR_PER_REG;

      /* Disable interrupt */

      putreg32(BIT64_MASK(GIC_NUM_INTR_PER_REG),
               ICENABLER(base, idx));

      /* Clear pending */

      putreg32(BIT64_MASK(GIC_NUM_INTR_PER_REG),
               ICPENDR(base, idx));
      putreg32(IGROUPR_VAL, IGROUPR(base, idx));
      putreg32(BIT64_MASK(GIC_NUM_INTR_PER_REG),
               IGROUPMODR(base, idx));
    }

  /* wait for rwp on GICD */

  gic_wait_rwp(GIC_SPI_INT_BASE);

  /* Configure default priorities for all SPIs. */

  for (intid = GIC_SPI_INT_BASE; intid < num_ints;
       intid += GIC_NUM_PRI_PER_REG)
    {
      putreg32(GIC_INT_DEF_PRI_X4, IPRIORITYR(base, intid));
    }

  /* Configure all SPIs as active low, level triggered by default */

  for (intid = GIC_SPI_INT_BASE; intid < num_ints;
       intid += GIC_NUM_CFG_PER_REG)
    {
      idx = intid / GIC_NUM_CFG_PER_REG;
      putreg32(0, ICFGR(base, idx));
    }

  /* TODO: Some arrch64 Cortex-A core maybe without security state
   * it has different GIC configure with standard arrch64 A or R core
   */

#ifdef CONFIG_ARCH_SINGLE_SECURITY_STATE
  /* For GIC single security state(ARMv8-R), the config means
   * the GIC is under single security state which has
   * only two groups:
   *  group 0 and group 1.
   * Then set GICD_CTLR_ARE and GICD_CTLR_ENABLE_G1 to enable Group 1
   * interrupt.
   * Since the GICD_CTLR_ARE and GICD_CTRL_ARE_S share BIT(4), and
   * similarly the GICD_CTLR_ENABLE_G1 and GICD_CTLR_ENABLE_G1NS share
   * BIT(1), we can reuse them.
   */

  putreg32(BIT(GICD_CTRL_ARE_S) | BIT(GICD_CTLR_ENABLE_G1NS),
                 GICD_CTLR);

#else
  /* Enable distributor with ARE */

  putreg32(BIT(GICD_CTRL_ARE_NS) | BIT(GICD_CTLR_ENABLE_G1NS),
           GICD_CTLR);
#endif
}

void up_enable_irq(int irq)
{
  /* TODO: add common interface to set IRQ type for NuttX */

  arm64_gic_irq_set_priority(irq, IRQ_DEFAULT_PRIORITY, IRQ_TYPE_LEVEL);
  arm64_gic_irq_enable(irq);
}

void up_disable_irq(int irq)
{
  arm64_gic_irq_disable(irq);
}

/***************************************************************************
 * Name: arm64_decodeirq
 *
 * Description:
 *   This function is called from the IRQ vector handler in arm64_vectors.S.
 *   At this point, the interrupt has been taken and the registers have
 *   been saved on the stack.  This function simply needs to determine the
 *   the irq number of the interrupt and then to call arm_doirq to dispatch
 *   the interrupt.
 *
 *  Input Parameters:
 *   regs - A pointer to the register save area on the stack.
 ***************************************************************************/

uint64_t * arm64_decodeirq(uint64_t * regs)
{
  int irq;

  /* Read the interrupt acknowledge register and get the interrupt ID */

  irq = arm64_gic_get_active();

  /* Ignore spurions IRQs.  ICCIAR will report 1023 if there is no pending
   * interrupt.
   */

  DEBUGASSERT(irq < NR_IRQS || irq == 1023);
  if (irq < NR_IRQS)
    {
      /* Dispatch the interrupt */

      regs = arm64_doirq(irq, regs);
    }

  /* Write to the end-of-interrupt register */

  arm64_gic_eoi(irq);

  return regs;
}

static int gic_validate_dist_version(void)
{
  uint32_t  typer;
  bool      has_rss;
  uint32_t  reg = getreg32(GICD_PIDR2) & GICD_PIDR2_ARCH_MASK;
  int       spis;
  int       espis;

  if (reg == GICD_PIDR2_ARCH_GICV3)
    {
      sinfo("GICv3 version detect\n");
    }
  else if (reg == GICD_PIDR2_ARCH_GICV4)
    {
      sinfo("GICv4 version detect\n");
    }
  else
    {
      sinfo("No GIC version detect\n");
      return -ENODEV;
    }

  /* Find out how many interrupts are supported. */

  typer = getreg32(GICD_TYPER);
  spis  = MIN(GICD_TYPER_SPIS(typer), 1020U) - 32;
  espis = GICD_TYPER_ESPIS(typer);

  sinfo("GICD_TYPER = 0x%x\n", typer);
  sinfo("%d SPIs implemented\n", spis);
  sinfo("%d Extended SPIs implemented\n", espis);

  has_rss = !!(typer & GICD_TYPER_RSS);
  sinfo("Distributor has %sRange Selector support\n", has_rss ? "" : "no ");

  if (typer & GICD_TYPER_MBIS)
    {
      sinfo("MBIs is present, But No support\n");
    }

  return 0;
}

static int gic_validate_redist_version(void)
{
  uint64_t      typer;
  unsigned int  ppi_nr;
  bool          has_vlpis       = true;
  bool          has_direct_lpi  = true;
  uint32_t      reg;
  unsigned long redist_base = gic_get_rdist();

  ppi_nr    = (~0U);
  reg       = getreg32(redist_base +
             GICR_PIDR2) & GICR_PIDR2_ARCH_MASK;
  if (reg != GICR_PIDR2_ARCH_GICV3 &&
             reg != GICR_PIDR2_ARCH_GICV4)
    {
      sinfo("No redistributor present 0x%lx\n", redist_base);
      return -ENODEV;
    }

  typer             = getreg64(redist_base + GICR_TYPER);
  has_vlpis         &= !!(typer & GICR_TYPER_VLPIS);
  has_direct_lpi    &= !!(typer & GICR_TYPER_DIRECTLPIS);
  ppi_nr            = MIN(GICR_TYPER_NR_PPIS(typer), ppi_nr);

  if (ppi_nr == (~0U))
    {
      ppi_nr = 0;
    }

  sinfo("GICR_TYPER = 0x%"PRIx64"\n", typer);
  sinfo("%d PPIs implemented\n", ppi_nr);
  sinfo("%sVLPI support, %sdirect LPI support\n", !has_vlpis ? "no " : "",
        !has_direct_lpi ? "no " : "");

  return 0;
}

static void arm64_gic_init(void)
{
  uint8_t   cpu;
  int       err;

  cpu               = this_cpu();
  gic_rdists[cpu]   = CONFIG_GICR_BASE +
                     MPIDR_TO_CORE(GET_MPIDR()) * 0x20000;

  err = gic_validate_redist_version();
  if (err)
    {
      sinfo("no redistributor detected, giving up ret=%d\n", err);
      return;
    }

  gicv3_rdist_enable(gic_get_rdist());

  gicv3_cpuif_init();
}

int arm64_gic_initialize(void)
{
  int err;

  err = gic_validate_dist_version();
  if (err)
    {
      sinfo("no distributor detected, giving up ret=%d\n", err);
      return err;
    }

  gicv3_dist_init();

  arm64_gic_init();

  return 0;
}

#ifdef CONFIG_SMP
void arm64_gic_secondary_init(void)
{
  arm64_gic_init();
}

#endif

#endif /* CONFIG_ARM_GIC_VERSION == 3 || CONFIG_ARM_GIC_VERSION == 4 */
