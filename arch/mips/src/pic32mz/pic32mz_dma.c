/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_dma.c
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
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "mips_internal.h"
#include "sched/sched.h"

#include "hardware/pic32mz_dma.h"
#include "pic32mz_dma.h"

#ifdef CONFIG_PIC32MZ_DMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Convert a virtual address to a physical address */

#define PHYS_ADDR(va) ((uint32_t)(va) & 0x1fffffff)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct pic32mz_dmach_s
{
  uint8_t        chan;            /* DMA channel number (0-8) */
  uint8_t        irq;             /* DMA channel IRQ number */
  uint32_t       base;            /* DMA register channel base address */
  bool           inuse;           /* true: The DMA channel is in use */
  dma_callback_t callback;        /* Callback invoked when the DMA completes */
  void          *arg;             /* Argument passed to callback function */
  struct pic32mz_dma_chcfg_s cfg; /* Channel's config */
};

/* This structure describes the state of the DMA controller */

struct pic32mz_dmac_s
{
  /* Protects the channels' table */

  sem_t chsem;

  /* Describes all DMA channels */

  struct pic32mz_dmach_s dmachs[CHIP_NDMACH];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pic32mz_dma_takesem(struct pic32mz_dmac_s *dmac);
static inline void pic32mz_dma_givesem(struct pic32mz_dmac_s *dmac);

static inline uint32_t pic32mz_dma_getreg(struct pic32mz_dmach_s *dmach,
                                          uint8_t offset);
static inline void pic32mz_dma_putreg(struct pic32mz_dmach_s *dmach,
                                      uint8_t offset, uint32_t value);
static inline void pic32mz_dma_modifyreg(struct pic32mz_dmach_s *dmach,
                                         uint8_t offset,
                                         uint32_t clrbits, uint32_t setbits);
static inline uint32_t pic32mz_dma_getglobal(uint8_t offset);
static inline void pic32mz_dma_putglobal(uint8_t offset, uint32_t value);

static inline void pic32mz_dma_enable(struct pic32mz_dmach_s *dmach);
static inline void pic32mz_dma_disable(struct pic32mz_dmach_s *dmach);
static inline void pic32mz_dma_priority(struct pic32mz_dmach_s *dmach,
                                        uint8_t priority);
static inline void pic32mz_dma_srcaddr(struct pic32mz_dmach_s *dmach,
                                       uint32_t addr);
static inline void pic32mz_dma_destaddr(struct pic32mz_dmach_s *dmach,
                                        uint32_t addr);
static inline void pic32mz_dma_srcsize(struct pic32mz_dmach_s *dmach,
                                       uint16_t size);
static inline void pic32mz_dma_destsize(struct pic32mz_dmach_s *dmach,
                                        uint16_t size);
static inline void pic32mz_dma_cellsize(struct pic32mz_dmach_s *dmach,
                                        uint16_t size);
static inline void pic32mz_dma_startirq(struct pic32mz_dmach_s *dmach,
                                        int irq);
static inline void pic32mz_dma_forcestart(struct pic32mz_dmach_s *dmach);
static inline void pic32mz_dma_abortirq(struct pic32mz_dmach_s *dmach,
                                        int irq);
static inline void pic32mz_dma_forceabort(struct pic32mz_dmach_s *dmach);

static inline void pic32mz_dma_intctrl(struct pic32mz_dmach_s *dmach,
                                       uint8_t cfg);
static inline void pic32mz_dma_intclr(struct pic32mz_dmach_s *dmach);
static int pic32mz_dma_interrupt(int irq, void *context, void *arg);

static void pic32mz_dma_mode(struct pic32mz_dmach_s *dmach,
                             uint8_t mode);
static void pic32mz_dma_config(struct pic32mz_dmach_s *dmach,
                               const struct pic32mz_dma_chcfg_s *cfg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array describes the state of each DMA */

static struct pic32mz_dmac_s g_dmac =
{
  .dmachs =
    {
      {
        .chan = 0,
        .irq  = PIC32MZ_IRQ_DMA0,
        .base = PIC32MZ_DMACH0_K1BASE,
      },
      {
        .chan = 1,
        .irq  = PIC32MZ_IRQ_DMA1,
        .base = PIC32MZ_DMACH1_K1BASE,
      },
      {
        .chan = 2,
        .irq  = PIC32MZ_IRQ_DMA2,
        .base = PIC32MZ_DMACH2_K1BASE,
      },
      {
        .chan = 3,
        .irq  = PIC32MZ_IRQ_DMA3,
        .base = PIC32MZ_DMACH3_K1BASE,
      },
      {
        .chan = 4,
        .irq  = PIC32MZ_IRQ_DMA4,
        .base = PIC32MZ_DMACH4_K1BASE,
      },
      {
        .chan = 5,
        .irq  = PIC32MZ_IRQ_DMA5,
        .base = PIC32MZ_DMACH5_K1BASE,
      },
      {
        .chan = 6,
        .irq  = PIC32MZ_IRQ_DMA6,
        .base = PIC32MZ_DMACH6_K1BASE,
      },
      {
        .chan = 7,
        .irq  = PIC32MZ_IRQ_DMA7,
        .base = PIC32MZ_DMACH7_K1BASE,
      },
    }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_dma_takesem
 *
 * Description:
 *   Take the exclusive access, waiting as necessary
 *
 ****************************************************************************/

static int pic32mz_dma_takesem(struct pic32mz_dmac_s *dmac)
{
  return nxsem_wait_uninterruptible(&dmac->chsem);
}

/****************************************************************************
 * Name: pic32mz_dma_givesem
 *
 * Description:
 *   Release the semaphore
 *
 ****************************************************************************/

static inline void pic32mz_dma_givesem(struct pic32mz_dmac_s *dmac)
{
  nxsem_post(&dmac->chsem);
}

/****************************************************************************
 * Name: pic32mz_dma_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t pic32mz_dma_getreg(struct pic32mz_dmach_s *dmach,
                                          uint8_t offset)
{
  return getreg32(dmach->base + offset);
}

/****************************************************************************
 * Name: pic32mz_dma_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void pic32mz_dma_putreg(struct pic32mz_dmach_s *dmach,
                                      uint8_t offset, uint32_t value)
{
  putreg32(value, dmach->base + offset);
}

/****************************************************************************
 * Name: pic32mz_dma_modifyreg
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void pic32mz_dma_modifyreg(struct pic32mz_dmach_s *dmach,
                                         uint8_t offset,
                                         uint32_t clrbits, uint32_t setbits)
{
  modifyreg32(dmach->base + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: pic32mz_dma_getglobal
 *
 * Description:
 *   Get a 32-bit global register value by offset
 *
 ****************************************************************************/

static inline uint32_t pic32mz_dma_getglobal(uint8_t offset)
{
  return getreg32(PIC32MZ_DMA_K1BASE + offset);
}

/****************************************************************************
 * Name: pic32mz_dma_putglobal
 *
 * Description:
 *   Put a 32-bit global register value by offset
 *
 ****************************************************************************/

static inline void pic32mz_dma_putglobal(uint8_t offset, uint32_t value)
{
  putreg32(value, PIC32MZ_DMA_K1BASE + offset);
}

/****************************************************************************
 * Name: pic32mz_dma_enable
 *
 * Description:
 *  Enable the DMA channel.
 *
 ****************************************************************************/

static inline void pic32mz_dma_enable(struct pic32mz_dmach_s *dmach)
{
  pic32mz_dma_putreg(dmach, PIC32MZ_DMA_CONSET_OFFSET, DMACH_CON_CHEN);
}

/****************************************************************************
 * Name: pic32mz_dma_disable
 *
 * Description:
 *  Disable the DMA channel.
 *
 ****************************************************************************/

static inline void pic32mz_dma_disable(struct pic32mz_dmach_s *dmach)
{
  pic32mz_dma_putreg(dmach, PIC32MZ_DMA_CONCLR_OFFSET, DMACH_CON_CHEN);
}

/****************************************************************************
 * Name: pic32mz_dma_priority
 *
 * Description:
 *   Set the channel's priority.
 *
 ****************************************************************************/

static inline void pic32mz_dma_priority(struct pic32mz_dmach_s *dmach,
                                        uint8_t priority)
{
  /* Highest priority is 3. */

  if (priority <= 3)
    {
      pic32mz_dma_modifyreg(dmach, PIC32MZ_DMA_CON_OFFSET,
                            DMACH_CON_CHPRI_MASK,
                            priority << DMACH_CON_CHPRI_SHIFT);
    }
}

/****************************************************************************
 * Name: pic32mz_dma_srcaddr
 *
 * Description:
 *   Set the channel's source address.
 *
 ****************************************************************************/

static inline void pic32mz_dma_srcaddr(struct pic32mz_dmach_s *dmach,
                                       uint32_t addr)
{
  pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_SSA_OFFSET, PHYS_ADDR(addr));
}

/****************************************************************************
 * Name: pic32mz_dma_destaddr
 *
 * Description:
 *   Set the channel's destination address.
 *
 ****************************************************************************/

static inline void pic32mz_dma_destaddr(struct pic32mz_dmach_s *dmach,
                                        uint32_t addr)
{
  pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_DSA_OFFSET, PHYS_ADDR(addr));
}

/****************************************************************************
 * Name: pic32mz_dma_srcsize
 *
 * Description:
 *   Set the channel's source size.
 *
 ****************************************************************************/

static inline void pic32mz_dma_srcsize(struct pic32mz_dmach_s *dmach,
                                       uint16_t size)
{
  pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_SSIZ_OFFSET, size);
}

/****************************************************************************
 * Name: pic32mz_dma_destsize
 *
 * Description:
 *   Set the channel's destination size.
 *
 ****************************************************************************/

static inline void pic32mz_dma_destsize(struct pic32mz_dmach_s *dmach,
                                        uint16_t size)
{
  pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_DSIZ_OFFSET, size);
}

/****************************************************************************
 * Name: pic32mz_dma_cellsize
 *
 * Description:
 *   Set the channel's cell size.
 *
 ****************************************************************************/

static inline void pic32mz_dma_cellsize(struct pic32mz_dmach_s *dmach,
                                        uint16_t size)
{
  pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_CSIZ_OFFSET, size);
}

/****************************************************************************
 * Name: pic32mz_dma_startirq
 *
 * Description:
 *   Set the channel's start irq.
 *
 ****************************************************************************/

static inline void pic32mz_dma_startirq(struct pic32mz_dmach_s *dmach,
                                        int irq)
{
  /* Enable start irq matching. */

  pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_ECONSET_OFFSET, DMACH_ECON_SIRQEN);

  /* Clear the field before setting it. */

  pic32mz_dma_modifyreg(dmach, PIC32MZ_DMACH_ECON_OFFSET,
                        DMACH_ECON_CHSIRQ_MASK,
                        irq << DMACH_ECON_CHSIRQ_SHIFT);
}

/****************************************************************************
 * Name: pic32mz_dma_forcestart
 *
 * Description:
 *   Initiate the transfer by software.
 *
 ****************************************************************************/

static inline void pic32mz_dma_forcestart(struct pic32mz_dmach_s *dmach)
{
  pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_ECONSET_OFFSET, DMACH_ECON_CFORCE);
}

/****************************************************************************
 * Name: pic32mz_dma_abortirq
 *
 * Description:
 *   Set the channel's abort irq.
 *
 ****************************************************************************/

static inline void pic32mz_dma_abortirq(struct pic32mz_dmach_s *dmach,
                                        int irq)
{
  /* Enable abort irq matching. */

  pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_ECONSET_OFFSET, DMACH_ECON_AIRQEN);

  /* Clear the field before setting it. */

  pic32mz_dma_modifyreg(dmach, PIC32MZ_DMACH_ECON_OFFSET,
                        DMACH_ECON_CHAIRQ_MASK,
                        irq << DMACH_ECON_CHAIRQ_SHIFT);
}

/****************************************************************************
 * Name: pic32mz_dma_forceabort
 *
 * Description:
 *   Initiate the transfer by software.
 *
 ****************************************************************************/

static inline void pic32mz_dma_forceabort(struct pic32mz_dmach_s *dmach)
{
  pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_ECONSET_OFFSET, DMACH_ECON_CABORT);
}

/****************************************************************************
 * Name: pic32mz_dma_intclr
 *
 * Description:
 *  Clears the channel's interrupt flags.
 *
 ****************************************************************************/

static inline void pic32mz_dma_intclr(struct pic32mz_dmach_s *dmach)
{
  pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_INTCLR_OFFSET,
                     DMACH_INT_FLAGS_MASK);
}

/****************************************************************************
 * Name: pic32mz_dma_intctrl
 *
 * Description:
 *  Enables the different interrupt's flags.
 *
 ****************************************************************************/

static inline void pic32mz_dma_intctrl(struct pic32mz_dmach_s *dmach,
                                       uint8_t cfg)
{
  /* Clear all interrupts flags */

  pic32mz_dma_intclr(dmach);

  /* Disable previous interrupts */

  pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_INTCLR_OFFSET,
                     DMACH_INT_EN_MASK);

  /* Enable the interrupts requested. */

  pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_INTSET_OFFSET,
                     cfg << DMACH_INT_EN_SHIFT);
}

/****************************************************************************
 * Name: pic32mz_dma_interrupt
 *
 * Description:
 *  DMA interrupt handler.
 *
 ****************************************************************************/

static int pic32mz_dma_interrupt(int irq, void *context, void *arg)
{
  struct pic32mz_dmach_s *dmach;
  uint8_t status;
  int chndx = 0;

  /* Get the channel structure from the irq. */

  chndx = irq - PIC32MZ_IRQ_DMA0;
  dmach = &g_dmac.dmachs[chndx];

  /* Get the interrupt status. */

  status = (uint8_t)(pic32mz_dma_getreg(dmach, PIC32MZ_DMACH_INT_OFFSET) &
                     DMACH_INT_FLAGS_MASK);

  /* Clear the interrupt flags. */

  up_clrpend_irq(dmach->irq);
  pic32mz_dma_intclr(dmach);

  /* Invoke the callback. */

  if (dmach->callback)
    {
      dmach->callback(dmach, status, dmach->arg);
    }

  return OK;
}

/****************************************************************************
 * Name: pic32mz_dma_mode
 *
 * Description:
 *   Set the channel's mode
 *
 ****************************************************************************/

static void pic32mz_dma_mode(struct pic32mz_dmach_s *dmach,
                             uint8_t mode)
{
  if (mode & PIC32MZ_DMA_MODE_BASIC)
    {
      /* Basic mode doesn't need any config */
    }

  if (mode & PIC32MZ_DMA_MODE_AUTOEN)
    {
      pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_CONSET_OFFSET,
                         DMACH_CON_CHAEN);
    }
}

/****************************************************************************
 * Name: pic32mz_dma_config
 *
 * Description:
 *   Config the DMA channel
 *
 ****************************************************************************/

static void pic32mz_dma_config(struct pic32mz_dmach_s *dmach,
                               const struct pic32mz_dma_chcfg_s *cfg)
{
  /* Set the channel's priority */

  pic32mz_dma_priority(dmach, cfg->priority);

  /* Set the channel's start and abort IRQs if they are specified */

  if (cfg->startirq != PIC32MZ_DMA_NOIRQ)
    {
      pic32mz_dma_startirq(dmach, cfg->startirq);
    }
  else
    {
      pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_ECONCLR_OFFSET,
                         DMACH_ECON_SIRQEN);
    }

  if (cfg->abortirq != PIC32MZ_DMA_NOIRQ)
    {
      pic32mz_dma_abortirq(dmach, cfg->abortirq);
    }
  else
    {
      pic32mz_dma_putreg(dmach, PIC32MZ_DMACH_ECONCLR_OFFSET,
                         DMACH_ECON_AIRQEN);
    }

  /* Set the interrupt event(s) */

  pic32mz_dma_intctrl(dmach, cfg->event);

  /* Set the channel's mode */

  pic32mz_dma_mode(dmach, cfg->mode);

  /* Copy the config */

  dmach->cfg = *cfg;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_dma_sample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void pic32mz_dma_sample(DMA_HANDLE handle, struct pic32mz_dmaregs_s *regs)
{
  struct pic32mz_dmach_s *dmach = (struct pic32mz_dmach_s *)handle;
  irqstate_t flags;

  flags = enter_critical_section();

  regs->gbl.con = pic32mz_dma_getglobal(PIC32MZ_DMA_CON_OFFSET);
  regs->gbl.stat = pic32mz_dma_getglobal(PIC32MZ_DMA_STAT_OFFSET);
  regs->gbl.addr = pic32mz_dma_getglobal(PIC32MZ_DMA_ADDR_OFFSET);

  regs->crc.con = pic32mz_dma_getglobal(PIC32MZ_DMA_CRCCON_OFFSET);
  regs->crc.data = pic32mz_dma_getglobal(PIC32MZ_DMA_CRCDATA_OFFSET);
  regs->crc.xor  = pic32mz_dma_getglobal(PIC32MZ_DMA_CRCXOR_OFFSET);

  regs->ch.con = pic32mz_dma_getreg(dmach, PIC32MZ_DMACH_CON_OFFSET);
  regs->ch.econ = pic32mz_dma_getreg(dmach, PIC32MZ_DMACH_ECON_OFFSET);
  regs->ch.intcon = pic32mz_dma_getreg(dmach, PIC32MZ_DMACH_INT_OFFSET);
  regs->ch.ssa = pic32mz_dma_getreg(dmach, PIC32MZ_DMACH_SSA_OFFSET);
  regs->ch.dsa = pic32mz_dma_getreg(dmach, PIC32MZ_DMACH_DSA_OFFSET);
  regs->ch.ssiz  = pic32mz_dma_getreg(dmach, PIC32MZ_DMACH_SSIZ_OFFSET);
  regs->ch.dsiz = pic32mz_dma_getreg(dmach, PIC32MZ_DMACH_DSIZ_OFFSET);
  regs->ch.sptr = pic32mz_dma_getreg(dmach, PIC32MZ_DMACH_SPTR_OFFSET);
  regs->ch.dptr  = pic32mz_dma_getreg(dmach, PIC32MZ_DMACH_DPTR_OFFSET);
  regs->ch.csiz = pic32mz_dma_getreg(dmach, PIC32MZ_DMACH_CSIZ_OFFSET);
  regs->ch.cptr = pic32mz_dma_getreg(dmach, PIC32MZ_DMACH_CPTR_OFFSET);
  regs->ch.dat  = pic32mz_dma_getreg(dmach, PIC32MZ_DMACH_DAT_OFFSET);

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: pic32mz_dma_dump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void pic32mz_dma_dump(DMA_HANDLE handle,
                      const struct pic32mz_dmaregs_s *regs,
                      const char *msg)
{
  struct pic32mz_dmach_s *dmach = (struct pic32mz_dmach_s *)handle;

  dmainfo("DMA Registers: %s\n", msg);
  dmainfo("  DMACON: %08x\n", regs->gbl.con);
  dmainfo("  DMASTAT: %08x\n", regs->gbl.stat);
  dmainfo("  DMAADDR: %08x\n", regs->gbl.addr);

  dmainfo("  DCRCCON: %08x\n", regs->crc.con);
  dmainfo("  DCRCDATA: %08x\n", regs->crc.data);
  dmainfo("  DCRCXOR: %08x\n", regs->crc.xor);

  dmainfo("  DCHCON[%08x]: %08x\n", dmach->base + PIC32MZ_DMACH_CON_OFFSET,
                                    regs->gbl.con);
  dmainfo("  DCHECON[%08x]: %08x\n", dmach->base + PIC32MZ_DMACH_ECON_OFFSET,
                                     regs->gbl.stat);
  dmainfo("  DCHINT[%08x]: %08x\n", dmach->base + PIC32MZ_DMACH_INT_OFFSET,
                                    regs->gbl.addr);
  dmainfo("  DCHSSA[%08x]: %08x\n", dmach->base + PIC32MZ_DMACH_SSA_OFFSET,
                                    regs->crc.con);
  dmainfo("  DCHDSA[%08x]: %08x\n", dmach->base + PIC32MZ_DMACH_DSA_OFFSET,
                                    regs->crc.data);
  dmainfo("  DCHSSIZ[%08x]: %08x\n", dmach->base + PIC32MZ_DMACH_SSIZ_OFFSET,
                                     regs->crc.xor);
  dmainfo("  DCHDSIZ[%08x]: %08x\n", dmach->base + PIC32MZ_DMACH_DSIZ_OFFSET,
                                     regs->gbl.con);
  dmainfo("  DCHSPTR[%08x]: %08x\n", dmach->base + PIC32MZ_DMACH_SPTR_OFFSET,
                                     regs->gbl.stat);
  dmainfo("  DCHDPTR[%08x]: %08x\n", dmach->base + PIC32MZ_DMACH_DPTR_OFFSET,
                                     regs->gbl.addr);
  dmainfo("  DCHCSIZ[%08x]: %08x\n", dmach->base + PIC32MZ_DMACH_CSIZ_OFFSET,
                                     regs->crc.con);
  dmainfo("  DCHCPTR[%08x]: %08x\n", dmach->base + PIC32MZ_DMACH_CPTR_OFFSET,
                                     regs->crc.data);
  dmainfo("  DCHDAT[%08x]: %08x\n", dmach->base + PIC32MZ_DMACH_DAT_OFFSET,
                                    regs->crc.xor);
}
#endif

/****************************************************************************
 * Name: pic32mz_dma_initialize
 *
 * Description:
 *   Initialize the DMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function up_dma_initialize(void)
{
  struct pic32mz_dmach_s *dmach;
  int i;

  /* Initialize each DMA channel. */

  for (i = 0; i < CHIP_NDMACH; i++)
    {
      dmach = &g_dmac.dmachs[i];

      dmach->inuse = false;

      /* Attach DMA interrupt vectors. */

      irq_attach(dmach->irq, pic32mz_dma_interrupt, NULL);

      /* Disable the DMA channel. */

      pic32mz_dma_disable(dmach);

      /* Set channel priority to zero. */

      pic32mz_dma_priority(dmach, 0);

      /* Clear any pending interrupts */

      pic32mz_dma_intclr(dmach);
      up_clrpend_irq(dmach->irq);

      /* Enable the IRQ. */

      up_enable_irq(dmach->irq);
    }

  /* Enable the DMA module. */

  pic32mz_dma_putglobal(PIC32MZ_DMA_CONSET_OFFSET, DMA_CON_ON);

  /* Initialize the semaphore. */

  nxsem_init(&g_dmac.chsem, 0, 1);
}

/****************************************************************************
 * Name: pic32mz_dma_alloc
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   On success, this function returns a non-NULL, void* DMA channel handle.
 *   NULL is returned on any failure.
 *   This function can fail only if no DMA channel is available.
 *
 ****************************************************************************/

DMA_HANDLE pic32mz_dma_alloc(const struct pic32mz_dma_chcfg_s *cfg)
{
  struct pic32mz_dmach_s *dmach = NULL;
  unsigned int chndx;
  int ret;

  /* Search for an available DMA channel */

  ret = pic32mz_dma_takesem(&g_dmac);
  if (ret < 0)
    {
      return NULL;
    }

  for (chndx = 0; chndx < CHIP_NDMACH; chndx++)
    {
      struct pic32mz_dmach_s *candidate = &g_dmac.dmachs[chndx];

      if (!candidate->inuse)
        {
          dmach = candidate;
          dmach->inuse = true;

          DEBUGASSERT(chndx == dmach->chan);

          /* Clear any pending interrupts on the channel */

          pic32mz_dma_intclr(dmach);
          up_clrpend_irq(dmach->irq);

          /* Disable the channel */

          pic32mz_dma_disable(dmach);

          /* Config this channel */

          if (cfg != NULL)
            {
              pic32mz_dma_config(dmach, cfg);
            }

          break;
        }
    }

  pic32mz_dma_givesem(&g_dmac);

  /* Show the result of the allocation */

  if (dmach != NULL)
    {
      dmainfo("CH%d: returning dmach: %p\n", dmach->chan, dmach);
    }
  else
    {
      dmaerr("ERROR: Failed to allocate a DMA channel\n");
    }

  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: pic32mz_dma_free
 *
 * Description:
 *   Release a DMA channel.
 *   NOTE:  The 'handle' used in this argument must NEVER be used again until
 *   pic32mz_dma_alloc() is called again to re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pic32mz_dma_free(DMA_HANDLE handle)
{
  struct pic32mz_dmach_s *dmach = (struct pic32mz_dmach_s *)handle;

  DEBUGASSERT(handle != NULL);

  /* Mark the channel as no longer in use */

  dmach->inuse = false;

  /* Disable the channel */

  pic32mz_dma_disable(dmach);

  /* Clear any pending interrupt */

  pic32mz_dma_intclr(dmach);
  up_clrpend_irq(dmach->irq);
}

/****************************************************************************
 * Name: pic32mz_dma_chcfg
 *
 * Description:
 *   Configure a DMA channel.
 *   This config can be done during alloc, however if reconfig is needed,
 *   this functions should be used.
 *
 ****************************************************************************/

int pic32mz_dma_chcfg(DMA_HANDLE handle,
                      const struct pic32mz_dma_chcfg_s *cfg)
{
  struct pic32mz_dmach_s *dmach = (struct pic32mz_dmach_s *)handle;

  DEBUGASSERT(dmach != NULL);

  pic32mz_dma_config(dmach, cfg);

  return OK;
}

/****************************************************************************
 * Name: pic32mz_dma_xfrsetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int pic32mz_dma_xfrsetup(DMA_HANDLE handle,
                         const struct pic32mz_dma_xfrcfg_s *cfg)
{
  struct pic32mz_dmach_s *dmach = (struct pic32mz_dmach_s *)handle;

  DEBUGASSERT(dmach != NULL);

  /* Set source and destination addresses */

  pic32mz_dma_srcaddr(dmach, cfg->srcaddr);
  pic32mz_dma_destaddr(dmach, cfg->destaddr);

  /* Set transfer size (source, destination and cell) */

  pic32mz_dma_srcsize(dmach,  cfg->srcsize);
  pic32mz_dma_destsize(dmach, cfg->destsize);
  pic32mz_dma_cellsize(dmach, cfg->cellsize);

  return OK;
}

/****************************************************************************
 * Name: pic32mz_dma_start
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int pic32mz_dma_start(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct pic32mz_dmach_s *dmach = (struct pic32mz_dmach_s *)handle;

  DEBUGASSERT(dmach != NULL);

  /* Save the callback info */

  dmach->callback = callback;
  dmach->arg      = arg;

  /* Enable the channel */

  pic32mz_dma_enable(dmach);

  /* If no irq is set to start the channel, force it */

  if (dmach->cfg.startirq == PIC32MZ_DMA_NOIRQ)
    {
      pic32mz_dma_forcestart(dmach);
    }

  return OK;
}

/****************************************************************************
 * Name: pic32mz_dma_stop
 *
 * Description:
 *   Cancel the DMA.
 *   After pic32mz_dma_stop() is called, the DMA channel is reset
 *   and pic32mz_dma_xfrsetup() must be called before pic32mz_dma_start()
 *   can be called again.
 *
 ****************************************************************************/

void pic32mz_dma_stop(DMA_HANDLE handle)
{
  struct pic32mz_dmach_s *dmach = (struct pic32mz_dmach_s *)handle;

  DEBUGASSERT(dmach != NULL);

  /* Abort the transfer and disable the channel */

  pic32mz_dma_forceabort(dmach);
  pic32mz_dma_disable(dmach);

  /* Clear any pending interrupts */

  pic32mz_dma_intclr(dmach);
  up_clrpend_irq(dmach->irq);
}

#endif /* CONFIG_PIC32MZ_DMA */
