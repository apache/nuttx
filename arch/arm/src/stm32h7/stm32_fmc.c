/****************************************************************************
 * arch/arm/src/stm32h7/stm32_fmc.c
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

#if defined(CONFIG_STM32H7_FMC)

#include "stm32.h"

#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/****************************************************************************
 * To use FMC, you must first enable it in configuration:
 *
 * CONFIG_STM32H7_FMC=y
 *
 * FMC is statically configured at startup. Its configuration is adjusted
 * using BOARD_XXX macros described below, which should be declared
 * in your board.h file.
 *
 * The BOARD_FMC_CLK macro can be defined to select the clock source for FMC.
 * This can be one of RCC_D1CCIPR_FMCSEL_xxx macros, and defaults to
 * RCC_D1CCIPR_FMCSEL_PLL2 (which means PLL2R).
 *
 *
 * To correctly initialize GPIO pins the macro BOARD_FMC_GPIO_CONFIGS should
 * define a list of 32-bit GPIO configuration bitmasks. Example:
 *
 * #define BOARD_FMC_GPIO_CONFIGS       GPIO_FMC_A0,GPIO_FMC_A1,\
 *         GPIO_FMC_A2,...,GPIO_FMC_D0,...,GPIO_FMC_NBL0,...
 *
 *
 * For SRAM/NOR-Flash memories FMC defines 4 x 64Mb sub-banks at addresses
 * 0x60000000, 0x64000000, 0x68000000, 0x6C000000. The following macros
 * can be defined in board.h to initialize FMC for use with SRAM/NOR-Flash:
 *
 * *** CURRENTLY SRAM/NOR-FLASH SUPPORT IS NOT IMPLEMENTED ***
 *
 * BOARD_FMC_BCR[1..4] - Initial value for control registers
 *      for subbanks 1-4.
 * BOARD_FMC_BTR[1..4] - Initial value for SRAM/NOR-Flash timing registers
 *      for subbanks 1-4.
 * BOARD_FMC_BWTR[1..4] - Initial value for SRAM/NOR-Flash write timing
 *      registers for subbanks 1-4.
 *
 *
 * For NAND flash memories FMC reserves 64Mb at 0x80000000. Define the
 * following macros in your board.h file to initialize FMC for this type
 * of memory:
 *
 * *** CURRENTLY NAND FLASH SUPPORT IS NOT IMPLEMENTED ***
 *
 * BOARD_FMC_PCR - Initial value for NAND flash control register.
 * BOARD_FMC_PMEM - Initial value for NAND flash common memory space timing
 *      register.
 * BOARD_FMC_PATT - Initial value for NAND flash attribute memory space
 *      timing register.
 *
 *
 * For SDRAM memory FMC reserves 2 x 256Mb address ranges at 0xC0000000
 * and 0xD0000000. Define the following macros to initialize FMC to work
 * with SDRAM:
 *
 * BOARD_FMC_SDCR[1..2] - Initial value for SDRAM control registers for SDRAM
 *      bank 1-2. Note that some bits in SDCR1 influence both SDRAM banks and
 *      are unused in SDCR2!
 * BOARD_FMC_SDTR[1..2] - Initial value for SDRAM timing registers for SDRAM
 *      bank 1-2. Note that some bits in SDTR1 influence both SDRAM banks and
 *      are unused in SDTR2!
 * BOARD_FMC_SDRAM_REFR_PERIOD - The SDRAM refresh rate period in FMC clocks,
 *      OR
 * BOARD_FMC_SDRAM_REFR_CYCLES and BOARD_FMC_SDRAM_REFR_PERIOD can be used to
 *      automatically compute refresh counter using data from SDRAM datasheet
 *      (cycles usually is 4096, 8192 and such, and typical period is 64 ms)
 *      and knowing FMC clock frequency.
 * BOARD_FMC_SDRAM_AUTOREFRESH may be defined to a number between 1 and 16 to
 *      issue given number of SDRAM auto-refresh cycles before using it. This
 *      defaults to 3.
 *
 *
 * Special notes:
 *      - FMC bank remapping (FMC_BCR_BMAP*) is not currently supported.
 *
 *
 * Here's a working example of a configured IS42S16320D SDRAM on a particular
 * board:
 *
 *#define BOARD_SDRAM1_SIZE               (64*1024*1024)
 *
 *#define BOARD_FMC_SDCR1 \
 *      (FMC_SDCR_COLBITS_10 | FMC_SDCR_ROWBITS_13 | FMC_SDCR_WIDTH_16 |\
 *       FMC_SDCR_BANKS_4 | FMC_SDCR_CASLAT_2 | FMC_SDCR_SDCLK_2X |\
 *       FMC_SDCR_BURST_READ | FMC_SDCR_RPIPE_1)
 *#define BOARD_FMC_SDTR1 \
 *      (FMC_SDTR_TMRD(2) | FMC_SDTR_TXSR(7) | FMC_SDTR_TRAS(4) | \
 *       FMC_SDTR_TRC(7) | FMC_SDTR_TWR(4) | FMC_SDTR_TRP(2) | \
 *       FMC_SDTR_TRCD(2))
 *#define BOARD_FMC_SDRAM_REFR_CYCLES     8192
 *#define BOARD_FMC_SDRAM_REFR_PERIOD     64
 *#define BOARD_FMC_SDRAM_MODE \
 *      FMC_SDCMR_MRD_BURST_LENGTH_8 | \
 *      FMC_SDCMR_MRD_BURST_TYPE_SEQUENTIAL | \
 *      FMC_SDCMR_MRD_CAS_LATENCY_2
 *
 *#define BOARD_FMC_GPIO_CONFIGS \
 *      GPIO_FMC_A0,GPIO_FMC_A1,GPIO_FMC_A2,GPIO_FMC_A3,GPIO_FMC_A4,\
 *      GPIO_FMC_A5,GPIO_FMC_A6,GPIO_FMC_A7,GPIO_FMC_A8,GPIO_FMC_A9,\
 *      GPIO_FMC_A10,GPIO_FMC_A11,GPIO_FMC_A12,\
 *      GPIO_FMC_D0,GPIO_FMC_D1,GPIO_FMC_D2,GPIO_FMC_D3,GPIO_FMC_D4,\
 *      GPIO_FMC_D5,GPIO_FMC_D6,GPIO_FMC_D7,GPIO_FMC_D8,GPIO_FMC_D9,\
 *      GPIO_FMC_D10,GPIO_FMC_D11,GPIO_FMC_D12,GPIO_FMC_D13,\
 *      GPIO_FMC_D14,GPIO_FMC_D15,\
 *      GPIO_FMC_NBL0,GPIO_FMC_NBL1,GPIO_FMC_BA0,GPIO_FMC_BA1,\
 *      GPIO_FMC_SDNWE_2,GPIO_FMC_SDNCAS,GPIO_FMC_SDNRAS,\
 *      GPIO_FMC_SDNE0_1,GPIO_FMC_SDCKE0_1,GPIO_FMC_SDCLK
 *
 ****************************************************************************/

#ifndef BOARD_FMC_CLK
/* Clock FMC from PLL2R by default */

#  define BOARD_FMC_CLK         RCC_D1CCIPR_FMCSEL_PLL2
#endif

/* A couple of macros to know if user uses SDRAM1 and/or SDRAM2 */

#if defined BOARD_FMC_SDCR1 && ((BOARD_FMC_SDCR1 & FMC_SDCR_CASLAT_MASK) != 0)
#  define HAVE_FMC_SDRAM1        1
#endif
#if defined BOARD_FMC_SDCR2 && ((BOARD_FMC_SDCR2 & FMC_SDCR_CASLAT_MASK) != 0)
#  define HAVE_FMC_SDRAM2        1
#endif

#if defined HAVE_FMC_SDRAM1 || defined HAVE_FMC_SDRAM2
#  define HAVE_FMC_SDRAM         1
#endif

/* Number of auto-refresh cycles to issue at SDRAM initialization */

#ifndef BOARD_FMC_SDRAM_AUTOREFRESH
#  define BOARD_FMC_SDRAM_AUTOREFRESH   3
#endif

/* Find out the clock frequency for FMC */

#if BOARD_FMC_CLK == RCC_D1CCIPR_FMCSEL_HCLK
#  define FMC_CLK_FREQUENCY     STM32_HCLK_FREQUENCY
#elif BOARD_FMC_CLK == RCC_D1CCIPR_FMCSEL_PLL1
#  define FMC_CLK_FREQUENCY     STM32_PLL1Q_FREQUENCY
#elif BOARD_FMC_CLK == RCC_D1CCIPR_FMCSEL_PLL2
#  define FMC_CLK_FREQUENCY     STM32_PLL2R_FREQUENCY
#elif BOARD_FMC_CLK == RCC_D1CCIPR_FMCSEL_PER
#  define FMC_CLK_FREQUENCY     STM32_PER_FREQUENCY
#else
#  error "BOARD_FMC_CLK has unknown value!"
#endif

/* Compute the refresh rate in clocks */

#ifndef BOARD_FMC_SDRAM_REFR_PERIOD

#  if !defined(BOARD_FMC_SDRAM_REFR_CYCLES) || !defined(BOARD_FMC_SDRAM_REFR_PERIOD)
#    error "Both BOARD_FMC_SDRAM_REFR_CYCLES and BOARD_FMC_SDRAM_REFR_PERIOD have to be defined to compute BOARD_FMC_SDRAM_REFR_PERIOD!"
#  else
     /* Take care not to overflow on 32-bit arithmetic */

#    define BOARD_FMC_SDRAM_REFR_PERIOD        ((uint32_t)((uint64_t)BOARD_FMC_SDRAM_REFR_PERIOD * FMC_CLK_FREQUENCY / BOARD_FMC_SDRAM_REFR_CYCLES - 20))
#  endif

#endif /* BOARD_FMC_SDRAM_REFR_PERIOD */

/* The bits in FMC_SDCR we will alter at initialization */

#define FMC_SDCR_MASK           (FMC_SDCR_COLBITS_MASK | FMC_SDCR_ROWBITS_MASK | \
    FMC_SDCR_WIDTH_MASK | FMC_SDCR_BANKS_MASK | FMC_SDCR_CASLAT_MASK | FMC_SDCR_WP | \
    FMC_SDCR_SDCLK_MASK | FMC_SDCR_BURST_READ | FMC_SDCR_RPIPE_MASK)

/* The bits in FMC_SDTR we will alter at initialization */

#define FMC_SDTR_MASK           (FMC_SDTR_TMRD_MASK | FMC_SDTR_TXSR_MASK | \
    FMC_SDTR_TRAS_MASK | FMC_SDTR_TRC_MASK | FMC_SDTR_TWR_MASK | FMC_SDTR_TRP_MASK | \
    FMC_SDTR_TRCD_MASK)

/* The timeout while waiting for SDRAM controller to initialize, in us */

#define SDRAM_INIT_TIMEOUT      (1000)

/****************************************************************************
 * Private data
 ****************************************************************************/

static uint32_t fmc_gpios[] =
{
    BOARD_FMC_GPIO_CONFIGS
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void stm32_fmc_sdram_init(void);
static int stm32_fmc_sdram_wait(unsigned timeout);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_fmc_init
 *
 * Description:
 *   Initialize the FMC peripheral. Because FMC initialization is highly
 *   dependent on the used parts, definition of the initial values for FMC
 *   registers is mostly left to board designer.
 *
 *   Typically called from arm_addregion().
 *
 ****************************************************************************/

void stm32_fmc_init(void)
{
  uint32_t regval;

  /* Reset the FMC on the AHB3 bus */

  regval  = getreg32(STM32_RCC_AHB3RSTR);
  regval |= RCC_AHB3RSTR_FMCRST;
  putreg32(regval, STM32_RCC_AHB3RSTR);

  /* Leave reset state */

  regval &= ~RCC_AHB3RSTR_FMCRST;
  putreg32(regval, STM32_RCC_AHB3RSTR);

  /* Set FMC clocking */

  modreg32 (BOARD_FMC_CLK, RCC_D1CCIPR_FMCSEL_MASK, STM32_RCC_D1CCIPR);

  /* Set up FMC GPIOs */

  for (regval = 0; regval < ARRAY_SIZE(fmc_gpios); regval++)
    stm32_configgpio(fmc_gpios[regval]);

  /* Set up FMC registers */

#ifdef BOARD_FMC_SDCR1
  modreg32(BOARD_FMC_SDCR1, FMC_SDCR_MASK, STM32_FMC_SDCR1);
#endif
#ifdef BOARD_FMC_SDCR2
  modreg32(BOARD_FMC_SDCR2, FMC_SDCR_MASK, STM32_FMC_SDCR2);
#endif

#ifdef BOARD_FMC_SDTR1
  modreg32(BOARD_FMC_SDTR1, FMC_SDTR_MASK, STM32_FMC_SDTR1);
#endif
#ifdef BOARD_FMC_SDTR2
  modreg32(BOARD_FMC_SDTR2, FMC_SDTR_MASK, STM32_FMC_SDTR2);
#endif

  /* Enable the FMC peripheral */

  modreg32(FMC_BCR_FMCEN, FMC_BCR_FMCEN, STM32_FMC_BCR1);

  /* Initialize the SDRAM chips themselves */

#ifdef HAVE_FMC_SDRAM
  stm32_fmc_sdram_init();
#endif

  /* Set up SDRAM refresh timings */

#ifdef BOARD_FMC_SDRAM_REFR_PERIOD
  /* ... The programmed COUNT value must not be equal to the sum of the
   * following timings: TWR+TRP+TRC+TRCD+4 memory clock cycles
   */

#ifdef BOARD_FMC_SDTR1
  DEBUGASSERT (BOARD_FMC_SDRAM_REFR_PERIOD !=
    (4 +
     (1 + ((BOARD_FMC_SDTR1 & FMC_SDTR_TWR_MASK) >> FMC_SDTR_TWR_SHIFT)) +
     (1 + ((BOARD_FMC_SDTR1 & FMC_SDTR_TRP_MASK) >> FMC_SDTR_TRP_SHIFT)) +
     (1 + ((BOARD_FMC_SDTR1 & FMC_SDTR_TRC_MASK) >> FMC_SDTR_TRC_SHIFT)) +
     (1 + ((BOARD_FMC_SDTR1 & FMC_SDTR_TRCD_MASK) >> FMC_SDTR_TRCD_SHIFT))));
#endif
#ifdef BOARD_FMC_SDTR2
  DEBUGASSERT (BOARD_FMC_SDRAM_REFR_PERIOD !=
    (4 +
     (1 + ((BOARD_FMC_SDTR2 & FMC_SDTR_TWR_MASK) >> FMC_SDTR_TWR_SHIFT)) +
     (1 + ((BOARD_FMC_SDTR2 & FMC_SDTR_TRP_MASK) >> FMC_SDTR_TRP_SHIFT)) +
     (1 + ((BOARD_FMC_SDTR2 & FMC_SDTR_TRC_MASK) >> FMC_SDTR_TRC_SHIFT)) +
     (1 + ((BOARD_FMC_SDTR2 & FMC_SDTR_TRCD_MASK) >> FMC_SDTR_TRCD_SHIFT))));
#endif

  stm32_fmc_sdram_set_refresh_rate(BOARD_FMC_SDRAM_REFR_PERIOD);
#endif /* BOARD_FMC_SDRTR */
}

#ifdef HAVE_FMC_SDRAM

/****************************************************************************
 * Name: stm32_fmc_sdram_init
 *
 * Description:
 *   Initialize the SDRAM chips.
 *
 ****************************************************************************/

#if defined HAVE_FMC_SDRAM1 && defined HAVE_FMC_SDRAM2
#  define FMC_SDCMR_CTB                 FMC_SDCMR_CTB1 | FMC_SDCMR_CTB2
#elif defined HAVE_FMC_SDRAM1
#  define FMC_SDCMR_CTB                 FMC_SDCMR_CTB1
#else
#  define FMC_SDCMR_CTB                 FMC_SDCMR_CTB2
#endif

void stm32_fmc_sdram_init(void)
{
  /* What is happening here:
   * ... A 100μs delay is required prior to issuing any command [...]
   * a PRECHARGE command should be applied once the 100μs delay has been
   * satisfied.
   *
   * All banks must be precharged.  This will leave all banks in an idle
   * state after which at least two AUTO REFRESH cycles must be performed.
   * After the AUTO REFRESH cycles are complete, the SDRAM is then ready
   * for mode register programming.
   *
   * The mode register should be loaded prior to applying any operational
   * command because it will power up in an unknown state.
   */

  up_udelay(100);

  /* Clock Configuration Enable */

  stm32_fmc_sdram_command(FMC_SDCMR_CTB | FMC_SDCMR_MODE_CLK_ENABLE);

  /* Wait for clock to stabilize */

  up_mdelay(1);

  /* Precharge all banks */

  stm32_fmc_sdram_wait(SDRAM_INIT_TIMEOUT);
  stm32_fmc_sdram_command(FMC_SDCMR_CTB | FMC_SDCMR_MODE_PALL);

  /* Auto refresh */

  stm32_fmc_sdram_wait(SDRAM_INIT_TIMEOUT);
  stm32_fmc_sdram_command(FMC_SDCMR_CTB | FMC_SDCMR_MODE_AUTO_REFRESH |
                          FMC_SDCMR_NRFS(BOARD_FMC_SDRAM_AUTOREFRESH));

  /* Program the SDRAM mode register */

  /* If using two SDRAM chips, this will write same mode for both.
   * If different mode is required for every chip (why?), it would be
   * needed to split code into two separate LOAD_MODE commands,
   * one for every SDRAM chip.
   */

  stm32_fmc_sdram_wait(SDRAM_INIT_TIMEOUT);
  stm32_fmc_sdram_command(FMC_SDCMR_CTB | FMC_SDCMR_MODE_LOAD_MODE |
                          BOARD_FMC_SDRAM_MODE);
}

/****************************************************************************
 * Name: stm32_fmc_sdram_wait
 *
 * Description:
 *   Wait until SDRAM controller is ready for all configured SDRAM banks.
 *
 * Parameters:
 *   timeout - number of microseconds before giving up waiting.
 *
 * Returns:
 *   0 if all configured SDRAM banks are ready, -1 on timeout
 *
 ****************************************************************************/

int stm32_fmc_sdram_wait(unsigned timeout)
{
  /* Wait until the SDRAM controller is ready */

  for (; timeout; timeout--)
    {
      uint32_t sdsr = getreg32(STM32_FMC_SDSR);
      if (1
#if defined HAVE_FMC_SDRAM1
          && ((sdsr & FMC_SDSR_MODES1_MASK) == FMC_SDSR_MODES1_NORMAL)
#endif
#if defined HAVE_FMC_SDRAM2
          && ((sdsr & FMC_SDSR_MODES2_MASK) == FMC_SDSR_MODES2_NORMAL)
#endif
         )
          return 0;

      up_udelay(1);
    }

  return -1;
}

#endif

/****************************************************************************
 * Name: stm32_fmc_sdram_write_protect
 *
 * Description:
 *   Enable/Disable writes to an SDRAM.
 *
 ****************************************************************************/

void stm32_fmc_sdram_write_protect(int bank, bool state)
{
  uint32_t val;
  uint32_t sdcr;

  DEBUGASSERT(bank == 1 || bank == 2);
  sdcr = (bank == 1) ? STM32_FMC_SDCR1 : STM32_FMC_SDCR2;

  val = getreg32(sdcr);
  if (state)
    {
      val |= FMC_SDCR_WP;       /* wp == 1 */
    }
  else
    {
      val &= ~FMC_SDCR_WP;      /* wp == 0 */
    }

  putreg32(val, sdcr);
}

/****************************************************************************
 * Name: stm32_fmc_sdram_set_refresh_rate
 *
 * Description:
 *   Set the SDRAM refresh rate.
 *
 ****************************************************************************/

void stm32_fmc_sdram_set_refresh_rate(int count)
{
  DEBUGASSERT(count <= 0x1fff && count >= 0x29);
  putreg32(FMC_SDRTR_COUNT(count), STM32_FMC_SDRTR);
}

/****************************************************************************
 * Name: stm32_fmc_sdram_set_timing
 *
 * Description:
 *   Set the SDRAM timing parameters.
 *
 ****************************************************************************/

void stm32_fmc_sdram_set_timing(int bank, uint32_t timing)
{
  uint32_t val;
  uint32_t sdtr;

  DEBUGASSERT((bank == 1) || (bank == 2));
  DEBUGASSERT((timing & FMC_SDTR_RESERVED) == 0);

  sdtr = (bank == 1) ? STM32_FMC_SDTR1 : STM32_FMC_SDTR2;
  val  = getreg32(sdtr);
  val &= FMC_SDTR_RESERVED;             /* preserve reserved bits */
  val |= timing;
  putreg32(val, sdtr);
}

/****************************************************************************
 * Name: stm32_fmc_enable
 *
 * Description:
 *   Enable FMC SDRAM. Do this after issue refresh rate.
 *
 ****************************************************************************/

void stm32_fmc_sdram_enable(void)
{
  uint32_t val;
  val = FMC_BCR_FMCEN | getreg32(STM32_FMC_BCR1);
  putreg32(val, STM32_FMC_BCR1);
}

/****************************************************************************
 * Name: stm32_fmc_sdram_set_control
 *
 * Description:
 *   Set the SDRAM control parameters.
 *
 ****************************************************************************/

void stm32_fmc_sdram_set_control(int bank, uint32_t ctrl)
{
  uint32_t val;
  uint32_t sdcr;

  DEBUGASSERT((bank == 1) || (bank == 2));
  DEBUGASSERT((ctrl & FMC_SDCR_RESERVED) == 0);

  sdcr = (bank == 1) ? STM32_FMC_SDCR1 : STM32_FMC_SDCR2;
  val  = getreg32(sdcr);
  val &= FMC_SDCR_RESERVED;             /* preserve reserved bits */
  val |= ctrl;
  putreg32(val, sdcr);
}

/****************************************************************************
 * Name: stm32_fmc_sdram_command
 *
 * Description:
 *   Send a command to the SDRAM.
 *
 ****************************************************************************/

void stm32_fmc_sdram_command(uint32_t cmd)
{
  DEBUGASSERT((cmd & FMC_SDCMR_RESERVED) == 0);
  putreg32(cmd, STM32_FMC_SDCMR);
}

#endif /* CONFIG_STM32H7_FMC */
