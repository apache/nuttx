/****************************************************************************
 * boards/arm/sama5/sama5d3x-ek/src/sam_sdram.c
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

#include <debug.h>

#include "arm_internal.h"
#include "sam_periphclks.h"
#include "hardware/sam_memorymap.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_sfr.h"
#include "hardware/sam_mpddrc.h"

#include "sama5d3x-ek.h"

/* This file requires:
 *
 * CONFIG_SAMA5_DDRCS -- DRAM support is enabled, and
 * !CONFIG_SAMA5_BOOT_SDRAM - We did not boot into SRAM.
 */

#if defined(CONFIG_SAMA5_DDRCS) && !defined(CONFIG_SAMA5_BOOT_SDRAM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SDRAM differences */

#if defined(CONFIG_SAMA5D3XEK_MT47H128M16RT)

  /* Used for SDRAM command handshaking */

#  define DDR2_BA0    (1 << 26)
#  define DDR2_BA1    (1 << 27)

#elif defined(CONFIG_SAMA5D3XEK_MT47H64M16HR)

  /* Used for SDRAM command handshaking */

#  define DDR2_BA0    (1 << 25)
#  define DDR2_BA1    (1 << 26)

#else
#  error Unknown SDRAM type
#endif

/* The delay loop in sam_sdram_delay requires 6 core cycles per iteration.
 *
 *   At 384MHz:
 *
 *     (6 cycles/iteration) / (0.384 cycles/nanosecond) =
 *       15.6250 nanoseconds per iteration
 *
 *   At 396MHz:
 *
 *     (6 cycles/iteration) / (0.396 cycles/nanosecond) =
 *       15.1515 nanoseconds per iteration
 *
 *   At 528MHz:
 *
 *     (6 cycles/iteration) / (0.528 cycles/nanosecond) =
 *       11.3636 nanoseconds per iteration
 */

#define LOOP_GUARD 100
#  define CYCLES_TO_COUNT(cycles) (((cycles) / 6) + LOOP_GUARD)

#if defined(CONFIG_SAMA5D3XEK_384MHZ)
#  define NSEC_TO_COUNT(nsec)     ((((nsec) * 1000) / 15625) + LOOP_GUARD)
#  define USEC_TO_COUNT(usec)     ((((usec) * 1000000) / 15625) + LOOP_GUARD)
#elif defined(CONFIG_SAMA5D3XEK_528MHZ)
#  define NSEC_TO_COUNT(nsec)     ((((nsec) * 1000) / 11364) + LOOP_GUARD)
#  define USEC_TO_COUNT(usec)     ((((usec) * 1000000) / 11364) + LOOP_GUARD)
#else /* #elif defined(CONFIG_SAMA5D3XEK_396MHZ) */
#  define NSEC_TO_COUNT(nsec)     ((((nsec) * 1000) / 15152) + LOOP_GUARD)
#  define USEC_TO_COUNT(usec)     ((((usec) * 1000000) / 15152) + LOOP_GUARD)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sdram_delay
 *
 * Description:
 *   Precision delay function for SDRAM configuration.
 *
 *   This delay loop requires 6 core cycles per iteration.  The actual
 *   amount of time delayed will then vary with PCK.
 *
 ****************************************************************************/

static inline void sam_sdram_delay(unsigned int loops)
{
  volatile unsigned int i;

  for (i = 0; i < loops; i++)
    {
      asm("nop");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sdram_config
 *
 * Description:
 *   Configures DDR2 (MT47H128M16RT 128MB or, optionally,  MT47H64M16HR)
 *
 *   Per the SAMA5D3x-EK User guide:
 *   "Two SDRAM/DDR2 used as main system memory.
 *   MT47H128M16 - 2 Gb - 16 Meg x 16 x 8 banks, the board provides up to
 *   2 Gb on-board, soldered DDR2 SDRAM.
 *   The memory bus is 32 bits wide and operates with up to 166 MHz."
 *
 *   From the Atmel Code Example:
 *     MT47H64M16HR : 8 Meg x 16 x 8 banks
 *     Refresh count: 8K
 *     Row address: A[12:0] (8K)
 *     Column address A[9:0] (1K)
 *     Bank address BA[2:0] a(24,25) (8)
 *
 *  This logic was taken from Atmel sample code for the SAMA5D3x-EK.
 *
 *  Input Parameters:
 *     devtype - Either DDRAM_MT47H128M16RT or DDRAM_MT47H64M16HR
 *
 *  Assumptions:
 *    The DDR memory regions is configured as strongly ordered memory.
 *    When we complete initialization of SDRAM and it is ready for use,
 *    we will make DRAM into normal memory.
 *
 ****************************************************************************/

void sam_sdram_config(void)
{
  volatile uint8_t *ddr = (uint8_t *)SAM_DDRCS_VSECTION;
  uint32_t regval;

  /* Enable x2 clocking to the MPDDRC */

  sam_mpddrc_enableclk();

  /* Enable DDR clocking */

  regval  = getreg32(SAM_PMC_SCER);
  regval |= PMC_DDRCK;
  putreg32(regval, SAM_PMC_SCER);

  /* Clear the low power register */

  putreg32(0, SAM_MPDDRC_LPR);

  /* Enable autofresh during calibration (undocumented) */

  regval  = getreg32(SAM_MPDDRC_HS);
  regval |= MPDDRC_HS_AUTOREFRESH_CAL;
  putreg32(regval, SAM_MPDDRC_HS);

  /* Force DDR_DQ and DDR_DQS input buffer always on */

  regval  = getreg32(SAM_SFR_DDRCFG);
  regval |= SFR_FDQIEN | SFR_FDQSIEN;
  putreg32(regval, SAM_SFR_DDRCFG);

  /* Configure the slave offset register */

  regval = MPDDRC_DLL_SOR_S0OFF(1) |      /* DLL Slave 0 Delay Line Offset */
           MPDDRC_DLL_SOR_S1OFF(0) |      /* DLL Slave 1 Delay Line Offset */
           MPDDRC_DLL_SOR_S2OFF(1) |      /* DLL Slave 2 Delay Line Offset */
           MPDDRC_DLL_SOR_S3OFF(1);       /* DLL Slave 3 Delay Line Offset */
  putreg32(regval, SAM_MPDDRC_DLL_SOR);

  /* Configure the master offset register (including upper mystery bits) */

  regval = MPDDRC_DLL_MOR_MOFF(7) |       /* DLL Master Delay Line Offset */
           MPDDRC_DLL_MOR_CLK90OFF(31) |  /* DLL CLK90 Delay Line Offset */
           MPDDRC_DLL_MOR_SELOFF |        /* DLL Offset Selection */
           MPDDRC_DLL_MOR_KEY;            /* Undocumented key */
  putreg32(regval, SAM_MPDDRC_DLL_MOR);

  /* Configure the I/O calibration register */

  regval  =  getreg32(SAM_MPDDRC_IO_CALIBR);
  regval &= ~(MPDDRC_IO_CALIBR_RDIV_MASK | MPDDRC_IO_CALIBR_TZQIO_MASK);
  regval |= (MPDDRC_IO_CALIBR_RZQ48_40 |  MPDDRC_IO_CALIBR_TZQIO(3));
  putreg32(regval, SAM_MPDDRC_IO_CALIBR);

  /* Force DDR_DQ and DDR_DQS input buffer always on */

  putreg32(SFR_FDQIEN | SFR_FDQSIEN, SAM_SFR_DDRCFG);

  /* Step 1: Program the memory device type
   *
   *   DBW = 0 (32-bit bus wide)
   *   Memory Device = DDR2-SDRAM
   */

  putreg32(MPDDRC_MD_DDR2_SDRAM, SAM_MPDDRC_MD);

  /* Step 2: Program the features of DDR2-SDRAM device into the Timing
   * Register
   */

#if defined(CONFIG_SAMA5D3XEK_MT47H128M16RT)

  /* For MT47H128M16RT
   *
   *   NC      = 10 DDR column bits
   *   NR      = 14 DDR row bits
   *   CAS     = DDR2/LPDDR2 CAS Latency 4
   *   DLL     = Disable reset (0)
   *   DIC_DS  = 0
   *   DIS_DLL = Enable PLL (0)
   *   ZQ      = Calibration command after initialization (0)
   *   OCD     = OCD calibration mode exit, maintain setting (0)
   *   DQMS    = Not shared (0)
   *   ENDRM   = Disable read measure (0)
   *   NB      = 8 banks
   *   NDQS    = Not DQS disabled
   *   DECODE  = Sequential decoding (0)
   *   UNAL    = Unaliged access supported
   */

  regval = MPDDRC_CR_NC_10 |    /* Number of Column Bits */
           MPDDRC_CR_NR_14 |    /* Number of Row Bits */
           MPDDRC_CR_CAS_4 |    /* CAS Latency */
           MPDDRC_CR_OCD_EXIT | /* Off-chip Driver */
           MPDDRC_CR_8BANKS |   /* Number of Banks */
           MPDDRC_CR_NDQS |     /* Not DQS */
           MPDDRC_CR_UNAL;      /* upport Unaligned Access */

#elif defined(CONFIG_SAMA5D3XEK_MT47H64M16HR)
  /* For MT47H64M16HR
   *
   *   NC      = 10 DDR column bits
   *   NR      = 13 DDR row bits
   *   CAS     = DDR2/LPDDR2 CAS Latency 3
   *   DLL     = Disable reset (0)
   *   DIC_DS  = 0
   *   DIS_DLL = Enable PLL (0)
   *   ZQ      = Calibration command after initialization (0)
   *   OCD     = OCD calibration mode exit, maintain setting (0)
   *   DQMS    = Not shared (0)
   *   ENDRM   = Disable read measure (0)
   *   NB      = 8 banks
   *   NDQS    = Not DQS disabled
   *   DECODE  = Sequential decoding (0)
   *   UNAL    = Unaliged access supported
   */

  regval = MPDDRC_CR_NC_10 |    /* Number of Column Bits */
           MPDDRC_CR_NR_13 |    /* Number of Row Bits */
           MPDDRC_CR_CAS_3 |    /* CAS Latency */
           MPDDRC_CR_OCD_EXIT | /* Off-chip Driver */
           MPDDRC_CR_8BANKS |   /* Number of Banks */
           MPDDRC_CR_NDQS |     /* Not DQS */
           MPDDRC_CR_UNAL;      /* upport Unaligned Access */

#else
#  error Unknown SDRAM type
#endif

  putreg32(regval, SAM_MPDDRC_CR);

  /* Configure the Timing Parameter 0 Register */

  regval = MPDDRC_TPR0_TRAS(6) |    /*  Active to Precharge Delay:    6 * 7.5 = 45 ns */
           MPDDRC_TPR0_TRCD(2) |    /*  Row to Column Delay:          2 * 7.5 = 15 ns */
           MPDDRC_TPR0_TWR(2) |     /*  Write Recovery Delay:         3 * 7.5 = 22.5 ns */
           MPDDRC_TPR0_TRC(8) |     /*  Row Cycle Delay:              8 * 7.5 = 60 ns */
           MPDDRC_TPR0_TRP(2) |     /*  Row Precharge Delay:          2 * 7.5 = 15 ns */
           MPDDRC_TPR0_TRRD(1) |    /*  Active BankA to Active BankB: 2 * 7.5 = 15 ns */
           MPDDRC_TPR0_TWTR(2) |    /*  Internal Write to Read Delay: 2 clock cycle */
           MPDDRC_TPR0_TMRD(2);     /*  Load Mode Register Command to
                                     *  Activate or Refresh Command:  2 clock cycles
                                     */
  putreg32(regval, SAM_MPDDRC_TPR0);

  /* Configure the Timing Parameter 1 Register */

  regval = MPDDRC_TPR1_TRFC(14) |   /* Row Cycle Delay:
                                     *   18 * 7.5 = 135 ns (min 127.5 ns for 1Gb DDR)
                                     */
           MPDDRC_TPR1_TXSNR(16) |  /* Exit Self Refresh Delay to Non Read Command:
                                     *   20 * 7.5 > 142.5ns TXSNR: Exit self refresh
                                     *   delay to non read command
                                     */
           MPDDRC_TPR1_TXSRD(208) | /* Exit Self Refresh Delay to Read Command:
                                     *   min 200 clock cycles, TXSRD: Exit self refresh
                                     *   delay to Read command
                                     */
           MPDDRC_TPR1_TXP(2);      /* Exit Power-down Delay to First Command:
                                     *   2 * 7.5 = 15 ns
                                     */
  putreg32(regval, SAM_MPDDRC_TPR1);

  /* Configure the Timing Parameter 2 Register */

  regval = MPDDRC_TPR2_TXARD(7) |   /* Exit Active Power Down Delay to Read Command in Mode 'Fast Exit':
                                     *   min 2 clock cycles
                                     */
           MPDDRC_TPR2_TXARDS(7) |  /* Exit Active Power Down Delay to Read Command in Mode 'Slow Exit':
                                     *   min 7 clock cycles
                                     */
           MPDDRC_TPR2_TRPA(2) |    /* Row Precharge All Delay:
                                     *   min 18ns
                                     */
           MPDDRC_TPR2_TRTP(2) |    /* Four Active Windows:
                                     *   2 * 7.5 = 15 ns (min 7.5ns)
                                     */
           MPDDRC_TPR2_TFAW(10);
  putreg32(regval, SAM_MPDDRC_TPR2);

  /* DDRSDRC Low-power Register */

  sam_sdram_delay(USEC_TO_COUNT(200));

  regval = MPDDRC_LPR_LPCB_DISABLED |  /* Low-power Feature is inhibited */
           MPDDRC_LPR_TIMEOUT_0CLKS |  /* Activates low-power mode after the end of transfer */
           MPDDRC_LPR_APDE_FAST;       /* Active Power Down Exit Time */
  putreg32(regval, SAM_MPDDRC_LPR);

  /* Step 3: An NOP command is issued to the DDR2-SDRAM. Program the NOP
   * command into the Mode Register, the application must set MODE to 1 in
   * the Mode Register.
   */

  putreg32(MPDDRC_MR_MODE_NOP, SAM_MPDDRC_MR);

  /* Perform a write access to any DDR2-SDRAM address to acknowledge this
   * command.
   */

  *ddr = 0;

  /* Now clocks which drive DDR2-SDRAM device are enabled.
   *
   * A minimum pause of 200 usec is provided to precede any signal toggle.
   * (6 core cycles per iteration, core is at 396MHz: min 13200 loops)
   */

  sam_sdram_delay(USEC_TO_COUNT(200));

  /* Step 4:  An NOP command is issued to the DDR2-SDRAM */

  putreg32(MPDDRC_MR_MODE_NOP, SAM_MPDDRC_MR);

  /* Perform a write access to any DDR2-SDRAM address to
   * acknowledge this command.
   */

  *ddr = 0;

  /* Now CKE is driven high. */

  /* Wait 400 ns min */

  sam_sdram_delay(NSEC_TO_COUNT(400));

  /* Step 5: An all banks precharge command is issued to
   * the DDR2-SDRAM.
   */

  putreg32(MPDDRC_MR_MODE_PRCGALL, SAM_MPDDRC_MR);

  /* Perform a write access to any DDR2-SDRAM address to
   * acknowledge this command.
   */

  *ddr = 0;

  /* Wait 400 ns min */

  sam_sdram_delay(NSEC_TO_COUNT(400));

  /* Step 6: An Extended Mode Register set (EMRS2) cycle is  issued to chose
   * between commercialor high  temperature operations.
   *
   * The write address must be chosen so that BA[1] is set to 1 and BA[0] is
   * set to 0.
   */

  putreg32(MPDDRC_MR_MODE_EXTLMR, SAM_MPDDRC_MR);
  *((volatile uint8_t *)(ddr + DDR2_BA1)) = 0;

  /* Wait 2 cycles min */

  sam_sdram_delay(100 /* CYCLES_TO_COUNT(2) */);

  /* Step 7: An Extended Mode Register set (EMRS3) cycle is issued to set
   * all registers to 0.
   *
   * The write address must be chosen so that BA[1] is set to 1 and BA[0] is
   * set to 1.
   */

  putreg32(MPDDRC_MR_MODE_LMR, SAM_MPDDRC_MR);
  *((volatile uint8_t *)(ddr + DDR2_BA1 + DDR2_BA0)) = 0;

  /* Wait 2 cycles min */

  sam_sdram_delay(100 /* CYCLES_TO_COUNT(2) */);

  /* Step 8:
   * An Extended Mode Register set (EMRS1) cycle is issued to enable DLL.
   *
   * The write address must be chosen so that BA[1] is set to 0 and BA[0]
   * is set to 1.
   */

  putreg32(MPDDRC_MR_MODE_EXTLMR, SAM_MPDDRC_MR);
  *((volatile uint8_t *)(ddr + DDR2_BA0)) = 0;

  /* An additional 200 cycles of clock are required for locking DLL */

  sam_sdram_delay(10000 /* CYCLES_TO_COUNT(200) */);

  /* Step 9:  Program DLL field into the Configuration Register. */

  regval  = getreg32(SAM_MPDDRC_CR);
  regval |= MPDDRC_CR_DLL;
  putreg32(regval, SAM_MPDDRC_CR);

  /* Step 10: A Mode Register set (MRS) cycle is issued to reset DLL.
   *
   * The write address must be chosen so that BA[1:0] bits are set to 0.
   */

  putreg32(MPDDRC_MR_MODE_LMR, SAM_MPDDRC_MR);
  *ddr = 0;

  /* Wait 2 cycles min */

  sam_sdram_delay(100 /* CYCLES_TO_COUNT(2) */);

  /* Step 11: An all banks precharge command is issued to the DDR2-SDRAM.
   *
   * Perform a write access to any DDR2-SDRAM address to acknowledge this
   * command
   */

  putreg32(MPDDRC_MR_MODE_PRCGALL, SAM_MPDDRC_MR);
  *ddr = 0;

  /* Wait 2 cycles min */

  sam_sdram_delay(100 /* CYCLES_TO_COUNT(2) */);

  /* Step 12: Two auto-refresh (CBR) cycles are provided. Program the auto
   * refresh command (CBR) into the Mode Register.
   *
   * Perform a write access to any DDR2-SDRAM address to acknowledge this
   * command.
   */

  putreg32(MPDDRC_MR_MODE_RFSH, SAM_MPDDRC_MR);
  *ddr = 0;

  /* Wait 2 cycles min */

  sam_sdram_delay(100 /* CYCLES_TO_COUNT(2) */);

  /* Configure 2nd CBR.
   *
   * Perform a write access to any DDR2-SDRAM address to
   * acknowledge this command.
   */

  putreg32(MPDDRC_MR_MODE_RFSH, SAM_MPDDRC_MR);
  *ddr = 0;

  /* Wait 2 cycles min */

  sam_sdram_delay(100 /* CYCLES_TO_COUNT(2) */);

  /* Step 13: Program DLL field into the Configuration Register to low
   * (Disable DLL reset).
   */

  regval  = getreg32(SAM_MPDDRC_CR);
  regval &= ~MPDDRC_CR_DLL;
  putreg32(regval, SAM_MPDDRC_CR);

  /* Step 14: A Mode Register set (MRS) cycle is issued to program the
   * parameters of the DDR2-SDRAM devices.
   *
   * The write address must be chosen so that BA[1:0] are set to 0.
   */

  putreg32(MPDDRC_MR_MODE_LMR, SAM_MPDDRC_MR);
  *ddr = 0;

  /* Wait 2 cycles min */

  sam_sdram_delay(100 /* CYCLES_TO_COUNT(2) */);

  /* Step 15: Program OCD field into the Configuration Register to high (OCD
   * calibration default).
   */

  regval  = getreg32(SAM_MPDDRC_CR);
  regval |= MPDDRC_CR_OCD_DEFAULT;
  putreg32(regval, SAM_MPDDRC_CR);

  /* Step 16: An Extended Mode Register set (EMRS1) cycle is issued to OCD
   * default value.
   *
   * The write address must be chosen so that BA[1] is set to 0 and BA[0] is
   * set to 1.
   */

  putreg32(MPDDRC_MR_MODE_EXTLMR, SAM_MPDDRC_MR);
  *((volatile uint8_t *)(ddr + DDR2_BA0)) = 0;

  /* Wait 2 cycles min */

  sam_sdram_delay(100 /* CYCLES_TO_COUNT(2) */);

  /* Step 17: Program OCD field into the Configuration Register to low (OCD
   * calibration mode exit).
   */

#if 0
  regval  = getreg32(SAM_MPDDRC_CR);
  regval &= ~MPDDRC_CR_OCD_MASK;
  putreg32(regval, SAM_MPDDRC_CR);
#endif

  /* Step 18: An Extended Mode Register set (EMRS1) cycle is issued to
   * enable OCD exit.
   *
   * The write address must be chosen so that BA[1] is set to 0 and BA[0] is
   * set to 1.
   */

  putreg32(MPDDRC_MR_MODE_EXTLMR, SAM_MPDDRC_MR);
  *((volatile uint8_t *)(ddr + DDR2_BA0)) = 0;

  /* Wait 2 cycles min */

  sam_sdram_delay(100 /* CYCLES_TO_COUNT(2) */);

  /* Step 19,20: A mode Normal command is provided. Program the Normal mode
   * into Mode Register.
   */

  putreg32(MPDDRC_MR_MODE_NORMAL, SAM_MPDDRC_MR);
  *ddr = 0;

  /* Step 21: Write the refresh rate into the count field in the Refresh
   * Timer register. The DDR2-SDRAM device requires a refresh every 15.625
   * usec or 7.81 usec.
   *
   * With a 100MHz frequency, the refresh timer count register must to be
   * set with (15.625 /100 MHz) = 1562 i.e. 0x061A or (7.81 /100MHz) = 781
   * i.e. 0x030d.
   */

  /* For MT47H64M16HR, The refresh period is 64ms (commercial), This equates
   * to an average refresh rate of 7.8125usec (commercial), To ensure all
   * rows of all banks are properly refreshed, 8192 REFRESH commands must be
   * issued every 64ms (commercial)
   */

  /* ((64 x 10(^-3))/8192) x133 x (10^6) */

  /* Set Refresh timer 7.8125 us */

  putreg32(MPDDRC_RTR_COUNT(300), SAM_MPDDRC_RTR);

  /* OK now we are ready to work on the DDRSDR */

  /* Wait for end of calibration */

  sam_sdram_delay(500);
}
#endif /* CONFIG_SAMA5_DDRCS && !CONFIG_SAMA5_BOOT_SDRAM */
