/****************************************************************************
 * boards/arm/sama5/sama5d4-ek/src/sam_sdram.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Most of this file derives from Atmel sample code for the SAMA5D4x-EK
 * board.  That sample code has licensing that is compatible with the NuttX
 * modified BSD license:
 *
 *   Copyright (c) 2013, Atmel Corporation
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor Atmel nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* The DDR/SDR SDRAM Controller (DDRSDRC) is a multi-port memory controller.
 * It comprises four slave AHB interfaces. All simultaneous accesses (four
 * independent AHB ports) are interleaved to maximize memory bandwidth and
 * minimize transaction latency due to SDRAM protocol.
 *
 * -------------------------------------------------------------------------
 * DDR2 Configuration
 *
 * The DDR2-SDRAM devices are initialized by the following sequence:
 *
 * * EBI Chip Select 1 is assigned to the DDR2SDR Controller, Enable DDR2
 *   clock x2 in PMC.
 * * Step 1: Program the memory device type
 * * Step 2:
 *   - Program the features of DDR2-SDRAM device into the Configuration
 *     Register.
 *   - Program the features of DDR2-SDRAM device into the Timing Register
 *     HDDRSDRC2_T0PR.
 *   - Program the features of DDR2-SDRAM device into the Timing Register
 *     HDDRSDRC2_T1PR.
 *   - Program the features of DDR2-SDRAM device into the Timing Register
 *     HDDRSDRC2_T2PR.
 * * Step 3: An NOP command is issued to the DDR2-SDRAM to enable clock.
 * * Step 4:  An NOP command is issued to the DDR2-SDRAM
 * * Step 5: An all banks pre-charge command is issued to the DDR2-SDRAM.
 * * Step 6: An Extended Mode Register set (EMRS2) cycle is  issued to chose
 *   between commercial or high  temperature operations.
 * * Step 7: An Extended Mode Register set (EMRS3) cycle is issued to set
 *   all registers to 0.
 * * Step 8:  An Extended Mode Register set (EMRS1) cycle is issued to
 *   enable DLL.
 * * Step 9:  Program DLL field into the Configuration Register.
 * * Step 10: A Mode Register set (MRS) cycle is issued to reset DLL.
 * * Step 11: An all banks pre-charge command is issued to the DDR2-SDRAM.
 * * Step 12: Two auto-refresh (CBR) cycles are provided. Program the
 *   auto refresh command (CBR) into the Mode Register.
 * * Step 13: Program DLL field into the Configuration Register to
 *   low(Disable DLL reset).
 * * Step 14: A Mode Register set (MRS) cycle is issued to program the
 *   parameters of the DDR2-SDRAM devices.
 * * Step 15: Program OCD field into the Configuration Register to high (OCD
 *   calibration default).
 * * Step 16: An Extended Mode Register set (EMRS1) cycle is issued to OCD
 *   default value.
 * * Step 17: Program OCD field into the Configuration Register to low (OCD
 *   calibration mode exit).
 * * Step 18: An Extended Mode Register set (EMRS1) cycle is issued to
 *   enable OCD exit.
 * * Step 19,20: A mode Normal command is provided. Program the Normal mode
 *   into Mode Register.
 * * Step 21: Write the refresh rate into the count field in the Refresh
 *   Timer register. The DDR2-SDRAM device requires a refresh every 15.625
 *   or 7.81.
 *
 * -------------------------------------------------------------------------
 * SDRAM Configuration
 *
 * The SDR-SDRAM devices are initialized by the following sequence:
 *
 * * EBI Chip Select 1 is assigned to the DDR2SDR Controller, Enable DDR2
 *   clock x2 in PMC.
 * * Step 1. Program the memory device type into the Memory Device Register
 * * Step 2. Program the features of the SDR-SDRAM device into the Timing
 *   Register and into the Configuration Register.
 * * Step 3. For low-power SDRAM, temperature-compensated self refresh
 *   (TCSR), drive strength (DS) and partial array self refresh (PASR) must
 *   be set in the Low-power Register.
 * * Step 4. A NOP command is issued to the SDR-SDRAM. Program NOP command
 *   into Mode Register, the application must set Mode to 1 in the Mode
 *   Register. Perform a write access to any SDR-SDRAM address to
 *   acknowledge this command. Now the clock which drives SDR-SDRAM device
 *   is enabled.
 * * Step 5. An all banks pre-charge command is issued to the SDR-SDRAM.
 *   Program all banks pre-charge command into Mode Register, the application
 *   must set Mode to 2 in the Mode Register . Perform a write access to any
 *   SDRSDRAM address to acknowledge this command.
 * * Step 6. Eight auto-refresh (CBR) cycles are provided. Program the auto
 *   refresh command (CBR) into Mode Register, the application must set Mode
 *   to 4 in the Mode Register.  Once in the idle state, two AUTO REFRESH
 *   cycles must be performed.
 * * Step 7. A Mode Register set (MRS) cycle is issued to program the
 *   parameters of the SDRSDRAM devices, in particular CAS latency and burst
 *   length.
 * * Step 8. For low-power SDR-SDRAM initialization, an Extended Mode
 *   Register set (EMRS) cycle is issued to program the SDR-SDRAM parameters
 *   (TCSR, PASR, DS). The write address must be chosen so that BA[1] is set
 *   to 1 and BA[0] is set to 0
 * * Step 9. The application must go into Normal Mode, setting Mode to 0 in
 *   the Mode Register and perform a write access at any location in the
 *   SDRAM to acknowledge this command.
 * * Step 10. Write the refresh rate into the count field in the DDRSDRC
 *   Refresh Timer register
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>

#include "arm_arch.h"

#include "sam_periphclks.h"
#include "hardware/sam_memorymap.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_sfr.h"
#include "hardware/sam_matrix.h"
#include "hardware/sam_mpddrc.h"

#include "sama5d4-ek.h"

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

#if defined(CONFIG_SAMA5D4EK_MT47H128M16RT)

  /* Used for SDRAM command handshaking */

#  define DDR2_BA0(r) (1 << ((r) + 26))
#  define DDR2_BA1(r) (1 << ((r) + 27))

#elif defined(CONFIG_SAMA5D4EK_MT47H64M16HR)

  /* Used for SDRAM command handshaking */

#  define DDR2_BA0(r) (1 << ((r) + 25))
#  define DDR2_BA1(r) (1 << ((r) + 26))

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

#if defined(CONFIG_SAMA5D4EK_384MHZ)
#  define NSEC_TO_COUNT(nsec)     ((((nsec) * 1000) / 15625) + LOOP_GUARD)
#  define USEC_TO_COUNT(usec)     ((((usec) * 1000000) / 15625) + LOOP_GUARD)
#elif defined(CONFIG_SAMA5D4EK_528MHZ)
#  define NSEC_TO_COUNT(nsec)     ((((nsec) * 1000) / 11364) + LOOP_GUARD)
#  define USEC_TO_COUNT(usec)     ((((usec) * 1000000) / 11364) + LOOP_GUARD)
#else /* #elif defined(CONFIG_SAMA5D4EK_396MHZ) */
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
 * Name: sam_sdram_delay
 *
 * Description:
 *   Precision delay function for SDRAM configuration.
 *
 *   This delay loop requires 6 core cycles per iteration.  The actual
 *   amount of time delayed will then vary with PCK.
 *
 ****************************************************************************/

static void sam_config_slaveddr(void)
{
  int ddrport;

  /* Disable write protection */

  putreg32(MPDDRC_WPCR_WPKEY, SAM_MATRIX0_WPMR);

  /* Partition internal SRAM */

  putreg32(0, SAM_MATRIX0_SSR(11));
  putreg32(5, SAM_MATRIX0_SRTSR(11));
  putreg32(4, SAM_MATRIX0_SASSR(11));

  /* Partition external DDR.  DDR port 0 not used from NWd */

  for (ddrport = 1 ; ddrport < 8 ; ddrport++)
    {
      putreg32(0x00ffffff, SAM_MATRIX0_SSR(H64MX_DDR_SLAVE_PORT0 + ddrport));
      putreg32(0x0000000f, SAM_MATRIX0_SRTSR(H64MX_DDR_SLAVE_PORT0 + ddrport));
      putreg32(0x0000ffff, SAM_MATRIX0_SASSR(H64MX_DDR_SLAVE_PORT0 + ddrport));
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
 *   Per the SAMA5D3-EK User guide:
 *   "Two DDR2/SDRAM (MT47H64M16HR) used as main system memory (256 MByte).
 *   The board includes 2 Gbits of on-board solderedDDR2 (double data rate)
 *   SDRAM. The footprints can also host two DDR2 (MT47H128M16RT) from Micron®
 *   for a total of 512 MBytes of DDR2 memory. The memory bus is 32 bits wide
 *   and operates with a frequency of up to 166 MHz."
 *
 *   From the Atmel Code Example:
 *     MT47H64M16HR : 8 Meg x 16 x 8 banks
 *     Refresh count: 8K
 *     Row address: A[12:0] (8K)
 *     Column address A[9:0] (1K)
 *     Bank address BA[2:0] a(24,25) (8)
 *
 *  This logic was taken from Atmel sample code for the SAMA5D4-EK.
 *
 *  Input Parameters:
 *    None
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

  /* Setup DDR partitions */

  sam_config_slaveddr();

  /* Enable x2 clocking to the MPDDRC */

  sam_mpddrc_enableclk();

  /* Enable DDR clocking */

  regval  = getreg32(SAM_PMC_SCER);
  regval |= PMC_DDRCK;
  putreg32(regval, SAM_PMC_SCER);

  /* Step 1: Program the memory device type
   *
   * DBW = 0 (32 bits bus wide)
   * Memory Device = 6 = DDR2-SDRAM = 0x00000006
   */

  putreg32(MPDDRC_MD_DDR2_SDRAM, SAM_MPDDRC_MD);
  putreg32(MPDDRC_RD_DATA_PATH_SHIFT_SAMPLING_1CYCLE,
           SAM_MPDDRC_RD_DATA_PATH);

  regval = MPDDRC_TPR0_TRAS(9)    | /* 6 * 7.5 = 45 ns */
           MPDDRC_TPR0_TRCD(3)    | /* 2 * 7.5 = 15 ns */
           MPDDRC_TPR0_TWR(3)     | /* 3 * 7.5 = 22.5 ns */
           MPDDRC_TPR0_TRC(10)    | /* 8 * 7.5 = 60 ns */
           MPDDRC_TPR0_TRP(3)     | /* 2 * 7.5 = 15 ns */
           MPDDRC_TPR0_TRRD(2)    | /* 2 * 7.5 = 15 ns */
           MPDDRC_TPR0_TWTR(2)    | /* 2 clock cycle */
           MPDDRC_TPR0_TMRD(2);     /* 2 clock cycles */
  putreg32(regval, SAM_MPDDRC_TPR0);

  regval = MPDDRC_TPR1_TRFC(31)   | /* 18 * 7.5 = 135 ns (min 127.5 ns for 1Gb DDR) */
           MPDDRC_TPR1_TXSNR(33)  | /* 20 * 7.5 > 142.5ns TXSNR: Exit self refresh delay to non read command */
           MPDDRC_TPR1_TXSRD(200) | /* min 200 clock cycles, TXSRD: Exit self refresh delay to Read command */
           MPDDRC_TPR1_TXP(2);      /* 2 * 7.5 = 15 ns */
  putreg32(regval, SAM_MPDDRC_TPR1);

  regval = MPDDRC_TPR2_TXARD(7)   | /* min 2 clock cycles */
           MPDDRC_TPR2_TXARDS(7)  | /*  min 7 clock cycles */
           MPDDRC_TPR2_TRPA(4)    | /* min 18ns */
           MPDDRC_TPR2_TRTP(2)    | /* 2 * 7.5 = 15 ns (min 7.5ns) */
           MPDDRC_TPR2_TFAW(9);
  putreg32(regval, SAM_MPDDRC_TPR2);

  /* Clear the low power register */

  putreg32(0, SAM_MPDDRC_LPR);

  regval = getreg32(SAM_MPDDRC_IO_CALIBR);
  regval &= ~(MPDDRC_IO_CALIBR_RDIV_MASK | MPDDRC_IO_CALIBR_TZQIO_MASK);
  regval |=  (MPDDRC_IO_CALIBR_RZQ60_50  | MPDDRC_IO_CALIBR_TZQIO(5) |
              MPDDRC_IO_CALIBR_EN_CALIB);
  putreg32(regval, SAM_MPDDRC_IO_CALIBR);

  /* Step 2: Program the features of DDR2-SDRAM device into the Timing Register */

#if defined(CONFIG_SAMA5D4EK_MT47H128M16RT)

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

  regval = MPDDRC_CR_NC_10    | /* Number of Column Bits */
           MPDDRC_CR_NR_14    | /* Number of Row Bits */
           MPDDRC_CR_CAS_3    | /* CAS Latency */
           MPDDRC_CR_OCD_EXIT | /* Off-chip Driver */
           MPDDRC_CR_ZQ_INIT  |
           MPDDRC_CR_8BANKS   | /* Number of Banks */
           MPDDRC_CR_NDQS     | /* Not DQS */
           MPDDRC_CR_UNAL;      /* Support Unaligned Access */

#elif defined(CONFIG_SAMA5D4EK_MT47H64M16HR)
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

  regval = MPDDRC_CR_NC_10    | /* Number of Column Bits */
           MPDDRC_CR_NR_13    | /* Number of Row Bits */
           MPDDRC_CR_CAS_3    | /* CAS Latency */
           MPDDRC_CR_OCD_EXIT | /* Off-chip Driver */
           MPDDRC_CR_ZQ_INIT  |
           MPDDRC_CR_8BANKS   | /* Number of Banks */
           MPDDRC_CR_NDQS     | /* Not DQS */
           MPDDRC_CR_UNAL;      /* upport Unaligned Access */

#else
#  error Unknown SDRAM type
#endif

  putreg32(regval, SAM_MPDDRC_CR);

  /* DDRSDRC Low-power Register */

  sam_sdram_delay(USEC_TO_COUNT(200));

#if 0
  regval = MPDDRC_LPR_LPCB_DISABLED |  /* Low-power Feature is inhibited */
           MPDDRC_LPR_TIMEOUT_0CLKS |  /* Activates low-power mode after the end of transfer */
           MPDDRC_LPR_APDE_FAST;       /* Active Power Down Exit Time */
  putreg32(regval, SAM_MPDDRC_LPR);
#endif

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

  /* Perform a write access to any DDR2-SDRAM address to acknowledge this
   * command.
   */

  *ddr = 0;

  /* Now CKE is driven high. */

  /* Wait 400 ns min */

  sam_sdram_delay(NSEC_TO_COUNT(400));

  /* Step 5: An all banks pre-charge command is issued to the DDR2-SDRAM. */

  putreg32(MPDDRC_MR_MODE_PRCGALL, SAM_MPDDRC_MR);

  /* Perform a write access to any DDR2-SDRAM address to acknowledge this
   * command.
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
  *((volatile uint8_t *)(ddr + DDR2_BA1(0))) = 0;

  /* Wait 2 cycles min */

  sam_sdram_delay(100 /* CYCLES_TO_COUNT(2) */);

  /* Step 7: An Extended Mode Register set (EMRS3) cycle is issued to set
   * all registers to 0.
   *
   * The write address must be chosen so that BA[1] is set to 1 and BA[0] is
   * set to 1.
   */

  putreg32(MPDDRC_MR_MODE_LMR, SAM_MPDDRC_MR);
  *((volatile uint8_t *)(ddr + DDR2_BA1(0) + DDR2_BA0(0))) = 0;

  /* Wait 2 cycles min */

  sam_sdram_delay(100 /* CYCLES_TO_COUNT(2) */);

  /* Step 8:  An Extended Mode Register set (EMRS1) cycle is issued to
   * enable DLL.
   *
   * The write address must be chosen so that BA[1] is set to 0 and BA[0]
   * is set to 1.
   */

  putreg32(MPDDRC_MR_MODE_EXTLMR, SAM_MPDDRC_MR);
  *((volatile uint8_t *)(ddr + DDR2_BA0(0))) = 0;

  /* An additional 200 cycles of clock are required for locking DLL */

  sam_sdram_delay(10000); /* CYCLES_TO_COUNT(200) */

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
   * Perform a write access to any DDR2-SDRAM address to acknowledge this
   * command.
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
  *((volatile uint8_t *)(ddr + DDR2_BA0(0))) = 0;

  /* Wait 2 cycles min */

  sam_sdram_delay(100 /* CYCLES_TO_COUNT(2) */);

  /* Step 17: Program OCD field into the Configuration Register to low (OCD
   * calibration mode exit).
   */

  regval  = getreg32(SAM_MPDDRC_CR);
  regval &= ~MPDDRC_CR_OCD_MASK;
  putreg32(regval, SAM_MPDDRC_CR);

  /* Step 18: An Extended Mode Register set (EMRS1) cycle is issued to
   * enable OCD exit.
   *
   * The write address must be chosen so that BA[1] is set to 0 and BA[0] is
   * set to 1.
   */

  putreg32(MPDDRC_MR_MODE_EXTLMR, SAM_MPDDRC_MR);
  *((volatile uint8_t *)(ddr + DDR2_BA0(0))) = 0;

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
   *
   * For MT47H64M16HR, The refresh period is 64ms (commercial), This equates
   * to an average refresh rate of 7.8125usec (commercial), To ensure all
   * rows of all banks are properly refreshed, 8192 REFRESH commands must be
   * issued every 64ms (commercial)
   *
   *   ((64 x 10(^-3))/8192) x133 x (10^6)
   */

  /* Set Refresh timer 7.8125 us */

  putreg32(MPDDRC_RTR_COUNT(695), SAM_MPDDRC_RTR);

  /* OK now we are ready to work on the DDRSDR.
   *
   * Wait for end of calibration
   */

  sam_sdram_delay(500);
}

#endif /* CONFIG_SAMA5_DDRCS && !CONFIG_SAMA5_BOOT_SDRAM */
