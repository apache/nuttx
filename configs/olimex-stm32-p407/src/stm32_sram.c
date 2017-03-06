/************************************************************************************
 * configs/olimex-stm32-p407/src/stm32_sram.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32.h"
#include "stm3240g-eval.h"

#ifdef CONFIG_STM32_FSMC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif

#if defined(CONFIG_STM32_USART3) || defined(CONFIG_STM32_USART6)
#  error "USART3 and USART6 conflict with use of SRAM"
#endif

/* SRAM Timing
 * REVIST:  These were ported from the STM3240G-EVAL and have not been verified on
 * this platform.
 */

#define SRAM_ADDRESS_SETUP_TIME      3
#define SRAM_ADDRESS_HOLD_TIME       0
#define SRAM_DATA_SETUP_TIME         6
#define SRAM_BUS_TURNAROUND_DURATION 1
#define SRAM_CLK_DIVISION            0
#define SRAM_DATA_LATENCY            0

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* GPIOs Configuration **************************************************************
 *---------------------+------------------+------------------+-----------------+
 * GPIO  FSMC     NOTE |GPIO  FSMC    NOTE|GPIO  FSMC    NOTE|GPIO  FSMC   NOTE|
 *---------------------+------------------+------------------+-----------------+
 * PD0   FSMC_D2       |PE0   FSMC_NBL0   |PF0   FSMC_A0     |PG0   FSMC_A10   |
 * PD1   FSMC_D3       |PE1   FSMC_NBL1   |PF1   FSMC_A1     |PG1   FSMC_A11   |
 *                     |                  |PF2   FSMC_A2     |PG2   FSMC_A12   |
 *                     |                  |PF3   FSMC_A3     |PG3   FSMC_A13   |
 * PD4   FSMC_NOE   2  |                  |PF4   FSMC_A4     |PG4   FSMC_A14   |
 * PD5   FSMC_NWE      |                  |PF5   FSMC_A5     |PG5   FSMC_A15   |
 *                     |                  |                  |                 |
 * PD7   FSMC_NE1/NCE2 |PE7   FSMC_D4     |                  |                 |
 * PD8   FSMC_D13   1  |PE8   FSMC_D5     |                  |                 |
 * PD9   FSMC_D14   1  |PE9   FSMC_D6     |                  |                 |
 * PD10  FSMC_D15   1  |PE10  FSMC_D7     |                  |                 |
 * PD11  FSMC_A16   1  |PE11  FSMC_D8     |                  |                 |
 * PD12  FSMC_A17      |PE12  FSMC_D9     |PF12  FSMC_A6     |                 |
 *                     |PE13  FSMC_D10    |PF13  FSMC_A7     |                 |
 * PD14  FSMC_D0       |PE14  FSMC_D11    |PF14  FSMC_A8     |                 |
 * PD15  FSMC_D1       |PE15  FSMC_D12    |PF15  FSMC_A9     |                 |
 *---------------------+------------------+------------------+-----------------+
 *
 * NOTES:
 *  (1) Shared with USART3:   PD8=USART3_TX PD9=USART3_RX PD11=USART3_CTS
 *                            PD12=USART3_RTS
 *  (2) Shared with USB:      PD4=USB_HS_FAULT
 */

/* SRAM GPIO configuration */

static const uint32_t g_sramconfig[] =
{
  /* Address configuration:  FSMC_A0-FSMC_A17 */

  GPIO_FSMC_A0,  GPIO_FSMC_A1 , GPIO_FSMC_A2,  GPIO_FSMC_A3,  GPIO_FSMC_A4 , GPIO_FSMC_A5,
  GPIO_FSMC_A6,  GPIO_FSMC_A7,  GPIO_FSMC_A8,  GPIO_FSMC_A9,  GPIO_FSMC_A10, GPIO_FSMC_A11,
  GPIO_FSMC_A12, GPIO_FSMC_A13, GPIO_FSMC_A14, GPIO_FSMC_A15, GPIO_FSMC_A16, GPIO_FSMC_A17,

  /* Data Configuration: FSMC_D0-FSMC_D15 */

  GPIO_FSMC_D0,  GPIO_FSMC_D1 , GPIO_FSMC_D2,  GPIO_FSMC_D3,  GPIO_FSMC_D4 , GPIO_FSMC_D5,
  GPIO_FSMC_D6,  GPIO_FSMC_D7,  GPIO_FSMC_D8,  GPIO_FSMC_D9,  GPIO_FSMC_D10, GPIO_FSMC_D11,
  GPIO_FSMC_D12, GPIO_FSMC_D13, GPIO_FSMC_D14, GPIO_FSMC_D15

  /* Control Signals:
   *
   *  /CS  = PD7, FSMC_NE1
   *  /OE  = PD4, FSMC_NOE
   *  /WE  = PD5, FSMC_NWE
   *  /BHE = PE0, FSMC_NBL0
   *  /BHL = PE1, PSMC_NBL1
   */

  GPIO_FSMC_NE1, GPIO_FSMC_NOE, GPIO_FSMC_NWE, GPIO_FSMC_NBL0, GPIO_FSMC_NBL1
};
#define NSRAM_CONFIG (sizeof(g_sramconfig)/sizeof(uint32_t))

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_enablefsmc
 *
 * Description:
 *   Enable clocking to the FSMC module
 *
 ************************************************************************************/

static void stm32_enablefsmc(void)
{
  uint32_t regval;

  /* Enable AHB clocking to the FSMC */

  regval  = getreg32( STM32_RCC_AHB3ENR);
  regval |= RCC_AHB3ENR_FSMCEN;
  putreg32(regval, STM32_RCC_AHB3ENR);
}

/************************************************************************************
 * Name: stm32_sramgpios
 *
 * Description:
 *   Configure SRAM GPIO pins
 *
 ************************************************************************************/

static void stm32_sramgpios(void)
{
  int i;

  /* Configure SRAM GPIOs */

  for (i = 0; i < NSRAM_CONFIG; i++)
    {
      stm32_configgpio(g_sramconfig[i]);
    }
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_stram_configure
 *
 * Description:
 *   Initialize to access external SRAM.  SRAM will be visible at the FSMC Bank
 *   NOR/SRAM2 base address (0x64000000)
 *
 *   General transaction rules.  The requested AHB transaction data size can be 8-,
 *   16- or 32-bit wide whereas the SRAM has a fixed 16-bit data width. Some simple
 *   transaction rules must be followed:
 *
 *   Case 1: AHB transaction width and SRAM data width are equal
 *     There is no issue in this case.
 *   Case 2: AHB transaction size is greater than the memory size
 *     In this case, the FSMC splits the AHB transaction into smaller consecutive
 *     memory accesses in order to meet the external data width.
 *   Case 3: AHB transaction size is smaller than the memory size.
 *     SRAM supports the byte select feature.
 *     a) FSMC allows write transactions accessing the right data through its
 *        byte lanes (NBL[1:0])
 *     b) Read transactions are allowed (the controller reads the entire memory
 *        word and uses the needed byte only). The NBL[1:0] are always kept low
 *        during read transactions.
 *
 ************************************************************************************/

void stm32_stram_configure(void)
{
  /* Configure GPIO pins */

  stm32_extmemgpios(g_sramconfig, NSRAM_CONFIG);  /* SRAM-specific control lines */

  /* Enable AHB clocking to the FSMC */

  stm32_enablefsmc();

  /* Bank1 NOR/SRAM control register configuration
   *
   *   Bank enable        : Not yet
   *   Data address mux   : Disabled
   *   Memory Type        : PSRAM
   *   Data bus width     : 16-bits
   *   Flash access       : Disabled
   *   Burst access mode  : Disabled
   *   Polarity           : Low
   *   Wrapped burst mode : Disabled
   *   Write timing       : Before state
   *   Write enable       : Yes
   *   Wait signal        : Disabled
   *   Extended mode      : Disabled
   *   Asynchronous wait  : Disabled
   *   Write burst        : Disabled
   */

  putreg32((FSMC_BCR_PSRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN), STM32_FSMC_BCR2);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32((FSMC_BTR_ADDSET(SRAM_ADDRESS_SETUP_TIME) | FSMC_BTR_ADDHLD(SRAM_ADDRESS_HOLD_TIME) |
            FSMC_BTR_DATAST(SRAM_DATA_SETUP_TIME)    | FSMC_BTR_BUSTURN(SRAM_BUS_TURNAROUND_DURATION) |
            FSMC_BTR_CLKDIV(SRAM_CLK_DIVISION)       | FSMC_BTR_DATLAT(SRAM_DATA_LATENCY) |
            FSMC_BTR_ACCMODA),
           STM32_FSMC_BTR2);

  /* Bank1 NOR/SRAM timing register for write configuration, if extended mode is used */

  putreg32(0xffffffff, STM32_FSMC_BWTR2);  /* Extended mode not used */

  /* Enable the bank */

  putreg32((FSMC_BCR_MBKEN | FSMC_BCR_PSRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN), STM32_FSMC_BCR2);
}

#endif /* CONFIG_STM32_FSMC */
