/************************************************************************************
 * configs/stm3210e-eval/src/up_extmem.c
 * arch/arm/src/board/up_extmem.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_fsmc.h"
#include "stm32_gpio.h"
#include "stm32_internal.h"
#include "stm3210e-internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_STM32_FSMC
#  warning "FSMC is not enabled"
#endif

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* 512Kx16 SRAM is connected to bank2 of the FSMC interface and both 8- and 16-bit
 * accesses are allowed by BLN0 and BLN1 connected to BLE and BHE of SRAM,
 * respectively.
 *
 * Pin Usage (per schematic)
 *                         FLASH   SRAM    NAND
 *   D[0..15]              [0..15] [0..15] [0..7]
 *   A[0..23]              [0..22] [0..18] [16,17]
 *   PSMC_NE3   PG10  OUT  ~CE     ---     ---
 *   PSMC_NBL0  PE0   OUT  ~BLE    ---     ---
 *   PSMC_NBL1  PE1   OUT  ~BHE    ---     ---
 *   PSMC_NE2   PG9   OUT  ---     ~E      ---
 *   PSMC_NWE   PD5   OUT  ~WE     ~W      ~W
 *   PSMC_NOE   PD4   OUT  ~OE     ~G      ~R
 *   PSMC_NWAIT PD6   IN   ---     R~B     ---
 *   PSMC_INT2  PG6*  IN   ---     ---     R~B
 *
 *   *JP7 will switch to PD6
 */

/* It would be much more efficient to brute force these all into the
 * the appropriate registers.  Just a little tricky.
 */

/* GPIO configurations common to SRAM and NOR Flash */

static const uint16 g_commonconfig[] =
{
  /* A0... A18 */

  GPIO_NPS_A0,  GPIO_NPS_A1,  GPIO_NPS_A2,  GPIO_NPS_A3,
  GPIO_NPS_A4,  GPIO_NPS_A5,  GPIO_NPS_A6,  GPIO_NPS_A7,
  GPIO_NPS_A8,  GPIO_NPS_A9,  GPIO_NPS_A10, GPIO_NPS_A11,
  GPIO_NPS_A12, GPIO_NPS_A13, GPIO_NPS_A14, GPIO_NPS_A15,
  GPIO_NPS_A16, GPIO_NPS_A17, GPIO_NPS_A18,

  /* D0... D15 */

  GPIO_NPS_D0,  GPIO_NPS_D1,  GPIO_NPS_D2,  GPIO_NPS_D3,
  GPIO_NPS_D4,  GPIO_NPS_D5,  GPIO_NPS_D6,  GPIO_NPS_D7,
  GPIO_NPS_D8,  GPIO_NPS_D9,  GPIO_NPS_D10, GPIO_NPS_D11,
  GPIO_NPS_D12, GPIO_NPS_D13, GPIO_NPS_D14, GPIO_NPS_D15,

  /* NOE, NWE, NE3  */

  GPIO_NPS_NOE, GPIO_NPS_NWE
};
#define NCOMMON_CONFIG (sizeof(g_commonconfig)/sizeof(uint16))

/* GPIO configurations unique to SRAM  */

static const uint16 g_sramconfig[] =
{
  /* NE3, NBL0, NBL1,  */

  GPIO_NPS_NE3, GPIO_NPS_NBL0, GPIO_NPS_NBL1
};
#define NSRAM_CONFIG (sizeof(g_sramconfig)/sizeof(uint16))

/* GPIO configurations unique to NOR Flash  */

static const uint16 g_norconfig[] =
{
  /* A19... A22 */

  GPIO_NPS_A19, GPIO_NPS_A20, GPIO_NPS_A21, GPIO_NPS_A22,

  /* NE2  */

  GPIO_NPS_NE2
};
#define NNOR_CONFIG (sizeof(g_norconfig)/sizeof(uint16))

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for NOR or SRAM
 *
 ************************************************************************************/

static void stm32_extmemgpios(const uint16 *gpios, int ngpios)
{
  int i;

  /* Configure GPIOs */

  for (i = 0; i < ngpios; i++)
    {
      stm32_configgpio(gpios[i]);
    }
}

/************************************************************************************
 * Name: stm32_savegpios
 *
 * Description:
 *  Save current GPIOs that will used by external memory configurations
 *
 ************************************************************************************/

static void stm32_savegpios(struct extmem_save_s *save)
{
  DEBUGASSERT(save != NULL);
  save->gpiod_crl = getreg32(STM32_GPIOE_CRL);
  save->gpiod_crh = getreg32(STM32_GPIOE_CRH);
  save->gpioe_crl = getreg32(STM32_GPIOD_CRL);
  save->gpioe_crh = getreg32(STM32_GPIOD_CRH);
  save->gpiof_crl = getreg32(STM32_GPIOF_CRL);
  save->gpiof_crh = getreg32(STM32_GPIOF_CRH);
  save->gpiog_crl = getreg32(STM32_GPIOG_CRL);
  save->gpiog_crh = getreg32(STM32_GPIOG_CRH);
}

/************************************************************************************
 * Name: stm32_restoregpios
 *
 * Description:
 *  Restore GPIOs that were used by external memory configurations
 *
 ************************************************************************************/

static void stm32_restoregpios(struct extmem_save_s *restore)
{
  DEBUGASSERT(restore != NULL);
  putreg32(restore->gpiod_crl, STM32_GPIOE_CRL);
  putreg32(restore->gpiod_crh, STM32_GPIOE_CRH);
  putreg32(restore->gpioe_crl, STM32_GPIOD_CRL);
  putreg32(restore->gpioe_crh, STM32_GPIOD_CRH);
  putreg32(restore->gpiof_crl, STM32_GPIOF_CRL);
  putreg32(restore->gpiof_crh, STM32_GPIOF_CRH);
  putreg32(restore->gpiog_crl, STM32_GPIOG_CRL);
  putreg32(restore->gpiog_crh, STM32_GPIOG_CRH);
}

/************************************************************************************
 * Name: stm32_enableclocks
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ************************************************************************************/

static void  stm32_enableclocks(void)
{
  uint32 regval;

  /* Enable AHB clocking to the FSMC */

  regval  = getreg32( STM32_RCC_AHBENR);
  regval |= RCC_AHBENR_FSMCEN;
  putreg32(regval, STM32_RCC_AHBENR);
}

/************************************************************************************
 * Name: stm32_disableclocks
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ************************************************************************************/

static void  stm32_disableclocks(void)
{
  uint32 regval;

  /* Enable AHB clocking to the FSMC */

  regval  = getreg32( STM32_RCC_AHBENR);
  regval &= ~RCC_AHBENR_FSMCEN;
  putreg32(regval, STM32_RCC_AHBENR);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_selectnor
 *
 * Description:
 *   Initialize to access NOR flash
 *
 ************************************************************************************/

void stm32_selectnor(struct extmem_save_s *save)
{
  /* Save current GPIO state */

  stm32_savegpios(save);

  /* Configure new GPIO state */

  stm32_extmemgpios(g_commonconfig, NCOMMON_CONFIG);
  stm32_extmemgpios(g_sramconfig, NNOR_CONFIG);

  /* Enable AHB clocking to the FSMC */

  stm32_enableclocks();

  /* Bank1 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_NOR|FSMC_BCR_FACCEN|FSMC_BCR_MWID16|FSMC_BCR_WREN, STM32_FSMC_BCR2);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(3)|FSMC_BTR_ADDHLD(1)|FSMC_BTR_DATAST(6)|FSMC_BTR_BUSTRUN(1)|
           FSMC_BTR_CLKDIV(1)|FSMC_BTR_DATLAT(2)|FSMC_BTR_ACCMODB, STM32_FSMC_BTR2);

  putreg32(0x0fffffff, STM32_FSMC_BCR3);

  /* Enable the bank */

  putreg32(FSMC_BCR_MBKEN|FSMC_BCR_NOR|FSMC_BCR_FACCEN|FSMC_BCR_MWID16|FSMC_BCR_WREN, STM32_FSMC_BCR2);
}

/************************************************************************************
 * Name: stm32_deselectnor
 *
 * Description:
 *   Disable NOR FLASH
 *
 ************************************************************************************/

void stm32_deselectnor(struct extmem_save_s *restore)
{
  /* Restore registers to their power up settings */

  putreg32(0x000030d2, STM32_FSMC_BCR2);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(0x0fffffff, STM32_FSMC_BTR2);
 
  /* Disable AHB clocking to the FSMC */

  stm32_disableclocks();

  /* Restore GPIOs */

  stm32_restoregpios(restore);
}

/************************************************************************************
 * Name: stm32_selectsram
 *
 * Description:
 *   Initialize to access external SRAM
 *
 ************************************************************************************/

void stm32_selectsram(struct extmem_save_s *save)
{
  /* Save current GPIO state */

  stm32_savegpios(save);

  /* Configure new GPIO state */

  stm32_extmemgpios(g_commonconfig, NCOMMON_CONFIG);
  stm32_extmemgpios(g_norconfig, NSRAM_CONFIG);

  /* Enable AHB clocking to the FSMC */

  stm32_enableclocks();

  /* Bank1 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_MWID16|FSMC_BCR_WREN, STM32_FSMC_BCR3);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(1)|FSMC_BTR_ADDHLD(1)|FSMC_BTR_DATAST(3)|FSMC_BTR_BUSTRUN(1)|
           FSMC_BTR_CLKDIV(1)|FSMC_BTR_DATLAT(2)|FSMC_BTR_ACCMODA, STM32_FSMC_BTR3);

  putreg32(0xffffffff, STM32_FSMC_BCR3);

  /* Enable the bank */

  putreg32(FSMC_BCR_MBKEN|FSMC_BCR_MWID16|FSMC_BCR_WREN, STM32_FSMC_BCR3);
}

/************************************************************************************
 * Name: stm32_deselectsram
 *
 * Description:
 *   Disable NOR FLASH
 *
 ************************************************************************************/

void stm32_deselectsram(struct extmem_save_s *restore)
{
  /* Restore registers to their power up settings */

  putreg32(0x000030d2, STM32_FSMC_BCR3);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(0x0fffffff, STM32_FSMC_BTR3);
 
  /* Disable AHB clocking to the FSMC */

  stm32_disableclocks();

  /* Restore GPIOs */

  stm32_restoregpios(restore);
}


