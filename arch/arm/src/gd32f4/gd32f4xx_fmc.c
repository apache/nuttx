/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_fmc.c
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
#include <nuttx/semaphore.h>

#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "gd32f4xx_fmc.h"
#include "gd32f4xx.h"

#if defined(CONFIG_GD32F4_FLASH_CONFIG_DEFAULTT)
#  warning "Default Flash Configuration Used - See Override Flash Size Designator"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_gd32_fmc_sem = SEM_INITIALIZER(1);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_fmc_sem_lock
 *
 * Description:
 *   Lock semaphore
 *
 * Return Value:
 *   Zero(OK)  - On success
 *   EINVAL    - Invalid attempt to get the semaphore
 *   ECANCELED - May be returned if the thread is canceled while waiting
 *
 ****************************************************************************/

static int gd32_fmc_sem_lock(void)
{
  return nxsem_wait_uninterruptible(&g_gd32_fmc_sem);
}

/****************************************************************************
 * Name: gd32_fmc_sem_unlock
 *
 * Description:
 *   Lock semaphore
 *
 ****************************************************************************/

static void gd32_fmc_sem_unlock(void)
{
  nxsem_post(&g_gd32_fmc_sem);
}

/****************************************************************************
 * Name: gd32_fmc_state_get
 *
 * Description:
 *   Get the FMC state
 *
 * Returned Value:
 *  State of FMC
 *
 ****************************************************************************/

static gd32_fmc_state_enum gd32_fmc_state_get(void)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;

  if (getreg32(GD32_FMC_STAT) & FMC_STAT_BUSY)
    {
      fmc_state = FMC_BUSY;
    }
  else if (getreg32(GD32_FMC_STAT) & FMC_STAT_RDDERR)
    {
      fmc_state = FMC_RDDERR;
    }
  else if (getreg32(GD32_FMC_STAT) & FMC_STAT_PGSERR)
    {
      fmc_state = FMC_PGSERR;
    }
  else if (getreg32(GD32_FMC_STAT) & FMC_STAT_PGMERR)
    {
      fmc_state = FMC_PGMERR;
    }
  else if (getreg32(GD32_FMC_STAT) & FMC_STAT_WPERR)
    {
      fmc_state = FMC_WPERR;
    }
  else if (getreg32(GD32_FMC_STAT) & FMC_STAT_OPERR)
    {
      fmc_state = FMC_OPERR;
    }
  else
    {
      fmc_state = FMC_READY;
    }

  /* Return the FMC state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_ready_wait
 *
 * Description:
 *   Check whether FMC is ready or not
 *
 * Returned Value:
 *  State of FMC
 *
 ****************************************************************************/

static gd32_fmc_state_enum gd32_fmc_ready_wait(uint32_t timeout)
{
  gd32_fmc_state_enum fmc_state = FMC_BUSY;

  /* Wait for FMC ready */

  do
    {
      /* Get FMC state */

      fmc_state = gd32_fmc_state_get();
      timeout--;
    }
  while ((FMC_BUSY == fmc_state) && (timeout > 0));

  if (0 == timeout)
    {
      fmc_state = FMC_TOERR;
    }

  /* Return the FMC state */

  return fmc_state;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_fmc_wscnt_set
 *
 * Description:
 *   Set the wait state counter value
 *
 * Parameters:
 *   wscnt - Wait state counter value
 *
 ****************************************************************************/

void gd32_fmc_wscnt_set(uint32_t wscnt)
{
  uint32_t regval;

  regval = getreg32(GD32_FMC_WS);

  /* set the wait state counter value */

  regval &= ~FMC_WS_WSCNT_MASK;
  regval |= wscnt;
  putreg32(regval, GD32_FMC_WS);
}

/****************************************************************************
 * Name: gd32_fmc_unlock
 *
 * Description:
 *   Unlock the main FMC operation
 *
 ****************************************************************************/

int gd32_fmc_unlock(void)
{
  int ret;

  ret = gd32_fmc_sem_lock();
  if (ret < 0)
    {
      return ret;
    }

  if (getreg32(GD32_FMC_CTL) & FMC_CTL_LK)
    {
      /* Write the FMC key */

      putreg32(FMC_UNLOCK_KEY0, GD32_FMC_KEY);
      putreg32(FMC_UNLOCK_KEY1, GD32_FMC_KEY);
    }

  gd32_fmc_sem_unlock();

  return ret;
}

/****************************************************************************
 * Name: gd32_fmc_lock
 *
 * Description:
 *   Lock the main FMC operation
 *
 ****************************************************************************/

int gd32_fmc_lock(void)
{
  int ret;

  ret = gd32_fmc_sem_lock();
  if (ret < 0)
    {
      return ret;
    }

  /* Set the LK bit */

  modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_LK);

  gd32_fmc_sem_unlock();

  return ret;
}

/****************************************************************************
 * Name: gd32_fmc_sector_erase
 *
 * Description:
 *   Erase sector
 *
 * Parameters:
 *   fmc_sector - Select the sector to erase
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_sector_erase(uint32_t fmc_sector)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Start sector erase */

      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_SER);
      modifyreg32(GD32_FMC_CTL, FMC_CTL_SN_MASK, fmc_sector);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_START);

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the SER bit */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_SER, 0);
      modifyreg32(GD32_FMC_CTL, FMC_CTL_SN_MASK, 0);
    }

  /* return the FMC state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_mass_erase
 *
 * Description:
 *   Erase whole chip
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_mass_erase(void)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Start whole chip erase */

      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_MER0);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_MER1);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_START);

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the MER bits */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_MER0, 0);
      modifyreg32(GD32_FMC_CTL, FMC_CTL_MER1, 0);
    }

  /* Return the fmc state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_bank0_erase
 *
 * Description:
 *   Erase whole bank0
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_bank0_erase(void)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Start FMC bank0 erase */

      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_MER0);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_START);

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the MER0 bit */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_MER0, 0);
    }

  /* Return the fmc state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_bank1_erase
 *
 * Description:
 *   Erase whole bank1
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_bank1_erase(void)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Start FMC bank1 erase */

      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_MER1);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_START);

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the MER1 bit */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_MER1, 0);
    }

  /* Return the fmc state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_word_program
 *
 * Description:
 *   Program a word at the corresponding address
 *
 * Parameters:
 *   address - Address to program
 *   data - Word to program(0x00000000 - 0xFFFFFFFF)
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_word_program(uint32_t address, uint32_t data)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Set the PG bit to start program */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_PSZ_MASK, FMC_CTL_PSZ_WORD);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_PG);

      putreg32(data, address);

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the PG bit */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_PG, 0);
    }

  /* Return the FMC state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_halfword_program
 *
 * Description:
 *   Program a half word at the corresponding address
 *
 * Parameters:
 *   address - Address to program
 *   data - Word to program(0x0000 - 0xFFFF)
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_halfword_program(uint32_t address,
                                              uint16_t data)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Set the PG bit to start program */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_PSZ_MASK, FMC_CTL_PSZ_HALF_WORD);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_PG);

      putreg16(data, address);

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the PG bit */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_PG, 0);
    }

  /* Return the FMC state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_fmc_byte_program
 *
 * Description:
 *   Program a byte at the corresponding address
 *
 * Parameters:
 *   address - Address to program
 *   data - Byte to program(0x00 - 0xFF)
 *
 * Return Value:
 *    State of FMC
 *
 ****************************************************************************/

gd32_fmc_state_enum gd32_fmc_byte_program(uint32_t address, uint8_t data)
{
  gd32_fmc_state_enum fmc_state = FMC_READY;

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      /* Set the PG bit to start program */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_PSZ_MASK, FMC_CTL_PSZ_BYTE);
      modifyreg32(GD32_FMC_CTL, 0, FMC_CTL_PG);

      putreg8(data, address);

      /* Wait for the FMC ready */

      fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

      /* Reset the PG bit */

      modifyreg32(GD32_FMC_CTL, FMC_CTL_PG, 0);
    }

  /* Return the FMC state */

  return fmc_state;
}

/****************************************************************************
 * Name: gd32_ob_unlock
 *
 * Description:
 *   Unlock the option byte operation
 *
 ****************************************************************************/

void gd32_ob_unlock(void)
{
  if (getreg32(GD32_FMC_OBCTL0) & FMC_OBCTL0_OB_LK)
    {
      /* Write the FMC key */

      putreg32(FMC_OB_UNLOCK_KEY0, GD32_FMC_OBKEY);
      putreg32(FMC_OB_UNLOCK_KEY1, GD32_FMC_OBKEY);
    }
}

/****************************************************************************
 * Name: gd32_ob_lock
 *
 * Description:
 *   Lock the option byte operation
 *
 ****************************************************************************/

void gd32_ob_lock(void)
{
  /* Reset the OB_LK bit */

  modifyreg32(GD32_FMC_OBCTL0, 0, FMC_OBCTL0_OB_LK);
}

/****************************************************************************
 * Name: gd32_ob_start
 *
 * Description:
 *   Send option byte change command
 *
 ****************************************************************************/

void gd32_ob_start(void)
{
  /* Set the OB_START bit in OBCTL0 register */

  modifyreg32(GD32_FMC_OBCTL0, 0, FMC_OBCTL0_OB_START);
}

/****************************************************************************
 * Name: gd32_ob_write_protection_enable
 *
 * Description:
 *   Enable write protection
 *
 * Parameters:
 *   ob_wp - Specify sector to be write protected
 *
 ****************************************************************************/

int gd32_ob_write_protection_enable(uint32_t ob_wp)
{
  uint32_t regval0;
  uint32_t regval1;

  regval0 = getreg32(GD32_FMC_OBCTL0);
  regval1 = getreg32(GD32_FMC_OBCTL1);

  gd32_fmc_state_enum fmc_state = FMC_READY;
  if (regval0 & FMC_OBCTL0_DRP)
    {
      return -1;
    }

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      regval0 &= (~(uint32_t)ob_wp << 16U);
      regval1 &= (~(ob_wp & 0xffff0000u));
      putreg32(regval0, GD32_FMC_OBCTL0);
      putreg32(regval1, GD32_FMC_OBCTL1);

      return 0;
    }
  else
    {
      return -1;
    }
}

/****************************************************************************
 * Name: gd32_ob_write_protection_disable
 *
 * Description:
 *   Disable write protection
 *
 * Parameters:
 *   ob_wp - Specify sector to be write protected
 *
 ****************************************************************************/

int gd32_ob_write_protection_disable(uint32_t ob_wp)
{
  uint32_t regval0;
  uint32_t regval1;

  regval0 = getreg32(GD32_FMC_OBCTL0);
  regval1 = getreg32(GD32_FMC_OBCTL1);

  gd32_fmc_state_enum fmc_state = FMC_READY;
  if (regval0 & FMC_OBCTL0_DRP)
    {
      return -1;
    }

  /* Wait for the FMC ready */

  fmc_state = gd32_fmc_ready_wait(FMC_TIMEOUT_COUNT);

  if (FMC_READY == fmc_state)
    {
      regval0 |= ((uint32_t)(ob_wp << 16U));
      regval1 |= (ob_wp & 0xffff0000u);
      putreg32(regval0, GD32_FMC_OBCTL0);
      putreg32(regval1, GD32_FMC_OBCTL1);

      return 0;
    }
  else
    {
      return -1;
    }
}

/****************************************************************************
 * Name: gd32_fmc_flag_clear
 *
 * Description:
 *   Clear the FMC pending flag
 *
 * Parameters:
 *   fmc_flag - FMC flag
 *
 ****************************************************************************/

void gd32_fmc_flag_clear(uint32_t fmc_flag)
{
  /* Clear the flags */

  modifyreg32(GD32_FMC_STAT, fmc_flag, 0);
}
