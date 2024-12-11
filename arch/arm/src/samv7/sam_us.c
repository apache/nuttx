/****************************************************************************
 * arch/arm/src/samv7/sam_us.c
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

#include <nuttx/config.h>

#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <arch/barriers.h>
#include <arch/samv7/chip.h>

#include "arm_internal.h"

#include "hardware/sam_memorymap.h"

#include "sam_eefc.h"
#include "sam_us.h"

#ifdef CONFIG_SAMV7_USER_SIGNATURE

#define SAMV7_US_START      (SAM_INTFLASH_BASE)
#define SAMV7_US_SIZE       (512)
#define SAMV7_US_PAGE_WORDS (SAMV7_US_SIZE / sizeof(uint32_t))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_page_buffer[SAMV7_US_PAGE_WORDS];
static mutex_t g_page_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_erase_user_signature
 *
 * Description:
 *   Erases user signature page.
 *
 * Returned Value:
 *   Zero on success, negated errno value on error.
 *
 ****************************************************************************/

int sam_erase_user_signature(void)
{
  int ret;

  ret = sam_eefc_command(EEFC_FCR_FCMD_EUS, 0);
  if (ret < 0)
    {
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: sam_write_user_signature
 *
 * Description:
 *   Writes data to user signature page.
 *
 * Input Parameters:
 *   buffer  - The buffer to be written to user signature.
 *   buflen  - Number of bytes to be written.
 *
 * Returned Value:
 *   Number of written bytes on success, negated errno on error.
 *
 ****************************************************************************/

int sam_write_user_signature(void *buffer, size_t buflen)
{
  uint32_t *dest;
  int ret;

  if (buflen > SAMV7_US_SIZE)
    {
      return -ENOMEM;
    }

  /* Ensure single access to gloval g_page_buffer */

  ret = nxmutex_lock(&g_page_lock);
  if (ret < 0)
    {
      return ret;
    }

  memcpy((uint8_t *)g_page_buffer, buffer, buflen);

  /* Reference manual 22.4.3.9: Write the full page, at any page address,
   * within the internal memory area address space.
   */

  dest = (uint32_t *)SAMV7_US_START;
  for (int i = 0; i < SAMV7_US_PAGE_WORDS; i++)
    {
      *dest++ = g_page_buffer[i];

#ifdef CONFIG_ARMV7M_DCACHE_WRITETHROUGH
      UP_DMB();
#endif
    }

  /* Flush the data cache to memory */

  up_clean_dcache(SAMV7_US_START, SAMV7_US_START + SAMV7_US_SIZE);

  /* EEFC_FCR_FARG does not have any affect for user signature,
   * therefore second argument can be zero.
   */

  ret = sam_eefc_command(EEFC_FCR_FCMD_WUS, 0);
  if (ret < 0)
    {
      nxmutex_unlock(&g_page_lock);
      return ret;
    }

  nxmutex_unlock(&g_page_lock);
  return buflen;
}

/****************************************************************************
 * Name: sam_get_user_signature
 *
 * Description:
 *   Get bytes from user signature area.
 *
 * Input Parameters:
 *   buffer  - The buffer to store user signature.
 *   buflen  - Number of bytes to be read.
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/

int sam_read_user_signature(void *buffer, size_t buflen)
{
  size_t nwords;
  int ret;

  /* Ensure single access to gloval g_page_buffer */

  ret = nxmutex_lock(&g_page_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* sam_eefc_readsequence requires read length in bit words. */

  nwords = (buflen + sizeof(uint32_t)) / sizeof(uint32_t);
  sam_eefc_readsequence(FCMD_STUS, FCMD_SPUS, g_page_buffer, nwords);

  /* Copy local buffer to void *buffer provided by the user. */

  memcpy(buffer, (uint8_t *)g_page_buffer, buflen);

  nxmutex_unlock(&g_page_lock);
  return buflen;
}

#endif /* CONFIG_SAMV7_USER_SIGNATURE */
