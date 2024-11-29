/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_dsn.c
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
#include <unistd.h>
#include <nuttx/signal.h>
#include "mpfs_dsn.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_SCBCTRL_BASE  0x37020000ul
#define SERVICES_CR_OFFSET 0x50
#define SERVICES_SR_OFFSET 0x54

#define SERVICES_CR        (MPFS_SCBCTRL_BASE + SERVICES_CR_OFFSET)
#define SERVICES_SR        (MPFS_SCBCTRL_BASE + SERVICES_SR_OFFSET)

/* Command bits */

#define SCBCTRL_SERVICESCR_REQ  (1 << 0)

/* Status bits */

#define SCBCTRL_SERVICESSR_BUSY (1 << 1)

/* 2kB long mailbox. */

#define MSS_SCBMAILBOX     0x37020800ul

/* Retry count */

#define RETRIES            500

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_read_dsn
 *
 * Description:
 *   Read n bytes of the device serial number. Full serial number is 16 bytes
 *
 * Parameters:
 *   dsn - A pointer to the destination buffer
 *   len - Number of bytes to read
 *
 * Returned Value:
 *   Number of bytes read, -ETIMEDOUT on error
 *
 ****************************************************************************/

int mpfs_read_dsn(uint8_t *dsn, size_t len)
{
  uint32_t reg;
  uintptr_t p = MSS_SCBMAILBOX;
  irqstate_t flags = enter_critical_section();
  unsigned retries = RETRIES;

  /* Wait until the system controller is not busy.
   * Read the SN inside critical section, just in case someone else is
   * using the system controller services
   */

  while ((getreg32(SERVICES_SR) & SCBCTRL_SERVICESSR_BUSY) && --retries > 0)
    {
      leave_critical_section(flags);
      nxsig_usleep(1000);
      flags = enter_critical_section();
    }

  if (retries == 0)
    {
      goto out;
    }

  /* Read at max MPFS_DSN_LENGTH bytes, set the rest to 0 */

  if (len > MPFS_DSN_LENGTH)
    {
      len = MPFS_DSN_LENGTH;
    }

  /* Command: bits 0 to 6 is the opcode, bits 7 to 15 is the Mailbox
   * offset. In this case, opcode == 0 and offset == 0.
   */

  putreg32(SCBCTRL_SERVICESCR_REQ, SERVICES_CR);

  /* Wait until the system controller has started processing the command */

  retries = RETRIES;
  do
    {
      reg = getreg32(SERVICES_CR);
    }
  while ((reg & SCBCTRL_SERVICESCR_REQ) && --retries);

  if (retries == 0)
    {
      goto out;
    }

  /* Wait for the completion of the command */

  retries = RETRIES;
  do
    {
      reg = getreg32(SERVICES_SR);
    }
  while ((reg & SCBCTRL_SERVICESSR_BUSY) && --retries);

  if (retries == 0)
    {
      goto out;
    }

  /* Read the bytes of serial from service mailbox */

  for (uint8_t i = 0; i < len; i++)
    {
      dsn[i] = getreg8(p++);
    }

out:

  leave_critical_section(flags);

  return retries > 0 ? len : -ETIMEDOUT;
}
