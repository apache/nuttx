/****************************************************************************
 * arch/arm/src/moxart/moxart_16550.c
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
#include <stdio.h>
#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/mxser.h>

#include "arm.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void uart_decodeirq(int irq, void *context)
{
  int i;
  uint32_t status;
  static int os = 0;

  status = *((volatile uart_addrwidth_t *)CONFIG_UART_MOXA_IRQ_STATUS_REG);

  if ((status & 0x3f) == 0x3f)
    {
      return;
    }

  i = 0;
  do
    {
      if (!(status & 0x1))
        {
          irq_dispatch(VIRQ_START + i, context);
        }

      status >>= 1;
    }
  while (++i <= 4);
}

#ifdef CONFIG_SERIAL_UART_ARCH_IOCTL
int uart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  struct u16550_s   *priv  = (struct u16550_s *)dev->priv;
  int                ret   = -ENOTTY;
  uint32_t           vmode;
  unsigned int       opmode;
  int                bitm_off;

  /* TODO: calculate bit offset from UART_BASE address.
   *  E.g.:
   *       0x9820_0000 -> 0
   *       0x9820_0020 -> 1
   *       0x9820_0040 -> 2
   */

  /* HARD: coded value for UART1 */

  bitm_off = 1;

  switch (cmd)
    {
      case MOXA_SET_OP_MODE:
        {
          irqstate_t flags;
          opmode = *(unsigned long *)arg;

          /* Check for input data */

          if (opmode & ~OP_MODE_MASK)
            {
              ret = -EINVAL;
              break;
            }

          flags = enter_critical_section();

          /* Update mode register with requested mode */

          vmode = getreg32(CONFIG_UART_MOXA_MODE_REG);
          putreg32(vmode & ~(OP_MODE_MASK << 2 * bitm_off),
                   CONFIG_UART_MOXA_MODE_REG);
          vmode = opmode << 2 * bitm_off;
          putreg32(getreg32(CONFIG_UART_MOXA_MODE_REG) | vmode,
                   CONFIG_UART_MOXA_MODE_REG);

          leave_critical_section(flags);
          ret = OK;
          break;
        }

      case MOXA_GET_OP_MODE:
        {
          irqstate_t flags;
          flags = enter_critical_section();

          /* Read from mode register */

          opmode = (getreg32(CONFIG_UART_MOXA_MODE_REG) >> 2 * bitm_off) &
                    OP_MODE_MASK;

          leave_critical_section(flags);
          *(unsigned long *)arg = opmode;
          ret = OK;
          break;
        }
    }

  return ret;
}
#endif
