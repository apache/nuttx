/****************************************************************************
 * arch/arm/src/moxart/moxart_irq.c
 * Driver for MoxaRT IRQ controller
 *
 *   Copyright (C) 2015, 2016 Gregory Nutt. All rights reserved.
 *   Author: Anton D. Kachalov <mouse@mayc.ru>
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
#include "up_arch.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void uart_decodeirq(int irq, FAR void *context)
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
          putreg32(vmode & ~(OP_MODE_MASK << 2 * bitm_off), CONFIG_UART_MOXA_MODE_REG);
          vmode = opmode << 2 * bitm_off;
          putreg32(getreg32(CONFIG_UART_MOXA_MODE_REG) | vmode, CONFIG_UART_MOXA_MODE_REG);

          leave_critical_section(flags);
          ret = OK;
          break;
        }

      case MOXA_GET_OP_MODE:
        {
          irqstate_t flags;
          flags = enter_critical_section();

          /* Read from mode register */

          opmode = (getreg32(CONFIG_UART_MOXA_MODE_REG) >> 2 * bitm_off) & OP_MODE_MASK;

          leave_critical_section(flags);
          *(unsigned long *)arg = opmode;
          ret = OK;
          break;
        }
    }

  return ret;
}
#endif
