/****************************************************************************
 * arch/arm/src/rtl8730e/rtl8730e_serial.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include "arm_internal.h"
#include "hardware/rtl8730e_loguart.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTL8730E AmebaSmart LOGUART console driver.
 *
 * TX: CA32 uses path 4 (THR3 / LSR bit23).  Polling, no TX interrupt.
 *
 * RX: The physical LOGUART RX FIFO is owned by KM0 (LP), which handles
 * all keyboard input and routes commands with a '~' prefix to CA32 via
 * IPC.  CA32 never reads RBR directly; it receives complete command lines
 * through an IPC interrupt (GIC SPI 24).
 *
 * IPC LP→AP LOGUART channel (IPC_L2A_LOGUART_RX_SWITCH = 0):
 *   - IPCAP registers: 0x41000580 (already in the IO MMU section)
 *   - KM0 IPC shared memory: 0x2301fd00 (KM0 SRAM, needs its own section)
 *   - IPC_MSG_STRUCT slot index for IPC_LP_TO_AP(0x01) ch0:
 *       16 * ((0x01>>4)&0xf) + 8 * (0x01&0xf) + 0  =  8
 *     → message slot at 0x2301fd00 + 8*16 = 0x2301fd80
 *     → msg field (u32 at offset 4)  at 0x2301fd84
 *   - msg field holds the physical address of UART_LOG_BUF (also KM0 SRAM)
 *
 * UART_LOG_BUF layout (CONFIG_LONGER_CMD not set):
 *   u8 BufCount;          — number of command chars (no trailing newline)
 *   u8 UARTLogBuf[127];   — command string
 *
 * Usage: type commands directly (no prefix).  KM0 routes unprefixed input
 * to CA32 (AP_CPU_ID) via IPC when CONFIG_WHC_INTF_IPC is set (WiFi build).
 * '~' routes to KM4; '@' routes to KM0 itself.
 * NuttX appends '\n' before feeding the line to readline.
 */

/* IPCAP hardware registers (base 0x41000580, mapped as IO) */

#define RTL8730E_IPCAP_BASE        0x41000580UL
#define IPCAP_ISR                  (RTL8730E_IPCAP_BASE + 0x008)
#define IPCAP_IMR                  (RTL8730E_IPCAP_BASE + 0x00c)

/* LP->AP LOGUART channel 0 = ISR/IMR bit 24
 * (IPC_BIT_ISR_RX0_FULL_STATUS0)
 */

#define IPCAP_L2A_LOGUART_BIT      (1u << 24)

/* KM0 IPC shared memory (mapped as device/IO, no D-cache issues).
 * __km0_ipc_memory_start__ = ORIGIN(KM0_IPC_RAM) = 0x2301fd00 per
 * ameba_layout.ld.  Each IPC_MSG_STRUCT is 16 bytes {type,msg,len,rsvd}.
 * Slot index 8 for IPC_LP_TO_AP channel 0 (see block comment above).
 */

#define RTL8730E_KM0_IPC_BASE      0x2301fd00UL
#define IPC_L2A_LOGUART_SLOT_IDX   8
#define IPC_MSG_STRIDE             16u
#define IPC_MSG_MSG_OFF            4u   /* byte offset of 'msg' field */

#define IPC_L2A_LOGUART_SLOT \
  (RTL8730E_KM0_IPC_BASE + IPC_L2A_LOGUART_SLOT_IDX * IPC_MSG_STRIDE)

/* UART_LOG_BUF offsets */

#define UART_LOG_BUFCOUNT_OFF      0u
#define UART_LOG_BUFDATA_OFF       1u
#define UART_LOG_CMDLEN            127u

/* Buffer sizes */

#define LOGUART_RXBUFSIZE          256
#define LOGUART_TXBUFSIZE          256

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  loguart_setup(struct uart_dev_s *dev);
static void loguart_shutdown(struct uart_dev_s *dev);
static int  loguart_attach(struct uart_dev_s *dev);
static void loguart_detach(struct uart_dev_s *dev);
static int  loguart_ipc_interrupt(int irq, void *context, void *arg);
static int  loguart_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  loguart_receive(struct uart_dev_s *dev, unsigned int *status);
static void loguart_rxint(struct uart_dev_s *dev, bool enable);
static bool loguart_rxavailable(struct uart_dev_s *dev);
static void loguart_send(struct uart_dev_s *dev, int ch);
static void loguart_txint(struct uart_dev_s *dev, bool enable);
static bool loguart_txready(struct uart_dev_s *dev);
static bool loguart_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_loguart_ops =
{
  .setup       = loguart_setup,
  .shutdown    = loguart_shutdown,
  .attach      = loguart_attach,
  .detach      = loguart_detach,
  .ioctl       = loguart_ioctl,
  .receive     = loguart_receive,
  .rxint       = loguart_rxint,
  .rxavailable = loguart_rxavailable,
  .send        = loguart_send,
  .txint       = loguart_txint,
  .txready     = loguart_txready,
  .txempty     = loguart_txempty,
};

static char g_loguart_rxbuffer[LOGUART_RXBUFSIZE];
static char g_loguart_txbuffer[LOGUART_TXBUFSIZE];

static struct uart_dev_s g_loguart_port =
{
  .isconsole = true,
  .recv      =
  {
    .size    = LOGUART_RXBUFSIZE,
    .buffer  = g_loguart_rxbuffer,
  },
  .xmit      =
  {
    .size    = LOGUART_TXBUFSIZE,
    .buffer  = g_loguart_txbuffer,
  },
  .ops       = &g_loguart_ops,
};

/* Software RX FIFO filled by loguart_ipc_interrupt.
 * The IPC interrupt delivers a complete command line at once; we buffer
 * it here so that loguart_receive() can serve it one byte at a time.
 */

static char    g_ipc_rxbuf[UART_LOG_CMDLEN + 1];  /* +1 for appended '\n' */
static uint8_t g_ipc_rxcount;                     /* total chars available */
static uint8_t g_ipc_rxhead;                      /* next index to serve */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int loguart_setup(struct uart_dev_s *dev)
{
  UNUSED(dev);
  return OK;
}

static void loguart_shutdown(struct uart_dev_s *dev)
{
  loguart_rxint(dev, false);
}

static int loguart_attach(struct uart_dev_s *dev)
{
  int ret;

  /* Clear any stale pending IPC interrupt before enabling. */

  putreg32(IPCAP_L2A_LOGUART_BIT, IPCAP_ISR);

  ret = irq_attach(RTL8730E_IRQ_IPC_AP, loguart_ipc_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(RTL8730E_IRQ_IPC_AP);
    }

  return ret;
}

static void loguart_detach(struct uart_dev_s *dev)
{
  UNUSED(dev);
  up_disable_irq(RTL8730E_IRQ_IPC_AP);
  irq_detach(RTL8730E_IRQ_IPC_AP);
}

/* IPC LP→AP LOGUART interrupt handler.
 *
 * Called when KM0 (LP) forwards a '~'-prefixed command line to CA32.
 * Reads the UART_LOG_BUF from KM0 SRAM, copies chars into the software
 * RX FIFO, appends '\n', then drives uart_recvchars() to push them into
 * the NuttX serial layer.
 *
 * Protocol order matters: read the data BEFORE clearing the ISR bit,
 * because clearing the bit also frees KM0's TX channel (the hardware
 * clears LP's IPC_TX_DATA bit automatically when CA32 clears ISR).
 */

/* Weak hook called at the end of every IPC_AP interrupt to give WiFi a
 * chance to process its NP->AP channels (bits 16/17).
 * rtl8730e_wifi_init.c
 * provides the strong definition when CONFIG_RTL8730E_WIFI is enabled.
 * The weak stub here is a no-op so non-WiFi builds compile without change.
 */

void __attribute__((weak)) rtl8730e_wifi_ipc_dispatch(void *ipcx)
{
  UNUSED(ipcx);
}

static int loguart_ipc_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  uint32_t log_buf_addr;
  volatile uint8_t *log_buf;
  uint8_t count;
  uint8_t i;

  UNUSED(irq);
  UNUSED(context);

  /* LP→AP LOGUART channel: bit 24 of IPCAP_ISR. */

  if ((getreg32(IPCAP_ISR) & IPCAP_L2A_LOGUART_BIT) != 0)
    {
      /* Read IPC message slot: msg field holds UART_LOG_BUF
       * physical address.  KM0 SRAM is device/IO (non-cacheable).
       */

      log_buf_addr = getreg32(IPC_L2A_LOGUART_SLOT + IPC_MSG_MSG_OFF);

      /* Sanity-check: must be within KM0 SRAM */

      if (log_buf_addr >= 0x23000000ul && log_buf_addr < 0x23020000ul)
        {
          log_buf = (volatile uint8_t *)log_buf_addr;
          count   = log_buf[UART_LOG_BUFCOUNT_OFF];

          if (count > 0 && count <= UART_LOG_CMDLEN)
            {
              for (i = 0; i < count; i++)
                {
                  g_ipc_rxbuf[i] = (char)log_buf[UART_LOG_BUFDATA_OFF + i];
                }

              g_ipc_rxbuf[count] = '\n';
              g_ipc_rxhead        = 0;
              g_ipc_rxcount       = count + 1;
            }
        }

      /* Clear ISR bit (RW1CB). This also unblocks KM0's TX channel. */

      putreg32(IPCAP_L2A_LOGUART_BIT, IPCAP_ISR);

      if (g_ipc_rxcount > 0)
        {
          uart_recvchars(dev);
        }
    }

  /* Forward remaining IPC_AP bits (NP→AP WiFi channels 16/17) to the
   * WiFi IPC dispatcher.  This is a no-op weak stub in non-WiFi builds.
   */

  rtl8730e_wifi_ipc_dispatch((void *)RTL8730E_IPCAP_BASE);

  return OK;
}

static int loguart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  UNUSED(filep);
  UNUSED(cmd);
  UNUSED(arg);
  return -ENOTTY;
}

static int loguart_receive(struct uart_dev_s *dev, unsigned int *status)
{
  UNUSED(dev);
  *status = 0;
  return (int)(unsigned char)g_ipc_rxbuf[g_ipc_rxhead++];
}

/* Enable/disable the IPCAP LP→AP channel 0 interrupt mask. */

static void loguart_rxint(struct uart_dev_s *dev, bool enable)
{
  uint32_t imr;

  UNUSED(dev);

  imr = getreg32(IPCAP_IMR);
  if (enable)
    {
      imr |= IPCAP_L2A_LOGUART_BIT;
    }
  else
    {
      imr &= ~IPCAP_L2A_LOGUART_BIT;
    }

  putreg32(imr, IPCAP_IMR);
}

static bool loguart_rxavailable(struct uart_dev_s *dev)
{
  UNUSED(dev);
  return g_ipc_rxhead < g_ipc_rxcount;
}

static void loguart_send(struct uart_dev_s *dev, int ch)
{
  UNUSED(dev);

  while ((getreg32(RTL8730E_LOGUART_LSR) & LOGUART_LSR_TP4F_NOT_FULL) == 0)
    {
    }

  putreg32((uint32_t)(unsigned char)ch, RTL8730E_LOGUART_THR3);
}

/* TX is synchronous (loguart_send polls for FIFO space), so there is no
 * hardware TX interrupt.  Pump the transmit buffer when asked to enable.
 */

static void loguart_txint(struct uart_dev_s *dev, bool enable)
{
  irqstate_t flags;

  if (!enable)
    {
      return;
    }

  flags = enter_critical_section();
  uart_xmitchars(dev);
  leave_critical_section(flags);
}

static bool loguart_txready(struct uart_dev_s *dev)
{
  UNUSED(dev);
  return true;
}

static bool loguart_txempty(struct uart_dev_s *dev)
{
  UNUSED(dev);
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_putc(int ch)
{
  if (ch == '\n')
    {
      while ((getreg32(RTL8730E_LOGUART_LSR) &
              LOGUART_LSR_TP4F_NOT_FULL) == 0)
        {
        }

      putreg32((uint32_t)'\r', RTL8730E_LOGUART_THR3);
    }

  while ((getreg32(RTL8730E_LOGUART_LSR) & LOGUART_LSR_TP4F_NOT_FULL) == 0)
    {
    }

  putreg32((uint32_t)(unsigned char)ch, RTL8730E_LOGUART_THR3);
}

void arm_earlyserialinit(void)
{
}

void arm_serialinit(void)
{
  uart_register("/dev/console", &g_loguart_port);
}
