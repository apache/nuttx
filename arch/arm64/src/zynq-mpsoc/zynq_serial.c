/***************************************************************************
 * arch/arm64/src/zynq-mpsoc/zynq_serial.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/serial.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "zynq_serial.h"
#include "arm64_arch_timer.h"
#include "zynq_boot.h"
#include "arm64_gic.h"

#ifdef USE_SERIALDRIVER

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* UART0 Settings should be same as U-Boot Bootloader */

#ifndef CONFIG_UART0_BAUD
#  define CONFIG_UART0_BAUD 115200
#endif

#ifndef CONFIG_UART0_BITS
#  define CONFIG_UART0_BITS 8
#endif

#ifndef CONFIG_UART0_PARITY
#  define CONFIG_UART0_PARITY 0
#endif

#ifndef CONFIG_UART0_2STOP
#  define CONFIG_UART0_2STOP 0
#endif

#ifndef CONFIG_UART0_RXBUFSIZE
#  define CONFIG_UART0_RXBUFSIZE 256
#endif

#ifndef CONFIG_UART0_TXBUFSIZE
#  define CONFIG_UART0_TXBUFSIZE 256
#endif

/* UART0 is console and ttyS0 */

#define CONSOLE_DEV     g_uart0port     /* UART0 is console */
#define TTYS0_DEV       g_uart0port     /* UART0 is ttyS0 */
#define UART0_ASSIGNED  1

/***************************************************************************
 * ZYNQ_MPSOC UART Registers and Bit Definitions
 *
 * Register Map
 *
 * Register offsets for the UART.
 ***************************************************************************/

#define XUARTPS_CR_OFFSET      0x0000U  /* Control Register [8:0] */
#define XUARTPS_MR_OFFSET      0x0004U  /* Mode Register [9:0] */
#define XUARTPS_IER_OFFSET     0x0008U  /* Interrupt Enable [12:0] */
#define XUARTPS_IDR_OFFSET     0x000CU  /* Interrupt Disable [12:0] */
#define XUARTPS_IMR_OFFSET     0x0010U  /* Interrupt Mask [12:0] */
#define XUARTPS_ISR_OFFSET     0x0014U  /* Interrupt Status [12:0]*/
#define XUARTPS_BAUDGEN_OFFSET 0x0018U  /* Baud Rate Generator [15:0] */
#define XUARTPS_RXTOUT_OFFSET  0x001CU  /* RX Timeout [7:0] */
#define XUARTPS_RXWM_OFFSET    0x0020U  /* RX FIFO Trigger Level [5:0] */
#define XUARTPS_MODEMCR_OFFSET 0x0024U  /* Modem Control [5:0] */
#define XUARTPS_MODEMSR_OFFSET 0x0028U  /* Modem Status [8:0] */
#define XUARTPS_SR_OFFSET      0x002CU  /* Channel Status [14:0] */
#define XUARTPS_FIFO_OFFSET    0x0030U  /* FIFO [7:0] */
#define XUARTPS_BAUDDIV_OFFSET 0x0034U  /* Baud Rate Divider [7:0] */
#define XUARTPS_FLOWDEL_OFFSET 0x0038U  /* Flow Delay [5:0] */
#define XUARTPS_TXWM_OFFSET    0x0044U  /* TX FIFO Trigger Level [5:0] */
#define XUARTPS_RXBS_OFFSET    0x0048U  /* RX FIFO Byte Status [11:0] */

/***************************************************************************
 * Control Register
 *
 * The Control register (CR) controls the major functions of the device.
 *
 * Control Register Bit Definition
 ***************************************************************************/

#define XUARTPS_CR_STOPBRK     0x00000100U  /* Stop transmission of break */
#define XUARTPS_CR_STARTBRK    0x00000080U  /* Set break */
#define XUARTPS_CR_TORST       0x00000040U  /* RX timeout counter restart */
#define XUARTPS_CR_TX_DIS      0x00000020U  /* TX disabled. */
#define XUARTPS_CR_TX_EN       0x00000010U  /* TX enabled */
#define XUARTPS_CR_RX_DIS      0x00000008U  /* RX disabled. */
#define XUARTPS_CR_RX_EN       0x00000004U  /* RX enabled */
#define XUARTPS_CR_EN_DIS_MASK 0x0000003CU  /* Enable/disable Mask */
#define XUARTPS_CR_TXRST       0x00000002U  /* TX logic reset */
#define XUARTPS_CR_RXRST       0x00000001U  /* RX logic reset */

/***************************************************************************
 * Mode Register
 *
 * The mode register (MR) defines the mode of transfer as well as the data
 * format. If this register is modified during transmission or reception,
 * data validity cannot be guaranteed.
 *
 * Mode Register Bit Definition
 *
 ***************************************************************************/
#define XUARTPS_MR_CCLK             0x00000400U /* Input clock selection */
#define XUARTPS_MR_CHMODE_R_LOOP    0x00000300U /* Remote loopback mode */
#define XUARTPS_MR_CHMODE_L_LOOP    0x00000200U /* Local loopback mode */
#define XUARTPS_MR_CHMODE_ECHO      0x00000100U /* Auto echo mode */
#define XUARTPS_MR_CHMODE_NORM      0x00000000U /* Normal mode */
#define XUARTPS_MR_CHMODE_SHIFT     8U          /* Mode shift */
#define XUARTPS_MR_CHMODE_MASK      0x00000300U /* Mode mask */
#define XUARTPS_MR_STOPMODE_2_BIT   0x00000080U /* 2 stop bits */
#define XUARTPS_MR_STOPMODE_1_5_BIT 0x00000040U /* 1.5 stop bits */
#define XUARTPS_MR_STOPMODE_1_BIT   0x00000000U /* 1 stop bit */
#define XUARTPS_MR_STOPMODE_SHIFT   6U          /* Stop bits shift */
#define XUARTPS_MR_STOPMODE_MASK    0x000000A0U /* Stop bits mask */
#define XUARTPS_MR_PARITY_NONE      0x00000020U /* No parity mode */
#define XUARTPS_MR_PARITY_MARK      0x00000018U /* Mark parity mode */
#define XUARTPS_MR_PARITY_SPACE     0x00000010U /* Space parity mode */
#define XUARTPS_MR_PARITY_ODD       0x00000008U /* Odd parity mode */
#define XUARTPS_MR_PARITY_EVEN      0x00000000U /* Even parity mode */
#define XUARTPS_MR_PARITY_SHIFT     3U          /* Parity setting shift */
#define XUARTPS_MR_PARITY_MASK      0x00000038U /* Parity mask */
#define XUARTPS_MR_CHARLEN_6_BIT    0x00000006U /* 6 bits data */
#define XUARTPS_MR_CHARLEN_7_BIT    0x00000004U /* 7 bits data */
#define XUARTPS_MR_CHARLEN_8_BIT    0x00000000U /* 8 bits data */
#define XUARTPS_MR_CHARLEN_SHIFT    1U          /* Data Length shift */
#define XUARTPS_MR_CHARLEN_MASK     0x00000006U /* Data length mask */
#define XUARTPS_MR_CLKSEL           0x00000001U /* Input clock selection */

#define UART_DEFAULT_MODE           XUARTPS_MR_CHMODE_NORM    | \
                                    XUARTPS_MR_STOPMODE_1_BIT | \
                                    XUARTPS_MR_PARITY_NONE    | \
                                    XUARTPS_MR_CHARLEN_8_BIT

/***************************************************************************
 * Interrupt Registers
 *
 * Interrupt control logic uses the interrupt enable register (IER) and the
 * interrupt disable register (IDR) to set the value of the bits in the
 * interrupt mask register (IMR). The IMR determines whether to pass an
 * interrupt to the interrupt status register (ISR).
 * Writing a 1 to IER Enbables an interrupt, writing a 1 to IDR disables an
 * interrupt. IMR and ISR are read only, and IER and IDR are write only.
 * Reading either IER or IDR returns 0x00.
 *
 * All four registers have the same bit definitions.
 *
 ***************************************************************************/

#define XUARTPS_IXR_RBRK    0x00002000U /* Rx FIFO break detect interrupt */
#define XUARTPS_IXR_TOVR    0x00001000U /* Tx FIFO Overflow interrupt */
#define XUARTPS_IXR_TNFUL   0x00000800U /* Tx FIFO Nearly Full interrupt */
#define XUARTPS_IXR_TTRIG   0x00000400U /* Tx Trig interrupt */
#define XUARTPS_IXR_DMS     0x00000200U /* Modem status change interrupt */
#define XUARTPS_IXR_TOUT    0x00000100U /* Timeout error interrupt */
#define XUARTPS_IXR_PARITY  0x00000080U /* Parity error interrupt */
#define XUARTPS_IXR_FRAMING 0x00000040U /* Framing error interrupt */
#define XUARTPS_IXR_OVER    0x00000020U /* Overrun error interrupt */
#define XUARTPS_IXR_TXFULL  0x00000010U /* TX FIFO full interrupt. */
#define XUARTPS_IXR_TXEMPTY 0x00000008U /* TX FIFO empty interrupt. */
#define XUARTPS_IXR_RXFULL  0x00000004U /* RX FIFO full interrupt. */
#define XUARTPS_IXR_RXEMPTY 0x00000002U /* RX FIFO empty interrupt. */
#define XUARTPS_IXR_RXOVR   0x00000001U /* RX FIFO trigger interrupt. */
#define XUARTPS_IXR_MASK    0x00003FFFU /* Valid bit mask */

/***************************************************************************
 * Baud Rate Generator Register
 *
 * The baud rate generator control register (BRGR) is a 16 bit register that
 * controls the receiver bit sample clock and baud rate.
 * Valid values are 1 - 65535.
 *
 * Bit Sample Rate = CCLK / BRGR, where the CCLK is selected by the MR_CCLK
 * bit in the MR register.
 *
 ***************************************************************************/

#define XUARTPS_BAUDGEN_DISABLE   0x00000000U /* Disable clock */
#define XUARTPS_BAUDGEN_MASK      0x0000FFFFU /* Valid bits mask */
#define XUARTPS_BAUDGEN_RESET_VAL 0x0000028BU /* Reset value */

/***************************************************************************
 * Baud Divisor Rate register
 *
 * The baud rate divider register (BDIV) controls how much the bit sample
 * rate is divided by. It sets the baud rate.
 * Valid values are 0x04 to 0xFF. Writing a value less than 4 will be
 * ignored.
 *
 * Baud rate = CCLK / ((BAUDDIV + 1) x BRGR), where the CCLK is selected by
 * the MR_CCLK bit in the MR register.
 *
 ***************************************************************************/

#define XUARTPS_BAUDDIV_MASK        0x000000FFU /* 8 bit baud divider mask */
#define XUARTPS_BAUDDIV_RESET_VAL   0x0000000FU /* Reset value */

/***************************************************************************
 * The following constant defines the amount of error that is allowed for
 * a specified baud rate. This error is the difference between the actual
 * baud rate that will be generated using the specified clock and the
 * desired baud rate.
 ***************************************************************************/

#define XUARTPS_MAX_BAUD_ERROR_RATE 3U /* max % error allowed */

/***************************************************************************
 * The following constants indicate the max and min baud rates and these
 * numbers are based only on the testing that has been done. The hardware
 * is capable of other baud rates.
 ***************************************************************************/

#define XUARTPS_MAX_RATE 6240000U
#define XUARTPS_MIN_RATE 110U

/***************************************************************************
 * Receiver Timeout Register
 *
 * Use the receiver timeout register (RTR) to detect an idle condition on
 * the receiver data line.
 *
 ***************************************************************************/

#define XUARTPS_RXTOUT_DISABLE  0x00000000U  /* Disable time out */
#define XUARTPS_RXTOUT_MASK     0x000000FFU  /* Valid bits mask */

/***************************************************************************
 * Receiver FIFO Trigger Level Register
 *
 * Use the Receiver FIFO Trigger Level Register (RTRIG) to set the value at
 * which the RX FIFO triggers an interrupt event.
 *
 ***************************************************************************/

#define XUARTPS_RXWM_DISABLE   0x00000000U /* Disable RX trigger interrupt */
#define XUARTPS_RXWM_MASK      0x0000003FU /* Valid bits mask */
#define XUARTPS_RXWM_RESET_VAL 0x00000020U /* Reset value */

/***************************************************************************
 * Transmit FIFO Trigger Level Register
 *
 * Use the Transmit FIFO Trigger Level Register (TTRIG) to set the value at
 * which the TX FIFO triggers an interrupt event.
 *
 ***************************************************************************/

#define XUARTPS_TXWM_MASK       0x0000003FU  /* Valid bits mask */
#define XUARTPS_TXWM_RESET_VAL  0x00000020U  /* Reset value */

/***************************************************************************
 * Modem Control Register
 *
 * This register (MODEMCR) controls the interface with the modem or data
 * set, or a peripheral device emulating a modem.
 *
 ***************************************************************************/

#define XUARTPS_MODEMCR_FCM 0x00000020U  /* Flow control mode */
#define XUARTPS_MODEMCR_RTS 0x00000002U  /* Request to send */
#define XUARTPS_MODEMCR_DTR 0x00000001U  /* Data terminal ready */

/***************************************************************************
 * Modem Status Register
 *
 * This register (MODEMSR) indicates the current state of the control lines
 * from a modem, or another peripheral device, to the CPU. In addition, four
 * bits of the modem status register provide change information. These bits
 * are set to a logic 1 whenever a control input from the modem changes
 * state.
 *
 * Note: Whenever the DCTS, DDSR, TERI, or DDCD bit is set to logic 1, a
 * modem status interrupt is generated and this is reflected in the modem
 * status register.
 *
 ***************************************************************************/

#define XUARTPS_MODEMSR_FCMS    0x00000100U  /* Flow control mode (FCMS) */
#define XUARTPS_MODEMSR_DCD     0x00000080U  /* Complement of DCD input */
#define XUARTPS_MODEMSR_RI      0x00000040U  /* Complement of RI input */
#define XUARTPS_MODEMSR_DSR     0x00000020U  /* Complement of DSR input */
#define XUARTPS_MODEMSR_CTS     0x00000010U  /* Complement of CTS input */
#define XUARTPS_MODEMSR_DDCD    0x00000008U  /* Delta DCD indicator */
#define XUARTPS_MODEMSR_TERI    0x00000004U  /* Trailing Edge Ring Indicator */
#define XUARTPS_MODEMSR_DDSR    0x00000002U  /* Change of DSR */
#define XUARTPS_MODEMSR_DCTS    0x00000001U  /* Change of CTS */

/***************************************************************************
 * Channel Status Register
 *
 * The channel status register (CSR) is provided to enable the control logic
 * to monitor the status of bits in the channel interrupt status register,
 * even if these are masked out by the interrupt mask register.
 *
 ***************************************************************************/

#define XUARTPS_SR_TNFUL   0x00004000U /* TX FIFO Nearly Full Status */
#define XUARTPS_SR_TTRIG   0x00002000U /* TX FIFO Trigger Status */
#define XUARTPS_SR_FLOWDEL 0x00001000U /* RX FIFO fill over flow delay */
#define XUARTPS_SR_TACTIVE 0x00000800U /* TX active */
#define XUARTPS_SR_RACTIVE 0x00000400U /* RX active */
#define XUARTPS_SR_TXFULL  0x00000010U /* TX FIFO full */
#define XUARTPS_SR_TXEMPTY 0x00000008U /* TX FIFO empty */
#define XUARTPS_SR_RXFULL  0x00000004U /* RX FIFO full */
#define XUARTPS_SR_RXEMPTY 0x00000002U /* RX FIFO empty */
#define XUARTPS_SR_RXOVR   0x00000001U /* RX FIFO fill over trigger */

/***************************************************************************
 * Flow Delay Register
 *
 * Operation of the flow delay register (FLOWDEL) is very similar to the
 * receive FIFO trigger register. An internal trigger signal activates when
 * the FIFO is filled to the level set by this register. This trigger will
 * not cause an interrupt, although it can be read through the channel
 * status register. In hardware flow control mode, RTS is deactivated when
 * the trigger becomes active. RTS only resets when the FIFO level is four
 * less than the level of the flow delay trigger and the flow delay trigger
 * is not activated. A value less than 4 disables the flow delay.
 *
 ***************************************************************************/

#define XUARTPS_FLOWDEL_MASK    XUARTPS_RXWM_MASK /* Valid bit mask */

/***************************************************************************
 * Receiver FIFO Byte Status Register
 *
 * The Receiver FIFO Status register is used to have a continuous
 * monitoring of the raw unmasked byte status information. The register
 * contains frame, parity and break status information for the top
 * four bytes in the RX FIFO.
 *
 * Receiver FIFO Byte Status Register Bit Definition
 *
 ***************************************************************************/

#define XUARTPS_RXBS_BYTE3_BRKE 0x00000800U /* Byte3 Break Error */
#define XUARTPS_RXBS_BYTE3_FRME 0x00000400U /* Byte3 Frame Error */
#define XUARTPS_RXBS_BYTE3_PARE 0x00000200U /* Byte3 Parity Error */
#define XUARTPS_RXBS_BYTE2_BRKE 0x00000100U /* Byte2 Break Error */
#define XUARTPS_RXBS_BYTE2_FRME 0x00000080U /* Byte2 Frame Error */
#define XUARTPS_RXBS_BYTE2_PARE 0x00000040U /* Byte2 Parity Error */
#define XUARTPS_RXBS_BYTE1_BRKE 0x00000020U /* Byte1 Break Error */
#define XUARTPS_RXBS_BYTE1_FRME 0x00000010U /* Byte1 Frame Error */
#define XUARTPS_RXBS_BYTE1_PARE 0x00000008U /* Byte1 Parity Error */
#define XUARTPS_RXBS_BYTE0_BRKE 0x00000004U /* Byte0 Break Error */
#define XUARTPS_RXBS_BYTE0_FRME 0x00000002U /* Byte0 Frame Error */
#define XUARTPS_RXBS_BYTE0_PARE 0x00000001U /* Byte0 Parity Error */
#define XUARTPS_RXBS_MASK       0x00000007U /* 3 bit RX byte status mask */

/***************************************************************************
 * Private Types
 ***************************************************************************/

/* ZYNQ_MPSOC UART Configuration */

struct zynq_uart_config
{
  unsigned long uart;  /* UART Base Address */
};

/* ZYNQ_MPSOC UART Device Data */

struct zynq_uart_data
{
  uint32_t baud_rate;  /* UART Baud Rate */
  uint32_t ier;        /* Saved IER value */
  uint8_t  parity;     /* 0=none, 1=odd, 2=even */
  uint8_t  bits;       /* Number of bits (7 or 8) */
  bool     stopbits2;  /* true: Configure with 2 stop bits instead of 1 */
};

/* ZYNQ_MPSOC UART Port */

struct zynq_uart_port_s
{
  struct zynq_uart_data data;     /* UART Device Data */
  struct zynq_uart_config config; /* UART Configuration */
  unsigned int irq_num;           /* UART IRQ Number */
  bool is_console;                /* 1 if this UART is console */
};

/***************************************************************************
 * Private Function Prototypes
 ***************************************************************************/

static void zynq_uart_rxint(struct uart_dev_s *dev, bool enable);
static void zynq_uart_txint(struct uart_dev_s *dev, bool enable);

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/***************************************************************************
 * Name: zynq_uart_irq_handler
 *
 * Description:
 *   This is the common UART interrupt handler.  It should call
 *   uart_xmitchars or uart_recvchars to perform the appropriate data
 *   transfers.
 *
 * Input Parameters:
 *   irq     - IRQ Number
 *   context - Interrupt Context
 *   arg     - UART Device
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ***************************************************************************/

static int zynq_uart_irq_handler(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct zynq_uart_port_s *port = (struct zynq_uart_port_s *)dev->priv;
  struct zynq_uart_config *config = &port->config;
  uint32_t status;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);

  status = getreg32(config->uart + XUARTPS_ISR_OFFSET);
  putreg32(status, (config->uart + XUARTPS_ISR_OFFSET));

  if (status & XUARTPS_IXR_FRAMING)
    {
      while (!(getreg32(config->uart + XUARTPS_SR_OFFSET) &
             XUARTPS_SR_RXEMPTY))
        {
          if (!getreg8(config->uart + XUARTPS_FIFO_OFFSET))
            {
              status &= ~XUARTPS_IXR_FRAMING;
            }
        }

      putreg32(XUARTPS_IXR_FRAMING, (config->uart + XUARTPS_ISR_OFFSET));
    }

  if (status & XUARTPS_IXR_TXEMPTY)
    {
      uart_xmitchars(dev);
    }

  if (status & (XUARTPS_IXR_RXOVR | XUARTPS_IXR_TOUT | XUARTPS_IXR_RXFULL))
    {
      while (1)
        {
          if ((getreg32(config->uart + XUARTPS_SR_OFFSET) &
              XUARTPS_SR_RXEMPTY) == XUARTPS_SR_RXEMPTY)
            {
              break;
            }
          else
            {
              uart_recvchars(dev);
            }
        }
    }

  return OK;
}

static int zynq_uart_baudrate(struct uart_dev_s *dev, uint32_t in_clk)
{
  struct zynq_uart_port_s *port = (struct zynq_uart_port_s *)dev->priv;
  const struct zynq_uart_config *config = &port->config;
  struct zynq_uart_data *data = &port->data;
  uint32_t iter_baud_div;     /* Iterator for available baud divisor values */
  uint32_t brgr_value;        /* Calculated value for baud rate generator */
  uint32_t calc_baudrate;     /* Calculated baud rate */
  uint32_t baud_error;        /* Diff between calculated and requested baud rate */
  uint32_t best_brgr = 0U;    /* Best value for baud rate generator */
  uint8_t best_baud_div = 0U; /* Best value for baud divisor */
  uint32_t best_error = 0xffffffff;
  uint32_t percent_error;
  uint32_t reg;
  uint32_t input_clk;
  uint32_t temp_reg;

  /* Asserts validate the input arguments */

  DEBUGASSERT(data->baud_rate <= (uint32_t)XUARTPS_MAX_RATE);
  DEBUGASSERT(data->baud_rate >= (uint32_t)XUARTPS_MIN_RATE);

  /* Make sure the baud rate is not impossilby large.
   * Fastest possible baud rate is Input Clock / 2.
   */

  if ((data->baud_rate * 2) > in_clk)
    {
      return ERROR;
    }

  /* Check whether the input clock is divided by 8 */

  reg = getreg32(config->uart + XUARTPS_MR_OFFSET);

  input_clk = in_clk;
  if (reg & XUARTPS_MR_CLKSEL)
    {
      input_clk = in_clk / 8;
    }

  /* Determine the Baud divider. It can be 4to 254.
   * Loop through all possible combinations
   */

  for (iter_baud_div = 4; iter_baud_div < 255; iter_baud_div++)
    {
      /* Calculate the value for BRGR register */

      brgr_value = input_clk / (data->baud_rate * (iter_baud_div + 1));

      /* Calculate the baud rate from the BRGR value */

      calc_baudrate = input_clk / (brgr_value * (iter_baud_div + 1));

      /* Avoid unsigned integer underflow */

      if (data->baud_rate > calc_baudrate)
        {
          baud_error = data->baud_rate - calc_baudrate;
        }
        else
        {
          baud_error = calc_baudrate - data->baud_rate;
        }

      /* Find the calculated baud rate closest to requested baud rate. */

      if (best_error > baud_error)
        {
          best_brgr = brgr_value;
          best_baud_div = iter_baud_div;
          best_error = baud_error;
        }
    }

  /* Make sure the best error is not too large. */

  percent_error = (best_error * 100) / data->baud_rate;
  if (XUARTPS_MAX_BAUD_ERROR_RATE < percent_error)
    {
      return ERROR;
    }

  /* Disable TX and RX to avoid glitches when setting the baud rate. */

  temp_reg = (((getreg32(config->uart + XUARTPS_CR_OFFSET)) &
               ((uint32_t)(~XUARTPS_CR_EN_DIS_MASK))) |
              ((uint32_t)XUARTPS_CR_RX_DIS | (uint32_t)XUARTPS_CR_TX_DIS));
  putreg32(temp_reg, config->uart + XUARTPS_CR_OFFSET);

  /* Set the baud rate divisor */

  putreg32(best_brgr, config->uart + XUARTPS_BAUDGEN_OFFSET);
  putreg32(best_baud_div, config->uart + XUARTPS_BAUDDIV_OFFSET);

  /* RX and TX SW reset */

  putreg32(XUARTPS_CR_TXRST | XUARTPS_CR_RXRST,
           config->uart + XUARTPS_CR_OFFSET);

  /* Enable device */

  temp_reg = (((getreg32(config->uart + XUARTPS_CR_OFFSET)) &
               ((uint32_t)(~XUARTPS_CR_EN_DIS_MASK))) |
              ((uint32_t)XUARTPS_CR_RX_EN | (uint32_t)XUARTPS_CR_TX_EN));
  putreg32(temp_reg, config->uart + XUARTPS_CR_OFFSET);

  return OK;
}

/***************************************************************************
 * Name: zynq_uart_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ***************************************************************************/

static int zynq_uart_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct zynq_uart_port_s *port = (struct zynq_uart_port_s *)dev->priv;
  const struct zynq_uart_config *config = &port->config;
  struct zynq_uart_data *data = &port->data;
  uint32_t reg = 0U;

  DEBUGASSERT(data != NULL);

  if (zynq_uart_baudrate(dev, CONFIG_XPAR_PSU_UART_0_UART_CLK_FREQ_HZ)
      != OK)
    {
      return ERROR;
    }

  /* Set the parity mode */

  reg = getreg32(config->uart + XUARTPS_MR_OFFSET);

  /* Mask off what's already there */

  reg &= (~((uint32_t)XUARTPS_MR_CHARLEN_MASK |
            (uint32_t)XUARTPS_MR_STOPMODE_MASK |
            (uint32_t)XUARTPS_MR_PARITY_MASK));

  switch (data->bits)
    {
    case 6:
        reg |= (uint32_t)XUARTPS_MR_CHARLEN_6_BIT;
        break;
    case 7:
        reg |= (uint32_t)XUARTPS_MR_CHARLEN_7_BIT;
        break;
    case 8:
        reg |= (uint32_t)XUARTPS_MR_CHARLEN_8_BIT;
        break;
    default:
        reg |= (uint32_t)XUARTPS_MR_CHARLEN_8_BIT;
        break;
    }

  if (data->stopbits2)
    {
      reg |= (uint32_t)XUARTPS_MR_STOPMODE_2_BIT;
    }
  else
    {
      reg |= (uint32_t)XUARTPS_MR_STOPMODE_1_BIT;
    }

  switch (data->parity)
    {
    case 0:
        reg |= (uint32_t)XUARTPS_MR_PARITY_NONE;
        break;
    case 1:
        reg |= (uint32_t)XUARTPS_MR_PARITY_ODD;
        break;
    case 2:
        reg |= (uint32_t)XUARTPS_MR_PARITY_EVEN;
        break;
    default:
        reg |= (uint32_t)XUARTPS_MR_PARITY_NONE;
        break;
    }

  /* Write the mode register out */

  putreg32(reg, config->uart + XUARTPS_MR_OFFSET);

  /* Set the RX FIFO trigger at 8 data bytes. */

  putreg32(0x08, config->uart + XUARTPS_RXWM_OFFSET);

  /* Set the RX timeout to 1, which will be 4 character time */

  putreg32(0x01, config->uart + XUARTPS_RXTOUT_OFFSET);

  /* Disable all interrupts, polled mode is the default */

  putreg32(XUARTPS_IXR_MASK, config->uart + XUARTPS_IDR_OFFSET);

#endif /* CONFIG_SUPPRESS_UART_CONFIG */
  return OK;
}

/***************************************************************************
 * Name: zynq_uart_shutdown
 *
 * Description:
 *   Disable the UART Port.  This method is called when the serial
 *   port is closed.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void zynq_uart_shutdown(struct uart_dev_s *dev)
{
  /* Disable the Receive and Transmit Interrupts */

  zynq_uart_rxint(dev, false);
  zynq_uart_txint(dev, false);
}

/***************************************************************************
 * Name: zynq_uart_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.
 *   This method is called when the serial port is opened.
 *   Normally, this is just after the setup() method is called,
 *   however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method
 *   (unless the hardware supports multiple levels of interrupt
 *   enabling).  The RX and TX interrupts are not enabled until
 *   the txint() and rxint() methods are called.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ***************************************************************************/

static int zynq_uart_attach(struct uart_dev_s *dev)
{
  int ret;
  struct zynq_uart_port_s *port = (struct zynq_uart_port_s *)dev->priv;

  DEBUGASSERT(port != NULL);

  /* Attach UART Interrupt Handler */

  ret = irq_attach(port->irq_num, zynq_uart_irq_handler, dev);

  /* Set Interrupt Priority in Generic Interrupt Controller v2 */

  arm64_gic_irq_set_priority(port->irq_num, 0, IRQ_TYPE_LEVEL);

  /* Enable UART Interrupt */

  if (ret == OK)
    {
      up_enable_irq(port->irq_num);
    }
  else
    {
      _err("IRQ attach failed, ret=%d\n", ret);
    }

  return ret;
}

/***************************************************************************
 * Name: zynq_uart_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void zynq_uart_detach(struct uart_dev_s *dev)
{
  struct zynq_uart_port_s *port = (struct zynq_uart_port_s *)dev->priv;

  DEBUGASSERT(port != NULL);

  /* Disable UART Interrupt */

  up_disable_irq(port->irq_num);

  /* Detach UART Interrupt Handler */

  irq_detach(port->irq_num);
}

/***************************************************************************
 * Name: zynq_uart_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *   filep - File Struct
 *   cmd   - ioctl Command
 *   arg   - ioctl Argument
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ***************************************************************************/

static int zynq_uart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

  UNUSED(filep);
  UNUSED(arg);

  switch (cmd)
    {
      case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/***************************************************************************
 * Name: zynq_uart_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 * Input Parameters:
 *   dev    - UART Device
 *   status - Return status, zero on success
 *
 * Returned Value:
 *   Received character
 *
 ***************************************************************************/

static int zynq_uart_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct zynq_uart_port_s *port = (struct zynq_uart_port_s *)dev->priv;
  const struct zynq_uart_config *config = &port->config;
  uint32_t rbr;

  *status = getreg8(config->uart + XUARTPS_SR_OFFSET) & XUARTPS_SR_RXEMPTY;

  /* Wait until there is data */

  if (*status)
    {
      return -1;
    }

  rbr = getreg8(config->uart + XUARTPS_FIFO_OFFSET);
  return rbr;
}

/***************************************************************************
 * Name: zynq_uart_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 * Input Parameters:
 *   dev    - UART Device
 *   enable - True to enable RX interrupts; false to disable
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void zynq_uart_rxint(struct uart_dev_s *dev, bool enable)
{
  struct zynq_uart_port_s *port = (struct zynq_uart_port_s *)dev->priv;
  struct zynq_uart_config *config = &port->config;

  /* Write to Interrupt Enable Register (UART_IER) */

  if (enable)
    {
      /* Write the new value for the FIFO control register to it such that
       * the threshold is changed
       */

      putreg32(1, config->uart + XUARTPS_RXWM_OFFSET);

      /* Set XUARTPS_IXR_RXOVR bit (Enable Rx Data Available Interrupt) */

      modreg32(XUARTPS_IXR_RXOVR, XUARTPS_IXR_RXOVR,
               config->uart + XUARTPS_IER_OFFSET);

      modreg32(0, XUARTPS_IXR_RXOVR, config->uart + XUARTPS_IDR_OFFSET);

      /* Set XUARTPS_IXR_RXFULL bit (Enable RxFifo full Interrupt) */

      modreg32(XUARTPS_IXR_RXFULL, XUARTPS_IXR_RXFULL,
               config->uart + XUARTPS_IER_OFFSET);

      modreg32(0, XUARTPS_IXR_RXFULL, config->uart + XUARTPS_IDR_OFFSET);
    }
  else
    {
      /* Set XUARTPS_IXR_RXOVR bit (Enable Rx Data Available Interrupt) */

      modreg32(0, XUARTPS_IXR_RXOVR, config->uart + XUARTPS_IER_OFFSET);
      modreg32(XUARTPS_IXR_RXOVR, XUARTPS_IXR_RXOVR,
               config->uart + XUARTPS_IDR_OFFSET);

      /* Set XUARTPS_IXR_RXFULL bit (Enable RxFifo full Interrupt) */

      modreg32(0, XUARTPS_IXR_RXFULL, config->uart + XUARTPS_IER_OFFSET);
      modreg32(XUARTPS_IXR_RXFULL, XUARTPS_IXR_RXFULL,
               config->uart + XUARTPS_IDR_OFFSET);
    }
}

/***************************************************************************
 * Name: zynq_uart_rxavailable
 *
 * Description:
 *   Return true if the Receive FIFO is not empty
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   True if the Receive FIFO is not empty; false otherwise
 *
 ***************************************************************************/

static bool zynq_uart_rxavailable(struct uart_dev_s *dev)
{
  struct zynq_uart_port_s *port = (struct zynq_uart_port_s *)dev->priv;
  struct zynq_uart_config *config = &port->config;

  /* RxFifo empty bit is 1 if Rx Data is unavailable */

  return ((getreg8(config->uart + XUARTPS_SR_OFFSET) &
          XUARTPS_SR_RXEMPTY) == 0);
}

/***************************************************************************
 * Name: zynq_uart_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 * Input Parameters:
 *   dev - UART Device
 *   ch  - Character to be sent
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void zynq_uart_send(struct uart_dev_s *dev, int ch)
{
  struct zynq_uart_port_s *port = (struct zynq_uart_port_s *)dev->priv;
  struct zynq_uart_config *config = &port->config;

  /* Write char to Transmit FIFO Register */

  putreg8(ch, config->uart + XUARTPS_FIFO_OFFSET);
}

/***************************************************************************
 * Name: zynq_uart_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 * Input Parameters:
 *   dev    - UART Device
 *   enable - True to enable TX interrupts; false to disable
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void zynq_uart_txint(struct uart_dev_s *dev, bool enable)
{
  struct zynq_uart_port_s *port = (struct zynq_uart_port_s *)dev->priv;
  struct zynq_uart_config *config = &port->config;

  /* Write to Interrupt Enable Register (UART_IER) */

  if (enable)
    {
      /* Set XUARTPS_IXR_TXEMPTY bit (Enable Tx Fifo Empty Interrupt) */

      modreg32(XUARTPS_IXR_TXEMPTY, XUARTPS_IXR_TXEMPTY,
               config->uart + XUARTPS_IER_OFFSET);

      modreg32(0, XUARTPS_IXR_TXEMPTY, config->uart + XUARTPS_IDR_OFFSET);

      /* Fake a TX interrupt */

      uart_xmitchars(dev);
    }
  else
    {
      /* Clear XUARTPS_IXR_TXEMPTY bit (Disable Tx Fifo Empty Interrupt) */

      modreg32(0, XUARTPS_IXR_TXEMPTY, config->uart + XUARTPS_IER_OFFSET);
      modreg32(XUARTPS_IXR_TXEMPTY, XUARTPS_IXR_TXEMPTY,
               config->uart + XUARTPS_IDR_OFFSET);
    }
}

/***************************************************************************
 * Name: zynq_uart_txready
 *
 * Description:
 *   Return true if the Transmit FIFO is not full
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   True if the Transmit FIFO is not full; false otherwise
 *
 ***************************************************************************/

static bool zynq_uart_txready(struct uart_dev_s *dev)
{
  struct zynq_uart_port_s *port = (struct zynq_uart_port_s *)dev->priv;
  struct zynq_uart_config *config = &port->config;

  /* Tx FIFO is ready if THRE Bit is 1 (Tx Holding Register Empty) */

  return (getreg8(config->uart + XUARTPS_SR_OFFSET)
          & XUARTPS_SR_TXFULL) == 0;
}

/***************************************************************************
 * Name: zynq_uart_txempty
 *
 * Description:
 *   Return true if the Transmit FIFO is empty
 *
 * Input Parameters:
 *   dev - UART Device
 *
 * Returned Value:
 *   True if the Transmit FIFO is empty; false otherwise
 *
 ***************************************************************************/

static bool zynq_uart_txempty(struct uart_dev_s *dev)
{
  /* Tx FIFO is empty if Tx FIFO is not full (for now) */

  return zynq_uart_txready(dev);
}

/***************************************************************************
 * Name: zynq_uart_wait_send
 *
 * Description:
 *   Wait for Transmit FIFO until it is not full, then transmit the
 *   character over UART.
 *
 * Input Parameters:
 *   dev - UART Device
 *   ch  - Character to be sent
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static void zynq_uart_wait_send(struct uart_dev_s *dev, int ch)
{
  DEBUGASSERT(dev != NULL);
  while (!zynq_uart_txready(dev));
  zynq_uart_send(dev, ch);
}

/***************************************************************************
 * Private Data
 ***************************************************************************/

/* UART Operations for Serial Driver */

static const struct uart_ops_s g_uart_ops =
{
  .setup    = zynq_uart_setup,
  .shutdown = zynq_uart_shutdown,
  .attach   = zynq_uart_attach,
  .detach   = zynq_uart_detach,
  .ioctl    = zynq_uart_ioctl,
  .receive  = zynq_uart_receive,
  .rxint    = zynq_uart_rxint,
  .rxavailable = zynq_uart_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol    = NULL,
#endif
  .send     = zynq_uart_send,
  .txint    = zynq_uart_txint,
  .txready  = zynq_uart_txready,
  .txempty  = zynq_uart_txempty,
};

#ifdef CONFIG_ZYNQ_MPSOC_UART0

/* UART0 Port State (Console) */

static struct zynq_uart_port_s g_uart0priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART0_BAUD,
      .parity     = CONFIG_UART0_PARITY,
      .bits       = CONFIG_UART0_BITS,
      .stopbits2  = CONFIG_UART0_2STOP
    },

  .config =
    {
      .uart       = ZYNQ_MPSOC_UART0_ADDR
    },

    .irq_num      = ZYNQ_MPSOC_IRQ_UART0,
    .is_console   = 1
};

/* UART0 I/O Buffers (Console) */

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

/* UART0 Port Definition (Console) */

static struct uart_dev_s g_uart0port =
{
  .recv  =
    {
      .size   = CONFIG_UART0_RXBUFSIZE,
      .buffer = g_uart0rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART0_TXBUFSIZE,
      .buffer = g_uart0txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart0priv,
};

#endif /* CONFIG_ZYNQ_MPSOC_UART */

#ifdef CONFIG_ZYNQ_MPSOC_UART1

/* UART1 Port State */

static struct zynq_uart_port_s g_uart1priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART1_BAUD,
      .parity     = CONFIG_UART1_PARITY,
      .bits       = CONFIG_UART1_BITS,
      .stopbits2  = CONFIG_UART1_2STOP
    },

  .config =
    {
      .uart       = ZYNQ_MPSOC_UART1_ADDR
    },

    .irq_num      = ZYNQ_MPSOC_IRQ_UART1,
    .is_console   = 0
};

/* UART1 I/O Buffers */

static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

/* UART1 Port Definition */

static struct uart_dev_s g_uart1port =
{
  .recv  =
    {
      .size   = CONFIG_UART1_RXBUFSIZE,
      .buffer = g_uart1rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART1_TXBUFSIZE,
      .buffer = g_uart1txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart1priv,
};

#endif /* CONFIG_ZYNQ_MPSOC_UART1 */

/* Pick ttys1.  This could be any of UART1. */

#if defined(CONFIG_ZYNQ_MPSOC_UART1) && !defined(UART1_ASSIGNED)
#  define TTYS1_DEV           g_uart1port /* UART1 is ttyS1 */
#  define UART1_ASSIGNED      1
#endif

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: arm64_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before arm64_serialinit.
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void arm64_earlyserialinit(void)
{
  int ret;

  /* NOTE: This function assumes that UART0 low level hardware configuration
   * -- including all clocking and pin configuration
   */

#ifdef CONFIG_ZYNQ_MPSOC_UART1
  /* Configure UART1 */

  ret = zynq_uart_setup(&g_uart1port);
  if (ret < 0)
    {
      _err("UART1 config failed, ret=%d\n", ret);
    }
#endif /* CONFIG_ZYNQ_MPSOC_UART1 */

#ifdef CONSOLE_DEV
  /* Enable the console at UART0 */

  CONSOLE_DEV.isconsole = true;
  zynq_uart_setup(&CONSOLE_DEV);
#endif

  UNUSED(ret);
}

/***************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 * Input Parameters:
 *   ch - Character to be transmitted over UART
 *
 * Returned Value:
 *   Character that was transmitted
 *
 ***************************************************************************/

void up_putc(int ch)
{
#ifdef CONSOLE_DEV
  struct uart_dev_s *dev = &CONSOLE_DEV;

  zynq_uart_wait_send(dev, ch);
#endif
}

/***************************************************************************
 * Name: arm64_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that zynq_earlyserialinit was called previously.
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

void arm64_serialinit(void)
{
#ifdef CONSOLE_DEV
  int ret;

  ret = uart_register("/dev/console", &CONSOLE_DEV);
  if (ret < 0)
    {
      _err("Register /dev/console failed, ret=%d\n", ret);
    }

  ret = uart_register("/dev/ttyS0", &TTYS0_DEV);

  if (ret < 0)
    {
      _err("Register /dev/ttyS0 failed, ret=%d\n", ret);
    }

#ifdef TTYS1_DEV
  ret = uart_register("/dev/ttyS1", &TTYS1_DEV);

  if (ret < 0)
    {
      _err("Register /dev/ttyS1 failed, ret=%d\n", ret);
    }
#endif /* TTYS1_DEV */

#endif
}

#endif /* USE_SERIALDRIVER */
