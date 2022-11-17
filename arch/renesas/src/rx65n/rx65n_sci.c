/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_sci.c
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
#include "sys/types.h"
#include "rx65n_macrodriver.h"
#include "rx65n_sci.h"
#include "chip.h"
#include "renesas_internal.h"
#include "rx65n_definitions.h"
#include "arch/board/rx65n_gpio.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI0
/* SCI0 transmit buffer address */

volatile uint8_t *gp_sci0_tx_address;

/* SCI0 receive buffer address */

volatile uint8_t *gp_sci0_rx_address;

/* SCI0 transmit data number */

volatile uint16_t g_sci0_tx_count;

/* SCI0 receive data number */

volatile uint16_t g_sci0_rx_count;

/* SCI0 receive data length */

volatile uint16_t g_sci0_rx_length;
#endif

#ifdef CONFIG_RX65N_SCI1
/* SCI1 transmit buffer address */

volatile uint8_t *gp_sci1_tx_address;

/* SCI1 receive buffer address */

volatile uint8_t *gp_sci1_rx_address;

/* SCI1 transmit data number */

volatile uint16_t g_sci1_tx_count;

/* SCI1 receive data number */

volatile uint16_t g_sci1_rx_count;

/* SCI1 receive data length */

volatile uint16_t g_sci1_rx_length;
#endif

#ifdef CONFIG_RX65N_SCI2
/* SCI2 transmit buffer address */

volatile uint8_t *gp_sci2_tx_address;

/* SCI2 receive buffer address  */

volatile uint8_t *gp_sci2_rx_address;

/* SCI2 transmit data number   */

volatile uint16_t g_sci2_tx_count;

/* SCI2 receive data number */

volatile uint16_t g_sci2_rx_count;

/* SCI2 receive data length */

volatile uint16_t g_sci2_rx_length;
#endif

#ifdef CONFIG_RX65N_SCI3
/* SCI3 transmit buffer address */

volatile uint8_t *gp_sci3_tx_address;

/* SCI3 receive buffer address */

volatile uint8_t *gp_sci3_rx_address;

/* SCI3 transmit data number */

volatile uint16_t g_sci3_tx_count;

/* SCI3 receive data number */

volatile uint16_t g_sci3_rx_count;

/* SCI3 receive data length */

volatile uint16_t g_sci3_rx_length;
#endif

#ifdef CONFIG_RX65N_SCI4
/* SCI4 transmit buffer address */

volatile uint8_t *gp_sci4_tx_address;

/* SCI4 receive buffer address */

volatile uint8_t *gp_sci4_rx_address;

/* SCI4 transmit data number */

volatile uint16_t g_sci4_tx_count;

/* SCI4 receive data number */

volatile uint16_t g_sci4_rx_count;

/* SCI4 receive data length */

volatile uint16_t g_sci4_rx_length;
#endif

#ifdef CONFIG_RX65N_SCI5
/* SCI5 transmit buffer address */

volatile uint8_t *gp_sci5_tx_address;

/* SCI5 receive buffer address */

volatile uint8_t *gp_sci5_rx_address;

/* SCI5 transmit data number */

volatile uint16_t g_sci5_tx_count;

/* SCI5 receive data number */

volatile uint16_t g_sci5_rx_count;

/* SCI5 receive data length */

volatile uint16_t g_sci5_rx_length;
#endif

#ifdef CONFIG_RX65N_SCI6

/* SCI6 transmit buffer address */

volatile uint8_t *gp_sci6_tx_address;

/* SCI6 receive buffer address */

volatile uint8_t *gp_sci6_rx_address;

/* SCI6 transmit data number */

volatile uint16_t g_sci6_tx_count;

/* SCI6 receive data number */

volatile uint16_t g_sci6_rx_count;

/* SCI6 receive data length */

volatile uint16_t g_sci6_rx_length;
#endif

#ifdef CONFIG_RX65N_SCI7

/* SCI7 transmit buffer address */

volatile uint8_t *gp_sci7_tx_address;

/* SCI7 receive buffer address */

volatile uint8_t *gp_sci7_rx_address;

/* SCI7 transmit data number */

volatile uint16_t g_sci7_tx_count;

/* SCI7 receive data number */

volatile uint16_t g_sci7_rx_count;

/* SCI7 receive data length */

volatile uint16_t g_sci7_rx_length;
#endif

#ifdef CONFIG_RX65N_SCI8
/* SCI8 transmit buffer address */

volatile uint8_t *gp_sci8_tx_address;

/* SCI8 receive buffer address  */

volatile uint8_t *gp_sci8_rx_address;

/* SCI8 transmit data number  */

volatile uint16_t g_sci8_tx_count;

/* SCI8 receive data number */

volatile uint16_t g_sci8_rx_count;

/* SCI8 receive data length */

volatile uint16_t g_sci8_rx_length;
#endif

#ifdef CONFIG_RX65N_SCI9
/* SCI9 transmit buffer address */

volatile uint8_t *gp_sci9_tx_address;

/* SCI9 receive buffer address */

volatile uint8_t *gp_sci9_rx_address;

/* SCI9 transmit data number */

volatile uint16_t g_sci9_tx_count;

/* SCI9 receive data number */

volatile uint16_t g_sci9_rx_count;

/* SCI9 receive data length */

volatile uint16_t g_sci9_rx_length;
#endif

#ifdef CONFIG_RX65N_SCI10
/* SCI10 transmit buffer address */

volatile uint8_t *gp_sci10_tx_address;

/* SCI10 receive buffer address */

volatile uint8_t *gp_sci10_rx_address;

/* SCI10 transmit data number */

volatile uint16_t g_sci10_tx_count;

/* SCI10 receive data number */

volatile uint16_t g_sci10_rx_count;

/* SCI10 receive data length */

volatile uint16_t g_sci10_rx_length;
#endif

#ifdef CONFIG_RX65N_SCI11
/* SCI11 transmit buffer address */

volatile uint8_t *gp_sci11_tx_address;

/* SCI11 receive buffer address */

volatile uint8_t *gp_sci11_rx_address;

/* SCI11 transmit data number */

volatile uint16_t g_sci11_tx_count;

/* SCI11 receive data number */

volatile uint16_t g_sci11_rx_count;

/* SCI11 receive data length */

volatile uint16_t g_sci11_rx_length;
#endif

#ifdef CONFIG_RX65N_SCI12
/* SCI12 transmit buffer address */

volatile uint8_t *gp_sci12_tx_address;

/* SCI12 receive buffer address */

volatile uint8_t *gp_sci12_rx_address;

/* SCI12 transmit data number */

volatile uint16_t g_sci12_tx_count;

/* SCI12 receive data number */

volatile uint16_t g_sci12_rx_count;

/* SCI12 receive data length */

volatile uint16_t g_sci12_rx_length;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rx_mpc_enable
 *
 * Description:
 * Enable writing to registers
 ****************************************************************************/

static inline void rx_mpc_enable(void)
{
/* Enable writing to registers related to operating modes,
 * LPC, CGC and software reset
 */

  SYSTEM.PRCR.WORD = 0xa50bu;

  /* Enable writing to MPC pin function control registers */

  MPC.PWPR.BIT.B0WI = 0u;
  MPC.PWPR.BIT.PFSWE = 1u;
}

/****************************************************************************
 * Name: rx_mpc_disable
 *
 * Description:
 * Disable writing to registers
 ****************************************************************************/

static inline void rx_mpc_disable(void)
{
  /* Disable writing to MPC pin function control registers */

  MPC.PWPR.BIT.PFSWE = 0u;
  MPC.PWPR.BIT.B0WI = 1u;

  /* Enable protection */

  SYSTEM.PRCR.WORD = 0xa500u;
}

/****************************************************************************
 * Name: sci4_init_port
 *
 * Description:
 * SCI4 Initialization
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI4
static inline void sci4_init_port(void)
{
  /* Set RXD4 pin (PXX)
   * MPC.PXXPFS.BYTE = 0x0au;
   * PORTX.PMR.BIT.BX = 1u;
   * Set TXD4 pin (PXX)
   * PORTX.PODR.BIT.BX = 1u;
   * MPC.PXXPFS.BYTE   = 0x0au;
   * PORTX.PDR.BIT.BX = 1u;
   * PORTX.PMR.BIT.BX = 1u;
   */
}
#endif

/****************************************************************************
 * Name: sci7_init_port
 *
 * Description:
 * SCI7 Initialization
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI7
static inline void sci7_init_port(void)
{
  /* Set RXD7 pin (PXX)
   * MPC.PXXPFS.BYTE = 0x0au;
   * PORTX.PMR.BIT.BX = 1u;
   * Set TXD7 pin (PXX)
   * PORTX.PODR.BIT.BX = 1u;
   * MPC.PXXPFS.BYTE   = 0x0au;
   * PORTX.PDR.BIT.BX = 1u;
   * PORTX.PMR.BIT.BX = 1u;
   */
}
#endif

/****************************************************************************
 * Name: sci9_init_port
 *
 * Description:
 * SCI9 Initialization
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI9
static inline void sci9_init_port(void)
{
  /* Set RXD9 pin (PXX)
   * MPC.PXXPFS.BYTE = 0x0au;
   * PORTX.PMR.BIT.BX = 1u;
   * Set TXD9 pin (PXX)
   * PORTX.PODR.BIT.BX = 1u;
   * MPC.PXXPFS.BYTE   = 0x0au;
   * PORTX.PDR.BIT.BX = 1u;
   * PORTX.PMR.BIT.BX = 1u;
   */
}
#endif

/****************************************************************************
 * Name: sci10_init_port
 *
 * Description:
 * SCI10 Initialization
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI10
static inline void sci10_init_port(void)
{
  /* Set RXD10 pin (PXX)
   * MPC.PXXPFS.BYTE = 0x0au;
   * PORTX.PMR.BIT.BX = 1u;
   * Set TXD10 pin (PXX)
   * PORTX.PODR.BIT.BX = 1u;
   * MPC.PXXPFS.BYTE   = 0x0au;
   * PORTX.PDR.BIT.BX = 1u;
   * PORTX.PMR.BIT.BX = 1u;
   */
}
#endif

/****************************************************************************
 * Name: sci11_init_port
 *
 * Description:
 * SCI11 Initialization
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI11
static inline void sci11_init_port(void)
{
  /* Set RXD11 pin (PXX)
   * MPC.PXXPFS.BYTE = 0x0au;
   * PORTX.PMR.BIT.BX = 1u;
   * Set TXD11 pin (PXX)
   * PORTX.PODR.BIT.BX = 1u;
   * MPC.PXXPFS.BYTE   = 0x0au;
   * PORTX.PDR.BIT.BX = 1u;
   * PORTX.PMR.BIT.BX = 1u;
   */
}
#endif

/****************************************************************************
 * Name: r_sci0_create
 *
 * Description:
 * SCI0 Initialization
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI0
void r_sci0_create(void)
{
  rx_mpc_enable();
  MSTP(SCI0)      = 0u;                  /* Cancel SCI0 module stop state */
  IPR(SCI0, RXI0) = 15;                  /* Set interrupt priority */
  IPR(SCI0, TXI0) = 15;                  /* Set interrupt priority */
  SCI0.SCR.BYTE   = 0u;                  /* Clear the control register */

  /* Set clock enable */

  SCI0.SCR.BYTE  = _00_SCI_INTERNAL_SCK_UNUSED;
  SCI0.SIMR1.BIT.IICM   = 0u;                   /* Clear SIMR1.IICM bit */
  SCI0.SPMR.BIT.CKPH    = 0u;                   /* Clear SPMR.CKPH bit */
  SCI0.SPMR.BIT.CKPOL   = 0u;                   /* Clear SPMR.CKPOL bit */

  /* Set control registers */

  SCI0.SPMR.BYTE  = _00_SCI_RTS;
  SCI0.SMR.BYTE   = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 |
                    _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
                    _00_SCI_MULTI_PROCESSOR_DISABLE |
                    _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
  SCI0.SCMR.BYTE  = _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST |
                    _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;

  SCI0.SEMR.BYTE  = _80_SCI_FALLING_EDGE_START_BIT |
                    _20_SCI_NOISE_FILTER_ENABLE |  _10_SCI_8_BASE_CLOCK |
                    _40_SCI_BAUDRATE_DOUBLE |
                    _04_SCI_BIT_MODULATION_ENABLE;
  SCI0.SNFR.BYTE  = _00_SCI_ASYNC_DIV_1;

  /* Set SCI0 pin */

  sci0_init_port();
  rx_mpc_disable();
}

/****************************************************************************
 * Name: r_sci0_start
 *
 * Description:
 * Starts SCI0
 ****************************************************************************/

void r_sci0_start(void)
{
  IR(SCI0, TXI0)  = 0u;  /* Clear interrupt flag */
  IR(SCI0, RXI0)  = 0u;  /* Clear interrupt flag */
  IEN(SCI0, TXI0) = 1u;  /* Enable SCI interrupt */
  IEN(SCI0, RXI0) = 1u;  /* Enable SCI interrupt */
  ICU.GENBL0.BIT.EN0 = 1u;
  ICU.GENBL0.BIT.EN1 = 1u;
}

/****************************************************************************
 * Name: r_sci0_stop
 *
 * Description:
 * Stops SCI0
 ****************************************************************************/

void r_sci0_stop(void)
{
  SCI0.SCR.BIT.TE  = 0u;   /* Disable serial transmit */
  SCI0.SCR.BIT.RE  = 0u;   /* Disable serial receive */
  SCI0.SCR.BIT.TIE = 0u;   /* disable TXI interrupt */
  SCI0.SCR.BIT.RIE = 0u;   /* disable RXI and ERI interrupt */
  IEN(SCI0, TXI0)  = 0u;
  ICU.GENBL0.BIT.EN0 = 0u;
  IR(SCI0, TXI0)  = 0u;
  IEN(SCI0, RXI0) = 0u;
  ICU.GENBL0.BIT.EN1 = 0u;
  IR(SCI0, RXI0) = 0u;
}

/****************************************************************************
 * Name: r_sci0_serial_receive
 *
 * Description:
 * Receives SCI0
 ****************************************************************************/

MD_STATUS r_sci0_serial_receive(uint8_t * const rx_buf, uint16_t rx_num)
{
  if (1u > rx_num)
    {
      return MD_ARGERROR;
    }

  g_sci0_rx_count  = 0u;
  g_sci0_rx_length = rx_num;
  gp_sci0_rx_address = rx_buf;
  SCI0.SCR.BIT.RIE  = 1u;
  SCI0.SCR.BIT.RE   = 1u;
  return OK;
}

/****************************************************************************
 * Name: r_sci0_serial_send
 *
 * Description:
 * Transmits SCI0 data
 ****************************************************************************/

MD_STATUS r_sci0_serial_send(uint8_t * const tx_buf, uint16_t tx_num)
{
  if (1u > tx_num)
    {
      return MD_ARGERROR;
    }

  gp_sci0_tx_address = tx_buf;
  g_sci0_tx_count    = tx_num;

  /* Set TXD0 pin */

  SCI0.SCR.BIT.TIE = 1u;
  SCI0.SCR.BIT.TE  = 1u;
  return OK;
}
#endif

/****************************************************************************
 * Name: r_sci1_create
 *
 * Description:
 * Initializes SCI1 data
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI1
void r_sci1_create(void)
{
  rx_mpc_enable();
  MSTP(SCI1)   = 0u;                    /* Cancel SCI1 module stop state */
  IPR(SCI1, RXI1) = 15;                 /* Set interrupt priority */
  IPR(SCI1, TXI1) = 15;                 /* Set interrupt priority */
  SCI1.SCR.BYTE   = 0u;                 /* Clear the control register */

  /* Set clock enable */

  SCI1.SCR.BYTE         = _00_SCI_INTERNAL_SCK_UNUSED;
  SCI1.SIMR1.BIT.IICM   = 0u;           /* Clear SIMR1.IICM bit */
  SCI1.SPMR.BIT.CKPH    = 0u;           /* Clear SPMR.CKPH bit */
  SCI1.SPMR.BIT.CKPOL   = 0u;           /* Clear SPMR.CKPOL bit */

  /* Set control registers */

  SCI1.SPMR.BYTE    = _00_SCI_RTS;
  SCI1.SMR.BYTE     = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 |
                      _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
                      _00_SCI_MULTI_PROCESSOR_DISABLE |
                      _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
  SCI1.SCMR.BYTE    = _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST |
                      _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
  SCI1.SEMR.BYTE    = _80_SCI_FALLING_EDGE_START_BIT |
                      _20_SCI_NOISE_FILTER_ENABLE |
                      _10_SCI_8_BASE_CLOCK | _40_SCI_BAUDRATE_DOUBLE |
                      _04_SCI_BIT_MODULATION_ENABLE;
  SCI1.SNFR.BYTE    = _00_SCI_ASYNC_DIV_1;

  /* Set SCI1 pin */

  sci1_init_port();
  rx_mpc_disable();
}

/****************************************************************************
 * Name: r_sci1_start
 *
 * Description:
 * Starts SCI1
 ****************************************************************************/

void r_sci1_start(void)
{
  IR(SCI1, TXI1) = 0u;   /* Clear interrupt flag */
  IR(SCI1, RXI1) = 0u;   /* Clear interrupt flag */
  IEN(SCI1, TXI1) = 1u;  /* Enable SCI interrupt */
  IEN(SCI1, RXI1) = 1u;  /* Enable SCI interrupt */
  ICU.GENBL0.BIT.EN2 = 1u;
  ICU.GENBL0.BIT.EN3 = 1u;
}

/****************************************************************************
 * Name: r_sci1_stop
 *
 * Description:
 * Stops SCI1
 ****************************************************************************/

void r_sci1_stop(void)
{
  SCI1.SCR.BIT.TE  = 0u;        /* Disable serial transmit */
  SCI1.SCR.BIT.RE  = 0u;        /* Disable serial receive */
  SCI1.SCR.BIT.TIE = 0u;        /* disable TXI interrupt */
  SCI1.SCR.BIT.RIE = 0u;        /* disable RXI and ERI interrupt */
  IEN(SCI1, TXI1)  = 0u;
  ICU.GENBL0.BIT.EN2 = 0u;
  IR(SCI1, TXI1)  = 0u;
  IEN(SCI1, RXI1) = 0u;
  ICU.GENBL0.BIT.EN3 = 0u;
  IR(SCI1, RXI1) = 0u;
}

/****************************************************************************
 * Name: r_sci1_serial_receive
 *
 * Description:
 * Receives SCI1 data
 ****************************************************************************/

MD_STATUS r_sci1_serial_receive(uint8_t * const rx_buf, uint16_t rx_num)
{
  if (1u > rx_num)
    {
      return MD_ARGERROR;
    }

  g_sci1_rx_count  = 0u;
  g_sci1_rx_length = rx_num;
  gp_sci1_rx_address = rx_buf;
  SCI1.SCR.BIT.RIE  = 1u;
  SCI1.SCR.BIT.RE   = 1u;
  return OK;
}

/****************************************************************************
 * Name: r_sci1_serial_send
 *
 * Description:
 * Transmit SCI1 data
 ****************************************************************************/

MD_STATUS r_sci1_serial_send(uint8_t * const tx_buf, uint16_t tx_num)
{
  if (1u > tx_num)
    {
      return MD_ARGERROR;
    }

  gp_sci1_tx_address = tx_buf;
  g_sci1_tx_count    = tx_num;

  /* Set TXD1 pin */

  SCI1.SCR.BIT.TIE = 1u;
  SCI1.SCR.BIT.TE  = 1u;
  return OK;
}
#endif

/****************************************************************************
 * Name: r_sci2_create
 *
 * Description:
 * Initialize SCI2
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI2
void r_sci2_create(void)
{
  rx_mpc_enable();
  MSTP(SCI2)    = 0u;                   /* Cancel SCI2 module stop state */
  IPR(SCI2, RXI2) = 15;                 /* Set interrupt priority */
  IPR(SCI2, TXI2) = 15;                 /* Set interrupt priority */
  SCI2.SCR.BYTE  = 0u;                  /* Clear the control register */

  /* Set clock enable */

  SCI2.SCR.BYTE         = _00_SCI_INTERNAL_SCK_UNUSED;
  SCI2.SIMR1.BIT.IICM   = 0u;                   /* Clear SIMR1.IICM bit */
  SCI2.SPMR.BIT.CKPH    = 0u;                   /* Clear SPMR.CKPH bit */
  SCI2.SPMR.BIT.CKPOL   = 0u;                   /* Clear SPMR.CKPOL bit */

  /* Set control registers */

  SCI2.SPMR.BYTE   = _00_SCI_RTS;
  SCI2.SMR.BYTE    = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 |
                     _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
                     _00_SCI_MULTI_PROCESSOR_DISABLE |
                     _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
  SCI2.SCMR.BYTE   = _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST |
                     _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
  SCI2.SEMR.BYTE   = _80_SCI_FALLING_EDGE_START_BIT |
                     _20_SCI_NOISE_FILTER_ENABLE | _10_SCI_8_BASE_CLOCK |
                     _40_SCI_BAUDRATE_DOUBLE | _04_SCI_BIT_MODULATION_ENABLE;
  SCI2.SNFR.BYTE   = _00_SCI_ASYNC_DIV_1;

  /* Set SCI2 pin */

  sci2_init_port();
  rx_mpc_disable();
}

/****************************************************************************
 * Name: r_sci2_start
 *
 * Description:
 * Start SCI2
 ****************************************************************************/

void r_sci2_start(void)
{
  IR(SCI2, TXI2) = 0u;   /* Clear interrupt flag */
  IR(SCI2, RXI2) = 0u;   /* Clear interrupt flag */
  IEN(SCI2, TXI2) = 1u;  /* Enable SCI interrupt */
  IEN(SCI2, RXI2) = 1u;  /* Enable SCI interrupt */
  ICU.GENBL0.BIT.EN4 = 1u;
  ICU.GENBL0.BIT.EN5 = 1u;
}

/****************************************************************************
 * Name: r_sci2_stop
 *
 * Description:
 * Stops SCI2
 ****************************************************************************/

void r_sci2_stop(void)
{
  SCI2.SCR.BIT.TE  = 0u;   /* Disable serial transmit */
  SCI2.SCR.BIT.RE  = 0u;   /* Disable serial receive */
  SCI2.SCR.BIT.TIE = 0u;   /* disable TXI interrupt */
  SCI2.SCR.BIT.RIE = 0u;   /* disable RXI and ERI interrupt */
  IEN(SCI2, TXI2)  = 0u;
  ICU.GENBL0.BIT.EN4 = 0u;
  IR(SCI2, TXI2)  = 0u;
  IEN(SCI2, RXI2) = 0u;
  ICU.GENBL0.BIT.EN5 = 0u;
  IR(SCI2, RXI2) = 0u;
}

/****************************************************************************
 * Name: r_sci2_serial_receive
 *
 * Description:
 * Receives SCI2 data
 ****************************************************************************/

MD_STATUS r_sci2_serial_receive(uint8_t * const rx_buf, uint16_t rx_num)
{
  if (1u > rx_num)
    {
      return MD_ARGERROR;
    }

  g_sci2_rx_count  = 0u;
  g_sci2_rx_length = rx_num;
  gp_sci2_rx_address = rx_buf;
  SCI2.SCR.BIT.RIE = 1u;
  SCI2.SCR.BIT.RE  = 1u;
  return OK;
}

/****************************************************************************
 * Name: r_sci2_serial_send
 *
 * Description:
 * Send SCI2 data
 ****************************************************************************/

MD_STATUS r_sci2_serial_send(uint8_t * const tx_buf, uint16_t tx_num)
{
  if (1u > tx_num)
    {
      return MD_ARGERROR;
    }

  gp_sci2_tx_address = tx_buf;
  g_sci2_tx_count    = tx_num;

  /* Set TXD2 pin */

  SCI2.SCR.BIT.TIE = 1u;
  SCI2.SCR.BIT.TE  = 1u;
  return OK;
}
#endif

/****************************************************************************
 * Name: r_sci3_create
 *
 * Description:
 * Initializes SCI3
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI3
void r_sci3_create(void)
{
  rx_mpc_enable();
  MSTP(SCI3)     = 0u;                  /* Cancel SCI3 module stop state */
  IPR(SCI3, RXI3) = 15;                 /* Set interrupt priority */
  IPR(SCI3, TXI3) = 15;                 /* Set interrupt priority */
  SCI3.SCR.BYTE  = 0u;                  /* Clear the control register */

  /* Set clock enable */

  SCI3.SCR.BYTE         = _00_SCI_INTERNAL_SCK_UNUSED;
  SCI3.SIMR1.BIT.IICM   = 0u;                   /* Clear SIMR1.IICM bit */
  SCI3.SPMR.BIT.CKPH    = 0u;                   /* Clear SPMR.CKPH bit */
  SCI3.SPMR.BIT.CKPOL   = 0u;                   /* Clear SPMR.CKPOL bit */

  /* Set control registers */

  SCI3.SPMR.BYTE  = _00_SCI_RTS;
  SCI3.SMR.BYTE   = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 |
                    _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
                    _00_SCI_MULTI_PROCESSOR_DISABLE |
                    _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
  SCI3.SCMR.BYTE  = _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST |
                    _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
  SCI3.SEMR.BYTE  = _80_SCI_FALLING_EDGE_START_BIT |
                    _20_SCI_NOISE_FILTER_ENABLE |
                    _10_SCI_8_BASE_CLOCK | _40_SCI_BAUDRATE_DOUBLE |
                    _04_SCI_BIT_MODULATION_ENABLE;
  SCI3.SNFR.BYTE  = _00_SCI_ASYNC_DIV_1;

  /* Set SCI3 pin */

  sci3_init_port();
  rx_mpc_disable();
}

/****************************************************************************
 * Name: r_sci3_create
 *
 * Description:
 * Initializes SCI3
 ****************************************************************************/

void r_sci3_start(void)
{
  IR(SCI3, TXI3)  = 0u;  /* Clear interrupt flag */
  IR(SCI3, RXI3)  = 0u;  /* Clear interrupt flag */
  IEN(SCI3, TXI3) = 1u;  /* Enable SCI interrupt */
  IEN(SCI3, RXI3) = 1u;  /* Enable SCI interrupt */
  ICU.GENBL0.BIT.EN6 = 0u;
  ICU.GENBL0.BIT.EN7 = 1u;
}

/****************************************************************************
 * Name: r_sci3_stop
 *
 * Description:
 * Stops SCI3
 ****************************************************************************/

void r_sci3_stop(void)
{
  SCI3.SCR.BIT.TE  = 0u; /* Disable serial transmit */
  SCI3.SCR.BIT.RE  = 0u; /* Disable serial receive */
  SCI3.SCR.BIT.TIE = 0u; /* disable TXI interrupt */
  SCI3.SCR.BIT.RIE = 0u; /* disable RXI and ERI interrupt */
  IEN(SCI3, TXI3)  = 0u;
  ICU.GENBL0.BIT.EN6 = 0u;
  IR(SCI3, TXI3)  = 0u;
  IEN(SCI3, RXI3) = 0u;
  ICU.GENBL0.BIT.EN7 = 0u;
  IR(SCI3, RXI3) = 0u;
}

/****************************************************************************
 * Name: r_sci3_serial_receive
 *
 * Description:
 * Receives SCI3 data
 ****************************************************************************/

MD_STATUS r_sci3_serial_receive(uint8_t * const rx_buf, uint16_t rx_num)
{
  if (1u > rx_num)
    {
      return MD_ARGERROR;
    }

  g_sci3_rx_count = 0u;
  g_sci3_rx_length = rx_num;
  gp_sci3_rx_address = rx_buf;
  SCI3.SCR.BIT.RIE = 1u;
  SCI3.SCR.BIT.RE  = 1u;
  return OK;
}

/****************************************************************************
 * Name: r_sci3_serial_send
 *
 * Description:
 * Send SCI3 data
 ****************************************************************************/

MD_STATUS r_sci3_serial_send(uint8_t * const tx_buf, uint16_t tx_num)
{
  if (1u > tx_num)
    {
      return MD_ARGERROR;
    }

  gp_sci3_tx_address = tx_buf;
  g_sci3_tx_count = tx_num;

  /* Set TXD3 pin */

  SCI3.SCR.BIT.TIE = 1u;
  SCI3.SCR.BIT.TE  = 1u;
  return OK;
}
#endif

/****************************************************************************
 * Name: r_sci4_create
 *
 * Description:
 * Initialize SCI4 data
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI4
void r_sci4_create(void)
{
  rx_mpc_enable();
  MSTP(SCI4) = 0u;              /* Cancel SCI4 module stop state */
  IPR(SCI4, RXI4) = 15;         /* Set interrupt priority */
  IPR(SCI4, TXI4) = 15;         /* Set interrupt priority */
  SCI4.SCR.BYTE  = 0u;          /* Clear the control register */

  /* Set clock enable */

  SCI4.SCR.BYTE = _00_SCI_INTERNAL_SCK_UNUSED;
  SCI4.SIMR1.BIT.IICM = 0u;                     /* Clear SIMR1.IICM bit */
  SCI4.SPMR.BIT.CKPH  = 0u;                     /* Clear SPMR.CKPH bit */
  SCI4.SPMR.BIT.CKPOL = 0u;                     /* Clear SPMR.CKPOL bit */

  /* Set control registers */

  SCI4.SPMR.BYTE = _00_SCI_RTS;
  SCI4.SMR.BYTE  = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 |
                   _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
                   _00_SCI_MULTI_PROCESSOR_DISABLE |
                   _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
  SCI4.SCMR.BYTE = _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST |
                   _10_SCI_DATA_LENGTH_8_OR_7 |
                   _62_SCI_SCMR_DEFAULT;
  SCI4.SEMR.BYTE = _80_SCI_FALLING_EDGE_START_BIT |
                   _20_SCI_NOISE_FILTER_ENABLE |
                   _10_SCI_8_BASE_CLOCK | _40_SCI_BAUDRATE_DOUBLE |
                   _04_SCI_BIT_MODULATION_ENABLE;
  SCI4.SNFR.BYTE = _00_SCI_ASYNC_DIV_1;

  /* Set SCI4 pin */

  sci4_init_port();
  rx_mpc_disable();
}

/****************************************************************************
 * Name: r_sci4_start
 *
 * Description:
 * Start SCI4
 ****************************************************************************/

void r_sci4_start(void)
{
  rx_mpc_enable();
  IR(SCI4, TXI4) = 0u;   /* Clear interrupt flag */
  IR(SCI4, RXI4) = 0u;   /* Clear interrupt flag */
  IEN(SCI4, TXI4) = 1u;  /* Enable SCI interrupt */
  IEN(SCI4, RXI4) = 1u;  /* Enable SCI interrupt */
  ICU.GENBL0.BIT.EN8 = 0u;
  ICU.GENBL0.BIT.EN9 = 1u;
}

/****************************************************************************
 * Name: r_sci4_stop
 *
 * Description:
 * Stop SCI4
 ****************************************************************************/

void r_sci4_stop(void)
{
  SCI4.SCR.BIT.TE  = 0u;        /* Disable serial transmit */
  SCI4.SCR.BIT.RE  = 0u;        /* Disable serial receive */
  SCI4.SCR.BIT.TIE = 0u;        /* disable TXI interrupt */
  SCI4.SCR.BIT.RIE = 0u;        /* disable RXI and ERI interrupt */
  IEN(SCI4, TXI4)  = 0u;
  ICU.GENBL0.BIT.EN8 = 0u;
  IR(SCI4, TXI4)  = 0u;
  IEN(SCI4, RXI4) = 0u;
  ICU.GENBL0.BIT.EN9 = 0u;
  IR(SCI4, RXI4) = 0u;
}

/****************************************************************************
 * Name: r_sci4_serial_receive
 *
 * Description:
 * Receive SCI4 data
 ****************************************************************************/

MD_STATUS r_sci4_serial_receive(uint8_t * const rx_buf, uint16_t rx_num)
{
  if (1u > rx_num)
    {
      return MD_ARGERROR;
    }

  g_sci4_rx_count  = 0u;
  g_sci4_rx_length = rx_num;
  gp_sci4_rx_address = rx_buf;
  SCI4.SCR.BIT.RIE = 1u;
  SCI4.SCR.BIT.RE  = 1u;
  return OK;
}

/****************************************************************************
 * Name: r_sci4_serial_send
 *
 * Description:
 * Send SCI4 data
 ****************************************************************************/

MD_STATUS r_sci4_serial_send(uint8_t * const tx_buf, uint16_t tx_num)
{
  if (1u > tx_num)
    {
      return MD_ARGERROR;
    }

  gp_sci4_tx_address = tx_buf;
  g_sci4_tx_count    = tx_num;

  /* Set TXD4 pin */

  SCI4.SCR.BIT.TIE = 1u;
  SCI4.SCR.BIT.TE  = 1u;
  return OK;
}
#endif

/****************************************************************************
 * Name: r_sci5_create
 *
 * Description:
 * SCI5 Initialization
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI5
void r_sci5_create(void)
{
  rx_mpc_enable();
  MSTP(SCI5)     = 0u;                  /* Cancel SCI0 module stop state */
  IPR(SCI5, RXI5) = 15;                 /* Set interrupt priority */
  IPR(SCI5, TXI5) = 15;                 /* Set interrupt priority */
  SCI5.SCR.BYTE  = 0u;                  /* Clear the control register */

  /* Set clock enable */

  SCI5.SCR.BYTE         = _00_SCI_INTERNAL_SCK_UNUSED;
  SCI5.SIMR1.BIT.IICM   = 0u;                   /* Clear SIMR1.IICM bit */
  SCI5.SPMR.BIT.CKPH    = 0u;                   /* Clear SPMR.CKPH bit */
  SCI5.SPMR.BIT.CKPOL   = 0u;                   /* Clear SPMR.CKPOL bit */

  /* Set control registers */

  SCI5.SPMR.BYTE = _00_SCI_RTS;
  SCI5.SMR.BYTE  = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 |
                   _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
                   _00_SCI_MULTI_PROCESSOR_DISABLE |
                   _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
  SCI5.SCMR.BYTE = _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST |
                   _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
  SCI5.SEMR.BYTE = _80_SCI_FALLING_EDGE_START_BIT |
                   _20_SCI_NOISE_FILTER_ENABLE | _10_SCI_8_BASE_CLOCK |
                                   _40_SCI_BAUDRATE_DOUBLE |
                   _04_SCI_BIT_MODULATION_ENABLE;
  SCI5.SNFR.BYTE = _00_SCI_ASYNC_DIV_1;

  /* Set SCI5 pin */

  sci5_init_port();
  rx_mpc_disable();
}

/****************************************************************************
 * Name: r_sci5_start
 *
 * Description:
 * Start SCI5
 ****************************************************************************/

void r_sci5_start(void)
{
  IR(SCI5, TXI5) = 0u;   /* Clear interrupt flag */
  IR(SCI5, RXI5) = 0u;   /* Clear interrupt flag */
  IEN(SCI5, TXI5) = 1u;  /* Enable SCI interrupt */
  IEN(SCI5, RXI5) = 1u;  /* Enable SCI interrupt */
  ICU.GENBL0.BIT.EN10 = 0u;
  ICU.GENBL0.BIT.EN11 = 1u;
}

/****************************************************************************
 * Name: r_sci5_stop
 *
 * Description:
 * Stop SCI5
 ****************************************************************************/

void r_sci5_stop(void)
{
  SCI5.SCR.BIT.TE  = 0u;     /* Disable serial transmit */
  SCI5.SCR.BIT.RE  = 0u;     /* Disable serial receive */
  SCI5.SCR.BIT.TIE = 0u;     /* disable TXI interrupt */
  SCI5.SCR.BIT.RIE = 0u;     /* disable RXI and ERI interrupt */
  IEN(SCI5, TXI5)  = 0u;
  ICU.GENBL0.BIT.EN10 = 0u;
  IR(SCI5, TXI5) = 0u;
  IEN(SCI5, RXI5) = 0u;
  ICU.GENBL0.BIT.EN11 = 0u;
  IR(SCI5, RXI5) = 0u;
}

/****************************************************************************
 * Name: r_sci5_serial_receive
 *
 * Description:
 * Receive SCI5
 ****************************************************************************/

MD_STATUS r_sci5_serial_receive(uint8_t * const rx_buf, uint16_t rx_num)
{
  if (1u > rx_num)
    {
      return MD_ARGERROR;
    }

  g_sci5_rx_count = 0u;
  g_sci5_rx_length = rx_num;
  gp_sci5_rx_address = rx_buf;
  SCI5.SCR.BIT.RIE = 1u;
  SCI5.SCR.BIT.RE  = 1u;
  return OK;
}

/****************************************************************************
 * Name: r_sci5_serial_send
 *
 * Description:
 * Transmit SCI5 data
 ****************************************************************************/

MD_STATUS r_sci5_serial_send(uint8_t * const tx_buf, uint16_t tx_num)
{
  if (1u > tx_num)
    {
      return MD_ARGERROR;
    }

  gp_sci5_tx_address = tx_buf;
  g_sci5_tx_count    = tx_num;

  /* Set TXD5 pin */

  SCI5.SCR.BIT.TIE      = 1u;
  SCI5.SCR.BIT.TE       = 1u;
  return OK;
}
#endif

/****************************************************************************
 * Name: r_sci6_create
 *
 * Description:
 * Initialization of SCI6
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI6
void r_sci6_create(void)
{
  rx_mpc_enable();
  MSTP(SCI6)      = 0u;                  /* Cancel SCI0 module stop state */
  IPR(SCI6, RXI6) = 15;                  /* Set interrupt priority */
  IPR(SCI6, TXI6) = 15;                  /* Set interrupt priority */
  SCI6.SCR.BYTE   = 0u;                  /* Clear the control register */

  /* Set clock enable */

  SCI6.SCR.BYTE   = _00_SCI_INTERNAL_SCK_UNUSED;
  SCI6.SIMR1.BIT.IICM   = 0u;                   /* Clear SIMR1.IICM bit */
  SCI6.SPMR.BIT.CKPH    = 0u;                   /* Clear SPMR.CKPH bit */
  SCI6.SPMR.BIT.CKPOL   = 0u;                   /* Clear SPMR.CKPOL bit */

  /* Set control registers */

  SCI6.SPMR.BYTE = _00_SCI_RTS;
  SCI6.SMR.BYTE  = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 |
                   _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
                   _00_SCI_MULTI_PROCESSOR_DISABLE |
                   _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
  SCI6.SCMR.BYTE = _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST |
                   _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
  SCI6.SEMR.BYTE = _80_SCI_FALLING_EDGE_START_BIT |
                   _20_SCI_NOISE_FILTER_ENABLE |
                   _10_SCI_8_BASE_CLOCK | _40_SCI_BAUDRATE_DOUBLE |
                   _04_SCI_BIT_MODULATION_ENABLE;
  SCI6.SNFR.BYTE = _00_SCI_ASYNC_DIV_1;

  /* Set SCI6 pin */

  sci6_init_port();
  rx_mpc_disable();
}

/****************************************************************************
 * Name: r_sci6_start
 *
 * Description:
 * Start SCI6
 ****************************************************************************/

void r_sci6_start(void)
{
  IR(SCI6, TXI6) = 0u;   /* Clear interrupt flag */
  IR(SCI6, RXI6) = 0u;   /* Clear interrupt flag */
  IEN(SCI6, TXI6) = 1u;  /* Enable SCI interrupt */
  IEN(SCI6, RXI6) = 1u;  /* Enable SCI interrupt */
  ICU.GENBL0.BIT.EN12 = 0u;
  ICU.GENBL0.BIT.EN13 = 1u;
}

/****************************************************************************
 * Name: r_sci6_stop
 *
 * Description:
 * Stop SCI6
 ****************************************************************************/

void r_sci6_stop(void)
{
  SCI6.SCR.BIT.TE = 0u;     /* Disable serial transmit */
  SCI6.SCR.BIT.RE = 0u;     /* Disable serial receive */
  SCI6.SCR.BIT.TIE = 0u;    /* disable TXI interrupt */
  SCI6.SCR.BIT.RIE = 0u;    /* disable RXI and ERI interrupt */
  IEN(SCI6, TXI6)  = 0u;
  ICU.GENBL0.BIT.EN12 = 0u;
  IR(SCI6, TXI6) = 0u;
  IEN(SCI6, RXI6) = 0u;
  ICU.GENBL0.BIT.EN13 = 0u;
  IR(SCI6, RXI6) = 0u;
}

/****************************************************************************
 * Name: r_sci6_serial_receive
 *
 * Description:
 * Stop SCI6
 ****************************************************************************/

MD_STATUS r_sci6_serial_receive(uint8_t * const rx_buf, uint16_t rx_num)
{
  if (1u > rx_num)
    {
      return MD_ARGERROR;
    }

  g_sci6_rx_count = 0u;
  g_sci6_rx_length = rx_num;
  gp_sci6_rx_address = rx_buf;
  SCI6.SCR.BIT.RIE = 1u;
  SCI6.SCR.BIT.RE  = 1u;
  return OK;
}

/****************************************************************************
 * Name: r_sci6_serial_send
 *
 * Description:
 * Stop SCI6
 ****************************************************************************/

MD_STATUS r_sci6_serial_send(uint8_t * const tx_buf, uint16_t tx_num)
{
  if (1u > tx_num)
    {
      return MD_ARGERROR;
    }

  gp_sci6_tx_address = tx_buf;
  g_sci6_tx_count    = tx_num;

  /* Set TXD0 pin */

  SCI6.SCR.BIT.TIE = 1u;
  SCI6.SCR.BIT.TE  = 1u;
  return OK;
}
#endif

/****************************************************************************
 * Name: r_sci7_create
 *
 * Description:
 * SCI7
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI7
void r_sci7_create(void)
{
  rx_mpc_enable();
  MSTP(SCI7)      = 0u;                 /* Cancel SCI7 module stop state */
  IPR(SCI7, RXI7) = 15;                 /* Set interrupt priority */
  IPR(SCI7, TXI7) = 15;                 /* Set interrupt priority */
  SCI7.SCR.BYTE  = 0u;                  /* Clear the control register */

  /* Set clock enable */

  SCI7.SCR.BYTE         = _00_SCI_INTERNAL_SCK_UNUSED;
  SCI7.SIMR1.BIT.IICM   = 0u;   /* Clear SIMR1.IICM bit */
  SCI7.SPMR.BIT.CKPH    = 0u;   /* Clear SPMR.CKPH bit */
  SCI7.SPMR.BIT.CKPOL   = 0u;   /* Clear SPMR.CKPOL bit */

  /* Set control registers */

  SCI7.SPMR.BYTE = _00_SCI_RTS;
  SCI7.SMR.BYTE  = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 |
                   _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
                   _00_SCI_MULTI_PROCESSOR_DISABLE |
                   _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
  SCI7.SCMR.BYTE = _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST |
                   _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;

  SCI7.SEMR.BYTE = _80_SCI_FALLING_EDGE_START_BIT |
                   _20_SCI_NOISE_FILTER_ENABLE |
                   _10_SCI_8_BASE_CLOCK | _40_SCI_BAUDRATE_DOUBLE |
                   _04_SCI_BIT_MODULATION_ENABLE;
  SCI7.SNFR.BYTE  = _00_SCI_ASYNC_DIV_1;

  /* Set SCI7 pin */

  sci7_init_port();
  rx_mpc_disable();
}

/****************************************************************************
 * Name: r_sci7_start
 *
 * Description:
 * SCI7 Start
 ****************************************************************************/

void r_sci7_start(void)
{
  IR(SCI7, TXI7) = 0u;   /* Clear interrupt flag */
  IR(SCI7, RXI7) = 0u;   /* Clear interrupt flag */
  IEN(SCI7, TXI7) = 1u;  /* Enable SCI interrupt */
  IEN(SCI7, RXI7) = 1u;  /* Enable SCI interrupt */
  ICU.GENBL0.BIT.EN14 = 0u;
  ICU.GENBL0.BIT.EN15 = 1u;
}

/****************************************************************************
 * Name: r_sci7_stop
 *
 * Description:
 * SCI7 Stop
 ****************************************************************************/

void r_sci7_stop(void)
{
  SCI7.SCR.BIT.TE = 0u;    /* Disable serial transmit */
  SCI7.SCR.BIT.RE = 0u;    /* Disable serial receive */
  SCI7.SCR.BIT.TIE = 0u;   /* disable TXI interrupt */
  SCI7.SCR.BIT.RIE = 0u;   /* disable RXI and ERI interrupt */
  IEN(SCI7, TXI7) = 0u;
  IR(SCI7, TXI7)  = 0u;
  IEN(SCI7, RXI7) = 0u;
  IR(SCI7, RXI7)  = 0u;
  ICU.GENBL0.BIT.EN14 = 0u;
  ICU.GENBL0.BIT.EN15 = 0u;
}

/****************************************************************************
 * Name: r_sci7_serial_receive
 *
 * Description:
 * Receive SCI7 data
 ****************************************************************************/

MD_STATUS r_sci7_serial_receive(uint8_t * const rx_buf, uint16_t rx_num)
{
  if (1u > rx_num)
    {
      return MD_ARGERROR;
    }

  g_sci7_rx_count   = 0u;
  g_sci7_rx_length  = rx_num;
  gp_sci7_rx_address = rx_buf;
  SCI7.SCR.BIT.RIE   = 1u;
  SCI7.SCR.BIT.RE    = 1u;
  return OK;
}

/****************************************************************************
 * Name: r_sci7_serial_send
 *
 * Description:
 * Send SCI7 data
 ****************************************************************************/

MD_STATUS r_sci7_serial_send(uint8_t * const tx_buf, uint16_t tx_num)
{
  if (1u > tx_num)
    {
      return MD_ARGERROR;
    }

  gp_sci7_tx_address = tx_buf;
  g_sci7_tx_count    = tx_num;

  /* Set TXD0 pin */

  SCI7.SCR.BIT.TIE = 1u;
  SCI7.SCR.BIT.TE  = 1u;
  return OK;
}
#endif

/****************************************************************************
 * Name: r_sci8_create
 *
 * Description:
 * SCI8
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI8
void r_sci8_create(void)
{
  rx_mpc_enable();
  MSTP(SCI8)      = 0u;                 /* Cancel SCI8 module stop state */
  IPR(SCI8, RXI8) = 15;                 /* Set interrupt priority */
  IPR(SCI8, TXI8) = 15;                 /* Set interrupt priority */
  SCI8.SCR.BYTE  = 0u;                  /* Clear the control register */

  /* Set clock enable */

  SCI8.SCR.BYTE         = _00_SCI_INTERNAL_SCK_UNUSED;
  SCI8.SIMR1.BIT.IICM   = 0u;   /* Clear SIMR1.IICM bit */
  SCI8.SPMR.BIT.CKPH    = 0u;   /* Clear SPMR.CKPH bit */
  SCI8.SPMR.BIT.CKPOL   = 0u;   /* Clear SPMR.CKPOL bit */

  /* Set control registers */

  SCI8.SPMR.BYTE = _00_SCI_RTS;
  SCI8.SMR.BYTE  = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 |
                   _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
                   _00_SCI_MULTI_PROCESSOR_DISABLE |
                   _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
  SCI8.SCMR.BYTE = _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST |
                   _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;

  SCI8.SEMR.BYTE = _80_SCI_FALLING_EDGE_START_BIT |
                   _20_SCI_NOISE_FILTER_ENABLE |
                   _10_SCI_8_BASE_CLOCK | _40_SCI_BAUDRATE_DOUBLE |
                   _04_SCI_BIT_MODULATION_ENABLE;
  SCI8.SNFR.BYTE  = _00_SCI_ASYNC_DIV_1;

  /* Set SCI8 pin */

  sci8_init_port();
  rx_mpc_disable();
}

/****************************************************************************
 * Name: r_sci8_start
 *
 * Description:
 * SCI8 Start
 ****************************************************************************/

void r_sci8_start(void)
{
  IR(SCI8, TXI8) = 0u;   /* Clear interrupt flag */
  IR(SCI8, RXI8) = 0u;   /* Clear interrupt flag */
  IEN(SCI8, TXI8) = 1u;  /* Enable SCI interrupt */
  IEN(SCI8, RXI8) = 1u;  /* Enable SCI interrupt */
  ICU.GENBL1.BIT.EN24 = 0u;
  ICU.GENBL1.BIT.EN25 = 1u;
}

/****************************************************************************
 * Name: r_sci8_stop
 *
 * Description:
 * SCI8 Stop
 ****************************************************************************/

void r_sci8_stop(void)
{
  SCI8.SCR.BIT.TE = 0u;    /* Disable serial transmit */
  SCI8.SCR.BIT.RE = 0u;    /* Disable serial receive */
  SCI8.SCR.BIT.TIE = 0u;   /* disable TXI interrupt */
  SCI8.SCR.BIT.RIE = 0u;   /* disable RXI and ERI interrupt */
  IEN(SCI8, TXI8) = 0u;
  IR(SCI8, TXI8)  = 0u;
  IEN(SCI8, RXI8) = 0u;
  IR(SCI8, RXI8)  = 0u;
  ICU.GENBL1.BIT.EN24 = 0u;
  ICU.GENBL1.BIT.EN25 = 0u;
}

/****************************************************************************
 * Name: r_sci8_serial_receive
 *
 * Description:
 * Receive SCI8 data
 ****************************************************************************/

MD_STATUS r_sci8_serial_receive(uint8_t * const rx_buf, uint16_t rx_num)
{
  if (1u > rx_num)
    {
      return MD_ARGERROR;
    }

  g_sci8_rx_count   = 0u;
  g_sci8_rx_length  = rx_num;
  gp_sci8_rx_address = rx_buf;
  SCI8.SCR.BIT.RIE   = 1u;
  SCI8.SCR.BIT.RE    = 1u;
  return OK;
}

/****************************************************************************
 * Name: r_sci8_serial_send
 *
 * Description:
 * Send SCI8 data
 ****************************************************************************/

MD_STATUS r_sci8_serial_send(uint8_t * const tx_buf, uint16_t tx_num)
{
  if (1u > tx_num)
    {
      return MD_ARGERROR;
    }

  gp_sci8_tx_address = tx_buf;
  g_sci8_tx_count    = tx_num;

  /* Set TXD0 pin */

  SCI8.SCR.BIT.TIE = 1u;
  SCI8.SCR.BIT.TE  = 1u;
  return OK;
}
#endif

/****************************************************************************
 * Name: r_sci9_create
 *
 * Description:
 * SCI9
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI9
void r_sci9_create(void)
{
  rx_mpc_enable();
  MSTP(SCI9)      = 0u;                 /* Cancel SCI9 module stop state */
  IPR(SCI9, RXI9) = 15;                 /* Set interrupt priority */
  IPR(SCI9, TXI9) = 15;                 /* Set interrupt priority */
  SCI9.SCR.BYTE  = 0u;                  /* Clear the control register */

  /* Set clock enable */

  SCI9.SCR.BYTE         = _00_SCI_INTERNAL_SCK_UNUSED;
  SCI9.SIMR1.BIT.IICM   = 0u;   /* Clear SIMR1.IICM bit */
  SCI9.SPMR.BIT.CKPH    = 0u;   /* Clear SPMR.CKPH bit */
  SCI9.SPMR.BIT.CKPOL   = 0u;   /* Clear SPMR.CKPOL bit */

  /* Set control registers */

  SCI9.SPMR.BYTE = _00_SCI_RTS;
  SCI9.SMR.BYTE  = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 |
                   _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
                   _00_SCI_MULTI_PROCESSOR_DISABLE |
                   _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
  SCI9.SCMR.BYTE = _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST |
                   _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;

  SCI9.SEMR.BYTE = _80_SCI_FALLING_EDGE_START_BIT |
                   _20_SCI_NOISE_FILTER_ENABLE |
                   _10_SCI_8_BASE_CLOCK | _40_SCI_BAUDRATE_DOUBLE |
                   _04_SCI_BIT_MODULATION_ENABLE;
  SCI9.SNFR.BYTE  = _00_SCI_ASYNC_DIV_1;

  /* Set SCI9 pin */

  sci9_init_port();
  rx_mpc_disable();
}

/****************************************************************************
 * Name: r_sci9_start
 *
 * Description:
 * SCI9 Start
 ****************************************************************************/

void r_sci9_start(void)
{
  IR(SCI9, TXI9) = 0u;   /* Clear interrupt flag */
  IR(SCI9, RXI9) = 0u;   /* Clear interrupt flag */
  IEN(SCI9, TXI9) = 1u;  /* Enable SCI interrupt */
  IEN(SCI9, RXI9) = 1u;  /* Enable SCI interrupt */
  ICU.GENBL1.BIT.EN26 = 0u;
  ICU.GENBL1.BIT.EN27 = 1u;
}

/****************************************************************************
 * Name: r_sci9_stop
 *
 * Description:
 * SCI9 Stop
 ****************************************************************************/

void r_sci9_stop(void)
{
  SCI9.SCR.BIT.TE = 0u;    /* Disable serial transmit */
  SCI9.SCR.BIT.RE = 0u;    /* Disable serial receive */
  SCI9.SCR.BIT.TIE = 0u;   /* disable TXI interrupt */
  SCI9.SCR.BIT.RIE = 0u;   /* disable RXI and ERI interrupt */
  IEN(SCI9, TXI9) = 0u;
  IR(SCI9, TXI9)  = 0u;
  IEN(SCI9, RXI9) = 0u;
  IR(SCI9, RXI9)  = 0u;
  ICU.GENBL1.BIT.EN26 = 0u;
  ICU.GENBL1.BIT.EN27 = 0u;
}

/****************************************************************************
 * Name: r_sci9_serial_receive
 *
 * Description:
 * Receive SCI9 data
 ****************************************************************************/

MD_STATUS r_sci9_serial_receive(uint8_t * const rx_buf, uint16_t rx_num)
{
  if (1u > rx_num)
    {
      return MD_ARGERROR;
    }

  g_sci9_rx_count   = 0u;
  g_sci9_rx_length  = rx_num;
  gp_sci9_rx_address = rx_buf;
  SCI9.SCR.BIT.RIE   = 1u;
  SCI9.SCR.BIT.RE    = 1u;
  return OK;
}

/****************************************************************************
 * Name: r_sci9_serial_send
 *
 * Description:
 * Send SCI9 data
 ****************************************************************************/

MD_STATUS r_sci9_serial_send(uint8_t * const tx_buf, uint16_t tx_num)
{
  if (1u > tx_num)
    {
      return MD_ARGERROR;
    }

  gp_sci9_tx_address = tx_buf;
  g_sci9_tx_count    = tx_num;

  /* Set TXD0 pin */

  SCI9.SCR.BIT.TIE = 1u;
  SCI9.SCR.BIT.TE  = 1u;
  return OK;
}
#endif

/****************************************************************************
 * Name: r_sci10_create
 *
 * Description:
 * SCI10 Initialization
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI10
void r_sci10_create(void)
{
  rx_mpc_enable();
  MSTP(SCI10)      = 0u;                  /* Cancel SCI10 module stop state */
  IPR(SCI10, RXI10) = 15;                 /* Set interrupt priority */
  IPR(SCI10, TXI10) = 15;                 /* Set interrupt priority */
  SCI10.SCR.BYTE   = 0u;                  /* Clear the control register */

  /* Set clock enable */

  SCI10.SCR.BYTE  = _00_SCI_INTERNAL_SCK_UNUSED;
  SCI10.SIMR1.BIT.IICM   = 0u;                   /* Clear SIMR1.IICM bit */
  SCI10.SPMR.BIT.CKPH    = 0u;                   /* Clear SPMR.CKPH bit */
  SCI10.SPMR.BIT.CKPOL   = 0u;                   /* Clear SPMR.CKPOL bit */

  /* Set control registers */

  SCI10.SPMR.BYTE  = _00_SCI_RTS;
  SCI10.SMR.BYTE   = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 |
                    _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
                    _00_SCI_MULTI_PROCESSOR_DISABLE |
                    _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
  SCI10.SCMR.BYTE  = _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST |
                    _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;

  SCI10.SEMR.BYTE  = _80_SCI_FALLING_EDGE_START_BIT |
                    _20_SCI_NOISE_FILTER_ENABLE |  _10_SCI_8_BASE_CLOCK |
                    _40_SCI_BAUDRATE_DOUBLE |
                    _04_SCI_BIT_MODULATION_ENABLE;
  SCI10.SNFR.BYTE  = _00_SCI_ASYNC_DIV_1;

  /* Set SCI10 pin */

  sci10_init_port();
  rx_mpc_disable();
}

/****************************************************************************
 * Name: r_sci10_start
 *
 * Description:
 * Starts SCI10
 ****************************************************************************/

void r_sci10_start(void)
{
  IR(SCI10, TXI10)  = 0u;  /* Clear interrupt flag */
  IR(SCI10, RXI10)  = 0u;  /* Clear interrupt flag */
  IEN(SCI10, TXI10) = 1u;  /* Enable SCI interrupt */
  IEN(SCI10, RXI10) = 1u;  /* Enable SCI interrupt */
  ICU.GENAL0.BIT.EN8 = 1u;
  ICU.GENAL0.BIT.EN9 = 1u;
}

/****************************************************************************
 * Name: r_sci10_stop
 *
 * Description:
 * Stops SCI10
 ****************************************************************************/

void r_sci10_stop(void)
{
  SCI10.SCR.BIT.TE  = 0u;   /* Disable serial transmit */
  SCI10.SCR.BIT.RE  = 0u;   /* Disable serial receive */
  SCI10.SCR.BIT.TIE = 0u;   /* disable TXI interrupt */
  SCI10.SCR.BIT.RIE = 0u;   /* disable RXI and ERI interrupt */
  IEN(SCI10, TXI10)  = 0u;
  ICU.GENAL0.BIT.EN8 = 0u;
  IR(SCI10, TXI10)  = 0u;
  IEN(SCI10, RXI10) = 0u;
  ICU.GENAL0.BIT.EN9 = 0u;
  IR(SCI10, RXI10) = 0u;
}

/****************************************************************************
 * Name: r_sci10_serial_receive
 *
 * Description:
 * Receives SCI10
 ****************************************************************************/

MD_STATUS r_sci10_serial_receive(uint8_t * const rx_buf, uint16_t rx_num)
{
  if (1u > rx_num)
    {
      return MD_ARGERROR;
    }

  g_sci10_rx_count  = 0u;
  g_sci10_rx_length = rx_num;
  gp_sci10_rx_address = rx_buf;
  SCI10.SCR.BIT.RIE  = 1u;
  SCI10.SCR.BIT.RE   = 1u;
  return OK;
}

/****************************************************************************
 * Name: r_sci10_serial_send
 *
 * Description:
 * Transmits SCI10 data
 ****************************************************************************/

MD_STATUS r_sci10_serial_send(uint8_t * const tx_buf, uint16_t tx_num)
{
  if (1u > tx_num)
    {
      return MD_ARGERROR;
    }

  gp_sci10_tx_address = tx_buf;
  g_sci10_tx_count    = tx_num;

  /* Set TXD0 pin */

  SCI10.SCR.BIT.TIE = 1u;
  SCI10.SCR.BIT.TE  = 1u;
  return OK;
}
#endif

/****************************************************************************
 * Name: r_sci11_create
 *
 * Description:
 * Initializes SCI11 data
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI11
void r_sci11_create(void)
{
  rx_mpc_enable();
  MSTP(SCI11)   = 0u;                    /* Cancel SCI11 module stop state */
  IPR(SCI11, RXI11) = 15;                /* Set interrupt priority */
  IPR(SCI11, TXI11) = 15;                /* Set interrupt priority */
  SCI11.SCR.BYTE   = 0u;                 /* Clear the control register */

  /* Set clock enable */

  SCI11.SCR.BYTE         = _00_SCI_INTERNAL_SCK_UNUSED;
  SCI11.SIMR1.BIT.IICM   = 0u;                   /* Clear SIMR1.IICM bit */
  SCI11.SPMR.BIT.CKPH    = 0u;                   /* Clear SPMR.CKPH bit */
  SCI11.SPMR.BIT.CKPOL   = 0u;                   /* Clear SPMR.CKPOL bit */

  /* Set control registers */

  SCI11.SPMR.BYTE    = _00_SCI_RTS;
  SCI11.SMR.BYTE     = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 |
                      _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
                      _00_SCI_MULTI_PROCESSOR_DISABLE |
                      _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
  SCI11.SCMR.BYTE    = _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST |
                      _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
  SCI11.SEMR.BYTE    = _80_SCI_FALLING_EDGE_START_BIT |
                      _20_SCI_NOISE_FILTER_ENABLE |
                      _10_SCI_8_BASE_CLOCK | _40_SCI_BAUDRATE_DOUBLE |
                      _04_SCI_BIT_MODULATION_ENABLE;
  SCI11.SNFR.BYTE    = _00_SCI_ASYNC_DIV_1;

  /* Set SCI11 pin */

  sci11_init_port();
  rx_mpc_disable();
}

/****************************************************************************
 * Name: r_sci11_start
 *
 * Description:
 * Starts SCI11
 ****************************************************************************/

void r_sci11_start(void)
{
  IR(SCI11, TXI11) = 0u;   /* Clear interrupt flag */
  IR(SCI11, RXI11) = 0u;   /* Clear interrupt flag */
  IEN(SCI11, TXI11) = 1u;  /* Enable SCI interrupt */
  IEN(SCI11, RXI11) = 1u;  /* Enable SCI interrupt */
  ICU.GENAL0.BIT.EN12 = 1u;
  ICU.GENAL0.BIT.EN13 = 1u;
}

/****************************************************************************
 * Name: r_sci11_stop
 *
 * Description:
 * Stops SCI11
 ****************************************************************************/

void r_sci11_stop(void)
{
  SCI11.SCR.BIT.TE  = 0u;        /* Disable serial transmit */
  SCI11.SCR.BIT.RE  = 0u;        /* Disable serial receive */
  SCI11.SCR.BIT.TIE = 0u;        /* disable TXI interrupt */
  SCI11.SCR.BIT.RIE = 0u;        /* disable RXI and ERI interrupt */
  IEN(SCI11, TXI11)  = 0u;
  ICU.GENAL0.BIT.EN12 = 0u;
  IR(SCI11, TXI11)  = 0u;
  IEN(SCI11, RXI11) = 0u;
  ICU.GENAL0.BIT.EN13 = 0u;
  IR(SCI11, RXI11) = 0u;
}

/****************************************************************************
 * Name: r_sci11_serial_receive
 *
 * Description:
 * Receives SCI11 data
 ****************************************************************************/

MD_STATUS r_sci11_serial_receive(uint8_t * const rx_buf, uint16_t rx_num)
{
  if (1u > rx_num)
    {
      return MD_ARGERROR;
    }

  g_sci11_rx_count  = 0u;
  g_sci11_rx_length = rx_num;
  gp_sci11_rx_address = rx_buf;
  SCI11.SCR.BIT.RIE  = 1u;
  SCI11.SCR.BIT.RE   = 1u;
  return OK;
}

/****************************************************************************
 * Name: r_sci11_serial_send
 *
 * Description:
 * Transmit SCI11 data
 ****************************************************************************/

MD_STATUS r_sci11_serial_send(uint8_t * const tx_buf, uint16_t tx_num)
{
  if (1u > tx_num)
    {
      return MD_ARGERROR;
    }

  gp_sci11_tx_address = tx_buf;
  g_sci11_tx_count    = tx_num;

  /* Set TXD1 pin */

  SCI11.SCR.BIT.TIE = 1u;
  SCI11.SCR.BIT.TE  = 1u;
  return OK;
}
#endif

/****************************************************************************
 * Name: r_sci12_create
 *
 * Description:
 * Initialize SCI12
 ****************************************************************************/

#ifdef CONFIG_RX65N_SCI12
void r_sci12_create(void)
{
  rx_mpc_enable();
  MSTP(SCI12)    = 0u;                   /* Cancel SCI12 module stop state */
  IPR(SCI12, RXI12) = 15;                /* Set interrupt priority */
  IPR(SCI12, TXI12) = 15;                /* Set interrupt priority */
  SCI12.SCR.BYTE  = 0u;                  /* Clear the control register */

  /* Set clock enable */

  SCI12.SCR.BYTE         = _00_SCI_INTERNAL_SCK_UNUSED;
  SCI12.SIMR1.BIT.IICM   = 0u;                   /* Clear SIMR1.IICM bit */
  SCI12.SPMR.BIT.CKPH    = 0u;                   /* Clear SPMR.CKPH bit */
  SCI12.SPMR.BIT.CKPOL   = 0u;                   /* Clear SPMR.CKPOL bit */

  /* Set control registers */

  SCI12.SPMR.BYTE   = _00_SCI_RTS;
  SCI12.SMR.BYTE    = _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 |
                     _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
                     _00_SCI_MULTI_PROCESSOR_DISABLE |
                     _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
  SCI12.SCMR.BYTE   = _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST |
                     _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
  SCI12.SEMR.BYTE   = _80_SCI_FALLING_EDGE_START_BIT |
                     _20_SCI_NOISE_FILTER_ENABLE | _10_SCI_8_BASE_CLOCK |
                     _40_SCI_BAUDRATE_DOUBLE | _04_SCI_BIT_MODULATION_ENABLE;
  SCI12.SNFR.BYTE   = _00_SCI_ASYNC_DIV_1;

  /* Set SCI12 pin */

  sci12_init_port();
  rx_mpc_disable();
}

/****************************************************************************
 * Name: r_sci12_start
 *
 * Description:
 * Start SCI12
 ****************************************************************************/

void r_sci12_start(void)
{
  IR(SCI12, TXI12) = 0u;   /* Clear interrupt flag */
  IR(SCI12, RXI12) = 0u;   /* Clear interrupt flag */
  IEN(SCI12, TXI12) = 1u;  /* Enable SCI interrupt */
  IEN(SCI12, RXI12) = 1u;  /* Enable SCI interrupt */
  ICU.GENBL0.BIT.EN16 = 1u;
  ICU.GENBL0.BIT.EN17 = 1u;
}

/****************************************************************************
 * Name: r_sci12_stop
 *
 * Description:
 * Stops SCI12
 ****************************************************************************/

void r_sci12_stop(void)
{
  SCI12.SCR.BIT.TE  = 0u;   /* Disable serial transmit */
  SCI12.SCR.BIT.RE  = 0u;   /* Disable serial receive */
  SCI12.SCR.BIT.TIE = 0u;   /* disable TXI interrupt */
  SCI12.SCR.BIT.RIE = 0u;   /* disable RXI and ERI interrupt */
  IEN(SCI12, TXI12)  = 0u;
  ICU.GENBL0.BIT.EN16 = 0u;
  IR(SCI12, TXI12)  = 0u;
  IEN(SCI12, RXI12) = 0u;
  ICU.GENBL0.BIT.EN17 = 0u;
  IR(SCI12, RXI12) = 0u;
}

/****************************************************************************
 * Name: r_sci12_serial_receive
 *
 * Description:
 * Receives SCI12 data
 ****************************************************************************/

MD_STATUS r_sci12_serial_receive(uint8_t * const rx_buf, uint16_t rx_num)
{
  if (1u > rx_num)
    {
      return MD_ARGERROR;
    }

  g_sci12_rx_count  = 0u;
  g_sci12_rx_length = rx_num;
  gp_sci12_rx_address = rx_buf;
  SCI12.SCR.BIT.RIE = 1u;
  SCI12.SCR.BIT.RE  = 1u;
  return OK;
}

/****************************************************************************
 * Name: r_sci12_serial_send
 *
 * Description:
 * Send SCI12 data
 ****************************************************************************/

MD_STATUS r_sci12_serial_send(uint8_t * const tx_buf, uint16_t tx_num)
{
  if (1u > tx_num)
    {
      return MD_ARGERROR;
    }

  gp_sci12_tx_address = tx_buf;
  g_sci12_tx_count    = tx_num;

  /* Set TXD2 pin */

  SCI12.SCR.BIT.TIE = 1u;
  SCI12.SCR.BIT.TE  = 1u;
  return OK;
}
#endif
