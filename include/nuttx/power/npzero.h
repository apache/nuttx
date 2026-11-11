/****************************************************************************
 * include/nuttx/power/npzero.h
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

#ifndef __INCLUDE_NUTTX_POWER_NPZERO_H
#define __INCLUDE_NUTTX_POWER_NPZERO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Nanopower nPZero G1S is a power-saving IC.  While the host is
 * powered (Standby state) it is an I2C slave of the host.  When the host
 * sends the Idle command, it switches the host off and polls up to four
 * I2C or SPI peripherals by itself, and it switches the host on again when
 * a wake-up condition happens.
 */

#define NPZERO_I2C_ADDR          0x3d   /* 7-bit I2C address */
#define NPZERO_ID                0x60   /* Value of the ID register */
#define NPZERO_NPERIPH           4      /* Number of peripherals */
#define NPZERO_SRAM_SIZE         128    /* Bytes of SRAM */

/* Registers ****************************************************************/

#define NPZERO_IDLE_RST          0x00   /* Idle/reset command */
#define NPZERO_ID_REG            0x01   /* Device ID */
#define NPZERO_STA1              0x02   /* Status 1 */
#define NPZERO_STA2              0x03   /* Status 2 */
#define NPZERO_PSWCTL            0x04   /* Power switch control */
#define NPZERO_SYSCFG1           0x05   /* System configuration 1 */
#define NPZERO_SYSCFG2           0x06   /* System configuration 2 */
#define NPZERO_SYSCFG3           0x07   /* System configuration 3 */
#define NPZERO_TOUT_L            0x08   /* Global time-out, low byte */
#define NPZERO_TOUT_H            0x09   /* Global time-out, high byte */
#define NPZERO_INTCFG            0x0a   /* Interrupt pins configuration */

/* Peripheral registers, n = 1..4 */

#define NPZERO_PERIPH_BASE(n)    (0x10 + 13 * ((n) - 1))
#define NPZERO_CFGP(n)           (NPZERO_PERIPH_BASE(n) + 0)
#define NPZERO_MODP(n)           (NPZERO_PERIPH_BASE(n) + 1)
#define NPZERO_PERP_L(n)         (NPZERO_PERIPH_BASE(n) + 2)
#define NPZERO_PERP_H(n)         (NPZERO_PERIPH_BASE(n) + 3)
#define NPZERO_NCMDP(n)          (NPZERO_PERIPH_BASE(n) + 4)
#define NPZERO_ADDRP(n)          (NPZERO_PERIPH_BASE(n) + 5)
#define NPZERO_RREGP(n)          (NPZERO_PERIPH_BASE(n) + 6)
#define NPZERO_THROVP_L(n)       (NPZERO_PERIPH_BASE(n) + 7)
#define NPZERO_THROVP_H(n)       (NPZERO_PERIPH_BASE(n) + 8)
#define NPZERO_THRUNP_L(n)       (NPZERO_PERIPH_BASE(n) + 9)
#define NPZERO_THRUNP_H(n)       (NPZERO_PERIPH_BASE(n) + 10)
#define NPZERO_TWTP(n)           (NPZERO_PERIPH_BASE(n) + 11)
#define NPZERO_TCFGP(n)          (NPZERO_PERIPH_BASE(n) + 12)

#define NPZERO_THROVA1           0x44   /* VBAT ADC over-threshold */
#define NPZERO_THRUNA1           0x45   /* VBAT ADC under-threshold */
#define NPZERO_THROVA2           0x46   /* ADC_IN ADC over-threshold */
#define NPZERO_THRUNA2           0x47   /* ADC_IN ADC under-threshold */
#define NPZERO_VALP_L(n)         (0x50 + 2 * ((n) - 1))
#define NPZERO_VALP_H(n)         (0x51 + 2 * ((n) - 1))
#define NPZERO_ADC_CORE          0x58   /* Last VBAT ADC value */
#define NPZERO_ADC_EXT           0x59   /* Last ADC_IN ADC value */
#define NPZERO_SRAM              0x80   /* SRAM, 0x80-0xff */

/* Register bit fields ******************************************************/

/* IDLE_RST */

#define NPZERO_IDLE_RST_IDLE     0xff   /* Enter Idle state */
#define NPZERO_IDLE_RST_RESET    0xa5   /* Soft reset */

/* STA1 */

#define NPZERO_STA1_RSTSRC_MASK  0x0f       /* Reset source (see errata #5) */
#define NPZERO_STA1_RSTSRC_POR   0x01       /*   Power-on reset */
#define NPZERO_STA1_RSTSRC_EXT   0x02       /*   External reset (RSTN pin) */
#define NPZERO_STA1_RSTSRC_BOR   0x04       /*   Brown-out reset */
#define NPZERO_STA1_RSTSRC_SOFT  0x08       /*   Soft reset */
#define NPZERO_STA1_FA2          (1 << 5)   /* ADC_IN channel trigger */
#define NPZERO_STA1_FA1          (1 << 6)   /* VBAT channel trigger */
#define NPZERO_STA1_FTOUT        (1 << 7)   /* Global time-out */

/* STA2, n = 1..4 */

#define NPZERO_STA2_FP(n)        (1 << (2 * ((n) - 1)))     /* Trigger */
#define NPZERO_STA2_FNAKP(n)     (1 << (2 * ((n) - 1) + 1)) /* NAK */

/* PSWCTL */

#define NPZERO_PSWCTL_PSWINT(n)  (1 << ((n) - 1))
#define NPZERO_PSWCTL_PSWH_SHIFT 4
#define NPZERO_PSWCTL_PSWH_MASK  (3 << NPZERO_PSWCTL_PSWH_SHIFT)
#define NPZERO_PSWCTL_PSW_EN_VN  (1 << 6)
#define NPZERO_PSWCTL_PSW_VN_ON  (1 << 7)

/* SYSCFG1 */

#define NPZERO_SYSCFG1_WUP_P(n)  (1 << ((n) - 1))
#define NPZERO_SYSCFG1_WUP_A1    (1 << 4)
#define NPZERO_SYSCFG1_WUP_A2    (1 << 5)
#define NPZERO_SYSCFG1_WUPMOD    (1 << 6)

/* SYSCFG2 */

#define NPZERO_SYSCFG2_SCLK_DIV_EN      (1 << 0)
#define NPZERO_SYSCFG2_SCLK_DIV_SHIFT   1
#define NPZERO_SYSCFG2_SCLK_SEL         (1 << 3)
#define NPZERO_SYSCFG2_ADC_EXT_ON       (1 << 4)
#define NPZERO_SYSCFG2_ADC_CLK_SHIFT    5

/* SYSCFG3 */

#define NPZERO_SYSCFG3_IO_STR           (1 << 0)
#define NPZERO_SYSCFG3_I2C_PUP_SHIFT    1
#define NPZERO_SYSCFG3_SPI_AUTO         (1 << 3)
#define NPZERO_SYSCFG3_CLKOUT_SHIFT     4
#define NPZERO_SYSCFG3_SCLK_SEL_STATUS  (1 << 7)

/* CFGP */

#define NPZERO_CFGP_PWMOD_SHIFT  0
#define NPZERO_CFGP_TMOD_SHIFT   2
#define NPZERO_CFGP_PSWMOD_SHIFT 4
#define NPZERO_CFGP_INTMOD_SHIFT 6

/* MODP */

#define NPZERO_MODP_CMOD         (1 << 0)
#define NPZERO_MODP_DTYPE_SHIFT  1
#define NPZERO_MODP_SEQRW        (1 << 3)
#define NPZERO_MODP_WUNAK        (1 << 4)
#define NPZERO_MODP_SWPRREG      (1 << 5)
#define NPZERO_MODP_SPIMOD_SHIFT 6

/* ADDRP */

#define NPZERO_ADDRP_ADDR_MASK   0x7f
#define NPZERO_ADDRP_SPIEN       (1 << 7)

/* TCFGP */

#define NPZERO_TCFGP_TWT_SHIFT   0      /* TWT_EN and TWT_EXT */
#define NPZERO_TCFGP_TINIT_SHIFT 2      /* TINIT_EN and TINIT_EXT */
#define NPZERO_TCFGP_I2CRET_SHIFT 4

/* Values of the configuration fields ***************************************/

/* Host power switch mode (npzero_config_s::hostmode) */

#define NPZERO_HOSTMODE_SWITCH   0      /* SW_HP is a power switch */
#define NPZERO_HOSTMODE_LOGIC    2      /* Logic output, high when on */
#define NPZERO_HOSTMODE_LOGICINV 3      /* Logic output, low when on */

/* System clock source (npzero_config_s::sclksel) */

#define NPZERO_SCLK_LPOSC        0      /* 10 Hz low-power oscillator */
#define NPZERO_SCLK_XO           1      /* 32.768 kHz XO / 2048 (16 Hz) */

/* System clock divider (npzero_config_s::sclkdiv) */

#define NPZERO_SCLKDIV_1         0
#define NPZERO_SCLKDIV_2         1
#define NPZERO_SCLKDIV_4         2
#define NPZERO_SCLKDIV_8         3
#define NPZERO_SCLKDIV_16        4

/* ADC sampling clock (npzero_config_s::adcclk).  The XO clocks need
 * sclksel = NPZERO_SCLK_XO.
 */

#define NPZERO_ADCCLK_SCLK       0      /* System clock */
#define NPZERO_ADCCLK_64HZ       1      /* XO / 512 */
#define NPZERO_ADCCLK_256HZ      2      /* XO / 128 */
#define NPZERO_ADCCLK_1024HZ     3      /* XO / 32 */

/* I2C pull-ups of the nPZero (npzero_config_s::i2cpull) */

#define NPZERO_I2CPULL_OFF       0      /* Disabled */
#define NPZERO_I2CPULL_ON        1      /* Always enabled */
#define NPZERO_I2CPULL_AUTO      3      /* Disabled in Idle state */

/* CLK_OUT pin (npzero_config_s::clkout) */

#define NPZERO_CLKOUT_OFF        0
#define NPZERO_CLKOUT_32KHZ      1
#define NPZERO_CLKOUT_16KHZ      2
#define NPZERO_CLKOUT_8KHZ       3
#define NPZERO_CLKOUT_4KHZ       4
#define NPZERO_CLKOUT_2KHZ       5
#define NPZERO_CLKOUT_1KHZ       6

/* Pull-up of the INT1-4 pins (npzero_config_s::intpull) */

#define NPZERO_INTPULL_OFF       0      /* No pull-up */
#define NPZERO_INTPULL_100K      1      /* About 100 kOhm */
#define NPZERO_INTPULL_50K       3      /* About 50 kOhm */

/* Peripheral power mode (npzero_periph_s::pwmod) */

#define NPZERO_PWMOD_DISABLED    0
#define NPZERO_PWMOD_PERIODIC    1      /* Powered every polling period */
#define NPZERO_PWMOD_ALWAYSON    3

/* Peripheral polling mode (npzero_periph_s::tmod) */

#define NPZERO_TMOD_READ         0      /* Init, read and compare */
#define NPZERO_TMOD_INTREAD      1      /* Init, wait INT, read, compare */
#define NPZERO_TMOD_INT          2      /* Init and wait for INT */
#define NPZERO_TMOD_ASYNCINT     3      /* Wait for INT */

/* Peripheral power switch mode, SW_LP pin (npzero_periph_s::pswmod).  The
 * mode 0 (power switch with voltage rise detection) can lock up the
 * polling and it is refused by the driver.
 */

#define NPZERO_PSWMOD_SWITCH     1      /* Power switch */
#define NPZERO_PSWMOD_LOGIC      2      /* Logic output, high when on */
#define NPZERO_PSWMOD_LOGICINV   3      /* Logic output, low when on */

/* Peripheral INT pin mode (npzero_periph_s::intmod) */

#define NPZERO_INTMOD_IN_HIGH    0      /* Interrupt input, active high */
#define NPZERO_INTMOD_IN_LOW     1      /* Interrupt input, active low */
#define NPZERO_INTMOD_OUT_HIGH   2      /* Trigger output, active high */
#define NPZERO_INTMOD_OUT_LOW    3      /* Trigger output, active low */

/* Peripheral data type (npzero_periph_s::dtype) */

#define NPZERO_DTYPE_UINT16      0
#define NPZERO_DTYPE_INT16       1
#define NPZERO_DTYPE_UINT8       2

/* Wait times (npzero_periph_s::tinit and twait) in periods of the internal
 * 400 kHz oscillator, multiplied by npzero_periph_s::twtp.
 */

#define NPZERO_WAIT_NONE         0
#define NPZERO_WAIT_256          1      /* twtp x 256 periods (640 us) */
#define NPZERO_WAIT_4096         3      /* twtp x 4096 periods (10.24 ms) */

/* IOCTL commands ***********************************************************/

/* Command:      NPZEROIOC_GETID
 * Description:  Read the ID register (NPZERO_ID)
 * Argument:     FAR uint8_t *
 *
 * Command:      NPZEROIOC_GETSTATUS
 * Description:  Read the STA1 and STA2 registers, the reset source and
 *               the wake-up reason
 * Argument:     FAR struct npzero_status_s *
 *
 * Command:      NPZEROIOC_CONFIGURE
 * Description:  Write the whole configuration and the SRAM.  It must be
 *               sent in Standby state, before NPZEROIOC_IDLE.
 * Argument:     FAR const struct npzero_config_s *
 *
 * Command:      NPZEROIOC_GETVALUE
 * Description:  Read the last value read from a peripheral (VALP)
 * Argument:     FAR struct npzero_value_s *
 *
 * Command:      NPZEROIOC_GETADC
 * Description:  Read the last ADC values (ADC_CORE and ADC_EXT)
 * Argument:     FAR struct npzero_adc_s *
 *
 * Command:      NPZEROIOC_IDLE
 * Description:  Enter Idle state.  The nPZero switches the host off
 *               according to the host power switch mode.
 * Argument:     None
 *
 * Command:      NPZEROIOC_RESET
 * Description:  Soft reset of the nPZero
 * Argument:     None
 *
 * Command:      NPZEROIOC_READREG
 * Description:  Read a register
 * Argument:     FAR struct npzero_reg_s *
 *
 * Command:      NPZEROIOC_WRITEREG
 * Description:  Write a register.  The SRAM can only be written with
 *               NPZEROIOC_CONFIGURE (errata #2).
 * Argument:     FAR const struct npzero_reg_s *
 */

#define NPZEROIOC_GETID          _NPZEROIOC(1)
#define NPZEROIOC_GETSTATUS      _NPZEROIOC(2)
#define NPZEROIOC_CONFIGURE      _NPZEROIOC(3)
#define NPZEROIOC_GETVALUE       _NPZEROIOC(4)
#define NPZEROIOC_GETADC         _NPZEROIOC(5)
#define NPZEROIOC_IDLE           _NPZEROIOC(6)
#define NPZEROIOC_RESET          _NPZEROIOC(7)
#define NPZEROIOC_READREG        _NPZEROIOC(8)
#define NPZEROIOC_WRITEREG       _NPZEROIOC(9)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Configuration of a peripheral */

struct npzero_periph_s
{
  uint8_t  pwmod;      /* Power mode, NPZERO_PWMOD_* */
  uint8_t  tmod;       /* Polling mode, NPZERO_TMOD_* */
  uint8_t  pswmod;     /* SW_LP pin mode, NPZERO_PSWMOD_* */
  uint8_t  intmod;     /* INT pin mode, NPZERO_INTMOD_* */
  bool     spi;        /* true: SPI peripheral, false: I2C peripheral */
  uint8_t  spimode;    /* SPI mode, 0 to 3 */
  uint8_t  dtype;      /* Type of the value, NPZERO_DTYPE_* */
  bool     inverted;   /* Trigger between the thresholds instead of
                        * above the over- or below the under-threshold */
  bool     seqrw;      /* Multi-byte transfers for sequential addresses */
  bool     wunak;      /* Wake up if the I2C peripheral does not ACK */
  bool     swap;       /* Swap the bytes of the 16-bit value */
  uint8_t  addr;       /* 7-bit address of the I2C peripheral */
  uint8_t  rreg;       /* I2C register of the value (low byte) */
  uint8_t  retries;    /* I2C retries on NAK, 0 to 3 */
  uint16_t period;     /* Polling period, in system clock periods (>= 2) */
  uint16_t throver;    /* Over-threshold */
  uint16_t thrunder;   /* Under-threshold */
  uint8_t  twtp;       /* Wait time unit count (TWTP) */
  uint8_t  tinit;      /* Wait before the initialization, NPZERO_WAIT_* */
  uint8_t  twait;      /* Wait after the initialization (before the
                        * read or for the interrupt), NPZERO_WAIT_* */

  /* Initialization commands, copied to the SRAM.  I2C: ninit pairs of
   * register address and value (2 * ninit bytes).  SPI: ninit bytes.
   */

  uint8_t  ninit;
  FAR const uint8_t *init;

  /* SPI only: bytes shifted out to read the value */

  uint8_t  nread;
  FAR const uint8_t *read;
};

/* Configuration of the nPZero (NPZEROIOC_CONFIGURE) */

struct npzero_config_s
{
  uint8_t  hostmode;   /* Host power switch mode, NPZERO_HOSTMODE_* */
  uint8_t  pswint;     /* Bit n-1 set: SW_LPn is on in Standby state */
  bool     gateboost;  /* Power switch gate boost (lower Rds(on)) */
  uint8_t  wakeup;     /* Wake-up sources, NPZERO_SYSCFG1_WUP_* bits */
  bool     wakeupall;  /* Wake up when all the sources trigger */
  uint8_t  sclksel;    /* System clock, NPZERO_SCLK_* */
  uint8_t  sclkdiv;    /* System clock divider, NPZERO_SCLKDIV_* */
  uint8_t  adcclk;     /* ADC sampling clock, NPZERO_ADCCLK_* */
  bool     adcext;     /* Sample the ADC_IN pin too */
  bool     iostrength; /* High strength of the I/O pins */
  uint8_t  i2cpull;    /* I2C pull-ups, NPZERO_I2CPULL_* */
  bool     spiauto;    /* SPI outputs in high-Z when not used */
  uint8_t  clkout;     /* CLK_OUT pin, NPZERO_CLKOUT_* */
  uint16_t timeout;    /* Global time-out, in system clock periods */

  /* Pull-ups of the INT1-4 pins, NPZERO_INTPULL_* */

  uint8_t  intpull[NPZERO_NPERIPH];

  /* ADC thresholds, [0] is the VBAT channel (5-bit), [1] the ADC_IN
   * channel (6-bit).
   */

  uint8_t  adcover[2];
  uint8_t  adcunder[2];

  /* Peripherals 1 to 4, NULL if disabled */

  FAR const struct npzero_periph_s *periph[NPZERO_NPERIPH];
};

/* Status (NPZEROIOC_GETSTATUS) */

struct npzero_status_s
{
  uint8_t sta1;        /* NPZERO_STA1_* bits */
  uint8_t sta2;        /* NPZERO_STA2_* bits */
};

/* Value of a peripheral (NPZEROIOC_GETVALUE) */

struct npzero_value_s
{
  uint8_t  periph;     /* Peripheral, 1 to 4 (in) */
  uint16_t value;      /* Last value read, of type dtype (out) */
};

/* ADC values (NPZEROIOC_GETADC), see the ADC codes in the datasheet */

struct npzero_adc_s
{
  uint8_t core;        /* VBAT channel */
  uint8_t ext;         /* ADC_IN channel */
};

/* Register access (NPZEROIOC_READREG and NPZEROIOC_WRITEREG) */

struct npzero_reg_s
{
  uint8_t reg;         /* Register address */
  uint8_t val;         /* Register value */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: npzero_register
 *
 * Description:
 *   Check the nPZero G1S on the I2C bus, apply the errata workarounds that
 *   are needed after each reset of the nPZero and register the character
 *   driver.  It should be called at each boot of the host, since the host
 *   is powered off in Idle state.
 *
 * Input Parameters:
 *   devpath - The path of the device, e.g. "/dev/npzero0"
 *   i2c     - The I2C bus of the nPZero
 *   addr    - The 7-bit I2C address, NPZERO_I2C_ADDR
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

struct i2c_master_s;
int npzero_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                    uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_POWER_NPZERO_H */
