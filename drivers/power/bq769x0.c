/****************************************************************************
 * drivers/power/bq769x0.c
 * Lower half driver for BQ769x0 battery monitor
 *
 *   Copyright (C) 2019 2G Engineering. All rights reserved.
 *   Author: Josh Lange <jlange@2g-eng.com>
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/* The bq76920/bq76930/bq76940 battery monitor ICs provide voltage, current,
 * and temperature monitoring of up to 15-series cells.  These ICs also provide
 * Coulomb counting for state-of-charge measurement, balance drivers for all
 * cells, and drivers for external cell protection switches.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <crc8.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_monitor.h>
#include <nuttx/power/battery_ioctl.h>

#include <nuttx/power/bq769x0.h>

/* This driver requires:
 *
 * CONFIG_BATTERY_MONITOR- Upper half battery driver support
 * CONFIG_I2C - I2C support
 * CONFIG_I2C_BQ769X0 - And the driver must be explicitly selected.
 */

#if defined(CONFIG_BATTERY_MONITOR) && defined(CONFIG_I2C) && \
    defined(CONFIG_I2C_BQ769X0)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helpers ******************************************************************/

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif
#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

/* The CRC function expects to see address bytes as they appear on the wire */
#define WR_ADDR(a)  ((a) << 1)
#define RD_ADDR(a)  (((a) << 1) | 1)

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_BQ769X0
#  define baterr    _err
#  define batreg    _err
#  define batinfo   _info
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define baterr(x...)
#    define batreg(x...)
#    define batinfo(x...)
#  else
#    define baterr(void)
#    define batreg(void)
#    define batinfo(void)
#  endif
#endif

/****************************************************************************
 * Private
 ****************************************************************************/

struct bq769x0_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  FAR const struct battery_monitor_operations_s *ops; /* Battery operations */
  sem_t batsem;                /* Enforce mutually exclusive access */

  /* Data fields specific to the lower half BQ769x0 driver follow */

  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  uint8_t chip;                 /* Chip Type (e.g. CHIP_76920) */
  uint8_t cellcount;            /* Number of cells attached to chip */
  uint32_t frequency;           /* I2C frequency */
  uint32_t gain;                /* ADC gain value in uV */
  uint32_t offset;              /* ADC offset value in uV */
  const uint8_t *mapping;       /* Pointer to cell mapping table */
  bool crc;                     /* Whether or not the device has CRC enabled */
};

/* Cell mapping tables
 * Some channels are not used depending on how many cells are connected
 * to the BQ769X0.  These tables map cell number (array index) to physical
 * cell channel (array value).  See TI datasheet for cell connections table.
 */
static const uint8_t bq76920_3cell_mapping[] =
    {
        0, 1, 4
    };
static const uint8_t bq76920_4cell_mapping[] =
    {
        0, 1, 2, 4
    };
static const uint8_t bq76920_5cell_mapping[] =
    {
        0, 1, 2, 3, 4
    };
static const uint8_t *bq76920_cell_mapping[] =
    {
        bq76920_3cell_mapping,
        bq76920_4cell_mapping,
        bq76920_5cell_mapping
    };

static const uint8_t bq76930_6cell_mapping[] =
{
    0, 1, 4, 5, 6, 9
};
static const uint8_t bq76930_7cell_mapping[] =
{
    0,1, 2, 4, 5, 6, 9
};
static const uint8_t bq76930_8cell_mapping[] =
{
    0, 1, 2, 4, 5, 6, 7, 9
};
static const uint8_t bq76930_9cell_mapping[] =
{
    0, 1, 2, 3, 4, 5, 6, 7, 9
};
static const uint8_t bq76930_10cell_mapping[] =
{
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9
};
static const uint8_t *bq76930_cell_mapping[] =
    {
        bq76930_6cell_mapping,
        bq76930_7cell_mapping,
        bq76930_8cell_mapping,
        bq76930_9cell_mapping,
        bq76930_10cell_mapping
    };

static const uint8_t bq76940_9cell_mapping[] =
    {
        0, 1, 4, 5, 6, 9, 10, 11, 14
    };
static const uint8_t bq76940_10cell_mapping[] =
    {
        0, 1, 2, 4, 5, 6, 9, 10, 11, 14
    };
static const uint8_t bq76940_11cell_mapping[] =
    {
        0, 1, 2, 4, 5, 6, 7, 9, 10, 11, 14
    };
static const uint8_t bq76940_12cell_mapping[] =
    {
        0, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 14
    };
static const uint8_t bq76940_13cell_mapping[] =
    {
        0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 14
    };
static const uint8_t bq76940_14cell_mapping[] =
    {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14
    };
static const uint8_t bq76940_15cell_mapping[] =

    {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
      };
static const uint8_t *bq76940_cell_mapping[] =
    {
        bq76940_9cell_mapping,
        bq76940_10cell_mapping,
        bq76940_11cell_mapping,
        bq76940_12cell_mapping,
        bq76940_13cell_mapping,
        bq76940_14cell_mapping,
        bq76940_15cell_mapping,
    };

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* I2C support functions */

static int bq769x0_getreg8(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval);
static int bq769x0_putreg8(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                           uint8_t regval);
static int bq769x0_getreg16(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                            FAR uint16_t *regval);
static int bq769x0_getnreg16(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                             FAR uint16_t *regvals, unsigned int count);

/* Device functions */
static inline int bq769x0_getreport(FAR struct bq769x0_dev_s *priv,
                                    uint8_t *report);
static inline int bq769x0_getvolt(FAR struct bq769x0_dev_s *priv, int *volts);
static inline int bq769x0_getcc(FAR struct bq769x0_dev_s *priv, int *cc);
static inline int bq769x0_getcellvolt(FAR struct bq769x0_dev_s *priv,
                                      struct battery_monitor_voltage_s *voltages);
static inline int bq769x0_gettemperature(FAR struct bq769x0_dev_s *priv,
                                         struct battery_monitor_temperature_s *temps);
static inline int bq769x0_setbalance(FAR struct bq769x0_dev_s *priv,
                                         struct battery_monitor_balance_s *bal);
static inline int bq769x0_updategain(FAR struct bq769x0_dev_s *priv);

static int bq769x0_chip_cellcount(FAR struct bq769x0_dev_s *priv);

/* Battery driver lower half methods */

static int bq769x0_state(struct battery_monitor_dev_s *dev, int *status);
static int bq769x0_health(struct battery_monitor_dev_s *dev, int *health);
static int bq769x0_online(struct battery_monitor_dev_s *dev, bool *status);
static int bq769x0_voltage(struct battery_monitor_dev_s *dev, int *value);
static int bq769x0_cellvoltage(struct battery_monitor_dev_s *dev,
                               struct battery_monitor_voltage_s *cellv);
static int bq769x0_current(struct battery_monitor_dev_s *dev, int *value);
static int bq769x0_soc(struct battery_monitor_dev_s *dev, b16_t *value);
static int bq769x0_coulombs(struct battery_monitor_dev_s *dev, int *coulombs);
static int bq769x0_temp(struct battery_monitor_dev_s *dev,
                        struct battery_monitor_temperature_s *temps);
static int bq769x0_balance(struct battery_monitor_dev_s *dev,
                           struct battery_monitor_balance_s *bal);
static int bq769x0_operate(struct battery_monitor_dev_s *dev, uintptr_t param);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_monitor_operations_s g_bq769x0ops =
{
  bq769x0_state,
  bq769x0_health,
  bq769x0_online,
  bq769x0_voltage,
  bq769x0_cellvoltage,
  bq769x0_current,
  bq769x0_soc,
  bq769x0_coulombs,
  bq769x0_temp,
  bq769x0_balance,
  bq769x0_operate,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq769x0_getreg8
 *
 * Description:
 *   Read a 8-bit value from a BQ769x0 register.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 NO-ACK STOP
 *
 ****************************************************************************/

static int bq769x0_getreg8(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval)
{
  struct i2c_config_s config;
  uint8_t val[2];
  int ret;
  int datalen;
  uint8_t sl_addr;
  uint8_t crc;

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      baterr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Our expected data length varies depending on whether or not a CRC is used */

  if (priv->crc)
    {
      datalen = 2;
    }
  else
    {
      datalen = 1;
    }

  /* Restart and read 8-bits from the register */

  ret = i2c_read(priv->i2c, &config, val, datalen);
  if (ret < 0)
    {
      baterr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* If CRC is used, verify that it is correct */
  if (priv->crc)
    {
      sl_addr = RD_ADDR(priv->addr);
      crc = crc8ccittpart(&sl_addr, 1, 0);
      crc = crc8ccittpart(val, 1, crc);
      if (crc != val[1])
        {
          baterr("ERROR: CRC mismatch: Got %02x, Expected %02x\n", val[1], crc);
          return ERROR;
        }
    }

  /* Copy 8-bit value to be returned */

  *regval = val[0];
  return OK;
}

/****************************************************************************
 * Name: bq769x0_putreg8
 *
 * Description:
 *   Write a 8-bit value to a BQ769x0 register.
 *
 *   START <I2C write address> ACK <Reg address> ACK Data0 ACK STOP
 *
 ****************************************************************************/

static int bq769x0_putreg8(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[3];
  int datalen;
  uint8_t sl_addr;
  uint8_t crc;

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  batreg("addr: %02x regval: %02x\n", regaddr, regval);

  /* Set up a 3 byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Our expected data length varies depending on whether or not a CRC is used */
  if (priv->crc)
    {
      datalen = 3;
      sl_addr = WR_ADDR(priv->addr);
      crc = crc8ccittpart(&sl_addr, 1, 0);
      crc = crc8ccittpart(buffer, 2, crc);
      buffer[2] = crc;
      batreg("write crc: %02x\n", crc);
    }
  else
    {
      datalen = 2;
    }

  /* Write the register address followed by the data (no RESTART) */

  return i2c_write(priv->i2c, &config, buffer, datalen);
}

/****************************************************************************
 * Name: bq769x0_getreg16
 *
 * Description:
 *   Read a 16-bit value from a BQ769x0 register pair.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 ACK Data1 NO-ACK STOP
 *
 ****************************************************************************/

static int bq769x0_getreg16(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                           FAR uint16_t *regval)
{
    return bq769x0_getnreg16(priv, regaddr, regval, 1);
}
/****************************************************************************
 * Name: bq769x0_getnreg16
 *
 * Description:
 *   Read an array of 16-bit values from BQ769x0 register pairs.
 *
 *   START <I2C write address> ACK <Reg address> ACK
 *   REPEATED-START <I2C read address> ACK Data0 ACK Data1 NO-ACK STOP
 *
 *   count is number of 16-bit words to read
 *
 ****************************************************************************/

static int bq769x0_getnreg16(FAR struct bq769x0_dev_s *priv, uint8_t regaddr,
                           FAR uint16_t *regvals, unsigned int count)
{
  struct i2c_config_s config;
  uint8_t tmp_val[(4 * 22)]; /* Maximum of 22 registers per read with CRC */
  int ret;
  int datalen;
  int byte_count;
  uint8_t sl_addr;
  uint8_t crc;
  int i;
  if (priv->crc)
    {
      if (count >= (sizeof(tmp_val) / 4))
        {
          count = sizeof(tmp_val) / 4;
        }
    }
  else
    {
  if (count >= (sizeof(tmp_val) / 2))
    {
      count = sizeof(tmp_val) / 2;
    }
  }

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      baterr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  byte_count = 2 * count;

  /* Our expected I2C data length varies depending on whether or not a CRC is used */

  if (priv->crc)
    {
      /* When reading multiple bytes, there is 1 CRC byte per data byte */
      datalen = (4 * count);
    }
  else
    {
      datalen = byte_count;
    }

  /* Restart and read 16-bits from the register */

  ret = i2c_read(priv->i2c, &config, tmp_val, datalen);
  if (ret < 0)
    {
      baterr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* If CRC is used, verify that it is correct
   * We only include the address with the first data byte.
   * After that, we compare the CRC of each byte with its following byte*/
  if (priv->crc)
    {
      sl_addr = RD_ADDR(priv->addr);
      crc = crc8ccittpart(&sl_addr, 1, 0);
      for (i = 0; i < byte_count; i += 2) {
        crc = crc8ccittpart(&tmp_val[i], 1, crc);
        if (crc != tmp_val[i + 1])
          {
            baterr("ERROR: CRC mismatch: Got %02x, Expected %02x\n", tmp_val[2], crc);
            return ERROR;
          }
        crc = 0;
      }
      /* Copy 16-bit values to be returned, skipping CRC bytes*/
      for (i = 0; i < datalen; i += 4) {
          *regvals = (uint16_t)tmp_val[i] << 8 | (uint16_t)tmp_val[i + 2];
          regvals += 1;
      }
    }
  else
    {
      /* Copy 16-bit values to be returned */
      for (i = 0; i < datalen; i += 2) {
          *regvals = (uint16_t)tmp_val[i] << 8 | (uint16_t)tmp_val[i + 1];
          regvals += 1;
      }
    }
  return OK;
}

/****************************************************************************
 * Name: bq769x0_getreport
 *
 * Description:
 *   Read the BQ769X0 SYS_STAT register
 *
 ****************************************************************************/

static inline int bq769x0_getreport(FAR struct bq769x0_dev_s *priv,
                                    uint8_t *report)
{
  uint8_t regval = 0;
  int ret;

  ret = bq769x0_getreg8(priv, BQ769X0_REG_SYS_STAT, &regval);
  if (ret == OK)
    {
      *report = regval;
    }

  return ret;
}

/****************************************************************************
 * Name: bq769x0_updategain
 *
 * Description:
 *   Updates the local copies of the BQ769x0 ADC gain registers.
 *   These are used when converting ADC values to actual voltages.
 *
 ****************************************************************************/

static inline int bq769x0_updategain(FAR struct bq769x0_dev_s *priv)
{
  int ret;
  uint8_t gainreg1;
  uint8_t gainreg2;
  uint8_t gain;
  int8_t offset;

  /* Read current register values */

  ret = bq769x0_getreg8(priv, BQ769X0_REG_ADCGAIN1, &gainreg1);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }
  ret = bq769x0_getreg8(priv, BQ769X0_REG_ADCGAIN2, &gainreg2);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }
  ret = bq769x0_getreg8(priv, BQ769X0_REG_ADCOFFSET, (uint8_t *)&offset);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Calculate actual gain & offset values
   * gainreg1 contains gain bits 4-3
   * gainreg2 contains gain bits 2-0
   */

  gainreg1 &= BQ769X0_ADCGAIN1_MASK;
  gainreg2 &= BQ769X0_ADCGAIN2_MASK;
  gain = (gainreg1 << 1) | (gainreg2 >> 5);

  priv->gain = gain + BQ769X0_BASE_GAIN;
  priv->offset = offset * 1000; /* Convert mV to uV */

  batinfo("Battery monitor gain: %d uV/LSB, offset: %d uV.\n", priv->gain, priv->offset);

  return OK;
}

/****************************************************************************
 * Name: bq769x0_state
 *
 * Description:
 *   Return the current battery state
 *
 ****************************************************************************/

static int bq769x0_state(struct battery_monitor_dev_s *dev, int *status)
{
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  uint8_t regval = 0;
  int ret;

  ret = bq769x0_getreport(priv, &regval);
  if (ret < 0)
    {
      *status = BATTERY_UNKNOWN;
      return ret;
    }

#warning fixme convert to useful state information?
  *status = regval;

  return OK;
}

/****************************************************************************
 * Name: bq769x0_health
 *
 * Description:
 *   Return the current battery health state
 *
 * Note: if more than one fault happened the user needs to call this ioctl
 * again to read a new fault, repeat until receive a BATTERY_HEALTH_GOOD.
 *
 ****************************************************************************/

static int bq769x0_health(struct battery_monitor_dev_s *dev, int *health)
{
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  uint8_t regval = 0;
  int ret;

  ret = bq769x0_getreport(priv, &regval);
  if (ret < 0)
    {
      *health = BATTERY_HEALTH_UNKNOWN;
      return ret;
    }
#warning fixme convert to useful state information?
  switch (regval)
  {

  default:
    *health = BATTERY_HEALTH_UNKNOWN;
    break;
  }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_online
 *
 * Description:
 *   Return true if the battery is online
 *
 ****************************************************************************/

static int bq769x0_online(struct battery_monitor_dev_s *dev, bool *status)
{
  /* There is no concept of online/offline in this driver */

  *status = true;
  return OK;
}

/****************************************************************************
 * Name: bq769x0_getvolt
 *
 * Description:
 *   Gets the battery stack voltage in uV.
 *
 ****************************************************************************/

static inline int bq769x0_getvolt(FAR struct bq769x0_dev_s *priv, int *volts)
{
  uint16_t regval;
  int ret;

  ret = bq769x0_getreg16(priv, BQ769X0_REG_BAT_HI, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading voltage from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Voltage is returned from the chip in units of <gain>uV/LSB
   * An offset also needs to be added.
   * Reading the two bytes in a single operation guarantees atomic access.
   * The pack voltage is divided by 4 in order to fit in a 16-bit register
   * Multiply gain by 4, and offset by number of channels on the chip, since
   * it is cumulative.  See TI appnote SLUUB41.
   */

*volts = ((uint32_t) regval * priv->gain * 4) +
          (priv->offset * bq769x0_chip_cellcount(priv));

  return OK;
}

/****************************************************************************
 * Name: bq769x0_getvolt
 *
 * Description:
 *   Gets one or more battery cell voltages from the monitor.
 *
 ****************************************************************************/

static inline int bq769x0_getcellvolt(FAR struct bq769x0_dev_s *priv,
                                      struct battery_monitor_voltage_s *voltages)
  {
  uint16_t regvals[BQ769X0_MAX_CELLS];
  int ret;
  int i;
  int cellsread;

  if (voltages)
    {

      /*Check how many cells were requested.  If more than available,
       * overwrite with the number available.
       */

     if (voltages->cell_count > priv->cellcount)
       {
         voltages->cell_count = priv->cellcount;
       }
    }

  /* Due to gaps in cell voltages when the whole stack is not filled,
   * We'll read the maximum number of cells supported by the chip
   * and discard what we don't need.
   */
  cellsread = bq769x0_chip_cellcount(priv);
  ret = bq769x0_getnreg16(priv, BQ769X0_REG_VC1_HI, regvals, cellsread);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  for (i = 0; i < voltages->cell_count; i += 1)
    {
      /* Voltage is returned from the chip in units of <gain>uV/LSB
       * An offset also needs to be added.
       * We use the mapping table to determine mapping between cell number
       * and ADC channel
       */

      voltages->cell_voltages[i] = ((uint32_t) regvals[priv->mapping[i]] *
                                    priv->gain) + priv->offset;
  }
  return OK;
}

/****************************************************************************
 * Name: bq769x0_gettemperature
 *
 * Description:
 *   Gets the voltage(s) at the temperature sensor input(s) of the chip
 *   It is up to the user to convert these voltage values into temperature
 *   values, as many types of temperature sensors exist.
 *
 ****************************************************************************/
static inline int bq769x0_gettemperature(FAR struct bq769x0_dev_s *priv,
                                         struct battery_monitor_temperature_s *temps)
{
  int chip_sensors;
  int ret;
  int i;
  uint16_t regvals[3];

  /* The number of temperature registers varies depending on the chip variant */

  switch (priv->chip)
    {
    case CHIP_BQ76920:
      chip_sensors = BQ76920_TEMP_COUNT;
      break;
    case CHIP_BQ76930:
      chip_sensors = BQ76930_TEMP_COUNT;
      break;
    default:
    case CHIP_BQ76940:
      chip_sensors = BQ76940_TEMP_COUNT;
      break;
    }

  /* Read the number of sensors requested or available, whichever is smaller
   * We replace the requested count with the number of channels actually read
   */

  temps->sensor_count = MIN(chip_sensors, temps->sensor_count);
  ret = bq769x0_getnreg16(priv, BQ769X0_REG_TS1_HI, regvals, temps->sensor_count);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Convert temp sensor ADC values to microvolts */

  for (i = 0; i < temps->sensor_count; i += 1)
    {
      temps->temperatures[i] = ((uint32_t) regvals[i] * priv->gain) + priv->offset;
    }
  return OK;

}

/****************************************************************************
 * Name: bq769x0_chip_cellcount
 *
 * Description:
 *   Returns the number of cell channels on the specified part
 *
 ****************************************************************************/
static int bq769x0_chip_cellcount(FAR struct bq769x0_dev_s *priv)
{
  switch (priv->chip)
    {
    case CHIP_BQ76920:
      return BQ76920_MAX_CELL_COUNT;
      break;

    case CHIP_BQ76930:
      return BQ76930_MAX_CELL_COUNT;
      break;

    default:
    case CHIP_BQ76940:
      return BQ76940_MAX_CELL_COUNT;
      break;
    }
}

/****************************************************************************
 * Name: bq769x0_getcc
 *
 * Description:
 *   Gets the value of the Coulomb counter from the BQ769X0
 *
 ****************************************************************************/

static inline int bq769x0_getcc(FAR struct bq769x0_dev_s *priv, int *cc)
{
  int16_t regval;
  int ret;

#warning scaling
  ret = bq769x0_getreg16(priv, BQ769X0_REG_CC_HI, (uint16_t *)&regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  *cc = regval;
  return OK;
}

/****************************************************************************
 * Name: bq769x0_setbalance
 *
 * Description:
 *   Sets the values of the BQ769X0 balance switches
 *
 ****************************************************************************/

static inline int bq769x0_setbalance(FAR struct bq769x0_dev_s *priv,
                                         struct battery_monitor_balance_s *bal)
{
  int i;
  int j;
  int cellnum;
  int ret;
  uint8_t regval;

  /*Check how many balance switches were requested.  If more than available,
   * overwrite with the number available.
   */

  if (bal->balance_count > priv->cellcount)
    {
      bal->balance_count = priv->cellcount;
    }

  /* revisit: can we write CELBAL2/CELBAL3 on parts without these registers? */
  for (i = 0; i < BQ769X0_BAL_REG_COUNT; i += 1)
    {
      regval = 0;
      for (j = 0; j < BQ769X0_BAL_BITS_PER_REG; j += 1)
        {
          cellnum = (i * BQ769X0_BAL_BITS_PER_REG) + j;
          if (cellnum < bal->balance_count)
            {
              #warning not currently mapped correctly
              /* Fixme: remap balance bits depending on cell count */
              /* also fixme: do we need to set intermediate switches to on or off when balancing an incomplete set of cells? */
              regval |= (bal->balance[cellnum] ? 1 : 0) << j;
            }
        }
      ret = bq769x0_putreg8(priv, BQ769X0_REG_CELLBAL1 + i, regval);
      if (ret < 0)
        {
          baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
          return ret;
            }
    }
  return OK;
}

/****************************************************************************
 * Name: bq769x0_voltage
 *
 * Description:
 *   Get the pack voltage
 *
 ****************************************************************************/

static int bq769x0_voltage(struct battery_monitor_dev_s *dev, int *value)
{
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  int ret;

  /* Get pack voltage from battery monitor */

  ret = bq769x0_getvolt(priv, value);
  if (ret < 0)
    {
      baterr("ERROR: Error getting voltage from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_voltage
 *
 * Description:
 *   Get 1 or more cell voltages
 *
 ****************************************************************************/

static int bq769x0_cellvoltage(struct battery_monitor_dev_s *dev,
                               struct battery_monitor_voltage_s *cellv) {

  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  int ret;

  /* Get cell voltages from battery monitor */

  ret = bq769x0_getcellvolt(priv, cellv);
  if (ret < 0)
    {
      baterr("ERROR: Error getting voltage from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_current
 *
 * Description:
 *   Get the pack current
 *
 ****************************************************************************/

static int bq769x0_current(struct battery_monitor_dev_s *dev, int *value)
{

  /* The BQ769X0 does not support directly reporting pack curent, although you could
   * come up with an average value by looking at the Coulomb counter over time.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: bq769x0_soc
 *
 * Description:
 *   Get the pack state of charge (in percent)
 *
 ****************************************************************************/

static int bq769x0_soc(struct battery_monitor_dev_s *dev, b16_t *value) {

  /* The BQ769X0 does not support directly reporting pack state of charge.
   * You should be able to come up with a state-of-charge value by knowing an
   * initial value and looking at the Coulomb counter
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: bq769x0_coulombs
 *
 * Description:
 *   Get the raw value of the pack coulomb counter
 *
 ****************************************************************************/

static int bq769x0_coulombs(struct battery_monitor_dev_s *dev, int *coulombs) {

  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  int ret;

  /* Get cell voltages from battery monitor */

  ret =     bq769x0_getcc(priv, coulombs);
  if (ret < 0)
    {
      baterr("ERROR: Error getting coulombs from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_temp
 *
 * Description:
 *   Get the pack temperature(s)
 *
 ****************************************************************************/

static int bq769x0_temp(struct battery_monitor_dev_s *dev,
                        struct battery_monitor_temperature_s *temps)
{
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  int ret;

  ret =  bq769x0_gettemperature(priv, temps);
  if (ret < 0)
    {
      baterr("ERROR: Error getting temperature from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_balance
 *
 * Description:
 *   Set the specified cell balance switches
 *
 ****************************************************************************/

static int bq769x0_balance(struct battery_monitor_dev_s *dev,
                           struct battery_monitor_balance_s *bal) {
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  int ret;

  ret =  bq769x0_setbalance(priv, bal);
  if (ret < 0)
    {
      baterr("ERROR: Error getting temperature from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  return OK;
}
/****************************************************************************
 * Name: bq769x0_operate
 *
 * Description:
 *   Do miscellaneous battery ioctl()
 *
 ****************************************************************************/

static int bq769x0_operate(struct battery_monitor_dev_s *dev, uintptr_t param)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq769x0_initialize
 *
 * Description:
 *   Initialize the BQ769x0 battery driver and return an instance of the
 *   lower_half interface that may be used with battery_monitor_register();
 *
 *   This driver requires:
 *
 *   CONFIG_BATTERY_MONITOR - Upper half battery driver support
 *   CONFIG_I2C - I2C support
 *   CONFIG_I2C_BQ769X0 - And the driver must be explictly selected.
 *
 * Input Parameters:
 *   i2c       - An instance of the I2C interface to use to communicate with
 *               the BQ769x0
 *   addr      - The I2C address of the BQ769X0 (Can be 0x08 or 0x18).
 *   frequency - The I2C frequency
 *   crc       - True if the device has CRC enabled (see TI datasheet)
 *   cellcount - The number of battery cells attached to the BQ769X0.  The
 *               mapping of the cells changes based on count - see datasheet.
 *   chip      - The chip type (either CHIP_BQ76920, CHIP_BQ76930, or
 *               CHIP_BQ76940).  This is used to map cell numbers when the
 *               full capacity of the chip is not used.  See the TI datasheet
 *               for cell wiring information.
 *
 * Returned Value:
 *   A pointer to the initialized lower-half driver instance.  A NULL pointer
 *   is returned on a failure to initialize the BQ769x0 lower half.
 *
 ****************************************************************************/

FAR struct battery_monitor_dev_s *
  bq769x0_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                     uint32_t frequency, bool crc, uint8_t cellcount,
                     uint8_t chip)
{
  FAR struct bq769x0_dev_s *priv;
  int ret;

  /* Initialize the BQ769x0 device structure */

  priv = (FAR struct bq769x0_dev_s *)kmm_zalloc(sizeof(struct bq769x0_dev_s));
  if (priv)
    {
      /* Initialize the BQ769x0 device structure */

      nxsem_init(&priv->batsem, 0, 1);
      priv->ops       = &g_bq769x0ops;
      priv->i2c       = i2c;
      priv->addr      = addr;
      priv->frequency = frequency;
      priv->crc       = crc;
      priv->chip      = chip;
      priv->cellcount = cellcount;

      /* Sanity check the device setup and assign cell mapping table */
      switch (chip)
        {
        case CHIP_BQ76920:
          if (cellcount < BQ76920_MIN_CELL_COUNT ||
              cellcount > BQ76920_MAX_CELL_COUNT)
            {
              berr("ERROR: Invalid number of cells (%d) for BQ76920\n",
                   cellcount);
              kmm_free(priv);
              return NULL;
            } else {
                priv->mapping = bq76920_cell_mapping[cellcount -
                                                     BQ76920_MIN_CELL_COUNT];
            }
          break;

        case CHIP_BQ76930:
          if (cellcount < BQ76930_MIN_CELL_COUNT ||
              cellcount > BQ76930_MAX_CELL_COUNT)
            {
              berr("ERROR: Invalid number of cells (%d) for BQ76930\n",
                   cellcount);
              kmm_free(priv);
              return NULL;
            } else {
                priv->mapping = bq76930_cell_mapping[cellcount -
                                                     BQ76930_MIN_CELL_COUNT];
            }
          break;

        case CHIP_BQ76940:
          if (cellcount < BQ76940_MIN_CELL_COUNT ||
              cellcount > BQ76940_MAX_CELL_COUNT)
            {
              berr("ERROR: Invalid number of cells (%d) for BQ76940\n",
                   cellcount);
              kmm_free(priv);
              return NULL;
            } else {
                priv->mapping = bq76940_cell_mapping[cellcount -
                                                     BQ76940_MIN_CELL_COUNT];
            }
          break;

        default:
          berr("ERROR: Unrecognized chip type: %d\n", chip);
          kmm_free(priv);
          return NULL;
          break;
        }

      /* Configure the BQ769x0 */
      ret = bq769x0_putreg8(priv, BQ769X0_REG_CC_CFG, BQ769X0_CC_CFG_DEFAULT_VAL);
      if (ret < 0)
        {
          baterr("ERROR: Failed to configure the BQ769x0: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Pull the factory-calibrated gain and offset values from the chip. */

      ret = bq769x0_updategain(priv);
      if (ret < 0)
        {
          baterr("ERROR: Failed to get gain/offset values from the BQ769x0: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }


    }

  return (FAR struct battery_monitor_dev_s *)priv;
}

#endif /* CONFIG_BATTERY_MONITOR && CONFIG_I2C && CONFIG_I2C_BQ769X0 */
