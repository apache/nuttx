/****************************************************************************
 * drivers/power/bq769x0.c
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

/* Lower half driver for BQ769x0 battery monitor */

/* The bq76920/bq76930/bq76940 battery monitor ICs provide voltage, current,
 * and temperature monitoring of up to 15-series cells.  These ICs also
 * provide Coulomb counting for state-of-charge measurement, balance drivers
 * for all cells, and drivers for external cell protection switches.
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
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
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
#  define baterr    _none
#  define batreg    _none
#  define batinfo   _none
#endif

/****************************************************************************
 * Private
 ****************************************************************************/

struct bq769x0_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  struct battery_monitor_dev_s dev; /* Battery monitor device */

  /* Data fields specific to the lower half BQ769x0 driver follow */

  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* I2C address */
  uint8_t chip;                 /* Chip Type (e.g. CHIP_76920) */
  uint8_t cellcount;            /* Number of cells attached to chip */
  uint8_t fault_cache;          /* Cache of last-read fault bits */
  uint32_t frequency;           /* I2C frequency */
  uint32_t gain;                /* ADC gain value in uV */
  uint32_t offset;              /* ADC offset value in uV */
  uint32_t sense_r;             /* Current sense resistor, in uOhm */
  const uint8_t *mapping;       /* Pointer to cell mapping table */
  bool crc;                     /* True if the device has CRC enabled */
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
    0, 1, 2, 4, 5, 6, 9
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

/* Current sense limit mapping tables
 * Maps a voltage threshold (in mV, array value)
 * to a register field value (array index)
 * The last values in each list are somewhat arbitrary upper bounds -
 * The algorithm rounds down when selecting a register value
 */

static const uint8_t ocd_t_rsns_0_limits[] =
    {
        8, 11, 14, 17, 19, 22, 25, 28, 31, 33, 36, 39, 42, 44, 47, 50, 53
    };
static const uint8_t ocd_t_rsns_1_limits[] =
    {
        17, 22, 28, 33, 39, 44, 50, 56, 61, 67, 72, 78, 83, 89, 94, 100, 106
    };
static const uint8_t scd_t_rsns_0_limits[] =
    {
        22, 33, 44, 56, 67, 78, 89, 100, 105
    };
static const uint8_t scd_t_rsns_1_limits[] =
    {
        44, 67, 89, 111, 133, 155, 178, 200, 210
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

static int bq769x0_getreport(FAR struct bq769x0_dev_s *priv,
                             FAR uint8_t *report);
static int bq769x0_getvolt(FAR struct bq769x0_dev_s *priv, FAR int *volts);
static int bq769x0_getcurrent(FAR struct bq769x0_dev_s *priv,
                              FAR struct battery_monitor_current_s *current);
static int bq769x0_getcellvolt(FAR struct bq769x0_dev_s *priv,
                             FAR struct battery_monitor_voltage_s *voltages);
static int bq769x0_gettemperature(FAR struct bq769x0_dev_s *priv,
                            FAR struct battery_monitor_temperature_s *temps);
static int bq769x0_setbalance(FAR struct bq769x0_dev_s *priv,
                              FAR struct battery_monitor_balance_s *bal);
static int bq769x0_doshutdown(FAR struct bq769x0_dev_s *priv);
static int bq769x0_setlimits(FAR struct bq769x0_dev_s *priv,
                             FAR struct battery_monitor_limits_s *limits);
static int bq769x0_setchgdsg(FAR struct bq769x0_dev_s *priv,
                             FAR struct battery_monitor_switches_s *sw);
static int bq769x0_clear_chipfaults(FAR struct bq769x0_dev_s *priv,
                                    uint8_t faults);
static int bq769x0_updategain(FAR struct bq769x0_dev_s *priv);
static int bq769x0_chip_cellcount(FAR struct bq769x0_dev_s *priv);

/* Battery driver lower half methods */

static int bq769x0_state(struct battery_monitor_dev_s *dev, int *status);
static int bq769x0_health(struct battery_monitor_dev_s *dev, int *health);
static int bq769x0_online(struct battery_monitor_dev_s *dev, bool *status);
static int bq769x0_voltage(struct battery_monitor_dev_s *dev, int *value);
static int bq769x0_cellvoltage(struct battery_monitor_dev_s *dev,
                               struct battery_monitor_voltage_s *cellv);
static int bq769x0_current(struct battery_monitor_dev_s *dev,
                           struct battery_monitor_current_s *current);
static int bq769x0_soc(struct battery_monitor_dev_s *dev, b16_t *value);
static int bq769x0_coulombs(struct battery_monitor_dev_s *dev,
                            int *coulombs);
static int bq769x0_temp(struct battery_monitor_dev_s *dev,
                        struct battery_monitor_temperature_s *temps);
static int bq769x0_balance(struct battery_monitor_dev_s *dev,
                           struct battery_monitor_balance_s *bal);
static int bq769x0_shutdown(struct battery_monitor_dev_s *dev,
                            uintptr_t param);
static int bq769x0_limits(struct battery_monitor_dev_s *dev,
                          struct battery_monitor_limits_s *limits);
static int bq769x0_chgdsg(struct battery_monitor_dev_s *dev,
                          struct battery_monitor_switches_s *sw);
static int bq769x0_clearfaults(struct battery_monitor_dev_s *dev,
                               uintptr_t param);
static int bq769x0_operate(struct battery_monitor_dev_s *dev,
                           uintptr_t param);

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
  bq769x0_shutdown,
  bq769x0_limits,
  bq769x0_chgdsg,
  bq769x0_clearfaults,
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

  /* Our expected data length varies depending on whetherCRC is used */

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
          baterr("ERROR: CRC mismatch: Got %02x, Expected %02x\n", val[1],
                 crc);
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

  /* Our expected data length varies depending on whether CRC is used */

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

  /* Make sure specified number of registers will fit in our buffer.
   * If not, limit read to the available buffer size
   */

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

  /* Our expected I2C data length varies depending on whether CRC is used */

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
   * After that, we compare the CRC of each byte with its following byte
   */

  if (priv->crc)
    {
      sl_addr = RD_ADDR(priv->addr);
      crc = crc8ccittpart(&sl_addr, 1, 0);
      for (i = 0; i < byte_count; i += 2)
        {
          crc = crc8ccittpart(&tmp_val[i], 1, crc);
          if (crc != tmp_val[i + 1])
            {
              baterr("ERROR: CRC mismatch: Got %02x, Expected %02x\n",
                     tmp_val[2], crc);
              return ERROR;
            }

          crc = 0;
        }

      /* Copy 16-bit values to be returned, skipping CRC bytes */

      for (i = 0; i < datalen; i += 4)
        {
          *regvals = (uint16_t)tmp_val[i] << 8 | (uint16_t)tmp_val[i + 2];
          regvals += 1;
        }
    }
  else
    {
      /* Copy 16-bit values to be returned */

      for (i = 0; i < datalen; i += 2)
        {
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

static int bq769x0_getreport(FAR struct bq769x0_dev_s *priv,
                             FAR uint8_t *report)
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

static int bq769x0_updategain(FAR struct bq769x0_dev_s *priv)
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

  batinfo("Battery monitor gain: %d uV/LSB, offset: %d uV.\n", priv->gain,
          priv->offset);

  return OK;
}

/****************************************************************************
 * Name: bq769x0_do_shutdown
 *
 * Description:
 * Put the device into a low-power SHIP mode.
 * External hardware may be required to wake the device up from this state.
 *
 ****************************************************************************/

static int bq769x0_doshutdown(FAR struct bq769x0_dev_s *priv)
{
  int ret;
  uint8_t regval;

  /* Read current register value */

  ret = bq769x0_getreg8(priv, BQ769X0_REG_SYS_CTRL1, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Mask off the writeable bits */

  regval &= BQ769X0_SYS_CTRL1_WRITE_MASK;

  /* Set SHUT_A and SHUT_B to 0 */

  regval &= ~BQ769X0_SYS_CTRL1_SHUTDOWN_MASK;

  /* Write the shutdown sequence */

  ret = bq769x0_putreg8(priv, BQ769X0_REG_SYS_CTRL1, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Set SHUT_A to 0 and SHUT_B to 1 */

  regval |= BQ769X0_SHUT_B;

  ret = bq769x0_putreg8(priv, BQ769X0_REG_SYS_CTRL1, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Set SHUT_A to 1 and SHUT_B to 0 */

  regval &= ~BQ769X0_SYS_CTRL1_SHUTDOWN_MASK;
  regval |= BQ769X0_SHUT_A;

  ret = bq769x0_putreg8(priv, BQ769X0_REG_SYS_CTRL1, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ769X0! Error = %d\n", ret);
      return ret;
    }

  batinfo("Device should now be in ship mode\n");

  return OK;
}

/****************************************************************************
 * Name: bq769x0_setlimits
 *
 * Description:
 * Set the safety cutoff limits of the device.
 *
 ****************************************************************************/

static int bq769x0_setlimits(FAR struct bq769x0_dev_s *priv,
                             FAR struct battery_monitor_limits_s *limits)
{
  int ret;
  int i;
  uint8_t regval;
  uint32_t tripval;
  bool rsns_0_scd_found;
  bool rsns_1_scd_found;
  bool rsns_0_ocd_found;
  bool rsns_1_ocd_found;

  uint8_t rsns_0_scd_idx;
  uint8_t rsns_1_scd_idx;
  uint8_t rsns_0_ocd_idx;
  uint8_t rsns_1_ocd_idx;

  regval = 0;

  /* The OCD (Over current in discharge) and SCD
   * (Short circuit in discharge) registers are both
   * affected by the RSNS bit.  We ideally want to find
   * a mapping that satisfies both registers for the provided values
   * using only a single RSNS value.
   */

  /* Compute overcurrent voltage trip point based on provided
   * current trip point.
   */

  tripval = limits->overcurrent_limit * priv->sense_r;

  /* result is in milli-amps * micro-ohms
   * e.g. 20A * 5 milli-ohms = 20000 * 5000 = 100000000
   * Divide by 1000000 to get millivolts
   */

  tripval /= 1000000UL;

  batinfo("Overcurrent trip voltage is %d mV\n", tripval);

  /* Now look up overcurrent limit value in the OCD_T
   * lookup tables.  Check both RSNS = 0 and RSNS = 1
   * so we can make a decision about which one to use.
   * Note that limits lower than the minimum will be set to the
   * minimum value.
   */

  rsns_0_ocd_found = false;
  rsns_1_ocd_found = false;
  rsns_0_ocd_idx = 0;
  rsns_1_ocd_idx = 0;
  for (i = 1; i < sizeof(ocd_t_rsns_0_limits) /
       sizeof(ocd_t_rsns_0_limits[0]); i += 1)
    {
      if (tripval <= ocd_t_rsns_0_limits[i])
        {
          rsns_0_ocd_idx = i - 1; /* round down */
          rsns_0_ocd_found = true;
          break;
        }
    }

  for (i = 1; i < sizeof(ocd_t_rsns_1_limits) /
       sizeof(ocd_t_rsns_1_limits[0]); i += 1)
    {
      if (tripval <= ocd_t_rsns_1_limits[i])
        {
          rsns_1_ocd_idx = i - 1; /* round down */
          rsns_1_ocd_found = true;
          break;
        }
    }

  if (!rsns_0_ocd_found && !rsns_1_ocd_found)
    {
      baterr("ERROR: Failed to find suitable value for OCD_T\n");
      return -EINVAL;
    }

  /* Compute short circuit voltage trip point based on provided
   * current trip point
   */

  tripval = limits->shortcircuit_limit * priv->sense_r;

  /* result is in milli-amps * micro-ohms
   * e.g. 20A * 5 milli-ohms = 20000 * 5000 = 100000000
   * Divide by 1000000 to get millivolts
   */

  tripval /= 1000000UL;
  batinfo("Short circuit trip voltage is %d mV\n", tripval);

  /* Now look up the short circuit limit value in the SCD_T
   * lookup tables.  Check both RSNS = 0 and RSNS = 1
   * so we can make a decision about which one to use.
   * Note that limits lower than the minimum will be set to the
   * minimum value.
   */

  rsns_0_scd_found = false;
  rsns_1_scd_found = false;
  rsns_0_scd_idx = 0;
  rsns_1_scd_idx = 0;

  /* Don't look at the first element since we're rounding down anyway */

  for (i = 1; i < sizeof(scd_t_rsns_0_limits) /
       sizeof(scd_t_rsns_0_limits[0]); i += 1)
    {
      if (tripval < scd_t_rsns_0_limits[i])
        {
          rsns_0_scd_idx = i - 1; /* round down */
          rsns_0_scd_found = true;
          break;
        }
    }

  for (i = 1; i < sizeof(scd_t_rsns_1_limits) /
       sizeof(scd_t_rsns_1_limits[0]); i += 1)
    {
      if (tripval < scd_t_rsns_1_limits[i])
        {
          rsns_1_scd_idx = i - 1; /* round down */
          rsns_1_scd_found = true;
          break;
        }
    }

  if (!rsns_0_scd_found && !rsns_1_scd_found)
    {
      baterr("ERROR: Failed to find suitable value for SCD_T\n");
      return -EINVAL;
    }

  /* Now let's figure out RSNS.
   * We prefer RSNS = 0 if available, because it gives us finer-grained
   * control over the actual trip voltage
   */

  if (rsns_0_ocd_found && rsns_0_scd_found)
    {
      batinfo("Using RSNS = 0\n");
      batinfo("Using SCD_T %x\n", rsns_0_scd_idx);
      regval |= (rsns_0_scd_idx << BQ769X0_SCD_THRESH_SHIFT) &
                BQ769X0_SCD_THRESH_MASK;
    }
  else if (rsns_1_ocd_found && rsns_1_scd_found)
    {
      batinfo("Using RSNS = 1\n");
      batinfo("Using SCD_T %x\n", rsns_1_scd_idx);
      regval |= BQ769X0_RSNS;
      regval |= (rsns_1_scd_idx << BQ769X0_SCD_THRESH_SHIFT) &
                BQ769X0_SCD_THRESH_MASK;
    }
  else
    {
      /* Not possible to meet both trip points with a single RSNS value
       * For now, let's call that an error.
       */

      limits->overcurrent_limit = 0;
      limits->shortcircuit_limit = 0;
      baterr("ERROR: OCD_T and SCD_T could not agree on RSNS.\n");
      return -EINVAL;
    }

  /* Configure short circuit delay and threshold.
   * Always round down if we are less than the
   * next highest value.
   * Throw an error if we are out of bounds
   * (+/- an arbitrarily-chosen buffer)
   */

  if (limits->shortcircuit_delay < 68)
    {
      limits->shortcircuit_delay = 0;
      baterr("ERROR: Short circuit delay is too short\n");
      return -EINVAL;
    }
  else if (limits->shortcircuit_delay < 100)
    {
      limits->shortcircuit_delay = 70;
      regval |= BQ769X0_SCD_DELAY_70US;
      batinfo("Short circuit delay set to 70uS\n");
    }
  else if (limits->shortcircuit_delay < 200)
    {
      limits->shortcircuit_delay = 100;
      regval |= BQ769X0_SCD_DELAY_100US;
      batinfo("Short circuit delay set to 100uS\n");
    }
  else if (limits->shortcircuit_delay < 400)
    {
      limits->shortcircuit_delay = 200;
      regval |= BQ769X0_SCD_DELAY_200US;
      batinfo("Short circuit delay set to 200uS\n");
    }
  else if (limits->shortcircuit_delay < 410)
    {
      limits->shortcircuit_delay = 400;
      regval |= BQ769X0_SCD_DELAY_400US;
      batinfo("Short circuit delay set to 400uS\n");
    }
  else
    {
      limits->shortcircuit_delay = 0;
      baterr("ERROR: Short circuit delay is too long\n");
      return -EINVAL;
    }

  ret = bq769x0_putreg8(priv, BQ769X0_REG_PROTECT1, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Configure overcurrent delay and threshold
   * Always round down if we are less than the
   * next highest value.
   * Throw an error if we are out of bounds
   * (+/- an arbitrarily-chosen buffer)
   */

  regval = 0;
  if (limits->overcurrent_delay < (7 * USEC_PER_MSEC))
    {
      limits->overcurrent_delay = 0;
      baterr("ERROR: Overcurrent delay is too short\n");
      return -EINVAL;
    }
  else if (limits->overcurrent_delay < (20 * USEC_PER_MSEC))
    {
      limits->overcurrent_delay = 8 * USEC_PER_MSEC;
      regval |= BQ769X0_OCD_DELAY_8MS;
      batinfo("Overcurrent delay set to 8mS\n");
    }
  else if (limits->overcurrent_delay < (40 * USEC_PER_MSEC))
    {
      limits->overcurrent_delay = 20 * USEC_PER_MSEC;
      regval |= BQ769X0_OCD_DELAY_20MS;
      batinfo("Overcurrent delay set to 20mS\n");
    }
  else if (limits->overcurrent_delay < (80 * USEC_PER_MSEC))
    {
      limits->overcurrent_delay = 40 * USEC_PER_MSEC;
      regval |= BQ769X0_OCD_DELAY_40MS;
      batinfo("Overcurrent delay set to 40mS\n");
    }
  else if (limits->overcurrent_delay < (160 * USEC_PER_MSEC))
    {
      limits->overcurrent_delay = 80 * USEC_PER_MSEC;
      regval |= BQ769X0_OCD_DELAY_80MS;
      batinfo("Overcurrent delay set to 80mS\n");
    }
  else if (limits->overcurrent_delay < (320 * USEC_PER_MSEC))
    {
      limits->overcurrent_delay = 160 * USEC_PER_MSEC;
      regval |= BQ769X0_OCD_DELAY_160MS;
      batinfo("Overcurrent delay set to 160mS\n");
    }
  else if (limits->overcurrent_delay < (640 * USEC_PER_MSEC))
    {
      limits->overcurrent_delay = 320 * USEC_PER_MSEC;
      regval |= BQ769X0_OCD_DELAY_320MS;
      batinfo("Overcurrent delay set to 320mS\n");
    }
  else if (limits->overcurrent_delay < (1280 * USEC_PER_MSEC))
    {
      limits->overcurrent_delay = 640 * USEC_PER_MSEC;
      regval |= BQ769X0_OCD_DELAY_640MS;
      batinfo("Overcurrent delay set to 640mS\n");
    }
  else if (limits->overcurrent_delay < 1300 * USEC_PER_MSEC)
    {
      limits->overcurrent_delay = 1280 * USEC_PER_MSEC;
      regval |= BQ769X0_OCD_DELAY_1280MS;
      batinfo("Overcurrent delay set to 1280mS\n");
    }
  else
    {
      limits->overcurrent_delay = 0;
      baterr("ERROR: Overcurrent delay is too long\n");
      return -EINVAL;
    }

  /* If neither rsns_0 or rsns_1 work, we would have
   * errored out before this point.
   */

  if (rsns_0_ocd_found)
    {
      batinfo("Using OCD_T %x\n", rsns_0_ocd_idx);
      regval |= (rsns_0_ocd_idx << BQ769X0_OCD_THRESH_SHIFT) &
                BQ769X0_OCD_THRESH_MASK;
    }
  else if (rsns_1_ocd_found)
    {
      batinfo("Using OCD_T %x\n", rsns_1_ocd_idx);
      regval |= (rsns_1_ocd_idx << BQ769X0_OCD_THRESH_SHIFT) &
                BQ769X0_OCD_THRESH_MASK;
    }

  ret = bq769x0_putreg8(priv, BQ769X0_REG_PROTECT2, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Configure overvoltage and undervoltage delays
   * Throw an error if we are out of bounds
   * (+/- an arbitrarily-chosen buffer)
   */

  regval = 0;
  if (limits->overvoltage_delay < (1 * USEC_PER_SEC))
    {
      limits->overvoltage_delay = 0;
      baterr("ERROR: overvoltage delay is too short\n");
      return -EINVAL;
    }
  else if (limits->overvoltage_delay < (2 * USEC_PER_SEC))
    {
      limits->overvoltage_delay = 1 * USEC_PER_SEC;
      regval |= BQ769X0_OV_DELAY_1S;
      batinfo("Overvoltage delay set to 1S\n");
    }
  else if (limits->overvoltage_delay < (4 * USEC_PER_SEC))
    {
      limits->overvoltage_delay = 2 * USEC_PER_SEC;
      regval |= BQ769X0_OV_DELAY_2S;
      batinfo("Overvoltage delay set to 2S\n");
    }
  else if (limits->overvoltage_delay < (8 * USEC_PER_SEC))
    {
      limits->overvoltage_delay = 4 * USEC_PER_SEC;
      regval |= BQ769X0_OV_DELAY_4S;
      batinfo("Overvoltage delay set to 4S\n");
    }
  else if (limits->overvoltage_delay < (10 * USEC_PER_SEC))
    {
      limits->overvoltage_delay = 8 * USEC_PER_SEC;
      regval |= BQ769X0_OV_DELAY_8S;
      batinfo("Overvoltage delay set to 8S\n");
    }
  else
    {
      limits->overvoltage_delay = 0;
      baterr("ERROR: overvoltage delay is too long\n");
      return -EINVAL;
    }

  if (limits->undervoltage_delay < (1 * USEC_PER_SEC))
    {
      limits->undervoltage_delay = 0;
      baterr("ERROR: undervoltage delay is too short\n");
      return -EINVAL;
    }
  else if (limits->undervoltage_delay < (4 * USEC_PER_SEC))
    {
      limits->undervoltage_delay = 1 * USEC_PER_SEC;
      regval |= BQ769X0_UV_DELAY_1S;
      batinfo("Undervoltage delay set to 1S\n");
    }
  else if (limits->undervoltage_delay < (8 * USEC_PER_SEC))
    {
      limits->undervoltage_delay = 4 * USEC_PER_SEC;
      regval |= BQ769X0_UV_DELAY_4S;
      batinfo("Undervoltage delay set to 4S\n");
    }
  else if (limits->undervoltage_delay < (16 * USEC_PER_SEC))
    {
      limits->undervoltage_delay = 8 * USEC_PER_SEC;
      regval |= BQ769X0_UV_DELAY_8S;
      batinfo("Undervoltage delay set to 8S\n");
    }
  else if (limits->undervoltage_delay < (20 * USEC_PER_SEC))
    {
      limits->undervoltage_delay = 16 * USEC_PER_SEC;
      regval |= BQ769X0_UV_DELAY_16S;
      batinfo("Undervoltage delay set to 16S\n");
    }
  else
    {
      limits->undervoltage_delay = 0;
      baterr("ERROR: undervoltage delay is too long\n");
      return -EINVAL;
    }

  ret = bq769x0_putreg8(priv, BQ769X0_REG_PROTECT3, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Calculate OV_TRIP register value based on provided limit.
   * Note that the register format limits the trip range to
   * approximately 3.15V to 4.7V
   */

  tripval = (limits->overvoltage_limit - priv->offset) / priv->gain;
  tripval >>= 4;
  regval = (uint8_t)(tripval & 0xff);
  ret = bq769x0_putreg8(priv, BQ769X0_REG_OV_TRIP, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Calculate UV_TRIP register value based on provided limit.
   * Note that the register format limits the trip range to
   * approximately 1.58V to 3.1V
   */

  tripval = (limits->undervoltage_limit - priv->offset) / priv->gain;
  tripval >>= 4;
  regval = (uint8_t)(tripval & 0xff);
  ret = bq769x0_putreg8(priv, BQ769X0_REG_UV_TRIP, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ769X0! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_setlimits
 *
 * Description:
 * Set the device's charge/discharge switches
 *
 ****************************************************************************/

static int bq769x0_setchgdsg(FAR struct bq769x0_dev_s *priv,
                             FAR struct battery_monitor_switches_s *sw)
{
  int ret;
  uint8_t regval;

  /* Read current register value */

  ret = bq769x0_getreg8(priv, BQ769X0_REG_SYS_CTRL2, &regval);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Mask off the writeable bits */

  regval &= BQ769X0_SYS_CTRL2_WRITE_MASK;

  /* Set CHG_ON and DSG_ON */

  regval &= ~BQ769X0_SYS_CTRL2_CHGDSG_MASK;
  if (sw->charge)
    {
      regval |= BQ769X0_CHG_ON;
      batinfo("Turned on charge switch\n");
    }

  if (sw->discharge)
    {
      regval |= BQ769X0_DSG_ON;
      batinfo("Turned on discharge switch\n");
    }

  /* Write the new register value */

  ret = bq769x0_putreg8(priv, BQ769X0_REG_SYS_CTRL2, regval);
  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ769X0! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_clear_chip_faults
 *
 * Description:
 * Clear the specified chip faults
 *
 ****************************************************************************/

static int bq769x0_clear_chipfaults(FAR struct bq769x0_dev_s *priv,
                                    uint8_t faults)
{
  int ret;

  batinfo("Clearing battery faults: %02x\n", faults);

  /* Write the new register value */

  ret = bq769x0_putreg8(priv, BQ769X0_REG_SYS_STAT, faults);

  if (ret < 0)
    {
      baterr("ERROR: Error writing to BQ769X0! Error = %d\n", ret);
      return ret;
    }

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

  if (regval & BQ769X0_FAULT_MASK)
    {
      *status = BATTERY_FAULT;
    }
  else
    {
      *status = BATTERY_IDLE;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_health
 *
 * Description:
 *   Return the current battery health state
 *
 * Note: if more than one fault happened the user will need to
 * clear the fault and call this ioctl again to read a new fault,
 * repeat until receive a BATTERY_HEALTH_GOOD.
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

  priv->fault_cache = regval;

  if (regval & BQ769X0_DEVICE_XREADY)
    {
      *health = BATTERY_HEALTH_WD_TMR_EXP;
    }
  else if (regval & BQ769X0_SCD)
    {
      *health = BATTERY_HEALTH_SHORT_CIRCUIT;
    }
  else if (regval & BQ769X0_OCD)
    {
      *health = BATTERY_HEALTH_OVERCURRENT;
    }
  else if (regval & BQ769X0_OV)
    {
      *health = BATTERY_HEALTH_OVERVOLTAGE;
    }
  else if (regval & BQ769X0_UV)
    {
      *health = BATTERY_HEALTH_UNDERVOLTAGE;
    }
  else if (regval & BQ769X0_OVRD_ALERT)
    {
      *health = BATTERY_HEALTH_UNSPEC_FAIL;
    }
  else
    {
     *health = BATTERY_HEALTH_GOOD;
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

static int bq769x0_getvolt(FAR struct bq769x0_dev_s *priv, int *volts)
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

static int bq769x0_getcellvolt(FAR struct bq769x0_dev_s *priv,
                              FAR struct battery_monitor_voltage_s *voltages)
{
  uint16_t regvals[BQ769X0_MAX_CELLS];
  int ret;
  int i;
  int cellsread;

  if (voltages)
    {
      /* Check how many cells were requested.  If more than available,
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

static int bq769x0_gettemperature(FAR struct bq769x0_dev_s *priv,
                             FAR struct battery_monitor_temperature_s *temps)
{
  int chip_sensors;
  int ret;
  int i;
  uint16_t regvals[3];

  /* The number of temperature registers varies depending on the
   * chip variant
   */

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
  ret = bq769x0_getnreg16(priv, BQ769X0_REG_TS1_HI, regvals,
                          temps->sensor_count);
  if (ret < 0)
    {
      baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  /* Convert temp sensor ADC values to microvolts */

  for (i = 0; i < temps->sensor_count; i += 1)
    {
      temps->temperatures[i] = ((uint32_t) regvals[i] * priv->gain) +
                               priv->offset;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_chip_cellcount
 *
 * Description:
 *   Returns the number of cell channels on the specified device
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
 * Name: bq769x0_getcurrent
 *
 * Description:
 *   Gets the value of the battery current as measured by the BQ769X0
 *
 ****************************************************************************/

static int bq769x0_getcurrent(FAR struct bq769x0_dev_s *priv,
                              FAR struct battery_monitor_current_s *current)
{
  /* The BQ769X0's "coulomb counter" reports average current over a 250ms
   * period. This can be integrated by the user application to measure
   * amp-hours.
   */

  int i;
  uint8_t regval;
  int16_t ccval;
  int32_t ccvolts;
  int32_t ccamps;
  int ret;

  /* Poll SYS_STAT register until a new Coulomb counter value is ready
   * or until we time out
   */

  for (i = 0; i < 6; i += 1)
    {
      ret = bq769x0_getreg8(priv, BQ769X0_REG_SYS_STAT, &regval);
      if (ret < 0)
        {
          baterr("ERROR: Failed to read BQ769X0 Status! Error = %d\n", ret);
          return ret;
        }

      if (regval & BQ769X0_CC_READY)
        {
          /* Clear the CC_ready flag */

          ret = bq769x0_putreg8(priv, BQ769X0_REG_SYS_STAT,
                                BQ769X0_CC_READY);
          if (ret < 0)
            {
              baterr("ERROR: Error writing to BQ769X0! Error = %d\n", ret);
              return ret;
            }

          /* Get the CC register data (a signed value) */

          ret = bq769x0_getreg16(priv, BQ769X0_REG_CC_HI,
                                 (uint16_t *)&ccval);
          if (ret < 0)
            {
              baterr("ERROR: Error reading from BQ769X0! Error = %d\n", ret);
              return ret;
            }

          batinfo("Coulomb counter raw value: %d\n", ccval);

          /* Convert coulomb counter to real units
           * Multiply by 4 for some extra resolution
           */

          ccvolts = (int32_t)ccval * (int32_t)BQ769X0_CC_SCALE * (int32_t)4;

          /* ccvolts is nV, sense_r is uOhm.  Result is in mA
           * convert to uA, and don't forget to divide the 4 back out
           */

          ccamps = ccvolts / ((int32_t)priv->sense_r);
          ccamps *= (int32_t)1000;
          ccamps /= (int32_t)4;
          current->current = ccamps;

          /* Acquisition time is constant with this device */

          current->time = (BQ769X0_CC_TIME * USEC_PER_MSEC);
          return OK;
        }

      /* Sample is not complete, wait and try again */

      nxsig_usleep(BQ769X0_CC_POLL_INTERVAL * USEC_PER_MSEC);
    }

  /* CC value didn't become available in the expected amount of time */

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: bq769x0_setbalance
 *
 * Description:
 *   Sets the values of the BQ769X0 balance switches
 *
 ****************************************************************************/

static int bq769x0_setbalance(FAR struct bq769x0_dev_s *priv,
                              FAR struct battery_monitor_balance_s *bal)
{
  int i;
  int j;
  int ret;
  uint8_t regval;
  uint16_t balancebits;

  bool currentbit;
  bool lastbit;
  int currentindex;

  /* Check how many balance switches were requested.  If more than available,
   * overwrite with the number available.
   */

  if (bal->balance_count > priv->cellcount)
    {
      bal->balance_count = priv->cellcount;
    }

  /* Scan through the input and look for adjacent cells in each group.
   * This is not allowed by the chip, so we will remove them.
   * At the same time, copy the balance inputs into a single bit field.
   * This allows us to get cell remapping out of the way.
   * We will never have more than 15 cells, so we can store the
   * result in a 16-bit int.
   */

  balancebits = 0;

  for (i = 0; i < BQ769X0_BAL_REG_COUNT; i += 1)
    {
      lastbit = false;
      for (j = 0; j < BQ769X0_BAL_BITS_PER_REG; j += 1)
        {
          currentindex = i * BQ769X0_BAL_BITS_PER_REG + j;
          if (currentindex >= bal->balance_count)
            {
              break;
            }

          currentbit = bal->balance[currentindex];
          if (currentbit && lastbit)
            {
              bal->balance[currentindex] = false;
              batinfo("Skipping cell %d because balance is set and previous "
                      "cell balance is set\n", currentindex);
            }
          else
            {
              balancebits |= (currentbit ? 1 : 0) <<
                             priv->mapping[currentindex];
              batinfo("Setting cell balance %d to %d\n", currentindex,
                      currentbit);
              batinfo("Balance bits are %02x\n", balancebits);
            }

          lastbit = currentbit;
        }
    }

  /* Now split the result into 3 groups of 5 and send */

  for (i = 0; i < BQ769X0_BAL_REG_COUNT; i += 1)
    {
      regval = (balancebits >> (i * 5)) & BQ769X0_CELLBAL_MASK;
      ret = bq769x0_putreg8(priv, BQ769X0_REG_CELLBAL1 + i, regval);
      if (ret < 0)
        {
          baterr("ERROR: Error writing to BQ769X0! Error = %d\n", ret);
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
                               struct battery_monitor_voltage_s *cellv)
{
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

static int bq769x0_current(struct battery_monitor_dev_s *dev,
    struct battery_monitor_current_s *current)
{
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  int ret;

  /* Get current from battery monitor */

  ret = bq769x0_getcurrent(priv, current);
  if (ret < 0)
    {
      baterr("ERROR: Error getting current from BQ769X0! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_soc
 *
 * Description:
 *   Get the pack state of charge (in percent)
 *
 ****************************************************************************/

static int bq769x0_soc(struct battery_monitor_dev_s *dev, b16_t *value)
{
  /* The BQ769X0 does not support directly reporting pack state of charge.
   * You should be able to come up with a state-of-charge value by knowing an
   * initial value and looking at the Coulomb counter.  This is out of scope
   * for this driver, though.
   */

  return -ENOSYS;
}

/****************************************************************************
 * Name: bq769x0_coulombs
 *
 * Description:
 *   Get the raw value of the coulomb counter
 *
 ****************************************************************************/

static int bq769x0_coulombs(struct battery_monitor_dev_s *dev, int *coulombs)
{
  /* The data from the coulomb counter on this part can be accessed via
   * the "get current" command.
   */

  return -ENOSYS;
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
      baterr("ERROR: Error getting temperature from BQ769X0! Error = %d\n",
             ret);
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
                           struct battery_monitor_balance_s *bal)
{
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  int ret;

  ret =  bq769x0_setbalance(priv, bal);
  if (ret < 0)
    {
      baterr("ERROR: Error getting temperature from BQ769X0! Error = %d\n",
             ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_shutdown
 *
 * Description:
 *   Put the battery in a low-power state
 *
 ****************************************************************************/

static int bq769x0_shutdown(struct battery_monitor_dev_s *dev,
                            uintptr_t param)
{
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  int ret;

  ret =  bq769x0_doshutdown(priv);
  if (ret < 0)
    {
      baterr("ERROR: putting BQ769X0 into low-power state! Error = %d\n",
             ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_limits
 *
 * Description:
 *   Set the built-in safety limit values for the battery
 *
 ****************************************************************************/

static int bq769x0_limits(struct battery_monitor_dev_s *dev,
                          struct battery_monitor_limits_s *limits)
{
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  int ret;

  ret =  bq769x0_setlimits(priv, limits);
  if (ret < 0)
    {
      baterr("ERROR: Error updating BQ769X0 safety limits! Error = %d\n",
             ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_chgdsg
 *
 * Description:
 *   Set the battery charge/discharge switches in order to
 *   accept/provide current
 *
 ****************************************************************************/

static int bq769x0_chgdsg(struct battery_monitor_dev_s *dev,
                          struct battery_monitor_switches_s *sw)
{
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  int ret;

  ret =  bq769x0_setchgdsg(priv, sw);
  if (ret < 0)
    {
      baterr("ERROR: Error setting BQ769X0 switches! Error = %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: bq769x0_clearfaults
 *
 * Description:
 *   Clear the battery monitor faults one at a time in order of priority
 *   Uses the most recent fault register read in order to avoid
 *   race conditions.
 *
 ****************************************************************************/

static int bq769x0_clearfaults(struct battery_monitor_dev_s *dev,
                               uintptr_t param)
{
  FAR struct bq769x0_dev_s *priv = (FAR struct bq769x0_dev_s *)dev;
  int ret;
  uint8_t faults = priv->fault_cache;
  uint8_t to_clear = 0;

  if (faults & BQ769X0_DEVICE_XREADY)
    {
      to_clear = BQ769X0_DEVICE_XREADY;
    }
  else if (faults & BQ769X0_SCD)
    {
      to_clear = BQ769X0_SCD;
    }
  else if (faults & BQ769X0_OCD)
    {
      to_clear = BQ769X0_OCD;
    }
  else if (faults & BQ769X0_OV)
    {
      to_clear = BQ769X0_OV;
    }
  else if (faults & BQ769X0_UV)
    {
      to_clear = BQ769X0_UV;
    }
  else if (faults & BQ769X0_OVRD_ALERT)
    {
      to_clear = BQ769X0_OVRD_ALERT;
    }

  if (to_clear)
    {
      ret =  bq769x0_clear_chipfaults(priv, to_clear);
      if (ret < 0)
        {
          baterr("ERROR: Error clearing faults! Error = %d\n", ret);
          return ret;
        }
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

static int bq769x0_operate(struct battery_monitor_dev_s *dev,
                           uintptr_t param)
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
 *   CONFIG_I2C_BQ769X0 - And the driver must be explicitly selected.
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
 *   sense_r   - The value of the current sense resistor, in micro ohms.
 *               This value is used to calculate reported current, and when
 *               setting overcurrent thresholds.
 *
 * Returned Value:
 *   A pointer to the initialized lower-half driver instance.  A NULL pointer
 *   is returned on a failure to initialize the BQ769x0 lower half.
 *
 ****************************************************************************/

FAR struct battery_monitor_dev_s *
  bq769x0_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                     uint32_t frequency, bool crc, uint8_t cellcount,
                     uint8_t chip, uint32_t sense_r)
{
  FAR struct bq769x0_dev_s *priv;
  int ret;
  uint8_t regval;

  /* Initialize the BQ769x0 device structure */

  priv = kmm_zalloc(sizeof(struct bq769x0_dev_s));
  if (priv)
    {
      /* Initialize the BQ769x0 device structure */

      priv->dev.ops     = &g_bq769x0ops;
      priv->i2c         = i2c;
      priv->addr        = addr;
      priv->frequency   = frequency;
      priv->crc         = crc;
      priv->chip        = chip;
      priv->cellcount   = cellcount;
      priv->sense_r     = sense_r;
      priv->fault_cache = 0;

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
            }
          else
            {
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
            }
          else
            {
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
            }
          else
            {
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

      /* Configure the BQ769x0
       * Set default CC_CFG register (required per datasheet)
       */

      ret = bq769x0_putreg8(priv, BQ769X0_REG_CC_CFG,
                            BQ769X0_CC_CFG_DEFAULT_VAL);
      if (ret < 0)
        {
          baterr("ERROR: Failed to configure the BQ769x0: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Set up DELAY_DIS, CC_EN, and CC_ONESHOT bits,
       * making sure not to modify existing DSG_ON/CHG_ON state
       */

      ret = bq769x0_getreg8(priv, BQ769X0_REG_SYS_CTRL2, &regval);
      if (ret < 0)
        {
          baterr("ERROR: Failed to configure the BQ769x0: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Keep the existing DSG_ON/CHG_ON bits and set CC_EN */

      regval &= BQ769X0_SYS_CTRL2_CHGDSG_MASK;
      regval |= BQ769X0_CC_EN;

      ret = bq769x0_putreg8(priv, BQ769X0_REG_SYS_CTRL2, regval);
      if (ret < 0)
        {
          baterr("ERROR: Failed to configure the BQ769x0: %d\n", ret);
          kmm_free(priv);
          return NULL;
        }

      /* Set ADC_EN and TEMP_SEL bit */

      regval = BQ769X0_ADC_EN;
#ifndef CONFIG_BQ769X0_USE_INTERNAL_TS
      regval |= BQ769X0_TEMP_SEL;
#endif
      ret = bq769x0_putreg8(priv, BQ769X0_REG_SYS_CTRL1, regval);
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
          baterr("ERROR: Failed to get gain/offset values from the BQ769x0: "
                 "%d\n", ret);
          kmm_free(priv);
          return NULL;
        }
    }

  return (FAR struct battery_monitor_dev_s *)priv;
}

#endif /* CONFIG_BATTERY_MONITOR && CONFIG_I2C && CONFIG_I2C_BQ769X0 */
