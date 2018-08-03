/****************************************************************************
 * drivers/timers/cs2100-cp.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/timers/cs2100-cp.h>

#ifdef CONFIG_TIMERS_CS2100CP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Driver Definitions *******************************************************/

#define MAX_REFCLK_FREQ 75000000
#define MAX_REFCLK_XTAL 50000000

#define MAX_SYSCLK      18750000

#define MAX_SKIP_FREQ   80000000

/* Debug ********************************************************************/

#undef cserr
#ifdef CONFIG_CS2100CP_DEBUG
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define cserr(format, ...)    _err(format, ##__VA_ARGS__)
#  else
#    define cserr                 _err
#  endif
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define cserr(x...)
#  else
#    define cserr                (void)
#  endif
#endif

#undef reginfo
#ifdef CONFIG_CS2100CP_REGDEBUG
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define reginfo(format, ...)  _err(format, ##__VA_ARGS__)
#  else
#    define reginfo               _err
#  endif
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define reginfo(x...)
#  else
#    define reginfo              (void)
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  cs2100_write_reg
 *
 * Description:
 *   Write an 8 bit value to a CS2100 8-bit register.
 *
 * Input Parameters:
 *   config  - CS2100-CP configuration
 *   regaddr - CS2100 register address
 *   regval  - CS2100 register value to write
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cs2100_write_reg(FAR const struct cs2100_config_s *config,
                            uint8_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msgs[2];
  int ret;

  reginfo("%02x<-%02x\n", regaddr, regval);
  DEBUGASSERT(config->i2c->ops && config->i2c->ops->transfer);

  /* Construct the I2C message (write N+1 bytes with no restart) */

  msga[0].frequency = config->i2cfreq;
  msgs[0].addr      = config->i2caddr;
  msgs[0].flags     = 0;
  msgs[0].buffer    = &regaddr;
  msgs[0].length    = 1;

  msga[1].frequency = config->i2cfreq;
  msgs[1].addr      = config->i2caddr;
  msgs[1].flags     = I2C_M_NOSTART;
  msgs[1].buffer    = &regval;
  msgs[1].length    = 1;

  /* Send the message */

  ret = I2C_TRANSFER(config->i2c, msgs, 2);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: cs2100_read_reg
 *
 * Description:
 *   Read an 8 bit value from a CS2100 8-bit register.
 *
 * Input Parameters:
 *   config  - CS2100-CP configuration
 *   regaddr - CS2100 register address
 *   regval  - Location to return the CS2100 register value
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_CS2100CP_DEBUG
static int cs2100_read_reg(FAR const struct cs2100_config_s *config,
                           uint8_t regaddr, uint8_t *regval)
{
  struct i2c_msg_s msg;
  int ret;

  DEBUGASSERT(config->i2c->ops && config->i2c->ops->transfer);

  /* Construct the I2C message (write 1 bytes, restart, read N bytes) */

  msg.frequency = config->i2cfreq;
  msg.addr      = config->i2caddr;
  msg.flags     = 0;
  msg.buffer    = &regaddr;
  msg.length    = 1;

  /* Send the address followed by a STOP */

  ret = I2C_TRANSFER(config->i2c, &msg, 1);
  if (ret >= 0)
    {
      msg.frequency = config->i2cfreq;
      msg.addr      = config->i2caddr;
      msg.flags     = I2C_M_READ;
      msg.buffer    = regval;
      msg.length    = 1;

      /* Read the register beginning with another START */

      ret = I2C_TRANSFER(config->i2c, &msg, 1);
      if (ret >= 0)
        {
          reginfo("%02x->%02x\n", regaddr, *regval);
        }
    }

  return (ret >= 0) ? OK : ret;
}
#endif

/****************************************************************************
 * Name:  cs2100_write_reg
 *
 * Description:
 *   Write the 32-bit ratio value to CS2100 Ratio registers.
 *
 * Input Parameters:
 *   config  - CS2100-CP configuration
 *   ratio   - CS2100 ratio value to write
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cs2100_write_ratio(FAR const struct cs2100_config_s *config,
                              uint32_t ratio)
{
  struct i2c_msg_s msg;
  uint8_t buffer[5];
  int ret;

  reginfo("%02x<-%04l\n", CS2100_RATIO0, (unsigned long)ratio);
  DEBUGASSERT(config->i2c->ops && config->i2c->ops->transfer);

  /* Construct the I2C message (write N+1 bytes with no restart) */

  buffer[0]     = CS2100_RATIO0;
  buffer[1]     = (uint8_t)(ratio >> 24);
  buffer[2]     = (uint8_t)((ratio >> 16) & 0xff);
  buffer[3]     = (uint8_t)((ratio >> 8) & 0xff);
  buffer[4]     = (uint8_t)(ratio  & 0xff);

  msg.frequency = config->i2cfreq;
  msg.addr      = config->i2caddr;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 5;

  /* Send the message */

  ret = I2C_TRANSFER(config->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name:  cs2100_read_ratio
 *
 * Description:
 *   Read the 32-bit ratio value from the CS2100 Ratio registers.
 *
 * Input Parameters:
 *   config  - CS2100-CP configuration
 *   ratio   - Location to return the CS2100 ratio
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_CS2100CP_DEBUG
static int cs2100_read_ratio(FAR const struct cs2100_config_s *config,
                             uint32_t *ratio)
{
  struct i2c_msg_s msg;
  uint8_t buffer[4];
  int ret;

  DEBUGASSERT(config->i2c->ops && config->i2c->ops->transfer);

  /* Construct the I2C message (write N+1 bytes with no restart) */

  buffer[0]     = CS2100_RATIO0;

  msg.frequency = config->i2cfreq;
  msg.addr      = config->i2caddr;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 1;

  /* Send the address followed by a STOP */

  ret = I2C_TRANSFER(config->i2c, &msg, 1);
  if (ret >= 0)
    {
      msg.frequency = config->i2cfreq;
      msg.addr      = config->i2caddr;
      msg.flags     = I2C_M_READ;
      msg.buffer    = buffer;
      msg.length    = 4;

      /* Read the ratio registers beginning with another START */

      ret = I2C_TRANSFER(config->i2c, &msg, 1);

      /* Return the ratio */

      if (ret >= 0)
        {
           *ratio = ((uint32_t)buffer[0] << 24) |
                    ((uint32_t)buffer[1] << 16) |
                    ((uint32_t)buffer[2] << 8) |
                     (uint32_t)buffer[0];

           reginfo("%02x->%04l\n", CS2100_RATIO0, (unsigned long)*ratio);
        }
    }

  return (ret >= 0) ? OK : ret;
}
#endif

/****************************************************************************
 * Name:  cs2100_refclk
 *
 * Description:
 *   Set the reference clock divider value.
 *
 * Input Parameters:
 *   config  - CS2100-CP configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cs2100_refclk(FAR const struct cs2100_config_s *config)
{
  uint8_t regval;
  int ret;

  DEBUGASSERT((config->xtal && config->refclk <= MAX_REFCLK_XTAL) ||
              (!config->xtal && config->refclk <= MAX_REFCLK_FREQ));

  /* Calculate and set the RefClk the divider */

  if (config->refclk <= MAX_SYSCLK)
    {
      regval  = CS2100_FNCCFG1_REFCLKDIV_NONE;
    }
  else if (config->refclk <= (MAX_SYSCLK / 2))
    {
      regval = CS2100_FNCCFG1_REFCLKDIV_DIV2;
    }
  else if (config->refclk <= (MAX_SYSCLK / 4))
    {
      regval = CS2100_FNCCFG1_REFCLKDIV_DIV4;
    }
  else
    {
      cserr("ERROR: reflck too large: %ul\n", (unsigned long)config->refclk);
      return -EINVAL;
    }

  /* Enable CLK_IN skipping mode? */

  if (config->refclk <= MAX_SKIP_FREQ)
    {
      regval |= CS2100_FNCCFG1_CLKSKIPEN;
    }

  ret = cs2100_write_reg(config, CS2100_FNCCFG1, regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to set CS2100_FNCCFG1: %d\n", ret);
      return ret;
    }

  /* Set the minimum loop bandwidth */

  DEBUGASSERT(config->loopbw >= 1 && config->loopbw <= 128);

  if (config->loopbw < 2)
    {
      regval = CS2100_FNCCFG3_CLKINBW_1HZ;
    }
  else if (config->loopbw < 3)
    {
      regval = CS2100_FNCCFG3_CLKINBW_2HZ;
    }
  else if (config->loopbw < 6)
    {
      regval = CS2100_FNCCFG3_CLKINBW_4HZ;
    }
  else if (config->loopbw < 12)
    {
      regval = CS2100_FNCCFG3_CLKINBW_8HZ;
    }
  else if (config->loopbw < 24)
    {
      regval = CS2100_FNCCFG3_CLKINBW_16HZ;
    }
  else if (config->loopbw < 48)
    {
      regval = CS2100_FNCCFG3_CLKINBW_32HZ;
    }
  else if (config->loopbw < 96)
    {
      regval = CS2100_FNCCFG3_CLKINBW_64HZ;
    }
  else /* if (config->loopbw <= 128) */
    {
      regval = CS2100_FNCCFG3_CLKINBW_128HZ;
    }

  ret = cs2100_write_reg(config, CS2100_FNCCFG3, regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to set CS2100_FNCCFG3: %d\n", ret);
      return ret;
    }

  /* Configure so that CLK_OUT will be enabled when the registers are
   * unlocked (also clears other settings).
   * NOTE: This implicitly sets High Multiplier mode for the Rud.
   */

  ret = cs2100_write_reg(config, CS2100_FNCCFG2, CS2100_FNCCFG2_CLKOUTUNL);
  if (ret < 0)
    {
      cserr("ERROR: Failed to set CS2100_FNCCFG2: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name:  cs2100_ratio
 *
 * Description:
 *   Calculate the effective input-to-output ratio
 *
 * Input Parameters:
 *   config  - CS2100-CP configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cs2100_ratio(FAR const struct cs2100_config_s *config)
{
  uint64_t rudb24;
  uint32_t rud;
  uint8_t regval;
  bool highmul;
  int rmod;
  int ret;

  DEBUGASSERT(config->clkin > 0 && config->clkout > 0);

  /* Calculate a 64-bit RUD value:
   *
   *  R-Mod * clkout / clkin
   *
   * Initial calculation has 24-bits of accuracy (b24)
   */

  rudb24 = ((uint64_t)config->clkout << 24) / config->clkin;

  /* If the b23 rudb24 is less than (1 << 39), then it can be represented as
   * a high-precision (b20) value.
   */

  if (rudb24 < (1ull << (32+7)))
    {
      highmul = false;

      /* Brute force! */

      if (rudb24 >= (1ull << (32+6)))
        {
          rud  = (uint32_t)rudb24 >> 7;  /* RUD = RUDb20 / 8 */
          rmod = 3;                      /* Reff = 8 * RUD */
        }
      else if (rudb24 >= (1ull << (32+5)))
        {
          rud  = (uint32_t)rudb24 >> 6;  /* RUD = RUDb20 / 4 */
          rmod = 3;                      /* Reff = 4 * RUD */
        }
      else if (rudb24 >= (1ull << (32+4)))
        {
          rud  = (uint32_t)rudb24 >> 5;  /* RUD = RUDb20 / 2 */
          rmod = 1;                      /* Reff = 2 * RUD */
        }
      else if (rudb24 >= (1ull << (32+3)))
        {
          rud  = (uint32_t)rudb24 >> 4;  /* RUD = RUDb20 */
          rmod = 0;                      /* Reff = RUD */
        }
      else if (rudb24 >= (1ull << (32+2)))
        {
          rud  = (uint32_t)rudb24 >> 3;  /* RUD -> 2*RUDb20 */
          rmod = 4;                      /* Reff = RUD / 2 */
        }
      else if (rudb24 >= (1ull << (32+1)))
        {
          rud  = (uint32_t)rudb24 >> 2;  /* RUD -> 4*RUDb20 */
          rmod = 5;                      /* Reff = RUD / 4 */
        }
      else if (rudb24 >= (1ull << 32))
        {
          rud  = (uint32_t)rudb24 >> 1;  /* RUD -> 8*RUDb20 */
          rmod = 6;                      /* Reff = RUD / 8 */
        }
      else
        {
          rud  = (uint32_t)rudb24;       /* RUD -> 16*RUDb20 */
          rmod = 7;                      /* Reff = RUD / 16 */
        }
    }

  /* If the b23 rudb24 is less than (1 << 47), then it can be represented as
   * a high-multiplication (b12) value.
   */

  else if (rudb24 < (1ull << (32+12)))
    {
      highmul = true;

      if (rudb24 >= (1ull << (32+11)))
        {
          rud  = (uint32_t)rudb24 >> 12;  /* RUD  = RUDb12 */
          rmod = 0;                       /* Reff = RUD */
        }
      else if (rudb24 >= (1ull << (32+10)))
        {
          rud  = (uint32_t)rudb24 >> 11;  /* RUD  = 2*RUDb12 */
          rmod = 4;                       /* Reff = RUD / 2 */
        }
      else if (rudb24 >= (1ull << (32+9)))
        {
          rud  = (uint32_t)rudb24 >> 10;  /* RUD  = 4*RUDb12 */
          rmod = 5;                       /* Reff = RUD / 4 */
        }
      else if (rudb24 >= (1ull << (32+8)))
        {
          rud  = (uint32_t)rudb24 >> 9;  /* RUD  = 8*RUDb12 */
          rmod = 6;                       /* Reff = RUD / 8 */
        }
      else /* if (rudb24 >= (1ull << (32+7))) */
        {
          rud  = (uint32_t)rudb24 >> 8;  /* RUD  = 16*RUDb12 */
          rmod = 7;                       /* Reff = RUD / 16 */
        }
    }
  else
    {
      cserr("ERROR: Ratio too large: %08llx\n", rudb24);
      return -E2BIG;
    }

  /* Save the ratio */

  ret = cs2100_write_ratio(config, rud);
  if (ret < 0)
    {
      cserr("ERROR: Failed to set ratio: %d\n", ret);
      return ret;
    }

  /* Save the R-Mod value and EnDevCfg1.  The device won't be fully enabled
   * until EnDevCfg2 is setand registers are unfrozen and unlocked.
   * REVISIT: Also sets AuxOutSrc to RefClk.
   */

  regval = (rmod << CS2100_DEVCFG1_RMODSEL_SHIFT) | CS2100_DEVCFG1_ENDEVCFG1;
  ret    = cs2100_write_reg(config, CS2100_DEVCFG1, regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to set CS2100_DEVCFG1: %d\n", ret);
      return ret;
    }

  /* Set High Resolution mode if needed.  NOTE: this depends on the fact
   * that High Multiplier mode was previously selected.
   */

  if (!highmul)
    {
      /* Preserve the ClkOutUnl bit */

      regval = CS2100_FNCCFG2_CLKOUTUNL | CS2100_FNCCFG2_LFRATIOCFG;
      ret    = cs2100_write_reg(config, CS2100_FNCCFG2, regval);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs2100_enable
 *
 * Description:
 *   Enable CS2100 CLK_OUT using the provide parameters
 *
 * Input Parameters:
 *   config  - CS2100-CP configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cs2100_enable(FAR const struct cs2100_config_s *config)
{
  uint8_t regval;
  int ret;

  DEBUGASSERT(config && config->i2c);

  /* Lock the CS2100 and disable CLK_OUT and AUX_OUT.  Subsequent settings
   * will not take effect until the registers are unlocked.
   */

  regval = CS2100_DEVCTL_AUXOUTDIS | CS2100_DEVCTL_CLKOUTDIS;
  ret = cs2100_write_reg(config, CS2100_DEVCTL, regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to set CS2100_DEVCTL: %d\n", ret);
      return ret;
    }

  /* Set the internal timing reference clock divider */

  ret = cs2100_refclk(config);
  if (ret < 0)
    {
      cserr("ERROR: cs2100_refclk failed: %d\n", ret);
      return ret;
    }

  /* Freeze device control registers.  This allows modifications to r0-r4
   * but the modifications will not take effect until the registers are
   * unfrozen.
   */

  ret = cs2100_write_reg(config, CS2100_GBLCFG, CS2100_GBLCFG_FREEZE);
  if (ret < 0)
    {
      cserr("ERROR: Failed to set CS2100_GBLCFG: %d\n", ret);
      return ret;
    }

  /* Calculate the effective ratio */

  ret = cs2100_ratio(config);
  if (ret < 0)
    {
      cserr("ERROR: cs2100_ratio failed: %d\n", ret);
      return ret;
    }

  /* Unfreeze the r0-r4 and set EnDevCfg2 */

  ret = cs2100_write_reg(config, CS2100_GBLCFG, CS2100_GBLCFG_ENDEVCFG2);
  if (ret < 0)
    {
      cserr("ERROR: Failed to set CS2100_GBLCFG: %d\n", ret);
      return ret;
    }

  /* Unlock and enable the CS2100 and CLK_OUT */

  regval = CS2100_DEVCTL_UNLOCK | CS2100_DEVCTL_AUXOUTDIS;
  ret = cs2100_write_reg(config, CS2100_DEVCTL, regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to set CS2100_DEVCTL: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: cs2100_disable
 *
 * Description:
 *   Disable CS2100 CLK_OUT
 *
 * Input Parameters:
 *   config  - CS2100-CP configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int cs2100_disable(FAR const struct cs2100_config_s *config)
{
  uint8_t regval;
  int ret;

  /* Unlock and disable AUX_OUT and CLK_OUT */

  regval = CS2100_DEVCTL_UNLOCK | CS2100_DEVCTL_AUXOUTDIS |
           CS2100_DEVCTL_CLKOUTDIS;
  ret = cs2100_write_reg(config, CS2100_DEVCTL, regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to set CS2100_DEVCTL: %d\n", ret);
      return ret;
    }

  /* Clear EndDevCfg2 and unfreeze R0-R4 */

  ret = cs2100_write_reg(config, CS2100_GBLCFG, 0);
  if (ret < 0)
    {
      cserr("ERROR: Failed to set CS2100_GBLCFG: %d\n", ret);
      return ret;
    }

  /* Clear EndDevCfg1 */

  ret = cs2100_write_reg(config, CS2100_DEVCFG1, 0);
  if (ret < 0)
    {
      cserr("ERROR: Failed to set CS2100_DEVCFG1: %d\n", ret);
      return ret;
    }

  /* Lock the CS2100 */

  regval = CS2100_DEVCTL_AUXOUTDIS | CS2100_DEVCTL_CLKOUTDIS;
  ret = cs2100_write_reg(config, CS2100_DEVCTL, regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to set CS2100_DEVCTL: %d\n", ret);
    }

  return ret;
}

/********************************************************************************************
 * Name: cs2100_dump
 *
 * Description:
 *   Dump CS2100-CP registers to the SysLog
 *
 * Input Parameters:
 *   config  - CS2100-CP configuration (Needed only for I2C access: i2c and i2caddr)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ********************************************************************************************/

#ifdef CONFIG_CS2100CP_DEBUG
int cs2100_dump(FAR const struct cs2100_config_s *config)
{
  uint32_t ratio;
  uint8_t regval;
  int ret;

  csinfo("CS200-CP Registers:\n");

  ret = cs2100_read_reg(config, CS2100_DEVID, &regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to read CS2100_DEVID: %d\n", ret);
      return ret;
    }

  csinfo("     Devid: %02x\n", regval);

  ret = cs2100_read_reg(config, CS2100_DEVCTL, &regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to read CS2100_DEVCTL: %d\n", ret);
      return ret;
    }

  csinfo("    DevCtl: %02x\n", regval);

  ret = cs2100_read_reg(config, CS2100_DEVCFG1, &regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to read CS2100_DEVCFG1: %d\n", ret);
      return ret;
    }

  csinfo("   DevCfg1: %02x\n", regval);

  ret = cs2100_read_reg(config, CS2100_GBLCFG, &regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to read CS2100_GBLCFG: %d\n", ret);
      return ret;
    }

  csinfo("    GblCfg: %02x\n", regval);

  ret = cs2100_read_ratio(config, &ratio);
  if (ret < 0)
    {
      cserr("ERROR: cs2100_read_ratio failed: %d\n", ret);
      return ret;
    }

  csinfo("     Ratio: %04lx\n", (unsigned long)ratio);

  ret = cs2100_read_reg(config, CS2100_FNCCFG1, &regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to read CS2100_FNCCFG1: %d\n", ret);
      return ret;
    }

  csinfo("  FuncCfg1: %02x\n", regval);

  ret = cs2100_read_reg(config, CS2100_FNCCFG2, &regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to read CS2100_FNCCFG2: %d\n", ret);
      return ret;
    }

  csinfo("  FuncCfg2: %02x\n", regval);

  ret = cs2100_read_reg(config, CS2100_FNCCFG3, &regval);
  if (ret < 0)
    {
      cserr("ERROR: Failed to read CS2100_FNCCFG3: %d\n", ret);
      return ret;
    }

  csinfo("  FuncCfg3: %02x\n", regval);
  return OK;
}

#endif /* CONFIG_CS2100CP_DEBUG */
#endif /* CONFIG_TIMERS_CS2100CP */
