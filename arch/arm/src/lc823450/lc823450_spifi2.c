/****************************************************************************
 * arch/arm/src/lc823450/lc823450_spifi2.c
 *
 *   Copyright 2014,2015,2016,2017,2018 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <errno.h>

#include "chip.h"
#include "up_arch.h"

#include "lc823450_syscontrol.h"
#include "lc823450_spifi2.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wait_txfifo_empty
 ****************************************************************************/

static int wait_txfifo_empty(void)
{
  int t;
  t = 1000;

  while ((getreg32(SF_FIFO_NUM) & SF_FIFO_NUM_T_MASK) && t)
    {
      up_udelay(1);
      t--;
    }

  return t == 0 ? -ETIMEDOUT : 0;
}

/****************************************************************************
 * Name: wait_rxfifo_notempty
 ****************************************************************************/

static int wait_rxfifo_notempty(void)
{
  int t;
  t = 1000;

  while (!(getreg32(SF_FIFO_NUM) & SF_FIFO_NUM_R_MASK) && t)
    {
      up_udelay(1);
      t--;
    }

  return t == 0 ? -ETIMEDOUT : 0;
}

/****************************************************************************
 * Name: spiflash_cmd_only
 ****************************************************************************/

static int spiflash_cmd_only(int cmd)
{
  /* COMMAND(1byte) = 1 */

  putreg32(SF_SIZE_NOREAD | 1 << SF_SIZE_T_SHIFT, SF_SIZE);
  putreg32(0, SF_DUMMY);
  putreg8(cmd, SF_T_FIFO);

  /* Start Transfer */

  putreg32(SF_CTL_ACT, SF_CTL);
  return wait_txfifo_empty();
}

/****************************************************************************
 * Name: spiflash_read_jdid
 ****************************************************************************/

static int spiflash_read_jdid(void)
{
  int ret = 0;

  /* COMMAND(1byte) + jid(3byte) */

  putreg32(1 << SF_SIZE_UL_SHIFT | 4 << SF_SIZE_T_SHIFT, SF_SIZE);
  putreg32(SF_DUMMY_DUMMY, SF_DUMMY);

  /* COMMAND */

  putreg8(SF_CMD_READ_JID, SF_T_FIFO);

  /* Start Transfer */

  putreg32(SF_CTL_ACT, SF_CTL);

  /* MID */

  wait_rxfifo_notempty();
  ret |= getreg8(SF_R_FIFO);

  /* MType */

  wait_rxfifo_notempty();
  ret <<= 8;
  ret |= getreg8(SF_R_FIFO);

  /* CapID */

  wait_rxfifo_notempty();

  ret <<= 8;
  ret |= getreg8(SF_R_FIFO);

  return ret;
}

/****************************************************************************
 * Name: spiflash_quad_enable_winbond
 ****************************************************************************/

static void spiflash_quad_enable_winbond(void)
{
  /* SR Write Enable */

  if (spiflash_cmd_only(SF_CMD_WRITE_EN))
    {
      return;
    }

  /* Quad Enable */

  putreg32(SF_SIZE_NOREAD | 3 << SF_SIZE_T_SHIFT, SF_SIZE);
  putreg32(0, SF_DUMMY);
  putreg8(SF_CMD_WRITE_STATUS, SF_T_FIFO);
  putreg8(0, SF_T_FIFO);      /* status1 */
  putreg8(SF_STATUS2_QE, SF_T_FIFO);  /* status2 */

  /* Start Transfer */

  putreg32(SF_CTL_ACT, SF_CTL);

  wait_txfifo_empty();
  return;
}

/****************************************************************************
 * Name: spiflash_read_quad_enable_winbond
 ****************************************************************************/

static int spiflash_read_quad_enable_winbond(void)
{
  /* COMMAND(1byte) + status(1byte) */

  putreg32(1 << SF_SIZE_UL_SHIFT | 2 << SF_SIZE_T_SHIFT, SF_SIZE);
  putreg32(SF_DUMMY_DUMMY, SF_DUMMY);

  /* COMMAND */

  putreg8(SF_CMD_READ_STATUS2, SF_T_FIFO);

  /* Start Transfer */

  putreg32(SF_CTL_ACT, SF_CTL);

  wait_rxfifo_notempty();

  return getreg8(SF_R_FIFO) & SF_STATUS2_QE;
}

/****************************************************************************
 * Name: spiflash_quad_enable_macronix
 ****************************************************************************/

static void spiflash_quad_enable_macronix(void)
{
  /* SR Write Enable */

  if (spiflash_cmd_only(SF_CMD_WRITE_EN))
    {
      return;
    }

  /* Quad Enable */

  putreg32(SF_SIZE_NOREAD | 2 << SF_SIZE_T_SHIFT, SF_SIZE);
  putreg32(0, SF_DUMMY);
  putreg8(SF_CMD_WRITE_STATUS, SF_T_FIFO);
  putreg8(1 << 6, SF_T_FIFO); /* status1 */

  /* Start Transfer */

  putreg32(SF_CTL_ACT, SF_CTL);

  wait_txfifo_empty();
  return;
}

/****************************************************************************
 * Name: spiflash_read_quad_enable_macronix
 ****************************************************************************/

static int spiflash_read_quad_enable_macronix(void)
{
  /* COMMAND(1byte) + status(1byte) */

  putreg32(1 << SF_SIZE_UL_SHIFT | 2 << SF_SIZE_T_SHIFT, SF_SIZE);
  putreg32(SF_DUMMY_DUMMY, SF_DUMMY);

  /* COMMAND */

  putreg8(SF_CMD_READ_STATUS1, SF_T_FIFO);

  /* Start Transfer */

  putreg32(SF_CTL_ACT, SF_CTL);

  wait_rxfifo_notempty();

  return getreg8(SF_R_FIFO) & (1 << 6);
}

/****************************************************************************
 * Name: spiflash_quad_enable
 ****************************************************************************/

static void spiflash_quad_enable(int jdid)
{
  if ((jdid & SF_JID_MID_MASK) == SF_JID_MID_MACRONIX)
    {
      spiflash_quad_enable_macronix();
    }
  else
    {
      spiflash_quad_enable_winbond();
    }
}

/****************************************************************************
 * Name: spiflash_read_quad_enable
 ****************************************************************************/

static int spiflash_read_quad_enable(int jdid)
{
  if ((jdid & SF_JID_MID_MASK) == SF_JID_MID_MACRONIX)
    {
      return spiflash_read_quad_enable_macronix();
    }

  return spiflash_read_quad_enable_winbond();
}

/****************************************************************************
 * Name: spiflash_write_busy
 ****************************************************************************/

static int spiflash_write_busy(void)
{
  /* COMMAND(1byte) + status(1byte) */

  putreg32(1 << SF_SIZE_UL_SHIFT | 2 << SF_SIZE_T_SHIFT, SF_SIZE);
  putreg32(SF_DUMMY_DUMMY, SF_DUMMY);

  /* COMMAND */

  putreg8(SF_CMD_READ_STATUS1, SF_T_FIFO);

  /* Start Transfer */

  putreg32(SF_CTL_ACT, SF_CTL);

  wait_rxfifo_notempty();

  return getreg8(SF_R_FIFO) & SF_STATUS1_BUSY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_spiflash_earlyinit
 ****************************************************************************/

void lc823450_spiflash_earlyinit(void)
{
#ifdef CONFIG_LC823450_SPIFI_QUADIO
  int jdid;
#endif

  /* S-Flash controller: Enable clock & unreset */

  modifyreg32(MCLKCNTBASIC, 0, MCLKCNTBASIC_SFIF_CLKEN);
  modifyreg32(MRSTCNTBASIC, 0, MRSTCNTBASIC_SFIF_RSTB);

  /* S-Flash cache: Enable clock & unreset */

  modifyreg32(MCLKCNTBASIC, 0, MCLKCNTBASIC_CACHE_CLKEN);
  modifyreg32(MRSTCNTBASIC, 0, MRSTCNTBASIC_CACHE_RSTB);

  /* Set S-Flash I/O voltage to 3.3v */

  putreg32(0x1, SFIFSEL);

#ifdef CONFIG_LC823450_SPIFI_QUADIO
  jdid = spiflash_read_jdid();

  /* Check if QUAD mode supported */

  if (!spiflash_read_quad_enable(jdid))
    {
      spiflash_quad_enable(jdid);
      while (spiflash_write_busy())
        {
          up_udelay(10);
        }
    }

  /* bus accelerator enable : mode 3(use FAST_READ_QUAD_IO) */

  putreg32(SF_BUS_BUSEN | 3 << SF_BUS_BUSMODE_SHIFT | SF_BUS_LOOKAHEAD,
           SF_BUS);
#else
  /* bus accelerator enable : mode 1(use FAST_READ) */

  putreg32(SF_BUS_BUSEN | 1 << SF_BUS_BUSMODE_SHIFT | SF_BUS_LOOKAHEAD,
           SF_BUS);
#endif /* CONFIG_LC823450_SPIFI_QUADIO */

  /* S-Flash cache: Flash and Re-enable cache */

  modifyreg32(CACHE_CTL, CACHE_CTL_USE, 0);
  modifyreg32(CACHE_CTL, 0, CACHE_CTL_USE);
}
