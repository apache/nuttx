/****************************************************************************
 * arch/arm/src/ameba/ameba_efuse.c
 *
 *   Copyright (C) 2021 Xiaomi InC. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in
 *  the documentation and/or other materials provided with the
 *  distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *  used to endorse or promote products derived from this software
 *  without specific prior written permission.
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
#include "hal_api.h"
#include "hal_efuse.h"
#include "hal_efuse_nsc.h"
#include "platform_conf.h"
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#if CONFIG_EFUSE_EN

//#define EFUSE_LOGICAL_SIM

#define EFUSE_LOGICAL_MAP_SIZE 512
#define EFUSE_LOGICAL_MAP_HW_SIZE 0xD0 //sync with EFUSE_OOB_PROTECT_BYTES
#define EFUSE_LOGICAL_SBLOCK_OFFSET 0x19

#ifdef EFUSE_LOGICAL_SIM
uint8_t ameba_efuse_sim_map[256];
#endif

int ameba_efuse_logical_read(uint16_t laddr, uint16_t size, uint8_t *pbuf)
{
  uint8_t offset, wden, header, extheader, data;
  uint16_t phy_addr = 0, i;
  uint32_t ret;

  if (!pbuf)
    return -EIO;

  rtw_memset(pbuf, 0xFF, size);

  while (phy_addr < EFUSE_LOGICAL_MAP_HW_SIZE)
  {
    /*First two bytes are reserved for physical*/
    if (phy_addr == 0 || phy_addr == 1)
    {
      phy_addr++;
      continue;
    }
#ifdef EFUSE_LOGICAL_SIM
    static int map_inited = 0;
    if (!map_inited)
    {
      map_inited = 1;
      for (i = 0; i < EFUSE_LOGICAL_MAP_HW_SIZE; i++)
        hal_efuse_read(i, &ameba_efuse_sim_map[i], LDO_OUT_DEFAULT_VOLT);
    }
    ret = _TRUE;
    header = ameba_efuse_sim_map[phy_addr++];
#else
    ret = hal_efuse_read(phy_addr++, &header, LDO_OUT_DEFAULT_VOLT);
#endif
    if (ret != _TRUE)
      return -EIO;

    if (header == 0xFF)
      break;

    /* Check PG header for section num. */
    if ((header & 0x1F) == 0x0F)
    {
      /* extended header */
      offset = (header & 0xE0) >> 5;
#ifdef EFUSE_LOGICAL_SIM
      ret = _TRUE;
      extheader = ameba_efuse_sim_map[phy_addr++];
#else
      ret = hal_efuse_read(phy_addr++, &extheader, LDO_OUT_DEFAULT_VOLT);
#endif
      if (ret != _TRUE)
        return -EIO;
      if (((extheader & 0x0F) == 0x0F))
      {
        continue;
      }
      offset |= ((extheader & 0xF0) >> 1);
      wden = (extheader & 0x0F);
    }
    else
    {
      offset = ((header >> 4) & 0x0F);
      wden = (header & 0x0F);
    }

    /*One section has 8 bytes data, logical map has 512/8 = 64 sections*/
    if (offset < (EFUSE_LOGICAL_MAP_SIZE >> 3))
    {
      uint16_t addr = 0;
      /* Get word enable value from PG header */
      addr = offset * 8;
      /*Each section has 4 words data*/
      for (i = 0; i < 4; i++)
      {
        /* Check word enable condition in the section */
        if (!(wden & (0x01 << i)))
        {
#ifdef EFUSE_LOGICAL_SIM
          ret = _TRUE;
          data = ameba_efuse_sim_map[phy_addr++];
#else
          ret = hal_efuse_read(phy_addr++, &data, LDO_OUT_DEFAULT_VOLT);
#endif
          if (ret != _TRUE)
            return -EIO;
          if (addr >= laddr && addr < (laddr + size))
            pbuf[addr - laddr] = data;
#ifdef EFUSE_LOGICAL_SIM
          ret = _TRUE;
          data = ameba_efuse_sim_map[phy_addr++];
#else
          ret = hal_efuse_read(phy_addr++, &data, LDO_OUT_DEFAULT_VOLT);
#endif
          if (ret != _TRUE)
            return -EIO;
          if ((addr + 1) >= laddr && (addr + 1) < (laddr + size))
            pbuf[addr + 1 - laddr] = data;
        }
        addr += 2;
      }
    }
    else
    {
      uint8_t word_cnts = 0;
      if (!(wden & BIT(0)))
        word_cnts++; // 0 : write enable
      if (!(wden & BIT(1)))
        word_cnts++;
      if (!(wden & BIT(2)))
        word_cnts++;
      if (!(wden & BIT(3)))
        word_cnts++;
      phy_addr += word_cnts * 2;
    }
  }

  /*return used bytes*/
  return phy_addr - EIO;
}

#if 0
static int ameba_efuse_pg_packet(uint8_t offset, uint8_t wden, uint8_t *data)
{
  uint16_t idx = 2;       //the first two addresses are reserved
  uint8_t temp0, temp1, WordEn;
  uint8_t ret;
  uint8_t Len = 0;
  uint8_t word_idx;

  /* WordEnable bit=0 means write this bit */
  if ((wden & 0xF) == 0xF)
    return -EIO;

  if (offset == 0)
  {
    if (!(wden & BIT(0)))
      return -EIO;
  }
  /* count the physical written num of word */
  while (idx < EFUSE_LOGICAL_MAP_HW_SIZE)
  {
#ifdef EFUSE_LOGICAL_SIM
    ret = _TRUE;
    temp0 = ameba_efuse_sim_map[idx];
#else
    ret = hal_efuse_read(idx, &temp0, LDO_OUT_DEFAULT_VOLT);
#endif
    if (ret != _TRUE)
      return -EIO;

    if (temp0 != 0xff)
    {
      if ((temp0 & 0x0f) == 0xf)
      {
        idx++;
#ifdef EFUSE_LOGICAL_SIM
        ret = _TRUE;
        temp1 = ameba_efuse_sim_map[idx];
#else
        ret = hal_efuse_read(idx, &temp1, LDO_OUT_DEFAULT_VOLT);
#endif
        if (ret != _TRUE)
          return -EIO;

		/* ~ write enbale */
        WordEn = ((~temp1) & 0x0f);

        /* bit=0: word not write */
        /* bit=1: word have write */
        while (WordEn != 0)
        {
		  /* word have write */
          if (WordEn & BIT0)
            idx = idx + 2;
          WordEn = WordEn >> 1;
        }
      }
      else
      { /* normal header */
        WordEn = ((~temp0) & 0x0f);
        while (WordEn != 0)
        {
          if (WordEn & BIT0)
            idx = idx + 2;
          WordEn = WordEn >> 1;
        }
      }
    }
    else
    { /* find address not written*/
      break;
    }
    idx++;
  }

  wden = wden & 0xf;

  if ((wden & BIT(0)) == 0)
    Len = Len + 2;
  if ((wden & BIT(1)) == 0)
    Len = Len + 2;
  if ((wden & BIT(2)) == 0)
    Len = Len + 2;
  if ((wden & BIT(3)) == 0)
    Len = Len + 2;

  /* Efuse could PG to 0x99 at most */
  if ((idx + Len) < (EFUSE_LOGICAL_MAP_HW_SIZE))
  {
    if (offset >= 0xF)
    { /*ext header */
#ifdef EFUSE_LOGICAL_SIM
      ret = _TRUE;
      ameba_efuse_sim_map[idx] = (((offset << 5) | 0x0f));
#else
      ret = hal_efuse_write(idx, (((offset << 5) | 0x0f)), LDO_OUT_DEFAULT_VOLT);    //addr[2:0]
#endif
      if (ret != _TRUE)
        return -EIO;
      idx++;
#ifdef EFUSE_LOGICAL_SIM
      ret = _TRUE;
      ameba_efuse_sim_map[idx] = (((offset << 1) & 0xf0) | wden);
#else
      ret = hal_efuse_write(idx, (((offset << 1) & 0xf0) | wden), LDO_OUT_DEFAULT_VOLT); //addr[6:3]
#endif
      if (ret != _TRUE)
        return -EIO;

	  idx++;
    }
    else
    {
#ifdef EFUSE_LOGICAL_SIM
      ret = _TRUE;
      ameba_efuse_sim_map[idx] = (((offset << 4) & 0xf0) | wden);
#else
      ret = hal_efuse_write(idx, (((offset << 4) & 0xf0) | wden), LDO_OUT_DEFAULT_VOLT);
#endif
      if (ret != _TRUE)
        return -EIO;

      idx++;
    }
    for (word_idx = 0; word_idx < 4; word_idx++)
    {
      if ((wden & BIT(word_idx)) == 0)
      {
#ifdef EFUSE_LOGICAL_SIM
        ret = _TRUE;
        ameba_efuse_sim_map[idx] = *(data + word_idx * 2);
#else
        ret = hal_efuse_write(idx, *(data + word_idx * 2), LDO_OUT_DEFAULT_VOLT);
#endif
        if (ret != _TRUE)
          return -EIO;

        idx++;
#ifdef EFUSE_LOGICAL_SIM
        ret = _TRUE;
        ameba_efuse_sim_map[idx] = *(data + word_idx * 2 + 1);
#else
        ret = hal_efuse_write(idx, *(data + word_idx * 2 + 1), LDO_OUT_DEFAULT_VOLT);
#endif
        if (ret != _TRUE)
          return -EIO;

        idx++;
      }
    }
  }
  else
  {
    DBG_8710C("EFUSE PG No Enough Space!\n");
    return -EIO;
  }
  return 0;
}

int ameba_efuse_logical_write_raw(uint16_t addr, uint16_t cnts, uint8_t *data)
{
  uint8_t offset, word_en;
  uint8_t map[EFUSE_LOGICAL_MAP_SIZE];
  uint16_t mapLen = EFUSE_LOGICAL_MAP_SIZE;
  uint8_t newdata[8];
  int32_t i, idx;
  int ret = 0, used_bytes;

  if ((addr + cnts) > mapLen)
    return -EIO;

  used_bytes = ameba_efuse_logical_read(0, mapLen, map);
  if (used_bytes < 0)
    return -EIO;

  offset = (addr >> 3);
  word_en = 0xF;
  rtw_memset(newdata, 0xFF, 8);
  i = addr & 0x7; // index of one package
  idx = 0;    // data index

  if (i & 0x1)
  {
    // odd start
    if (data[idx] != map[addr + idx])
    {
      word_en &= ~(BIT(i >> 1));
      newdata[i - EIO] = map[addr + idx - EIO];
      newdata[i] = data[idx];
    }
    i++;
    idx++;
  }
  do
  {
    for (; i < 8; i += 2)
    {
      if (cnts == idx)
        break;
      if ((cnts - idx) == 1)
      {
        if (data[idx] != map[addr + idx])
        {
          word_en &= ~(BIT(i >> 1));
          newdata[i] = data[idx];
          newdata[i + 1] = map[addr + idx + 1];
        }
        idx++;
        break;
      }
      else
      {
        if ((data[idx] != map[addr + idx]) ||
          (data[idx + 1] != map[addr + idx + 1]))
        {
          word_en &= ~(BIT(i >> 1));
          newdata[i] = data[idx];
          newdata[i + 1] = data[idx + 1];
        }
        idx += 2;
      }
      if (idx == cnts)
        break;
    }

    if (word_en != 0xF)
    {
      ret = ameba_efuse_pg_packet(offset, word_en, newdata);
      if (ret < 0)
        return ret;
    }

    if (idx == cnts)
      break;

    offset++;
    i = 0;
    word_en = 0xF;
    rtw_memset(newdata, 0xFF, 8);
  } while (1);

#ifdef EFUSE_LOGICAL_SIM
  printf("\n\r");
  for (i = 0; i < EFUSE_LOGICAL_MAP_HW_SIZE; i += 16)
  {
    int j;
    printf("0x%03x\t", i);
    for (j = 0; j < 8; j++)
      printf("%02X ", ameba_efuse_sim_map[i + j]);
	printf("\t");
    for (; j < 16; j++)
      printf("%02X ", ameba_efuse_sim_map[i + j]);
	printf("\n\r");
  }
#endif
  return 0;
}

int ameba_efuse_logical_write(uint16_t addr, uint16_t cnts, uint8_t *data)
{
  if ((addr + cnts) < 0x20)
    return -EIO;

  if (addr < 0x20 && (addr + cnts) >= 0x20)
    return ameba_efuse_logical_write_raw(0x20, cnts - (0x20 - addr), data + (0x20 - addr));
  else
    return ameba_efuse_logical_write_raw(addr, cnts, data);
}

int ameba_efuse_fw_verify_enable(void)
{
  int ret;
  efuse_boot_cfg_t *pameba_efuse_boot_cfg = eFuseBootCfg;
  uint8_t sb_lock[2];

  sb_lock[0] = pameba_efuse_boot_cfg->word & 0xff;
  sb_lock[1] = (pameba_efuse_boot_cfg->word >> 8) & 0xff;

  // if logical eFuse 0x19[7] = 1 -> non-encrypted fw won't be allowed to boot
  if (pameba_efuse_boot_cfg->byte.cfg2.bit.secure_lock == 1)
    return 0;
  sb_lock[1] |= BIT7;

  ret = ameba_efuse_logical_write_raw(EFUSE_LOGICAL_SBLOCK_OFFSET - EIO, 2, sb_lock); //write one word in 0x18 & 0x19 two bytes
  if (ret >= 0)
    return 0; /* secure boot is enabled */
  else
    return -EIO;
}

int ameba_efuse_fw_verify_check(void)
{
  int ret;
  uint8_t sb_lock = 0xFF;
  efuse_boot_cfg_t *pameba_efuse_boot_cfg = eFuseBootCfg;
  if (pameba_efuse_boot_cfg->byte.cfg2.bit.secure_lock == 1)
    return 1;
  ret = ameba_efuse_logical_read(EFUSE_LOGICAL_SBLOCK_OFFSET, 1, &sb_lock);
  if (sb_lock != 0xFF)
  {
    if (ret >= 0 && ((sb_lock)&BIT7))
      return 1;
  }
  return 0;
}

int ameba_efuse_boot_message_disable(void)
{
  int ret;
  efuse_boot_cfg_t *pameba_efuse_boot_cfg = eFuseBootCfg;
  if (pameba_efuse_boot_cfg->byte.dbg_cfg.bit.dbg_en == BootDbgMsgOn)
  {
    uint8_t sb_lock[4];
    sb_lock[0] = pameba_efuse_boot_cfg->word & 0xff;
    sb_lock[1] = (pameba_efuse_boot_cfg->word >> 8) & 0xff;
    sb_lock[2] = (pameba_efuse_boot_cfg->word >> 16) & 0xff;
    sb_lock[3] = (pameba_efuse_boot_cfg->word >> 24) & 0xff;
    sb_lock[2] |= BIT4;
    ret = ameba_efuse_logical_write_raw(EFuseBootCfg1Offset, 4, sb_lock);
    if (ret >= 0)
      return 0;/* Boot debugging message is disabled! */
    else
      return -EIO;
  }
  else
  {
    /* Boot debugging message is disabled! */
  }
  return 0;
}

int ameba_efuse_boot_message_enable(void)
{
  int ret;
  efuse_boot_cfg_t *pameba_efuse_boot_cfg = eFuseBootCfg;
  if (pameba_efuse_boot_cfg->byte.dbg_cfg.bit.dbg_en == BootDbgMsgOff)
  {
    uint8_t sb_lock[4];
    sb_lock[0] = pameba_efuse_boot_cfg->word & 0xff;
    sb_lock[1] = (pameba_efuse_boot_cfg->word >> 8) & 0xff;
    sb_lock[2] = (pameba_efuse_boot_cfg->word >> 16) & 0xff;
    sb_lock[3] = (pameba_efuse_boot_cfg->word >> 24) & 0xff;
    sb_lock[2] ^= BIT4;
    ret = ameba_efuse_logical_write_raw(EFuseBootCfg1Offset, 4, sb_lock);
    if (ret >= 0)
      return 0; /* Boot debugging message is enabled! */
    else
      return -EIO;
  }
  else
  {
    /* Boot debugging message is disabled! */
  }
  return 0;
}
#endif
#endif
