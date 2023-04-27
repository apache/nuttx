/****************************************************************************
 * include/nuttx/efuse/sama5_sfc_fuses.h
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

#ifndef __INCLUDE_NUTTX_EFUSE_SAMA5_FUSES_H
#define __INCLUDE_NUTTX_EFUSE_SAMA5_FUSES_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(ATSAMA5D2)
#define SAM_SFC_EFUSE_MAX_LEN  544    /* Max length of sfc area. */
#elif defined ATSAMA5D4
#define SAM_SFC_EFUSE_MAX_LEN  512    /* Max length of sfc area. */
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

#if defined(CONFIG_EFUSE) && defined(CONFIG_SAMA5_SFC)

/* enum of SFC blocks for SAMA5 */

enum
{
  SFC_DR0 = 0,
  SFC_DR1,
  SFC_DR2,
  SFC_DR3,
  SFC_DR4,
  SFC_DR5,
  SFC_DR6,
  SFC_DR7,
  SFC_DR8,
  SFC_DR9,
  SFC_DR10,
  SFC_DR11,
  SFC_DR12,
  SFC_DR13,
  SFC_DR14,
#ifdef ATSAMA5D2
  SFC_DR15,
  SFC_BOOT,
  SFC_JTAG,
  SFC_SEC_BOOT,
#else
  SFC_S,
  SFC_MD,
  SFC_SECURE_DEBUG,
  SFC_JTAG_DIS,
#endif
  SFC_DR_END,
};

/* Generic descriptions for the SAM SFC fuses, 32 bits wide.
 * These can be replaced by user definitions in board code if required.
 */

static const efuse_desc_t SAMA5_SFC_DATA0[] =
{
  {
    0, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA1[] =
{
  {
    32, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA2[] =
{
  {
    64, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA3[] =
{
  {
    96, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA4[] =
{
  {
    128, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA5[] =
{
  {
    160, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA6[] =
{
  {
    192, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA7[] =
{
  {
    224, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA8[] =
{
  {
    256, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA9[] =
{
  {
    288, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA10[] =
{
  {
    320, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA11[] =
{
  {
    352, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA12[] =
{
  {
    384, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA13[] =
{
  {
    416, 32
  },
};

static const efuse_desc_t SAMA5_SFC_DATA14[] =
{
  {
    448, 32
  },
};
 #ifdef ATSAMA5D2
static const efuse_desc_t SAMA5_SFC_DATA15[] =
{
  {
    480, 32
  },
};

static const efuse_desc_t SAMA5_SFC_BOOT_CFG[] =
{
  {
    512, 30
  },
};

static const efuse_desc_t SAMA5_SFC_JTAG_DIS[] =
{
  {
    542, 1
  },
};

static const efuse_desc_t SAMA5_SFC_SECURE_DEBUG[] =
{
  {
    543, 1
  },
};

#else

static const efuse_desc_t SAMA5_SFC_S[] =
{
  {
    480, 1
  },
};

static const efuse_desc_t SAMA5_SFC_MD[] =
{
  {
    482, 1
  },
};

static const efuse_desc_t SAMA5_SFC_SECURE_DEBUG[] =
{
  {
    510, 1
  },
};

static const efuse_desc_t SAMA5_SFC_JTAG_DIS[] =
{
  {
    511, 1
  },
};

#endif

#endif /* defined(CONFIG_EFUSE) && defined(CONFIG_SAMA5_SFC) */
#endif /* __INCLUDE_NUTTX_EFUSE_SAMA5_FUSES_H */
