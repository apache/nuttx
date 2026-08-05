/****************************************************************************
 * drivers/wireless/lpwan/sx1301/sx1301_region.c
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

/* Channel plans of the supported regions.
 *
 * Each plan lists the absolute frequency of the eight multi-SF channels and
 * of the LoRa standard channel, plus the centre frequency of the two radios.
 * The intermediate frequency of every demodulator is the difference between
 * the two, and must stay inside about +/-460 kHz for a 125 kHz channel,
 * which is what sx1301_region_apply() checks.
 *
 * The AU915 and US915 bands are split in eight sub-bands of eight 125 kHz
 * channels.  Networks pick one: The Things Network and the Brazilian
 * deployments use the second sub-band (channels 8 to 15 plus the 500 kHz
 * channel 65), so that is the default here.  The first sub-band (channels 0
 * to 7 plus channel 64) is available as "AU915-1" and "US915-1".
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/wireless/wireless.h>

#include "sx1301_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Front-end RSSI offset, in 0.1 dBm units, for the SX125x radios of the
 * reference designs.
 */

#define SX1301_RSSI_OFFSET_DBM10 (-1660)

/* Largest usable intermediate frequency of a 125 kHz channel */

#define SX1301_IF_LIMIT_HZ       460000

#define MULTI(f, rf)                                  \
  {                                                   \
    .freq_hz = (f), .rf_chain = (rf),                 \
    .type = LORA_GW_CHAN_MULTI_SF,                    \
    .bandwidth = LORA_GW_BW_125K, .datarate = 0,      \
    .enable = true                                    \
  }

#define STD(f, rf, bw, sf)                            \
  {                                                   \
    .freq_hz = (f), .rf_chain = (rf),                 \
    .type = LORA_GW_CHAN_STD,                         \
    .bandwidth = (bw), .datarate = (sf),              \
    .enable = true                                    \
  }

#define CHAN_OFF(t)                                   \
  {                                                   \
    .freq_hz = 0, .rf_chain = 0, .type = (t),         \
    .bandwidth = LORA_GW_BW_UNDEFINED, .datarate = 0, \
    .enable = false                                   \
  }

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct lora_gw_regioninfo_s g_sx1301_regions[] =
{
  {
    .name = "AU915",
    .desc = "AU915 sub-band 2, ch 8-15+65 (BR/AU, TTN)",
    .radio_freq =
      {
        917100000, 917900000
      },
    .channels =
      {
        MULTI(916800000, 0), MULTI(917000000, 0),
        MULTI(917200000, 0), MULTI(917400000, 0),
        MULTI(917600000, 1), MULTI(917800000, 1),
        MULTI(918000000, 1), MULTI(918200000, 1),
        STD(917500000, 0, LORA_GW_BW_500K, 8),
        CHAN_OFF(LORA_GW_CHAN_FSK)
      }
  },
  {
    .name = "AU915-1",
    .desc = "AU915 sub-band 1, ch 0-7+64",
    .radio_freq =
      {
        915500000, 916300000
      },
    .channels =
      {
        MULTI(915200000, 0), MULTI(915400000, 0),
        MULTI(915600000, 0), MULTI(915800000, 0),
        MULTI(916000000, 1), MULTI(916200000, 1),
        MULTI(916400000, 1), MULTI(916600000, 1),
        STD(915900000, 0, LORA_GW_BW_500K, 8),
        CHAN_OFF(LORA_GW_CHAN_FSK)
      }
  },
  {
    .name = "US915",
    .desc = "US915 sub-band 2, ch 8-15+65 (TTN)",
    .radio_freq =
      {
        904200000, 905000000
      },
    .channels =
      {
        MULTI(903900000, 0), MULTI(904100000, 0),
        MULTI(904300000, 0), MULTI(904500000, 0),
        MULTI(904700000, 1), MULTI(904900000, 1),
        MULTI(905100000, 1), MULTI(905300000, 1),
        STD(904600000, 0, LORA_GW_BW_500K, 8),
        CHAN_OFF(LORA_GW_CHAN_FSK)
      }
  },
  {
    .name = "US915-1",
    .desc = "US915 sub-band 1, ch 0-7+64",
    .radio_freq =
      {
        903500000, 904300000
      },
    .channels =
      {
        MULTI(903200000, 0), MULTI(903400000, 0),
        MULTI(903600000, 0), MULTI(903800000, 0),
        MULTI(904000000, 1), MULTI(904200000, 1),
        MULTI(904400000, 1), MULTI(904600000, 1),
        STD(903900000, 0, LORA_GW_BW_500K, 8),
        CHAN_OFF(LORA_GW_CHAN_FSK)
      }
  },
  {
    .name = "EU868",
    .desc = "EU863-870, 8 channels + 868.3 BW250",
    .radio_freq =
      {
        867500000, 868500000
      },
    .channels =
      {
        MULTI(867100000, 0), MULTI(867300000, 0),
        MULTI(867500000, 0), MULTI(867700000, 0),
        MULTI(867900000, 0), MULTI(868100000, 1),
        MULTI(868300000, 1), MULTI(868500000, 1),
        STD(868300000, 1, LORA_GW_BW_250K, 7),
        CHAN_OFF(LORA_GW_CHAN_FSK)
      }
  },
  {
    .name = "AS923",
    .desc = "AS923-1, 922.2-923.6 MHz",
    .radio_freq =
      {
        922500000, 923300000
      },
    .channels =
      {
        MULTI(922200000, 0), MULTI(922400000, 0),
        MULTI(922600000, 0), MULTI(922800000, 0),
        MULTI(923000000, 1), MULTI(923200000, 1),
        MULTI(923400000, 1), MULTI(923600000, 1),
        STD(923200000, 1, LORA_GW_BW_250K, 7),
        CHAN_OFF(LORA_GW_CHAN_FSK)
      }
  },
  {
    .name = "KR920",
    .desc = "KR920-923, 7 channels",
    .radio_freq =
      {
        922400000, 923100000
      },
    .channels =
      {
        MULTI(922100000, 0), MULTI(922300000, 0),
        MULTI(922500000, 0), MULTI(922700000, 0),
        MULTI(922900000, 1), MULTI(923100000, 1),
        MULTI(923300000, 1), CHAN_OFF(LORA_GW_CHAN_MULTI_SF),
        CHAN_OFF(LORA_GW_CHAN_STD),
        CHAN_OFF(LORA_GW_CHAN_FSK)
      }
  },
  {
    .name = "IN866",
    .desc = "IN865-867, 3 mandatory + 3 extra channels",
    .radio_freq =
      {
        865520000, 866300000
      },
    .channels =
      {
        MULTI(865062500, 0), MULTI(865402500, 0),
        MULTI(865985000, 0), MULTI(866100000, 1),
        MULTI(866300000, 1), MULTI(866500000, 1),
        CHAN_OFF(LORA_GW_CHAN_MULTI_SF),
        CHAN_OFF(LORA_GW_CHAN_MULTI_SF),
        CHAN_OFF(LORA_GW_CHAN_STD),
        CHAN_OFF(LORA_GW_CHAN_FSK)
      }
  }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx1301_region_count
 ****************************************************************************/

int sx1301_region_count(void)
{
  return nitems(g_sx1301_regions);
}

/****************************************************************************
 * Name: sx1301_region_byname
 *
 * Description:
 *   Look up a region by name, case insensitively.
 *
 * Returned Value:
 *   The region index, or -ENOENT if there is no such region.
 *
 ****************************************************************************/

int sx1301_region_byname(FAR const char *name)
{
  size_t i;

  if (name == NULL)
    {
      return -EINVAL;
    }

  for (i = 0; i < nitems(g_sx1301_regions); i++)
    {
      if (strcasecmp(name, g_sx1301_regions[i].name) == 0)
        {
          return i;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: sx1301_region_getinfo
 ****************************************************************************/

int sx1301_region_getinfo(int region,
                          FAR struct lora_gw_regioninfo_s *info)
{
  if (info == NULL)
    {
      return -EINVAL;
    }

  if (region < 0 || region >= sx1301_region_count())
    {
      return -ENODEV;
    }

  memcpy(info, &g_sx1301_regions[region], sizeof(*info));
  return OK;
}

/****************************************************************************
 * Name: sx1301_region_apply
 *
 * Description:
 *   Translate a channel plan into the radio and demodulator configuration
 *   used by sx1301_start().  Only radio A is enabled for transmission, as
 *   on the reference designs where radio B has no PA path.
 *
 ****************************************************************************/

int sx1301_region_apply(FAR struct sx1301_dev_s *priv, int region)
{
  FAR const struct lora_gw_regioninfo_s *ri;
  FAR const struct lora_gw_chaninfo_s *ch;
  uint32_t centre;
  int32_t iffreq;
  int i;

  if (region < 0 || region >= sx1301_region_count())
    {
      return -ENODEV;
    }

  ri = &g_sx1301_regions[region];

  /* Radios.  Both receive, only radio A transmits. */

  for (i = 0; i < LORA_GW_RF_CHAIN_NB; i++)
    {
      priv->rf[i].enable            = true;
      priv->rf[i].tx_enable         = (i == 0);
      priv->rf[i].freq_hz           = ri->radio_freq[i];
      priv->rf[i].rssi_offset_dbm10 = SX1301_RSSI_OFFSET_DBM10;
    }

  /* Multi-SF demodulators */

  for (i = 0; i < LORA_GW_MULTI_NB; i++)
    {
      ch     = &ri->channels[i];
      centre = ri->radio_freq[ch->rf_chain];

      memset(&priv->ifc[i], 0, sizeof(priv->ifc[i]));

      if (!ch->enable)
        {
          continue;
        }

      iffreq = (int32_t)ch->freq_hz - (int32_t)centre;
      if (iffreq > SX1301_IF_LIMIT_HZ || iffreq < -SX1301_IF_LIMIT_HZ)
        {
          wlerr("ERROR: %s channel %d at %" PRIu32 " Hz is %" PRId32
                " Hz away from radio %d\n",
                ri->name, i, ch->freq_hz, iffreq, ch->rf_chain);
          return -ERANGE;
        }

      priv->ifc[i].enable   = true;
      priv->ifc[i].rf_chain = ch->rf_chain;
      priv->ifc[i].freq_hz  = iffreq;
    }

  /* LoRa standard demodulator (IF8) */

  ch = &ri->channels[LORA_GW_MULTI_NB];
  memset(&priv->std, 0, sizeof(priv->std));

  if (ch->enable)
    {
      centre = ri->radio_freq[ch->rf_chain];
      iffreq = (int32_t)ch->freq_hz - (int32_t)centre;

      priv->std.enable   = true;
      priv->std.rf_chain = ch->rf_chain;
      priv->std.freq_hz  = iffreq;
      priv->std.bandwidth = ch->bandwidth;
      priv->std.datarate  = ch->datarate;
    }

  priv->region = region;

  wlinfo("Region %s selected (%s)\n", ri->name, ri->desc);
  return OK;
}
