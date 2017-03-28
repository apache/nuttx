/****************************************************************************
 * net/sixlowpan/sixlowpan_hc06.c
 * 6lowpan HC06 implementation (draft-ietf-6lowpan-hc-06)
 *
 *   Copyright (C) 2017, Gregory Nutt, all rights reserved
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from Contiki:
 *
 *   Copyright (c) 2008, Swedish Institute of Computer Science.
 *   All rights reserved.
 *   Authors: Adam Dunkels <adam@sics.se>
 *            Nicolas Tsiftes <nvt@sics.se>
 *            Niclas Finne <nfi@sics.se>
 *            Mathilde Durvy <mdurvy@cisco.com>
 *            Julien Abeille <jabeille@cisco.com>
 *            Joakim Eriksson <joakime@sics.se>
 *            Joel Hoglund <joel@sics.se>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/* FOR HC-06 COMPLIANCE TODO:
 *  
 * -Add compression options to UDP, currently only supports
 *  both ports compressed or both ports elided
 * -Verify TC/FL compression works
 * -Add stateless multicast option
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/net/sixlowpan.h>

#include "sixlowpan/sixlowpan.h"

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC06

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* An address context for IPHC address compression each context can have up
 * to 8 bytes
 */

struct sixlowpan_addrcontext_s
{
  uint8_t used;       /* Possibly use as prefix-length */
  uint8_t number;
  uint8_t prefix[8];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* HC06 specific variables */

#if CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 0
/* Addresses contexts for IPHC. */

static struct sixlowpan_addrcontext_s
  g_hc06_addrcontexts[CONFIG_NET_6LOWPAN_MAXADDRCONTEXT];
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_hc06_initialize
 *
 * Description:
 *   sixlowpan_hc06_initialize() is called during OS initialization at power-up
 *   reset.  It is called from the common sixlowpan_initialize() function.
 *   sixlowpan_hc06_initialize() configures HC06 networking data structures.
 *   It is called prior to platform-specific driver initialization so that
 *   the 6loWPAN networking subsystem is prepared to deal with network
 *   driver initialization actions.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sixlowpan_hc06_initialize(void)
{
#if CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 0
#if CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 1
  int i;
#endif

  /* Preinitialize any address contexts for better header compression
   * (Saves up to 13 bytes per 6lowpan packet).
   */

  g_hc06_addrcontexts[0].used      = 1;
  g_hc06_addrcontexts[0].number    = 0;

  g_hc06_addrcontexts[0].prefix[0] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_0_0;
  g_hc06_addrcontexts[0].prefix[1] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_0_1;

#if CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 1
  for (i = 1; i < CONFIG_NET_6LOWPAN_MAXADDRCONTEXT; i++)
    {
#ifdef CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREINIT_1
      if (i == 1)
        {
          g_hc06_addrcontexts[1].used      = 1;
          g_hc06_addrcontexts[1].number    = 1;

          g_hc06_addrcontexts[1].prefix[0] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_1_0;
          g_hc06_addrcontexts[1].prefix[1] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_1_1;
        }
      else
#ifdef CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREINIT_2
      if (i == 2)
        {
          g_hc06_addrcontexts[2].used      = 1;
          g_hc06_addrcontexts[2].number    = 2;

          g_hc06_addrcontexts[2].prefix[0] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_1_0;
          g_hc06_addrcontexts[2].prefix[1] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_1_1;
        }
      else
#endif /* SIXLOWPAN_CONF_ADDR_CONTEXT_2 */
        {
          g_hc06_addrcontexts[i].used = 0;
        }
#else
      g_hc06_addrcontexts[i].used = 0;
#endif /* SIXLOWPAN_CONF_ADDR_CONTEXT_1 */
    }
#endif /* CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 1 */
#endif /* CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 0 */
}

#endif /* CONFIG_NET_6LOWPAN_COMPRESSION_HC06 */
