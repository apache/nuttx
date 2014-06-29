/****************************************************************************
 * net/igmp/igmp.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef _NET_IGMP_IGMP_H
#define _NET_IGMP_IGMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Defined in igmp_init.c ***************************************************/

void igmp_initialize(void);

/* Defined in igmp_input.c **************************************************/

void igmp_input(struct net_driver_s *dev);

/* Defined in igmp_group.c **************************************************/

void igmp_grpinit(void);
FAR struct igmp_group_s *igmp_grpalloc(FAR struct net_driver_s *dev,
                                       FAR const net_ipaddr_t *addr);
FAR struct igmp_group_s *igmp_grpfind(FAR struct net_driver_s *dev,
                                      FAR const net_ipaddr_t *addr);
FAR struct igmp_group_s *igmp_grpallocfind(FAR struct net_driver_s *dev,
                                           FAR const net_ipaddr_t *addr);
void igmp_grpfree(FAR struct net_driver_s *dev,
                  FAR struct igmp_group_s *group);

/* Defined in igmp_msg.c ****************************************************/

void igmp_schedmsg(FAR struct igmp_group_s *group, uint8_t msgid);
void igmp_waitmsg(FAR struct igmp_group_s *group, uint8_t msgid);

/* Defined in igmp_poll.c ***************************************************/

void igmp_poll(FAR struct net_driver_s *dev);

/* Defined in igmp_send.c ***************************************************/

void igmp_send(FAR struct net_driver_s *dev, FAR struct igmp_group_s *group,
                  FAR net_ipaddr_t *dest);

/* Defined in igmp_timer.c **************************************************/

int igmp_decisec2tick(int decisecs);
void igmp_startticks(FAR struct igmp_group_s *group, int ticks);
void igmp_starttimer(FAR struct igmp_group_s *group, uint8_t decisecs);
bool igmp_cmptimer(FAR struct igmp_group_s *group, int maxticks);

/* Defined in igmp_mcastmac *************************************************/

void igmp_addmcastmac(FAR struct net_driver_s *dev, FAR net_ipaddr_t *ip);
void igmp_removemcastmac(FAR struct net_driver_s *dev, FAR net_ipaddr_t *ip);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_IGMP */
#endif /* _NET_IGMP_IGMP_H */
