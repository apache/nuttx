/****************************************************************************
 * net/netfilter/iptables.h
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

#ifndef __NET_NETFILTER_IPTABLES_H
#define __NET_NETFILTER_IPTABLES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netfilter/ip_tables.h>

#ifdef CONFIG_NET_IPTABLES

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ipt_setsockopt
 *
 * Description:
 *   setsockopt function of iptables.
 *
 ****************************************************************************/

int ipt_setsockopt(FAR struct socket *psock, int option,
                   FAR const void *value, socklen_t value_len);

/****************************************************************************
 * Name: ipt_getsockopt
 *
 * Description:
 *   getsockopt function of iptables.
 *
 ****************************************************************************/

int ipt_getsockopt(FAR struct socket *psock, int option,
                   FAR void *value, FAR socklen_t *value_len);

/****************************************************************************
 * Name: ipt_alloc_table
 *
 * Description:
 *   Allocate an initial table info with valid_hooks specified.
 *   Will generate a default entry with standard target for each valid hook,
 *   and an entry with error target at the end of entry table.
 *
 * Input Parameters:
 *   table       - The name of the table.
 *   valid_hooks - The valid_hooks of the table, it's a bitmap of hooks.
 *
 * Returned Value:
 *   Newly generated ipt_replace structure.
 *
 ****************************************************************************/

FAR struct ipt_replace *ipt_alloc_table(FAR const char *table,
                                        unsigned int valid_hooks);

/****************************************************************************
 * Name: ipt_nat_init
 *
 * Description:
 *   Init NAT table data.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NAT
FAR struct ipt_replace *ipt_nat_init(void);
#endif

/****************************************************************************
 * Name: ipt_nat_apply
 *
 * Description:
 *   Try to apply NAT rules, will do nothing if failed.
 *
 * Input Parameters:
 *   repl   - The config got from user space to control NAT table.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NAT
int ipt_nat_apply(FAR const struct ipt_replace *repl);
#endif

#endif /* CONFIG_NET_IPTABLES */
#endif /* __NET_NETFILTER_IPTABLES_H */
