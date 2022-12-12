/************************************************************************************
 * include/nuttx/wireless/cellular/cellular.h
 * Cellular network device commands
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
 ************************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_CELLULAR_CELLULAR_H
#define __INCLUDE_NUTTX_WIRELESS_CELLULAR_CELLULAR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/socket.h>
#include <stdint.h>

#include <net/if.h>
#include <nuttx/fs/ioctl.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Sizing parameters */

#define IFCELLDEVPARAMSIZ   136 /* Big enough to store cellelur net paramters */

/* Network Driver IOCTL Commands ****************************************************/

/* Use of these IOCTL commands requires a socket descriptor created by the socket()
 * interface.
 */

#define SIOCSCELLNETDEV     _CELLIOC(0x0000)  /* Set info in dev */
#define SIOCGCELLNETDEV     _CELLIOC(0x0001)  /* Get info in dev */

/* cell member for struct icellreq */

#define ifr_cellinfo        icellr_ifru.icellru_info;
#define ifr_celldata        icellr_ifru.ioctl_param;

/************************************************************************************
 * Public Type Definitions
 ************************************************************************************/

struct cellnetinfo_s
{
  uint8_t mdp_chnidx; /* MDP(Multi Data Path) channel idx */
  uint8_t sim_id;     /* Sim ID in dual sim system */
  uint8_t cell_id;    /* Cell Identification in mobile phone networks */
  uint8_t if_type;
};

/* This is the structure used to exchange data in cellular IOCTLs.
 * same as 'struct ifreq', but defined for use with cellular IOCTLs.
 */

struct icellreq
{
  char ifr_name[IFNAMSIZ];    /* Interface name, e.g. "cell0" */
  union
  {
    struct  cellnetinfo_s     icellru_info;
    uint8_t                   ioctl_param[IFCELLDEVPARAMSIZ];
  }icellr_ifru;
};

#endif /* __INCLUDE_NUTTX_WIRELESS_CELLULAR_CELLULAR_H */
