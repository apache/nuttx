/****************************************************************************
 * arch/sim/src/sim/up_cliopts.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>

#include "up_hostinternal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern int optind;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct option opts[] =
  {
#ifdef CONFIG_SIM_NETDEV_VPNKIT
    { "vpnkit-sock", required_argument, NULL, 'f' },
#endif
    { 0, 0, NULL, 0 },
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usage
 *
 * Description:
 *   Print usage and exit.
 *
 ****************************************************************************/

static void usage()
{
  printf("USAGE: nuttx [options]\n\n");
#ifdef CONFIG_SIM_NETDEV_VPNKIT
  printf("OPTIONS:\n"
         "  --vpnkit-sock <value>   Specify the unix domain socket to\n"
         "                          communicate with VPNKit\n"
         "\n");
#endif
  exit(EXIT_FAILURE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: process_cli_options
 *
 * Description:
 *
 ****************************************************************************/

void process_cli_options(int argc, char **argv)
{
  int ch;

  while ((ch = getopt_long(argc, argv, "", opts, NULL)) != -1)
    {
      switch (ch)
        {
#ifdef CONFIG_SIM_NETDEV_VPNKIT
          case 'f':
            vpnkit_set_sock(optarg);
            break;
#endif
          case '?':
          default:
            usage();
        }
    }

    argc -= optind;
    argv += optind;
    if (argc != 0)
      {
        usage();
      }
}
