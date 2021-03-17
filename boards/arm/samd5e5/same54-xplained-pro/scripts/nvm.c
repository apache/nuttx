/****************************************************************************
 * boards/arm/samd5e5/same54-xplained-pro/scripts/nvm.c
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

#include <stdio.h>
#include <stdint.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t nvm[20] =
{
  0x14,                                           /* Count 20 bytes */
  0x80, 0x40, 0x00,                               /* 24-address : 804000 */
  0x39, 0x92, 0x9a, 0xfe, 0x80, 0xff, 0xec, 0xae, /* 16-bytes of NVM data */
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  unsigned int csum;
  int i;

  printf("S2");

  for (i = 0, csum = 0; i < 20; i++)
    {
      csum += nvm[i];
      printf("%02X", (unsigned int)nvm[i]);
    }

  printf("%02X\r\n", ~csum & 0xff);
  printf("S9030000FC\r\n");
  return 0;
}
