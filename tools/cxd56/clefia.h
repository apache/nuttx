/****************************************************************************
 * tools/cxd56/clefia.h
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

#ifndef _TOOLS_CXD56_CLEFIA_H_
#define _TOOLS_CXD56_CLEFIA_H_

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct cipher
  {
    int mode;
    int dir;
    uint8_t rk[8 * 26 + 16];
    uint8_t vector[16];
    int round;
    uint8_t k1[16];
    uint8_t k2[16];
  };

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct cipher *cipher_init(uint8_t * key, uint8_t * iv);
void cipher_deinit(struct cipher *c);
int cipher_calc_cmac(struct cipher *c, void *data, int size, void *cmac);
void bytexor(unsigned char *dst, const unsigned char *a,
             const unsigned char *b, int bytelen);
int clefiakeyset(unsigned char *rk, const unsigned char *skey);
void clefiaencrypt(unsigned char *ct, const unsigned char *pt,
                   const unsigned char *rk, const int r);

#endif
