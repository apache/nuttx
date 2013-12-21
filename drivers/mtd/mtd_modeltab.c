/**************************************************************************************************
 * drivers/mtd/mtd_modeltab.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was based on Atmel sample code with modifications for
 * better integration with NuttX.  The Atmel sample code has a BSD
 * compatible license that requires this copyright notice:
 *
 *   Copyright (c) 2012, Atmel Corporation
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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
 **************************************************************************************************/

/**************************************************************************************************
 * Included Files
 **************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd/nand_config.h>

#include <nuttx/mtd/nand_scheme.h>
#include <nuttx/mtd/nand_model.h>

/**************************************************************************************************
 * Public Data
 **************************************************************************************************/

/* List of NandFlash models which can be recognized by the software */

const struct nand_model_s g_nandmodels[NAND_NMODELS] =
{
/*----------|------------------------------+------+-------+------+-------+------------------------
 * |  ID    | OPTIONS                      | PAGE | SPARE | DEV  | BLOCK | SCHEME
 * |        |                              | SIZE | SIZE  | SIZE | SIZE  |
 *----------|------------------------------+------+-------+------+-------+------------------------*/

    {0x6e,   NANDMODEL_DATAWIDTH8,            256,  0,        1,      4,  &g_nand_sparescheme256},
    {0x64,   NANDMODEL_DATAWIDTH8,            256,  0,        2,      4,  &g_nand_sparescheme256},
    {0x68,   NANDMODEL_DATAWIDTH8,           4096,  0,      224,   1024,  &g_nand_sparescheme4096},
    {0x6b,   NANDMODEL_DATAWIDTH8,            512,  0,        4,      8,  &g_nand_sparescheme512},
    {0xe8,   NANDMODEL_DATAWIDTH8,            256,  0,        1,      4,  &g_nand_sparescheme256},
    {0xec,   NANDMODEL_DATAWIDTH8,            256,  0,        1,      4,  &g_nand_sparescheme256},
    {0xea,   NANDMODEL_DATAWIDTH8,            256,  0,        2,      4,  &g_nand_sparescheme256},
    {0xd5,   NANDMODEL_DATAWIDTH8,            512,  0,        4,      8,  &g_nand_sparescheme512},
    {0xe3,   NANDMODEL_DATAWIDTH8,            512,  0,        4,      8,  &g_nand_sparescheme512},
    {0xe5,   NANDMODEL_DATAWIDTH8,            512,  0,        4,      8,  &g_nand_sparescheme512},
    {0xd6,   NANDMODEL_DATAWIDTH8,            512,  0,        8,      8,  &g_nand_sparescheme512},

/*----------|------------------------------+------+-------+------+-------+------------------------*/

    {0x39,   NANDMODEL_DATAWIDTH8,            512,  0,        8,      8,  &g_nand_sparescheme512},
    {0xe6,   NANDMODEL_DATAWIDTH8,            512,  0,        8,      8,  &g_nand_sparescheme512},
    {0x49,   NANDMODEL_DATAWIDTH16,           512,  0,        8,      8,  &g_nand_sparescheme512},
    {0x59,   NANDMODEL_DATAWIDTH16,           512,  0,        8,      8,  &g_nand_sparescheme512},

/*----------|------------------------------+------+-------+------+-------+------------------------*/

    {0x33,   NANDMODEL_DATAWIDTH8,            512,  0,       16,     16,  &g_nand_sparescheme512},
    {0x73,   NANDMODEL_DATAWIDTH8,            512,  0,       16,     16,  &g_nand_sparescheme512},
    {0x43,   NANDMODEL_DATAWIDTH16,           512,  0,       16,     16,  &g_nand_sparescheme512},
    {0x53,   NANDMODEL_DATAWIDTH16,           512,  0,       16,     16,  &g_nand_sparescheme512},

/*----------|------------------------------+------+-------+------+-------+------------------------*/

    {0x35,   NANDMODEL_DATAWIDTH8,            512,  0,       32,     16,  &g_nand_sparescheme512},
    {0x75,   NANDMODEL_DATAWIDTH8,            512,  0,       32,     16,  &g_nand_sparescheme512},
    {0x45,   NANDMODEL_DATAWIDTH16,           512,  0,       32,     16,  &g_nand_sparescheme512},
    {0x55,   NANDMODEL_DATAWIDTH16,           512,  0,       32,     16,  &g_nand_sparescheme512},

/*----------|------------------------------+------+-------+------+-------+------------------------*/

    {0x36,   NANDMODEL_DATAWIDTH8,            512,  0,       64,     16,  &g_nand_sparescheme512},
    {0x76,   NANDMODEL_DATAWIDTH8,            512,  0,       64,     16,  &g_nand_sparescheme512},
    {0x46,   NANDMODEL_DATAWIDTH16,           512,  0,       64,     16,  &g_nand_sparescheme512},
    {0x56,   NANDMODEL_DATAWIDTH16,           512,  0,       64,     16,  &g_nand_sparescheme512},

/*----------|------------------------------+------+-------+------+-------+------------------------*/

    {0x78,   NANDMODEL_DATAWIDTH8,            512,  0,      128,     16,  &g_nand_sparescheme512},
    {0x39,   NANDMODEL_DATAWIDTH8,            512,  0,      128,     16,  &g_nand_sparescheme512},
    {0x79,   NANDMODEL_DATAWIDTH8,            512,  0,      128,     16,  &g_nand_sparescheme512},
    {0x72,   NANDMODEL_DATAWIDTH16,           512,  0,      128,     16,  &g_nand_sparescheme512},
    {0x49,   NANDMODEL_DATAWIDTH16,           512,  0,      128,     16,  &g_nand_sparescheme512},
    {0x74,   NANDMODEL_DATAWIDTH16,           512,  0,      128,     16,  &g_nand_sparescheme512},
    {0x59,   NANDMODEL_DATAWIDTH16,           512,  0,      128,     16,  &g_nand_sparescheme512},

/*----------|------------------------------+------+-------+------+-------+------------------------*/

    {0x71,   NANDMODEL_DATAWIDTH8,            512,  0,      256,     16,  &g_nand_sparescheme512},

/* Large blocks devices. Parameters must be fetched from the extended I */

#define OPTIONS NANDMODEL_COPYBACK

/*----------|------------------------------+------+-------+------+-------+------------------------
 * |  ID    | OPTIONS                      | PAGE | SPARE | DEV  | BLOCK | SCHEME
 * |        |                              | SIZE | SIZE  | SIZE | SIZE  |
 *----------|------------------------------+------+-------+------+-------+------------------------*/

    {0xA2,   NANDMODEL_DATAWIDTH8  | OPTIONS, 0,    0,       64,      0,  &g_nand_sparescheme2048},
    {0xF2,   NANDMODEL_DATAWIDTH8  | OPTIONS, 0,    0,       64,      0,  &g_nand_sparescheme2048},
    {0xB2,   NANDMODEL_DATAWIDTH16 | OPTIONS, 0,    0,       64,      0,  &g_nand_sparescheme2048},
    {0xC2,   NANDMODEL_DATAWIDTH16 | OPTIONS, 0,    0,       64,      0,  &g_nand_sparescheme2048},

/*----------|------------------------------+------+-------+------+-------+------------------------*/

    {0xA1,   NANDMODEL_DATAWIDTH8  | OPTIONS, 0,    0,      128,      0,  &g_nand_sparescheme2048},
    {0xF1,   NANDMODEL_DATAWIDTH8  | OPTIONS, 0,    0,      128,      0,  &g_nand_sparescheme2048},
    {0xB1,   NANDMODEL_DATAWIDTH16 | OPTIONS, 0,    0,      128,      0,  &g_nand_sparescheme2048},
    {0xC1,   NANDMODEL_DATAWIDTH16 | OPTIONS, 0,    0,      128,      0,  &g_nand_sparescheme2048},

/*----------|------------------------------+------+-------+------+-------+------------------------*/

    {0xAA,   NANDMODEL_DATAWIDTH8  | OPTIONS, 0,    0,      256,      0,  &g_nand_sparescheme2048},
    {0xDA,   NANDMODEL_DATAWIDTH8  | OPTIONS, 0,    0,      256,      0,  &g_nand_sparescheme2048},
    {0xBA,   NANDMODEL_DATAWIDTH16 | OPTIONS, 0,    0,      256,      0,  &g_nand_sparescheme2048},
    {0xCA,   NANDMODEL_DATAWIDTH16 | OPTIONS, 0,    0,      256,      0,  &g_nand_sparescheme2048},

/*----------|------------------------------+------+-------+------+-------+------------------------*/

    {0xAC,   NANDMODEL_DATAWIDTH8  | OPTIONS, 0,    0,      512,      0,  &g_nand_sparescheme2048},
    {0xDC,   NANDMODEL_DATAWIDTH8  | OPTIONS, 0,    0,      512,      0,  &g_nand_sparescheme2048},
    {0xBC,   NANDMODEL_DATAWIDTH16 | OPTIONS, 0,    0,      512,      0,  &g_nand_sparescheme2048},
    {0xCC,   NANDMODEL_DATAWIDTH16 | OPTIONS, 0,    0,      512,      0,  &g_nand_sparescheme2048},

/*----------|------------------------------+------+-------+------+-------+------------------------*/

    {0xA3,   NANDMODEL_DATAWIDTH8  | OPTIONS, 0,    0,     1024,      0,  &g_nand_sparescheme2048},
    {0xD3,   NANDMODEL_DATAWIDTH8  | OPTIONS, 0,    0,     1024,      0,  &g_nand_sparescheme2048},
    {0xB3,   NANDMODEL_DATAWIDTH16 | OPTIONS, 0,    0,     1024,      0,  &g_nand_sparescheme2048},
    {0xC3,   NANDMODEL_DATAWIDTH16 | OPTIONS, 0,    0,     1024,      0,  &g_nand_sparescheme2048},

/*----------|------------------------------+------+-------+------+-------+------------------------*/

    {0xA5,   NANDMODEL_DATAWIDTH8  | OPTIONS, 0,    0,     2048,      0,  &g_nand_sparescheme2048},
    {0xD5,   NANDMODEL_DATAWIDTH8  | OPTIONS, 0,    0,     2048,      0,  &g_nand_sparescheme2048},
    {0xB5,   NANDMODEL_DATAWIDTH16 | OPTIONS, 0,    0,     2048,      0,  &g_nand_sparescheme2048},
    {0xC5,   NANDMODEL_DATAWIDTH16 | OPTIONS, 0,    0,     2048,      0,  &g_nand_sparescheme2048},
    {0x38,   NANDMODEL_DATAWIDTH8  | OPTIONS, 0,    0,     1024,      0,  &g_nand_sparescheme4096}

/*----------|------------------------------+------+-------+------+-------+------------------------*/
};
