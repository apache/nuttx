/****************************************************************************
 * drivers/mtd/cfi.h
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

#ifndef __DRIVERS_MTD_CFI_H
#define __DRIVERS_MTD_CFI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/mtd/mtd.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct cfi_info_s
{
  uint8_t qry[3];                   /* query-unique ASCII string 'QRY' */
  uint16_t p_id;                    /* primary vendor command set and control
                                     * interface id */
  uint16_t p_addr;                  /* primary algorithm extended query table
                                     * address */
  uint16_t a_id;                    /* alternative vendor command set and
                                     * control interface id */
  uint16_t a_addr;                  /* alternative algorithm extended query
                                     * table address */
  uint8_t vccmin;                   /* Vcc logic supply minimum program/erase
                                     * or write voltage */
  uint8_t vccmax;                   /* Vcc logic supply maximum program/erase
                                     * or write voltage */
  uint8_t vppmin;                   /* Vpp logic supply minimum program/erase
                                     * or write voltage */
  uint8_t vppmax;                   /* Vpp logic supply maximum program/erase
                                     * or write voltage */
  uint8_t single_write_timeout_typ; /* typical timeout per single byte/word/
                                     * D-word program, 2^N us */
  uint8_t buffer_write_timeout_typ; /* typical timeout for maximum-size
                                     * multi-byte program, 2^N us */
  uint8_t block_erase_timeout_typ;  /* typical timeout per individual block
                                     * erase, 2^N ms */
  uint8_t chip_erase_timeout_typ;   /* typical timeout for full chip erase,
                                     * 2^N ms */
  uint8_t single_write_timeout_max; /* maximum timeout per single byte/word/
                                     * D-word program, 2^N times typical */
  uint8_t buffer_write_timeout_max; /* maximum timeout for multi-byte program,
                                     * 2^N times typical */
  uint8_t block_erase_timeout_max;  /* maximum timeout per individual block
                                     * erase, 2^N times typical */
  uint8_t chip_erase_timeout_max;   /* typical timeout for full chip erase,
                                     * 2^N times typical */
  uint8_t device_size;              /* device size = 2^N in number of bytes */
  uint16_t interface_desc;          /* flash device interface code
                                     * description */
  uint16_t max_write_bytes_num;     /* maximum number of bytes in multi-byte
                                     * program = 2^N */
  uint8_t erase_region_num;         /* number of erase block regions */
  uint32_t erase_region_info[4];    /* erase block region information
                                     * bits 31-16 = z, where the erase blocks
                                     * within this region are z times 256
                                     * bytes.
                                     * bits 15-0 = y, where y + 1 = number of
                                     * erase blocks within the erase block
                                     * region. */
}end_packed_struct;

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct skel_dev_s.
 */

struct cfi_dev_s
{
  /* Publicly visible representation of the interface */

  struct mtd_dev_s mtd;

  /* Other implementation specific data may follow here */

  uintptr_t base_addr;        /* base address of cfi flash */
  uintptr_t end_addr;         /* end address of cfi flash */
  size_t page_size;           /* page size for read and write */
  uint8_t bankwidth;          /* port width of the whole device */
  uint8_t dev_width;          /* port width of single chip */
  uint8_t dev_num;            /* equals to bankwith divided by dev_width */
  uint8_t cfi_offset;         /* cfi offset address, 0x55 or 0x555 */
  uint32_t unlock_addr1;      /* unlock addr1 for amd */
  uint32_t unlock_addr2;      /* unlock addr2 for amd */
  struct cfi_info_s info;     /* struct of cfi information */
};

/****************************************************************************
 * Public function prototypes
 ****************************************************************************/

size_t cfi_get_blocksize(FAR struct cfi_dev_s *cfi, uint8_t region);
blkcnt_t cfi_get_blocknum(FAR struct cfi_dev_s *cfi, uint8_t region);
blkcnt_t cfi_find_block(FAR struct cfi_dev_s *cfi, off_t offset);
blkcnt_t cfi_get_total_blocknum(FAR struct cfi_dev_s *cfi);
int cfi_check(FAR struct cfi_dev_s *cfi);
void cfi_reset(FAR struct cfi_dev_s *cfi);
int cfi_erase(FAR struct cfi_dev_s *cfi, blkcnt_t startblock,
              blkcnt_t blockcnt);
int cfi_read(FAR struct cfi_dev_s *cfi, off_t offset, size_t nbytes,
             FAR uint8_t *buffer);
int cfi_write(FAR struct cfi_dev_s *cfi, off_t offset, size_t nbytes,
              FAR const uint8_t *buffer);

#endif  /* __DRIVERS_MTD_CFI_H */
