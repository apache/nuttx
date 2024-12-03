/****************************************************************************
 * arch/x86_64/include/acpi.h
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

#ifndef __ARCH_X86_64_INCLUDE_ACPI_H
#define __ARCH_X86_64_INCLUDE_ACPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACPI_SIG_RSDP "RSD PTR " /* Root System Description Pointer */

/* Tables defined by ACPI spec */

#define ACPI_SIG_APIC "APIC"    /* Multiple APIC Description Table (MADT) */
#define ACPI_SIG_BERT "BERT"
#define ACPI_SIG_BGRT "BGRT"
#define ACPI_SIG_CCEL "CCEL"
#define ACPI_SIG_CPEP "CPEP"
#define ACPI_SIG_DSDT "DSDT"
#define ACPI_SIG_ECDT "ECDT"
#define ACPI_SIG_EINJ "EINJ"
#define ACPI_SIG_ERST "ERST"
#define ACPI_SIG_FACP "FACP"
#define ACPI_SIG_FACS "FACS"
#define ACPI_SIG_FPDT "FPDT"
#define ACPI_SIG_GTDT "GTDT"
#define ACPI_SIG_HEST "HEST"
#define ACPI_SIG_MISC "MISC"
#define ACPI_SIG_MSCT "MSCT"
#define ACPI_SIG_MPST "MPST"
#define ACPI_SIG_NFIT "NFIT"
#define ACPI_SIG_OEMx "OEMx"
#define ACPI_SIG_PCCT "PCCT"
#define ACPI_SIG_PHAT "PHAT"
#define ACPI_SIG_PMTT "PMTT"
#define ACPI_SIG_PPTT "PPTT"
#define ACPI_SIG_PSDT "PSDT"
#define ACPI_SIG_RASF "RASF"
#define ACPI_SIG_RAS2 "RAS2"
#define ACPI_SIG_RSDT "RSDT"
#define ACPI_SIG_SBST "SBST"
#define ACPI_SIG_SDEV "SDEV"
#define ACPI_SIG_SLIT "SLIT"
#define ACPI_SIG_SRAT "SRAT"
#define ACPI_SIG_SSDT "SSDT"
#define ACPI_SIG_SVKL "SVKL"
#define ACPI_SIG_XSDT "XSDT"

/* Tables not defined by ACPI spec */

#define ACPI_SIG_AEST "AEST"
#define ACPI_SIG_AGDI "AGDI"
#define ACPI_SIG_APMT "APMT"
#define ACPI_SIG_BDAT "BDAT"
#define ACPI_SIG_BOOT "BOOT"
#define ACPI_SIG_CEDT "CEDT"
#define ACPI_SIG_CSRT "CSRT"
#define ACPI_SIG_DBGT "DBGT"
#define ACPI_SIG_DBG2 "DBG2"
#define ACPI_SIG_DMAR "DMAR"
#define ACPI_SIG_DRTM "DRTM"
#define ACPI_SIG_DTPR "DTPR"
#define ACPI_SIG_ETDT "ETDT"
#define ACPI_SIG_HPET "HPET"
#define ACPI_SIG_IBFT "IBFT"
#define ACPI_SIG_IERS "IERS"
#define ACPI_SIG_IORT "IORT"
#define ACPI_SIG_IVRS "IVRS"
#define ACPI_SIG_KEYP "KEYP"
#define ACPI_SIG_LPIT "LPIT"
#define ACPI_SIG_MCFG "MCFG"     /* PCI Express Memory-mapped Configuration table */
#define ACPI_SIG_MCHI "MCHI"
#define ACPI_SIG_MHSP "MHSP"
#define ACPI_SIG_MPAM "MPAM"
#define ACPI_SIG_MSDM "MSDM"
#define ACPI_SIG_NBFT "NBFT"
#define ACPI_SIG_PRMT "PRMT"
#define ACPI_SIG_RGRT "RGRT"
#define ACPI_SIG_SDEI "SDEI"
#define ACPI_SIG_SLIC "SLIC"
#define ACPI_SIG_SPCR "SPCR"
#define ACPI_SIG_SPMI "SPMI"
#define ACPI_SIG_STAO "STAO"
#define ACPI_SIG_SWFT "SWFT"
#define ACPI_SIG_TCPA "TCPA"
#define ACPI_SIG_TPM2 "TPM2"
#define ACPI_SIG_UEFI "UEFI"
#define ACPI_SIG_WAET "WAET"
#define ACPI_SIG_WDAT "WDAT"
#define ACPI_SIG_WDDT "WDDT"
#define ACPI_SIG_WDRT "WDRT"
#define ACPI_SIG_WPBT "WPBT"
#define ACPI_SIG_WSMT "WSMT"
#define ACPI_SIG_XENV "XENV"

/* MADT Interrupt Controller Structure types */

#define ACPI_MADT_TYPE_LOCAL_APIC   (0)
#define ACPI_MADT_TYPE_IO_APIC      (1)
#define ACPI_MADT_TYPE_LOCAL_APIC64 (5)
#define ACPI_MADT_TYPE_LOCAL_X2APIC (9)

/* Local APIC Flags */

#define ACPI_LAPIC_FLAGS_ENABLED     (1 << 0)
#define ACPI_LAPIC_FLAGS_ONLINECAP   (1 << 1)
#define ACPI_LAPIC_FLAGS_RESERVED    (0xfffffffc)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Root System Description Pointer (RSDP) Structure */

begin_packed_struct struct acpi_rsdp_s
{
  char     signature[8];     /* "RSD PTR " sugnature */
  uint8_t  checksum;         /* ACPI 1.0 checksum */
  char     oem_id[6];        /* An OEM-supplied string */
  uint8_t  revision;         /* The revision of this structure */
  uint32_t rsdt_addr;        /* 32 bit physical address of the RSDT */
  uint32_t length;           /* The length of the table */
  uint64_t xsdt_addr;        /* 64 bit physical address of the XSDT */
  uint8_t  ext_checksum;     /* Extended checksum */
  uint8_t  reserved[3];      /* Reserved field */
} end_packed_struct;

/* System Description Table Header */

begin_packed_struct struct acpi_sdt_s
{
  char     signature[4];        /* Table ASCII identifier */
  uint32_t length;              /* The length of the table in bytes */
  uint8_t  revision;            /* The revision of the structure */
  uint8_t  checksum;            /* The entire table checksum */
  char     oem_id[6];           /* An OEM identification string */
  char     oem_table_id[8];     /* An OEM data table string */
  uint32_t oem_revision;        /* An OEM revision number */
  uint32_t creator_id;          /* Vendor ID */
  uint32_t creator_revision;    /* Vendor revision */
} end_packed_struct;

/* Root System Description Table */

begin_packed_struct struct acpi_rsdt_s
{
  struct acpi_sdt_s sdt;
  uint32_t          table_ptrs;
} end_packed_struct;

/* Extended System Descriptior Table */

begin_packed_struct struct acpi_xsdt_s
{
  struct acpi_sdt_s sdt;
  uint64_t          table_ptrs;
} end_packed_struct;

/* Common structure for tables entry */

begin_packed_struct struct acpi_entry_s
{
  uint8_t type;
  uint8_t length;
} end_packed_struct;

/* Multiple APIC Description Table */

begin_packed_struct struct acpi_madt_s
{
  struct acpi_sdt_s   sdt;
  uint32_t            loapic;
  uint32_t            flags;
  struct acpi_entry_s entries;
} end_packed_struct;

/* Multiple APIC Description Table */

begin_packed_struct struct acpi_lapic_s
{
  struct acpi_entry_s entry;
  uint8_t             acpi_id;
  uint8_t             apic_id;
  uint8_t             flags;
} end_packed_struct;

/* Configuration space base address allocation structure */

begin_packed_struct struct acpi_pciseg_s
{
  uint64_t base_addr;           /* Base address */
  uint16_t seg_group_num;       /* PCI Segment Group Number */
  uint8_t  start_bus;           /* Strt PCI bus number */
  uint8_t  end_bus;             /* End PCI bus number */
  uint32_t reserved;            /* Reserved */
} end_packed_struct;

/* PCI Express Memory-mapped Configuration Table */

begin_packed_struct struct acpi_mcfg_s
{
  struct acpi_sdt_s    sdt;       /* Header */
  uint64_t             reserved;  /* Reserved */
  struct acpi_pciseg_s segs;      /* Configuration space base addresses */
} end_packed_struct;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: acpi_init
 *
 * Description:
 *   Initialize ACPI parser.
 *
 ****************************************************************************/

int acpi_init(uintptr_t rsdp);

/****************************************************************************
 * Name: acpi_madt_get
 *
 * Description:
 *   Find the n'th occurence of a MADT entry with a given type.
 *
 ****************************************************************************/

int acpi_madt_get(int type, int n, struct acpi_entry_s **entry);

/****************************************************************************
 * Name: acpi_lapi_get
 *
 * Description:
 *   Get Local APIC entry for a given CPU.
 *
 ****************************************************************************/

int acpi_lapic_get(int cpu, struct acpi_lapic_s **lapic);

#ifdef CONFIG_ARCH_X86_64_ACPI_DUMP
/****************************************************************************
 * Name: acpi_dump
 *
 * Description:
 *   Dump ACPI tables.
 *
 ****************************************************************************/

void acpi_dump(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_X86_64_INCLUDE_ACPI_H */
