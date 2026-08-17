/****************************************************************************
 * boards/arm/imxrt/frdm-imxrt1186/src/imxrt_flexspi_nor_xmcd.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * External Memory Configuration Data for Boot ROM.  Same option words as
 * Zephyr boards/nxp/frdm_imxrt1186 (HyperRAM on FlexSPI1).  Without this
 * block at flash offset 0x800, and without EXTERN(g_xmcd_data) so the
 * archive object is pulled in, the XMCD slot stays erased (0xff).
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

locate_data(".boot_hdr.xmcd_data")
const uint32_t g_xmcd_data[] =
{
  0xc001000c, /* FlexSPI instance 1 */
  0xc1001800, /* Option words = 2 */
  0x10000000  /* PORTB */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
