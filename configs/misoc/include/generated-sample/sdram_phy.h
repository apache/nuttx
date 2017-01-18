#ifndef __GENERATED_SDRAM_PHY_H
#define __GENERATED_SDRAM_PHY_H
#include <hw/common.h>
#include <generated/csr.h>
#include <hw/flags.h>

#define DFII_NPHASES 4

static void cdelay(int i);

static void command_p0(int cmd)
{
    sdram_dfii_pi0_command_write(cmd);
    sdram_dfii_pi0_command_issue_write(1);
}
static void command_p1(int cmd)
{
    sdram_dfii_pi1_command_write(cmd);
    sdram_dfii_pi1_command_issue_write(1);
}
static void command_p2(int cmd)
{
    sdram_dfii_pi2_command_write(cmd);
    sdram_dfii_pi2_command_issue_write(1);
}
static void command_p3(int cmd)
{
    sdram_dfii_pi3_command_write(cmd);
    sdram_dfii_pi3_command_issue_write(1);
}


#define sdram_dfii_pird_address_write(X) sdram_dfii_pi0_address_write(X)
#define sdram_dfii_piwr_address_write(X) sdram_dfii_pi2_address_write(X)

#define sdram_dfii_pird_baddress_write(X) sdram_dfii_pi0_baddress_write(X)
#define sdram_dfii_piwr_baddress_write(X) sdram_dfii_pi2_baddress_write(X)

#define command_prd(X) command_p0(X)
#define command_pwr(X) command_p2(X)

#define DFII_PIX_DATA_SIZE CSR_SDRAM_DFII_PI0_WRDATA_SIZE

const unsigned int sdram_dfii_pix_wrdata_addr[4] = {
    CSR_SDRAM_DFII_PI0_WRDATA_ADDR,
	CSR_SDRAM_DFII_PI1_WRDATA_ADDR,
	CSR_SDRAM_DFII_PI2_WRDATA_ADDR,
	CSR_SDRAM_DFII_PI3_WRDATA_ADDR
};

const unsigned int sdram_dfii_pix_rddata_addr[4] = {
    CSR_SDRAM_DFII_PI0_RDDATA_ADDR,
	CSR_SDRAM_DFII_PI1_RDDATA_ADDR,
	CSR_SDRAM_DFII_PI2_RDDATA_ADDR,
	CSR_SDRAM_DFII_PI3_RDDATA_ADDR
};

#define DDR3_MR1 6

static void init_sequence(void)
{
	/* Release reset */
	sdram_dfii_pi0_address_write(0x0);
	sdram_dfii_pi0_baddress_write(0);
	sdram_dfii_control_write(DFII_CONTROL_ODT|DFII_CONTROL_RESET_N);
	cdelay(50000);

	/* Bring CKE high */
	sdram_dfii_pi0_address_write(0x0);
	sdram_dfii_pi0_baddress_write(0);
	sdram_dfii_control_write(DFII_CONTROL_CKE|DFII_CONTROL_ODT|DFII_CONTROL_RESET_N);
	cdelay(10000);

	/* Load Mode Register 2 */
	sdram_dfii_pi0_address_write(0x408);
	sdram_dfii_pi0_baddress_write(2);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);

	/* Load Mode Register 3 */
	sdram_dfii_pi0_address_write(0x0);
	sdram_dfii_pi0_baddress_write(3);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);

	/* Load Mode Register 1 */
	sdram_dfii_pi0_address_write(0x6);
	sdram_dfii_pi0_baddress_write(1);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);

	/* Load Mode Register 0, CL=7, BL=8 */
	sdram_dfii_pi0_address_write(0x930);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_RAS|DFII_COMMAND_CAS|DFII_COMMAND_WE|DFII_COMMAND_CS);
	cdelay(200);

	/* ZQ Calibration */
	sdram_dfii_pi0_address_write(0x400);
	sdram_dfii_pi0_baddress_write(0);
	command_p0(DFII_COMMAND_WE|DFII_COMMAND_CS);
	cdelay(200);

}
#endif
