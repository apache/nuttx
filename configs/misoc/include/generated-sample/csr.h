#ifndef __GENERATED_CSR_H
#define __GENERATED_CSR_H
#include <hw/common.h>

/* ddrphy */
#define CSR_DDRPHY_BASE 0xe0008800
#define CSR_DDRPHY_DLY_SEL_ADDR 0xe0008800
#define CSR_DDRPHY_DLY_SEL_SIZE 1
static inline unsigned char ddrphy_dly_sel_read(void) {
	unsigned char r = MMPTR(0xe0008800);
	return r;
}
static inline void ddrphy_dly_sel_write(unsigned char value) {
	MMPTR(0xe0008800) = value;
}
#define CSR_DDRPHY_RDLY_DQ_RST_ADDR 0xe0008804
#define CSR_DDRPHY_RDLY_DQ_RST_SIZE 1
static inline unsigned char ddrphy_rdly_dq_rst_read(void) {
	unsigned char r = MMPTR(0xe0008804);
	return r;
}
static inline void ddrphy_rdly_dq_rst_write(unsigned char value) {
	MMPTR(0xe0008804) = value;
}
#define CSR_DDRPHY_RDLY_DQ_INC_ADDR 0xe0008808
#define CSR_DDRPHY_RDLY_DQ_INC_SIZE 1
static inline unsigned char ddrphy_rdly_dq_inc_read(void) {
	unsigned char r = MMPTR(0xe0008808);
	return r;
}
static inline void ddrphy_rdly_dq_inc_write(unsigned char value) {
	MMPTR(0xe0008808) = value;
}
#define CSR_DDRPHY_RDLY_DQ_BITSLIP_ADDR 0xe000880c
#define CSR_DDRPHY_RDLY_DQ_BITSLIP_SIZE 1
static inline unsigned char ddrphy_rdly_dq_bitslip_read(void) {
	unsigned char r = MMPTR(0xe000880c);
	return r;
}
static inline void ddrphy_rdly_dq_bitslip_write(unsigned char value) {
	MMPTR(0xe000880c) = value;
}

/* ethmac */
#define CSR_ETHMAC_BASE 0xe000f800
#define CSR_ETHMAC_SRAM_WRITER_SLOT_ADDR 0xe000f800
#define CSR_ETHMAC_SRAM_WRITER_SLOT_SIZE 1
static inline unsigned char ethmac_sram_writer_slot_read(void) {
	unsigned char r = MMPTR(0xe000f800);
	return r;
}
#define CSR_ETHMAC_SRAM_WRITER_LENGTH_ADDR 0xe000f804
#define CSR_ETHMAC_SRAM_WRITER_LENGTH_SIZE 4
static inline unsigned int ethmac_sram_writer_length_read(void) {
	unsigned int r = MMPTR(0xe000f804);
	r <<= 8;
	r |= MMPTR(0xe000f808);
	r <<= 8;
	r |= MMPTR(0xe000f80c);
	r <<= 8;
	r |= MMPTR(0xe000f810);
	return r;
}
#define CSR_ETHMAC_SRAM_WRITER_EV_STATUS_ADDR 0xe000f814
#define CSR_ETHMAC_SRAM_WRITER_EV_STATUS_SIZE 1
static inline unsigned char ethmac_sram_writer_ev_status_read(void) {
	unsigned char r = MMPTR(0xe000f814);
	return r;
}
static inline void ethmac_sram_writer_ev_status_write(unsigned char value) {
	MMPTR(0xe000f814) = value;
}
#define CSR_ETHMAC_SRAM_WRITER_EV_PENDING_ADDR 0xe000f818
#define CSR_ETHMAC_SRAM_WRITER_EV_PENDING_SIZE 1
static inline unsigned char ethmac_sram_writer_ev_pending_read(void) {
	unsigned char r = MMPTR(0xe000f818);
	return r;
}
static inline void ethmac_sram_writer_ev_pending_write(unsigned char value) {
	MMPTR(0xe000f818) = value;
}
#define CSR_ETHMAC_SRAM_WRITER_EV_ENABLE_ADDR 0xe000f81c
#define CSR_ETHMAC_SRAM_WRITER_EV_ENABLE_SIZE 1
static inline unsigned char ethmac_sram_writer_ev_enable_read(void) {
	unsigned char r = MMPTR(0xe000f81c);
	return r;
}
static inline void ethmac_sram_writer_ev_enable_write(unsigned char value) {
	MMPTR(0xe000f81c) = value;
}
#define CSR_ETHMAC_SRAM_READER_START_ADDR 0xe000f820
#define CSR_ETHMAC_SRAM_READER_START_SIZE 1
static inline unsigned char ethmac_sram_reader_start_read(void) {
	unsigned char r = MMPTR(0xe000f820);
	return r;
}
static inline void ethmac_sram_reader_start_write(unsigned char value) {
	MMPTR(0xe000f820) = value;
}
#define CSR_ETHMAC_SRAM_READER_READY_ADDR 0xe000f824
#define CSR_ETHMAC_SRAM_READER_READY_SIZE 1
static inline unsigned char ethmac_sram_reader_ready_read(void) {
	unsigned char r = MMPTR(0xe000f824);
	return r;
}
#define CSR_ETHMAC_SRAM_READER_SLOT_ADDR 0xe000f828
#define CSR_ETHMAC_SRAM_READER_SLOT_SIZE 1
static inline unsigned char ethmac_sram_reader_slot_read(void) {
	unsigned char r = MMPTR(0xe000f828);
	return r;
}
static inline void ethmac_sram_reader_slot_write(unsigned char value) {
	MMPTR(0xe000f828) = value;
}
#define CSR_ETHMAC_SRAM_READER_LENGTH_ADDR 0xe000f82c
#define CSR_ETHMAC_SRAM_READER_LENGTH_SIZE 2
static inline unsigned short int ethmac_sram_reader_length_read(void) {
	unsigned short int r = MMPTR(0xe000f82c);
	r <<= 8;
	r |= MMPTR(0xe000f830);
	return r;
}
static inline void ethmac_sram_reader_length_write(unsigned short int value) {
	MMPTR(0xe000f82c) = value >> 8;
	MMPTR(0xe000f830) = value;
}
#define CSR_ETHMAC_SRAM_READER_EV_STATUS_ADDR 0xe000f834
#define CSR_ETHMAC_SRAM_READER_EV_STATUS_SIZE 1
static inline unsigned char ethmac_sram_reader_ev_status_read(void) {
	unsigned char r = MMPTR(0xe000f834);
	return r;
}
static inline void ethmac_sram_reader_ev_status_write(unsigned char value) {
	MMPTR(0xe000f834) = value;
}
#define CSR_ETHMAC_SRAM_READER_EV_PENDING_ADDR 0xe000f838
#define CSR_ETHMAC_SRAM_READER_EV_PENDING_SIZE 1
static inline unsigned char ethmac_sram_reader_ev_pending_read(void) {
	unsigned char r = MMPTR(0xe000f838);
	return r;
}
static inline void ethmac_sram_reader_ev_pending_write(unsigned char value) {
	MMPTR(0xe000f838) = value;
}
#define CSR_ETHMAC_SRAM_READER_EV_ENABLE_ADDR 0xe000f83c
#define CSR_ETHMAC_SRAM_READER_EV_ENABLE_SIZE 1
static inline unsigned char ethmac_sram_reader_ev_enable_read(void) {
	unsigned char r = MMPTR(0xe000f83c);
	return r;
}
static inline void ethmac_sram_reader_ev_enable_write(unsigned char value) {
	MMPTR(0xe000f83c) = value;
}
#define CSR_ETHMAC_PREAMBLE_CRC_ADDR 0xe000f840
#define CSR_ETHMAC_PREAMBLE_CRC_SIZE 1
static inline unsigned char ethmac_preamble_crc_read(void) {
	unsigned char r = MMPTR(0xe000f840);
	return r;
}

/* ethphy */
#define CSR_ETHPHY_BASE 0xe000f000
#define CSR_ETHPHY_CRG_RESET_ADDR 0xe000f000
#define CSR_ETHPHY_CRG_RESET_SIZE 1
static inline unsigned char ethphy_crg_reset_read(void) {
	unsigned char r = MMPTR(0xe000f000);
	return r;
}
static inline void ethphy_crg_reset_write(unsigned char value) {
	MMPTR(0xe000f000) = value;
}
#define CSR_ETHPHY_MDIO_W_ADDR 0xe000f004
#define CSR_ETHPHY_MDIO_W_SIZE 1
static inline unsigned char ethphy_mdio_w_read(void) {
	unsigned char r = MMPTR(0xe000f004);
	return r;
}
static inline void ethphy_mdio_w_write(unsigned char value) {
	MMPTR(0xe000f004) = value;
}
#define CSR_ETHPHY_MDIO_R_ADDR 0xe000f008
#define CSR_ETHPHY_MDIO_R_SIZE 1
static inline unsigned char ethphy_mdio_r_read(void) {
	unsigned char r = MMPTR(0xe000f008);
	return r;
}

/* sdram */
#define CSR_SDRAM_BASE 0xe0004000
#define CSR_SDRAM_DFII_CONTROL_ADDR 0xe0004000
#define CSR_SDRAM_DFII_CONTROL_SIZE 1
static inline unsigned char sdram_dfii_control_read(void) {
	unsigned char r = MMPTR(0xe0004000);
	return r;
}
static inline void sdram_dfii_control_write(unsigned char value) {
	MMPTR(0xe0004000) = value;
}
#define CSR_SDRAM_DFII_PI0_COMMAND_ADDR 0xe0004004
#define CSR_SDRAM_DFII_PI0_COMMAND_SIZE 1
static inline unsigned char sdram_dfii_pi0_command_read(void) {
	unsigned char r = MMPTR(0xe0004004);
	return r;
}
static inline void sdram_dfii_pi0_command_write(unsigned char value) {
	MMPTR(0xe0004004) = value;
}
#define CSR_SDRAM_DFII_PI0_COMMAND_ISSUE_ADDR 0xe0004008
#define CSR_SDRAM_DFII_PI0_COMMAND_ISSUE_SIZE 1
static inline unsigned char sdram_dfii_pi0_command_issue_read(void) {
	unsigned char r = MMPTR(0xe0004008);
	return r;
}
static inline void sdram_dfii_pi0_command_issue_write(unsigned char value) {
	MMPTR(0xe0004008) = value;
}
#define CSR_SDRAM_DFII_PI0_ADDRESS_ADDR 0xe000400c
#define CSR_SDRAM_DFII_PI0_ADDRESS_SIZE 2
static inline unsigned short int sdram_dfii_pi0_address_read(void) {
	unsigned short int r = MMPTR(0xe000400c);
	r <<= 8;
	r |= MMPTR(0xe0004010);
	return r;
}
static inline void sdram_dfii_pi0_address_write(unsigned short int value) {
	MMPTR(0xe000400c) = value >> 8;
	MMPTR(0xe0004010) = value;
}
#define CSR_SDRAM_DFII_PI0_BADDRESS_ADDR 0xe0004014
#define CSR_SDRAM_DFII_PI0_BADDRESS_SIZE 1
static inline unsigned char sdram_dfii_pi0_baddress_read(void) {
	unsigned char r = MMPTR(0xe0004014);
	return r;
}
static inline void sdram_dfii_pi0_baddress_write(unsigned char value) {
	MMPTR(0xe0004014) = value;
}
#define CSR_SDRAM_DFII_PI0_WRDATA_ADDR 0xe0004018
#define CSR_SDRAM_DFII_PI0_WRDATA_SIZE 4
static inline unsigned int sdram_dfii_pi0_wrdata_read(void) {
	unsigned int r = MMPTR(0xe0004018);
	r <<= 8;
	r |= MMPTR(0xe000401c);
	r <<= 8;
	r |= MMPTR(0xe0004020);
	r <<= 8;
	r |= MMPTR(0xe0004024);
	return r;
}
static inline void sdram_dfii_pi0_wrdata_write(unsigned int value) {
	MMPTR(0xe0004018) = value >> 24;
	MMPTR(0xe000401c) = value >> 16;
	MMPTR(0xe0004020) = value >> 8;
	MMPTR(0xe0004024) = value;
}
#define CSR_SDRAM_DFII_PI0_RDDATA_ADDR 0xe0004028
#define CSR_SDRAM_DFII_PI0_RDDATA_SIZE 4
static inline unsigned int sdram_dfii_pi0_rddata_read(void) {
	unsigned int r = MMPTR(0xe0004028);
	r <<= 8;
	r |= MMPTR(0xe000402c);
	r <<= 8;
	r |= MMPTR(0xe0004030);
	r <<= 8;
	r |= MMPTR(0xe0004034);
	return r;
}
#define CSR_SDRAM_DFII_PI1_COMMAND_ADDR 0xe0004038
#define CSR_SDRAM_DFII_PI1_COMMAND_SIZE 1
static inline unsigned char sdram_dfii_pi1_command_read(void) {
	unsigned char r = MMPTR(0xe0004038);
	return r;
}
static inline void sdram_dfii_pi1_command_write(unsigned char value) {
	MMPTR(0xe0004038) = value;
}
#define CSR_SDRAM_DFII_PI1_COMMAND_ISSUE_ADDR 0xe000403c
#define CSR_SDRAM_DFII_PI1_COMMAND_ISSUE_SIZE 1
static inline unsigned char sdram_dfii_pi1_command_issue_read(void) {
	unsigned char r = MMPTR(0xe000403c);
	return r;
}
static inline void sdram_dfii_pi1_command_issue_write(unsigned char value) {
	MMPTR(0xe000403c) = value;
}
#define CSR_SDRAM_DFII_PI1_ADDRESS_ADDR 0xe0004040
#define CSR_SDRAM_DFII_PI1_ADDRESS_SIZE 2
static inline unsigned short int sdram_dfii_pi1_address_read(void) {
	unsigned short int r = MMPTR(0xe0004040);
	r <<= 8;
	r |= MMPTR(0xe0004044);
	return r;
}
static inline void sdram_dfii_pi1_address_write(unsigned short int value) {
	MMPTR(0xe0004040) = value >> 8;
	MMPTR(0xe0004044) = value;
}
#define CSR_SDRAM_DFII_PI1_BADDRESS_ADDR 0xe0004048
#define CSR_SDRAM_DFII_PI1_BADDRESS_SIZE 1
static inline unsigned char sdram_dfii_pi1_baddress_read(void) {
	unsigned char r = MMPTR(0xe0004048);
	return r;
}
static inline void sdram_dfii_pi1_baddress_write(unsigned char value) {
	MMPTR(0xe0004048) = value;
}
#define CSR_SDRAM_DFII_PI1_WRDATA_ADDR 0xe000404c
#define CSR_SDRAM_DFII_PI1_WRDATA_SIZE 4
static inline unsigned int sdram_dfii_pi1_wrdata_read(void) {
	unsigned int r = MMPTR(0xe000404c);
	r <<= 8;
	r |= MMPTR(0xe0004050);
	r <<= 8;
	r |= MMPTR(0xe0004054);
	r <<= 8;
	r |= MMPTR(0xe0004058);
	return r;
}
static inline void sdram_dfii_pi1_wrdata_write(unsigned int value) {
	MMPTR(0xe000404c) = value >> 24;
	MMPTR(0xe0004050) = value >> 16;
	MMPTR(0xe0004054) = value >> 8;
	MMPTR(0xe0004058) = value;
}
#define CSR_SDRAM_DFII_PI1_RDDATA_ADDR 0xe000405c
#define CSR_SDRAM_DFII_PI1_RDDATA_SIZE 4
static inline unsigned int sdram_dfii_pi1_rddata_read(void) {
	unsigned int r = MMPTR(0xe000405c);
	r <<= 8;
	r |= MMPTR(0xe0004060);
	r <<= 8;
	r |= MMPTR(0xe0004064);
	r <<= 8;
	r |= MMPTR(0xe0004068);
	return r;
}
#define CSR_SDRAM_DFII_PI2_COMMAND_ADDR 0xe000406c
#define CSR_SDRAM_DFII_PI2_COMMAND_SIZE 1
static inline unsigned char sdram_dfii_pi2_command_read(void) {
	unsigned char r = MMPTR(0xe000406c);
	return r;
}
static inline void sdram_dfii_pi2_command_write(unsigned char value) {
	MMPTR(0xe000406c) = value;
}
#define CSR_SDRAM_DFII_PI2_COMMAND_ISSUE_ADDR 0xe0004070
#define CSR_SDRAM_DFII_PI2_COMMAND_ISSUE_SIZE 1
static inline unsigned char sdram_dfii_pi2_command_issue_read(void) {
	unsigned char r = MMPTR(0xe0004070);
	return r;
}
static inline void sdram_dfii_pi2_command_issue_write(unsigned char value) {
	MMPTR(0xe0004070) = value;
}
#define CSR_SDRAM_DFII_PI2_ADDRESS_ADDR 0xe0004074
#define CSR_SDRAM_DFII_PI2_ADDRESS_SIZE 2
static inline unsigned short int sdram_dfii_pi2_address_read(void) {
	unsigned short int r = MMPTR(0xe0004074);
	r <<= 8;
	r |= MMPTR(0xe0004078);
	return r;
}
static inline void sdram_dfii_pi2_address_write(unsigned short int value) {
	MMPTR(0xe0004074) = value >> 8;
	MMPTR(0xe0004078) = value;
}
#define CSR_SDRAM_DFII_PI2_BADDRESS_ADDR 0xe000407c
#define CSR_SDRAM_DFII_PI2_BADDRESS_SIZE 1
static inline unsigned char sdram_dfii_pi2_baddress_read(void) {
	unsigned char r = MMPTR(0xe000407c);
	return r;
}
static inline void sdram_dfii_pi2_baddress_write(unsigned char value) {
	MMPTR(0xe000407c) = value;
}
#define CSR_SDRAM_DFII_PI2_WRDATA_ADDR 0xe0004080
#define CSR_SDRAM_DFII_PI2_WRDATA_SIZE 4
static inline unsigned int sdram_dfii_pi2_wrdata_read(void) {
	unsigned int r = MMPTR(0xe0004080);
	r <<= 8;
	r |= MMPTR(0xe0004084);
	r <<= 8;
	r |= MMPTR(0xe0004088);
	r <<= 8;
	r |= MMPTR(0xe000408c);
	return r;
}
static inline void sdram_dfii_pi2_wrdata_write(unsigned int value) {
	MMPTR(0xe0004080) = value >> 24;
	MMPTR(0xe0004084) = value >> 16;
	MMPTR(0xe0004088) = value >> 8;
	MMPTR(0xe000408c) = value;
}
#define CSR_SDRAM_DFII_PI2_RDDATA_ADDR 0xe0004090
#define CSR_SDRAM_DFII_PI2_RDDATA_SIZE 4
static inline unsigned int sdram_dfii_pi2_rddata_read(void) {
	unsigned int r = MMPTR(0xe0004090);
	r <<= 8;
	r |= MMPTR(0xe0004094);
	r <<= 8;
	r |= MMPTR(0xe0004098);
	r <<= 8;
	r |= MMPTR(0xe000409c);
	return r;
}
#define CSR_SDRAM_DFII_PI3_COMMAND_ADDR 0xe00040a0
#define CSR_SDRAM_DFII_PI3_COMMAND_SIZE 1
static inline unsigned char sdram_dfii_pi3_command_read(void) {
	unsigned char r = MMPTR(0xe00040a0);
	return r;
}
static inline void sdram_dfii_pi3_command_write(unsigned char value) {
	MMPTR(0xe00040a0) = value;
}
#define CSR_SDRAM_DFII_PI3_COMMAND_ISSUE_ADDR 0xe00040a4
#define CSR_SDRAM_DFII_PI3_COMMAND_ISSUE_SIZE 1
static inline unsigned char sdram_dfii_pi3_command_issue_read(void) {
	unsigned char r = MMPTR(0xe00040a4);
	return r;
}
static inline void sdram_dfii_pi3_command_issue_write(unsigned char value) {
	MMPTR(0xe00040a4) = value;
}
#define CSR_SDRAM_DFII_PI3_ADDRESS_ADDR 0xe00040a8
#define CSR_SDRAM_DFII_PI3_ADDRESS_SIZE 2
static inline unsigned short int sdram_dfii_pi3_address_read(void) {
	unsigned short int r = MMPTR(0xe00040a8);
	r <<= 8;
	r |= MMPTR(0xe00040ac);
	return r;
}
static inline void sdram_dfii_pi3_address_write(unsigned short int value) {
	MMPTR(0xe00040a8) = value >> 8;
	MMPTR(0xe00040ac) = value;
}
#define CSR_SDRAM_DFII_PI3_BADDRESS_ADDR 0xe00040b0
#define CSR_SDRAM_DFII_PI3_BADDRESS_SIZE 1
static inline unsigned char sdram_dfii_pi3_baddress_read(void) {
	unsigned char r = MMPTR(0xe00040b0);
	return r;
}
static inline void sdram_dfii_pi3_baddress_write(unsigned char value) {
	MMPTR(0xe00040b0) = value;
}
#define CSR_SDRAM_DFII_PI3_WRDATA_ADDR 0xe00040b4
#define CSR_SDRAM_DFII_PI3_WRDATA_SIZE 4
static inline unsigned int sdram_dfii_pi3_wrdata_read(void) {
	unsigned int r = MMPTR(0xe00040b4);
	r <<= 8;
	r |= MMPTR(0xe00040b8);
	r <<= 8;
	r |= MMPTR(0xe00040bc);
	r <<= 8;
	r |= MMPTR(0xe00040c0);
	return r;
}
static inline void sdram_dfii_pi3_wrdata_write(unsigned int value) {
	MMPTR(0xe00040b4) = value >> 24;
	MMPTR(0xe00040b8) = value >> 16;
	MMPTR(0xe00040bc) = value >> 8;
	MMPTR(0xe00040c0) = value;
}
#define CSR_SDRAM_DFII_PI3_RDDATA_ADDR 0xe00040c4
#define CSR_SDRAM_DFII_PI3_RDDATA_SIZE 4
static inline unsigned int sdram_dfii_pi3_rddata_read(void) {
	unsigned int r = MMPTR(0xe00040c4);
	r <<= 8;
	r |= MMPTR(0xe00040c8);
	r <<= 8;
	r |= MMPTR(0xe00040cc);
	r <<= 8;
	r |= MMPTR(0xe00040d0);
	return r;
}

/* timer0 */
#define CSR_TIMER0_BASE 0xe0002000
#define CSR_TIMER0_LOAD_ADDR 0xe0002000
#define CSR_TIMER0_LOAD_SIZE 4
static inline unsigned int timer0_load_read(void) {
	unsigned int r = MMPTR(0xe0002000);
	r <<= 8;
	r |= MMPTR(0xe0002004);
	r <<= 8;
	r |= MMPTR(0xe0002008);
	r <<= 8;
	r |= MMPTR(0xe000200c);
	return r;
}
static inline void timer0_load_write(unsigned int value) {
	MMPTR(0xe0002000) = value >> 24;
	MMPTR(0xe0002004) = value >> 16;
	MMPTR(0xe0002008) = value >> 8;
	MMPTR(0xe000200c) = value;
}
#define CSR_TIMER0_RELOAD_ADDR 0xe0002010
#define CSR_TIMER0_RELOAD_SIZE 4
static inline unsigned int timer0_reload_read(void) {
	unsigned int r = MMPTR(0xe0002010);
	r <<= 8;
	r |= MMPTR(0xe0002014);
	r <<= 8;
	r |= MMPTR(0xe0002018);
	r <<= 8;
	r |= MMPTR(0xe000201c);
	return r;
}
static inline void timer0_reload_write(unsigned int value) {
	MMPTR(0xe0002010) = value >> 24;
	MMPTR(0xe0002014) = value >> 16;
	MMPTR(0xe0002018) = value >> 8;
	MMPTR(0xe000201c) = value;
}
#define CSR_TIMER0_EN_ADDR 0xe0002020
#define CSR_TIMER0_EN_SIZE 1
static inline unsigned char timer0_en_read(void) {
	unsigned char r = MMPTR(0xe0002020);
	return r;
}
static inline void timer0_en_write(unsigned char value) {
	MMPTR(0xe0002020) = value;
}
#define CSR_TIMER0_UPDATE_VALUE_ADDR 0xe0002024
#define CSR_TIMER0_UPDATE_VALUE_SIZE 1
static inline unsigned char timer0_update_value_read(void) {
	unsigned char r = MMPTR(0xe0002024);
	return r;
}
static inline void timer0_update_value_write(unsigned char value) {
	MMPTR(0xe0002024) = value;
}
#define CSR_TIMER0_VALUE_ADDR 0xe0002028
#define CSR_TIMER0_VALUE_SIZE 4
static inline unsigned int timer0_value_read(void) {
	unsigned int r = MMPTR(0xe0002028);
	r <<= 8;
	r |= MMPTR(0xe000202c);
	r <<= 8;
	r |= MMPTR(0xe0002030);
	r <<= 8;
	r |= MMPTR(0xe0002034);
	return r;
}
#define CSR_TIMER0_EV_STATUS_ADDR 0xe0002038
#define CSR_TIMER0_EV_STATUS_SIZE 1
static inline unsigned char timer0_ev_status_read(void) {
	unsigned char r = MMPTR(0xe0002038);
	return r;
}
static inline void timer0_ev_status_write(unsigned char value) {
	MMPTR(0xe0002038) = value;
}
#define CSR_TIMER0_EV_PENDING_ADDR 0xe000203c
#define CSR_TIMER0_EV_PENDING_SIZE 1
static inline unsigned char timer0_ev_pending_read(void) {
	unsigned char r = MMPTR(0xe000203c);
	return r;
}
static inline void timer0_ev_pending_write(unsigned char value) {
	MMPTR(0xe000203c) = value;
}
#define CSR_TIMER0_EV_ENABLE_ADDR 0xe0002040
#define CSR_TIMER0_EV_ENABLE_SIZE 1
static inline unsigned char timer0_ev_enable_read(void) {
	unsigned char r = MMPTR(0xe0002040);
	return r;
}
static inline void timer0_ev_enable_write(unsigned char value) {
	MMPTR(0xe0002040) = value;
}

/* uart */
#define CSR_UART_BASE 0xe0001000
#define CSR_UART_RXTX_ADDR 0xe0001000
#define CSR_UART_RXTX_SIZE 1
static inline unsigned char uart_rxtx_read(void) {
	unsigned char r = MMPTR(0xe0001000);
	return r;
}
static inline void uart_rxtx_write(unsigned char value) {
	MMPTR(0xe0001000) = value;
}
#define CSR_UART_TXFULL_ADDR 0xe0001004
#define CSR_UART_TXFULL_SIZE 1
static inline unsigned char uart_txfull_read(void) {
	unsigned char r = MMPTR(0xe0001004);
	return r;
}
#define CSR_UART_RXEMPTY_ADDR 0xe0001008
#define CSR_UART_RXEMPTY_SIZE 1
static inline unsigned char uart_rxempty_read(void) {
	unsigned char r = MMPTR(0xe0001008);
	return r;
}
#define CSR_UART_EV_STATUS_ADDR 0xe000100c
#define CSR_UART_EV_STATUS_SIZE 1
static inline unsigned char uart_ev_status_read(void) {
	unsigned char r = MMPTR(0xe000100c);
	return r;
}
static inline void uart_ev_status_write(unsigned char value) {
	MMPTR(0xe000100c) = value;
}
#define CSR_UART_EV_PENDING_ADDR 0xe0001010
#define CSR_UART_EV_PENDING_SIZE 1
static inline unsigned char uart_ev_pending_read(void) {
	unsigned char r = MMPTR(0xe0001010);
	return r;
}
static inline void uart_ev_pending_write(unsigned char value) {
	MMPTR(0xe0001010) = value;
}
#define CSR_UART_EV_ENABLE_ADDR 0xe0001014
#define CSR_UART_EV_ENABLE_SIZE 1
static inline unsigned char uart_ev_enable_read(void) {
	unsigned char r = MMPTR(0xe0001014);
	return r;
}
static inline void uart_ev_enable_write(unsigned char value) {
	MMPTR(0xe0001014) = value;
}

/* uart_phy */
#define CSR_UART_PHY_BASE 0xe0000800
#define CSR_UART_PHY_TUNING_WORD_ADDR 0xe0000800
#define CSR_UART_PHY_TUNING_WORD_SIZE 4
static inline unsigned int uart_phy_tuning_word_read(void) {
	unsigned int r = MMPTR(0xe0000800);
	r <<= 8;
	r |= MMPTR(0xe0000804);
	r <<= 8;
	r |= MMPTR(0xe0000808);
	r <<= 8;
	r |= MMPTR(0xe000080c);
	return r;
}
static inline void uart_phy_tuning_word_write(unsigned int value) {
	MMPTR(0xe0000800) = value >> 24;
	MMPTR(0xe0000804) = value >> 16;
	MMPTR(0xe0000808) = value >> 8;
	MMPTR(0xe000080c) = value;
}

/* constants */
#define UART_INTERRUPT 0
#define TIMER0_INTERRUPT 1
#define ETHMAC_INTERRUPT 2
#define SYSTEM_CLOCK_FREQUENCY 100000000
#define A7DDRPHY_BITSLIP 2
#define A7DDRPHY_DELAY 6
#define L2_SIZE 8192

#endif
