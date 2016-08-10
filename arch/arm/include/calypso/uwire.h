#ifndef __ARCH_ARM_INCLUDE_CALYPSO_UWIRE_H
#define __ARCH_ARM_INCLUDE_CALYPSO_UWIRE_H
void uwire_init(void);
int uwire_xfer(int cs, int bitlen, const void *dout, void *din);
#endif

