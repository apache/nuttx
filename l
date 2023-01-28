OPENOCD_ROOT=~/tmp//mr/openocd_arm64/openocd_arm64
TARGET=nuttx

pkill openocd

	$OPENOCD_ROOT/bin/openocd -f $OPENOCD_ROOT/bin/wch-riscv.cfg -c init -c halt -c "flash erase_sector wch_riscv 0 last " -c exit
	# $OPENOCD_ROOT/bin/openocd -f $OPENOCD_ROOT/bin/wch-riscv.cfg -c init -c halt -c "program $TARGET 0x08000000" -c exit
	$OPENOCD_ROOT/bin/openocd -f $OPENOCD_ROOT/bin/wch-riscv.cfg -c init -c halt -c "program $TARGET " -c exit
	#$(OPENOCD_ROOT)/bin/openocd -f $(OPENOCD_ROOT)/bin/wch-riscv.cfg -c init -c halt -c "verify_image $TARGET" -c exit
	$OPENOCD_ROOT/bin/openocd -f $OPENOCD_ROOT/bin/wch-riscv.cfg -c init -c "reset" -c exit || exit 0

