OPENOCD_ROOT=~/tmp//mr/openocd_arm64/openocd_arm64
TARGET=nuttx

pkill openocd

$OPENOCD_ROOT/bin/openocd -f $OPENOCD_ROOT/bin/wch-riscv.cfg &
trap 'jobs -p | xargs -r kill' EXIT
# trap 'kill $(jobs -pr)' SIGINT SIGTERM EXIT



# /Users/robertlipe/.espressif/tools/riscv32-esp-elf/esp-2021r1-8.4.0/riscv32-esp-elf/bin/riscv32-esp-elf-gdb $TARGET

riscv64-unknown-elf-gdb $TARGET

