set -e
make -j10 V=9 || echo "FAILURE!"
riscv64-unknown-elf-objdump --disassemble --source nuttx > nuttx.dis
riscv64-unknown-elf-nm nuttx > nuttx.sym
