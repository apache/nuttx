1. Download and install toolchain

  https://occ.t-head.cn/community/download

2. Download and install qemu

  https://occ.t-head.cn/community/download

3. Modify defconfig

4. Configure and build NuttX

  $ make distclean
  $ ./tools/configure.sh smartl-c906:nsh
  $ make -j

5. Run the nuttx by downloading elf to RAM via HW debugger

6. TODO

  Support FPU
  Support ELF based file applications
  Support RISC-V User mode

