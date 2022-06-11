1. Download and install toolchain

  $ curl https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz

2. Build and install qemu

  $ git clone https://github.com/qemu/qemu
  $ cd qemu
  $ ./configure --target-list=riscv32-softmmu
  $ make
  $ sudo make install

3. Modify defconfig

index c449421741..5a76600785 100644
--- a/boards/risc-v/fe310/hifive1-revb/configs/nsh/defconfig
+++ b/boards/risc-v/fe310/hifive1-revb/configs/nsh/defconfig
@@ -14,7 +14,7 @@ CONFIG_ARCH_BOARD="hifive1-revb"
 CONFIG_ARCH_BOARD_HIFIVE1_REVB=y
 CONFIG_ARCH_CHIP="fe310"
 CONFIG_ARCH_CHIP_FE310=y
-CONFIG_ARCH_CHIP_FE310_G002=y
+CONFIG_ARCH_CHIP_FE310_QEMU=y
 CONFIG_ARCH_INTERRUPTSTACK=1536
 CONFIG_ARCH_RISCV=y
 CONFIG_ARCH_STACKDUMP=y

4. Configure and build NuttX

  $ mkdir ./nuttx; cd ./nuttx
  $ git clone https://github.com/apache/incubator-nuttx.git nuttx
  $ git clone https://github.com/apache/incubator-nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh hifive1-revb:nsh
  $ make V=1

5. Run the nuttx with qemu

  $ qemu-system-riscv32 -nographic -machine sifive_e -kernel ./nuttx

6. TODO

  Support RISC-V User mode
