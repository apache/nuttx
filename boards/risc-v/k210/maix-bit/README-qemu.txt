1. Download and install toolchain

  $ curl https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz

2. Build and install qemu

  $ git clone https://github.com/qemu/qemu
  $ cd qemu
  $ ./configure --target-list=riscv64-softmmu
  $ make
  $ sudo make install

3. Modify defconfig

--- a/boards/risc-v/k210/maix-bit/configs/nsh/defconfig
+++ b/boards/risc-v/k210/maix-bit/configs/nsh/defconfig
@@ -25,6 +25,7 @@ CONFIG_EXAMPLES_HELLO=y
 CONFIG_FS_PROCFS=y
 CONFIG_IDLETHREAD_STACKSIZE=2048
 CONFIG_INTELHEX_BINARY=y
+CONFIG_K210_WITH_QEMU=y
 CONFIG_LIBC_PERROR_STDOUT=y
 CONFIG_LIBC_STRERROR=y

4. Configure and build NuttX

  $ mkdir ./nuttx; cd ./nuttx
  $ git clone https://github.com/apache/incubator-nuttx.git nuttx
  $ git clone https://github.com/apache/incubator-nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh maix-bit:nsh
  $ make V=1

5. Run the nuttx with qemu

  $ qemu-system-riscv64 -nographic -machine sifive_u -bios ./nuttx

  NOTE: To run nuttx for kostest, gdb needs to be used to load both nuttx_user.elf and nuttx

  $ qemu-system-riscv64 -nographic -machine sifive_u -s -S
  $ riscv64-unknown-elf-gdb -ex 'target extended-remot:1234' -ex 'load nuttx_user.elf' -ex 'load nuttx' -ex 'c'

6. TODO

  Support FPU
  Support RISC-V User mode
