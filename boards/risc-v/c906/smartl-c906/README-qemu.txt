1. Download and install toolchain

  https://occ.t-head.cn/community/download

2. Download and install qemu

  https://occ.t-head.cn/community/download

3. Modify defconfig

  CONFIG_C906_WITH_QEMU=y

4. Configure and build NuttX

  $ make distclean
  $ ./tools/configure.sh smartl-c906:nsh
  $ make -j

5. Run the nuttx with qemu

  Modify the soc config file "smarth_906_cfg.xml", enlarge the RAM size.
-        <mem name="smart_inst_mem" addr="0x0" size ="0x00020000" attr ="MEM_RAM"></mem>
+        <mem name="smart_inst_mem" addr="0x0" size ="0x00400000" attr ="MEM_RAM"></mem>
...
-                smart_inst_mem, Start: 0x0, Length: 0x20000
+                smart_inst_mem, Start: 0x0, Length: 0x400000

  Then launch QEMU:
  $ ./cskysim -soc $PATH_TO_SOCCFG/smarth_906_cfg.xml -nographic -kernel $PATH_TO_NUTTX_BUILD_DIR/nuttx

6. TODO

  Support protect mode via PMP
  Support RISC-V User mode
