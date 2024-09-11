======================
``fastboot`` fastbootd
======================
Prepare
==========================

- Check fastboot tool(Host): :code:`fastboot --version`
- Download fastboot tool and install(Host): `platform-tools <https://developer.android.com/tools/releases/platform-tools>`__
- Enable the fastbootd application(Device): :code:`CONFIG_USBFASTBOOT=y` and :code:`CONFIG_SYSTEM_FASTBOOTD=y`
- Start fastbootd(Device): :code:`fastbootd &`

Commands
==========================
- :code:`fastboot reboot [FLAG]`: Reboot the device, more details for :code:`[FLAG]`: `g_resetflag <https://github.com/apache/nuttx-apps/blob/master/nshlib/nsh_syscmds.c#L114>`__ and `boardioc_softreset_subreason_e <https://github.com/apache/nuttx/blob/master/include/sys/boardctl.h#L458>`__
- :code:`fastboot flash <PARTITION> <FILENAME>`: Flash partition :code:`<PARTITION>` using the given :code:`<FILENAME>`
- :code:`fastboot erase <PARTITION>`: Erase given partition
- Get Variables
   - :code:`fastboot getvar product`: Get product name
   - :code:`fastboot getvar kernel`: Get kernel name
   - :code:`fastboot getvar version`: Get OS version string
   - :code:`fastboot getvar slot-count`: Get slot count
   - :code:`fastboot getvar max-download-size`: Get max download size
- OEM
   - :code:`fastboot oem filedump <PARTITION> [OFFSET] [LENGTH]`: Get :code:`<LENGTH>` (full by default) bytes of :code:`<PARTITION>` from :code:`<OFFSET>` (zero by default)
   - :code:`fastboot oem memdump <ADDRESS> <LENGTH>`: Dump :code:`<LENGTH>` bytes memory from address :code:`<ADDRESS>`
- :code:`fastboot get_staged <OUT_FILE>`: Writes data staged by the last command to file :code:`<OUT_FILE>`. e.g. "oem filedump" and "oem memdump"

Examples
==========================
- Exit fastboot mode: :code:`fastboot reboot`
- Flash app.bin to partition /dev/app: :code:`fastboot flash app ./app.bin`
- Erase partition /dev/userdata: :code:`fastboot erase userdata`
- Dump partition /dev/app: :code:`fastboot filedump /dev/app` and then :code:`fastboot get_staged ./dump_app.bin`
- Dump memory from 0x44000000 to 0x440b6c00: :code:`fastboot oem memdump 0x44000000 0xb6c00` and then :code:`fastboot get_staged ./mem_44000000_440b6c00.bin`
