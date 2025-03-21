============================================
``mtd`` MTD test and transfer rate benchmark
============================================

This testing/benchmark application performs an erase/write operation to
evaluate write transfer rate and then reads the written content back to
evaluate the read transfer rate. Finally, it compares the read data with
the previously written data to ensure the MTD device is working as expected.

EXAMPLE::

  nsh> mtd /dev/mtdblock0
  FLASH Test on device with:
    Sector size:        4096
    Sector count:        256
    Erase block:        4096
    Total size:      1048576

  Starting write operation...

  Write operation completed in 5.46 seconds
  Total bytes written: 1048576
  Transfer rate [write]: 187.55 KiB/s

  Starting read operation...

  Read operation completed in 0.11 seconds
  Total bytes read: 1048576
  Transfer rate [read]: 9309.09 KiB/s

  Data verification successful: read data matches written data
