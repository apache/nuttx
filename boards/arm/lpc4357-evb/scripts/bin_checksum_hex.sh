checksum nuttx.bin -p LPC4357 -v
srec_cat nuttx.bin -binary -offset 0x1a000000 -o nuttx.hex -intel --line-length=44
