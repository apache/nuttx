#/bin/sh


genromfs -f romfs.img -d ../romfs -v -V "romfs"

xxd -i romfs.img etc_romfs.c

cp etc_romfs.c ../src

rm romfs.img etc_romfs.c

