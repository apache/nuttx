#/bin/sh


genromfs -f romfs.img -d ../romfs -v -V "romfs"

xxd -i romfs.img nsh_romfsimg.h

cp nsh_romfsimg.h ../include

rm romfs.img nsh_romfsimg.h

