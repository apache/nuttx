set LPCScrypt=C:\NXP\LPCScrypt\bin

call %LPCScrypt%\..\scripts\boot_lpcscrypt.cmd

%LPCScrypt%\lpcscrypt.exe erase SPIFI
%LPCScrypt%\lpcscrypt.exe program ..\..\..\nuttx.bin SPIFI
	