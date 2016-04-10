set LPCScrypt=C:\NXP\LPCScrypt\bin

call %LPCScrypt%\..\scripts\boot_lpcscrypt.cmd

TIMEOUT /T 5

%LPCScrypt%\lpcscrypt.exe erase SPIFI
%LPCScrypt%\lpcscrypt.exe program ..\..\..\nuttx.bin SPIFI
	