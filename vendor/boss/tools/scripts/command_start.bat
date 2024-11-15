::
:: 声明：该文件通过ESP项目改写而来，详细请访问https://docs.espressif.com/projects/esp-idf。
:: ‌Declaration‌: This file has been rewritten using ESP. For more details, please 
::              visit https://docs.espressif.com/projects/esp-idf.
::

@echo off

title 工程命令行
color 0A

set "BOSS_TARGETS=esp32s3"

if exist flag.txt (
    echo. 
    echo 请关闭其他窗口,重新执行！！！
    echo. 
    goto :_EXCEPT
)

echo. > flag.txt
cmd /k "bat_cmd\main.bat"

set BOSS_TARGETS=
exit

:_EXCEPT
set BOSS_TARGETS=
del flag.txt
pause
