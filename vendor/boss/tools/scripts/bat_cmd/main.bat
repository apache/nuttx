::
:: 声明：该文件通过ESP项目改写而来，详细请访问https://docs.espressif.com/projects/esp-idf。
:: ‌Declaration‌: This file has been rewritten using ESP. For more details, please 
::              visit https://docs.espressif.com/projects/esp-idf.
::

@echo off

set "BAT_TOOLS_DIR=%~dp0"

call %BAT_TOOLS_DIR%\check_python_env.cmd
if %errorlevel% equ 0 (
    echo. 
    echo python env check SUCCESS!
    echo. 
    goto :_GOON
)

:_ByeBye
pause
exit

:_GOON
pause
