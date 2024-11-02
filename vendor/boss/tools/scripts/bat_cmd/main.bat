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

echo 开始自动安装离线python压缩包
call %BAT_TOOLS_DIR%\install_python_zip.cmd
if %errorlevel% equ 0 (
    echo. 
    echo install python zip SUCCESS!
    echo. 
)
set Path="%cd%\..\.boss\idf-python-3.11.2-embed-win64\";%PATH%
call %BAT_TOOLS_DIR%\check_python_env.cmd
if %errorlevel% equ 0 (
    echo python env check SUCCESS!
    echo. 
    goto :_GOON
)

:_ByeBye
pause
exit

:_GOON
pause
