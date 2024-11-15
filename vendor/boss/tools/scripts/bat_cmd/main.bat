::
:: 声明：该文件通过ESP项目改写而来，详细请访问https://docs.espressif.com/projects/esp-idf。
:: ‌Declaration‌: This file has been rewritten using ESP. For more details, please 
::              visit https://docs.espressif.com/projects/esp-idf.
::

@echo off

set "BAT_TOOLS_DIR=%~dp0"
for %%i in ("%cd%\..") do set "ROOT_TOOLS_DIR=%%~fi"

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
set "OLD_PATH=%PATH%"
set PATH="%ROOT_TOOLS_DIR%\.boss\idf-python-3.11.2-embed-win64\";%PATH%
call %BAT_TOOLS_DIR%\check_python_env.cmd
if %errorlevel% neq 0 (
    echo python env check SUCCESS!
    echo. 
    goto :_ByeBye
)

echo. 
echo To Dwonload tools
python %cd%\idf-tools.py download --targets %BOSS_TARGETS%
if %errorlevel% equ 0 (
    echo python env check SUCCESS!
    echo. 
    goto :_GOON
)

:_ByeBye
pause
exit

:_GOON
set "IDF_TOOLS_EXPORTS_FILE=%TEMP%\idf_export_vars.tmp"
set "EXECUTE_CMD=python --version"
set "line=lines"

%EXECUTE_CMD%

echo. 
echo Install python env and tools
set "EXECUTE_CMD=python %cd%\idf-tools.py install --python-env"
setlocal enabledelayedexpansion
for /f "delims=" %%i in ('!EXECUTE_CMD!') do (
    echo %%i
    set "line=%%i"
    if "!line:~0,7!"=="Output:" (
        for /f "tokens=1,* delims=: " %%a in ("%%i") do (
            echo IDF_PYTHON_ENV_PATH=%%b > !IDF_TOOLS_EXPORTS_FILE!
        )
        endlocal
        goto :_GOON1
    )
)

pause
exit

:_GOON1
python %cd%\idf-tools.py export --format key-value --targets %BOSS_TARGETS% >> %IDF_TOOLS_EXPORTS_FILE%

REM 恢复原始的PATH环境
set "PATH=%OLD_PATH%"
set "OLD_PATH=%PATH%"
for /f "usebackq tokens=1,2 eol=# delims== " %%a in ("%IDF_TOOLS_EXPORTS_FILE%") do (
  call set "%%a=%%b"
)
set PATH="%IDF_PYTHON_ENV_PATH%\Scripts\";%PATH%
echo 新环境：
echo %PATH%
echo 旧环境：
echo %OLD_PATH%

DOSKEY idf.py=python.exe "%ROOT_TOOLS_DIR%\scripts\idf.py" $*

:_UNSET
set SCRIPT_EXIT_CODE=
set OLD_PATH=
set EXECUTE_CMD=
set line=
set BAT_TOOLS_DIR=
set ROOT_TOOLS_DIR=

echo.
echo Done! You can now compile FREE-IDF projects.
echo Go to the project directory and run:
echo.
echo   idf.py build
echo.

pause
