@echo off
setlocal
set "ROBOT_ROLE=%~1"
if "%ROBOT_ROLE%"=="" set "ROBOT_ROLE=H"
if /i "%ROBOT_ROLE%"=="H" goto role_h
if /i "%ROBOT_ROLE%"=="L" goto role_l
goto usage

:role_h
set "ROBOT_ROLE=H"
set "ROBOT_SUFFIX=h"
set "ROBOT_DEFAULT_PROBE=Vllink"
set "ROBOT_GDB_PORT=3333"
set "ROBOT_TCL_PORT=6666"
set "ROBOT_TELNET_PORT=4444"
goto options

:role_l
set "ROBOT_ROLE=L"
set "ROBOT_SUFFIX=l"
set "ROBOT_DEFAULT_PROBE=Vllink"
set "ROBOT_GDB_PORT=3334"
set "ROBOT_TCL_PORT=6667"
set "ROBOT_TELNET_PORT=4445"

:options
set "ROBOT_ACTION=%~2"
if "%ROBOT_ACTION%"=="" set "ROBOT_ACTION=Flash"
if /i "%ROBOT_ACTION%"=="Build" goto probe
if /i "%ROBOT_ACTION%"=="Flash" goto probe
if /i "%ROBOT_ACTION%"=="Server" goto probe
goto usage

:probe
set "ROBOT_PROBE=%~3"
if "%ROBOT_PROBE%"=="" set "ROBOT_PROBE=Auto"
if /i "%ROBOT_PROBE%"=="Auto" set "ROBOT_PROBE=%ROBOT_DEFAULT_PROBE%"
if /i "%ROBOT_PROBE%"=="Vllink" goto probe_vllink
if /i "%ROBOT_PROBE%"=="DAP" goto probe_dap
goto usage

:probe_vllink
set "ROBOT_PROBE_CONFIG=vllink.cfg"
goto speed

:probe_dap
set "ROBOT_PROBE_CONFIG=daplink.cfg"

:speed
set "ROBOT_SPEED=%~4"
if "%ROBOT_SPEED%"=="" set "ROBOT_SPEED=10"
for /f "delims=0123456789" %%I in ("%ROBOT_SPEED%") do goto usage
if %ROBOT_SPEED% LSS 10 goto usage
if %ROBOT_SPEED% GTR 4000 goto usage
if not "%~5"=="" goto usage

for %%I in ("%~dp0..") do set "ROBOT_ROOT=%%~fI"
set "ROBOT_ROOT=%ROBOT_ROOT:\=/%"
set "ROBOT_SOURCE=%ROBOT_ROOT%/firmware/Dust_HeavyRobot_%ROBOT_ROLE%"
set "ROBOT_BUILD=%ROBOT_ROOT%/build/build_%ROBOT_SUFFIX%"
set "ZEPHYR_BASE=D:/MCU/zephyr/zephyrproject/zephyr"
set "ZEPHYR_TOOLCHAIN_VARIANT=zephyr"
set "ZEPHYR_SDK_INSTALL_DIR=D:/MCU/zephyr/zephyr-sdk"
set "HPM_SDK_DIR=D:/MCU/hpm/hpm_sdk"
if /i "%ROBOT_ACTION%"=="Server" goto server
call :build
if errorlevel 1 exit /b %ERRORLEVEL%
if /i "%ROBOT_ACTION%"=="Build" exit /b 0

:flash
if not exist "%ROBOT_BUILD%/zephyr/Dust_HeavyRobot.bin" exit /b 1

"D:/MCU/hpm/tools/openocd/openocd.exe" -s "D:/MCU/hpm/tools/openocd/tcl" -f "%ROBOT_ROOT%/Board/hpm5361/%ROBOT_PROBE_CONFIG%" -c "adapter speed %ROBOT_SPEED%" -c "init; halt; hpm5361.cpu0 configure -work-area-phys 0x0 -work-area-size 0x20000 -work-area-backup 0; set flash_status [catch {program {%ROBOT_BUILD%/zephyr/Dust_HeavyRobot.bin} verify 0x80000000} flash_result]; hpm5361.cpu0 configure -work-area-phys 0xf0406000 -work-area-size 0x2000 -work-area-backup 0; if {$flash_status} {error $flash_result}; reset run; shutdown"
exit /b %ERRORLEVEL%

:server
"D:/MCU/hpm/tools/openocd/openocd.exe" -s "D:/MCU/hpm/tools/openocd/tcl" -f "%ROBOT_ROOT%/Board/hpm5361/%ROBOT_PROBE_CONFIG%" -c "adapter speed %ROBOT_SPEED%" -c "gdb port %ROBOT_GDB_PORT%" -c "tcl port %ROBOT_TCL_PORT%" -c "telnet port %ROBOT_TELNET_PORT%" -c init
exit /b %ERRORLEVEL%

:build

set "ROBOT_PRISTINE=never"
set "ROBOT_OLD_SOURCE="
set "ROBOT_CACHE=%ROBOT_BUILD:/=\%\CMakeCache.txt"
if not exist "%ROBOT_CACHE%" goto build_run
for /f "usebackq tokens=1,* delims==" %%A in ("%ROBOT_CACHE%") do if "%%A"=="APPLICATION_SOURCE_DIR:PATH" set "ROBOT_OLD_SOURCE=%%B"
set "ROBOT_OLD_SOURCE=%ROBOT_OLD_SOURCE:\=/%"
if /i not "%ROBOT_OLD_SOURCE%"=="%ROBOT_SOURCE%" set "ROBOT_PRISTINE=always"
:build_run
"D:/MCU/zephyr/zephyrproject/.venv/Scripts/west.exe" build -b hpm5361icb -s "%ROBOT_SOURCE%" -d "%ROBOT_BUILD%" -p %ROBOT_PRISTINE%
exit /b %ERRORLEVEL%

:usage
echo Usage: "%~nx0" [H^|L] [Build^|Flash^|Server] [Auto^|Vllink^|DAP] [10..4000 kHz]
exit /b 2
