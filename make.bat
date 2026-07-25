@echo off
REM This batch file does an out-of-source CMake build in ../build_windows
REM This is for users who like to configure & build Blender with a single command.
setlocal EnableDelayedExpansion
setlocal ENABLEEXTENSIONS
set BLENDER_DIR=%~dp0

call "%BLENDER_DIR%\build_files\windows\reset_variables.cmd"

call "%BLENDER_DIR%\build_files\windows\check_spaces_in_path.cmd"
if errorlevel 1 goto EOF

call "%BLENDER_DIR%\build_files\windows\parse_arguments.cmd" %*
if errorlevel 1 goto EOF

call "%BLENDER_DIR%\build_files\windows\find_dependencies.cmd"
if errorlevel 1 goto EOF

REM if it is one of the convenience targets and BLENDER_BIN is set
REM skip compiler detection
if "%ICONS%%ICONS_GEOM%%DOC_PY%" == "1" (
	if EXIST "%BLENDER_BIN%" (
		goto convenience_targets
	)
)

if "%BUILD_SHOW_HASHES%" == "1" (
	call "%BLENDER_DIR%\build_files\windows\show_hashes.cmd"
	goto EOF
)

if "%SHOW_HELP%" == "1" (
	call "%BLENDER_DIR%\build_files\windows\show_help.cmd"
	goto EOF
)

if "%FORMAT%" == "1" (
	call "%BLENDER_DIR%\build_files\windows\format.cmd"
	goto EOF
)

if "%LICENSE%" == "1" (
	call "%BLENDER_DIR%\build_files\windows\license.cmd"
	goto EOF
)

call "%BLENDER_DIR%\build_files\windows\detect_architecture.cmd"
if errorlevel 1 goto EOF

REM Enforce the default compiler to be clang on ARM64
if "%BUILD_ARCH%" == "arm64" (
	if not "%WITH_CLANG%" == "1" (
		if "%WITH_MSVC%" == "1" (
			echo WARNING, MSVC compilation on Windows ARM64 is unsupported, and errors may occur.
		) else (
			echo Windows ARM64 builds with clang by default, enabling. If you wish to use MSVC ^(unsupported^), please use the msvc switch.
			set WITH_CLANG=1
		)
	)
)

if "%BUILD_VS_YEAR%" == "" (
	call "%BLENDER_DIR%\build_files\windows\autodetect_msvc.cmd"
	if errorlevel 1 (
		echo Visual Studio not found ^(try with the 'verbose' switch for more information^)
		goto EOF
	)
) else (
	call "%BLENDER_DIR%\build_files\windows\detect_msvc%BUILD_VS_YEAR%.cmd"
	if errorlevel 1 (
		echo Visual Studio %BUILD_VS_YEAR% not found ^(try with the 'verbose' switch for more information^)
		goto EOF
	)
)

if "%BUILD_VS_YEAR%"=="2026" (
	set VS_SLN_EXT=slnx
) else (
	set VS_SLN_EXT=sln
)

if "%BUILD_UPDATE%" == "1" (
	REM Initialize git submodules for precompiled libraries.
	call "%BLENDER_DIR%\build_files\windows\check_libraries.cmd"
	if errorlevel 1 goto EOF
	if "%BUILD_UPDATE_SVN%" == "1" (
		call "%BLENDER_DIR%\build_files\windows\lib_update.cmd"
	)
	call "%BLENDER_DIR%\build_files\windows\update_sources.cmd"
	goto EOF
)

if "%BUILD_SETUP%" == "1" (
	echo.
	echo Initializing Submodules and LFS ...
	"%GIT%" -C "%BLENDER_DIR%\" config --local "submodule.lib/windows_x64.update" "checkout"
	"%GIT%" -C "%BLENDER_DIR%\" config --local "submodule.lib/windows_arm64.update" "checkout"
	"%GIT%" -C "%BLENDER_DIR%\" submodule sync --recursive
	set GIT_LFS_SKIP_SMUDGE=1
	"%GIT%" -C "%BLENDER_DIR%\" submodule update --init --recursive --progress
	set GIT_LFS_SKIP_SMUDGE=
	"%GIT%" -C "%BLENDER_DIR%\" config --local --unset "submodule.lib/windows_x64.update"
	"%GIT%" -C "%BLENDER_DIR%\" config --local --unset "submodule.lib/windows_arm64.update"
	"%GIT%" -C "%BLENDER_DIR%\" lfs pull
	echo.
	echo Setup complete. Run 'make.bat full' to build.
	goto EOF
)

call "%BLENDER_DIR%\build_files\windows\set_build_dir.cmd"

:convenience_targets

if "%ICONS_GEOM%" == "1" (
	call "%BLENDER_DIR%\build_files\windows\icons_geom.cmd"
	goto EOF
)

if "%DOC_PY%" == "1" (
	call "%BLENDER_DIR%\build_files\windows\doc_py.cmd"
	goto EOF
)

if "%CMAKE%" == "" (
	echo Cmake not found in path, required for building, exiting...
	exit /b 1
)

if "%WITH_CLANG%" == "1" (
	call "%BLENDER_DIR%\build_files\windows\find_llvm.cmd"
	if errorlevel 1 (
		echo LLVM/Clang not found ^(try with the 'verbose' switch for more information^)
		goto EOF
	)
)

echo Building blender with VS%BUILD_VS_YEAR% for %BUILD_ARCH% in %BUILD_DIR%

call "%BLENDER_DIR%\build_files\windows\check_libraries.cmd"
if errorlevel 1 goto EOF

if "%TEST%" == "1" (
	call "%BLENDER_DIR%\build_files\windows\test.cmd"
	goto EOF
)

if "%BUILD_WITH_NINJA%" == "" (
	call "%BLENDER_DIR%\build_files\windows\configure_msbuild.cmd"
	if errorlevel 1 goto EOF

	call "%BLENDER_DIR%\build_files\windows\build_msbuild.cmd"
	if errorlevel 1 goto EOF
) else (
	call "%BLENDER_DIR%\build_files\windows\configure_ninja.cmd"
	if errorlevel 1 goto EOF

	call "%BLENDER_DIR%\build_files\windows\build_ninja.cmd"
	if errorlevel 1 goto EOF
)

:EOF
if errorlevel 1 exit /b %errorlevel%
