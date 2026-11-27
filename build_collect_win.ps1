<#
.SYNOPSIS
  Windows (MSVC x64) configure/build/test entry point for the MVS collection program.

.DESCRIPTION
  Windows counterpart of build_collect.sh: initializes the MSVC x64 toolchain through
  vswhere plus VsDevCmd.bat environment import, configures with Ninja (Release, GUI ON,
  tests ON, UTF-8 source), builds and runs ctest. Every external command exit code is
  checked and a non-zero code aborts the script. CMake probes the localized
  compiler prefix for reliable Ninja header dependency tracking. Tests use the supplied runtime directories.

.EXAMPLE
  powershell -File build_collect_win.ps1
  powershell -File build_collect_win.ps1 -GuiOn:$false -BuildType Debug
#>
[CmdletBinding()]
param(
    [string]$QtDir = 'C:\Qt\5.15.2\msvc2019_64',
    [string]$OpenCVDir = 'C:\soft\opencv-4.13.0\build\install',
    [string]$CgxDir = 'C:\soft\CGXi-Robot-SDK-v2.50d-full-20260807\sdk\C&C++',
    [string]$MvsSdkDir = 'C:\Program Files (x86)\MVS',
    # 留空时按 MVS 安装目录推导 Common Files\MVS\Runtime\Win64_x64。
    [string]$MvsRuntimeDir = '',
    [string]$BuildDir = 'win-build\build',
    [ValidateSet('Ninja', 'Ninja Multi-Config')]
    [string]$Generator = 'Ninja',
    [ValidateSet('Release', 'Debug', 'RelWithDebInfo', 'MinSizeRel')]
    [string]$BuildType = 'Release',
    [bool]$GuiOn = $true,
    [bool]$TestsOn = $true,
    [int]$Jobs = 4,
    [switch]$SkipBuild,
    [switch]$SkipTests
)

$ErrorActionPreference = 'Stop'

$Repo = (Resolve-Path -LiteralPath $PSScriptRoot).Path
if (-not $MvsRuntimeDir) {
    # Split-Path 的 -Parent 属于 Path 参数集，和 -LiteralPath 不能同时使用；用 .NET 取父目录。
    $MvsRuntimeDir = Join-Path ([System.IO.Path]::GetDirectoryName($MvsSdkDir)) 'Common Files\MVS\Runtime\Win64_x64'
}

# Prefer English diagnostics when the compiler language resources are installed.
$env:VSLANG = '1033'
$OutputEncoding = [System.Text.UTF8Encoding]::new($false)

if (-not [System.IO.Path]::IsPathRooted($BuildDir)) {
    $BuildDir = Join-Path $Repo $BuildDir
}
$BuildDir = [System.IO.Path]::GetFullPath($BuildDir)

# Run an external program and fail on any non-zero exit code.
function Invoke-External {
    param(
        [string]$Exe,
        [string[]]$CmdArgs = @()
    )
    Write-Host ('> ' + $Exe + ' ' + ($CmdArgs -join ' '))
    & $Exe @CmdArgs
    $code = $LASTEXITCODE
    if ($code -ne 0) {
        throw ('External command failed with exit code {0}: {1} {2}' -f $code, $Exe, ($CmdArgs -join ' '))
    }
}

# Import the MSVC x64 developer environment (PATH, INCLUDE, LIB) from VsDevCmd.bat.
function Import-VsDevCmdEnvironment {
    param([string]$VsInstallPath)

    $vsDevCmd = Join-Path $VsInstallPath 'Common7\Tools\VsDevCmd.bat'
    if (-not (Test-Path -LiteralPath $vsDevCmd)) {
        throw ('VsDevCmd.bat not found under Visual Studio install path: {0}' -f $VsInstallPath)
    }

    $dump = & cmd.exe /v:off /c ('call "' + $vsDevCmd + '" -arch=amd64 -host_arch=amd64 -no_logo && set')
    if ($LASTEXITCODE -ne 0) {
        throw ('VsDevCmd.bat failed with exit code {0}' -f $LASTEXITCODE)
    }

    $imported = 0
    foreach ($line in $dump) {
        if ($line -notmatch '^([^=]+)=(.*)$') { continue }
        Set-Item -Path ('env:' + $matches[1]) -Value $matches[2]
        $imported++
    }
    if ($imported -eq 0) { throw 'VsDevCmd.bat produced no environment variables' }
    Write-Host ('Imported {0} environment variables from VsDevCmd.bat' -f $imported)
}

# 1) Locate Visual Studio through vswhere and initialize MSVC x64.
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path -LiteralPath $vswhere)) {
    $vswhere = Join-Path $env:ProgramFiles 'Microsoft Visual Studio\Installer\vswhere.exe'
}
if (-not (Test-Path -LiteralPath $vswhere)) { throw 'vswhere.exe not found; Visual Studio 2019+ is required' }

$vsInstallPath = (& $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath)
if ($LASTEXITCODE -ne 0) { throw ('vswhere failed with exit code {0}' -f $LASTEXITCODE) }
if (-not $vsInstallPath) { throw 'vswhere found no Visual Studio install with the MSVC x86/x64 toolset' }

Write-Host ('Visual Studio: ' + $vsInstallPath)
Import-VsDevCmdEnvironment -VsInstallPath $vsInstallPath

$cl = Get-Command 'cl.exe' -ErrorAction SilentlyContinue
if (-not $cl) { throw 'cl.exe is not on PATH after the VsDevCmd import' }
Write-Host ('Compiler: ' + $cl.Source)

$ninja = Get-Command 'ninja' -ErrorAction SilentlyContinue
if (-not $ninja) { throw 'ninja not found on PATH; install it or select another generator' }
Write-Host ('Generator: ' + $ninja.Source)

# 2) Report the SDK locations handed to CMake before configuring.
foreach ($entry in @(@('QtDir', $QtDir), @('OpenCVDir', $OpenCVDir), @('CgxDir', $CgxDir), @('MvsSdkDir', $MvsSdkDir))) {
    if (Test-Path -LiteralPath $entry[1]) {
        Write-Host ($entry[0] + ': ' + $entry[1])
    } else {
        Write-Host ($entry[0] + ' missing: ' + $entry[1])
    }
}

# 测试进程需要的运行期目录：Qt 插件与 DLL、OpenCV DLL、MVS 相机运行库、CGX SDK DLL。
$QtBin = Join-Path $QtDir 'bin'
$OpenCVBin = Join-Path $OpenCVDir 'x64\vc17\bin'
$CgxLibDir = Join-Path $CgxDir 'libs\windows\x64'
foreach ($entry in @(@('Qt bin', $QtBin), @('OpenCV bin', $OpenCVBin), @('MVS runtime', $MvsRuntimeDir), @('CGX libs', $CgxLibDir))) {
    if (Test-Path -LiteralPath $entry[1]) {
        Write-Host ($entry[0] + ': ' + $entry[1])
    } else {
        Write-Host ($entry[0] + ' missing: ' + $entry[1])
    }
}

$cmakeArgs = @(
    '-S', $Repo,
    '-B', $BuildDir,
    '-G', $Generator,
    ('-DCMAKE_BUILD_TYPE=' + $BuildType),
    ('-DCMAKE_C_FLAGS_INIT=/utf-8'),
    ('-DCMAKE_CXX_FLAGS_INIT=/utf-8'),
    ('-DCAMERA_BACKEND=MVS'),
    ('-DBUILD_QT_GUI=' + $(if ($GuiOn) { 'ON' } else { 'OFF' })),
    ('-DBUILD_TESTS=' + $(if ($TestsOn) { 'ON' } else { 'OFF' })),
    ('-DQt5_DIR=' + (Join-Path $QtDir 'lib\cmake\Qt5')),
    ('-DOpenCV_DIR=' + $OpenCVDir),
    ('-DCGX_SDK_DIR=' + $CgxDir),
    ('-DMVS_SDK_DIR=' + $MvsSdkDir)
)

# 3) Configure, build and test.
Invoke-External -Exe 'cmake' -CmdArgs $cmakeArgs

if (-not $SkipBuild) {
    # --config 对 Ninja 这类单配置生成器无副作用，对多配置生成器是必需的。
    $buildArgs = @('--build', $BuildDir, '--config', $BuildType)
    if ($Jobs -gt 0) { $buildArgs += @('--parallel', $Jobs) }
    Invoke-External -Exe 'cmake' -CmdArgs $buildArgs
}

if ($TestsOn -and -not $SkipTests) {
    # ctest 也要按 BuildType 解析测试可执行文件路径。
    # 运行期目录放在原 PATH 之前，但 CGX 目录放最后：SDK 自带 msvcr100.dll、ucrtbase.dll、
    # vcruntime140*.dll 这些旧 CRT，若排在系统运行库之前会优先被加载。
    $searchDirs = @($QtBin, $OpenCVBin, $MvsRuntimeDir) | Where-Object { Test-Path -LiteralPath $_ }
    $testPath = ($searchDirs -join ';')
    if ($testPath) { $testPath = $testPath + ';' }
    $testPath = $testPath + $env:PATH
    if (Test-Path -LiteralPath $CgxLibDir) { $testPath = $testPath + ';' + $CgxLibDir }
    $env:PATH = $testPath
    Write-Host ('Test runtime PATH: ' + $env:PATH)
    Invoke-External -Exe 'ctest' -CmdArgs @('--test-dir', $BuildDir, '-C', $BuildType, '--output-on-failure')
}

if ($Generator -eq 'Ninja Multi-Config') { $BuildDir = Join-Path $BuildDir $BuildType }
Write-Host ('Collection program: ' + (Join-Path $BuildDir 'collect-mvs.exe'))
exit 0
