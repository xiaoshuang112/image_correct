#!/usr/bin/env pwsh
# ============================================================
#  collect-mvs Windows 10 x64 发布包打包脚本
#
#  用法:
#    powershell -File tools/package_mvs_release_win.ps1
#    powershell -File tools/package_mvs_release_win.ps1 -BuildDir build-win -Output dist/collect-mvs-win10-x64
#    powershell -File tools/package_mvs_release_win.ps1 -SkipSmoke
#
#  产物布局（exe 与运行库同目录）:
#    <output>/collect-mvs.exe
#    <output>/platforms/ QtQuick.2/ QtQuick/Controls.2/ QtQuick/Layouts/ QtQuick/Window.2/ imageformats/ translations/
#    <output>/qt.conf            固定使用 exe 旁的插件与 QML 模块
#    <output>/                   OpenCV、CGXi、MVS、VC143 CRT 运行库
#    <output>/config.yaml param/ 配置与参数目录（不复制示例标定结果）
#    <output>/BUILD_INFO.txt SHA256SUMS
#
#  关键约束：CGXi 与 MVS SDK 自带的 CRT（msvcp140*/vcruntime140*/ucrtbase/concrt140）版本较旧，
#  与 OpenCV 4.13 vc17 运行库不兼容；同目录时 Windows 加载器优先挑到旧版，OpenCV 初始化会失败
#  （0xc0000142）。这些一律排除，改由 VS2022 的 Microsoft.VC143.CRT 补齐，保证脱离开发机的 Win10 独立运行。
#  CGXi 与 MVS 仍需要 msvcr100.dll 和 pthreadVC2.dll，必须保留。
#
#  目标机仍需真实安装海康 MVS 驱动；打包只分发运行库，不覆盖驱动安装。硬件未验证。
# ============================================================

param(
    [string]$BuildDir      = '',
    [string]$Output        = '',
    [ValidateSet('Release')]
    [string]$Configuration = 'Release',
    [string]$QtDir         = 'C:\Qt\5.15.2\msvc2019_64',
    [string]$OpenCVDir     = 'C:\soft\opencv-4.13.0\build\install',
    [string]$CgxDir        = 'C:\soft\CGXi-Robot-SDK-v2.50d-full-20260807\sdk\C&C++',
    [string]$MvsRuntime    = 'C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64',
    [string]$RedistDir     = '',
    [string]$Log           = '',
    [switch]$SkipSmoke
)

$ErrorActionPreference = 'Stop'

$Repo = Split-Path -Parent $PSScriptRoot
if (-not $BuildDir) { $BuildDir = Join-Path $Repo 'build-win' }
if (-not $Output)   { $Output   = Join-Path $Repo 'dist\collect-mvs-win10-x64' }
$BuildDir = [System.IO.Path]::GetFullPath($BuildDir)
if (-not $Log) { $Log = Join-Path $BuildDir 'windows-package.log' }
$Log = [System.IO.Path]::GetFullPath($Log)
$Output = [System.IO.Path]::GetFullPath($Output)

New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Log) | Out-Null
function Say([string]$text) {
    Write-Host $text
    Add-Content -LiteralPath $Log -Value $text -Encoding utf8
}

# SDK 自带 CRT 一律不复制；MSVC 14.3（VC143）现代 CRT 单独从 VS2022 分发目录补齐。
$ExcludedCrt = @(
    'msvcp140.dll', 'msvcp140_1.dll', 'msvcp140_2.dll', 'msvcp140_3.dll',
    'msvcp140_atomic_wait.dll', 'msvcp140_codecvt_ids.dll',
    'vcruntime140.dll', 'vcruntime140_1.dll', 'vcruntime140_threads.dll',
    'concrt140.dll', 'vccorlib140.dll', 'ucrtbase.dll'
)
# 旧 CRT 中仍被 SDK 依赖、必须保留: msvcr100.dll、pthreadVC2.dll。

# ---------- 输入校验：缺输入直接失败 ----------
$exeName = 'collect-mvs.exe'
$Bin = Join-Path $BuildDir $exeName
if (-not (Test-Path -LiteralPath $Bin)) {
    $alt = Join-Path (Join-Path $BuildDir $Configuration) $exeName
    if (Test-Path -LiteralPath $alt) { $Bin = $alt } else { throw "未找到 $exeName，请先以 Release 构建: $BuildDir" }
}

$windeployqt = Join-Path $QtDir 'bin\windeployqt.exe'
$cvbin = Join-Path $OpenCVDir 'x64\vc17\bin'
$cgxbin = Join-Path $CgxDir 'libs\windows\x64'
foreach ($entry in @($Bin, $windeployqt, $cvbin, $cgxbin, $MvsRuntime)) {
    if (-not (Test-Path -LiteralPath $entry)) { throw "缺少必需输入: $entry" }
}

if (@(Get-ChildItem -LiteralPath $cvbin -Filter 'opencv_world*.dll').Count -eq 0) { throw "OpenCV 运行库不完整: $cvbin" }
if (-not (Test-Path -LiteralPath (Join-Path $cgxbin 'cr_sdk.dll'))) { throw "CGXi 运行库不完整: $cgxbin\cr_sdk.dll" }
if (-not (Test-Path -LiteralPath (Join-Path $MvsRuntime 'MvCameraControl.dll'))) { throw "MVS 运行库不完整: $MvsRuntime\MvCameraControl.dll" }

New-Item -ItemType Directory -Force -Path $Output | Out-Null

# VS2022 VC143 CRT（脱离开发机的 Win10 独立运行所需）
$crtDir = $RedistDir
if (-not $crtDir) {
    $vswhere = Join-Path 'C:\Program Files (x86)\Microsoft Visual Studio\Installer' 'vswhere.exe'
    if (-not (Test-Path -LiteralPath $vswhere)) { throw '缺少 vswhere，无法定位 VS2022 VC143 运行库' }
    $vsRoot = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Redist.143 -property installationPath
    if ($LASTEXITCODE -ne 0) { throw 'vswhere failed' }
    if (-not $vsRoot) {
        $vsRoot = & $vswhere -latest -products * -property installationPath
        if ($LASTEXITCODE -ne 0) { throw 'vswhere failed' }
    }
    if (-not $vsRoot) { throw 'vswhere 找不到已安装的 VS2022' }
    $crtDir = Get-ChildItem -LiteralPath (Join-Path $vsRoot 'VC\Redist\MSVC') -Directory |
        ForEach-Object { Join-Path $_.FullName 'x64\Microsoft.VC143.CRT' } |
        Where-Object { Test-Path -LiteralPath $_ } |
        Sort-Object -Descending | Select-Object -First 1
    if (-not $crtDir) { throw '找不到 VS2022 Redist MSVC x64 Microsoft.VC143.CRT' }
}
if (-not (Test-Path -LiteralPath (Join-Path $crtDir 'msvcp140.dll'))) { throw "VC143 CRT 目录不完整: $crtDir" }

Say "collect-mvs Windows 发布包打包 $( [DateTime]::UtcNow.ToString('yyyy-MM-ddTHH:mm:ssZ') )"
Say "BuildDir=$BuildDir Output=$Output Configuration=$Configuration"

# ---------- 1) 可执行文件 ----------
$OutputBin = Join-Path $Output $exeName
if (-not [string]::Equals($Bin, $OutputBin, [System.StringComparison]::OrdinalIgnoreCase)) {
    Copy-Item -LiteralPath $Bin -Destination $OutputBin -Force
}
Say "exe: $OutputBin"

# ---------- 2) Qt 运行库与 QML 模块 ----------
$qtBuildMode = if ($Configuration -eq 'Debug') { '--debug' } else { '--release' }
$platformPlugin = if ($Configuration -eq 'Debug') { 'platforms\qwindowsd.dll' } else { 'platforms\qwindows.dll' }
& $windeployqt $qtBuildMode --no-compiler-runtime --qmldir (Join-Path $Repo 'hand_eye_gui\qml') $OutputBin
if ($LASTEXITCODE -ne 0) { throw "windeployqt 失败，退出码 $LASTEXITCODE" }
Say 'windeployqt: exit 0（--qmldir hand_eye_gui/qml）'

# 固定使用 exe 旁的插件与 QML 模块，不依赖开发机 Qt 路径或环境变量。
@"
[Paths]
Prefix=.
Plugins=.
Qml2Imports=.
Translations=translations
"@ | Set-Content -LiteralPath (Join-Path $Output 'qt.conf') -Encoding ascii

# ---------- 3) OpenCV ----------
# opencv_world4130.dll 需要 tbb12、freetype、harfbuzz；freetype 又需要 zlib1、bz2、libpng16、brotli。
# ffmpeg videoio 后端是运行期 dlopen 的可选组件，本工程不使用 VideoCapture，不打包（约 28 MB）。
$cvSkipped = @('opencv_videoio_ffmpeg4130_64.dll')
$cvCopied = 0
foreach ($dll in Get-ChildItem -LiteralPath $cvbin -Filter '*.dll') {
    if ($cvSkipped -contains $dll.Name) { continue }
    Copy-Item -LiteralPath $dll.FullName -Destination $Output -Force
    $cvCopied++
}
Say "OpenCV: 复制 $cvCopied 个 DLL（跳过 ffmpeg videoio 后端）"

# ---------- 4) CGXi SDK ----------
$cgxCopied = 0
foreach ($dll in Get-ChildItem -LiteralPath $cgxbin -Filter '*.dll') {
    if ($ExcludedCrt -contains $dll.Name) { continue }
    Copy-Item -LiteralPath $dll.FullName -Destination $Output -Force
    $cgxCopied++
}
Say "CGXi: 复制 $cgxCopied 个 DLL（排除自带 CRT，保留 msvcr100 与 pthreadVC2）"

# ---------- 5) MVS 运行库 ----------
$mvsCopied = 0
foreach ($file in Get-ChildItem -LiteralPath $MvsRuntime -File) {
    if ($file.Extension -notin @('.dll', '.ini', '.cti')) { continue }
    if ($file.Extension -eq '.dll' -and $ExcludedCrt -contains $file.Name) { continue }
    Copy-Item -LiteralPath $file.FullName -Destination $Output -Force
    $mvsCopied++
}
# 运行期加载的组件目录（GenTL producer 依赖 ThirdParty）原样保留。
foreach ($file in Get-ChildItem -LiteralPath $MvsRuntime -Recurse -File) {
    $relative = $file.FullName.Substring($MvsRuntime.TrimEnd('\').Length + 1)
    if ($relative -notmatch '[\\/]') { continue }
    if ($file.Extension -eq '.dll' -and $ExcludedCrt -contains $file.Name) { continue }
    $target = Join-Path $Output $relative
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $target) | Out-Null
    Copy-Item -LiteralPath $file.FullName -Destination $target -Force
    $mvsCopied++
}
Say "MVS: 复制 $mvsCopied 个文件（DLL/ini/cti 与组件目录，排除自带 CRT）"

# ---------- 6) VC143 现代 CRT ----------
$crtNames = @()
foreach ($dll in Get-ChildItem -LiteralPath $crtDir -Filter '*.dll') {
    Copy-Item -LiteralPath $dll.FullName -Destination $Output -Force
    $crtNames += $dll.Name
}
Say "VC143 CRT: 复制 $($crtNames.Count) 个 DLL（$crtDir）"
foreach ($name in $crtNames) {
    $out = Join-Path $Output $name
    if ((Get-FileHash -LiteralPath $out -Algorithm SHA256).Hash -ne (Get-FileHash -LiteralPath (Join-Path $crtDir $name) -Algorithm SHA256).Hash) {
        throw "输出中的 $name 不是 VC143 版本（被 SDK 自带 CRT 覆盖）"
    }
}

# ---------- 7) 配置与参数目录 ----------
# 重新部署必须保留目标机已有的 config.yaml；param/ 只建目录，不复制示例标定结果。
$configTarget = Join-Path $Output 'config.yaml'
if (Test-Path -LiteralPath $configTarget) {
    Say 'config.yaml: 保留已有配置'
} else {
    Copy-Item -LiteralPath (Join-Path $Repo 'config\collect.example.yaml') -Destination $configTarget
    Say 'config.yaml: 从 config/collect.example.yaml 初始化'
}
New-Item -ItemType Directory -Force -Path (Join-Path $Output 'param') | Out-Null
Say 'param/: 已创建（未复制示例标定文件）'

# ---------- 8) 必需文件校验 ----------
$required = @($exeName, 'qt.conf', $platformPlugin, 'config.yaml',
    'QtQuick.2\qmldir', 'QtQuick\Controls.2\qmldir', 'QtQuick\Layouts\qmldir', 'QtQuick\Window.2\qmldir',
    'MvCameraControl.dll', 'cr_sdk.dll', 'msvcp140.dll', 'msvcp140_2.dll',
    'vcruntime140.dll', 'vcruntime140_1.dll', 'msvcr100.dll', 'pthreadVC2.dll')
foreach ($item in $required) {
    if (-not (Test-Path -LiteralPath (Join-Path $Output $item))) { throw "运行目录不完整，缺少 $item" }
}
if (@(Get-ChildItem -LiteralPath $Output -Filter 'opencv_world*.dll').Count -eq 0) { throw '运行目录不完整，缺少 OpenCV 运行库' }
Say '必需文件: 全部存在'

# ---------- 9) 构建身份 ----------
$buildInfoHeader = Join-Path $BuildDir 'generated\AppBuildInfo.hpp'
$info = @{}
if (Test-Path -LiteralPath $buildInfoHeader) {
    foreach ($line in Get-Content -LiteralPath $buildInfoHeader) {
        if ($line -match '#define\s+(APP_\w+)\s+"(.*)"') { $info[$matches[1]] = $matches[2] }
    }
}
$gitState = $info['APP_GIT_WORKTREE_STATE']
if (-not $gitState) { $gitState = 'unknown' }
$binarySha = (Get-FileHash -LiteralPath $OutputBin -Algorithm SHA256).Hash
@(
    "package_time_utc=$([DateTime]::UtcNow.ToString('yyyy-MM-ddTHH:mm:ssZ'))",
    "version=$($info['APP_VERSION'])",
    "build_type=$($info['APP_BUILD_TYPE'])",
    "build_time_utc=$($info['APP_BUILD_TIMESTAMP'])",
    "git_commit=$($info['APP_GIT_COMMIT'])",
    "git_worktree_state=$gitState",
    "camera_backend=$($info['APP_CAMERA_BACKEND'])",
    "target=$Configuration",
    "qt=$QtDir",
    "opencv=$OpenCVDir",
    "cgx_sdk=$CgxDir",
    "mvs_runtime=$MvsRuntime",
    "vc143_crt=$crtDir",
    "binary_sha256=$binarySha"
) | Set-Content -LiteralPath (Join-Path $Output 'BUILD_INFO.txt') -Encoding utf8
Say 'BUILD_INFO.txt: 已生成'

# Smoke logs go to a fresh build directory, preserving any application logs.
$readinessOk = $false
$closeOk = $false
$exitCodeInfo = 'not run'
if ($SkipSmoke) {
    Say 'Smoke launch skipped'
} else {
    $smokeDirectory = Join-Path $BuildDir ('smoke-' + [Guid]::NewGuid().ToString('N'))
    New-Item -ItemType Directory -Force -Path $smokeDirectory | Out-Null
    $logFile = Join-Path $smokeDirectory 'hand-eye.log'
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = $OutputBin
    $psi.WorkingDirectory = $BuildDir
    $psi.UseShellExecute = $false
    $psi.CreateNoWindow = $true
    $psi.WindowStyle = [System.Diagnostics.ProcessWindowStyle]::Hidden
    foreach ($name in @($psi.EnvironmentVariables.Keys)) {
        if ($name -match '^(QT_|QML|QSG_)') { $psi.EnvironmentVariables.Remove($name) }
    }
    $psi.EnvironmentVariables['PATH'] = "$env:SystemRoot\System32;$env:SystemRoot"
    $psi.EnvironmentVariables['CAITU_LOG_DIR'] = $smokeDirectory
    Say "Smoke launch with system-only PATH, external working directory, log=$logFile"
    $proc = [System.Diagnostics.Process]::Start($psi)
    try {
        $deadline = [DateTime]::UtcNow.AddSeconds(25)
        while ([DateTime]::UtcNow -lt $deadline -and -not $proc.HasExited) {
            if (Test-Path -LiteralPath $logFile) {
                $logText = Get-Content -LiteralPath $logFile -Encoding utf8 -Raw
                if ($logText -match 'event=qml result=loaded') {
                    $readinessOk = $true
                    break
                }
            }
            Start-Sleep -Milliseconds 200
        }
        if ($readinessOk -and -not $proc.HasExited) {
            $closeOk = $proc.CloseMainWindow()
            if ($closeOk -and $proc.WaitForExit(15000)) {
                $exitCodeInfo = $proc.ExitCode
                $closeOk = $proc.ExitCode -eq 0
            } else { $closeOk = $false }
        }
        Say "Smoke readiness=$readinessOk graceful_close=$closeOk exit=$exitCodeInfo"
    } finally {
        if (-not $proc.HasExited) { $proc.Kill(); $proc.WaitForExit() }
        $proc.Dispose()
    }
}

# ---------- 11) SHA256SUMS（不含自身与运行日志） ----------
$sums = @()
foreach ($file in Get-ChildItem -LiteralPath $Output -Recurse -File) {
    $relative = $file.FullName.Substring($Output.Length + 1).Replace('\', '/')
    if ($relative -eq 'SHA256SUMS') { continue }
    if ($relative -like 'logs/*' -or $relative -like 'Log/*') { continue }
    $sums += '{0}  {1}' -f (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash, $relative
}
$sums | Set-Content -LiteralPath (Join-Path $Output 'SHA256SUMS') -Encoding utf8
Say "SHA256SUMS: $($sums.Count) 个文件"

Say "发布包: $Output"
Say '目标机必须真实安装海康 MVS 驱动；本包只分发运行库。硬件与相机/机器人取图未验证。'

if (-not $SkipSmoke -and (-not $readinessOk -or -not $closeOk)) { exit 1 }
