# Builds MicroScribe Inspector and tools with the C# compiler that ships with Windows
# (.NET Framework 4.x, C# 5). No SDK or Visual Studio needed.
#   powershell -ExecutionPolicy Bypass -File build.ps1 [-Target all|app|probe|tests] [-Debug]
param(
    [ValidateSet('all', 'app', 'probe', 'tests')] [string]$Target = 'all',
    [switch]$Debug
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$csc = Join-Path $env:WINDIR 'Microsoft.NET\Framework64\v4.0.30319\csc.exe'
if (-not (Test-Path $csc)) { $csc = Join-Path $env:WINDIR 'Microsoft.NET\Framework\v4.0.30319\csc.exe' }
$fx = Split-Path $csc
$wpf = Join-Path $fx 'WPF'
$bin = Join-Path $root 'bin'
New-Item -ItemType Directory -Force $bin, (Join-Path $bin 'tools') | Out-Null

function Src([string[]]$dirs) {
    $files = @()
    foreach ($d in $dirs) { $files += Get-ChildItem (Join-Path $root $d) -Filter *.cs -Recurse | ForEach-Object { $_.FullName } }
    return $files
}

function Compile([string]$out, [string]$kind, [string[]]$sources, [string[]]$refs, [string]$platform = 'anycpu', [string[]]$extra = @()) {
    $opt = if ($Debug) { @('/debug+', '/define:DEBUG') } else { @('/optimize+', '/debug:pdbonly') }
    $args = @('/nologo', "/target:$kind", "/platform:$platform", "/out:$out", '/warn:4', '/nowarn:1591') + $opt + $extra
    foreach ($r in $refs) { $args += "/reference:$r" }
    $args += $sources
    Write-Host "csc -> $out" -ForegroundColor Cyan
    & $csc @args
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $out" }
}

$core = Src @('src\Core', 'src\Driver')

if ($Target -in 'all', 'probe') {
    Compile (Join-Path $bin 'tools\MsProbe.exe') 'exe' ($core + (Src @('src\Tools\MsProbe'))) @('System.dll') 'x64'
}

if ($Target -in 'all', 'tests') {
    if (Test-Path (Join-Path $root 'src\Tests')) {
        Compile (Join-Path $bin 'tools\MsTests.exe') 'exe' ($core + (Src @('src\Geometry', 'src\Export', 'src\Tests'))) @('System.dll', 'System.Core.dll')
    }
}

if ($Target -in 'all', 'app') {
    if (Test-Path (Join-Path $root 'src\App')) {
        $refs = @('System.dll', 'System.Core.dll', 'System.Xaml.dll', 'System.Xml.dll',
            (Join-Path $wpf 'PresentationCore.dll'), (Join-Path $wpf 'PresentationFramework.dll'),
            (Join-Path $wpf 'WindowsBase.dll'), 'System.Windows.Forms.dll', 'System.Drawing.dll')
        $icon = Join-Path $root 'src\App\Resources\app.ico'
        $extra = @()
        if (Test-Path $icon) { $extra += "/win32icon:$icon" }
        $manifest = Join-Path $root 'src\App\app.manifest'
        if (Test-Path $manifest) { $extra += "/win32manifest:$manifest" }
        Compile (Join-Path $bin 'MicroScribeInspector.exe') 'winexe' ($core + (Src @('src\Geometry', 'src\Export', 'src\App'))) $refs 'anycpu' $extra
    }
}
Write-Host 'Build OK' -ForegroundColor Green
