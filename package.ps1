# Builds everything and assembles a portable distribution in dist\ (+ zip).
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
& powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $root 'build.ps1') -Target all
if ($LASTEXITCODE -ne 0) { throw 'build failed' }
& (Join-Path $root 'bin\tools\MsTests.exe')
if ($LASTEXITCODE -ne 0) { throw 'tests failed' }

$ver = (Get-Item (Join-Path $root 'bin\MicroScribeInspector.exe')).VersionInfo.FileVersion
$name = 'MicroScribe Inspector'
$dist = Join-Path $root 'dist'
$dir = Join-Path $dist $name
if (Test-Path $dir) { Remove-Item -Recurse -Force $dir }
New-Item -ItemType Directory -Force $dir, (Join-Path $dir 'tools'), (Join-Path $dir 'docs') | Out-Null
Copy-Item (Join-Path $root 'bin\MicroScribeInspector.exe') $dir
Copy-Item (Join-Path $root 'bin\tools\MsProbe.exe') (Join-Path $dir 'tools')
Copy-Item (Join-Path $root 'docs\*') (Join-Path $dir 'docs') -Recurse
Copy-Item (Join-Path $root 'README.md') (Join-Path $dir 'docs\README.md')
Copy-Item (Join-Path $root 'LICENSE') (Join-Path $dir 'LICENSE.txt')
Copy-Item (Join-Path $root 'LEGAL.md') (Join-Path $dir 'docs\LEGAL.md')
Set-Content -Path (Join-Path $dir 'Install for this user.cmd') -Encoding ASCII -Value "@echo off`r`n""%~dp0MicroScribeInspector.exe"" /install /desktop`r`n"
$zip = Join-Path $dist ("MicroScribeInspector-$ver-portable.zip")
if (Test-Path $zip) { Remove-Item $zip }
Add-Type -AssemblyName System.IO.Compression.FileSystem
[System.IO.Compression.ZipFile]::CreateFromDirectory($dir, $zip, 'Optimal', $true)
Write-Host "Portable folder: $dir"
Write-Host "Zip:             $zip"
