# Draws the application icon (digitizer arm touching a surface) into src\App\Resources\app.ico
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$outDir = Join-Path $root 'src\App\Resources'
New-Item -ItemType Directory -Force $outDir | Out-Null

function Draw([int]$s) {
    $bmp = New-Object System.Drawing.Bitmap $s, $s
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.SmoothingMode = 'AntiAlias'
    $g.Clear([System.Drawing.Color]::Transparent)
    $k = $s / 256.0
    # rounded dark tile
    $path = New-Object System.Drawing.Drawing2D.GraphicsPath
    $r = 48 * $k; $w = 240 * $k; $o = 8 * $k
    $path.AddArc($o, $o, $r, $r, 180, 90); $path.AddArc($o + $w - $r, $o, $r, $r, 270, 90)
    $path.AddArc($o + $w - $r, $o + $w - $r, $r, $r, 0, 90); $path.AddArc($o, $o + $w - $r, $r, $r, 90, 90); $path.CloseFigure()
    $bg = New-Object System.Drawing.Drawing2D.LinearGradientBrush ([System.Drawing.PointF]::new(0, 0)), ([System.Drawing.PointF]::new(0, $s)), ([System.Drawing.Color]::FromArgb(255, 44, 52, 68)), ([System.Drawing.Color]::FromArgb(255, 16, 19, 26))
    $g.FillPath($bg, $path)
    # measured surface (blue parallelogram)
    $surf = [System.Drawing.PointF[]]@(([System.Drawing.PointF]::new(40 * $k, 205 * $k)), ([System.Drawing.PointF]::new(95 * $k, 160 * $k)), ([System.Drawing.PointF]::new(225 * $k, 160 * $k)), ([System.Drawing.PointF]::new(170 * $k, 205 * $k)))
    $g.FillPolygon((New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(200, 59, 130, 246))), $surf)
    # arm
    $pen = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(255, 226, 232, 240)), (17 * $k)
    $pen.StartCap = 'Round'; $pen.EndCap = 'Round'; $pen.LineJoin = 'Round'
    $pts = [System.Drawing.PointF[]]@(([System.Drawing.PointF]::new(62 * $k, 140 * $k)), ([System.Drawing.PointF]::new(80 * $k, 60 * $k)), ([System.Drawing.PointF]::new(170 * $k, 78 * $k)), ([System.Drawing.PointF]::new(150 * $k, 150 * $k)))
    $g.DrawLines($pen, $pts)
    $g.FillEllipse((New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(255, 148, 163, 184))), 36 * $k, 132 * $k, 52 * $k, 28 * $k)
    foreach ($p in $pts[1..2]) { $g.FillEllipse((New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(255, 249, 115, 22))), $p.X - 12 * $k, $p.Y - 12 * $k, 24 * $k, 24 * $k) }
    # tip touching the surface
    $g.FillEllipse((New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(255, 74, 222, 128))), 138 * $k, 166 * $k, 24 * $k, 24 * $k)
    $g.Dispose()
    return $bmp
}

function Dib([System.Drawing.Bitmap]$b) {
    # 32-bit BGRA DIB icon entry: BITMAPINFOHEADER (double height) + bottom-up pixels + AND mask
    $s = $b.Width
    $ms = New-Object System.IO.MemoryStream
    $w = New-Object System.IO.BinaryWriter $ms
    $w.Write([uint32]40); $w.Write([int32]$s); $w.Write([int32]($s * 2)); $w.Write([uint16]1); $w.Write([uint16]32)
    $w.Write([uint32]0); $w.Write([uint32]0); $w.Write([int32]0); $w.Write([int32]0); $w.Write([uint32]0); $w.Write([uint32]0)
    for ($y = $s - 1; $y -ge 0; $y--) { for ($x = 0; $x -lt $s; $x++) { $c = $b.GetPixel($x, $y); $w.Write([byte]$c.B); $w.Write([byte]$c.G); $w.Write([byte]$c.R); $w.Write([byte]$c.A) } }
    $rowBytes = [int]([math]::Ceiling($s / 32.0) * 4)
    for ($y = 0; $y -lt $s; $y++) { $w.Write((New-Object byte[] $rowBytes)) }
    $w.Flush()
    return , $ms.ToArray()
}

$sizes = 256, 64, 48, 32, 16
$pngs = @()
foreach ($s in $sizes) {
    $b = Draw $s
    if ($s -ge 256) {
        $ms = New-Object System.IO.MemoryStream
        $b.Save($ms, [System.Drawing.Imaging.ImageFormat]::Png)
        $pngs += , $ms.ToArray()
    } else { $pngs += , (Dib $b) }
    $b.Dispose()
}
$ico = New-Object System.IO.MemoryStream
$bw = New-Object System.IO.BinaryWriter $ico
$bw.Write([uint16]0); $bw.Write([uint16]1); $bw.Write([uint16]$sizes.Count)
$offset = 6 + 16 * $sizes.Count
for ($i = 0; $i -lt $sizes.Count; $i++) {
    $s = $sizes[$i]; $d = $pngs[$i]
    $bw.Write([byte]($(if ($s -ge 256) { 0 } else { $s }))); $bw.Write([byte]($(if ($s -ge 256) { 0 } else { $s })))
    $bw.Write([byte]0); $bw.Write([byte]0); $bw.Write([uint16]1); $bw.Write([uint16]32)
    $bw.Write([uint32]$d.Length); $bw.Write([uint32]$offset)
    $offset += $d.Length
}
foreach ($d in $pngs) { $bw.Write($d) }
$bw.Flush()
[System.IO.File]::WriteAllBytes((Join-Path $outDir 'app.ico'), $ico.ToArray())
Write-Host "Wrote $(Join-Path $outDir 'app.ico')"
