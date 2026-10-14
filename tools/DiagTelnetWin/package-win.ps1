param(
    [ValidateSet("win-x64","win-arm64")]
    [string]$Runtime = "win-x64",

    [string]$Configuration = "Release",

    [string]$Version = "v1.0.0"
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = (Resolve-Path (Join-Path $ScriptDir "../..")).Path
$Project = Join-Path $ScriptDir "DiagTelnetWin.csproj"
$PublishDir = Join-Path $ScriptDir "bin/$Configuration/net8.0-windows/$Runtime/publish"
$PackageName = "DiagTelnetWin-$Runtime"
$PackageDir = Join-Path $RepoRoot "dist/$PackageName"
$ZipPath = Join-Path $RepoRoot "dist/$PackageName.zip"

Write-Host "== Empaquetado DiagTelnetWin ==" -ForegroundColor Cyan
Write-Host "Runtime: $Runtime | Configuración: $Configuration | Versión: $Version"

# Eliminar publicaciones anteriores para no reutilizar archivos obsoletos.
if (Test-Path -LiteralPath $PublishDir) {
    Remove-Item -LiteralPath $PublishDir -Recurse -Force
}

dotnet restore $Project -r $Runtime -p:PublishSingleFile=true -p:SelfContained=true
if ($LASTEXITCODE -ne 0) { throw "Falló dotnet restore" }

dotnet publish $Project `
    -c $Configuration `
    -r $Runtime `
    --no-restore `
    --self-contained true `
    -p:PublishSingleFile=true `
    -p:DebugType=None `
    -p:DebugSymbols=false `
    --output $PublishDir
if ($LASTEXITCODE -ne 0) { throw "Falló dotnet publish" }

if (-not (Test-Path -LiteralPath $PublishDir -PathType Container)) {
    throw "No existe la carpeta publish: $PublishDir"
}
if (-not (Test-Path -LiteralPath (Join-Path $PublishDir "DiagTelnetWin.exe") -PathType Leaf)) {
    throw "No se encontró DiagTelnetWin.exe en: $PublishDir"
}

if (Test-Path -LiteralPath $PackageDir) {
    Remove-Item -LiteralPath $PackageDir -Recurse -Force
}
New-Item -ItemType Directory -Path $PackageDir -Force | Out-Null
Get-ChildItem -LiteralPath $PublishDir -Force |
    Copy-Item -Destination $PackageDir -Recurse -Force

$Readme = @"
DiagTelnetWin - paquete portable ($Runtime)

Ejecución en Windows 10/11:
1) Extrae todo el ZIP en una carpeta y ejecuta DiagTelnetWin.exe.
2) Introduce la IP real del dispositivo (ejemplo: 192.168.1.22).
3) Usa el puerto de diagnóstico 15920 y un intervalo de 1000 ms.
4) Pulsa Conectar.

Notas:
- No usar el puerto 15919: es NMEA, no diagnóstico.
- No requiere instalación ni instalar .NET (paquete self-contained).
- Utilizar únicamente en una red local de confianza.
"@
Set-Content -LiteralPath (Join-Path $PackageDir "README-EJECUCION.txt") -Value $Readme -Encoding UTF8

$VersionText = @"
Versión: $Version
Runtime: $Runtime
Timestamp: $(Get-Date -Format "o")
"@
Set-Content -LiteralPath (Join-Path $PackageDir "VERSION.txt") -Value $VersionText -Encoding UTF8

if (Test-Path -LiteralPath $ZipPath) {
    Remove-Item -LiteralPath $ZipPath -Force
}
Compress-Archive -Path (Join-Path $PackageDir "*") -DestinationPath $ZipPath -CompressionLevel Optimal

Write-Host "OK carpeta: $PackageDir" -ForegroundColor Green
Write-Host "OK ZIP: $ZipPath" -ForegroundColor Green
