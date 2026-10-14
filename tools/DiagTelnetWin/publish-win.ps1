param(
    [ValidateSet("win-x64","win-arm64")]
    [string]$Runtime = "win-x64",

    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"

Write-Host "== DiagTelnetWin publish ==" -ForegroundColor Cyan

# Ir a la raíz del repo desde tools/DiagTelnetWin
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot  = Resolve-Path (Join-Path $ScriptDir "..\..")
Set-Location $RepoRoot

$Project = "tools/DiagTelnetWin/DiagTelnetWin.csproj"

Write-Host "Proyecto: $Project"
Write-Host "Runtime : $Runtime"
Write-Host "Config  : $Configuration"
Write-Host ""

# Verificar dotnet
try {
    $dotnetVersion = (dotnet --version).Trim()
    Write-Host "dotnet SDK detectado: $dotnetVersion" -ForegroundColor Green
} catch {
    Write-Error "No se encontró dotnet en PATH. Instala .NET 8 SDK y vuelve a intentar."
    exit 1
}

# Restaurar y publicar
dotnet restore $Project
if ($LASTEXITCODE -ne 0) { throw "Falló dotnet restore" }

dotnet publish $Project `
  -c $Configuration `
  -r $Runtime `
  --self-contained true `
  -p:PublishSingleFile=true `
  -p:DebugType=None `
  -p:DebugSymbols=false

if ($LASTEXITCODE -ne 0) { throw "Falló dotnet publish" }

$OutDir = "tools/DiagTelnetWin/bin/$Configuration/net8.0-windows/$Runtime/publish"
$ExePath = Join-Path $OutDir "DiagTelnetWin.exe"

Write-Host ""
Write-Host "Publicación completada." -ForegroundColor Green
Write-Host "Salida: $OutDir"

if (Test-Path $ExePath) {
    Write-Host "EXE: $ExePath" -ForegroundColor Yellow
} else {
    Write-Warning "No se encontró el EXE esperado en: $ExePath"
}
