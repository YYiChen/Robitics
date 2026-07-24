param(
    [Parameter(Mandatory = $true)]
    [string]$StreamUrl
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$python = (Get-Command python -ErrorAction Stop).Source
& $python (Join-Path $root 'windows_mjpeg_recorder.py') --stream-url $StreamUrl --fps 5
exit $LASTEXITCODE
