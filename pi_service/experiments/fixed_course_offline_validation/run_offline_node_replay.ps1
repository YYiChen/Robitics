param(
    [Parameter(Mandatory = $true)]
    [string]$Source,
    [string]$OutputDirectory = (Join-Path $PSScriptRoot "outputs")
)

$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\.."))
New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
& python (Join-Path $PSScriptRoot "offline_node_replay.py") `
    --source $Source `
    --output-video (Join-Path $OutputDirectory "annotated-replay.mp4") `
    --output-jsonl (Join-Path $OutputDirectory "replay.jsonl")
