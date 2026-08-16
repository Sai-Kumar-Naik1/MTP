$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
& "$PSScriptRoot\venv\Scripts\python.exe" -m gui.map_window
