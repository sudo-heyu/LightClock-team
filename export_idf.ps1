# Wrapper to activate ESP-IDF environment in the current PowerShell session.
# Usage:
#   .\export_idf.ps1

$ErrorActionPreference = 'Stop'

$env:IDF_PATH = 'F:\ESP32\ESP-IDF'
$env:IDF_TOOLS_PATH = 'F:\ESP32\IDF_TOOLS\Espressif'
$env:IDF_PYTHON_ENV_PATH = 'F:\ESP32\IDF_TOOLS\Espressif\python_env\idf5.5_py3.11_env'

Set-ExecutionPolicy -Scope Process Bypass -Force
. "$env:IDF_PATH\export.ps1"
