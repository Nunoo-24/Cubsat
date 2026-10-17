$ErrorActionPreference = 'Continue'
$distro = 'Ubuntu'
$wslExe = Join-Path $env:SystemRoot 'System32\wsl.exe'
$linuxDiagnostic = '/home/nunot/Mestrado_Codex/AE/outputs/Python_sim/NR/Test_drive_1/outputs/Python_sim/Launchers/diagnose_ports.py'
$desktop = [Environment]::GetFolderPath('Desktop')
$transcript = Join-Path $desktop 'AE_Connection_Diagnostic.txt'

Start-Transcript -Path $transcript -Force | Out-Null
try {
    Write-Host 'AE first test-drive connection diagnostic'
    Write-Host "Time: $(Get-Date -Format o)"
    Write-Host "Expected WSL distribution: $distro"
    Write-Host "Windows host: $env:COMPUTERNAME"
    Write-Host ''

    Write-Host '--- Registered WSL distributions ---'
    & $wslExe --list --quiet
    Write-Host "wsl --list exit code: $LASTEXITCODE"
    Write-Host ''

    Write-Host '--- WSL-local Python and service checks ---'
    & $wslExe --distribution $distro --exec python3 $linuxDiagnostic
    Write-Host "WSL diagnostic exit code: $LASTEXITCODE"
    Write-Host ''

    Write-Host '--- Windows listener table ---'
    $listeners = Get-NetTCPConnection -State Listen -LocalPort 8000,8001 -ErrorAction SilentlyContinue
    if ($listeners) {
        $listeners | Select-Object LocalAddress, LocalPort, OwningProcess | Format-Table -AutoSize
    } else {
        Write-Host 'No Windows listener entries found for ports 8000 or 8001.'
    }
    Write-Host ''

    foreach ($port in @(8000, 8001)) {
        Write-Host "--- Windows connection test: 127.0.0.1:$port ---"
        Test-NetConnection -ComputerName 127.0.0.1 -Port $port -InformationLevel Detailed -WarningAction SilentlyContinue
    }
}
finally {
    Stop-Transcript | Out-Null
    Write-Host "Diagnostic saved to: $transcript"
}
