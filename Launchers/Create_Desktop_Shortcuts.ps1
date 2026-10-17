$ErrorActionPreference = 'Stop'

$desktopPath = [Environment]::GetFolderPath('Desktop')
$shell = New-Object -ComObject WScript.Shell
$launchers = @(
    @{ Name = 'AE 1 - Simulation Environment'; File = 'Start_01_Simulation_Environment.bat' },
    @{ Name = 'AE 2 - CubeSat SIL'; File = 'Start_02_CubeSat_SIL.bat' },
    @{ Name = 'AE 3 - Ground Station'; File = 'Start_03_Ground_Station.bat' }
)

foreach ($launcher in $launchers) {
    $targetPath = Join-Path $PSScriptRoot $launcher.File
    if (-not (Test-Path -LiteralPath $targetPath)) {
        throw "Launcher not found: $targetPath"
    }

    $shortcutPath = Join-Path $desktopPath ($launcher.Name + '.lnk')
    $shortcut = $shell.CreateShortcut($shortcutPath)
    $shortcut.TargetPath = $targetPath
    $shortcut.WorkingDirectory = $PSScriptRoot
    $shortcut.Description = 'Start the AE CubeSat first test drive component'
    $shortcut.Save()
}

Write-Host "Created three AE test-drive shortcuts on: $desktopPath"
