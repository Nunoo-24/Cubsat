param(
    [Parameter(Mandatory = $true)]
    [ValidateRange(1, 65535)]
    [int]$Port,

    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$Url,

    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string]$ExpectedText,

    [ValidateRange(1, 300)]
    [int]$TimeoutSeconds = 45
)

$deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
while ([DateTime]::UtcNow -lt $deadline) {
    try {
        $response = Invoke-WebRequest -UseBasicParsing -Uri $Url -TimeoutSec 2
        if ([int]$response.StatusCode -eq 200 -and
            $response.Content.Contains($ExpectedText)) {
            Write-Host "Service on port $Port is ready at $Url"
            exit 0
        }
    }
    catch {
        # The server may still be starting or Windows-to-WSL forwarding may
        # not be available yet. Keep polling until the explicit deadline.
    }

    Start-Sleep -Milliseconds 500
}

Write-Error "Expected page content was not returned from $Url within $TimeoutSeconds seconds. Check the matching service terminal and Windows-to-WSL localhost forwarding."
exit 1
