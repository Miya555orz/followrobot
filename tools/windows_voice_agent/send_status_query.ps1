param(
    [string]$JetsonUrl = "http://192.168.50.35:8081"
)

$ErrorActionPreference = "Stop"

$token = $env:FCR_VOICE_AUTH_TOKEN
if ([string]::IsNullOrWhiteSpace($token)) {
    throw "FCR_VOICE_AUTH_TOKEN is empty. Set the same token on Windows and Jetson first."
}

$requestId = [guid]::NewGuid().ToString()
$payload = @{
    protocol_version = 1
    request_id       = $requestId
    timestamp        = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() / 1000.0
    raw_text         = "查询系统状态"
    source           = "windows_voice_agent_smoke_test"
    intents          = @("status_query")
    confidences      = @(1.0)
    parameters       = @{
        distance          = -1.0
        unit              = ""
        distance_relative = $false
        angle             = -1.0
        direction         = 0
        speed             = ""
        target_desc       = ""
        follow            = $false
    }
} | ConvertTo-Json -Depth 6

$headers = @{ Authorization = "Bearer $token" }
$response = Invoke-RestMethod `
    -Method Post `
    -Uri "$($JetsonUrl.TrimEnd('/'))/voice/command" `
    -Headers $headers `
    -ContentType "application/json; charset=utf-8" `
    -Body ([Text.Encoding]::UTF8.GetBytes($payload))

$response | ConvertTo-Json -Depth 6
Write-Host "Queued request_id=$requestId. Check final Jetson decision on /voice/dispatch_status."
