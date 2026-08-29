param(
  [string]$Root = "D:\followrobot"
)

$ErrorActionPreference = "Stop"

$repos = @(
  @("https://github.com/cuiangA/fcr_ros2_3.git", "upstream\cuiangA\fcr_ros2_3"),
  @("https://github.com/limxdynamics/limxsdk-lowlevel.git", "upstream\limxdynamics\limxsdk-lowlevel"),
  @("https://github.com/limxdynamics/tron1-rl-deploy-ros2.git", "upstream\limxdynamics\tron1-rl-deploy-ros2"),
  @("https://github.com/limxdynamics/tron1-gazebo-ros2.git", "upstream\limxdynamics\tron1-gazebo-ros2"),
  @("https://github.com/limxdynamics/tron1-robot-description.git", "upstream\limxdynamics\tron1-robot-description"),
  @("https://github.com/limxdynamics/robot-visualization.git", "upstream\limxdynamics\robot-visualization"),
  @("https://github.com/limxdynamics/ros2-bridger.git", "upstream\limxdynamics\ros2-bridger"),
  @("https://github.com/limxdynamics/robot-joystick.git", "upstream\limxdynamics\robot-joystick")
)

foreach ($repo in $repos) {
  $url = $repo[0]
  $dir = Join-Path $Root $repo[1]
  $parent = Split-Path -Parent $dir
  New-Item -ItemType Directory -Force -Path $parent | Out-Null

  if (Test-Path -LiteralPath (Join-Path $dir ".git")) {
    git -C $dir fetch --all --prune
  } elseif (Test-Path -LiteralPath $dir) {
    Write-Warning "Skipping non-git path: $dir"
  } else {
    git clone $url $dir
  }
}
