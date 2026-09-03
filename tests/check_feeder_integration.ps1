# Static integration contracts complement the executable policy/PID tests.
$ErrorActionPreference = 'Stop'
$src = Join-Path (Split-Path $PSScriptRoot -Parent) 'STM32F405'
$checks = 0
function Check([bool]$condition, [string]$message) {
    if (-not $condition) { throw "FAIL integration: $message" }
    $script:checks++
}
$rc = Get-Content -Raw (Join-Path $src 'RC.cpp')
$control = Get-Content -Raw (Join-Path $src 'control.cpp')
$motor = Get-Content -Raw (Join-Path $src 'motor.cpp')
$feeder = Get-Content -Raw (Join-Path $src 'feeder.cpp')
$can = Get-Content -Raw (Join-Path $src 'can.cpp')
$main = Get-Content -Raw (Join-Path $src 'STM32F405.cpp')
Check ($main -match 'Motor\(M3508,\s*SPD,\s*supply,\s*ID5,\s*PID\(20\.f,\s*0\.1f,\s*1\.5f,\s*0\.f\)\)') 'feeder baseline must be P=20, I=0.1, unchanged D=1.5 and alpha=0'
Check ($rc -match 'if\s*\(pd_Rx != pdTRUE\) return;') 'empty RC queue must not refresh the command'
Check ($rc -match 'feeder.UpdateCommand\(rc.s\[0\], rc.s\[1\], rc.ch\[1\], HAL_GetTick\(\)\)') 'vertical channel mapping'
Check ($rc -match 'void RC::OnRC\(\)\s*\{\s*if \(UpdateFireMode\(\)\) return;') 'FIRE precedes chassis joystick mapping'
Check ($rc -match 'fire_selected = rc.s\[0\] == DOWN && rc.s\[1\] == DOWN') 'both DOWN selects FIRE'
Check ($control -match '(?s)void CONTROL::CHASSIS::Update\(\).*?ctrl.mode == RESET \|\| ctrl.mode == FIRE.*?speedx = 0;\s*speedy = 0;\s*speedz = 0;') 'FIRE chassis target is zero'
Check ($control -notmatch 'supply_motor\[0\]->setspeed\s*=') 'no competing feeder target writer'
Check ($motor -match 'if \(function == supply\)\s*feeder.Update\(\*this\);\s*else\s*setcurrent = pid\[speed\].Position') 'only supply speed loop is routed to feeder'
Check ($can -match '(?s)hcan == &can2.hcan && id == 0x205.*?IDE == CAN_ID_STD.*?RTR == CAN_RTR_DATA.*?DLC == 8\)\s*feeder.OnFeedback') 'feedback belongs to CAN2 ID5 standard data frame'
$project = Get-Content -Raw (Join-Path $src 'STM32F405.vcxproj')
Check ($project -match '<ClCompile Include="feeder.cpp"') 'module is compiled'
$header = Get-Content -Raw (Join-Path $src 'control.h')
Check ($header -match 'AUTO, FIRE') 'FIRE appended without renumbering old modes'
Check ($feeder -match 'taskENTER_CRITICAL\(\);\s*(?://[^\r\n]*\r?\n\s*)*const uint32_t tick = HAL_GetTick\(\);') 'read tick AFTER masking feedback IRQ to avoid unsigned time underflow'
Write-Output "PASS integration contracts: $checks checks"
