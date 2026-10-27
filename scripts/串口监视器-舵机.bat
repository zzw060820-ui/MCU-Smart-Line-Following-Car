@echo off
setlocal
title 循线小车 - 舵机中值  (CALIB_MODE 1)

echo ============================================================
echo       循线小车   舵机中值  (CALIB_MODE 1)
echo       9600 bps / 8 数据位 / 无校验 / 1 停止位
echo ============================================================
echo.
echo   前提：板子里烧的是 CALIB_MODE = 1 的程序。
echo.
echo   你应该看到什么（数字一点一点往外弹，扫到头自动掉头往回扫）：
echo.
echo       SERVO_C = 1480
echo       SERVO_C = 1482
echo       SERVO_C = 1484
echo       ...
echo.
echo   盯【每一个轮子】：数值扫过哪一点时，它【反转方向】—— 那个点就是它的中值。
echo   左右轮中值不一样是正常的，最后取两个中值的【平均】填回 SERVO_C。
echo.

set "PORT=%~1"
if "%PORT%"=="" (
    echo 当前系统上可用的串口：
    powershell -NoProfile -Command "[System.IO.Ports.SerialPort]::GetPortNames() -join '   '"
    echo.
    set /p "PORT=请输入串口号（例如 COM6）: "
)

if "%PORT%"=="" (
    echo 没有选择串口，退出。
    pause
    exit /b 1
)

echo.
echo 正在打开 %PORT% ...
echo 按 Ctrl+C 可以随时中断
echo ------------------------------------------------------------
echo.

powershell -NoProfile -ExecutionPolicy Bypass -Command "$ErrorActionPreference='Continue'; $want='%PORT%'; $avail=[System.IO.Ports.SerialPort]::GetPortNames(); if ($avail -notcontains $want) { Write-Host ''; Write-Host ('  [打不开] ' + $want + ' 现在不在系统里。') -ForegroundColor Red; Write-Host ('  系统现在真正可用的串口只有: ' + $(if ($avail) { $avail -join ', ' } else { '(一个都没有)' })) -ForegroundColor Yellow; Write-Host ''; Write-Host '  按顺序检查：' -ForegroundColor Cyan; Write-Host '    1) 板子 USB 线插好了吗？换一根线试试（有的线只能充电、不能传数据）' -ForegroundColor Cyan; Write-Host '    2) 换个电脑 USB 口（别用扩展坞）' -ForegroundColor Cyan; Write-Host '    3) 板子「下载与串口切换开关」拨到 USART 了吗？' -ForegroundColor Cyan; Write-Host '    4) 打开设备管理器，看有没有新出现带感叹号的设备' -ForegroundColor Cyan; Write-Host ''; Read-Host '  按 Enter 退出'; exit 1 }; $p = New-Object System.IO.Ports.SerialPort; $p.PortName=$want; $p.BaudRate=9600; $p.Parity='None'; $p.DataBits=8; $p.StopBits='One'; $p.Handshake='None'; try { $p.Open() } catch { Write-Host ('  [打开 ' + $want + ' 失败] ' + $_.Exception.Message) -ForegroundColor Red; Read-Host '  按 Enter 退出'; exit 1 }; $p.DiscardInBuffer(); Write-Host ('  [已打开 ' + $want + '，等待板子输出...]') -ForegroundColor Green; Write-Host '  （按一下板子上的系统复位键，应该出现 SERVO_C = 1480 这样的数字）' -ForegroundColor Green; Write-Host ''; while ($true) { $s = $p.ReadExisting(); if ($s) { Write-Host -NoNewline $s }; Start-Sleep -Milliseconds 50 }"

echo.
echo.
echo （串口已关闭）
pause
endlocal
