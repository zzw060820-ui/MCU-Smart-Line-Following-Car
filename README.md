# MCU-Smart-Line-Following-Car

基于 AT89S52 的四路数字传感器循迹小车工程，以及独立的表格型 Q-learning 模拟训练实验。项目用于保存课程开发代码、研究流程与已有结果，便于后续维护。

**当前固件使用规则控制。强化学习策略尚未集成到固件，已有训练和评估指标仅来自简化模拟环境。**

## 硬件与软件

| 项目 | 依据与现状 |
| --- | --- |
| 控制器 | Keil 工程选择 AT89S52；历史资料描述为 C51 机器人教学板 |
| 检测 | 四路 QTI 数字输入：最左 P0.7、左中 P0.6、右中 P0.5、最右 P0.4；程序约定黑线为 1 |
| 执行器 | 左右连续旋转舵机，信号引脚分别为 P1.1、P1.0，右轮镜像映射 |
| 开发环境 | Windows、Keil uVision / C51；历史烧录流程使用 USBasp / ProgISP，工具需自行合法安装 |
| 时钟 | 本地串口头文件按 11.0592 MHz 配置；工程 XML 中 CLOCK 为 33 MHz，二者不一致，需根据实物核对 |
| Python | 代码要求 Python 3.8 或以上，使用标准库，无第三方 Python 依赖 |

## 固件功能（FIELD-1）

`firmware/Ant/` 同步自当前采用的 FIELD-1 工程，控制集中在 `god.c`。2026-10-28 收工记录明确选择 FIELD-1，FIELD-2 暂缓；没有按文件时间将试验版自动替换为主版本。

- 上电进入 WAIT，连续 2 帧捕线图案后自动起步；找不到线时等待，重新放到线上可恢复。
- TRACK 按四路加权误差 `err = -3*LL - L + R + 3*RR` 控制；宽线或全黑不会直接锁停。
- TURN 保持方向，3 帧后允许捕线、最多 32 帧；SEARCH 按 20/40/20 帧有限搜索，当前不使用后退动作。
- 捕线后低速过渡 6 帧；未完成恢复的事件最多 160 帧，然后回 WAIT。
- 常规参数沿用 `SERVO_C=1520`、`SPD_GO=200`、`SPD_TRIM=150`、`SPD_TURN=450`；等待脉宽为左 1535、右 1505，尚未确认物理零速，WAIT 不能代替急停。

FIELD-1 仅在上电输出串口提示，不包含旧 Ant 的 `CALIB_MODE` 扫描/探头观测入口。旧代码保留在 Git 历史中，需要旧标定功能时应另外取出历史版本，避免混用固件。

软件延时和两路顺序脉冲意味着帧长并非精确 20 ms。本次仅同步已有代码，不修改控制逻辑和参数。

## 强化学习模块

这是实际的**表格型 Q-learning 模拟训练**，并非仅执行固定动作或遍历参数。智能体使用 ε-greedy 选动作，并按下式更新 16×6 的 Q 表：

`Q(s,a) ← Q(s,a) + α[r + γ max Q(s′,a′) − Q(s,a)]`

四路探头编码为 16 个离散状态，动作为直行、左转、右转、左微调、右微调、后退。环境以简化差速模型模拟横向偏移和航向，使用假设的轮距、脉宽到速度映射和赛道参数；奖励鼓励保持在线和重新找线，惩罚转向、后退及丢线失败。训练在电脑上离线完成，不在单片机上训练。

导出器生成 16 字节策略数组和 96 字节量化 Q 数组。当前策略中的 8 个全零 Q 行使用规则兜底，这些动作不是学习得到的；“全零行”只是导出器判断未学习状态的启发式条件，没有独立的状态访问计数。导出头文件未被 `god.c` 引用。

已有 JSON、CSV 和头文件按原字节保留，未重新训练。评估结果存在策略名称误标问题，不能只凭 JSON 的 `policy` 字段识别算法；参见 [审计说明](docs/AUDIT.md) 和 [历史结果说明](reinforcement-learning/results/README.md)。

## 目录

```text
firmware/Ant/                 FIELD-1 源码、必要头文件、Keil 工程和主机测试
reinforcement-learning/
  src/                        环境、Q-learning、训练、评估、导出
  tests/                      原有 unittest 测试（部分会训练）
  tools/                      演示与策略头文件编译检查源码
  firmware_policy/            已有导出头文件
  results/                    已有 Q 表、训练日志、汇总与评估 JSON
scripts/                      串口监视器和显式训练入口
docs/                         审计、依赖、排除范围与更新说明
```

## 使用

### 固件

使用 Windows 与合法安装的 Keil uVision/C51，打开 `firmware/Ant/KGOD.uvproj`，执行 Rebuild。工程只编译 `god.c`，输出为 `Objects/KGOD_FIELD.hex`；三个必需的项目头文件已随工程保留。工具链标准头文件、运行库与启动代码仍由 Keil 提供，见 [依赖说明](docs/DEPENDENCIES.md)。

源码及第三方头文件保持原始 GBK/ASCII 字节；不要自动转为 UTF-8。代码 include 与文件名存在大小写差异（`BoeBot.h`/`Boebot.h`、`AT89X52.h`/`AT89X52.H`），当前面向大小写不敏感的 Windows 文件系统。

工程 XML 的 `CLOCK(33000000)` 与 `uart.h` 的 `XTAL=11059200` 不一致；本次保留原配置，烧录前应根据实物核对。使用 USBasp/ProgISP 按板卡流程烧录，并先架空确认轮子方向和等待时的停转情况。

已有主机测试可用 MinGW GCC 执行（PowerShell，仓库根目录，`gcc` 需在 PATH 中）：

```powershell
Push-Location .\firmware\Ant
gcc -std=c89 -pedantic -Wall -Wextra -Werror -O2 -Itests/stubs tests/test_field.c -o tests/test_field.exe
if ($LASTEXITCODE -eq 0) { .\tests\test_field.exe }
Pop-Location
```

测试使用模拟寄存器和延时桩，不能验证实物时序。不要定义 `TEST_OLD_FSM`：它是旧工程对比入口，需要未纳入此仓库的历史 FSM 工程。测试文件不参与 Keil 固件编译。

两份串口 BAT 保留为辅助工具；FIELD-1 只输出上电提示，不会持续输出旧标定数据。

### Python 模拟实验

以下为用户主动运行的命令，本次整理没有执行它们。PowerShell 从仓库根目录进入模块：

```powershell
Set-Location .\reinforcement-learning
python src/evaluate.py --episodes 300 --seed 12345
python src/evaluate.py --baseline --episodes 300 --seed 12345
```

不带 `--save` 的评估不会改写已保存结果，但当前打印的策略标签存在已知错误。若明确要重新训练和覆盖结果，再使用：

```powershell
python src/train.py --episodes 6000 --seed 0
python src/evaluate.py --episodes 300 --seed 12345 --save
python src/export_policy.py
```

也可以在 Windows 中主动运行 `scripts/run-training.bat`；该入口会训练并覆盖发布副本内的训练产物。原有完整测试命令为 `python -m unittest discover -s tests -v`，其中部分测试会短程训练，不能将其视为纯静态检查。

## 进度与局限

- 本轮核验、来源与版本选择见 [2026-10-29 更新说明](docs/UPDATE_20261029.md)。
- 本地 2026-10-28 收工记录记载 FIELD-1 已跑完矩形和三角形；五角星最小尖角仍未通过。这是既有现场反馈，本次同步没有重做实车试验，也不将失败原因确定为已证明的硬件极限。
- 源码及早期说明中的“未实车”注释早于收工记录；本次保留源码原字节，通过本节说明时间差异。
- WAIT 停车脉宽未校准，复杂线路可能误捕；不能据软件测试声称小车已全面可靠。
- 强化学习仍是独立模拟实验，未集成 FIELD-1；原有模拟数据、标签问题和简化模型边界见 [历史审计](docs/AUDIT.md)。本轮没有重训或改写结果。
- 工程时钟配置差异仍需核对。第三方头文件保留原声明；仓库没有统一开源许可证，也不据文件收录推定取得第三方再分发许可。

维护和上传方式见 [更新说明](docs/MAINTENANCE.md)。
