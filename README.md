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

## 固件功能

`firmware/Ant/god.c` 实现直行、后退、左右原地转向、差速微调和丢线恢复。四路探头的决策量为 `err = -3*LL - L + R + 3*RR`：居中直行，小偏差微调，大偏差转向；全部丢线时先后退，再按最近一次方向找线。

源码当前配置：`SERVO_C=1520`、`SPD_GO=200`、`SPD_TURN=450`、`SPD_BACK=400`、`SPD_TRIM=150`、`CALIB_MODE=0`、`USE_LINE_MEMORY=1`。保留舵机中值扫描与探头观测模式。

舵机信号采用软件延时、两路顺序输出。`FRAME_MS=20` 是额外延时，并不保证整帧恰好 20 ms；实际时间还受延时函数、编译设置与晶振影响。本仓库未改动控制逻辑，也不据此声称新的实车效果。

## 强化学习模块

这是实际的**表格型 Q-learning 模拟训练**，并非仅执行固定动作或遍历参数。智能体使用 ε-greedy 选动作，并按下式更新 16×6 的 Q 表：

`Q(s,a) ← Q(s,a) + α[r + γ max Q(s′,a′) − Q(s,a)]`

四路探头编码为 16 个离散状态，动作为直行、左转、右转、左微调、右微调、后退。环境以简化差速模型模拟横向偏移和航向，使用假设的轮距、脉宽到速度映射和赛道参数；奖励鼓励保持在线和重新找线，惩罚转向、后退及丢线失败。训练在电脑上离线完成，不在单片机上训练。

导出器生成 16 字节策略数组和 96 字节量化 Q 数组。当前策略中的 8 个全零 Q 行使用规则兜底，这些动作不是学习得到的；“全零行”只是导出器判断未学习状态的启发式条件，没有独立的状态访问计数。导出头文件未被 `god.c` 引用。

已有 JSON、CSV 和头文件按原字节保留，未重新训练。评估结果存在策略名称误标问题，不能只凭 JSON 的 `policy` 字段识别算法；参见 [审计说明](docs/AUDIT.md) 和 [历史结果说明](reinforcement-learning/results/README.md)。

## 目录

```text
firmware/Ant/                 固件源码与 Keil 工程
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

先阅读 [外部头文件依赖](docs/DEPENDENCIES.md)，在本机从合法来源补齐 `uart.h`、`Boebot.h` 和 `AT89X52.H`。它们未纳入上传内容，**刚克隆的仓库尚不能直接完整编译固件**。

用 Keil 打开 `firmware/Ant/KGOD.uvproj`，核对晶振与头文件搜索路径，再编译；工程配置输出为 `Objects/KGOD.hex`。烧录前按实际板卡和执行器完成标定。固件文件保持原编码（GBK），避免编辑器自动转码。两份串口 BAT 使用 Windows PowerShell / .NET 串口类，9600、8N1，运行后选择实际 COM 口；串口模式需要板端对应的标定固件。

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

- 已有规则固件、模拟训练/评估/导出代码及保存产物；本次整理只做静态检查和文件完整性核验。
- 未进行本次固件编译、烧录、硬件测试、模型重训或评估复跑，历史文档中的通过记录不等于本次验证。
- 模拟动作固定时长，未复现固件不同动作的步数和完整丢线状态机；简化基线不能视为完整固件性能。
- 单帧探头状态不能充分区分弯道和直道，模拟模型未按实车标定；不据单一种子结果宣称算法优越。
- 头文件授权、时钟配置、评估元数据问题需后续处理。当前建议 Private，公开前再确认代码与第三方材料的权利范围。

维护和上传方式见 [更新说明](docs/MAINTENANCE.md)。本仓库尚未指定统一开源许可证，不代表第三方材料获得了再分发授权。
