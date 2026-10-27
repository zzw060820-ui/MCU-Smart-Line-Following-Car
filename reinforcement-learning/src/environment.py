"""
四路数字巡线传感器的简化巡线模拟环境
=====================================

!!!! 定位说明（非常重要，请先读这一段）!!!!

本环境是一个【用来验证算法流程跑的通的极简模型】，不是真实小车的物理仿真。
它刻意简化或忽略掉了下面这些东西：

  - 连续旋转舵机的死区、两颗舵机中值不一致（实测右轮≈1498、左轮偏差更大）
  - 舵机响应延迟、加减速过程
  - 轮子与地面的打滑、轮径误差
  - QTI 探头的作用距离、阈值个体差异、采样时刻不同步
  - 电池电压下降导致的速度变化
  - 车体质量与惯性

因此：**在本环境里训练出来的策略，只能证明"训练→导出→部署"这条流程是通的，
不能当作真实小车性能的结论。** 真实性能必须在实体车上测（见 05_对照实验设计.md）。

--------------------------------------------------------------------
状态编码：与 God.c 的传感器顺序、极性严格保持一致
--------------------------------------------------------------------
God.c 中的读取函数（Ant/god.c:115-118）：

    QTI_LL() -> P0_7  机器人最左
    QTI_L () -> P0_6  机器人左中
    QTI_R () -> P0_5  机器人右中
    QTI_RR() -> P0_4  机器人最右

极性：1 = 探头看到黑线（依据：标定模式 2 中"手指挡住-> 变 0"，
手指反光强被当成白底，所以黑线对应 1）。

状态编号定义为（bit3..bit0 = P0.7..P0.4）：

    state = (LL << 3) | (L << 2) | (R << 1) | RR

这个编号有个很好用的性质：**state == (P0 >> 4) & 0x0F**，
单片机端不需要任何查表，直接取 P0 的高四位就能得到状态号。
"""

import math
import random

# ============================================================
# 状态空间
# ============================================================

NUM_STATES = 16   # 2^4，四路数字传感器的全部组合
NUM_ACTIONS = 6

#: 每位对应的探头名，bit3 在最左
BIT_NAMES = ("RR", "R", "L", "LL")   # bit0, bit1, bit2, bit3

#: 每个状态的文字说明，用于报告、导出文件注释和人工核对。
#: 措辞统一从【线相对车的位置】出发（不是"车相对线"），因为这个方向直接决定该往哪转：
#:   线在车的左边 -> 左转；线在车的右边 -> 右转。
STATE_NAMES = {
    0b0000: "全部为白 —— 丢线",
    0b0001: "仅最右探头压线 —— 线在车右侧很远（车偏左严重），应右转",
    0b0010: "仅右中探头压线 —— 线在车右侧一点，应右转",
    0b0011: "右中+最右压线 —— 线在车右侧较多，应右转",
    0b0100: "仅左中探头压线 —— 线在车左侧一点，应左转",
    0b0101: "左中+最右压线（不常见，跨了两条线？）",
    0b0110: "左中+右中压线 —— 车在线中央（正常巡线态）",
    0b0111: "左中+右中+最右（不常见）",
    0b1000: "仅最左探头压线 —— 线在车左侧很远（车偏右严重），应左转",
    0b1001: "最左+最右压线（不常见）",
    0b1010: "最左+右中压线（不常见）",
    0b1011: "最左+右中+最右（不常见）",
    0b1100: "最左+左中压线 —— 线在车左侧较多，应左转",
    0b1101: "最左+左中+最右（不常见）",
    0b1110: "最左+左中+右中（不常见）",
    0b1111: "四路全压线 —— 交叉线 / 横线 / 起跑标记",
}


def encode_state(sensors):
    """把四路探头读数打包成状态号。

    :param sensors: 长度为 4 的序列，顺序为 (LL, L, R, RR)，取值 0/1
    :return: 0~15 的状态编号
    """
    ll, l, r, rr = (1 if s else 0 for s in sensors)
    return (ll << 3) | (l << 2) | (r << 1) | rr


def decode_state(state):
    """状态号 -> (LL, L, R, RR) 四元组，encode_state 的逆运算。"""
    if not 0 <= state < NUM_STATES:
        raise ValueError(f"状态号越界: {state}")
    return (
        (state >> 3) & 1,
        (state >> 2) & 1,
        (state >> 1) & 1,
        state & 1,
    )


def line_position_error(state):
    """复刻 God.c 主循环里的 err 决策量（Ant/god.c:365-369）。

        err = -3*LL - 1*L + 1*R + 3*RR

    err < 0：线在车的【左】侧 -> 需要左转
    err > 0：线在车的【右】侧 -> 需要右转
    err == 0：居中（或左右对称的特殊组合）

    这与 God.c 的决策表完全一致：
        if (err > 0) RightTurn/TrimRight;  else if (err < 0) LeftTurn/TrimLeft;

    注意与 God.c 的变量名对应关系：那里的 L=QTI_LL、M1=QTI_L、M2=QTI_R、R=QTI_RR。
    """
    ll, l, r, rr = decode_state(state)
    return -3 * ll - 1 * l + 1 * r + 3 * rr


def state_is_on_line(state):
    """state != 0 即至少有一个探头压线。"""
    return state != 0


# ============================================================
# 动作空间
# ============================================================

#: 动作编号 -> 名称。顺序固定，导出的 C 数组下标必须与这里一致。
ACTION_NAMES = {
    0: "GO",
    1: "LEFT",
    2: "RIGHT",
    3: "TRIM_LEFT",
    4: "TRIM_RIGHT",
    5: "BACK",
}

#: 动作编号 -> God.c 里的同名函数
ACTION_FIRMWARE_FUNC = {
    0: "Go(1)",
    1: "LeftTurn(STEP_TURN)",
    2: "RightTurn(STEP_TURN)",
    3: "TrimLeft(STEP_TRIM)",
    4: "TrimRight(STEP_TRIM)",
    5: "Backward(STEP_BACK)",
}

A_GO = 0
A_LEFT = 1
A_RIGHT = 2
A_TRIM_LEFT = 3
A_TRIM_RIGHT = 4
A_BACK = 5

#: 动作 -> (左轮速度, 右轮速度)，单位是"偏离舵机中值 SERVO_C 的微秒数"。
#:
#: 这些数字**直接抄自 Ant/god.c 的 #define**，不是重新设计的：
#:     SPD_GO=200, SPD_TURN=450, SPD_BACK=400, SPD_TRIM=150
#:     Go      -> Drive( +200,   +200)
#:     LeftTurn-> Drive( -450,   +450)
#:     RightTurn->Drive( +450,   -450)
#:     TrimLeft-> Drive(  200-150, 200+150) = (  50, +350)
#:     TrimRight->Drive( 200+150, 200-150) = (+350,   50)
#:     Backward-> Drive( -400,   -400)
#:
#: 也就是说：**强化学习用的动作集合，和现有程序已经实现的动作完全一致**，
#: 部署时不需要新增任何电机控制代码。
ACTION_WHEEL_SPEED = {
    A_GO:         (200, 200),
    A_LEFT:       (-450, 450),
    A_RIGHT:      (450, -450),
    A_TRIM_LEFT:  (50, 350),
    A_TRIM_RIGHT: (350, 50),
    A_BACK:       (-400, -400),
}

#: 需要转向的动作，用于奖励里的"频繁转向惩罚"
TURNING_ACTIONS = (A_LEFT, A_RIGHT)
TRIM_ACTIONS = (A_TRIM_LEFT, A_TRIM_RIGHT)


# ============================================================
# 模拟环境
# ============================================================

class LineFollowerEnv:
    """极简巡线环境。

    坐标系约定（**标准机器人约定**，务必先看懂这一段，符号搞反会导致
    导出的策略在实车上左右反着转）：

        lat   车体中心相对线路中线的横向偏移，**正 = 车偏在线路的【左】侧**
        theta 车头方向相对"线路当地方向"的夹角，**正 = 车头指向左侧**（逆时针为正）

    物理模型（连续差速小车，把车头前方的探头也考虑进去）：

        v     = (v_L + v_R) / 2                     线速度
        omega = (v_R - v_L) / WHEEL_BASE            角速度（右轮快 -> 向左转 -> theta 变大）

        lat   += v * sin(theta) * dt                横向偏移
        s     += v * cos(theta) * dt                沿线路方向的位移
        theta += omega * dt - kappa * ds            航向角；kappa 是赛道曲率，
                                                    kappa>0 表示线路向左弯

    第 i 个探头（相对车体中线的安装位置 o_i，车头前方 LOOK_AHEAD 处）
    看到的世界横向位置：

        lat_i = lat + LOOK_AHEAD * theta + o_i

    若 |lat_i| < LINE_HALF_WIDTH 则认为该探头压在黑线上，读数 1。

    用这组参数可以复现 God.c 的决策结构（这是**回归测试**保护的对象）：
        lat=0     -> state 0b0110，err=0，直行
        lat=+1.0  -> state 0b0011，err=+3..+4 -> God.c 判右转（车偏左，要往右回来）
        lat=-1.0  -> state 0b1100，err=-3..-4 -> God.c 判左转
        |lat|>2.4 -> 全部为白，丢线
    """

    # ---- 几何参数（单位：cm，全部是【待实测标定】的估计值）----
    #: 四路探头相对车体中线的横向安装位置，**正数 = 车体【左】侧**（与 lat 同向）。
    #: 顺序固定为 (RR, R, L, LL)：最右的 RR 在 -1.5，最左的 LL 在 +1.5。
    #: ⚠️ 这个符号曾经写反过，导致学到的策略左右镜像 —— 见 tests 里的
    #:    test_recovery_direction_matches_god_c。
    SENSOR_OFFSETS = (-1.5, -0.5, 0.5, 1.5)   # 顺序 (RR, R, L, LL)
    #: 探头安装位置在车头前方多远（越大越"提前看"）
    LOOK_AHEAD = 1.0
    #: 黑线半宽。1.8cm 宽的胶带 -> 0.9
    LINE_HALF_WIDTH = 0.9
    #: 左右轮距
    WHEEL_BASE = 8.0
    #: 每"微秒脉宽偏差"对应的车轮线速度（cm/s per us）—— 纯假设值，未标定
    US_TO_CMPS = 0.075

    # ---- 赛道 ----
    #: 赛道由若干 (弧长 cm, 曲率 1/cm) 段首尾相接组成，跑满总长算一圈。
    #: 曲率 > 0 = 左弯，< 0 = 右弯，0 = 直道。半径 40cm 的弯 -> 曲率 1/40。
    #:
    #: ⚠️ 为什么要加弯道？**只有直道的话，"永远直行"就是最优策略**，
    #:    Q 表根本不需要学任何东西 —— 那样的环境验证不了算法流程。
    #:    加上弯道后，直行必然冲出去，策略必须学会"偏离了就修回来"。
    #:
    #: ⚠️ 赛道尺寸是**假设值**，不是量出来的。真实赛道请按 05_*.md 的表格记录。
    TRACK_SEGMENTS = (
        (35.0, 0.0),          # 直道
        (35.0, +1.0 / 40.0),  # 左弯，半径 40cm
        (35.0, 0.0),          # 直道
        (35.0, -1.0 / 40.0),  # 右弯，半径 40cm
        (35.0, 0.0),          # 直道
    )
    TRACK_LENGTH = sum(seg[0] for seg in TRACK_SEGMENTS)   # 一圈 175cm

    # ---- 回合参数 ----
    #: 单步仿真时长。真实一帧约 23ms，这里放大到 50ms 以加快训练
    #: —— 这是为了跑得快而做的简化，会放大每步的转向角度，属于已知偏差。
    DT = 0.05
    LOST_PATIENCE = 3         # 连续几步全白就判失败
    MAX_STEPS = 600           # 单回合硬上限，防止死循环

    # ---- 起点随机化（避免策略只适应单一初始条件）----
    INIT_LAT_RANGE = (-1.2, 1.2)
    INIT_THETA_RANGE = (-0.20, 0.20)

    # ---- 奖励里的动作代价（可调，见 03_*.md 第 4.3 节）----
    #: 这些值**故意做成可调参数**，因为它们是本项目中最需要调的一项。
    #: 【实测】默认值 (−0.05) 下，学到的策略过度使用"硬转"，
    #: 每回合转向 44.5 次，而基线规则只有 1.36 次 —— 典型的"画龙"。
    TURN_PENALTY = -0.05
    TRIM_PENALTY = -0.01
    BACK_PENALTY = -0.10

    def __init__(self, seed=None):
        self.rng = random.Random(seed)
        self.s = 0.0
        self.lat = 0.0
        self.theta = 0.0
        self.steps = 0
        self.lost_count = 0
        self.lost_streak = 0
        self.max_lost_streak = 0
        self.turn_count = 0
        self.done = False
        self.success = False
        self.state = 0

    # --------------------------------------------------------

    def reset(self):
        """开始新回合。起点随机化：横向偏移和初始航向都在小范围内随机。"""
        self.s = 0.0
        self.lat = self.rng.uniform(*self.INIT_LAT_RANGE)
        self.theta = self.rng.uniform(*self.INIT_THETA_RANGE)
        self.steps = 0
        self.lost_count = 0
        self.lost_streak = 0
        self.max_lost_streak = 0
        self.turn_count = 0
        self.done = False
        self.success = False
        self.state = self._read_sensors()
        return self.state

    def _curvature(self, s):
        """赛道在弧长 s 处的曲率（1/cm）。赛道循环，所以对总长取模。"""
        pos = s % self.TRACK_LENGTH
        acc = 0.0
        for length, kappa in self.TRACK_SEGMENTS:
            if pos < acc + length:
                return kappa
            acc += length
        return 0.0

    def _read_sensors(self):
        """按当前位姿计算四路探头读数，返回状态号。"""
        # (RR, R, L, LL) 的安装位置
        lats = [
            self.lat + self.LOOK_AHEAD * self.theta + off
            for off in self.SENSOR_OFFSETS
        ]
        rr, r, l, ll = (1 if abs(v) < self.LINE_HALF_WIDTH else 0 for v in lats)
        return encode_state((ll, l, r, rr))

    def step(self, action):
        """执行一个动作，返回 (next_state, reward, done, info)。"""
        if self.done:
            raise RuntimeError("回合已结束，请先 reset()")

        v_left_us, v_right_us = ACTION_WHEEL_SPEED[action]
        v_left = v_left_us * self.US_TO_CMPS
        v_right = v_right_us * self.US_TO_CMPS

        v = 0.5 * (v_left + v_right)
        omega = (v_right - v_left) / self.WHEEL_BASE   # 右轮快 -> 向左转

        # 弯道上"线路自己的朝向"在转，所以车的【相对航向】还要减去这一项。
        # theta 是【相对线路的航向角】，直道上 kappa=0 退化成普通差速模型。
        kappa = self._curvature(self.s)
        ds = v * math.cos(self.theta) * self.DT

        self.s += ds
        self.lat += v * math.sin(self.theta) * self.DT
        self.theta += omega * self.DT - kappa * ds

        # 车体偏出模拟范围太远时夹住，避免数值发散
        self.lat = max(-20.0, min(20.0, self.lat))
        self.theta = max(-math.pi / 2, min(math.pi / 2, self.theta))

        prev_state = self.state
        self.state = self._read_sensors()
        next_state = self.state

        self.steps += 1
        if self.state == 0:
            self.lost_count += 1
            self.lost_streak += 1
        else:
            self.lost_streak = 0
        self.max_lost_streak = max(self.max_lost_streak, self.lost_streak)
        if action in TURNING_ACTIONS:
            self.turn_count += 1

        reached_goal = self.s >= self.TRACK_LENGTH
        failed = self.lost_streak >= self.LOST_PATIENCE
        timeout = self.steps >= self.MAX_STEPS

        self.done = reached_goal or failed or timeout
        self.success = reached_goal

        reward = self._reward(prev_state, next_state, action,
                              reached_goal, failed, timeout)

        info = {
            "s": self.s,
            "lat": self.lat,
            "theta": self.theta,
            "reached_goal": reached_goal,
            "failed": failed,
            "timeout": timeout,
        }
        return next_state, reward, self.done, info

    def _reward(self, prev_state, next_state, action, reached_goal, failed, timeout):
        """奖励函数。

        设计原则（详见 03_Q-learning算法与51单片机部署方案.md 第三节）：
          1. 主信号是"保持在线路上"，量级最大；
          2. 转向只在"必要"时才有价值，所以给小惩罚抑制抖动（画龙）；
          3. 丢线惩罚必须显著大于单步正奖励，否则策略宁愿乱跑；
          4. 各项目标量级要接近，避免某一项压倒其它项。

        注意：下面这组数字是【一组可用的初始值，不是唯一正确解】。
        不同赛道宽度、车速、传感器间距都要重新调。
        """
        r = 0.0

        # --- 终端奖励 ---
        # 终点奖励刻意给得比"单步正奖励之和"小一个量级：
        # 一圈约 230 步，每步最多 +1，所以累计正奖励约 230，
        # 终点 +100 是"额外"的一大块，但不至于让策略为了冲终点而放弃巡线。
        # ⚠️ 这是个权衡：如果希望策略更偏好"快跑完"，就调大这一项；
        #    如果发现策略在终点前犹豫，就调小。见 03_*.md 第 3 节。
        if reached_goal:
            return 100.0
        if failed:
            # 丢线判失败：一次性重罚。必须明显大于单步正奖励，
            # 否则策略会觉得"反正丢线也就扣一点"。
            return -10.0

        # --- 在线路上的正奖励 ---
        if state_is_on_line(next_state):
            err = line_position_error(next_state)
            if err == 0:
                r += 1.0          # 正中央
            else:
                r += 0.3          # 还在线上但偏了

        # --- 恢复奖励：上一步丢线，这一步重新找到线 ---
        if prev_state == 0 and next_state != 0:
            r += 1.0

        # --- 动作代价：抑制不必要的频繁转向 ---
        if action in TURNING_ACTIONS:
            r += self.TURN_PENALTY
        elif action in TRIM_ACTIONS:
            r += self.TRIM_PENALTY
        elif action == A_BACK:
            r += self.BACK_PENALTY

        if timeout:
            r -= 1.0

        return r

    # --------------------------------------------------------

    def render(self):
        """一行 ASCII 可视化，方便调试和演示。"""
        cells = []
        for state_bit, label in ((3, "LL"), (2, "L"), (1, "R"), (0, "RR")):
            on = (self.state >> state_bit) & 1
            cells.append(f"{label}={'#' if on else '.'}")
        return (f"step={self.steps:3d} s={self.s:6.1f} lat={self.lat:+5.2f} "
                f"th={self.theta:+.3f} state={self.state:2d} "
                f"({' '.join(cells)})")
