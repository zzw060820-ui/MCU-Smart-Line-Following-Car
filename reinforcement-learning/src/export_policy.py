"""
策略导出：把训练好的 Q 表转成 51 单片机可用的 C 代码
====================================================

生成三个文件（全部放在 firmware_policy/ 下，不碰原有工程）：

1. policy_table.h
   方案 B —— 离线策略表。16 个字节，单片机只做"查表 -> 执行动作"。

2. q_table.h
   方案 A —— 完整 Q 表（量化成 unsigned char）。
   96 字节，单片机自己在片上取 argmax 选动作。

3. README.md
   状态编号 / 动作编号 / 动作对应函数的对照说明，方便人工核对。

量化说明：
    Keil C51 支持 float，但 8051 没有硬件浮点，浮点运算会拖慢控制周期且增加代码体积。
    所以完整 Q 表导出时按 (q - min) / (max - min) * 255 线性量化成字节。
    导出时会自动校验"量化后每行 argmax 是否与量化前一致"，
    不一致的状态会打印警告 —— 这类状态在实体车上可能选错动作。

用法：
    python src/export_policy.py
    python src/export_policy.py --q-table results/q_table.json --out firmware_policy
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from environment import (NUM_STATES, NUM_ACTIONS, ACTION_NAMES,          # noqa: E402
                         ACTION_FIRMWARE_FUNC, ACTION_WHEEL_SPEED,
                         STATE_NAMES, decode_state, line_position_error,
                         A_GO, A_LEFT, A_RIGHT,
                         A_TRIM_LEFT, A_TRIM_RIGHT, A_BACK)
from q_learning import QLearningAgent                                    # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(ROOT, "results")
FIRMWARE_DIR = os.path.join(ROOT, "firmware_policy")

GENERATED_BANNER = (
    "/* 本文件由 rl_line_follower/src/export_policy.py 自动生成，请勿手工修改。\n"
    " * 重新生成: python src/export_policy.py\n"
    " * 生成时间等信息请以 results/train_summary.json 为准。 */\n"
)


def _state_comment(s, fallback_states=()):
    bits = "".join(str(b) for b in decode_state(s))
    mark = "  [规则兜底]" if s in fallback_states else ""
    return (f"/* s={s:2d}  探头LL L R RR = {bits}  err={line_position_error(s):+d}  "
            f"{STATE_NAMES[s]}{mark} */")


def god_c_rule_action(state):
    """God.c 主循环的规则决策表，逐行照抄（Ant/god.c:345-386）。

    用途：给"简化模拟环境里从未出现过"的状态兜底。

    为什么需要兜底：模拟环境只有一条特定赛道，有些传感器组合（例如 1111 四路
    全压线，在实车上出现在交叉线/横线/探头抖动时）在仿真里**一次都不会出现**，
    对应的 Q 值全是 0，argmax 就退化成"永远选动作 0"。这类状态在实体车上是
    真实存在的 —— 直接部署会在这些状态下乱转。兜底用规则把这些状态填上。
    """
    if state == 0:                      # 丢线
        return A_BACK
    err = line_position_error(state)
    if err >= 3:
        return A_RIGHT
    if err >= 1:
        return A_TRIM_RIGHT
    if err <= -3:
        return A_LEFT
    if err <= -1:
        return A_TRIM_LEFT
    return A_GO


def build_policy(agent, use_fallback=True):
    """生成最终部署用策略：学习结果 + （可选）未访问状态的规则兜底。

    :return: (policy 列表, fallback_states 集合, visited 集合)
    """
    learned = agent.greedy_policy()
    visited = {s for s in range(NUM_STATES) if any(v != 0.0 for v in agent.q[s])}

    policy = list(learned)
    fallback_states = set()
    if use_fallback:
        for s in range(NUM_STATES):
            if s not in visited:
                policy[s] = god_c_rule_action(s)
                fallback_states.add(s)
    return policy, fallback_states, visited


def build_policy_header(policy, agent, fallback_states=()):
    """方案 B：离线策略表头文件。"""
    n_fb = len(fallback_states)
    lines = [
        GENERATED_BANNER,
        "#ifndef POLICY_TABLE_H",
        "#define POLICY_TABLE_H",
        "",
        "/* ============================================================",
        " * 方案 B：离线策略表  policy[state] = action",
        " *",
        " * 单片机端只需要：",
        " *   1. 读四路探头 -> state = (P0 >> 4) & 0x0F",
        " *   2. action = policy_action[state]",
        " *   3. 按 action 调用现有函数（见 policy_action_to_func 注释）",
        " *",
        " * 占用的 code 空间：16 字节",
        " * ============================================================ */",
        "",
        "#define POLICY_NUM_STATES   16",
        "#define POLICY_NUM_ACTIONS   6",
        "",
        "/* 动作编号定义 —— 与 src/environment.py 的 ACTION_NAMES 一一对应 */",
        "#define ACT_GO           0   /* Go(1)                         直行 */",
        "#define ACT_LEFT         1   /* LeftTurn(STEP_TURN)           左转 */",
        "#define ACT_RIGHT        2   /* RightTurn(STEP_TURN)          右转 */",
        "#define ACT_TRIM_LEFT    3   /* TrimLeft(STEP_TRIM)           左微调 */",
        "#define ACT_TRIM_RIGHT   4   /* TrimRight(STEP_TRIM)          右微调 */",
        "#define ACT_BACK         5   /* Backward(STEP_BACK)           后退 */",
        "",
    ]
    if n_fb:
        lines += [
            f"/* 说明：有 {n_fb} 个状态在简化模拟环境里【从未出现过】，Q 值全为 0。",
            " *      这些状态的动作来自 God.c 的规则决策表（标注 [规则兜底]），",
            " *      不是学出来的。实体车上首次运行前，这些行必须人工复核。 */",
            "",
        ]
    lines += [
        "/* state -> 动作。下标就是 (P0 >> 4) & 0x0F */",
        "code unsigned char policy_action[POLICY_NUM_STATES] = {",
    ]
    for s in range(NUM_STATES):
        a = policy[s]
        lines.append(f"    ACT_{ACTION_NAMES[a]:<10s},   "
                     f"{_state_comment(s, fallback_states)}")
    lines += [
        "};",
        "",
        "/* 对应的 God.c 函数调用（仅供人工核对，不是可编译代码）:",
    ]
    for a in range(NUM_ACTIONS):
        lines.append(f"     ACT_{ACTION_NAMES[a]:<10s} -> {ACTION_FIRMWARE_FUNC[a]}")
    lines += ["*/", "", "#endif /* POLICY_TABLE_H */", ""]
    return "\n".join(lines)


def build_qtables_header(agent):
    """方案 A：完整 Q 表（量化）。返回 (文本, 量化后 argmax 与原始不一致的状态列表)。"""
    flat = [v for row in agent.q for v in row]
    qmin, qmax = min(flat), max(flat)
    span = (qmax - qmin) or 1.0

    def quant(v):
        return int(round((v - qmin) / span * 255.0))

    mismatch = []
    q_rows = []
    for s in range(NUM_STATES):
        row = agent.q[s]
        qrow = [quant(v) for v in row]
        q_rows.append(qrow)
        if qrow.index(max(qrow)) != row.index(max(row)):
            mismatch.append(s)

    lines = [
        GENERATED_BANNER,
        "#ifndef Q_TABLE_H",
        "#define Q_TABLE_H",
        "",
        "/* ============================================================",
        " * 方案 A：完整 Q 表（线性量化到 0~255）",
        " *",
        " * 单片机端：读状态 -> 在该行里找最大值的下标 -> 执行该动作。",
        " * 比方案 B 多一个 16x6 的遍历（96 次比较），换来的是",
        " * 可以在实体上改用不同动作选择规则（例如加温度/迟滞）。",
        " *",
        " * 占用的 code 空间：96 字节",
        " * ============================================================ */",
        "",
        "#define Q_NUM_STATES   16",
        "#define Q_NUM_ACTIONS   6",
        "",
        f"/* 量化参数：原 Q 值范围 [{qmin:.4f}, {qmax:.4f}]，",
        f" * q_int = (q - Q_MIN) / Q_SPAN * 255",
        f" * 注意 Q_MIN/Q_SPAN 是浮点，仅用于人工换算；",
        f" * 单片机端做 argmax 不需要用到它们。 */",
        f"#define Q_MIN      ({qmin:.4f})",
        f"#define Q_SPAN     ({span:.4f})",
        "",
        "code unsigned char q_table[Q_NUM_STATES][Q_NUM_ACTIONS] = {",
    ]
    for s in range(NUM_STATES):
        row = ", ".join(f"{v:3d}" for v in q_rows[s])
        lines.append(f"    {{ {row} }},   {_state_comment(s)}")
    lines += ["};", ""]

    if mismatch:
        lines += [
            "/* ⚠️ 警告：以下状态在【量化之后】的最优动作与量化前不一致，",
            " *    说明这些状态的 Q 值差异太小，量化把它抹平了。",
            " *    实体车用方案 A 时这些状态可能选错动作，建议优先用方案 B。",
            f" *    不一致的状态: {mismatch}",
            " */",
            "",
        ]
    else:
        lines += [
            "/* ✅ 已校验：全部 16 个状态量化后的 argmax 与量化前完全一致。 */",
            "",
        ]

    lines += ["#endif /* Q_TABLE_H */", ""]
    return "\n".join(lines), mismatch


def build_readme(agent, policy, mismatch, summary_note,
                 fallback_states=(), visited=()):
    lines = [
        "# firmware_policy — 导出的单片机策略文件",
        "",
        "> 本目录由 `src/export_policy.py` 自动生成。",
        "> **这里的所有文件都是新建的，不影响原有 `Ant/` 工程。**",
        "",
        "## 文件清单",
        "",
        "| 文件 | 方案 | 体积 | 说明 |",
        "|---|---|---|---|",
        "| `policy_table.h` | B 离线策略表 | 16 字节 | **推荐**，只查表 |",
        "| `q_table.h` | A 完整 Q 表 | 96 字节 | 片上取 argmax |",
        "| `README.md` | — | — | 本文件 |",
        "",
        "## 状态编号",
        "",
        "状态 = `(P0 >> 4) & 0x0F`，即 P0 的高四位，bit3..bit0 = P0.7..P0.4 = LL L R RR。",
        "**1 = 探头看到黑线。**",
        "",
        "| state | 探头 LL L R RR | err | 来源 | 说明 | 策略动作 |",
        "|---|---|---|---|---|---|",
    ]
    for s in range(NUM_STATES):
        bits = " ".join(str(b) for b in decode_state(s))
        src = "规则兜底" if s in fallback_states else "Q-learning"
        lines.append(f"| {s} | {bits} | {line_position_error(s):+d} | {src} | "
                     f"{STATE_NAMES[s]} | `{ACTION_NAMES[policy[s]]}` |")

    if fallback_states:
        lines += [
            "",
            "### ⚠️ 规则兜底说明",
            "",
            f"下面 {len(fallback_states)} 个状态在简化模拟环境里**从未出现过**，"
            "Q 值全为 0，学习结果对它们没有信息：",
            "",
            f"`{sorted(fallback_states)}`",
            "",
            "这些行用的是 God.c 的规则决策表，**不是学出来的**。",
            "在实体车上这些状态是真实存在的（四路全压线=交叉线/横线、探头抖动等），",
            "所以必须有人复核这几行做得对不对。若发现某一行不对，直接手改 `policy_table.h`。",
        ]
    lines += [
        "",
        "## 动作编号 → God.c 函数",
        "",
        "| 动作号 | 名称 | God.c 调用 | 左右轮速度(µs 偏差) |",
        "|---|---|---|---|",
    ]
    for a in range(NUM_ACTIONS):
        v = ACTION_WHEEL_SPEED[a]
        lines.append(f"| {a} | `{ACTION_NAMES[a]}` | `{ACTION_FIRMWARE_FUNC[a]}` | "
                     f"({v[0]:+d}, {v[1]:+d}) |")
    lines += [
        "",
        "## 使用步骤（不修改原有程序）",
        "",
        "1. **先别烧录**。先在 `src/evaluate.py` 里确认策略表没有平局、状态含义正确。",
        "2. 把 `policy_table.h` 复制到 `Ant/` 目录下（`god.c` 的 `#include` 会搜源文件所在目录）。",
        "3. **复制一份 `god.c` 出来改**（例如 `Ant/god_rl.c`），不要动原文件；",
        "   或者新建一个独立的 Keil 工程引用它，保留原 `KGOD.uvproj` 不动。",
        "4. 把主循环里的决策段替换成查表，大致是：",
        "",
        "```c",
        "#include \"policy_table.h\"",
        "",
        "state = (P0 >> 4) & 0x0F;          /* 四路探头 -> 状态号 */",
        "switch (policy_action[state]) {",
        "    case ACT_GO:         Go(1);                 break;",
        "    case ACT_LEFT:       LeftTurn(STEP_TURN);   break;",
        "    case ACT_RIGHT:      RightTurn(STEP_TURN);  break;",
        "    case ACT_TRIM_LEFT:  TrimLeft(STEP_TRIM);   break;",
        "    case ACT_TRIM_RIGHT: TrimRight(STEP_TRIM);  break;",
        "    case ACT_BACK:       Backward(STEP_BACK);   break;",
        "}",
        "```",
        "",
        "5. **目标芯片不变**：仍然是 AT89S52，仍然用 ProgISP + USBasp 烧录。",
        "",
        "## ⚠️ 当前策略文件的状态",
        "",
    ]
    if mismatch:
        lines.append(f"- **量化校验未全部通过**：状态 {mismatch} 的最优动作在量化后改变。")
        lines.append("- 结论：**优先采用方案 B（policy_table.h）**，不要用方案 A。")
    else:
        lines.append("- 量化校验通过：16 个状态的最优动作在量化前后一致。")
        lines.append("- 方案 A / B 都可用；推荐方案 B（更小、更快、更简单）。")
    lines += [
        f"- {summary_note}",
        "- **本策略只在简化模拟环境中训练，尚未在实体小车上验证过。**",
        "",
    ]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="导出 Q 表为 51 单片机 C 代码")
    ap.add_argument("--q-table", default=os.path.join(RESULTS_DIR, "q_table.json"))
    ap.add_argument("--out", default=FIRMWARE_DIR)
    ap.add_argument("--summary", default=os.path.join(RESULTS_DIR, "train_summary.json"))
    ap.add_argument("--no-fallback", action="store_true",
                    help="不给'模拟环境从未访问过'的状态做规则兜底（不推荐）")
    args = ap.parse_args()

    if not os.path.exists(args.q_table):
        print(f"[错误] 找不到 Q 表: {args.q_table}")
        print("       请先运行: python src/train.py")
        sys.exit(1)

    agent = QLearningAgent().load(args.q_table)
    policy, fallback_states, visited = build_policy(
        agent, use_fallback=not args.no_fallback)

    note = "训练参数见 results/train_summary.json。"
    if os.path.exists(args.summary):
        with open(args.summary, "r", encoding="utf-8") as f:
            s = json.load(f)
        note = (f"训练 {s.get('episodes')} 回合，模拟环境末段成功率 "
                f"{s.get('final_window_success_rate', 0) * 100:.1f}%。")

    os.makedirs(args.out, exist_ok=True)

    p1 = os.path.join(args.out, "policy_table.h")
    with open(p1, "w", encoding="utf-8") as f:
        f.write(build_policy_header(policy, agent, fallback_states))

    q_text, mismatch = build_qtables_header(agent)
    p2 = os.path.join(args.out, "q_table.h")
    with open(p2, "w", encoding="utf-8") as f:
        f.write(q_text)

    p3 = os.path.join(args.out, "README.md")
    with open(p3, "w", encoding="utf-8") as f:
        f.write(build_readme(agent, policy, mismatch, note,
                             fallback_states, visited))

    print("=" * 62)
    print(" 策略导出完成")
    print("=" * 62)
    print(f" 方案 B 离线策略表 -> {p1}   (16 字节)")
    print(f" 方案 A 完整 Q 表   -> {p2}   (96 字节)")
    print(f" 说明文档          -> {p3}")
    print()
    print(f" 模拟环境访问过的状态: {sorted(visited)}")
    if fallback_states:
        print(f" 未访问、改用规则兜底的状态: {sorted(fallback_states)}")
    print()
    print(" 策略表内容:")
    for s in range(NUM_STATES):
        bits = "".join(str(b) for b in decode_state(s))
        tag = "  <- 规则兜底" if s in fallback_states else ""
        print(f"   state {s:2d} ({bits}) -> {ACTION_NAMES[policy[s]]:<11s}{tag}")
    if mismatch:
        print()
        print(f" [警告] 量化后最优动作改变的状态: {mismatch}")
        print("        建议只使用方案 B。")
    else:
        print()
        print(" [通过] 量化校验通过，方案 A/B 均可用。")


if __name__ == "__main__":
    main()
