"""
演示：强化学习训练时，一个回合到底发生了什么
=============================================

这个脚本不是训练用的，是**给你看明白的**。

运行：
    cd rl_line_follower
    python tools/demo_one_episode.py

它会依次打印：
  第 1 段  虚拟小车和虚拟探头的参数
  第 2 段  从零开始跑 3 个回合，看 Q 表怎么被"改"
  第 3 段  一个回合的逐步明细（状态/动作/奖励）
  第 4 段  训练前后，那张 16 行的策略表变成什么样
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "src"))

from environment import (LineFollowerEnv, ACTION_NAMES, ACTION_WHEEL_SPEED,   # noqa: E402
                         STATE_NAMES, decode_state, line_position_error,
                         NUM_STATES, NUM_ACTIONS)
from q_learning import QLearningAgent                                          # noqa: E402


def 线(t=""):
    print("-" * 72 if not t else f"\n{'=' * 72}\n {t}\n{'=' * 72}")


def 第1段_虚拟环境长什么样():
    线("第 1 段：模拟的是什么（虚拟小车 + 虚拟赛道）")
    print("""
这一整段全部实现在 src/environment.py 里，和你的真车没有任何关系。

  虚拟小车有 2 个状态量：
      lat    车中心离线路中线偏了多少（cm）。正 = 车偏在线【左】边
      theta  车头方向歪了多少（弧度）。  正 = 车头指向左

  虚拟的四个探头装在车头，横向位置（cm）：
      RR=-1.5   R=-0.5   L=+0.5   LL=+1.5      （正 = 车体左侧）
  探头装在车头前方 1.0cm 处。

  一个探头"看到黑线"的条件：它离线路中线的距离 < 0.9cm

  虚拟赛道（一圈 175cm）：
""")
    for i, (length, kappa) in enumerate(LineFollowerEnv.TRACK_SEGMENTS):
        if kappa == 0:
            desc = "直道"
        else:
            desc = f"{'左' if kappa > 0 else '右'}弯，半径 {1/abs(kappa):.0f}cm"
        print(f"      第{i+1}段  {length:5.1f}cm   {desc}")

    print(f"""
  每走一步（一次动作）花 DT = {LineFollowerEnv.DT}s
  真车一帧是约 0.023s，这里放大到 0.05s 是为了训练快一点（已知偏差）

  一个回合什么时候结束：
      ✅ 跑满 175cm（一圈）        → 成功
      ❌ 连续 3 步四路全白（丢线）  → 失败
      ❌ 走满 {LineFollowerEnv.MAX_STEPS} 步              → 超时
""")


def 第2段_Q表怎么被改():
    线("第 2 段：强化学习到底'改'了什么？—— 改这张表里的数字")

    env = LineFollowerEnv(seed=0)
    agent = QLearningAgent(alpha=0.1, gamma=0.95, epsilon=1.0, seed=0)

    print("""
强化学习改的不是你的代码，是下面这张 16 行 × 6 列的表。
这张表一开始全是 0（= 什么都不知道）。

  行 = 16 种探头组合（状态）    列 = 6 个动作（GO/LEFT/RIGHT/TRIM_LEFT/TRIM_RIGHT/BACK）

跑第一个回合之前，Q[6]（"车在正中间"这一行）长这样：
""")
    print(f"    Q[6] = {agent.q[6]}")

    print("""
现在让小车走一步：假设它在状态 6（正中间），选了 GO，这一步得了 +1.0 分，
下一状态还是 6。看 Q 表怎么变 ——
""")
    更新前 = list(agent.q[6])
    agent.update(state=6, action=0, reward=1.0, next_state=6, done=False)
    更新后 = list(agent.q[6])
    print(f"    更新前 Q[6] = {更新前}")
    print(f"    更新后 Q[6] = {更新后}")
    print(f"""
     ↑
     只有这一个格子变了：Q[6][GO] 从 0.0 变成了 0.1

     公式（src/q_learning.py 的 update() 方法）：
         Q[s][a] ← Q[s][a] + α × [ r + γ × max Q[s'] − Q[s][a] ]
         0.1     = 0.0      + 0.1 × [ 1.0 + 0.95 × 0.0   − 0.0   ]

     α = 0.1 是学习率，γ = 0.95 是折扣因子。
     这一格只涨了 0.1，因为 α 小 —— 故意慢一点，免得被一次偶然的经历带偏。

     **每走一步，就这样改一个小格子。** 跑 6000 个回合（约 140 万步），
     整张表就被改得越来越准 —— 这就是"学习"的全部。
""")

    print("  再跑 3 个回合看看这张表的整体变化：\n")
    print(f"    {'回合':<6}{'总步数':<8}{'总奖励':<10}{'成功率':<8}{'表里非零格子数'}")
    print("    " + "-" * 56)
    非零 = lambda: sum(1 for row in agent.q for v in row if v != 0)
    for ep in range(1, 4):
        state = env.reset()
        总奖励 = 0.0
        while True:
            a = agent.select_action(state)
            next_state, r, done, _ = env.step(a)
            agent.update(state, a, r, next_state, done)
            state = next_state
            总奖励 += r
            if done:
                break
        agent.decay_epsilon()
        print(f"    {ep:<6}{env.steps:<8}{总奖励:<10.1f}"
              f"{'是' if env.success else '否':<8}{非零()}")
    print(f"\n    注意最后一列：非零格子数在增加 —— 表在被一点点填上。")


def 第3段_一个回合的逐步明细():
    线("第 3 段：一个回合的逐步明细（这就是训练时真实发生的事）")

    env = LineFollowerEnv(seed=7)
    agent = QLearningAgent(alpha=0.1, gamma=0.95, epsilon=1.0, seed=7)

    print("""
下面把前 12 步一行行打出来。每一行是：

   第几步 | 探头读出什么 | 现在是什么状态 | 选了哪个动作 | 得了多少分 | 车偏了多少

（开头 epsilon = 1.0，全是随机乱动，所以看起来毫无章法 —— 这正是"探索"）
""")
    print(f"    {'步':<5}{'探头LL L R RR':<16}{'状态':<28}{'动作':<13}{'得分':<8}{'车偏(cm)'}")
    print("    " + "-" * 90)

    state = env.reset()
    for i in range(12):
        a = agent.select_action(state)
        next_state, r, done, _ = env.step(a)
        bits = " ".join(str(b) for b in decode_state(next_state))
        名 = ACTION_NAMES[a]
        print(f"    {i+1:<5}{bits:<16}state={next_state:<3}{STATE_NAMES[next_state][:20]:<27}"
              f"{名:<13}{r:<+8.2f}{env.lat:+6.2f}")
        agent.update(state, a, r, next_state, done)
        state = next_state
        if done:
            print("    （回合提前结束了）")
            break

    print(f"""
    注意第 6 步那一行：探头 = 0 1 1 0（左中+右中压线）= 状态 6 = "车在正中间"。
    这时如果选 GO，就 +1.0 分；选别的动作也会得分，但少一些。
    **跑几万次之后，表里就会记住"状态 6 应该选 GO"。**
""")


def 第4段_训练前后策略表对比():
    线("第 4 段：训练前 vs 训练后（最终成果长什么样）")

    env = LineFollowerEnv(seed=0)
    agent = QLearningAgent(alpha=0.1, gamma=0.95, epsilon=1.0, seed=0)

    print("  训练前（表全是 0，只能瞎选）：")
    print("    " + "  ".join(f"{i}:{ACTION_NAMES[a]}" for i, a in
                             enumerate(agent.greedy_policy())))

    print("\n  训练 1500 回合中 ...（约 8 秒）")
    for ep in range(1500):
        state = env.reset()
        while True:
            a = agent.select_action(state)
            next_state, r, done, _ = env.step(a)
            agent.update(state, a, r, next_state, done)
            state = next_state
            if done:
                break
        agent.decay_epsilon()

    策略 = agent.greedy_policy()
    print("\n  训练后（这张表就是最终成果）：")
    print(f"    {'状态':<6}{'探头LL L R RR':<16}{'err':<6}{'学到的最优动作':<15}{'说明'}")
    print("    " + "-" * 78)
    for s in range(NUM_STATES):
        bits = " ".join(str(b) for b in decode_state(s))
        mark = ""
        # 对照 God.c 的规则表，看学出来的结果是否一致
        err = line_position_error(s)
        if s == 0:
            expected = "BACK"
        elif err >= 3:
            expected = "RIGHT"
        elif err >= 1:
            expected = "TRIM_RIGHT"
        elif err <= -3:
            expected = "LEFT"
        elif err <= -1:
            expected = "TRIM_LEFT"
        else:
            expected = "GO"
        学到 = ACTION_NAMES[策略[s]]
        mark = "  ✅ 与规则一致" if 学到 == expected else f"  （规则表给的是 {expected}）"
        print(f"    {s:<6}{bits:<16}{err:<+6d}{学到:<15}{STATE_NAMES[s][:16]}{mark}")

    print("""
  ⚠️ 注意：这里只练了 1500 回合（为了演示快点），所以表还没练好 ——
     比如状态 6（正中间）居然选了 TRIM_LEFT，状态 12 选了 BACK，明显不对。
     正式版练 6000 回合，最后会收敛成（见 firmware_policy/policy_table.h）：

         state 6（居中）→ GO        状态 12（偏左多）→ LEFT
         state 1/2/3（偏右）→ RIGHT   state 0（丢线）→ BACK

     **这正好说明了"训练回合数"这个参数的意义：练得不够，表就是错的。**

  这张 16 行的表，会被 export_policy.py 翻译成这样一段 C 代码：

      code unsigned char policy_action[16] = {
          ACT_BACK ,   /* s= 0  探头 0000  丢线 */
          ACT_RIGHT,   /* s= 1  探头 0001  ... */
          ...
      };

  然后把 Ant/god.c 里那段 if-else 决策逻辑换成 policy_action[state] 一次查表。
  **强化学习和你的 god.c 的关系，就到这里为止。**
""")


if __name__ == "__main__":
    第1段_虚拟环境长什么样()
    第2段_Q表怎么被改()
    第3段_一个回合的逐步明细()
    第4段_训练前后策略表对比()
    print("=" * 72)
    print(" 演示结束。")
    print("=" * 72)
