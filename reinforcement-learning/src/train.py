"""
Q-learning 训练主程序
=====================

用法：
    cd rl_line_follower
    python src/train.py --episodes 3000 --seed 0

输出：
    results/q_table.json          训练好的 Q 表（JSON，可人工查看）
    results/train_log.csv         每个回合的奖励 / 成功 / 丢线次数 / epsilon
    results/train_summary.json    本次训练的参数和最终统计

⚠️ 再次提醒：这里的"成功率"是【简化模拟环境里的成功率】，
   不能当作真实小车的性能。原因见 src/environment.py 开头的定位说明。
"""

import argparse
import csv
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from environment import LineFollowerEnv          # noqa: E402
from q_learning import QLearningAgent            # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(ROOT, "results")


def train(episodes=3000, seed=0, alpha=0.1, gamma=0.95,
          epsilon=1.0, epsilon_min=0.05, epsilon_decay=0.995,
          turn_penalty=None, log_every=200, verbose=True):
    """跑完整训练流程，返回 (agent, log_rows, summary)。"""
    env = LineFollowerEnv(seed=seed)
    if turn_penalty is not None:
        env.TURN_PENALTY = turn_penalty
    agent = QLearningAgent(alpha=alpha, gamma=gamma, epsilon=epsilon,
                           epsilon_min=epsilon_min,
                           epsilon_decay=epsilon_decay, seed=seed)

    log_rows = []
    best_success_rate = 0.0
    window = max(1, episodes // 20)      # 用于计算滑动成功率的窗口

    for ep in range(1, episodes + 1):
        state = env.reset()
        total_reward = 0.0

        while True:
            action = agent.select_action(state)
            next_state, reward, done, info = env.step(action)
            agent.update(state, action, reward, next_state, done)
            total_reward += reward
            state = next_state
            if done:
                break

        agent.decay_epsilon()

        log_rows.append({
            "episode": ep,
            "reward": round(total_reward, 3),
            "success": int(env.success),
            "lost_count": env.lost_count,
            "max_lost_streak": env.max_lost_streak,
            "turn_count": env.turn_count,
            "steps": env.steps,
            "epsilon": round(agent.epsilon, 5),
        })

        recent = log_rows[-window:]
        rate = sum(r["success"] for r in recent) / len(recent)
        best_success_rate = max(best_success_rate, rate)

        if verbose and (ep % log_every == 0 or ep == 1):
            avg_r = sum(r["reward"] for r in recent) / len(recent)
            print(f"ep {ep:5d} | 平均奖励 {avg_r:8.2f} | "
                  f"近 {len(recent)} 回合成功率 {rate * 100:5.1f}% | "
                  f"epsilon {agent.epsilon:.3f}")

    summary = {
        "episodes": episodes,
        "seed": seed,
        "alpha": alpha,
        "gamma": gamma,
        "epsilon_start": epsilon,
        "epsilon_min": epsilon_min,
        "epsilon_decay": epsilon_decay,
        "turn_penalty": env.TURN_PENALTY,
        "final_epsilon": round(agent.epsilon, 5),
        "final_window_success_rate": round(rate, 4),
        "best_window_success_rate": round(best_success_rate, 4),
        "note": ("以上全部为【简化模拟环境】指标，不代表真实 51 小车性能。"
                 "实体车性能必须另行实测。"),
    }
    return agent, log_rows, summary


def main():
    ap = argparse.ArgumentParser(description="训练表格型 Q-learning 巡线策略")
    ap.add_argument("--episodes", type=int, default=3000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--alpha", type=float, default=0.1)
    ap.add_argument("--gamma", type=float, default=0.95)
    ap.add_argument("--epsilon-decay", type=float, default=0.995)
    ap.add_argument("--turn-penalty", type=float, default=None,
                    help="硬转的动作代价（默认 -0.05）。调大（更负）可抑制画龙")
    ap.add_argument("--log-every", type=int, default=200)
    args = ap.parse_args()

    os.makedirs(RESULTS_DIR, exist_ok=True)

    print("=" * 62)
    print(" 表格型 Q-learning 巡线策略训练（简化模拟环境）")
    print("=" * 62)
    print(f" 回合数={args.episodes}  种子={args.seed}  "
          f"alpha={args.alpha}  gamma={args.gamma}")

    agent, rows, summary = train(
        episodes=args.episodes, seed=args.seed, alpha=args.alpha,
        gamma=args.gamma, epsilon_decay=args.epsilon_decay,
        turn_penalty=args.turn_penalty,
        log_every=args.log_every,
    )

    q_path = os.path.join(RESULTS_DIR, "q_table.json")
    csv_path = os.path.join(RESULTS_DIR, "train_log.csv")
    sum_path = os.path.join(RESULTS_DIR, "train_summary.json")

    agent.save(q_path)
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with open(sum_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print("-" * 62)
    print(f" 末段成功率（模拟环境）: {summary['final_window_success_rate'] * 100:.1f}%")
    print(f" Q 表     -> {q_path}")
    print(f" 训练日志 -> {csv_path}")
    print(f" 汇总     -> {sum_path}")
    print(" 下一步: python src/evaluate.py   然后   python src/export_policy.py")


if __name__ == "__main__":
    main()
