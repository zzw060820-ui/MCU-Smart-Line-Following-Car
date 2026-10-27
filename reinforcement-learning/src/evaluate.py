"""
独立策略评估程序
================

用训练好的 Q 表，在【完全不探索】（greedy）的情况下跑独立回合，
输出成功率、丢线次数、转向次数、平均完成步数等指标。

用法：
    python src/evaluate.py --episodes 200 --seed 12345

评估必须用【和训练不同的随机种子】，否则测的是"背下来的起点"，不是泛化能力。
见 03_*.md 第 5 节"如何避免策略只适用于单一起点"。
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from environment import (LineFollowerEnv, NUM_STATES, ACTION_NAMES,   # noqa: E402
                         STATE_NAMES, decode_state, line_position_error)
from q_learning import QLearningAgent                                  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(ROOT, "results")


def evaluate(agent, episodes=200, seed=12345, verbose=True, action_fn=None):
    """跑独立评估。

    :param action_fn: 若提供，则用它选动作（签名 action_fn(state) -> action），
                      用于在【同一个模拟环境里】公平对照基线规则策略。
                      不提供时用 agent 的 greedy 策略。
    """
    env = LineFollowerEnv(seed=seed)

    if action_fn is None:
        def action_fn(s):
            return agent.select_action(s, greedy=True)

    n_success = 0
    total_lost = 0
    total_turns = 0
    total_steps = 0
    total_reward = 0.0
    failures = {"lost": 0, "timeout": 0}

    for _ in range(episodes):
        state = env.reset()
        ep_reward = 0.0
        while True:
            action = action_fn(state)
            state, reward, done, info = env.step(action)
            ep_reward += reward
            if done:
                break

        total_reward += ep_reward
        total_lost += env.lost_count
        total_turns += env.turn_count
        total_steps += env.steps
        if env.success:
            n_success += 1
        elif info["failed"]:
            failures["lost"] += 1
        else:
            failures["timeout"] += 1

    result = {
        "policy": "god_c_rule" if action_fn else "q_learning_greedy",
        "episodes": episodes,
        "eval_seed": seed,
        "success_rate": round(n_success / episodes, 4),
        "avg_lost_per_episode": round(total_lost / episodes, 3),
        "avg_turns_per_episode": round(total_turns / episodes, 3),
        "avg_steps_per_episode": round(total_steps / episodes, 1),
        "avg_reward": round(total_reward / episodes, 3),
        "failure_breakdown": failures,
        "note": "简化模拟环境指标，不代表真实小车性能。",
    }

    if verbose:
        label = "基线规则策略（God.c 决策表）" if action_fn else "Q-learning 策略（greedy）"
        print("=" * 62)
        print(f" 独立评估：{label}")
        print(f" 回合数={episodes}  种子={seed}")
        print("=" * 62)
        print(f" 成功率          : {result['success_rate'] * 100:.1f}%")
        print(f" 平均丢线次数/回合: {result['avg_lost_per_episode']}")
        print(f" 平均转向次数/回合: {result['avg_turns_per_episode']}")
        print(f" 平均步数/回合    : {result['avg_steps_per_episode']}")
        print(f" 平均累计奖励     : {result['avg_reward']}")
        print(f" 失败构成         : {failures}")
    return result


def print_policy_table(agent):
    """打印"状态 -> 策略动作"对照表，供人工核对，也是导出前的最后一道检查。"""
    policy = agent.greedy_policy()
    conf = agent.policy_confidence()

    print()
    print("=" * 78)
    print(" 离线策略表  policy[state] = action")
    print("=" * 78)
    print(" state  四路探头(LL L R RR)   err   动作            Q值置信度  说明")
    print("-" * 78)
    flags = []
    for s in range(NUM_STATES):
        bits = "".join(str(b) for b in decode_state(s))
        err = line_position_error(s)
        c = conf[s]
        warn = ""
        if s != 0 and c < 1e-9:
            warn = "  <-- 平局！策略不可信"
            flags.append(s)
        print(f"  {s:2d}    {bits}                {err:+3d}   "
              f"{ACTION_NAMES[policy[s]]:<12s}  {c:8.3f}   "
              f"{STATE_NAMES[s]}{warn}")
    print("-" * 78)
    if flags:
        print(f" [警告] 有 {len(flags)} 个状态存在平局: {flags}")
        print("        这些状态的动作基本是随机选的，实体车上表现会不稳。")
        print("        处理办法：加大训练回合数，或给这些状态补一点探索。")
    else:
        print(" [通过] 所有状态的 Q 值都有明显偏好，没有平局。")
    return policy


def main():
    ap = argparse.ArgumentParser(description="评估训练好的 Q-learning 策略")
    ap.add_argument("--q-table", default=os.path.join(RESULTS_DIR, "q_table.json"))
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--seed", type=int, default=12345,
                    help="评估用种子，必须与训练种子不同")
    ap.add_argument("--save", action="store_true", help="把评估结果写入 results/")
    ap.add_argument("--baseline", action="store_true",
                    help="评估 God.c 的规则策略（基线），而不是 Q-learning 策略。"
                         "用于在同一个模拟环境里做公平对照。")
    args = ap.parse_args()

    if args.baseline:
        # 基线：God.c 的规则决策表。用同一个环境、同一个种子，才能公平对照。
        from export_policy import god_c_rule_action
        result = evaluate(None, episodes=args.episodes, seed=args.seed,
                          action_fn=god_c_rule_action)
        if args.save:
            os.makedirs(RESULTS_DIR, exist_ok=True)
            path = os.path.join(RESULTS_DIR, "eval_baseline.json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump(result, f, indent=2, ensure_ascii=False)
            print(f"\n 基线评估结果已写入 {path}")
        return

    if not os.path.exists(args.q_table):
        print(f"[错误] 找不到 Q 表: {args.q_table}")
        print("       请先运行: python src/train.py")
        sys.exit(1)

    agent = QLearningAgent(seed=args.seed).load(args.q_table)
    result = evaluate(agent, episodes=args.episodes, seed=args.seed)
    print_policy_table(agent)

    if args.save:
        os.makedirs(RESULTS_DIR, exist_ok=True)
        path = os.path.join(RESULTS_DIR, "eval_result.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
        print(f"\n 评估结果已写入 {path}")


if __name__ == "__main__":
    main()
