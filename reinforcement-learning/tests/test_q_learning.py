"""
Q-learning 智能体与训练流程的测试。
"""

import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "src"))

from environment import NUM_STATES, NUM_ACTIONS                    # noqa: E402
from q_learning import QLearningAgent                              # noqa: E402
from train import train                                            # noqa: E402


class TestQLearningUpdate(unittest.TestCase):

    def test_q_table_shape(self):
        agent = QLearningAgent()
        self.assertEqual(len(agent.q), NUM_STATES)
        self.assertTrue(all(len(row) == NUM_ACTIONS for row in agent.q))

    def test_bellman_update_non_terminal(self):
        """Q(s,a) <- Q(s,a) + alpha*(r + gamma*max Q(s',.) - Q(s,a))"""
        agent = QLearningAgent(alpha=0.5, gamma=0.9, epsilon=0.0, seed=1)
        agent.q[5][2] = 4.0
        agent.q[7] = [1.0, 2.0, 8.0, 0.0, 0.0, 0.0]    # max = 8.0
        agent.update(state=5, action=2, reward=10.0, next_state=7, done=False)
        expected = 4.0 + 0.5 * (10.0 + 0.9 * 8.0 - 4.0)
        self.assertAlmostEqual(agent.q[5][2], expected, places=9)

    def test_bellman_update_terminal(self):
        """终止态没有后继，target 就是 r。"""
        agent = QLearningAgent(alpha=0.5, gamma=0.9, epsilon=0.0, seed=1)
        agent.q[5][2] = 4.0
        agent.update(state=5, action=2, reward=10.0, next_state=7, done=True)
        self.assertAlmostEqual(agent.q[5][2], 4.0 + 0.5 * (10.0 - 4.0), places=9)

    def test_update_only_touches_one_cell(self):
        agent = QLearningAgent(alpha=0.1, epsilon=0.0, seed=1)
        agent.q[3][4] = 1.0
        before = [row[:] for row in agent.q]
        agent.update(3, 4, 5.0, 9, False)
        for s in range(NUM_STATES):
            for a in range(NUM_ACTIONS):
                if (s, a) != (3, 4):
                    self.assertEqual(agent.q[s][a], before[s][a])


class TestEpsilonGreedy(unittest.TestCase):

    def test_greedy_always_exploits(self):
        agent = QLearningAgent(epsilon=1.0, seed=1)
        agent.q[0] = [0.0, 0.0, 9.9, 0.0, 0.0, 0.0]
        for _ in range(50):
            self.assertEqual(agent.select_action(0, greedy=True), 2)

    def test_greedy_flag_ignores_epsilon(self):
        agent = QLearningAgent(epsilon=1.0, seed=1)
        agent.q[0] = [0.0, 0.0, 9.9, 0.0, 0.0, 0.0]
        self.assertEqual(agent.select_action(0, greedy=True), 2)

    def test_epsilon_zero_never_explores(self):
        agent = QLearningAgent(epsilon=0.0, seed=1)
        agent.q[0] = [0.0, 0.0, 9.9, 0.0, 0.0, 0.0]
        for _ in range(50):
            self.assertEqual(agent.select_action(0), 2)

    def test_full_epsilon_explores_all_actions(self):
        agent = QLearningAgent(epsilon=1.0, seed=1)
        seen = {agent.select_action(0) for _ in range(400)}
        self.assertEqual(seen, set(range(NUM_ACTIONS)))

    def test_epsilon_decay_respects_floor(self):
        agent = QLearningAgent(epsilon=1.0, epsilon_min=0.05, epsilon_decay=0.5)
        for _ in range(100):
            agent.decay_epsilon()
        self.assertAlmostEqual(agent.epsilon, 0.05, places=9)


class TestPolicyExtraction(unittest.TestCase):

    def test_greedy_policy_is_argmax(self):
        agent = QLearningAgent()
        agent.q[0] = [1.0, 5.0, 2.0, 0.0, 0.0, 0.0]
        agent.q[1] = [0.0, 0.0, 0.0, 0.0, 0.0, 3.0]
        policy = agent.greedy_policy()
        self.assertEqual(policy[0], 1)
        self.assertEqual(policy[1], 5)
        self.assertEqual(len(policy), NUM_STATES)

    def test_greedy_policy_is_deterministic_on_ties(self):
        """平局必须固定取编号最小的动作，导出的 C 数组才可复现。"""
        agent = QLearningAgent(seed=99)
        agent.q[4] = [7.0, 7.0, 0.0, 0.0, 0.0, 0.0]
        for _ in range(20):
            self.assertEqual(agent.greedy_policy()[4], 0)

    def test_policy_confidence(self):
        agent = QLearningAgent()
        agent.q[0] = [1.0, 5.0, 2.0, 0.0, 0.0, 0.0]
        self.assertAlmostEqual(agent.policy_confidence()[0], 3.0, places=9)


class TestSaveLoad(unittest.TestCase):

    def test_roundtrip(self):
        agent = QLearningAgent(alpha=0.3, gamma=0.8, seed=1)
        agent.q[2][3] = 1.25
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "q.json")
            agent.save(path)
            self.assertTrue(os.path.exists(path))
            other = QLearningAgent().load(path)
        self.assertAlmostEqual(other.alpha, 0.3, places=9)
        self.assertAlmostEqual(other.gamma, 0.8, places=9)
        self.assertAlmostEqual(other.q[2][3], 1.25, places=9)

    def test_json_is_human_readable(self):
        """导出成 JSON 而不是 pickle/npy，是为了能直接人工检查 Q 值。"""
        agent = QLearningAgent(seed=1)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "q.json")
            agent.save(path)
            with open(path, encoding="utf-8") as f:
                text = f.read()
        self.assertIn("\"q\"", text)
        self.assertIn("[", text)


class TestTrainingRuns(unittest.TestCase):
    """训练流程端到端能跑通，并且确实在学习。"""

    def test_short_training_improves(self):
        # 用较少回合，只验证"趋势正确"，不验证收敛到某个具体数字
        agent, rows, summary = train(episodes=800, seed=0, log_every=10000,
                                     verbose=False)
        self.assertEqual(len(rows), 800)
        for key in ("episode", "reward", "success", "lost_count", "epsilon"):
            self.assertIn(key, rows[0])
        first = sum(r["reward"] for r in rows[:100]) / 100
        last = sum(r["reward"] for r in rows[-100:]) / 100
        self.assertGreater(last, first,
                           f"训练后平均奖励没有提升（首 {first:.1f} -> 末 {last:.1f}）")

    def test_training_is_reproducible(self):
        """同种子必须得到完全相同的 Q 表，否则实验结果无法复现。"""
        a1, _, _ = train(episodes=200, seed=123, log_every=10000, verbose=False)
        a2, _, _ = train(episodes=200, seed=123, log_every=10000, verbose=False)
        self.assertEqual(a1.q, a2.q)

    def test_different_seeds_differ(self):
        a1, _, _ = train(episodes=200, seed=1, log_every=10000, verbose=False)
        a2, _, _ = train(episodes=200, seed=2, log_every=10000, verbose=False)
        self.assertNotEqual(a1.q, a2.q)

    def test_epsilon_decays_during_training(self):
        agent, _, _ = train(episodes=500, seed=0, log_every=10000, verbose=False)
        self.assertLess(agent.epsilon, 1.0)
        self.assertGreaterEqual(agent.epsilon, agent.epsilon_min)


class TestBaselineEvaluation(unittest.TestCase):
    """基线对照路径：必须能在同一个环境里跑 God.c 的规则策略。

    这是 05_对照实验设计.md 要求的"模拟环境内的公平对照"。
    """

    def test_baseline_eval_runs_and_succeeds(self):
        from evaluate import evaluate
        from export_policy import god_c_rule_action

        result = evaluate(None, episodes=20, seed=999, verbose=False,
                          action_fn=god_c_rule_action)
        self.assertEqual(result["policy"], "god_c_rule")
        # 规则策略在简化环境里应当表现良好；若这里不成立，
        # 说明环境或规则表出了问题（而不是"策略好坏"的问题）
        self.assertGreater(result["success_rate"], 0.8,
                           f"基线规则成功率异常低: {result['success_rate']}")

    def test_baseline_is_deterministic(self):
        """规则策略是确定性的 —— 同样的种子必须给同样的结果。

        这正是它相对 Q-learning 的一个优势（Q-learning 随种子波动，
        见 05_*.md 第 5.4 节的多种子数据）。
        """
        from evaluate import evaluate
        from export_policy import god_c_rule_action

        a = evaluate(None, episodes=20, seed=999, verbose=False,
                     action_fn=god_c_rule_action)
        b = evaluate(None, episodes=20, seed=999, verbose=False,
                     action_fn=god_c_rule_action)
        self.assertEqual(a, b)


if __name__ == "__main__":
    unittest.main(verbosity=2)
