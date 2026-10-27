"""
表格型 Q-learning
=================

经典更新式（Sutton & Barto, Reinforcement Learning: An Introduction, 2nd ed., 6.5 节）：

    Q(s,a) <- Q(s,a) + alpha * [ r + gamma * max_a' Q(s',a') - Q(s,a) ]

为什么这个项目适合表格型 Q-learning，而不是 DQN / PPO：

    状态数 16 x 动作数 6 = 96 个 Q 值。整个 Q 表用 unsigned char 存也就 96 字节，
    塞进 AT89S52 的 256 字节 RAM 里绰绰有余。神经网络在这个规模上：
      1. 没有任何表达力上的收益（输入本来就是离散的 16 种组合）；
      2. 训练时需要 GPU / 大依赖，和"轻量、可复现"的目标相反；
      3. 部署到 8051 上根本跑不动（没有硬件浮点，8KB Flash 装不下网络权重）。
    只有当状态空间变成连续量（比如加装模拟量距离传感器、编码器测速）时，
    才需要重新考虑函数逼近方法。
"""

import json
import random

from environment import NUM_ACTIONS, NUM_STATES


class QLearningAgent:
    """ε-greedy 表格型 Q-learning 智能体。

    :param alpha: 学习率。太大 -> Q 值震荡不收敛；太小 -> 学得慢。
    :param gamma: 折扣因子。越接近 1 越看重长远收益。
    :param epsilon: 初始探索率。
    :param epsilon_min: 探索率下限（保留一点探索，避免过早锁死）。
    :param epsilon_decay: 每回合结束后的乘性衰减系数。
    :param seed: 随机种子，保证可复现。
    """

    def __init__(self, alpha=0.1, gamma=0.95, epsilon=1.0,
                 epsilon_min=0.05, epsilon_decay=0.995, seed=None):
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay
        self.rng = random.Random(seed)
        # Q 表：Q[state][action]
        self.q = [[0.0] * NUM_ACTIONS for _ in range(NUM_STATES)]

    # --------------------------------------------------------

    def select_action(self, state, greedy=False):
        """ε-greedy 选择动作。

        greedy=True 时完全不探索，用于评估和最终导出。
        平局用随机打破（random tie-breaking），否则 Q 全 0 时永远只会选动作 0。
        """
        if (not greedy) and self.rng.random() < self.epsilon:
            return self.rng.randrange(NUM_ACTIONS)
        return self.best_action(state)

    def best_action(self, state):
        """返回该状态下 Q 值最大的动作，平局随机选取。"""
        row = self.q[state]
        best = max(row)
        candidates = [a for a, v in enumerate(row) if v == best]
        if len(candidates) == 1:
            return candidates[0]
        return self.rng.choice(candidates)

    def update(self, state, action, reward, next_state, done):
        """执行一次 Q-learning 更新。"""
        predict = self.q[state][action]
        if done:
            target = reward
        else:
            target = reward + self.gamma * max(self.q[next_state])
        self.q[state][action] = predict + self.alpha * (target - predict)

    def decay_epsilon(self):
        """每回合结束后衰减探索率，不低于 epsilon_min。"""
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)

    # --------------------------------------------------------
    # 存取
    # --------------------------------------------------------

    def save(self, path):
        """保存为 JSON（人类可读，便于人工检查每个状态的取值，不依赖 numpy）。"""
        payload = {
            "alpha": self.alpha,
            "gamma": self.gamma,
            "epsilon": self.epsilon,
            "q": self.q,
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)

    def load(self, path):
        with open(path, "r", encoding="utf-8") as f:
            payload = json.load(f)
        self.alpha = payload.get("alpha", self.alpha)
        self.gamma = payload.get("gamma", self.gamma)
        self.epsilon = payload.get("epsilon", self.epsilon)
        self.q = payload["q"]
        return self

    def greedy_policy(self):
        """把 Q 表压成离线策略表：policy[state] = 最优动作编号。

        注意：平局时这里固定取编号最小的动作（确定性），
        因为导出的 C 数组必须每次生成都一样（可复现、可 review）。
        """
        policy = []
        for s in range(NUM_STATES):
            row = self.q[s]
            best = max(row)
            policy.append(row.index(best))
        return policy

    def policy_confidence(self):
        """每个状态"最优动作比次优动作好多少"，用来判断策略可信度。

        值接近 0 说明该状态下几个动作差不多，策略是"随便选的"，
        这种状态在实体车上表现会不稳定 —— 是很重要的诊断指标。
        """
        out = []
        for s in range(NUM_STATES):
            row = sorted(self.q[s], reverse=True)
            out.append(row[0] - row[1])
        return out
