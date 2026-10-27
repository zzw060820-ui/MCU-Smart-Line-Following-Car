"""
策略导出测试：验证生成的 C 数组内容正确、可被人工核对、且与训练结果一致。
"""

import os
import re
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "src"))

import export_policy                                            # noqa: E402
from environment import (NUM_STATES, NUM_ACTIONS, ACTION_NAMES,  # noqa: E402
                         ACTION_FIRMWARE_FUNC, decode_state, line_position_error)
from q_learning import QLearningAgent                            # noqa: E402


def make_agent():
    """造一个 Q 值有明确偏好的假 agent，用于确定性测试。"""
    agent = QLearningAgent(seed=0)
    # 每个状态给一个不同的最优动作，方便校验下标
    for s in range(NUM_STATES):
        for a in range(NUM_ACTIONS):
            agent.q[s][a] = 0.0
        agent.q[s][s % NUM_ACTIONS] = 10.0 + s
    return agent


def strip_comments(text):
    """去掉 C 注释，方便从生成的代码里解析数组内容。"""
    return re.sub(r"/\*.*?\*/", "", text, flags=re.S)


def parse_flat_array(text, name, size_macro):
    """从生成的 C 代码里解析一维数组的元素（已去注释）。"""
    m = re.search(rf"{name}\[{size_macro}\]\s*=\s*\{{(.*?)\}};",
                  strip_comments(text), re.S)
    assert m, f"没有找到数组 {name} 的定义"
    return [x.strip() for x in m.group(1).split(",") if x.strip()]


def parse_2d_array(text, name, rows_macro, cols_macro):
    """解析二维数组，返回 [行][列] 的整数列表。

    rows_macro/cols_macro 传源码里真实出现的下标写法（通常是宏名，如 Q_NUM_STATES），
    不要传数字 —— 生成的代码里写的是宏名。
    """
    m = re.search(rf"{name}\[{rows_macro}\]\[{cols_macro}\]\s*=\s*\{{(.*?)\n\}};",
                  strip_comments(text), re.S)
    assert m, f"没有找到数组 {name} 的定义"
    out = []
    for row in re.findall(r"\{\s*([0-9,\s]+?)\s*\}", m.group(1)):
        out.append([int(x) for x in row.split(",") if x.strip()])
    return out


class TestPolicyHeader(unittest.TestCase):

    def setUp(self):
        self.agent = make_agent()
        self.policy = self.agent.greedy_policy()
        self.text = export_policy.build_policy_header(self.policy, self.agent)

    def test_has_include_guard(self):
        self.assertIn("#ifndef POLICY_TABLE_H", self.text)
        self.assertIn("#endif", self.text)

    def test_uses_code_keyword_for_8051(self):
        """必须用 code 关键字，让数组落在 flash 而不是 256 字节的 RAM 里。"""
        self.assertIn("code unsigned char policy_action", self.text)

    def test_array_length_is_16(self):
        entries = parse_flat_array(self.text, "policy_action", "POLICY_NUM_STATES")
        self.assertEqual(len(entries), NUM_STATES)

    def test_entries_match_policy(self):
        entries = parse_flat_array(self.text, "policy_action", "POLICY_NUM_STATES")
        for s, entry in enumerate(entries):
            self.assertEqual(entry, f"ACT_{ACTION_NAMES[self.policy[s]]}",
                             f"状态 {s} 导出的动作与 policy 不一致")

    def test_action_macros_defined(self):
        for a in range(NUM_ACTIONS):
            self.assertIn(f"#define ACT_{ACTION_NAMES[a]}", self.text)

    def test_firmware_function_names_present(self):
        for a in range(NUM_ACTIONS):
            self.assertIn(ACTION_FIRMWARE_FUNC[a], self.text)

    def test_every_state_has_comment(self):
        """每个状态都要有中文注释，导出后能人工核对。"""
        for s in range(NUM_STATES):
            bits = "".join(str(b) for b in decode_state(s))
            self.assertIn(bits, self.text)
            self.assertIn(f"err={line_position_error(s):+d}", self.text)


class TestQTableHeader(unittest.TestCase):

    def setUp(self):
        self.agent = make_agent()
        self.text, self.mismatch = export_policy.build_qtables_header(self.agent)

    def test_has_include_guard(self):
        self.assertIn("#ifndef Q_TABLE_H", self.text)

    def test_uses_code_keyword(self):
        self.assertIn("code unsigned char q_table", self.text)

    def test_shape_16x6(self):
        rows = parse_2d_array(self.text, "q_table", "Q_NUM_STATES", "Q_NUM_ACTIONS")
        self.assertEqual(len(rows), NUM_STATES)
        for s, row in enumerate(rows):
            self.assertEqual(len(row), NUM_ACTIONS, f"状态 {s} 的行长度不对")
            for v in row:
                self.assertTrue(0 <= v <= 255, f"状态 {s} 的量化值越界: {v}")

    def test_quantized_argmax_preserved(self):
        """量化不能改变最优动作，否则片上会选错。"""
        rows = parse_2d_array(self.text, "q_table", "Q_NUM_STATES", "Q_NUM_ACTIONS")
        for s, row in enumerate(rows):
            self.assertEqual(row.index(max(row)),
                             self.agent.q[s].index(max(self.agent.q[s])),
                             f"状态 {s} 量化后最优动作改变")
        self.assertEqual(self.mismatch, [])

    def test_quantization_is_monotonic(self):
        """线性量化必须保序：原 Q 值 q1 < q2 => 量化值 v1 <= v2。

        保序是"量化后 argmax 不变"的充分条件，这里直接验证保序性。
        """
        rows = parse_2d_array(self.text, "q_table", "Q_NUM_STATES", "Q_NUM_ACTIONS")
        pairs = [(self.agent.q[s][a], rows[s][a])
                 for s in range(NUM_STATES) for a in range(NUM_ACTIONS)]

        for q1, v1 in pairs:
            for q2, v2 in pairs:
                if q1 < q2:
                    self.assertLessEqual(v1, v2,
                                         f"量化破坏了顺序: q={q1}->{v1} 但 q={q2}->{v2}")


class TestRuleFallback(unittest.TestCase):
    """未访问状态的规则兜底必须与 God.c 的决策表一致。"""

    def test_rule_matches_god_c_decision_table(self):
        g = export_policy.god_c_rule_action
        # God.c: err>0 偏右修正；err<0 偏左修正；|err|>=3 用硬转，否则微调
        self.assertEqual(ACTION_NAMES[g(0b0001)], "RIGHT")        # err=+3
        self.assertEqual(ACTION_NAMES[g(0b0010)], "TRIM_RIGHT")   # err=+1
        self.assertEqual(ACTION_NAMES[g(0b0011)], "RIGHT")        # err=+4
        self.assertEqual(ACTION_NAMES[g(0b0100)], "TRIM_LEFT")    # err=-1
        self.assertEqual(ACTION_NAMES[g(0b1000)], "LEFT")         # err=-3
        self.assertEqual(ACTION_NAMES[g(0b1100)], "LEFT")         # err=-4
        self.assertEqual(ACTION_NAMES[g(0b0110)], "GO")           # err=0
        self.assertEqual(ACTION_NAMES[g(0b0000)], "BACK")         # 丢线

    def test_rule_is_defined_for_every_state(self):
        for s in range(NUM_STATES):
            a = export_policy.god_c_rule_action(s)
            self.assertIn(a, range(NUM_ACTIONS))

    def test_unvisited_states_get_fallback(self):
        agent = make_agent()
        # 把所有状态的 Q 清零，模拟"一个状态都没访问过"
        for s in range(NUM_STATES):
            agent.q[s] = [0.0] * NUM_ACTIONS
        policy, fb, visited = export_policy.build_policy(agent, use_fallback=True)
        self.assertEqual(visited, set())
        self.assertEqual(fb, set(range(NUM_STATES)))
        for s in range(NUM_STATES):
            self.assertEqual(policy[s], export_policy.god_c_rule_action(s))

    def test_visited_states_keep_learned_action(self):
        agent = make_agent()
        agent.q[6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        agent.q[6][2] = 5.0            # 状态 6 学过：选动作 2
        policy, fb, visited = export_policy.build_policy(agent, use_fallback=True)
        self.assertIn(6, visited)
        self.assertNotIn(6, fb)
        self.assertEqual(policy[6], 2)

    def test_no_fallback_flag(self):
        agent = make_agent()
        for s in range(NUM_STATES):
            agent.q[s] = [0.0] * NUM_ACTIONS
        policy, fb, _ = export_policy.build_policy(agent, use_fallback=False)
        self.assertEqual(fb, set())
        self.assertTrue(all(a == 0 for a in policy))

    def test_policy_header_marks_fallback_rows(self):
        agent = make_agent()
        agent.q[0] = [0.0] * NUM_ACTIONS
        policy, fb, _ = export_policy.build_policy(agent, use_fallback=True)
        text = export_policy.build_policy_header(policy, agent, fb)
        self.assertIn("[规则兜底]", text)
        self.assertIn("从未出现过", text)


class TestWriteAll(unittest.TestCase):

    def test_main_writes_three_files(self):
        """export_policy.main() 真的会写出三个文件。"""
        agent = make_agent()
        with tempfile.TemporaryDirectory() as d:
            q_path = os.path.join(d, "q_table.json")
            agent.save(q_path)
            out = os.path.join(d, "firmware_policy")

            argv_backup = sys.argv
            try:
                sys.argv = ["export_policy.py", "--q-table", q_path, "--out", out,
                            "--summary", os.path.join(d, "nope.json")]
                export_policy.main()
            finally:
                sys.argv = argv_backup

            for name in ("policy_table.h", "q_table.h", "README.md"):
                self.assertTrue(os.path.exists(os.path.join(out, name)),
                                f"没有生成 {name}")

            with open(os.path.join(out, "policy_table.h"), encoding="utf-8") as f:
                text = f.read()
            self.assertIn("policy_action", text)

            with open(os.path.join(out, "README.md"), encoding="utf-8") as f:
                readme = f.read()
            self.assertIn("state", readme)
            self.assertIn("动作编号", readme)
            # 必须明确声明"尚未在实体车上验证"
            self.assertIn("尚未在实体小车", readme)


class TestFirmwareArtifactsInRepo(unittest.TestCase):
    """仓库里实际生成的 firmware_policy/ 应当与 JSON 里的 Q 表一致。"""

    def setUp(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.q_path = os.path.join(root, "results", "q_table.json")
        self.pol_path = os.path.join(root, "firmware_policy", "policy_table.h")

    def test_files_exist_after_export(self):
        missing = [p for p in (self.q_path, self.pol_path) if not os.path.exists(p)]
        if missing:
            self.skipTest("尚未运行 train.py / export_policy.py，跳过: "
                          + ", ".join(missing))

        agent = QLearningAgent().load(self.q_path)
        # 导出时默认会做规则兜底，所以这里也必须用 build_policy 才能对上
        policy, _, _ = export_policy.build_policy(agent, use_fallback=True)
        with open(self.pol_path, encoding="utf-8") as f:
            text = f.read()
        entries = parse_flat_array(text, "policy_action", "POLICY_NUM_STATES")
        self.assertEqual(len(entries), NUM_STATES)
        for s, entry in enumerate(entries):
            self.assertEqual(entry, f"ACT_{ACTION_NAMES[policy[s]]}",
                             f"仓库里的 policy_table.h 与 q_table.json 不一致（状态 {s}）")


if __name__ == "__main__":
    unittest.main(verbosity=2)
