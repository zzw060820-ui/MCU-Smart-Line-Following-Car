"""
环境 / 状态编码 / 动作映射的基础测试。

运行：
    cd rl_line_follower
    python -m unittest discover -s tests -v
（不需要安装 pytest；同时也兼容 pytest。）
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "src"))

from environment import (LineFollowerEnv, encode_state, decode_state,   # noqa: E402
                         line_position_error, state_is_on_line,
                         NUM_STATES, NUM_ACTIONS,
                         ACTION_WHEEL_SPEED, ACTION_NAMES,
                         A_GO, A_LEFT, A_RIGHT, A_TRIM_LEFT, A_TRIM_RIGHT, A_BACK)


class TestStateEncoding(unittest.TestCase):
    """状态编码必须与 God.c 的探头顺序、极性一致。"""

    def test_encode_matches_p0_high_nibble(self):
        """state 必须等于 (P0 >> 4) & 0x0F。

        God.c 里 LL=P0_7, L=P0_6, R=P0_5, RR=P0_4，
        所以状态号可以直接由 P0 的高四位得到，单片机端不需要查表。
        """
        for p0 in range(256):
            ll = (p0 >> 7) & 1
            l = (p0 >> 6) & 1
            r = (p0 >> 5) & 1
            rr = (p0 >> 4) & 1
            self.assertEqual(encode_state((ll, l, r, rr)), (p0 >> 4) & 0x0F)

    def test_encode_decode_roundtrip(self):
        for s in range(NUM_STATES):
            self.assertEqual(encode_state(decode_state(s)), s)

    def test_state_order(self):
        """最左探头是高位。对应 God.c 的 QTI_LL() -> P0_7。"""
        self.assertEqual(encode_state((1, 0, 0, 0)), 0b1000)   # 仅最左
        self.assertEqual(encode_state((0, 0, 0, 1)), 0b0001)   # 仅最右

    def test_all_white_is_lost(self):
        self.assertEqual(encode_state((0, 0, 0, 0)), 0)
        self.assertFalse(state_is_on_line(0))

    def test_centered_state(self):
        """车在线中央 = 左中 + 右中压线 = 0b0110 = 6。"""
        self.assertEqual(encode_state((0, 1, 1, 0)), 6)

    def test_decode_out_of_range(self):
        with self.assertRaises(ValueError):
            decode_state(16)


class TestPositionError(unittest.TestCase):
    """err 必须复刻 God.c:365-369 的决策表。"""

    def test_err_formula(self):
        # God.c: err = -3*L - 1*M1 + 1*M2 + 3*R  (L=LL, M1=L, M2=R, R=RR)
        for s in range(NUM_STATES):
            ll, l, r, rr = decode_state(s)
            self.assertEqual(line_position_error(s), -3 * ll - 1 * l + 1 * r + 3 * rr)

    def test_known_cases(self):
        self.assertEqual(line_position_error(0b0110), 0)     # 居中
        self.assertEqual(line_position_error(0b1000), -3)    # 仅最左 -> 左转
        self.assertEqual(line_position_error(0b0001), +3)    # 仅最右 -> 右转
        self.assertEqual(line_position_error(0b1100), -4)    # 最左+左中
        self.assertEqual(line_position_error(0b0100), -1)    # 仅左中 -> 微调


class TestActionSpace(unittest.TestCase):
    """动作的轮速必须与 God.c 的 #define 一致。"""

    def test_wheel_speeds_match_god_c(self):
        # God.c: SPD_GO=200 SPD_TURN=450 SPD_BACK=400 SPD_TRIM=150
        self.assertEqual(ACTION_WHEEL_SPEED[A_GO], (200, 200))
        self.assertEqual(ACTION_WHEEL_SPEED[A_LEFT], (-450, 450))
        self.assertEqual(ACTION_WHEEL_SPEED[A_RIGHT], (450, -450))
        self.assertEqual(ACTION_WHEEL_SPEED[A_TRIM_LEFT], (200 - 150, 200 + 150))
        self.assertEqual(ACTION_WHEEL_SPEED[A_TRIM_RIGHT], (200 + 150, 200 - 150))
        self.assertEqual(ACTION_WHEEL_SPEED[A_BACK], (-400, -400))

    def test_all_actions_covered(self):
        self.assertEqual(len(ACTION_WHEEL_SPEED), NUM_ACTIONS)
        self.assertEqual(len(ACTION_NAMES), NUM_ACTIONS)

    def test_pulse_width_safe_range(self):
        """脉宽 = SERVO_C ± 速度 必须落在舵机行程内。

        God.c 备注：SERVO_C=1520，上限是 min(2000-1520, 1520-1000)=480。
        """
        servo_c = 1520
        for a, (lv, rv) in ACTION_WHEEL_SPEED.items():
            for spd in (lv, rv):
                self.assertLessEqual(abs(spd), 480,
                                     f"动作 {ACTION_NAMES[a]} 速度 {spd} 超出安全范围")
                self.assertGreaterEqual(servo_c + spd, 1000)
                self.assertLessEqual(servo_c + spd, 2000)


class TestRewardTuning(unittest.TestCase):
    """动作代价必须是可调参数（这是本项目最需要调的一个旋钮）。"""

    def test_penalty_attributes_exist_and_are_negative(self):
        env = LineFollowerEnv(seed=0)
        for name in ("TURN_PENALTY", "TRIM_PENALTY", "BACK_PENALTY"):
            self.assertTrue(hasattr(env, name), f"缺少 {name}")
            self.assertLess(getattr(env, name), 0, f"{name} 应为负值")

    def test_turn_penalty_actually_changes_reward(self):
        """把转向代价调得更负，同一个转移的奖励必须变小。"""
        env = LineFollowerEnv(seed=0)
        env.reset()
        env.lat = 1.0
        env.theta = 0.0
        _, r_default, _, _ = env.step(A_RIGHT)

        env2 = LineFollowerEnv(seed=0)
        env2.TURN_PENALTY = -10.0
        env2.reset()
        env2.lat = 1.0
        env2.theta = 0.0
        _, r_strong, _, _ = env2.step(A_RIGHT)

        self.assertLess(r_strong, r_default)

    def test_train_respects_turn_penalty_flag(self):
        from train import train
        env_seen = train(episodes=1, seed=0, turn_penalty=-2.0,
                         log_every=10000, verbose=False)[2]
        self.assertAlmostEqual(env_seen["turn_penalty"], -2.0, places=9)


class TestEnvironment(unittest.TestCase):
    """环境几何必须能复现 God.c 的决策结构。"""

    def setUp(self):
        self.env = LineFollowerEnv(seed=42)

    def test_centered_reads_state_6(self):
        self.env.reset()
        self.env.lat = 0.0
        self.env.theta = 0.0
        self.assertEqual(self.env._read_sensors(), 0b0110)

    def test_car_left_of_line_puts_line_on_right(self):
        """lat > 0 = 车偏在线左侧 -> 线在车的右侧 -> 右边的探头压线 -> err > 0。"""
        self.env.reset()
        self.env.theta = 0.0
        self.env.lat = 1.0
        s = self.env._read_sensors()
        self.assertEqual(s, 0b0011)                  # 右中 + 最右
        self.assertGreater(line_position_error(s), 0)

    def test_car_right_of_line_puts_line_on_left(self):
        self.env.reset()
        self.env.theta = 0.0
        self.env.lat = -1.0
        s = self.env._read_sensors()
        self.assertEqual(s, 0b1100)                  # 最左 + 左中
        self.assertLess(line_position_error(s), 0)

    def test_far_drift_is_lost(self):
        self.env.reset()
        self.env.theta = 0.0
        self.env.lat = 5.0
        self.assertEqual(self.env._read_sensors(), 0)

    def test_left_and_right_are_mirrored(self):
        """几何必须左右对称。"""
        self.env.reset()
        self.env.theta = 0.0
        self.env.lat = 1.0
        left = self.env._read_sensors()
        self.env.lat = -1.0
        right = self.env._read_sensors()
        self.assertEqual(decode_state(left), tuple(reversed(decode_state(right))))

    def test_recovery_direction_matches_god_c(self):
        """★ 关键回归测试：环境的左右几何必须与 God.c 的决策表一致。

        这条测试防止"环境里传感器左右装反"这类错误 ——
        装反之后模拟环境内部仍然自洽（照样能训到 100% 成功率），
        但导出的策略在实体车上会【左右反着转】，直接把车开出赛道。

        绑定关系：
            God.c:  err > 0 -> RightTurn，err < 0 -> LeftTurn
            环境:   err > 0 必须真的能让车往右走回线路上
        """
        env = LineFollowerEnv(seed=0)
        cases = [
            (+1.5, A_RIGHT, "车偏在线左侧很远"),
            (+1.0, A_RIGHT, "车偏在线左侧"),
            (-1.0, A_LEFT, "车偏在线右侧"),
            (-1.5, A_LEFT, "车偏在线右侧很远"),
        ]
        for lat0, expected_action, desc in cases:
            env.reset()
            env.lat = lat0
            env.theta = 0.0

            s = env._read_sensors()
            err = line_position_error(s)
            self.assertNotEqual(err, 0, f"{desc}: err 不该为 0")
            action = A_RIGHT if err > 0 else A_LEFT
            self.assertEqual(action, expected_action,
                             f"{desc}: God.c 的 err={err:+d} 应给出 "
                             f"{ACTION_NAMES[expected_action]}")

            # 转向 + 直行之后，车必须朝线路方向靠近（|lat| 变小）
            before = abs(env.lat)
            env.step(action)      # 先转
            env.step(A_GO)        # 再走一步
            after = abs(env.lat)
            self.assertLess(after, before,
                            f"{desc}: 执行 {ACTION_NAMES[action]} 后车反而离线路更远了"
                            f"（|lat| {before:.2f} -> {after:.2f}），左右几何可能装反")

    def test_step_returns_expected_tuple(self):
        self.env.reset()
        out = self.env.step(A_GO)
        self.assertEqual(len(out), 4)
        _, reward, done, info = out
        self.assertIsInstance(reward, float)
        self.assertIsInstance(done, bool)
        self.assertIn("lat", info)

    def test_go_moves_forward(self):
        self.env.reset()
        self.env.lat = 0.0
        self.env.theta = 0.0
        s0 = self.env.s
        for _ in range(5):
            self.env.step(A_GO)
        self.assertGreater(self.env.s, s0)
        self.assertAlmostEqual(self.env.theta, 0.0, places=6)

    def test_left_turn_increases_theta(self):
        """左转动作应该让航向角变大（逆时针）。"""
        self.env.reset()
        self.env.theta = 0.0
        self.env.step(A_LEFT)
        self.assertGreater(self.env.theta, 0.0)

    def test_right_turn_decreases_theta(self):
        self.env.reset()
        self.env.theta = 0.0
        self.env.step(A_RIGHT)
        self.assertLess(self.env.theta, 0.0)

    def test_reset_is_randomized(self):
        """起点必须随机化，否则策略只会背一条轨迹。"""
        seen = set()
        for _ in range(30):
            self.env.reset()
            seen.add(round(self.env.lat, 3))
        self.assertGreater(len(seen), 10, "起点随机化不足，策略会过拟合单一初始条件")

    def test_episode_terminates(self):
        env = LineFollowerEnv(seed=7)
        state = env.reset()
        steps = 0
        while True:
            state, _, done, _ = env.step(A_GO)
            steps += 1
            self.assertLessEqual(steps, env.MAX_STEPS, "回合没有终止，存在死循环")
            if done:
                break

    def test_pure_go_fails_on_curved_track(self):
        """赛道有弯道，纯"永远直行"必然冲出去。

        这个测试保证环境不是一个"不学就能过"的退化任务：
        如果赛道是纯直道，从正中出发一直直行就能跑完，
        Q 表什么都不用学 —— 那样的环境验证不了算法流程。
        """
        for seed in range(5):
            env = LineFollowerEnv(seed=seed)
            env.reset()
            while True:
                _, _, done, _ = env.step(A_GO)
                if done:
                    break
            self.assertFalse(env.success,
                             f"seed={seed}: 纯直行竟然跑完了，赛道太简单")

    def test_goal_reward_beats_surviving(self):
        """跑完全程的总回报必须明显高于"在线路上撑一会儿然后丢线"。"""
        env = LineFollowerEnv(seed=1)
        env.reset()
        failed_total = 0.0
        while True:
            _, r, done, _ = env.step(A_GO)
            failed_total += r
            if done:
                break
        self.assertFalse(env.success)
        # 一个跑完全程的回合，总回报 = 每步正奖励 + 终点奖励，应当远高于此
        self.assertLess(failed_total, 150.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
