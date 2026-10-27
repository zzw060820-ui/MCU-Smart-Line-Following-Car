/* 本文件由 rl_line_follower/src/export_policy.py 自动生成，请勿手工修改。
 * 重新生成: python src/export_policy.py
 * 生成时间等信息请以 results/train_summary.json 为准。 */

#ifndef POLICY_TABLE_H
#define POLICY_TABLE_H

/* ============================================================
 * 方案 B：离线策略表  policy[state] = action
 *
 * 单片机端只需要：
 *   1. 读四路探头 -> state = (P0 >> 4) & 0x0F
 *   2. action = policy_action[state]
 *   3. 按 action 调用现有函数（见 policy_action_to_func 注释）
 *
 * 占用的 code 空间：16 字节
 * ============================================================ */

#define POLICY_NUM_STATES   16
#define POLICY_NUM_ACTIONS   6

/* 动作编号定义 —— 与 src/environment.py 的 ACTION_NAMES 一一对应 */
#define ACT_GO           0   /* Go(1)                         直行 */
#define ACT_LEFT         1   /* LeftTurn(STEP_TURN)           左转 */
#define ACT_RIGHT        2   /* RightTurn(STEP_TURN)          右转 */
#define ACT_TRIM_LEFT    3   /* TrimLeft(STEP_TRIM)           左微调 */
#define ACT_TRIM_RIGHT   4   /* TrimRight(STEP_TRIM)          右微调 */
#define ACT_BACK         5   /* Backward(STEP_BACK)           后退 */

/* 说明：有 8 个状态在简化模拟环境里【从未出现过】，Q 值全为 0。
 *      这些状态的动作来自 God.c 的规则决策表（标注 [规则兜底]），
 *      不是学出来的。实体车上首次运行前，这些行必须人工复核。 */

/* state -> 动作。下标就是 (P0 >> 4) & 0x0F */
code unsigned char policy_action[POLICY_NUM_STATES] = {
    ACT_BACK      ,   /* s= 0  探头LL L R RR = 0000  err=+0  全部为白 —— 丢线 */
    ACT_RIGHT     ,   /* s= 1  探头LL L R RR = 0001  err=+3  仅最右探头压线 —— 线在车右侧很远（车偏左严重），应右转 */
    ACT_RIGHT     ,   /* s= 2  探头LL L R RR = 0010  err=+1  仅右中探头压线 —— 线在车右侧一点，应右转 */
    ACT_RIGHT     ,   /* s= 3  探头LL L R RR = 0011  err=+4  右中+最右压线 —— 线在车右侧较多，应右转 */
    ACT_LEFT      ,   /* s= 4  探头LL L R RR = 0100  err=-1  仅左中探头压线 —— 线在车左侧一点，应左转 */
    ACT_TRIM_RIGHT,   /* s= 5  探头LL L R RR = 0101  err=+2  左中+最右压线（不常见，跨了两条线？）  [规则兜底] */
    ACT_GO        ,   /* s= 6  探头LL L R RR = 0110  err=+0  左中+右中压线 —— 车在线中央（正常巡线态） */
    ACT_RIGHT     ,   /* s= 7  探头LL L R RR = 0111  err=+3  左中+右中+最右（不常见）  [规则兜底] */
    ACT_LEFT      ,   /* s= 8  探头LL L R RR = 1000  err=-3  仅最左探头压线 —— 线在车左侧很远（车偏右严重），应左转 */
    ACT_GO        ,   /* s= 9  探头LL L R RR = 1001  err=+0  最左+最右压线（不常见）  [规则兜底] */
    ACT_TRIM_LEFT ,   /* s=10  探头LL L R RR = 1010  err=-2  最左+右中压线（不常见）  [规则兜底] */
    ACT_TRIM_RIGHT,   /* s=11  探头LL L R RR = 1011  err=+1  最左+右中+最右（不常见）  [规则兜底] */
    ACT_LEFT      ,   /* s=12  探头LL L R RR = 1100  err=-4  最左+左中压线 —— 线在车左侧较多，应左转 */
    ACT_TRIM_LEFT ,   /* s=13  探头LL L R RR = 1101  err=-1  最左+左中+最右（不常见）  [规则兜底] */
    ACT_LEFT      ,   /* s=14  探头LL L R RR = 1110  err=-3  最左+左中+右中（不常见）  [规则兜底] */
    ACT_GO        ,   /* s=15  探头LL L R RR = 1111  err=+0  四路全压线 —— 交叉线 / 横线 / 起跑标记  [规则兜底] */
};

/* 对应的 God.c 函数调用（仅供人工核对，不是可编译代码）:
     ACT_GO         -> Go(1)
     ACT_LEFT       -> LeftTurn(STEP_TURN)
     ACT_RIGHT      -> RightTurn(STEP_TURN)
     ACT_TRIM_LEFT  -> TrimLeft(STEP_TRIM)
     ACT_TRIM_RIGHT -> TrimRight(STEP_TRIM)
     ACT_BACK       -> Backward(STEP_BACK)
*/

#endif /* POLICY_TABLE_H */
