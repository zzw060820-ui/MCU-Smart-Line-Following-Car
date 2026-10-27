/*================================================================
 *  导出的策略表 —— 编译验证程序
 *
 *  目的：证明 export_policy.py 生成的 policy_table.h / q_table.h
 *        能被真实的 Keil C51 编译器、以真实的 8051 目标编译通过，
 *        而不只是"Python 侧看起来对"。
 *
 *  这不是小车的运行程序。它只包含：
 *    - 两个导出头文件的 include
 *    - 与 God.c 同名的空函数壳子
 *    - 两种查表方式的完整调用路径
 *
 *  编译方式（在 tools/ 目录下）：
 *      C51.EXE policy_compile_check.c INCDIR(../firmware_policy) INCDIR(D:\Keil_v5\C51\INC)
 *
 *  约定：状态号 = (P0 >> 4) & 0x0F，与 God.c 的四路探头一致。
 *================================================================*/

#include <REG52.H>
#include "policy_table.h"
#include "q_table.h"

/* ---- 与 God.c 同名的空壳。只为编译验证调用路径，不是真实实现。 ---- */
void Go(unsigned char steps)        { steps = steps; }
void Backward(unsigned char steps)  { steps = steps; }
void LeftTurn(unsigned char steps)  { steps = steps; }
void RightTurn(unsigned char steps) { steps = steps; }
void TrimLeft(unsigned char steps)  { steps = steps; }
void TrimRight(unsigned char steps) { steps = steps; }

/* God.c 里的步数常量，这里保持同名以便照抄调用形式 */
#define STEP_TURN   4
#define STEP_TRIM   2
#define STEP_BACK   2

/* ---- 方式一：方案 B，直接查离线策略表（16 字节）---- */
static unsigned char action_from_policy_table(void)
{
    unsigned char state = (P0 >> 4) & 0x0F;   /* 四路探头 -> 状态号 */
    return policy_action[state];
}

/* ---- 方式二：方案 A，在片上对 6 个 Q 值取 argmax（96 字节）---- */
static unsigned char action_from_q_table(void)
{
    unsigned char state = (P0 >> 4) & 0x0F;
    unsigned char a;
    unsigned char best = 0;
    unsigned char bestv = 0;

    for (a = 0; a < Q_NUM_ACTIONS; a++)
    {
        if (q_table[state][a] > bestv)
        {
            bestv = q_table[state][a];
            best  = a;
        }
    }
    return best;
}

static void run_action(unsigned char a)
{
    switch (a)
    {
        case ACT_GO:         Go(1);                 break;
        case ACT_LEFT:       LeftTurn(STEP_TURN);   break;
        case ACT_RIGHT:      RightTurn(STEP_TURN);  break;
        case ACT_TRIM_LEFT:  TrimLeft(STEP_TRIM);   break;
        case ACT_TRIM_RIGHT: TrimRight(STEP_TRIM);  break;
        case ACT_BACK:       Backward(STEP_BACK);   break;
        default:                                    break;
    }
}

void main(void)
{
    unsigned char a;

    while (1)
    {
        /* 两种方式都必须能编译、能索引、能落到同一个 switch 上 */
        a = action_from_policy_table();
        run_action(a);

        a = action_from_q_table();
        run_action(a);

        /* 覆盖全部 16 个状态的下标，确认没有越界（编译期即可看出常量范围）*/
        if (policy_action[15] == policy_action[0]) { a = 0; }
    }
}
