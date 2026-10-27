#include <uart.h>
#include <BoeBot.h>


/*舵机中值，SERVO_C = (n_L + n_R) / 2 = (1535 + 1505) / 2 = 1520  */
#define SERVO_C         1520

/* 速度档位：偏离中值的微秒数。正数 = 前进，负数 = 后退。
 *   数值越大车越快。上限由 SERVO_C 挤出来：脉宽 = SERVO_C ± 速度，
 *   必须落在 1000~2000 微秒内，所以 |速度| <= min(2000-SERVO_C, SERVO_C-1000)。
 *   现在 SERVO_C = 1520，上限是 480（之前 SERVO_C = 1400 时是 600）。 */
#define SPD_GO          200     /* 直行速度 */
#define SPD_TURN        450     /* 转弯速度（左右对称，两边一样快）
                                 * 500 改成 450：1520+500=2020 微秒，超了 2000 上限 */
#define SPD_BACK        400     /* 后退速度 */

/* 微调的【差速量】：两轮都【前进】，一快一慢，走出缓弧 —— 不是原地转。
左微调 = Drive(SPD_GO - SPD_TRIM, SPD_GO + SPD_TRIM)
右微调 = Drive(SPD_GO + SPD_TRIM, SPD_GO - SPD_TRIM)
 * 150 让速度变成 350 / 50，脉宽 1870 / 1470，两头都在安全区内。 */
#define SPD_TRIM        150     /* 微调差速量 */

/* 控制周期：每个舵机帧的时长，单位毫秒。20ms 即 50Hz。 */
#define FRAME_MS        20

/* 转弯步数：越大修正越猛。赛道弯急就加大，车画龙就减小。 */
#define STEP_TURN       4       /* 脱线时转弯的步数 */
#define STEP_TRIM       2       /* 微调步数：小幅修正少走几步，避免画龙 */
#define STEP_BACK       2       /* 丢线后退的步数 */
#define STEP_SEARCH     4       /* 丢线后找线转弯的步数 */

#define FALLBACK_DIR    (+1)    

#define USE_LINE_MEMORY 1       /* 1 = 丢线时朝"最后看到线的方向"找（推荐）
                                 * 0 = 丢线时永远朝 FALLBACK_DIR 找
                                 *     （想复现原始程序的固定方向策略就改成 0） */

/* 标定模式：0 = 正常运行，1 = 标定舵机中值，2 = 观测探头极性 */
#define CALIB_MODE      0

/* 标定模式 1 的扫描参数（只有 CALIB_MODE == 1 时才用到）
 *   CAL_HOLD_MS 看不清就调大（比如 2500），单位毫秒             */
#define CAL_MIN         1480    /* 扫描起点（微秒）—— 精扫（第 4 遍） */
#define CAL_MAX         1550    /* 扫描终点（微秒）—— 精扫（第 4 遍） */
#define CAL_STEP        2      /* 每步多少微秒（精扫：2） */
#define CAL_HOLD_MS     1500    /* 每个值停多久（毫秒） */
#define CAL_MID         ((CAL_MIN + CAL_MAX) / 2)
#define CAL_MARK        6       /* 中值附近 ±多少微秒内打 "<== look here" */

/*          P0_7 = QTIL    最左   <- 机器人【左】侧
 *          P0_6 = QTIM1   左中
 *          P0_5 = QTIM2   右中
 *          P0_4 = QTIR    最右   <- 机器人【右】侧
 *QTI探测到【黑线】输出高电平(1)，白色底板输出低电平(0)*/

int QTI_LL(void) { return (P0 & 0x80) ? 1 : 0; }   /* P0_7  最左 */
int QTI_L (void) { return (P0 & 0x40) ? 1 : 0; }   /* P0_6  左中 */
int QTI_R (void) { return (P0 & 0x20) ? 1 : 0; }   /* P0_5  右中 */
int QTI_RR(void) { return (P0 & 0x10) ? 1 : 0; }   /* P0_4  最右 */

/*舵机驱动*/

/*  输出一帧舵机信号：
 *      P1_1（左轮）输出高电平 LeftP  微秒
 *      P1_0（右轮）输出高电平 RightP 微秒
 *      最后用 FRAME_MS 把整帧补满，形成约 50Hz 的刷新率
 *
 *  注意：两个通道是【串行】输出的（先左后右）。
 *        整帧时长 = LeftP + RightP + FRAME_MS*1000，约 22ms。   */
void MoveAStep(int LeftP, int RightP)
{
    P1_1 = 1;   delay_nus(LeftP);    P1_1 = 0;      /* 左轮 P1_1 */
    P1_0 = 1;   delay_nus(RightP);   P1_0 = 0;      /* 右轮 P1_0 */
    delay_nms(FRAME_MS);
}

/*  按"速度"驱动：
 *      left_spd / right_spd  正数前进，负数后退，单位是偏离中值的微秒数
 *  车体上左右两个舵机是【镜像安装】的（朝向相反），右轮要取反：
 *      左轮脉宽 = SERVO_C + left_spd
 *      右轮脉宽 = SERVO_C - right_spd
          Drive(200, 200)  → MoveAStep(1600, 1200)  两轮同速 → 直行
 *        Drive(-500, 500) → MoveAStep( 900,  900)  左倒右正 → 左原地转
 *        Drive( 500,-500) → MoveAStep(1900, 1900)  左正右倒 → 右原地转 */
void Drive(int left_spd, int right_spd, int steps)
{
    int i;

    for (i = 0; i < steps; i++)
        MoveAStep(SERVO_C + left_spd, SERVO_C - right_spd);
}

/*基本动作 */

void Go(int steps)          { Drive( SPD_GO,       SPD_GO,      steps); }
void Backward(int steps)    { Drive(-SPD_BACK,    -SPD_BACK,    steps); }
void LeftTurn(int steps)    { Drive(-SPD_TURN,     SPD_TURN,    steps); }
void RightTurn(int steps)   { Drive( SPD_TURN,    -SPD_TURN,    steps); }
/* 微调：差速弧线 —— 两轮同向，只是一个快一个慢，车头缓缓转向 */
void TrimLeft(int steps)    { Drive(SPD_GO - SPD_TRIM, SPD_GO + SPD_TRIM, steps); }
void TrimRight(int steps)   { Drive(SPD_GO + SPD_TRIM, SPD_GO - SPD_TRIM, steps); }


void main(void)
{
#if CALIB_MODE == 1

    /*自动扫描，找舵机中值 SERVO_C，上电后自动在 CAL_MIN..CAL_MAX 之间来回扫，每个值保持CAL_HOLD_MS 毫秒，并把当前值打印到串口（9600）。*/
    {
        unsigned int  v;
        unsigned int  i;
        unsigned int  frames;
        unsigned char down;

        uart_Init();
        printf("\r\n");
        printf("========================================\r\n");
        printf("  SERVO_C auto sweep\r\n");
        printf("  range %u..%u step %u, hold %u ms\r\n",
               CAL_MIN, CAL_MAX, CAL_STEP, CAL_HOLD_MS);
        printf("  Note EACH wheel separately:\r\n");
        printf("    - where does the LEFT wheel reverse direction?\r\n");
        printf("    - where does the RIGHT wheel reverse direction?\r\n");
        printf("========================================\r\n\r\n");

        down = 0;
        while (1)
        {
            if (down == 0)
            {
                /* 从小往大扫 */
                for (v = CAL_MIN; v <= CAL_MAX; v = v + CAL_STEP)
                {
                    frames = (unsigned int)
                             (((unsigned long)CAL_HOLD_MS * 1000UL) /
                              (2UL * (unsigned long)v +
                               (unsigned long)FRAME_MS * 1000UL));
                    if (frames == 0) frames = 1;

                    printf("SERVO_C = %u", v);
                    if (v >= CAL_MID - CAL_MARK && v <= CAL_MID + CAL_MARK) printf("   <== look here first");
                    printf("\r\n");

                    for (i = 0; i < frames; i++)
                    {
                        P1_1 = 1;   delay_nus(v);   P1_1 = 0;
                        P1_0 = 1;   delay_nus(v);   P1_0 = 0;
                        delay_nms(FRAME_MS);
                    }
                }
            }
            else
            {
                /* 从大往小扫 */
                for (v = CAL_MAX; ; v = v - CAL_STEP)
                {
                    frames = (unsigned int)
                             (((unsigned long)CAL_HOLD_MS * 1000UL) /
                              (2UL * (unsigned long)v +
                               (unsigned long)FRAME_MS * 1000UL));
                    if (frames == 0) frames = 1;

                    printf("SERVO_C = %u", v);
                    if (v >= CAL_MID - CAL_MARK && v <= CAL_MID + CAL_MARK) printf("   <== look here first");
                    printf("\r\n");

                    for (i = 0; i < frames; i++)
                    {
                        P1_1 = 1;   delay_nus(v);   P1_1 = 0;
                        P1_0 = 1;   delay_nus(v);   P1_0 = 0;
                        delay_nms(FRAME_MS);
                    }

                    if (v <= CAL_MIN + CAL_STEP) break;
                }
            }

            down = !down;
            printf("\r\n---- turn around ----\r\n\r\n");
        }
    }


#elif CALIB_MODE == 2

    /*标定模式 2：四路探头
     第 1 个（最左）压黑 -> L  变 1，松开变 0
     第 2 个（左中）压黑 -> M1 变 1
     第 3 个（右中）压黑 -> M2 变 1
     第 4 个（最右）压黑 -> R  变 1*/
    uart_Init();
    printf("\r\nQTI 4-way check\r\n");
    printf("press each sensor, ROBOT-LEFT first\r\n");
    printf("expect: L  M1 M2 R  =  P07 P06 P05 P04\r\n\r\n");
    while (1)
    {
        printf("L=%d  M1=%d  M2=%d  R=%d\r\n",
               QTI_LL(), QTI_L(), QTI_R(), QTI_RR());
        delay_nms(200);
    }

#else

    /*循线*/
    int last_dir = FALLBACK_DIR;    /* 最后看到线的方向：-1=左, +1=右 */
    int L, M1, M2, R;               /* 四个探头：最左 / 左中 / 右中 / 最右 */
    int cnt, err;

    uart_Init();
    printf("Program Running!\n");   /* 上电只打印这一次，确认芯片已启动 */

    while (1)
    {
        /* ---- 1. 读四个探头 ---- */
        L  = QTI_LL();      /* P0_7 最左 */
        M1 = QTI_L();       /* P0_6 左中 */
        M2 = QTI_R();       /* P0_5 右中 */
        R  = QTI_RR();      /* P0_4 最右 */
        cnt = L + M1 + M2 + R;

        /* ---- 2. 四个都没压线 -> 丢线 ---- */
        if (cnt == 0)
        {
            Backward(STEP_BACK);
#if USE_LINE_MEMORY
            /* 朝"最后看到线的方向"找 */
            if (last_dir >= 0) RightTurn(STEP_SEARCH);
            else               LeftTurn(STEP_SEARCH);
#else
            /* 固定方向找 —— 保留原始程序的策略，用于对照实验 */
#if FALLBACK_DIR >= 0
            RightTurn(STEP_SEARCH);
#else
            LeftTurn(STEP_SEARCH);
#endif
#endif
            continue;
        }

        /* ---- 3. 位置误差：正 = 线偏右（车偏左）-> 要右转 ---- */
        err = 0;
        if (L)  err -= 3;       /* 最左压线：线在很左边 */
        if (M1) err -= 1;       /* 左中压线：线略偏左   */
        if (M2) err += 1;       /* 右中压线：线略偏右   */
        if (R)  err += 3;       /* 最右压线：线在很右边 */

        if (err == 0)
        {
            Go(1);                      /* 正中骑线 -> 直行 1 步 */
        }
        else if (err > 0)
        {
            last_dir = +1;              /* 记住：线在右边 */
            if (err >= 3) RightTurn(STEP_TURN);   /* 偏得多 -> 大修正 */
            else          TrimRight(STEP_TRIM);   /* 偏一点 -> 温和微调 */
        }
        else
        {
            last_dir = -1;              /* 记住：线在左边 */
            if (err <= -3) LeftTurn(STEP_TURN);
            else           TrimLeft(STEP_TRIM);
        }
    }

#endif
}
