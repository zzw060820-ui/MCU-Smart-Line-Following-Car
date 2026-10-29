/*==============================================================================
 * FIELD-1 现场实验版：单文件循线控制程序
 * 工程：KGOD.uvproj（AT89S52，晶振见 uart.h XTAL = 11.0592 MHz）
 * 说明：god.c 是本工程唯一参与编译的源文件；uart.h / BoeBot.h 只提供基础函数，
 *       控制逻辑、参数和状态全部集中在本文件。原 Ant 版与 FSM-1 版保留未动。
 *
 * 硬件接口：
 *   P1_1 -> 左轮舵机脉冲，P1_0 -> 右轮舵机脉冲（脉冲顺序沿用 Ant 原版）
 *   P0.7..P0.4 -> 四路循迹探头，从车体左侧到右侧排列，黑线 = 1
 *   串口   -> 9600 bit/s，仅用于上电提示，不参与控制
 *
 * 控制节拍（一帧）：
 *   读取一次 P0 -> ControlStep 计算动作 -> ApplyAction 输出左右脉冲 -> 延时 20 ms
 *   注意：帧长不等于精确的 20 ms，delay_nus / delay_nms 均为软件延时。
 *
 * 状态机（各状态细节见对应函数注释）：
 *   WAIT    上电或事件预算耗尽后等待，连续 2 帧捕线图案才自动启动
 *   TRACK   普通巡线；单侧/同侧双探头图案进入 TURN，全白进入 SEARCH
 *   TURN    保持同一方向转向 3..32 帧，允许确认到的捕线提前结束
 *   SEARCH  有限搜索，20/40/20 帧三段；捕线回 TRACK，耗尽回 WAIT
 *   EPISODE 从进入 TURN/SEARCH 开始计帧，160 帧仍未恢复即强制回 WAIT
 *
 * 本文件不声称以下事项，均需实车验证：
 *   停车脉宽未标定；复杂交叉点（如五角星）不能保证正确选路；
 *   所有帧数与速度均为初值；没有进行过实车验证。
 *
 * 编码：GBK（与工程内 uart.h / BoeBot.h 一致），供 Keil C51 编译。
 *============================================================================*/
#include <uart.h>
#include <BoeBot.h>

/*------------------------------------------------------------------------------
 * 舵机与速度参数
 *   DriveFrame 把 left / right 当作相对中位 SERVO_C 的带符号偏移量：
 *     左轮脉宽 = SERVO_C + left，右轮脉宽 = SERVO_C - right。
 *   因此同一个数值加到左右轮时，脉宽变化方向相反（适配两侧舵机的镜像安装）。
 *   下列数值沿用 Ant 基线，本轮未重新标定。
 *----------------------------------------------------------------------------*/
#define SERVO_C 1520       /* 舵机中位脉宽（名义 us，实际由软件延时实现） */
#define SPD_GO 200         /* 直行速度偏移 */
#define SPD_TRIM 150       /* 小幅修正偏移，与 SPD_GO 叠加或相减 */
#define SPD_TURN 450       /* 转向偏移 */
#define SPD_BACK 400       /* 后退偏移；动作表保留，当前状态机不会输出 */
#define FRAME_MS 20        /* 每帧固定延时（ms），在左右两个脉冲之后执行 */

/* WAIT 状态输出的“停车候选”脉宽：
 * 仅为历史经验值，不是已标定的零速点。等待时可能慢转，必须架空单独标定；
 * 不得通过修改 SPD_GO 来“修复”该现象。 */
#define STOP_LEFT_PULSE 1535
#define STOP_RIGHT_PULSE 1505

/*------------------------------------------------------------------------------
 * 状态机帧数参数（单位：帧；1 帧 = 1 次 ControlStep + 1 次电机输出）
 *   全部为待实车验证的初值。计数变量为 unsigned char，上限 254，
 *   下面的 #if 检查会在配置非法时直接终止编译。
 *----------------------------------------------------------------------------*/
#define TURN_MIN_FRAMES 3        /* TURN 至少保持的转向帧数，之后才允许捕线提前退出 */
#define TURN_MAX_FRAMES 32       /* TURN 最长帧数，超时进入 SEARCH */
#define CAPTURE_FRAMES 2         /* 捕线确认：连续 2 帧满足 HasCenterLine */
#define SEARCH_FIRST_FRAMES 20   /* 搜索第 1 段：朝 turn_dir 转 */
#define SEARCH_OTHER_FRAMES 40   /* 搜索第 2 段：朝反方向转 */
#define SEARCH_FINAL_FRAMES 20   /* 搜索第 3 段：再朝 turn_dir 转；三段共 80 帧 */
#define EPISODE_MAX_FRAMES 160   /* 单次事件总预算：从进入 TURN/SEARCH 起算 */
#define RECOVER_FRAMES 6         /* 捕线后低速过渡帧数（捕线帧本身算第 1 帧） */
#define RECOVER_SPEED 160        /* 低速过渡的基础速度偏移 */
#define RECOVER_TRIM 80          /* 低速过渡时的修正偏移 */

/* 编译期检查：下列计数最终存入 unsigned char，非法配置直接编译失败，
 * 避免在现场因计数溢出产生难以解释的行为。 */
#if TURN_MIN_FRAMES < 1 || TURN_MAX_FRAMES <= TURN_MIN_FRAMES || TURN_MAX_FRAMES > 254
#error Invalid turn frame limits
#endif
#if CAPTURE_FRAMES < 2 || CAPTURE_FRAMES > 254 || RECOVER_FRAMES < 1 || RECOVER_FRAMES > 254
#error Invalid capture or recovery frame count
#endif
#if SEARCH_FIRST_FRAMES < 1 || SEARCH_OTHER_FRAMES < 1 || SEARCH_FINAL_FRAMES < 1 || (SEARCH_FIRST_FRAMES+SEARCH_OTHER_FRAMES+SEARCH_FINAL_FRAMES) > 254
#error Invalid search frame limits
#endif
#if EPISODE_MAX_FRAMES < 1 || EPISODE_MAX_FRAMES > 254
#error Invalid episode frame limit
#endif

/*------------------------------------------------------------------------------
 * 状态与动作编码
 *   MODE_*   ：control.mode 的取值，主状态机见 ControlStep。
 *   ACT_*    ：ControlStep 返回、ApplyAction 解释的动作编号。
 *----------------------------------------------------------------------------*/
#define MODE_TRACK 0       /* 正常巡线 */
#define MODE_TURN 1        /* 锁定方向转向 */
#define MODE_SEARCH 2      /* 有限搜索 */
#define MODE_WAIT 3        /* 等待放车 / 自动恢复 */
#define ACT_STOP 0         /* 停车候选脉宽（未标定） */
#define ACT_GO 1           /* 正常直行 */
#define ACT_TRIM_LEFT 2    /* 正常速度小幅左修 */
#define ACT_TRIM_RIGHT 3   /* 正常速度小幅右修 */
#define ACT_LEFT 4         /* 左转（左右轮同脉宽，是否原地旋转取决于安装方向） */
#define ACT_RIGHT 5        /* 右转（同上） */
#define ACT_BACK 6         /* 后退；当前状态机不输出 */
#define ACT_SLOW_GO 7      /* 低速直行（捕线后恢复阶段） */
#define ACT_SLOW_LEFT 8    /* 低速左修 */
#define ACT_SLOW_RIGHT 9   /* 低速右修 */

/*------------------------------------------------------------------------------
 * 探头图案编码（ReadLineMask 的返回值，0..15）
 *   bit3 = P0.7 最左，bit2 = P0.6 左二，bit1 = P0.5 右二，bit0 = P0.4 最右；
 *   黑线 = 1，白底 = 0。
 *   例：0110（m=6）表示中间两路同时压线；0000（m=0）表示全白。
 * ControlStep 中的加权：最左 -3、左二 -1、右二 +1、最右 +3，
 *   err < 0 表示线偏左，err > 0 表示线偏右，err = 0 表示居中。
 * 常见图案含义：
 *   0000 全白 -> 丢线，进入 SEARCH
 *   0001 / 0011 / 1000 / 1100 单侧或同侧双探头 -> 直接进入 TURN
 *   0010 / 0100 / 0110 等含中央探头的图案 -> 可作为捕线图案
 *   0101 / 1001 / 1010 / 1011 / 1101 / 1111 等歧义或分裂图案
 *       -> 按 err 动作，但不更新方向记忆（见 RememberDirection）
 *----------------------------------------------------------------------------*/

/* 全部控制状态集中在此结构体中，不对外暴露；上电时由 ControlInit 清零。 */
struct {
    unsigned char mode, action;   /* 当前状态；最近一次输出的动作（供测试/观察） */
    signed char err, last_valid_dir, turn_dir;
        /* err：本帧加权偏差；
         * last_valid_dir：最近可靠方向（-1 左 / +1 右 / 0 未知）；
         * turn_dir：TURN/SEARCH 当前保持的方向 */
    unsigned char turn_frames, search_frames, center_count, recover_left;
        /* TURN 已用帧数；SEARCH 已用帧数；
         * 连续捕线帧数；剩余低速恢复帧数 */
    unsigned char episode_active, episode_frames;
        /* 是否处于一次事件中；事件已用帧数（受 EPISODE_MAX_FRAMES 限制） */
} control;

/* 读取一次 P0 并取出高 4 位作为探头图案；每帧只读一次（见 main）。
 * P0.7..P0.4 对应车体左到右，黑线 = 1，返回到 bit3..bit0。 */
unsigned char ReadLineMask(void)
{
    unsigned char snapshot;
    snapshot = P0; /* P0.7..P0.4 = robot left to right, black=1 */
    return (snapshot >> 4) & 15;
}

/* 输出一帧：先左轮（P1_1）后右轮（P1_0），两个脉冲结束后固定延时 FRAME_MS。
 * 脉冲顺序与延时实现沿用 Ant，未验证前不要改变顺序。 */
void MoveAStep(int LeftP, int RightP)
{
    P1_1 = 1; delay_nus(LeftP); P1_1 = 0;
    P1_0 = 1; delay_nus(RightP); P1_0 = 0;
    delay_nms(FRAME_MS);
}

/* 由带符号偏移（相对中位）生成左右脉宽；right 用减号以适配镜像安装。 */
void DriveFrame(int left, int right)
{
    MoveAStep(SERVO_C + left, SERVO_C - right);
}

/* 把动作编码转换为实际脉宽。
 * ACT_LEFT / ACT_RIGHT 会让左右轮得到相同脉宽（分别为 1070 / 1970），
 * 是否对应原地旋转取决于舵机安装方向，需实车确认。
 * 默认分支（ACT_STOP）输出独立停车候选；WAIT 慢转问题尚未解决。 */
void ApplyAction(unsigned char action)
{
    switch(action) {
    case ACT_GO: DriveFrame(SPD_GO, SPD_GO); break;
    case ACT_TRIM_LEFT: DriveFrame(SPD_GO-SPD_TRIM, SPD_GO+SPD_TRIM); break;
    case ACT_TRIM_RIGHT: DriveFrame(SPD_GO+SPD_TRIM, SPD_GO-SPD_TRIM); break;
    case ACT_LEFT: DriveFrame(-SPD_TURN, SPD_TURN); break;
    case ACT_RIGHT: DriveFrame(SPD_TURN, -SPD_TURN); break;
    case ACT_BACK: DriveFrame(-SPD_BACK, -SPD_BACK); break;
    case ACT_SLOW_GO: DriveFrame(RECOVER_SPEED, RECOVER_SPEED); break;
    case ACT_SLOW_LEFT: DriveFrame(RECOVER_SPEED-RECOVER_TRIM, RECOVER_SPEED+RECOVER_TRIM); break;
    case ACT_SLOW_RIGHT: DriveFrame(RECOVER_SPEED+RECOVER_TRIM, RECOVER_SPEED-RECOVER_TRIM); break;
    default:
        /* 历史候选值。WAIT 仍可能慢转：必须架空标定。 */
        MoveAStep(STOP_LEFT_PULSE, STOP_RIGHT_PULSE);
        break;
    }
}

/* 记录并返回本帧动作。control.action 仅用于观察（测试/串口调试），
 * 真正驱动电机的是 main 中的 ApplyAction。 */
static unsigned char Emit(unsigned char action)
{
    control.action = action;
    return action;
}

/* 捕线图案：必须是连续单段且包含至少一路中央探头（bit2/bit1），
 * 即 0010 / 0011 / 0100 / 0110 / 0111 / 1100 / 1110。
 * 7/14 含中央但两侧更宽，允许启动/捕线，但不用于可靠方向记忆。
 * 该条件较宽：交叉支路可能被误捕，几何选路未解决。 */
static unsigned char HasCenterLine(unsigned char m)
{
    return m==2 || m==3 || m==4 || m==6 || m==7 || m==12 || m==14;
}

/* 连续 CAPTURE_FRAMES 帧命中捕线图案才算确认；
 * 任一中途图案不符合即清零重计。 */
static unsigned char ConfirmCenter(unsigned char m)
{
    if(!HasCenterLine(m)) control.center_count=0;
    else if(control.center_count<CAPTURE_FRAMES) control.center_count++;
    return control.center_count>=CAPTURE_FRAMES;
}

/* 仅用可靠单侧图案更新方向记忆：
 *   0001 / 0010 / 0011 -> 右（+1）
 *   0100 / 1000 / 1100 -> 左（-1）
 * 歧义或宽线图案不更新，避免把分裂图案误判为方向。 */
static void RememberDirection(unsigned char m)
{
    if(m==1 || m==2 || m==3) control.last_valid_dir=1;
    else if(m==4 || m==8 || m==12) control.last_valid_dir=-1;
}

/* 低速帧的修正动作：沿用 err 的方向，仅在捕线后的恢复阶段使用。 */
static unsigned char SlowAction(void)
{
    if(control.err<0) return ACT_SLOW_LEFT;
    if(control.err>0) return ACT_SLOW_RIGHT;
    return ACT_SLOW_GO;
}

/* 把方向编码转换为转向动作：-1 左，+1 右（turn_dir 不会为 0）。 */
static unsigned char TurnAction(signed char dir)
{
    return dir<0 ? ACT_LEFT : ACT_RIGHT;
}

/* 开始一次事件并置第 1 帧；若已在事件中则不清零，
 * 保证 EPISODE_MAX_FRAMES 约束的是整个未完成事件。 */
static void StartEpisode(void)
{
    if(!control.episode_active) {
        control.episode_active=1;
        control.episode_frames=1;
    }
}

/* 进入 WAIT：输出停车候选并清零确认/恢复计数。
 * 上电和事件预算耗尽都走这里；之后靠连续捕线帧自动恢复，无需按复位键。 */
static unsigned char WaitForLine(void)
{
    control.mode=MODE_WAIT;
    control.center_count=0;
    control.recover_left=0;
    return Emit(ACT_STOP);
}

/* 从 WAIT/TURN/SEARCH 进入 TRACK 的统一入口：
 *   - 清零捕线确认计数；
 *   - 剩余恢复帧数设为 RECOVER_FRAMES-1，因为本帧本身就按低速帧输出；
 *   - 更新方向记忆后输出第一帧低速动作。 */
static unsigned char CaptureLine(unsigned char m)
{
    control.mode=MODE_TRACK;
    control.center_count=0;
    control.recover_left=RECOVER_FRAMES-1; /* capture output is slow frame 1 */
    RememberDirection(m);
    return Emit(SlowAction());
}

/* 进入 SEARCH 并记录搜索方向；dir = 0 表示方向未知，
 * 明确选择先去右侧探测（不使用歧义图案推断方向）。 */
static void StartSearch(signed char dir)
{
    StartEpisode();
    control.mode=MODE_SEARCH;
    control.turn_dir=dir ? dir : 1; /* unknown: explicit right-first exploration */
    control.search_frames=0;
    control.center_count=0;
    control.recover_left=0;
}

/* 有界搜索：三段计时，任一段内确认捕线即进入 TRACK；
 *   第 1 段（FIRST）      朝 turn_dir
 *   第 2 段（OTHER）      朝 -turn_dir
 *   第 3 段（FINAL）      再朝 turn_dir
 * 三段全部用完则回到 WAIT。搜索期间图案不会改变 turn_dir，
 * 因此丢线后不会因短暂全白立刻反向。 */
static unsigned char SearchStep(unsigned char m)
{
    if(control.search_frames >= SEARCH_FIRST_FRAMES+SEARCH_OTHER_FRAMES+SEARCH_FINAL_FRAMES)
        return WaitForLine();
    control.search_frames++;
    if(ConfirmCenter(m)) return CaptureLine(m);
    if(HasCenterLine(m)) return Emit(ACT_SLOW_GO);
    if(control.search_frames<=SEARCH_FIRST_FRAMES)
        return Emit(TurnAction(control.turn_dir));
    if(control.search_frames<=SEARCH_FIRST_FRAMES+SEARCH_OTHER_FRAMES)
        return Emit(TurnAction(-control.turn_dir));
    return Emit(TurnAction(control.turn_dir));
}

/* 上电初始化：先进入 WAIT，再让用户把车放到线上，
 * 避免上电后立即把搜索预算耗光。所有状态字段清零。 */
void ControlInit(void)
{
    control.mode=MODE_WAIT; /* power on first, then place on a recognized line */
    control.action=ACT_STOP;
    control.err=0;
    control.last_valid_dir=0;
    control.turn_dir=0;
    control.turn_frames=0;
    control.search_frames=0;
    control.center_count=0;
    control.recover_left=0;
    control.episode_active=0;
    control.episode_frames=0;
}

/*------------------------------------------------------------------------------
 * 每帧调用一次：输入探头图案 m（0..15），返回动作编码。
 *
 * 处理顺序有明确含义，调整时不要随意交换：
 *   1. 先计算本帧加权偏差 err（慢速帧和捕线启动都依赖它）；
 *   2. WAIT 优先判断，连续捕线才启动；
 *   3. 事件预算检查：超出 EPISODE_MAX_FRAMES 直接回 WAIT；
 *   4. 按 TURN -> SEARCH -> TRACK 的顺序分派。
 *
 * TRACK 的 15 种非零图案沿用原加权动作表：
 *   err >= 3 全速右转，err <= -3 全速左转，
 *   err = ±1/±2 小幅修正，err = 0 直行；
 *   1/3/8/12 为单侧或同侧双探头，立即进入 TURN 并锁向；
 *   0（全白）用最近可靠方向启动 SEARCH；
 *   其余歧义图案按 err 动作，但不改写方向记忆。
 * 注意：全黑 1111、三路黑等图案不会触发锁停，宽线/交叉时继续行驶。
 *----------------------------------------------------------------------------*/
unsigned char ControlStep(unsigned char m)
{
    m &= 15;
    /* 加权求和：最左 -3，左二 -1，右二 +1，最右 +3。
     * 负值表示线偏左，正值表示线偏右。 */
    control.err=0;
    if(m&8) control.err-=3;
    if(m&4) control.err-=1;
    if(m&2) control.err+=1;
    if(m&1) control.err+=3;

    /* WAIT：上电或预算耗尽后的静止等待。连续 CAPTURE_FRAMES 帧
     * 命中捕线图案才启动，并清空上一次事件的计数。 */
    if(control.mode==MODE_WAIT) {
        if(ConfirmCenter(m)) {
            control.episode_active=0;
            control.episode_frames=0;
            return CaptureLine(m);
        }
        return Emit(ACT_STOP);
    }
    /* 事件总预算：从进入 TURN/SEARCH 起累计，超限直接放弃并回 WAIT。
     * 该检查在 TURN/SEARCH 处理之前执行，保证任何分支都受同一上限约束。 */
    if(control.episode_active) {
        if(control.episode_frames>=EPISODE_MAX_FRAMES) return WaitForLine();
        control.episode_frames++;
    }

    /* TURN：保持 turn_dir 转向。
     *   - 超过最大帧数：转入 SEARCH；
     *   - 前 TURN_MIN_FRAMES 帧只转向，不响应捕线，避免刚入弯就误捕；
     *   - 之后若确认捕线：回 TRACK；若出现中央线但未确认：低速直行；
     *   - 否则继续同向转向。 */
    if(control.mode==MODE_TURN) {
        if(control.turn_frames>=TURN_MAX_FRAMES) StartSearch(control.turn_dir);
        else {
            if(control.turn_frames>=TURN_MIN_FRAMES) {
                if(ConfirmCenter(m)) return CaptureLine(m);
                if(HasCenterLine(m)) {
                    control.turn_frames++;
                    return Emit(ACT_SLOW_GO);
                }
            }
            control.turn_frames++;
            return Emit(TurnAction(control.turn_dir));
        }
    }
    /* SEARCH：有界的 20/40/20 帧搜索，细节见 SearchStep。 */
    if(control.mode==MODE_SEARCH) return SearchStep(m);

    /* TRACK：普通巡线，匹配原版的加权 16 图案动作表。 */
    /* 全白丢线：优先按最近可靠方向搜索；没有记忆时 StartSearch 内部选择右侧。 */
    if(!m) { StartSearch(control.last_valid_dir); return SearchStep(m); }
    RememberDirection(m);
    /* 捕线后的低速恢复期：倒计时，结束后清除事件计数并恢复常规速度。
     * 期间只要不中断（全白会提前进入上面的搜索分支），就按 err 低速修正。 */
    if(control.recover_left) {
        control.recover_left--;
        if(!control.recover_left) {
            control.episode_active=0;
            control.episode_frames=0;
        }
        return Emit(SlowAction());
    }
    /* 单侧或同侧双探头（0001/0011/1000/1100）：立即进入 TURN，
     * 方向由 err 决定；turn_frames 从 1 开始计数（本帧就是第 1 帧）。 */
    if(m==1 || m==3 || m==8 || m==12) {
        StartEpisode();
        control.mode=MODE_TURN;
        control.turn_dir=control.err<0 ? -1 : 1;
        control.turn_frames=1;
        control.center_count=0;
        return Emit(TurnAction(control.turn_dir));
    }
    /* 其余图案按加权偏差动作：大幅偏差全速转向，小幅偏差微调，居中直行。 */
    if(control.err>=3) return Emit(ACT_RIGHT);
    if(control.err<=-3) return Emit(ACT_LEFT);
    if(control.err>0) return Emit(ACT_TRIM_RIGHT);
    if(control.err<0) return Emit(ACT_TRIM_LEFT);
    return Emit(ACT_GO); /* 0110/1001/1111: baseline behavior, no forced stop */
}

/* 主循环：每帧读取一次探头 -> 计算动作 -> 实际输出。
 * 上电先输出串口提示，然后永久循环；无硬件看门狗或自动复位逻辑。 */
void main(void)
{
    unsigned char m, action;
    P1_1=0; P1_0=0;
    ControlInit();
    uart_Init();
    printf("FIELD-1 AUTO-RESUME; WAIT PULSES UNVERIFIED\r\n");
    while(1) {
        m=ReadLineMask();
        action=ControlStep(m);
        ApplyAction(action);
    }
}
