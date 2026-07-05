/* 可移植 PID（与芯片无关，可主机编译/单测）—— firmware-scaffold 中间件层
 *
 * 【本模块在小车系统中的角色】
 *   本模块是"控制算法引擎"，被 app 层的两个闭环调用：
 *
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │ 1. 循迹外环（app.c 的 track_loop_step，位置式 pid_update）     │
 *   │    输入: 灰度传感器的中线偏差 gray_get_error()                  │
 *   │    输出: "转向量" → 叠加到左右轮目标速度 g_vel_tgt_l/r          │
 *   │    作用: 车身偏左时加大右轮速度、减小左轮速度，把车拉回中线       │
 *   │    关联: gray.h(偏差来源), app.c(外环调用方, 写 g_vel_tgt_l/r)  │
 *   │                                                              │
 *   │ 2. 速度内环（app.c 的 vel_loop_step，增量式 pid_inc_update）   │
 *   │    输入: 编码器实测速度 enc_get_delta() * 标定系数               │
 *   │    输出: PWM 占空增量 Δu → 累加后写入 motor_set()              │
 *   │    作用: 让左右轮各自跑到目标速度，抹平电机/负载差异             │
 *   │    关联: encoder.h(速度来源), motor.h(执行器), app.c(目标速度)  │
 *   └──────────────────────────────────────────────────────────────┘
 *
 *   ⚠ 所有增益 kp/ki/kd 与限幅都是占位，必须在真车整定
 *     （见 design/test_plan.md §B/§C）。
 */
#ifndef PID_H
#define PID_H

/**
 * @brief PID 控制器结构体（位置式 + 增量式共用同一个结构体定义）
 *
 * 本结构体在小车中被实例化 3 次（见 app.c）：
 *   - g_pid_track: 循迹外环（位置式），灰度偏差 → 转向量
 *   - g_pid_vel_l: 左轮速度内环（增量式），编码器速度 → 占空增量
 *   - g_pid_vel_r: 右轮速度内环（增量式），编码器速度 → 占空增量
 *
 * 增量式模式下字段复用关系（与位置式含义不同，见各字段说明）：
 *   integ    ← 复用为 e(k-2)，即"上上次误差"，不是积分累加
 *   prev_err ← 复用为 e(k-1)，即"上次误差"
 */
typedef struct {

    /**
     * @brief 比例增益 Kp —— 误差的"即时放大系数"
     *
     * 物理含义：误差每大 1 个单位，输出就多修正 Kp 倍。
     *   - 外环(循迹): Kp 越大，车身偏离中线时转向越猛。太大会左右摆动(振荡)，太小车反应迟钝压不住线。
     *   - 内环(速度): Kp 越大，电机转速偏离目标时加/减速越猛。太大会嗡嗡响(速度振荡)，太小起步慢。
     *
     * 关联: 与 kp/kd 配合决定系统响应速度与稳定性；整定时先调 kp 到临界振荡再退。
     */
    float kp;

    /**
     * @brief 积分增益 Ki —— 消除"稳态误差"的累加补偿
     *
     * 物理含义：误差持续存在时，Ki 会把历史误差不断累加进来，逐渐加大修正力度。
     *   - 外环(循迹): 如果小车一直偏左(比如左轮阻力大)，Ki 会积累一个额外的右转补偿，直到车回到中线。
     *   - 内环(速度): 如果电机一直跑不到目标速度(比如电池电压低)，Ki 会逐渐加大占空比补上差额。
     *
     * 关联: 与 integ_max(积分限幅) 配合防"积分饱和"(误差太大时积分项无限增长导致超调)。
     *
     * ⚠ 增量式模式下：本字段被复用为 e(k-2)（上上次误差），Ki 的实际含义由 pid_inc_update
     *   公式重新定义，不再是传统积分增益。初始化时传 ki=0 即可。
     */
    float ki;

    /**
     * @brief 微分增益 Kd —— 误差变化趋势的"阻尼"
     *
     * 物理含义：Kd 根据"本次误差与上次误差的差"来提前制动，抑制误差快速变化。
     *   - 外环(循迹): 车身快速偏离中线时(如过弯)，Kd 会提前加大修正，防止冲过头再反打(减少过冲/摆动)。
     *   - 内环(速度): 电机转速突变时(如起步/制动)，Kd 会缓冲加减速，让过渡更平滑。
     *
     * 关联: 与 prev_err(e(k-1)) 配合计算误差变化率 Δe = e(k) - e(k-1)。
     *       Kd 太大会放大噪声(编码器抖动/灰度毛刺)，太小则阻尼不足、车会甩尾。
     */
    float kd;

    /**
     * @brief 积分累加项 / 上上次误差(字段复用，含义取决于模式)
     *
     * 【位置式模式(循迹外环 g_pid_track)】
     *   含义: 历史误差的累加和 Σe(i)，反映"过去一段时间车一直偏了多少"。
     *   作用: 与 Ki 相乘后加入输出，消除稳态误差(如单侧轮阻力大导致的持续偏移)。
     *   关联: 受 integ_max 限幅(防积分饱和)；pid_reset() 时清零。
     *
     * 【增量式模式(速度内环 g_pid_vel_l/r)】
     *   含义: 被复用为 e(k-2)，即"上上次的误差"，用于计算二阶差分(微分项)。
     *   作用: Δu 公式中 kd*(e(k)-2*e(k-1)+e(k-2)) 的一部分，平滑速度变化。
     *   关联: 与 prev_err(e(k-1)) 配合；pid_inc_update 内部自动更新: integ ← prev_err。
     *   ⚠ 此模式下本字段不是积分累加，无积分饱和问题。
     */
    float integ;

    /**
     * @brief 上一次误差 e(k-1) —— 两次采样之间的误差差值基准
     *
     * 物理含义: 上一个控制周期结束时的误差值，用于计算"误差变化量"。
     *   - 位置式: Δe = e(k) - e(k-1)，乘以 Kd 得到微分项(阻尼)。
     *   - 增量式: 同上，同时 integ 字段被更新为本值(e(k-2) ← e(k-1))。
     *
     * 关联: 与 kp/kd 配合；pid_reset() 时清零(换状态时必须清，否则历史误差残留导致突变)。
     *       在 app.c 中，从 IDLE 进 RUN 前会 pid_reset() 清掉 prev_err，防止上次停车的残留。
     */
    float prev_err;

    /**
     * @brief 输出下限 out_min —— 控制量的最小边界
     *
     * 物理含义: PID 计算出的输出值不会低于此限(夹紧)。
     *   - 外环(循迹): 设为 -TRACK_OUT_LIM(如 -400)，限制最大左转量，防止转向过猛打滑。
     *   - 内环(速度): 设为 -VEL_OUT_MAX(如 -1000)，对应电机最大反向占空比。
     *
     * 关联: 与 out_max 对称使用；motor_set() 的 duty 范围 -1000..1000 与此对应。
     *       超出范围的输出被 clampf(out_min, out_max) 截断，不会让电机/转向失控。
     */
    float out_min;

    /**
     * @brief 输出上限 out_max —— 控制量的最大边界
     *
     * 物理含义: PID 计算出的输出值不会高于此限(夹紧)。
     *   - 外环(循迹): 设为 +TRACK_OUT_LIM(如 +400)，限制最大右转量。
     *   - 内环(速度): 设为 +VEL_OUT_MAX(如 +1000)，对应电机最大正向占空比。
     *
     * 关联: motor.c 的 MOTOR_MAX_DUTY(600) 是硬件层限幅(60%防超压)，此处是算法层限幅，
     *       两层独立保护。整定时一般让算法限幅 >= 硬件限幅，实际出力由硬件层兜底。
     */
    float out_max;

    /**
     * @brief 积分限幅 integ_max —— 防止积分饱和(anti-windup)的上限
     *
     * 物理含义: 积分累加项 integ 的绝对值不会超过此值。
     *   当执行器已经饱和(如电机占空比到顶了还追不上目标速度)，误差会持续存在，
     *   如果不限制积分项，integ 会无限增长；等误差反向时，要花很长时间"退积分"，
     *   导致巨大的超调(车猛冲回来)。限幅后积分项最多到 integ_max，超调可控。
     *
     * 关联: 仅位置式模式使用(循迹外环 g_pid_track)；增量式模式无积分项，传 0 即可。
     *       典型值: VEL_OUT_MAX 的 50%~100%(如 500)，需真车整定。
     */
    float integ_max;

} pid_t;

/**
 * @brief 初始化 PID 控制器（设定增益与限幅，清零历史状态）
 *
 * 在小车启动阶段由 app_init() 调用 3 次，分别建 3 个控制器：
 *   pid_init(&g_pid_track, TRACK_KP, 0, TRACK_KD, -TRACK_OUT_LIM, +TRACK_OUT_LIM, 0);  // 循迹外环
 *   pid_init(&g_pid_vel_l, VEL_KP, VEL_KI, VEL_KD, VEL_OUT_MIN, VEL_OUT_MAX, VEL_INTEG_MAX); // 左轮内环
 *   pid_init(&g_pid_vel_r, VEL_KP, VEL_KI, VEL_KD, VEL_OUT_MIN, VEL_OUT_MAX, VEL_INTEG_MAX); // 右轮内环
 *
 * @param p          PID 控制器指针(不可为 NULL)
 * @param kp         比例增益 Kp(两种模式通用)
 * @param ki         积分增益 Ki(位置式用); 增量式传 0
 * @param kd         微分增益 Kd(两种模式通用)
 * @param out_min    输出下限(两种模式通用; 外环为转向量下限, 内环为占空增量下限)
 * @param out_max    输出上限(两种模式通用; 外环为转向量上限, 内环为占空增量上限)
 * @param integ_max  积分限幅(仅位置式用; 增量式传 0)
 */
void  pid_init(pid_t *p, float kp, float ki, float kd,
               float out_min, float out_max, float integ_max);

/**
 * @brief 复位 PID 控制器（清积分项 integ 与历史误差 prev_err，不动增益/限幅参数）
 *
 * 在小车状态切换时必须调用：
 *   - IDLE → RUN: 清掉上次停车残留的积分/误差，否则启动瞬间会突变
 *   - ESTOP 解除: 清掉急停期间积累的误差
 *
 * @param p PID 控制器指针(不可为 NULL)
 */
void  pid_reset(pid_t *p);

/**
 * @brief 位置式 PID 一步计算（循迹外环用：灰度偏差 → 转向量）
 *
 * 在 app.c 的 track_loop_step() 中每 20ms 调用一次：
 *   output = Kp*e + Ki*Σe + Kd*Δe
 *   其中 e = target - meas = 0(目标中线) - gray_get_error()(实际偏差)
 *   输出的"转向量"被叠加到左右轮目标速度 g_vel_tgt_l/r，再喂给速度内环。
 *
 * @param p      循迹外环 PID 控制器指针(&g_pid_track)
 * @param target 目标值(循迹时固定为 0，即"对准中线")
 * @param meas   实测值(gray_get_error() 返回的中线偏差，左偏为负/右偏为正)
 * @return 限幅后的转向量(正值=向右修正, 负值=向左修正, 范围 [-TRACK_OUT_LIM, +TRACK_OUT_LIM])
 */
float pid_update(pid_t *p, float target, float meas);

/**
 * @brief 增量式 PID 一步计算（速度内环用：编码器速度 → PWM 占空增量）
 *
 * 在 app.c 的 vel_loop_step() 中每 10ms 调用一次(左右轮各调一次)：
 *   Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
 *   调用方把 Δu 累加到上一次输出 g_vel_out_l/r，再限幅后写入 motor_set()。
 *
 * 与位置式的区别：
 *   - 输出是"增量"而非绝对值，天然抗积分饱和(执行器到顶时 Δu 自然趋零)
 *   - 字段复用: prev_err=e(k-1), integ=e(k-2)，无独立积分累加
 *   - 更适合电机控制: PWM 占空比是连续累加的，增量式直接给"加多少/减多少"
 *
 * @param p      速度内环 PID 控制器指针(&g_pid_vel_l 或 &g_pid_vel_r)
 * @param target 目标速度(由循迹外环 track_loop_step 写入 g_vel_tgt_l/r，速度当量非占空)
 * @param meas   实测速度(enc_get_delta(ch) * ENC_CNT_TO_SPEED / dt，编码器换算后的速度当量)
 * @return 本次占空增量 Δu(正值=加速, 负值=减速, 范围 [VEL_OUT_MIN, VEL_OUT_MAX])
 */
float pid_inc_update(pid_t *p, float target, float meas);

#endif /* PID_H */