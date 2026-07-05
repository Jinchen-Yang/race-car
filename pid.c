#include "pid.h"

/**
 * @brief 浮点值限幅（内部工具函数）
 *
 * 把 v 夹紧到 [lo, hi] 范围内。用于：
 *   - pid_update:  积分项限幅(防饱和) + 输出限幅(防执行器越界)
 *   - pid_inc_update: 输出增量限幅(防 Δu 突变)
 *
 * @param v  待限幅的原始值
 * @param lo 下限(如 out_min = -TRACK_OUT_LIM)
 * @param hi 上限(如 out_max = +TRACK_OUT_LIM)
 * @return 夹紧后的值: v<lo 返回 lo, v>hi 返回 hi, 否则返回 v
 */
static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/**
 * @brief 初始化 PID 控制器（设定增益与限幅，清零历史状态）
 *
 * 在小车上电后由 app_init() 调用。填入真车整定后的增益值(Kp/Ki/Kd)与限幅值，
 * 并把 integ 和 prev_err 清零，使控制器从"干净"状态起步。
 *
 * @param p          PID 控制器指针
 * @param kp         比例增益(两种模式通用)
 * @param ki         积分增益(位置式用); 增量式传 0
 * @param kd         微分增益(两种模式通用)
 * @param out_min    输出下限(外环=转向量下限, 内环=占空增量下限)
 * @param out_max    输出上限(外环=转向量上限, 内环=占空增量上限)
 * @param integ_max  积分限幅(仅位置式用; 增量式传 0)
 */
void pid_init(pid_t *p, float kp, float ki, float kd,
              float out_min, float out_max, float integ_max) {
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->out_min = out_min; p->out_max = out_max; p->integ_max = integ_max;
    p->integ = 0.0f; p->prev_err = 0.0f;
}

/**
 * @brief 复位 PID 控制器（清积分项与历史误差，不动增益/限幅参数）
 *
 * 小车状态切换时调用(如 IDLE→RUN, ESTOP 解除)，防止上次运行的积分/误差残留
 * 导致启动瞬间输出突变(车猛冲)。只清 integ 和 prev_err，增益/限幅保持不变。
 *
 * @param p PID 控制器指针
 */
void pid_reset(pid_t *p) { p->integ = 0.0f; p->prev_err = 0.0f; }

/**
 * @brief 位置式 PID 一步计算（循迹外环：灰度偏差 → 转向量）
 *
 * 公式: output = Kp*e + Ki*Σe + Kd*Δe
 *   e = target - meas (目标中线 0 减去灰度偏差)
 *   Σe = integ(历史误差累加，受 integ_max 限幅)
 *   Δe = e - prev_err(误差变化率)
 *
 * 在小车系统中的数据流:
 *   灰度传感器 → gray_get_error() → 本函数(target=0, meas=偏差)
 *   → 输出转向量 → 叠加到 g_vel_tgt_l/r → 速度内环 → 电机
 *
 * @param p      循迹外环 PID 控制器指针(&g_pid_track)
 * @param target 目标值(循迹时固定为 0，即"对准中线")
 * @param meas   实测值(gray_get_error() 的中线偏差)
 * @return 限幅后的转向量(正值=向右修正, 负值=向左修正)
 */
float pid_update(pid_t *p, float target, float meas) {
    float err = target - meas;
    p->integ = clampf(p->integ + err, -p->integ_max, p->integ_max);  /* 积分限幅(防饱和) */
    float d = err - p->prev_err;
    float out = p->kp * err + p->ki * p->integ + p->kd * d;
    p->prev_err = err;
    return clampf(out, p->out_min, p->out_max);                      /* 输出限幅(防越界) */
}

/**
 * @brief 增量式 PID 一步计算（速度内环：编码器速度 → PWM 占空增量）
 *
 * 公式: Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
 *   字段复用: prev_err = e(k-1), integ = e(k-2)(非积分累加)
 *
 * 在小车系统中的数据流:
 *   编码器 → enc_get_delta() * 标定系数 → 本函数(target=目标速度, meas=实测速度)
 *   → 输出 Δu → g_vel_out_l/r += Δu → 限幅 → motor_set(ch, duty)
 *
 * 与位置式的区别: 输出是"增量"(加多少/减多少)，不是绝对值。
 *   执行器到顶时 Δu 自然趋零，不会像位置式那样积分无限增长(天然抗饱和)。
 *
 * @param p      速度内环 PID 控制器指针(&g_pid_vel_l 或 &g_pid_vel_r)
 * @param target 目标速度(由循迹外环写入 g_vel_tgt_l/r)
 * @param meas   实测速度(编码器换算后的速度当量)
 * @return 本次占空增量 Δu(正值=加速, 负值=减速)
 */
float pid_inc_update(pid_t *p, float target, float meas) {
    float err = target - meas;
    float du = p->kp * (err - p->prev_err)
             + p->ki * err
             + p->kd * (err - 2.0f * p->prev_err + p->integ);
    p->integ = p->prev_err;   /* e(k-2) <- e(k-1) (轮转历史误差) */
    p->prev_err = err;        /* e(k-1) <- e(k)   (存本次误差供下次用) */
    return clampf(du, p->out_min, p->out_max);
}