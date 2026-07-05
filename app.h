/* app —— 顶层应用骨架(控制环 + 状态机)  [firmware-scaffold APP 层]
 *
 * 【模块作用】
 *   把各底层驱动(motor/enc/gray/imu/servo/hmi/k230) + 中间件(pid) 编排成整车行为:
 *     1) 速度内环  vel_loop_step(): 编码器实测速度 -> 增量式 PID -> motor_set, 让左右轮
 *        各自跑到"目标速度", 抹平左右电机/负载差异(整车直行/差速的执行基础)。
 *     2) 循迹外环  track_loop_step(): 灰度中线偏差 -> 位置式 PID 出"转向量" -> 叠加成
 *        左右目标速度 -> 喂给内环。丢线(虚线/弧段)时切 IMU 航向锁定盲走(占位)。
 *     3) 状态机    app_fsm_step(): IDLE/RUN/AIM/STOP/ESTOP 之间切换, 串起"启动->巡迹->
 *        到点声光停车->瞄准->急停"的完整作业流程。
 *     4) 瞄准      aim_geometry(): 由里程/航向几何反解云台 pan/tilt 指向(无视觉主路径),
 *        若 K230 found 再做末端像素精修。
 *
 *   ⚠ 本文件只搭"结构与接口", 所有控制逻辑/增益/阈值/几何公式全部留占位(标 待整定/TODO),
 *     必须真车整定, 不含任何真实标定值。
 *
 * 【依赖的其它驱动 / 中间件】(对外函数, 学长学姐照此名建驱动后即可编译)
 *   motor.c   : motor_set(ch,duty) / motor_stop_all()           [已实测]
 *   pid.c     : pid_init / pid_reset / pid_update / pid_inc_update [中间件, 复用 middleware/pid]
 *   encoder.c : enc_get_delta(ch) (ch=ENC_LEFT/ENC_RIGHT)       [编码器测速, 左=QEI 右=GPIO中断]
 *   gray.c    : gray_get_error() + gray_is_lost()               [感为8路灰度, 加权质心]
 *   mpu6050.c : imu_get_yaw()                                   [MPU6050 航向, 丢线盲走用]
 *   servo.c   : servo_set_pan(deg) / servo_set_tilt(deg) / servo_rail_power(en)  [云台]
 *   hmi.c     : beep_on/beep_off / led_run_set(on) / led_err_set(on) [声光: 蜂鸣器 + RUN/ERR 灯]
 *   k230.c    : k230_get_aim(&dx,&dy) -> found                  [UART2 收 aim_offset 帧]
 *
 * 【调度约定】vel_loop_step / track_loop_step / app_fsm_step 各放调度任务表一行
 *            (周期见各函数 @note), 由 scheduler 到点调用; 不在本文件起任务表。
 */
#ifndef APP_H
#define APP_H

#include <stdint.h>

/* ===== 控制环周期(ms) —— 与调度任务表登记的 period_ms 必须一致 ===== */
#define APP_VEL_DT_MS    10   /**< 速度内环周期: 10ms (100Hz), 编码器测速窗口 */
#define APP_TRACK_DT_MS  20   /**< 循迹外环周期: 20ms (50Hz), 外环慢于内环 */
#define APP_FSM_DT_MS    10   /**< 状态机轮询周期: 10ms, 用于内部计时/超时累加 */

/* ===== 整车作业状态机 ===== */
typedef enum {
    APP_ST_IDLE = 0,   /**< 待机: 执行器全停, 等按键/上位机启动 */
    APP_ST_RUN,        /**< 行驶: 循迹外环 + 速度内环闭环跑线 */
    APP_ST_AIM,        /**< 瞄准: 整车停, 云台几何指向 + K230 末端精修 */
    APP_ST_STOP,       /**< 正常停车: 到点/完赛, 电机停 + 到点声光 */
    APP_ST_ESTOP,      /**< 急停安全态: 切电机轨 + 断舵机轨, 须清故障才离开 */
} app_state_e;

/* ---- 生命周期 ---- */

/**
 * @brief 初始化应用层(main 在所有外设 init 之后调用一次)
 * @note  建三个 PID(外环 + 左右内环), 复位计时, 状态置 APP_ST_IDLE。
 *        ⚠ 此处填的 Kp/Ki/Kd/限幅全是占位, 必须真车整定后回填。
 */
void app_init(void);

/* ---- 控制环(各自登记到调度任务表) ---- */

/**
 * @brief 速度内环一步: 编码器实测速度 -> 增量式 PID -> motor_set(左右各一路)
 * @note  周期 = APP_VEL_DT_MS。流程: enc_get_delta 取增量 -> 换算实测速度 ->
 *        pid_inc_update(目标速度, 实测速度) 出占空增量 -> 累加限幅 -> motor_set。
 *        目标速度由 track_loop_step / 状态机写入 g_vel_tgt_l/r。
 *        ⚠ 增益与"脉冲->速度"标定系数留占位, 待整定。
 */
void vel_loop_step(void);

/**
 * @brief 循迹外环一步: 灰度偏差 -> 位置式 PID 出转向量 -> 叠加成左右目标速度 -> 喂内环
 * @note  周期 = APP_TRACK_DT_MS。不压线(lost=0)时走灰度闭环; 丢线(lost=1)时切
 *        IMU 航向锁定盲走(占位)。本函数只更新 g_vel_tgt_l/r, 真正出力在 vel_loop_step。
 *        ⚠ 外环 Kp/Kd、基速、左右符号补偿留占位, 待整定。
 */
void track_loop_step(void);

/**
 * @brief 状态机一步: 在 IDLE/RUN/AIM/STOP/ESTOP 间推进作业流程
 * @note  周期 = APP_FSM_DT_MS。骨架只搭状态迁移框架, 关键点判定/启动条件/
 *        声光时序均留 TODO。RUN 态由外/内环负责跑线, 本函数只管"何时换态"。
 */
void app_fsm_step(void);

/* ---- 安全 / 外部事件入口 ---- */

/**
 * @brief 急停(按键/看门狗/上位机调用): 立即切电机 + 断舵机轨, 进 APP_ST_ESTOP
 * @note  可在中断里调, 只做最小动作(停电机 + 断舵机轨 + 置态), 不做耗时操作。
 *        清除故障后由 app_fsm_step 决定能否回 IDLE。
 */
void app_estop(void);

/**
 * @brief 请求启动一次作业(按键/上位机置位): IDLE 下收到则进入 RUN
 * @note  只置内部"启动请求"标志, 真正迁移在 app_fsm_step 里判, 避免中断里跑业务。
 */
void app_start_request(void);

/**
 * @brief 读取当前状态机状态(供 VOFA/调试/上位机查看)
 * @return 当前 app_state_e
 */
app_state_e app_get_state(void);

/**
 * @brief 切换运行模式(F4: 按键选模式): 1=F1一圈 2=F2定点瞄准 3=F3联动 4=四圈连跑。仅 IDLE 态有效。
 * @note  IDLE 态 RUN 灯每 2 秒闪"模式号"次作为人机反馈; START 按当前模式启动对应任务。
 */
void app_mode_cycle(void);

/** @brief 串口直设运行模式(F4 "串口设定模式"): 1..APP_MODE_MAX, 仅 IDLE 态生效 */
void app_mode_set(uint8_t mode);

/** @brief 读当前运行模式(1..4), 供 VOFA/调试显示 */
uint8_t app_get_mode(void);

/**
 * @brief 急停软清障: ESTOP 态下调用(如按键), 2 秒内累计 3 次 -> 回 IDLE(免按 RESET)
 * @note  三连按是防误触的确认动作; 清障后仍需 START 才会再动。
 */
void app_estop_ack(void);

/**
 * @brief 串口在线调参一步: which='p'(循迹KP,±0.05) / 'd'(KD,±0.25) / 'v'(基速,±25mm/s)
 *        / 'h'(盲走航向锁KP,±0.5,唯一允许负值项), dir=±1
 * @note  KP/KD 热改 g_pid_track, 下一拍生效; 基速带 [100,600] 安全夹; 航向KP夹±10。
 *        现场整定免重编译。
 */
void app_tune_step(char which, int dir);

/** @brief 读当前在线调参值(kp/kd/基速/航向kp), 供串口命令台 '?' 回显; 不需要的传 NULL */
void app_tune_get(float *kp, float *kd, int *base, float *hkp);

/**
 * @brief 读取速度内环调试快照(目标/实测速度, 单位 mm/s), 给 VOFA 波形用
 * @param tgt_l/meas_l/tgt_r/meas_r 输出指针(不需要的传 NULL)
 * @note  纯只读、无副作用, 可放任意调试任务里周期调; 整定内环 PID 就盯"目标 vs 实测"这两条线。
 */
void app_get_vel_debug(float *tgt_l, float *meas_l, float *tgt_r, float *meas_r);

/**
 * @brief 读取两轮当前下发占空(内环累加输出, -1000..1000), 给 VOFA 波形用
 * @param out_l/out_r 输出指针(不需要的传 NULL)
 * @note  纯只读、无副作用。与实测速度对照可区分"积分没干活/对象增益低/输出顶格"等情况。
 */
void app_get_vel_out(float *out_l, float *out_r);

#endif /* APP_H */
