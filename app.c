/* app —— 顶层应用骨架(控制环 + 状态机)  [firmware-scaffold APP 层]
 *
 * 见 app.h 顶部说明。本文件只搭"结构与接口", 控制逻辑/增益/阈值/几何公式全部占位:
 *   - 凡 #define 的调参常量都标 "待整定", 给的是占位数, 不是真值;
 *   - 凡需现场标定/题目回填的算法步骤都标 TODO, 骨架里走"安全的占位行为"(多为停/中位/直行)。
 *
 * 【依赖驱动对外函数】(若某驱动尚未由学长学姐建好, 该 #include 会报缺头, 属预期:
 *   按 app.h 列出的契约名建好对应 .h/.c 即可编译。本层只调对外函数, 不碰 DL_*。)
 *   motor.h   : motor_set / motor_stop_all / MOTOR_LEFT / MOTOR_RIGHT
 *   pid.h     : pid_t / pid_init / pid_reset / pid_update / pid_inc_update
 *   encoder.h : enc_get_delta / ENC_LEFT / ENC_RIGHT
 *   gray.h    : gray_get_error / gray_is_lost
 *   mpu6050.h : imu_get_yaw
 *   servo.h   : servo_set_angle(ch,deg) / servo_rail_enable(on) / SERVO_PAN / SERVO_TILT
 *   hmi.h     : beep_on / beep_off / led_run_set / led_err_set
 *   k230.h    : k230_get_aim
 */
#include <stdint.h>
#include <math.h>       /* aim_geometry: atan2f/cosf/sinf/sqrtf, ticlang 自带 */
#include "app.h"
#include "motor.h"
#include "pid.h"
#include "encoder.h"
#include "gray.h"
#include "mpu6050.h"
#include "servo.h"
#include "hmi.h"
#include "protocol.h"
#include "k230.h"

/* empty.c 的 1ms 全局时基(SysTick 里自增)。测速用真实经过时间而非名义周期:
 * track 环的 I2C 读失败超时会拖延调度, 若按名义 10ms 算速度会虚高 2~3 倍(2026-07-04 实测坐实)。 */
extern volatile uint32_t g_tick_ms;

/* ============================================================================
 *  占位调参常量 —— 全部 #define + "待整定", 不填真值
 * ==========================================================================*/

/* ★台架诊断开关(诊断完必须改回 0!): 1 = 速度环旁路——RUN 态两轮恒占空开环, 只测速不闭环。
 * 用途: 把"电机方向 / 编码器符号 / 测速换算 / 电气噪声"与 PID 动力学拆开定位。
 * 判读: 恒占空下轮子应匀速前进、I1/I2 应稳定上涨、I5/I7 应为稳定正值;
 *       哪一条不满足, 问题就在被控对象/传感器那一侧, 与 PID 无关。(2026-07-04 排障用) */
#define VEL_OPEN_LOOP_TEST   0
#define VEL_OPEN_LOOP_DUTY   200   /* 恒定占空(-1000..1000), 与六月电机开环实测同款值 */

/* --- 速度内环(增量式 PID, 每轮一个) ---
 * 2026-07-04 开环标定: 占空200 -> 左≈400/右≈380 mm/s => 对象增益 G≈2.0 mm/s每占空单位;
 * 跑 300mm/s 稳态占空≈150。增益按"回路增益 Kp*G≈0.3"设计, 积分时间常数≈170ms。
 * (历史: Kp=1/Ki=0 剧烈振荡; Kp=0.3/Ki=0.08 仍振荡——对 G=2.0 的对象回路增益过高。) */
#define VEL_KP            0.15f  /* 待整定: 空载已按实测对象增益设计; 带载后地面复整 */
#define VEL_KI            0.03f  /* 待整定: 同上; 收敛慢可加大, 振荡则减小 */
#define VEL_KD            0.0f   /* 待整定: 速度内环微分增益(占位) */
#define VEL_OUT_MIN   (-1000.0f) /* 待整定: 内环输出(占空)下限, 对应 motor duty 标度 */
#define VEL_OUT_MAX    (1000.0f) /* 待整定: 内环输出(占空)上限 */
#define VEL_INTEG_MAX   (500.0f) /* 待整定: 内环积分限幅(抗饱和) */
/* 编码器计数 -> 实测速度(mm/s)换算, 必须分轮——左右解码倍频不同(左 QEI 4倍频/右单相1倍频),
 * 每圈计数已实测并存 encoder.h(左 1017.4 / 右 265.2, 2026-07-03 十圈法)。
 * 每计数毫米数 = 轮周长(π·D) / 每圈计数; 实测速度 = delta * 每计数毫米数 / dt, 单位 mm/s。 */
#define WHEEL_DIAM_MM     47.0f  /* 2026-07-05 实测≈4.7cm; 再精一步的标法: 贴地划线推整 2.00m 读计数反推 */
#define ENC_L_MM_PER_CNT  (3.1415926f * WHEEL_DIAM_MM / ENC_L_CNT_PER_REV)  /* 左轮 mm/计数 */
#define ENC_R_MM_PER_CNT  (3.1415926f * WHEEL_DIAM_MM / ENC_R_CNT_PER_REV)  /* 右轮 mm/计数 */
#define VEL_MEAS_LPF      0.5f   /* 待整定: 测速一阶低通系数(0~1,越小越平滑),抑制编码器抖动(等效参考工程的多点滑窗) */

/* --- r25 SelfTurn 转向环(CTY 架构: 统一吃 DeltaYaw°, 出差速占空) --- */
#define TRACK_KP          3.0f   /* SelfTurn P: 占空差/°; dy=40° 时差速 120('P/p'±0.5 在线调) */
#define TRACK_KD          2.0f   /* SelfTurn D: 抑制查表台阶跳变的momentum('D/d'±1 在线调) */
#define TRACK_OUT_LIM   (400.0f) /* (遗留, 仅供闲置的 g_pid_track 初始化) */
#define BASE_SPEED        150    /* ⚠r25 语义变更: 巡航"占空"(不再是 mm/s)! ≈300mm/s.
                                  * 'V/v' ±10 在线调, 夹 [60,300] */
/* --- r25 段状态机(CTY Task4StateUpdate 移植) --- */
#define LAP_CW            1      /* 题目正式路线 A→B 先行 = 顺时针圈(两弧皆右转); 反向跑改0 */
#define TURN_DIFF_LIM     150    /* SelfTurn 差速限幅(占空) */
#define SEG_LINE_TH       0x08   /* 漏积分 ≤此 = 确认在线(≈3个有线帧, 60ms) */
#define SEG_BLANK_TH      0x10   /* 漏积分 ≥此 = 确认空白(16空白帧≈320ms) */
#define START_PROTECT_MS  1500   /* 起步保护: 强制seg0+无视灰度(A点残端免疫) */
#define TRANS_PROTECT_MS  1000   /* 段切换保护窗: 窗内禁再切+弧内丢线强拐生效期 */
#define ARC_SEARCH_DEG    50.0f  /* 弧内丢线强拐角(CTY ±50): 方向取弧内侧 */
/* 左右轮安装多为镜像, 整车直行需对其中一轮取反: 这两个符号现场标定。 */
#define WHEEL_SIGN_L     (+1)    /* 待整定: 左轮目标速度符号(占位, 可能取反) */
#define WHEEL_SIGN_R     (+1)    /* 待整定: 右轮目标速度符号(占位, 可能取反) */

/* --- 丢线盲走(IMU 航向锁定) --- */
#define HEADING_KP        2.0f   /* r14 开机默认 +2.0(脱机盲穿实验值; 符号由三证定案: 左转I12=+90
                                  * + 循迹环收敛反推 turn>0=向左 + 负反馈推导闭合)。
                                  * 串口 'H'/'h' ±0.5 在线调(允许负值); 实验失败改回 0.0f 即回退。
                                  * 历史: r5-r7 时代 ±3 两个符号都打转 —— 当时无开机漂移自检,
                                  * 摔机后 yaw 快漂, 航向锁追着漂移参考跑, 哪个符号都画圈;
                                  * r8 加自检后病根排除(2026-07-05 boot 实测 -0.65°/2s 合格)。 */
#define HEADING_LPF_A     0.05f  /* 压线期航向滑动平均系数(20ms拍): τ≈0.4s, 滤掉蛇形摆动的
                                  * 相位, 得到黑线真实走向; 丢线时锁的是这个平均值而非瞬时值 */
#define HEADING_LPF_A_FAST 0.15f /* r16 快平均(τ≈0.13s), 与慢平均配对做滞后外推:
                                  * 弧线末端出线时航向以 v/R(≈43°/s@300mm/s,R=400)恒速转,
                                  * 单个滑动平均滞后 ω·τ≈17° 且方向恒偏弯内侧(2026-07-05
                                  * 真场地 6 次全偏左实锤)。外推 lock=fast+(fast-slow)·
                                  * τf/(τs-τf)=fast+0.5(fast-slow) 恒速转时精确消滞后;
                                  * 直线段两平均相等, 外推项自动归零, 不伤原行为。 */
#define HEADING_EXTRAP_LIM 25.0f /* 外推项保险夹(°): 防快平均噪声被 1.5/-0.5 组合放大 */
#define HEADING_ONLINE_MIN_MS 600 /* r17: 压线不足此时长就丢线, 平均值没收敛不可信 -> 锁定不加外推 */
#define HEADING_TURN_LIM   100   /* r17: 盲走转向量硬限幅(mm/s)。数学上限 2×180°=360 够原地画圈;
                                  * 夹 100 后坏锁定值最多走 R≈21cm 缓弧(减灾, 非根治) */
/* (r18 的"边缘掉线补转±30°"已于 r19 移除 —— 对抗审计定案: 单帧 gray_last_error 判据
 *  零区分度(边缘逃逸掉线最后一帧必为±350, 真末端也约半数>门限), 固定30°还过补,
 *  是"大量右转圈"的直接推手。根治改为 TRACK_FF_GAIN 曲率前馈, 见下。) */
#define TRACK_ACQ_N        3     /* r19 盲走解锁去抖: 连续在线帧数达此值才算"确认重新压线"。
                                  * 审计定案("30°之谜"): 弧末线骑传感器边缘, 1~2 帧闪断频发;
                                  * 旧逻辑单帧即解锁+再丢线重锁, 把首锁(带外推的好值)秒换成
                                  * 冻结的 fast 裸值, 且 online 计时永远到不了 600ms —— 外推
                                  * 在唯一需要它的地方被自我缴械。闪断期间沿用原锁定继续走。 */
#define TRACK_FF_GAIN      4.6f  /* r19 弧上曲率前馈: turn_ff = 增益×(fast-slow)。
                                  * 推导: ω̂(°/s)=(fast-slow)/(τs-τf)=d/0.267s;
                                  * turn_ff(mm/s)=ω̂·(π/180)·(轮距/2≈70mm) ≈ 4.58×d。
                                  * 作用: P 外环维持弧上 43°/s 需稳态偏差≈131 -> 线骑传感器
                                  * 边缘(闪断/提前掉线的总根源); 前馈接管稳态转向后 err_ss→0,
                                  * 线回传感器中央, 能跟到几何末端。直线段 d≈0 自动归零。
                                  * ⚠轮距 140mm 为估值, 实测后回填(增益∝轮距)。 */
#define TRACK_FF_LIM      80.0f  /* 前馈限幅(mm/s): 防快慢平均差噪声放大 */

/* ===== r20 驱动架构切换: 编码器出局, 开环占空 + 陀螺/灰度差速 =====
 * 台架实锤(2026-07-05): 左编码器间歇无信号(测速读零→保险丝正确ESTOP), 右编码器
 * ±20% 电气毛刺(悬空空转即有) —— 速度闭环骑在两个说谎的传感器上, 是"走不直/
 * 时左时右/循迹艰难"全部怪相的第一因。本题不需要轮速环(2024H 开源方案佐证:
 * 直线=航向锁差速, 弧线=灰度差速, 定占空开环足够)。编码器仅保留测速遥测
 * (I5/I7, 持续观察病情)与右轮里程参考, 不再参与控制。
 * 修好编码器后可把 DRIVE_OPEN_LOOP 置 0 回到速度闭环, 两套路径都保留。 */
#define DRIVE_OPEN_LOOP   1
#define DUTY_PER_MMS      0.5f   /* mm/s->占空 换算(开环标定 G≈2.0mm/s每占空 的倒数)。
                                  * 出口换算的好处: 原 mm/s 量纲的全部增益/限幅/在线调参
                                  * ('V'基速/'H'航向KP/循迹KP·KD)语义不变, 免全案重整定。
                                  * ⚠电池电压跌落会让此系数变小(占空同样速度更低), 换算是
                                  * 近似; 差速转向只依赖左右对称性, 影响可容忍。 */

/* --- r20 捕线强转(借鉴参考方案"进弧强制转向", 改造成非阻塞) --- */
#define CAPTURE_ERR_TH    250    /* 确认压线时|err|超此值=斜插进线, 先原地对正再交 PID */
#define CAPTURE_DONE_TH   100    /* 原地转到|err|小于此值, 交还循迹 PID */
#define CAPTURE_PIVOT_DUTY 120   /* 原地转向占空(左右反向) 待整定 */
#define CAPTURE_PIVOT_MAX_MS 700 /* 强转超时保险: 超时放弃, 回正常流程 */

/* --- r20 事件节点(取代 KP_GATES 里程/航向门限; 参考方案 beep_flag_num 思路) ---
 * 本场地一圈恰好 4 个"压线/离线"事件: B(压上弧)→C(离弧)→D(压上弧)→A(离弧,停)。
 * 压线节点 = 连续压线达 NODE_HIT_MIN_MS(短于此的擦线/捕获失败不算节点);
 * 离线节点 = 充分压线(≥HEADING_ONLINE_MIN_MS)后的正式出线锁定(START 首锁不算)。 */
#define NODE_HIT_MIN_MS   400

/* --- r24 手动配平(治"直线时左时右"的第3味药, 用户批准): 右轮占空恒定偏置,
 * 串口 'T'(+3)/'t'(-3) 在线调, 夹 ±40。语义: 车向右偏 -> 按 T(右轮加力往左修);
 * 车向左偏 -> 按 t。无任何自动逻辑, 值断电即失 —— 调好后把 '?' 回显或 ch23 的
 * 数值报回, 写死进本默认值再烧, 脱机跑才带得走。 --- */
#define DRIVE_TRIM_DUTY_R  -4     /* 待整定: 右轮占空偏置默认值(现场调好后回填) */

/* --- r21A 盲走航向 PI: 速度环退役后左右电机天然差异直通航向, 纯 P 只能"顶住",
 * 顶的结果是恒定航向残差(≈3~5°, 方向随电机差异/电量走 = "时左时右"的来源)。
 * 积分项把恒定不对称学掉, 残差归零。积分每次进盲走清零, 输出限幅防饱和。 --- */
#define HEADING_KI        0.0f   /* r22: 置 0 停用(=纯P, r20 行为)。r21 启用后用户观察到两轮
                                  * 转速明显不一致而叫停。注意待厘清: 若观察发生在台架(车不能
                                  * 转动, 航向误差无法消除, 积分必然顶到饱和=正常伪象)则本项
                                  * 无辜可复启; 若发生在地面且伴随画弧则真有问题。直线时左时右
                                  * 的替代药: 串口在线配平旋钮('T/t' 固定占空偏置), 待批准。 */
#define HEADING_INTEG_LIM 30.0f  /* 积分项输出限幅(mm/s): 上限=可学掉的不对称幅度 */

/* --- r21B 出线原地对正: 正式出线时车头与锁定方向差超门限, 先原地转正再起步
 * (参考方案 goAtoC "先转到位再直行" 的非阻塞版), 消灭"边走边拉"的横向积累;
 * 失败横穿后的沿用旧锁分支同样受益(转回原切线再走)。 --- */
#define EXIT_ALIGN_EN       0    /* r23: 出线对正总开关(用户要求回退到纯 r20 行为核对偏移;
                                  * 置 1 恢复 r21B 的"出线先转正再走") */
#define EXIT_ALIGN_TH_DEG   6.0f /* 出线锁定时航向差超此值触发对正 */
#define EXIT_ALIGN_DONE_DEG 3.0f /* 对正到差值以内交还直行 */
#define EXIT_ALIGN_MAX_MS   800  /* 对正超时保险: 超时放弃, 按当前航向直接走 */
#define LOST_BLIND_MAX    2000   /* 待整定: 盲走最大里程当量, 超限报错(占位) */

/* --- 瞄准 --- */
#define AIM_TIMEOUT_MS    5000   /* 待整定: 单点瞄准限时(题目 F2≤5s, 占位) */
#define AIM_TOL_PX        8      /* 待整定: K230 命中容差(像素)(占位) */
#define AIM_K_PAN         0.02f  /* 待整定: dx->pan 微调增益(符号待标)(占位) */
#define AIM_K_TILT        0.02f  /* 待整定: dy->tilt 微调增益(符号待标)(占位) */

/* --- 瞄准几何: 场地/车体/云台坐标(mm) ---
 * 世界坐标系: 原点=A, x 轴沿 A→B, y 轴垂直 AB 指向靶侧(y>0), z 垂直向上。
 * F2/F3 对靶时车都停在 B, 车头基本沿 A→B(+x)方向。IMU 出发航向 g_lap_yaw0
 * 作为世界系的 +x, 当前航向偏差量 = 车头相对 +x 的转角(度)。
 *
 * ★不做全场推航: IMU 已知 1°/2s 漂移, 累积到 B 会跑偏 20cm+ (车已跑 5s 以上),
 *   静态"车在 B"的常量近似比动态积分更靠谱(2026-07-04 firmware-codebase 记忆结论)。 */
#define AIM_PI            3.14159265358979f
#define AIM_DEG2RAD(d)    ((d) * (AIM_PI / 180.0f))
#define AIM_RAD2DEG(r)    ((r) * (180.0f / AIM_PI))

#define AB_LEN_MM         1400.0f  /* 待整定: A→B 直线长度(2024H 同款推测≈1400mm, 与 KP_GATES 一致) */
#define B_X_MM            AB_LEN_MM /* 车停 B 时的世界 x = AB 长度 */
#define B_Y_MM            0.0f      /* 车停 B 时的世界 y = 0 (B 在 x 轴上) */

/* 靶位: 题目 PDF 确认"靶距 AB 外侧 50cm, 与 AB 平行", 沿 x 方向落点未硬定 */
#define TARGET_X_MM       700.0f   /* 待整定: 靶心沿AB方向x坐标(占位=AB中点对面, 场地量) */
#define TARGET_Y_MM       500.0f   /* PDF 确定: 靶距 AB 外侧 500mm */
#define TARGET_Z_MM       500.0f   /* 待整定: 靶心离地高度(题目≤500mm, 占位=500) */

/* 云台安装(相对车中心, 车体坐标系): dx 前正、dy 右正、z 离地 */
#define CAR_GIMBAL_DX_MM   0.0f    /* 待整定: 云台安装前后偏移(前+, 占位=车中心正上) */
#define CAR_GIMBAL_DY_MM   0.0f    /* 待整定: 云台左右偏移(右+, 占位=车中线) */
#define CAR_GIMBAL_Z_MM  250.0f    /* 待整定: 云台旋转中心离地高度(车高 250mm, 占位=车顶) */

/* 舵机零点/符号: 与舵机装配朝向相关, 第一次通电试转标定
 * PAN_ZERO_DEG   = 车头正前方(pan 目标=0°几何角)对应的舵机输入角度
 * TILT_ZERO_DEG  = 云台水平(tilt=0°几何角)对应的舵机输入角度
 * _SIGN 若正是"几何角增大, 舵机角也增大"; 若装反了置 -1 */
#define AIM_PAN_ZERO_DEG   90.0f
#define AIM_TILT_ZERO_DEG  90.0f
#define AIM_PAN_SIGN      (+1.0f)  /* 待标: 第一次目视试转, 反了改 -1 */
#define AIM_TILT_SIGN     (+1.0f)  /* 待标: 同上 */

/* ===== 运行模式(F4: 按键/串口选模式; IDLE 态 MODE 键循环或串口发'1'~'4', RUN 灯闪"模式号"次) =====
 * 1 = F1 自动巡迹一圈回 A 停车(≤30s)
 * 2 = F2 定点瞄准(静止, 5s 内对靶)
 * 3 = F3 巡迹到靶联动(B 停车对靶 -> C/D 过点声光 -> 回 A 停车, ≤40s)
 * 4 = 发挥3 四圈连跑(每圈过 A 声光计圈, 跑满 4 圈停车) */
#define APP_MODE_MAX      4
#define APP_LAPS_MODE4    4      /* 发挥3 要求的连跑圈数 */

/* --- 关键点顺序门限表(B,C,D,A; 双条件"都超过"才判到点; 全部待整定) ---
 * 场地 = 2024H 同款(题目 Figure1 标注): A→B 直线, B→C 半弧(累计+180°), C→D 直线, D→A 半弧(+360°)。
 * 航向用 |净转角|(顺/逆时针通吃), 里程用右轮累计。理论: 直线≈1400mm, 半弧≈π*400≈1257mm。
 * 门限取"理论值 × ~0.8"留裕量, 靠顺序推进防误触; 真场地量了尺寸后回填。 */
#if !DRIVE_OPEN_LOOP   /* r20: 事件节点(压线/离线)取代里程门限, 本表仅速度闭环模式保留 */
typedef struct {
    float yaw_min_deg;    /**< 累计|航向|门限(°); 0 = 不设航向条件(纯里程) */
    float dist_min_mm;    /**< 累计里程门限(mm) */
} kp_gate_t;
static const kp_gate_t KP_GATES[4] = {
    /* B */ {   0.0f, 1100.0f },   /* 待整定: 第一直线尽头, 航向基本没转, 纯里程判 */
    /* C */ { 150.0f, 2100.0f },   /* 待整定: 第一个半弧走完(航向≥150°) */
    /* D */ { 150.0f, 3400.0f },   /* 待整定: 第二直线尽头 */
    /* A */ { 330.0f, 4000.0f },   /* 待整定: 整圈(原 F1 判定, F1/F3 共用) */
};
#endif /* !DRIVE_OPEN_LOOP (KP_GATES 表; keypoint_event 函数另有同条件包裹) */

/* --- 到点声光 --- */
#define KEYPOINT_BEEP_MS  400    /* 待整定: 到点蜂鸣时长; 线上评审靠视频, 给足 400ms 让声光在片子里清晰可辨 */

/* ============================================================================
 *  模块内状态
 * ==========================================================================*/

/* 可在中断里被 app_estop 写、主循环 FSM 读, 必须 volatile(防编译器把它缓存到寄存器)。 */
static volatile app_state_e g_state = APP_ST_IDLE;   /**< 当前状态机状态 */

/* 三个 PID 控制器(占位增益, 真车整定) */
static pid_t g_pid_track;   /**< 循迹外环: 灰度/航向偏差 -> 转向量(位置式) */
static pid_t g_pid_vel_l;   /**< 速度内环-左: 编码器速度(增量式) */
static pid_t g_pid_vel_r;   /**< 速度内环-右: 编码器速度(增量式) */

/* 外环写、内环读的"左右目标速度"(速度当量, 非占空) */
static float g_vel_tgt_l = 0.0f;   /**< 左轮目标速度(track 外环设定) */
static float g_vel_tgt_r = 0.0f;   /**< 右轮目标速度 */

/* 内环累加输出(增量式 PID 的 Δu 累加到这里, 即下发给 motor 的占空) */
static float g_vel_out_l = 0.0f;   /**< 左轮当前下发占空(累加值) */
static float g_vel_out_r = 0.0f;   /**< 右轮当前下发占空(累加值) */

/* 可在中断里被 app_start_request 写、主循环 FSM 读, 必须 volatile。 */
static volatile uint8_t g_start_req = 0;   /**< 启动请求标志(app_start_request 置位, FSM 消费) */
static uint32_t g_aim_timer  = 0;   /**< 瞄准计时(ms), AIM 态累加用于超时判定 */
static uint32_t g_lost_dist  = 0;   /**< 丢线盲走累计里程当量(超 LOST_BLIND_MAX 报错) */

/* --- 速度内环测速滤波(每轮一阶低通状态) --- */
static float g_meas_filt[2] = {0.0f, 0.0f};   /**< [ENC_LEFT/RIGHT] 测速低通状态,抑抖 */
/* --- 丢线盲走(IMU 航向锁定)状态 --- */
static float   g_yaw_lock     = 0.0f;   /**< 丢线时锁定的航向基准(度, 取压线期滑动平均+滞后外推) */
static float   g_blind_turn_dbg = 0.0f; /**< 最近一拍盲走转向量(VOFA ch21 用; 压线时清 0) */
static float   g_yaw_line_avg = 0.0f;   /**< 压线期航向滑动平均-慢(τ≈0.4s, 滤蛇形) */
static float   g_yaw_line_avg_f = 0.0f; /**< 压线期航向滑动平均-快(τ≈0.13s, 与慢配对外推消弧线滞后) */
static int32_t g_blind_ref    = 0;      /**< 丢线瞬间的里程基准(右轮计数) */
static uint8_t g_blind_active = 0;      /**< 盲走中标志(抓丢线上升沿,基准只锁一次; ⚠START必须清=r17教训) */
static uint16_t g_online_ms   = 0;      /**< 本次连续压线时长(ms): 不足门槛丢线时, 平均值不可信 */
static uint8_t g_acq_cnt      = 0;      /**< r19: 盲走中连续在线帧计数(解锁去抖, 闪断不算重新压线) */
static uint8_t g_lock_valid   = 0;      /**< r19: 已有一次正式锁定(短压线再丢线时沿用旧锁, 防污染重锁) */
static float    g_head_integ  = 0.0f;   /**< r21A: 盲走航向积分(°·s), 每次进盲走清零 */
static uint16_t g_align_ms    = 0;      /**< r21B: 出线原地对正剩余时限 ms(0=不在对正) */
static uint16_t g_pivot_ms    = 0;      /**< (r20 遗留, r25 起未用) */
/* --- r25 CTY 段状态机 --- */
static uint8_t  g_seg         = 0;      /**< 当前段 0=A→B空白 1=B→C弧 2=C→D空白 3=D→A弧 */
static uint8_t  g_nodata      = 0xFF;   /**< CTY 漏积分丢线计数(空白帧++, 有线帧减半) */
static float    g_datum_yaw   = 0.0f;   /**< 出发基准角 z = START 时车头(=A→B 方向)绝对航向 */
static uint16_t g_protect_ms  = 0;      /**< 段切换保护窗(ms) */
static uint16_t g_startprot_ms = 0;     /**< 起步保护(ms): 强制 seg0+无视灰度 */
static float    g_dyaw_prev   = 0.0f;   /**< SelfTurn D 项上拍 DeltaYaw */
static uint8_t g_node_pending = 0;      /**< r20: 待 FSM 消费的节点事件数(track 写, FSM 读, 同主循环无竞态) */
static uint8_t g_node_hit_fired = 0;    /**< r20: 本次压线的"压线节点"已发过(one-shot) */
/* --- 状态机入态检测 + STOP 声光一次性 --- */
static app_state_e g_prev_state = APP_ST_IDLE;   /**< 上一拍状态(检测"刚入态") */
static int32_t g_stop_beep_ms   = 0;             /**< STOP 到点蜂鸣剩余时长(ms),非阻塞关断 */
/* --- 关键点判定基准(进 RUN 时快照)与任务进度 --- */
static float   g_lap_yaw0 = 0.0f;   /**< 出发瞬间航向(°), 累计净转角的零点 */
static int32_t g_lap_odo0 = 0;      /**< 出发瞬间右轮累计计数, 累计里程的零点 */
static uint8_t g_run_mode = 1;      /**< 运行模式 1..APP_MODE_MAX(IDLE 态 MODE 键循环) */
static uint8_t g_kp_idx   = 0;      /**< 关键点序列进度: 0=还没到B ... 4=已回A */
static uint8_t g_lap_count = 0;     /**< 已完成圈数(模式4 四圈连跑用) */

/* --- 在线调参副本(串口实时调, #define 只当默认值; 现场整定免重编译烧录) --- */
static float g_track_kp   = TRACK_KP;    /**< 循迹外环 KP(运行时可调) */
static float g_track_kd   = TRACK_KD;    /**< 循迹外环 KD(运行时可调) */
static int   g_base_speed = BASE_SPEED;  /**< 巡迹基速 mm/s(运行时可调) */
static float g_heading_kp = HEADING_KP;  /**< 盲走航向锁 KP(串口 'H'/'h' ±0.5, 可为负) */
static int   g_trim_duty  = DRIVE_TRIM_DUTY_R; /**< r24: 右轮占空配平('T'/'t' ±3, 夹±40) */
static uint8_t g_aim_return_run = 0;/**< AIM 结束后回 RUN(F3 中途对靶)还是进 STOP(F2) */
static int32_t g_run_beep_ms = 0;   /**< RUN 态过点声光剩余时长(ms), 非阻塞关断 */

/* ============================================================================
 *  小工具
 * ==========================================================================*/

/* 浮点限幅(本层小工具, 不引外部数学库) */
static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* 角度归一到 [-180,180]：处理航向在 ±180 处的跨界翻转(借鉴参考工程的 Yaw wrap 处理),
 * 否则 (yaw0 - yaw) 在越过 ±180 时会突变一大截,导致盲走瞬间猛打方向。 */
static float wrap180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* ============================================================================
 *  速度内环 —— 每轮一个增量式 PID
 * ==========================================================================*/

/**
 * @brief 单轮速度内环计算并下发(内部用)
 * @param ch     轮通道: ENC_LEFT/ENC_RIGHT(=MOTOR_LEFT/MOTOR_RIGHT, 编号同值 0/1,
 *               既给 enc_get_delta 取编码器增量, 也给 motor_set 下发占空)
 * @param p      该轮的 PID 控制器
 * @param target 目标速度(当量)
 * @param out    [in/out] 该轮累加输出(占空), 本函数把 Δu 累加进去并限幅
 * @note  实测速度(mm/s) = enc_get_delta(ch) * 该轮 mm/计数 / dt(按轮取 ENC_L/R_MM_PER_CNT,
 *        每圈计数已实测, 见 encoder.h)。增量式 PID 返回本次占空增量 Δu, 累加到 out 后限幅再 motor_set。
 *        ⚠ 轮径 WHEEL_DIAM_MM 与 PID 增益仍是占位, 待实测/整定。
 */
/**
 * @brief 单轮测速(内部用): 编码器增量 -> 低通后的实测速度(mm/s), 并刷新 g_meas_filt
 * @note  与控制解耦, 开环诊断模式下也照常调用(只测不控)。
 *        ⚠ 换算系数必须按轮取: 左右解码倍频差 3.84 倍, 用错轮的系数速度环立刻乱。
 *        ⚠ dt 用 g_tick_ms 实测而非名义 APP_VEL_DT_MS: 调度可能被 I2C 超时等拖延,
 *          按名义周期算会把速度虚高 2~3 倍, 环会追着假数据振荡(2026-07-04 实测坐实)。
 */
static float vel_measure(int ch)
{
    int32_t delta = enc_get_delta(ch);
    int fi = (ch == ENC_RIGHT) ? 1 : 0;
    static uint32_t s_last_ms[2] = {0u, 0u};
    uint32_t now_ms = g_tick_ms;
    uint32_t dt_ms  = now_ms - s_last_ms[fi];
    s_last_ms[fi]   = now_ms;
    if (dt_ms == 0u || dt_ms > 200u) {
        dt_ms = APP_VEL_DT_MS;   /* 首拍/长暂停(调试器断点等)兜底, 防除零与荒谬尖峰 */
    }
    float mm_per_cnt = (ch == ENC_RIGHT) ? ENC_R_MM_PER_CNT : ENC_L_MM_PER_CNT;
    float meas_raw = (float)delta * mm_per_cnt / ((float)dt_ms / 1000.0f);
    /* 尖峰保护: 本车物理极限约 ±1200mm/s(占空600, 12V空载外推), 超 ±1500 必是毛刺
     * (噪声/计数异常/时序抖动), 夹掉防止污染滤波器与控制环(2026-07-04 曾实测到 -1931 的假值)。 */
    if (meas_raw >  1500.0f) meas_raw =  1500.0f;
    if (meas_raw < -1500.0f) meas_raw = -1500.0f;
    /* 一阶低通滤波抑制编码器抖动/量化噪声(等效参考工程的多点滑窗测速),每轮各一份状态。 */
    g_meas_filt[fi] = g_meas_filt[fi] * (1.0f - VEL_MEAS_LPF) + meas_raw * VEL_MEAS_LPF;
    return g_meas_filt[fi];
}

static void vel_loop_one(int ch, pid_t *p, float target, float *out)
{
    /* 1) 测速(带低通, mm/s)。 */
    float meas = vel_measure(ch);

    /* 2) 增量式 PID: 返回本次占空"增量" Δu(增量式天然抗积分饱和)。 */
    float du = pid_inc_update(p, target, meas);

    /* 3) 累加到该轮下发占空并限幅(增量式必须自己维护累加量)。 */
    *out = clampf(*out + du, VEL_OUT_MIN, VEL_OUT_MAX);

    /* 4) "瞎油门保险丝"(2026-07-05, 摔坏左编码器实锤的教训): 编码器一瞎, 闭环变成
     *    瞎子踩油门——占空顶满而测速为零, 轮子实际全速失控。判据: |占空|≥90%上限
     *    且 |实测|<50mm/s 持续 500ms => 传感器/执行器故障, 急停 + 红灯。 */
    {
        static uint16_t s_stall_ms[2] = { 0u, 0u };
        int fi = (ch == ENC_RIGHT) ? 1 : 0;
        float mag_out  = (*out < 0.0f) ? -*out : *out;
        float mag_meas = (meas < 0.0f) ? -meas : meas;
        if (mag_out >= 0.9f * VEL_OUT_MAX && mag_meas < 50.0f) {
            s_stall_ms[fi] = (uint16_t)(s_stall_ms[fi] + APP_VEL_DT_MS);
            if (s_stall_ms[fi] >= 500u) {
                s_stall_ms[0] = s_stall_ms[1] = 0u;
                app_estop();   /* 内部停电机断舵机轨, FSM 进 ESTOP(红灯), RESET 才能退出 */
                return;
            }
        } else {
            s_stall_ms[fi] = 0u;
        }
    }

    /* 5) 下发到电机(motor_set 内部带方向/限幅; duty 标度 -1000..1000)。 */
    motor_set(ch, (int)(*out));
}

void vel_loop_step(void)
{
    /* 仅在"该出力"的态下闭环驱动; 其它态(IDLE/AIM/STOP/ESTOP)电机另行处理。 */
    if (g_state != APP_ST_RUN) {
        return;
    }
#if DRIVE_OPEN_LOOP
    /* r20: 编码器只测不控 —— I5/I7 遥测保留(持续观察两个带病编码器的病情),
     * 电机由 track_loop_step 直接以占空驱动; 速度闭环与"瞎油门保险丝"停用
     * (保险丝以编码器为证词, 编码器说谎时它反而会误杀, 台架已发生过)。 */
    (void)vel_measure(ENC_LEFT);
    (void)vel_measure(ENC_RIGHT);
#elif VEL_OPEN_LOOP_TEST
    /* ★开环诊断: 恒占空直驱 + 照常测速(只测不控)。VOFA 上 I5/I7 显示的是真实测速,
     * I4/I6 目标值此模式下无意义。诊断完把 VEL_OPEN_LOOP_TEST 改回 0! */
    (void)vel_measure(ENC_LEFT);
    (void)vel_measure(ENC_RIGHT);
    g_vel_out_l = g_vel_out_r = (float)VEL_OPEN_LOOP_DUTY;   /* 供调试观察 */
    motor_set(MOTOR_LEFT,  VEL_OPEN_LOOP_DUTY);
    motor_set(MOTOR_RIGHT, VEL_OPEN_LOOP_DUTY);
#else
    vel_loop_one(ENC_LEFT,  &g_pid_vel_l, g_vel_tgt_l, &g_vel_out_l);
    vel_loop_one(ENC_RIGHT, &g_pid_vel_r, g_vel_tgt_r, &g_vel_out_r);
#endif
}

/* ============================================================================
 *  r25 驱动核心 —— CTY(24H第四问开源) 架构移植: 统一航向误差接口
 *  (用户 2026-07-05 拍板: 弃自研"出线锁航向"体系, 按拿奖开源重构)
 *
 *  一切控制汇聚成一个量 DeltaYaw(°, 车头还需转多少, 左正右负):
 *    空白直线段: DeltaYaw = wrap180(段目标绝对航向 − 当前航向)
 *                段目标 = 出发基准角 z(A→B) / z+180(C→D) —— 场地几何锚定,
 *                彻底不依赖"出线瞬间锁的航向"(旧架构一切玄学问题的源头)
 *    弧线段循线: DeltaYaw = 灰度查表(哪路压线→偏差角 ±5/15/25/40°, CTY 实测梯度)
 *    弧内丢线:   保护窗内向弧内侧强拐 ARC_SEARCH_DEG 找线
 *  然后 SelfTurn PD 出差速, 开环占空执行(编码器仍不参与控制)。
 *
 *  段状态机(一圈恰 4 段; 起点=A, 车头手动对准 B):
 *    seg0 A→B空白(目标z) → seg1 B→C弧(循线) → seg2 C→D空白(目标z+180)
 *    → seg3 D→A弧(循线) → 回 seg0(=过A, FSM 计圈/停车)
 *  切换判据 = CTY 漏积分计数 g_nodata: 空白帧(0xFF/0x00)自增, 有线帧减半
 *  (快认线/慢认空, 天然去抖); 每次切换发节点事件(FSM 声光/路由)并开保护窗。
 *  起步保护 1.5s 强制 seg0+无视灰度(A 点胶带残端免疫)。
 * ==========================================================================*/

/**
 * @brief 灰度字节 -> 航向偏差角查表(CTY 130 速档实测梯度)
 * @note  bit=0=压线(黑); bit0=最左探头(1号朝左, 实车已验向)。线在左=应左转=正角。
 */
static float gray_ladder(uint8_t b)
{
    if (!(b & 0x01)) return +40.0f;
    if (!(b & 0x80)) return -40.0f;
    if (!(b & 0x02)) return +25.0f;
    if (!(b & 0x40)) return -25.0f;
    if (!(b & 0x04)) return +15.0f;
    if (!(b & 0x20)) return -15.0f;
    if (!(b & 0x08) && !(b & 0x10)) return 0.0f;   /* 中间两路都压 = 正中 */
    if (!(b & 0x08)) return +5.0f;
    if (!(b & 0x10)) return -5.0f;
    return 0.0f;   /* 全白兜底(调用方已挡) */
}

void track_loop_step(void)
{
    if (g_state != APP_ST_RUN) {
        return;
    }

    /* 1) 灰度取数: gray_get_error 刷一次 I2C(质心值仅供 ch10 遥测), 原始字节走
     *    diag 通道取。字节语义: bit=0=压线(黑), 0xFF=全白, 0x00=全黑/失联。 */
    (void)gray_get_error();
    uint8_t byte = 0xFF;
    gray_get_diag(0, 0, &byte);

    /* 2) CTY 漏积分丢线计数: 空白帧自增(封顶), 有线帧减半 —— 快认线/慢认空 */
    if (byte == 0xFF || byte == 0x00) {
        if (g_nodata < 0xFF) g_nodata++;
    } else if (g_nodata > 0) {
        g_nodata >>= 1;
    }

    /* 3) 起步保护: 强制 seg0 + 无视灰度(A 点残端/起步抖动免疫, CTY BeginProtect) */
    if (g_startprot_ms > 0u) {
        g_startprot_ms = (g_startprot_ms > APP_TRACK_DT_MS)
                       ? (uint16_t)(g_startprot_ms - APP_TRACK_DT_MS) : 0u;
        g_seg = 0;
        g_nodata = 0xFF;
    }

    /* 4) 切换保护窗递减(CTY ProtectFlag: 切换后一段时间禁止再切, 防抖) */
    if (g_protect_ms > 0u) {
        g_protect_ms = (g_protect_ms > APP_TRACK_DT_MS)
                     ? (uint16_t)(g_protect_ms - APP_TRACK_DT_MS) : 0u;
    }

    /* 5) 段切换: 每次切换 = 一个节点事件(B/C/D/A), FSM 消费(声光/AIM/计圈/停车) */
    if (g_protect_ms == 0u && g_startprot_ms == 0u) {
        uint8_t lined = (uint8_t)(g_nodata <= SEG_LINE_TH);
        uint8_t blank = (uint8_t)(g_nodata >= SEG_BLANK_TH);
        if ((g_seg == 0u || g_seg == 2u) && lined) {
            g_seg++;                                    /* 0→1 到B / 2→3 到D */
            g_protect_ms = TRANS_PROTECT_MS;
            g_node_pending++;
        } else if ((g_seg == 1u || g_seg == 3u) && blank) {
            g_seg = (uint8_t)((g_seg + 1u) & 0x03u);    /* 1→2 过C / 3→0 回A */
            g_protect_ms = TRANS_PROTECT_MS;
            g_node_pending++;
        }
    }

    /* 6) 统一航向误差 DeltaYaw(°, 左正) —— 本架构唯一的控制接口 */
    float yaw = imu_get_yaw();
    float dy;
    if (g_seg == 0u) {
        dy = wrap180(g_datum_yaw - yaw);                /* A→B: 出发基准角 z */
    } else if (g_seg == 2u) {
        dy = wrap180(g_datum_yaw + 180.0f - yaw);       /* C→D: z+180(反向) */
    } else if (byte != 0xFF && byte != 0x00) {
        dy = gray_ladder(byte);                         /* 弧上循线: 查表 */
    } else if (g_protect_ms > 0u) {
        /* 刚进弧就丢线(冲过线头): 保护窗内向弧内侧强拐找线(CTY 手法) */
        dy = LAP_CW ? -ARC_SEARCH_DEG : +ARC_SEARCH_DEG;
    } else {
        dy = 0.0f;                                      /* 弧上短暂全白: 保持直走 */
    }

    /* 7) SelfTurn PD -> 差速占空('P/p' 调 KP ±0.5, 'D/d' 调 KD ±1) */
    float diff_f = g_track_kp * dy + g_track_kd * (dy - g_dyaw_prev);
    g_dyaw_prev = dy;
    int diff = (int)clampf(diff_f, -(float)TURN_DIFF_LIM, (float)TURN_DIFF_LIM);

    /* 8) 输出: 开环占空差速(dy>0=左转=右轮加左轮减; 'V/v' 调基速占空 ±10) */
    {
        int dl = WHEEL_SIGN_L * (g_base_speed - diff);
        int dr = WHEEL_SIGN_R * (g_base_speed + diff) + g_trim_duty;
        g_vel_tgt_l = dy;                  /* 遥测 ch4 = DeltaYaw(°) */
        g_vel_tgt_r = (float)g_seg;        /* 遥测 ch6 = 当前段号 0..3 */
        g_vel_out_l = (float)dl;           /* 遥测 ch8/ch9 = 实发占空 */
        g_vel_out_r = (float)dr;
        motor_set(MOTOR_LEFT,  dl);
        motor_set(MOTOR_RIGHT, dr);
    }
    g_blind_turn_dbg = (float)diff;        /* 遥测 ch21 = 差速 */
    g_yaw_lock = (g_seg == 0u) ? g_datum_yaw
               : (g_seg == 2u) ? wrap180(g_datum_yaw + 180.0f) : yaw;  /* ch20 = 当前段目标 */
}

/* ============================================================================
 *  瞄准 —— 几何主路径(无视觉) + K230 末端精修
 * ==========================================================================*/

/**
 * @brief 几何反解云台指向(无视觉主路径, 2026-07-05 落地真公式)
 * @param pan_deg  [out] PAN 目标舵机角(度, 0..180)
 * @param tilt_deg [out] TILT 目标舵机角(度, 0..180)
 *
 * 【坐标系】原点=A, x沿A→B, y垂直AB指向靶(y>0), z 垂直向上。
 * 【假设】F2/F3 对靶时车停在 B; 车头相对世界 +x 的角度 = 当前IMU航向 − 出发航向(度)。
 *         若真实静止位置不在 B, 只需改 B_X/Y 或加"F2专用车位"常量, 计算不变。
 * 【流程】1) 由车头朝向把云台安装偏移(车体系)旋转到世界系, 得云台在场上坐标 G=(Gx,Gy);
 *         2) 靶 T=(TARGET_X, TARGET_Y, TARGET_Z), 相对G水平向量 (rx,ry) = (Tx-Gx, Ty-Gy);
 *         3) Pan(世界系) = atan2(ry, rx); Pan(车体系) = Pan(世界系) − 车头朝向;
 *         4) 水平距离 d = √(rx²+ry²), 垂直差 dz = Tz − 云台高;
 *         5) Tilt = atan2(dz, d);
 *         6) 加舵机零点/符号 → 输出目标舵机角, 夹在 0..180。
 * 【依赖占位】AB_LEN_MM、TARGET_X_MM、TARGET_Z_MM、CAR_GIMBAL_DX/DY/Z、
 *           AIM_PAN/TILT_ZERO_DEG、AIM_PAN/TILT_SIGN
 *           均在头部常量组标 "待整定/待标", 场地量了+首次通电试转后回填。
 */
static void aim_geometry(float *pan_deg, float *tilt_deg)
{
    /* 1) 车头朝向(世界系, 弧度): 当前IMU航向减出发航向, wrap 到 (-180,180] 防跨界 */
    float theta_deg = wrap180(imu_get_yaw() - g_lap_yaw0);
    float theta = AIM_DEG2RAD(theta_deg);
    float ct = cosf(theta), st = sinf(theta);

    /* 2) 云台旋转中心的世界坐标 G:
     *    车中心(B_X, B_Y) + 云台安装偏移(dx,dy)按车头旋转到世界系。 */
    float Gx = B_X_MM + CAR_GIMBAL_DX_MM * ct - CAR_GIMBAL_DY_MM * st;
    float Gy = B_Y_MM + CAR_GIMBAL_DX_MM * st + CAR_GIMBAL_DY_MM * ct;

    /* 3) 靶相对云台的水平向量(世界系) */
    float rx = TARGET_X_MM - Gx;
    float ry = TARGET_Y_MM - Gy;

    /* 4) Pan: 世界系"云台→靶"方向 − 车头方向 = 车体系水平指向角
     *    atan2 输出 (-π, π], 减 theta 后再 wrap 一次防溢出 servo 0..180 边界 */
    float pan_world = atan2f(ry, rx);          /* 弧度 */
    float pan_body  = pan_world - theta;
    while (pan_body >  AIM_PI) pan_body -= 2.0f * AIM_PI;
    while (pan_body < -AIM_PI) pan_body += 2.0f * AIM_PI;
    float pan = AIM_PAN_ZERO_DEG + AIM_PAN_SIGN * AIM_RAD2DEG(pan_body);

    /* 5) Tilt: 水平距离 vs 垂直高差; 靶高于云台=仰(正), 低于=俯(负) */
    float d_horiz = sqrtf(rx * rx + ry * ry);
    float dz = TARGET_Z_MM - CAR_GIMBAL_Z_MM;
    /* d_horiz≈0 保护: 车恰在靶正下方(几何上不可能, 但 F2 万一放错)则朝天 */
    float tilt_rad = (d_horiz < 1.0f) ? (AIM_PI * 0.5f) : atan2f(dz, d_horiz);
    float tilt = AIM_TILT_ZERO_DEG + AIM_TILT_SIGN * AIM_RAD2DEG(tilt_rad);

    /* 6) 夹在舵机可动区间。servo_set_angle 内部还会再限, 这里先夹防负数溢出 int 转换 */
    if (pan  <   0.0f) pan  =   0.0f;
    if (pan  > 180.0f) pan  = 180.0f;
    if (tilt <   0.0f) tilt =   0.0f;
    if (tilt > 180.0f) tilt = 180.0f;

    *pan_deg  = pan;
    *tilt_deg = tilt;
}

/**
 * @brief K230 末端精修(辅助; found=0 或无帧则不动几何指向)
 * @param pan_deg  [in/out] 在几何指向基础上微调 PAN
 * @param tilt_deg [in/out] 微调 TILT
 * @return 1=已命中(|dx|,|dy| 在容差内); 0=未命中/无新帧/无视觉(回退几何)
 * @note  无视觉独立性: 拔摄像头或 found=0 时本函数原样返回 0, 不破坏几何指向。
 *        ⚠ AIM_K_PAN/TILT 与 dx/dy 符号待标(test_plan §D, 与视觉 lane 对齐)。
 */
static int aim_refine_k230(float *pan_deg, float *tilt_deg)
{
    int16_t dx, dy;

    /* 取最新一帧瞄准偏差: k230 模块内部已跑协议状态机并判超时, 返回 found 标志。
     * found=0(未找到/无帧/失联) 一律回退几何指向, 绝不阻塞等视觉(拔摄像头也不卡死)。 */
    uint8_t found = k230_get_aim(&dx, &dy);
    if (!found) {
        return 0;   /* found=0: 回退几何, 不微调 */
    }

    /* 像素偏差 -> 角度微调(增益/符号占位待标)。 */
    *pan_deg  += AIM_K_PAN  * (float)dx;
    *tilt_deg += AIM_K_TILT * (float)dy;

    /* 命中判定: dx/dy 都进容差。 */
    return (dx > -AIM_TOL_PX && dx < AIM_TOL_PX &&
            dy > -AIM_TOL_PX && dy < AIM_TOL_PX);
}

/**
 * @brief 执行一次对靶(几何主 + K230 精修)并驱动云台
 * @return 1=命中; 0=未命中
 */
static int aim_step_once(void)
{
    float pan, tilt;
    aim_geometry(&pan, &tilt);                 /* 1) 无视觉几何指向(主) */
    int hit = aim_refine_k230(&pan, &tilt);    /* 2) K230 精修(辅, 自动回退) */
    servo_set_angle(SERVO_PAN,  (int)pan);
    servo_set_angle(SERVO_TILT, (int)tilt);
    return hit;
}

/* ============================================================================
 *  关键点判定 —— 里程 + 灰度特征
 * ==========================================================================*/

#if !DRIVE_OPEN_LOOP   /* r20: 事件节点取代本函数, 仅速度闭环模式保留 */
/**
 * @brief 关键点顺序判定(2026-07-04 实装, 由"仅整圈"扩展为 B/C/D/A 序列)
 * @return 0=无事件; 1=到B 2=到C 3=到D 4=回A(完赛)
 * @note  双确认判据(航向累计 + 里程累计, 都超过门限才触发), 顺序推进防跳点/误触:
 *        F3(模式3)按 B→C→D→A 逐点走; F1(模式1)只看最后一行(整圈), 中途不发事件。
 *        航向是 IMU 连续累计值直接做差取绝对值, 顺/逆时针跑圈通吃;
 *        里程取右轮 int32 累计(不回绕)。基准在 IDLE→RUN 迁移时快照。
 */
static int keypoint_event(void)
{
    if (g_kp_idx >= 4u) {
        return 0;   /* 序列已走完 */
    }

    float dyaw = imu_get_yaw() - g_lap_yaw0;               /* 出发以来净转角(°, 带符号) */
    if (dyaw < 0.0f) dyaw = -dyaw;                          /* 兼容顺/逆时针 */
    float dist_mm = (float)(enc_get_count(ENC_RIGHT) - g_lap_odo0) * ENC_R_MM_PER_CNT;
    if (dist_mm < 0.0f) dist_mm = -dist_mm;

    /* 模式3 逐点推进; 其它模式只关心"整圈"那一行(下标3) */
    uint8_t idx = (g_run_mode == 3u) ? g_kp_idx : 3u;
    const kp_gate_t *gate = &KP_GATES[idx];

    if (dyaw >= gate->yaw_min_deg && dist_mm >= gate->dist_min_mm) {
        g_kp_idx = (uint8_t)(idx + 1u);
        return (int)idx + 1;   /* 1=B 2=C 3=D 4=A */
    }
    return 0;
}
#endif /* !DRIVE_OPEN_LOOP */

/* ============================================================================
 *  状态机
 * ==========================================================================*/

void app_fsm_step(void)
{
    /* 入态检测: 本拍状态 != 上一拍 = 刚进入该态, 用于到点声光等"只做一次"的动作。 */
    uint8_t entered = (g_state != g_prev_state);
    g_prev_state = g_state;

    switch (g_state) {

    case APP_ST_IDLE:
        /* 待机: 执行器保持停; RUN 灯每 2s 闪"模式号"次(F4 按键选模式的人机反馈)。 */
        motor_stop_all();
        {
            static uint16_t s_blink_ms = 0;
            s_blink_ms = (uint16_t)((s_blink_ms + APP_FSM_DT_MS) % 2000u);
            uint16_t slot = s_blink_ms / 200u;   /* 把 2s 切成 10 格, 每格 200ms */
            /* 前 mode*2 格里"亮灭交替" -> 闪 mode 次, 其余时间灭 */
            led_run_set((slot < (uint16_t)(g_run_mode * 2u)) && ((slot & 1u) == 0u));
        }
        if (g_start_req) {
            g_start_req = 0;
            /* 启动前复位: 清 PID 历史 + 内环累加 + 盲走里程 + 关键点进度, 防上次残留。 */
            pid_reset(&g_pid_track);
            pid_reset(&g_pid_vel_l);
            pid_reset(&g_pid_vel_r);
            g_vel_out_l = g_vel_out_r = 0.0f;
            g_vel_tgt_l = g_vel_tgt_r = 0.0f;
            g_lost_dist = 0;
            g_blind_active = 0;   /* ★r17: 上轮跑到一半丢线急停时此标志滞留=1, 不清则新一轮
                                   * 第一拍盲走沿用上轮陈旧锁定航向 -> 启动即猛转(真机转圈实锤) */
            g_online_ms = 0;
            g_acq_cnt = 0;
            g_lock_valid = 0;     /* r19: 新一轮的第一次锁定必须重算, 不许沿用上轮的锁 */
            g_pivot_ms = 0;
            g_node_pending = 0;
            g_node_hit_fired = 0;
            g_head_integ = 0.0f;
            g_align_ms = 0;
            /* r25 CTY 段状态机复位 + 基准角快照: 车此刻手动对准 A→B 方向,
             * 这一记 yaw 就是全圈两段空白直线的绝对航向锚(z 与 z+180)。 */
            g_datum_yaw = imu_get_yaw();
            g_seg = 0;
            g_nodata = 0xFF;
            g_protect_ms = 0;
            g_startprot_ms = START_PROTECT_MS;
            g_dyaw_prev = 0.0f;
            /* r19 卫生项(审计): 消费搬车期间滞留的编码器增量 + 清测速滤波,
             * 防起步首拍速度环吃到 ±1500 假速度脉冲(非 RUN 态无人读 delta, 会攒一大坨) */
            (void)enc_get_delta(ENC_LEFT);
            (void)enc_get_delta(ENC_RIGHT);
            g_meas_filt[0] = g_meas_filt[1] = 0.0f;
            g_kp_idx = 0;
            g_lap_count = 0;
            g_run_beep_ms = 0;
            /* 关键点判定基准快照: 以"此刻"的航向/里程为零点(见 keypoint_event) */
            g_lap_yaw0 = imu_get_yaw();
            g_lap_odo0 = enc_get_count(ENC_RIGHT);
            g_yaw_line_avg   = imu_get_yaw(); /* 线向估计(快慢两份)从出发航向起步 */
            g_yaw_line_avg_f = g_yaw_line_avg;
            /* TODO: 上电自检(电源轨/编码器空转/IMU WHO_AM_I/K230 握手), 失败进 ESTOP。 */
            led_run_set(1);
            if (g_run_mode == 2u) {
                /* F2: 静止定点瞄准, 不巡迹, 直接进 AIM; 结束进 STOP */
                g_aim_timer = 0;
                g_aim_return_run = 0;
                g_state = APP_ST_AIM;
            } else {
                g_state = APP_ST_RUN;
            }
        }
        break;

    case APP_ST_RUN:
        /* 行驶: 跑线由 track_loop_step(外环) + vel_loop_step(内环) 各自任务完成,
         * 本函数管"到点事件"分流与过点声光。 */
        if (g_run_beep_ms > 0) {          /* 过点声光: 非阻塞脉冲, 到时自动关 */
            g_run_beep_ms -= APP_FSM_DT_MS;
            if (g_run_beep_ms <= 0) {
                beep_off();
            }
        }
        {
            /* r20 事件节点路由(取代里程/航向门限): 一圈 4 节点 = B压线→C离线→D压线→A离线。
             * 节点由 track 环产生(压线≥400ms / 正式出线), 天然对准真实场地, 免标定。 */
            while (g_node_pending > 0u) {
                g_node_pending--;
                g_kp_idx++;
                beep_on();                       /* 每个节点声光提示(题目要求) */
                g_run_beep_ms = KEYPOINT_BEEP_MS;
                if (g_run_mode == 3u && g_kp_idx == 1u) {
                    /* F3 到 B(第1节点): 停车对靶, 结束后回 RUN 续跑 */
                    g_aim_timer = 0;
                    g_aim_return_run = 1;
                    g_state = APP_ST_AIM;
                } else if (g_kp_idx >= 4u) {
                    if (g_run_mode == 4u && (g_lap_count + 1u) < APP_LAPS_MODE4) {
                        g_lap_count++;           /* 发挥3: 过 A 计圈, 继续跑下一圈 */
                        g_kp_idx = 0;
                    } else {
                        g_state = APP_ST_STOP;   /* 回 A: 完赛停车(F1/F3/第4圈共用) */
                    }
                }
            }
        }
        break;

    case APP_ST_AIM:
        /* 瞄准: 整车停, 云台几何指向 + K230 精修。命中或超时离开。 */
        motor_stop_all();
        if (entered) {
            /* 限制⑥: 舵机轨平时断电, 对靶作业期间才供电(r15 补, 此前全工程无人开轨=
             * MOS 开关装上后会对着断电舵机瞄准)。现红线直连 5V 时本调用无实际效果。 */
            servo_rail_enable(1);
        }
        {
            int hit = aim_step_once();
            g_aim_timer += APP_FSM_DT_MS;
            if (hit || g_aim_timer >= AIM_TIMEOUT_MS) {
                servo_rail_enable(0);   /* 离开对靶 -> 舵机轨断电(F3 续跑/F2 停车都关) */
                if (g_aim_return_run) {
                    /* F3: 对靶完成, 继续巡迹。重置内环状态, 从静止平滑重加速。 */
                    g_aim_return_run = 0;
                    pid_reset(&g_pid_vel_l);
                    pid_reset(&g_pid_vel_r);
                    g_vel_out_l = g_vel_out_r = 0.0f;
                    led_run_set(1);
                    g_state = APP_ST_RUN;
                } else {
                    g_state = APP_ST_STOP;   /* F2: 定点作业完成即收 */
                }
            }
        }
        break;

    case APP_ST_STOP:
        /* 正常停车: 电机停 + 到点声光(一次性, 非阻塞)。 */
        motor_stop_all();
        led_run_set(0);
        if (entered) {                 /* 到点只在"刚入 STOP"时响一次, 不每拍响 */
            beep_on();
            g_stop_beep_ms = KEYPOINT_BEEP_MS;
        }
        if (g_stop_beep_ms > 0) {      /* 非阻塞关断: 到时自动 beep_off, 不在此忙等 */
            g_stop_beep_ms -= APP_FSM_DT_MS;
            if (g_stop_beep_ms <= 0) {
                beep_off();
            }
        }
        /* TODO: 多圈/多点任务在此判是否回 RUN 续跑; 占位停在 STOP。 */
        break;

    case APP_ST_ESTOP:
        /* 急停安全态: 持续保持电机停 + 舵机轨断电, 直到外部清故障。 */
        motor_stop_all();
        servo_rail_enable(0);
        led_err_set(1);
        /* TODO: 清故障条件满足后 -> led_err_set(0); g_state = APP_ST_IDLE。 */
        break;

    default:
        g_state = APP_ST_ESTOP;   /* 异常态兜底进急停 */
        break;
    }
}

/* ============================================================================
 *  安全 / 外部事件入口
 * ==========================================================================*/

void app_estop(void)
{
    /* 最小动作, 可在中断里调: 切电机 + 断舵机轨 + 置态。耗时操作留给 FSM。 */
    motor_stop_all();        /* 两轮 0 占空 + TB6612 整片待机 */
    servo_rail_enable(0);     /* 断舵机轨(独立可控, 限制⑥) */
    g_state = APP_ST_ESTOP;
}

void app_start_request(void)
{
    g_start_req = 1;   /* 只置标志, 迁移交 app_fsm_step(避免中断里跑业务) */
}

app_state_e app_get_state(void)
{
    return g_state;
}

void app_mode_cycle(void)
{
    /* 只允许待机态换模式(运动中换模式没有安全语义); 1→2→...→APP_MODE_MAX→1 循环 */
    if (g_state == APP_ST_IDLE) {
        g_run_mode = (uint8_t)(g_run_mode % APP_MODE_MAX) + 1u;
    }
}

void app_mode_set(uint8_t mode)
{
    /* 串口直设模式(F4 "串口设定运行模式"); 同样只在待机态生效 */
    if (g_state == APP_ST_IDLE && mode >= 1u && mode <= APP_MODE_MAX) {
        g_run_mode = mode;
    }
}

uint8_t app_get_mode(void)
{
    return g_run_mode;
}

void app_estop_ack(void)
{
    /* 急停软清障: ESTOP 态下 2 秒内连按 3 次 START -> 回待机(红灯灭)。
     * 用"连按三次"作确认动作, 防误触; 清障后执行器仍全停, 需重新 START 才动。 */
    static uint8_t  s_cnt = 0;
    static uint32_t s_first_ms = 0;

    if (g_state != APP_ST_ESTOP) {
        s_cnt = 0;
        return;
    }
    if (s_cnt == 0u || (g_tick_ms - s_first_ms) > 2000u) {
        s_cnt = 1u;                 /* 窗口过期/首次: 重新开窗计数 */
        s_first_ms = g_tick_ms;
        return;
    }
    s_cnt++;
    if (s_cnt >= 3u) {
        s_cnt = 0;
        led_err_set(0);
        g_state = APP_ST_IDLE;      /* 清障回待机; IDLE 态会保持电机停 */
    }
}

void app_tune_step(char which, int dir)
{
    /* 串口在线调参(现场整定免重编译): 'p'=循迹KP ±0.05, 'd'=KD ±0.25, 'v'=基速 ±25mm/s。
     * KP/KD 直接热改 g_pid_track 的增益字段, 下一拍外环立即生效。 */
    float d = (dir >= 0) ? 1.0f : -1.0f;
    if (which == 'p') {
        g_track_kp += 0.5f * d;                       /* r25: SelfTurn KP(占空差/°) */
        if (g_track_kp < 0.0f) g_track_kp = 0.0f;
        g_pid_track.kp = g_track_kp;
    } else if (which == 'd') {
        g_track_kd += 1.0f * d;                       /* r25: SelfTurn KD */
        if (g_track_kd < 0.0f) g_track_kd = 0.0f;
        g_pid_track.kd = g_track_kd;
    } else if (which == 'v') {
        g_base_speed += (dir >= 0) ? 10 : -10;        /* r25: 巡航占空 */
        if (g_base_speed < 60)  g_base_speed = 60;    /* 下限: 太慢过不了弧 */
        if (g_base_speed > 300) g_base_speed = 300;   /* 上限: 60%限幅的一半, 安全帽 */
    } else if (which == 'h') {
        /* 盲走航向锁 KP: 允许负值的调参项(符号本身待现场判定), 夹在 ±10 */
        g_heading_kp += 0.5f * d;
        if (g_heading_kp >  10.0f) g_heading_kp =  10.0f;
        if (g_heading_kp < -10.0f) g_heading_kp = -10.0f;
    } else if (which == 't') {
        /* r24 配平: 右轮占空偏置 ±3/次, 夹 ±40。车右偏按'T', 左偏按't' */
        g_trim_duty += (dir >= 0) ? 3 : -3;
        if (g_trim_duty >  40) g_trim_duty =  40;
        if (g_trim_duty < -40) g_trim_duty = -40;
    }
}

int app_get_trim(void)
{
    return g_trim_duty;   /* 供 VOFA ch23 / '?' 回显; 调好后把值写死进 DRIVE_TRIM_DUTY_R */
}

void app_tune_get(float *kp, float *kd, int *base, float *hkp)
{
    if (kp)   *kp   = g_track_kp;
    if (kd)   *kd   = g_track_kd;
    if (base) *base = g_base_speed;
    if (hkp)  *hkp  = g_heading_kp;
}

void app_get_vel_debug(float *tgt_l, float *meas_l, float *tgt_r, float *meas_r)
{
    /* 只读快照, 给 VOFA 调试波形用(main 的 task_vofa 调)。
     * 读的是内环真正在用的量: 目标速度(track/FSM 写入)与低通后的实测速度, 单位都是 mm/s。
     * 注: 这里不调 enc_get_delta/imu_get_yaw 等有副作用的函数, 不会干扰控制环。 */
    if (tgt_l)  *tgt_l  = g_vel_tgt_l;
    if (meas_l) *meas_l = g_meas_filt[0];
    if (tgt_r)  *tgt_r  = g_vel_tgt_r;
    if (meas_r) *meas_r = g_meas_filt[1];
}

void app_get_vel_out(float *out_l, float *out_r)
{
    /* 只读快照: 两轮当前下发占空(内环累加输出, -1000..1000)。
     * 整定时与实测速度对照看: 占空稳/爬/顶格 分别对应不同的诊断结论。 */
    if (out_l) *out_l = g_vel_out_l;
    if (out_r) *out_r = g_vel_out_r;
}

void app_get_blind_debug(float *lock, float *turn, float *online)
{
    /* 只读快照: 盲走锁定航向 + 最近盲走转向量(压线时 turn=0) + 连续压线时长(ms),
     * 给 VOFA ch20/21/22。复盘判读: ch20(锁定) vs ch12(实际航向) 的差 × HKP 应≈ch21;
     * ch22 锯齿的齿高<600 即"闪断缴械"证据。三线对照当场区分病灶。 */
    if (lock)   *lock   = g_yaw_lock;
    if (turn)   *turn   = g_blind_turn_dbg;
    if (online) *online = (float)g_online_ms;
}

/* ============================================================================
 *  初始化
 * ==========================================================================*/

void app_init(void)
{
    /* ⚠ 占位增益, 必须真车整定(test_plan §B/§C)。
     * 外环位置式: 偏差->转向量; 内环增量式: 速度->占空增量。 */
    pid_init(&g_pid_track, g_track_kp, 0.0f, g_track_kd,   /* 用运行时副本(串口可调) */
             -TRACK_OUT_LIM, TRACK_OUT_LIM, 0.0f);          /* 外环(位置式) */
    pid_init(&g_pid_vel_l, VEL_KP, VEL_KI, VEL_KD,
             VEL_OUT_MIN, VEL_OUT_MAX, VEL_INTEG_MAX);       /* 内环左(增量式) */
    pid_init(&g_pid_vel_r, VEL_KP, VEL_KI, VEL_KD,
             VEL_OUT_MIN, VEL_OUT_MAX, VEL_INTEG_MAX);       /* 内环右(增量式) */

    g_vel_tgt_l = g_vel_tgt_r = 0.0f;
    g_vel_out_l = g_vel_out_r = 0.0f;
    g_start_req = 0;
    g_aim_timer = 0;
    g_lost_dist = 0;
    g_kp_idx = 0;
    g_run_beep_ms = 0;
    g_aim_return_run = 0;
    /* g_run_mode 不复位: 保持用户选择(app_init 只在上电跑一次, 默认 1=F1) */
    g_meas_filt[0] = g_meas_filt[1] = 0.0f;
    g_blind_active = 0;
    g_online_ms = 0;
    g_acq_cnt = 0;
    g_lock_valid = 0;
    g_pivot_ms = 0;
    g_node_pending = 0;
    g_node_hit_fired = 0;
    g_head_integ = 0.0f;
    g_align_ms = 0;
    g_seg = 0;
    g_nodata = 0xFF;
    g_datum_yaw = 0.0f;
    g_protect_ms = 0;
    g_startprot_ms = 0;
    g_dyaw_prev = 0.0f;
    g_blind_ref = 0;   /* (遗留字段, 保持归零) */
    g_stop_beep_ms = 0;
    g_prev_state = APP_ST_IDLE;
    g_state = APP_ST_IDLE;
}
