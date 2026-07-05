/* hmi —— 人机交互三合一驱动（蜂鸣器 + 状态灯 + 按键）实现 —— firmware-scaffold driver 层
 *
 * 引脚 / SysConfig 实例宏见 hmi.h 顶部。本文件只调 DriverLib 的:
 *   DL_GPIO_setPins / DL_GPIO_clearPins / DL_GPIO_togglePins / DL_GPIO_readPins
 * （均已在 C:/TI/mspm0_sdk_2_10_00_04/source/ti/driverlib/dl_gpio.h 核实签名:
 *    void setPins/clearPins/togglePins(GPIO_Regs* gpio, uint32_t pins);
 *    uint32_t readPins(GPIO_Regs* gpio, uint32_t pins)  —— 返回入参 pins 上的当前电平,
 *    某位=1 表示该脚为高电平。我们对单脚读, 故结果非 0 即为高、为 0 即为低。）
 *
 * 按键消抖采用"连续 N 次采样一致才认状态翻转"的积分式消抖, 比单次延时更抗抖动毛刺。
 * 检出"由松到按"的下降沿(高->低)记一次按下事件, app 用 key_pressed() 取走(取走即清零),
 * 这样即使两次查询之间发生过按下也不会丢, 也不会因长按重复触发。
 */
#include "ti_msp_dl_config.h"
#include "hmi.h"

/* ===== 调参常量（消抖）===== */
/* 消抖深度: 需连续这么多次采样读到同一电平, 才确认按键状态发生翻转。
 * 实际去抖时间 ≈ KEY_DEBOUNCE_CNT × key_scan() 调用周期。
 * 例: 周期 10ms × 3 = 30ms, 一般轻触按键足够。 */
#define KEY_DEBOUNCE_CNT  3        /* 待整定: 消抖一致采样次数(配合 key_scan 调用周期) */

/* ===== 蜂鸣器电平定义（有源低触发）=====
 * 拉低=响, 拉高=静音。集中成宏, 万一换成高触发蜂鸣器只改这两行。 */
#define BUZZER_ACTIVE()   DL_GPIO_clearPins(GPIO_IOB_PORT, GPIO_IOB_BUZZER_PIN)  /* 拉低=响 */
#define BUZZER_SILENT()   DL_GPIO_setPins(GPIO_IOB_PORT, GPIO_IOB_BUZZER_PIN)    /* 拉高=静音 */

/* ===== 按键运行时状态 ===== */
/* 每个按键一份: 去抖计数 + 已确认的稳定状态 + 待取走的"按下事件"标志。 */
typedef struct {
    GPIO_Regs *port;     /**< 该键所在 GPIO 组端口(两键跨组, 各存各的) */
    uint32_t   pin;      /**< 该键引脚掩码 */
    uint8_t    cnt;      /**< 当前电平连续一致的采样计数(到阈值才翻转 stable) */
    uint8_t    stable;   /**< 已确认的稳定状态: 1=按下(低电平) 0=松开(高电平) */
    uint8_t    last;     /**< 上一次确认的稳定状态, 用于检出"由松到按"边沿 */
    uint8_t    event;    /**< 待取走的按下事件: 1=发生过一次按下, key_pressed 读后清 0 */
} key_t;

/* 下标必须与 hmi.h 的 KEY_START / KEY_MODE 对应。port/pin 在 hmi_init 里填(用 SysConfig 宏)。 */
static key_t s_keys[HMI_KEY_NUM];

/**
 * @brief 初始化人机交互外设(蜂鸣器静音、状态灯灭、按键状态清零)
 * @note  必须在 SYSCFG_DL_init() 之后调用——此前各脚的 IOMUX/初值尚未配好。
 *        蜂鸣器有源低触发, 这里第一时间拉高静音, 防上电就鸣。
 */
void hmi_init(void)
{
    /* 蜂鸣器: 先静音(拉高), 兜住上电误鸣 */
    BUZZER_SILENT();

    /* 运行灯 / 错误灯: 上电先灭(按高=亮处理, 故拉低) */
    DL_GPIO_clearPins(GPIO_IOA_PORT, GPIO_IOA_LED_RUN_PIN);   /* PA24 灭 */
    DL_GPIO_clearPins(GPIO_IOA_PORT, GPIO_IOA_LED_ERR_PIN);   /* PA26 灭 */

    /* 按键表: 绑定各自端口/引脚, 状态按"松开(高电平)"初始化, 清掉残留事件 */
    s_keys[KEY_START].port = GPIO_IOB_PORT;                   /* PB19 在 GPIOB 组 */
    s_keys[KEY_START].pin  = GPIO_IOB_KEY_START_PIN;
    s_keys[KEY_MODE].port  = GPIO_IOA_PORT;                   /* PA22 在 GPIOA 组 */
    s_keys[KEY_MODE].pin   = GPIO_IOA_KEY_MODE_PIN;
    for (int i = 0; i < HMI_KEY_NUM; i++) {
        s_keys[i].cnt    = 0;
        s_keys[i].stable = 0;   /* 0=松开 */
        s_keys[i].last   = 0;
        s_keys[i].event  = 0;
    }
}

/**
 * @brief 蜂鸣器响
 * @note  有源低触发: 拉低 PB18 即出声(蜂鸣器自带振荡, 无需 PWM)。
 */
void beep_on(void)
{
    BUZZER_ACTIVE();   /* 拉低 PB18 = 响 */
}

/**
 * @brief 蜂鸣器静音
 * @note  拉高 PB18 关声。
 */
void beep_off(void)
{
    BUZZER_SILENT();   /* 拉高 PB18 = 静音 */
}

/**
 * @brief 用布尔量驱动蜂鸣器(便于 app 直接传开关量)
 * @param on 非 0 = 响, 0 = 静音
 */
void beep_set(int on)
{
    if (on) BUZZER_ACTIVE();   /* 拉低=响 */
    else    BUZZER_SILENT();   /* 拉高=静音 */
}

/**
 * @brief 设置运行灯(PA24)亮灭
 * @param on 非 0 = 亮, 0 = 灭
 * @note  此处按"高电平=亮"驱动; 若实测为低有效, 把下面 set/clear 对调即可。
 */
void led_run_set(int on)
{
    if (on) DL_GPIO_setPins(GPIO_IOA_PORT, GPIO_IOA_LED_RUN_PIN);     /* 拉高=亮 */
    else    DL_GPIO_clearPins(GPIO_IOA_PORT, GPIO_IOA_LED_RUN_PIN);   /* 拉低=灭 */
}

/**
 * @brief 设置错误灯(PA26)亮灭
 * @param on 非 0 = 亮, 0 = 灭
 * @note  此处按"高电平=亮"驱动; 若实测为低有效, 把下面 set/clear 对调即可。
 */
void led_err_set(int on)
{
    if (on) DL_GPIO_setPins(GPIO_IOA_PORT, GPIO_IOA_LED_ERR_PIN);     /* 拉高=亮 */
    else    DL_GPIO_clearPins(GPIO_IOA_PORT, GPIO_IOA_LED_ERR_PIN);   /* 拉低=灭 */
}

/**
 * @brief 翻转心跳灯(PB26)
 * @note  放心跳任务里周期调用(如 500ms 一次 => 1Hz 慢闪), 直观表示"程序在跑没卡死"。
 */
void led_heart_toggle(void)
{
    DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_HEART_PIN);   /* 翻转 PB26 */
}

/**
 * @brief 按键消抖扫描(周期调用, 建议 10ms)
 * @note  积分式消抖: 同一电平连续读到 KEY_DEBOUNCE_CNT 次才确认状态翻转;
 *        确认出"由松到按"(高->低)的下降沿时, 记一次按下事件供 key_pressed 取走。
 *        必须周期调用——单次调用不消抖。
 */
void key_scan(void)
{
    for (int i = 0; i < HMI_KEY_NUM; i++) {
        key_t *k = &s_keys[i];

        /* 读当前电平: 内部上拉, 低=按下。readPins 对单脚返回非 0 即为高电平 */
        uint8_t pressed_now = (DL_GPIO_readPins(k->port, k->pin) == 0) ? 1 : 0;

        if (pressed_now == k->stable) {
            k->cnt = 0;          /* 与已确认状态一致, 无需翻转, 计数清零 */
        } else {
            /* 与已确认状态不同, 累计一致采样; 够阈值才真正翻转, 滤掉短抖动 */
            if (++k->cnt >= KEY_DEBOUNCE_CNT) {
                k->stable = pressed_now;   /* 确认新稳定状态 */
                k->cnt    = 0;
                /* 检出"由松(0)到按(1)"下降沿, 记一次按下事件 */
                if (k->stable && !k->last) {
                    k->event = 1;
                }
                k->last = k->stable;
            }
        }
    }
}

/**
 * @brief 查询并清除某键的"按下事件"(边沿, 读后即清)
 * @param key 按键编号: KEY_START 或 KEY_MODE
 * @return 1 = 自上次查询以来发生过一次按下(已被本次读取清除); 0 = 无新按下
 * @note  事件是"按下边沿"语义: 长按只触发一次, 不会重复; 非法 key 返回 0。
 */
int key_pressed(int key)
{
    if (key < 0 || key >= HMI_KEY_NUM) return 0;   /* 非法编号: 防越界读, 返回无事件 */

    if (s_keys[key].event) {
        s_keys[key].event = 0;   /* 取走即清, 保证一次按下只被消费一次 */
        return 1;
    }
    return 0;
}
