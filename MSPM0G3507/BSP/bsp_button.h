#ifndef __BSP_BUTTON_H
#define __BSP_BUTTON_H

#include "ti_msp_dl_config.h"

/* ================================================================
 * 通用按键驱动 (BSP 层)
 *
 * 特性:
 *   - 支持任意 GPIO 引脚复用为按键
 *   - 可配置高/低电平触发
 *   - 软件消抖 (状态机, 无需硬件定时器)
 *   - 提供电平触发 (is_pressed) 和边沿触发 (is_clicked / is_released)
 *
 * 依赖:
 *   - mid_delay.h 中的 get_system_ms() 提供时基
 *   - 调用者需周期性调用 button_update(), 建议周期 5~10ms
 *
 * 分层:
 *   Application  ──►  bsp_button  ──►  DL_GPIO (TI SDK)
 *                          │
 *                      mid_delay (时基)
 * ================================================================ */

/* ---- 消抖参数 ---- */

/** 消抖采样间隔 (ms), 也是调用 button_update() 的建议周期 */
#define BUTTON_DEBOUNCE_MS      5U

/** 连续稳定采样次数阈值 (达到此值才确认状态变化)
 *  实际消抖时间 = BUTTON_DEBOUNCE_MS × BUTTON_STABLE_COUNT
 *  默认 5ms × 4 = 20ms */
#define BUTTON_STABLE_COUNT     4U

/* ---- 按键触发电平 ---- */

typedef enum {
    BUTTON_ACTIVE_LOW  = 0,   /**< 低电平触发 (按下为低) */
    BUTTON_ACTIVE_HIGH = 1,   /**< 高电平触发 (按下为高) */
} Button_ActiveLevel;

/* ---- 按键句柄 (每个按键一个实例) ---- */

typedef struct {
    /* ---- 用户配置 (通过 button_init 设置) ---- */
    GPIO_Regs        *port;          /**< GPIO 端口寄存器 */
    uint32_t          pin;           /**< GPIO 引脚掩码 */
    Button_ActiveLevel active_level; /**< 有效触发电平 */

    /* ---- 内部状态 (由 button_update 维护, 用户只读) ---- */
    uint8_t           pressed;       /**< 当前消抖后的稳定状态: 1=按下, 0=释放 */
    uint8_t           clicked;       /**< 边沿标记: 本周期检测到按下事件 (读后自动清零) */
    uint8_t           released;      /**< 边沿标记: 本周期检测到释放事件 (读后自动清零) */

    /* ---- 消抖状态机 (私有, 勿直接修改) ---- */
    uint32_t          last_sample_tick; /**< 上次采样时刻 (ms) */
    uint8_t           raw_stable;       /**< 当前正在确认的原始电平 */
    uint8_t           stable_cnt;       /**< 连续稳定计数 */
} Button;

/* ================================================================
 *  API
 * ================================================================ */

/**
 * @brief  初始化按键句柄
 * @param  btn          按键句柄指针
 * @param  port         GPIO 端口 (如 GPIOA, GPIOB)
 * @param  pin          GPIO 引脚掩码 (如 DL_GPIO_PIN_15)
 * @param  active_level 有效触发电平 (BUTTON_ACTIVE_HIGH 或 BUTTON_ACTIVE_LOW)
 * @note   调用时机: SYSCFG_DL_init() 之后, 主循环之前
 *
 * 示例:
 * @code
 *   Button btn_user;
 *   button_init(&btn_user, GPIOB, DL_GPIO_PIN_15, BUTTON_ACTIVE_HIGH);
 * @endcode
 */
void button_init(Button *btn, GPIO_Regs *port, uint32_t pin,
                 Button_ActiveLevel active_level);

/**
 * @brief  按键状态机更新 (消抖采样)
 * @param  btn 按键句柄指针
 * @note   需周期性调用, 建议周期 5~10ms.
 *         调用频率过低会导致消抖时间变长、边沿事件丢失.
 *
 * 典型用法 (在 while(1) 中):
 * @code
 *   while (1) {
 *       button_update(&btn_user);
 *       if (button_is_clicked(&btn_user)) {
 *           // 处理按键按下事件 ...
 *       }
 *       delay_ms(5);  // 与 BUTTON_DEBOUNCE_MS 一致
 *   }
 * @endcode
 */
void button_update(Button *btn);

/**
 * @brief  查询按键是否处于按下状态 (电平触发)
 * @param  btn 按键句柄指针
 * @return 1 = 按下, 0 = 释放
 * @note   消抖后的稳定状态, 可多次读取
 */
uint8_t button_is_pressed(const Button *btn);

/**
 * @brief  查询是否有按下事件 (边沿触发, 释放→按下)
 * @param  btn 按键句柄指针
 * @return 1 = 本次查询到按下边沿, 0 = 无
 * @note   读取后自动清零, 同一边沿不会重复触发
 */
uint8_t button_is_clicked(Button *btn);

/**
 * @brief  查询是否有释放事件 (边沿触发, 按下→释放)
 * @param  btn 按键句柄指针
 * @return 1 = 本次查询到释放边沿, 0 = 无
 * @note   读取后自动清零, 同一边沿不会重复触发
 */
uint8_t button_is_released(Button *btn);

/**
 * @brief  读取按键原始 GPIO 电平 (不经消抖)
 * @param  btn 按键句柄指针
 * @return 原始引脚电平 (0 或 1)
 * @note   用于调试或特殊场景, 常规使用请用消抖后的 API
 */
uint8_t button_read_raw(const Button *btn);

#endif /* __BSP_BUTTON_H */
