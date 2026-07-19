#include "bsp_button.h"
#include "mid_delay.h"

/* ================================================================
 * 通用按键驱动 — 实现
 *
 * 消抖原理:
 *   周期性 (~5ms) 采样 GPIO 电平, 当连续 BUTTON_STABLE_COUNT 次
 *   读到同一电平时, 才确认状态变化。机械抖动通常在 5~15ms 内结束,
 *   20ms 的消抖窗口足够覆盖绝大多数轻触开关。
 *
 * 状态机:
 *   IDLE ──(采样变化)──► DEBOUNCE ──(稳定N次)──► STATE_CHANGE
 *                              │
 *                        (中途抖动)
 *                              │
 *                       重置计数, 重采
 * ================================================================ */

/**
 * @brief  读取 GPIO 原始电平并映射到有效电平
 * @param  btn 按键句柄
 * @return 1 = 有效 (按下), 0 = 无效 (释放)
 */
static uint8_t button_sample_raw(const Button *btn)
{
    uint32_t raw;

    raw = DL_GPIO_readPins(btn->port, btn->pin);

    if (btn->active_level == BUTTON_ACTIVE_HIGH) {
        return (raw != 0U) ? 1U : 0U;
    } else {
        /* BUTTON_ACTIVE_LOW: 读到 0 表示按下 */
        return (raw == 0U) ? 1U : 0U;
    }
}

/**
 * @brief  初始化按键句柄
 */
void button_init(Button *btn, GPIO_Regs *port, uint32_t pin,
                 Button_ActiveLevel active_level)
{
    btn->port         = port;
    btn->pin          = pin;
    btn->active_level = active_level;

    /* 读取当前物理电平作为初始稳定状态 */
    btn->raw_stable   = button_sample_raw(btn);
    btn->pressed      = btn->raw_stable;
    btn->stable_cnt   = BUTTON_STABLE_COUNT;  /* 初始状态直接确认 */

    btn->clicked      = 0U;
    btn->released     = 0U;
    btn->last_sample_tick = get_system_ms();
}

/**
 * @brief  按键状态机更新
 *
 *         消抖采样 + 边沿检测。需周期性调用 (建议 5ms)。
 */
void button_update(Button *btn)
{
    uint32_t now;
    uint8_t  raw;
    uint8_t  prev_pressed;

    now = get_system_ms();

    /* 消抖采样间隔检查 */
    if ((now - btn->last_sample_tick) < BUTTON_DEBOUNCE_MS) {
        return;
    }
    btn->last_sample_tick = now;

    /* 读取当前原始电平 */
    raw = button_sample_raw(btn);

    /* 消抖状态机 */
    if (raw == btn->raw_stable) {
        /* 与上次稳定值一致, 累计计数 */
        if (btn->stable_cnt < BUTTON_STABLE_COUNT) {
            btn->stable_cnt++;
        }

        /* 达到稳定阈值, 确认新状态 */
        if ((btn->stable_cnt >= BUTTON_STABLE_COUNT) &&
            (btn->pressed != btn->raw_stable)) {

            prev_pressed   = btn->pressed;
            btn->pressed   = btn->raw_stable;

            /* 边沿检测: 记录事件 (不清零, 等用户读取) */
            if ((btn->pressed == 1U) && (prev_pressed == 0U)) {
                btn->clicked = 1U;   /* 释放 → 按下 */
            } else if ((btn->pressed == 0U) && (prev_pressed == 1U)) {
                btn->released = 1U;  /* 按下 → 释放 */
            }
        }
    } else {
        /* 电平波动 (抖动), 重置消抖计数, 更新参考值 */
        btn->raw_stable = raw;
        btn->stable_cnt = 1U;
    }
}

/**
 * @brief  查询按键是否按下 (消抖后电平)
 */
uint8_t button_is_pressed(const Button *btn)
{
    return btn->pressed;
}

/**
 * @brief  查询按下边沿事件 (读后清零)
 */
uint8_t button_is_clicked(Button *btn)
{
    uint8_t val;

    val = btn->clicked;
    if (val != 0U) {
        btn->clicked = 0U;
    }
    return val;
}

/**
 * @brief  查询释放边沿事件 (读后清零)
 */
uint8_t button_is_released(Button *btn)
{
    uint8_t val;

    val = btn->released;
    if (val != 0U) {
        btn->released = 0U;
    }
    return val;
}

/**
 * @brief  读取按键原始 GPIO 电平 (不经消抖)
 */
uint8_t button_read_raw(const Button *btn)
{
    return button_sample_raw(btn);
}
