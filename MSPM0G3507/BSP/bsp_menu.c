#include "bsp_menu.h"
#include "bsp_OLED.h"
#include "mid_delay.h"

/* ---- 按键扫描 ---- */

static uint8_t getnum(void)
{
    static uint8_t db1 = 0, db2 = 0;
    const  uint8_t TH  = 5;

    /* KEY1 — 高电平有效 */
    if (DL_GPIO_readPins(KEY_PORT, KEY_KEY1_PIN) > 0) {
        if (++db1 >= TH) { db1 = 0; return 1; }
    } else {
        db1 = 0;
    }

    /* KEY2 — 高电平有效 */
    if (DL_GPIO_readPins(KEY_PORT, KEY_KEY2_PIN) > 0) {
        if (++db2 >= TH) { db2 = 0; return 2; }
    } else {
        db2 = 0;
    }

    return 0;
}

/* ---- 辅助: 刷新菜单行 (用 ">" 前缀标记选中项) ---- */
static void menu_draw_row(uint8_t line, uint8_t sel, const char *text)
{
    if (sel) {
        OLED_ShowString(line, 1, ">");
    } else {
        OLED_ShowString(line, 1, " ");
    }
    OLED_ShowString(line, 2, (char *)text);
}

/* ---- 辅助: 绘制整屏菜单 ---- */
static void menu_draw_all(uint8_t sel,
                          const char *t0, const char *t1,
                          const char *t2, const char *t3)
{
    OLED_Clear();
    menu_draw_row(1, sel == 1, t0);
    menu_draw_row(2, sel == 2, t1);
    menu_draw_row(3, sel == 3, t2);
    menu_draw_row(4, sel == 4, t3);
}

/* ================================================================
 *  menu0 — 一级菜单: FIRST / SECOND / THIRD / FOURTH
 * ================================================================ */
int menu0(void)
{
    uint8_t flag = 1;
    uint8_t prev = 0;

    menu_draw_all(flag, "FIRST", "SECOND", "THIRD", "FOURTH");

    while (1) {
        uint8_t k = getnum();

        if (k == 1) {          /* KEY1 — 下一项 */
            flag++;
            if (flag > 4) flag = 1;
        }
        if (k == 2) {          /* KEY2 — 确认选择 */
            return flag;
        }

        /* 选项变化时刷新显示 */
        if (flag != prev) {
            prev = flag;
            menu_draw_all(flag, "FIRST", "SECOND", "THIRD", "FOURTH");
        }
    }
}

/* ================================================================
 *  menu1 — FIRST: 时钟/日期显示 (占位)
 * ================================================================ */
int menu1(void)
{
    uint8_t flag = 1;
    uint8_t prev = 0;

    OLED_Clear();
    OLED_ShowString(1, 1, ">2026");
    OLED_ShowString(2, 1, " 1/14");
    OLED_ShowString(3, 1, " 18:00");
    OLED_ShowString(4, 1, " <Back");

    while (1) {
        uint8_t k = getnum();

        if (k == 1) {
            flag = !flag;
        }
        if (k == 2) {
            if (flag == 1) { OLED_Clear(); return 0; }
        }

        if (flag != prev) {
            prev = flag;
            OLED_Clear();
            OLED_ShowString(1, 1, ">2026");
            OLED_ShowString(2, 1, " 1/14");
            OLED_ShowString(3, 1, " 18:00");
            OLED_ShowString(4, 1, flag ? " <Back" : "> Back");
        }
    }
}

/* ================================================================
 *  menu2_1 — PID 参数编辑
 * ================================================================ */
int menu2_1(void)
{
    uint8_t  flag = 1;
    uint8_t  prev = 0;
    uint16_t p = 0, i = 0, d = 0;

    OLED_Clear();
    OLED_ShowString(1, 1, ">Back");
    OLED_ShowString(2, 1, " P:0");
    OLED_ShowString(3, 1, " I:0");
    OLED_ShowString(4, 1, " D:0");

    while (1) {
        uint8_t k = getnum();

        if (k == 1) {
            flag++;
            if (flag > 4) flag = 1;
        }
        if (k == 2) {
            if (flag == 1)      { OLED_Clear(); return 0; }
            else if (flag == 2) { p += 10; }
            else if (flag == 3) { i += 10; }
            else if (flag == 4) { d += 10; }
        }

        if (flag != prev) {
            prev = flag;
            OLED_Clear();
            menu_draw_row(1, flag == 1, "Back");
            OLED_ShowString(2, 1, (flag == 2) ? ">" : " ");
            OLED_ShowString(2, 2, "P:");
            OLED_ShowNum(2, 4, p, 3);
            OLED_ShowString(3, 1, (flag == 3) ? ">" : " ");
            OLED_ShowString(3, 2, "I:");
            OLED_ShowNum(3, 4, i, 3);
            OLED_ShowString(4, 1, (flag == 4) ? ">" : " ");
            OLED_ShowString(4, 2, "D:");
            OLED_ShowNum(4, 4, d, 3);
        }
        /* 实时刷新数值 */
        OLED_ShowNum(2, 4, p, 3);
        OLED_ShowNum(3, 4, i, 3);
        OLED_ShowNum(4, 4, d, 3);
    }
}

/* ================================================================
 *  menu2 — SECOND: PID / MPU6050 / MSPM0
 * ================================================================ */
int menu2(void)
{
    uint8_t flag = 1;
    uint8_t prev = 0;

    menu_draw_all(flag, "Back", "PID", "mpu6050", "MSPM0");

    while (1) {
        uint8_t k = getnum();

        if (k == 1) {
            flag++;
            if (flag > 4) flag = 1;
        }
        if (k == 2) {
            OLED_Clear();
            if (flag == 1)      { return 0; }
            else if (flag == 2) { menu2_1(); }
            else if (flag == 3) { /* mpu6050 test */ }
            else if (flag == 4) { /* MSPM0 info   */ }
        }

        if (flag != prev) {
            prev = flag;
            menu_draw_all(flag, "Back", "PID", "mpu6050", "MSPM0");
        }
    }
}

/* ================================================================
 *  menu3 — THIRD
 * ================================================================ */
int menu3(void)
{
    uint8_t flag = 1;
    uint8_t prev = 0;

    menu_draw_all(flag, "Back", "Item 2", "Item 3", "Item 4");

    while (1) {
        uint8_t k = getnum();

        if (k == 1) {
            flag++;
            if (flag > 4) flag = 1;
        }
        if (k == 2) {
            OLED_Clear();
            if (flag == 1) { return 0; }
            /* items 2-4: return to main menu */
            return 0;
        }

        if (flag != prev) {
            prev = flag;
            menu_draw_all(flag, "Back", "Item 2", "Item 3", "Item 4");
        }
    }
}

/* ================================================================
 *  menu4 — FOURTH
 * ================================================================ */
int menu4(void)
{
    uint8_t flag = 1;
    uint8_t prev = 0;

    menu_draw_all(flag, "( 1 )", "( 2 )", "( 3 )", "Exit");

    while (1) {
        uint8_t k = getnum();

        if (k == 1) {
            flag++;
            if (flag > 4) flag = 1;
        }
        if (k == 2) {
            OLED_Clear();
            if (flag == 1) {
                OLED_ShowString(2, 2, "BMP-1 N/A");
                delay_ms(1000);
            } else if (flag == 2) {
                OLED_ShowString(2, 2, "BMP-2 N/A");
                delay_ms(1000);
            } else if (flag == 3) {
                OLED_ShowString(2, 2, "BMP-3 N/A");
                delay_ms(1000);
            } else if (flag == 4) {
                return 0;
            }
        }

        if (flag != prev) {
            prev = flag;
            menu_draw_all(flag, "( 1 )", "( 2 )", "( 3 )", "Exit");
        }
    }
}
