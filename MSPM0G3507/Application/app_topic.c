#include "app.h"
#include "bsp_OLED.h"
#include "bsp_button.h"
#include "bsp_zdt_x35.h"
#include "drv_uart.h"
#include "drv_zdt_x35_uart.h"
#include "mid_delay.h"

#define TOPIC_COUNT 6

uint8_t topicselect = 1;
static Button btn_next;    /* KEY1: 切换 1-6 */
static Button btn_enter;   /* KEY2: 确认执行 */

static void topic_show_menu(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "Select Topic:");
    OLED_ShowNum(3, 1, topicselect, 1);
    OLED_ShowString(4, 1, "KEY1:Next KEY2:OK");
}

void topic(void)
{
    OLED_Init();
    button_init(&btn_next,  KEY_PORT, KEY_KEY1_PIN, BUTTON_ACTIVE_HIGH);
    button_init(&btn_enter, KEY_PORT, KEY_KEY2_PIN, BUTTON_ACTIVE_HIGH);

    while (1) {
        topic_show_menu();

        /* 等待按键，10ms 轮询消抖 */
        while (1) {
            button_update(&btn_next);
            button_update(&btn_enter);

            if (button_is_clicked(&btn_next)) {
                topicselect++;
                if (topicselect > TOPIC_COUNT) topicselect = 1;
                break;  /* 刷新 OLED */
            }
            if (button_is_clicked(&btn_enter)) {
                /* 执行选定题目，结束后返回菜单 */
                switch (topicselect) {
                case 1: topic1(); break;
                case 2: topic2(); break;
                case 3: topic3(); break;
                case 4: topic4(); break;
                case 5: topic5(); break;
                case 6: topic6(); break;
                }
                break;  /* topic 返回后回到菜单 */
            }

            delay_ms(10);
        }
    }
}



void topic1(void) {}

void topic6(void) {}