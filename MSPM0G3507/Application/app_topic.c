#include "app.h"
#include "bsp_zdt_x35.h"
#include "drv_uart.h"
#include "drv_zdt_x35_uart.h"
#include "mid_delay.h"

void topic1(void)
{

}


void topic2(void)
{

}

void topic3(void)
{

    /**初始化**/
    drv_uart0_init();
    zdt_x35_init();

    /*使能*/
    zdt_x35_set_enable(true, false);
    delay_ms(500);

    /*开始计时*/

    /*开始运动*/
    app_zdt_x35_move_down(2999, 17000);
     delay_ms(1800);
     app_zdt_x35_move_up(2999, 10800*2);
     delay_ms(2150);

    app_zdt_x35_move_down(2999, 8800);
    delay_ms(4000);

    /**关闭使能**/
    zdt_x35_set_enable(false, false);

}

void topic4(void)
{

}
   
void topic5(void)
{

}

void topic6(void)
{

}