#include "app.h"
#include "bsp_zdt_x35.h"
#include "drv_uart.h"
#include "drv_zdt_x35_uart.h"
#include "mid_delay.h"

uint16_t target_x=400;
uint16_t target_y=400;

void topic4(void)
{
    drv_vision_uart3_init();
    zdt_x35_init();


    
}