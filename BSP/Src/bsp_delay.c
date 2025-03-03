#include "bsp_delay.h"
#include "main.h"

// 估算每个NOP指令的时钟周期数，通常为1个时钟周期
#define NOP_CYCLES 1 // 每个NOP指令占用的时钟周期数，假设为1


void delay_init(void)
{
}

void delay_us(uint16_t nus)
{
    // 计算需要的NOP次数
    uint32_t nop_count = nus * (SystemCoreClock / 1000000) * NOP_CYCLES;

    // 执行NOP指令实现延时
    for (uint32_t i = 0; i < nop_count; i++) {
        __NOP();  // 执行No Operation指令
    }
}

void delay_ms(uint16_t nms)
{
    // 计算需要的NOP次数
    uint32_t nop_count = nms * (SystemCoreClock / 1000) * NOP_CYCLES;

    // 执行NOP指令实现延时
    for (uint32_t i = 0; i < nop_count; i++) {
        __NOP();  // 执行No Operation指令
    }
}



//#include "main.h"

//static uint8_t fac_us = 0;
//static uint32_t fac_ms = 0;

//void delay_init(void)
//{
//    fac_us = SystemCoreClock / 1000000;
//    fac_ms = SystemCoreClock / 1000;

//}

//void delay_us(uint16_t nus)
//{
//    uint32_t ticks = 0;
//    uint32_t told = 0;
//    uint32_t tnow = 0;
//    uint32_t tcnt = 0;
//    uint32_t reload = 0;
//    reload = SysTick->LOAD;
//    ticks = nus * fac_us;
//    told = SysTick->VAL;
//    while (1)
//    {
//        tnow = SysTick->VAL;
//        if (tnow != told)
//        {
//            if (tnow < told)
//            {
//                tcnt += told - tnow;
//            }
//            else
//            {
//                tcnt += reload - tnow + told;
//            }
//            told = tnow;
//            if (tcnt >= ticks)
//            {
//                break;
//            }
//        }
//    }
//}

//void delay_ms(uint16_t nms)
//{
//    uint32_t ticks = 0;
//    uint32_t told = 0;
//    uint32_t tnow = 0;
//    uint32_t tcnt = 0;
//    uint32_t reload = 0;
//    reload = SysTick->LOAD;
//    ticks = nms * fac_ms;
//    told = SysTick->VAL;
//    while (1)
//    {
//        tnow = SysTick->VAL;
//        if (tnow != told)
//        {
//            if (tnow < told)
//            {
//                tcnt += told - tnow;
//            }
//            else
//            {
//                tcnt += reload - tnow + told;
//            }
//            told = tnow;
//            if (tcnt >= ticks)
//            {
//                break;
//            }
//        }
//    }
//}
