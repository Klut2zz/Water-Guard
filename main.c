/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2020/04/30
* Description        : Main program body.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "ch32v30x.h"
#include <rtthread.h>
#include <rthw.h>
#include "drivers/pin.h"
#include "rtdevice.h"
#include "lcd.h"
#include "string.h"
#include "ch32v30x_rng.h"
//#include "font.h"
/* Global typedef */

#define  USART3_MAX_RECV_LEN 4
/* Global define */

/* Global Variable */
#define LED0_PIN  18   //PC3
#define SW1_PIN   3    //PE4
#define SW2_PIN   4    //PE5
#define LED1_PIN  42   //PE11
#define WAKE_UP_PIN 23 //PA0
#define PD11_PIN 58
#define PD10_PIN 57
#define PD3_PIN 84
#define PE15_PIN 46
static rt_timer_t timer1;
static int cnt = 0;
rt_uint8_t num1,num2,num3;
rt_uint8_t pump_status = 0;
/*********************************************************************
 * @fn      LED1_BLINK_INIT
 *
 * @brief   LED1通过直接调用底层驱动
 *
 * @return  none
 */
void LED1_BLINK_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void Init_UART(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    // 开时钟（USART1 TX-A9 RX-A10）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);

    // 初始化GPIO
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    // 初始化USART
    USART_InitStruct.USART_BaudRate = bound;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStruct);

    // 使能串口
    USART_Cmd(USART3, ENABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);

   // 初始化GPIO
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStruct);
   GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
   GPIO_Init(GPIOA, &GPIO_InitStruct);

   // 初始化USART
   USART_InitStruct.USART_BaudRate = 115200;
   USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_InitStruct.USART_Parity = USART_Parity_No;
   USART_InitStruct.USART_StopBits = USART_StopBits_1;
   USART_InitStruct.USART_WordLength = USART_WordLength_8b;
   USART_Init(USART1, &USART_InitStruct);

   // 使能串口
   USART_Cmd(USART1, ENABLE);

}

void UART_SendByte(USART_TypeDef * USARTx,uint8_t data)
{
    USART_SendData(USARTx, data);
    while(!USART_GetFlagStatus(USARTx, USART_FLAG_TXE));
}
void UART_SendString(USART_TypeDef * USARTx,unsigned char* string)
{
    unsigned char *str = string;

        UART_SendByte(USARTx,str[0]);
        //printf("%x ",str[0]);

}

u8 USART3_RX_BUF[USART3_MAX_RECV_LEN];

/*********************************************************************
 * @fn      main
 *
 * @brief   main只是一个线程之一，除此之外还有tshell,idle
 *          本main只是一个LED闪烁，main线程的注册在rtthread_startup中，tshell使用了串口
 *          接收中断，中断栈和线程栈使用分开.
 *
 * @return  none
 */
int main(void)
{
    rt_kprintf("\r\n MCU: CH32V307\r\n");
    rt_kprintf(" SysClk: %dHz\r\n",SystemCoreClock);
    rt_kprintf(" www.wch.cn\r\n");
    LED1_BLINK_INIT();
    GPIO_SetBits(GPIOE, GPIO_Pin_11);
    GPIO_ResetBits(GPIOA,GPIO_Pin_0);

}

void open(void){
    rt_pin_mode(PE15_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(PE15_PIN, PIN_HIGH);
    pump_status = 1;

}
void close(void){
    rt_pin_mode(PE15_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(PE15_PIN, PIN_LOW);
    pump_status = 0;
}
static void timeout1(void *parameter)
{

    rt_pin_mode(PD3_PIN, PIN_MODE_INPUT_PULLDOWN);
    rt_pin_mode(PD10_PIN, PIN_MODE_INPUT_PULLDOWN);
    rt_pin_mode(PD11_PIN, PIN_MODE_INPUT_PULLDOWN);

    num1 = rt_pin_read(PD3_PIN);
    num2 = rt_pin_read(PD10_PIN);
    num3 = rt_pin_read(PD11_PIN);
    if(num1==0&&num2==0&&num3==0)
    {
        rt_kprintf("close\n");
        close();
    }else if(num1==0&&num2==1&&num3==1)
    {
        rt_kprintf("open\n");
        open();
    }else if(num1==1&&num2==1&&num3==1)
    {
        rt_kprintf("wrong!!\n");
        open();
        lcd_init();
        lcd_fill(0,0,239,239,RED);
        lcd_show_string(60, 120, 32, "WARNING!!");
    }
    rt_kprintf("level1\r\tlevel2\tlevel3\n");
    rt_kprintf("%d\r\t%d\t%d\n" , num1,num2,num3);

    if (cnt++ >= 12000)
    {
        rt_timer_stop(timer1);
        close();
        rt_kprintf("periodic timer was stopped! \n");
    }
}
int levelcontrol(void){
    cnt = 0;
    timer1 = rt_timer_create("timer1", timeout1, RT_NULL, 1000, RT_TIMER_FLAG_PERIODIC);
    if(timer1 != RT_NULL) rt_timer_start(timer1);
    return 0;
}

int water(void)
{
    Init_UART(9600);
    unsigned char c0[]={0xA0};
    unsigned char c1[]={0x00};
    unsigned char c2[]={0x00};
    unsigned char c3[]={0x00};
    unsigned char c4[]={0x00};
    unsigned char c5[]={0xA0};
        UART_SendString(USART3, c0);
        Delay_Ms(1000);
        UART_SendString(USART3, c1);
        Delay_Ms(1000);
        UART_SendString(USART3, c2);
        Delay_Ms(1000);
        UART_SendString(USART3, c3);
        Delay_Ms(1000);
        UART_SendString(USART3, c4);
        Delay_Ms(1000);
        UART_SendString(USART3, c5);
        Delay_Ms(1000);
        unsigned char buf[12];
        uint8_t index;
        unsigned char tmp;
        memset(buf,0,sizeof(buf));
        index=0;

        while(1){
            if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){
                USART_ClearITPendingBit(USART3,USART_IT_RXNE);
                tmp = USART_ReceiveData(USART3);
                buf[index++] = tmp;
            }
            if(index==12) break;
        }
       // printf("finish\r\n");
        uint16_t TDS1=((uint16_t)buf[1])*256+(uint16_t)buf[2];
        uint16_t TDS2=((uint16_t)buf[3])*256+(uint16_t)buf[4];
        printf("TDS1:%d\r\nTDS2:%d\r\n",TDS1,TDS2);
        uint16_t TMP1=(((uint16_t)buf[7])*256+(uint16_t)buf[8]);
        uint16_t TMP2=(((uint16_t)buf[9])*256+(uint16_t)buf[10]);
        uint16_t a1 = TMP1/100;
        uint16_t b1 = TMP1%100;
        uint16_t a2 = TMP2/100;
        uint16_t b2 = TMP2%100;
        printf("TMP1:%d.%d\r\nTMP2:%d.%d\r\n",a1,b1,a2,b2);
        lcd_init();
        lcd_fill(0,0,239,239,BLACK);
        lcd_show_string(0, 50, 32, "TDS1:");
        lcd_show_string(0, 80, 32, "TDS2:");
        lcd_show_num(100,50,TDS1,4,32);
        lcd_show_num(100,80,TDS2,4,32);
        lcd_show_string(0, 130, 32, "TMP1:   .");
        lcd_show_string(0, 160, 32, "TMP2:   .");
        lcd_show_num(100, 130, a1, 4, 32);
        lcd_show_num(100, 160, a2, 4, 32);
        lcd_show_num(142, 130, b1, 4, 32);
        lcd_show_num(142, 160, b2, 4, 32);
        if(pump_status == 1 )
            lcd_show_string(0, 190, 32, "PUMP is open!");
        else lcd_show_string(0, 190, 32, "PUMP is close!");
        Delay_Ms(2000);
        return 0;

}

MSH_CMD_EXPORT(water,  water sample by using I/O drivers);
MSH_CMD_EXPORT(levelcontrol,  levelcontrol sample by using I/O drivers);
