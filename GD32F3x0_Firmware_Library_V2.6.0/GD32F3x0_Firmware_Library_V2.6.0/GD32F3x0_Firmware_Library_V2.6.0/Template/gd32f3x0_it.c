/*!
    \file    gd32f3x0_it.c
    \brief   interrupt service routines

    \version 2026-01-01, V2.6.0, firmware for GD32F3x0
*/

/*
    Copyright (c) 2026, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f3x0_it.h"
#include "main.h"
#include "systick.h"
#include "gd32f3x0.h"

extern uint8_t transfersize;
extern uint8_t receivesize;
extern __IO uint8_t txcount;
extern __IO uint16_t rxcount;
extern uint8_t receiver_buffer[32];
extern uint8_t transmitter_buffer[];

/* USART0 DMA + 空闲中断接收 外部变量 */
extern uint8_t  usart0_rx_buffer[];
extern volatile uint16_t usart0_rx_len;
extern volatile uint8_t  usart0_rx_flag;
extern volatile uint8_t  usart0_tx_busy;
/* USART0_RX_BUF_SIZE 统一在 main.h 定义（勿在此重复定义，避免与 main.c 不同步） */

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
    /* if NMI exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
    /* if SVC exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
    /* if DebugMon exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
    /* if PendSV exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    // led_spark();
    delay_decrement();
}

/*!
    \brief      USART0 空闲中断（DMA接收完成一帧时触发）
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART0_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE)) {
        /* 清除空闲中断标志 */
        usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE);

        /* DMA 单次模式：计数器剩余值即本次接收长度 */
        uint16_t remain = (uint16_t)dma_transfer_number_get(DMA_CH2);
        uint16_t received = USART0_RX_BUF_SIZE - remain;

        if (received > 0 && received <= USART0_RX_BUF_SIZE) {
            usart0_rx_len  = received;
            usart0_rx_flag = 1;

            /* 数据在 buffer[0..received-1]，重新配置 DMA 准备下一帧 */
            dma_channel_disable(DMA_CH2);
            /* ★ 必须重置内存地址到缓冲区头：单次模式 DMA 收完一帧后地址已前移，
               只重设计数不重置地址会让下一帧写到错误偏移，解析到脏数据 */
            dma_memory_address_config(DMA_CH2, (uint32_t)usart0_rx_buffer);
            dma_transfer_number_config(DMA_CH2, USART0_RX_BUF_SIZE);
            dma_channel_enable(DMA_CH2);
        }
    }
}

extern volatile uint8_t imu_loop_flag;

void TIMER1_IRQHandler(void)
{
    if (timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP) == SET)
    {
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
        imu_loop_flag = 1;
        // printf("1\n");
    }
}