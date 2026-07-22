/*!
    \file    main.c
    \brief   Deepsleep wakeup

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

#include "gd32f3x0.h"
#include "gd32f350r_eval.h"
#include "systick.h"

extern __IO uint8_t counter0;
static void system_clock_reconfig(void);
void com_usart_init(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    systick_config();

    gd_eval_led_init(LED2);

    /* USART configuration the CK_IRC8M as USART clock */
    rcu_usart_clock_config(RCU_USART0SRC_IRC8M);
    com_usart_init();

    nvic_irq_enable(USART0_IRQn, 0, 0);

    delay_1ms(2000);

    /* use start bit wakeup MCU */
    usart_wakeup_mode_config(USART0, USART_WUM_STARTB);

    /* enable USART */
    usart_enable(USART0);
    /* ensure USART is enabled */
    while(RESET == usart_flag_get(USART0, USART_FLAG_REA)) {
    }
    /* check USART is not transmitting */
    while(SET == usart_flag_get(USART0, USART_FLAG_BSY)) {
    }

    usart_wakeup_enable(USART0);
    /* enable the WUIE interrupt */
    usart_interrupt_enable(USART0, USART_INT_WU);

    /* enable PWU APB clock */
    rcu_periph_clock_enable(RCU_PMU);
    /* enter deep-sleep mode */
    pmu_to_deepsleepmode(PMU_LDO_LOWPOWER, PMU_LOWDRIVER_ENABLE, WFI_CMD);

    /* wait a WUIE interrupt event */
    while(0x00 == counter0);

    /* disable USART peripheral in deepsleep mode */
    usart_wakeup_disable(USART0);

    while(RESET == usart_flag_get(USART0, USART_FLAG_RBNE));
    usart_data_receive(USART0);

    usart_receive_config(USART0, USART_RECEIVE_ENABLE);

    while(RESET == usart_flag_get(USART0, USART_FLAG_TC));

    /* disable the USART */
    usart_disable(USART0);

    /* reconfigure systemclock */
    system_clock_reconfig();

    systick_config();

    while(1) {
    }
}

/*!
    \brief      initialize the USART configuration of the com
    \param[in]  none
    \param[out] none
    \retval     none
*/
void com_usart_init(void)
{
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USART TX */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);
    /* connect port to USART RX */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

    /* configure USART TX as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);

    /* configure USART RX as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

    usart_enable(USART0);
}

/*!
    \brief      reconfigure system clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_reconfig(void)
{
    /* enable HXTAL */
    RCU_CTL0 |= RCU_CTL0_HXTALEN;

    while(SUCCESS != rcu_osci_stab_wait(RCU_HXTAL)) {
    }
    /* configure AHB */
    rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1);

    /* configure APB1, APB2 */
    rcu_apb1_clock_config(RCU_APB1_CKAHB_DIV1);
    rcu_apb2_clock_config(RCU_APB2_CKAHB_DIV1);

    /*  PLL configuration: = HXTAL/2 * 27 = 72 MHz */
    rcu_hxtal_prediv_config(RCU_PLL_PREDV2);
    rcu_pll_config(RCU_PLLSRC_HXTAL_IRC48M, RCU_PLL_MUL27);

    /* enable PLL */
    RCU_CTL0 |= RCU_CTL0_PLLEN;

    /* select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;
}

/*!
    \brief      LED spark
    \param[in]  none
    \param[out] none
    \retval     none
*/
void led_spark(void)
{
    static __IO uint32_t time_delay = 0;

    if(0x00 != time_delay) {
        if(time_delay < 500) {
            /* light on */
            gd_eval_led_on(LED2);
        } else {
            /* light off */
            gd_eval_led_off(LED2);
        }
        time_delay--;
    } else {
        time_delay = 1000;
    }
}
