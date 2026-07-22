#include "gd32f3x0.h"
#include "stdio.h"
#include "systick.h"

#define delay_ms(x) delay_1ms(x)

void com_usart_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);

    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);

    usart_deinit(USART0);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
}

void timer_config(void)
{
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);

    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler = 8400 - 1;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = 10000 - 1;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1, &timer_initpara);

    timer_interrupt_enable(TIMER1, TIMER_INT_UP);
    nvic_irq_enable(TIMER1_IRQn, 0, 1);
    timer_enable(TIMER1);
}

volatile uint32_t timer_count = 0;

void TIMER1_IRQHandler(void)
{
    if (timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP) == SET)
    {
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
        timer_count++;
    }
}

void main(void)
{
    systick_config();

    printf("Step 1: Initializing USART...\n");
    com_usart_init();
    printf("Step 2: USART init done!\n");

    printf("Step 3: Initializing TIMER1...\n");
    timer_config();
    printf("Step 4: TIMER1 init done!\n");

    printf("Step 5: Entering main loop...\n");
    while (1)
    {
        if (timer_count >= 100)
        {
            timer_count = 0;
            printf("Timer interrupt working! Count: %lu\n", timer_count);
        }
    }
}
