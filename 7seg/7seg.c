#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

char rec;
char buf[256];
int last = 0;
static void
uart_setup(void) {

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_USART2);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
	usart_set_baudrate(USART2, 9600);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_enable(USART2);
	gpio_set_mode(GPIOA,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		GPIO_USART1_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4 | GPIO5 | GPIO6 | GPIO7);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1 | GPIO6 | GPIO8 | GPIO10);
	usart_set_baudrate(USART1,115200);
	usart_set_databits(USART1,8);
	usart_set_stopbits(USART1,USART_STOPBITS_1);
	usart_set_mode(USART1,USART_MODE_TX_RX);
	usart_set_parity(USART1,USART_PARITY_NONE);
	usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);
	usart_enable(USART1);
}

static void tim_setup(void)
{
        TIM1_SMCR &= ~TIM_SMCR_SMS_MASK;
        TIM1_CR1 &= ~TIM_CR1_CEN;

        TIM1_CR1 = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;
        TIM1_ARR = 65535;
        TIM1_PSC = 2;
        TIM1_EGR = TIM_EGR_UG;

        TIM1_CCMR1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;

        TIM1_CCER |= TIM_CCER_CC1E;
        TIM1_CCR1 = 1000;
        TIM1_CCMR1 |= TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
/*
        TIM1_CCER |= TIM_CCER_CC2E;
        TIM1_CCR2 = 1000;
        TIM1_CCMR2 |= TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;
        TIM1_CCER |= TIM_CCER_CC3E;


        TIM1_CCMR2 |= TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE;
        TIM1_CCER |= TIM_CCER_CC4E;
        TIM1_CR1 |= TIM_CR1_ARPE;
*/
        TIM1_BDTR |= TIM_BDTR_MOE;
        TIM1_CR1 |= TIM_CR1_CEN;
}


static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable clocks for GPIO port B */
	rcc_periph_clock_enable(RCC_GPIOA | RCC_GPIOB);
}

static void reset_pins(void) {
	
	gpio_clear(GPIOA, GPIO4);
	gpio_clear(GPIOA, GPIO5);
	gpio_clear(GPIOA, GPIO6);
	// gpio_clear(GPIOA, GPIO7); /* do not clear DP */
	gpio_clear(GPIOB, GPIO8);
	gpio_clear(GPIOB, GPIO6);
	gpio_clear(GPIOB, GPIO1);
	gpio_clear(GPIOB, GPIO10);
}

static void seg(int pin) {
	switch(pin) {
		case 0:
			reset_pins();
			TIM1_CCR1 = pin*6000;
			gpio_set(GPIOB, GPIO8);
			gpio_set(GPIOB, GPIO10);
			gpio_set(GPIOA, GPIO4);
			gpio_set(GPIOA, GPIO5);
			gpio_set(GPIOA, GPIO6);
			gpio_set(GPIOB, GPIO6);
			break;

		case 1:
			reset_pins();
			TIM1_CCR1=pin*6000;
			gpio_set(GPIOB, GPIO6);
			gpio_set(GPIOA, GPIO6);
			break;
		case 2:
			reset_pins();
			
			TIM1_CCR1=pin*6000;
			gpio_set(GPIOB, GPIO8);
			gpio_set(GPIOB, GPIO6);
			gpio_set(GPIOB, GPIO1);
			gpio_set(GPIOA, GPIO4);
			gpio_set(GPIOA, GPIO5);

			break;
		case 3:
			reset_pins();		
			TIM1_CCR1=pin*6000;
			gpio_set(GPIOB, GPIO8);
			gpio_set(GPIOB, GPIO6);
			gpio_set(GPIOB, GPIO1);
			gpio_set(GPIOA, GPIO6);
			gpio_set(GPIOA, GPIO5);
			break;

		case 4:
			reset_pins();
			TIM1_CCR1=pin*6000;
			gpio_set(GPIOA, GPIO6);
			gpio_set(GPIOB, GPIO6);
			gpio_set(GPIOB, GPIO1);
			gpio_set(GPIOB, GPIO10);
			break;

		case 5:
			reset_pins();
			TIM1_CCR1=pin*6000;
			gpio_set(GPIOB, GPIO8);
			gpio_set(GPIOA, GPIO6);
			gpio_set(GPIOB, GPIO1);
			gpio_set(GPIOB, GPIO10);
			gpio_set(GPIOA, GPIO5);

			break;
		case 6:
			reset_pins();
			TIM1_CCR1=pin*6000;
			gpio_set(GPIOB, GPIO8 );
			gpio_set(GPIOA, GPIO6 );
			gpio_set(GPIOB, GPIO1 );
			gpio_set(GPIOB, GPIO10);
			gpio_set(GPIOA, GPIO5 );
			gpio_set(GPIOA, GPIO4 );
			break;
		case 7:
			reset_pins();
			TIM1_CCR1=pin*6000;
			gpio_set(GPIOB, GPIO8);
			gpio_set(GPIOA, GPIO6);
			gpio_set(GPIOB, GPIO6);
			break;

		case 8:
			reset_pins();
			TIM1_CCR1=pin*6000;
			gpio_set(GPIOB, GPIO8);
			gpio_set(GPIOA, GPIO6);
			gpio_set(GPIOB, GPIO1);
			gpio_set(GPIOB, GPIO10);
			gpio_set(GPIOA, GPIO5);
			gpio_set(GPIOA, GPIO4);
			gpio_set(GPIOB, GPIO6);
			break;

		case 9:
			reset_pins();	
			TIM1_CCR1=pin*6000;
			gpio_set(GPIOB, GPIO8);
			gpio_set(GPIOA, GPIO6);
			gpio_set(GPIOB, GPIO1);
			gpio_set(GPIOB, GPIO10);
			gpio_set(GPIOA, GPIO5);
			gpio_set(GPIOB, GPIO6);
			break;

		/*case '.' - '0':
			gpio_toggle(GPIOA, GPIO7);
			break;
		*/

		default:
			break;
	}
}

static void nvic_setup(void)
{
	/* Without this the timer interrupt routine will never be called. */
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_set_priority(NVIC_TIM2_IRQ, 1);
}

void tim2_isr(void)
{
	gpio_toggle(GPIOA, GPIO7);   /* LED2 on/off. */
	TIM_SR(TIM2) &= ~TIM_SR_UIF; /* Clear interrrupt flag. */
}

int main(void) {
	clock_setup();
	uart_setup();
	nvic_setup();

	gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO15);

	    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                      GPIO_TIM1_CH1);


	            rcc_periph_clock_enable(RCC_TIM1);
				rcc_periph_clock_enable(RCC_TIM2);

        rcc_periph_clock_enable(RCC_AFIO);

	// int i = 0;
		TIM_CNT(TIM2) = 1;

	/* Set timer prescaler. 72MHz/1440 => 50000 counts per second. */
	//TIM_PSC(TIM2) = 720;
	timer_set_prescaler(TIM2, 720);
	/* End timer value. If this is reached an interrupt is generated. */
	//TIM_ARR(TIM2) = 50000;
	timer_enable_preload(TIM2);
	timer_set_period(TIM2, 50000);
	/* Update interrupt enable. */
	//TIM_DIER(TIM2) |= TIM_DIER_UIE;
	timer_enable_irq(TIM2, TIM_DIER_UIE);
	/* Start timer. */
	//TIM_CR1(TIM2) |= TIM_CR1_CEN;
	timer_enable_counter(TIM2);
	tim_setup();
	while(1) {
			seg(usart_recv_blocking(USART2) - '0');
			// seg(usart_recv_blocking(USART1) - '0');
	}
	return 0;
}
