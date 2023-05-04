#include "mytm.h"
#include "mainq.h"
#include "util.h"

#include "stdio.h"

static uint32_t g_pwm0;
static uint32_t g_pwm1;
static int32_t g_pwmD;
static uint32_t g_timer0;
static uint32_t g_timerD;
static uint32_t g_print0;

#if 0
static void alarm_isr() {
}

void ac_pwmhf_isr(void) {
	g_ac_pwmlf++;
	pwm_hw->intr = 1 << AC_SLICE_ID_PWMHF;
}
#endif

static void stat_mainq_fun() {
	printf("[mytm] pwm1=%u pwmD=%d timerD=%u\n", g_pwm1, g_pwmD, g_timerD);
}

static void pwm_query1_isr() {
	g_pwm1 = MYTM_PWM_SLICE->ctr;
	uint32_t timer1 = timer_hw->timerawl;
 	UTIL_ALIAS_CLR(timer_hw->intr) = 1 << MYTM_ALARM_ID;
	timer_hw->alarm[MYTM_ALARM_ID] += MYTM_ALARM_DUR;

	g_pwmD = g_pwm1 - g_pwm0;
	g_timerD = timer1 - g_timer0;
	g_pwm0 = g_pwm1;
	g_timer0 = timer1;
	if(1000000 <= (timer1 - g_print0)) {
		mainq_pro1(&g_mainq, stat_mainq_fun);
		g_print0 = timer1;
	}
}


static void pwm_query0_isr() {
	g_pwm0 = MYTM_PWM_SLICE->ctr;
	g_timer0 = timer_hw->timerawl;
 	UTIL_ALIAS_CLR(timer_hw->intr) = 1 << MYTM_ALARM_ID;
	util_irq_isr(MYTM_ALARM_IRQ, pwm_query1_isr);
	timer_hw->alarm[MYTM_ALARM_ID] += MYTM_ALARM_DUR;

	g_print0 = g_timer0;
}

static void pwm_sync_isr() {



	UTIL_ALIAS_SET(pwm_hw->en) = 1 << MYTM_PWM_SLICE_ID;
 	UTIL_ALIAS_CLR(timer_hw->intr) = 1 << MYTM_ALARM_ID;
	util_irq_isr(MYTM_ALARM_IRQ, pwm_query0_isr);
	timer_hw->alarm[MYTM_ALARM_ID] += MYTM_ALARM_DUR;
	printf("pwm_sync_isr\n");
}
	
void mytm_init() {
	// pwm (125 MHz), do not start
	MYTM_PWM_SLICE->div = UTIL_PWM_DIV_FRAC_INT(0, 1);
	//MYTM_PWM_SLICE->top = 0xFFFF;
	//MYTM_PWM_SLICE->cc = UTIL_PWM_CC_A_B(0xFFFE, 0); // low for 2 cycles, for a reason (todo)
	MYTM_PWM_SLICE->csr = UTIL_PWM_CSR_EN_PHCORRECT_AINV_BINV_DIVMODE_PHRET_PHADV
		(0, 0, 0, 0, 0, 0, 0);

	// timer (1 MHz)
	util_irq_isr(MYTM_ALARM_IRQ, pwm_sync_isr);
	nvic_hw->iser = 1 << MYTM_ALARM_IRQ;
	UTIL_ALIAS_SET(timer_hw->inte) = 1 << MYTM_ALARM_ID;
	timer_hw->alarm[MYTM_ALARM_ID] = timer_hw->timerawl + MYTM_ALARM_DUR;
	UTIL_ALIAS_SET(pwm_hw->en) = 1 << MYTM_PWM_SLICE_ID;


	printf("mytm_init\n");
}
