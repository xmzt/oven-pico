#ifndef UTIL_H
#define UTIL_H

#include "base.h"

//-----------------------------------------------------------------------------------------------------------------------
// alias

#define UTIL_ALIAS_RW(x) (x)
#define UTIL_ALIAS_XOR(x) ( * (typeof(&(x))) ((1<<12) | (uintptr_t)&(x)) )
#define UTIL_ALIAS_SET(x) ( * (typeof(&(x))) ((2<<12) | (uintptr_t)&(x)) )
#define UTIL_ALIAS_CLR(x) ( * (typeof(&(x))) ((3<<12) | (uintptr_t)&(x)) )

//-----------------------------------------------------------------------------------------------------------------------
// util_dma

typedef struct {
	uint32_t read_addr;
	uint32_t write_addr;
	uint32_t transfer_count;
	uint32_t ctrl_trig;
} util_dma_block_0_t;

#define UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ(en,dsz,incr,incw,rsz,rsel,to,treq,iq) \
	((en) << DMA_CH0_CTRL_TRIG_EN_LSB									\
	 /*| 0 << DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_LSB*/						\
	 | (dsz) << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB							\
	 | (incr) << DMA_CH0_CTRL_TRIG_INCR_READ_LSB						\
	 | (incw) << DMA_CH0_CTRL_TRIG_INCR_WRITE_LSB						\
	 | (rsz) << DMA_CH0_CTRL_TRIG_RING_SIZE_LSB							\
	 | /*r0w1*/ (rsel) << DMA_CH0_CTRL_TRIG_RING_SEL_LSB					\
	 | (to) << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB							\
	 | (treq) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB							\
	 | (iq) << DMA_CH0_CTRL_TRIG_IRQ_QUIET_LSB							\
	 /*| 0 << DMA_CH0_CTRL_TRIG_BSWAP_LSB*/								\
	 /*| 0 << DMA_CH0_CTRL_TRIG_SNIFF_EN_LSB*/							\
	 )

//-----------------------------------------------------------------------------------------------------------------------
// util_i2c

#define UTIL_I2C_DATA_WRITE_RESTART_STOP_DATA(restart,stop,data) \
	((restart) << I2C_IC_DATA_CMD_RESTART_LSB		   \
	 | (stop) << I2C_IC_DATA_CMD_STOP_LSB			   \
	 | /*w0r1*/ 0 << I2C_IC_DATA_CMD_CMD_LSB		   \
	 | (data))

#define UTIL_I2C_DATA_READ_RESTART_STOP(restart,stop)	\
	((restart) << I2C_IC_DATA_CMD_RESTART_LSB	\
	 | (stop) << I2C_IC_DATA_CMD_STOP_LSB		\
	 | /*w0r1*/ 1 << I2C_IC_DATA_CMD_CMD_LSB)

#define UTIL_I2C_CON_ME_SPD_TENAS_TENAM_RE_SLVD_SDIA_TXEC_RXFFHC(me,spd,tenas,tenam,re,slvd,sdia,txec,rxffhc) \
	((me) << I2C_IC_CON_MASTER_MODE_LSB									\
	 | (spd) << I2C_IC_CON_SPEED_LSB									\
	 | (tenas) << I2C_IC_CON_IC_10BITADDR_SLAVE_LSB						\
	 | (tenam) << I2C_IC_CON_IC_10BITADDR_MASTER_LSB					\
	 | (re) << I2C_IC_CON_IC_RESTART_EN_LSB								\
	 | (slvd) << I2C_IC_CON_IC_SLAVE_DISABLE_LSB						\
	 | (sdia) << I2C_IC_CON_STOP_DET_IFADDRESSED_LSB					\
	 | (txec) << I2C_IC_CON_TX_EMPTY_CTRL_LSB							\
	 | (rxffhc) << I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL_LSB					\
	 )

void util_i2c_set_baudrate_lt_1e6(i2c_hw_t *hw, uint baudrate);

//-----------------------------------------------------------------------------------------------------------------------
// util_io

#define UTIL_IO_BANK0_GPIO_CTRL_FUNCSEL_OUTOVER_OEOVER_INOVER_IRQOVER(funcsel,outover,oeover,inover,irqover) \
	((funcsel) << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB						\
	 | (outover) << IO_BANK0_GPIO0_CTRL_OUTOVER_LSB						\
	 | (oeover) << IO_BANK0_GPIO0_CTRL_OEOVER_LSB						\
	 | (inover) << IO_BANK0_GPIO0_CTRL_INOVER_LSB						\
	 | (irqover) << IO_BANK0_GPIO0_CTRL_IRQOVER_LSB						\
	 )

//-----------------------------------------------------------------------------------------------------------------------
// util_irq util_isr

typedef void util_isr_t(void);

inline static void util_irq_isr(uint irq, util_isr_t *isr) {
    ((util_isr_t**)scb_hw->vtor)[VTABLE_FIRST_IRQ + irq] = isr;
}

//-----------------------------------------------------------------------------------------------------------------------
// util_pads

#define UTIL_PADS_BANK0_GPIO_OD_IE_DRIVE_PUE_PDE_SCHMITT_SLEWFAST(od,ie,drive,pue,pde,schmitt,slewfast) \
	((od) << PADS_BANK0_GPIO0_OD_LSB									\
	 | (ie) << PADS_BANK0_GPIO0_IE_LSB									\
	 | (drive) << PADS_BANK0_GPIO0_DRIVE_LSB							\
	 | (pue) << PADS_BANK0_GPIO0_PUE_LSB								\
	 | (pde) << PADS_BANK0_GPIO0_PDE_LSB								\
	 | (schmitt) << PADS_BANK0_GPIO0_SCHMITT_LSB						\
	 | (slewfast) << PADS_BANK0_GPIO0_SLEWFAST_LSB						\
	 )

//-----------------------------------------------------------------------------------------------------------------------
// util_pio

#define UTIL_PIO_CTRL_SME_SMR_CDR(sme,smr,cdr) \
	((sme) << PIO_CTRL_SM_ENABLE_LSB		   \
	 | (smr) << PIO_CTRL_SM_RESTART_LSB		   \
	 | (cdr) << PIO_CTRL_CLKDIV_RESTART_LSB	   \
	 )

#define UTIL_PIO_SM_CLKDIV_FRAC_INT(frac,int_)	\
	((frac) << PIO_SM0_CLKDIV_FRAC_LSB			\
	 | (int_) << PIO_SM0_CLKDIV_INT_LSB			\
	 )

// wrap is from top to bottom. top to bottom. as in hi PC to lo PC. not how code looks on a screen. don't forget.
#define UTIL_PIO_SM_EXECCTRL_STN_STS_WB_WT_OS_IOE_OES_JP_SPD_SEN(stn,sts,wb,wt,os,ioe,oes,jp,spd,sen) \
	((stn) << PIO_SM0_EXECCTRL_STATUS_N_LSB								\
	 | (sts) << PIO_SM0_EXECCTRL_STATUS_SEL_LSB							\
	 | (wb) << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB							\
	 | (wt) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB							\
	 | (os) << PIO_SM0_EXECCTRL_OUT_STICKY_LSB							\
	 | (ioe) << PIO_SM0_EXECCTRL_INLINE_OUT_EN_LSB						\
	 | (oes) << PIO_SM0_EXECCTRL_OUT_EN_SEL_LSB							\
	 | (jp) << PIO_SM0_EXECCTRL_JMP_PIN_LSB								\
	 | (spd) << PIO_SM0_EXECCTRL_SIDE_PINDIR_LSB						\
	 | (sen) << PIO_SM0_EXECCTRL_SIDE_EN_LSB							\
	 )

#define UTIL_PIO_SM_SHIFTCTRL_APUSH_APULL_ISD_OSD_PUSHT_PULLT_FJT_FJR(apush,apull,isd,osd,pusht,pullt,fjt,fjr) \
	((apush) << PIO_SM0_SHIFTCTRL_AUTOPUSH_LSB							\
	 | (apull) << PIO_SM0_SHIFTCTRL_AUTOPULL_LSB						\
	 | /*l0r1*/ (isd) << PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_LSB				\
	 | /*l0r1*/ (osd) << PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_LSB				\
	 | /*0->32*/ (pusht) << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB			\
	 | /*0->32*/ (pullt) << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB			\
	 | (fjt) << PIO_SM0_SHIFTCTRL_FJOIN_TX_LSB							\
	 | (fjr) << PIO_SM0_SHIFTCTRL_FJOIN_RX_LSB							\
	 )

#define UTIL_PIO_SM_PINCTRL_OB_SB_SSB_IB_OC_SC_SSC(ob,sb,ssb,ib,oc,sc,ssc) \
	((ob) << PIO_SM0_PINCTRL_OUT_BASE_LSB								\
	 | (sb) << PIO_SM0_PINCTRL_SET_BASE_LSB								\
	 | (ssb) << PIO_SM0_PINCTRL_SIDESET_BASE_LSB						\
	 | (ib) << PIO_SM0_PINCTRL_IN_BASE_LSB								\
	 | (oc) << PIO_SM0_PINCTRL_OUT_COUNT_LSB							\
	 | (sc) << PIO_SM0_PINCTRL_SET_COUNT_LSB							\
	 | (ssc) << PIO_SM0_PINCTRL_SIDESET_COUNT_LSB						\
	 )

// util_pio_load
//
// No DMA because
// (1) src is 16b instructions, dst is 32b registers
// (2) relocation modifications to non-zero dst_off (jmp instrs must be incremented by dst_off)

inline static uint util_pio_load(io_wo_32 *dst_base, uint dst_off, const uint16_t *src, uint count) {
	dst_base += dst_off;
	for(uint i = 0; i < count; i++) {
        uint16_t instr = src[i];
		if(pio_instr_bits_jmp == _pio_major_instr_bits(instr))
			instr += dst_off;
		dst_base[i] = instr;
    }
	return dst_off;
}

//-----------------------------------------------------------------------------------------------------------------------
// util_pwm

#define UTIL_PWM_CSR_EN_PHCORRECT_AINV_BINV_DIVMODE_PHRET_PHADV(en,phcorrect,ainv,binv,divmode,phret,phadv) \
	((en) << PWM_CH0_CSR_EN_LSB											\
	 | (phcorrect) << PWM_CH0_CSR_PH_CORRECT_LSB						\
	 | (ainv) << PWM_CH0_CSR_A_INV_LSB									\
	 | (binv) << PWM_CH0_CSR_B_INV_LSB									\
	 | (divmode) << PWM_CH0_CSR_DIVMODE_LSB								\
	 | (phret) << PWM_CH0_CSR_PH_RET_LSB								\
	 | (phadv) << PWM_CH0_CSR_PH_ADV_LSB)

#define UTIL_PWM_DIV_FRAC_INT(frac,int_)		\
	((frac) << PWM_CH0_DIV_FRAC_LSB				\
	 | (int_) << PWM_CH0_DIV_INT_LSB			\
	 )

#define UTIL_PWM_CC_A_B(a,b)					\
	((a) << PWM_CH0_CC_A_LSB					\
	 | (b) << PWM_CH0_CC_B_LSB					\
	 )

//-----------------------------------------------------------------------------------------------------------------------
// util_rando

inline static uint32_t util_rando(uint len) {
	uint32_t acc;
	for(uint i = 0; i < len; i++)
		acc = (acc << 1) ^ rosc_hw->randombit;
	return acc;
}

#endif
