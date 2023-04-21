#ifndef UTIL_H
#define UTIL_H

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/structs/i2c.h"
#include "hardware/structs/rosc.h"
#include "hardware/structs/scb.h"
#include "hardware/structs/timer.h"

#define ATOMIC_ALIAS_XOR 0x1000
#define ATOMIC_ALIAS_SET 0x2000
#define ATOMIC_ALIAS_CLR 0x3000

//-----------------------------------------------------------------------------------------------------------------------
// util_dma

typedef struct {
	uint32_t read_addr;
	uint32_t write_addr;
	uint32_t transfer_count;
	uint32_t ctrl_trig;
} util_dma_block_0_t;

#define UTIL_DMA_CTRL_TRIG_EN_DSZ_INCR_INCW_RSZ_RSEL_TO_TREQ_IQ(en, dsz, incr, incw, rsz, rsel, to, treq, iq) \
	((en) << DMA_CH0_CTRL_TRIG_EN_LSB									\
	 /*| 0<<DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_LSB*/						\
	 | (dsz) << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB							\
	 | (incr) << DMA_CH0_CTRL_TRIG_INCR_READ_LSB						\
	 | (incw) << DMA_CH0_CTRL_TRIG_INCR_WRITE_LSB						\
	 | (rsz) << DMA_CH0_CTRL_TRIG_RING_SIZE_LSB							\
	 | (rsel) << DMA_CH0_CTRL_TRIG_RING_SEL_LSB							\
	 | (to) << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB							\
	 | (treq) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB							\
	 | (iq) << DMA_CH0_CTRL_TRIG_IRQ_QUIET_LSB							\
	 /*| 0 << DMA_CH0_CTRL_TRIG_BSWAP_LSB*/								\
	 /*| 0 << DMA_CH0_CTRL_TRIG_SNIFF_EN_LSB*/							\
	 )

//-----------------------------------------------------------------------------------------------------------------------
// util_i2c

#define UTIL_I2C_DATA_WRITE_R_S_D(restart, stop, data) \
	((restart) << I2C_IC_DATA_CMD_RESTART_LSB		   \
	 | (stop) << I2C_IC_DATA_CMD_STOP_LSB			   \
	 | /*w0r1*/ 0 << I2C_IC_DATA_CMD_CMD_LSB		   \
	 | (data))

#define UTIL_I2C_DATA_READ_R_S(restart, stop)	\
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

//-----------------------------------------------------------------------------------------------------------------------
// util_io

#define UTIL_IO_BANK0_GPIO_CTRL_FS(fs) \
	((fs) << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB \
	 /*| 0 << IO_BANK0_GPIO0_CTRL_OUTOVER_LSB*/	\
	 /*| 0 << IO_BANK0_GPIO0_CTRL_OEOVER_LSB*/	\
	 /*| 0 << IO_BANK0_GPIO0_CTRL_INOVER_LSB*/	\
	 /*| 0 << IO_BANK0_GPIO0_CTRL_IRQOVER_LSB*/	\
	 )

//-----------------------------------------------------------------------------------------------------------------------
// util_irq

inline static void util_irq_iser(uint32_t mask) {
	// todo? ICPR as well as ISER?
	*((io_rw_32 *)(PPB_BASE + M0PLUS_NVIC_ISER_OFFSET)) = mask;
}

inline static void util_irq_hand(uint num, irq_handler_t hand) {
    ((irq_handler_t *)scb_hw->vtor)[VTABLE_FIRST_IRQ + num] = hand;
}

inline static void util_irq_hand_iser(uint num, irq_handler_t hand) {
    util_irq_hand(num, hand);
	util_irq_iser(1<<num);
}

//-----------------------------------------------------------------------------------------------------------------------
// util_pads

#define UTIL_PADS_BANK0_GPIO_OD_IE_DRV_PUE_PDE_SCH_SF(od, ie, drv, pue, pde, sch, sf) \
	((od) << PADS_BANK0_GPIO0_OD_LSB									\
	 | (ie) << PADS_BANK0_GPIO0_IE_LSB									\
	 | (drv) << PADS_BANK0_GPIO0_DRIVE_LSB								\
	 | (pue) << PADS_BANK0_GPIO0_PUE_LSB								\
	 | (pde) << PADS_BANK0_GPIO0_PDE_LSB								\
	 | (sch) << PADS_BANK0_GPIO0_SCHMITT_LSB							\
	 | (sf) << PADS_BANK0_GPIO0_SLEWFAST_LSB							\
	 )

//-----------------------------------------------------------------------------------------------------------------------
// util_pio

#define UTIL_PIO_CTRL_SME_SMR_CDR(sme,smr,cdr) \
	((sme) << PIO_CTRL_SM_ENABLE_LSB		   \
	 | (smr) << PIO_CTRL_SM_RESTART_LSB		   \
	 | (cdr) << PIO_CTRL_CLKDIV_RESTART_LSB	   \
	 )

#define UTIL_PIO_CTRL_STATE_SME_SMR_CDR(state,sme,smr,cdr)	\
	((sme) << PIO_CTRL_SM_ENABLE_LSB		   \
	 | (smr) << PIO_CTRL_SM_RESTART_LSB		   \
	 | (cdr) << PIO_CTRL_CLKDIV_RESTART_LSB	   \
	 )


#define UTIL_PIO_SM_CLKDIV_F_I(f, i)			\
	((f) << PIO_SM0_CLKDIV_FRAC_LSB				\
	 | (i) << PIO_SM0_CLKDIV_INT_LSB			\
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
	 | (isd) << PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_LSB						\
	 | (osd) << PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_LSB						\
	 | (pusht) << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB						\
	 | (pullt) << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB						\
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
// util_rando

inline static uint32_t util_rando(uint len) {
	uint32_t acc;
	for(uint i = 0; i < len; i++)
		acc = (acc << 1) ^ rosc_hw->randombit;
	return acc;
}

#endif
