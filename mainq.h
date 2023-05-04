#ifndef MAINQ_H
#define MAINQ_H

#include "base.h"

//-----------------------------------------------------------------------------------------------------------------------
// mainq
//
// a: start of list
// b: tail of list
// ab: link towards b
//
// item->ab == item signifies item is tail of list
// item->ab == 0 signifies item not in list
// this can be used to avoid calling za_enq on an item already in list.

typedef void mainq_fun_t(void);

#define MAINQ_RING_Z (1 << 7)
#define MAINQ_RING_MASK (MAINQ_RING_Z - 1)

extern mainq_fun_t * g_mainq_ring[MAINQ_RING_Z] __attribute__ (( aligned(MAINQ_RING_Z) ));

typedef struct {
	mainq_fun_t **pro;
	mainq_fun_t **con;
	uint z_max;
	uint dropN;
} mainq_t;
extern mainq_t g_mainq;

inline static void mainq_init(mainq_t *self, mainq_fun_t **ring) {
	self->pro = self->con = ring;
	self->z_max = 0;
	self->dropN = 0;
}

inline static int mainq_pro1(mainq_t *self, mainq_fun_t *item) {
	uint z = self->pro - self->con;
	if(self->z_max < z) self->z_max = z;
	if(MAINQ_RING_Z > z)
		*(mainq_fun_t**)((uint32_t)g_mainq_ring | ((uint32_t)self->pro++ & MAINQ_RING_MASK)) = item;
	else
		self->dropN++;
}

inline static mainq_fun_t * mainq_con1(mainq_t *self) {
	return self->con == self->pro ? (void*)0 :
		*(mainq_fun_t**)((uint32_t)g_mainq_ring | ((uint32_t)self->con++ & MAINQ_RING_MASK));
}

#endif
